"""创建并启动 drone_harness 运行时。"""

from __future__ import annotations

import threading
import sys
from dataclasses import dataclass
from typing import Any

from drone_harness.bus import InputServer, MessageBus
from drone_harness.config.loader import load_profile
from drone_harness.logging.task_log import create_session_id, log_agent_message, log_task_state, log_navigation_plan
from drone_harness.llm.client import create_llm_client
from drone_harness.llm.prompts import build_system_prompt
from drone_harness.runtime.task_state import TaskState, format_task_state_line
from drone_harness.runtime.task_memory import input_directive, prepare_navigation_turn
from drone_harness.runtime.safety import SafetyHandoffRequired
from drone_harness.runtime.terminal import open_input_terminal
from drone_harness.tools.registry import ToolContext


@dataclass(frozen=True)
class RuntimeStartResult:
    """保存 profile 解析后的运行时摘要，供测试和诊断使用。"""

    profile_name: str
    mode: str
    node_name: str
    ros_started: bool


def prepare_runtime(profile_name: str) -> RuntimeStartResult:
    """只加载并校验 profile，不启动 ROS2。"""
    profile = load_profile(profile_name)
    return RuntimeStartResult(
        profile_name=profile.name,
        mode=profile.mode,
        node_name=profile.ros.node_name,
        ros_started=False,
    )


def start_runtime(profile_name: str) -> None:
    """加载 profile 并直接启动真实 ROS2 运行时。"""
    profile = load_profile(profile_name)
    _start_live_runtime(profile)


def controller_class_for_profile(mode: str) -> type[Any]:
    """返回当前 profile 使用的原名 PX4 控制器类。"""
    if mode not in {"simulation", "real"}:
        raise ValueError(f"unsupported runtime mode: {mode}")
    from drone_harness.px4.controller import Px4Controller

    return Px4Controller


def _join_started_thread(thread: threading.Thread | None) -> None:
    """仅等待已经启动过的线程，避免初始化失败掩盖原始异常。"""
    if thread is not None and thread.ident is not None:
        thread.join(timeout=1.0)


def _start_live_runtime(profile) -> None:
    """创建 ROS2、PX4 controller 和 agent loop 的完整运行时。"""
    import rclpy
    from rclpy.executors import SingleThreadedExecutor

    from drone_harness.runtime.agent_loop import agent_loop

    rclpy.init()
    executor = SingleThreadedExecutor()
    controller = None
    executor_thread = None
    input_server = None

    try:
        session_id = create_session_id()
        task_state = TaskState(task_id=session_id)
        message_bus = MessageBus()
        input_server = InputServer(message_bus)
        input_server.start()
        controller_class = controller_class_for_profile(profile.mode)
        controller = controller_class(
            node_name=profile.ros.node_name,
            camera_scene_topic=profile.ros.camera_scene_topic,
            camera_depth_topic=profile.ros.camera_depth_topic,
            camera_depth_info_topic=profile.ros.camera_depth_info_topic,
            observation_config=profile.observation,
            mavros_namespace=profile.ros.mavros_namespace,
        )
        executor.add_node(controller)
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        client = create_llm_client(profile)
        context = ToolContext(
            controller=controller,
            profile=profile,
            session_id=session_id,
            task_state=task_state,
            message_bus=message_bus,
            navigation_enabled=True,
        )
        executor_thread.start()
        input_terminal_started = _start_input_terminal(input_server, profile.name)
        log_agent_message(profile, context.session_id, "system", build_system_prompt(profile))
        _run_interactive_loop(
            client,
            profile.llm.model,
            context,
            agent_loop,
            input_terminal_started=input_terminal_started,
        )
    except SafetyHandoffRequired as exc:
        message = str(exc)
        print(message, file=sys.stderr, flush=True)
        log_agent_message(profile, session_id, "safety", message)
    finally:
        if input_server is not None:
            input_server.stop()
        executor.shutdown()
        if controller is not None:
            controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        _join_started_thread(executor_thread)


def _configure_readline() -> None:
    """配置终端输入行为，避免中文编辑异常。"""
    try:
        import readline
    except ImportError:
        return

    readline.parse_and_bind("set bind-tty-special-chars off")
    readline.parse_and_bind("set input-meta on")
    readline.parse_and_bind("set output-meta on")
    readline.parse_and_bind("set convert-meta off")


def _run_interactive_loop(
    client: Any,
    model: str,
    context: ToolContext,
    agent_loop: Any,
    *,
    input_terminal_started: bool,
) -> None:
    """运行命令行交互循环，并把每轮输入交给 agent loop。"""
    _configure_readline()
    if input_terminal_started:
        print("输入终端已打开；当前终端只显示 agent、tool、ROS2 消息。")
        print("请在输入终端输入自然语言或 HITL 确认，输入 exit 退出。")
    else:
        print("未能自动打开输入终端，回退为当前终端输入模式。")
        _start_input_thread(context)
        print("输入自然语言与 agent 对话，输入 exit 退出。")
    _record_task_state(context)

    conversation: list[dict[str, str]] = []
    while True:
        if context.task_state is not None and context.task_state.intervention_pending:
            user_input = context.task_state.intervention_message or ""
            context.task_state.clear_intervention()
        else:
            if context.message_bus is None:
                break
            user_input = context.message_bus.consume_user_message().content.strip()
        if not user_input:
            continue
        if user_input.lower() in {"exit", "quit"}:
            break
        directive = input_directive(user_input) if context.navigation_enabled else "guide"
        if directive in {"cancel", "replace"}:
            from drone_harness.runtime.agent_loop import _stop_with_message

            _stop_with_message(context, "原任务已取消。", safety_stop=True)
            if context.navigation_plan is not None:
                context.navigation_plan.status = "cancelled"
                log_navigation_plan(context.profile, context.session_id, "cancelled", context.navigation_plan)
            context.navigation_plan = None
            context.navigation_resumable = False
            # 明确取消或替换时隔离旧任务对话，避免模型从旧文字自行恢复。
            conversation.clear()
            if directive == "cancel":
                context.observation = None
                context.depth_rules = None
                if context.task_state is not None:
                    context.task_state.start_new_goal(user_input)
                    context.task_state.set_idle()
                log_agent_message(context.profile, context.session_id, "user", user_input)
                conversation.extend([{"role": "user", "content": user_input},
                                     {"role": "assistant", "content": "任务已取消，不再自动继续原任务。"}])
                continue
        resumed = prepare_navigation_turn(context, user_input, replace_task=directive == "replace")
        if resumed:
            print("plan> 已保留原任务和清单，接收本轮用户引导。")
            log_navigation_plan(context.profile, context.session_id, "resumed", context.navigation_plan)
        conversation.append({"role": "user", "content": user_input})
        messages: list[dict[str, Any]] = [
            {"role": "system", "content": build_system_prompt(context.profile)},
            *conversation,
        ]
        turn_start = len(messages)
        log_agent_message(context.profile, context.session_id, "user", user_input)
        answer = agent_loop(client, model, messages, context)
        # 跨轮只带用户与 AI 文字，不带工具协议、执行结果或相机观测。
        for message in messages[turn_start:]:
            content = message.get("content")
            if message.get("role") == "assistant" and isinstance(content, str) and content:
                conversation.append({"role": "assistant", "content": content})
        # 降落、失败和中断的返回说明未必已加入 messages，也需留给下轮。
        last = messages[-1]
        if isinstance(answer, str) and answer and not (
            last.get("role") == "assistant" and not last.get("tool_calls")
            and last.get("content") == answer
        ):
            conversation.append({"role": "assistant", "content": answer})


def _start_input_terminal(input_server: InputServer, profile_name: str) -> bool:
    """根据输入服务信息打开独立输入终端。"""
    info = input_server.info
    return open_input_terminal(info.host, info.port, info.token, profile_name)


def _start_input_thread(context: ToolContext) -> None:
    """启动后台输入线程，把用户输入写入消息总线。"""
    if context.message_bus is None:
        return

    def _read_input() -> None:
        """持续读取终端输入并发布到消息总线。"""
        while True:
            try:
                user_input = input("you> ").strip()
            except EOFError:
                context.message_bus.publish_user_message("exit")
                return
            if user_input:
                context.message_bus.publish_user_message(user_input)
            if user_input.lower() in {"exit", "quit"}:
                return

    threading.Thread(target=_read_input, daemon=True).start()


def _record_task_state(context: ToolContext) -> None:
    """打印并记录当前会话状态。"""
    if context.task_state is None:
        return
    print(format_task_state_line(context.task_state))
    log_task_state(context.profile, context.session_id, context.task_state)
