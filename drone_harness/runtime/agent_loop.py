"""管理大模型与工具调用循环。"""

from __future__ import annotations

import json
import math
import time
from time import monotonic_ns, perf_counter
from typing import Any

from drone_harness.bus.intervention import interrupt_if_requested
from drone_harness.llm.messages import (
    build_assistant_message, prepare_model_messages, reasoning_request_details, response_reasoning_chars,
)
from drone_harness.llm.request_trace import trace_model_request
from drone_harness.logging.task_log import (
    log_agent_message, log_context_usage, log_model_request, log_navigation_plan,
    log_observation, log_task_state, log_tool_call,
    save_observation_image,
)
from drone_harness.runtime.navigation import (
    compact_navigation_history, navigation_messages,
)
from drone_harness.runtime.observation import ObservationSnapshot, build_observation_message
from drone_harness.runtime.safety import EndCurrentTurn, SafetyHandoffRequired, request_confirmed_hover
from drone_harness.runtime.skill_search import SEARCH_MARKER, build_search_message, clear_search_images
from drone_harness.runtime.reflection import (
    clear_target_assessment, discard_stale_target_assessment, record_target_assessment,
)
from drone_harness.runtime.task_state import format_task_state_line
from drone_harness.runtime.task_memory import is_recoverable_rejection
from drone_harness.runtime.target_measurement import clear_target_measurement, target_measurement_snapshot
from drone_harness.runtime.tool_dispatcher import dispatch_tool_call
from drone_harness.tools.registry import ToolContext, get_tool_schemas
from drone_harness.vision.depth_rules import compute_depth_rules


MAX_TOOL_CALLS_PER_TURN = 50
MAX_NAVIGATION_REQUESTS = 200
MAX_CONSECUTIVE_NO_PROGRESS = 3
MAX_CORRECTIONS = 2

MODEL_TOOL_RESULT_FIELDS = {
    "measure_distance": {"success", "valid", "distance_m", "reason", "message"},
    "observe": {"success", "view", "depth_valid", "forward_max_m", "message"},
    "forward": {"success", "motion_executed", "requested_distance_m", "commanded_distance_m",
                "final_position_ned", "message"},
    "takeoff": {"success", "target_height", "final_position_ned", "message"},
    "up": {"success", "body_offset_frd", "final_position_ned", "message"},
    "down": {"success", "body_offset_frd", "final_position_ned", "message"},
    "rotate": {"success", "direction", "degrees", "target_yaw_rad", "final_position_ned", "message"},
    "land": {"success", "final_position_ned", "message"},
}


def agent_loop(
    client: Any,
    model: str,
    messages: list[dict[str, Any]],
    context: ToolContext,
) -> str:
    """运行主循环并在任何出口释放 skill 临时图片。"""
    try:
        return _run_agent_loop(client, model, messages, context)
    finally:
        context.search_observations = None
        clear_search_images(messages)


def _run_agent_loop(
    client: Any,
    model: str,
    messages: list[dict[str, Any]],
    context: ToolContext,
) -> str:
    """让同一模型管理清单，继续复用单工具循环和按需观察。"""
    task_start = len(messages)
    context.navigation_instruction = (context.navigation_plan.original_instruction if context.navigation_plan else
                                     next((item["content"] for item in reversed(messages)
                                           if item["role"] == "user" and isinstance(item["content"], str)), ""))
    navigation_enabled = context.navigation_enabled
    requests = 0
    corrections = 0
    while True:
        discard_stale_target_assessment(context)
        plan = context.navigation_plan
        request_limit = MAX_NAVIGATION_REQUESTS if plan is not None else MAX_TOOL_CALLS_PER_TURN
        if requests >= request_limit:
            break
        if context.task_state is not None and context.task_state.consecutive_no_progress >= MAX_CONSECUTIVE_NO_PROGRESS:
            return _stop_with_message(context, "连续动作没有可测位移，已停止自主规划。", safety_stop=True)
        if plan is not None:
            interruption = interrupt_if_requested(context, hover_on_flight_tool=False)
            if interruption is not None:
                return _stop_with_message(context, interruption["message"], safety_stop=True)
        _record_task_state(context, "thinking")
        requests += 1
        has_current_image = _navigation_observation_current(context) or any(
            isinstance(item.get("content"), list)
            and any(part.get("type") == "image_url" for part in item["content"])
            and item["content"][0].get("text", "").startswith(SEARCH_MARKER)
            for item in messages)
        try:
            request_messages = (navigation_messages(
                messages, plan, observation_current=_navigation_observation_current(context),
                latest_target_measurement=target_measurement_snapshot(context))
                                if navigation_enabled else messages)
            request_messages = prepare_model_messages(request_messages, context.profile.llm, model)
            reasoning_details = reasoning_request_details(messages, request_messages)
            image_count = sum(part.get("type") == "image_url" for item in request_messages
                              if isinstance(item.get("content"), list) for part in item["content"])
            request_id = f"request-{monotonic_ns()}"
            log_model_request(context.profile, context.session_id, model, request_id, "started", image_count,
                              request_details=reasoning_details)
            with trace_model_request(context.profile.llm.api_key,
                                     before_retry=lambda: _check_retry_interruption(context)) as trace:
                started = perf_counter()
                try:
                    response = client.chat.completions.create(
                        model=model, messages=request_messages,
                        tools=get_tool_schemas(context.profile, navigation_enabled=navigation_enabled),
                        tool_choice="auto", temperature=0.0, reasoning_effort="high",
                    )
                except BaseException as exc:
                    log_model_request(context.profile, context.session_id, model, request_id, "failed", image_count,
                                      elapsed_s=perf_counter() - started, error_type=type(exc).__name__,
                                      request_details={**trace.details(error=exc), **reasoning_details})
                    raise
                log_model_request(context.profile, context.session_id, model, request_id, "completed", image_count,
                                  elapsed_s=perf_counter() - started, usage=getattr(response, "usage", None),
                                  request_details={**trace.details(response=response), **reasoning_details,
                                                   'reasoning_output_chars': response_reasoning_chars(response)})
            log_context_usage(context.profile, context.session_id, model, getattr(response, "usage", None))
            message = response.choices[0].message
            tool_calls = message.tool_calls or []
        except EndCurrentTurn as exc:
            return _stop_with_message(context, str(exc), safety_stop=True)
        except Exception as exc:
            return _stop_with_message(context, f"模型请求或响应失败，已停止本轮：{type(exc).__name__}",
                                      safety_stop=True)
        finally:
            clear_search_images(messages)
        if not isinstance(tool_calls, (list, tuple)) or any(
            not isinstance(getattr(call, "id", None), str)
            or not isinstance(getattr(getattr(call, "function", None), "name", None), str)
            or not isinstance(getattr(getattr(call, "function", None), "arguments", None), str)
            for call in tool_calls
        ):
            return _stop_with_message(context, "模型工具响应结构无效，已停止本轮。", safety_stop=True)

        assistant_text = message.content or ""
        if not isinstance(assistant_text, str):
            return _stop_with_message(context, "模型文本响应结构无效，已停止本轮。", safety_stop=True)
        record_target_assessment(context, assistant_text, has_current_image=has_current_image)
        display_text = assistant_text
        messages.append(build_assistant_message(message, tool_calls))
        if display_text:
            print(f"agent> {display_text}")
            log_agent_message(context.profile, context.session_id, "assistant", display_text)
        if len(tool_calls) > 1:
            refusal = {"success": False, "error": "MULTIPLE_TOOL_CALLS_REJECTED",
                       "motion_executed": False,
                       "message": "本轮模型提出多个工具调用，全部拒绝且未执行；请每次只调用一个工具。"}
            _skip_navigation_calls(context, messages, tool_calls, refusal)
            corrections += 1
            if navigation_enabled and corrections <= MAX_CORRECTIONS:
                continue
            ending = refusal["message"] + ("纠错次数已用尽，已停止本轮。" if navigation_enabled else "")
            return _stop_with_message(context, ending, safety_stop=True)

        if not tool_calls:
            if plan is not None:
                if plan.status == "finishing" and assistant_text.strip():
                    return _finish_navigation_hold(context)
                plan.feedback = ("任务尚未结束，请继续执行当前段。" if plan.current is not None else
                                 "所有导航段已确认；用户要求降落时调用 land，否则用文字说明完成以结束本轮。")
                continue
            _record_task_state(context, "idle")
            if context.task_state is not None and "候选完成" in assistant_text:
                context.task_state.completion_candidate = assistant_text
            return assistant_text

        if plan is not None and plan.status == "finishing" and tool_calls[0].function.name not in {
                "get_state", "land", "update_navigation_plan"}:
            _skip_navigation_calls(context, messages, tool_calls, {
                "success": True, "motion_executed": False, "error": "NAVIGATION_FINISHING",
                "message": "导航段全部确认，请按原指令降落或用文字结束；需要继续导航时先更正清单。",
            })
            plan.feedback = "请按原指令降落或用文字结束，或通过计划工具纠正清单。"
            continue

        call = tool_calls[0]
        before_pose = _current_pose(context)
        try:
            tool_result = dispatch_tool_call(context, call)
        except EndCurrentTurn as exc:
            _append_turn_end_tool_results(messages, tool_calls, 0, exc)
            return _stop_with_message(context, str(exc), safety_stop=plan is not None)
        except SafetyHandoffRequired:
            context.navigation_resumable = False
            if plan is not None:
                plan.finish(completed=False, reason="飞行工具触发原 PX4 安全退出，导航未完成。")
                log_navigation_plan(context.profile, context.session_id, "incomplete", plan)
            raise
        recoverable = navigation_enabled and is_recoverable_rejection(call.function.name, tool_result)
        if recoverable:
            tool_result = {**tool_result, "motion_executed": False,
                           "recovery_hint": "未发布本次位置目标；请结合状态纠正参数或选择其他工具。"}
        view_changed = bool(tool_result.get("success")) and _motion_changed_view(call.function.name, tool_result)
        if view_changed:
            clear_target_assessment(context)
            clear_target_measurement(context)
            if context.profile.post_motion_wait_enabled:
                # 仅暂停规划线程，ROS executor 继续收数据、发布 setpoint。
                wait_s = context.profile.post_motion_wait_s
                print(f"tool> 动作完成，等待 {wait_s:g} 秒稳定后继续规划。")
                time.sleep(wait_s)
            tool_result["observation_current"] = False
            tool_result["observation_note"] = "动作前的图像仅供历史参考；请调用 observe 重新观察后再规划。"
        messages.append(_build_tool_message(call.id, tool_result, call.function.name))
        if call.function.name == "update_navigation_plan":
            if not tool_result.get("success"):
                if navigation_enabled and tool_result.get("error") in {
                        "INVALID_TOOL_ARGUMENTS", "INVALID_NAVIGATION_PLAN"}:
                    continue
                return _stop_with_message(context, "计划工具未成功，已停止本轮。", safety_stop=True)
            plan = context.navigation_plan
            if tool_result.get("compact_history"):
                compact_navigation_history(messages, task_start,
                                           observation_current=_navigation_observation_current(context))
            if plan.status == "finishing":
                if context.task_state is not None:
                    context.task_state.completion_candidate = plan.conversation_summary()
            elif context.task_state is not None:
                context.task_state.completion_candidate = None
            continue
        if call.function.name == "observe" and not tool_result.get("success"):
            clear_target_measurement(context)
            context.observation = None
            context.depth_rules = None
            if context.task_state is not None:
                context.task_state.clear_observation()
            _compact_history(messages, keep_latest=False)
            continue
        if call.function.name == "measure_distance":
            continue
        if call.function.name == "activate_skill" and tool_result.get("success"):
            try:
                messages.append(build_search_message(context, tool_result["prompt"]))
            except Exception:
                return _stop_with_message(context, "四向搜索图组无效，已停止本轮。", safety_stop=True)
            _compact_history(messages)
            continue
        if not tool_result.get("success"):
            if recoverable:
                corrections += 1
                if corrections <= MAX_CORRECTIONS:
                    continue
            return _stop_with_message(context, f"{call.function.name} 未成功，已停止本轮。", safety_stop=True)
        # 查询和清单更新不抵消飞行动作的纠错次数，避免反复查询绕过上限。
        if view_changed or call.function.name == "land":
            corrections = 0
        if call.function.name == "land":
            clear_target_measurement(context)
            context.navigation_resumable = False
            if plan is not None:
                completed = plan.status == "finishing"
                text = ("模型已确认全部子目标，降落完成，本轮已结束（未独立核验到达）。" if completed else
                        "降落完成，但导航子目标尚未全部确认，本轮已结束。")
                return _stop_with_message(context, text, navigation_complete=completed)
            return _stop_with_message(context, "降落完成，本轮已结束。")
        if call.function.name == "observe":
            snapshot = context.observation
            rules = context.depth_rules
            if snapshot is None or rules is None or rules.observation_id != snapshot.observation_id:
                return _stop_with_message(context, "observe 未提供同号 RGB-D 观测，已停止本轮。", safety_stop=True)
            try:
                append_observation(context, messages, snapshot, prompt=tool_result.get("prompt"), rules=rules)
            except Exception:
                clear_target_measurement(context)
                context.observation = None
                context.depth_rules = None
                return _stop_with_message(context, "observe 图像无效，已停止本轮。", safety_stop=True)
            _compact_history(messages)
            continue
        if context.task_state is not None:
            after_pose = _result_pose(tool_result) or _current_pose(context)
            context.task_state.record_motion_progress(
                call.function.name, before_pose, after_pose,
                rotation_degrees=tool_result.get("degrees"),
            )
        if view_changed:
            context.observation = None
            context.depth_rules = None
            if context.task_state is not None:
                context.task_state.clear_observation()
        _compact_history(messages)

    text = "导航模型请求预算耗尽，任务未完成，已停止。" if plan is not None else "本轮工具调用次数过多，已停止。"
    return _stop_with_message(context, text, safety_stop=True)


def _check_retry_interruption(context: ToolContext) -> None:
    """每次SDK重试发送前优先处理已到达的用户介入。"""
    interruption = interrupt_if_requested(context, hover_on_flight_tool=False)
    if interruption is not None:
        raise EndCurrentTurn(interruption['message'], interruption)


def _navigation_observation_current(context: ToolContext) -> bool:
    """向模型说明缓存是否仍为当前配对观测，不限制清单写入。"""
    return (context.observation is not None and context.depth_rules is not None
            and context.observation.observation_id == context.depth_rules.observation_id)


def _skip_navigation_calls(
    context: ToolContext, messages: list[dict[str, Any]], calls: list[Any], result: dict[str, Any],
) -> None:
    """记录未执行工具并补齐协议，压缩后仍可从日志核查。"""
    for call in calls:
        messages.append(_build_tool_message(call.id, result, call.function.name))
        log_tool_call(context.profile, context.session_id, call.function.name,
                      {"raw_arguments": call.function.arguments}, result)


def _finish_navigation_hold(context: ToolContext) -> str:
    """确认处于空中受控状态，再复用原悬停交接收尾。"""
    controller = context.controller
    flight_state = getattr(controller, "flight_state", None)
    mode = getattr(getattr(controller, "vehicle_status", None), "mode", None)
    if not callable(flight_state) or flight_state() != "IN_AIR" or mode not in {"OFFBOARD", "AUTO.LOITER"}:
        return _stop_with_message(context, "全部子目标已确认，但无法确认空中悬停状态，任务未完成。", safety_stop=True)
    return _stop_with_message(context, "模型已确认全部子目标，保持当前位置，本轮已结束（未独立核验到达）。",
                              safety_stop=True, navigation_complete=True)


def append_observation(
    context: ToolContext,
    messages: list[dict[str, Any]],
    snapshot: ObservationSnapshot,
    *,
    prompt: str | None = None,
    rules: Any | None = None,
) -> None:
    """把 observe 得到的同号图、提示和深度摘要追加给同一模型。"""
    if rules is None:
        rules = compute_depth_rules(snapshot, context.profile.observation,
                                    context.profile.forward_step_limit_m)
    message = build_observation_message(snapshot, rules, prompt)
    context.observation = snapshot
    context.depth_rules = rules
    if context.task_state is not None:
        context.task_state.set_observation(snapshot.observation_id)
    messages.append(message)
    log_observation(context.profile, context.session_id, snapshot.observation_id,
                    snapshot.rgb_stamp_ns, snapshot.depth_stamp_ns,
                    rules.depth_valid, rules.forward_max_m, rules.reason,
                    save_observation_image(context.profile, context.session_id, snapshot.observation_id, message),
                    view=snapshot.view,
                    down_rgb_stamp_ns=(snapshot.down_rgb.stamp_ns if snapshot.down_rgb is not None else
                                       snapshot.rgb_stamp_ns if snapshot.view == "down" else None))


def _compact_history(messages: list[dict[str, Any]], *, keep_latest: bool = True) -> None:
    """保留最新观察，旧图与自动文字一并失效，取图失败时清理全部观察。"""
    image_messages = [index for index, message in enumerate(messages)
                      if isinstance(message.get("content"), list)
                      and any(part.get("type") == "image_url" for part in message["content"])]
    for index in (image_messages[:-1] if keep_latest else image_messages):
        message = messages[index]
        messages[index] = {**message, "content": [{
            "type": "text", "text": "历史观察已失效，图片和自动深度信息已移除。",
        }]}


def _current_pose(context: ToolContext) -> tuple[float, float, float] | None:
    """读取控制器当前三轴位姿，供无图时计算动作进展。"""
    resolver = getattr(context.controller, "current_position_ned", None)
    try:
        pose = resolver() if callable(resolver) else None
    except Exception:
        return None
    return _valid_pose(pose)


def _valid_pose(pose: Any) -> tuple[float, float, float] | None:
    """只接受三个有限数值组成的 NED 位姿。"""
    if not isinstance(pose, (list, tuple)) or len(pose) != 3:
        return None
    if any(isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value)
           for value in pose):
        return None
    return tuple(float(value) for value in pose)


def _result_pose(result: dict[str, Any]) -> tuple[float, float, float] | None:
    """优先使用工具明确回报的最终位姿。"""
    return _valid_pose(result.get("final_position_ned"))


def _motion_changed_view(tool_name: str, result: dict[str, Any]) -> bool:
    """判断成功工具是否应清掉动作前的视觉与深度规则。"""
    if tool_name == "forward" and result.get("motion_executed") is False:
        return False
    if tool_name == "rotate":
        degrees = result.get("degrees")
        return (
            isinstance(degrees, (int, float))
            and not isinstance(degrees, bool)
            and math.isfinite(degrees)
            and degrees > 0
        )
    return tool_name in {"takeoff", "forward", "up", "down"}


def _stop_with_message(
    context: ToolContext, assistant_text: str, *, safety_stop: bool = False, navigation_complete: bool = False,
) -> str:
    """异常停止时先确认悬停，再输出不含原图和密钥的原因。"""
    try:
        if safety_stop:
            controller = context.controller
            flight_state = getattr(controller, "flight_state", None)
            status = getattr(controller, "vehicle_status", None)
            if flight_state is not None and flight_state() == "IN_AIR" and getattr(status, "mode", None) == "OFFBOARD":
                request_confirmed_hover(controller, action_name="agent loop stop")
    except SafetyHandoffRequired:
        context.navigation_resumable = False
        if context.navigation_plan is not None:
            context.navigation_plan.finish(completed=False, reason="悬停未确认，已交给原 PX4 安全退出流程。")
            log_navigation_plan(context.profile, context.session_id, "incomplete", context.navigation_plan)
        raise
    if context.navigation_plan is not None:
        context.navigation_plan.finish(completed=navigation_complete, reason=assistant_text)
        log_navigation_plan(context.profile, context.session_id, context.navigation_plan.status, context.navigation_plan)
        _record_task_state(context, "idle")
    print(f"agent> {assistant_text}")
    log_agent_message(context.profile, context.session_id, "assistant", assistant_text)
    return assistant_text


def _record_task_state(context: ToolContext, phase: str) -> None:
    """更新当前阶段，并同步打印和落盘。"""
    if context.task_state is None:
        return
    if phase == "thinking":
        context.task_state.set_thinking()
    elif phase == "idle":
        context.task_state.set_idle()
    print(format_task_state_line(context.task_state))
    log_task_state(context.profile, context.session_id, context.task_state)


def _append_turn_end_tool_results(
    messages: list[dict[str, Any]],
    tool_calls: list[Any],
    stopped_index: int,
    exc: EndCurrentTurn,
) -> None:
    """补齐当前 assistant tool_calls 的 tool 响应，避免下一轮请求非法。"""
    result = exc.tool_result or {
        "success": False,
        "error": "TURN_ABORTED",
        "message": str(exc),
    }
    stopped_call = tool_calls[stopped_index]
    messages.append(_build_tool_message(stopped_call.id, result, stopped_call.function.name))

    skipped_result = {
        "success": False,
        "error": "SKIPPED_DUE_TO_TURN_END",
        "message": str(exc),
    }
    for pending_call in tool_calls[stopped_index + 1 :]:
        messages.append(_build_tool_message(pending_call.id, skipped_result, pending_call.function.name))


def _build_tool_message(tool_call_id: str, result: dict[str, Any], tool_name: str) -> dict[str, Any]:
    """按工具精简成功反馈，失败保留完整模型视图，不修改原始结果。"""
    model_result = {key: value for key, value in result.items()
                    if key not in {"observation_id", "rgb_stamp_ns", "depth_stamp_ns", "compact_history"}}
    if result.get("success") is True and not result.get("error") and tool_name in MODEL_TOOL_RESULT_FIELDS:
        fields = MODEL_TOOL_RESULT_FIELDS[tool_name] | {"observation_current"}
        if result.get("depth_reason") or result.get("depth_valid") is False:
            fields = fields | {"depth_reason"}
        if tool_name == "forward" and result.get("motion_executed") is False:
            fields = fields - {"final_position_ned"}
        model_result = {key: value for key, value in model_result.items() if key in fields}
    return {
        "role": "tool",
        "tool_call_id": tool_call_id,
        "content": json.dumps(model_result, ensure_ascii=False),
    }
