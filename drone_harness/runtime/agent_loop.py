"""管理大模型与工具调用循环。"""

from __future__ import annotations

import json
import time
from typing import Any

from drone_harness.logging.task_log import log_agent_message, log_observation, log_task_state
from drone_harness.runtime.observation import ObservationSnapshot, build_observation_message
from drone_harness.runtime.safety import EndCurrentTurn, request_confirmed_hover
from drone_harness.runtime.task_state import format_task_state_line
from drone_harness.runtime.tool_dispatcher import dispatch_tool_call
from drone_harness.tools.registry import ToolContext, get_tool_schemas
from drone_harness.vision.depth_rules import compute_depth_rules


MAX_TOOL_CALLS_PER_TURN = 50
MAX_CONSECUTIVE_NO_PROGRESS = 3


def agent_loop(
    client: Any,
    model: str,
    messages: list[dict[str, Any]],
    context: ToolContext,
) -> str:
    """每轮只执行零或一个动作，并在下一轮前注入新 RGB-D。"""
    if context.observation is None or context.depth_rules is None:
        return _stop_with_message(context, "没有可用的新 RGB 观测，本轮未请求模型。")
    for _ in range(MAX_TOOL_CALLS_PER_TURN):
        if context.task_state is not None and context.task_state.consecutive_no_progress >= MAX_CONSECUTIVE_NO_PROGRESS:
            return _stop_with_message(context, "连续动作没有可测位移，已停止自主规划。", safety_stop=True)
        _record_task_state(context, "thinking")
        try:
            response = client.chat.completions.create(
                model=model,
                messages=messages,
                tools=get_tool_schemas(),
                tool_choice="auto",
                temperature=0.0,
            )
            message = response.choices[0].message
            tool_calls = message.tool_calls or []
        except Exception as exc:
            return _stop_with_message(context, f"模型请求或响应失败，已停止本轮：{type(exc).__name__}",
                                      safety_stop=True)
        if not isinstance(tool_calls, (list, tuple)) or any(
            not isinstance(getattr(call, "id", None), str)
            or not isinstance(getattr(getattr(call, "function", None), "name", None), str)
            or not isinstance(getattr(getattr(call, "function", None), "arguments", None), str)
            for call in tool_calls
        ):
            return _stop_with_message(context, "模型工具响应结构无效，已停止本轮。", safety_stop=True)

        if not tool_calls:
            _record_task_state(context, "idle")
            assistant_text = message.content or ""
            if not isinstance(assistant_text, str):
                return _stop_with_message(context, "模型文本响应结构无效，已停止本轮。", safety_stop=True)
            print(f"agent> {assistant_text}")
            messages.append({"role": "assistant", "content": assistant_text})
            log_agent_message(context.profile, context.session_id, "assistant", assistant_text)
            if context.task_state is not None and "候选完成" in assistant_text:
                context.task_state.completion_candidate = assistant_text
            return assistant_text

        messages.append(_assistant_tool_message(message, tool_calls))
        if len(tool_calls) != 1:
            refusal = {"success": False, "error": "MULTIPLE_TOOL_CALLS_REJECTED",
                       "message": "本轮模型提出多个动作，全部拒绝且未执行。"}
            for call in tool_calls:
                messages.append(_build_tool_message(call.id, refusal))
            return _stop_with_message(context, refusal["message"], safety_stop=True)

        call = tool_calls[0]
        before_pose = context.observation.pose_ned
        try:
            tool_result = dispatch_tool_call(context, call)
        except EndCurrentTurn as exc:
            _append_turn_end_tool_results(messages, tool_calls, 0, exc)
            return _stop_with_message(context, str(exc))
        messages.append(_build_tool_message(call.id, tool_result))
        if not tool_result.get("success"):
            return _stop_with_message(context, f"{call.function.name} 未成功，已停止本轮。", safety_stop=True)
        if call.function.name == "land":
            return _stop_with_message(context, "降落完成，本轮已结束。")

        action_end_stamp_ns = time.time_ns()
        wait_for_observation = getattr(context.controller, "wait_for_observation", None)
        try:
            snapshot = (
                wait_for_observation(after_stamp_ns=action_end_stamp_ns)
                if wait_for_observation is not None else None
            )
        except Exception:
            snapshot = None
        min_new_stamp_ns = action_end_stamp_ns + int(context.profile.observation.max_clock_skew_s * 1e9)
        if snapshot is None or snapshot.rgb_stamp_ns <= min_new_stamp_ns:
            context.observation = None
            context.depth_rules = None
            return _stop_with_message(context, "动作后没有新 RGB 观测，已停止自主规划。", safety_stop=True)
        try:
            append_observation(context, messages, snapshot)
        except ValueError:
            context.observation = None
            context.depth_rules = None
            return _stop_with_message(context, "动作后观测无效，已停止自主规划。", safety_stop=True)
        if context.task_state is not None:
            context.task_state.record_motion_progress(call.function.name, before_pose, snapshot.pose_ned)
        _compact_history(messages)

    return _stop_with_message(context, "本轮工具调用次数过多，已停止。", safety_stop=True)


def append_observation(
    context: ToolContext,
    messages: list[dict[str, Any]],
    snapshot: ObservationSnapshot,
) -> None:
    """绑定同号深度规则并追加一条图片和摘要同在的消息。"""
    rules = compute_depth_rules(snapshot, context.profile.observation,
                                context.profile.safety.max_relative_move_m)
    message = build_observation_message(snapshot, rules)
    context.observation = snapshot
    context.depth_rules = rules
    if context.task_state is not None:
        context.task_state.set_observation(snapshot.observation_id)
    messages.append(message)
    log_observation(context.profile, context.session_id, snapshot.observation_id,
                    snapshot.rgb_stamp_ns, snapshot.depth_stamp_ns,
                    rules.depth_valid, rules.forward_max_m, rules.reason)


def _assistant_tool_message(message: Any, tool_calls: list[Any]) -> dict[str, Any]:
    """完整保留一个模型回复中的工具调用，保证后续结果可配对。"""
    return {
        "role": "assistant",
        "content": message.content or "",
        "tool_calls": [
            {"id": call.id, "type": "function",
             "function": {"name": call.function.name, "arguments": call.function.arguments}}
            for call in tool_calls
        ],
    }


def _compact_history(messages: list[dict[str, Any]]) -> None:
    """只留目标和最近一次动作前后两张图及配对工具结果。"""
    if len(messages) > 6:
        messages[:] = messages[:2] + messages[-4:]


def _stop_with_message(context: ToolContext, assistant_text: str, *, safety_stop: bool = False) -> str:
    """异常停止时先确认悬停，再输出不含原图和密钥的原因。"""
    if safety_stop:
        controller = context.controller
        flight_state = getattr(controller, "flight_state", None)
        status = getattr(controller, "vehicle_status", None)
        if flight_state is not None and flight_state() == "IN_AIR" and getattr(status, "mode", None) == "OFFBOARD":
            request_confirmed_hover(controller, action_name="agent loop stop")
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
    messages.append(_build_tool_message(tool_calls[stopped_index].id, result))

    skipped_result = {
        "success": False,
        "error": "SKIPPED_DUE_TO_TURN_END",
        "message": str(exc),
    }
    for pending_call in tool_calls[stopped_index + 1 :]:
        messages.append(_build_tool_message(pending_call.id, skipped_result))


def _build_tool_message(tool_call_id: str, result: dict[str, Any]) -> dict[str, Any]:
    """构造符合 OpenAI tool_call 协议的 tool message。"""
    return {
        "role": "tool",
        "tool_call_id": tool_call_id,
        "content": json.dumps(result, ensure_ascii=False),
    }
