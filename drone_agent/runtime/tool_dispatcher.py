"""解析并分发模型返回的工具调用。"""

from __future__ import annotations

import json
import math
from typing import Any

from drone_agent.bus.intervention import interrupt_if_requested
from drone_agent.logging.task_log import log_task_state, log_tool_call
from drone_agent.runtime.safety import (
    EndCurrentTurn,
    FLIGHT_TOOL_NAMES,
    requires_human_in_the_loop,
    should_end_turn_after_tool_result,
)
from drone_agent.runtime.task_state import format_task_state_line
from drone_agent.tools.registry import ToolContext, get_tool_definition


def dispatch_tool_call(context: ToolContext, call: Any) -> dict:
    """解析并执行一次模型返回的工具调用。"""
    tool_name = call.function.name
    raw_arguments = call.function.arguments or "{}"
    is_flight_tool = tool_name in FLIGHT_TOOL_NAMES
    print(f"tool> calling {tool_name} args={raw_arguments}")

    definition = get_tool_definition(tool_name)
    if definition is None:
        result = {
            "success": False,
            "error": "UNSUPPORTED_TOOL",
            "message": f"unsupported tool: {tool_name}",
        }
        log_tool_call(
            context.profile,
            context.session_id,
            tool_name,
            {"raw_arguments": raw_arguments},
            result,
        )
        _update_task_state(context, "tool_finished", tool_name, result=result)
        return result

    try:
        arguments = _parse_tool_arguments(raw_arguments)
    except (json.JSONDecodeError, ValueError) as exc:
        result = {
            "success": False,
            "error": "INVALID_TOOL_ARGUMENTS",
            "message": f"failed to parse tool arguments: {exc}",
        }
        log_tool_call(
            context.profile,
            context.session_id,
            tool_name,
            {"raw_arguments": raw_arguments},
            result,
        )
        _update_task_state(context, "tool_finished", tool_name, result=result)
        return result

    if not isinstance(arguments, dict):
        result = {
            "success": False,
            "error": "INVALID_TOOL_ARGUMENTS",
            "message": "tool arguments must be a JSON object",
        }
        log_tool_call(context.profile, context.session_id, tool_name, arguments, result)
        _update_task_state(context, "tool_finished", tool_name, result=result)
        return result

    #用户输入介入
    result = interrupt_if_requested(context, hover_on_flight_tool=is_flight_tool)
    if result is not None:
        log_tool_call(context.profile, context.session_id, tool_name, arguments, result)
        _update_task_state(context, "interrupted", tool_name, result=result)
        raise EndCurrentTurn(result["message"], result)

    #要求用户确认
    if requires_human_in_the_loop(context.profile, tool_name):
        _update_task_state(
            context,
            "waiting_for_confirmation",
            tool_name,
            arguments=arguments,
            is_flight_tool=is_flight_tool,
        )
        try:
            _confirm_flight_tool(context, tool_name, arguments)
        except EndCurrentTurn as exc:
            result = exc.tool_result or {
                "success": False,
                "error": "HUMAN_IN_THE_LOOP_DECLINED",
                "message": str(exc),
            }
            log_tool_call(context.profile, context.session_id, tool_name, arguments, result)
            _update_task_state(context, "interrupted", tool_name, result=result)
            raise EndCurrentTurn(str(exc), result) from exc

    _update_task_state(
        context,
        "tool_running",
        tool_name,
        arguments=arguments,
        is_flight_tool=is_flight_tool,
    )
    result = definition.handler(context, arguments)
    log_tool_call(context.profile, context.session_id, tool_name, arguments, result)
    _update_task_state(context, "tool_finished", tool_name, result=result)
    if should_end_turn_after_tool_result(result):
        _update_task_state(context, "interrupted", tool_name, result=result)
        raise EndCurrentTurn(
            result.get("message", "当前工具执行超时，本轮已结束。"),
            result,
        )
    return result


def _parse_tool_arguments(raw_arguments: str) -> Any:
    """解析工具参数并拒绝所有非有限 JSON 数值。"""
    arguments = json.loads(raw_arguments, parse_constant=_reject_json_constant)
    if _contains_non_finite_number(arguments):
        raise ValueError("tool arguments contain a non-finite number")
    return arguments


def _reject_json_constant(value: str) -> None:
    """拒绝 JSON 标准之外的 NaN 和 Infinity 常量。"""
    raise ValueError(f"non-finite JSON number is not allowed: {value}")


def _contains_non_finite_number(value: Any) -> bool:
    """递归检查解析后的 JSON 值。"""
    if isinstance(value, float):
        return not math.isfinite(value)
    if isinstance(value, list):
        return any(_contains_non_finite_number(item) for item in value)
    if isinstance(value, dict):
        return any(_contains_non_finite_number(item) for item in value.values())
    return False


def _update_task_state(
    context: ToolContext,
    phase: str,
    tool_name: str,
    arguments: dict[str, Any] | None = None,
    is_flight_tool: bool = False,
    result: dict[str, Any] | None = None,
) -> None:
    """按阶段统一更新工具相关状态。"""
    if context.task_state is None:
        return
    if phase == "waiting_for_confirmation":
        context.task_state.set_waiting_for_confirmation(
            tool_name,
            arguments or {},
            is_flight_tool,
        )
    elif phase == "tool_running":
        context.task_state.start_tool(
            tool_name,
            arguments or {},
            is_flight_tool,
        )
    elif phase == "tool_finished":
        context.task_state.finish_tool(tool_name, result or {})
    elif phase == "interrupted":
        context.task_state.interrupt(tool_name, result or {})
    else:
        return
    _record_task_state(context)


def _confirm_flight_tool(
    context: ToolContext,
    tool_name: str,
    arguments: dict[str, Any],
) -> None:
    """通过消息总线等待飞行工具人工确认。"""
    if context.message_bus is None:
        raise EndCurrentTurn(
            f"已取消本次 {tool_name} 执行。",
            {
                "success": False,
                "error": "HUMAN_IN_THE_LOOP_UNAVAILABLE",
                "message": "message bus is unavailable for human-in-the-loop confirmation",
            },
        )
    prompt = (
        f"human-in-the-loop> {tool_name} args={json.dumps(arguments, ensure_ascii=False)} "
        "| 执行该飞行动作？[Y/N]: "
    )
    print(prompt, flush=True)
    while True:
        answer = context.message_bus.consume_user_message().content.strip().lower()
        if answer == "y":
            return
        if answer == "n":
            raise EndCurrentTurn(
                f"已取消本次 {tool_name} 执行。",
                {
                    "success": False,
                    "error": "HUMAN_IN_THE_LOOP_DECLINED",
                    "message": f"已取消本次 {tool_name} 执行。",
                },
            )
        print("human-in-the-loop> 请输入 Y 或 N。")


def _record_task_state(context: ToolContext) -> None:
    """把当前工具状态打印到终端并写入日志。"""
    if context.task_state is None:
        return
    print(format_task_state_line(context.task_state))
    log_task_state(context.profile, context.session_id, context.task_state)
