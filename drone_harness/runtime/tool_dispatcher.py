"""解析并分发模型返回的工具调用。"""

from __future__ import annotations

import json
import math
import time
from datetime import datetime, timedelta
from typing import Any

from drone_harness.bus.intervention import interrupt_if_requested
from drone_harness.logging.task_log import log_task_state, log_tool_call
from drone_harness.runtime.safety import (
    EndCurrentTurn,
    FLIGHT_TOOL_NAMES,
    requires_human_in_the_loop,
    should_end_turn_after_tool_result,
)
from drone_harness.runtime.task_state import format_task_state_line
from drone_harness.runtime.task_memory import is_recoverable_rejection
from drone_harness.tools.registry import ToolContext, get_tool_definition
from drone_harness.tools import flight

HITL_CONFIRM_TIMEOUT_S = 120.0


def dispatch_tool_call(context: ToolContext, call: Any) -> dict:
    """解析并执行一次模型返回的工具调用。"""
    tool_name = call.function.name
    raw_arguments = call.function.arguments or "{}"
    is_flight_tool = tool_name in FLIGHT_TOOL_NAMES
    print(f"tool> calling {tool_name} args={raw_arguments}")

    definition = get_tool_definition(tool_name)
    if tool_name == "update_navigation_plan" and not context.navigation_enabled:
        result = {"success": False, "error": "NAVIGATION_DISABLED", "plan_changed": False}
        log_tool_call(context.profile, context.session_id, tool_name, {"raw_arguments": raw_arguments}, result)
        return result
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
        if tool_name == "update_navigation_plan":
            result["plan_changed"] = False
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
        if tool_name == "update_navigation_plan":
            result["plan_changed"] = False
        log_tool_call(context.profile, context.session_id, tool_name, arguments, result)
        _update_task_state(context, "tool_finished", tool_name, result=result)
        return result

    if tool_name == "forward":
        precheck = flight.validate_forward(context, arguments.get("distance_m"))
        if precheck is not None:
            log_tool_call(context.profile, context.session_id, tool_name, arguments, precheck)
            _update_task_state(context, "tool_finished", tool_name, result=precheck)
            return precheck

    #用户输入介入
    result = interrupt_if_requested(context, hover_on_flight_tool=is_flight_tool)
    if result is not None:
        log_tool_call(context.profile, context.session_id, tool_name, arguments, result)
        _update_task_state(context, "interrupted", tool_name, result=result)
        raise EndCurrentTurn(result["message"], result)

    #要求用户确认
    needs_confirmation = requires_human_in_the_loop(context.profile, tool_name)
    if tool_name == "land" and context.profile.mode == "simulation":
        # 仿真降落直接复用原飞控检查，不额外要求观测绑定或人工确认。
        needs_confirmation = False
    elif tool_name == "land" and not bool(getattr(context.task_state, "landing_authorized", False)):
        needs_confirmation = True
    if needs_confirmation:
        initial_state = _approval_state(context)
        if initial_state is None:
            result = {"success": False, "error": "APPROVAL_OBSERVATION_INVALID",
                      "message": "approval requires a bound observation and flight state"}
            log_tool_call(context.profile, context.session_id, tool_name, arguments, result)
            _update_task_state(context, "tool_finished", tool_name, result=result)
            return result
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
        if _approval_state(context) != initial_state:
            result = {"success": False, "error": "APPROVAL_STATE_CHANGED",
                      "message": "bound observation, limit or flight state changed during approval"}
            log_tool_call(context.profile, context.session_id, tool_name, arguments, result)
            _update_task_state(context, "tool_finished", tool_name, result=result)
            return result
        if tool_name == "land" and context.task_state is not None:
            context.task_state.landing_authorized = True
        if tool_name == "forward":
            postcheck = flight.validate_forward(context, arguments.get("distance_m"))
            if postcheck is not None:
                log_tool_call(context.profile, context.session_id, tool_name, arguments, postcheck)
                _update_task_state(context, "tool_finished", tool_name, result=postcheck)
                return postcheck

    _update_task_state(
        context,
        "tool_running",
        tool_name,
        arguments=arguments,
        is_flight_tool=is_flight_tool,
    )
    result = definition.handler(context, arguments)
    if context.navigation_enabled and is_recoverable_rejection(tool_name, result):
        result = {**result, "motion_executed": False}
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
    """按独立于观测帧龄的期限等待逐动作人工确认。"""
    if context.message_bus is None:
        raise EndCurrentTurn(
            f"已取消本次 {tool_name} 执行。",
            {
                "success": False,
                "error": "HUMAN_IN_THE_LOOP_UNAVAILABLE",
                "message": "message bus is unavailable for human-in-the-loop confirmation",
            },
        )
    snapshot = context.observation
    expiry = time.monotonic() + HITL_CONFIRM_TIMEOUT_S
    expiry_label = (datetime.now().astimezone() + timedelta(seconds=HITL_CONFIRM_TIMEOUT_S)).isoformat(
        timespec="seconds")
    if tool_name == "forward":
        limit = f"{min(context.depth_rules.forward_max_m, context.profile.forward_step_limit_m):.2f}m"
    elif tool_name == "takeoff":
        limit = f"{context.profile.safety.max_takeoff_height_m:.2f}m"
    elif tool_name in {"up", "down"}:
        limit = f"{context.profile.safety.max_vertical_move_m:.2f}m/次"
    elif tool_name == "rotate":
        limit = f"{context.profile.safety.max_rotation_deg:.1f}deg"
    else:
        limit = "仅本次明确批准"
    prompt = (
        f"human-in-the-loop> 动作={tool_name} 参数={json.dumps(arguments, ensure_ascii=False)} "
        f"观测号={snapshot.observation_id} 上限={limit} "
        f"确认期限至={expiry_label} "
        "| 执行该飞行动作？[Y/N]: "
    )
    print(prompt, flush=True)
    while time.monotonic() <= expiry:
        response = context.message_bus.get_next_user_message()
        if response is None:
            time.sleep(0.05)
            continue
        answer = response.content.strip().lower()
        if answer == "y":
            if time.monotonic() <= expiry:
                return
            break
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
    raise EndCurrentTurn(
        f"{tool_name} 人工确认已过期限，未执行。",
        {"success": False, "error": "HUMAN_IN_THE_LOOP_EXPIRED",
         "message": "approval expired before confirmation"},
    )


def _approval_state(context: ToolContext) -> tuple[Any, ...] | None:
    """绑定 A 观测、数值上限与飞控状态，不按等待帧龄重算深度。"""
    snapshot = context.observation
    rules = context.depth_rules
    if snapshot is None or rules is None or rules.observation_id != snapshot.observation_id:
        return None
    if snapshot.rgb_stamp_ns <= 0:
        return None
    status = getattr(context.controller, "vehicle_status", None)
    resolver = getattr(context.controller, "flight_state", None)
    if status is None or resolver is None:
        return None
    return (
        snapshot.observation_id,
        id(snapshot),
        snapshot.rgb_stamp_ns,
        snapshot.depth_stamp_ns,
        id(rules),
        rules.depth_valid,
        rules.forward_max_m,
        rules.reason,
        context.profile.forward_step_limit_m,
        context.profile.safety.max_takeoff_height_m,
        context.profile.safety.max_vertical_move_m,
        context.profile.safety.max_rotation_deg,
        bool(getattr(status, "connected", False)),
        bool(getattr(status, "armed", False)),
        getattr(status, "mode", None),
        resolver(),
    )


def _record_task_state(context: ToolContext) -> None:
    """把当前工具状态打印到终端并写入日志。"""
    if context.task_state is None:
        return
    print(format_task_state_line(context.task_state))
    log_task_state(context.profile, context.session_id, context.task_state)
