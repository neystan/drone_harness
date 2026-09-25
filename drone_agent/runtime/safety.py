"""集中处理 agent 侧的安全判定。"""

from __future__ import annotations

import time
from typing import Any

from drone_agent.config.schema import RuntimeProfile


FLIGHT_TOOL_NAMES = {
    "takeoff",
    "land",
    "disarm",
    "return_home",
    "rotate",
    "move",
}

SKILL_ACTIVATION_END_ERRORS = {
    "SKILL_NOT_FOUND",
    "SKILL_DISABLED",
    "SKILL_MODE_MISMATCH",
    "SKILL_ACTIVATION_DECLINED",
    "SKILL_ACTIVATION_UNAVAILABLE",
}


class EndCurrentTurn(RuntimeError):
    """用于立即结束当前 agent 执行轮次。"""

    def __init__(self, message: str, tool_result: dict[str, Any] | None = None) -> None:
        super().__init__(message)
        self.tool_result = tool_result


class SafetyHandoffRequired(RuntimeError):
    """表示 Agent 必须退出以交给 PX4 Offboard-loss failsafe。"""


def request_confirmed_hover(
    controller: Any,
    timeout_s: float = 2.0,
    action_name: str | None = None,
) -> dict[str, Any]:
    """请求悬停并确认 PX4 已进入 AUTO_LOITER。"""
    deadline = time.monotonic() + max(0.0, timeout_s)
    reason_prefix = f"{action_name} timeout; " if action_name else ""
    try:
        request = controller.send_hover_command()
        ack_timeout = max(0.0, deadline - time.monotonic())
        ack = controller.wait_for_command_ack(request, timeout_s=ack_timeout)
        if ack is not None and not _is_ack_accepted(controller, ack):
            result_name = _ack_result_name(controller, ack)
            raise SafetyHandoffRequired(
                "SAFETY_HANDOFF_REQUIRED:\n"
                f"{reason_prefix}PX4 rejected AUTO_LOITER command ({result_name}); "
                "Agent exiting to trigger PX4 Offboard-loss failsafe."
            )

        loiter_state = _nav_state_constant(controller, "NAVIGATION_STATE_AUTO_LOITER", 4)
        state_timeout = max(0.0, deadline - time.monotonic())
        state_confirmed = controller.wait_for_nav_state(loiter_state, timeout_s=state_timeout)
        if not state_confirmed:
            ack_detail = " PX4 ACK timeout;" if ack is None else ""
            raise SafetyHandoffRequired(
                "SAFETY_HANDOFF_REQUIRED:\n"
                f"{reason_prefix}AUTO_LOITER was not confirmed;{ack_detail} "
                "Agent exiting to trigger PX4 Offboard-loss failsafe."
            )
        controller.stop_position_hold()
        return {
            "safety_state": "HOLD_CONFIRMED",
            "ack_received": ack is not None,
            "message": "PX4 AUTO_LOITER confirmed",
        }
    except SafetyHandoffRequired:
        controller.stop_position_hold()
        raise
    except Exception as exc:
        controller.stop_position_hold()
        raise SafetyHandoffRequired(
            "SAFETY_HANDOFF_REQUIRED:\n"
            f"{reason_prefix}failed to confirm AUTO_LOITER: {exc}; "
            "Agent exiting to trigger PX4 Offboard-loss failsafe."
        ) from exc


def _is_ack_accepted(controller: Any, ack: Any) -> bool:
    """调用控制器的 ACK 判断，测试替身缺少该方法时回退到 PX4 常量。"""
    checker = getattr(controller, "is_command_ack_accepted", None)
    if checker is not None:
        return bool(checker(ack))
    return int(getattr(ack, "result", -1)) == 0


def _ack_result_name(controller: Any, ack: Any) -> str:
    """读取 ACK 的稳定结果名称。"""
    formatter = getattr(controller, "command_ack_result_name", None)
    if formatter is not None:
        return str(formatter(ack))
    return str(getattr(ack, "result_name", getattr(ack, "result", "UNKNOWN")))


def _nav_state_constant(controller: Any, name: str, fallback: int) -> int:
    """读取 PX4 状态常量。"""
    resolver = getattr(controller, "nav_state_constant", None)
    if resolver is not None:
        return int(resolver(name, fallback))
    return int(getattr(controller.vehicle_status.__class__, name, fallback))


def requires_human_in_the_loop(profile: RuntimeProfile, tool_name: str) -> bool:
    """判断当前工具和模式是否需要人工确认后才能执行。"""
    if not profile.safety.human_in_the_loop_for_flight_tools:
        return False
    return (
        tool_name in FLIGHT_TOOL_NAMES
        and tool_name not in profile.safety.human_in_the_loop_exempt_flight_tools
    )


def should_end_turn_after_tool_result(result: dict[str, Any]) -> bool:
    """判断工具结果是否应直接结束当前轮。"""
    error = str(result.get("error", "")).strip()
    return (
        error.endswith("_TIMEOUT")
        or error == "INTERRUPTED_BY_USER"
        or error in SKILL_ACTIVATION_END_ERRORS
    )
