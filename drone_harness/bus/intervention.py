"""集中处理用户语言介入。"""

from __future__ import annotations

from typing import Any


INTERRUPTED_BY_USER = "INTERRUPTED_BY_USER"


def should_interrupt(context: Any) -> bool:
    """判断当前是否存在待处理的用户介入。"""
    message_bus = getattr(context, "message_bus", None)
    if message_bus is None:
        return False
    return message_bus.has_pending_user_message()


def consume_intervention(context: Any) -> str | None:
    """取出一条用户介入消息并更新任务状态。"""
    message_bus = getattr(context, "message_bus", None)
    if message_bus is None:
        return None
    message = message_bus.get_next_user_message()
    if message is None:
        return None
    if context.task_state is not None:
        context.task_state.mark_intervention(message.content)
    print(f"intervention> 收到用户介入：{message.content}")
    return message.content


def build_interrupted_result(
    intervention_message: str,
    final_position_ned: list[float] | None = None,
) -> dict[str, Any]:
    """构造用户介入导致的工具中断结果。"""
    result: dict[str, Any] = {
        "success": False,
        "error": INTERRUPTED_BY_USER,
        "message": "tool interrupted by user input",
        "intervention_message": intervention_message,
    }
    if final_position_ned is not None:
        result["final_position_ned"] = final_position_ned
    return result


def interrupt_if_requested(context: Any, *, hover_on_flight_tool: bool) -> dict[str, Any] | None:
    """检测介入消息，必要时执行悬停并返回中断结果。"""
    if not should_interrupt(context):
        return None
    intervention_message = consume_intervention(context)
    if intervention_message is None:
        return None
    controller = getattr(context, "controller", None)
    if hover_on_flight_tool and controller is not None:
        controller.stop_position_hold()
        controller.send_hover_command()
    final_position = controller.current_position_ned() if controller is not None else None
    return build_interrupted_result(intervention_message, final_position)
