"""处理任务续接和可纠正拒绝，不恢复旧图或动作授权。"""

from typing import Any


def is_recoverable_rejection(tool: str, result: dict[str, Any]) -> bool:
    """白名单只包含已核对为位置目标下发前返回的拒绝。"""
    return (not result.get("success") and result.get("motion_executed") is not True
            and ((tool == "takeoff" and result.get("error") == "ALREADY_IN_AIR")
                 or (tool == "down" and result.get("error") == "TARGET_Z_TOO_LOW")))


def input_directive(text: str) -> str:
    """仅识别明确的取消短句和新任务前缀，不猜测一般自然语言。"""
    value = text.strip().rstrip("。！!.").strip()
    if value in {"取消任务", "取消当前任务", "停止任务", "停止当前任务", "停止", "停下"}:
        return "cancel"
    if value.startswith(("新任务：", "新任务:", "换个任务：", "换个任务:")):
        return "replace"
    return "guide"


def prepare_navigation_turn(context: Any, user_input: str, *, replace_task: bool = False) -> bool:
    """续接未完成清单，刷新观测和审批但不重置无进展计数。"""
    plan = context.navigation_plan
    resume = (context.navigation_enabled and not replace_task and context.navigation_resumable
              and plan is not None and plan.status in {"running", "finishing", "incomplete"})
    context.observation = None
    context.depth_rules = None
    if resume:
        plan.status = "finishing" if plan.completed_count == len(plan.subgoals) else "running"
        plan.stop_reason = ""
        plan.feedback = "用户提供了新的引导，请结合原任务继续；先重新确认状态与观察。"
        context.navigation_instruction = plan.original_instruction
        if context.task_state is not None:
            state = context.task_state
            state.set_thinking()
            state.current_user_goal = plan.original_instruction
            state.clear_intervention()
            state.clear_observation()
            state.landing_authorized = False
            state.completion_candidate = None
    else:
        context.navigation_plan = None
        context.navigation_instruction = user_input
        context.navigation_resumable = True
        if context.task_state is not None:
            context.task_state.start_new_goal(user_input)
    return bool(resume)
