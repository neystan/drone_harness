"""记录主 VLM 的参照物判断，不触发动作或附加模型请求。"""

import json
from typing import Any


def _goal_key(context: Any) -> tuple[str, ...]:
    """用当前任务及子目标内容隔离临时判断，避免串到下一目标。"""
    plan = context.navigation_plan
    current = plan.current if plan is not None else None
    if current is not None:
        return (plan.original_instruction, current.id, current.description, current.completion_condition)
    return (plan.original_instruction if plan is not None else context.navigation_instruction,)


def clear_target_assessment(context: Any) -> None:
    """新观察、动作或新任务开始后清除临时目标判断。"""
    context.target_assessment = None
    context.target_assessment_goal = None


def discard_stale_target_assessment(context: Any) -> None:
    """子目标切换或内容变化后停止使用此前判断。"""
    if getattr(context, "target_assessment_goal", None) != _goal_key(context):
        clear_target_assessment(context)


def record_target_assessment(context: Any, text: str, *, has_current_image: bool) -> None:
    """只记录同次看图回复中的合法判断，缺失或格式错误不补造结论。"""
    discard_stale_target_assessment(context)
    if not has_current_image:
        return
    clear_target_assessment(context)
    candidate = text.lstrip()
    if candidate.startswith("```"):
        candidate = candidate.partition("\n")[2].lstrip()
    try:
        value, _end = json.JSONDecoder().raw_decode(candidate)
    except (json.JSONDecodeError, ValueError):
        return
    if (not isinstance(value, dict) or set(value) != {"target_status", "message"}
            or not isinstance(value["target_status"], str)
            or value["target_status"] not in {"confirmed", "uncertain", "not_found"}
            or not isinstance(value["message"], str) or not value["message"].strip()):
        return
    context.target_assessment = {"target_status": value["target_status"], "message": value["message"].strip()}
    context.target_assessment_goal = _goal_key(context)
