"""记录主 VLM 的参照物判断，不触发动作或附加模型请求。"""

import json
from copy import deepcopy
from typing import Any


def current_goal_key(context: Any) -> tuple[str, ...]:
    """用当前任务及子目标内容隔离临时判断与实际测距。"""
    plan = context.navigation_plan
    current = plan.current if plan is not None else None
    if current is not None:
        return (plan.original_instruction, current.id, current.description, current.completion_condition)
    return (plan.original_instruction if plan is not None else context.navigation_instruction,)


def clear_target_assessment(context: Any, *, keep_history: bool = True) -> None:
    """清除当前判断，同段仅保留上一份判断供历史对照。"""
    if keep_history and context.target_assessment is not None:
        context.previous_target_assessment = deepcopy(context.target_assessment)
        context.previous_target_assessment_goal = context.target_assessment_goal
    elif not keep_history:
        context.previous_target_assessment = None
        context.previous_target_assessment_goal = None
    context.target_assessment = None
    context.target_assessment_goal = None


def discard_stale_target_assessment(context: Any) -> None:
    """子目标切换或内容变化后停止使用此前判断。"""
    key = current_goal_key(context)
    if context.target_assessment is not None and context.target_assessment_goal != key:
        clear_target_assessment(context, keep_history=False)
    if context.previous_target_assessment_goal != key:
        context.previous_target_assessment = None
        context.previous_target_assessment_goal = None
    if context.navigation_plan is None or not context.navigation_plan.terminal_stage:
        for record in (context.target_assessment, context.previous_target_assessment):
            if record is not None:
                record.pop("arrival", None)


def target_assessment_snapshot(context: Any) -> dict[str, Any]:
    """提供同段当前与上一份判断，来源失效后仅作为历史。"""
    discard_stale_target_assessment(context)
    current = context.target_assessment
    if current is not None:
        source = current["source"]
        observation_id = getattr(context.observation, "observation_id", None)
        if (source["type"] != "observation" or not observation_id
                or source.get("observation_id") != observation_id):
            clear_target_assessment(context)
    return deepcopy({"current": context.target_assessment, "previous": context.previous_target_assessment})


def record_target_assessment(context: Any, text: str, *, has_current_image: bool,
                             source: dict[str, Any] | None = None) -> None:
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
    fields = ("target_status", "candidate", "support", "conflict", "open_issue")
    terminal = context.navigation_plan is not None and context.navigation_plan.terminal_stage
    allowed = set(fields) | ({"arrival"} if terminal else set())
    if (not isinstance(value, dict) or not set(fields).issubset(value) or set(value) - allowed
            or not isinstance(value["target_status"], str)
            or value["target_status"] not in {"confirmed", "uncertain", "not_found"}
            or any(not isinstance(value[key], str) for key in fields)
            or (value["target_status"] != "not_found" and not value["candidate"].strip())
            or not any(value[key].strip() for key in ("support", "conflict", "open_issue"))):
        return
    previous = context.previous_target_assessment
    if previous is not None and previous["candidate"] != value["candidate"].strip():
        from drone_harness.runtime.target_measurement import clear_target_measurement

        clear_target_measurement(context)
    context.target_assessment = {key: value[key].strip() for key in fields}
    arrival = value.get("arrival")
    if (terminal and isinstance(arrival, dict) and set(arrival) == {"position_status", "evidence"}
            and isinstance(arrival["position_status"], str)
            and arrival["position_status"] in {"met", "unmet", "unknown"}
            and isinstance(arrival["evidence"], str) and arrival["evidence"].strip()):
        context.target_assessment["arrival"] = {key: val.strip() for key, val in arrival.items()}
    context.target_assessment["source"] = deepcopy(source) if source is not None else {
        "type": "observation", "observation_id": getattr(context.observation, "observation_id", None),
    }
    context.target_assessment_goal = current_goal_key(context)
