"""保存任务续接所需的最小事实，不恢复旧图或动作授权。"""

from copy import deepcopy
from datetime import datetime, timezone
import json
from typing import Any

FLIGHT_TOOLS = {"takeoff", "forward", "up", "down", "rotate", "land"}


def is_recoverable_rejection(tool: str, result: dict[str, Any]) -> bool:
    """白名单只包含已核对为位置目标下发前返回的拒绝。"""
    return (not result.get("success") and result.get("motion_executed") is not True
            and ((tool == "takeoff" and result.get("error") == "ALREADY_IN_AIR")
                 or (tool == "down" and result.get("error") == "TARGET_Z_TOO_LOW")))


def record_execution_fact(context: Any, tool: str, result: dict[str, Any]) -> None:
    """只从实际工具反馈提取有界摘要，标明记录时间与未知状态。"""
    if not context.navigation_enabled:
        return
    record = {"tool": tool, "recorded_at": datetime.now(timezone.utc).isoformat(),
              "result": {key: deepcopy(result[key]) for key in (
                  "success", "error", "motion_executed", "final_position_ned", "position_ned_m",
                  "connected", "armed", "mode", "in_air", "degrees") if key in result}}
    if tool in FLIGHT_TOOLS:
        context.execution_facts["last_motion"] = record
        if result.get("success") and result.get("motion_executed") is not False:
            context.execution_facts["last_successful_motion"] = record
    if tool == "get_state" and result.get("success"):
        context.execution_facts["last_state"] = record
    if not result.get("success"):
        context.execution_facts["last_failure"] = record


def inject_execution_facts(messages: list[dict[str, Any]], facts: dict[str, Any]) -> list[dict[str, Any]]:
    """每次请求注入最新摘要，不向历史反复追加。"""
    if not facts:
        return messages
    text = "\n程序记录的历史执行事实（非实时状态，未返回的字段未知）：\n" + json.dumps(facts, ensure_ascii=False)
    if messages and messages[0]["role"] == "system":
        return [{**messages[0], "content": messages[0]["content"] + text}, *messages[1:]]
    return [{"role": "system", "content": text}, *messages]


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
        context.execution_facts.clear()
        context.navigation_instruction = user_input
        context.navigation_resumable = True
        if context.task_state is not None:
            context.task_state.start_new_goal(user_input)
    return bool(resume)
