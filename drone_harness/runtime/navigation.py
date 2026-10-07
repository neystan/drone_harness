"""保存 VLM 自主管理的导航清单，不代替实际到达核验。"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from typing import Any

from drone_harness.llm.prompts import NAVIGATION_EXECUTION_PROMPT


@dataclass
class NavigationSubgoal:
    """保存模型提交的目标、状态与依据。"""

    id: str
    description: str
    completion_condition: str
    status: str
    evidence: str


@dataclass
class NavigationPlan:
    """保存本轮完整清单，允许模型调整或重新打开目标。"""

    original_instruction: str
    subgoals: list[NavigationSubgoal]
    finish_action: str
    reason: str
    status: str = "running"
    stop_reason: str = ""
    feedback: str = ""

    @property
    def completed_count(self) -> int:
        """按条目实际状态计数，不假定完成项连续。"""
        return sum(goal.status == "completed" for goal in self.subgoals)

    @property
    def current(self) -> NavigationSubgoal | None:
        """优先返回当前项，否则返回待选焦点，不修改提交状态。"""
        if self.status != "running":
            return None
        return next((goal for goal in self.subgoals if goal.status == "in_progress"),
                    next((goal for goal in self.subgoals if goal.status == "pending"), None))

    def finish(self, *, completed: bool, reason: str) -> None:
        """全部模型标记完成且收尾成功，才记录整体完成。"""
        self.status = "completed" if completed and self.completed_count == len(self.subgoals) else "incomplete"
        self.stop_reason = reason

    def snapshot(self) -> dict[str, Any]:
        """生成日志和模型共用快照，不包含内部观测元数据。"""
        current = self.current
        return {
            "original_instruction": self.original_instruction,
            "subgoals": [asdict(goal) for goal in self.subgoals],
            "finish_action": self.finish_action,
            "reason": self.reason,
            "status": self.status,
            "completed_count": self.completed_count,
            "total_count": len(self.subgoals),
            "current_subgoal_id": current.id if current else None,
            "current_is_explicit": current is not None and current.status == "in_progress",
            "stop_reason": self.stop_reason,
            "feedback": self.feedback,
        }

    def conversation_summary(self) -> str:
        """跨轮保留模型进度文字，不恢复工具状态。"""
        completed = "；".join(f"{goal.description}（依据：{goal.evidence or '未提供'}）"
                             for goal in self.subgoals if goal.status == "completed") or "无"
        return f"导航进度：{self.completed_count}/{len(self.subgoals)} 段由模型确认；已确认：{completed}。"


def parse_navigation_plan(value: Any, original_instruction: str) -> NavigationPlan:
    """只校验完整清单的结构，不冻结目标或检查语义完成。"""
    if not isinstance(value, dict) or set(value) != {"subgoals", "finish_action", "reason"}:
        raise ValueError("参数必须且只能包含 subgoals、finish_action、reason")
    if not isinstance(value["finish_action"], str) or value["finish_action"] not in {"land", "hold"}:
        raise ValueError("finish_action 必须为 land 或 hold")
    if not isinstance(value["reason"], str) or not value["reason"].strip():
        raise ValueError("reason 必须为非空文字")
    goals = value["subgoals"]
    if not isinstance(goals, list) or not goals:
        raise ValueError("subgoals 必须为非空列表")
    fields = {"id", "description", "completion_condition", "status", "evidence"}
    parsed = []
    for goal in goals:
        if not isinstance(goal, dict) or set(goal) != fields:
            raise ValueError("子目标字段必须为 id、description、completion_condition、status、evidence")
        if any(not isinstance(goal[key], str) for key in fields):
            raise ValueError("子目标字段必须为文字")
        if any(not goal[key].strip() for key in ("id", "description", "completion_condition")):
            raise ValueError("id、description、completion_condition 不能为空")
        if goal["status"] not in {"pending", "in_progress", "completed"}:
            raise ValueError("子目标 status 无效")
        parsed.append(NavigationSubgoal(**{key: goal[key].strip() for key in fields}))
    if len({goal.id for goal in parsed}) != len(parsed):
        raise ValueError("子目标 id 不能重复")
    if sum(goal.status == "in_progress" for goal in parsed) > 1:
        raise ValueError("最多一个 in_progress 子目标")
    plan = NavigationPlan(original_instruction, parsed, value["finish_action"], value["reason"].strip())
    if plan.completed_count == len(parsed):
        plan.status = "finishing"
    return plan


def is_pure_progress(old: NavigationPlan | None, new: NavigationPlan) -> bool:
    """只有目标不变且纯推进时才允许压缩，不限制计划改写。"""
    if old is None or old.finish_action != new.finish_action or len(old.subgoals) != len(new.subgoals):
        return False
    progressed = False
    for before, after in zip(old.subgoals, new.subgoals):
        if (before.id, before.description, before.completion_condition) != (
                after.id, after.description, after.completion_condition):
            return False
        if before.status == "completed":
            if before != after:
                return False
        elif after.status == "completed":
            progressed = True
        elif before.evidence != after.evidence or (before.status, after.status) not in {
                ("pending", "pending"), ("pending", "in_progress"), ("in_progress", "in_progress")}:
            return False
    return progressed


def update_navigation_plan(context: Any, arguments: dict[str, Any]) -> dict[str, Any]:
    """原子替换清单，拒绝候选不会破坏已接受的状态。"""
    if not context.navigation_enabled:
        return {"success": False, "error": "NAVIGATION_DISABLED", "plan_changed": False}
    old = context.navigation_plan
    try:
        plan = parse_navigation_plan(arguments, old.original_instruction if old else context.navigation_instruction)
    except ValueError as exc:
        return {"success": False, "error": "INVALID_NAVIGATION_PLAN", "message": str(exc), "plan_changed": False}
    changed = old is None or (old.subgoals, old.finish_action, old.reason) != (
        plan.subgoals, plan.finish_action, plan.reason)
    compact = changed and is_pure_progress(old, plan)
    if changed:
        context.navigation_plan = plan
        from drone_harness.logging.task_log import log_navigation_plan

        log_navigation_plan(context.profile, context.session_id, "created" if old is None else "updated", plan)
        print(f"plan> {plan.completed_count}/{len(plan.subgoals)} 段由模型确认；{plan.reason}")
        for goal in plan.subgoals:
            print(f"plan> [{goal.status}] {goal.id} {goal.description}；完成条件：{goal.completion_condition}"
                  + (f"；依据：{goal.evidence}" if goal.evidence else ""))
    else:
        plan = old
    return {"success": True, "changed": changed, "compact_history": compact,
            "completed_count": plan.completed_count, "total_count": len(plan.subgoals),
            "current_subgoal_id": plan.current.id if plan.current else None, "status": plan.status,
            "message": "清单已接受；完成状态为模型判断，未独立核验到达。"}


def navigation_messages(messages: list[dict[str, Any]], plan: NavigationPlan | None,
                        *, observation_current: bool) -> list[dict[str, Any]]:
    """每轮注入最新清单，不累积旧快照。"""
    content = NAVIGATION_EXECUTION_PROMPT
    if plan is not None:
        snapshot = plan.snapshot()
        snapshot["observation_current"] = observation_current
        content += "\n当前导航计划：\n" + json.dumps(snapshot, ensure_ascii=False)
    if messages and messages[0]["role"] == "system":
        return [{**messages[0], "content": messages[0]["content"] + "\n" + content}, *messages[1:]]
    return [{"role": "system", "content": content}, *messages]


def compact_navigation_history(messages: list[dict[str, Any]], task_start: int,
                               *, observation_current: bool) -> None:
    """压缩已完成段，保留最新观察和本次计划调用的完整协议对。"""
    latest = next((message for message in reversed(messages[task_start:])
                   if isinstance(message.get("content"), list)
                   and any(part.get("type") == "image_url" for part in message["content"])), None)
    pair = messages[-2:]
    messages[task_start:] = ([latest] if observation_current and latest is not None else []) + pair
