"""保存有序导航计划及同一模型的完成判断。"""

from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Any

from drone_harness.llm.prompts import NAVIGATION_EXECUTION_PROMPT, NAVIGATION_PLANNING_PROMPT


@dataclass
class NavigationSubgoal:
    """保存一个导航段及其已确认依据。"""

    description: str
    completion_condition: str
    evidence: str = ""
    observation_id: str | None = None


@dataclass
class NavigationPlan:
    """按顺序推进当前任务，不重写计划或恢复历史任务。"""

    original_instruction: str
    subgoals: list[NavigationSubgoal]
    finish_action: str
    current_index: int = 0
    status: str = "running"
    stop_reason: str = ""
    feedback: str = ""

    @property
    def current(self) -> NavigationSubgoal | None:
        """返回唯一活动段，收尾时返回空。"""
        if self.status != "running" or self.current_index >= len(self.subgoals):
            return None
        return self.subgoals[self.current_index]

    def advance(self, evidence: str, observation_id: str | None) -> bool:
        """有依据和当前观察时，完成当前段并激活下一段。"""
        current = self.current
        if current is None or not evidence.strip() or not observation_id:
            self.feedback = "当前段尚不能确认，请先 observe 获取当前观察并给出完成依据。"
            return False
        current.evidence = evidence.strip()
        current.observation_id = observation_id
        self.current_index += 1
        self.feedback = ""
        if self.current_index == len(self.subgoals):
            self.status = "finishing"
        return True

    def finish(self, *, completed: bool, reason: str) -> None:
        """只有全部导航段已确认并完成收尾，才记录整体完成。"""
        self.status = "completed" if completed and self.current_index == len(self.subgoals) else "incomplete"
        self.stop_reason = reason

    def snapshot(self, *, include_observation_ids: bool = True) -> dict[str, Any]:
        """生成日志或模型摘要，不向模型泄露内部观测号。"""
        subgoals = []
        for index, goal in enumerate(self.subgoals):
            item = {
                "number": index + 1,
                "description": goal.description,
                "completion_condition": goal.completion_condition,
                "status": ("completed" if index < self.current_index else
                           "in_progress" if index == self.current_index and self.status == "running" else "pending"),
                "evidence": goal.evidence,
            }
            if include_observation_ids:
                item["observation_id"] = goal.observation_id
            subgoals.append(item)
        return {
            "original_instruction": self.original_instruction,
            "status": self.status,
            "completed_count": self.current_index,
            "total_count": len(self.subgoals),
            "current_subgoal_number": self.current_index + 1 if self.current is not None else None,
            "finish_action": self.finish_action,
            "subgoals": subgoals,
            "stop_reason": self.stop_reason,
            "feedback": self.feedback,
        }

    def conversation_summary(self) -> str:
        """跨轮只保留人类可读进度，不携带工具或判断协议。"""
        completed = "；".join(f"{goal.description}（依据：{goal.evidence}）"
                              for goal in self.subgoals[:self.current_index]) or "无"
        return f"导航进度：{self.current_index}/{len(self.subgoals)} 段已确认；已确认：{completed}。"


@dataclass(frozen=True)
class NavigationDecision:
    """一次模型回复的完成判断，不代表独立核验。"""

    subgoal_complete: bool = False
    evidence: str = ""
    scene_description: str = ""

    @property
    def display_text(self) -> str:
        """将协议转为终端可读的描述与判断依据。"""
        parts = [self.scene_description] if self.scene_description else []
        if self.subgoal_complete:
            parts.append(f"模型完成判断依据：{self.evidence}")
        return "\n".join(parts)


def _json_object(content: str) -> dict[str, Any]:
    """解析模型 JSON，可接受外层代码围栏。"""
    text = content.strip()
    if text.startswith("```") and text.endswith("```"):
        text = text.split("\n", 1)[-1].rsplit("\n", 1)[0]
    try:
        value = json.loads(text)
    except (json.JSONDecodeError, TypeError) as exc:
        raise ValueError("模型导航 JSON 无效") from exc
    if not isinstance(value, dict):
        raise ValueError("模型导航结果必须是对象")
    return value


def parse_navigation_plan(content: str, original_instruction: str) -> NavigationPlan | None:
    """校验一次初始拆分；普通交流不建立计划。"""
    value = _json_object(content)
    if type(value.get("navigation")) is not bool:
        raise ValueError("navigation 必须是布尔值")
    if not value["navigation"]:
        return None
    goals = value.get("subgoals")
    if not isinstance(goals, list) or not goals:
        raise ValueError("导航计划须包含非空子目标列表")
    if value.get("finish_action") not in ("land", "hold"):
        raise ValueError("收尾动作必须为 land 或 hold")
    subgoals = []
    for goal in goals:
        if not isinstance(goal, dict) or any(
            not isinstance(goal.get(key), str) or not goal[key].strip()
            for key in ("description", "completion_condition")
        ):
            raise ValueError("子目标描述和完成条件须为非空文字")
        subgoals.append(NavigationSubgoal(goal["description"].strip(), goal["completion_condition"].strip()))
    return NavigationPlan(original_instruction, subgoals, value["finish_action"])


def request_navigation_plan(client: Any, model: str, messages: list[dict[str, Any]]) -> NavigationPlan | None:
    """程序固定请求同一模型拆分，不提供飞行工具或图片。"""
    dialogue = [{"role": message["role"], "content": message["content"]}
                for message in messages if message.get("role") in {"user", "assistant"}
                and isinstance(message.get("content"), str)]
    instruction = next(message["content"] for message in reversed(dialogue) if message["role"] == "user")
    response = client.chat.completions.create(
        model=model, messages=[{"role": "system", "content": NAVIGATION_PLANNING_PROMPT}, *dialogue],
        temperature=0.0,
    )
    message = response.choices[0].message
    if getattr(message, "tool_calls", None) or not isinstance(message.content, str):
        raise ValueError("初始拆分不能提出工具调用，且须返回文字结构")
    return parse_navigation_plan(message.content, instruction)


def parse_navigation_decision(content: str) -> NavigationDecision:
    """普通文字或空工具回复不确认完成，显式结构须合法。"""
    text = content.strip()
    if not text.startswith(("{", "[", "```")):
        return NavigationDecision(scene_description=text)
    value = _json_object(text)
    complete = value.get("subgoal_complete")
    evidence = value.get("evidence")
    scene = value.get("scene_description", "")
    if type(complete) is not bool or not isinstance(evidence, str) or not isinstance(scene, str):
        raise ValueError("完成判断字段类型无效")
    if complete and not evidence.strip():
        raise ValueError("确认完成须提供依据")
    return NavigationDecision(complete, evidence.strip(), scene.strip())


def navigation_messages(
    messages: list[dict[str, Any]], plan: NavigationPlan, *, observation_current: bool,
) -> list[dict[str, Any]]:
    """临时注入当前计划摘要，不反复追加或修改原消息。"""
    snapshot = plan.snapshot(include_observation_ids=False)
    snapshot["observation_current"] = observation_current
    content = messages[0]["content"] + "\n" + NAVIGATION_EXECUTION_PROMPT + "\n当前导航计划：\n"
    content += json.dumps(snapshot, ensure_ascii=False)
    return [{**messages[0], "content": content}, *messages[1:]]


def compact_navigation_history(
    messages: list[dict[str, Any]], task_start: int, *, observation_current: bool,
) -> None:
    """整段压缩已完成的执行消息，保留最新可用观测且不留下孤立工具结果。"""
    latest = next((message for message in reversed(messages[task_start:])
                   if isinstance(message.get("content"), list)
                   and any(part.get("type") == "image_url" for part in message["content"])), None)
    messages[task_start:] = [latest] if observation_current and latest is not None else []


def navigation_conversation_text(content: str) -> str:
    """下一轮只继承场景文字，完成依据由计划摘要统一保存。"""
    try:
        return parse_navigation_decision(content).scene_description
    except ValueError:
        return ""
