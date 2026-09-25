"""提供 skill 启用工具。"""

from __future__ import annotations

from typing import Any

from drone_agent.skills.context import format_activated_skill_line
from drone_agent.skills.loader import Skill, SkillsLoader


def activate_skill(context: Any, name: str | None) -> dict[str, Any]:
    """校验并启用指定 skill，成功后返回完整 skill 正文。"""
    skill_name = str(name or "").strip()
    if not skill_name:
        return _failure("SKILL_NOT_FOUND", "未提供 skill 名称。")

    skill = _find_skill(skill_name)
    if skill is None:
        return _failure("SKILL_NOT_FOUND", f"未找到 skill：{skill_name}。")
    if not skill.enabled:
        return _failure("SKILL_DISABLED", f"skill 未启用：{skill.name}。")
    if context.profile.name not in skill.modes:
        return _failure("SKILL_MODE_MISMATCH", f"当前 profile 不允许启用 skill：{skill.name}。")

    confirmed = _confirm_skill_activation(context, skill)
    if confirmed is not None:
        return confirmed

    print(format_activated_skill_line(skill))
    return {
        "success": True,
        "skill_name": skill.name,
        "skill_content": skill.body,
        "message": f"已启用 skill：{skill.name}。请根据 skill_content 继续完成用户任务。",
    }


def _find_skill(name: str) -> Skill | None:
    """按名称查找内置 skill。"""
    for skill in SkillsLoader().load_skills():
        if skill.name == name:
            return skill
    return None


def _confirm_skill_activation(context: Any, skill: Skill) -> dict[str, Any] | None:
    """通过消息总线等待用户确认是否启用 skill。"""
    if context.message_bus is None:
        return _failure("SKILL_ACTIVATION_UNAVAILABLE", "message bus 不可用，无法确认 skill 启用。")

    prompt = f"human-in-the-loop> 启用 skill {skill.name}？[Y/N]: "
    print(prompt, flush=True)
    while True:
        answer = context.message_bus.consume_user_message().content.strip().lower()
        if answer == "y":
            return None
        if answer == "n":
            return _failure("SKILL_ACTIVATION_DECLINED", f"已取消启用 skill：{skill.name}。")
        print("human-in-the-loop> 请输入 Y 或 N。")


def _failure(error: str, message: str) -> dict[str, Any]:
    """构造 skill 启用失败结果。"""
    return {
        "success": False,
        "error": error,
        "message": message,
    }
