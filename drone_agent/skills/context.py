"""构造 skills index 和终端显示内容。"""

from __future__ import annotations

from .loader import Skill

BLUE = "\033[34m"
RESET = "\033[0m"


def build_skills_index(skills: list[Skill], profile_name: str) -> str:
    """生成只包含 name 和 description 的 skills index。"""
    enabled_skills = [
        skill for skill in skills if skill.enabled and profile_name in skill.modes
    ]
    if not enabled_skills:
        return ""

    lines = [
        "# Skills Index",
        "",
        "以下是当前可用的 skills。它们只是可启用能力，不会自动生效。",
        "如果用户明确要求使用某个 skill，或用户请求明显符合某个 skill 描述，先调用 activate_skill(name)。",
        "",
        "当前可用的 skills：",
    ]
    for skill in enabled_skills:
        lines.append(f"- {skill.name}: {skill.description}")
    return "\n".join(lines)


def format_activated_skill_line(skill: Skill) -> str:
    """生成蓝色 skill 启用提示行。"""
    return f"{BLUE}skill> {skill.name}{RESET}"
