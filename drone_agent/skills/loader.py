"""加载项目内置说明型 skills。"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .validator import SkillValidationError, strip_frontmatter, validate_skill_dir


DEFAULT_SKILLS_DIR = Path(__file__).resolve().parent


@dataclass(frozen=True)
class Skill:
    """保存一个已校验 skill 的元数据和正文。"""

    name: str
    description: str
    enabled: bool
    modes: tuple[str, ...]
    body: str
    path: Path


class SkillsLoader:
    """扫描并加载内置 skills。"""

    def __init__(self, skills_dir: Path | None = None) -> None:
        self.skills_dir = skills_dir or DEFAULT_SKILLS_DIR

    def load_skills(self) -> list[Skill]:
        """加载全部合法 skill，非法 skill 会被跳过。"""
        skills: list[Skill] = []
        if not self.skills_dir.exists():
            return skills

        for skill_dir in sorted(self.skills_dir.iterdir()):
            if not skill_dir.is_dir() or skill_dir.name.startswith("__"):
                continue
            skill = self.load_skill_dir(skill_dir)
            if skill is not None:
                skills.append(skill)
        return skills

    def load_skill_dir(self, skill_dir: Path) -> Skill | None:
        """加载单个 skill 目录。"""
        try:
            metadata = validate_skill_dir(skill_dir)
        except SkillValidationError:
            return None

        skill_file = skill_dir / "SKILL.md"
        body = strip_frontmatter(skill_file.read_text(encoding="utf-8"))
        return _build_skill(metadata, body, skill_file)


def _build_skill(metadata: dict[str, Any], body: str, path: Path) -> Skill:
    """把 frontmatter 字典转换为 Skill 对象。"""
    return Skill(
        name=str(metadata["name"]).strip(),
        description=str(metadata["description"]).strip(),
        enabled=bool(metadata["enabled"]),
        modes=tuple(str(item).strip() for item in metadata["mode"]),
        body=body,
        path=path,
    )
