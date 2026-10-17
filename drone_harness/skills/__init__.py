"""加载项目内置 skill 的摘要与正文。"""

from dataclasses import dataclass
from pathlib import Path

import yaml


@dataclass(frozen=True)
class Skill:
    """保存经过校验的 skill 元数据和正文。"""

    name: str
    description: str
    enabled: bool
    modes: tuple[str, ...]
    body: str


def load_skills(directory: Path | None = None) -> list[Skill]:
    """沿用目录与 YAML 元数据加载方式，不读取或执行任意脚本。"""
    directory = directory or Path(__file__).parent
    skills = []
    for path in sorted(directory.glob("*/SKILL.md")):
        text = path.read_text(encoding="utf-8")
        header, separator, body = text.removeprefix("---\n").partition("\n---\n")
        if not text.startswith("---\n") or not separator:
            raise ValueError(f"skill 缺少 frontmatter：{path.parent.name}")
        metadata = yaml.safe_load(header)
        if (not isinstance(metadata, dict) or metadata.get("name") != path.parent.name
                or not isinstance(metadata.get("description"), str) or not metadata["description"].strip()
                or not isinstance(metadata.get("enabled"), bool)
                or not isinstance(metadata.get("mode"), list) or not metadata["mode"]
                or any(mode not in {"sim", "real"} for mode in metadata["mode"])
                or not body.strip()):
            raise ValueError(f"skill 元数据或正文无效：{path.parent.name}")
        skills.append(Skill(metadata["name"], metadata["description"].strip(), metadata["enabled"],
                            tuple(metadata["mode"]), body.strip()))
    return skills


def build_skills_index(profile_name: str) -> str:
    """只注入可用 skill 摘要，正文在模型按需激活后返回。"""
    available = [skill for skill in load_skills() if skill.enabled and profile_name in skill.modes]
    if not available:
        return ""
    return "\n可用 skills（按需调用 activate_skill，提供 name 和本次搜索 prompt）：\n" + "\n".join(
        f"- {skill.name}: {skill.description}" for skill in available)
