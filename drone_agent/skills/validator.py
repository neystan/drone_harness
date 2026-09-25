"""校验说明型 SKILL.md 的结构。"""

from __future__ import annotations

import re
from pathlib import Path
from typing import Any

import yaml


ALLOWED_FRONTMATTER_KEYS = {
    "name",
    "description",
    "enabled",
    "mode",
}
ALLOWED_SKILL_ROOT_DIRS = {"examples", "references"}
SKILL_NAME_PATTERN = re.compile(r"^[a-z0-9]+(?:-[a-z0-9]+)*$")
PLACEHOLDER_MARKERS = ("[todo", "todo:")


class SkillValidationError(ValueError):
    """表示 skill 文件格式不符合约定。"""


def validate_skill_dir(skill_dir: Path) -> dict[str, Any]:
    """校验 skill 目录并返回 frontmatter。"""
    skill_dir = skill_dir.resolve()
    if not skill_dir.is_dir():
        raise SkillValidationError(f"skill path is not a directory: {skill_dir}")

    skill_file = skill_dir / "SKILL.md"
    if not skill_file.exists():
        raise SkillValidationError(f"SKILL.md not found: {skill_dir}")

    content = skill_file.read_text(encoding="utf-8")
    frontmatter_text = extract_frontmatter_text(content)
    metadata = parse_frontmatter(frontmatter_text)
    validate_frontmatter(metadata, skill_dir.name)
    validate_skill_root_files(skill_dir)
    return metadata


def extract_frontmatter_text(content: str) -> str:
    """从 SKILL.md 中提取 YAML frontmatter 文本。"""
    lines = content.splitlines()
    if not lines or lines[0].strip() != "---":
        raise SkillValidationError("SKILL.md must start with YAML frontmatter")

    for index in range(1, len(lines)):
        if lines[index].strip() == "---":
            return "\n".join(lines[1:index])

    raise SkillValidationError("SKILL.md frontmatter is not closed")


def strip_frontmatter(content: str) -> str:
    """去掉 SKILL.md 的 frontmatter，只保留正文。"""
    lines = content.splitlines()
    if not lines or lines[0].strip() != "---":
        return content.strip()

    for index in range(1, len(lines)):
        if lines[index].strip() == "---":
            return "\n".join(lines[index + 1 :]).strip()

    return content.strip()


def parse_frontmatter(frontmatter_text: str) -> dict[str, Any]:
    """解析 YAML frontmatter。"""
    try:
        metadata = yaml.safe_load(frontmatter_text)
    except yaml.YAMLError as exc:
        raise SkillValidationError(f"invalid SKILL.md frontmatter: {exc}") from exc

    if not isinstance(metadata, dict):
        raise SkillValidationError("SKILL.md frontmatter must be a mapping")

    return {str(key): value for key, value in metadata.items()}


def validate_frontmatter(metadata: dict[str, Any], folder_name: str) -> None:
    """校验 frontmatter 字段和值。"""
    unexpected = sorted(set(metadata) - ALLOWED_FRONTMATTER_KEYS)
    if unexpected:
        raise SkillValidationError(f"unexpected frontmatter keys: {', '.join(unexpected)}")

    name = _required_str(metadata, "name")
    if not SKILL_NAME_PATTERN.fullmatch(name):
        raise SkillValidationError("skill name must be lowercase hyphen-case")
    if name != folder_name:
        raise SkillValidationError("skill name must match directory name")

    description = _required_str(metadata, "description")
    if any(marker in description.lower() for marker in PLACEHOLDER_MARKERS):
        raise SkillValidationError("skill description still contains TODO placeholder")

    if not isinstance(metadata.get("enabled"), bool):
        raise SkillValidationError("enabled must be a boolean")

    _required_str_list(metadata, "mode", allowed_values={"sim", "real"})


def validate_skill_root_files(skill_dir: Path) -> None:
    """限制 skill 根目录内容，避免引入额外执行入口。"""
    for child in skill_dir.iterdir():
        if child.name == "SKILL.md":
            continue
        if child.is_symlink():
            raise SkillValidationError(f"symlink is not allowed: {child.name}")
        if child.is_dir() and child.name in ALLOWED_SKILL_ROOT_DIRS:
            continue
        raise SkillValidationError(
            f"unexpected skill root entry: {child.name}; only SKILL.md, examples/, references/ are allowed"
        )


def _required_str(metadata: dict[str, Any], key: str) -> str:
    """读取必填字符串字段。"""
    value = metadata.get(key)
    if not isinstance(value, str) or not value.strip():
        raise SkillValidationError(f"{key} must be a non-empty string")
    return value.strip()


def _required_str_list(
    metadata: dict[str, Any],
    key: str,
    *,
    allowed_values: set[str] | None = None,
) -> list[str]:
    """读取必填字符串列表字段。"""
    value = metadata.get(key)
    if not _is_str_list(value) or not value:
        raise SkillValidationError(f"{key} must be a non-empty string list")
    items = [item.strip() for item in value]
    if allowed_values is not None:
        invalid = sorted(set(items) - allowed_values)
        if invalid:
            raise SkillValidationError(f"{key} contains invalid values: {', '.join(invalid)}")
    return items


def _is_str_list(value: Any) -> bool:
    """判断值是否为字符串列表。"""
    return isinstance(value, list) and all(isinstance(item, str) and item.strip() for item in value)
