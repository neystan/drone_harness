"""创建标准格式的手写 skill 草稿。"""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

from .loader import DEFAULT_SKILLS_DIR
from .validator import validate_skill_dir


def normalize_skill_name(name: str) -> str:
    """把用户输入的名称转换为小写连字符格式。"""
    normalized = re.sub(r"[^a-z0-9]+", "-", name.strip().lower())
    normalized = re.sub(r"-{2,}", "-", normalized).strip("-")
    if not normalized:
        raise ValueError("skill name is empty after normalization")
    return normalized


def create_skill(
    name: str,
    description: str,
    mode: list[str],
    *,
    skills_dir: Path | None = None,
    enabled: bool = False,
) -> Path:
    """创建 SKILL.md 草稿并返回 skill 目录。"""
    skill_name = normalize_skill_name(name)
    target_root = skills_dir or DEFAULT_SKILLS_DIR
    skill_dir = target_root / skill_name
    skill_file = skill_dir / "SKILL.md"
    if skill_dir.exists():
        raise FileExistsError(f"skill already exists: {skill_dir}")

    skill_dir.mkdir(parents=True)
    skill_file.write_text(
        _render_skill_template(
            skill_name=skill_name,
            description=description,
            mode=mode,
            enabled=enabled,
        ),
        encoding="utf-8",
    )
    validate_skill_dir(skill_dir)
    return skill_dir


def _render_skill_template(
    *,
    skill_name: str,
    description: str,
    mode: list[str],
    enabled: bool,
) -> str:
    """渲染标准 SKILL.md 模板。"""
    return f"""---
name: {skill_name}
description: {description}
enabled: {str(enabled).lower()}
mode: {_format_yaml_list(mode)}
---

# {skill_name}

## 使用场景

说明这个 skill 应该在什么用户请求下使用。

## 工作流程

1. 按顺序描述 agent 应该如何完成任务。

## 可调用工具

列出推荐使用的现有 tools，并说明使用顺序。

## 安全约束

写清楚必须保守处理或停止执行的情况。

## 失败处理

说明工具失败、超时、低置信度或用户介入时如何响应。

## 反例

说明哪些请求不应该使用这个 skill。

## 示例

用户：

推荐工具调用顺序：
"""


def _format_yaml_list(values: list[str]) -> str:
    """把字符串列表格式化为单行 YAML 列表。"""
    quoted = [json.dumps(value, ensure_ascii=False) for value in values]
    return "[" + ", ".join(quoted) + "]"


def main() -> int:
    """提供最小 CLI，用于初始化 skill 草稿。"""
    parser = argparse.ArgumentParser(description="Create a drone_agent skill draft.")
    parser.add_argument("name")
    parser.add_argument(
        "--description",
        required=True,
        help="中文描述，说明这个 skill 适合处理哪类用户请求。",
    )
    parser.add_argument("--mode", action="append", choices=["sim", "real"], required=True)
    parser.add_argument("--enabled", action="store_true")
    args = parser.parse_args()

    skill_dir = create_skill(
        args.name,
        args.description,
        args.mode,
        enabled=args.enabled,
    )
    print(skill_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
