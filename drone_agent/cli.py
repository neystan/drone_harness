from __future__ import annotations

import argparse
import sys
from collections.abc import Sequence

from drone_agent.config.loader import ConfigError
from drone_agent.runtime.runtime import start_runtime


def build_parser(default_profile: str = "sim") -> argparse.ArgumentParser:
    """构建命令行参数解析器。"""
    parser = argparse.ArgumentParser(
        prog="drone_agent",
        description="Natural-language UAV control agent.",
    )
    parser.add_argument(
        "--profile",
        choices=("sim", "real"),
        default=default_profile,
        help="Runtime profile to load.",
    )
    return parser


def main_sim(argv: Sequence[str] | None = None) -> int:
    """运行仿真专用入口。"""
    parser = build_parser(default_profile="sim")
    parser.set_defaults(profile="sim")
    parser.parse_args(argv)
    return _run(profile_name="sim")


def main_real(argv: Sequence[str] | None = None) -> int:
    """运行真机专用入口。"""
    parser = build_parser(default_profile="real")
    parser.set_defaults(profile="real")
    parser.parse_args(argv)
    return _run(profile_name="real")


def _run(profile_name: str) -> int:
    """加载 profile 并直接启动对应运行时。"""
    try:
        start_runtime(profile_name=profile_name)
    except ConfigError as exc:
        print(f"config error: {exc}", file=sys.stderr)
        return 2
    return 0
