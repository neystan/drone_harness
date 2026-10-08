from __future__ import annotations

import argparse
import sys
from pathlib import Path
from collections.abc import Sequence

from drone_harness.config.loader import ConfigError
from drone_harness.runtime.runtime import start_runtime, start_single_runtime


def build_parser(default_profile: str = "sim") -> argparse.ArgumentParser:
    """构建命令行参数解析器。"""
    parser = argparse.ArgumentParser(
        prog="drone_harness",
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
    parser.add_argument("--instruction-file", type=Path, help="仿真批测：只执行文件中的一条导航指令")
    parser.add_argument("--result-file", type=Path, help="单任务结束结果 JSON")
    parser.add_argument("--log-dir", type=Path, help="本条任务独立日志目录")
    parser.add_argument("--startup-timeout-s", type=float, default=60.0, help="单任务等待连接、位姿和 RGB-D 的秒数")
    args = parser.parse_args(argv)
    if args.instruction_file is not None:
        if args.result_file is None or args.log_dir is None:
            parser.error("--instruction-file requires --result-file and --log-dir")
        if not 0 < args.startup_timeout_s < float("inf"):
            parser.error("--startup-timeout-s must be finite and positive")
        return start_single_runtime(args.instruction_file, args.result_file, args.log_dir,
                                    startup_timeout_s=args.startup_timeout_s)
    if args.result_file is not None or args.log_dir is not None:
        parser.error("--result-file/--log-dir require --instruction-file")
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
