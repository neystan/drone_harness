"""运行在独立终端中的用户输入客户端。"""

from __future__ import annotations

import argparse
import socket
import sys
from collections.abc import Sequence


def main(argv: Sequence[str] | None = None) -> int:
    """连接主进程输入服务，并把用户输入逐行发送过去。"""
    parser = _build_parser()
    args = parser.parse_args(argv)
    _configure_readline()
    try:
        with socket.create_connection((args.host, args.port), timeout=10.0) as sock:
            return _run_input_loop(sock, args.token, args.profile)
    except OSError as exc:
        print(f"input-terminal> 无法连接 drone_agent 主进程：{exc}", file=sys.stderr)
        return 2


def _build_parser() -> argparse.ArgumentParser:
    """构建输入终端参数解析器。"""
    parser = argparse.ArgumentParser(prog="drone_agent_input_terminal")
    parser.add_argument("--host", required=True)
    parser.add_argument("--port", required=True, type=int)
    parser.add_argument("--token", required=True)
    parser.add_argument("--profile", required=True)
    return parser


def _configure_readline() -> None:
    """配置终端输入行为，避免中文编辑异常。"""
    try:
        import readline
    except ImportError:
        return

    readline.parse_and_bind("set bind-tty-special-chars off")
    readline.parse_and_bind("set input-meta on")
    readline.parse_and_bind("set output-meta on")
    readline.parse_and_bind("set convert-meta off")


def _run_input_loop(sock: socket.socket, token: str, profile: str) -> int:
    """读取用户输入并发送到主进程 MessageBus。"""
    reader = sock.makefile("r", encoding="utf-8", newline="\n")
    writer = sock.makefile("w", encoding="utf-8", newline="\n")
    writer.write(f"{token}\n")
    writer.flush()
    if reader.readline().strip() != "OK":
        print("input-terminal> 主进程拒绝连接。", file=sys.stderr)
        return 3

    print(f"drone_agent_{profile} 输入终端")
    print("请输入自然语言；HITL 确认时输入 Y 或 N；输入 exit 退出。")
    while True:
        try:
            user_input = input("you> ").strip()
        except EOFError:
            user_input = "exit"
        if user_input:
            writer.write(f"{user_input}\n")
            writer.flush()
        if user_input.lower() in {"exit", "quit"}:
            return 0


if __name__ == "__main__":
    raise SystemExit(main())
