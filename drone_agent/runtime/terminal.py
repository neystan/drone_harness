"""负责为 drone agent 打开独立输入终端。"""

from __future__ import annotations

import os
import shlex
import shutil
import subprocess
import sys


def open_input_terminal(host: str, port: int, token: str, profile_name: str) -> bool:
    """自动打开独立输入终端，成功返回 True。"""
    command = _build_input_command(host, port, token, profile_name)
    custom_terminal = os.environ.get("DRONE_AGENT_INPUT_TERMINAL_CMD", "").strip()
    if custom_terminal:
        return _open_custom_terminal(custom_terminal, command)

    for launcher in _candidate_launchers(command, profile_name):
        try:
            subprocess.Popen(launcher)
            return True
        except OSError:
            continue
    return False


def _build_input_command(host: str, port: int, token: str, profile_name: str) -> str:
    """构造输入终端内执行的 Python 命令。"""
    pythonpath = _current_pythonpath()
    parts = [
        sys.executable,
        "-m",
        "drone_agent.bus.input_terminal",
        "--host",
        host,
        "--port",
        str(port),
        "--token",
        token,
        "--profile",
        profile_name,
    ]
    python_command = " ".join(shlex.quote(part) for part in parts)
    return (
        f"cd {shlex.quote(os.getcwd())} && "
        f"PYTHONPATH={shlex.quote(pythonpath)} exec {python_command}"
    )


def _current_pythonpath() -> str:
    """把当前进程可导入路径传给新终端。"""
    paths: list[str] = []
    for path in sys.path:
        paths.append(path if path else os.getcwd())
    env_pythonpath = os.environ.get("PYTHONPATH", "").strip()
    if env_pythonpath:
        paths.extend(env_pythonpath.split(os.pathsep))
    return os.pathsep.join(dict.fromkeys(path for path in paths if path))


def _open_custom_terminal(template: str, command: str) -> bool:
    """使用用户自定义终端命令打开输入终端。"""
    rendered = template.format(command=command)
    try:
        subprocess.Popen(rendered, shell=True)
    except OSError:
        return False
    return True


def _candidate_launchers(command: str, profile_name: str) -> list[list[str]]:
    """按当前环境返回可尝试的终端启动命令。"""
    title = f"drone_agent_{profile_name}_input"
    launchers: list[list[str]] = []
    has_wsl = shutil.which("wsl.exe") is not None

    if shutil.which("wt.exe"):
        if has_wsl:
            launchers.append(
                ["wt.exe", "new-tab", "--title", title, "wsl.exe", "bash", "-lc", command]
            )
        else:
            launchers.append(["wt.exe", "new-tab", "--title", title, "bash", "-lc", command])
    if shutil.which("cmd.exe") and has_wsl:
        launchers.append(
            [
                "cmd.exe",
                "/c",
                "start",
                title,
                "wsl.exe",
                "bash",
                "-lc",
                command,
            ]
        )
    if shutil.which("gnome-terminal"):
        launchers.append(["gnome-terminal", "--title", title, "--", "bash", "-lc", command])
    if shutil.which("konsole"):
        launchers.append(
            ["konsole", "--new-tab", "--workdir", os.getcwd(), "-e", "bash", "-lc", command]
        )
    if shutil.which("xfce4-terminal"):
        launchers.append(
            [
                "xfce4-terminal",
                "--title",
                title,
                "--command",
                f"bash -lc {shlex.quote(command)}",
            ]
        )
    if shutil.which("xterm"):
        launchers.append(["xterm", "-T", title, "-e", "bash", "-lc", command])

    return launchers
