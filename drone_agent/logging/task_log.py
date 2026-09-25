"""提供 JSONL 任务日志记录能力。"""

from __future__ import annotations

import json
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any

from drone_agent.config.schema import RuntimeProfile
from drone_agent.runtime.task_state import TaskState

BEIJING_TZ = timezone(timedelta(hours=8))


def append_jsonl(log_dir: str, filename: str, event: dict[str, Any]) -> None:
    """向指定日志文件追加一条 JSONL 记录。"""
    path = Path(log_dir)
    path.mkdir(parents=True, exist_ok=True)
    logfile = path / filename
    with logfile.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(event, ensure_ascii=False) + "\n")


def _timestamp() -> str:
    """生成北京时间字符串时间戳。"""
    return datetime.now(BEIJING_TZ).strftime("%Y-%m-%d %H:%M:%S")


def create_session_id() -> str:
    """生成北京时间会话编号。"""
    return datetime.now(BEIJING_TZ).strftime("%Y%m%d_%H%M%S")


def _session_log_dir(profile: RuntimeProfile, session_id: str) -> Path:
    """返回当前会话日志目录。"""
    return Path(profile.storage.log_dir) / f"session_{session_id}"


def log_tool_call(
    profile: RuntimeProfile,
    session_id: str,
    tool_name: str,
    arguments: Any,
    result: dict[str, Any],
) -> None:
    """记录一次工具调用及其结果。"""
    event = {
        "timestamp": _timestamp(),
        "profile_name": profile.name,
        "event_type": "tool_call",
        "tool_name": tool_name,
        "arguments": arguments,
        "result": result,
    }
    try:
        append_jsonl(str(_session_log_dir(profile, session_id)), "tool_calls.jsonl", event)
    except OSError:
        pass


def log_agent_message(
    profile: RuntimeProfile,
    session_id: str,
    role: str,
    content: str,
) -> None:
    """记录一次 agent 消息。"""
    event = {
        "timestamp": _timestamp(),
        "profile_name": profile.name,
        "event_type": "agent_message",
        "role": role,
        "content": content,
    }
    try:
        append_jsonl(str(_session_log_dir(profile, session_id)), "agent_messages.jsonl", event)
    except OSError:
        pass


def log_task_state(
    profile: RuntimeProfile,
    session_id: str,
    task_state: TaskState,
) -> None:
    """记录一次当前会话的任务状态快照。"""
    snapshot = task_state.snapshot()
    event = {
        "timestamp": _timestamp(),
        "profile_name": profile.name,
        "event_type": "task_state",
        "task_id": snapshot["task_id"],
        "current_phase": snapshot["current_phase"],
        "current_user_goal": snapshot["current_user_goal"],
        "active_tool_name": snapshot["active_tool_name"],
        "active_tool_is_flight_tool": snapshot["active_tool_is_flight_tool"],
        "waiting_for_user_confirmation": snapshot["waiting_for_user_confirmation"],
        "intervention_pending": snapshot["intervention_pending"],
        "intervention_message": snapshot["intervention_message"],
        "last_tool_name": snapshot["last_tool_name"],
        "last_error": snapshot["last_error"],
    }
    try:
        append_jsonl(str(_session_log_dir(profile, session_id)), "task_state.jsonl", event)
    except OSError:
        pass
