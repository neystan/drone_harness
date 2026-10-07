"""提供 JSONL 任务日志记录能力。"""

from __future__ import annotations

import json
import base64
import hashlib
import sys
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any

from drone_harness.config.schema import RuntimeProfile
from drone_harness.runtime.navigation import NavigationPlan
from drone_harness.runtime.task_state import TaskState

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
        "observation_id": snapshot["observation_id"],
        "step_id": snapshot["step_id"],
        "consecutive_rejections": snapshot["consecutive_rejections"],
        "consecutive_no_progress": snapshot["consecutive_no_progress"],
        "landing_authorized": snapshot["landing_authorized"],
        "completion_candidate": snapshot["completion_candidate"],
    }
    try:
        append_jsonl(str(_session_log_dir(profile, session_id)), "task_state.jsonl", event)
    except OSError:
        pass


def log_observation(
    profile: RuntimeProfile,
    session_id: str,
    observation_id: str,
    rgb_stamp_ns: int,
    depth_stamp_ns: int | None,
    depth_valid: bool,
    forward_max_m: float,
    reason: str,
    image_path: str | None = None,
) -> None:
    """只记录观测元数据，不把原始图片或 base64 写入日志。"""
    event = {
        "timestamp": _timestamp(),
        "profile_name": profile.name,
        "event_type": "observation",
        "observation_id": observation_id,
        "rgb_stamp_ns": rgb_stamp_ns,
        "depth_stamp_ns": depth_stamp_ns,
        "depth_valid": depth_valid,
        "forward_max_m": forward_max_m,
        "reason": reason,
        "image_path": image_path,
    }
    try:
        append_jsonl(str(_session_log_dir(profile, session_id)), "observations.jsonl", event)
    except OSError:
        pass


def save_observation_image(profile: RuntimeProfile, session_id: str,
                           observation_id: str, message: dict[str, Any]) -> str | None:
    """可选归档模型消息中的原 JPEG 字节，失败不影响执行。"""
    if not profile.storage.save_observation_images:
        return None
    try:
        url = next(part["image_url"]["url"] for part in message["content"]
                   if part.get("type") == "image_url")
        if not url.startswith("data:image/jpeg;base64,"):
            raise ValueError("unsupported observation image")
        payload = base64.b64decode(url.split(",", 1)[1], validate=True)
        # 使用内容摘要命名，不把外部观测编号当作路径。
        digest = hashlib.sha256(observation_id.encode() + payload).hexdigest()
        relative = Path("observations") / f"{digest}.jpg"
        path = _session_log_dir(profile, session_id) / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("xb") as handle:
            handle.write(payload)
        return str(relative)
    except FileExistsError:
        return str(relative)
    except (OSError, ValueError, KeyError, TypeError, StopIteration) as exc:
        warning = f"观察图片保存失败：{type(exc).__name__}；不影响工具执行。"
        print(f"log> {warning}", file=sys.stderr)
        log_agent_message(profile, session_id, "system", warning)
        return None


def log_navigation_plan(
    profile: RuntimeProfile, session_id: str, event_type: str, plan: NavigationPlan,
) -> None:
    """记录计划建立、推进和停止，不包含图片或连接配置。"""
    event = {"timestamp": _timestamp(), "profile_name": profile.name,
             "event_type": event_type, "plan": plan.snapshot()}
    try:
        append_jsonl(str(_session_log_dir(profile, session_id)), "navigation_plan.jsonl", event)
    except OSError:
        pass
