"""在 agent 与相机进程之间传递鼠标选点状态。"""

from __future__ import annotations

import json
import os
import time
import uuid
from pathlib import Path
from typing import Any


MOUSE_SELECTION_STATE_PATH = Path("/tmp/drone_agent_mouse_selection.json")


def begin_mouse_selection(image_path: Path) -> str:
    """创建一次等待相机窗口点击的请求。"""
    request_id = uuid.uuid4().hex
    _write_state(
        {
            "request_id": request_id,
            "status": "waiting",
            "image_path": str(image_path),
            "created_at": time.time(),
        }
    )
    return request_id


def read_mouse_selection() -> dict[str, Any]:
    """读取当前鼠标选点状态。"""
    return _read_state()


def complete_mouse_selection(
    request_id: str,
    point_xy_px: list[int],
    frame_seq: int,
) -> bool:
    """提交与请求匹配的点击坐标和图像帧号。"""
    state = _read_state()
    if state.get("request_id") != request_id or state.get("status") != "waiting":
        return False
    state.update(
        {
            "status": "selected",
            "point_xy_px": point_xy_px,
            "frame_seq": frame_seq,
            "selected_at": time.time(),
        }
    )
    _write_state(state)
    return True


def cancel_mouse_selection(request_id: str, reason: str = "cancelled") -> bool:
    """取消与 request ID 匹配的等待请求。"""
    state = _read_state()
    if state.get("request_id") != request_id or state.get("status") != "waiting":
        return False
    state.update(
        {
            "status": "cancelled",
            "reason": reason,
            "updated_at": time.time(),
        }
    )
    _write_state(state)
    return True


def fail_mouse_selection(request_id: str, message: str) -> bool:
    """记录相机进程保存点击帧失败。"""
    state = _read_state()
    if state.get("request_id") != request_id or state.get("status") != "waiting":
        return False
    state.update(
        {
            "status": "failed",
            "message": message,
            "updated_at": time.time(),
        }
    )
    _write_state(state)
    return True


def _read_state() -> dict[str, Any]:
    """读取状态文件，文件异常时返回空字典。"""
    if not MOUSE_SELECTION_STATE_PATH.exists():
        return {}
    try:
        state = json.loads(MOUSE_SELECTION_STATE_PATH.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}
    return state if isinstance(state, dict) else {}


def _write_state(state: dict[str, Any]) -> None:
    """原子写入鼠标选点状态。"""
    MOUSE_SELECTION_STATE_PATH.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = MOUSE_SELECTION_STATE_PATH.with_name(
        f"{MOUSE_SELECTION_STATE_PATH.name}.{os.getpid()}.tmp"
    )
    tmp_path.write_text(
        json.dumps(state, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    tmp_path.replace(MOUSE_SELECTION_STATE_PATH)
