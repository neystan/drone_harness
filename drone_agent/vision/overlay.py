"""相机预览窗口使用的视觉 overlay 状态。"""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any


OVERLAY_STATE_PATH = Path("/tmp/drone_agent_vision_overlay.json")


def write_detection_overlay(
    objects: list[dict[str, Any]],
    path: Path = OVERLAY_STATE_PATH,
) -> None:
    """写入只显示一帧的目标检测框。"""
    state = _read_state(path)
    state["detection"] = {
        "updated_at": time.time(),
        "objects": objects,
    }
    _write_state(path, state)


def write_tracking_overlay(
    result: dict[str, Any],
    path: Path = OVERLAY_STATE_PATH,
) -> None:
    """写入相机窗口实时追踪所需的控制状态。"""
    state = _read_state(path)
    state["tracking"] = {
        "updated_at": time.time(),
        "track_id": result.get("track_id"),
        "target_description": result.get("target_description"),
        "bbox_xyxy_px": result.get("bbox_xyxy_px"),
        "tracker_base_url": result.get("tracker_base_url"),
        "tracker_timeout_s": result.get("tracker_timeout_s"),
        "tracking_frame_dir": result.get("tracking_frame_dir"),
    }
    _write_state(path, state)


def clear_detection_overlay(path: Path = OVERLAY_STATE_PATH) -> None:
    """清除检测框 overlay。"""
    state = _read_state(path)
    state["detection"] = None
    _write_state(path, state)


def clear_tracking_overlay(path: Path = OVERLAY_STATE_PATH) -> None:
    """清除追踪分割 overlay。"""
    state = _read_state(path)
    state["tracking"] = None
    _write_state(path, state)


def read_overlay_state(path: Path = OVERLAY_STATE_PATH) -> dict[str, Any]:
    """读取当前 overlay 状态。"""
    return _read_state(path)


def _read_state(path: Path) -> dict[str, Any]:
    """读取状态文件，文件不存在或损坏时返回空状态。"""
    if not path.exists():
        return {"detection": None, "tracking": None}
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {"detection": None, "tracking": None}
    if not isinstance(raw, dict):
        return {"detection": None, "tracking": None}
    return {
        "detection": raw.get("detection"),
        "tracking": raw.get("tracking"),
    }


def _write_state(path: Path, state: dict[str, Any]) -> None:
    """原子写入状态文件，避免相机进程读到半截 JSON。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = path.with_suffix(f"{path.suffix}.tmp")
    tmp_path.write_text(
        json.dumps(state, ensure_ascii=False, indent=2, default=str),
        encoding="utf-8",
    )
    tmp_path.replace(path)
