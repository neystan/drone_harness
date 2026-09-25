"""DINO-XSEEK 语义检测封装。"""

from __future__ import annotations

import json
import site
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any

from drone_agent.config.schema import RuntimeProfile
from drone_agent.vision.overlay import clear_detection_overlay, write_detection_overlay


def detect_image_targets(
    profile: RuntimeProfile,
    image_path: Path,
    frame_shape: tuple[int, ...],
    target_description: str,
) -> dict[str, Any]:
    """调用 DINO-XSEEK 检测当前图像中的语义目标。"""
    started_at = time.perf_counter()
    result = call_dinoxseek(profile, image_path, target_description)
    latency_ms = int((time.perf_counter() - started_at) * 1000)
    raw_result_path = save_raw_result(result, image_path)
    normalized = normalize_dinoxseek_result(
        profile=profile,
        raw_result=result,
        target_description=target_description,
        image_path=image_path,
        raw_result_path=raw_result_path,
        frame_shape=frame_shape,
        latency_ms=latency_ms,
    )
    add_detection_visualization(normalized, image_path)
    return normalized


def add_detection_visualization(result: dict[str, Any], image_path: Path) -> None:
    """把检测框写入相机预览 overlay。"""
    objects = result.get("objects", [])
    if not objects:
        clear_detection_overlay()
        return

    try:
        write_detection_overlay(objects)
        result["overlay_updated"] = True
    except Exception as exc:
        result["visualization_error"] = str(exc)


def call_dinoxseek(
    profile: RuntimeProfile,
    image_path: Path,
    target_description: str,
) -> dict[str, Any]:
    """通过 DDS Cloud API 调用 DINO-XSEEK。"""
    ensure_user_site_packages()
    try:
        from dds_cloudapi_sdk import Client, Config
        from dds_cloudapi_sdk.tasks.v2_task import create_task_with_local_image_auto_resize
    except ImportError as exc:
        raise RuntimeError(
            "dds_cloudapi_sdk is required for DINO-XSEEK detection; "
            f"runtime_python={sys.executable}; "
            f"sys_path_head={sys.path[:5]}"
        ) from exc

    client = Client(Config(profile.detector.api_key))
    task = create_task_with_local_image_auto_resize(
        api_path=profile.detector.api_path,
        api_body_without_image={
            "model": profile.detector.model,
            "prompt": {
                "type": "text",
                "text": target_description,
            },
            "targets": ["bbox"],
        },
        image_path=str(image_path),
    )
    client.run_task(task)
    result = task.result
    if not isinstance(result, dict):
        raise ValueError("DINO-XSEEK response must be a mapping")
    return result


def ensure_user_site_packages() -> None:
    """确保 ROS2 启动脚本也能读取用户 pip 安装的 SDK。"""
    user_site = site.getusersitepackages()
    if user_site and user_site not in sys.path:
        sys.path.append(user_site)


def save_raw_result(raw_result: dict[str, Any], image_path: Path) -> Path:
    """保存完整 API 返回，避免把 raw JSON 全量塞进 LLM 上下文。"""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    raw_result_path = image_path.with_name(
        f"{image_path.stem}_dinoxseek_{timestamp}.json"
    )
    raw_result_path.write_text(
        json.dumps(raw_result, ensure_ascii=False, indent=2, default=str),
        encoding="utf-8",
    )
    return raw_result_path


def normalize_dinoxseek_result(
    *,
    profile: RuntimeProfile,
    raw_result: dict[str, Any],
    target_description: str,
    image_path: Path,
    raw_result_path: Path,
    frame_shape: tuple[int, ...],
    latency_ms: int,
) -> dict[str, Any]:
    """把 DINO-XSEEK 原始结果整理成 LLM 可读的稳定结构。"""
    image_height = int(frame_shape[0])
    image_width = int(frame_shape[1])
    raw_objects = raw_result.get("objects", [])
    if not isinstance(raw_objects, list):
        raw_objects = []
    objects = [
        normalized
        for normalized in (
            normalize_object(obj, index, image_width, image_height, target_description)
            for index, obj in enumerate(raw_objects)
        )
        if normalized is not None
    ]
    return {
        "success": True,
        "provider": profile.detector.provider,
        "model": profile.detector.model,
        "target_description": target_description,
        "target_found": bool(objects),
        "object_count": len(objects),
        "image_path": str(image_path),
        "raw_result_path": str(raw_result_path),
        "image_width": image_width,
        "image_height": image_height,
        "latency_ms": latency_ms,
        "objects": objects,
    }


def normalize_object(
    obj: Any,
    index: int,
    image_width: int,
    image_height: int,
    target_description: str,
) -> dict[str, Any] | None:
    """归一化单个候选目标，并计算中心偏移等几何字段。"""
    bbox = read_optional(obj, "bbox")
    if not isinstance(bbox, (list, tuple)) or len(bbox) < 4:
        return None

    try:
        x1, y1, x2, y2 = [float(value) for value in bbox[:4]]
    except (TypeError, ValueError):
        return None

    x1, y1, x2, y2 = clamp_bbox(x1, y1, x2, y2, image_width, image_height)
    if x2 <= x1 or y2 <= y1:
        return None

    width = x2 - x1
    height = y2 - y1
    center_x = x1 + width / 2.0
    center_y = y1 + height / 2.0

    return {
        "index": index,
        "label": str(read_optional(obj, "category", target_description)),
        "bbox_xyxy_px": [round(x1, 2), round(y1, 2), round(x2, 2), round(y2, 2)],
        "bbox_cxcywh_px": [
            round(center_x, 2),
            round(center_y, 2),
            round(width, 2),
            round(height, 2),
        ],
        "center_offset_x": round(
            (center_x - image_width / 2.0) / (image_width / 2.0),
            4,
        ),
        "center_offset_y": round(
            (center_y - image_height / 2.0) / (image_height / 2.0),
            4,
        ),
        "area_ratio": round((width * height) / float(image_width * image_height), 6),
    }


def clamp_bbox(
    x1: float,
    y1: float,
    x2: float,
    y2: float,
    image_width: int,
    image_height: int,
) -> tuple[float, float, float, float]:
    """把候选框裁剪到图像范围内。"""
    return (
        max(0.0, min(float(image_width), x1)),
        max(0.0, min(float(image_height), y1)),
        max(0.0, min(float(image_width), x2)),
        max(0.0, min(float(image_height), y2)),
    )


def read_optional(obj: Any, key: str, default: Any = None) -> Any:
    """同时兼容 dict 和 SDK 对象字段读取。"""
    if isinstance(obj, dict):
        return obj.get(key, default)
    return getattr(obj, key, default)
