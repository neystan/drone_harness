"""负责保存拍照和分析图像。"""

from __future__ import annotations

from datetime import datetime
from pathlib import Path
from typing import Any


def _write_image(image_path: Path, frame: Any) -> bool:
    """把图像帧写入指定路径。"""
    import cv2

    return bool(cv2.imwrite(str(image_path), frame))


def save_photo(frame: Any, photo_save_dir: str) -> dict:
    """保存一张拍照图片并返回基础元数据。"""
    photo_dir = Path(photo_save_dir)
    try:
        photo_dir.mkdir(parents=True, exist_ok=True)
    except OSError as exc:
        raise OSError(f"failed to create photo directory: {exc}") from exc

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    image_path = photo_dir / f"photo_{timestamp}.png"
    if not _write_image(image_path, frame):
        raise OSError("failed to save RGB photo to local file")

    return {
        "image_path": str(image_path),
        "image_width": int(frame.shape[1]),
        "image_height": int(frame.shape[0]),
    }


def save_analysis_frame(frame: Any, analysis_save_dir: str) -> Path:
    """保存一张用于视觉分析的临时图片。"""
    analysis_dir = Path(analysis_save_dir)
    try:
        analysis_dir.mkdir(parents=True, exist_ok=True)
    except OSError as exc:
        raise OSError(f"failed to create analysis directory: {exc}") from exc

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    image_path = analysis_dir / f"analysis_{timestamp}.png"
    if not _write_image(image_path, frame):
        raise OSError("failed to save analysis frame")
    return image_path
