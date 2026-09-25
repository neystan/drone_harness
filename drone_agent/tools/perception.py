"""感知工具实现。"""

from __future__ import annotations

from typing import Any

from drone_agent.vision.image_store import save_analysis_frame, save_photo
from drone_agent.vision.dinoxseek import detect_image_targets
from drone_agent.vision.vlm import analyze_image


def take_photo(context: Any, _arguments: dict[str, Any]) -> dict:
    """保存当前最新相机画面为一张图片。"""
    frame = getattr(context.controller, "latest_rgb_frame", None)
    if frame is None:
        return {
            "success": False,
            "error": "IMAGE_NOT_READY",
            "message": "no RGB image has been received yet",
            "camera_topic": context.profile.ros.camera_scene_topic,
        }
    try:
        photo_result = save_photo(frame, context.profile.storage.photo_save_dir)
    except OSError as exc:
        message = str(exc)
        if "mkdir" in message.lower() or "directory" in message.lower():
            return {
                "success": False,
                "error": "PHOTO_DIR_CREATE_FAILED",
                "message": f"failed to create photo directory: {exc}",
                "photo_dir": context.profile.storage.photo_save_dir,
            }
        return {
            "success": False,
            "error": "PHOTO_SAVE_FAILED",
            "message": str(exc),
        }

    return {
        "success": True,
        "message": "photo captured successfully",
        "camera_topic": context.profile.ros.camera_scene_topic,
        **photo_result,
    }


def analyze_view(context: Any, arguments: dict[str, Any]) -> dict:
    """调用视觉模型分析当前画面。"""
    frame = getattr(context.controller, "latest_rgb_frame", None)
    if frame is None:
        return {
            "success": False,
            "error": "IMAGE_NOT_READY",
            "message": "no RGB image has been received yet",
            "camera_topic": context.profile.ros.camera_scene_topic,
        }
    if not context.profile.vlm.enabled:
        return {
            "success": False,
            "error": "VLM_DISABLED",
            "message": "vlm is disabled in the current runtime profile",
        }

    try:
        image_path = save_analysis_frame(frame, context.profile.storage.analysis_save_dir)
    except OSError as exc:
        return {
            "success": False,
            "error": "ANALYSIS_FRAME_SAVE_FAILED",
            "message": f"failed to save analysis frame: {exc}",
        }

    target_description = arguments.get("target_description")
    try:
        return analyze_image(context.profile, image_path, target_description)
    except Exception as exc:
        return {
            "success": False,
            "error": "VLM_ANALYSIS_FAILED",
            "message": f"failed to analyze camera view: {exc}",
            "image_path": str(image_path),
            "target_description": target_description or None,
        }


def detect_target(context: Any, arguments: dict[str, Any]) -> dict:
    """调用语义检测模型返回当前画面中的全部候选目标。"""
    frame = getattr(context.controller, "latest_rgb_frame", None)
    if frame is None:
        return {
            "success": False,
            "error": "IMAGE_NOT_READY",
            "message": "no RGB image has been received yet",
            "camera_topic": context.profile.ros.camera_scene_topic,
        }
    if not context.profile.detector.enabled:
        return {
            "success": False,
            "error": "DETECTOR_DISABLED",
            "message": "detector is disabled in the current runtime profile",
        }

    target_description = str(arguments.get("target_description", "")).strip()
    if not target_description:
        return {
            "success": False,
            "error": "TARGET_DESCRIPTION_REQUIRED",
            "message": "target_description is required",
        }

    try:
        image_path = save_analysis_frame(frame, context.profile.storage.analysis_save_dir)
    except OSError as exc:
        return {
            "success": False,
            "error": "DETECTION_FRAME_SAVE_FAILED",
            "message": f"failed to save detection frame: {exc}",
        }

    try:
        return detect_image_targets(
            context.profile,
            image_path,
            tuple(frame.shape),
            target_description,
        )
    except Exception as exc:
        return {
            "success": False,
            "error": "DINOXSEEK_DETECTION_FAILED",
            "message": f"failed to detect target with DINO-XSEEK: {exc}",
            "exception_type": type(exc).__name__,
            "image_path": str(image_path),
            "target_description": target_description,
        }
