"""SAM2 语义检测和鼠标选点追踪工具实现。"""

from __future__ import annotations

import time
from pathlib import Path
from typing import Any

from drone_agent.bus.intervention import interrupt_if_requested
from drone_agent.vision import mouse_selection, tracking
from drone_agent.vision.dinoxseek import detect_image_targets
from drone_agent.vision.image_store import save_analysis_frame
from drone_agent.vision.sam2_client import Sam2ClientError


MOUSE_SELECTION_TIMEOUT_S = 60.0


def sam_tracking(context: Any, arguments: dict[str, Any]) -> dict:
    """启动、查询、重启或停止 SAM2 语义目标追踪。"""
    if not context.profile.tracker.enabled:
        return {
            "success": False,
            "error": "TRACKER_DISABLED",
            "message": "tracker is disabled in the current runtime profile",
        }

    action = str(arguments.get("action", "")).strip()
    if action not in {"start", "restart", "stop", "status"}:
        return {
            "success": False,
            "error": "INVALID_TRACKING_ACTION",
            "message": "action must be one of start, restart, stop, status",
        }

    if action == "stop":
        try:
            return tracking.stop_tracking(context.profile)
        except Sam2ClientError as exc:
            return {
                "success": False,
                "error": "SAM2_SERVICE_UNAVAILABLE",
                "action": "stop",
                "retryable": True,
                "message": f"SAM2 service is unavailable: {exc}",
            }

    frame = getattr(context.controller, "latest_rgb_frame", None)
    if frame is None:
        return {
            "success": False,
            "error": "IMAGE_NOT_READY",
            "message": "no RGB image has been received yet",
            "camera_topic": context.profile.ros.camera_scene_topic,
        }

    try:
        image_path = save_analysis_frame(frame, context.profile.storage.analysis_save_dir)
    except OSError as exc:
        return {
            "success": False,
            "error": "TRACKING_FRAME_SAVE_FAILED",
            "message": f"failed to save tracking frame: {exc}",
        }

    if action == "status":
        return _tracking_status_with_hover_on_lost(context, image_path)

    target_description = str(arguments.get("target_description", "")).strip()
    if not target_description:
        return {
            "success": False,
            "error": "TARGET_DESCRIPTION_REQUIRED",
            "message": "target_description is required for start or restart",
        }

    return _start_or_restart_tracking(context, action, image_path, target_description, arguments)


def mouse_tracking(context: Any, arguments: dict[str, Any]) -> dict:
    """通过相机窗口鼠标选点启动或管理 SAM2 追踪。"""
    if not context.profile.tracker.enabled:
        return {
            "success": False,
            "error": "TRACKER_DISABLED",
            "message": "tracker is disabled in the current runtime profile",
        }

    action = str(arguments.get("action", "")).strip()
    if action not in {"start", "restart", "stop", "status"}:
        return {
            "success": False,
            "error": "INVALID_TRACKING_ACTION",
            "message": "action must be one of start, restart, stop, status",
        }
    if action in {"status", "stop"}:
        return sam_tracking(context, {"action": action})

    image_path = (
        Path(context.profile.storage.analysis_save_dir)
        / f"mouse_tracking_{time.time_ns()}.png"
    )
    request_id = mouse_selection.begin_mouse_selection(image_path)
    print("mouse-tracking> 请在相机窗口中左键点击目标，右键取消。", flush=True)
    return _wait_for_mouse_selection(context, action, request_id)


def _wait_for_mouse_selection(
    context: Any,
    action: str,
    request_id: str,
) -> dict[str, Any]:
    """等待相机进程提交点击结果，并支持语言介入和超时。"""
    deadline = time.monotonic() + MOUSE_SELECTION_TIMEOUT_S
    while time.monotonic() < deadline:
        interrupted = interrupt_if_requested(context, hover_on_flight_tool=False)
        if interrupted is not None:
            mouse_selection.cancel_mouse_selection(request_id, "interrupted_by_user")
            return interrupted

        state = mouse_selection.read_mouse_selection()
        if state.get("request_id") != request_id:
            return {
                "success": False,
                "error": "MOUSE_SELECTION_REPLACED",
                "message": "mouse selection request was replaced",
            }
        status = state.get("status")
        if status == "selected":
            return _start_tracking_from_mouse_selection(context, action, state)
        if status == "cancelled":
            return {
                "success": False,
                "error": "MOUSE_SELECTION_CANCELLED",
                "message": "mouse target selection was cancelled",
            }
        if status == "failed":
            return {
                "success": False,
                "error": "MOUSE_SELECTION_FAILED",
                "message": str(state.get("message") or "mouse target selection failed"),
            }
        time.sleep(0.05)

    mouse_selection.cancel_mouse_selection(request_id, "timeout")
    return {
        "success": False,
        "error": "MOUSE_SELECTION_TIMEOUT",
        "message": "mouse target selection timed out after 60 seconds",
    }


def _start_tracking_from_mouse_selection(
    context: Any,
    action: str,
    state: dict[str, Any],
) -> dict[str, Any]:
    """校验点击结果并调用 SAM2 点提示启动接口。"""
    image_path = Path(str(state.get("image_path", "")))
    point_xy_px = state.get("point_xy_px")
    if not image_path.is_file() or not _is_valid_point(point_xy_px):
        return {
            "success": False,
            "error": "INVALID_MOUSE_SELECTION",
            "message": "mouse selection did not provide a valid frame and point",
        }

    try:
        if action == "restart":
            result = tracking.restart_point_tracking(
                context.profile,
                image_path,
                point_xy_px,
            )
        else:
            result = tracking.start_point_tracking(
                context.profile,
                image_path,
                point_xy_px,
            )
    except Exception as exc:
        return {
            "success": False,
            "error": "SAM_TRACKING_START_FAILED",
            "message": f"failed to start SAM2 tracking from mouse point: {exc}",
            "exception_type": type(exc).__name__,
            "image_path": str(image_path),
            "point_xy_px": point_xy_px,
        }

    result.update(
        {
            "point_xy_px": point_xy_px,
            "selection_frame_seq": state.get("frame_seq"),
        }
    )
    return result


def _is_valid_point(value: Any) -> bool:
    """判断鼠标坐标是否为两个非负整数。"""
    return (
        isinstance(value, list)
        and len(value) == 2
        and all(
            isinstance(item, int) and not isinstance(item, bool) and item >= 0
            for item in value
        )
    )


def _tracking_status_with_hover_on_lost(context: Any, image_path: Any) -> dict:
    """查询追踪状态，并在目标丢失时立即悬停。"""
    try:
        result = tracking.tracking_status(context.profile, image_path)
    except Exception as exc:
        return {
            "success": False,
            "error": "SAM_TRACKING_STATUS_FAILED",
            "message": f"failed to query SAM2 tracking status: {exc}",
            "exception_type": type(exc).__name__,
            "image_path": str(image_path),
        }

    if not result.get("lost"):
        return result

    hover_result = _force_hover(context.controller)
    result.update(
        {
            "success": False,
            "error": "TRACK_TARGET_LOST",
            "hovered": bool(hover_result.get("success", False)),
            "hover_result": hover_result,
            "message": "SAM2 lost target. Drone switched to hover.",
        }
    )
    return result


def _start_or_restart_tracking(
    context: Any,
    action: str,
    image_path: Any,
    target_description: str,
    arguments: dict[str, Any],
) -> dict:
    """检测语义目标并用选中的候选框初始化 SAM2。"""
    if not context.profile.detector.enabled:
        return {
            "success": False,
            "error": "DETECTOR_DISABLED",
            "message": "detector is required to start SAM2 tracking",
        }

    try:
        detection = detect_image_targets(
            context.profile,
            image_path,
            tuple(context.controller.latest_rgb_frame.shape),
            target_description,
        )
    except Exception as exc:
        return {
            "success": False,
            "error": "DINOXSEEK_DETECTION_FAILED",
            "message": f"failed to detect tracking target with DINO-XSEEK: {exc}",
            "exception_type": type(exc).__name__,
            "image_path": str(image_path),
            "target_description": target_description,
        }

    target = _select_tracking_target(detection, arguments.get("target_index"))
    if target.get("error"):
        target.update(
            {
                "success": False,
                "action": action,
                "target_description": target_description,
                "detection": detection,
            }
        )
        return target

    try:
        if action == "restart":
            result = tracking.restart_tracking(
                context.profile,
                image_path,
                target_description,
                target,
            )
        else:
            result = tracking.start_tracking(
                context.profile,
                image_path,
                target_description,
                target,
            )
    except Exception as exc:
        return {
            "success": False,
            "error": "SAM_TRACKING_START_FAILED",
            "message": f"failed to start SAM2 tracking: {exc}",
            "exception_type": type(exc).__name__,
            "image_path": str(image_path),
            "target_description": target_description,
        }

    result["detection"] = detection
    return result


def _select_tracking_target(
    detection: dict[str, Any],
    target_index: Any,
) -> dict[str, Any]:
    """从 DINO 候选目标中选择一个用于 SAM2 初始化。"""
    objects = detection.get("objects", [])
    if not objects:
        return {
            "error": "TRACK_TARGET_NOT_FOUND",
            "message": "DINO-XSEEK did not find any target to track",
        }

    if target_index is None:
        if len(objects) == 1:
            return objects[0]
        return {
            "error": "TARGET_SELECTION_REQUIRED",
            "message": "multiple targets were detected; provide target_index",
            "needs_target_selection": True,
            "candidate_count": len(objects),
        }

    try:
        requested_index = int(target_index)
    except (TypeError, ValueError):
        return {
            "error": "INVALID_TARGET_INDEX",
            "message": "target_index must be an integer",
        }

    for obj in objects:
        if int(obj.get("index", -1)) == requested_index:
            return obj
    return {
        "error": "TARGET_INDEX_NOT_FOUND",
        "message": "target_index does not exist in detection result",
        "requested_index": requested_index,
    }


def _force_hover(controller: Any) -> dict:
    """目标丢失时尽量直接切换悬停。"""
    try:
        controller.stop_position_hold()
        controller.send_hover_command()
        return {
            "success": True,
            "message": "hover command sent after SAM2 target lost",
        }
    except Exception as exc:
        return {
            "success": False,
            "error": "HOVER_FAILED",
            "message": f"failed to switch to hover after target lost: {exc}",
            "exception_type": type(exc).__name__,
        }
