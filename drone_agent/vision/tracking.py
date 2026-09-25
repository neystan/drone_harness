"""维护单目标 SAM2 追踪会话。"""

from __future__ import annotations

import time
import threading
from collections import OrderedDict, deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from drone_agent.config.schema import RuntimeProfile
from drone_agent.vision.overlay import clear_tracking_overlay, write_tracking_overlay
from drone_agent.vision.sam2_client import Sam2TrackingClient


@dataclass
class TrackingSession:
    """记录当前唯一追踪目标。"""

    track_id: str
    target_description: str
    target_index: int | None
    selection_method: str


_CURRENT_SESSION: TrackingSession | None = None


class DisplayFrameRateMeter:
    """Measure how often a distinct frame sequence becomes visible."""

    def __init__(self, window_s: float = 2.0) -> None:
        if window_s <= 0:
            raise ValueError("display FPS window must be positive")
        self.window_s = float(window_s)
        self._last_frame_seq: int | None = None
        self._change_times: deque[float] = deque()

    def update(self, frame_seq: int, now: float | None = None) -> float:
        """Record a display callback and return distinct-frame FPS."""
        current_time = time.monotonic() if now is None else float(now)
        if frame_seq != self._last_frame_seq:
            self._last_frame_seq = frame_seq
            self._change_times.append(current_time)

        cutoff = current_time - self.window_s
        while self._change_times and self._change_times[0] <= cutoff:
            self._change_times.popleft()
        if len(self._change_times) < 2:
            return 0.0

        elapsed = current_time - self._change_times[0]
        if elapsed <= 0:
            return 0.0
        return (len(self._change_times) - 1) / elapsed

    def reset(self) -> None:
        """Discard samples collected under the previous display state."""
        self._last_frame_seq = None
        self._change_times.clear()


class FrameSequenceBuffer:
    """按帧号保存有界的相机原始帧副本。"""

    def __init__(self, capacity: int = 40) -> None:
        if capacity <= 0:
            raise ValueError("frame buffer capacity must be positive")
        self.capacity = capacity
        self._frames: OrderedDict[int, Any] = OrderedDict()
        self._lock = threading.Lock()

    def put(self, frame_seq: int, frame: Any) -> None:
        """保存帧副本，并淘汰最早的帧。"""
        with self._lock:
            self._frames.pop(frame_seq, None)
            self._frames[frame_seq] = frame.copy()
            while len(self._frames) > self.capacity:
                self._frames.popitem(last=False)

    def get(self, frame_seq: int) -> Any | None:
        """返回指定帧的副本。"""
        with self._lock:
            frame = self._frames.get(frame_seq)
            return None if frame is None else frame.copy()

    def clear(self) -> None:
        """清空所有缓存帧。"""
        with self._lock:
            self._frames.clear()


def read_display_mode(value: str | None) -> str:
    """解析预览模式，默认严格匹配 mask 和源帧。"""
    mode = (value or "matched").strip().lower()
    if mode not in {"matched", "live"}:
        raise ValueError(
            "SAM2_DISPLAY_MODE must be 'matched' or 'live', "
            f"got {value!r}"
        )
    return mode


def select_display_frame(
    latest_frame: Any,
    latest_frame_seq: int,
    *,
    tracking_active: bool,
    display_mode: str,
    mask_frame_seq: int | None,
    frame_buffer: FrameSequenceBuffer,
) -> tuple[Any, int, bool]:
    """选择预览帧，并返回是否允许绘制当前 mask。"""
    if tracking_active and display_mode == "matched" and mask_frame_seq is not None:
        matched_frame = frame_buffer.get(mask_frame_seq)
        if matched_frame is not None:
            return matched_frame, mask_frame_seq, True
        return latest_frame.copy(), latest_frame_seq, False

    draw_mask = tracking_active and display_mode == "live" and mask_frame_seq is not None
    return latest_frame.copy(), latest_frame_seq, draw_mask


class RealtimeSamTracker:
    """把最新相机帧送入 SAM2，并保存最新 mask 供窗口绘制。"""

    def __init__(self) -> None:
        """初始化实时追踪状态。"""
        self.displayed_mask: Any | None = None
        self.displayed_mask_path: str | None = None
        self.displayed_mask_frame_seq: int | None = None
        self._mask_lock = threading.Lock()
        self.last_warn_time = 0.0
        self.last_perf_log_time = 0.0
        self.latest_write_ms = 0.0
        self.latest_http_ms = 0.0
        self.latest_total_ms = 0.0
        self.latest_mask_read_ms = 0.0
        self.latest_service_result: dict[str, Any] = {}
        self.latest_service_timing: dict[str, Any] = {}
        self.update_times: deque[float] = deque()

    def update(self, control: dict[str, Any], frame: Any, frame_seq: int, logger: Any) -> bool:
        """用当前帧更新一次 SAM2 追踪结果。"""
        track_id = _read_required_string(control, "track_id")
        base_url = _read_required_string(control, "tracker_base_url")
        frame_dir = _read_required_string(control, "tracking_frame_dir")
        if not track_id or not base_url or not frame_dir:
            return True

        image_path = Path(frame_dir) / f"camera_view_tracking_frame_{frame_seq}.png"
        total_start = time.perf_counter()
        write_start = time.perf_counter()
        if not _write_frame(image_path, frame):
            return False
        self.latest_write_ms = _elapsed_ms(write_start)

        timeout_s = float(control.get("tracker_timeout_s") or 1.0)
        try:
            http_start = time.perf_counter()
            result = Sam2TrackingClient(base_url, timeout_s).status(
                {
                    "track_id": track_id,
                    "image_path": str(image_path),
                    "frame_seq": frame_seq,
                }
            )
            self.latest_http_ms = _elapsed_ms(http_start)
        except Exception as exc:
            self._warn_throttled(logger, f"SAM2 realtime tracking update failed: {exc}")
            return False
        self.latest_total_ms = _elapsed_ms(total_start)
        self.latest_service_result = dict(result)
        self.latest_service_timing = _read_timing(result)
        self._record_update_time()

        if result.get("lost"):
            self.clear()
            clear_tracking_overlay()
            return True

        self._update_displayed_mask(result)
        return True

    def displayed_mask_sequence(self) -> int | None:
        """返回当前已加载 mask 对应的源帧号。"""
        with self._mask_lock:
            return self.displayed_mask_frame_seq

    def draw(
        self,
        frame: Any,
        frame_seq: int,
        *,
        require_sequence_match: bool = True,
    ) -> bool:
        """把 mask 画到帧上；严格模式只接受相同帧号。"""
        with self._mask_lock:
            mask = self.displayed_mask
            mask_frame_seq = self.displayed_mask_frame_seq
        if mask is None:
            return False
        if require_sequence_match and frame_seq != mask_frame_seq:
            return False

        import cv2

        if mask.shape[:2] != frame.shape[:2]:
            mask = cv2.resize(
                mask,
                (frame.shape[1], frame.shape[0]),
                interpolation=cv2.INTER_NEAREST,
            )

        active = mask > 0
        if not active.any():
            return False

        red_layer = frame.copy()
        red_layer[active] = (0, 0, 255)
        cv2.addWeighted(red_layer, 0.35, frame, 0.65, 0, frame)
        return True

    def maybe_log_perf(
        self,
        logger: Any,
        camera_fps: float,
        current_frame_seq: int,
        display_frame_seq: int | None = None,
        display_mode: str = "live",
        visible_fps: float | None = None,
    ) -> None:
        """每秒打印一次实时追踪性能，定位延迟发生在哪一段。"""
        with self._mask_lock:
            displayed_mask_path = self.displayed_mask_path
            displayed_mask_frame_seq = self.displayed_mask_frame_seq
        if not displayed_mask_path and not self.update_times:
            return
        now = time.time()
        if now - self.last_perf_log_time < 1.0:
            return
        self.last_perf_log_time = now

        stale_frames = None
        if displayed_mask_frame_seq is not None:
            stale_frames = max(0, current_frame_seq - displayed_mask_frame_seq)
        display_delay_frames = None
        if display_frame_seq is not None:
            display_delay_frames = max(0, current_frame_seq - display_frame_seq)
        display_delay_ms = None
        if display_delay_frames is not None and camera_fps > 0:
            display_delay_ms = display_delay_frames * 1000.0 / camera_fps
        mask_sync_error_frames = None
        if display_frame_seq is not None and displayed_mask_frame_seq is not None:
            mask_sync_error_frames = display_frame_seq - displayed_mask_frame_seq

        timing = self.latest_service_timing
        service_result = self.latest_service_result
        worker_timing_window = _read_worker_timing_window(service_result)
        logger.info(
            "tracking_perf> "
            f"camera_fps={camera_fps:.1f} "
            f"visible_fps={_format_optional_float(visible_fps)} "
            f"tracking_fps={self._tracking_fps():.1f} "
            f"current_frame_seq={current_frame_seq} "
            f"mask_frame_seq={displayed_mask_frame_seq} "
            f"stale_frames={stale_frames} "
            f"display_frame_seq={display_frame_seq} "
            f"display_delay_frames={display_delay_frames} "
            f"display_delay_ms={_format_optional_float(display_delay_ms)} "
            f"mask_sync_error_frames={mask_sync_error_frames} "
            f"display_mode={display_mode} "
            f"write_ms={self.latest_write_ms:.1f} "
            f"http_ms={self.latest_http_ms:.1f} "
            f"total_ms={self.latest_total_ms:.1f} "
            f"mask_read_ms={self.latest_mask_read_ms:.1f} "
            f"read_image_ms={_format_timing(timing, 'read_image_ms')} "
            f"sam2_infer_ms={_format_timing(timing, 'sam2_infer_ms')} "
            f"write_mask_ms={_format_timing(timing, 'write_mask_ms')} "
            f"service_total_ms={_format_timing(timing, 'total_ms')} "
            f"worker_fps={_format_service_value(service_result, 'worker_fps')} "
            f"worker_processed_count={_format_service_value(service_result, 'worker_processed_count')} "
            f"worker_running={_format_service_value(service_result, 'worker_running')} "
            f"worker_last_sam2_infer_ms={_format_service_value(service_result, 'worker_last_sam2_infer_ms')} "
            f"worker_last_processed_frame_seq={_format_service_value(service_result, 'worker_last_processed_frame_seq')} "
            f"worker_pending_frame_seq={_format_service_value(service_result, 'worker_pending_frame_seq')} "
            f"predicted={_format_service_value(service_result, 'predicted')} "
            f"source_mask_frame_seq={_format_service_value(service_result, 'source_mask_frame_seq')} "
            f"prediction_type={_format_service_value(service_result, 'prediction_type')} "
            f"prediction_delta_frames={_format_service_value(service_result, 'prediction_delta_frames')} "
            f"prediction_shift_xy_px={_format_service_value(service_result, 'prediction_shift_xy_px')} "
            f"worker_image_encoder_ms={_format_top_level_service_value(service_result, 'worker_last_image_encoder_ms')} "
            f"worker_track_step_ms={_format_top_level_service_value(service_result, 'worker_last_track_step_ms')} "
            f"worker_total_ms={_format_top_level_service_value(service_result, 'worker_last_total_ms')} "
            f"worker_memory_updated={_format_top_level_service_value(service_result, 'worker_last_memory_updated')} "
            f"diag_samples={_format_worker_timing_value(worker_timing_window, 'sample_count')} "
            f"queue_p50_ms={_format_worker_timing_value(worker_timing_window, 'queue_wait_ms', 'p50')} "
            f"queue_p95_ms={_format_worker_timing_value(worker_timing_window, 'queue_wait_ms', 'p95')} "
            f"queue_max_ms={_format_worker_timing_value(worker_timing_window, 'queue_wait_ms', 'max')} "
            f"encoder_p50_ms={_format_worker_timing_value(worker_timing_window, 'image_encoder_ms', 'p50')} "
            f"encoder_p95_ms={_format_worker_timing_value(worker_timing_window, 'image_encoder_ms', 'p95')} "
            f"encoder_max_ms={_format_worker_timing_value(worker_timing_window, 'image_encoder_ms', 'max')} "
            f"track_p50_ms={_format_worker_timing_value(worker_timing_window, 'track_step_ms', 'p50')} "
            f"track_p95_ms={_format_worker_timing_value(worker_timing_window, 'track_step_ms', 'p95')} "
            f"track_max_ms={_format_worker_timing_value(worker_timing_window, 'track_step_ms', 'max')} "
            f"compact_p50_ms={_format_worker_timing_value(worker_timing_window, 'compact_state_ms', 'p50')} "
            f"compact_p95_ms={_format_worker_timing_value(worker_timing_window, 'compact_state_ms', 'p95')} "
            f"compact_max_ms={_format_worker_timing_value(worker_timing_window, 'compact_state_ms', 'max')} "
            f"worker_total_p50_ms={_format_worker_timing_value(worker_timing_window, 'total_ms', 'p50')} "
            f"worker_total_p95_ms={_format_worker_timing_value(worker_timing_window, 'total_ms', 'p95')} "
            f"worker_total_max_ms={_format_worker_timing_value(worker_timing_window, 'total_ms', 'max')}"
        )

    def clear(self) -> None:
        """清空当前显示 mask。"""
        with self._mask_lock:
            self.displayed_mask = None
            self.displayed_mask_path = None
            self.displayed_mask_frame_seq = None
        self.latest_service_result = {}
        self.update_times.clear()

    def _update_displayed_mask(self, result: dict[str, Any]) -> None:
        """仅用序号更新的成功响应替换当前显示 mask。"""
        mask_path = result.get("mask_path")
        mask_frame_seq = _read_optional_int(result.get("mask_frame_seq"))
        if not result.get("success") or not isinstance(mask_path, str) or not mask_path:
            return
        if mask_frame_seq is None:
            return
        with self._mask_lock:
            displayed_mask_frame_seq = self.displayed_mask_frame_seq
        if (
            displayed_mask_frame_seq is not None
            and mask_frame_seq <= displayed_mask_frame_seq
        ):
            return

        import cv2

        read_start = time.perf_counter()
        mask = cv2.imread(mask_path, cv2.IMREAD_GRAYSCALE)
        self.latest_mask_read_ms = _elapsed_ms(read_start)
        if mask is None:
            return

        with self._mask_lock:
            if (
                self.displayed_mask_frame_seq is not None
                and mask_frame_seq <= self.displayed_mask_frame_seq
            ):
                return
            self.displayed_mask = mask
            self.displayed_mask_path = mask_path
            self.displayed_mask_frame_seq = mask_frame_seq

    def _warn_throttled(self, logger: Any, message: str) -> None:
        """限制实时追踪失败日志频率。"""
        now = time.time()
        if now - self.last_warn_time >= 2.0:
            logger.warn(message)
            self.last_warn_time = now

    def _record_update_time(self) -> None:
        """记录一次追踪更新，用于计算近 1 秒追踪 FPS。"""
        now = time.time()
        self.update_times.append(now)
        while self.update_times and now - self.update_times[0] > 1.0:
            self.update_times.popleft()

    def _tracking_fps(self) -> float:
        """计算近 1 秒 SAM2 更新频率。"""
        if len(self.update_times) < 2:
            return float(len(self.update_times))
        elapsed = self.update_times[-1] - self.update_times[0]
        if elapsed <= 0:
            return float(len(self.update_times))
        return (len(self.update_times) - 1) / elapsed


def start_tracking(
    profile: RuntimeProfile,
    image_path: Path,
    target_description: str,
    target: dict[str, Any],
) -> dict[str, Any]:
    """用选中的 DINO 候选框启动 SAM2 追踪。"""
    target_index = int(target["index"])
    return _start_tracking_session(
        profile,
        {
            "image_path": str(image_path),
            "target_description": target_description,
            "target_index": target_index,
            "bbox_xyxy_px": target["bbox_xyxy_px"],
        },
        target_description,
        target_index,
        "dinoxseek",
    )


def start_point_tracking(
    profile: RuntimeProfile,
    image_path: Path,
    point_xy_px: list[int],
) -> dict[str, Any]:
    """用鼠标选择的单个正样本点启动 SAM2 追踪。"""
    return _start_tracking_session(
        profile,
        {
            "image_path": str(image_path),
            "target_description": "鼠标选中的目标",
            "point_xy_px": point_xy_px,
        },
        "鼠标选中的目标",
        None,
        "mouse",
    )


def _start_tracking_session(
    profile: RuntimeProfile,
    payload: dict[str, Any],
    target_description: str,
    target_index: int | None,
    selection_method: str,
) -> dict[str, Any]:
    """调用服务端并记录已启动的追踪会话。"""
    global _CURRENT_SESSION

    image_path = Path(str(payload["image_path"]))
    normalized = _normalize_service_result(_client(profile).start(payload))
    if normalized.get("success") is False:
        normalized.setdefault("action", "start")
        normalized.setdefault("tracking_active", False)
        return normalized
    track_id = str(normalized.get("track_id", "")).strip()
    if not track_id:
        return {
            "success": False,
            "error": "SAM_TRACKING_START_FAILED",
            "message": "SAM2 service did not return track_id",
            "service_result": normalized,
        }

    _CURRENT_SESSION = TrackingSession(
        track_id=track_id,
        target_description=target_description,
        target_index=target_index,
        selection_method=selection_method,
    )
    normalized.update(
        {
            "success": bool(normalized.get("success", True)),
            "action": "start",
            "tracking_active": True,
            "track_id": track_id,
            "target_description": target_description,
            "target_index": target_index,
            "selection_method": selection_method,
            "tracker_base_url": str(profile.tracker.base_url),
            "tracker_timeout_s": profile.tracker.timeout_s,
            "tracking_frame_dir": str(image_path.parent),
        }
    )
    write_tracking_overlay(normalized)
    return normalized


def restart_tracking(
    profile: RuntimeProfile,
    image_path: Path,
    target_description: str,
    target: dict[str, Any],
) -> dict[str, Any]:
    """停止旧目标后用新目标重新启动追踪。"""
    stop_tracking(profile)
    result = start_tracking(profile, image_path, target_description, target)
    result["action"] = "restart"
    return result


def restart_point_tracking(
    profile: RuntimeProfile,
    image_path: Path,
    point_xy_px: list[int],
) -> dict[str, Any]:
    """停止旧目标后用鼠标正样本点重新启动追踪。"""
    stop_tracking(profile)
    result = start_point_tracking(profile, image_path, point_xy_px)
    result["action"] = "restart"
    return result


def tracking_status(profile: RuntimeProfile, image_path: Path) -> dict[str, Any]:
    """用当前帧向 SAM2 查询一次最新追踪状态。"""
    global _CURRENT_SESSION

    if _CURRENT_SESSION is None:
        return {
            "success": False,
            "error": "SAM_TRACKING_NOT_ACTIVE",
            "tracking_active": False,
            "message": "no active SAM2 tracking session",
        }

    session = _CURRENT_SESSION
    result = _normalize_service_result(
        _client(profile).status(
            {
                "track_id": session.track_id,
                "image_path": str(image_path),
            }
        )
    )
    lost = bool(result.get("lost", False))
    if lost:
        _CURRENT_SESSION = None
        clear_tracking_overlay()
    result.update(
        {
            "success": bool(result.get("success", not lost)),
            "action": "status",
            "tracking_active": not lost,
            "lost": lost,
            "track_id": session.track_id,
            "target_description": session.target_description,
            "target_index": session.target_index,
            "selection_method": session.selection_method,
            "tracker_base_url": str(profile.tracker.base_url),
            "tracker_timeout_s": profile.tracker.timeout_s,
            "tracking_frame_dir": str(image_path.parent),
        }
    )
    if not lost:
        write_tracking_overlay(result)
    return result


def stop_tracking(profile: RuntimeProfile) -> dict[str, Any]:
    """停止当前 SAM2 追踪会话。"""
    global _CURRENT_SESSION

    if _CURRENT_SESSION is None:
        clear_tracking_overlay()
        return {
            "success": True,
            "action": "stop",
            "tracking_active": False,
            "message": "no active SAM2 tracking session",
        }

    session = _CURRENT_SESSION
    _CURRENT_SESSION = None
    clear_tracking_overlay()
    result = _normalize_service_result(_client(profile).stop({"track_id": session.track_id}))
    result.update(
        {
            "success": bool(result.get("success", True)),
            "action": "stop",
            "tracking_active": False,
            "track_id": session.track_id,
            "target_description": session.target_description,
            "target_index": session.target_index,
            "selection_method": session.selection_method,
        }
    )
    return result


def _client(profile: RuntimeProfile) -> Sam2TrackingClient:
    """根据 profile 创建 SAM2 服务客户端。"""
    tracker = profile.tracker
    return Sam2TrackingClient(str(tracker.base_url), tracker.timeout_s)


def _normalize_service_result(result: dict[str, Any]) -> dict[str, Any]:
    """复制服务返回，避免后续修改影响原对象。"""
    return dict(result)


def _read_required_string(data: dict[str, Any], key: str) -> str | None:
    """读取非空字符串字段。"""
    value = data.get(key)
    if isinstance(value, str) and value:
        return value
    return None


def _read_timing(result: dict[str, Any]) -> dict[str, Any]:
    """读取服务端返回的 timing 字段。"""
    timing = result.get("timing")
    if isinstance(timing, dict):
        return timing
    return {}


def _read_optional_int(value: Any) -> int | None:
    """把服务端返回的可选帧号转成 int。"""
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _elapsed_ms(start: float) -> float:
    """计算从 start 到现在的毫秒耗时。"""
    return (time.perf_counter() - start) * 1000.0


def _format_timing(timing: dict[str, Any], key: str) -> str:
    """格式化服务端 timing 字段。"""
    value = timing.get(key)
    if not isinstance(value, int | float):
        return "NA"
    return f"{float(value):.1f}"


def _format_optional_float(value: float | None) -> str:
    """Format an optional local diagnostic value."""
    return "NA" if value is None else f"{value:.1f}"


def _format_service_value(result: dict[str, Any], key: str) -> str:
    """格式化服务端返回的顶层或 debug 字段。"""
    value = result.get(key)
    if value is None and isinstance(result.get("debug"), dict):
        value = result["debug"].get(key)
    if value is None:
        return "NA"
    if isinstance(value, bool):
        return str(value).lower()
    if isinstance(value, float):
        return f"{value:.1f}"
    return str(value)


def _format_top_level_service_value(result: dict[str, Any], key: str) -> str:
    """格式化服务端顶层诊断字段。"""
    value = result.get(key)
    if value is None:
        return "NA"
    if isinstance(value, bool):
        return str(value).lower()
    if isinstance(value, int | float):
        return f"{float(value):.1f}"
    return str(value)


def _read_worker_timing_window(result: dict[str, Any]) -> dict[str, Any]:
    """读取服务端顶层的 worker 耗时窗口。"""
    window = result.get("worker_timing_window")
    if isinstance(window, dict):
        return window
    return {}


def _format_worker_timing_value(
    window: dict[str, Any], metric: str, statistic: str | None = None
) -> str:
    """格式化 worker 耗时窗口中的统计值。"""
    value = window.get(metric)
    if statistic is not None:
        if not isinstance(value, dict):
            return "NA"
        value = value.get(statistic)
    if not isinstance(value, int | float):
        return "NA"
    return f"{float(value):.1f}"


def _write_frame(image_path: Path, frame: Any) -> bool:
    """保存一帧实时追踪图像。"""
    import cv2

    image_path.parent.mkdir(parents=True, exist_ok=True)
    return bool(cv2.imwrite(str(image_path), frame))
