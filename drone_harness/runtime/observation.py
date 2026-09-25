"""在现有 ROS 节点内缓存并配对前视 RGB-D 观测。"""

from __future__ import annotations

import base64
import math
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Any

import numpy as np

from drone_harness.config.schema import ObservationConfig


def ros_stamp_ns(stamp: Any) -> int:
    """把 ROS header 时间戳转换为非负纳秒整数。"""
    seconds = int(stamp.sec)
    nanoseconds = int(stamp.nanosec)
    if seconds < 0 or not 0 <= nanoseconds < 1_000_000_000:
        raise ValueError("invalid ROS timestamp")
    return seconds * 1_000_000_000 + nanoseconds


@dataclass(frozen=True)
class CameraIntrinsics:
    """保存深度相机的针孔内参和对应采集时间。"""

    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    stamp_ns: int
    frame_id: str

    @classmethod
    def from_ros_message(cls, message: Any) -> CameraIntrinsics:
        """从 CameraInfo 提取经数值校验的内参。"""
        width = int(message.width)
        height = int(message.height)
        matrix = list(message.k)
        if width <= 0 or height <= 0 or len(matrix) != 9:
            raise ValueError("camera info dimensions or K matrix are invalid")
        distortion = [float(value) for value in getattr(message, "d", [])]
        if len(distortion) < 5:
            raise ValueError("camera distortion calibration is missing")
        if any(not math.isfinite(value) or abs(value) > 1e-9 for value in distortion):
            raise ValueError("distorted camera geometry is not supported")
        if any(abs(float(matrix[index])) > 1e-9 for index in (1, 3, 6, 7)):
            raise ValueError("non-pinhole camera K matrix is not supported")
        if not math.isfinite(float(matrix[8])) or abs(float(matrix[8]) - 1.0) > 1e-9:
            raise ValueError("camera K matrix scale is invalid")
        fx, fy = float(matrix[0]), float(matrix[4])
        cx, cy = float(matrix[2]), float(matrix[5])
        if not all(math.isfinite(value) for value in (fx, fy, cx, cy)):
            raise ValueError("camera intrinsics must be finite")
        if fx <= 0 or fy <= 0 or not 0 <= cx < width or not 0 <= cy < height:
            raise ValueError("camera intrinsics are out of range")
        return cls(
            width=width,
            height=height,
            fx=fx,
            fy=fy,
            cx=cx,
            cy=cy,
            stamp_ns=ros_stamp_ns(message.header.stamp),
            frame_id=str(message.header.frame_id),
        )


@dataclass(frozen=True)
class ImageSample:
    """保存一张不可变图像及其采集、接收时间。"""

    frame: np.ndarray
    stamp_ns: int
    received_monotonic_ns: int
    encoding: str
    frame_id: str


@dataclass(frozen=True)
class ObservationSnapshot:
    """绑定一次 RGB 与可选的同次深度、内参和飞行状态。"""

    observation_id: str
    rgb: np.ndarray
    rgb_stamp_ns: int
    rgb_frame_id: str
    depth: np.ndarray | None
    depth_stamp_ns: int | None
    depth_frame_id: str | None
    depth_encoding: str | None
    intrinsics: CameraIntrinsics | None
    depth_error: str
    received_monotonic_ns: int
    pose_ned: tuple[float, float, float] | None = None
    flight_state: str | None = None
    pose_age_s: float | None = None


class ObservationBuffer:
    """以小型有界缓冲配对同一时钟域的 RGB、深度与内参。"""

    def __init__(self, config: ObservationConfig, capacity: int = 8) -> None:
        """建立线程安全的有界观测缓存。"""
        if capacity < 2:
            raise ValueError("observation buffer capacity must be at least 2")
        self.config = config
        self._condition = threading.Condition()
        self._rgb: deque[ImageSample] = deque(maxlen=capacity)
        self._depth: deque[ImageSample] = deque(maxlen=capacity)
        self._intrinsics: deque[CameraIntrinsics] = deque(maxlen=capacity)

    def add_rgb(
        self,
        frame: np.ndarray,
        stamp_ns: int,
        frame_id: str = "",
        received_monotonic_ns: int | None = None,
    ) -> None:
        """复制并缓存一张带原始采集时间的 BGR 帧。"""
        image = np.asarray(frame)
        if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8:
            raise ValueError("RGB frame must be an HxWx3 uint8 array")
        if stamp_ns <= 0:
            raise ValueError("RGB timestamp must be positive")
        sample = ImageSample(
            frame=image.copy(),
            stamp_ns=int(stamp_ns),
            received_monotonic_ns=(
                time.monotonic_ns() if received_monotonic_ns is None else received_monotonic_ns
            ),
            encoding="bgr8",
            frame_id=frame_id,
        )
        sample.frame.setflags(write=False)
        with self._condition:
            if not self._rgb or sample.stamp_ns > self._rgb[-1].stamp_ns:
                self._rgb.append(sample)
                self._condition.notify_all()

    def add_depth(
        self,
        frame: np.ndarray,
        stamp_ns: int,
        encoding: str,
        frame_id: str = "",
        received_monotonic_ns: int | None = None,
    ) -> None:
        """复制并缓存深度图；编码和单位留待规则层严格校验。"""
        image = np.asarray(frame)
        if image.ndim != 2 or stamp_ns <= 0:
            raise ValueError("depth frame shape or timestamp is invalid")
        sample = ImageSample(
            frame=image.copy(),
            stamp_ns=int(stamp_ns),
            received_monotonic_ns=(
                time.monotonic_ns() if received_monotonic_ns is None else received_monotonic_ns
            ),
            encoding=str(encoding),
            frame_id=frame_id,
        )
        sample.frame.setflags(write=False)
        with self._condition:
            if not self._depth or sample.stamp_ns > self._depth[-1].stamp_ns:
                self._depth.append(sample)
                self._condition.notify_all()

    def add_camera_info(self, intrinsics: CameraIntrinsics) -> None:
        """缓存与深度 topic 同源的相机内参。"""
        with self._condition:
            if not self._intrinsics or intrinsics.stamp_ns > self._intrinsics[-1].stamp_ns:
                self._intrinsics.append(intrinsics)
                self._condition.notify_all()

    def latest_snapshot(self, after_stamp_ns: int = 0) -> ObservationSnapshot | None:
        """无阻塞地读取最新有效 RGB，缺深度时显式标记原因。"""
        with self._condition:
            return self._select_snapshot(after_stamp_ns, time.time_ns(), time.monotonic_ns())

    def wait_for_snapshot(
        self,
        after_stamp_ns: int = 0,
        timeout_s: float | None = None,
    ) -> ObservationSnapshot | None:
        """等待动作之后的新 RGB-D，超时可返回带失效深度的 RGB。"""
        budget_s = self.config.wait_timeout_s if timeout_s is None else timeout_s
        if not math.isfinite(budget_s) or budget_s < 0:
            raise ValueError("observation wait timeout is invalid")
        deadline_ns = time.monotonic_ns() + int(budget_s * 1_000_000_000)
        with self._condition:
            while True:
                snapshot = self._select_snapshot(
                    after_stamp_ns,
                    time.time_ns(),
                    time.monotonic_ns(),
                )
                if snapshot is not None and not snapshot.depth_error:
                    return snapshot
                remaining_ns = deadline_ns - time.monotonic_ns()
                if remaining_ns <= 0:
                    return snapshot
                self._condition.wait(remaining_ns / 1_000_000_000)

    def _select_snapshot(
        self,
        after_stamp_ns: int,
        now_wall_ns: int,
        now_monotonic_ns: int,
    ) -> ObservationSnapshot | None:
        """在锁内选择最新且确实晚于动作结束的配对。"""
        threshold_ns = (
            after_stamp_ns + int(self.config.max_clock_skew_s * 1_000_000_000)
            if after_stamp_ns > 0
            else 0
        )
        rgb = next(
            (
                item
                for item in reversed(self._rgb)
                if item.stamp_ns > threshold_ns
                and self._fresh(item, now_wall_ns, now_monotonic_ns)
            ),
            None,
        )
        if rgb is None:
            return None
        sync_ns = int(self.config.max_sync_delta_s * 1_000_000_000)
        depth_candidates = [
            item
            for item in self._depth
            if item.stamp_ns > threshold_ns
            and abs(item.stamp_ns - rgb.stamp_ns) <= sync_ns
            and self._fresh(item, now_wall_ns, now_monotonic_ns)
        ]
        depth = min(depth_candidates, key=lambda item: abs(item.stamp_ns - rgb.stamp_ns)) if depth_candidates else None
        reason = "" if depth is not None else (
            "DEPTH_UNSYNCED" if self._depth else "DEPTH_MISSING"
        )
        intrinsics = None
        if depth is not None:
            info_candidates = [
                item
                for item in self._intrinsics
                if abs(item.stamp_ns - depth.stamp_ns) <= sync_ns
            ]
            if info_candidates:
                intrinsics = min(
                    info_candidates,
                    key=lambda item: abs(item.stamp_ns - depth.stamp_ns),
                )
            else:
                reason = "CAMERA_INFO_MISSING_OR_UNSYNCED"
        return ObservationSnapshot(
            observation_id=f"rgb-{rgb.stamp_ns}",
            rgb=rgb.frame,
            rgb_stamp_ns=rgb.stamp_ns,
            rgb_frame_id=rgb.frame_id,
            depth=depth.frame if depth is not None else None,
            depth_stamp_ns=depth.stamp_ns if depth is not None else None,
            depth_frame_id=depth.frame_id if depth is not None else None,
            depth_encoding=depth.encoding if depth is not None else None,
            intrinsics=intrinsics,
            depth_error=reason,
            received_monotonic_ns=rgb.received_monotonic_ns,
        )

    def _fresh(self, sample: ImageSample, now_wall_ns: int, now_monotonic_ns: int) -> bool:
        """同时检查采集时间域和接收后的单调时钟龄。"""
        age_ns = now_wall_ns - sample.stamp_ns
        monotonic_age_ns = now_monotonic_ns - sample.received_monotonic_ns
        return (
            -int(self.config.max_clock_skew_s * 1_000_000_000) <= age_ns
            <= int(self.config.max_frame_age_s * 1_000_000_000)
            and 0 <= monotonic_age_ns <= int(self.config.max_frame_age_s * 1_000_000_000)
        )


def build_observation_message(snapshot: ObservationSnapshot, rules: Any) -> dict[str, Any]:
    """把同一观测的深度摘要与内存 JPEG 合成一条多模态消息。"""
    if snapshot.rgb is None or snapshot.rgb.size == 0 or snapshot.rgb_stamp_ns <= 0:
        raise ValueError("RGB observation is unavailable")
    if rules.observation_id != snapshot.observation_id:
        raise ValueError("depth rules do not match the RGB observation")
    if snapshot.rgb.shape[0] * snapshot.rgb.shape[1] > 640 * 480 * 2:
        raise ValueError("RGB observation exceeds image size limit")
    import cv2

    encoded_ok, encoded = cv2.imencode(".jpg", snapshot.rgb, [cv2.IMWRITE_JPEG_QUALITY, 80])
    if not encoded_ok or len(encoded) > 1_000_000:
        raise ValueError("RGB JPEG encoding failed or exceeded byte limit")
    data_url = "data:image/jpeg;base64," + base64.b64encode(encoded.tobytes()).decode("ascii")
    summary = (
        f"新观测：observation_id={snapshot.observation_id}; "
        f"rgb_stamp_ns={snapshot.rgb_stamp_ns}; {rules.as_text()}。"
        "只依据这张 RGB 和同号深度规则决定至多一个动作。"
    )
    return {
        "role": "user",
        "content": [
            {"type": "text", "text": summary},
            {"type": "image_url", "image_url": {"url": data_url, "detail": "low"}},
        ],
    }
