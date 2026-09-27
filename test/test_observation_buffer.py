"""验证带采集时间的 RGB-D 配对不会沿用旧帧。"""

from __future__ import annotations

import time
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

from drone_harness.config.schema import ObservationConfig
from drone_harness.runtime.observation import CameraIntrinsics, ObservationBuffer, ros_stamp_ns


def observation_config() -> ObservationConfig:
    """构造供同步测试使用的保守观测配置。"""
    return ObservationConfig(1.0, 0.05, 0.01, 0.01, "perspective_ray_m", 20.0, 0.5, 0.55,
                             0.45, 0.30, 0.15, 0.2, 0.1, 2.0)


def add_pair(buffer: ObservationBuffer, stamp_ns: int, depth_delta_ns: int = 0) -> None:
    """写入一对可匹配的小图像及同源相机内参。"""
    buffer.add_rgb(np.zeros((4, 4, 3), dtype=np.uint8), stamp_ns, "PX4/CameraDepth1_optical")
    buffer.add_depth(np.full((4, 4), 4.0, dtype=np.float32), stamp_ns + depth_delta_ns,
                     "32FC1", "PX4/CameraDepth1_optical")
    buffer.add_camera_info(CameraIntrinsics(4, 4, 4, 4, 1.5, 1.5, stamp_ns + depth_delta_ns,
                                            "CameraDepth1_optical"))


def test_matching_frames_share_one_observation_id() -> None:
    """近时 RGB、深度和内参形成一次可信配对。"""
    buffer = ObservationBuffer(observation_config())
    stamp = time.time_ns() - 20_000_000
    add_pair(buffer, stamp, 20_000_000)
    snapshot = buffer.latest_snapshot()
    assert snapshot is not None
    assert snapshot.observation_id == f"rgb-{stamp}"
    assert snapshot.depth_error == ""
    assert snapshot.depth_stamp_ns == stamp + 20_000_000
    assert snapshot.rgb.flags.writeable is False


def test_stale_or_late_old_frame_is_not_post_action_observation() -> None:
    """晚收到旧图不能冒充动作后新采集的图。"""
    config = replace(observation_config(), max_frame_age_s=0.05)
    buffer = ObservationBuffer(config)
    old_stamp = time.time_ns() - 100_000_000
    add_pair(buffer, old_stamp)
    assert buffer.latest_snapshot() is None
    fresh_stamp = time.time_ns()
    add_pair(buffer, fresh_stamp)
    assert buffer.latest_snapshot(after_stamp_ns=fresh_stamp) is None


def test_missing_and_unsynced_depth_fail_closed_without_reusing_previous_depth() -> None:
    """缺帧或错时后只返回明确无效的当前 RGB。"""
    buffer = ObservationBuffer(observation_config())
    stamp = time.time_ns() - 80_000_000
    add_pair(buffer, stamp)
    next_stamp = stamp + 70_000_000
    buffer.add_rgb(np.zeros((4, 4, 3), dtype=np.uint8), next_stamp, "PX4/CameraDepth1_optical")
    snapshot = buffer.wait_for_snapshot(after_stamp_ns=stamp - 10_000_000, timeout_s=0.001)
    assert snapshot is not None
    assert snapshot.observation_id == f"rgb-{next_stamp}"
    assert snapshot.depth is None
    assert snapshot.depth_error == "DEPTH_UNSYNCED"


def test_missing_camera_info_is_explicit_and_duplicate_frame_is_ignored() -> None:
    """内参缺失不能给深度规则放行，重复帧不替换缓存。"""
    buffer = ObservationBuffer(observation_config())
    stamp = time.time_ns()
    frame = np.zeros((4, 4, 3), dtype=np.uint8)
    buffer.add_rgb(frame, stamp)
    frame[:] = 99
    buffer.add_rgb(frame, stamp)
    buffer.add_depth(np.ones((4, 4), dtype=np.float32), stamp, "32FC1")
    snapshot = buffer.latest_snapshot()
    assert snapshot is not None
    assert snapshot.depth_error == "CAMERA_INFO_MISSING_OR_UNSYNCED"
    assert np.all(snapshot.rgb == 0)


def test_ros_stamp_rejects_invalid_nanoseconds() -> None:
    """非法 ROS 时间字段不进入观测时钟域。"""
    with pytest.raises(ValueError):
        ros_stamp_ns(SimpleNamespace(sec=1, nanosec=1_000_000_000))


def test_camera_info_rejects_uncalibrated_intrinsics() -> None:
    """缺内参或非零畸变不被当作已校准透视相机。"""
    message = SimpleNamespace(width=4, height=4, k=[4, 0, 1.5, 0, 4, 1.5, 0, 0, 1],
                              d=[0, 0, 0, 0, 0], header=SimpleNamespace(
                                  frame_id="CameraDepth1_optical",
                                  stamp=SimpleNamespace(sec=1, nanosec=0)))
    assert CameraIntrinsics.from_ros_message(message).fx == 4
    message.d[0] = 0.1
    with pytest.raises(ValueError):
        CameraIntrinsics.from_ros_message(message)
