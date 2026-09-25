"""验证相机几何规则与动态短步上限。"""

from __future__ import annotations

import time
from dataclasses import replace

import numpy as np

from drone_harness.runtime.observation import CameraIntrinsics, ObservationSnapshot
from drone_harness.vision.depth_rules import compute_depth_rules
from test_observation_buffer import observation_config


def snapshot_at(distance_m: float = 4.0) -> ObservationSnapshot:
    """构造经同帧配对的合成透视深度观测。"""
    stamp = time.time_ns()
    depth = np.full((12, 16), distance_m, dtype=np.float32)
    return ObservationSnapshot(
        observation_id=f"rgb-{stamp}", rgb=np.zeros((12, 16, 3), dtype=np.uint8),
        rgb_stamp_ns=stamp, rgb_frame_id="PX4/CameraDepth1_optical", depth=depth,
        depth_stamp_ns=stamp, depth_frame_id="PX4/CameraDepth1_optical",
        depth_encoding="32FC1", intrinsics=CameraIntrinsics(
            16, 12, 8.0, 8.0, 7.5, 5.5, stamp, "CameraDepth1_optical"),
        depth_error="", received_monotonic_ns=time.monotonic_ns(),
    )


def test_clearance_uses_perspective_geometry_and_profile_cap() -> None:
    """透视距离经相机内参投影后才成为机体前缘净空。"""
    rules = compute_depth_rules(snapshot_at(), observation_config(), 0.3)
    assert rules.depth_valid
    assert 3.9 < rules.front_clearance_m < 4.0
    assert rules.forward_max_m == 0.3
    assert rules.observation_id in rules.as_text()


def test_nearer_obstacle_never_increases_forward_limit() -> None:
    """细小正前方障碍缩小上限而通道外障碍不改变前方净空。"""
    config = observation_config()
    base = snapshot_at()
    far = compute_depth_rules(base, config, 5.0)
    near_depth = base.depth.copy()
    near_depth[5, 7] = 0.8
    near = compute_depth_rules(replace(base, depth=near_depth), config, 5.0)
    assert near.forward_max_m < far.forward_max_m
    assert near.front_clearance_m < far.front_clearance_m
    outer_depth = base.depth.copy()
    outer_depth[0, 0] = 2.0
    outer = compute_depth_rules(replace(base, depth=outer_depth), config, 5.0)
    assert outer.front_clearance_m == far.front_clearance_m


def test_unknown_pixels_and_wrong_unit_fail_closed() -> None:
    """通道未知、错误编码或未验证单位都给零前进上限。"""
    base = snapshot_at()
    broken = base.depth.copy()
    broken[5, 7] = np.nan
    for case, config in (
        (replace(base, depth=broken), observation_config()),
        (replace(base, depth_encoding="16UC1"), observation_config()),
        (base, replace(observation_config(), depth_semantics="unverified")),
        (replace(base, depth_error="DEPTH_UNSYNCED"), observation_config()),
    ):
        rules = compute_depth_rules(case, config, 5.0)
        assert not rules.depth_valid
        assert rules.forward_max_m == 0
        assert "forward_max=0.00m" in rules.as_text()


def test_camera_frame_mismatch_and_uncalibrated_braking_prevent_motion() -> None:
    """错相机直接失效，未标定制动余量即便测距有效也不前进。"""
    base = snapshot_at()
    mismatch = replace(base, depth_frame_id="AnotherCamera_optical")
    assert compute_depth_rules(mismatch, observation_config(), 5.0).reason == "RGB_DEPTH_CAMERA_MISMATCH"
    conservative = replace(observation_config(), braking_margin_m=25.0)
    rules = compute_depth_rules(base, conservative, 5.0)
    assert rules.depth_valid
    assert rules.forward_max_m == 0


def test_controller_callbacks_cache_depth_without_running_geometry() -> None:
    """ROS 回调只解码缓存，几何计算留在调用线程。"""
    from types import SimpleNamespace
    from drone_harness.px4.controller import Px4Controller
    from drone_harness.runtime.observation import ObservationBuffer

    controller = object.__new__(Px4Controller)
    controller.observation_buffer = ObservationBuffer(observation_config())
    controller.bridge = SimpleNamespace(imgmsg_to_cv2=lambda msg, _encoding: msg.frame)
    controller.latest_rgb_frame = None
    controller.get_logger = lambda: SimpleNamespace(error=lambda _message: None)
    stamp_ns = time.time_ns()
    stamp = SimpleNamespace(sec=stamp_ns // 1_000_000_000, nanosec=stamp_ns % 1_000_000_000)
    header = SimpleNamespace(stamp=stamp, frame_id="PX4/CameraDepth1_optical")
    controller.rgb_image_callback(SimpleNamespace(header=header, frame=np.zeros((12, 16, 3), dtype=np.uint8)))
    controller.depth_image_callback(SimpleNamespace(header=header, frame=np.ones((12, 16), dtype=np.float32),
                                                    encoding="32FC1"))
    controller.depth_camera_info_callback(SimpleNamespace(
        header=header, width=16, height=12, k=[8, 0, 7.5, 0, 8, 5.5, 0, 0, 1], d=[0] * 5))
    assert controller.observation_buffer.latest_snapshot().depth_error == ""
