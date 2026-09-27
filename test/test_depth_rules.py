"""验证相机几何规则与动态短步上限。"""

from __future__ import annotations

import time
from dataclasses import replace
from pathlib import Path

import numpy as np

from drone_harness.config.loader import load_profile
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
    assert rules.observation_id not in rules.as_text()
    assert "本次观测的前进上限：0.30 米" in rules.as_text()


def test_sim_twenty_meter_horizon_and_one_meter_planned_clearance() -> None:
    """仿真 20 米远景最多约 19 米，前缘 1.5 米障碍只给约 0.5 米。"""
    settings = Path(__file__).parents[1] / "settings.example.json"
    profile = load_profile("sim", settings_path=settings)
    config = profile.observation
    assert config.measurement_margin_m + config.braking_margin_m + config.latency_margin_m == 1.0
    assert profile.forward_step_limit_m == 19.0

    source = snapshot_at(65504.0)
    far = replace(source, intrinsics=replace(source.intrinsics, fx=32.0, fy=32.0))
    far_rules = compute_depth_rules(far, config, profile.forward_step_limit_m)
    assert far_rules.depth_valid
    assert far_rules.front_obstacle_m is None
    assert 18.7 < far_rules.forward_max_m <= 19.0

    near_depth = far.depth.copy()
    near_depth[5, 7] = 1.55
    near_rules = compute_depth_rules(replace(far, depth=near_depth), config,
                                     profile.forward_step_limit_m)
    assert near_rules.depth_valid
    assert 1.48 < near_rules.front_obstacle_m < 1.51
    assert 0.48 < near_rules.forward_max_m < 0.51
    assert "前方障碍距离：" in near_rules.as_text()

    blocked_depth = far.depth.copy()
    blocked_depth[5, 7] = 1.05
    blocked_rules = compute_depth_rules(replace(far, depth=blocked_depth), config,
                                        profile.forward_step_limit_m)
    assert blocked_rules.front_obstacle_m < 1.0
    assert blocked_rules.forward_max_m == 0


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
    """通道缺测、错误编码或未验证单位都给零前进上限。"""
    base = snapshot_at()
    broken = base.depth.copy()
    broken[5, 7] = np.nan
    zero = base.depth.copy()
    zero[5, 7] = 0.0
    infinite = base.depth.copy()
    infinite[5, 7] = np.inf
    for case, config in (
        (replace(base, depth=broken), observation_config()),
        (replace(base, depth=zero), observation_config()),
        (replace(base, depth=infinite), observation_config()),
        (replace(base, depth_encoding="16UC1"), observation_config()),
        (base, replace(observation_config(), depth_semantics="unverified")),
        (replace(base, depth_error="DEPTH_UNSYNCED"), observation_config()),
    ):
        rules = compute_depth_rules(case, config, 5.0)
        assert not rules.depth_valid
        assert rules.forward_max_m == 0
        assert "前进上限：0.00 米" in rules.as_text()


def test_far_depth_only_proves_clearance_to_configured_horizon() -> None:
    """可信远景按 20 米下界计算，不伪造更远净空。"""
    base = snapshot_at(30.0)
    intrinsics = replace(base.intrinsics, fx=32.0, fy=32.0)
    medium = replace(base, intrinsics=intrinsics)
    very_far = replace(medium, depth=np.full_like(base.depth, 65504.0))
    medium_rules = compute_depth_rules(medium, observation_config(), 0.3)
    far_rules = compute_depth_rules(very_far, observation_config(), 0.3)
    assert medium_rules.depth_valid and far_rules.depth_valid
    assert medium_rules.front_clearance_m == far_rules.front_clearance_m
    assert 19.0 < far_rules.front_clearance_m < 20.0
    assert far_rules.forward_max_m == 0.3

    near_depth = very_far.depth.copy()
    near_depth[5, 7] = 0.7
    near_rules = compute_depth_rules(replace(very_far, depth=near_depth), observation_config(), 0.3)
    assert near_rules.depth_valid
    assert near_rules.front_clearance_m < far_rules.front_clearance_m
    assert near_rules.forward_max_m < far_rules.forward_max_m


def test_far_field_with_airsim_camera_geometry_stays_bounded() -> None:
    """实际相机远景只给 20 米净空，真缺测仍触发近场通道门。"""
    base = snapshot_at()
    depth = np.full((480, 640), 65504.0, dtype=np.float32)
    rgb = np.zeros((480, 640, 3), dtype=np.uint8)
    intrinsics = replace(base.intrinsics, width=640, height=480,
                         fx=298.4048, fy=298.4048, cx=319.5, cy=239.5)
    rules = compute_depth_rules(replace(base, depth=depth, rgb=rgb, intrinsics=intrinsics),
                                observation_config(), 0.3)
    assert rules.depth_valid
    assert 19.8 < rules.front_clearance_m < 20.0
    assert rules.forward_max_m == 0.3

    missing = depth.copy()
    missing[0, 0] = np.nan
    blocked = compute_depth_rules(replace(base, depth=missing, rgb=rgb, intrinsics=intrinsics),
                                  observation_config(), 0.3)
    assert blocked.reason == "DEPTH_CORRIDOR_UNKNOWN"
    assert blocked.forward_max_m == 0.0


def test_unknown_outside_projected_corridor_does_not_require_full_coverage() -> None:
    """非通道角落可缺测，通道内缺测仍拒绝。"""
    base = snapshot_at()
    depth = base.depth[:11, :15].copy()
    depth[0, 0] = np.nan
    rgb = base.rgb[:11, :15].copy()
    intrinsics = replace(base.intrinsics, width=15, height=11, fx=1.0, fy=1.0,
                         cx=7.0, cy=5.0)
    snapshot = replace(base, depth=depth, rgb=rgb, intrinsics=intrinsics)
    rules = compute_depth_rules(snapshot, observation_config(), 0.3)
    assert rules.depth_valid
    assert rules.forward_max_m == 0.3

    depth[5, 7] = np.nan
    blocked = compute_depth_rules(replace(snapshot, depth=depth), observation_config(), 0.3)
    assert blocked.reason == "DEPTH_CORRIDOR_UNKNOWN"
    assert blocked.forward_max_m == 0.0


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
