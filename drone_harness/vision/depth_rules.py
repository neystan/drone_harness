"""把已校准的透视深度转换为机体前方净空与单步上限。"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from drone_harness.config.schema import ObservationConfig
from drone_harness.runtime.observation import ObservationSnapshot


@dataclass(frozen=True)
class DepthRules:
    """保存与一张 RGB 帧绑定的可审计几何规则。"""

    observation_id: str
    depth_valid: bool
    front_clearance_m: float | None
    forward_max_m: float
    left_front: str
    right_front: str
    reason: str
    front_obstacle_m: float | None = None

    def as_text(self) -> str:
        """生成给同一 VLM 阅读的简短有单位摘要。"""
        clearance = (
            "unknown"
            if self.front_clearance_m is None
            else f"{self.front_clearance_m:.2f}m"
        )
        return (
            f"observation_id={self.observation_id}; "
            f"depth_valid={str(self.depth_valid).lower()}; "
            f"front_clearance={clearance}; forward_max={self.forward_max_m:.2f}m; "
            f"front_obstacle={'none_within_horizon' if self.front_obstacle_m is None else f'{self.front_obstacle_m:.2f}m'}; "
            f"left_front={self.left_front}; right_front={self.right_front}; "
            f"reason={self.reason or 'ok'}"
        )


def invalid_depth_rules(observation_id: str, reason: str) -> DepthRules:
    """对未知、错配或过期深度统一返回零前进上限。"""
    return DepthRules(
        observation_id=observation_id,
        depth_valid=False,
        front_clearance_m=None,
        forward_max_m=0.0,
        left_front="unknown",
        right_front="unknown",
        reason=reason,
    )


def compute_depth_rules(
    snapshot: ObservationSnapshot,
    config: ObservationConfig,
    profile_max_step_m: float,
) -> DepthRules:
    """由透视射线测距计算前缘净空，失效时不推断可飞距离。"""
    observation_id = snapshot.observation_id
    if snapshot.depth_error:
        return invalid_depth_rules(observation_id, snapshot.depth_error)
    if config.depth_semantics != "perspective_ray_m":
        return invalid_depth_rules(observation_id, "DEPTH_SEMANTICS_UNVERIFIED")
    if snapshot.depth is None or snapshot.intrinsics is None:
        return invalid_depth_rules(observation_id, "DEPTH_OR_INTRINSICS_MISSING")
    if snapshot.depth_encoding != "32FC1":
        return invalid_depth_rules(observation_id, "DEPTH_ENCODING_UNSUPPORTED")
    depth = np.asarray(snapshot.depth)
    intrinsics = snapshot.intrinsics
    if depth.dtype != np.float32 or depth.ndim != 2:
        return invalid_depth_rules(observation_id, "DEPTH_ARRAY_INVALID")
    if depth.shape != (intrinsics.height, intrinsics.width) or snapshot.rgb.shape[:2] != depth.shape:
        return invalid_depth_rules(observation_id, "RGB_DEPTH_GEOMETRY_MISMATCH")
    frame_names = (
        snapshot.rgb_frame_id.split("/")[-1],
        (snapshot.depth_frame_id or "").split("/")[-1],
        intrinsics.frame_id.split("/")[-1],
    )
    if not frame_names[0] or len(set(frame_names)) != 1:
        return invalid_depth_rules(observation_id, "RGB_DEPTH_CAMERA_MISMATCH")
    if (
        isinstance(profile_max_step_m, bool)
        or not isinstance(profile_max_step_m, (int, float))
        or not math.isfinite(profile_max_step_m)
        or profile_max_step_m <= 0
    ):
        return invalid_depth_rules(observation_id, "PROFILE_STEP_LIMIT_INVALID")

    # 有限正深度超过量程时，只能证明该射线在量程内未遇到表面。
    valid = np.isfinite(depth) & (depth > 0)
    # 保守策略：飞行通道内任何无法解释的像素都不可视为自由空间。
    near_plane_m = 0.10
    columns = (np.arange(intrinsics.width, dtype=np.float64) - intrinsics.cx) / intrinsics.fx
    rows = (np.arange(intrinsics.height, dtype=np.float64) - intrinsics.cy) / intrinsics.fy
    corridor_rays = (
        (np.abs(columns)[None, :] * near_plane_m <= config.body_half_width_m + config.measurement_margin_m)
        & (np.abs(rows)[:, None] * near_plane_m <= config.body_half_height_m + config.measurement_margin_m)
    )
    if not np.any(corridor_rays) or np.any(corridor_rays & ~valid):
        return invalid_depth_rules(observation_id, "DEPTH_CORRIDOR_UNKNOWN")

    bounded_range = np.minimum(depth.astype(np.float64), config.depth_max_m)
    ray_denominator = np.sqrt(1.0 + columns[None, :] ** 2 + rows[:, None] ** 2)
    forward_from_camera = bounded_range / ray_denominator
    lateral_from_camera = forward_from_camera * columns[None, :]
    vertical_from_camera = forward_from_camera * rows[:, None]
    body_forward = config.camera_forward_offset_m + forward_from_camera
    front_clearance = body_forward - config.body_front_offset_m
    inside_body_corridor = (
        valid
        & (np.abs(lateral_from_camera) <= config.body_half_width_m + config.measurement_margin_m)
        & (np.abs(vertical_from_camera) <= config.body_half_height_m + config.measurement_margin_m)
    )
    if not np.any(inside_body_corridor):
        return invalid_depth_rules(observation_id, "DEPTH_CORRIDOR_NOT_OBSERVED")
    clearance_m = max(0.0, float(np.min(front_clearance[inside_body_corridor])))
    detected_obstacles = inside_body_corridor & (depth < config.depth_max_m)
    nearest_obstacle_m = (
        max(0.0, float(np.min(front_clearance[detected_obstacles])))
        if np.any(detected_obstacles) else None
    )
    remaining_m = (
        clearance_m
        - config.measurement_margin_m
        - config.braking_margin_m
        - config.latency_margin_m
    )
    forward_max_m = max(0.0, min(float(profile_max_step_m), remaining_m))
    side_near = (
        valid
        & (front_clearance <= config.side_obstacle_distance_m)
        & (np.abs(vertical_from_camera) <= config.body_half_height_m + config.measurement_margin_m)
    )
    left_front = "obstacle" if np.any(side_near & (lateral_from_camera < 0)) else "clear"
    right_front = "obstacle" if np.any(side_near & (lateral_from_camera > 0)) else "clear"
    return DepthRules(
        observation_id=observation_id,
        depth_valid=True,
        front_clearance_m=clearance_m,
        forward_max_m=forward_max_m,
        left_front=left_front,
        right_front=right_front,
        reason="",
        front_obstacle_m=nearest_obstacle_m,
    )
