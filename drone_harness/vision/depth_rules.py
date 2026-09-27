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
    left_front_obstacle_m: float | None = None
    right_front_obstacle_m: float | None = None
    depth_max_m: float = 20.0

    def as_text(self) -> str:
        """统一输出三个方向的障碍距离，不把量程边界当作障碍。"""
        if not self.depth_valid:
            reason = f"原因：{self.reason}。" if self.reason else ""
            return f"深度无效，前方与左右距离未知；本次观测的前进上限：0.00 米。{reason}"
        def describe(label: str, status: str, distance: float | None) -> str:
            """区分实测障碍、范围内未检出与未知。"""
            if distance is not None:
                return f"{label}障碍距离：{distance:.2f} 米。"
            if status == "clear":
                return f"{label}：{self.depth_max_m:g} 米探测范围内未检测到障碍。"
            return f"{label}障碍距离：未知。"

        return (
            "深度有效。障碍距离均为沿当前朝向、距机体前缘的距离；左右数值不是转向后的可飞距离。\n"
            f"{describe('前方', 'clear', self.front_obstacle_m)}\n"
            f"{describe('左前', self.left_front, self.left_front_obstacle_m)}\n"
            f"{describe('右前', self.right_front, self.right_front_obstacle_m)}\n"
            f"本次观测的前进上限：{self.forward_max_m:.2f} 米。"
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
    side_band = (
        valid
        & (np.abs(vertical_from_camera) <= config.body_half_height_m + config.measurement_margin_m)
    )

    def side_distance(direction: np.ndarray) -> tuple[str, float | None]:
        """在同一量程内取侧方最近障碍，侧方缺测不声称畅通。"""
        sampled = side_band & direction
        obstacles = sampled & (depth < config.depth_max_m)
        # 无效像素无法投影，保守地将该半幅标为未知。
        if np.any(direction & ~valid) or not np.any(sampled):
            return "unknown", None
        if np.any(obstacles):
            return "obstacle", max(0.0, float(np.min(front_clearance[obstacles])))
        return "clear", None

    left_front, left_distance = side_distance(columns[None, :] < 0)
    right_front, right_distance = side_distance(columns[None, :] > 0)
    return DepthRules(
        observation_id=observation_id,
        depth_valid=True,
        front_clearance_m=clearance_m,
        forward_max_m=forward_max_m,
        left_front=left_front,
        right_front=right_front,
        reason="",
        front_obstacle_m=nearest_obstacle_m,
        left_front_obstacle_m=left_distance,
        right_front_obstacle_m=right_distance,
        depth_max_m=config.depth_max_m,
    )
