"""用当前前视观察的局部深度测量所选可见表面。"""

from __future__ import annotations

import math
import time
from typing import Any

import numpy as np

from drone_harness.config.schema import ObservationConfig
from drone_harness.runtime.observation import ObservationSnapshot
from drone_harness.vision.depth_rules import depth_geometry_error


def measure_target_distance(
    snapshot: ObservationSnapshot | None, config: ObservationConfig, x: Any, y: Any,
) -> dict[str, Any]:
    """在归一化选点的五乘五邻域汇总射线距离，不推断目标身份。"""
    result: dict[str, Any] = {
        "success": True, "valid": False, "distance_m": None,
        "camera": "front", "distance_reference": "camera_to_visible_surface_ray",
        "observation_id": snapshot.observation_id if snapshot is not None else None,
        "reason": "OBSERVATION_REQUIRED",
    }
    if any(isinstance(value, bool) or not isinstance(value, (int, float))
           or not math.isfinite(value) or not 0 <= value <= 1 for value in (x, y)):
        return {**result, "success": False, "reason": "INVALID_POINT",
                "message": "x、y 必须是 0 到 1 的归一化坐标，原点在图像左上角"}
    if snapshot is None:
        return {**result, "message": "请先 observe 获取前视图；动作后需要重新观察再选点"}
    error = depth_geometry_error(snapshot, config)
    if error:
        return {**result, "reason": error, "message": "当前前视图没有可靠的配对深度"}
    stamps = (snapshot.rgb_stamp_ns, snapshot.depth_stamp_ns, snapshot.intrinsics.stamp_ns)
    if any(stamp is None or stamp <= 0 for stamp in stamps) or (
        max(stamps) - min(stamps) > config.max_sync_delta_s * 1e9
    ):
        return {**result, "reason": "DEPTH_UNSYNCED", "message": "图像、深度与内参时间不匹配"}
    height, width = snapshot.depth.shape
    u, v = int(x * (width - 1) + 0.5), int(y * (height - 1) + 0.5)
    patch = snapshot.depth[max(0, v - 2):min(height, v + 3), max(0, u - 2):min(width, u + 3)]
    valid = np.isfinite(patch) & (patch > 0) & (patch < config.depth_max_m)
    values = patch[valid]
    result.update(pixel={"x": u, "y": v}, sample_count=int(patch.size),
                  valid_count=int(values.size), max_range_m=config.depth_max_m,
                  observation_age_s=max(0.0, (time.monotonic_ns() - snapshot.received_monotonic_ns) / 1e9))
    if values.size < math.ceil(patch.size * 0.8):
        return {**result, "reason": "INSUFFICIENT_VALID_DEPTH",
                "message": "有效深度不足；超出量程的值不能当作目标距离，请重新观察或选点"}
    distance = float(np.median(values))
    p10, p90 = np.percentile(values, [10, 90])
    spread = float(p90 - p10)
    tolerance = max(0.3, distance * 0.05)
    center = float(snapshot.depth[v, u])
    result["spread_m"] = spread
    if spread > tolerance or (math.isfinite(center) and center > 0 and abs(center - distance) > tolerance):
        return {**result, "reason": "MIXED_DEPTH",
                "message": "选点邻域存在明显前后深度差，请选择目标实体表面内部"}
    return {**result, "valid": True, "distance_m": distance, "reason": "OK",
            "message": "距离属于该观察拍摄时的所选可见表面，不等于可直接前进的距离或到达证明"}
