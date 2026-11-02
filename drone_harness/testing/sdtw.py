"""按实际飞行动作终点计算三维 SDTW，不向导航模型提供评分数据。"""

from __future__ import annotations

import math

import numpy as np


MOTION_TOOLS = frozenset({"takeoff", "move", "forward", "up", "down", "rotate", "return_home", "land"})
SAMPLING_METHOD = "motion_end_world_xyz_v1"


def is_motion_event(event: dict) -> bool:
    """选择已执行或有部分运动依据的飞行动作，排除明确未移动和零度旋转。"""
    result = event.get("result")
    name = event.get("tool_name")
    if event.get("event_type") != "tool_call" or name not in MOTION_TOOLS or not isinstance(result, dict):
        return False
    if result.get("motion_executed") is False:
        return False
    arguments = event.get("arguments")
    if name == "rotate" and isinstance(arguments, dict) and arguments.get("degrees") == 0:
        return False
    return (result.get("success") is True or result.get("motion_executed") is True
            or result.get("final_position_ned") is not None)


def trajectory_xyz(points) -> np.ndarray:
    """校验整条有限三维轨迹；参考位姿仅提取前三维，不跳过损坏点。"""
    array = np.asarray(points, dtype=float)
    if array.ndim != 2 or not len(array) or array.shape[1] < 3 or not np.isfinite(array[:, :3]).all():
        raise ValueError("轨迹缺失或包含非法三维坐标")
    return array[:, :3]


def calculate_sdtw(actual, reference, success: int, *, sampling_error: str | None = None) -> dict:
    """沿用官方 FastDTW 欧氏距离及20米归一化，只对实际轨迹去除连续相同位置。"""
    details = {"sampling_method": SAMPLING_METHOD, "distance_dimensions": "world_xyz",
               "normalization_distance_m": 20.0, "SDTW": None, "nDTW": None,
               "dtw_distance_m": None, "actual_point_count": 0, "reference_point_count": 0,
               "reason": sampling_error}
    if sampling_error:
        return details
    try:
        predicted = trajectory_xyz(actual)
        expected = trajectory_xyz(reference)
    except (TypeError, ValueError, OverflowError):
        details["reason"] = "missing_or_invalid_trajectory"
        return details
    predicted = predicted[np.r_[True, np.any(predicted[1:] != predicted[:-1], axis=1)]]
    from fastdtw import fastdtw

    distance, _ = fastdtw(predicted, expected, dist=2)
    ndtw = math.exp(-distance / (len(expected) * 20.0))
    details.update(SDTW=int(bool(success)) * ndtw, nDTW=ndtw, dtw_distance_m=float(distance),
                   actual_point_count=len(predicted), reference_point_count=len(expected))
    return details
