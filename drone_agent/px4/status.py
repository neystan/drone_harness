"""PX4 状态解析工具。"""

from __future__ import annotations

import math
from typing import Any


def enum_name_from_prefix(enum_cls: type[Any], prefix: str, value: int) -> str:
    """按前缀和数值反查 PX4 枚举名称。"""
    for name in dir(enum_cls):
        if not name.startswith(prefix):
            continue
        if getattr(enum_cls, name) == value:
            return name
    return f"UNKNOWN_{prefix}{value}"


def flight_mode_status_dict(controller: Any, vehicle_status_enum: type[Any]) -> dict:
    """把飞控状态整理成适合工具返回的字典。"""
    heading = getattr(controller.vehicle_local_position, "heading", float("nan"))
    nav_state = controller.vehicle_status.nav_state
    arming_state = controller.vehicle_status.arming_state
    flight_state = controller.flight_state()
    return {
        "success": True,
        "nav_state_name": enum_name_from_prefix(vehicle_status_enum, "NAVIGATION_STATE_", nav_state),
        "arming_state_name": enum_name_from_prefix(vehicle_status_enum, "ARMING_STATE_", arming_state),
        "flight_state": flight_state or "UNKNOWN",
        "in_air": controller.uav_is_in_air(),
        "position_valid": controller.uav_position_is_valid(),
        "heading_valid": math.isfinite(heading),
    }