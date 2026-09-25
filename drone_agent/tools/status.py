"""状态查询工具实现。"""

from __future__ import annotations

from typing import Any

from drone_agent.px4.status import flight_mode_status_dict


def current_position_status(controller: Any) -> dict:
    """返回当前无人机的本地 NED 位置。"""
    return {
        "success": True,
        "final_position_ned": controller.current_position_ned(),
    }


def battery_status(controller: Any) -> dict:
    """返回当前电池状态。"""
    battery = controller.battery_status
    return {
        "success": True,
        "connected": battery.connected,
        "voltage_v": battery.voltage_v,
        "current_a": battery.current_a,
        "remaining_ratio": battery.remaining,
        "remaining_percent": battery.remaining * 100.0 if battery.remaining >= 0.0 else None,
        "warning": battery.warning,
    }


def flight_mode_status(controller: Any) -> dict:
    """返回当前飞行模式、解锁状态和定位有效性。"""
    vehicle_status_enum = controller.vehicle_status.__class__
    return flight_mode_status_dict(controller, vehicle_status_enum)
