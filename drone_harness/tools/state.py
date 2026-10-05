"""向规划器提供现有飞控缓存中的位置与飞行状态。"""

from drone_harness.px4.frame import is_finite_number


def get_state(controller) -> dict:
    """读取最近状态；未收到或无效的数据返回未知，不伪造地面位置。"""
    status = getattr(controller, "vehicle_status", None)
    status_received = bool(getattr(controller, "vehicle_status_received", False))
    connected = bool(status.connected) if status_received else None
    position = getattr(controller, "vehicle_local_position", None)
    pose_received = bool(getattr(controller, "pose_received", False))
    position_valid = (
        pose_received and connected is not False
        and bool(getattr(position, "xy_valid", False))
        and bool(getattr(position, "z_valid", False))
        and all(is_finite_number(getattr(position, axis, None)) for axis in ("x", "y", "z"))
    )
    flight_state = (
        controller.flight_state()
        if connected is not False and getattr(controller, "extended_state_received", False)
        else None
    )
    if flight_state not in ("ON_GROUND", "IN_AIR"):
        flight_state = "UNKNOWN"
    height = controller.height_above_ground_m() if position_valid else None
    return {
        "success": True,
        "connected": connected,
        "armed": bool(status.armed) if status_received and connected else None,
        "mode": status.mode or "UNKNOWN" if status_received and connected else "UNKNOWN",
        "in_air": {"ON_GROUND": False, "IN_AIR": True}.get(flight_state),
        "position_ned_m": (
            {"north": float(position.x), "east": float(position.y), "down": float(position.z)}
            if position_valid else None
        ),
        "height_above_reference_m": float(height) if is_finite_number(height) else None,
    }
