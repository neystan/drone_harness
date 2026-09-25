"""飞行动作工具实现。"""

from __future__ import annotations

import math
import time
from typing import Any

from drone_agent.bus.intervention import interrupt_if_requested
from drone_agent.px4.frame import is_finite_number
from drone_agent.runtime.safety import request_confirmed_hover


WAIT_FOR_POSITION_TIMEOUT_S = 3.0
LAND_TIMEOUT_S = 45.0
COMMAND_ACK_TIMEOUT_S = 2.0


def _flight_state(controller: Any) -> str | None:
    """读取控制器确认的空地状态。"""
    resolver = getattr(controller, "flight_state", None)
    return resolver() if resolver is not None else None


def _flight_state_unavailable() -> dict:
    """返回 landed state 未就绪时的统一错误。"""
    return {
        "success": False,
        "error": "FLIGHT_STATE_UNKNOWN",
        "message": "MAVROS landed state is unavailable or unknown",
    }


def _require_in_air(controller: Any, message: str) -> dict | None:
    """要求 MAVROS 明确确认飞行器处于空中。"""
    state = _flight_state(controller)
    if state is None:
        return _flight_state_unavailable()
    if state != "IN_AIR":
        return {
            "success": False,
            "error": "ALREADY_ON_GROUND",
            "message": message,
        }
    return None


def _nav_state_constant(controller: Any, name: str, fallback: int) -> int:
    """读取 PX4 导航状态常量。"""
    resolver = getattr(controller, "nav_state_constant", None)
    if resolver is not None:
        return int(resolver(name, fallback))
    return int(getattr(controller.vehicle_status.__class__, name, fallback))


def _arming_state_constant(controller: Any, name: str, fallback: int) -> int:
    """读取 PX4 解锁状态常量。"""
    return int(getattr(controller.vehicle_status.__class__, name, fallback))


def _ack_accepted(controller: Any, ack: Any) -> bool:
    """判断 ACK 是否接受。"""
    checker = getattr(controller, "is_command_ack_accepted", None)
    if checker is not None:
        return bool(checker(ack))
    return int(getattr(ack, "result", -1)) == 0


def _ack_result_name(controller: Any, ack: Any) -> str:
    """读取 ACK 结果名称。"""
    formatter = getattr(controller, "command_ack_result_name", None)
    if formatter is not None:
        return str(formatter(ack))
    return str(getattr(ack, "result_name", getattr(ack, "result", "UNKNOWN")))


def _ack_result_or_timeout(controller: Any, ack: Any) -> str:
    """保留 ACK 结果，或明确标记未收到 ACK。"""
    return "ACK_TIMEOUT" if ack is None else _ack_result_name(controller, ack)


def _confirm_nav_command(
    controller: Any,
    request: Any,
    command_name: str,
    state_name: str,
    state_fallback: int,
    success_message: str,
) -> dict[str, Any]:
    """确认命令 ACK 和目标导航状态。"""
    ack = controller.wait_for_command_ack(request, timeout_s=COMMAND_ACK_TIMEOUT_S)
    if ack is not None and not _ack_accepted(controller, ack):
        return {
            "success": False,
            "error": "PX4_COMMAND_REJECTED",
            "command": command_name,
            "px4_result": _ack_result_name(controller, ack),
        }

    expected_state = _nav_state_constant(controller, state_name, state_fallback)
    state_confirmed = controller.wait_for_nav_state(
        expected_state,
        timeout_s=COMMAND_ACK_TIMEOUT_S,
    )
    if not state_confirmed:
        return {
            "success": False,
            "error": "PX4_STATE_UNCONFIRMED",
            "command": command_name,
            "ack_received": ack is not None,
            "px4_result": _ack_result_or_timeout(controller, ack),
        }

    controller.stop_position_hold()
    return {
        "success": True,
        "message": success_message,
        "command": command_name,
        "ack_received": ack is not None,
        "state_confirmed": True,
        "px4_result": _ack_result_or_timeout(controller, ack),
    }


def _handle_position_hold_start_failure(
    controller: Any,
    action_name: str,
    airborne: bool,
) -> dict[str, Any]:
    """处理 Offboard/解锁握手失败，并在空中确认悬停或移交安全控制。"""
    error = str(getattr(controller, "position_hold_start_error", None) or "OFFBOARD_NOT_CONFIRMED")
    detail = getattr(controller, "position_hold_start_detail", None)
    if airborne:
        safety_result = request_confirmed_hover(controller, action_name=action_name)
        return {
            "success": False,
            "error": error,
            **safety_result,
            **({"px4_result": detail} if detail else {}),
            "message": f"{action_name} could not confirm Offboard/arming; PX4 AUTO_LOITER confirmed",
        }
    return {
        "success": False,
        "error": error,
        **({"px4_result": detail} if detail else {}),
        "message": f"{action_name} could not confirm Offboard/arming",
    }


def _wait_for_valid_position(controller: Any) -> bool:
    """等待本地位置状态变为有效。"""
    wait_deadline = time.time() + WAIT_FOR_POSITION_TIMEOUT_S
    while time.time() < wait_deadline:
        if controller.uav_position_is_valid():
            return True
        time.sleep(controller.timer_period)
    return controller.uav_position_is_valid()


def _check_pre_takeoff_requirements(controller: Any, profile: Any) -> dict[str, Any] | None:
    """在真机起飞前执行最小安全检查。"""
    safety = profile.safety
    if not safety.pre_takeoff_gate_enabled:
        return None

    if safety.require_px4_status_ready_for_takeoff and not getattr(
        controller,
        "vehicle_status_received",
        False,
    ):
        return {
            "success": False,
            "error": "TAKEOFF_GATE_STATUS_UNAVAILABLE",
            "message": "px4 vehicle status is unavailable before takeoff",
        }

    if safety.require_battery_status_for_takeoff and not getattr(
        controller,
        "battery_status_received",
        False,
    ):
        return {
            "success": False,
            "error": "TAKEOFF_GATE_BATTERY_UNAVAILABLE",
            "message": "battery status is unavailable before takeoff",
        }

    if safety.require_battery_status_for_takeoff:
        battery = controller.battery_status
        remaining = getattr(battery, "remaining", -1.0)
        if not isinstance(remaining, (int, float)) or remaining < 0.0:
            return {
                "success": False,
                "error": "TAKEOFF_GATE_BATTERY_UNAVAILABLE",
                "message": "battery remaining is unavailable before takeoff",
            }
        remaining_percent = float(remaining) * 100.0
        if remaining_percent < safety.min_battery_percent_for_takeoff:
            return {
                "success": False,
                "error": "TAKEOFF_GATE_BATTERY_TOO_LOW",
                "message": "battery is below takeoff safety threshold",
                "remaining_percent": remaining_percent,
                "required_percent": safety.min_battery_percent_for_takeoff,
            }
    return None


def takeoff(context: Any, height: float) -> dict:
    """控制无人机原地起飞到目标高度。"""
    controller = context.controller
    profile = context.profile
    if not is_finite_number(height):
        return {
            "success": False,
            "error": "INVALID_HEIGHT_TYPE",
            "message": "height must be a number",
        }

    height = float(height)
    if height <= 0.0:
        return {
            "success": False,
            "error": "INVALID_HEIGHT_VALUE",
            "message": "height must be greater than 0",
        }

    if height > profile.safety.max_takeoff_height_m:
        return {
            "success": False,
            "error": "HEIGHT_TOO_LARGE",
            "message": "height exceeds safety limit",
        }

    precheck_result = _check_pre_takeoff_requirements(controller, profile)
    if precheck_result is not None:
        return precheck_result

    if not _wait_for_valid_position(controller):
        return {
            "success": False,
            "error": "POSITION_INVALID",
            "message": "local position is not valid",
        }

    state = _flight_state(controller)
    if state is None:
        return _flight_state_unavailable()
    if state == "IN_AIR":
        return {
            "success": False,
            "error": "ALREADY_IN_AIR",
            "message": "uav is already in air",
        }

    current_position = controller.vehicle_local_position
    target_position = [
        current_position.x,
        current_position.y,
        current_position.z - height,
    ]
    if controller.start_position_hold(target_position) is False:
        return _handle_position_hold_start_failure(controller, "takeoff", airborne=False)
    controller.get_logger().info(f"takeoff(height={height}) accepted, target={target_position}")

    timeout = time.time() + profile.safety.action_timeout_s
    while time.time() < timeout:
        interrupted = interrupt_if_requested(context, hover_on_flight_tool=True)
        if interrupted is not None:
            return interrupted
        if controller.is_at_target(target_position):
            time.sleep(0.5)
            return {
                "success": True,
                "message": f"takeoff complete, reached {height:.1f}m",
                "target_height": height,
                "reference_xy_ned": [current_position.x, current_position.y],
                "reference_z_ned": current_position.z,
                "target_position_ned": target_position,
                "final_position_ned": controller.current_position_ned(),
            }
        time.sleep(controller.timer_period)

    safety_result = request_confirmed_hover(controller, action_name="takeoff")
    return {
        "success": False,
        "error": "TAKEOFF_TIMEOUT",
        **safety_result,
        "message": "takeoff timed out; PX4 AUTO_LOITER confirmed",
        "target_position_ned": target_position,
        "final_position_ned": controller.current_position_ned(),
    }


def land(context: Any) -> dict:
    """控制无人机执行降落。"""
    controller = context.controller
    profile = context.profile
    if not _wait_for_valid_position(controller):
        return {
            "success": False,
            "error": "POSITION_INVALID",
            "message": "local position is not valid",
        }

    state = _flight_state(controller)
    if state is None:
        return _flight_state_unavailable()
    if state == "ON_GROUND":
        return {
            "success": False,
            "error": "ALREADY_ON_GROUND",
            "message": "uav is already on the ground",
        }

    command_result = _confirm_nav_command(
        controller,
        controller.send_land_command(),
        "land",
        "NAVIGATION_STATE_AUTO_LAND",
        18,
        "AUTO_LAND confirmed; landing started",
    )
    if not command_result["success"]:
        return command_result

    timeout = time.time() + max(profile.safety.action_timeout_s, LAND_TIMEOUT_S)
    while time.time() < timeout:
        interrupted = interrupt_if_requested(context, hover_on_flight_tool=True)
        if interrupted is not None:
            return interrupted
        if _flight_state(controller) == "ON_GROUND":
            return {
                "success": True,
                "message": "landing complete, uav is on the ground",
                "final_position_ned": controller.current_position_ned(),
            }
        time.sleep(controller.timer_period)

    safety_result = request_confirmed_hover(controller, action_name="land")
    return {
        "success": False,
        "error": "LAND_TIMEOUT",
        **safety_result,
        "message": "land timed out; PX4 AUTO_LOITER confirmed",
        "final_position_ned": controller.current_position_ned(),
    }


def disarm(controller: Any) -> dict:
    """发送电机上锁命令。"""
    request = controller.send_disarm_command()
    ack = controller.wait_for_command_ack(request, timeout_s=COMMAND_ACK_TIMEOUT_S)
    if ack is not None and not _ack_accepted(controller, ack):
        return {
            "success": False,
            "error": "PX4_COMMAND_REJECTED",
            "command": "disarm",
            "px4_result": _ack_result_name(controller, ack),
        }

    expected_state = _arming_state_constant(controller, "ARMING_STATE_DISARMED", 1)
    if not controller.wait_for_arming_state(expected_state, timeout_s=COMMAND_ACK_TIMEOUT_S):
        return {
            "success": False,
            "error": "PX4_STATE_UNCONFIRMED",
            "command": "disarm",
            "ack_received": ack is not None,
            "px4_result": _ack_result_or_timeout(controller, ack),
        }

    controller.stop_position_hold()
    return {
        "success": True,
        "message": "PX4 disarmed state confirmed",
        "command": "disarm",
        "ack_received": ack is not None,
        "state_confirmed": True,
        "px4_result": _ack_result_or_timeout(controller, ack),
    }


def timer(context: Any, seconds: int) -> dict:
    """执行一个简单的计时等待工具。"""
    if not isinstance(seconds, int):
        return {
            "success": False,
            "error": "INVALID_SECONDS_TYPE",
            "message": "seconds must be an integer",
        }

    if seconds <= 0:
        return {
            "success": False,
            "error": "INVALID_SECONDS_VALUE",
            "message": "seconds must be greater than 0",
        }

    if seconds > 600:
        return {
            "success": False,
            "error": "SECONDS_TOO_LARGE",
            "message": "seconds exceeds safety limit",
        }

    deadline = time.monotonic() + seconds
    print(f"timer> 开始计时，目标时长 {seconds:.1f}s")
    while True:
        interrupted = interrupt_if_requested(context, hover_on_flight_tool=False)
        if interrupted is not None:
            print()
            return interrupted
        now = time.monotonic()
        remaining = deadline - now
        if remaining <= 0:
            break
        elapsed = seconds - remaining
        print(f"\rtimer> 已计时 {elapsed:.1f}s / {seconds:.1f}s", end="", flush=True)
        time.sleep(min(remaining, 0.1))
    print(f"\rtimer> 已计时 {seconds:.1f}s / {seconds:.1f}s")

    return {
        "success": True,
        "message": f"timer complete after {seconds} seconds",
        "waited_seconds": seconds,
    }


def hover(controller: Any) -> dict:
    """切换到 PX4 AUTO_LOITER 悬停模式。"""
    if not _wait_for_valid_position(controller):
        return {
            "success": False,
            "error": "POSITION_INVALID",
            "message": "local position is not valid",
        }

    state_error = _require_in_air(controller, "uav is on the ground and cannot enter hover mode")
    if state_error is not None:
        return state_error

    return _confirm_nav_command(
        controller,
        controller.send_hover_command(),
        "hover",
        "NAVIGATION_STATE_AUTO_LOITER",
        4,
        "PX4 AUTO_LOITER confirmed",
    )


def return_home(controller: Any) -> dict:
    """切换到 PX4 RTL 返航模式。"""
    if not _wait_for_valid_position(controller):
        return {
            "success": False,
            "error": "POSITION_INVALID",
            "message": "local position is not valid",
        }

    state_error = _require_in_air(controller, "uav is on the ground and cannot return home")
    if state_error is not None:
        return state_error

    return _confirm_nav_command(
        controller,
        controller.send_return_home_command(),
        "return_home",
        "NAVIGATION_STATE_AUTO_RTL",
        5,
        "PX4 AUTO_RTL confirmed; return-to-home started",
    )


def rotate(context: Any, direction: str, degrees: float) -> dict:
    """在保持当前位置的同时执行定角度旋转。"""
    controller = context.controller
    profile = context.profile
    if direction not in {"left", "right"}:
        return {
            "success": False,
            "error": "INVALID_DIRECTION",
            "message": "direction must be 'left' or 'right'",
        }

    if not is_finite_number(degrees):
        return {
            "success": False,
            "error": "INVALID_DEGREES_TYPE",
            "message": "degrees must be a number",
        }

    degrees = float(degrees)
    if degrees < 0.0:
        return {
            "success": False,
            "error": "INVALID_DEGREES_VALUE",
            "message": "degrees must be greater than or equal to 0",
        }

    if degrees > profile.safety.max_rotation_deg:
        return {
            "success": False,
            "error": "DEGREES_TOO_LARGE",
            "message": "degrees exceeds safety limit",
        }

    if not _wait_for_valid_position(controller):
        return {
            "success": False,
            "error": "POSITION_INVALID",
            "message": "local position is not valid",
        }

    state_error = _require_in_air(controller, "uav must take off before rotating")
    if state_error is not None:
        return state_error

    current_heading = getattr(controller.vehicle_local_position, "heading", float("nan"))
    if not math.isfinite(current_heading):
        return {
            "success": False,
            "error": "HEADING_INVALID",
            "message": "local heading is not valid",
        }

    target_position = controller.current_position_ned()
    if degrees == 0.0:
        if controller.start_position_hold(target_position, current_heading) is False:
            return _handle_position_hold_start_failure(controller, "rotate", airborne=True)
        return {
            "success": True,
            "message": "rotate complete after 0.0 degrees",
            "direction": direction,
            "target_yaw_rad": current_heading,
        }

    commanded_yawspeed = math.radians(45.0)
    if direction == "left":
        commanded_yawspeed = -commanded_yawspeed

    if controller.start_position_hold(target_position, yawspeed=commanded_yawspeed) is False:
        return _handle_position_hold_start_failure(controller, "rotate", airborne=True)
    controller.get_logger().info(
        f"rotate(direction={direction}, degrees={degrees}) accepted, heading={current_heading}, yawspeed={commanded_yawspeed}"
    )

    accumulated_degrees = 0.0
    previous_heading = current_heading
    timeout = time.time() + max(profile.safety.action_timeout_s, degrees / 20.0)

    while time.time() < timeout:
        interrupted = interrupt_if_requested(context, hover_on_flight_tool=True)
        if interrupted is not None:
            return interrupted
        heading = getattr(controller.vehicle_local_position, "heading", float("nan"))
        if not math.isfinite(heading):
            controller.stop_position_hold()
            return {
                "success": False,
                "error": "HEADING_INVALID",
                "message": "local heading became invalid during rotate",
            }

        heading_delta = math.degrees(controller.normalize_angle(heading - previous_heading))
        previous_heading = heading

        if direction == "right" and heading_delta > 0.0:
            accumulated_degrees += heading_delta
        if direction == "left" and heading_delta < 0.0:
            accumulated_degrees += -heading_delta

        if accumulated_degrees >= degrees:
            final_heading = heading
            if controller.start_position_hold(target_position, final_heading) is False:
                return _handle_position_hold_start_failure(controller, "rotate", airborne=True)
            settle_timeout = time.time() + 3.0
            while time.time() < settle_timeout:
                interrupted = interrupt_if_requested(context, hover_on_flight_tool=True)
                if interrupted is not None:
                    return interrupted
                if controller.is_at_yaw_target(final_heading):
                    return {
                        "success": True,
                        "message": f"rotate complete after {direction} {degrees:.1f} degrees",
                        "direction": direction,
                        "target_yaw_rad": final_heading,
                        "final_position_ned": controller.current_position_ned(),
                    }
                time.sleep(controller.timer_period)
            return {
                "success": True,
                "message": f"rotate complete after {direction} {degrees:.1f} degrees",
                "direction": direction,
                "target_yaw_rad": final_heading,
                "final_position_ned": controller.current_position_ned(),
            }

        time.sleep(controller.timer_period)

    safety_result = request_confirmed_hover(controller, action_name="rotate")
    return {
        "success": False,
        "error": "ROTATE_TIMEOUT",
        **safety_result,
        "message": "rotate timed out; PX4 AUTO_LOITER confirmed",
        "direction": direction,
        "target_yaw_rad": previous_heading,
        "final_position_ned": controller.current_position_ned(),
    }


def move(context: Any, x: float, y: float, z: float) -> dict:
    """按机体系 FRD 偏移执行相对移动。"""
    controller = context.controller
    profile = context.profile
    for name, value in (("x", x), ("y", y), ("z", z)):
        if not is_finite_number(value):
            return {
                "success": False,
                "error": f"INVALID_{name.upper()}_TYPE",
                "message": f"{name} must be a number",
            }

    x = float(x)
    y = float(y)
    z = float(z)

    if abs(x) > profile.safety.max_relative_move_m:
        return {
            "success": False,
            "error": "X_OUT_OF_RANGE",
            "message": "x exceeds safety limit",
        }

    if abs(y) > profile.safety.max_relative_move_m:
        return {
            "success": False,
            "error": "Y_OUT_OF_RANGE",
            "message": "y exceeds safety limit",
        }

    if abs(z) > profile.safety.max_vertical_move_m:
        return {
            "success": False,
            "error": "Z_OUT_OF_RANGE",
            "message": "z exceeds safety limit",
        }

    if not _wait_for_valid_position(controller):
        return {
            "success": False,
            "error": "POSITION_INVALID",
            "message": "local position is not valid",
        }

    state_error = _require_in_air(controller, "uav must take off before moving to a target position")
    if state_error is not None:
        return state_error

    heading = getattr(controller.vehicle_local_position, "heading", float("nan"))
    if not math.isfinite(heading):
        return {
            "success": False,
            "error": "HEADING_INVALID",
            "message": "local heading is not valid",
        }

    current_position = controller.vehicle_local_position
    dx_ned, dy_ned, dz_ned = controller.body_to_ned(x, y, z, heading)
    target_position = [
        current_position.x + dx_ned,
        current_position.y + dy_ned,
        current_position.z + dz_ned,
    ]

    target_height = controller.height_above_ground_m(target_position[2])
    if target_height is None:
        return {
            "success": False,
            "error": "GROUND_REFERENCE_UNAVAILABLE",
            "message": "ground altitude reference is unavailable",
            "target_position_ned": target_position,
        }

    if target_height < 0.3:
        return {
            "success": False,
            "error": "TARGET_Z_TOO_LOW",
            "message": "target altitude is too close to the ground",
            "target_height_m": target_height,
            "target_position_ned": target_position,
        }

    if controller.start_position_hold(target_position) is False:
        return _handle_position_hold_start_failure(controller, "move", airborne=True)
    controller.get_logger().info(
        f"move(body_x={x}, body_y={y}, body_z={z}) accepted, heading={heading}, target={target_position}"
    )

    timeout = time.time() + profile.safety.action_timeout_s
    while time.time() < timeout:
        interrupted = interrupt_if_requested(context, hover_on_flight_tool=True)
        if interrupted is not None:
            return interrupted
        if controller.is_at_target(target_position):
            time.sleep(0.5)
            return {
                "success": True,
                "message": f"move complete after body-frame offset ({x:.2f}, {y:.2f}, {z:.2f})",
                "body_offset_frd": [x, y, z],
                "target_position_ned": target_position,
                "final_position_ned": controller.current_position_ned(),
            }
        time.sleep(controller.timer_period)

    safety_result = request_confirmed_hover(controller, action_name="move")
    return {
        "success": False,
        "error": "MOVE_TIMEOUT",
        **safety_result,
        "message": "move timed out; PX4 AUTO_LOITER confirmed",
        "body_offset_frd": [x, y, z],
        "target_position_ned": target_position,
        "final_position_ned": controller.current_position_ned(),
    }