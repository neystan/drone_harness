"""飞行动作工具实现。"""

from __future__ import annotations

import math
import time
from typing import Any

from drone_harness.bus.intervention import interrupt_if_requested
from drone_harness.px4.frame import is_finite_number
from drone_harness.runtime.safety import request_confirmed_hover
from drone_harness.vision.depth_rules import compute_depth_rules, invalid_depth_rules


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
            "degrees": degrees,
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
                        "degrees": degrees,
                        "target_yaw_rad": final_heading,
                        "final_position_ned": controller.current_position_ned(),
                    }
                time.sleep(controller.timer_period)
            return {
                "success": True,
                "message": f"rotate complete after {direction} {degrees:.1f} degrees",
                "direction": direction,
                "degrees": degrees,
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


def validate_forward(context: Any, distance_m: Any) -> dict[str, Any] | None:
    """校验参数与飞控状态；真机额外保留同号深度硬门。"""
    if not is_finite_number(distance_m):
        return {"success": False, "error": "INVALID_FORWARD_DISTANCE", "message": "distance_m must be finite"}
    distance = float(distance_m)
    if distance <= 0:
        return {"success": False, "error": "INVALID_FORWARD_DISTANCE", "message": "distance_m must be positive"}
    if context.profile.mode == "simulation":
        return _forward_vehicle_rejection(context)
    snapshot = getattr(context, "observation", None)
    rules = getattr(context, "depth_rules", None)
    if snapshot is None or rules is None or rules.observation_id != snapshot.observation_id:
        return {"success": False, "error": "FORWARD_OBSERVATION_MISSING", "message": "no matching RGB-D rules"}
    if not is_finite_number(rules.forward_max_m) or rules.forward_max_m < 0:
        return {"success": False, "error": "FORWARD_DEPTH_INVALID", "message": "bound RGB-D depth is invalid",
                "observation_id": snapshot.observation_id}
    if not rules.depth_valid:
        return {"success": False, "error": "FORWARD_DEPTH_INVALID", "message": "bound RGB-D depth is invalid",
                "observation_id": snapshot.observation_id}
    safe_limit = min(context.profile.forward_step_limit_m, rules.forward_max_m)
    if distance > safe_limit:
        return {"success": False, "error": "FORWARD_LIMIT_EXCEEDED", "message": "requested distance exceeds dynamic limit",
                "observation_id": snapshot.observation_id, "forward_max_m": safe_limit}
    return _forward_vehicle_rejection(context)


def _forward_vehicle_rejection(context: Any) -> dict[str, Any] | None:
    """沿用已连接、已解锁、空中及允许模式的飞控前提。"""
    controller = context.controller
    flight_state = _flight_state(controller)
    if flight_state != "IN_AIR":
        return _flight_state_unavailable() if flight_state is None else {
            "success": False, "error": "NOT_IN_AIR", "message": "forward requires confirmed airborne state"}
    status = getattr(controller, "vehicle_status", None)
    mode = getattr(status, "mode", None)
    allowed_modes = {"OFFBOARD", "AUTO.LOITER"} if context.profile.mode == "simulation" else {"OFFBOARD"}
    if not (bool(getattr(status, "connected", False)) and bool(getattr(status, "armed", False))
            and mode in allowed_modes):
        return {"success": False, "error": "PX4_STATE_INVALID",
            "message": f"PX4 must be connected and armed in {', '.join(sorted(allowed_modes))}; current mode={mode}"}
    return None


def _sim_forward_rules(context: Any):
    """只为本次仿真正前进等待新 RGB-D 并计算几何上限。"""
    started_ns = time.time_ns()
    waiter = getattr(context.controller, "wait_for_observation", None)
    if not callable(waiter):
        return invalid_depth_rules("none", "FORWARD_OBSERVATION_UNAVAILABLE")
    try:
        snapshot = waiter(after_stamp_ns=started_ns)
    except Exception as exc:
        return invalid_depth_rules("none", f"FORWARD_OBSERVATION_ERROR_{type(exc).__name__}")
    if snapshot is None:
        return invalid_depth_rules("none", "FORWARD_OBSERVATION_TIMEOUT")
    observation_id = getattr(snapshot, "observation_id", "none")
    if not isinstance(observation_id, str) or not observation_id:
        return invalid_depth_rules("none", "FORWARD_OBSERVATION_ID_INVALID")
    rgb_stamp_ns = getattr(snapshot, "rgb_stamp_ns", None)
    depth_stamp_ns = getattr(snapshot, "depth_stamp_ns", None)
    threshold_ns = started_ns + int(context.profile.observation.max_clock_skew_s * 1e9)
    if not isinstance(rgb_stamp_ns, int) or rgb_stamp_ns <= threshold_ns:
        return invalid_depth_rules(observation_id, "FORWARD_RGB_NOT_NEW")
    if getattr(snapshot, "depth_error", ""):
        return invalid_depth_rules(observation_id, snapshot.depth_error)
    if not isinstance(depth_stamp_ns, int) or depth_stamp_ns <= threshold_ns:
        return invalid_depth_rules(observation_id, "FORWARD_DEPTH_NOT_NEW")
    sync_ns = int(context.profile.observation.max_sync_delta_s * 1e9)
    intrinsics = getattr(snapshot, "intrinsics", None)
    intrinsics_stamp_ns = getattr(intrinsics, "stamp_ns", None)
    if (abs(rgb_stamp_ns - depth_stamp_ns) > sync_ns or intrinsics is None
            or not isinstance(intrinsics_stamp_ns, int)
            or abs(intrinsics_stamp_ns - depth_stamp_ns) > sync_ns):
        return invalid_depth_rules(observation_id, "FORWARD_RGB_DEPTH_UNSYNCED")
    try:
        rules = compute_depth_rules(snapshot, context.profile.observation,
                                    context.profile.forward_step_limit_m)
    except Exception as exc:
        return invalid_depth_rules(observation_id, f"FORWARD_DEPTH_PARSE_{type(exc).__name__}")
    if rules.observation_id != observation_id:
        return invalid_depth_rules(observation_id, "FORWARD_DEPTH_OBSERVATION_MISMATCH")
    return rules


def forward(context: Any, distance_m: Any) -> dict[str, Any]:
    """仿真每次重取深度后缩短前进，缺深度则正常回报零位移。"""
    rejection = validate_forward(context, distance_m)
    if rejection is not None:
        return rejection
    requested = float(distance_m)
    rules = _sim_forward_rules(context) if context.profile.mode == "simulation" else context.depth_rules
    if not is_finite_number(rules.forward_max_m) or rules.forward_max_m < 0:
        rules = invalid_depth_rules(rules.observation_id, "FORWARD_DEPTH_LIMIT_INVALID")
    commanded = min(requested, context.profile.forward_step_limit_m, rules.forward_max_m) if rules.depth_valid else 0.0
    feedback = {
        "requested_distance_m": requested,
        "commanded_distance_m": commanded,
        "forward_max_m": min(context.profile.forward_step_limit_m, rules.forward_max_m),
        "depth_valid": rules.depth_valid,
        "depth_reason": rules.reason,
        "front_clearance_m": rules.front_clearance_m,
        "front_obstacle_m": rules.front_obstacle_m,
        "observation_id": None if rules.observation_id == "none" else rules.observation_id,
        "clamped": commanded < requested,
    }
    if commanded <= 0:
        if not rules.depth_valid:
            reason = f"深度无效（{rules.reason}），前方距离未知"
        elif rules.front_obstacle_m is not None:
            reason = f"前方约 {rules.front_obstacle_m:.2f} 米有障碍物，需保留约 1 米计划净空"
        else:
            reason = "当前规则未确认可前进净空"
        return {"success": True, "motion_executed": False,
                "message": f"未移动：{reason}；本次前进指令为 0 米，请依据新观测重新规划。", **feedback}
    state_changed = _forward_vehicle_rejection(context)
    if state_changed is not None:
        return {**state_changed, **feedback}
    result = move(context, commanded, 0.0, 0.0,
                  completion_tolerance_m=min(0.05, commanded / 2.0))
    result.update(feedback)
    if result.get("success"):
        result["motion_executed"] = True
        if commanded < requested:
            if rules.front_obstacle_m is not None:
                why = f"前方约 {rules.front_obstacle_m:.2f} 米有障碍物，需保留约 1 米计划净空"
            else:
                why = f"20 米探测视距及单次 {context.profile.forward_step_limit_m:.2f} 米上限"
            result["message"] = (
                f"请求前进 {requested:.2f} 米，因{why}，按安全上限指令前进 {commanded:.2f} 米；"
                f"{result.get('message', '动作已完成')}"
            )
    return result


def _vertical_move(context: Any, distance_m: Any, *, upward: bool) -> dict[str, Any]:
    """校验一次纯垂直位移后复用原 move，不检查上下方向深度。"""
    if not is_finite_number(distance_m):
        return {"success": False, "error": "INVALID_VERTICAL_DISTANCE",
                "message": "distance_m must be a finite number"}
    distance = float(distance_m)
    if distance <= 0:
        return {"success": False, "error": "INVALID_VERTICAL_DISTANCE",
                "message": "distance_m must be positive"}
    limit = context.profile.safety.max_vertical_move_m
    if distance > limit:
        return {"success": False, "error": "VERTICAL_LIMIT_EXCEEDED",
                "message": f"distance_m exceeds the {limit:g} m per-call vertical limit",
                "vertical_max_m": limit}
    controller = context.controller
    state = _flight_state(controller)
    if state != "IN_AIR":
        return _flight_state_unavailable() if state is None else {
            "success": False, "error": "NOT_IN_AIR", "message": "up/down requires confirmed airborne state"}
    status = getattr(controller, "vehicle_status", None)
    mode = getattr(status, "mode", None)
    allowed_modes = {"OFFBOARD", "AUTO.LOITER"} if context.profile.mode == "simulation" else {"OFFBOARD"}
    if not (bool(getattr(status, "connected", False)) and bool(getattr(status, "armed", False))
            and mode in allowed_modes):
        return {"success": False, "error": "PX4_STATE_INVALID",
                "message": f"PX4 must be connected and armed in {', '.join(sorted(allowed_modes))}; current mode={mode}"}
    return move(context, 0.0, 0.0, -distance if upward else distance,
                completion_tolerance_m=min(0.05, distance / 2.0))


def up(context: Any, distance_m: Any) -> dict[str, Any]:
    """在空中按单次垂直限额上升，不设置累计高度上限。"""
    return _vertical_move(context, distance_m, upward=True)


def down(context: Any, distance_m: Any) -> dict[str, Any]:
    """在空中下降，近地目标继续由原 move 拒绝。"""
    return _vertical_move(context, distance_m, upward=False)


def move(
    context: Any,
    x: float,
    y: float,
    z: float,
    *,
    completion_tolerance_m: float | None = None,
) -> dict:
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

    x_limit = profile.safety.max_relative_move_m
    if getattr(profile, "mode", None) == "simulation" and x > 0 and y == 0 and z == 0:
        x_limit = profile.forward_step_limit_m
    if abs(x) > x_limit:
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
        reached_target = (
            controller.is_at_target(target_position)
            if completion_tolerance_m is None
            else math.dist(controller.current_position_ned(), target_position) <= completion_tolerance_m
        )
        if reached_target:
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
