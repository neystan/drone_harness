"""Tests for finite tool inputs, PX4 command acknowledgements, and safety handoff."""

from __future__ import annotations

import math
import sys
import types
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

def _install_ros_import_stubs() -> None:
    """让纯 Python 安全测试不依赖已安装的 ROS2 运行时。"""
    if "cv_bridge" not in sys.modules:
        cv_bridge = types.ModuleType("cv_bridge")
        cv_bridge.CvBridge = type("CvBridge", (), {})
        sys.modules["cv_bridge"] = cv_bridge
    if "rclpy.node" not in sys.modules:
        rclpy = types.ModuleType("rclpy")
        rclpy_node = types.ModuleType("rclpy.node")
        rclpy_node.Node = type("Node", (), {})
        sys.modules["rclpy"] = rclpy
        sys.modules["rclpy.node"] = rclpy_node
    if "rclpy.qos" not in sys.modules:
        rclpy_qos = types.ModuleType("rclpy.qos")
        rclpy_qos.DurabilityPolicy = type(
            "DurabilityPolicy", (), {"TRANSIENT_LOCAL": "transient_local"}
        )
        rclpy_qos.HistoryPolicy = type("HistoryPolicy", (), {"KEEP_LAST": "keep_last"})
        rclpy_qos.QoSProfile = type("QoSProfile", (), {})
        rclpy_qos.ReliabilityPolicy = type(
            "ReliabilityPolicy", (), {"BEST_EFFORT": "best_effort"}
        )
        rclpy_qos.qos_profile_sensor_data = object()
        sys.modules["rclpy.qos"] = rclpy_qos
    if "sensor_msgs.msg" not in sys.modules:
        sensor_msgs = types.ModuleType("sensor_msgs")
        sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
        sensor_msgs_msg.BatteryState = type("BatteryState", (), {})
        sensor_msgs_msg.Image = type("Image", (), {})
        sys.modules["sensor_msgs"] = sensor_msgs
        sys.modules["sensor_msgs.msg"] = sensor_msgs_msg

    if "geometry_msgs.msg" not in sys.modules:
        geometry_msgs = types.ModuleType("geometry_msgs")
        geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
        geometry_msgs_msg.PoseStamped = type("PoseStamped", (), {})
        sys.modules["geometry_msgs"] = geometry_msgs
        sys.modules["geometry_msgs.msg"] = geometry_msgs_msg

    if "mavros_msgs.msg" not in sys.modules:
        mavros_msgs = types.ModuleType("mavros_msgs")
        mavros_msgs_msg = types.ModuleType("mavros_msgs.msg")
        mavros_msgs_msg.ExtendedState = type("ExtendedState", (), {})
        mavros_msgs_msg.PositionTarget = type("PositionTarget", (), {})
        mavros_msgs_msg.State = type("State", (), {})
        mavros_msgs_msg.StatusText = type("StatusText", (), {})
        sys.modules["mavros_msgs"] = mavros_msgs
        sys.modules["mavros_msgs.msg"] = mavros_msgs_msg

    if "mavros_msgs.srv" not in sys.modules:
        mavros_msgs_srv = types.ModuleType("mavros_msgs.srv")
        mavros_msgs_srv.CommandBool = type("CommandBool", (), {})
        mavros_msgs_srv.CommandLong = type("CommandLong", (), {})
        mavros_msgs_srv.SetMode = type("SetMode", (), {})
        sys.modules["mavros_msgs.srv"] = mavros_msgs_srv
    if "px4_msgs.msg" not in sys.modules:
        px4_msgs = types.ModuleType("px4_msgs")
        px4_msgs_msg = types.ModuleType("px4_msgs.msg")
        for message_name in (
            "BatteryStatus",
            "OffboardControlMode",
            "TrajectorySetpoint",
            "VehicleCommand",
            "VehicleCommandAck",
            "VehicleLocalPosition",
            "VehicleStatus",
        ):
            setattr(px4_msgs_msg, message_name, type(message_name, (), {}))
        sys.modules["px4_msgs"] = px4_msgs
        sys.modules["px4_msgs.msg"] = px4_msgs_msg


_install_ros_import_stubs()

from drone_agent.px4.controller import CommandRequest, Px4Controller
from drone_agent.runtime.safety import SafetyHandoffRequired
from drone_agent.runtime.tool_dispatcher import _parse_tool_arguments
from drone_agent.tools import flight


class FiniteInputTest(unittest.TestCase):
    def test_json_parser_rejects_non_finite_numbers(self) -> None:
        """验证 JSON 层拒绝 NaN、Infinity 和溢出无限值。"""
        for raw_arguments in (
            '{"height": NaN}',
            '{"height": Infinity}',
            '{"height": -Infinity}',
            '{"height": 1e309}',
        ):
            with self.subTest(raw_arguments=raw_arguments):
                with self.assertRaises(ValueError):
                    _parse_tool_arguments(raw_arguments)

    def test_flight_tools_reject_non_finite_values_before_starting_hold(self) -> None:
        """验证飞行动作在调用 position hold 前拒绝异常数值。"""
        context = _flight_context()
        context.controller.start_position_hold = Mock()

        takeoff_result = flight.takeoff(context, float("nan"))
        move_result = flight.move(context, float("inf"), 0.0, 0.0)
        rotate_result = flight.rotate(context, "right", float("nan"))

        self.assertFalse(takeoff_result["success"])
        self.assertFalse(move_result["success"])
        self.assertFalse(rotate_result["success"])
        context.controller.start_position_hold.assert_not_called()


class SetpointFiniteInputTest(unittest.TestCase):
    def test_start_position_hold_rejects_non_finite_position(self) -> None:
        """验证 position hold 拒绝包含非有限数的位置向量。"""
        controller = object.__new__(Px4Controller)
        controller.target_position = None

        with self.assertRaises(ValueError):
            controller.start_position_hold([0.0, math.nan, 0.0])

    def test_publish_position_setpoint_rejects_non_finite_explicit_yaw(self) -> None:
        """验证 setpoint 发布拒绝显式非有限 yaw。"""
        controller = object.__new__(Px4Controller)
        controller.trajectory_setpoint_publisher = SimpleNamespace(publish=Mock())

        with self.assertRaises(ValueError):
            controller.publish_position_setpoint([0.0, 0.0, 0.0], yaw=math.inf)


class CommandAckTest(unittest.TestCase):
    def test_ack_request_only_accepts_post_request_ack_for_same_command(self) -> None:
        """验证 ACK 必须来自请求发送后且命令号匹配的消息。"""
        controller = object.__new__(Px4Controller)
        controller.vehicle_command_ack = None
        controller.vehicle_command_ack_sequence = 4
        controller.vehicle_command_ack_publisher = SimpleNamespace(publish=Mock())
        request = controller._build_command_request(176)

        self.assertIsNone(controller.get_command_ack(request))
        controller.vehicle_command_ack = SimpleNamespace(command=400, result=1)
        controller.vehicle_command_ack_sequence = 5
        self.assertIsNone(controller.get_command_ack(request))
        controller.vehicle_command_ack = SimpleNamespace(command=176, result=0)
        controller.vehicle_command_ack_sequence = 6
        self.assertEqual(controller.get_command_ack(request).result, 0)

    def test_hover_reports_state_unconfirmed_when_ack_is_accepted_but_mode_does_not_change(self) -> None:
        """验证 ACK 接受但模式未切换时返回状态未确认。"""
        context = _flight_context()
        context.controller.uav_is_in_air = Mock(return_value=True)
        request = CommandRequest(command=176, ack_sequence_before=3)
        context.controller.send_hover_command = Mock(return_value=request)
        context.controller.wait_for_command_ack = Mock(return_value=SimpleNamespace(result=0))
        context.controller.wait_for_nav_state = Mock(return_value=False)

        result = flight.hover(context.controller)

        self.assertFalse(result["success"])
        self.assertEqual(result["error"], "PX4_STATE_UNCONFIRMED")
        context.controller.stop_position_hold.assert_not_called()

    def test_hover_reports_command_rejected(self) -> None:
        """验证 PX4 拒绝悬停命令时返回命令拒绝。"""
        context = _flight_context()
        context.controller.uav_is_in_air = Mock(return_value=True)
        request = CommandRequest(command=176, ack_sequence_before=3)
        context.controller.send_hover_command = Mock(return_value=request)
        context.controller.wait_for_command_ack = Mock(
            return_value=SimpleNamespace(result=2, result_name="DENIED")
        )

        result = flight.hover(context.controller)

        self.assertFalse(result["success"])
        self.assertEqual(result["error"], "PX4_COMMAND_REJECTED")
        context.controller.stop_position_hold.assert_not_called()

    def test_hover_accepts_confirmed_state_when_ack_is_missing(self) -> None:
        """验证 ACK 丢失但 AUTO_LOITER 已确认时仍报告成功。"""
        context = _flight_context()
        context.controller.uav_is_in_air = Mock(return_value=True)
        request = CommandRequest(command=176, ack_sequence_before=3)
        context.controller.send_hover_command = Mock(return_value=request)
        context.controller.wait_for_command_ack = Mock(return_value=None)
        context.controller.wait_for_nav_state = Mock(return_value=True)

        result = flight.hover(context.controller)

        self.assertTrue(result["success"])
        self.assertFalse(result["ack_received"])
        self.assertTrue(result["state_confirmed"])
        self.assertEqual(result["px4_result"], "ACK_TIMEOUT")


class TimeoutHandoffTest(unittest.TestCase):
    def test_timeout_keeps_agent_running_when_hover_is_confirmed(self) -> None:
        """验证动作超时且悬停确认后只返回超时并继续运行。"""
        context = _flight_context()
        controller = context.controller
        controller.uav_is_in_air = Mock(return_value=True)
        controller.uav_position_is_valid = Mock(return_value=True)
        controller.vehicle_local_position.heading = 0.0
        controller.body_to_ned = Mock(return_value=(0.0, 0.0, 0.0))
        controller.start_position_hold = Mock()
        controller.is_at_target = Mock(return_value=False)
        controller.send_hover_command = Mock(
            return_value=CommandRequest(command=176, ack_sequence_before=3)
        )
        controller.wait_for_command_ack = Mock(return_value=SimpleNamespace(result=0))
        controller.wait_for_nav_state = Mock(return_value=True)
        context.profile.safety.action_timeout_s = 0.0

        with patch.object(flight.time, "time", return_value=0.0):
            result = flight.move(context, 0.0, 0.0, 0.0)

        self.assertFalse(result["success"])
        self.assertEqual(result["error"], "MOVE_TIMEOUT")
        self.assertEqual(result["safety_state"], "HOLD_CONFIRMED")
        self.assertEqual(result["message"], "move timed out; PX4 AUTO_LOITER confirmed")
        controller.stop_position_hold.assert_called_once()


class OffboardStartupTest(unittest.TestCase):
    def test_takeoff_returns_when_offboard_startup_fails(self) -> None:
        """验证 Offboard 启动握手失败时起飞立即返回失败。"""
        context = _flight_context()
        controller = context.controller
        controller.start_position_hold = Mock(return_value=False)
        controller.position_hold_start_error = "OFFBOARD_NOT_CONFIRMED"

        result = flight.takeoff(context, 1.0)

        self.assertFalse(result["success"])
        self.assertEqual(result["error"], "OFFBOARD_NOT_CONFIRMED")
        controller.start_position_hold.assert_called_once()

    def test_timeout_without_hover_confirmation_raises_safety_handoff(self) -> None:
        """验证悬停未确认时停止 hold 并触发安全交接异常。"""
        context = _flight_context()
        controller = context.controller
        controller.uav_is_in_air = Mock(return_value=True)
        controller.uav_position_is_valid = Mock(return_value=True)
        controller.vehicle_local_position.heading = 0.0
        controller.body_to_ned = Mock(return_value=(0.0, 0.0, 0.0))
        controller.start_position_hold = Mock()
        controller.is_at_target = Mock(return_value=False)
        controller.send_hover_command = Mock(
            return_value=CommandRequest(command=176, ack_sequence_before=3)
        )
        controller.wait_for_command_ack = Mock(return_value=None)
        controller.wait_for_nav_state = Mock(return_value=False)
        context.profile.safety.action_timeout_s = 0.0

        with patch.object(flight.time, "time", return_value=0.0):
            with self.assertRaisesRegex(
                SafetyHandoffRequired,
                "move timeout; AUTO_LOITER was not confirmed",
            ):
                flight.move(context, 0.0, 0.0, 0.0)

        controller.stop_position_hold.assert_called_once()


def _flight_context() -> SimpleNamespace:
    """创建飞行动作单元测试使用的最小上下文。"""
    position = SimpleNamespace(x=0.0, y=0.0, z=-1.0, heading=0.0)
    controller = SimpleNamespace(
        vehicle_local_position=position,
        vehicle_status=SimpleNamespace(nav_state=0, arming_state=0),
        battery_status=SimpleNamespace(remaining=1.0),
        timer_period=0.01,
        uav_position_is_valid=Mock(return_value=True),
        uav_is_in_air=Mock(return_value=False),
        is_at_target=Mock(return_value=False),
        current_position_ned=Mock(return_value=[0.0, 0.0, -1.0]),
        stop_position_hold=Mock(),
        get_logger=Mock(return_value=SimpleNamespace(info=Mock(), warn=Mock())),
    )
    safety = SimpleNamespace(
        human_in_the_loop_for_flight_tools=False,
        max_takeoff_height_m=3.0,
        max_relative_move_m=5.0,
        max_vertical_move_m=2.0,
        max_rotation_deg=180.0,
        action_timeout_s=1.0,
        hover_on_timeout=True,
        pre_takeoff_gate_enabled=False,
        require_battery_status_for_takeoff=False,
        min_battery_percent_for_takeoff=30.0,
        require_px4_status_ready_for_takeoff=False,
    )
    profile = SimpleNamespace(safety=safety)
    return SimpleNamespace(controller=controller, profile=profile, message_bus=None)


if __name__ == "__main__":
    unittest.main()
