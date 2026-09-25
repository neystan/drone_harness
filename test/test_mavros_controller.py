from __future__ import annotations

import math
import sys
import types
from pathlib import Path
from types import SimpleNamespace


def _install_ros_stubs() -> None:
    """为纯 Python 单元测试提供最小 ROS2 和 MAVROS 类型桩。"""
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
        rclpy_qos.DurabilityPolicy = type("DurabilityPolicy", (), {"TRANSIENT_LOCAL": 1})
        rclpy_qos.HistoryPolicy = type("HistoryPolicy", (), {"KEEP_LAST": 1})
        rclpy_qos.QoSProfile = type("QoSProfile", (), {})
        rclpy_qos.ReliabilityPolicy = type("ReliabilityPolicy", (), {"BEST_EFFORT": 1})
        rclpy_qos.qos_profile_sensor_data = object()
        sys.modules["rclpy.qos"] = rclpy_qos

    if "sensor_msgs.msg" not in sys.modules:
        sensor_msgs = types.ModuleType("sensor_msgs")
        sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
        sensor_msgs_msg.BatteryState = type("BatteryState", (), {})
        sensor_msgs_msg.Image = type("Image", (), {})
        sensor_msgs_msg.CameraInfo = type("CameraInfo", (), {})
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
        mavros_msgs_msg.PositionTarget = type(
            "PositionTarget",
            (),
            {
                "FRAME_LOCAL_NED": 1,
                "IGNORE_YAW": 1024,
                "IGNORE_YAW_RATE": 2048,
            },
        )
        mavros_msgs_msg.State = type("State", (), {})
        mavros_msgs_msg.StatusText = type("StatusText", (), {})
        sys.modules["mavros_msgs"] = mavros_msgs
        sys.modules["mavros_msgs.msg"] = mavros_msgs_msg

    if "mavros_msgs.srv" not in sys.modules:
        mavros_msgs_srv = types.ModuleType("mavros_msgs.srv")
        for service_name in ("CommandBool", "CommandLong", "CommandTOL", "SetMode"):
            setattr(mavros_msgs_srv, service_name, type(service_name, (), {}))
        sys.modules["mavros_msgs.srv"] = mavros_msgs_srv


_install_ros_stubs()

import drone_harness.px4.controller as controller_module
from drone_harness.px4.controller import Px4Controller


def test_ned_position_is_converted_to_mavros_enu() -> None:
    """验证 NED 位置转换为 MAVROS ENU 坐标。"""
    assert Px4Controller.ned_to_enu([1.0, 2.0, -3.0]) == [2.0, 1.0, 3.0]


def test_unspecified_yaw_fields_are_masked() -> None:
    """验证未指定 yaw 和 yaw-rate 时会设置忽略掩码。"""
    mask = Px4Controller.position_target_type_mask(None, None)
    assert mask & 1024
    assert mask & 2048


def test_explicit_yaw_fields_are_not_masked() -> None:
    """验证显式 yaw 和 yaw-rate 不会被忽略。"""
    mask = Px4Controller.position_target_type_mask(0.5, 0.2)
    assert not mask & 1024
    assert not mask & 2048


def test_mavros_state_maps_mode_and_arming_state() -> None:
    """验证 MAVROS 模式和 armed 状态映射为内部状态。"""
    controller = object.__new__(Px4Controller)
    controller.vehicle_status = SimpleNamespace(
        mode="",
        armed=False,
        nav_state=0,
        arming_state=1,
    )
    controller.vehicle_status_received = False

    controller.state_callback(SimpleNamespace(mode="AUTO.LOITER", armed=True))

    assert controller.vehicle_status.mode == "AUTO.LOITER"
    assert controller.vehicle_status.nav_state == 4
    assert controller.vehicle_status.arming_state == 2
    assert controller.vehicle_status_received


def test_mavros_battery_percentage_is_already_a_ratio() -> None:
    """验证 MAVROS 电量比例不会再次除以一百或误作低电告警。"""
    controller = object.__new__(Px4Controller)
    controller.battery_status_received = False

    controller.battery_status_callback(
        SimpleNamespace(
            percentage=1.0,
            voltage=16.2,
            current=1.0,
            power_supply_status=2,
        )
    )

    assert controller.battery_status.remaining == 1.0
    assert controller.battery_status.warning == 0
    assert controller.battery_status_received


def test_mavros_ack_result_is_preserved() -> None:
    """验证 MAVROS 拒绝结果不会被误判为接受。"""
    controller = object.__new__(Px4Controller)
    ack = SimpleNamespace(success=False, result=2)

    assert not controller.is_command_ack_accepted(ack)
    assert controller.command_ack_result_name(ack) == "DENIED"


def test_navigation_commands_use_mavlink_command_ack_operations() -> None:
    """验证悬停、返航和降落使用带 COMMAND_ACK 的操作。"""
    assert Px4Controller.command_definition("hover") == (176, (1.0, 4.0, 3.0))
    assert Px4Controller.command_definition("return_home") == (20, ())
    assert Px4Controller.command_definition("land") == (21, ())


def test_sensor_subscriptions_use_sensor_data_qos() -> None:
    """验证 MAVROS 传感器订阅使用兼容 BEST_EFFORT 的 QoS。"""
    controller_source = (Path(__file__).parents[1] / "drone_harness/px4/controller.py").read_text(
        encoding="utf-8"
    )

    assert "qos_profile_sensor_data" in controller_source


def test_timer_only_publishes_setpoint_during_synchronous_handshake() -> None:
    """验证定时器不与同步 Offboard/解锁握手重复发送命令。"""
    controller = object.__new__(Px4Controller)
    controller.target_position = [0.0, 0.0, -1.0]
    controller.target_yaw = None
    controller.target_yawspeed = None
    controller.setpoint_counter = 10
    controller.offboard_command_sent = False
    controller.arm_command_sent = False
    controller.offboard_confirmed = False
    controller.arming_confirmed = False
    controller.vehicle_status = SimpleNamespace(mode="", armed=False, connected=True)
    controller.publish_position_setpoint = lambda *_args: None
    offboard_requests: list[bool] = []
    controller.send_offboard_mode_command = lambda: offboard_requests.append(True)
    arm_requests: list[bool] = []
    controller.send_arm_command = lambda: arm_requests.append(True)

    controller.timer_callback()

    assert offboard_requests == []
    assert arm_requests == []

    controller.vehicle_status.mode = "OFFBOARD"
    controller.vehicle_status.armed = True
    controller.timer_callback()

    assert controller.offboard_confirmed is True
    assert controller.arming_confirmed is True
    assert offboard_requests == []
    assert arm_requests == []


def test_start_position_hold_returns_false_when_handshake_fails() -> None:
    """验证 Offboard/解锁握手失败时停止 hold 并返回失败。"""
    controller = object.__new__(Px4Controller)
    controller.vehicle_status = SimpleNamespace(mode="", armed=False, connected=True)
    controller.wait_for_offboard_and_arm = lambda _timeout_s: False
    controller.position_hold_start_error = "OFFBOARD_NOT_CONFIRMED"
    stopped: list[bool] = []
    controller.stop_position_hold = lambda: stopped.append(True)

    result = controller.start_position_hold([0.0, 0.0, -1.0])

    assert result is False
    assert stopped == [True]


def test_start_position_hold_rejects_disconnected_cached_offboard_state(monkeypatch) -> None:
    """验证断开连接时不能接受缓存的 Offboard 和解锁状态。"""
    controller = object.__new__(Px4Controller)
    controller.vehicle_status = SimpleNamespace(mode="OFFBOARD", armed=True, connected=False)
    controller.timer_period = 0.001
    controller.stop_position_hold = lambda: None
    monkeypatch.setattr(controller_module, "OFFBOARD_HANDSHAKE_TIMEOUT_S", 0.01)

    result = controller.start_position_hold([0.0, 0.0, -1.0])

    assert result is False
    assert controller.position_hold_start_error == "MAVROS_NOT_CONNECTED"
