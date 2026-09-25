from __future__ import annotations

import sys
import types


def _install_ros_stubs() -> None:
    """为 runtime 选择测试提供最小 ROS2 和 MAVROS 类型桩。"""
    cv_bridge = types.ModuleType("cv_bridge")
    cv_bridge.CvBridge = type("CvBridge", (), {})
    sys.modules.setdefault("cv_bridge", cv_bridge)
    rclpy = types.ModuleType("rclpy")
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = type("Node", (), {})
    sys.modules.setdefault("rclpy", rclpy)
    sys.modules.setdefault("rclpy.node", rclpy_node)
    rclpy_qos = types.ModuleType("rclpy.qos")
    rclpy_qos.qos_profile_sensor_data = object()
    sys.modules.setdefault("rclpy.qos", rclpy_qos)
    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
    geometry_msgs_msg.PoseStamped = type("PoseStamped", (), {})
    sys.modules.setdefault("geometry_msgs", geometry_msgs)
    sys.modules.setdefault("geometry_msgs.msg", geometry_msgs_msg)
    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.BatteryState = type("BatteryState", (), {})
    sensor_msgs_msg.Image = type("Image", (), {})
    sys.modules.setdefault("sensor_msgs", sensor_msgs)
    sys.modules.setdefault("sensor_msgs.msg", sensor_msgs_msg)
    mavros_msgs = types.ModuleType("mavros_msgs")
    mavros_msgs_msg = types.ModuleType("mavros_msgs.msg")
    mavros_msgs_msg.ExtendedState = type("ExtendedState", (), {})
    mavros_msgs_msg.PositionTarget = type("PositionTarget", (), {})
    mavros_msgs_msg.State = type("State", (), {})
    mavros_msgs_msg.StatusText = type("StatusText", (), {})
    sys.modules.setdefault("mavros_msgs", mavros_msgs)
    sys.modules.setdefault("mavros_msgs.msg", mavros_msgs_msg)
    mavros_msgs_srv = types.ModuleType("mavros_msgs.srv")
    mavros_msgs_srv.CommandBool = type("CommandBool", (), {})
    mavros_msgs_srv.CommandLong = type("CommandLong", (), {})
    mavros_msgs_srv.SetMode = type("SetMode", (), {})
    sys.modules.setdefault("mavros_msgs.srv", mavros_msgs_srv)


_install_ros_stubs()

from drone_harness.config.schema import RosConfig
from drone_harness.runtime.runtime import _join_started_thread, controller_class_for_profile


def test_simulation_ros_profile_exposes_mavros_connection_settings() -> None:
    """验证仿真 profile 暴露 MAVROS namespace 和 FCU URL。"""
    config = RosConfig(
        node_name="drone_harness_sim",
        camera_scene_topic="/camera",
        mavros_namespace="/mavros",
        mavros_fcu_url="udp://:14540@127.0.0.1:14580",
    )

    assert config.mavros_namespace == "/mavros"
    assert config.mavros_fcu_url.endswith("14540@127.0.0.1:14580")


def test_simulation_runtime_selects_original_px4_controller_name() -> None:
    """验证 simulation profile 仍使用 Px4Controller 原类名。"""
    controller_class = controller_class_for_profile("simulation")

    assert controller_class.__name__ == "Px4Controller"


def test_runtime_cleanup_ignores_thread_that_was_never_started() -> None:
    """LLM 初始化提前失败时，清理流程不应 join 尚未启动的线程。"""
    import threading

    executor_thread = threading.Thread(target=lambda: None)

    _join_started_thread(executor_thread)
