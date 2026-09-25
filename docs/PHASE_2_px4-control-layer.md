# drone_agent Phase 2 PX4 控制层迁移 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将 `/download/takeoff.py` 中的 PX4 DDS 控制层迁移到 `drone_agent/px4/`，形成直接、清晰、可被后续 tools 调用的 `Px4Controller`。

**Architecture:** Phase 2 不为了测试引入依赖注入抽象。`px4/controller.py` 直接定义 `class Px4Controller(Node)`，直接依赖 ROS2 `rclpy`、`px4_msgs`、`sensor_msgs` 和 `cv_bridge`；纯逻辑拆到 `frame.py`、`status.py`、`topics.py`。后续 `tools/flight.py`、`tools/status.py` 通过参数接收 runtime 创建的同一个 `Px4Controller` 实例并调用其基础方法。

**Tech Stack:** Python 3.10+、ROS2 `rclpy`、`px4_msgs`、`sensor_msgs`、`cv_bridge`、OpenCV、`pytest`。

---

## Scope

本计划只做 PX4 控制层：

- 新建 `drone_agent/px4/` 包。
- 新建 `topics.py` 集中 PX4 topic 常量。
- 新建 `frame.py` 保存 body-FRD 到 world-NED 转换和角度归一化。
- 新建 `status.py` 保存 PX4 enum 名称解析和状态 dict 构造。
- 新建 `controller.py`，从 `/download/takeoff.py` 迁移 `Px4Controller` 的 DDS publisher/subscriber、QoS、状态缓存、vehicle command、position hold、基础状态判断。

不做：

- 不迁移 `tools/flight.py`。
- 不迁移 agent loop。
- 不修改 `/download/takeoff.py`。
- 不为无 ROS 环境设计复杂依赖注入层。

## Target File Map

Create: `/download/drone_agent/drone_agent/px4/__init__.py`

PX4 控制层包标识。

Create: `/download/drone_agent/drone_agent/px4/topics.py`

集中保存 `/fmu/in/...` 和 `/fmu/out/...` topic 名称。

Create: `/download/drone_agent/drone_agent/px4/frame.py`

保存 `body_to_ned()` 和 `normalize_angle()`。

Create: `/download/drone_agent/drone_agent/px4/status.py`

保存 `enum_name_from_prefix()` 和 `flight_mode_status_dict()`。

Create: `/download/drone_agent/drone_agent/px4/controller.py`

直接定义 `Px4Controller(Node)`。该文件是 ROS2/PX4 环境文件，允许在普通环境中无法 import；这符合项目实际运行目标。

Create: `/download/drone_agent/tests/unit/test_px4_frame.py`

测试纯坐标逻辑。

Create: `/download/drone_agent/tests/unit/test_px4_status.py`

测试纯状态解析逻辑。

Create: `/download/drone_agent/tests/unit/test_px4_controller_source.py`

不 import ROS2，直接检查 `controller.py` 源码中关键控制方法和 topic 使用，避免为了普通 Python 测试引入 controller 依赖注入抽象。

## Task 1: PX4 Package, Topics, and Frame Helpers

**Files:**
- Create: `/download/drone_agent/drone_agent/px4/__init__.py`
- Create: `/download/drone_agent/drone_agent/px4/topics.py`
- Create: `/download/drone_agent/drone_agent/px4/frame.py`
- Create: `/download/drone_agent/tests/unit/test_px4_frame.py`

- [ ] **Step 1: Write frame tests**

Write `/download/drone_agent/tests/unit/test_px4_frame.py`:

```python
import math

import pytest

from drone_agent.px4.frame import body_to_ned, normalize_angle


def test_body_to_ned_with_zero_heading_keeps_forward_right_down():
    assert body_to_ned(1.0, 2.0, 3.0, 0.0) == pytest.approx((1.0, 2.0, 3.0))


def test_body_to_ned_with_ninety_degree_heading_rotates_body_axes():
    x_ned, y_ned, z_ned = body_to_ned(1.0, 0.0, -0.5, math.pi / 2.0)

    assert x_ned == pytest.approx(0.0, abs=1e-9)
    assert y_ned == pytest.approx(1.0)
    assert z_ned == pytest.approx(-0.5)


def test_body_to_ned_matches_takeoff_py_formula_for_right_offset():
    heading = math.radians(30.0)

    x_ned, y_ned, z_ned = body_to_ned(0.0, 2.0, 1.0, heading)

    assert x_ned == pytest.approx(-2.0 * math.sin(heading))
    assert y_ned == pytest.approx(2.0 * math.cos(heading))
    assert z_ned == pytest.approx(1.0)


def test_normalize_angle_wraps_to_minus_pi_pi():
    assert normalize_angle(3.0 * math.pi) == pytest.approx(math.pi)
    assert normalize_angle(-3.0 * math.pi) == pytest.approx(-math.pi)
    assert normalize_angle(0.25) == pytest.approx(0.25)
```

- [ ] **Step 2: Implement package, topics, and frame helpers**

Write `/download/drone_agent/drone_agent/px4/__init__.py`:

```python
"""PX4 DDS control layer for drone_agent."""
```

Write `/download/drone_agent/drone_agent/px4/topics.py`:

```python
OFFBOARD_CONTROL_MODE_TOPIC = "/fmu/in/offboard_control_mode"
TRAJECTORY_SETPOINT_TOPIC = "/fmu/in/trajectory_setpoint"
VEHICLE_COMMAND_TOPIC = "/fmu/in/vehicle_command"

VEHICLE_LOCAL_POSITION_TOPIC = "/fmu/out/vehicle_local_position"
VEHICLE_STATUS_TOPIC = "/fmu/out/vehicle_status"
BATTERY_STATUS_TOPIC = "/fmu/out/battery_status"
```

Write `/download/drone_agent/drone_agent/px4/frame.py`:

```python
from __future__ import annotations

import math


def body_to_ned(
    forward: float,
    right: float,
    down: float,
    heading: float,
) -> tuple[float, float, float]:
    x_ned = forward * math.cos(heading) - right * math.sin(heading)
    y_ned = forward * math.sin(heading) + right * math.cos(heading)
    return x_ned, y_ned, down


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))
```

- [ ] **Step 3: Verify frame tests**

Run:

```bash
cd /download/drone_agent && pytest tests/unit/test_px4_frame.py -v
```

Expected: all frame tests pass.

- [ ] **Step 4: Commit Task 1**

Run:

```bash
cd /download
git add drone_agent/drone_agent/px4/__init__.py \
  drone_agent/drone_agent/px4/topics.py \
  drone_agent/drone_agent/px4/frame.py \
  drone_agent/tests/unit/test_px4_frame.py
git commit -m "feat: add px4 topic and frame helpers"
```

## Task 2: PX4 Status Helpers

**Files:**
- Create: `/download/drone_agent/drone_agent/px4/status.py`
- Create: `/download/drone_agent/tests/unit/test_px4_status.py`

- [ ] **Step 1: Write status tests**

Write `/download/drone_agent/tests/unit/test_px4_status.py`:

```python
import math
from types import SimpleNamespace

from drone_agent.px4.status import enum_name_from_prefix, flight_mode_status_dict


class FakeVehicleStatusEnum:
    NAVIGATION_STATE_MANUAL = 0
    NAVIGATION_STATE_OFFBOARD = 14
    ARMING_STATE_DISARMED = 1
    ARMING_STATE_ARMED = 2


def test_enum_name_from_prefix_returns_matching_constant_name():
    assert (
        enum_name_from_prefix(
            FakeVehicleStatusEnum,
            "NAVIGATION_STATE_",
            14,
        )
        == "NAVIGATION_STATE_OFFBOARD"
    )


def test_enum_name_from_prefix_returns_unknown_with_value():
    assert (
        enum_name_from_prefix(
            FakeVehicleStatusEnum,
            "NAVIGATION_STATE_",
            999,
        )
        == "UNKNOWN_NAVIGATION_STATE_999"
    )


def test_flight_mode_status_dict_uses_controller_state_methods():
    controller = SimpleNamespace(
        vehicle_local_position=SimpleNamespace(heading=math.nan),
        vehicle_status=SimpleNamespace(nav_state=14, arming_state=2),
        uav_is_in_air=lambda: True,
        uav_position_is_valid=lambda: True,
    )

    result = flight_mode_status_dict(controller, FakeVehicleStatusEnum)

    assert result == {
        "success": True,
        "nav_state_name": "NAVIGATION_STATE_OFFBOARD",
        "arming_state_name": "ARMING_STATE_ARMED",
        "in_air": True,
        "position_valid": True,
        "heading_valid": False,
    }
```

- [ ] **Step 2: Implement status helpers**

Write `/download/drone_agent/drone_agent/px4/status.py`:

```python
from __future__ import annotations

import math
from typing import Any


def enum_name_from_prefix(enum_cls: type[Any], prefix: str, value: int) -> str:
    for name in dir(enum_cls):
        if not name.startswith(prefix):
            continue
        if getattr(enum_cls, name) == value:
            return name
    return f"UNKNOWN_{prefix}{value}"


def flight_mode_status_dict(controller: Any, vehicle_status_enum: type[Any]) -> dict:
    heading = getattr(controller.vehicle_local_position, "heading", float("nan"))
    nav_state = controller.vehicle_status.nav_state
    arming_state = controller.vehicle_status.arming_state
    return {
        "success": True,
        "nav_state_name": enum_name_from_prefix(
            vehicle_status_enum,
            "NAVIGATION_STATE_",
            nav_state,
        ),
        "arming_state_name": enum_name_from_prefix(
            vehicle_status_enum,
            "ARMING_STATE_",
            arming_state,
        ),
        "in_air": controller.uav_is_in_air(),
        "position_valid": controller.uav_position_is_valid(),
        "heading_valid": math.isfinite(heading),
    }
```

- [ ] **Step 3: Verify status tests**

Run:

```bash
cd /download/drone_agent && pytest tests/unit/test_px4_status.py -v
```

Expected: all status tests pass.

- [ ] **Step 4: Commit Task 2**

Run:

```bash
cd /download
git add drone_agent/drone_agent/px4/status.py drone_agent/tests/unit/test_px4_status.py
git commit -m "feat: add px4 status helpers"
```

## Task 3: Direct Px4Controller

**Files:**
- Create: `/download/drone_agent/drone_agent/px4/controller.py`
- Create: `/download/drone_agent/tests/unit/test_px4_controller_source.py`

- [ ] **Step 1: Write source-structure tests**

Write `/download/drone_agent/tests/unit/test_px4_controller_source.py`:

```python
from pathlib import Path


CONTROLLER = Path(__file__).parents[2] / "drone_agent" / "px4" / "controller.py"


def source() -> str:
    return CONTROLLER.read_text(encoding="utf-8")


def test_controller_is_direct_ros2_node_subclass():
    text = source()

    assert "from rclpy.node import Node" in text
    assert "class Px4Controller(Node):" in text
    assert "RosDependencies" not in text
    assert "build_px4_controller_class" not in text


def test_controller_exposes_methods_needed_by_future_tools():
    text = source()

    for method in [
        "publish_offboard_heartbeat",
        "publish_position_setpoint",
        "publish_vehicle_command",
        "send_offboard_mode_command",
        "send_arm_command",
        "send_disarm_command",
        "send_land_command",
        "send_hover_command",
        "send_return_home_command",
        "uav_is_in_air",
        "uav_position_is_valid",
        "is_at_target",
        "is_at_yaw_target",
        "wait_for_nav_state",
        "start_position_hold",
        "stop_position_hold",
        "timer_callback",
        "current_position_ned",
    ]:
        assert f"def {method}" in text


def test_controller_uses_central_topics_and_px4_qos():
    text = source()

    for name in [
        "OFFBOARD_CONTROL_MODE_TOPIC",
        "TRAJECTORY_SETPOINT_TOPIC",
        "VEHICLE_COMMAND_TOPIC",
        "VEHICLE_LOCAL_POSITION_TOPIC",
        "VEHICLE_STATUS_TOPIC",
        "BATTERY_STATUS_TOPIC",
    ]:
        assert name in text

    assert "ReliabilityPolicy.BEST_EFFORT" in text
    assert "DurabilityPolicy.TRANSIENT_LOCAL" in text
    assert "HistoryPolicy.KEEP_LAST" in text
    assert "depth=1" in text
```

- [ ] **Step 2: Implement direct controller**

Write `/download/drone_agent/drone_agent/px4/controller.py`:

```python
from __future__ import annotations

import math
import time

from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from px4_msgs.msg import (
    BatteryStatus,
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)

from drone_agent.px4.frame import body_to_ned, normalize_angle
from drone_agent.px4.topics import (
    BATTERY_STATUS_TOPIC,
    OFFBOARD_CONTROL_MODE_TOPIC,
    TRAJECTORY_SETPOINT_TOPIC,
    VEHICLE_COMMAND_TOPIC,
    VEHICLE_LOCAL_POSITION_TOPIC,
    VEHICLE_STATUS_TOPIC,
)


DEFAULT_POSITION_TOLERANCE_M = 0.3
DEFAULT_YAW_TOLERANCE_RAD = math.radians(5.0)
DEFAULT_TIMER_PERIOD_S = 0.1


class Px4Controller(Node):
    """Low-level PX4 DDS controller."""

    def __init__(
        self,
        node_name: str,
        camera_scene_topic: str | None = None,
    ) -> None:
        super().__init__(node_name)

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode,
            OFFBOARD_CONTROL_MODE_TOPIC,
            qos_profile,
        )
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint,
            TRAJECTORY_SETPOINT_TOPIC,
            qos_profile,
        )
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand,
            VEHICLE_COMMAND_TOPIC,
            qos_profile,
        )

        self.create_subscription(
            VehicleLocalPosition,
            VEHICLE_LOCAL_POSITION_TOPIC,
            self.vehicle_local_position_callback,
            qos_profile,
        )
        self.create_subscription(
            VehicleStatus,
            VEHICLE_STATUS_TOPIC,
            self.vehicle_status_callback,
            qos_profile,
        )
        self.create_subscription(
            BatteryStatus,
            BATTERY_STATUS_TOPIC,
            self.battery_status_callback,
            qos_profile,
        )
        if camera_scene_topic:
            self.create_subscription(
                Image,
                camera_scene_topic,
                self.rgb_image_callback,
                10,
            )

        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.battery_status = BatteryStatus()
        self.bridge = CvBridge()
        self.latest_rgb_frame = None
        self.position_tolerance = DEFAULT_POSITION_TOLERANCE_M
        self.yaw_tolerance = DEFAULT_YAW_TOLERANCE_RAD
        self.timer_period = DEFAULT_TIMER_PERIOD_S

        self.target_position = None
        self.target_yaw = None
        self.target_yawspeed = None
        self.setpoint_counter = 0
        self.offboard_command_sent = False
        self.arm_command_sent = False

        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def vehicle_local_position_callback(self, msg: VehicleLocalPosition) -> None:
        self.vehicle_local_position = msg

    def vehicle_status_callback(self, msg: VehicleStatus) -> None:
        self.vehicle_status = msg

    def battery_status_callback(self, msg: BatteryStatus) -> None:
        self.battery_status = msg

    def rgb_image_callback(self, msg: Image) -> None:
        try:
            self.latest_rgb_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as exc:
            self.get_logger().error(f"Failed to convert RGB image: {exc}")

    def timestamp_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)

    def publish_offboard_heartbeat(self) -> None:
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self.timestamp_us()
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(
        self,
        position: list[float],
        yaw: float | None = None,
        yawspeed: float | None = None,
    ) -> None:
        msg = TrajectorySetpoint()
        msg.position = position
        msg.yaw = float("nan") if yaw is None else yaw
        msg.yawspeed = float("nan") if yawspeed is None else yawspeed
        msg.timestamp = self.timestamp_us()
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command: int, **params: float) -> None:
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.timestamp_us()
        self.vehicle_command_publisher.publish(msg)

    def body_to_ned(
        self,
        forward: float,
        right: float,
        down: float,
        heading: float,
    ) -> tuple[float, float, float]:
        return body_to_ned(forward, right, down, heading)

    def normalize_angle(self, angle: float) -> float:
        return normalize_angle(angle)

    def send_offboard_mode_command(self) -> None:
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=6.0,
        )
        self.get_logger().info("Switching to offboard mode")

    def send_arm_command(self) -> None:
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0,
        )
        self.get_logger().info("Arm command sent")

    def send_disarm_command(self) -> None:
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=0.0,
        )
        self.get_logger().info("Disarm command sent")

    def send_land_command(self) -> None:
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def send_hover_command(self) -> None:
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=4.0,
            param3=3.0,
        )
        self.get_logger().info("Switching to AUTO_LOITER hover mode")

    def send_return_home_command(self) -> None:
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)
        self.get_logger().info("Switching to return-to-launch mode")

    def uav_is_in_air(self) -> bool:
        return (
            math.isfinite(self.vehicle_local_position.z)
            and self.vehicle_local_position.z < -0.3
        )

    def uav_position_is_valid(self) -> bool:
        pos = self.vehicle_local_position
        if not math.isfinite(pos.x) or not math.isfinite(pos.y) or not math.isfinite(pos.z):
            return False
        if hasattr(pos, "xy_valid") and not pos.xy_valid:
            return False
        if hasattr(pos, "z_valid") and not pos.z_valid:
            return False
        return True

    def is_at_target(self, position: list[float]) -> bool:
        dx = self.vehicle_local_position.x - position[0]
        dy = self.vehicle_local_position.y - position[1]
        dz = self.vehicle_local_position.z - position[2]
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        return distance <= self.position_tolerance

    def is_at_yaw_target(self, yaw: float) -> bool:
        current_heading = getattr(self.vehicle_local_position, "heading", float("nan"))
        if not math.isfinite(current_heading):
            return False
        yaw_error = self.normalize_angle(current_heading - yaw)
        return abs(yaw_error) <= self.yaw_tolerance

    def wait_for_nav_state(self, expected_nav_state: int, timeout_s: float) -> bool:
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            if self.vehicle_status.nav_state == expected_nav_state:
                return True
            time.sleep(self.timer_period)
        return self.vehicle_status.nav_state == expected_nav_state

    def start_position_hold(
        self,
        position: list[float],
        yaw: float | None = None,
        yawspeed: float | None = None,
    ) -> None:
        self.target_position = position
        self.target_yaw = yaw
        self.target_yawspeed = yawspeed
        self.setpoint_counter = 0
        self.offboard_command_sent = False
        self.arm_command_sent = False

    def stop_position_hold(self) -> None:
        self.target_position = None
        self.target_yaw = None
        self.target_yawspeed = None

    def current_position_ned(self) -> list[float]:
        return [
            self.vehicle_local_position.x,
            self.vehicle_local_position.y,
            self.vehicle_local_position.z,
        ]

    def timer_callback(self) -> None:
        if self.target_position is None:
            return

        self.publish_offboard_heartbeat()
        self.publish_position_setpoint(
            self.target_position,
            self.target_yaw,
            self.target_yawspeed,
        )

        if self.setpoint_counter == 10 and not self.offboard_command_sent:
            self.send_offboard_mode_command()
            self.offboard_command_sent = True

        if self.setpoint_counter == 11 and not self.arm_command_sent:
            self.send_arm_command()
            self.arm_command_sent = True

        if self.setpoint_counter < 12:
            self.setpoint_counter += 1
```

- [ ] **Step 3: Verify source tests and syntax**

Run:

```bash
cd /download/drone_agent && pytest tests/unit/test_px4_controller_source.py -v
cd /download/drone_agent && python3 -m py_compile drone_agent/px4/controller.py
```

Expected: source tests pass and `py_compile` exits 0.

- [ ] **Step 4: Commit Task 3**

Run:

```bash
cd /download
git add drone_agent/drone_agent/px4/controller.py drone_agent/tests/unit/test_px4_controller_source.py
git commit -m "feat: add direct px4 controller"
```

## Final Verification

- [ ] **Step 1: Run PX4 tests**

Run:

```bash
cd /download/drone_agent && pytest tests/unit/test_px4_frame.py tests/unit/test_px4_status.py tests/unit/test_px4_controller_source.py -v
```

Expected: all PX4 tests pass.

- [ ] **Step 2: Run all tests**

Run:

```bash
cd /download/drone_agent && pytest -v
```

Expected: all tests pass.

- [ ] **Step 3: Verify controller syntax**

Run:

```bash
cd /download/drone_agent && python3 -m py_compile drone_agent/px4/controller.py
```

Expected: exits 0.

- [ ] **Step 4: Verify current Phase 1 CLI still works**

Run:

```bash
cd /download/drone_agent && DRONE_AGENT_LLM_API_KEY=x DRONE_AGENT_VLM_API_KEY=y drone_agent --profile sim --task "查询状态"
```

Expected output contains:

```text
drone_agent runtime prepared: profile=sim mode=simulation node=drone_agent_sim ros_started=False
```

- [ ] **Step 5: Verify no API keys are committed**

Run:

```bash
cd /download/drone_agent && grep -R "sk-" -n drone_agent tests pyproject.toml README.md
```

Expected: no output and exit code 1.

## Plan Self-Review

Spec coverage:

- `px4/topics.py` topic centralization is covered by Task 1.
- `px4/frame.py` body-FRD to world-NED conversion is covered by Task 1.
- `px4/status.py` enum/status helpers are covered by Task 2.
- Direct `Px4Controller(Node)` migration is covered by Task 3.
- Future tools calling controller methods is supported by `start_position_hold()`、`send_*()`、`uav_position_is_valid()`、`is_at_target()`、`current_position_ned()` and related methods.

Known intentional gaps:

- `Px4Controller` is not wired into runtime in Phase 2.
- `tools/flight.py` and `tools/status.py` are deferred to Phase 3.
- Camera/VLM logic remains outside `px4/` except for caching latest camera frame.
