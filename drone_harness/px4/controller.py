"""封装 MAVROS 发布、订阅、命令确认和位置控制。"""

from __future__ import annotations

import math
import threading
import time
from collections import deque
from dataclasses import dataclass, replace
from typing import Any

from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import ExtendedState, PositionTarget, State, StatusText
from mavros_msgs.srv import CommandBool, CommandLong, SetMode
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState as MavrosBatteryState
from sensor_msgs.msg import CameraInfo, Image

from drone_harness.config.schema import ObservationConfig
from drone_harness.px4.frame import body_to_ned, is_finite_number, normalize_angle
from drone_harness.runtime.observation import CameraIntrinsics, ObservationBuffer, ObservationSnapshot, ros_stamp_ns


DEFAULT_POSITION_TOLERANCE_M = 0.3
DEFAULT_YAW_TOLERANCE_RAD = math.radians(5.0)
DEFAULT_TIMER_PERIOD_S = 0.1
OFFBOARD_HANDSHAKE_TIMEOUT_S = 8.0
SETPOINT_WARMUP_TIMEOUT_S = 3.0
MAV_RESULT_NAMES = {
    0: "ACCEPTED",
    1: "TEMPORARILY_REJECTED",
    2: "DENIED",
    3: "UNSUPPORTED",
    4: "FAILED",
    5: "IN_PROGRESS",
    6: "CANCELLED",
}
MODE_TO_NAV_STATE = {
    "POSCTL": 2,
    "AUTO.LOITER": 4,
    "AUTO.RTL": 5,
    "AUTO.LAND": 6,
    "OFFBOARD": 14,
}


@dataclass(frozen=True)
class CommandRequest:
    """记录 MAVROS 异步 service 请求和兼容旧 ACK 序号的句柄。"""

    operation: str = ""
    future: Any | None = None
    command: int | str | None = None
    ack_sequence_before: int = 0


@dataclass(frozen=True)
class CommandAck:
    """统一表示 MAVROS command service 返回结果。"""

    success: bool
    result: int


@dataclass
class LocalPosition:
    """保存内部 NED 坐标和航向。"""

    x: float = float("nan")
    y: float = float("nan")
    z: float = float("nan")
    heading: float = float("nan")
    xy_valid: bool = False
    z_valid: bool = False


@dataclass
class VehicleStatus:
    """把 MAVROS 状态归一化为原飞行工具使用的 PX4 字段。"""

    mode: str = ""
    armed: bool = False
    connected: bool = False
    nav_state: int = 0
    arming_state: int = 1

    NAVIGATION_STATE_AUTO_LOITER = 4
    NAVIGATION_STATE_AUTO_RTL = 5
    NAVIGATION_STATE_AUTO_LAND = 6
    NAVIGATION_STATE_OFFBOARD = 14
    NAVIGATION_STATE_POSCTL = 2
    ARMING_STATE_DISARMED = 1
    ARMING_STATE_ARMED = 2


@dataclass
class BatteryStatus:
    """把 MAVROS 电池状态归一化为原工具字段。"""

    connected: bool = False
    voltage_v: float = float("nan")
    current_a: float = float("nan")
    remaining: float = -1.0
    warning: int = 0


class Px4Controller(Node):
    """通过 MAVROS 为仿真提供原有 PX4 控制器接口。"""

    def __init__(
        self,
        node_name: str,
        camera_scene_topic: str | None = None,
        mavros_namespace: str = "/mavros",
        camera_depth_topic: str | None = None,
        camera_depth_info_topic: str | None = None,
        observation_config: ObservationConfig | None = None,
    ) -> None:
        """初始化 MAVROS topic、service、状态缓存和 setpoint 定时器。"""
        super().__init__(node_name)
        self.mavros_namespace = mavros_namespace.rstrip("/") or "/mavros"
        self.position_tolerance = DEFAULT_POSITION_TOLERANCE_M
        self.yaw_tolerance = DEFAULT_YAW_TOLERANCE_RAD
        self.timer_period = DEFAULT_TIMER_PERIOD_S

        self.vehicle_local_position = LocalPosition()
        self.vehicle_status = VehicleStatus()
        self.battery_status = BatteryStatus()
        self.extended_state = ExtendedState()
        self.vehicle_status_received = False
        self.battery_status_received = False
        self.pose_received = False
        self.pose_received_monotonic_ns: int | None = None
        self.extended_state_received = False
        self.ground_z_ned: float | None = None
        self.vehicle_command_ack = None
        self.vehicle_command_ack_sequence = 0
        self.vehicle_command_ack_history: deque[tuple[int, Any]] = deque(maxlen=64)
        self._vehicle_command_ack_lock = threading.Lock()
        self.bridge = CvBridge()
        self.latest_rgb_frame = None
        self.observation_buffer = ObservationBuffer(observation_config) if observation_config is not None else None
        self.latest_statustext = None

        self.setpoint_publisher = self.create_publisher(
            PositionTarget,
            self._topic("/setpoint_raw/local"),
            10,
        )
        self.trajectory_setpoint_publisher = self.setpoint_publisher
        self.arming_client = self.create_client(CommandBool, self._topic("/cmd/arming"))
        self.mode_client = self.create_client(SetMode, self._topic("/set_mode"))
        self.command_client = self.create_client(CommandLong, self._topic("/cmd/command"))

        self.create_subscription(State, self._topic("/state"), self.state_callback, 10)
        self.create_subscription(
            PoseStamped,
            self._topic("/local_position/pose"),
            self.pose_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            MavrosBatteryState,
            self._topic("/battery"),
            self.battery_status_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            ExtendedState,
            self._topic("/extended_state"),
            self.extended_state_callback,
            10,
        )
        self.create_subscription(
            StatusText,
            self._topic("/statustext/recv"),
            self.statustext_callback,
            10,
        )
        if camera_scene_topic:
            self.create_subscription(
                Image,
                camera_scene_topic,
                self.rgb_image_callback,
                qos_profile_sensor_data,
            )
        if camera_depth_topic:
            self.create_subscription(
                Image,
                camera_depth_topic,
                self.depth_image_callback,
                qos_profile_sensor_data,
            )
        if camera_depth_info_topic:
            self.create_subscription(
                CameraInfo,
                camera_depth_info_topic,
                self.depth_camera_info_callback,
                qos_profile_sensor_data,
            )

        self.target_position = None
        self.target_yaw = None
        self.target_yawspeed = None
        self.setpoint_counter = 0
        self.offboard_command_sent = False
        self.arm_command_sent = False
        self.last_offboard_request_time = 0.0
        self.last_arm_request_time = 0.0
        self.offboard_confirmed = False
        self.arming_confirmed = False
        self.position_hold_start_error = None
        self.position_hold_start_detail = None
        self.pending_arm_request = None
        self.pending_offboard_request = None
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def _topic(self, suffix: str) -> str:
        """拼接当前 MAVROS namespace 下的 topic 名称。"""
        return f"{self.mavros_namespace}{suffix}"

    @staticmethod
    def ned_to_enu(position: list[float] | tuple[float, float, float]) -> list[float]:
        """把内部 NED 位置转换为 MAVROS 使用的 ENU 位置。"""
        return [float(position[1]), float(position[0]), -float(position[2])]

    @staticmethod
    def position_target_type_mask(yaw: float | None, yawspeed: float | None) -> int:
        """生成只保留位置字段并按需忽略 yaw 的掩码。"""
        mask = 8 + 16 + 32 + 64 + 128 + 256
        if yaw is None:
            mask |= 1024
        if yawspeed is None:
            mask |= 2048
        return mask

    @staticmethod
    def command_definition(operation: str) -> tuple[int, tuple[float, ...]]:
        """返回飞行操作对应的 MAVLink command 和参数。"""
        definitions = {
            "offboard": (176, (1.0, 6.0)),
            "hover": (176, (1.0, 4.0, 3.0)),
            "return_home": (20, ()),
            "land": (21, ()),
        }
        try:
            return definitions[operation]
        except KeyError as exc:
            raise ValueError(f"unsupported MAVROS command operation: {operation}") from exc

    def state_callback(self, msg: State) -> None:
        """缓存 MAVROS 连接、模式和解锁状态。"""
        mode = str(msg.mode)
        self.vehicle_status.mode = mode
        self.vehicle_status.armed = bool(msg.armed)
        self.vehicle_status.connected = bool(getattr(msg, "connected", False))
        self.vehicle_status.nav_state = MODE_TO_NAV_STATE.get(mode, 0)
        self.vehicle_status.arming_state = (
            VehicleStatus.ARMING_STATE_ARMED if msg.armed else VehicleStatus.ARMING_STATE_DISARMED
        )
        self.vehicle_status_received = True
        self._capture_ground_z_if_ready()

    def pose_callback(self, msg: PoseStamped) -> None:
        """把 MAVROS ENU 位姿转换并缓存为内部 NED 位姿。"""
        position = msg.pose.position
        orientation = msg.pose.orientation
        yaw_enu = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        self.vehicle_local_position = LocalPosition(
            x=float(position.y),
            y=float(position.x),
            z=-float(position.z),
            heading=normalize_angle(math.pi / 2.0 - yaw_enu),
            xy_valid=True,
            z_valid=True,
        )
        self.pose_received = True
        self.pose_received_monotonic_ns = time.monotonic_ns()
        self._capture_ground_z_if_ready()

    def battery_status_callback(self, msg: MavrosBatteryState) -> None:
        """把 MAVROS 电池消息转换为现有工具使用的电池结构。"""
        percentage = float(msg.percentage)
        remaining = percentage if math.isfinite(percentage) and 0.0 <= percentage <= 1.0 else -1.0
        self.battery_status = BatteryStatus(
            connected=True,
            voltage_v=float(msg.voltage),
            current_a=float(msg.current),
            remaining=remaining,
            warning=0,
        )
        self.battery_status_received = True

    def extended_state_callback(self, msg: ExtendedState) -> None:
        """缓存 MAVROS 起飞、在空中和降落状态。"""
        self.extended_state = msg
        self.extended_state_received = True
        self._capture_ground_z_if_ready()

    def statustext_callback(self, msg: StatusText) -> None:
        """缓存 PX4 最近一条状态文本，便于解释解锁/模式拒绝。"""
        self.latest_statustext = str(getattr(msg, "text", "")).strip() or None

    def flight_state(self) -> str | None:
        """根据 MAVROS landed state 返回 ON_GROUND、IN_AIR 或未知。"""
        if not getattr(self, "extended_state_received", False):
            return None
        landed_state = int(getattr(self.extended_state, "landed_state", 0))
        if landed_state == int(getattr(ExtendedState, "LANDED_STATE_ON_GROUND", 1)):
            return "ON_GROUND"
        if landed_state in {
            int(getattr(ExtendedState, "LANDED_STATE_IN_AIR", 2)),
            int(getattr(ExtendedState, "LANDED_STATE_TAKEOFF", 3)),
            int(getattr(ExtendedState, "LANDED_STATE_LANDING", 4)),
        }:
            return "IN_AIR"
        return None

    def _capture_ground_z_if_ready(self) -> None:
        """在明确落地且未解锁时更新地面 NED 高度。"""
        if (
            self.flight_state() == "ON_GROUND"
            and not bool(getattr(getattr(self, "vehicle_status", None), "armed", False))
            and bool(getattr(self, "pose_received", False))
            and math.isfinite(getattr(getattr(self, "vehicle_local_position", None), "z", float("nan")))
        ):
            self.ground_z_ned = self.vehicle_local_position.z

    def height_above_ground_m(self, z_ned: float | None = None) -> float | None:
        """返回当前或指定 NED 位置相对已记录地面的高度。"""
        ground_z_ned = self.ground_z_ned
        if ground_z_ned is None or not math.isfinite(ground_z_ned):
            return None
        try:
            target_z = self.vehicle_local_position.z if z_ned is None else float(z_ned)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(target_z):
            return None
        return ground_z_ned - target_z

    def rgb_image_callback(self, msg: Image) -> None:
        """把 RGB 图像消息转换并连同采集时间缓存。"""
        try:
            self.latest_rgb_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            if self.observation_buffer is not None:
                self.observation_buffer.add_rgb(
                    self.latest_rgb_frame,
                    ros_stamp_ns(msg.header.stamp),
                    str(msg.header.frame_id),
                )
        except Exception as exc:
            self.get_logger().error(f"Failed to convert RGB image: {exc}")

    def depth_image_callback(self, msg: Image) -> None:
        """只解码并缓存原始深度图，不在 ROS 回调里做几何计算。"""
        if self.observation_buffer is None:
            return
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, "passthrough")
            self.observation_buffer.add_depth(
                depth,
                ros_stamp_ns(msg.header.stamp),
                str(msg.encoding),
                str(msg.header.frame_id),
            )
        except Exception as exc:
            self.get_logger().error(f"Failed to convert depth image: {exc}")

    def depth_camera_info_callback(self, msg: CameraInfo) -> None:
        """校验并缓存与深度图同源的针孔内参。"""
        if self.observation_buffer is None:
            return
        try:
            self.observation_buffer.add_camera_info(CameraIntrinsics.from_ros_message(msg))
        except Exception as exc:
            self.get_logger().error(f"Failed to read depth camera info: {exc}")

    def latest_observation(self, after_stamp_ns: int = 0) -> ObservationSnapshot | None:
        """读取最新观测并附上位姿及状态，供规划与安全门复用。"""
        if self.observation_buffer is None:
            return None
        snapshot = self.observation_buffer.latest_snapshot(after_stamp_ns)
        return self._attach_flight_state(snapshot)

    def wait_for_observation(
        self,
        after_stamp_ns: int = 0,
        timeout_s: float | None = None,
    ) -> ObservationSnapshot | None:
        """等待动作后采集的新观测，保持 setpoint 定时器独立运行。"""
        if self.observation_buffer is None:
            return None
        snapshot = self.observation_buffer.wait_for_snapshot(after_stamp_ns, timeout_s)
        return self._attach_flight_state(snapshot)

    def _attach_flight_state(self, snapshot: ObservationSnapshot | None) -> ObservationSnapshot | None:
        """为快照补充当前 NED 位姿、飞行状态与位姿龄。"""
        if snapshot is None:
            return None
        pose = self.vehicle_local_position
        pose_ned = None
        if self.pose_received and all(math.isfinite(value) for value in (pose.x, pose.y, pose.z)):
            pose_ned = (pose.x, pose.y, pose.z)
        received_ns = self.pose_received_monotonic_ns
        pose_age_s = None if received_ns is None else max(0.0, (time.monotonic_ns() - received_ns) / 1e9)
        return replace(
            snapshot,
            pose_ned=pose_ned,
            flight_state=self.flight_state(),
            pose_age_s=pose_age_s,
        )

    def nav_state_constant(self, name: str, fallback: int) -> int:
        """返回与现有 PX4 工具兼容的导航状态常量。"""
        return int(getattr(VehicleStatus, name, fallback))

    def timestamp_us(self) -> int:
        """返回当前 ROS 时间的微秒时间戳。"""
        return int(self.get_clock().now().nanoseconds / 1000)

    def _request(self, client: Any, request: Any, operation: str, command: int | str | None = None) -> CommandRequest:
        """异步发送 MAVROS service 请求，避免阻塞 ROS executor。"""
        if not client.service_is_ready():
            return CommandRequest(operation=operation, command=command)
        return CommandRequest(
            operation=operation,
            future=client.call_async(request),
            command=command,
        )

    @staticmethod
    def _ack_from_response(response: Any) -> CommandAck:
        """把 MAVROS service 响应归一化为统一 ACK 结构。"""
        if response is None:
            return CommandAck(success=False, result=4)
        # CommandBool/CommandLong 使用 success；SetMode 使用 mode_sent。
        accepted = bool(getattr(response, "success", getattr(response, "mode_sent", False)))
        result = int(getattr(response, "result", 0 if accepted else 2))
        return CommandAck(success=accepted, result=result)

    def _build_command_request(self, command: int) -> CommandRequest:
        """创建兼容旧 ACK 测试的命令请求句柄。"""
        return CommandRequest(
            operation=str(command),
            command=int(command),
            ack_sequence_before=int(getattr(self, "vehicle_command_ack_sequence", 0)),
        )

    def get_command_ack(self, request: CommandRequest) -> Any | None:
        """读取请求发送后且命令匹配的历史 ACK。"""
        history = tuple(getattr(self, "vehicle_command_ack_history", ()))
        for sequence, ack in reversed(history):
            if sequence > request.ack_sequence_before and getattr(ack, "command", None) == request.command:
                return ack
        latest_sequence = int(getattr(self, "vehicle_command_ack_sequence", 0))
        latest_ack = getattr(self, "vehicle_command_ack", None)
        if (
            latest_sequence > request.ack_sequence_before
            and latest_ack is not None
            and getattr(latest_ack, "command", None) == request.command
        ):
            return latest_ack
        return None

    def wait_for_command_ack(self, request: CommandRequest, timeout_s: float) -> CommandAck | Any | None:
        """等待 MAVROS service 响应或兼容旧历史 ACK。"""
        if request.future is None:
            if request.command is not None:
                return self.get_command_ack(request)
            return CommandAck(success=False, result=4)
        deadline = time.monotonic() + max(0.0, timeout_s)
        while not request.future.done() and time.monotonic() < deadline:
            time.sleep(self.timer_period)
        if not request.future.done():
            return None
        try:
            return self._ack_from_response(request.future.result())
        except Exception:
            return CommandAck(success=False, result=4)

    def is_command_ack_accepted(self, ack: Any) -> bool:
        """判断 ACK 是否接受，兼容旧 PX4 ACK 和 MAVROS ACK。"""
        result = int(getattr(ack, "result", -1))
        return result == 0 and bool(getattr(ack, "success", True))

    def command_ack_result_name(self, ack: Any) -> str:
        """把 MAVLink result 数值转换为稳定名称。"""
        result = int(getattr(ack, "result", -1))
        return MAV_RESULT_NAMES.get(result, f"UNKNOWN_{result}")

    def wait_for_nav_state(self, expected_nav_state: int, timeout_s: float) -> bool:
        """等待 MAVROS mode 映射到目标 PX4 导航状态。"""
        deadline = time.monotonic() + max(0.0, timeout_s)
        while time.monotonic() < deadline:
            if self.vehicle_status.nav_state == expected_nav_state:
                return True
            time.sleep(self.timer_period)
        return self.vehicle_status.nav_state == expected_nav_state

    def wait_for_arming_state(self, expected_arming_state: int, timeout_s: float) -> bool:
        """等待 MAVROS armed 状态映射到目标 PX4 解锁状态。"""
        deadline = time.monotonic() + max(0.0, timeout_s)
        while time.monotonic() < deadline:
            if self.vehicle_status.arming_state == expected_arming_state:
                return True
            time.sleep(self.timer_period)
        return self.vehicle_status.arming_state == expected_arming_state

    def wait_for_offboard_and_arm(self, timeout_s: float) -> bool:
        """按真机控制器顺序串行执行 ARM，再切换 Offboard。"""
        if not bool(getattr(self.vehicle_status, "connected", False)):
            self.position_hold_start_error = "MAVROS_NOT_CONNECTED"
            return False

        warmup_deadline = time.monotonic() + SETPOINT_WARMUP_TIMEOUT_S
        while self.setpoint_counter < 10 and time.monotonic() < warmup_deadline:
            time.sleep(self.timer_period)

        if not self.arming_confirmed:
            request = self.send_arm_command()
            if request.future is None:
                self.position_hold_start_error = "ARMING_SERVICE_UNAVAILABLE"
                return False
            ack = self.wait_for_command_ack(request, timeout_s=timeout_s)
            if ack is not None and not self.is_command_ack_accepted(ack):
                self.position_hold_start_error = "ARMING_NOT_CONFIRMED"
                self.position_hold_start_detail = self.command_ack_result_name(ack)
                return False
            if not self.wait_for_arming_state(
                int(getattr(VehicleStatus, "ARMING_STATE_ARMED", 2)),
                timeout_s=timeout_s,
            ):
                self.position_hold_start_error = "ARMING_NOT_CONFIRMED"
                self.position_hold_start_detail = getattr(self, "latest_statustext", None)
                return False
            self.arming_confirmed = True

        if self.vehicle_status.mode != "OFFBOARD":
            request = self.send_offboard_mode_command()
            if request.future is None:
                self.position_hold_start_error = "OFFBOARD_SERVICE_UNAVAILABLE"
                return False
            ack = self.wait_for_command_ack(request, timeout_s=timeout_s)
            if ack is not None and not self.is_command_ack_accepted(ack):
                self.position_hold_start_error = "OFFBOARD_NOT_CONFIRMED"
                self.position_hold_start_detail = self.command_ack_result_name(ack)
                return False
            if not self.wait_for_nav_state(
                int(getattr(VehicleStatus, "NAVIGATION_STATE_OFFBOARD", 14)),
                timeout_s=timeout_s,
            ):
                self.position_hold_start_error = "OFFBOARD_NOT_CONFIRMED"
                self.position_hold_start_detail = getattr(self, "latest_statustext", None)
                return False

        self.offboard_confirmed = True
        self.arming_confirmed = bool(self.vehicle_status.armed)
        return self.arming_confirmed

    @staticmethod
    def _validate_setpoint(position: list[float], yaw: float | None, yawspeed: float | None) -> None:
        """拒绝任何非有限位置、yaw 或 yaw-rate。"""
        if (
            not isinstance(position, (list, tuple))
            or len(position) != 3
            or any(not is_finite_number(value) for value in position)
        ):
            raise ValueError("position must contain exactly three finite numbers")
        if yaw is not None and not is_finite_number(yaw):
            raise ValueError("yaw must be finite when specified")
        if yawspeed is not None and not is_finite_number(yawspeed):
            raise ValueError("yawspeed must be finite when specified")

    def publish_position_setpoint(
        self,
        position: list[float],
        yaw: float | None = None,
        yawspeed: float | None = None,
    ) -> None:
        """发布转换为 MAVROS ENU 的 raw local position setpoint。"""
        self._validate_setpoint(position, yaw, yawspeed)
        msg = PositionTarget()
        msg.coordinate_frame = getattr(PositionTarget, "FRAME_LOCAL_NED", 1)
        msg.type_mask = self.position_target_type_mask(yaw, yawspeed)
        enu_position = self.ned_to_enu(position)
        msg.position.x, msg.position.y, msg.position.z = enu_position
        if yaw is not None:
            msg.yaw = normalize_angle(math.pi / 2.0 - float(yaw))
        if yawspeed is not None:
            msg.yaw_rate = -float(yawspeed)
        if hasattr(msg, "header"):
            msg.header.stamp = self.get_clock().now().to_msg()
        publisher = getattr(self, "setpoint_publisher", None)
        if publisher is None:
            publisher = self.trajectory_setpoint_publisher
        publisher.publish(msg)

    def _command(self, operation: str) -> CommandRequest:
        """通过 MAVROS command service 发送带 ACK 的飞行命令。"""
        command, parameters = self.command_definition(operation)
        request = CommandLong.Request()
        request.broadcast = False
        request.confirmation = 0
        request.command = command
        for index, value in enumerate(parameters, start=1):
            setattr(request, f"param{index}", value)
        return self._request(self.command_client, request, operation, command)

    def send_offboard_mode_command(self) -> CommandRequest:
        """通过 MAVROS set_mode 服务请求切换到 Offboard 模式。"""
        request = SetMode.Request()
        request.custom_mode = "OFFBOARD"
        return self._request(self.mode_client, request, "offboard_mode")

    def send_hover_command(self) -> CommandRequest:
        """请求 PX4 切换到 AUTO.LOITER 悬停模式。"""
        return self._command("hover")

    def send_return_home_command(self) -> CommandRequest:
        """请求 PX4 执行返航命令。"""
        return self._command("return_home")

    def send_land_command(self) -> CommandRequest:
        """请求 PX4 执行降落命令。"""
        return self._command("land")

    def send_arm_command(self) -> CommandRequest:
        """请求 MAVROS 解锁飞控。"""
        request = CommandBool.Request()
        request.value = True
        return self._request(self.arming_client, request, "arm")

    def send_disarm_command(self) -> CommandRequest:
        """请求 MAVROS 上锁飞控。"""
        request = CommandBool.Request()
        request.value = False
        return self._request(self.arming_client, request, "disarm")

    def uav_is_in_air(self) -> bool:
        """仅在 MAVROS 明确报告空中状态时返回 True。"""
        return self.flight_state() == "IN_AIR"

    def uav_position_is_valid(self) -> bool:
        """判断 MAVROS 本地位姿是否包含有效有限数值。"""
        position = self.vehicle_local_position
        return (
            self.pose_received
            and position.xy_valid
            and position.z_valid
            and all(math.isfinite(value) for value in (position.x, position.y, position.z))
        )

    def is_at_target(self, position: list[float]) -> bool:
        """判断当前 NED 位置是否进入目标误差范围。"""
        current = self.vehicle_local_position
        distance = math.sqrt(sum((actual - target) ** 2 for actual, target in zip(
            (current.x, current.y, current.z), position
        )))
        return distance <= self.position_tolerance

    def is_at_yaw_target(self, yaw: float) -> bool:
        """判断当前 NED 航向是否进入目标误差范围。"""
        current_heading = self.vehicle_local_position.heading
        if not math.isfinite(current_heading):
            return False
        return abs(normalize_angle(current_heading - yaw)) <= self.yaw_tolerance

    def body_to_ned(self, forward: float, right: float, down: float, heading: float) -> tuple[float, float, float]:
        """把机体系 FRD 位移转换为 NED 位移。"""
        return body_to_ned(forward, right, down, heading)

    def normalize_angle(self, angle: float) -> float:
        """把角度归一化到 [-pi, pi]。"""
        return normalize_angle(angle)

    def start_position_hold(self, position: list[float], yaw: float | None = None, yawspeed: float | None = None) -> bool:
        """开始发布 setpoint，并等待 Offboard 与解锁状态确认。"""
        self._validate_setpoint(position, yaw, yawspeed)
        self.target_position = list(position)
        self.target_yaw = yaw
        self.target_yawspeed = yawspeed
        self.setpoint_counter = 0
        self.offboard_command_sent = False
        self.arm_command_sent = False
        self.last_offboard_request_time = 0.0
        self.last_arm_request_time = 0.0
        self.pending_arm_request = None
        self.pending_offboard_request = None
        connected = bool(getattr(self.vehicle_status, "connected", False))
        self.arming_confirmed = connected and bool(self.vehicle_status.armed)
        self.offboard_confirmed = self.arming_confirmed and self.vehicle_status.mode == "OFFBOARD"
        self.position_hold_start_error = None
        self.position_hold_start_detail = None
        if self.wait_for_offboard_and_arm(OFFBOARD_HANDSHAKE_TIMEOUT_S):
            return True
        self.stop_position_hold()
        return False

    def stop_position_hold(self) -> None:
        """停止发布位置保持 setpoint。"""
        for request in (getattr(self, "pending_arm_request", None), getattr(self, "pending_offboard_request", None)):
            future = getattr(request, "future", None)
            if future is not None and not future.done():
                cancel = getattr(future, "cancel", None)
                if cancel is not None:
                    cancel()
        self.pending_arm_request = None
        self.pending_offboard_request = None
        self.target_position = None
        self.target_yaw = None
        self.target_yawspeed = None

    def current_position_ned(self) -> list[float]:
        """返回当前内部 NED 位置。"""
        position = self.vehicle_local_position
        return [position.x, position.y, position.z]

    def timer_callback(self) -> None:
        """定时发布 setpoint；Offboard/解锁握手由同步启动流程负责。"""
        if self.target_position is None:
            return
        self.publish_position_setpoint(self.target_position, self.target_yaw, self.target_yawspeed)
        # 模式可能被遥控器拨回 POSCTL；不能保留旧的确认标志。
        self.offboard_confirmed = self.vehicle_status.mode == "OFFBOARD" and bool(self.vehicle_status.armed)
        self.arming_confirmed = bool(self.vehicle_status.armed)
        if self.setpoint_counter < 10:
            self.setpoint_counter += 1
            return
