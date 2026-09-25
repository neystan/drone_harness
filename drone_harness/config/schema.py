"""定义 profile 的结构与校验规则。"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class RosConfig:
    """ROS2 相关配置。"""

    node_name: str
    camera_scene_topic: str | None
    mavros_namespace: str = "/mavros"
    mavros_fcu_url: str = ""

    def __post_init__(self) -> None:
        """校验 ROS 配置中的必填字段。"""
        if not self.node_name:
            raise ValueError("ros.node_name is required")
        if not self.mavros_namespace:
            raise ValueError("ros.mavros_namespace is required")


@dataclass(frozen=True)
class StorageConfig:
    """图片和日志存储配置。"""

    photo_save_dir: str
    analysis_save_dir: str
    log_dir: str

    def __post_init__(self) -> None:
        """校验存储目录配置是否完整。"""
        if not self.photo_save_dir:
            raise ValueError("storage.photo_save_dir is required")
        if not self.analysis_save_dir:
            raise ValueError("storage.analysis_save_dir is required")
        if not self.log_dir:
            raise ValueError("storage.log_dir is required")


@dataclass(frozen=True)
class ProviderConfig:
    """文本模型提供方配置。"""

    base_url: str
    model: str
    api_key: str

    def __post_init__(self) -> None:
        """校验文本模型配置与密钥是否可用。"""
        if not self.base_url:
            raise ValueError("provider base_url is required")
        if not self.model:
            raise ValueError("provider model is required")
        if not self.api_key:
            raise ValueError("provider api_key is required")


@dataclass(frozen=True)
class VlmConfig:
    """视觉模型提供方配置。"""

    enabled: bool
    base_url: str | None
    model: str | None
    api_key: str | None

    def __post_init__(self) -> None:
        """在启用视觉模型时校验相关字段。"""
        if not self.enabled:
            return
        if not self.base_url:
            raise ValueError("vlm.base_url is required when vlm.enabled=true")
        if not self.model:
            raise ValueError("vlm.model is required when vlm.enabled=true")
        if not self.api_key:
            raise ValueError("vlm.api_key is required when vlm.enabled=true")


@dataclass(frozen=True)
class DetectorConfig:
    """语义检测模型配置。"""

    enabled: bool
    provider: str | None
    api_key: str | None
    model: str | None
    api_path: str | None

    def __post_init__(self) -> None:
        """在启用检测器时校验 DINO-XSEEK 配置。"""
        if not self.enabled:
            return
        if self.provider != "dinoxseek":
            raise ValueError(
                "detector.provider must be 'dinoxseek' when detector.enabled=true"
            )
        if not self.api_key:
            raise ValueError("detector.api_key is required when detector.enabled=true")
        if not self.model:
            raise ValueError("detector.model is required when detector.enabled=true")
        if not self.api_path:
            raise ValueError("detector.api_path is required when detector.enabled=true")


@dataclass(frozen=True)
class TrackerConfig:
    """SAM2 目标跟踪服务配置。"""

    enabled: bool
    base_url: str | None
    timeout_s: float

    def __post_init__(self) -> None:
        """在启用追踪服务时校验 Docker 服务地址。"""
        if not self.enabled:
            return
        if not self.base_url:
            raise ValueError("tracker.base_url is required when tracker.enabled=true")
        if self.timeout_s <= 0:
            raise ValueError("tracker.timeout_s must be positive")


@dataclass(frozen=True)
class SafetyConfig:
    """飞行安全限制配置。"""

    human_in_the_loop_for_flight_tools: bool
    human_in_the_loop_exempt_flight_tools: frozenset[str]
    max_takeoff_height_m: float
    max_relative_move_m: float
    max_vertical_move_m: float
    max_rotation_deg: float
    action_timeout_s: float
    hover_on_timeout: bool
    pre_takeoff_gate_enabled: bool
    require_battery_status_for_takeoff: bool
    min_battery_percent_for_takeoff: float
    require_px4_status_ready_for_takeoff: bool

    def __post_init__(self) -> None:
        """校验安全阈值是否为正数。"""
        if self.max_takeoff_height_m <= 0:
            raise ValueError("max_takeoff_height_m must be positive")
        if self.max_relative_move_m <= 0:
            raise ValueError("max_relative_move_m must be positive")
        if self.max_vertical_move_m <= 0:
            raise ValueError("max_vertical_move_m must be positive")
        if self.max_rotation_deg <= 0:
            raise ValueError("max_rotation_deg must be positive")
        if self.action_timeout_s <= 0:
            raise ValueError("action_timeout_s must be positive")
        if not 0.0 <= self.min_battery_percent_for_takeoff <= 100.0:
            raise ValueError("min_battery_percent_for_takeoff must be within [0, 100]")


@dataclass(frozen=True)
class RuntimeProfile:
    """完整的运行时 profile。"""

    name: str
    mode: str
    ros: RosConfig
    storage: StorageConfig
    llm: ProviderConfig
    vlm: VlmConfig
    detector: DetectorConfig
    tracker: TrackerConfig
    safety: SafetyConfig

    def __post_init__(self) -> None:
        """校验 profile 名称和运行模式是否合法。"""
        if self.name not in {"sim", "real"}:
            raise ValueError("profile name must be 'sim' or 'real'")
        if self.mode not in {"simulation", "real"}:
            raise ValueError("profile mode must be 'simulation' or 'real'")
