"""负责加载和组装 runtime profile。"""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any

import yaml

from drone_harness.config.schema import (
    ObservationConfig,
    ProviderConfig,
    RosConfig,
    RuntimeProfile,
    SafetyConfig,
    StorageConfig,
)


class ConfigError(RuntimeError):
    """Raised when runtime profile configuration is invalid."""


DEFAULT_PROFILE_DIR = Path(__file__).resolve().parent / "profiles"
DEFAULT_SETTINGS_DIR = Path.home() / ".config" / "drone_harness"
DEFAULT_SETTINGS_PATH = DEFAULT_SETTINGS_DIR / "settings.json"
SETTINGS_ENV_VAR = "DRONE_HARNESS_SETTINGS"


def load_profile(
    profile_name: str,
    profile_dir: Path | None = None,
    settings_path: Path | None = None,
) -> RuntimeProfile:
    """从 YAML 和本地 settings.json 中加载指定 profile。"""
    selected_dir = profile_dir or DEFAULT_PROFILE_DIR
    profile_path = selected_dir / f"{profile_name}.yaml"
    selected_settings_path = resolve_settings_path(settings_path)
    if profile_name not in {"sim", "real"}:
        raise ConfigError(f"unknown profile: {profile_name}")
    if not profile_path.exists():
        raise ConfigError(f"profile file not found: {profile_path}")

    try:
        raw = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        raise ConfigError(f"failed to parse profile {profile_path}: {exc}") from exc

    if not isinstance(raw, dict):
        raise ConfigError(f"profile must be a mapping: {profile_path}")

    settings = _load_settings(selected_settings_path)

    try:
        return _build_profile(raw, settings)
    except (KeyError, TypeError, ValueError) as exc:
        raise ConfigError(str(exc)) from exc


def _load_settings(settings_path: Path) -> dict[str, Any]:
    """读取项目根目录的 settings.json。"""
    if not settings_path.exists():
        raise ConfigError(
            f"settings file not found: {settings_path}. "
            f"create this file or set {SETTINGS_ENV_VAR} to override the default path"
        )

    try:
        raw = json.loads(settings_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise ConfigError(f"failed to parse settings {settings_path}: {exc}") from exc

    if not isinstance(raw, dict):
        raise ConfigError(f"settings must be a mapping: {settings_path}")

    return raw


def resolve_settings_path(settings_path: Path | None = None) -> Path:
    """寻找 settings.json 的路径"""
    if settings_path is not None:
        return settings_path

    env_path = os.environ.get(SETTINGS_ENV_VAR, "").strip()
    if env_path:
        return Path(env_path).expanduser().resolve()

    return DEFAULT_SETTINGS_PATH


def _build_profile(raw: dict[str, Any], settings: dict[str, Any]) -> RuntimeProfile:
    """把原始字典转换成经过校验的 RuntimeProfile。"""
    ros = raw["ros"]
    storage = raw["storage"]
    observation = raw["observation"]
    safety = raw["safety"]
    llm_settings = settings.get("llm", {})
    legacy_model_keys = {"vlm", "detector", "tracker"} & settings.keys()
    if legacy_model_keys:
        raise ValueError(f"old multi-model settings are unsupported: {sorted(legacy_model_keys)}")

    if not isinstance(llm_settings, dict):
        raise ValueError("settings.llm must be a mapping")
    llm_api_key = str(os.environ.get("DRONE_HARNESS_LLM_API_KEY", llm_settings.get("api_key", ""))).strip()
    llm_base_url = str(os.environ.get("DRONE_HARNESS_LLM_BASE_URL", llm_settings.get("base_url", ""))).strip()
    llm_model = str(os.environ.get("DRONE_HARNESS_LLM_MODEL", llm_settings.get("model", ""))).strip()
    hitl_exempt_flight_tools = safety.get("human_in_the_loop_exempt_flight_tools", [])
    if not isinstance(hitl_exempt_flight_tools, list) or any(
        not isinstance(tool_name, str) or not tool_name.strip()
        for tool_name in hitl_exempt_flight_tools
    ):
        raise ValueError("safety.human_in_the_loop_exempt_flight_tools must be a list of non-empty strings")

    return RuntimeProfile(
        name=str(raw["name"]),
        mode=str(raw["mode"]),
        post_motion_wait_enabled=bool(raw.get("post_motion_wait_enabled", True)),
        ros=RosConfig(
            node_name=str(ros["node_name"]),
            camera_scene_topic=ros.get("camera_scene_topic"),
            camera_depth_topic=ros.get("camera_depth_topic"),
            camera_depth_info_topic=ros.get("camera_depth_info_topic"),
            mavros_namespace=str(ros.get("mavros_namespace", "/mavros")),
            mavros_fcu_url=str(ros.get("mavros_fcu_url", "")).strip(),
        ),
        storage=StorageConfig(
            photo_save_dir=str(storage["photo_save_dir"]),
            analysis_save_dir=str(storage["analysis_save_dir"]),
            log_dir=str(storage["log_dir"]),
        ),
        observation=ObservationConfig(
            max_frame_age_s=float(observation["max_frame_age_s"]),
            max_sync_delta_s=float(observation["max_sync_delta_s"]),
            max_clock_skew_s=float(observation["max_clock_skew_s"]),
            wait_timeout_s=float(observation["wait_timeout_s"]),
            depth_semantics=str(observation["depth_semantics"]),
            depth_max_m=float(observation["depth_max_m"]),
            camera_forward_offset_m=float(observation["camera_forward_offset_m"]),
            body_front_offset_m=float(observation["body_front_offset_m"]),
            body_half_width_m=float(observation["body_half_width_m"]),
            body_half_height_m=float(observation["body_half_height_m"]),
            measurement_margin_m=float(observation["measurement_margin_m"]),
            braking_margin_m=float(observation["braking_margin_m"]),
            latency_margin_m=float(observation["latency_margin_m"]),
            side_obstacle_distance_m=float(observation["side_obstacle_distance_m"]),
        ),
        llm=ProviderConfig(
            base_url=llm_base_url,
            model=llm_model,
            api_key=llm_api_key,
        ),
        safety=SafetyConfig(
            human_in_the_loop_for_flight_tools=bool(
                safety["human_in_the_loop_for_flight_tools"]
            ),
            human_in_the_loop_exempt_flight_tools=frozenset(
                tool_name.strip() for tool_name in hitl_exempt_flight_tools
            ),
            max_takeoff_height_m=float(safety["max_takeoff_height_m"]),
            max_relative_move_m=float(safety["max_relative_move_m"]),
            max_forward_m=(float(safety["max_forward_m"])
                           if safety.get("max_forward_m") is not None else None),
            max_vertical_move_m=float(safety["max_vertical_move_m"]),
            max_rotation_deg=float(safety["max_rotation_deg"]),
            action_timeout_s=float(safety["action_timeout_s"]),
            hover_on_timeout=bool(safety["hover_on_timeout"]),
            pre_takeoff_gate_enabled=bool(safety["pre_takeoff_gate_enabled"]),
            require_battery_status_for_takeoff=bool(safety["require_battery_status_for_takeoff"]),
            min_battery_percent_for_takeoff=float(safety["min_battery_percent_for_takeoff"]),
            require_px4_status_ready_for_takeoff=bool(safety["require_px4_status_ready_for_takeoff"]),
        ),
    )
