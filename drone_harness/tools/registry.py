"""向模型暴露按需观察与单目标飞行动作。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable

from drone_harness.bus import MessageBus
from drone_harness.config.schema import RuntimeProfile
from drone_harness.runtime.navigation import NavigationPlan, update_navigation_plan
from drone_harness.runtime.observation import ObservationSnapshot, observation_request_time_ns
from drone_harness.runtime.task_state import TaskState
from drone_harness.runtime.reflection import clear_target_assessment
from drone_harness.vision.depth_rules import DepthRules, compute_depth_rules, invalid_depth_rules
from drone_harness.vision.target_distance import measure_target_distance
from . import flight
from .state import get_state
from .schemas import (
    NAVIGATION_PLAN_TOOL_SCHEMA,
    DOWN_TOOL_SCHEMA,
    FORWARD_TOOL_SCHEMA,
    GET_STATE_TOOL_SCHEMA,
    LAND_TOOL_SCHEMA,
    OBSERVE_TOOL_SCHEMA,
    MEASURE_DISTANCE_TOOL_SCHEMA,
    ROTATE_TOOL_SCHEMA,
    TAKEOFF_TOOL_SCHEMA,
    UP_TOOL_SCHEMA,
    ACTIVATE_SKILL_TOOL_SCHEMA,
    get_tool_schemas,
)

ToolHandler = Callable[["ToolContext", dict[str, Any]], dict[str, Any]]


@dataclass
class ToolContext:
    """汇集一次工具执行所需的控制器、配置和会话状态。"""

    controller: Any
    profile: RuntimeProfile
    session_id: str = "adhoc"
    task_state: TaskState | None = None
    message_bus: MessageBus | None = None
    observation: ObservationSnapshot | None = None
    depth_rules: DepthRules | None = None
    navigation_enabled: bool = False
    navigation_plan: NavigationPlan | None = None
    navigation_instruction: str = ""
    navigation_resumable: bool = True
    search_observations: list[ObservationSnapshot] | None = None
    target_assessment: dict[str, str] | None = None
    target_assessment_goal: tuple[str, ...] | None = None


@dataclass(frozen=True)
class ToolDefinition:
    """记录模型可见工具的 schema 与处理函数。"""

    name: str
    schema: dict[str, Any]
    handler: ToolHandler


def _takeoff_handler(context: ToolContext, arguments: dict[str, Any]) -> dict:
    """沿用现有起飞安全实现。"""
    return flight.takeoff(context, arguments.get("height"))


def _forward_handler(context: ToolContext, arguments: dict[str, Any]) -> dict:
    """仅通过程序深度安全门调用正向底层 move。"""
    return flight.forward(context, arguments.get("distance_m"))


def _up_handler(context: ToolContext, arguments: dict[str, Any]) -> dict:
    """沿用底层 move 执行纯上升。"""
    return flight.up(context, arguments.get("distance_m"))


def _down_handler(context: ToolContext, arguments: dict[str, Any]) -> dict:
    """沿用底层 move 执行纯下降。"""
    return flight.down(context, arguments.get("distance_m"))


def _rotate_handler(context: ToolContext, arguments: dict[str, Any]) -> dict:
    """沿用现有定点转向实现。"""
    return flight.rotate(context, arguments.get("direction"), arguments.get("degrees"))


def _land_handler(context: ToolContext, _arguments: dict[str, Any]) -> dict:
    """沿用现有降落实现。"""
    return flight.land(context)


def _get_state_handler(context: ToolContext, _arguments: dict[str, Any]) -> dict:
    """只读已有飞控缓存，不发送动作或采集图像。"""
    return get_state(context.controller)


def _observe_handler(context: ToolContext, arguments: dict[str, Any]) -> dict:
    """按视角选择取新图组，前视附配对深度，下视只需 RGB。"""
    context.observation = None
    context.depth_rules = None
    clear_target_assessment(context)
    if context.task_state is not None:
        context.task_state.clear_observation()
    view = arguments.get("view", "front")
    if not isinstance(view, str) or view not in {"front", "down", "both"}:
        return {"success": False, "error": "INVALID_OBSERVE_VIEW",
                "message": "view 必须为 front、down 或 both"}
    prompt = arguments.get("prompt")
    if not isinstance(prompt, str) or not 1 <= len(prompt.strip()) <= 1000:
        return {"success": False, "error": "INVALID_OBSERVE_PROMPT",
                "message": "observe.prompt 必须是去掉首尾空白后 1 到 1000 字符的字符串"}
    wait_for_observation = getattr(context.controller, "wait_for_observation", None)
    if wait_for_observation is None:
        return {"success": False, "error": "OBSERVATION_UNAVAILABLE",
                "message": "当前控制器没有可用的 RGB-D 观测接口"}
    try:
        started_ns = observation_request_time_ns(context.controller)
        snapshot = wait_for_observation(after_stamp_ns=started_ns, view=view)
    except Exception as exc:
        return {"success": False, "error": "OBSERVATION_FAILED",
                "message": f"获取 RGB-D 失败：{type(exc).__name__}"}
    threshold_ns = started_ns + int(context.profile.observation.max_clock_skew_s * 1e9)
    try:
        rgb_ready = (
            snapshot is not None
            and snapshot.view == view
            and (view != "both" or (snapshot.down_rgb is not None
                 and snapshot.down_rgb.stamp_ns > threshold_ns
                 and abs(snapshot.down_rgb.stamp_ns - snapshot.rgb_stamp_ns)
                     <= context.profile.observation.max_sync_delta_s * 1e9))
            and isinstance(snapshot.observation_id, str)
            and bool(snapshot.observation_id)
            and isinstance(snapshot.rgb_stamp_ns, int)
            and snapshot.rgb_stamp_ns > threshold_ns
            and snapshot.rgb is not None
            and snapshot.rgb.size > 0
        )
    except (AttributeError, TypeError):
        rgb_ready = False
    if not rgb_ready:
        return {"success": False, "error": "OBSERVATION_UNAVAILABLE",
                "message": f"未取得 {view} 请求视角的新鲜且时间匹配图像"}
    try:
        rules = compute_depth_rules(snapshot, context.profile.observation,
                                    context.profile.forward_step_limit_m)
    except Exception as exc:
        rules = invalid_depth_rules(snapshot.observation_id, f"DEPTH_PARSE_{type(exc).__name__}")
    context.observation = snapshot
    context.depth_rules = rules
    return {"success": True, "prompt": prompt.strip(), "view": view,
            "observation_id": snapshot.observation_id,
            "depth_valid": rules.depth_valid,
            "forward_max_m": rules.forward_max_m,
            "depth_reason": rules.reason,
            "message": "已取得当前图像与深度摘要"}


def _measure_distance_handler(context: ToolContext, arguments: dict[str, Any]) -> dict:
    """只测量模型已看到且尚未因动作失效的前视观察。"""
    return measure_target_distance(context.observation, context.profile.observation,
                                   arguments.get("x"), arguments.get("y"))


def _activate_skill_handler(context: ToolContext, arguments: dict[str, Any]) -> dict:
    """将内置 skill 交给当前进程中的受控扫描执行。"""
    from drone_harness.runtime.skill_search import activate_skill

    return activate_skill(context, arguments)


TOOL_DEFINITIONS = [
    ToolDefinition("observe", OBSERVE_TOOL_SCHEMA, _observe_handler),
    ToolDefinition("measure_distance", MEASURE_DISTANCE_TOOL_SCHEMA, _measure_distance_handler),
    ToolDefinition("get_state", GET_STATE_TOOL_SCHEMA, _get_state_handler),
    ToolDefinition("takeoff", TAKEOFF_TOOL_SCHEMA, _takeoff_handler),
    ToolDefinition("forward", FORWARD_TOOL_SCHEMA, _forward_handler),
    ToolDefinition("up", UP_TOOL_SCHEMA, _up_handler),
    ToolDefinition("down", DOWN_TOOL_SCHEMA, _down_handler),
    ToolDefinition("rotate", ROTATE_TOOL_SCHEMA, _rotate_handler),
    ToolDefinition("land", LAND_TOOL_SCHEMA, _land_handler),
    ToolDefinition("activate_skill", ACTIVATE_SKILL_TOOL_SCHEMA, _activate_skill_handler),
]
TOOL_DEFINITION_BY_NAME = {definition.name: definition for definition in TOOL_DEFINITIONS}
TOOL_DEFINITION_BY_NAME["update_navigation_plan"] = ToolDefinition(
    "update_navigation_plan", NAVIGATION_PLAN_TOOL_SCHEMA, update_navigation_plan)


def get_tool_definitions() -> list[ToolDefinition]:
    """返回模型当前可见的观察及飞行动作定义。"""
    return list(TOOL_DEFINITIONS)


def get_tool_definition(name: str) -> ToolDefinition | None:
    """按名称查找模型可见动作。"""
    return TOOL_DEFINITION_BY_NAME.get(name)


__all__ = [
    "ToolContext",
    "ToolDefinition",
    "get_tool_definition",
    "get_tool_definitions",
    "get_tool_schemas",
]
