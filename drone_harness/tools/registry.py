"""只向模型暴露单目标闭环所需的四个飞行动作。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable

from drone_harness.bus import MessageBus
from drone_harness.config.schema import RuntimeProfile
from drone_harness.runtime.observation import ObservationSnapshot
from drone_harness.runtime.task_state import TaskState
from drone_harness.vision.depth_rules import DepthRules
from . import flight
from .schemas import (
    DOWN_TOOL_SCHEMA,
    FORWARD_TOOL_SCHEMA,
    LAND_TOOL_SCHEMA,
    ROTATE_TOOL_SCHEMA,
    TAKEOFF_TOOL_SCHEMA,
    UP_TOOL_SCHEMA,
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


TOOL_DEFINITIONS = [
    ToolDefinition("takeoff", TAKEOFF_TOOL_SCHEMA, _takeoff_handler),
    ToolDefinition("forward", FORWARD_TOOL_SCHEMA, _forward_handler),
    ToolDefinition("up", UP_TOOL_SCHEMA, _up_handler),
    ToolDefinition("down", DOWN_TOOL_SCHEMA, _down_handler),
    ToolDefinition("rotate", ROTATE_TOOL_SCHEMA, _rotate_handler),
    ToolDefinition("land", LAND_TOOL_SCHEMA, _land_handler),
]
TOOL_DEFINITION_BY_NAME = {definition.name: definition for definition in TOOL_DEFINITIONS}


def get_tool_definitions() -> list[ToolDefinition]:
    """返回模型当前可见的六个飞行动作定义。"""
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
