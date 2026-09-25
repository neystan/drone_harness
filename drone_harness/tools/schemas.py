"""定义单目标闭环的四个模型可见动作 schema。"""

from __future__ import annotations


TAKEOFF_TOOL_SCHEMA = {
    "type": "function",
    "function": {
        "name": "takeoff",
        "description": "Take off to a positive height in meters, subject to runtime limits.",
        "parameters": {
            "type": "object",
            "properties": {"height": {"type": "number", "description": "Takeoff height in meters."}},
            "required": ["height"],
            "additionalProperties": False,
        },
    },
}

FORWARD_TOOL_SCHEMA = {
    "type": "function",
    "function": {
        "name": "forward",
        "description": "Move a short distance only along the current forward direction when depth rules allow it.",
        "parameters": {
            "type": "object",
            "properties": {
                "distance_m": {
                    "type": "number",
                    "description": "Positive forward distance in meters, no greater than this observation's limit.",
                }
            },
            "required": ["distance_m"],
            "additionalProperties": False,
        },
    },
}

ROTATE_TOOL_SCHEMA = {
    "type": "function",
    "function": {
        "name": "rotate",
        "description": "Rotate left or right while holding the current position.",
        "parameters": {
            "type": "object",
            "properties": {
                "direction": {"type": "string", "enum": ["left", "right"]},
                "degrees": {"type": "number", "description": "Positive rotation angle in degrees."},
            },
            "required": ["direction", "degrees"],
            "additionalProperties": False,
        },
    },
}

LAND_TOOL_SCHEMA = {
    "type": "function",
    "function": {
        "name": "land",
        "description": "Land only when runtime holds explicit landing authorization.",
        "parameters": {"type": "object", "properties": {}, "additionalProperties": False},
    },
}

TOOL_SCHEMAS = [
    TAKEOFF_TOOL_SCHEMA,
    FORWARD_TOOL_SCHEMA,
    ROTATE_TOOL_SCHEMA,
    LAND_TOOL_SCHEMA,
]


def get_tool_schemas() -> list[dict]:
    """返回注册给同一多模态模型的四个工具 schema。"""
    return list(TOOL_SCHEMAS)
