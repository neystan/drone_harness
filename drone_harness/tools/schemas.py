"""定义单目标闭环的模型可见动作 schema。"""

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
        "description": "只沿当前朝向前进。程序在每次调用时读取新深度并计算上限；深度失效则执行 0 米，超限则缩短并反馈。",
        "parameters": {
            "type": "object",
            "properties": {
                "distance_m": {
                    "type": "number",
                    "description": "请求的正向米数；仿真单次最多按 19 米与本次深度上限中较小者执行。",
                }
            },
            "required": ["distance_m"],
            "additionalProperties": False,
        },
    },
}

UP_TOOL_SCHEMA = {
    "type": "function",
    "function": {
        "name": "up",
        "description": "仅在空中上升；distance_m 是本次米数，必须为正且不超过当前 profile 的单次垂直限额。可多次调用，不设累计高度上限；不检查上方障碍。",
        "parameters": {
            "type": "object",
            "properties": {"distance_m": {"type": "number", "description": "本次上升的正距离，单位米。"}},
            "required": ["distance_m"],
            "additionalProperties": False,
        },
    },
}

DOWN_TOOL_SCHEMA = {
    "type": "function",
    "function": {
        "name": "down",
        "description": "仅在空中下降；distance_m 是本次米数，必须为正且不超过当前 profile 的单次垂直限额。不检查下方障碍；目标高度不得低于离地 0.3 米，落地请调用 land。",
        "parameters": {
            "type": "object",
            "properties": {"distance_m": {"type": "number", "description": "本次下降的正距离，单位米。"}},
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

OBSERVE_TOOL_SCHEMA = {
    "type": "function",
    "function": {
        "name": "observe",
        "description": "按需获取当前 RGB 图像和同帧深度规则，交给同一个模型查看；不会飞行。",
        "parameters": {
            "type": "object",
            "properties": {
                "prompt": {
                    "type": "string",
                    "description": "说明本次想从画面里寻找或确认什么，去掉首尾空白后为 1 到 1000 字符。",
                }
            },
            "required": ["prompt"],
            "additionalProperties": False,
        },
    },
}

TOOL_SCHEMAS = [
    OBSERVE_TOOL_SCHEMA,
    TAKEOFF_TOOL_SCHEMA,
    FORWARD_TOOL_SCHEMA,
    UP_TOOL_SCHEMA,
    DOWN_TOOL_SCHEMA,
    ROTATE_TOOL_SCHEMA,
    LAND_TOOL_SCHEMA,
]


def get_tool_schemas() -> list[dict]:
    """返回一个观察工具与六个飞行动作的 schema。"""
    return list(TOOL_SCHEMAS)
