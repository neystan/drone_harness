"""定义导航工具用途、观察示例和当前配置的参数限额。"""

from __future__ import annotations

from copy import deepcopy

from drone_harness.config.schema import RuntimeProfile


TAKEOFF_TOOL_SCHEMA = {
    "type": "function",
    "function": {
        "name": "takeoff",
        "description": "从地面垂直起飞；已在空中时使用 up。",
        "parameters": {
            "type": "object",
            "properties": {"height": {"type": "number", "description": "本次起飞的相对高度，必须大于 0，不超过 10 米。"}},
            "required": ["height"],
            "additionalProperties": False,
        },
    },
}

FORWARD_TOOL_SCHEMA = {
    "type": "function",
    "function": {
        "name": "forward",
        "description": "仅在空中沿当前朝向向前移动。",
        "parameters": {
            "type": "object",
            "properties": {
                "distance_m": {
                    "type": "number",
                    "description": "本次请求的前进距离，必须大于 0，不超过 19 米。",
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
        "description": "仅在空中垂直上升，不检查上方障碍；可多次调用。",
        "parameters": {
            "type": "object",
            "properties": {"distance_m": {"type": "number", "description": "本次上升距离，必须大于 0，不超过 10 米。"}},
            "required": ["distance_m"],
            "additionalProperties": False,
        },
    },
}

DOWN_TOOL_SCHEMA = {
    "type": "function",
    "function": {
        "name": "down",
        "description": "仅在空中垂直下降，不检查下方障碍；目标不得低于地面参考高度以上 0.3 米，落地使用 land。",
        "parameters": {
            "type": "object",
            "properties": {"distance_m": {"type": "number", "description": "本次下降距离，必须大于 0，不超过 10 米。"}},
            "required": ["distance_m"],
            "additionalProperties": False,
        },
    },
}

ROTATE_TOOL_SCHEMA = {
    "type": "function",
    "function": {
        "name": "rotate",
        "description": "在空中保持位置，向左或向右旋转。",
        "parameters": {
            "type": "object",
            "properties": {
                "direction": {"type": "string", "enum": ["left", "right"], "description": "left 为左转，right 为右转。"},
                "degrees": {"type": "number", "description": "本次旋转角度，范围为 0 到 360 度。"},
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
        "description": "在当前位置降落。",
        "parameters": {"type": "object", "properties": {}, "additionalProperties": False},
    },
}

OBSERVE_TOOL_SCHEMA = {
    "type": "function",
    "function": {
        "name": "observe",
        "description": (
            "获取当前 RGB 图像与深度摘要。prompt 简洁描述观察目标，"
            "不重复索取深度摘要、询问飞行状态或要求计算移动距离。"
        ),
        "parameters": {
            "type": "object",
            "properties": {
                "prompt": {
                    "type": "string",
                    "description": (
                        "本次观察重点。示例："
                        "①寻找左前方十字路口，辨认入口方向及周围地标；"
                        "②确认红色店招与路口的相对位置；"
                        "③旋转后重新寻找目标建筑，确认它在画面中的方向；"
                        "④观察道路是否被树木或建筑遮挡。"
                    ),
                }
            },
            "required": ["prompt"],
            "additionalProperties": False,
        },
    },
}

GET_STATE_TOOL_SCHEMA = {
    "type": "function",
    "function": {
        "name": "get_state",
        "description": (
            "查询连接、解锁、飞行模式、是否在空中、本地三维位置和参考高度。"
            "位置采用本地北/东/下坐标（米），不是经纬度；高度相对记录的地面参考，不是实时离地距离。"
            "缺失数据返回未知。"
        ),
        "parameters": {"type": "object", "properties": {}, "additionalProperties": False},
    },
}

TOOL_SCHEMAS = [
    OBSERVE_TOOL_SCHEMA,
    GET_STATE_TOOL_SCHEMA,
    TAKEOFF_TOOL_SCHEMA,
    FORWARD_TOOL_SCHEMA,
    UP_TOOL_SCHEMA,
    DOWN_TOOL_SCHEMA,
    ROTATE_TOOL_SCHEMA,
    LAND_TOOL_SCHEMA,
]


def get_tool_schemas(profile: RuntimeProfile | None = None) -> list[dict]:
    """复制工具描述，并将当前配置的限额写入参数说明。"""
    schemas = deepcopy(TOOL_SCHEMAS)
    if profile is None:
        return schemas
    functions = {item["function"]["name"]: item["function"] for item in schemas}
    limits = (
        ("takeoff", "height", "本次起飞的相对高度", profile.safety.max_takeoff_height_m),
        ("forward", "distance_m", "本次请求的前进距离", profile.forward_step_limit_m),
        ("up", "distance_m", "本次上升距离", profile.safety.max_vertical_move_m),
        ("down", "distance_m", "本次下降距离", profile.safety.max_vertical_move_m),
    )
    for tool, parameter, label, limit in limits:
        functions[tool]["parameters"]["properties"][parameter]["description"] = (
            f"{label}，必须大于 0，不超过 {limit:g} 米。"
        )
    functions["rotate"]["parameters"]["properties"]["degrees"]["description"] = (
        f"本次旋转角度，范围为 0 到 {profile.safety.max_rotation_deg:g} 度。"
    )
    return schemas
