"""集中定义函数调用工具的 schema。"""

from __future__ import annotations


TAKEOFF_TOOL_SCHEMA = {
    "type": "function",
    "function": {
        "name": "takeoff",
        "description": "Take off the UAV to a target height in meters.",
        "parameters": {
            "type": "object",
            "properties": {
                "height": {
                    "type": "number",
                    "description": "Target takeoff height in meters. Valid range is 0 to 10.",
                }
            },
            "required": ["height"],
            "additionalProperties": False,
        },
    },
}

LAND_TOOL_SCHEMA = {
    "type": "function",
    "function": {
        "name": "land",
        "description": "Land the UAV safely at the current location.",
        "parameters": {
            "type": "object",
            "properties": {},
            "additionalProperties": False,
        },
    },
}

DISARM_TOOL_SCHEMA = {
    "type": "function",
    "function": {
        "name": "disarm",
        "description": "Disarm the UAV motors.",
        "parameters": {
            "type": "object",
            "properties": {},
            "additionalProperties": False,
        },
    },
}

TIMER_TOOL_SCHEMA = {
    "type": "function",
    "function": {
        "name": "timer",
        "description": "Wait for a specified number of seconds.",
        "parameters": {
            "type": "object",
            "properties": {
                "seconds": {
                    "type": "integer",
                    "description": "Number of seconds to wait. Must be between 1 and 600.",
                }
            },
            "required": ["seconds"],
            "additionalProperties": False,
        },
    },
}

HOVER_TOOL_SCHEMA = {
    "type": "function",
    "function": {
        "name": "hover",
        "description": "Switch the UAV to AUTO_LOITER hover mode at the current location.",
        "parameters": {
            "type": "object",
            "properties": {},
            "additionalProperties": False,
        },
    },
}

RETURN_HOME_TOOL_SCHEMA = {
    "type": "function",
    "function": {
        "name": "return_home",
        "description": "Command the UAV to return to home using PX4 RTL mode.",
        "parameters": {
            "type": "object",
            "properties": {},
            "additionalProperties": False,
        },
    },
}

CURRENT_POSITION_TOOL_SCHEMA = {
    "type": "function",
    "function": {
        "name": "current_position_status",
        "description": "Get the UAV current local position in NED coordinates.",
        "parameters": {
            "type": "object",
            "properties": {},
            "additionalProperties": False,
        },
    },
}

BATTERY_STATUS_TOOL_SCHEMA = {
    "type": "function",
    "function": {
        "name": "battery_status",
        "description": "Get the UAV battery status.",
        "parameters": {
            "type": "object",
            "properties": {},
            "additionalProperties": False,
        },
    },
}

FLIGHT_MODE_STATUS_TOOL_SCHEMA = {
    "type": "function",
    "function": {
        "name": "flight_mode_status",
        "description": "Get the UAV current flight mode and basic state.",
        "parameters": {
            "type": "object",
            "properties": {},
            "additionalProperties": False,
        },
    },
}

ROTATE_TOOL_SCHEMA = {
    "type": "function",
    "function": {
        "name": "rotate",
        "description": "Rotate the UAV left or right by a specified angle in degrees while holding the current position.",
        "parameters": {
            "type": "object",
            "properties": {
                "direction": {
                    "type": "string",
                    "enum": ["left", "right"],
                    "description": "Rotation direction. Use 'left' for counterclockwise and 'right' for clockwise.",
                },
                "degrees": {
                    "type": "number",
                    "description": "Rotation angle in degrees. Must be between 0 and 360.",
                },
            },
            "required": ["direction", "degrees"],
            "additionalProperties": False,
        },
    },
}

MOVE_TOOL_SCHEMA = {
    "type": "function",
    "function": {
        "name": "move",
        "description": "Move the UAV by a relative offset in the body FRD frame in meters.",
        "parameters": {
            "type": "object",
            "properties": {
                "x": {
                    "type": "number",
                    "description": "Body-frame x offset in meters. Positive is forward, negative is backward. Valid range is -20 to 20.",
                },
                "y": {
                    "type": "number",
                    "description": "Body-frame y offset in meters. Positive is right, negative is left. Valid range is -20 to 20.",
                },
                "z": {
                    "type": "number",
                    "description": "Body-frame z offset in meters. Positive is down, negative is up. Valid range is -10 to 10.",
                },
            },
            "required": ["x", "y", "z"],
            "additionalProperties": False,
        },
    },
}

TAKE_PHOTO_TOOL_SCHEMA = {
    "type": "function",
    "function": {
        "name": "take_photo",
        "description": "Capture one RGB photo from the front camera and save it to a local file.",
        "parameters": {
            "type": "object",
            "properties": {},
            "additionalProperties": False,
        },
    },
}

ANALYZE_VIEW_TOOL_SCHEMA = {
    "type": "function",
    "function": {
        "name": "analyze_view",
        "description": "Analyze the current camera view. Optionally search for a target in the scene.",
        "parameters": {
            "type": "object",
            "properties": {
                "target_description": {
                    "type": "string",
                    "description": "Optional target to search for in the current camera view. Leave empty to only describe the scene.",
                }
            },
            "additionalProperties": False,
        },
    },
}

DETECT_TARGET_TOOL_SCHEMA = {
    "type": "function",
    "function": {
        "name": "detect_target",
        "description": "使用语义检测模型在当前相机画面中检测目标，并返回全部候选目标框。调用时应完整保留用户描述中的位置、距离、序数和外观限定词。",
        "parameters": {
            "type": "object",
            "properties": {
                "target_description": {
                    "type": "string",
                    "description": "要检测的完整目标描述，例如“最近的树”“中间的路灯”“左侧第二盏路灯”“红色车辆”。不要把“最近的树”简化成“树”。",
                }
            },
            "required": ["target_description"],
            "additionalProperties": False,
        },
    },
}

SAM_TRACKING_TOOL_SCHEMA = {
    "type": "function",
    "function": {
        "name": "sam_tracking",
        "description": "使用 DINO-XSeek 自动语义检测初始化目标框，再由 SAM2 追踪。该路径可能产生付费 API 流量。支持 start、restart、status、stop；status 和 stop 不会重新检测。",
        "parameters": {
            "type": "object",
            "properties": {
                "action": {
                    "type": "string",
                    "enum": ["start", "restart", "status", "stop"],
                    "description": "start 启动追踪，restart 更换或重新锁定目标，status 被动查询一次目标位置，stop 停止追踪。",
                },
                "target_description": {
                    "type": "string",
                    "description": "start 或 restart 时必填，必须完整保留用户描述中的限定词，例如“最近的树”“左侧第二盏路灯”。",
                },
                "target_index": {
                    "type": "integer",
                    "description": "当 DINO-XSEEK 检测到多个候选目标时，指定红框图中的目标索引。",
                },
            },
            "required": ["action"],
            "additionalProperties": False,
        },
    },
}

MOUSE_TRACKING_TOOL_SCHEMA = {
    "type": "function",
    "function": {
        "name": "mouse_tracking",
        "description": "免费鼠标手动选点并由 SAM2 持续追踪，不调用 DINO-XSeek 或其他语义检测器。支持 start、restart、status、stop。",
        "parameters": {
            "type": "object",
            "properties": {
                "action": {
                    "type": "string",
                    "enum": ["start", "restart", "status", "stop"],
                    "description": "start 等待鼠标点击并启动追踪，restart 点击新目标并切换追踪，status 查询状态，stop 停止追踪。",
                }
            },
            "required": ["action"],
            "additionalProperties": False,
        },
    },
}

ACTIVATE_SKILL_TOOL_SCHEMA = {
    "type": "function",
    "function": {
        "name": "activate_skill",
        "description": "启用一个当前可用的 drone_agent skill，并返回该 skill 的完整内容。",
        "parameters": {
            "type": "object",
            "properties": {
                "name": {
                    "type": "string",
                    "description": "要启用的 skill 名称，必须使用 Skills Index 中列出的精确 name。",
                }
            },
            "required": ["name"],
            "additionalProperties": False,
        },
    },
}

TOOL_SCHEMAS = [
    ACTIVATE_SKILL_TOOL_SCHEMA,
    TAKEOFF_TOOL_SCHEMA,
    LAND_TOOL_SCHEMA,
    DISARM_TOOL_SCHEMA,
    TIMER_TOOL_SCHEMA,
    HOVER_TOOL_SCHEMA,
    RETURN_HOME_TOOL_SCHEMA,
    CURRENT_POSITION_TOOL_SCHEMA,
    BATTERY_STATUS_TOOL_SCHEMA,
    FLIGHT_MODE_STATUS_TOOL_SCHEMA,
    ROTATE_TOOL_SCHEMA,
    MOVE_TOOL_SCHEMA,
    TAKE_PHOTO_TOOL_SCHEMA,
    ANALYZE_VIEW_TOOL_SCHEMA,
    DETECT_TARGET_TOOL_SCHEMA,
    SAM_TRACKING_TOOL_SCHEMA,
    MOUSE_TRACKING_TOOL_SCHEMA,
]


def get_tool_schemas() -> list[dict]:
    """返回注册给大模型的全部工具 schema。"""
    return list(TOOL_SCHEMAS)
