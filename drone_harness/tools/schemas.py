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
                    "description": "本次请求的前进距离，必须大于 0，不超过 29 米。",
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
            "按 view 获取前方、下方或前下两图；每张图标注视角，下视仅RGB。默认 front。prompt 简洁描述观察目标，"
            "不重复索取深度摘要、询问飞行状态或要求计算移动距离。"
        ),
        "parameters": {
            "type": "object",
            "properties": {
                "view": {"type": "string", "enum": ["front", "down", "both"],
                         "description": "front 只看前方（默认），down 只看下方，both 在同一次模型请求中看两图。"},
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

MEASURE_DISTANCE_TOOL_SCHEMA = {
    "type": "function",
    "function": {
        "name": "measure_distance",
        "description": "以接近完成子目标或核实最终参照物时须实际测距；常规前进无需先测距。target说明所选参照物及表面，程序不据此确认身份；valid只表示选点深度有效，不证明目标身份、通道宽度或可通行性。只读最新前视配对深度，不移动；动作后先 observe。返回实际射线距离与有效性，不自动判定到达。",
        "parameters": {
            "type": "object", "additionalProperties": False,
            "properties": {
                "target": {"type": "string", "minLength": 1, "maxLength": 1000,
                           "description": "所测参照物实例及可见表面，例如指定白色集装箱朝向无人机的正面；不得以附近障碍或旧地标替代。"},
                "x": {"type": "number", "minimum": 0, "maximum": 1,
                      "description": "图像左侧为 0、右侧为 1；选择目标实体表面内部，避开边缘和门洞。"},
                "y": {"type": "number", "minimum": 0, "maximum": 1,
                      "description": "图像顶部为 0、底部为 1。"},
            },
            "required": ["target", "x", "y"],
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

ACTIVATE_SKILL_TOOL_SCHEMA = {
    "type": "function",
    "function": {
        "name": "activate_skill",
        "description": "按需启用内置 skill。four-direction-search 会固定完成四向扫描并恢复朝向，再一次提供四图。按指令动作并观察仍找不到参照物，或同一通路判断持续无进展、需要寻找绕行方向时使用。",
        "parameters": {
            "type": "object", "additionalProperties": False,
            "properties": {
                "name": {"type": "string", "enum": ["four-direction-search"]},
                "prompt": {"type": "string", "minLength": 1, "maxLength": 1000,
                           "description": "本次搜索重点，由你结合任务和已有观察生成：寻找什么、核对哪些特征或关系。"},
            },
            "required": ["name", "prompt"],
        },
    },
}


TOOL_SCHEMAS = [
    OBSERVE_TOOL_SCHEMA,
    MEASURE_DISTANCE_TOOL_SCHEMA,
    GET_STATE_TOOL_SCHEMA,
    TAKEOFF_TOOL_SCHEMA,
    FORWARD_TOOL_SCHEMA,
    UP_TOOL_SCHEMA,
    DOWN_TOOL_SCHEMA,
    ROTATE_TOOL_SCHEMA,
    LAND_TOOL_SCHEMA,
    ACTIVATE_SKILL_TOOL_SCHEMA,
]


NAVIGATION_PLAN_TOOL_SCHEMA = {
    "type": "function",
    "function": {
        "name": "update_navigation_plan",
        "description": "创建或替换本轮完整导航清单；按有明确阶段终态的完整语义段组织，保留前进、条件转向及路线顺序，同一目标沿用 ID。"
                       "仅在计划或进度变化时调用；确认当前段完成后先提交清单，将当前项设completed；有下一项时设in_progress，再执行该段，不只口头切换。"
                       "降落只作清单完成后的收尾，不写入条目或完成条件。",
        "parameters": {
            "type": "object", "additionalProperties": False,
            "required": ["subgoals", "reason"],
            "properties": {
                "subgoals": {"type": "array", "minItems": 1, "items": {
                    "type": "object", "additionalProperties": False,
                    "required": ["id", "description", "completion_condition", "status"],
                    "properties": {
                        "id": {"type": "string", "minLength": 1},
                        "description": {"type": "string", "minLength": 1,
                                        "description": "本段完整导航任务及原文触发前提；无独立结束位置的前进与后续定位动作合并，方向参照不强加接近，不补造路线距离或固定转角。"},
                        "completion_condition": {"type": "string", "minLength": 1,
                                                 "description": "整段全部动作后的可观察终态及所需依据；接近需当前目标有效实测≤20米，保留原文位置条件，不写完成该段前进、继续飞行或满足条件后降落。"},
                        "status": {"type": "string", "enum": ["pending", "in_progress", "completed"]},
                        "evidence": {"type": "string", "description": "实际动作、观察变化及所测目标与真实距离；不补造依据，不能可靠汇总时不写累计米数。可省略，已有目标沿用原依据，新目标为空。"},
                    },
                }},
                "reason": {"type": "string", "minLength": 1},
            },
        },
    },
}


def get_tool_schemas(profile: RuntimeProfile | None = None, *, navigation_enabled: bool = False) -> list[dict]:
    """复制工具描述，并将当前配置的限额写入参数说明。"""
    schemas = deepcopy(TOOL_SCHEMAS)
    if navigation_enabled:
        schemas.append(deepcopy(NAVIGATION_PLAN_TOOL_SCHEMA))
        next(item["function"] for item in schemas if item["function"]["name"] == "land")["description"] = (
            "在当前位置降落，成功立即结束本轮。导航任务先更新清单确认全部导航子目标完成，再调用本工具。"
        )
    if profile is None:
        return schemas
    functions = {item["function"]["name"]: item["function"] for item in schemas}
    forward_limit = profile.forward_step_limit_m
    if profile.mode == "simulation":
        forward_limit = min(forward_limit, 29.0)
        functions["down"]["description"] = (
            "仅在空中垂直下降，不检查下方障碍；允许下降到起飞参考面以下，落地使用 land。"
        )
    limits = (
        ("takeoff", "height", "本次起飞的相对高度", profile.safety.max_takeoff_height_m),
        ("forward", "distance_m", "本次请求的前进距离", forward_limit),
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
