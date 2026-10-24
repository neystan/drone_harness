"""绑定最新实际测距与当前目标，仅为模型提供反馈。"""

from typing import Any

from drone_harness.runtime.reflection import current_goal_key


def clear_target_measurement(context: Any) -> None:
    """清除已失效的目标测距及其来源绑定。"""
    context.latest_target_measurement = None
    context.target_measurement_goal = None
    context.target_measurement_observation_id = None


def record_target_measurement(context: Any, target: str, x: Any, y: Any,
                              result: dict[str, Any]) -> None:
    """保存工具真实结果，无效测量也替换之前的成功读数。"""
    plan = context.navigation_plan
    current = plan.current if plan is not None else None
    context.latest_target_measurement = {
        "subgoal_id": current.id if current else None,
        "target": target,
        "point": None if result["reason"] == "INVALID_POINT" else {"x": x, "y": y},
        "valid": result["valid"],
        "distance_m": result["distance_m"],
        "reason": result["reason"],
    }
    context.target_measurement_goal = current_goal_key(context)
    context.target_measurement_observation_id = result.get("observation_id")


def target_measurement_snapshot(context: Any) -> dict[str, Any] | None:
    """仅返回仍对应当前任务、子目标内容与观察的测距摘要。"""
    measurement = context.latest_target_measurement
    if measurement is None:
        return None
    observation_id = context.observation.observation_id if context.observation is not None else None
    if (context.target_measurement_goal != current_goal_key(context)
            or context.target_measurement_observation_id != observation_id):
        clear_target_measurement(context)
        return None
    return {**measurement, "point": dict(measurement["point"]) if measurement["point"] else None}
