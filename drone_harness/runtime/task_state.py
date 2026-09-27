"""定义当前会话的最小运行时任务状态。"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from typing import Any

STATE_COLOR_GREEN = "\033[32m"
STATE_COLOR_RESET = "\033[0m"


@dataclass
class TaskState:
    """保存当前会话中 agent 的核心运行状态。"""

    task_id: str
    current_user_goal: str | None = None
    current_phase: str = "idle"
    active_tool_name: str | None = None
    active_tool_arguments: dict[str, Any] | None = None
    active_tool_is_flight_tool: bool = False
    active_agent_name: str = "drone_harness"
    waiting_for_user_confirmation: bool = False
    intervention_pending: bool = False
    intervention_message: str | None = None
    last_tool_name: str | None = None
    last_tool_result: dict[str, Any] | None = None
    last_error: str | None = None
    observation_id: str | None = None
    step_id: int = 0
    consecutive_rejections: int = 0
    consecutive_no_progress: int = 0
    landing_authorized: bool = False
    completion_candidate: str | None = None

    def start_new_goal(self, user_input: str) -> None:
        """在用户输入新任务后刷新当前目标状态。"""
        self.current_user_goal = user_input
        self.current_phase = "thinking"
        self.active_tool_name = None
        self.active_tool_arguments = None
        self.active_tool_is_flight_tool = False
        self.waiting_for_user_confirmation = False
        self.intervention_pending = False
        self.intervention_message = None
        self.last_tool_name = None
        self.last_tool_result = None
        self.last_error = None
        self.observation_id = None
        self.step_id = 0
        self.consecutive_rejections = 0
        self.consecutive_no_progress = 0
        self.landing_authorized = False
        self.completion_candidate = None

    def set_thinking(self) -> None:
        """标记当前轮进入模型思考阶段。"""
        self.current_phase = "thinking"
        self.active_tool_name = None
        self.active_tool_arguments = None
        self.active_tool_is_flight_tool = False
        self.waiting_for_user_confirmation = False
        self.last_error = None

    def set_idle(self) -> None:
        """标记当前轮已回到空闲状态。"""
        self.current_phase = "idle"
        self.active_tool_name = None
        self.active_tool_arguments = None
        self.active_tool_is_flight_tool = False
        self.waiting_for_user_confirmation = False
        self.last_error = None

    def set_waiting_for_confirmation(
        self,
        tool_name: str,
        arguments: dict[str, Any],
        is_flight_tool: bool,
    ) -> None:
        """标记当前正在等待人工确认。"""
        self.current_phase = "waiting_for_confirmation"
        self.active_tool_name = tool_name
        self.active_tool_arguments = arguments
        self.active_tool_is_flight_tool = is_flight_tool
        self.waiting_for_user_confirmation = True
        self.last_error = None

    def start_tool(
        self,
        tool_name: str,
        arguments: dict[str, Any],
        is_flight_tool: bool,
    ) -> None:
        """标记当前正在执行工具。"""
        self.current_phase = "tool_running"
        self.active_tool_name = tool_name
        self.active_tool_arguments = arguments
        self.active_tool_is_flight_tool = is_flight_tool
        self.waiting_for_user_confirmation = False
        self.last_error = None

    def finish_tool(self, tool_name: str, result: dict[str, Any]) -> None:
        """根据工具结果更新成功或失败状态。"""
        self.current_phase = "tool_completed" if result.get("success") else "tool_failed"
        self.last_tool_name = tool_name
        self.last_tool_result = result
        self.step_id += 1
        self.consecutive_rejections = 0 if result.get("success") else self.consecutive_rejections + 1
        self.last_error = None if result.get("success") else str(result.get("error") or "")
        self.active_tool_name = None
        self.active_tool_arguments = None
        self.active_tool_is_flight_tool = False
        self.waiting_for_user_confirmation = False

    def interrupt(self, tool_name: str, result: dict[str, Any]) -> None:
        """标记当前轮因拒绝、超时等原因被中断。"""
        already_recorded = self.last_tool_result is result
        self.current_phase = "interrupted"
        self.last_tool_name = tool_name
        self.last_tool_result = result
        if not already_recorded:
            self.step_id += 1
            self.consecutive_rejections += 1
        self.last_error = str(result.get("error") or "")
        if result.get("intervention_message"):
            self.intervention_pending = True
            self.intervention_message = str(result["intervention_message"])
        self.active_tool_name = None
        self.active_tool_arguments = None
        self.active_tool_is_flight_tool = False
        self.waiting_for_user_confirmation = False

    def mark_intervention(self, message: str) -> None:
        """记录一条等待处理的用户介入消息。"""
        self.current_phase = "interrupted"
        self.intervention_pending = True
        self.intervention_message = message
        self.last_error = "INTERRUPTED_BY_USER"

    def clear_intervention(self) -> None:
        """清空已经交给 LLM 处理的介入状态。"""
        self.intervention_pending = False
        self.intervention_message = None

    def set_observation(self, observation_id: str) -> None:
        """记录本轮用于规划和安全核对的观测号。"""
        self.observation_id = observation_id

    def clear_observation(self) -> None:
        """移动或重取图时清掉不再代表当前位置的观测号。"""
        self.observation_id = None

    def record_motion_progress(
        self,
        tool_name: str,
        before_ned: tuple[float, float, float] | None,
        after_ned: tuple[float, float, float] | None,
        *,
        rotation_degrees: float | None = None,
    ) -> None:
        """平移按位姿计数；成功转向且取得新观测后允许重新探索。"""
        if tool_name == "rotate":
            if (isinstance(rotation_degrees, (int, float)) and not isinstance(rotation_degrees, bool)
                    and math.isfinite(rotation_degrees) and rotation_degrees > 0):
                self.consecutive_no_progress = 0
            return
        if tool_name not in {"takeoff", "forward", "up", "down"}:
            return
        if before_ned is None or after_ned is None:
            self.consecutive_no_progress += 1
            return
        distance_sq = sum((new - old) ** 2 for old, new in zip(before_ned, after_ned))
        self.consecutive_no_progress = 0 if distance_sq >= 0.05 ** 2 else self.consecutive_no_progress + 1

    def snapshot(self) -> dict[str, Any]:
        """导出当前状态快照，供日志记录使用。"""
        return asdict(self)


def format_task_state_line(task_state: TaskState) -> str:
    """格式化终端中的简洁状态输出。"""
    parts = [f"state> {task_state.current_phase}"]
    if task_state.active_tool_name:
        parts.append(task_state.active_tool_name)
    elif task_state.last_tool_name and task_state.current_phase in {"tool_completed", "tool_failed", "interrupted"}:
        parts.append(task_state.last_tool_name)
    if task_state.active_tool_is_flight_tool:
        parts.append("flight_tool=true")
    if task_state.waiting_for_user_confirmation:
        parts.append("waiting_confirmation=true")
    if task_state.last_error:
        parts.append(f"error={task_state.last_error}")
    return f"{STATE_COLOR_GREEN}{' '.join(parts)}{STATE_COLOR_RESET}"
