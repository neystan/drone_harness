"""定义当前会话的最小运行时任务状态。"""

from __future__ import annotations

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
    active_agent_name: str = "drone_agent"
    waiting_for_user_confirmation: bool = False
    intervention_pending: bool = False
    intervention_message: str | None = None
    last_tool_name: str | None = None
    last_tool_result: dict[str, Any] | None = None
    last_error: str | None = None

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
        self.last_error = None if result.get("success") else str(result.get("error") or "")
        self.active_tool_name = None
        self.active_tool_arguments = None
        self.active_tool_is_flight_tool = False
        self.waiting_for_user_confirmation = False

    def interrupt(self, tool_name: str, result: dict[str, Any]) -> None:
        """标记当前轮因拒绝、超时等原因被中断。"""
        self.current_phase = "interrupted"
        self.last_tool_name = tool_name
        self.last_tool_result = result
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
