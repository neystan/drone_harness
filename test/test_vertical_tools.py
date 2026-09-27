"""验证起飞后的纯垂直工具复用原移动和人工确认边界。"""

from __future__ import annotations

import math
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from drone_harness.config.loader import load_profile
from drone_harness.runtime.safety import requires_human_in_the_loop
from drone_harness.runtime.task_state import TaskState
from drone_harness.tools import flight
from drone_harness.tools.registry import ToolContext, get_tool_definition
from drone_harness.tools.schemas import get_tool_schemas


def vertical_context(mode: str = "sim") -> ToolContext:
    """只提供垂直工具校验所需的飞行状态和配置。"""
    profile = load_profile(mode, settings_path=Path(__file__).parents[1] / "settings.example.json")
    controller = SimpleNamespace(
        vehicle_status=SimpleNamespace(connected=True, armed=True, mode="OFFBOARD"),
        flight_state=lambda: "IN_AIR",
    )
    return ToolContext(controller=controller, profile=profile, task_state=TaskState("vertical-test"))


def test_up_and_down_only_request_vertical_move(monkeypatch) -> None:
    """上升和下降只改变 FRD 的 z，不提供侧移或后退入口。"""
    context = vertical_context()
    move = Mock(return_value={"success": True})
    monkeypatch.setattr(flight, "move", move)

    assert get_tool_definition("up").handler(context, {"distance_m": 2.0})["success"]
    assert move.call_args.args[1:] == (0.0, 0.0, -2.0)
    assert move.call_args.kwargs == {"completion_tolerance_m": 0.05}
    assert get_tool_definition("down").handler(context, {"distance_m": 1.0})["success"]
    assert move.call_args.args[1:] == (0.0, 0.0, 1.0)
    assert move.call_count == 2


@pytest.mark.parametrize("distance", [None, True, 0, -1, math.nan, math.inf, 10.01])
def test_invalid_vertical_distance_never_moves(monkeypatch, distance) -> None:
    """无效数值或超过单次限额时不下发目标，也不静默截短。"""
    context = vertical_context()
    move = Mock()
    monkeypatch.setattr(flight, "move", move)

    assert not flight.up(context, distance)["success"]
    assert not flight.down(context, distance)["success"]
    move.assert_not_called()


def test_repeated_up_has_no_absolute_ten_meter_ceiling(monkeypatch) -> None:
    """十米是单次上限，不是反复上升后的总高度上限。"""
    context = vertical_context()
    context.controller.height_above_ground_m = Mock(return_value=12.0)
    move = Mock(return_value={"success": True})
    monkeypatch.setattr(flight, "move", move)

    assert flight.up(context, 8.0)["success"]
    move.assert_called_once()


def test_sim_loiter_can_resume_but_disarmed_vehicle_cannot(monkeypatch) -> None:
    """仿真悬停可沿原控制器恢复，未解锁不能由工具偷偷重新解锁。"""
    context = vertical_context()
    context.controller.vehicle_status.mode = "AUTO.LOITER"
    move = Mock(return_value={"success": True})
    monkeypatch.setattr(flight, "move", move)
    assert flight.up(context, 0.5)["success"]
    context.controller.vehicle_status.armed = False
    assert flight.down(context, 0.5)["error"] == "PX4_STATE_INVALID"
    assert move.call_count == 1


def test_down_below_ground_clearance_is_rejected_before_setpoint(monkeypatch) -> None:
    """下降目标距地不足 0.3 米时沿用原 move 拒绝，不当作降落。"""
    context = vertical_context()
    controller = context.controller
    controller.vehicle_local_position = SimpleNamespace(x=0.0, y=0.0, z=-1.0, heading=0.0)
    controller.body_to_ned = Mock(return_value=(0.0, 0.0, 0.8))
    controller.height_above_ground_m = Mock(return_value=0.2)
    controller.start_position_hold = Mock()
    monkeypatch.setattr(flight, "_wait_for_valid_position", lambda _controller: True)

    result = flight.down(context, 0.8)

    assert result["error"] == "TARGET_Z_TOO_LOW"
    controller.start_position_hold.assert_not_called()


def test_vertical_tools_have_clear_schema_and_real_hitl() -> None:
    """工具说明写清空中使用、单次限额及不检查上下障碍，真机逐次审批。"""
    schemas = {item["function"]["name"]: item["function"] for item in get_tool_schemas()}
    for name in ("up", "down"):
        description = schemas[name]["description"]
        assert "空中" in description
        assert "单次" in description
        assert "障碍" in description
        assert requires_human_in_the_loop(vertical_context("real").profile, name)


def test_vertical_motion_counts_as_progress() -> None:
    """升降发生实测位移后清零连续未移动计数。"""
    state = TaskState("vertical-test")
    state.consecutive_no_progress = 2
    state.record_motion_progress("up", (0.0, 0.0, -1.0), (0.0, 0.0, -1.5))
    assert state.consecutive_no_progress == 0
    state.record_motion_progress("down", (0.0, 0.0, -1.5), (0.0, 0.0, -1.5))
    assert state.consecutive_no_progress == 1
