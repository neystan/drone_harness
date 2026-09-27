"""验证 S7 撤除深度守卫后仍沿用原 move 安全交接。"""

from __future__ import annotations

import inspect
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from drone_harness.runtime.safety import SafetyHandoffRequired
from drone_harness.tools import flight
from test_forward_gate import forward_context


def motion_context(tmp_path: Path):
    """给原 move 提供只记录目标点和飞控交接的空中替身。"""
    context = forward_context(tmp_path)
    controller = context.controller
    controller.vehicle_local_position = SimpleNamespace(x=0.0, y=0.0, z=-1.0, heading=0.0)
    controller.timer_period = 0.001
    controller.uav_position_is_valid = Mock(return_value=True)
    controller.body_to_ned = Mock(return_value=(0.2, 0.0, 0.0))
    controller.height_above_ground_m = Mock(return_value=1.0)
    controller.start_position_hold = Mock(return_value=True)
    controller.is_at_target = Mock(return_value=False)
    controller.current_position_ned = Mock(return_value=[0.0, 0.0, -1.0])
    controller.stop_position_hold = Mock()
    controller.send_hover_command = Mock()
    controller.get_logger = Mock(return_value=SimpleNamespace(info=Mock()))
    return context


def test_move_timeout_still_requests_confirmed_hover(tmp_path: Path, monkeypatch) -> None:
    """原 move 超时路径仍交给 PX4 确认悬停。"""
    context = motion_context(tmp_path)
    monkeypatch.setattr(flight, "_wait_for_valid_position", lambda _controller: True)
    monkeypatch.setattr(flight.time, "time", Mock(side_effect=[0.0, 100.0]))
    hover = Mock(return_value={"safety_state": "HOLD_CONFIRMED"})
    monkeypatch.setattr(flight, "request_confirmed_hover", hover)
    result = flight.move(context, 0.2, 0.0, 0.0)
    assert result["error"] == "MOVE_TIMEOUT"
    assert result["safety_state"] == "HOLD_CONFIRMED"
    context.controller.start_position_hold.assert_called_once_with([0.2, 0.0, -1.0])
    hover.assert_called_once()


def test_unconfirmed_hover_still_escalates_to_offboard_handoff(tmp_path: Path, monkeypatch) -> None:
    """超时且 PX4 未确认悬停时仍抛出原安全交接异常。"""
    context = motion_context(tmp_path)
    monkeypatch.setattr(flight, "_wait_for_valid_position", lambda _controller: True)
    monkeypatch.setattr(flight.time, "time", Mock(side_effect=[0.0, 100.0]))
    monkeypatch.setattr(flight, "request_confirmed_hover",
                        Mock(side_effect=SafetyHandoffRequired("SAFETY_HANDOFF_REQUIRED")))
    with pytest.raises(SafetyHandoffRequired):
        flight.move(context, 0.2, 0.0, 0.0)


def test_user_interrupt_retains_original_move_path(tmp_path: Path, monkeypatch) -> None:
    """人工介入沿用原 stop_position_hold 与 hover 命令路径。"""
    context = motion_context(tmp_path)
    context.message_bus = SimpleNamespace(
        has_pending_user_message=lambda: True,
        get_next_user_message=lambda: SimpleNamespace(content="stop"),
    )
    hover = Mock()
    monkeypatch.setattr(flight, "request_confirmed_hover", hover)
    result = flight.move(context, 0.2, 0.0, 0.0)
    assert result["error"] == "INTERRUPTED_BY_USER"
    context.controller.stop_position_hold.assert_called_once()
    context.controller.send_hover_command.assert_called_once()
    hover.assert_not_called()


def test_short_forward_waits_for_real_position_without_depth_guard(tmp_path: Path, monkeypatch) -> None:
    """0.2 米短步不在起点误判完成，也不轮询新深度。"""
    context = motion_context(tmp_path)
    context.controller.is_at_target = Mock(return_value=True)
    context.controller.latest_observation = Mock(side_effect=AssertionError("new depth was polled"))
    context.controller.current_position_ned = Mock(side_effect=[
        [0.0, 0.0, -1.0], [0.2, 0.0, -1.0], [0.2, 0.0, -1.0],
    ])
    monkeypatch.setattr(flight.time, "sleep", lambda _duration: None)
    result = flight.forward(context, 0.2)
    assert result["success"]
    assert context.controller.current_position_ned.call_count == 3
    context.controller.is_at_target.assert_not_called()
    context.controller.latest_observation.assert_not_called()
    assert "guard" not in inspect.signature(flight.move).parameters


def test_sim_forward_resumes_from_loiter_via_existing_offboard_handshake(tmp_path: Path, monkeypatch) -> None:
    """仿真悬停后前进应通过原位置保持握手恢复 OFFBOARD。"""
    context = motion_context(tmp_path)
    controller = context.controller
    controller.vehicle_status.mode = "AUTO.LOITER"
    monkeypatch.setattr(flight, "_wait_for_valid_position", lambda _controller: True)
    monkeypatch.setattr(flight.time, "sleep", lambda _seconds: None)
    controller.current_position_ned = Mock(return_value=[0.2, 0.0, -1.0])

    def confirm_offboard(_target_position):
        """模拟原控制器完成模式确认后才允许位置目标执行。"""
        controller.vehicle_status.mode = "OFFBOARD"
        return True

    controller.start_position_hold = Mock(side_effect=confirm_offboard)
    result = flight.forward(context, 0.2)

    assert result["success"]
    assert result["motion_executed"]
    assert controller.vehicle_status.mode == "OFFBOARD"
    controller.start_position_hold.assert_called_once_with([0.2, 0.0, -1.0])


def test_sim_forward_does_not_claim_success_when_offboard_reentry_fails(tmp_path: Path, monkeypatch) -> None:
    """切回 OFFBOARD 未确认时沿用原悬停交接，不报告前进成功。"""
    context = motion_context(tmp_path)
    controller = context.controller
    controller.vehicle_status.mode = "AUTO.LOITER"
    controller.start_position_hold = Mock(return_value=False)
    controller.position_hold_start_error = "OFFBOARD_NOT_CONFIRMED"
    monkeypatch.setattr(flight, "_wait_for_valid_position", lambda _controller: True)
    hover = Mock(return_value={"safety_state": "HOLD_CONFIRMED"})
    monkeypatch.setattr(flight, "request_confirmed_hover", hover)

    result = flight.forward(context, 0.2)

    assert not result["success"]
    assert result["error"] == "OFFBOARD_NOT_CONFIRMED"
    assert result["safety_state"] == "HOLD_CONFIRMED"
    hover.assert_called_once_with(controller, action_name="move")


def test_sim_only_positive_straight_move_gets_independent_nineteen_meter_cap(
    tmp_path: Path, monkeypatch,
) -> None:
    """仅纯正向可用 19 米，侧移、后退及超限正向仍受原硬限制。"""
    context = motion_context(tmp_path)
    controller = context.controller
    assert flight.move(context, 19.01, 0.0, 0.0)["error"] == "X_OUT_OF_RANGE"
    assert flight.move(context, -0.31, 0.0, 0.0)["error"] == "X_OUT_OF_RANGE"
    assert flight.move(context, 1.0, 0.01, 0.0)["error"] == "X_OUT_OF_RANGE"
    assert flight.move(context, 0.0, 0.31, 0.0)["error"] == "Y_OUT_OF_RANGE"
    controller.start_position_hold.assert_not_called()

    monkeypatch.setattr(flight, "_wait_for_valid_position", lambda _controller: True)
    monkeypatch.setattr(flight.time, "sleep", lambda _seconds: None)
    controller.body_to_ned = Mock(return_value=(19.0, 0.0, 0.0))
    controller.current_position_ned = Mock(return_value=[19.0, 0.0, -1.0])
    assert flight.move(context, 19.0, 0.0, 0.0, completion_tolerance_m=0.05)["success"]
    controller.start_position_hold.assert_called_once_with([19.0, 0.0, -1.0])
