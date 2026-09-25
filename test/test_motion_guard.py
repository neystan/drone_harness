"""验证原 move 轮询内的前进守卫和确认悬停交接。"""

from __future__ import annotations

import time
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from drone_harness.runtime.safety import SafetyHandoffRequired
from drone_harness.tools import flight
from test_forward_gate import forward_context


def test_guard_detects_obstacle_stale_depth_and_state_change(tmp_path: Path) -> None:
    """运动中最新深度、位姿或飞控状态异常必须触发守卫。"""
    context = forward_context(tmp_path)
    target = [0.2, 0.0, -1.0]
    assert flight._forward_motion_guard(context, target) is None
    near_depth = context.observation.depth.copy()
    near_depth[5, 7] = 0.5
    context.controller.latest_observation = lambda: replace(context.observation, depth=near_depth)
    assert flight._forward_motion_guard(context, target) == "FORWARD_CLEARANCE_SHRANK"
    context.controller.latest_observation = lambda: replace(
        context.observation, received_monotonic_ns=time.monotonic_ns() - 10_000_000_000)
    assert flight._forward_motion_guard(context, target) == "FORWARD_OBSERVATION_LOST"
    context.controller.latest_observation = lambda: replace(context.observation, pose_age_s=10.0)
    assert flight._forward_motion_guard(context, target) == "FORWARD_OBSERVATION_LOST"
    context.controller.vehicle_status.mode = "POSCTL"
    assert flight._forward_motion_guard(context, target) == "PX4_STATE_CHANGED"


def motion_context(tmp_path: Path):
    """给原 move 提供一个只记录目标点的空中飞控替身。"""
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
    controller.get_logger = Mock(return_value=SimpleNamespace(info=Mock()))
    return context


def test_guard_failure_after_setpoint_requests_confirmed_hover(tmp_path: Path, monkeypatch) -> None:
    """目标点发布后出现近障时必须确认 PX4 悬停，不能只发停止。"""
    context = motion_context(tmp_path)
    checks = iter([None, "FORWARD_CLEARANCE_SHRANK"])
    hover = Mock(return_value={"safety_state": "HOLD_CONFIRMED"})
    monkeypatch.setattr(flight, "request_confirmed_hover", hover)
    result = flight.move(context, 0.2, 0, 0, guard=lambda _target: next(checks))
    assert result["error"] == "FORWARD_GUARD_TRIGGERED"
    assert result["safety_state"] == "HOLD_CONFIRMED"
    context.controller.start_position_hold.assert_called_once_with([0.2, 0.0, -1.0])
    hover.assert_called_once()


def test_guard_rejection_before_setpoint_issues_no_target(tmp_path: Path) -> None:
    """发布位置目标前的失效直接拒绝，不触发危险位移。"""
    context = motion_context(tmp_path)
    result = flight.move(context, 0.2, 0, 0, guard=lambda _target: "DEPTH_MISSING")
    assert result["error"] == "FORWARD_GUARD_REJECTED"
    context.controller.start_position_hold.assert_not_called()


def test_hover_not_confirmed_escalates_to_offboard_loss_handoff(tmp_path: Path, monkeypatch) -> None:
    """PX4 未确认悬停时保留原安全异常并退出代理。"""
    context = motion_context(tmp_path)
    checks = iter([None, "FORWARD_OBSERVATION_LOST"])
    monkeypatch.setattr(flight, "request_confirmed_hover",
                        Mock(side_effect=SafetyHandoffRequired("SAFETY_HANDOFF_REQUIRED")))
    with pytest.raises(SafetyHandoffRequired):
        flight.move(context, 0.2, 0, 0, guard=lambda _target: next(checks))


def test_user_interrupt_during_forward_requests_confirmed_hover(tmp_path: Path, monkeypatch) -> None:
    """前进中的人工中断使用确认悬停而非单条 hover 命令。"""
    context = motion_context(tmp_path)
    pending = iter([False, True])
    context.message_bus = SimpleNamespace(
        has_pending_user_message=lambda: next(pending),
        get_next_user_message=lambda: SimpleNamespace(content="stop"),
    )
    hover = Mock(return_value={"safety_state": "HOLD_CONFIRMED"})
    monkeypatch.setattr(flight, "request_confirmed_hover", hover)
    result = flight.move(context, 0.2, 0, 0, guard=lambda _target: None)
    assert result["error"] == "INTERRUPTED_BY_USER"
    assert result["safety_state"] == "HOLD_CONFIRMED"
    hover.assert_called_once()


def test_short_forward_does_not_succeed_at_start_under_legacy_tolerance(tmp_path: Path) -> None:
    """0.2 m 短步不能被旧 0.3 m 到达容差在起点直接判成功。"""
    context = motion_context(tmp_path)
    context.controller.is_at_target = Mock(return_value=True)
    context.controller.current_position_ned = Mock(side_effect=[
        [0.0, 0.0, -1.0], [0.2, 0.0, -1.0], [0.2, 0.0, -1.0],
    ])
    result = flight.move(context, 0.2, 0, 0, guard=lambda _target: None)
    assert result["success"]
    assert context.controller.current_position_ned.call_count == 3
    context.controller.is_at_target.assert_not_called()
