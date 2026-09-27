"""以固定故障矩阵验证自主前进和续轮在失效时停住。"""

from __future__ import annotations

import time
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

import drone_harness.runtime.agent_loop as loop_module
from drone_harness.runtime.agent_loop import agent_loop, append_observation
from drone_harness.runtime.observation import ObservationBuffer
from drone_harness.tools import flight
from drone_harness.vision.depth_rules import compute_depth_rules
from test_agent_observation_loop import FakeClient, context_for, fake_call
from test_depth_rules import snapshot_at
from test_forward_gate import forward_context, fresh_snapshot
from test_observation_buffer import add_pair


def test_actual_sim_profile_uses_verified_meter_depth_but_not_scene_7(
    tmp_path: Path, monkeypatch,
) -> None:
    """仿真启用已抽测场景米制解释，决策视距与单步限额独立。"""
    context = context_for(tmp_path)
    snapshot = snapshot_at()
    context.observation = snapshot
    context.depth_rules = compute_depth_rules(snapshot, context.profile.observation,
                                              context.profile.forward_step_limit_m)

    assert context.profile.observation.depth_semantics == "perspective_ray_m"
    assert context.profile.observation.depth_max_m == 20
    assert context.profile.forward_step_limit_m == 19
    assert context.depth_rules.depth_valid
    assert context.depth_rules.observation_id == snapshot.observation_id
    assert context.depth_rules.forward_max_m > 0


@pytest.mark.parametrize("fault", [
    "depth_missing", "depth_unsynced", "depth_unknown", "depth_unit_unverified", "near_obstacle",
])
def test_depth_faults_issue_no_forward_setpoint(
    tmp_path: Path, monkeypatch, fault: str,
) -> None:
    """本次取图缺失、错配、未知单位或一米内障碍均零前进。"""
    context = forward_context(tmp_path)
    if fault == "depth_unit_unverified":
        context.profile = replace(context.profile, observation=replace(
            context.profile.observation, depth_semantics="unverified"))

    def wait_for_observation(*, after_stamp_ns: int):
        """把各类故障注入本次 forward 新取得的深度帧。"""
        snapshot = fresh_snapshot(after_stamp_ns)
        if fault == "depth_missing":
            return replace(snapshot, depth=None, depth_stamp_ns=None, depth_error="DEPTH_MISSING")
        if fault == "depth_unsynced":
            return replace(snapshot, depth_error="DEPTH_UNSYNCED")
        if fault in {"depth_unknown", "near_obstacle"}:
            depth = snapshot.depth.copy()
            depth[5, 7] = np.nan if fault == "depth_unknown" else 0.5
            return replace(snapshot, depth=depth)
        return snapshot

    context.controller.wait_for_observation = wait_for_observation
    move = Mock()
    monkeypatch.setattr(flight, "move", move)

    result = flight.forward(context, 0.1)
    assert result["success"] and not result["motion_executed"]
    assert result["commanded_distance_m"] == 0
    assert "未移动" in result["message"]
    assert result["forward_max_m"] == 0
    move.assert_not_called()


def test_stale_rgb_is_rejected_when_a_is_acquired(tmp_path: Path) -> None:
    """A 采集时的旧 RGB 不进观测缓冲，模型等待时间不参与此判定。"""
    context = forward_context(tmp_path)
    buffer = ObservationBuffer(context.profile.observation)
    add_pair(buffer, time.time_ns() - 10_000_000_000)
    assert buffer.latest_snapshot() is None


@pytest.mark.parametrize("fault", ["timeout", "malformed", "parallel_calls"])
def test_bad_model_reply_stops_without_dispatch_and_confirms_hover(
    tmp_path: Path, monkeypatch, fault: str,
) -> None:
    """服务超时、畸形及多调用均只请求模型一次且不触达飞控。"""
    context = forward_context(tmp_path)
    messages = [{"role": "system", "content": "test"}, {"role": "user", "content": "go"}]
    append_observation(context, messages, context.observation)
    if fault == "timeout":
        create = Mock(side_effect=TimeoutError("request timed out"))
        client = SimpleNamespace(chat=SimpleNamespace(completions=SimpleNamespace(create=create)))
    elif fault == "malformed":
        client = FakeClient([SimpleNamespace(content="", tool_calls=[SimpleNamespace(
            function=SimpleNamespace(name="forward", arguments='{"distance_m":0.1}'))])])
    else:
        client = FakeClient([SimpleNamespace(content="", tool_calls=[
            fake_call("rotate", "one"), fake_call("forward", "two")])])
    dispatch = Mock()
    hover = Mock(return_value={"safety_state": "HOLD_CONFIRMED"})
    monkeypatch.setattr(loop_module, "dispatch_tool_call", dispatch)
    monkeypatch.setattr(loop_module, "request_confirmed_hover", hover)

    answer = agent_loop(client, "test-vlm", messages, context)
    assert answer
    assert (create.call_count if fault == "timeout" else len(client.requests)) == 1
    assert context.observation.observation_id not in messages[2]["content"][0]["text"]
    assert context.depth_rules.observation_id == context.observation.observation_id
    dispatch.assert_not_called()
    hover.assert_called_once()


@pytest.mark.parametrize("fault", ["missing", "not_new"])
def test_explicit_observe_failure_stops_without_reusing_old_image(
    tmp_path: Path, monkeypatch, fault: str,
) -> None:
    """动作后不自动取图；主动观察失败时旧图不能续用。"""
    controller = SimpleNamespace(
        wait_for_observation=lambda **_kwargs: None if fault == "missing" else snapshot_at(),
        vehicle_status=SimpleNamespace(mode="OFFBOARD"), flight_state=lambda: "IN_AIR")
    context = context_for(tmp_path, controller)
    messages = [{"role": "system", "content": "test"}, {"role": "user", "content": "go"}]
    append_observation(context, messages, snapshot_at())
    observe_call = SimpleNamespace(id="observe-2", function=SimpleNamespace(
        name="observe", arguments='{"prompt":"查看新位置"}'))
    client = FakeClient([SimpleNamespace(content="", tool_calls=[fake_call("rotate")]),
                         SimpleNamespace(content="", tool_calls=[observe_call])])
    original_dispatch = loop_module.dispatch_tool_call

    def dispatch(current, call):
        """转向用假结果，观察仍走实际工具分发和取图。"""
        if call.function.name == "rotate":
            return {"success": True, "degrees": 15.0}
        return original_dispatch(current, call)

    hover = Mock(return_value={"safety_state": "HOLD_CONFIRMED"})
    monkeypatch.setattr(loop_module, "dispatch_tool_call", dispatch)
    monkeypatch.setattr(loop_module, "request_confirmed_hover", hover)

    assert "observe 未成功" in agent_loop(client, "test-vlm", messages, context)
    assert len(client.requests) == 2
    hover.assert_called_once()
    assert context.observation is None
