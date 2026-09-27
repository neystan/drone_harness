"""验证仿真前进每次读取新深度，真机保留原严格安全门。"""

from __future__ import annotations

import math
import time
from dataclasses import replace
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from drone_harness.tools import flight
from drone_harness.runtime.tool_dispatcher import dispatch_tool_call
from drone_harness.vision.depth_rules import compute_depth_rules
from test_agent_observation_loop import context_for
from test_depth_rules import snapshot_at


def fresh_snapshot(after_stamp_ns: int, distance_m: float = 4.0):
    """构造采集时间晚于前进调用的 RGB-D。"""
    source = snapshot_at(distance_m)
    stamp_ns = after_stamp_ns + 60_000_000
    return replace(source, observation_id=f"rgb-{stamp_ns}", rgb_stamp_ns=stamp_ns,
                   depth_stamp_ns=stamp_ns,
                   intrinsics=replace(source.intrinsics, stamp_ns=stamp_ns))


def near_snapshot(after_stamp_ns: int):
    """构造前方约 1.5 米细障和其他方向的远值。"""
    source = fresh_snapshot(after_stamp_ns, 65504.0)
    depth = source.depth.copy()
    depth[5, 7] = 1.55
    return replace(source, depth=depth,
                   intrinsics=replace(source.intrinsics, fx=32.0, fy=32.0))


def forward_context(tmp_path: Path):
    """构造已在 OFFBOARD 的合成安全观测与飞控替身。"""
    snapshot = replace(snapshot_at(), pose_ned=(0.0, 0.0, -1.0), flight_state="IN_AIR", pose_age_s=0.01)
    controller = SimpleNamespace(
        vehicle_status=SimpleNamespace(mode="OFFBOARD", connected=True, armed=True),
        flight_state=lambda: "IN_AIR", latest_observation=lambda: snapshot,
        wait_for_observation=lambda *, after_stamp_ns: fresh_snapshot(after_stamp_ns),
    )
    context = context_for(tmp_path, controller)
    config = context.profile.observation
    context.profile = replace(context.profile, observation=config)
    context.observation = snapshot
    context.depth_rules = compute_depth_rules(snapshot, config, context.profile.forward_step_limit_m)
    return context


@pytest.mark.parametrize("distance", [None, True, 0, -0.1, math.nan, math.inf])
def test_invalid_forward_never_reaches_move(tmp_path: Path, monkeypatch, distance) -> None:
    """类型、非有限和非正数仍严格拒绝，不转成零位移。"""
    context = forward_context(tmp_path)
    calls = Mock()
    monkeypatch.setattr(flight, "move", calls)
    result = flight.forward(context, distance)
    assert not result["success"]
    calls.assert_not_called()


def test_valid_forward_uses_only_positive_x_and_zero_y_z(tmp_path: Path, monkeypatch) -> None:
    """模型无法通过 forward 请求侧移、下降或后退。"""
    context = forward_context(tmp_path)
    calls = Mock(return_value={"success": True})
    monkeypatch.setattr(flight, "move", calls)
    assert flight.forward(context, 0.2)["success"]
    args, kwargs = calls.call_args
    assert args[1:] == (0.2, 0.0, 0.0)
    assert kwargs == {"completion_tolerance_m": 0.05}


def test_model_latency_does_not_replace_forward_time_depth(tmp_path: Path, monkeypatch) -> None:
    """模型思考七秒后仍在调用 forward 时另取一帧新深度。"""
    context = forward_context(tmp_path)
    context.observation = replace(context.observation,
                                  rgb_stamp_ns=time.time_ns() - 10_000_000_000,
                                  depth_stamp_ns=time.time_ns() - 10_000_000_000)
    wait = Mock(side_effect=lambda *, after_stamp_ns: fresh_snapshot(after_stamp_ns))
    context.controller.wait_for_observation = wait
    monkeypatch.setattr(flight, "move", Mock(return_value={"success": True}))
    assert flight.forward(context, 0.1)["success"]
    wait.assert_called_once()


def test_old_a_mismatch_is_ignored_but_new_invalid_depth_returns_no_motion(tmp_path: Path, monkeypatch) -> None:
    """仿真不拿旧 A 授权；调用时的新深度失效则零指令。"""
    context = forward_context(tmp_path)
    context.depth_rules = replace(context.depth_rules, observation_id="other")
    assert flight.validate_forward(context, 0.1) is None
    context.controller.wait_for_observation = lambda *, after_stamp_ns: replace(
        fresh_snapshot(after_stamp_ns), depth_error="DEPTH_UNSYNCED")
    calls = Mock()
    monkeypatch.setattr(flight, "move", calls)
    result = flight.forward(context, 0.1)
    assert result["success"] and not result["motion_executed"]
    assert result["requested_distance_m"] == 0.1
    assert result["commanded_distance_m"] == 0
    assert "未移动" in result["message"]
    calls.assert_not_called()


def test_new_sensor_frame_recalculates_forward_limit(tmp_path: Path, monkeypatch) -> None:
    """即使旧 A 显示远景，本次前进仍使用新帧的近障上限。"""
    context = forward_context(tmp_path)
    context.depth_rules = replace(context.depth_rules, forward_max_m=19.0)
    context.controller.wait_for_observation = lambda *, after_stamp_ns: near_snapshot(after_stamp_ns)
    move = Mock(return_value={"success": True, "message": "move complete"})
    monkeypatch.setattr(flight, "move", move)
    result = flight.forward(context, 5.0)
    assert 0.48 < result["commanded_distance_m"] < 0.51
    assert move.call_args.args[1] == result["commanded_distance_m"]


@pytest.mark.parametrize("mode", ["MANUAL", "POSCTL", "AUTO.RTL"])
def test_sim_forward_rejects_other_modes(tmp_path: Path, mode: str) -> None:
    """仅悬停模式可复用握手，其他模式不自动改为 OFFBOARD。"""
    context = forward_context(tmp_path)
    context.controller.vehicle_status.mode = mode
    assert flight.validate_forward(context, 0.2)["error"] == "PX4_STATE_INVALID"


@pytest.mark.parametrize("connected,armed", [(False, True), (True, False)])
def test_sim_loiter_requires_connection_and_arming(tmp_path: Path, connected: bool, armed: bool) -> None:
    """悬停恢复不能越过连接与解锁检查。"""
    context = forward_context(tmp_path)
    context.controller.vehicle_status.mode = "AUTO.LOITER"
    context.controller.vehicle_status.connected = connected
    context.controller.vehicle_status.armed = armed
    assert flight.validate_forward(context, 0.2)["error"] == "PX4_STATE_INVALID"


def test_real_forward_still_rejects_loiter(tmp_path: Path) -> None:
    """真机保持原有模式要求，不能绕过逐动作确认流程。"""
    context = forward_context(tmp_path)
    context.profile = replace(context.profile, mode="real")
    context.controller.vehicle_status.mode = "AUTO.LOITER"
    assert flight.validate_forward(context, 0.2)["error"] == "PX4_STATE_INVALID"


def test_sim_forward_clamps_to_new_depth_limit_and_profile_cap(tmp_path: Path, monkeypatch) -> None:
    """仿真超本次新深度或 19 米上限时按较小值下发并说明。"""
    context = forward_context(tmp_path)
    context.controller.wait_for_observation = lambda *, after_stamp_ns: near_snapshot(after_stamp_ns)
    move = Mock(return_value={"success": True, "message": "move complete"})
    monkeypatch.setattr(flight, "move", move)
    assert context.profile.forward_step_limit_m == 19
    assert context.profile.safety.max_relative_move_m == 0.3
    assert flight.validate_forward(context, 0.25) is None
    result = flight.forward(context, 2.0)
    assert result["success"] and result["clamped"]
    assert result["requested_distance_m"] == 2.0
    assert 0.48 < result["commanded_distance_m"] < 0.51
    assert "1.50 米有障碍物" in result["message"]
    assert "1 米" in result["message"]
    assert move.call_args.args[1:] == (result["commanded_distance_m"], 0.0, 0.0)

    context.controller.wait_for_observation = lambda *, after_stamp_ns: replace(
        fresh_snapshot(after_stamp_ns, 65504.0),
        intrinsics=replace(fresh_snapshot(after_stamp_ns).intrinsics, fx=100000.0, fy=100000.0))
    result = flight.forward(context, 30.0)
    assert 18.9 < result["commanded_distance_m"] <= 19.0
    assert move.call_args.args[1:] == (result["commanded_distance_m"], 0.0, 0.0)


def test_real_forward_keeps_strict_rejection(tmp_path: Path, monkeypatch) -> None:
    """真机即使继承仿真测试替身也不放宽限额或自动截短。"""
    context = forward_context(tmp_path)
    context.profile = replace(context.profile, mode="real", safety=replace(
        context.profile.safety, max_relative_move_m=0.2))
    context.depth_rules = replace(context.depth_rules, forward_max_m=1.0)
    move = Mock()
    monkeypatch.setattr(flight, "move", move)
    assert context.profile.forward_step_limit_m == 0.2
    assert flight.forward(context, 0.21)["error"] == "FORWARD_LIMIT_EXCEEDED"
    move.assert_not_called()


def test_dispatch_returns_clamp_feedback_in_normal_tool_result(tmp_path: Path, monkeypatch) -> None:
    """超限提议沿原工具分发链返回可读缩短结果，不触发异常终止。"""
    context = forward_context(tmp_path)
    context.controller.wait_for_observation = lambda *, after_stamp_ns: near_snapshot(after_stamp_ns)
    move = Mock(return_value={"success": True, "message": "move complete"})
    monkeypatch.setattr(flight, "move", move)
    call = SimpleNamespace(function=SimpleNamespace(name="forward", arguments=json.dumps({"distance_m": 2.0})))
    result = dispatch_tool_call(context, call)
    assert result["success"] and result["clamped"]
    assert result["requested_distance_m"] == 2
    assert 0.48 < result["commanded_distance_m"] < 0.51
    assert context.task_state.last_tool_result == result
    move.assert_called_once()


def test_runtime_prompt_explains_sim_and_real_limits(tmp_path: Path) -> None:
    """限额移入工具参数说明，系统提示只解释反馈与真机审批。"""
    from drone_harness.llm.prompts import build_system_prompt
    from drone_harness.tools.schemas import get_tool_schemas

    context = forward_context(tmp_path)
    sim_prompt = build_system_prompt(context.profile)
    assert "19 米" not in sim_prompt and "约 1 米" not in sim_prompt
    assert "缩短" in sim_prompt and "不执行" in sim_prompt
    assert "19 米" in str(get_tool_schemas(context.profile))
    real_profile = replace(context.profile, mode="real", safety=replace(
        context.profile.safety, max_relative_move_m=0.2))
    real_prompt = build_system_prompt(real_profile)
    assert "每次均须人工确认" in real_prompt
    assert "0.2 米" in str(get_tool_schemas(real_profile))
    assert "19 米" not in real_prompt


@pytest.mark.parametrize("invalid_limit", [math.nan, math.inf, -0.1])
def test_invalid_fresh_depth_limit_fails_closed(tmp_path: Path, monkeypatch, invalid_limit: float) -> None:
    """本次深度上限异常时不能利用浮点比较漏洞授权前进。"""
    context = forward_context(tmp_path)
    broken = replace(context.depth_rules, forward_max_m=invalid_limit)
    monkeypatch.setattr(flight, "compute_depth_rules", lambda snapshot, *_args: replace(
        broken, observation_id=snapshot.observation_id))
    move = Mock()
    monkeypatch.setattr(flight, "move", move)
    result = flight.forward(context, 0.1)
    assert result["success"] and result["commanded_distance_m"] == 0
    assert result["depth_reason"] == "FORWARD_DEPTH_LIMIT_INVALID"
    move.assert_not_called()


def test_unverified_real_profile_depth_cannot_authorize_forward(tmp_path: Path) -> None:
    """真实相机语义未核实前，即使给出合成数组也不能前进。"""
    context = forward_context(tmp_path)
    config = replace(context.profile.observation, depth_semantics="unverified")
    context.profile = replace(context.profile, mode="real", observation=config)
    context.depth_rules = compute_depth_rules(context.observation, config, 0.3)
    assert flight.validate_forward(context, 0.1)["error"] == "FORWARD_DEPTH_INVALID"
