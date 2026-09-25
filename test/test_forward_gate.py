"""验证前进安全门只接受同号、新鲜、限额内的正向请求。"""

from __future__ import annotations

import math
import time
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from drone_harness.tools import flight
from drone_harness.vision.depth_rules import compute_depth_rules
from test_agent_observation_loop import context_for
from test_depth_rules import snapshot_at


def forward_context(tmp_path: Path):
    """构造已在 OFFBOARD 的合成安全观测与飞控替身。"""
    snapshot = replace(snapshot_at(), pose_ned=(0.0, 0.0, -1.0), flight_state="IN_AIR", pose_age_s=0.01)
    controller = SimpleNamespace(
        vehicle_status=SimpleNamespace(mode="OFFBOARD", connected=True, armed=True),
        flight_state=lambda: "IN_AIR", latest_observation=lambda: snapshot,
    )
    context = context_for(tmp_path, controller)
    config = replace(context.profile.observation, braking_margin_m=0.2)
    context.profile = replace(context.profile, observation=config)
    context.observation = snapshot
    context.depth_rules = compute_depth_rules(snapshot, config, context.profile.safety.max_relative_move_m)
    return context


@pytest.mark.parametrize("distance", [None, True, 0, -0.1, math.nan, math.inf, 0.31])
def test_invalid_or_over_limit_forward_never_reaches_move(tmp_path: Path, monkeypatch, distance) -> None:
    """类型、非有限、负数及超动态上限均严格拒绝且不截短。"""
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
    assert callable(kwargs["guard"])


def test_old_or_mismatched_depth_blocks_forward(tmp_path: Path) -> None:
    """观测过期、变号或深度语义未知均给零动作。"""
    context = forward_context(tmp_path)
    old = replace(context.observation, rgb_stamp_ns=time.time_ns() - 10_000_000_000)
    context.observation = old
    assert flight.validate_forward(context, 0.1)["error"] == "FORWARD_DEPTH_STALE_OR_INVALID"
    context = forward_context(tmp_path)
    context.controller.latest_observation = lambda: replace(context.observation, observation_id="newer")
    assert flight.validate_forward(context, 0.1)["error"] == "FORWARD_OBSERVATION_CHANGED"
    context = forward_context(tmp_path)
    context.depth_rules = replace(context.depth_rules, depth_valid=False, forward_max_m=0.0)
    assert flight.validate_forward(context, 0.1)["error"] == "FORWARD_DEPTH_STALE_OR_INVALID"


def test_current_limit_shrinking_rejects_even_when_old_limit_allowed(tmp_path: Path) -> None:
    """执行前以最新深度重新计算限额而不信任缓存的较大值。"""
    context = forward_context(tmp_path)
    near_depth = context.observation.depth.copy()
    near_depth[5, 7] = 0.5
    latest = replace(context.observation, depth=near_depth)
    context.controller.latest_observation = lambda: latest
    assert flight.validate_forward(context, 0.2)["error"] == "FORWARD_LIMIT_CHANGED"


def test_unverified_real_profile_depth_cannot_authorize_forward(tmp_path: Path) -> None:
    """真实相机语义未核实前，即使给出合成数组也不能前进。"""
    context = forward_context(tmp_path)
    config = replace(context.profile.observation, depth_semantics="unverified")
    context.profile = replace(context.profile, mode="real", observation=config)
    context.depth_rules = compute_depth_rules(context.observation, config, 0.3)
    assert flight.validate_forward(context, 0.1)["error"] == "FORWARD_DEPTH_STALE_OR_INVALID"
