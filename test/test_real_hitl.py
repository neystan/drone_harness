"""验证四动作 HITL、降落授权与批准后状态复核。"""

from __future__ import annotations

import json
import time
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

import drone_harness.runtime.tool_dispatcher as dispatcher
from drone_harness.runtime.safety import EndCurrentTurn, requires_human_in_the_loop
from test_forward_gate import forward_context


def real_context(tmp_path: Path):
    """构造只在测试中使用的已校准真机 profile 替身。"""
    context = forward_context(tmp_path)
    safety = replace(context.profile.safety, human_in_the_loop_for_flight_tools=True,
                     human_in_the_loop_exempt_flight_tools=frozenset({"rotate", "land"}),
                     max_relative_move_m=0.2)
    context.profile = replace(context.profile, mode="real", safety=safety)
    context.task_state.start_new_goal("test")
    return context


def tool_call(name: str) -> SimpleNamespace:
    """提供符合模型响应形状的单个工具调用。"""
    arguments = {"takeoff": {"height": 1.0}, "forward": {"distance_m": 0.2},
                 "rotate": {"direction": "left", "degrees": 15}, "land": {}}[name]
    return SimpleNamespace(function=SimpleNamespace(name=name, arguments=json.dumps(arguments)))


@pytest.mark.parametrize("name", ["takeoff", "forward", "rotate", "land"])
def test_real_four_actions_all_require_individual_approval(
    tmp_path: Path, monkeypatch, capsys, name: str,
) -> None:
    """历史豁免也不能绕过真机四动作的逐次批准。"""
    context = real_context(tmp_path)
    answers = iter([SimpleNamespace(content="y")])
    context.message_bus = SimpleNamespace(get_next_user_message=lambda: next(answers),
                                          has_pending_user_message=lambda: False)
    handler = Mock(return_value={"success": True})
    monkeypatch.setattr(dispatcher, "get_tool_definition", lambda _name: SimpleNamespace(handler=handler))
    assert requires_human_in_the_loop(context.profile, name)
    result = dispatcher.dispatch_tool_call(context, tool_call(name))
    assert result["success"]
    handler.assert_called_once()
    prompt = capsys.readouterr().out
    assert f"动作={name}" in prompt
    assert "观测号=" in prompt and "上限=" in prompt and "确认期限至=" in prompt
    if name == "forward":
        assert "上限=0.20m" in prompt
    if name == "land":
        assert context.task_state.landing_authorized


def test_approval_does_not_recalculate_new_sensor_frame(tmp_path: Path, monkeypatch) -> None:
    """审批期间后台新帧变化不替换 VLM 已使用的 A 规则。"""
    context = real_context(tmp_path)
    context.controller.latest_observation = Mock(side_effect=AssertionError("B frame was fetched"))
    context.message_bus = SimpleNamespace(get_next_user_message=lambda: SimpleNamespace(content="y"),
                                          has_pending_user_message=lambda: False)
    handler = Mock(return_value={"success": True})
    monkeypatch.setattr(dispatcher, "get_tool_definition", lambda _name: SimpleNamespace(handler=handler))
    result = dispatcher.dispatch_tool_call(context, tool_call("forward"))
    assert result["success"]
    handler.assert_called_once()


def test_approval_rejects_bound_a_limit_change(tmp_path: Path, monkeypatch) -> None:
    """人工确认期间 A 的上限被替换时拒绝执行。"""
    context = real_context(tmp_path)

    def answer_after_a_change():
        """模拟审批等待期间 A 规则被替换。"""
        context.depth_rules = replace(context.depth_rules, forward_max_m=0.0)
        return SimpleNamespace(content="y")

    context.message_bus = SimpleNamespace(get_next_user_message=answer_after_a_change,
                                          has_pending_user_message=lambda: False)
    handler = Mock(return_value={"success": True})
    monkeypatch.setattr(dispatcher, "get_tool_definition", lambda _name: SimpleNamespace(handler=handler))
    result = dispatcher.dispatch_tool_call(context, tool_call("forward"))
    assert result["error"] == "APPROVAL_STATE_CHANGED"
    handler.assert_not_called()


def test_approval_does_not_use_a_frame_age_as_deadline(tmp_path: Path, monkeypatch) -> None:
    """模型思考十秒后仍能人工批准，确认期限从提问时起算。"""
    context = real_context(tmp_path)
    context.observation = replace(context.observation,
                                  rgb_stamp_ns=time.time_ns() - 10_000_000_000,
                                  depth_stamp_ns=time.time_ns() - 10_000_000_000)
    context.message_bus = SimpleNamespace(get_next_user_message=lambda: SimpleNamespace(content="y"),
                                          has_pending_user_message=lambda: False)
    handler = Mock(return_value={"success": True})
    monkeypatch.setattr(dispatcher, "get_tool_definition", lambda _name: SimpleNamespace(handler=handler))
    assert dispatcher.dispatch_tool_call(context, tool_call("forward"))["success"]
    handler.assert_called_once()


def test_approval_interaction_timeout_stops_without_action(tmp_path: Path, monkeypatch) -> None:
    """人工确认自身过期时不执行动作，与 A 的采集时间无关。"""
    context = real_context(tmp_path)
    context.message_bus = SimpleNamespace(get_next_user_message=lambda: None,
                                          has_pending_user_message=lambda: False)
    handler = Mock(return_value={"success": True})
    monkeypatch.setattr(dispatcher, "get_tool_definition", lambda _name: SimpleNamespace(handler=handler))
    monkeypatch.setattr(dispatcher, "HITL_CONFIRM_TIMEOUT_S", 0.001)
    with pytest.raises(EndCurrentTurn) as captured:
        dispatcher.dispatch_tool_call(context, tool_call("forward"))
    assert captured.value.tool_result["error"] == "HUMAN_IN_THE_LOOP_EXPIRED"
    handler.assert_not_called()


def test_land_requires_explicit_human_authorization_even_in_sim(tmp_path: Path, monkeypatch) -> None:
    """模型声称候选完成不能绕过单独降落确认。"""
    context = forward_context(tmp_path)
    context.task_state.completion_candidate = "候选完成"
    context.message_bus = SimpleNamespace(get_next_user_message=lambda: SimpleNamespace(content="n"),
                                          has_pending_user_message=lambda: False)
    handler = Mock(return_value={"success": True})
    monkeypatch.setattr(dispatcher, "get_tool_definition", lambda _name: SimpleNamespace(handler=handler))
    with pytest.raises(EndCurrentTurn):
        dispatcher.dispatch_tool_call(context, tool_call("land"))
    handler.assert_not_called()
    assert not context.task_state.landing_authorized


def test_land_without_approval_channel_cannot_execute(tmp_path: Path, monkeypatch) -> None:
    """没有人工确认通道时降落提议必须 fail-closed。"""
    context = forward_context(tmp_path)
    handler = Mock(return_value={"success": True})
    monkeypatch.setattr(dispatcher, "get_tool_definition", lambda _name: SimpleNamespace(handler=handler))
    with pytest.raises(EndCurrentTurn):
        dispatcher.dispatch_tool_call(context, tool_call("land"))
    handler.assert_not_called()


def test_actual_real_profile_has_no_flight_exemptions() -> None:
    """真实入口的加载配置不能保留旧 rotate/land 豁免。"""
    from drone_harness.config.loader import load_profile

    settings = Path(__file__).parents[1] / "settings.example.json"
    profile = load_profile("real", settings_path=settings)
    assert profile.safety.human_in_the_loop_exempt_flight_tools == frozenset()
    assert profile.safety.max_relative_move_m < 0.3
    assert all(requires_human_in_the_loop(profile, name)
               for name in ("takeoff", "forward", "rotate", "land"))


def test_approval_rejects_px4_state_change_before_execution(tmp_path: Path, monkeypatch) -> None:
    """人在确认期间 PX4 退出 OFFBOARD 时，不得沿用旧审批执行。"""
    context = real_context(tmp_path)

    def answer_after_mode_change():
        """模拟审批等待期间飞控模式发生变化。"""
        context.controller.vehicle_status.mode = "POSCTL"
        return SimpleNamespace(content="y")

    context.message_bus = SimpleNamespace(get_next_user_message=answer_after_mode_change,
                                          has_pending_user_message=lambda: False)
    handler = Mock(return_value={"success": True})
    monkeypatch.setattr(dispatcher, "get_tool_definition", lambda _name: SimpleNamespace(handler=handler))
    result = dispatcher.dispatch_tool_call(context, tool_call("rotate"))
    assert result["error"] == "APPROVAL_STATE_CHANGED"
    handler.assert_not_called()
