"""验证四动作 HITL、降落授权与批准后状态复核。"""

from __future__ import annotations

import json
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
                     human_in_the_loop_exempt_flight_tools=frozenset({"rotate", "land"}))
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
    assert "观测号=" in prompt and "上限=" in prompt and "有效期至=" in prompt
    if name == "land":
        assert context.task_state.landing_authorized


def test_approval_rejects_depth_limit_change_before_execution(tmp_path: Path, monkeypatch) -> None:
    """等待人工期间最新深度缩小上限，即使回复 Y 也零动作。"""
    context = real_context(tmp_path)
    latest = [context.observation]
    context.controller.latest_observation = lambda: latest[0]
    near_depth = context.observation.depth.copy()
    near_depth[5, 7] = 0.5

    def answer_after_depth_change():
        """模拟用户批准到来前近障进入相机视野。"""
        latest[0] = replace(context.observation, depth=near_depth)
        return SimpleNamespace(content="y")

    context.message_bus = SimpleNamespace(get_next_user_message=answer_after_depth_change,
                                          has_pending_user_message=lambda: False)
    handler = Mock(return_value={"success": True})
    monkeypatch.setattr(dispatcher, "get_tool_definition", lambda _name: SimpleNamespace(handler=handler))
    result = dispatcher.dispatch_tool_call(context, tool_call("forward"))
    assert result["error"] == "APPROVAL_STATE_CHANGED"
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
