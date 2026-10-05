"""验证动作后的稳定等待发生在下一次规划与观察之前。"""

from types import SimpleNamespace
from unittest.mock import Mock
from dataclasses import replace
from pathlib import Path

import pytest
import yaml

import drone_harness.runtime.agent_loop as loop_module
from drone_harness.config.loader import _build_profile
from drone_harness.tools import flight
from test_agent_observation_loop import FakeClient, context_for
from test_stage3_observe import fresh_snapshot, tool_call


@pytest.mark.parametrize("name", ["takeoff", "forward", "up", "down", "rotate"])
def test_motion_waits_two_seconds_before_next_model_request(tmp_path, monkeypatch, name):
    """每种成功运动先等待两秒，再把结果交给下一次模型请求。"""
    events = []
    context = context_for(tmp_path)
    responses = [
        SimpleNamespace(content="", tool_calls=[tool_call(name, {})]),
        SimpleNamespace(content="继续观察", tool_calls=[]),
    ]

    def create(**_kwargs):
        """记录模型请求顺序，避免调用真实模型。"""
        events.append("model")
        return SimpleNamespace(choices=[SimpleNamespace(message=responses.pop(0))])

    def dispatch(*_args):
        """模拟已成功执行的运动并标记完成时刻。"""
        events.append("motion_done")
        return {"success": True, "motion_executed": True, "degrees": 30.0}

    monkeypatch.setattr(loop_module, "dispatch_tool_call", dispatch)
    monkeypatch.setattr(loop_module, "time", SimpleNamespace(sleep=lambda seconds: events.append(("sleep", seconds))))
    client = SimpleNamespace(chat=SimpleNamespace(completions=SimpleNamespace(create=create)))
    loop_module.agent_loop(client, "test", [{"role": "user", "content": "导航"}], context)
    assert events == ["model", "motion_done", ("sleep", 2.0), "model"]


@pytest.mark.parametrize("name,result", [
    ("forward", {"success": True, "motion_executed": False, "commanded_distance_m": 0}),
    ("rotate", {"success": True, "degrees": 0}),
    ("get_state", {"success": True, "in_air": None}),
    ("up", {"success": False, "error": "PX4_STATE_INVALID"}),
    ("land", {"success": True}),
])
def test_non_motion_failure_and_land_do_not_wait(tmp_path, monkeypatch, name, result):
    """零动作、查询、失败和降落不额外添加稳定等待。"""
    context = context_for(tmp_path)
    sleep = Mock()
    monkeypatch.setattr(loop_module, "time", SimpleNamespace(sleep=sleep))
    monkeypatch.setattr(loop_module, "dispatch_tool_call", lambda *_args: dict(result))
    client = FakeClient([
        SimpleNamespace(content="", tool_calls=[tool_call(name, {})]),
        SimpleNamespace(content="已收到", tool_calls=[]),
    ])
    loop_module.agent_loop(client, "test", [{"role": "user", "content": "导航"}], context)
    sleep.assert_not_called()


def test_observe_takes_new_frame_only_after_motion_settle(tmp_path, monkeypatch):
    """转向后的 observe 必须在两秒等待结束后才能开始取新帧。"""
    events = []

    def rotate(*_args):
        """模拟转向完成，实际不连接飞控。"""
        events.append("rotated")
        return {"success": True, "degrees": 30.0}

    def capture(*, after_stamp_ns):
        """检查等待已结束，返回调用之后采集的新观测。"""
        assert events == ["rotated", ("sleep", 2.0)]
        events.append("capture")
        return fresh_snapshot(after_stamp_ns)

    context = context_for(tmp_path, SimpleNamespace(wait_for_observation=capture))
    monkeypatch.setattr(flight, "rotate", rotate)
    monkeypatch.setattr(loop_module, "time", SimpleNamespace(sleep=lambda seconds: events.append(("sleep", seconds))))
    client = FakeClient([
        SimpleNamespace(content="", tool_calls=[tool_call("rotate", {"direction": "left", "degrees": 30})]),
        SimpleNamespace(content="", tool_calls=[tool_call("observe", {"prompt": "重新寻找路口"})]),
        SimpleNamespace(content="看到路口了", tool_calls=[]),
    ])
    loop_module.agent_loop(client, "test", [{"role": "user", "content": "找路口"}], context)
    assert events == ["rotated", ("sleep", 2.0), "capture"]
    assert client.requests[2][-1]["content"][1]["type"] == "image_url"


def test_switch_off_skips_wait_and_still_continues_planning(tmp_path, monkeypatch):
    """关闭开关后动作成功立即继续下一次规划。"""
    context = context_for(tmp_path)
    context.profile = replace(context.profile, post_motion_wait_enabled=False)
    sleep = Mock()
    monkeypatch.setattr(loop_module, "time", SimpleNamespace(sleep=sleep))
    monkeypatch.setattr(loop_module, "dispatch_tool_call", lambda *_args: {"success": True, "degrees": 30})
    client = FakeClient([
        SimpleNamespace(content="", tool_calls=[tool_call("rotate", {})]),
        SimpleNamespace(content="继续", tool_calls=[]),
    ])
    loop_module.agent_loop(client, "test", [{"role": "user", "content": "找路口"}], context)
    sleep.assert_not_called()
    assert len(client.requests) == 2


@pytest.mark.parametrize("enabled", [True, False])
def test_yaml_switch_is_loaded_into_runtime_profile(enabled):
    """YAML 开关值能传入运行配置，不被固定开启。"""
    root = Path(__file__).parents[1]
    raw = yaml.safe_load((root / "drone_harness/config/profiles/sim.yaml").read_text())
    raw["post_motion_wait_enabled"] = enabled
    settings = {"llm": {"api_key": "test-only", "base_url": "https://example.test/v4", "model": "test"}}
    assert _build_profile(raw, settings).post_motion_wait_enabled is enabled
