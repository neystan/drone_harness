"""验证同一模型收图、收七工具及服务异常时零后续动作。"""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import drone_harness.runtime.agent_loop as loop_module
from drone_harness.runtime.agent_loop import agent_loop, append_observation
from drone_harness.tools.registry import get_tool_schemas
from test_agent_observation_loop import FakeClient, context_for
from test_depth_rules import snapshot_at


def test_one_request_contains_image_depth_summary_and_seven_tools(tmp_path: Path) -> None:
    """单一请求可携带按需图文与观察加六个飞行动作。"""
    context = context_for(tmp_path)
    messages = [{"role": "system", "content": "test"}, {"role": "user", "content": "go"}]
    append_observation(context, messages, snapshot_at())
    client = FakeClient([SimpleNamespace(content="已观察", tool_calls=[])])
    agent_loop(client, "glm-5.3-flash", messages, context)
    request = client.requests[0]
    assert request[-1]["content"][0]["type"] == "text"
    assert "forward_max=" in request[-1]["content"][0]["text"]
    assert request[-1]["content"][1]["type"] == "image_url"
    assert {schema["function"]["name"] for schema in get_tool_schemas()} == {
        "observe", "takeoff", "forward", "up", "down", "rotate", "land"}


def test_service_failure_causes_confirmed_hover_and_no_tool_dispatch(tmp_path: Path, monkeypatch) -> None:
    """空中服务异常不回退到无图规划，而是确认悬停并停止。"""
    controller = SimpleNamespace(vehicle_status=SimpleNamespace(mode="OFFBOARD"),
                                 flight_state=lambda: "IN_AIR")
    context = context_for(tmp_path, controller)
    messages = [{"role": "system", "content": "test"}, {"role": "user", "content": "go"}]
    append_observation(context, messages, snapshot_at())
    hover = Mock(return_value={"safety_state": "HOLD_CONFIRMED"})
    dispatch = Mock()
    monkeypatch.setattr(loop_module, "request_confirmed_hover", hover)
    monkeypatch.setattr(loop_module, "dispatch_tool_call", dispatch)

    class BrokenClient:
        """在模型调用处模拟网络异常。"""

        def __init__(self) -> None:
            """建立符合客户端形状的异常入口。"""
            self.chat = SimpleNamespace(completions=SimpleNamespace(create=self.create))

        def create(self, **_kwargs):
            """模拟服务不可达且不暴露请求内容。"""
            raise TimeoutError("network unavailable")

    answer = agent_loop(BrokenClient(), "glm-5.3-flash", messages, context)
    assert "TimeoutError" in answer
    hover.assert_called_once()
    dispatch.assert_not_called()


def test_malformed_model_tool_response_stops_without_dispatch(tmp_path: Path, monkeypatch) -> None:
    """响应缺工具调用 ID 时既不执行也不伪造结果。"""
    context = context_for(tmp_path)
    messages = [{"role": "system", "content": "test"}, {"role": "user", "content": "go"}]
    append_observation(context, messages, snapshot_at())
    client = FakeClient([SimpleNamespace(content="", tool_calls=[SimpleNamespace(function=SimpleNamespace(
        name="forward", arguments='{"distance_m":0.2}'))])])
    dispatch = Mock()
    monkeypatch.setattr(loop_module, "dispatch_tool_call", dispatch)
    assert "结构无效" in agent_loop(client, "glm-5.3-flash", messages, context)
    dispatch.assert_not_called()
