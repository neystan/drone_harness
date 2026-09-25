"""用假模型和假动作验证逐轮观测消息顺序。"""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import pytest

import drone_harness.runtime.agent_loop as loop_module
from drone_harness.config.loader import load_profile
from drone_harness.config.schema import StorageConfig
from drone_harness.runtime.agent_loop import agent_loop, append_observation
from drone_harness.runtime.task_state import TaskState
from drone_harness.tools.registry import ToolContext
from test_depth_rules import snapshot_at


def fake_call(name: str, call_id: str = "call-1") -> SimpleNamespace:
    """构造兼容 OpenAI tool-call 形状的动作提议。"""
    arguments = "{\"direction\":\"left\",\"degrees\":15}" if name == "rotate" else "{}"
    return SimpleNamespace(id=call_id, function=SimpleNamespace(name=name, arguments=arguments))


class FakeClient:
    """顺序返回预设模型消息并保存请求内容。"""

    def __init__(self, replies: list[SimpleNamespace]) -> None:
        """建立假 chat.completions.create 入口。"""
        self.replies = list(replies)
        self.requests: list[list[dict]] = []
        self.chat = SimpleNamespace(completions=SimpleNamespace(create=self.create))

    def create(self, **kwargs) -> SimpleNamespace:
        """记录一轮请求并弹出对应的假回复。"""
        self.requests.append(list(kwargs["messages"]))
        return SimpleNamespace(choices=[SimpleNamespace(message=self.replies.pop(0))])


def context_for(tmp_path: Path, controller: object | None = None) -> ToolContext:
    """把日志位置限制在测试目录并建立任务状态。"""
    settings = Path(__file__).parents[1] / "settings.example.json"
    profile = load_profile("sim", settings_path=settings)
    storage = StorageConfig(str(tmp_path / "pictures"), str(tmp_path / "analysis"), str(tmp_path / "logs"))
    return ToolContext(controller=controller or SimpleNamespace(), profile=replace(profile, storage=storage),
                       session_id="test", task_state=TaskState("test"))


def test_one_action_is_followed_by_new_multimodal_observation(tmp_path: Path, monkeypatch) -> None:
    """一个 runtime 内保留工具协议并在下一次决策前插入新图。"""
    after_stamps: list[int] = []

    def wait_for_observation(*, after_stamp_ns: int) -> object:
        """返回一张动作后采集的新观测。"""
        after_stamps.append(after_stamp_ns)
        snapshot = snapshot_at()
        stamp_ns = after_stamp_ns + 60_000_000
        return replace(snapshot, observation_id=f"rgb-{stamp_ns}", rgb_stamp_ns=stamp_ns,
                       depth_stamp_ns=stamp_ns, intrinsics=replace(snapshot.intrinsics, stamp_ns=stamp_ns))

    context = context_for(tmp_path, SimpleNamespace(wait_for_observation=wait_for_observation))
    context.task_state.start_new_goal("找到红色门")
    messages = [{"role": "system", "content": "test"}, {"role": "user", "content": "找到红色门"}]
    append_observation(context, messages, snapshot_at())
    calls: list[str] = []

    def dispatch(_context: ToolContext, call: SimpleNamespace) -> dict:
        """记录动作但不触达飞控。"""
        calls.append(call.function.name)
        return {"success": True, "message": "rotated"}

    monkeypatch.setattr(loop_module, "dispatch_tool_call", dispatch)
    client = FakeClient([
        SimpleNamespace(content="", tool_calls=[fake_call("rotate")]),
        SimpleNamespace(content="候选完成：画面中看到了红色门", tool_calls=[]),
    ])
    answer = agent_loop(client, "test-vlm", messages, context)
    assert calls == ["rotate"]
    assert len(after_stamps) == 1
    assert len(client.requests) == 2
    second = client.requests[1]
    assert [entry["role"] for entry in second] == ["system", "user", "user", "assistant", "tool", "user"]
    assert second[-1]["content"][0]["text"].startswith("新观测：observation_id=")
    assert second[-1]["content"][1]["image_url"]["url"].startswith("data:image/jpeg;base64,")
    assert sum(isinstance(entry["content"], list) for entry in second) == 2
    assert context.task_state.completion_candidate == answer


def test_multiple_calls_are_all_rejected_without_dispatch(tmp_path: Path, monkeypatch) -> None:
    """一个回复包含两个动作时两个都不得执行。"""
    context = context_for(tmp_path)
    messages = [{"role": "system", "content": "test"}, {"role": "user", "content": "go"}]
    append_observation(context, messages, snapshot_at())
    invoked: list[bool] = []
    monkeypatch.setattr(loop_module, "dispatch_tool_call", lambda *_args: invoked.append(True))
    client = FakeClient([SimpleNamespace(content="", tool_calls=[fake_call("rotate", "a"),
                                                             fake_call("takeoff", "b")])])
    agent_loop(client, "test-vlm", messages, context)
    assert invoked == []
    assert [entry["role"] for entry in messages[-3:]] == ["assistant", "tool", "tool"]
    assert all("MULTIPLE_TOOL_CALLS_REJECTED" in entry["content"] for entry in messages[-2:])


@pytest.mark.parametrize("result,expected", [
    ({"success": False, "error": "REJECTED"}, "未成功"),
    ({"success": True}, "没有新 RGB"),
])
def test_failure_or_missing_post_action_rgb_stops_without_second_decision(
    tmp_path: Path, monkeypatch, result: dict, expected: str,
) -> None:
    """动作失败或后续图超时都不能用旧画面续飞。"""
    context = context_for(tmp_path, SimpleNamespace(wait_for_observation=lambda **_kwargs: None))
    messages = [{"role": "system", "content": "test"}, {"role": "user", "content": "go"}]
    append_observation(context, messages, snapshot_at())
    monkeypatch.setattr(loop_module, "dispatch_tool_call", lambda *_args: result)
    client = FakeClient([SimpleNamespace(content="", tool_calls=[fake_call("rotate")])])
    assert expected in agent_loop(client, "test-vlm", messages, context)
    assert len(client.requests) == 1


def test_initial_missing_rgb_never_calls_model(tmp_path: Path) -> None:
    """未注入首帧前不向模型发送仅文本的飞行任务。"""
    context = context_for(tmp_path)
    client = FakeClient([])
    assert "没有可用" in agent_loop(client, "test-vlm", [], context)
    assert client.requests == []


def test_mismatched_summary_and_rgb_are_rejected(tmp_path: Path) -> None:
    """观测号不一致时不能把图片和规则拼接发给模型。"""
    from drone_harness.runtime.observation import build_observation_message
    from drone_harness.vision.depth_rules import invalid_depth_rules

    with pytest.raises(ValueError):
        build_observation_message(snapshot_at(), invalid_depth_rules("another", "TEST"))


def test_long_goal_keeps_at_most_two_images_and_logs_no_base64(tmp_path: Path, monkeypatch) -> None:
    """连续动作时历史和日志均不累积原图数据。"""
    def wait_for_observation(*, after_stamp_ns: int) -> object:
        """按动作结束时间生成新的合成采集帧。"""
        snapshot = snapshot_at()
        stamp_ns = after_stamp_ns + 60_000_000
        return replace(snapshot, observation_id=f"rgb-{stamp_ns}", rgb_stamp_ns=stamp_ns,
                       depth_stamp_ns=stamp_ns, intrinsics=replace(snapshot.intrinsics, stamp_ns=stamp_ns))

    context = context_for(tmp_path, SimpleNamespace(wait_for_observation=wait_for_observation))
    messages = [{"role": "system", "content": "test"}, {"role": "user", "content": "go"}]
    append_observation(context, messages, snapshot_at())
    monkeypatch.setattr(loop_module, "dispatch_tool_call", lambda *_args: {"success": True})
    client = FakeClient([SimpleNamespace(content="", tool_calls=[fake_call("rotate", f"call-{index}")])
                         for index in range(4)] + [SimpleNamespace(content="候选完成", tool_calls=[])])
    agent_loop(client, "test-vlm", messages, context)
    assert len(client.requests) == 5
    assert all(sum(isinstance(entry["content"], list) for entry in request) <= 2
               for request in client.requests)
    assert "data:image" not in (tmp_path / "logs" / "session_test" / "observations.jsonl").read_text()


def test_interactive_goal_without_initial_rgb_skips_agent(tmp_path: Path) -> None:
    """runtime 首帧等待失败时不进入模型或工具循环。"""
    from drone_harness.runtime.runtime import _run_interactive_loop

    inputs = iter(["go", "exit"])
    bus = SimpleNamespace(consume_user_message=lambda: SimpleNamespace(content=next(inputs)))
    context = context_for(tmp_path, SimpleNamespace(wait_for_observation=lambda: None))
    context.message_bus = bus
    invoked: list[bool] = []
    _run_interactive_loop(FakeClient([]), "test-vlm", context,
                          lambda *_args: invoked.append(True), input_terminal_started=True)
    assert invoked == []
