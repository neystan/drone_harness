"""验证本轮完整工具记录与跨轮纯文字会话的边界。"""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest

import drone_harness.runtime.agent_loop as loop_module
from drone_harness.runtime.agent_loop import _compact_history
from drone_harness.runtime.runtime import _run_interactive_loop
from drone_harness.runtime.safety import EndCurrentTurn
from drone_harness.tools import flight
from test_agent_observation_loop import FakeClient, context_for
from test_stage3_observe import fresh_snapshot, tool_call


def test_scene_description_survives_next_user_turn_without_tool_records(tmp_path, monkeypatch):
    """主模型看图写出的描述随会话保留，下一轮只携带用户和 AI 文字。"""
    inputs = iter(["寻找红色店招", "继续", "exit"])
    bus = SimpleNamespace(consume_user_message=lambda: SimpleNamespace(content=next(inputs)),
                          has_pending_user_message=lambda: False)
    context = context_for(tmp_path, SimpleNamespace(
        wait_for_observation=lambda *, after_stamp_ns: fresh_snapshot(after_stamp_ns)))
    context.message_bus = bus
    monkeypatch.setattr(flight, "rotate", Mock(return_value={"success": True, "degrees": 30}))
    scene = "观察：左前方有十字路口，红色店招在路口右侧。"
    client = FakeClient([
        SimpleNamespace(content="", tool_calls=[tool_call("observe", {"prompt": "寻找店招"}, "look")]),
        SimpleNamespace(content=scene, tool_calls=[tool_call("rotate", {"direction": "left", "degrees": 30}, "turn")]),
        SimpleNamespace(content="已转向，下次继续观察。", tool_calls=[]),
        SimpleNamespace(content="收到，继续寻找红色店招。", tool_calls=[]),
    ])
    _run_interactive_loop(client, "test", context, loop_module.agent_loop, input_terminal_started=True)
    assert len(client.requests) == 4
    same_turn = client.requests[2]
    assert [item["tool_call_id"] for item in same_turn if item["role"] == "tool"] == ["look", "turn"]
    next_turn = client.requests[3]
    assert [item["role"] for item in next_turn] == ["system", "user", "assistant", "assistant", "user"]
    assert [item["content"] for item in next_turn[1:]] == [
        "寻找红色店招", scene, "已转向，下次继续观察。", "继续"]
    assert all(isinstance(item["content"], str) and "tool_calls" not in item for item in next_turn)
    assert context.observation is None and context.depth_rules is None
    assert scene in (tmp_path / "logs/session_test/agent_messages.jsonl").read_text()


@pytest.mark.parametrize("ending", ["land", "failure", "interruption"])
def test_runtime_keeps_stop_message_without_orphan_tool_calls(tmp_path, monkeypatch, ending):
    """降落、失败和中断原因跨轮保留，同时移除所有旧工具协议字段。"""
    inputs = iter(["执行任务", "接下来呢", "exit"])
    context = context_for(tmp_path)
    context.message_bus = SimpleNamespace(consume_user_message=lambda: SimpleNamespace(content=next(inputs)))

    def dispatch(_context, _call):
        """返回不同终止结果，不连接飞控。"""
        if ending == "interruption":
            raise EndCurrentTurn("用户中断了动作", {"success": False, "error": "INTERRUPTED_BY_USER"})
        return {"success": ending == "land", "error": "TEST_FAILURE" if ending == "failure" else ""}

    monkeypatch.setattr(loop_module, "dispatch_tool_call", dispatch)
    client = FakeClient([
        SimpleNamespace(content="准备执行。", tool_calls=[tool_call("land" if ending == "land" else "rotate", {})]),
        SimpleNamespace(content="已收到上一轮结果。", tool_calls=[]),
    ])
    _run_interactive_loop(client, "test", context, loop_module.agent_loop, input_terminal_started=True)
    expected = {"land": "降落完成，本轮已结束。", "failure": "rotate 未成功，已停止本轮。",
                "interruption": "用户中断了动作"}[ending]
    followup = client.requests[1]
    assert [item["content"] for item in followup[1:]] == ["执行任务", "准备执行。", expected, "接下来呢"]
    assert all(set(item) == {"role", "content"} for item in followup)


def test_history_removes_only_old_images_without_mutating_prior_requests():
    """旧图片被替换为历史标记，所有文字、工具调用及结果按原序保留。"""
    messages = [{"role": "system", "content": "system"}, {"role": "user", "content": "目标"}]
    for index in range(5):
        messages.extend([
            {"role": "assistant", "content": f"描述 {index}", "tool_calls": [{"id": str(index)}]},
            {"role": "tool", "tool_call_id": str(index), "content": f"结果 {index}"},
            {"role": "user", "content": [{"type": "text", "text": f"深度摘要 {index}"},
                                            {"type": "image_url", "image_url": {"url": f"image-{index}"}}]},
        ])
    previous_request = list(messages)
    _compact_history(messages)
    _compact_history(messages)
    assert len(messages) == len(previous_request)
    assert [item["tool_call_id"] for item in messages if item["role"] == "tool"] == list(map(str, range(5)))
    images = [part["image_url"]["url"] for item in messages if isinstance(item["content"], list)
              for part in item["content"] if part["type"] == "image_url"]
    assert images == ["image-4"]
    assert "image-0" in str(previous_request)
    for index in range(5):
        assert f"描述 {index}" in str(messages) and f"结果 {index}" in str(messages)
        assert f"深度摘要 {index}" in str(messages)
