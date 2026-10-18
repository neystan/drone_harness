"""离线验证服务端输入用量、子目标清理后的下降与画面显示。"""

import json
from types import SimpleNamespace

import pytest

import drone_harness.runtime.agent_loop as loop
from drone_harness.llm.client import model_context_window_tokens
from drone_harness.testing.recording import EventTail
from drone_harness.testing.recording_display import ContextUsagePanel
from test_agent_observation_loop import FakeClient, context_for
from test_navigation_loop import messages, nav_context, planning_reply, reply


class UsageClient(FakeClient):
    """给每次假模型响应附加预设服务端用量，不访问网络。"""

    def __init__(self, replies, usages):
        """保存响应和对应的用量序列。"""
        super().__init__(replies)
        self.usages = iter(usages)

    def create(self, **kwargs):
        """保持原请求记录，并模拟 SDK 的可选 usage 对象。"""
        response = super().create(**kwargs)
        response.usage = next(self.usages)
        return response


def test_usage_follows_each_response_and_falls_after_real_history_cleanup(tmp_path, monkeypatch):
    """跑通计划完成清理及日志到面板，核对用量替换而非累加。"""
    context = nav_context(tmp_path)
    original = loop.dispatch_tool_call
    old_result = "已删除的旧状态结果" * 1000

    def dispatch(ctx, call):
        """只替代状态查询，其余继续执行正式计划更新和清理。"""
        if call.function.name == "get_state":
            return {"success": True, "message": old_result}
        return original(ctx, call)

    monkeypatch.setattr(loop, "dispatch_tool_call", dispatch)
    counts = [1000, 2000, 18000, 3000]
    client = UsageClient([planning_reply(), reply("get_state"),
                          planning_reply(("completed", "completed")), reply(text="完成")],
                         [SimpleNamespace(prompt_tokens=value) for value in counts])
    loop.agent_loop(client, "glm-5.3-flash", messages(), context)
    assert len(client.requests) == 4
    assert old_result in str(client.requests[2])
    assert old_result not in str(client.requests[3])
    path = tmp_path / "logs/session_test/context_usage.jsonl"
    events = EventTail(path).read()
    assert [event["prompt_tokens"] for event in events] == counts
    panel = ContextUsagePanel(800)
    for event in events:
        panel.update(event)
    assert "3,000 / 1,048,576 tokens" in panel.text and "0.29%" in panel.text
    assert "24,000" not in panel.text
    assert all("context_usage" not in str(request) for request in client.requests)


@pytest.mark.parametrize("value", [None, -1, True, 12.5, "123", 0])
def test_missing_or_invalid_usage_does_not_stop_the_loop(tmp_path, value):
    """缺失或异常计数记为未知，零值保留，导航回复照常返回。"""
    usage = None if value is None else SimpleNamespace(prompt_tokens=value)
    client = UsageClient([reply(text="正常回复")], [usage])
    assert loop.agent_loop(client, "glm-5.3-flash", messages(), context_for(tmp_path)) == "正常回复"
    event = json.loads((tmp_path / "logs/session_test/context_usage.jsonl").read_text())
    assert event["prompt_tokens"] == (0 if type(value) is int and value == 0 else None)


def test_usage_log_failure_does_not_change_agent_reply(tmp_path, monkeypatch):
    """日志无法落盘时仍返回模型结果，不触发导航失败。"""
    import drone_harness.logging.task_log as task_log

    def fail_write(*args):
        """模拟磁盘日志写入失败。"""
        raise OSError("offline disk failure")

    monkeypatch.setattr(task_log, "append_jsonl", fail_write)
    client = UsageClient([reply(text="正常回复")], [SimpleNamespace(prompt_tokens=123)])
    assert loop.agent_loop(client, "glm-5.3-flash", messages(), context_for(tmp_path)) == "正常回复"


def test_unknown_service_or_model_has_no_guessed_window():
    """模型名相同的其它服务也不冒用官方窗口。"""
    assert model_context_window_tokens("glm-5.3-flash", "https://open.bigmodel.cn/api/paas/v4") == 1_048_576
    assert model_context_window_tokens("other-model", "https://open.bigmodel.cn/api/paas/v4") is None
    assert model_context_window_tokens("glm-5.3-flash", "https://example.invalid/v1") is None


def test_panel_waiting_missing_counts_unknown_limit_and_cache():
    """区分等待与未提供用量，清除旧读数，并复用未变化的像素。"""
    panel = ContextUsagePanel(800)
    assert "等待模型响应" in panel.text
    first = panel.render()
    assert first.shape == (32, 800, 3) and panel.render() is first
    panel.update({"prompt_tokens": 524288, "context_window_tokens": 1048576})
    assert "50.00%" in panel.text
    known = panel.render()
    panel.update({"prompt_tokens": 524288, "context_window_tokens": 1048576})
    assert panel.render() is known
    panel.update({"prompt_tokens": None, "context_window_tokens": 1048576})
    assert "未提供用量" in panel.text and "524,288" not in panel.text
    panel.update({"prompt_tokens": 123, "context_window_tokens": None})
    assert "123 tokens" in panel.text and "上限未知" in panel.text and "%" not in panel.text
