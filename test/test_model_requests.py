"""验证现有模型请求的计时、图片数量和用量关联，不访问网络。"""

import json
from types import SimpleNamespace

import pytest

import drone_harness.logging.task_log as task_log
import drone_harness.runtime.agent_loop as loop
from test_agent_observation_loop import FakeClient, context_for
from test_navigation_loop import reply


@pytest.mark.parametrize("image_count", [0, 1, 2, 4])
def test_request_records_actual_images_elapsed_and_same_response_usage(tmp_path, monkeypatch, image_count):
    """计数实际发出的图组，计时及用量日志不增加请求或泄漏消息。"""
    context = context_for(tmp_path)
    messages = [{"role": "user", "content": [{"type": "text", "text": "private navigation"},
                *[{"type": "image_url", "image_url": {"url": "private-image-data"}}
                  for _ in range(image_count)]]}]
    client = FakeClient([reply(text="继续")])
    original = client.chat.completions.create

    def create(**kwargs):
        """让既有假响应携带同一次请求的服务端计数。"""
        response = original(**kwargs)
        response.usage = SimpleNamespace(prompt_tokens=160, completion_tokens=20, total_tokens=180)
        return response

    ticks = iter((100.0, 112.5))

    def clock():
        """提供不受日志与工具处理影响的请求时钟。"""
        return next(ticks)

    monkeypatch.setattr(loop, "perf_counter", clock)
    client.chat.completions.create = create
    assert loop.agent_loop(client, "test-model", messages, context) == "继续"
    path = tmp_path / "logs/session_test/model_requests.jsonl"
    records = [json.loads(line) for line in path.read_text().splitlines()]
    assert len(client.requests) == 1
    assert [row["status"] for row in records] == ["started", "completed"]
    assert records[0]["request_id"] == records[1]["request_id"]
    assert [row["image_count"] for row in records] == [image_count, image_count]
    assert records[0]["elapsed_s"] is None and records[1]["elapsed_s"] == 12.5
    assert [records[1][name] for name in ("prompt_tokens", "completion_tokens", "total_tokens")] == [160, 20, 180]
    assert "private" not in path.read_text()
    assert json.loads((path.parent / "context_usage.jsonl").read_text())["prompt_tokens"] == 160


@pytest.mark.parametrize("error", [TimeoutError, KeyboardInterrupt])
def test_failed_or_interrupted_request_keeps_started_record_and_exception_type(tmp_path, monkeypatch, error):
    """失败和中断均留下请求耗时，只记录异常类型且不重试。"""
    context = context_for(tmp_path)
    calls = []

    def fail(**kwargs):
        """模拟超时或取消，异常正文不得进入诊断日志。"""
        calls.append(kwargs)
        raise error("private failure detail")

    def stop(_context, message, **kwargs):
        """隔离原有安全收尾，本测试只验证请求日志。"""
        return message

    monkeypatch.setattr(loop, "_stop_with_message", stop)
    client = SimpleNamespace(chat=SimpleNamespace(completions=SimpleNamespace(create=fail)))
    if error is KeyboardInterrupt:
        with pytest.raises(KeyboardInterrupt):
            loop.agent_loop(client, "test-model", [], context)
    else:
        assert "TimeoutError" in loop.agent_loop(client, "test-model", [], context)
    text = (tmp_path / "logs/session_test/model_requests.jsonl").read_text()
    records = [json.loads(line) for line in text.splitlines()]
    assert len(calls) == 1
    assert [row["status"] for row in records] == ["started", "failed"]
    assert records[1]["error_type"] == error.__name__ and records[1]["elapsed_s"] >= 0
    assert records[1]["prompt_tokens"] is None and "private" not in text


def test_missing_or_invalid_usage_is_unknown_and_write_failure_does_not_change_reply(tmp_path, monkeypatch):
    """未知计数不补造，日志写入失败时模型回复仍正常返回。"""
    profile = context_for(tmp_path).profile
    task_log.log_model_request(profile, "test", "model", "request", "completed", 0,
                               usage=SimpleNamespace(prompt_tokens=True, completion_tokens=-1, total_tokens="12"))
    event = json.loads((tmp_path / "logs/session_test/model_requests.jsonl").read_text())
    assert all(event[name] is None for name in ("prompt_tokens", "completion_tokens", "total_tokens"))

    def fail_write(*args):
        """模拟诊断日志无法写入。"""
        raise OSError("disk unavailable")

    monkeypatch.setattr(task_log, "append_jsonl", fail_write)
    client = FakeClient([reply(text="正常回复")])
    assert loop.agent_loop(client, "model", [], context_for(tmp_path)) == "正常回复"
    assert len(client.requests) == 1
