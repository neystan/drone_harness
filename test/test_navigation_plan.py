"""离线验证计划拆分、顺序推进和上下文结构。"""

import json
from types import SimpleNamespace

import pytest

from drone_harness.runtime.navigation import (
    compact_navigation_history, navigation_conversation_text, navigation_messages,
    parse_navigation_decision, parse_navigation_plan, request_navigation_plan,
)
from test_agent_observation_loop import FakeClient


def plan_payload(*, finish_action="land"):
    """构造两个有序导航段。"""
    return {"navigation": True, "subgoals": [
        {"description": "到路口", "completion_condition": "观察支持位于路口"},
        {"description": "到红门前", "completion_condition": "观察支持位于红门前"},
    ], "finish_action": finish_action}


def make_plan(*, finish_action="land"):
    """解析测试计划。"""
    return parse_navigation_plan(json.dumps(plan_payload(finish_action=finish_action)), "沿路到路口，再到红门前")


def test_plan_advances_only_in_order_and_finishes_after_all_subgoals():
    """依据、观测和顺序均由程序保存。"""
    plan = make_plan()
    assert plan.current.description == "到路口"
    assert not plan.advance("已到路口", None)
    assert plan.current_index == 0
    assert plan.advance("已到路口", "obs-1")
    assert plan.current.description == "到红门前"
    assert plan.subgoals[0].observation_id == "obs-1"
    assert plan.advance("已到红门前", "obs-2")
    assert plan.status == "finishing" and plan.current is None
    assert not plan.advance("不能重复确认", "obs-2")
    plan.finish(completed=True, reason="降落完成")
    assert plan.status == "completed"
    assert [item["status"] for item in plan.snapshot()["subgoals"]] == ["completed", "completed"]


def test_early_finish_does_not_mark_navigation_completed():
    """提前结束保留未完成记录。"""
    plan = make_plan()
    plan.finish(completed=True, reason="提前降落")
    assert plan.status == "incomplete"
    assert plan.snapshot()["current_subgoal_number"] is None


@pytest.mark.parametrize("bad", [
    {}, {"navigation": "true"}, {"navigation": True, "subgoals": []},
    {**plan_payload(), "finish_action": "fly"},
    {**plan_payload(), "subgoals": [{}]},
    {**plan_payload(), "subgoals": [{"description": " ", "completion_condition": "条件"}]},
    {**plan_payload(), "subgoals": [{"description": "目标", "completion_condition": 1}]},
    [],
])
def test_invalid_plan_is_rejected(bad):
    """无效拆分不能进入执行。"""
    with pytest.raises(ValueError):
        parse_navigation_plan(json.dumps(bad), "导航任务")


def test_plain_chat_has_no_plan_and_code_fence_is_accepted():
    """普通交流和常见围栏响应有明确语义。"""
    assert parse_navigation_plan('{"navigation":false}', "你好") is None
    assert parse_navigation_plan("```json\n" + json.dumps(plan_payload()) + "\n```", "导航").current_index == 0


@pytest.mark.parametrize("content", [
    '{"subgoal_complete":"true","evidence":"已到达"}',
    '{"subgoal_complete":true,"evidence":" "}',
    '{"subgoal_complete":false,"evidence":null}',
    '{"subgoal_complete":false,"evidence":"","scene_description":1}',
    '{"subgoal_complete":true', '[]',
])
def test_invalid_completion_is_rejected(content):
    """格式错误或缺少依据不能确认。"""
    with pytest.raises(ValueError):
        parse_navigation_decision(content)


def test_empty_or_plain_text_does_not_confirm_completion():
    """自然语言宣称到达也不是结构化完成事件。"""
    assert not parse_navigation_decision("").subgoal_complete
    assert not parse_navigation_decision("已经到达目标，可以结束了").subgoal_complete
    content = '{"subgoal_complete":true,"evidence":"新观察支持到达","scene_description":"红门在正前方"}'
    assert parse_navigation_decision(content).subgoal_complete
    assert navigation_conversation_text(content) == "红门在正前方"


def test_plan_request_uses_same_model_without_tools_or_images():
    """拆分只读取文字会话，不自动取图或执行工具。"""
    client = FakeClient([SimpleNamespace(content=json.dumps(plan_payload()), tool_calls=[])])
    messages = [{"role": "system", "content": "原提示"}, {"role": "user", "content": "去红门前"}]
    plan = request_navigation_plan(client, "same-vlm", messages)
    assert plan.original_instruction == "去红门前"
    assert len(client.requests) == 1 and all(isinstance(item["content"], str) for item in client.requests[0])


def test_progress_injection_replaces_snapshot_without_mutating_history():
    """最新摘要独立生成，内部观测号不上送模型。"""
    messages = [{"role": "system", "content": "原提示"}, {"role": "user", "content": "任务"}]
    plan = make_plan()
    first = navigation_messages(messages, plan, observation_current=False)
    plan.advance("位于路口", "internal-observation")
    second = navigation_messages(messages, plan, observation_current=True)
    assert messages[0]["content"] == "原提示"
    assert '"current_subgoal_number": 1' in first[0]["content"]
    assert '"current_subgoal_number": 2' in second[0]["content"]
    assert "位于路口" in second[0]["content"] and "internal-observation" not in str(second)
    assert second[0]["content"].count("当前导航计划：") == 1


@pytest.mark.parametrize("current", [True, False])
def test_completed_segment_compaction_preserves_prefix_and_no_orphan_tools(current):
    """压缩按整段进行，仅保留当前有效的最新图。"""
    prefix = [{"role": "system", "content": "原提示"}, {"role": "user", "content": "完整指令"}]
    visual = {"role": "user", "content": [{"type": "image_url", "image_url": {"url": "new-image"}}]}
    messages = prefix + [
        {"role": "assistant", "content": "旧描述", "tool_calls": [{"id": "a"}]},
        {"role": "tool", "content": "动作反馈", "tool_call_id": "a"}, visual,
    ]
    old_request = list(messages)
    compact_navigation_history(messages, 2, observation_current=current)
    assert messages == prefix + ([visual] if current else [])
    assert old_request[2]["tool_calls"][0]["id"] == "a"
