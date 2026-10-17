"""验证同次视觉判断的结构化反思与原生工具调用共存。"""

import json
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from drone_harness.llm.prompts import build_system_prompt
from drone_harness.runtime.agent_loop import agent_loop, append_observation
from drone_harness.runtime.navigation import parse_navigation_plan, update_navigation_plan
from drone_harness.runtime.reflection import record_target_assessment
from drone_harness.runtime.task_memory import prepare_navigation_turn
from drone_harness.tools import flight
from drone_harness.tools.registry import get_tool_definition
from test_agent_observation_loop import FakeClient, context_for
from test_depth_rules import snapshot_at
from test_search_skill import activate_call, image_count, scan_context
from test_stage3_observe import fresh_snapshot, tool_call


def assessment(status="uncertain", message="车辆符合，但仓库关系被遮挡。"):
    """生成真实模型正文应使用的判断对象。"""
    return json.dumps({"target_status": status, "message": message}, ensure_ascii=False)


@pytest.mark.parametrize("status", ["confirmed", "uncertain", "not_found"])
def test_each_status_is_recorded_without_automatic_checks_or_search(tmp_path, status):
    """三种状态都不自动增加动作、观察或模型请求。"""
    context = context_for(tmp_path)
    messages = [{"role": "user", "content": "寻找仓库旁的卡车"}]
    append_observation(context, messages, snapshot_at())
    client = FakeClient([SimpleNamespace(content=assessment(status), tool_calls=[])])
    agent_loop(client, "test", messages, context)
    assert context.target_assessment["target_status"] == status
    assert len(client.requests) == 1
    assert not any(item["role"] == "tool" for item in messages)


def test_uncertain_reply_can_request_new_observation_in_same_model_turn(tmp_path):
    """结构化判断与原生observe共存，不需要额外纯反思请求。"""
    context = context_for(tmp_path, SimpleNamespace(wait_for_observation=
        lambda after_stamp_ns, view="front": fresh_snapshot(after_stamp_ns)))
    messages = [{"role": "user", "content": "寻找仓库旁的卡车"}]
    append_observation(context, messages, snapshot_at())
    client = FakeClient([
        SimpleNamespace(content=assessment(), tool_calls=[tool_call("observe", {"prompt": "查看旁边建筑特征和邻近关系"})]),
        SimpleNamespace(content=assessment("confirmed", "仓库特征及与卡车的邻近关系清晰。"), tool_calls=[]),
    ])
    agent_loop(client, "test", messages, context)
    assert len(client.requests) == 2
    assert context.target_assessment["target_status"] == "confirmed"
    assert [item["tool_call_id"] for item in messages if item["role"] == "tool"] == ["call-1"]
    result = next(json.loads(item["content"]) for item in messages if item["role"] == "tool")
    assert result["message"] == "已取得当前图像与深度摘要"
    assert "target_status" not in result


def test_joint_search_uses_same_assessment_without_preserving_four_images(tmp_path, monkeypatch):
    """搜索四图后的判断被记录，但图组与当前测距缓存均清理。"""
    context = scan_context(tmp_path, monkeypatch, [])
    messages = [{"role": "system", "content": build_system_prompt(context.profile)},
                {"role": "user", "content": "找卡车"}]
    client = FakeClient([
        SimpleNamespace(content="需要扩大范围", tool_calls=[activate_call()]),
        SimpleNamespace(content=assessment("uncertain", "原朝向左侧红卡车旁建筑被遮挡。"), tool_calls=[]),
    ])
    agent_loop(client, "test", messages, context)
    assert context.target_assessment["target_status"] == "uncertain"
    assert [image_count(request) for request in client.requests] == [0, 4]
    assert image_count(messages) == 0 and context.observation is None
    assert context.depth_rules is None
    assert "普通观察与四向搜索共用" in client.requests[0][0]["content"]


def test_motion_invalidates_assessment_and_does_not_confirm_completion(tmp_path, monkeypatch):
    """目标确认不会推进计划，运动之后清除临时判断。"""
    context = context_for(tmp_path)
    messages = [{"role": "user", "content": "找卡车"}]
    append_observation(context, messages, snapshot_at())
    monkeypatch.setattr(flight, "rotate", Mock(return_value={"success": True, "degrees": 10}))
    monkeypatch.setattr('drone_harness.runtime.agent_loop.time.sleep', lambda _seconds: None)
    client = FakeClient([
        SimpleNamespace(content=assessment("confirmed"), tool_calls=[tool_call("rotate", {"direction": "left", "degrees": 10})]),
        SimpleNamespace(content="准备重新观察", tool_calls=[]),
    ])
    agent_loop(client, "test", messages, context)
    assert context.target_assessment is None
    assert context.navigation_plan is None
    assert len(client.requests) == 2


def test_new_image_failure_and_subgoal_change_clear_old_assessment(tmp_path):
    """取图失败和切换子目标均不能沿用之前的confirmed。"""
    context = context_for(tmp_path, SimpleNamespace(wait_for_observation=lambda **_kwargs: None))
    context.navigation_enabled = True
    context.navigation_instruction = "依次找两辆卡车"
    first = {"id": "a", "description": "红卡车", "completion_condition": "到前方", "status": "in_progress"}
    second = {"id": "b", "description": "蓝卡车", "completion_condition": "到旁边", "status": "pending"}
    context.navigation_plan = parse_navigation_plan({"subgoals": [first, second], "reason": "原任务"}, context.navigation_instruction)
    record_target_assessment(context, assessment("confirmed"), has_current_image=True)
    assert context.navigation_plan.current.id == "a" and context.navigation_plan.completed_count == 0
    update_navigation_plan(context, {"subgoals": [{**first, "status": "completed"}, {**second, "status": "in_progress"}], "reason": "进入第二段"})
    assert context.target_assessment is None
    record_target_assessment(context, assessment("confirmed"), has_current_image=True)
    get_tool_definition("observe").handler(context, {"prompt": "查看蓝卡车"})
    assert context.target_assessment is None
    record_target_assessment(context, assessment("confirmed"), has_current_image=True)
    prepare_navigation_turn(context, "新目标", replace_task=True)
    assert context.target_assessment is None


@pytest.mark.parametrize("text", ["只描述图片，没有结构化判断", '{"target_status":"probably","message":"可能"}',
                                     '{"target_status":"confirmed","message":""}',
                                     '{"target_status":"confirmed","message":"看到目标","question":"再看一次"}'])
def test_invalid_or_missing_fields_are_not_fabricated_and_do_not_block_native_tools(tmp_path, text):
    """字段错误时保持未知，正常工具协议不被改写或增加检查轮次。"""
    context = context_for(tmp_path)
    messages = [{"role": "user", "content": "查看目标"}]
    append_observation(context, messages, snapshot_at())
    client = FakeClient([
        SimpleNamespace(content=text, tool_calls=[tool_call("get_state", {})]),
        SimpleNamespace(content="结束本轮说明", tool_calls=[]),
    ])
    agent_loop(client, "test", messages, context)
    assert context.target_assessment is None
    assert len(client.requests) == 2
    assert sum(item["role"] == "tool" for item in messages) == 1


def test_history_only_reply_cannot_create_a_current_target_assessment(tmp_path):
    """没有当前图的文字猜测不能成为当前参照物判断。"""
    context = context_for(tmp_path)
    messages = [{"role": "user", "content": "之前似乎看见过卡车"}]
    client = FakeClient([SimpleNamespace(content=assessment("confirmed"), tool_calls=[])])
    agent_loop(client, "test", messages, context)
    assert context.target_assessment is None
