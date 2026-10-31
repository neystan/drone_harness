"""验证同次视觉判断的结构化反思与原生工具调用共存。"""

import json
from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from drone_harness.llm.prompts import build_system_prompt
from drone_harness.runtime.agent_loop import agent_loop, append_observation
from drone_harness.runtime.navigation import parse_navigation_plan, update_navigation_plan
from drone_harness.runtime.reflection import (
    clear_target_assessment, record_target_assessment, target_assessment_snapshot,
)
from drone_harness.runtime.task_memory import prepare_navigation_turn
from drone_harness.tools import flight
from drone_harness.tools.registry import get_tool_definition
from test_agent_observation_loop import FakeClient, context_for
from test_depth_rules import snapshot_at
from test_search_skill import activate_call, image_count, scan_context
from test_stage3_observe import fresh_snapshot, tool_call


def assessment(status="uncertain", message="车辆符合，但仓库关系被遮挡。"):
    """生成真实模型正文应使用的判断对象。"""
    return json.dumps({"target_status": status, "candidate": "仓库旁的红色卡车",
                       "support": message, "conflict": "", "open_issue": ""}, ensure_ascii=False)


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


def test_current_and_previous_are_bound_to_images_and_do_not_accumulate(tmp_path):
    """新观察不继承确认，当前和历史最多各一份且快照不可反写。"""
    context = context_for(tmp_path)
    messages = []
    append_observation(context, messages, snapshot_at())
    record_target_assessment(context, assessment("confirmed", "邻接关系符合"), has_current_image=True)
    first = target_assessment_snapshot(context)
    assert first["current"]["source"]["observation_id"] == context.observation.observation_id
    first["current"]["support"] = "外部改写"
    assert context.target_assessment["support"] == "邻接关系符合"
    clear_target_assessment(context)
    context.observation = None
    stale = target_assessment_snapshot(context)
    assert stale["current"] is None and stale["previous"]["target_status"] == "confirmed"
    append_observation(context, messages, replace(snapshot_at(), observation_id="new-image"))
    assert target_assessment_snapshot(context)["current"] is None
    record_target_assessment(context, assessment("uncertain", "新图显示另一相似卡车"), has_current_image=True)
    assert target_assessment_snapshot(context)["previous"]["support"] == "邻接关系符合"
    record_target_assessment(context, assessment("confirmed", "仓库入口区分了两辆卡车"), has_current_image=True)
    state = target_assessment_snapshot(context)
    assert state["current"]["support"] == "仓库入口区分了两辆卡车"
    assert state["previous"]["support"] == "新图显示另一相似卡车"


def test_goal_rewrite_and_task_replacement_forget_candidate_history(tmp_path):
    """同ID目标改写与新任务不能继承旧候选或历史确认。"""
    context = context_for(tmp_path)
    context.navigation_enabled = True
    context.navigation_instruction = "到仓库旁卡车"
    goal = {"id": "a", "description": "卡车", "completion_condition": "在仓库旁", "status": "in_progress"}
    update_navigation_plan(context, {"subgoals": [goal], "reason": "建立"})
    append_observation(context, [], snapshot_at())
    record_target_assessment(context, assessment("confirmed"), has_current_image=True)
    clear_target_assessment(context)
    assert context.previous_target_assessment is not None
    update_navigation_plan(context, {"subgoals": [{**goal, "completion_condition": "在门前"}], "reason": "更正"})
    assert target_assessment_snapshot(context) == {"current": None, "previous": None}
    record_target_assessment(context, assessment("confirmed"), has_current_image=True)
    prepare_navigation_turn(context, "新任务", replace_task=True)
    assert target_assessment_snapshot(context) == {"current": None, "previous": None}


def test_search_judgement_becomes_history_after_joint_images_are_consumed(tmp_path):
    """四向判断绑定联合请求，后续不能冒充当前前视确认。"""
    context = context_for(tmp_path)
    record_target_assessment(context, assessment("uncertain"), has_current_image=True,
                             source={"type": "four_direction_search", "request_id": "request-search"})
    state = target_assessment_snapshot(context)
    assert state["current"] is None
    assert state["previous"]["source"] == {"type": "four_direction_search", "request_id": "request-search"}


@pytest.mark.parametrize("changes", [
    {"target_status": "probably"}, {"candidate": ""}, {"conflict": []},
    {"support": "", "conflict": "", "open_issue": ""}, {"source": {"type": "fake"}},
])
def test_invalid_structured_evidence_cannot_replace_current_with_fabricated_fields(tmp_path, changes):
    """缺失或非法证据保持当前未知，不允许模型伪造程序来源。"""
    context = context_for(tmp_path)
    append_observation(context, [], snapshot_at())
    record_target_assessment(context, assessment("confirmed"), has_current_image=True)
    bad = {**json.loads(assessment()), **changes}
    record_target_assessment(context, json.dumps(bad), has_current_image=True)
    state = target_assessment_snapshot(context)
    assert state["current"] is None and state["previous"]["target_status"] == "confirmed"


def test_not_found_can_have_no_candidate_and_keeps_unseen_separate_from_conflict(tmp_path):
    """没有候选时可以只记录信息缺口，不补造目标或矛盾。"""
    context = context_for(tmp_path)
    append_observation(context, [], snapshot_at())
    text = json.dumps({"target_status": "not_found", "candidate": "", "support": "", "conflict": "",
                       "open_issue": "道路另一侧被遮挡"})
    record_target_assessment(context, text, has_current_image=True)
    state = target_assessment_snapshot(context)["current"]
    assert state["candidate"] == state["conflict"] == ""
    assert state["open_issue"] == "道路另一侧被遮挡"


def test_navigation_request_uses_single_live_record_and_stale_history_after_motion(tmp_path, monkeypatch):
    """记录进入原导航请求，运动后只剩历史确认，不额外请求复核。"""
    context = context_for(tmp_path)
    context.navigation_enabled = True
    context.navigation_instruction = "寻找仓库旁卡车"
    context.navigation_plan = parse_navigation_plan({"subgoals": [{"id": "a", "description": "找卡车",
        "completion_condition": "到卡车旁", "status": "in_progress"}, {"id": "b", "description": "到路口",
        "completion_condition": "位于路口", "status": "pending"}], "reason": "建立"}, "寻找仓库旁卡车再到路口")
    messages = [{"role": "user", "content": "寻找仓库旁卡车"}]
    append_observation(context, messages, snapshot_at())
    monkeypatch.setattr(flight, "rotate", Mock(return_value={"success": True, "degrees": 10}))
    monkeypatch.setattr(flight, "land", Mock(return_value={"success": True}))
    monkeypatch.setattr('drone_harness.runtime.agent_loop.time.sleep', lambda _seconds: None)
    client = FakeClient([
        SimpleNamespace(content=assessment("uncertain"), tool_calls=[tool_call("get_state", {})]),
        SimpleNamespace(content=assessment("confirmed", "入口位置区分了相似卡车"),
                        tool_calls=[tool_call("rotate", {"direction": "left", "degrees": 10})]),
        SimpleNamespace(content="降落结束", tool_calls=[tool_call("land", {})]),
    ])
    agent_loop(client, "test", messages, context)
    states = [json.loads(request[0]["content"].splitlines()[-1]) for request in client.requests]
    assert len(states) == 3
    assert states[0] == {"current": None, "previous": None}
    assert states[1]["current"]["target_status"] == "uncertain"
    assert states[2]["current"] is None
    assert states[2]["previous"]["support"] == "入口位置区分了相似卡车"
    assert all(request[0]["content"].count("参照物记录（均为模型判断") == 1 for request in client.requests)
