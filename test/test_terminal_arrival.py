"""验证终点阶段注入、位置依据与原有测距和收尾流程。"""

import json

import pytest

from drone_harness.llm.prompts import TERMINAL_COMPLETION_PROMPT
from drone_harness.runtime.agent_loop import agent_loop, append_observation
from drone_harness.runtime.navigation import navigation_messages, parse_navigation_plan
from drone_harness.runtime.reflection import record_target_assessment, target_assessment_snapshot
from drone_harness.runtime.target_measurement import target_measurement_snapshot
from drone_harness.tools import flight
from drone_harness.tools.registry import get_tool_definition
from test_depth_rules import snapshot_at
from test_navigation_loop import RecordingClient, messages, nav_context, reply
from test_navigation_plan import plan_payload
from test_reflection import assessment


def terminal_context(tmp_path):
    """建立已进入末项、具备当前前视图的离线上下文。"""
    context = nav_context(tmp_path)
    context.navigation_plan = parse_navigation_plan(
        plan_payload(statuses=("completed", "in_progress")), messages()[-1]["content"])
    append_observation(context, [], snapshot_at(17.0))
    return context


def arrival_reply(*, status="confirmed", position="met", evidence="当前观察支持位于指定门前"):
    """生成同次视觉判断，距离始终由真实工具另行提供。"""
    value = json.loads(assessment(status))
    value.update(candidate="指定红门的实体表面", support="入口外观及道路邻接关系符合指令")
    value["arrival"] = {"position_status": position, "evidence": evidence}
    return json.dumps(value, ensure_ascii=False)


def endpoint_state(request):
    """读取原导航请求中的单份终点依据。"""
    line = next(line for line in request[0]["content"].splitlines()
                if line.startswith('{"latest_target_measurement":'))
    return json.loads(line)


@pytest.mark.parametrize("statuses,expected", [
    (("in_progress", "pending"), False), (("completed", "pending"), True),
    (("completed", "in_progress"), True), (("pending", "in_progress"), False),
    (("completed", "completed"), False),
])
def test_stage_follows_plan_before_final_completion_and_without_mutation(statuses, expected):
    """前项未完成时不启动，末项打勾进入收尾后不再触发终点核对。"""
    plan = parse_navigation_plan(plan_payload(statuses=statuses), "原始指令")
    history = [{"role": "system", "content": "基础规则"}]
    result = navigation_messages(history, plan, observation_current=False)
    assert plan.terminal_stage is expected
    assert (TERMINAL_COMPLETION_PROMPT in result[0]["content"]) is expected
    assert plan.snapshot()["terminal_stage"] is expected
    assert history == [{"role": "system", "content": "基础规则"}]
    assert plan.completed_count == statuses.count("completed")


def test_single_goal_is_terminal_but_stopped_plan_is_not():
    """单段任务也能收尾，已停止或失败的计划不能误触发。"""
    payload = plan_payload()
    payload["subgoals"].pop()
    plan = parse_navigation_plan(payload, "到目标旁悬停")
    assert plan.terminal_stage
    plan.finish(completed=False, reason="中断")
    assert not plan.terminal_stage


@pytest.mark.parametrize("position", ["met", "unmet", "unknown"])
def test_position_assessment_does_not_supply_distance_or_complete_task(tmp_path, position):
    """位置满足不代表接近或身份正确，字段不会代替测距和清单。"""
    context = terminal_context(tmp_path)
    record_target_assessment(context, arrival_reply(status="uncertain", position=position), has_current_image=True)
    result = navigation_messages(messages(), context.navigation_plan, observation_current=True,
                                 target_assessment=target_assessment_snapshot(context))
    state = endpoint_state(result)
    assert state["target_assessment"]["current"]["target_status"] == "uncertain"
    assert state["arrival"]["position_status"] == position
    assert state["latest_target_measurement"] is None
    assert state["position_requirement"] == context.navigation_plan.current.completion_condition
    assert context.navigation_plan.status == "running" and context.navigation_plan.completed_count == 1


@pytest.mark.parametrize("arrival", [None, {}, {"position_status": "met"},
    {"position_status": "done", "evidence": "观察"}, {"position_status": "met", "evidence": " "},
    {"position_status": "met", "evidence": 17},
    {"position_status": "met", "evidence": "观察", "distance_m": 17}])
def test_invalid_position_preserves_identity_but_does_not_fabricate_arrival(tmp_path, arrival):
    """位置字段非法时保持位置未知，不丢失合法身份依据或接受模型距离。"""
    context = terminal_context(tmp_path)
    value = json.loads(assessment("confirmed"))
    value["arrival"] = arrival
    record_target_assessment(context, json.dumps(value), has_current_image=True)
    state = target_assessment_snapshot(context)["current"]
    assert state["target_status"] == "confirmed" and "arrival" not in state


def test_middle_stage_does_not_accept_terminal_position_fields(tmp_path):
    """中途不启用终点格式，普通接近仍使用既有测距规则。"""
    context = terminal_context(tmp_path)
    context.navigation_plan = parse_navigation_plan(plan_payload(), "原文")
    record_target_assessment(context, arrival_reply(), has_current_image=True)
    assert target_assessment_snapshot(context)["current"] is None
    record_target_assessment(context, assessment(), has_current_image=True)
    result = navigation_messages(messages(), context.navigation_plan, observation_current=True,
                                 target_assessment=target_assessment_snapshot(context))
    assert TERMINAL_COMPLETION_PROMPT not in result[0]["content"]
    assert '"arrival"' not in result[0]["content"]
    assert 'measure_distance' in result[0]["content"]


def test_stale_position_and_measurement_remain_unknown_after_view_is_lost(tmp_path):
    """旧位置met和旧有效距离均不能进入当前终点依据。"""
    context = terminal_context(tmp_path)
    record_target_assessment(context, arrival_reply(), has_current_image=True)
    get_tool_definition("measure_distance").handler(context, {"target": "红门正面", "x": .5, "y": .5})
    context.observation = None
    result = navigation_messages(messages(), context.navigation_plan, observation_current=False,
                                 target_assessment=target_assessment_snapshot(context),
                                 latest_target_measurement=target_measurement_snapshot(context))
    state = endpoint_state(result)
    assert state["arrival"] is None and state["latest_target_measurement"] is None
    assert state["target_assessment"]["current"] is None
    assert state["target_assessment"]["previous"]["arrival"]["position_status"] == "met"


def test_original_loop_can_measure_update_and_land_without_extra_verification(tmp_path, monkeypatch):
    """复用三次工具决策完成测距、清单和降落，不增加独立复核请求。"""
    context = terminal_context(tmp_path)
    history = messages()
    append_observation(context, history, context.observation)
    monkeypatch.setattr(flight, "land", lambda _context: {"success": True})
    client = RecordingClient([
        reply("measure_distance", {"target": "红门实体表面", "x": .5, "y": .5}, text=arrival_reply()),
        reply("update_navigation_plan", plan_payload(statuses=("completed", "completed")), text=arrival_reply()),
        reply("land"),
    ])
    agent_loop(client, "test", history, context)
    assert len(client.requests) == 3
    first, second = map(endpoint_state, client.requests[:2])
    assert first["arrival"] is None and first["latest_target_measurement"] is None
    assert second["arrival"]["position_status"] == "met"
    assert second["latest_target_measurement"]["valid"] is True
    assert second["latest_target_measurement"]["distance_m"] <= 20
    assert second["latest_target_measurement"]["target"] == "红门实体表面"
    assert "arrival" not in second["target_assessment"]["current"]
    assert client.requests[1][0]["content"].count('"latest_target_measurement"') == 1
    assert TERMINAL_COMPLETION_PROMPT not in client.requests[2][0]["content"]
    assert context.navigation_plan.status == "completed"


def test_unknown_endpoint_does_not_create_a_new_landing_gate(tmp_path, monkeypatch):
    """终点依据只提供反馈，不改变原有降落执行与结束行为。"""
    context = terminal_context(tmp_path)
    calls = []

    def land(_context):
        """记录模拟降落，不连接飞控。"""
        calls.append("land")
        return {"success": True}

    monkeypatch.setattr(flight, "land", land)
    client = RecordingClient([reply("land")])
    agent_loop(client, "test", messages(), context)
    assert calls == ["land"] and len(client.requests) == 1
    assert context.navigation_plan.status == "incomplete"


def test_switching_candidate_cannot_reuse_other_objects_measurement(tmp_path):
    """同图改认另一候选后，原候选有效读数不能支持新终点接近。"""
    context = terminal_context(tmp_path)
    record_target_assessment(context, arrival_reply(), has_current_image=True)
    get_tool_definition("measure_distance").handler(context, {"target": "红门正面", "x": .5, "y": .5})
    assert target_measurement_snapshot(context)["valid"]
    value = json.loads(arrival_reply(status="uncertain", position="unknown", evidence="另一入口位置待核实"))
    value["candidate"] = "道路另一侧的相似红门"
    record_target_assessment(context, json.dumps(value), has_current_image=True)
    assert target_measurement_snapshot(context) is None
    assert target_assessment_snapshot(context)["current"]["candidate"] == "道路另一侧的相似红门"
