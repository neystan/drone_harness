"""验证目标测距来自实际工具，且只在当前来源有效时进入导航请求。"""

import json
from dataclasses import replace
from unittest.mock import Mock

import numpy as np
import pytest

from drone_harness.runtime.agent_loop import agent_loop, append_observation
from drone_harness.runtime.navigation import parse_navigation_plan, update_navigation_plan
from drone_harness.runtime.skill_search import activate_skill
from drone_harness.runtime.target_measurement import target_measurement_snapshot
from drone_harness.runtime.task_memory import prepare_navigation_turn
from drone_harness.tools import flight
from drone_harness.tools.registry import get_tool_definition
from drone_harness.tools.schemas import MEASURE_DISTANCE_TOOL_SCHEMA
from test_depth_rules import snapshot_at
from test_navigation_loop import RecordingClient, messages, nav_context, reply
from test_navigation_plan import plan_payload
from test_search_skill import scan_context


def measured_context(tmp_path, distance=18.6):
    """建立带原始清单与实际深度测量的上下文。"""
    context = nav_context(tmp_path)
    context.navigation_instruction = "沿道路到路口，再到红门前降落"
    context.navigation_plan = parse_navigation_plan(plan_payload(), context.navigation_instruction)
    context.observation = snapshot_at(distance)
    result = get_tool_definition("measure_distance").handler(
        context, {"target": "  指定红门的实体表面  ", "x": .5, "y": .6})
    return context, result


def measurement_in(request):
    """读取实际模型请求中单份当前测距摘要。"""
    return json.loads(request[0]["content"].splitlines()[-1])["latest_target_measurement"]


def test_target_schema_and_actual_measurement_binding(tmp_path):
    """目标说明必填，摘要只含真实数值和模型说明，不泄露内部编号。"""
    schema = MEASURE_DISTANCE_TOOL_SCHEMA["function"]["parameters"]
    assert schema["required"] == ["target", "x", "y"]
    context, result = measured_context(tmp_path)
    summary = target_measurement_snapshot(context)
    assert summary == {"subgoal_id": "g1", "target": "指定红门的实体表面",
                       "point": {"x": .5, "y": .6}, "valid": True,
                       "distance_m": result["distance_m"], "reason": "OK"}
    assert result["target"] == summary["target"]
    assert context.target_measurement_observation_id == context.observation.observation_id
    assert context.target_measurement_goal == (
        context.navigation_instruction, "g1", "到路口", "观察支持已到路口")
    summary["point"]["x"] = 0
    assert context.latest_target_measurement["point"]["x"] == .5


@pytest.mark.parametrize("distance,valid,reason", [
    (19.9, True, "OK"), (20, True, "OK"), (20.1, True, "OK"),
    (30, False, "OUT_OF_RANGE"), (np.nan, False, "INSUFFICIENT_VALID_DEPTH"),
])
def test_invalid_result_replaces_old_success_without_automatic_completion(tmp_path, distance, valid, reason):
    """范围内如实返回，远距或缺测替换成功读数，不自动勾选目标。"""
    context, _ = measured_context(tmp_path)
    context.observation.depth[:] = distance
    result = get_tool_definition("measure_distance").handler(
        context, {"target": "目标表面", "x": .5, "y": .5})
    summary = target_measurement_snapshot(context)
    assert summary["valid"] is valid and summary["reason"] == reason
    assert summary["distance_m"] == result["distance_m"]
    assert (summary["distance_m"] is not None) is valid
    assert context.navigation_plan.completed_count == 0


@pytest.mark.parametrize("target", [None, "", "  ", 5, [], "a" * 1001])
def test_invalid_target_clears_old_reading_without_measuring(tmp_path, monkeypatch, target):
    """缺少或错误目标说明不可沿用旧读数，也不触发测距。"""
    context, _ = measured_context(tmp_path)
    geometry = Mock(side_effect=AssertionError("不应测距"))
    monkeypatch.setattr("drone_harness.tools.registry.measure_target_distance", geometry)
    result = get_tool_definition("measure_distance").handler(
        context, {"target": target, "x": .5, "y": .5})
    assert result["reason"] == "INVALID_TARGET" and not result["valid"]
    assert target_measurement_snapshot(context) is None
    geometry.assert_not_called()


def test_invalid_point_does_not_keep_previous_valid_distance(tmp_path):
    """错误选点保留本次无效结果，避免旧距离冒充本次。"""
    context, _ = measured_context(tmp_path)
    get_tool_definition("measure_distance").handler(
        context, {"target": "目标表面", "x": "未知", "y": .5})
    summary = target_measurement_snapshot(context)
    assert summary["reason"] == "INVALID_POINT" and summary["point"] is None
    assert summary["distance_m"] is None and not summary["valid"]


@pytest.mark.parametrize("field", ["id", "description", "completion_condition", "instruction", "observation"])
def test_changed_source_discards_measurement(tmp_path, field):
    """任务、当前目标内容或观察变化后旧测量不得跨用。"""
    context, _ = measured_context(tmp_path)
    if field == "instruction":
        context.navigation_plan.original_instruction = "换一个任务"
    elif field == "observation":
        context.observation = replace(context.observation, observation_id="new-frame")
    else:
        setattr(context.navigation_plan.current, field, "改变的目标")
    assert target_measurement_snapshot(context) is None
    assert context.latest_target_measurement is None


def test_plan_reason_and_future_goal_edits_preserve_current_measurement(tmp_path):
    """说明或后续段调整不改变当前测距，当前项完成后则清除。"""
    context, _ = measured_context(tmp_path)
    previous = target_measurement_snapshot(context)
    payload = plan_payload()
    payload["reason"] = "补充说明"
    payload["subgoals"][1]["description"] = "后续红门"
    assert update_navigation_plan(context, payload)["success"]
    assert target_measurement_snapshot(context) == previous
    payload["subgoals"][0]["status"] = "completed"
    assert update_navigation_plan(context, payload)["success"]
    assert context.latest_target_measurement is None


@pytest.mark.parametrize("failed", [False, True])
def test_new_observation_attempt_clears_measurement(tmp_path, failed):
    """重新观察成功或失败均不能复用上一次帧的测距。"""
    context, _ = measured_context(tmp_path)
    result = get_tool_definition("observe").handler(
        context, {"prompt": "重新核对目标", "view": "invalid" if failed else "front"})
    assert bool(result["success"]) is not failed
    assert context.latest_target_measurement is None


@pytest.mark.parametrize("replace_task", [False, True])
def test_turn_resume_and_task_replacement_clear_measurement(tmp_path, replace_task):
    """用户续接或更换任务均刷新观察和实际测距。"""
    context, _ = measured_context(tmp_path)
    prepare_navigation_turn(context, "继续" if not replace_task else "新任务", replace_task=replace_task)
    assert context.latest_target_measurement is None


@pytest.mark.parametrize("failed", [False, True])
def test_four_direction_search_clears_measurement(tmp_path, monkeypatch, failed):
    """扫描成功或中途失败都清除扫描前的测距记录。"""
    context = scan_context(tmp_path, monkeypatch, [])
    context.observation = snapshot_at(8)
    get_tool_definition("measure_distance").handler(context, {"target": "目标", "x": .5, "y": .5})
    if failed:
        context.controller.wait_for_observation = Mock(return_value=None)
    result = activate_skill(context, {"name": "four-direction-search", "prompt": "寻找目标"})
    assert bool(result["success"]) is not failed
    assert context.latest_target_measurement is None


@pytest.mark.parametrize("tool,result,cleared", [
    ("rotate", {"success": True, "degrees": 90}, True),
    ("forward", {"success": True, "motion_executed": True}, True),
    ("forward", {"success": True, "motion_executed": False}, False),
    ("up", {"success": True}, True), ("down", {"success": True}, True),
    ("takeoff", {"success": True}, True), ("land", {"success": True}, True),
])
def test_motion_invalidation_keeps_nonexecuted_forward_reading(tmp_path, monkeypatch, tool, result, cleared):
    """执行动作后清除测距，明确未前进时保留原有效来源。"""
    context = nav_context(tmp_path)
    context.observation = snapshot_at(8)
    context.navigation_instruction = messages()[1]["content"]
    get_tool_definition("measure_distance").handler(context, {"target": "目标", "x": .5, "y": .5})
    monkeypatch.setattr(flight, tool, lambda *_args: result)
    client = RecordingClient([reply(tool, {"distance_m": 1, "direction": "right", "degrees": 90}),
                              reply(text="继续核对")])
    agent_loop(client, "test", messages(), context)
    assert (context.latest_target_measurement is None) is cleared


def test_actual_loop_exposes_only_current_measurement_without_extra_requests(tmp_path, monkeypatch):
    """真实工具反馈进入原请求，状态查询保留，旋转后改为无当前测距。"""
    context = nav_context(tmp_path)
    history = messages()
    append_observation(context, history, snapshot_at(18.6))
    monkeypatch.setattr(flight, "rotate", lambda *_: {"success": True, "degrees": 90})
    client = RecordingClient([
        reply("measure_distance", {"target": "红门正面", "x": .5, "y": .5}),
        reply("get_state"), reply("rotate", {"direction": "right", "degrees": 90}),
        reply("measure_distance", {"target": "红门正面", "x": .5, "y": .5}),
        reply(text="需要重新观察"),
    ])
    agent_loop(client, "test", history, context)
    assert len(client.requests) == 5
    assert measurement_in(client.requests[0]) is None
    first = measurement_in(client.requests[1])
    assert first["valid"] and first["target"] == "红门正面"
    assert first["distance_m"] == pytest.approx(18.6, abs=1e-5)
    assert measurement_in(client.requests[2]) == first
    assert measurement_in(client.requests[3]) is None
    assert measurement_in(client.requests[4])["reason"] == "OBSERVATION_REQUIRED"
    assert not measurement_in(client.requests[4])["valid"]
    feedback = json.loads(client.requests[1][-1]["content"])
    assert set(feedback) == {"success", "valid", "distance_m", "reason", "message"}
    assert feedback["valid"] and feedback["distance_m"] == first["distance_m"]
    raw_events = [json.loads(line) for line in
                  (tmp_path / "logs/session_test/tool_calls.jsonl").read_text().splitlines()]
    raw = next(event["result"] for event in raw_events if event["tool_name"] == "measure_distance")
    assert raw["target"] == first["target"] and raw["sample_count"] == raw["valid_count"] == 25
    assert raw["pixel"] == {"x": 8, "y": 6}
    assert all(request[0]["content"].count('"latest_target_measurement"') == 1
               for request in client.requests)
