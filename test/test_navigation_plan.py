"""验证清单只做基本校验，允许模型自主改写。"""

from copy import deepcopy
from dataclasses import replace

import pytest

from drone_harness.runtime.navigation import current_subgoal_completed, parse_navigation_plan, update_navigation_plan
from drone_harness.tools.schemas import get_tool_schemas
from test_agent_observation_loop import context_for


def plan_payload(*, finish_action="land", statuses=("in_progress", "pending")):
    """生成完整清单参数，不依赖模型服务。"""
    return {"subgoals": [
        {"id": f"g{i + 1}", "description": title, "completion_condition": f"观察支持已{title}",
         "status": status, "evidence": ""}
        for i, (title, status) in enumerate(zip(("到路口", "到红门前"), statuses))],
        "finish_action": finish_action, "reason": "按指令更新计划"}


def test_explicit_status_and_pending_focus():
    """非连续完成与全待执行均按提交状态保存。"""
    plan = parse_navigation_plan(plan_payload(statuses=("pending", "completed")), "原文")
    assert plan.completed_count == 1 and plan.current.id == "g1"
    assert not plan.snapshot()["current_is_explicit"]
    assert plan.snapshot()["subgoals"][0]["status"] == "pending"
    assert "到红门前" in plan.conversation_summary()


@pytest.mark.parametrize("change", [
    lambda p: p.update(extra=True), lambda p: p.pop("reason"),
    lambda p: p.update(reason=" "), lambda p: p.update(reason=1),
    lambda p: p.update(finish_action=[]), lambda p: p.update(finish_action="fly"),
    lambda p: p.update(subgoals=[]), lambda p: p.update(subgoals={}),
    lambda p: p["subgoals"][0].update(id=" "),
    lambda p: p["subgoals"][0].update(description=""),
    lambda p: p["subgoals"][0].update(completion_condition=""),
    lambda p: p["subgoals"][0].update(evidence=None),
    lambda p: p["subgoals"][0].update(status="done"),
    lambda p: p["subgoals"][0].update(extra="x"),
    lambda p: p["subgoals"][1].update(id="g1"),
    lambda p: p["subgoals"][1].update(status="in_progress"),
])
def test_invalid_candidate_is_atomic(tmp_path, change):
    """非法清单返回可修正错误，原计划保持不变。"""
    context = context_for(tmp_path)
    context.navigation_enabled = True
    context.navigation_instruction = "原文"
    assert update_navigation_plan(context, plan_payload())["success"]
    old = context.navigation_plan
    before = deepcopy(old.snapshot())
    bad = plan_payload()
    change(bad)
    result = update_navigation_plan(context, bad)
    assert result["error"] == "INVALID_NAVIGATION_PLAN" and not result["plan_changed"]
    assert context.navigation_plan is old and old.snapshot() == before


def test_free_rewrite_reopen_delete_and_repeat(tmp_path):
    """允许批量完成、改写条件、调序删除和重开，重复提交不压缩。"""
    context = context_for(tmp_path)
    context.navigation_enabled = True
    context.navigation_instruction = "原始要求"
    payload = plan_payload(statuses=("completed", "completed"))
    assert update_navigation_plan(context, payload)["status"] == "finishing"
    assert context.observation is None
    payload["subgoals"].reverse()
    payload["subgoals"].pop()
    payload["subgoals"][0].update(status="in_progress", completion_condition="更正条件")
    payload["finish_action"] = "hold"
    result = update_navigation_plan(context, payload)
    assert result["success"] and result["status"] == "running" and not result["compact_history"]
    assert context.navigation_plan.original_instruction == "原始要求"
    repeated = update_navigation_plan(context, payload)
    assert not repeated["changed"] and not repeated["compact_history"]


def test_current_completion_even_when_future_plan_changes():
    """当前项完成可清理，允许同时修改后续计划。"""
    old = parse_navigation_plan(plan_payload(), "原文")
    value = plan_payload(statuses=("completed", "in_progress"))
    new = parse_navigation_plan(value, "原文")
    assert current_subgoal_completed(old, new)
    value["subgoals"][1]["description"] = "改变目标"
    assert current_subgoal_completed(old, parse_navigation_plan(value, "原文"))
    assert not current_subgoal_completed(new, old)
    assert not current_subgoal_completed(None, new)


@pytest.mark.parametrize("enabled,mode,visible", [(True, "simulation", True),
                                                   (False, "simulation", False), (True, "real", True),
                                                   (False, "real", False)])
def test_schema_and_handler_share_enable_boundary(tmp_path, enabled, mode, visible):
    """两种模式统一由阶段二开关控制计划工具。"""
    context = context_for(tmp_path)
    context.navigation_enabled = enabled
    context.profile = replace(context.profile, mode=mode)
    schemas = get_tool_schemas(context.profile, navigation_enabled=enabled)
    assert ("update_navigation_plan" in {s["function"]["name"] for s in schemas}) == visible
    assert update_navigation_plan(context, plan_payload())["success"] == visible


def test_finish_needs_both_complete_list_and_execution():
    """收尾成功标志不能覆盖未完成清单，清单完成也不能覆盖收尾失败。"""
    plan = parse_navigation_plan(plan_payload(), "原文")
    plan.finish(completed=True, reason="提前降落")
    assert plan.status == "incomplete"
    plan = parse_navigation_plan(plan_payload(statuses=("completed", "completed")), "原文")
    plan.finish(completed=False, reason="降落失败")
    assert plan.status == "incomplete"
