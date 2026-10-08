"""验证当前子目标完成边界与文字、单图保留。"""

from copy import deepcopy

import pytest

from drone_harness.runtime.navigation import (
    compact_navigation_history, current_subgoal_completed, parse_navigation_plan,
)
from drone_harness.llm.prompts import build_system_prompt, NAVIGATION_EXECUTION_PROMPT
from test_navigation_plan import plan_payload
from test_navigation_loop import nav_context, assert_tool_pairs


@pytest.mark.parametrize("change", ["description", "switch", "delete", "other_complete", "reorder"])
def test_replanning_without_current_completion_keeps_history(change):
    """未完成当前项时，改写、切换、删除或完成其他项都不清理。"""
    old = parse_navigation_plan(plan_payload(), "原任务")
    value = plan_payload()
    if change == "description":
        value["subgoals"][0]["description"] = "调整路线"
    elif change == "switch":
        value["subgoals"][0]["status"] = "pending"
        value["subgoals"][1]["status"] = "in_progress"
    elif change == "delete":
        value["subgoals"].pop(0)
    elif change == "other_complete":
        value["subgoals"][1]["status"] = "completed"
    else:
        value["subgoals"].reverse()
    assert not current_subgoal_completed(old, parse_navigation_plan(value, "原任务"))


@pytest.mark.parametrize("current", [True, False])
def test_cleanup_keeps_all_text_latest_image_and_paired_update(current):
    """保留文字顺序、最新图和更新协议，不修改先前请求对象。"""
    history = [
        {"role": "system", "content": "system"},
        {"role": "user", "content": "原始任务"},
        {"role": "assistant", "content": "路口前方有卡车", "tool_calls": [{"id": "a"}]},
        {"role": "tool", "tool_call_id": "a", "content": "旧动作结果"},
        {"role": "user", "content": [{"type": "text", "text": "旧深度"}]},
        {"role": "assistant", "content": "路口两侧是围栏"},
        {"role": "user", "content": "保留的用户补充"},
        {"role": "user", "content": [{"type": "text", "text": "最新配对深度"},
                                      {"type": "image_url", "image_url": {"url": "latest"}}]},
        {"role": "assistant", "content": "路口确认，进入下一段", "tool_calls": [{"id": "plan"}]},
        {"role": "tool", "tool_call_id": "plan", "content": "更新成功"},
    ]
    earlier_request = deepcopy(history)
    compact_navigation_history(history, 2, observation_current=current)
    assert "旧动作结果" not in str(history) and "旧深度" not in str(history)
    assert [m["content"] for m in history if m["role"] == "assistant"] == [
        "路口前方有卡车", "路口两侧是围栏", "路口确认，进入下一段"]
    assert "保留的用户补充" in str(history)
    assert "最新配对深度" in str(history) and "latest" in str(history)
    assert ("深度不代表当前位置" in str(history)) is (not current)
    assert_tool_pairs(history)
    assert "tool_calls" in earlier_request[2]
    compact_navigation_history(history, 2, observation_current=current)
    assert_tool_pairs(history)
    assert "路口前方有卡车" in str(history)


def test_prompt_describes_visual_memory_without_sentence_limit(tmp_path):
    """计划先行只作提示，图片描述不再限制一句两句。"""
    prompt = build_system_prompt(nav_context(tmp_path).profile)
    assert "一两句话" not in prompt
    assert "视觉记忆" in prompt and "不确定判断" in prompt
    assert "先使用 update_navigation_plan" in NAVIGATION_EXECUTION_PROMPT
