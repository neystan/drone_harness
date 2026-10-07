"""离线验证阶段二续接、事实记忆、有限纠错与观察归档。"""

import base64
from dataclasses import replace
import json
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

import drone_harness.runtime.agent_loop as loop
from drone_harness.runtime.navigation import parse_navigation_plan
from drone_harness.runtime.runtime import _run_interactive_loop
from drone_harness.runtime.safety import EndCurrentTurn
from test_navigation_loop import (
    RecordingClient, assert_tool_pairs, messages, nav_context, observe_reply, planning_reply, reply,
)
from test_navigation_plan import plan_payload


def test_prompt_contains_general_completion_examples(tmp_path):
    """完成示例覆盖不同空间关系，不要求机械环视。"""
    context = nav_context(tmp_path)
    client = RecordingClient([reply(text="收到")])
    loop.agent_loop(client, "test", messages(), context)
    prompt = client.requests[0][0]["content"]
    assert all(word in prompt for word in ("十字路口中央", "蓝色卡车", "白色建筑", "不必固定环视"))


@pytest.mark.parametrize("enabled", [False, True])
def test_archive_matches_model_image(tmp_path, enabled):
    """只归档发给模型的 JPEG，关闭时没有图片文件。"""
    context = nav_context(tmp_path)
    context.profile = replace(context.profile, storage=replace(
        context.profile.storage, save_observation_images=enabled))
    client = RecordingClient([observe_reply(), reply(text="已观察")])
    loop.agent_loop(client, "test", messages(), context)
    directory = tmp_path / "logs/session_test"
    event = json.loads((directory / "observations.jsonl").read_text().splitlines()[0])
    if enabled:
        item = next(item for item in client.requests[1] if isinstance(item["content"], list))
        url = next(part["image_url"]["url"] for part in item["content"] if part["type"] == "image_url")
        assert (directory / event["image_path"]).read_bytes() == base64.b64decode(url.split(",", 1)[1])
    else:
        assert not list(directory.rglob("*.jpg"))
        assert event.get("image_path") is None
    assert "base64" not in (directory / "observations.jsonl").read_text()


def test_multiple_calls_recover_and_log_without_dispatch(tmp_path, monkeypatch):
    """多工具全拒绝后可重新选择，日志及协议完整。"""
    context = nav_context(tmp_path)
    dispatch = Mock(side_effect=AssertionError("不得执行"))
    monkeypatch.setattr(loop, "dispatch_tool_call", dispatch)
    response = planning_reply()
    response.tool_calls += reply("takeoff", {"height": 2}, call_id="other").tool_calls
    client = RecordingClient([response, reply(text="已纠正")])
    history = messages()
    assert loop.agent_loop(client, "test", history, context) == "已纠正"
    dispatch.assert_not_called()
    assert_tool_pairs(history)
    records = [json.loads(s) for s in (tmp_path / "logs/session_test/tool_calls.jsonl").read_text().splitlines()]
    assert len(records) == 2
    assert all(r["result"]["motion_executed"] is False for r in records)


@pytest.mark.parametrize("tool,error", [("takeoff", "ALREADY_IN_AIR"), ("down", "TARGET_Z_TOO_LOW")])
def test_recoverable_error_has_two_correction_chances(tmp_path, monkeypatch, tool, error):
    """第三次连续错误停轮，不自动重放动作。"""
    context = nav_context(tmp_path)
    dispatch = Mock(return_value={"success": False, "error": error})
    monkeypatch.setattr(loop, "dispatch_tool_call", dispatch)
    client = RecordingClient([reply(tool), reply(tool), reply(tool), reply(text="不应请求")])
    answer = loop.agent_loop(client, "test", messages(), context)
    assert dispatch.call_count == 3 and len(client.requests) == 3
    assert "停止" in answer


@pytest.mark.parametrize("error", ["TIMEOUT", "POSITION_INVALID", "HUMAN_IN_THE_LOOP_DECLINED"])
def test_other_errors_still_stop(tmp_path, monkeypatch, error):
    """未进入白名单的错误不恢复。"""
    context = nav_context(tmp_path)
    monkeypatch.setattr(loop, "dispatch_tool_call", lambda *_: {"success": False, "error": error})
    client = RecordingClient([reply("down"), reply(text="不应请求")])
    loop.agent_loop(client, "test", messages(), context)
    assert len(client.requests) == 1


def test_guidance_resumes_plan_and_facts_without_images(tmp_path, monkeypatch):
    """引导保留原目标与动作事实，中断动作不被当作成功。"""
    context = nav_context(tmp_path)
    inputs = iter(["去路口后到红门", "降低一点", "exit"])
    context.message_bus = SimpleNamespace(consume_user_message=lambda: SimpleNamespace(content=next(inputs)),
                                          has_pending_user_message=lambda: False)
    original_dispatch = loop.dispatch_tool_call

    def dispatch(ctx, call):
        """模拟移动中断，其他工具沿用真实分发。"""
        if call.function.name == "forward":
            raise EndCurrentTurn("用户中断", {"success": False, "error": "INTERRUPTED_BY_USER",
                                               "final_position_ned": [2, 0, -3]})
        return original_dispatch(ctx, call)

    monkeypatch.setattr(loop, "dispatch_tool_call", dispatch)
    client = RecordingClient([planning_reply(), observe_reply(), reply("forward", {"distance_m": 5}),
                              planning_reply(("completed", "completed"), "hold")])
    _run_interactive_loop(client, "test", context, loop.agent_loop, input_terminal_started=True)
    request = client.requests[-1]
    assert "去路口后到红门" in request[0]["content"]
    assert "当前导航计划" in request[0]["content"]
    assert "INTERRUPTED_BY_USER" in request[0]["content"]
    assert "[2, 0, -3]" in request[0]["content"]
    assert all(isinstance(item["content"], str) for item in request)
    assert context.navigation_plan.original_instruction == "去路口后到红门"
    assert context.task_state.landing_authorized is False


def test_compaction_retains_actual_motion_fact(tmp_path, monkeypatch):
    """纯推进删除工具过程后，实际结束位置仍进入下一请求。"""
    context = nav_context(tmp_path)
    from drone_harness.tools import flight
    monkeypatch.setattr(flight, "up", Mock(return_value={"success": True, "final_position_ned": [0, 0, -7]}))
    client = RecordingClient([planning_reply(), reply("up", {"distance_m": 5}), observe_reply(),
                              planning_reply(("completed", "in_progress")),
                              planning_reply(("completed", "completed"), "hold")])
    loop.agent_loop(client, "test", messages(), context)
    assert "[0, 0, -7]" in client.requests[-1][0]["content"]
    assert "last_motion" in client.requests[-1][0]["content"]
    assert_tool_pairs(client.requests[-1])


@pytest.mark.parametrize("instruction", ["取消任务", "停止任务", "新任务：去蓝门"])
def test_explicit_cancel_or_replace_does_not_resume(tmp_path, instruction):
    """明确取消不请求模型，新任务不会继承旧计划。"""
    context = nav_context(tmp_path)
    context.navigation_plan = parse_navigation_plan(plan_payload(), "旧任务")
    context.navigation_plan.status = "incomplete"
    inputs = iter([instruction, "继续", "exit"] if instruction != "新任务：去蓝门" else [instruction, "exit"])
    context.message_bus = SimpleNamespace(consume_user_message=lambda: SimpleNamespace(content=next(inputs)),
                                          has_pending_user_message=lambda: False)
    client = RecordingClient([reply(text="收到新输入")])
    _run_interactive_loop(client, "test", context, loop.agent_loop, input_terminal_started=True)
    assert len(client.requests) == 1
    assert "当前导航计划" not in client.requests[0][0]["content"]
    assert context.navigation_plan is None


def test_image_write_failure_does_not_end_observation(tmp_path, monkeypatch, capsys):
    """图片无法落盘不丢模型观察，且提供可见诊断。"""
    from pathlib import Path
    context = nav_context(tmp_path)
    context.profile = replace(context.profile, storage=replace(context.profile.storage, save_observation_images=True))
    original = Path.open

    def fail_image(path, *args, **kwargs):
        """只模拟图片写失败，不影响 JSONL。"""
        if path.suffix == ".jpg":
            raise OSError("test disk failure")
        return original(path, *args, **kwargs)

    monkeypatch.setattr(Path, "open", fail_image)
    client = RecordingClient([observe_reply(), reply(text="已看到")])
    assert loop.agent_loop(client, "test", messages(), context) == "已看到"
    assert "图片保存失败" in capsys.readouterr().err
    assert any(isinstance(item["content"], list) for item in client.requests[-1])


@pytest.mark.parametrize("value", ["false", 1, None])
def test_image_switch_rejects_non_boolean(tmp_path, value):
    """配置不会把字符串 false 隐式解释成开启。"""
    context = nav_context(tmp_path)
    with pytest.raises(ValueError, match="boolean"):
        replace(context.profile.storage, save_observation_images=value)


def test_query_and_plan_do_not_reset_correction_limit(tmp_path, monkeypatch):
    """插入查询或更新计划不能绕过本轮纠错上限。"""
    context = nav_context(tmp_path)
    original = loop.dispatch_tool_call

    def dispatch(ctx, call):
        """用假拒绝及状态绕开真实飞控。"""
        if call.function.name == "down":
            return {"success": False, "error": "TARGET_Z_TOO_LOW"}
        if call.function.name == "get_state":
            return {"success": True, "in_air": True}
        return original(ctx, call)

    monkeypatch.setattr(loop, "dispatch_tool_call", dispatch)
    client = RecordingClient([reply("down"), reply("get_state"), reply("down"), planning_reply(),
                              reply("down"), reply(text="不应请求")])
    loop.agent_loop(client, "test", messages(), context)
    assert len(client.requests) == 5
    assert context.navigation_plan.status == "incomplete"


def test_recovery_then_success_continues(tmp_path, monkeypatch):
    """收到错误后由模型选择新动作，而非重放失败调用。"""
    context = nav_context(tmp_path)
    dispatch = Mock(side_effect=[{"success": False, "error": "ALREADY_IN_AIR"},
                                 {"success": True, "final_position_ned": [0, 0, -4]}])
    monkeypatch.setattr(loop, "dispatch_tool_call", dispatch)
    client = RecordingClient([reply("takeoff"), reply("up", {"distance_m": 2}), reply(text="已调整")])
    assert loop.agent_loop(client, "test", messages(), context) == "已调整"
    assert [call.args[1].function.name for call in dispatch.call_args_list] == ["takeoff", "up"]
    assert_tool_pairs(client.requests[-1])


@pytest.mark.parametrize("tool,result", [
    ("rotate", {"success": False, "error": "ALREADY_IN_AIR"}),
    ("down", {"success": False, "error": "TARGET_Z_TOO_LOW", "motion_executed": True}),
])
def test_whitelist_requires_correct_tool_and_no_executed_motion(tmp_path, monkeypatch, tool, result):
    """不能仅凭错误字符串放行，也不能恢复已执行动作。"""
    context = nav_context(tmp_path)
    monkeypatch.setattr(loop, "dispatch_tool_call", Mock(return_value=result))
    client = RecordingClient([reply(tool), reply(text="不应请求")])
    loop.agent_loop(client, "test", messages(), context)
    assert len(client.requests) == 1


@pytest.mark.parametrize("status,resumable,expected", [
    ("incomplete", True, True), ("completed", True, False),
    ("cancelled", True, False), ("incomplete", False, False),
])
def test_resume_preserves_counters_but_clears_authority(tmp_path, status, resumable, expected):
    """续接不绕过无位移限制，完成或安全退出后不恢复。"""
    from drone_harness.runtime.task_memory import prepare_navigation_turn
    context = nav_context(tmp_path)
    context.navigation_plan = parse_navigation_plan(plan_payload(), "原任务")
    context.navigation_plan.status = status
    context.navigation_resumable = resumable
    context.task_state.consecutive_no_progress = 3
    context.task_state.landing_authorized = True
    context.observation = object()
    context.depth_rules = object()
    assert prepare_navigation_turn(context, "继续") is expected
    assert context.observation is None and context.depth_rules is None
    assert context.task_state.landing_authorized is False
    if expected:
        assert context.task_state.consecutive_no_progress == 3
        assert context.task_state.current_user_goal == "原任务"
    else:
        assert context.navigation_plan is None


def test_failed_attempt_does_not_erase_successful_motion(tmp_path):
    """最近失败与最近成功动作分别保留，摘要有界且不是引用原结果。"""
    from drone_harness.runtime.task_memory import record_execution_fact
    context = nav_context(tmp_path)
    result = {"success": True, "final_position_ned": [19, 0, -30], "target_position_ned": [18, 0, -30]}
    record_execution_fact(context, "forward", result)
    result["final_position_ned"][0] = 999
    for _ in range(10):
        record_execution_fact(context, "takeoff", {"success": False, "error": "ALREADY_IN_AIR"})
    assert len(context.execution_facts) == 3
    assert context.execution_facts["last_successful_motion"]["result"]["final_position_ned"] == [19, 0, -30]
    assert "target_position_ned" not in str(context.execution_facts)


def test_real_correction_requires_new_approval(tmp_path, monkeypatch):
    """实机拒绝后重新规划的动作仍需再次审批。"""
    from test_real_hitl import real_context
    from drone_harness.tools import flight
    context = real_context(tmp_path)
    context.navigation_enabled = True
    context.profile = replace(context.profile, post_motion_wait_enabled=False)
    answers = Mock(return_value=SimpleNamespace(content="y"))
    context.message_bus = SimpleNamespace(get_next_user_message=answers, has_pending_user_message=lambda: False)
    takeoff = Mock(return_value={"success": False, "error": "ALREADY_IN_AIR"})
    up = Mock(return_value={"success": True, "final_position_ned": [0, 0, -4]})
    monkeypatch.setattr(flight, "takeoff", takeoff)
    monkeypatch.setattr(flight, "up", up)
    client = RecordingClient([reply("takeoff", {"height": 2}), reply("up", {"distance_m": 2}), reply(text="完成动作")])
    loop.agent_loop(client, "test", messages(), context)
    assert answers.call_count == 2
    takeoff.assert_called_once()
    up.assert_called_once()


def test_actual_takeoff_rejection_does_not_publish_target(tmp_path, monkeypatch):
    """核查恢复白名单对应的真实起飞分支在目标下发前返回。"""
    from drone_harness.tools import flight
    from test_move_safety import motion_context
    context = motion_context(tmp_path)
    monkeypatch.setattr(flight, "_wait_for_valid_position", lambda _: True)
    result = flight.takeoff(context, 2)
    assert result["error"] == "ALREADY_IN_AIR"
    context.controller.start_position_hold.assert_not_called()


def test_multiple_call_budget_and_logs_are_bounded(tmp_path):
    """反复多调用在第三次停止，所有拒绝均有日志。"""
    context = nav_context(tmp_path)
    response = planning_reply()
    response.tool_calls += reply("takeoff", {"height": 2}, call_id="other").tool_calls
    client = RecordingClient([response] * 4)
    history = messages()
    assert "纠错次数已用尽" in loop.agent_loop(client, "test", history, context)
    assert len(client.requests) == 3
    assert_tool_pairs(history)
    assert len((tmp_path / "logs/session_test/tool_calls.jsonl").read_text().splitlines()) == 6
