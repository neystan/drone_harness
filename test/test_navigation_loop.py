"""用假模型、合成观测和假控制器验证清单工具闭环。"""

from dataclasses import replace
import json
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

import drone_harness.runtime.agent_loop as loop_module
from drone_harness.runtime.navigation import parse_navigation_plan
from drone_harness.runtime.runtime import _run_interactive_loop
from drone_harness.runtime.safety import EndCurrentTurn, SafetyHandoffRequired
from drone_harness.runtime.tool_dispatcher import dispatch_tool_call
from drone_harness.tools import flight
from test_agent_observation_loop import FakeClient, context_for
from test_navigation_plan import plan_payload
from test_stage3_observe import fresh_snapshot, tool_call


class RecordingClient(FakeClient):
    """保存实际请求选项，核查工具与同一模型链。"""

    def __init__(self, replies):
        """初始化假响应和请求记录。"""
        super().__init__(replies)
        self.options = []

    def create(self, **kwargs):
        """记录模型请求但不连接服务。"""
        self.options.append(kwargs)
        return super().create(**kwargs)


def reply(name=None, args=None, text="", call_id="call-1"):
    """构造自然语言和可选单工具回复。"""
    return SimpleNamespace(content=text, tool_calls=[tool_call(name, args or {}, call_id)] if name else [])


def planning_reply(statuses=("in_progress", "pending"), finish_action="land"):
    """模型通过工具提交完整计划。"""
    return reply("update_navigation_plan", plan_payload(statuses=statuses, finish_action=finish_action))


def nav_context(tmp_path):
    """建立无 ROS 的仿真测试上下文。"""
    controller = SimpleNamespace(
        wait_for_observation=lambda *, after_stamp_ns: fresh_snapshot(after_stamp_ns),
        vehicle_status=SimpleNamespace(connected=True, armed=True, mode="AUTO.LOITER"),
        flight_state=lambda: "IN_AIR", current_position_ned=lambda: (0.0, 0.0, -2.0))
    context = context_for(tmp_path, controller)
    context.profile = replace(context.profile, post_motion_wait_enabled=False)
    context.navigation_enabled = True
    return context


def messages():
    """提供本轮原始指令。"""
    return [{"role": "system", "content": "原飞行规则"},
            {"role": "user", "content": "沿道路到路口，再到红门前降落"}]


def observe_reply():
    """取得一组新的配对观察。"""
    return reply("observe", {"prompt": "观察道路和红门"})


def events(tmp_path):
    """读取离线目录中的计划事件。"""
    return [json.loads(line) for line in (tmp_path / "logs/session_test/navigation_plan.jsonl").read_text().splitlines()]


def assert_tool_pairs(history):
    """所有工具调用与结果必须完整配对。"""
    pending = []
    for item in history:
        if pending:
            assert item["role"] == "tool" and item["tool_call_id"] == pending.pop(0)
        elif item["role"] == "assistant" and item.get("tool_calls"):
            pending = [call["id"] for call in item["tool_calls"]]
        else:
            assert item["role"] != "tool"
    assert not pending


def test_one_input_runs_motion_progress_observation_and_landing(tmp_path, monkeypatch):
    """一次输入自动导航，原前进深度门和实际位移反馈仍生效。"""
    context = nav_context(tmp_path)
    inputs = iter([messages()[-1]["content"], "exit"])
    context.message_bus = SimpleNamespace(consume_user_message=lambda: SimpleNamespace(content=next(inputs)),
                                          has_pending_user_message=lambda: False)
    pose = [0., 0., -2.]
    context.controller.current_position_ned = lambda: tuple(pose)

    def move(_context, forward_m, _right, _down, **_kwargs):
        """用位置变化替代飞控执行。"""
        pose[0] += forward_m
        return {"success": True, "final_position_ned": list(pose)}

    moved = Mock(side_effect=move)
    landed = Mock(return_value={"success": True})
    monkeypatch.setattr(flight, "move", moved)
    monkeypatch.setattr(flight, "land", landed)
    client = RecordingClient([planning_reply(), observe_reply(),
                              planning_reply(("completed", "in_progress")),
                              reply("forward", {"distance_m": 1}), observe_reply(),
                              planning_reply(("completed", "completed")), reply("land")])
    _run_interactive_loop(client, "same-vlm", context, loop_module.agent_loop, input_terminal_started=True)
    assert len(client.requests) == 7 and context.navigation_plan.status == "completed"
    assert pose[0] == 1 and context.navigation_plan.completed_count == 2
    moved.assert_called_once()
    landed.assert_called_once()
    for option in client.options:
        assert option["model"] == "same-vlm" and len(option["tools"]) == 9
    assert '"current_subgoal_id": "g2"' in client.requests[3][0]["content"]
    for history in client.requests:
        assert_tool_pairs(history)
        assert sum(part["type"] == "image_url" for item in history if isinstance(item["content"], list)
                   for part in item["content"]) <= 1
    assert [event["event_type"] for event in events(tmp_path)] == ["created", "updated", "updated", "completed"]
    assert "未独立核验" in events(tmp_path)[-1]["plan"]["stop_reason"]


@pytest.mark.parametrize("raw", ['{bad', '[]', '{"subgoals": []}', '{"reason": NaN}'])
def test_invalid_plan_can_be_corrected_without_motion(tmp_path, raw):
    """格式错误返回工具反馈，旧状态不丢失，模型可继续修正。"""
    context = nav_context(tmp_path)
    bad = planning_reply()
    bad.tool_calls[0].function.arguments = raw
    client = RecordingClient([planning_reply(), bad, planning_reply(("completed", "completed"), "hold")])
    loop_module.agent_loop(client, "vlm", messages(), context)
    assert context.navigation_plan.status == "completed"
    assert len(client.requests) == 3
    assert '"success": false' in client.requests[2][-1]["content"]
    assert '"completed_count": 0' in client.requests[2][0]["content"]


def test_plain_text_and_old_json_are_not_completion_protocol(tmp_path, monkeypatch):
    """自由文字不会解析失败或推进计划，预算仍有限。"""
    context = nav_context(tmp_path)
    monkeypatch.setattr(loop_module, "MAX_NAVIGATION_REQUESTS", 4)
    client = RecordingClient([planning_reply(), reply(text="到达了\n{bad json"),
                              reply(text='{"subgoal_complete":true}'), reply(text="继续")])
    assert "预算耗尽" in loop_module.agent_loop(client, "vlm", messages(), context)
    assert context.navigation_plan.completed_count == 0
    assert len(client.requests) == 4


def test_completion_without_fresh_observation_does_not_block_plan(tmp_path, monkeypatch):
    """旧观测仍失效，但清单写入不再有完成观测硬门。"""
    context = nav_context(tmp_path)
    monkeypatch.setattr(flight, "rotate", Mock(return_value={"success": True, "degrees": 15}))
    client = RecordingClient([planning_reply(), observe_reply(),
                              reply("rotate", {"direction": "left", "degrees": 15}),
                              planning_reply(("completed", "completed"), "hold")])
    loop_module.agent_loop(client, "vlm", messages(), context)
    assert '"observation_current": false' in client.requests[-1][0]["content"]
    assert context.observation is None and context.navigation_plan.status == "completed"


def test_updates_preserve_observation_counter_and_do_not_wait(tmp_path, monkeypatch):
    """计划不是动作，不等待、不清图，也不重置无位移保护。"""
    context = nav_context(tmp_path)
    context.profile = replace(context.profile, post_motion_wait_enabled=True)
    context.task_state.consecutive_no_progress = 2
    wait = Mock(side_effect=AssertionError("清单不应等待"))
    monkeypatch.setattr(loop_module.time, "sleep", wait)
    client = RecordingClient([observe_reply(), planning_reply(), planning_reply(),
                              planning_reply(("completed", "completed"), "hold")])
    loop_module.agent_loop(client, "vlm", messages(), context)
    assert context.observation is not None and context.task_state.consecutive_no_progress == 2
    assert len(events(tmp_path)) == 3
    wait.assert_not_called()


def test_rewrite_preserves_history_and_can_reopen_finishing(tmp_path, monkeypatch):
    """改写不是完成压缩，收尾前允许重新打开计划。"""
    context = nav_context(tmp_path)
    changed = plan_payload()
    changed["subgoals"].reverse()
    changed["subgoals"][0]["description"] = "重新找入口"
    monkeypatch.setattr(flight, "land", Mock(return_value={"success": True}))
    client = RecordingClient([planning_reply(), reply(text="需要确认入口"),
                              reply("update_navigation_plan", changed),
                              planning_reply(("completed", "completed")), planning_reply(), reply("land")])
    loop_module.agent_loop(client, "vlm", messages(), context)
    assert "需要确认入口" in str(client.requests[3])
    assert '"status": "running"' in client.requests[5][0]["content"]
    assert context.navigation_plan.status == "incomplete"


@pytest.mark.parametrize("success,all_done", [(True, False), (False, False), (False, True), (True, True)])
def test_landing_completion_depends_on_both_list_and_result(tmp_path, monkeypatch, success, all_done):
    """清单打勾和实际降落结果分别记录。"""
    context = nav_context(tmp_path)
    monkeypatch.setattr(flight, "land", Mock(return_value={"success": success}))
    client = RecordingClient([planning_reply(("completed", "completed") if all_done else ("in_progress", "pending")),
                              reply("land")])
    loop_module.agent_loop(client, "vlm", messages(), context)
    assert (context.navigation_plan.status == "completed") == (success and all_done)


@pytest.mark.parametrize("hover_fails", [False, True])
def test_hold_preserves_hover_handoff(tmp_path, monkeypatch, hover_fails):
    """复用原确认悬停，失败不能算整体完成。"""
    context = nav_context(tmp_path)
    context.controller.vehicle_status.mode = "OFFBOARD"
    hover = Mock(side_effect=SafetyHandoffRequired("失败") if hover_fails else None,
                 return_value={"safety_state": "HOLD_CONFIRMED"})
    monkeypatch.setattr(loop_module, "request_confirmed_hover", hover)
    client = RecordingClient([planning_reply(("completed", "completed"), "hold")])
    if hover_fails:
        with pytest.raises(SafetyHandoffRequired):
            loop_module.agent_loop(client, "vlm", messages(), context)
        assert context.navigation_plan.status == "incomplete"
    else:
        loop_module.agent_loop(client, "vlm", messages(), context)
        assert context.navigation_plan.status == "completed"
    hover.assert_called_once()


@pytest.mark.parametrize("state,mode", [("ON_GROUND", "AUTO.LOITER"), ("UNKNOWN", "AUTO.LOITER"),
                                        ("IN_AIR", "MANUAL")])
def test_hold_requires_airborne_control(tmp_path, state, mode):
    """未知或地面/手动状态不声称悬停完成。"""
    context = nav_context(tmp_path)
    context.controller.flight_state = lambda: state
    context.controller.vehicle_status.mode = mode
    client = RecordingClient([planning_reply(("completed", "completed"), "hold")])
    assert "无法确认" in loop_module.agent_loop(client, "vlm", messages(), context)
    assert context.navigation_plan.status == "incomplete"


def test_chat_needs_no_classification_request(tmp_path):
    """普通聊天只请求一次，不建立计划。"""
    context = nav_context(tmp_path)
    client = RecordingClient([reply(text="你好")])
    assert loop_module.agent_loop(client, "vlm", messages(), context) == "你好"
    assert len(client.requests) == 1 and context.navigation_plan is None


def test_dynamic_budget_counts_requests_before_creation(tmp_path, monkeypatch):
    """创建前观察也计入预算，重复更新不重置总额。"""
    context = nav_context(tmp_path)
    monkeypatch.setattr(loop_module, "MAX_TOOL_CALLS_PER_TURN", 2)
    monkeypatch.setattr(loop_module, "MAX_NAVIGATION_REQUESTS", 4)
    client = RecordingClient([observe_reply(), planning_reply(), planning_reply(), planning_reply()])
    assert "预算耗尽" in loop_module.agent_loop(client, "vlm", messages(), context)
    assert len(client.requests) == 4


@pytest.mark.parametrize("enabled,mode", [(False, "simulation"), (False, "real")])
def test_disabled_tool_cannot_be_dispatched(tmp_path, enabled, mode):
    """隐藏 schema 之外还要在分发端拒绝越界调用。"""
    context = nav_context(tmp_path)
    context.navigation_enabled = enabled
    context.profile = replace(context.profile, mode=mode)
    assert dispatch_tool_call(context, tool_call("update_navigation_plan", plan_payload()))["error"] == "NAVIGATION_DISABLED"
    assert context.navigation_plan is None


def test_multiple_tools_execute_neither(tmp_path, monkeypatch):
    """不能同轮写计划并飞行，拒绝结果仍配对。"""
    context = nav_context(tmp_path)
    dispatch = Mock(side_effect=AssertionError("不应执行"))
    monkeypatch.setattr(loop_module, "dispatch_tool_call", dispatch)
    response = planning_reply()
    response.tool_calls.append(tool_call("takeoff", {"height": 2}, "other"))
    history = messages()
    client = RecordingClient([response, reply(text="重新选择单个工具")])
    loop_module.agent_loop(client, "vlm", history, context)
    assert len(client.requests) == 2
    dispatch.assert_not_called()
    assert_tool_pairs(history)


def test_next_user_turn_keeps_text_but_clears_plan(tmp_path):
    """跨轮只继承文字与模型进度摘要，不恢复图片或清单协议。"""
    context = nav_context(tmp_path)
    inputs = iter(["去路口后悬停", "说明刚才", "exit"])
    context.message_bus = SimpleNamespace(consume_user_message=lambda: SimpleNamespace(content=next(inputs)),
                                          has_pending_user_message=lambda: False)
    finish = planning_reply(("completed", "completed"), "hold")
    finish.content = "路口两侧道路清晰"
    client = RecordingClient([observe_reply(), finish, reply(text="上轮是模型完成判断")])
    _run_interactive_loop(client, "vlm", context, loop_module.agent_loop, input_terminal_started=True)
    assert "路口两侧道路清晰" in str(client.requests[-1])
    assert "导航进度" in str(client.requests[-1])
    assert all(isinstance(item["content"], str) and "tool_calls" not in item for item in client.requests[-1])
    assert context.navigation_plan is None and context.observation is None


@pytest.mark.parametrize("exception", [EndCurrentTurn("中断", {"success": False}), SafetyHandoffRequired("退出")])
def test_original_handoff_and_interrupt_are_preserved(tmp_path, monkeypatch, exception):
    """工具中断及安全退出不留下运行中的计划。"""
    context = nav_context(tmp_path)
    context.navigation_plan = parse_navigation_plan(plan_payload(), "原文")
    monkeypatch.setattr(loop_module, "dispatch_tool_call", Mock(side_effect=exception))
    client = RecordingClient([observe_reply()])
    history = messages()
    if isinstance(exception, SafetyHandoffRequired):
        with pytest.raises(SafetyHandoffRequired):
            loop_module.agent_loop(client, "vlm", history, context)
    else:
        loop_module.agent_loop(client, "vlm", history, context)
        assert_tool_pairs(history)
    assert context.navigation_plan.status == "incomplete"


def test_model_failure_preserves_hover(tmp_path, monkeypatch):
    """模型请求失败沿用安全交接。"""
    context = nav_context(tmp_path)
    context.controller.vehicle_status.mode = "OFFBOARD"
    hover = Mock(return_value={"safety_state": "HOLD_CONFIRMED"})
    monkeypatch.setattr(loop_module, "request_confirmed_hover", hover)
    assert "模型请求或响应失败" in loop_module.agent_loop(RecordingClient([planning_reply()]), "vlm", messages(), context)
    hover.assert_called_once()
    assert context.navigation_plan.status == "incomplete"


def test_finishing_refuses_motion(tmp_path, monkeypatch):
    """全部打勾后拒绝继续运动，仍可降落。"""
    context = nav_context(tmp_path)
    rotate = Mock(side_effect=AssertionError("不应旋转"))
    monkeypatch.setattr(flight, "rotate", rotate)
    monkeypatch.setattr(flight, "land", Mock(return_value={"success": True}))
    client = RecordingClient([planning_reply(("completed", "completed")),
                              reply("rotate", {"direction": "left", "degrees": 20}), reply("land")])
    loop_module.agent_loop(client, "vlm", messages(), context)
    assert "NAVIGATION_FINISHING" in client.requests[-1][-1]["content"]
    rotate.assert_not_called()


def test_pending_intervention_stops_before_request(tmp_path, monkeypatch):
    """有活动计划时纯文字阶段也响应用户介入。"""
    context = nav_context(tmp_path)
    context.navigation_plan = parse_navigation_plan(plan_payload(), "原文")
    context.controller.vehicle_status.mode = "OFFBOARD"
    context.message_bus = SimpleNamespace(has_pending_user_message=lambda: True,
                                          get_next_user_message=lambda: SimpleNamespace(content="停止"))
    hover = Mock(return_value={"safety_state": "HOLD_CONFIRMED"})
    monkeypatch.setattr(loop_module, "request_confirmed_hover", hover)
    client = RecordingClient([])
    loop_module.agent_loop(client, "vlm", messages(), context)
    assert not client.requests and context.task_state.intervention_pending
    assert context.navigation_plan.status == "incomplete"
    hover.assert_called_once()


@pytest.mark.parametrize("mode,enabled", [("simulation", True), ("real", True)])
def test_runtime_wires_phase2_for_both_profiles(tmp_path, monkeypatch, mode, enabled):
    """全假运行资产验证入口开关，不启动真实 ROS 或仿真。"""
    import sys
    import drone_harness.runtime.runtime as runtime_module

    profile = replace(nav_context(tmp_path).profile, mode=mode)
    fake_ros = SimpleNamespace(init=Mock(), ok=lambda: False, shutdown=Mock())
    executor = SimpleNamespace(add_node=Mock(), spin=Mock(), shutdown=Mock())
    monkeypatch.setitem(sys.modules, "rclpy", fake_ros)
    monkeypatch.setitem(sys.modules, "rclpy.executors", SimpleNamespace(SingleThreadedExecutor=lambda: executor))
    server = SimpleNamespace(start=Mock(), stop=Mock())
    controller = SimpleNamespace(destroy_node=Mock())
    monkeypatch.setattr(runtime_module, "InputServer", lambda _bus: server)
    monkeypatch.setattr(runtime_module, "controller_class_for_profile", lambda _mode: lambda **_kwargs: controller)
    monkeypatch.setattr(runtime_module, "create_llm_client", lambda _profile: object())
    monkeypatch.setattr(runtime_module, "_start_input_terminal", lambda *_args: True)
    observed = []
    monkeypatch.setattr(runtime_module, "_run_interactive_loop",
                        lambda _client, _model, context, _loop, **_kwargs: observed.append(context.navigation_enabled))
    runtime_module._start_live_runtime(profile)
    assert observed == [enabled]
    controller.destroy_node.assert_called_once()
    server.stop.assert_called_once()


def test_plan_update_with_natural_language_never_parses_text_json(tmp_path):
    """复现自然描述加 JSON 的回复，状态仅由工具参数决定。"""
    context = nav_context(tmp_path)
    response = planning_reply(("completed", "completed"), "hold")
    response.content = '当前画面有路灯。\n{"subgoal_complete": true}'
    client = RecordingClient([response])
    assert "保持空中" in loop_module.agent_loop(client, "vlm", messages(), context)
    assert context.navigation_plan.status == "completed"


def test_no_plan_budget_and_single_action_stay_original(tmp_path, monkeypatch):
    """单步动作无需先建计划，未建计划仍沿用原请求预算。"""
    context = nav_context(tmp_path)
    monkeypatch.setattr(loop_module, "MAX_TOOL_CALLS_PER_TURN", 2)
    state = reply("get_state")
    client = RecordingClient([state, state])
    assert "次数过多" in loop_module.agent_loop(client, "vlm", messages(), context)
    assert len(client.requests) == 2 and context.navigation_plan is None


def test_existing_no_progress_stop_not_bypassed_by_plan_update(tmp_path):
    """达到原无位移阈值后不能通过修改计划继续。"""
    context = nav_context(tmp_path)
    context.task_state.consecutive_no_progress = 3
    client = RecordingClient([planning_reply()])
    assert "没有可测位移" in loop_module.agent_loop(client, "vlm", messages(), context)
    assert not client.requests


@pytest.mark.parametrize("approve", [True, False])
def test_real_plan_loop_keeps_landing_approval(tmp_path, monkeypatch, approve):
    """实机清单自动更新，但实际降落仍须逐次人工确认。"""
    from test_real_hitl import real_context
    from drone_harness.llm.prompts import build_system_prompt

    context = real_context(tmp_path)
    context.navigation_enabled = True
    context.controller.vehicle_status.mode = "AUTO.LOITER"
    answers = Mock(return_value=SimpleNamespace(content="y" if approve else "n"))
    context.message_bus = SimpleNamespace(get_next_user_message=answers,
                                          has_pending_user_message=lambda: False)
    landed = Mock(return_value={"success": True})
    monkeypatch.setattr(flight, "land", landed)
    history = messages()
    history[0]["content"] = build_system_prompt(context.profile)
    client = RecordingClient([planning_reply(), planning_reply(("completed", "completed")), reply("land")])
    loop_module.agent_loop(client, "vlm", history, context)
    assert (context.navigation_plan.status == "completed") == approve
    assert landed.call_count == int(approve)
    answers.assert_called_once()
    for option in client.options:
        assert len(option["tools"]) == 9
    assert "update_navigation_plan" in client.requests[0][0]["content"]
    assert "实机六种飞行动作每次均须人工确认" in client.requests[0][0]["content"]
    assert '"completed_count": 2' in client.requests[-1][0]["content"]
    assert_tool_pairs(history)


def test_real_plan_rewrite_and_correction_need_no_approval(tmp_path, monkeypatch):
    """实机沿用相同清单改写和纠错机制，不把计划操作当飞行动作。"""
    import drone_harness.runtime.tool_dispatcher as dispatcher

    context = nav_context(tmp_path)
    context.profile = replace(context.profile, mode="real")
    confirm = Mock(side_effect=AssertionError("清单操作不应审批"))
    monkeypatch.setattr(dispatcher, "_confirm_flight_tool", confirm)
    monkeypatch.setattr(loop_module, "MAX_NAVIGATION_REQUESTS", 4)
    changed = plan_payload(statuses=("pending", "in_progress"))
    changed["subgoals"].reverse()
    client = RecordingClient([planning_reply(), reply("update_navigation_plan", {"invalid": True}),
                              reply("update_navigation_plan", changed), reply(text="继续")])
    assert "预算耗尽" in loop_module.agent_loop(client, "vlm", messages(), context)
    assert context.navigation_plan.subgoals[0].id == "g2"
    assert "INVALID_NAVIGATION_PLAN" in str(client.requests[2])
    confirm.assert_not_called()
