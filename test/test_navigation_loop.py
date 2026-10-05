"""用假模型、合成观测和假控制器验证固定子目标闭环。"""

from dataclasses import replace
import json
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

import drone_harness.runtime.agent_loop as loop_module
from drone_harness.runtime.runtime import _run_interactive_loop
from drone_harness.runtime.safety import EndCurrentTurn, SafetyHandoffRequired
from drone_harness.tools import flight
from test_agent_observation_loop import FakeClient, context_for
from test_navigation_plan import plan_payload
from test_stage3_observe import fresh_snapshot, tool_call


class RecordingClient(FakeClient):
    """同时保存模型、工具与消息，核查同一 VLM 的请求链。"""

    def __init__(self, replies):
        """建立请求记录。"""
        super().__init__(replies)
        self.options = []

    def create(self, **kwargs):
        """记录完整请求配置。"""
        self.options.append(kwargs)
        return super().create(**kwargs)


def reply(*, complete=False, evidence="", scene="", calls=None):
    """构造同轮完成判断与至多一个工具。"""
    return SimpleNamespace(content=json.dumps({"subgoal_complete": complete, "evidence": evidence,
                                               "scene_description": scene}, ensure_ascii=False),
                           tool_calls=calls or [])


def planning_reply(*, finish_action="land", single=False):
    """构造初始拆分结果。"""
    payload = plan_payload(finish_action=finish_action)
    if single:
        payload["subgoals"] = payload["subgoals"][:1]
    return SimpleNamespace(content=json.dumps(payload, ensure_ascii=False), tool_calls=[])


def nav_context(tmp_path):
    """建立启用阶段二的离线仿真上下文，不连接 ROS。"""
    controller = SimpleNamespace(
        wait_for_observation=lambda *, after_stamp_ns: fresh_snapshot(after_stamp_ns),
        vehicle_status=SimpleNamespace(connected=True, armed=True, mode="AUTO.LOITER"),
        flight_state=lambda: "IN_AIR", current_position_ned=lambda: (0.0, 0.0, -2.0),
    )
    context = context_for(tmp_path, controller)
    context.profile = replace(context.profile, post_motion_wait_enabled=False)
    context.navigation_enabled = True
    return context


def messages():
    """保存完整原始导航指令。"""
    return [{"role": "system", "content": "原飞行规则"},
            {"role": "user", "content": "沿道路到路口，再到红门前降落"}]


def observe_reply(call_id="look"):
    """主动观察是唯一获取模型图片的方式。"""
    return reply(calls=[tool_call("observe", {"prompt": "确认当前路口和红门位置"}, call_id)])


def events(tmp_path):
    """读取计划 JSONL 快照。"""
    return [json.loads(line) for line in (tmp_path / "logs/session_test/navigation_plan.jsonl").read_text().splitlines()]


def assert_tool_pairs(request):
    """检查每个请求没有悬空工具调用或孤立结果。"""
    pending = []
    for item in request:
        if pending:
            assert item["role"] == "tool" and item["tool_call_id"] == pending.pop(0)
        elif item["role"] == "assistant" and item.get("tool_calls"):
            pending = [call["id"] for call in item["tool_calls"]]
        else:
            assert item["role"] != "tool"
    assert not pending


def test_one_input_runs_two_segments_motion_observation_and_landing(tmp_path, monkeypatch):
    """一次用户输入自动完成两段，仍走原分发与前进深度门。"""
    context = nav_context(tmp_path)
    inputs = iter(["沿路到路口，再到红门前降落", "exit"])
    consumed = []

    def consume():
        """记录用户输入次数，不在目标之间提供继续指令。"""
        text = next(inputs)
        consumed.append(text)
        return SimpleNamespace(content=text)

    context.message_bus = SimpleNamespace(consume_user_message=consume, has_pending_user_message=lambda: False)
    pose = [0.0, 0.0, -2.0]
    context.controller.current_position_ned = lambda: tuple(pose)

    def move(_context, forward_m, _right_m, _down_m, **_kwargs):
        """仅改变假位置，原 forward 仍负责调用时的新深度限制。"""
        pose[0] += forward_m
        return {"success": True, "final_position_ned": list(pose)}

    moved = Mock(side_effect=move)
    landed = Mock(return_value={"success": True})
    monkeypatch.setattr(flight, "move", moved)
    monkeypatch.setattr(flight, "land", landed)
    client = RecordingClient([
        planning_reply(), observe_reply("look-1"),
        reply(complete=True, evidence="当前观察支持已在路口", scene="路口两侧道路入口清晰。"),
        reply(calls=[tool_call("forward", {"distance_m": 1.0}, "move")]),
        observe_reply("look-2"),
        reply(complete=True, evidence="前进后新观察支持已在红门前", scene="红门在正前方。"),
        reply(calls=[tool_call("land", {}, "finish")]),
    ])
    _run_interactive_loop(client, "same-vlm", context, loop_module.agent_loop, input_terminal_started=True)
    assert consumed == ["沿路到路口，再到红门前降落", "exit"]
    assert len(client.requests) == 7 and context.navigation_plan.status == "completed"
    assert context.navigation_plan.current_index == 2 and pose[0] == 1.0
    moved.assert_called_once()
    landed.assert_called_once()
    assert all(option["model"] == "same-vlm" for option in client.options)
    assert "tools" not in client.options[0]
    expected_tools = {"observe", "get_state", "takeoff", "forward", "up", "down", "rotate", "land"}
    assert all({tool["function"]["name"] for tool in option["tools"]} == expected_tools
               for option in client.options[1:])
    assert all(isinstance(item["content"], str) for item in client.requests[0])
    assert all(isinstance(item["content"], str) for item in client.requests[1])
    assert '"current_subgoal_number": 2' in client.requests[3][0]["content"]
    assert "当前观察支持已在路口" in client.requests[3][0]["content"]
    assert not any(item["role"] == "tool" for item in client.requests[3])
    assert all("沿路到路口，再到红门前降落" in str(request) for request in client.requests)
    assert all(sum(part["type"] == "image_url" for item in request if isinstance(item["content"], list)
                   for part in item["content"]) <= 1 for request in client.requests)
    for request in client.requests:
        assert_tool_pairs(request)
    records = events(tmp_path)
    assert [event["event_type"] for event in records] == ["created", "advanced", "advanced", "completed"]
    assert "未独立核验" in records[-1]["plan"]["stop_reason"]
    log = (tmp_path / "logs/session_test/tool_calls.jsonl").read_text()
    assert '"tool_name": "forward"' in log and '"motion_executed": true' in log
    assert "base64" not in (tmp_path / "logs/session_test/navigation_plan.jsonl").read_text()


def test_completion_after_motion_needs_new_observe(tmp_path, monkeypatch):
    """动作后的旧图不能确认，先重新观察再自动推进。"""
    context = nav_context(tmp_path)
    rotated = Mock(return_value={"success": True, "degrees": 15})
    monkeypatch.setattr(flight, "rotate", rotated)
    monkeypatch.setattr(flight, "land", Mock(return_value={"success": True}))
    client = RecordingClient([
        planning_reply(single=True), observe_reply(),
        reply(calls=[tool_call("rotate", {"direction": "left", "degrees": 15})]),
        reply(complete=True, evidence="仅用动作前画面宣称完成"),
        observe_reply("fresh"), reply(complete=True, evidence="转向后重新观察确认位于路口"),
        reply(calls=[tool_call("land", {})]),
    ])
    loop_module.agent_loop(client, "vlm", messages(), context)
    after_rejected = client.requests[4][0]["content"]
    assert '"completed_count": 0' in after_rejected and '"observation_current": false' in after_rejected
    assert "请先 observe" in after_rejected
    assert context.navigation_plan.subgoals[0].evidence == "转向后重新观察确认位于路口"
    assert context.navigation_plan.status == "completed"
    rotated.assert_called_once()


def test_completion_with_tool_does_not_execute_proposed_motion(tmp_path, monkeypatch):
    """完成回复内的工具不执行，下段能看到明确未执行提示。"""
    context = nav_context(tmp_path)
    rotated = Mock(side_effect=AssertionError("完成回复内不应旋转"))
    monkeypatch.setattr(flight, "rotate", rotated)
    monkeypatch.setattr(flight, "land", Mock(return_value={"success": True}))
    client = RecordingClient([
        planning_reply(), observe_reply(),
        reply(complete=True, evidence="已位于路口", calls=[tool_call("rotate", {"direction": "left", "degrees": 30})]),
        reply(complete=True, evidence="当前观察也支持已位于红门前"),
        reply(calls=[tool_call("land", {})]),
    ])
    loop_module.agent_loop(client, "vlm", messages(), context)
    rotated.assert_not_called()
    assert "工具未执行" in client.requests[3][0]["content"]
    assert "SKIPPED_ON_SUBGOAL_CONFIRMATION" in (tmp_path / "logs/session_test/tool_calls.jsonl").read_text()
    for request in client.requests:
        assert_tool_pairs(request)


def test_plain_text_keeps_planning_without_user_continue(tmp_path, monkeypatch):
    """模型只说继续或到达不会停轮，也不能直接标记完成。"""
    context = nav_context(tmp_path)
    monkeypatch.setattr(loop_module, "MAX_NAVIGATION_REQUESTS", 4)
    client = RecordingClient([planning_reply(),
                              SimpleNamespace(content="下一步继续前进", tool_calls=[]),
                              SimpleNamespace(content="已经到达目标", tool_calls=[]), reply()])
    answer = loop_module.agent_loop(client, "vlm", messages(), context)
    assert len(client.requests) == 4 and "预算耗尽" in answer
    assert context.navigation_plan.status == "incomplete" and context.navigation_plan.current_index == 0


@pytest.mark.parametrize("hover_fails", [False, True])
def test_hold_reuses_confirmed_hover_and_does_not_land(tmp_path, monkeypatch, hover_fails):
    """保持空中仅复用原悬停路径，悬停失败不写成功。"""
    context = nav_context(tmp_path)
    context.controller.vehicle_status.mode = "OFFBOARD"
    hover = Mock(side_effect=SafetyHandoffRequired("测试悬停失败") if hover_fails else None,
                 return_value={"safety_state": "HOLD_CONFIRMED"})
    landed = Mock(side_effect=AssertionError("保持空中不应降落"))
    monkeypatch.setattr(loop_module, "request_confirmed_hover", hover)
    monkeypatch.setattr(flight, "land", landed)
    client = RecordingClient([planning_reply(finish_action="hold", single=True), observe_reply(),
                              reply(complete=True, evidence="新观察支持位于路口")])
    if hover_fails:
        with pytest.raises(SafetyHandoffRequired):
            loop_module.agent_loop(client, "vlm", messages(), context)
        assert context.navigation_plan.status == "incomplete"
    else:
        assert "保持空中" in loop_module.agent_loop(client, "vlm", messages(), context)
        assert context.navigation_plan.status == "completed"
    hover.assert_called_once()
    landed.assert_not_called()


@pytest.mark.parametrize("landing_success", [True, False])
def test_landing_before_goals_is_not_navigation_success(tmp_path, monkeypatch, landing_success):
    """提前落地或降落失败都不冒充导航完成。"""
    context = nav_context(tmp_path)
    monkeypatch.setattr(flight, "land", Mock(return_value={"success": landing_success}))
    client = RecordingClient([planning_reply(), reply(calls=[tool_call("land", {})])])
    loop_module.agent_loop(client, "vlm", messages(), context)
    assert context.navigation_plan.status == "incomplete" and context.navigation_plan.current_index == 0
    assert events(tmp_path)[-1]["event_type"] == "incomplete"


def test_finishing_refuses_more_navigation_motion(tmp_path, monkeypatch):
    """全部确认后只收尾，拒绝新的运动并继续降落。"""
    context = nav_context(tmp_path)
    rotated = Mock(side_effect=AssertionError("收尾不应旋转"))
    monkeypatch.setattr(flight, "rotate", rotated)
    monkeypatch.setattr(flight, "land", Mock(return_value={"success": True}))
    client = RecordingClient([planning_reply(single=True), observe_reply(),
                              reply(complete=True, evidence="观察支持位于路口"),
                              reply(calls=[tool_call("rotate", {"direction": "left", "degrees": 30})]),
                              reply(calls=[tool_call("land", {})])])
    loop_module.agent_loop(client, "vlm", messages(), context)
    rotated.assert_not_called()
    assert "NAVIGATION_FINISHING" in client.requests[4][-1]["content"]
    assert context.navigation_plan.status == "completed"


def test_malformed_judgment_stops_before_dispatch_with_paired_result(tmp_path, monkeypatch):
    """判断结构错误不执行同条回复提出的动作，工具协议仍完整。"""
    context = nav_context(tmp_path)
    invoked = Mock(side_effect=AssertionError("判断无效不应分发动作"))
    monkeypatch.setattr(loop_module, "dispatch_tool_call", invoked)
    client = RecordingClient([planning_reply(), SimpleNamespace(
        content='{"subgoal_complete":true,"evidence":""}', tool_calls=[tool_call("takeoff", {"height": 2})])])
    history = messages()
    answer = loop_module.agent_loop(client, "vlm", history, context)
    assert "判断结构无效" in answer and context.navigation_plan.status == "incomplete"
    invoked.assert_not_called()
    assert_tool_pairs(history)
    assert "INVALID_NAVIGATION_DECISION" in history[-1]["content"]


@pytest.mark.parametrize("bad_reply", [
    SimpleNamespace(content="不是 JSON", tool_calls=[]),
    SimpleNamespace(content=json.dumps(plan_payload()), tool_calls=[tool_call("takeoff", {"height": 2})]),
])
def test_invalid_initial_plan_never_dispatches_tools(tmp_path, monkeypatch, bad_reply):
    """拆分失败不进入飞行工具循环。"""
    context = nav_context(tmp_path)
    invoked = Mock(side_effect=AssertionError("拆分失败不应执行工具"))
    monkeypatch.setattr(loop_module, "dispatch_tool_call", invoked)
    client = RecordingClient([bad_reply])
    assert "拆分失败" in loop_module.agent_loop(client, "vlm", messages(), context)
    invoked.assert_not_called()
    assert len(client.requests) == 1 and context.navigation_plan is None


def test_classification_of_chat_uses_original_text_loop(tmp_path):
    """普通交流分类后沿用原纯文字返回，不建立计划。"""
    context = nav_context(tmp_path)
    client = RecordingClient([SimpleNamespace(content='{"navigation":false}', tool_calls=[]),
                              SimpleNamespace(content="你好，我可以帮助导航。", tool_calls=[])])
    assert "你好" in loop_module.agent_loop(client, "vlm", messages(), context)
    assert context.navigation_plan is None and len(client.requests) == 2
    assert "当前导航计划" not in client.requests[1][0]["content"]


def test_model_failure_records_incomplete_and_confirms_hover(tmp_path, monkeypatch):
    """后续请求失败保留进度并执行原悬停交接。"""
    context = nav_context(tmp_path)
    context.controller.vehicle_status.mode = "OFFBOARD"
    hover = Mock(return_value={"safety_state": "HOLD_CONFIRMED"})
    monkeypatch.setattr(loop_module, "request_confirmed_hover", hover)
    client = RecordingClient([planning_reply()])
    answer = loop_module.agent_loop(client, "vlm", messages(), context)
    assert "模型请求或响应失败" in answer and context.navigation_plan.status == "incomplete"
    hover.assert_called_once()
    assert events(tmp_path)[-1]["event_type"] == "incomplete"


def test_subgoal_change_does_not_reset_no_progress_counter(tmp_path, monkeypatch):
    """子目标切换不能绕过原连续无位移停止规则。"""
    context = nav_context(tmp_path)
    context.task_state.consecutive_no_progress = 2
    client = RecordingClient([planning_reply(), observe_reply(),
                              reply(complete=True, evidence="观察支持已在路口")])
    loop_module.agent_loop(client, "vlm", messages(), context)
    assert context.navigation_plan.current_index == 1
    assert context.task_state.consecutive_no_progress == 2


def test_pending_intervention_stops_even_without_tool_call(tmp_path, monkeypatch):
    """固定循环的纯文字阶段也响应原用户介入。"""
    context = nav_context(tmp_path)
    context.controller.vehicle_status.mode = "OFFBOARD"
    context.message_bus = SimpleNamespace(has_pending_user_message=lambda: True,
                                          get_next_user_message=lambda: SimpleNamespace(content="停止"))
    hover = Mock(return_value={"safety_state": "HOLD_CONFIRMED"})
    monkeypatch.setattr(loop_module, "request_confirmed_hover", hover)
    client = RecordingClient([planning_reply()])
    loop_module.agent_loop(client, "vlm", messages(), context)
    assert len(client.requests) == 1 and context.task_state.intervention_pending
    assert context.navigation_plan.status == "incomplete"
    hover.assert_called_once()


def test_next_user_turn_keeps_summary_but_resets_plan_and_observation(tmp_path, monkeypatch):
    """跨轮不携带工具、图片或判断 JSON，旧计划不自动恢复。"""
    context = nav_context(tmp_path)
    inputs = iter(["到路口后保持空中", "说明刚才的任务", "exit"])
    context.message_bus = SimpleNamespace(consume_user_message=lambda: SimpleNamespace(content=next(inputs)),
                                          has_pending_user_message=lambda: False)
    client = RecordingClient([
        planning_reply(finish_action="hold", single=True), observe_reply(),
        reply(complete=True, evidence="已在路口", scene="两侧道路入口清晰"),
        SimpleNamespace(content='{"navigation":false}', tool_calls=[]),
        SimpleNamespace(content="上一轮模型确认已在路口，尚无独立到达核验。", tool_calls=[]),
    ])
    _run_interactive_loop(client, "vlm", context, loop_module.agent_loop, input_terminal_started=True)
    assert len(client.requests) == 5
    following = client.requests[4]
    assert "到路口（依据：已在路口）" in str(following)
    assert "subgoal_complete" not in str(following) and "tool_call_id" not in str(following)
    assert all(isinstance(item["content"], str) and "tool_calls" not in item for item in following)
    assert context.navigation_plan is None and context.observation is None and context.depth_rules is None


def test_flight_safety_handoff_records_incomplete_before_propagation(tmp_path, monkeypatch):
    """原飞行工具的安全退出不能留下运行中的任务快照。"""
    context = nav_context(tmp_path)
    dispatch = Mock(side_effect=SafetyHandoffRequired("原飞控安全退出"))
    monkeypatch.setattr(loop_module, "dispatch_tool_call", dispatch)
    client = RecordingClient([planning_reply(), reply(calls=[tool_call("up", {"distance_m": 1})])])
    with pytest.raises(SafetyHandoffRequired):
        loop_module.agent_loop(client, "vlm", messages(), context)
    assert context.navigation_plan.status == "incomplete"
    assert events(tmp_path)[-1]["event_type"] == "incomplete"


def test_tool_interruption_confirms_hover_and_preserves_pairs(tmp_path, monkeypatch):
    """模型请求期间收到介入，工具中断后仍停止完整导航任务。"""
    context = nav_context(tmp_path)
    context.controller.vehicle_status.mode = "OFFBOARD"
    dispatch = Mock(side_effect=EndCurrentTurn("用户中断", {"success": False, "error": "INTERRUPTED_BY_USER"}))
    hover = Mock(return_value={"safety_state": "HOLD_CONFIRMED"})
    monkeypatch.setattr(loop_module, "dispatch_tool_call", dispatch)
    monkeypatch.setattr(loop_module, "request_confirmed_hover", hover)
    client = RecordingClient([planning_reply(), observe_reply()])
    history = messages()
    loop_module.agent_loop(client, "vlm", history, context)
    assert context.navigation_plan.status == "incomplete"
    hover.assert_called_once()
    assert_tool_pairs(history)


@pytest.mark.parametrize("mode, enabled", [("simulation", True), ("real", False)])
def test_runtime_wires_phase2_only_for_simulation(tmp_path, monkeypatch, mode, enabled):
    """用全假 ROS、输入服务和控制器验证入口，不启动真实运行资产。"""
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


@pytest.mark.parametrize("flight_state, mode", [("ON_GROUND", "AUTO.LOITER"), ("UNKNOWN", "AUTO.LOITER"),
                                                ("IN_AIR", "MANUAL")])
def test_hold_does_not_claim_completion_without_airborne_control(tmp_path, flight_state, mode):
    """地面、未知或非受控模式不能宣称按要求保持空中。"""
    context = nav_context(tmp_path)
    context.controller.flight_state = lambda: flight_state
    context.controller.vehicle_status.mode = mode
    client = RecordingClient([planning_reply(finish_action="hold", single=True), observe_reply(),
                              reply(complete=True, evidence="观察支持已在路口")])
    answer = loop_module.agent_loop(client, "vlm", messages(), context)
    assert "无法确认空中悬停状态" in answer
    assert context.navigation_plan.status == "incomplete"
