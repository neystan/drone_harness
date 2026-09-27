"""验证按需观察和每次前进时重新读取深度的合同。"""

from __future__ import annotations

import json
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import drone_harness.runtime.agent_loop as loop_module
from drone_harness.runtime.agent_loop import agent_loop
from drone_harness.tools import flight
from drone_harness.tools.registry import get_tool_definition
from drone_harness.runtime.tool_dispatcher import dispatch_tool_call
from test_agent_observation_loop import FakeClient, context_for
from test_depth_rules import snapshot_at


def tool_call(name: str, arguments: dict, call_id: str = "call-1") -> SimpleNamespace:
    """构造一条带 JSON 参数的模型工具调用。"""
    return SimpleNamespace(id=call_id, function=SimpleNamespace(
        name=name, arguments=json.dumps(arguments, ensure_ascii=False)))


def fresh_snapshot(after_stamp_ns: int, distance_m: float = 4.0):
    """让合成 RGB-D 的采集时间晚于本次工具调用。"""
    source = snapshot_at(distance_m)
    stamp_ns = after_stamp_ns + 60_000_000
    return replace(source, observation_id=f"rgb-{stamp_ns}", rgb_stamp_ns=stamp_ns,
                   depth_stamp_ns=stamp_ns,
                   intrinsics=replace(source.intrinsics, stamp_ns=stamp_ns))


def test_first_model_request_is_text_only_then_observe_feeds_same_model(tmp_path: Path) -> None:
    """首图必须由 observe 获取，并在成对工具结果后交回同一个模型。"""
    seen_stamps: list[int] = []

    def wait_for_observation(*, after_stamp_ns: int, timeout_s: float | None = None):
        """模拟按需等待一组新鲜的同步图像。"""
        seen_stamps.append(after_stamp_ns)
        return fresh_snapshot(after_stamp_ns)

    context = context_for(tmp_path, SimpleNamespace(wait_for_observation=wait_for_observation))
    messages = [{"role": "system", "content": "test"}, {"role": "user", "content": "寻找红色门"}]
    client = FakeClient([
        SimpleNamespace(content="", tool_calls=[tool_call("observe", {"prompt": "找红色门"})]),
        SimpleNamespace(content="我看到了画面", tool_calls=[]),
    ])

    assert "看到了" in agent_loop(client, "test-vlm", messages, context)
    assert len(seen_stamps) == 1
    assert len(client.requests) == 2
    assert [item["role"] for item in client.requests[0]] == ["system", "user"]
    second = client.requests[1]
    assert [item["role"] for item in second] == ["system", "user", "assistant", "tool", "user"]
    result = json.loads(second[-2]["content"])
    assert result["success"] and result["observation_id"] == context.observation.observation_id
    assert "data:image" not in second[-2]["content"]
    assert "找红色门" in second[-1]["content"][0]["text"]
    assert result["observation_id"] in second[-1]["content"][0]["text"]
    assert second[-1]["content"][1]["image_url"]["url"].startswith("data:image/jpeg;base64,")


def test_observe_rejects_empty_prompt_before_waiting_for_image(tmp_path: Path) -> None:
    """空提示词不能触发取图，模型须得到明确参数错误。"""
    wait = Mock()
    context = context_for(tmp_path, SimpleNamespace(wait_for_observation=wait))
    context.observation = snapshot_at()
    context.task_state.set_observation(context.observation.observation_id)
    result = dispatch_tool_call(context, tool_call("observe", {"prompt": "  "}))
    assert result["success"] is False
    assert result["error"] == "INVALID_OBSERVE_PROMPT"
    assert context.observation is None and context.depth_rules is None
    assert context.task_state.observation_id is None
    wait.assert_not_called()


def test_observe_with_invalid_depth_still_sends_rgb_to_same_model(tmp_path: Path) -> None:
    """深度缺失只标未知，不吞掉已取得的 RGB 图像。"""
    controller = SimpleNamespace(wait_for_observation=lambda *, after_stamp_ns: replace(
        fresh_snapshot(after_stamp_ns), depth=None, depth_stamp_ns=None, depth_error="DEPTH_MISSING"))
    context = context_for(tmp_path, controller)
    messages = [{"role": "system", "content": "test"}, {"role": "user", "content": "找目标"}]
    client = FakeClient([
        SimpleNamespace(content="", tool_calls=[tool_call("observe", {"prompt": "看建筑门口"})]),
        SimpleNamespace(content="深度未知", tool_calls=[]),
    ])

    agent_loop(client, "test-vlm", messages, context)
    response = client.requests[1]
    result = json.loads(response[-2]["content"])
    assert result["success"] and not result["depth_valid"]
    assert result["forward_max_m"] == 0
    assert "depth_valid=false" in response[-1]["content"][0]["text"]
    assert response[-1]["content"][1]["type"] == "image_url"


def test_real_observe_does_not_ask_for_flight_hitl(tmp_path: Path) -> None:
    """观察是非飞行工具，实机逐动作审批不应误拦取图。"""
    controller = SimpleNamespace(wait_for_observation=lambda *, after_stamp_ns: fresh_snapshot(after_stamp_ns))
    context = context_for(tmp_path, controller)
    context.profile = replace(context.profile, mode="real")
    result = dispatch_tool_call(context, tool_call("observe", {"prompt": "检查前方"}))
    assert result["success"] and result["observation_id"] == context.observation.observation_id


def test_successful_motion_does_not_automatically_observe(tmp_path: Path, monkeypatch) -> None:
    """动作后旧规则失效，但下一次模型请求不被自动附图。"""
    wait = Mock(side_effect=AssertionError("动作后不应自动取图"))
    controller = SimpleNamespace(wait_for_observation=wait,
                                 current_position_ned=lambda: (0.0, 0.0, -1.0))
    context = context_for(tmp_path, controller)
    context.observation = snapshot_at()
    context.depth_rules = SimpleNamespace(observation_id=context.observation.observation_id)
    context.task_state.set_observation(context.observation.observation_id)
    messages = [{"role": "system", "content": "test"}, {"role": "user", "content": "继续寻找"}]
    monkeypatch.setattr(loop_module, "dispatch_tool_call", lambda *_args: {
        "success": True, "motion_executed": True, "degrees": 15.0})
    client = FakeClient([
        SimpleNamespace(content="", tool_calls=[tool_call("rotate", {"direction": "left", "degrees": 15})]),
        SimpleNamespace(content="我会再观察", tool_calls=[]),
    ])

    agent_loop(client, "test-vlm", messages, context)
    wait.assert_not_called()
    assert context.observation is None and context.depth_rules is None
    assert context.task_state.observation_id is None
    assert len(client.requests) == 2
    assert all(not isinstance(item["content"], list) for item in client.requests[1])
    assert "历史参考" in json.loads(client.requests[1][-1]["content"])["observation_note"]


def test_motion_then_explicit_observe_sends_new_frame_to_same_model(tmp_path: Path, monkeypatch) -> None:
    """移动后只有模型主动调用 observe，下一次请求才带新图。"""
    wait = Mock(side_effect=lambda *, after_stamp_ns: fresh_snapshot(after_stamp_ns))
    controller = SimpleNamespace(wait_for_observation=wait,
                                 current_position_ned=lambda: (0.0, 0.0, -1.0))
    context = context_for(tmp_path, controller)
    messages = [{"role": "system", "content": "test"}, {"role": "user", "content": "找门"}]
    original_dispatch = loop_module.dispatch_tool_call

    def dispatch(current, call):
        """只替换实际转向，观察继续走注册工具。"""
        if call.function.name == "rotate":
            return {"success": True, "degrees": 30.0}
        return original_dispatch(current, call)

    monkeypatch.setattr(loop_module, "dispatch_tool_call", dispatch)
    client = FakeClient([
        SimpleNamespace(content="", tool_calls=[tool_call("rotate", {"direction": "left", "degrees": 30}, "turn")]),
        SimpleNamespace(content="", tool_calls=[tool_call("observe", {"prompt": "旋转后找门"}, "look")]),
        SimpleNamespace(content="门在画面中", tool_calls=[]),
    ])

    assert "门在画面中" in agent_loop(client, "test-vlm", messages, context)
    wait.assert_called_once()
    assert all(not isinstance(item["content"], list) for item in client.requests[1])
    assert client.requests[2][-1]["role"] == "user"
    assert "旋转后找门" in client.requests[2][-1]["content"][0]["text"]
    assert client.requests[2][-1]["content"][1]["type"] == "image_url"


def test_sim_forward_gets_new_depth_without_observe_and_clamps(tmp_path: Path, monkeypatch) -> None:
    """没有模型可见图像也必须先获取新深度，近障上限优先于 19 米。"""
    observed: list[int] = []

    def wait_for_observation(*, after_stamp_ns: int, timeout_s: float | None = None):
        """提供只属于本次前进调用的新深度。"""
        observed.append(after_stamp_ns)
        return fresh_snapshot(after_stamp_ns, 1.55)

    controller = SimpleNamespace(
        wait_for_observation=wait_for_observation,
        vehicle_status=SimpleNamespace(mode="OFFBOARD", connected=True, armed=True),
        flight_state=lambda: "IN_AIR",
    )
    context = context_for(tmp_path, controller)
    move = Mock(return_value={"success": True, "message": "move complete"})
    monkeypatch.setattr(flight, "move", move)

    result = flight.forward(context, 15.0)
    assert result["success"] and result["clamped"]
    assert len(observed) == 1
    assert context.observation is None
    assert 0 < result["commanded_distance_m"] < 1.0
    assert move.call_args.args[2:] == (0.0, 0.0)
    assert move.call_args.args[1] == result["commanded_distance_m"]
    assert result["observation_id"].startswith("rgb-")


def test_sim_forward_missing_depth_returns_normal_zero_motion(tmp_path: Path, monkeypatch) -> None:
    """调用时没有新 RGB-D 则回报 0 米，不能退回 19 米盲飞。"""
    wait = Mock(return_value=None)
    controller = SimpleNamespace(
        wait_for_observation=wait,
        vehicle_status=SimpleNamespace(mode="OFFBOARD", connected=True, armed=True),
        flight_state=lambda: "IN_AIR",
    )
    context = context_for(tmp_path, controller)
    move = Mock()
    monkeypatch.setattr(flight, "move", move)

    result = flight.forward(context, 19.0)
    assert result["success"] and not result["motion_executed"]
    assert result["commanded_distance_m"] == 0
    assert "未移动" in result["message"]
    wait.assert_called_once()
    move.assert_not_called()


def test_sim_forward_rejects_stale_frame_and_mismatched_rules(tmp_path: Path, monkeypatch) -> None:
    """本次等待返回旧帧或错号规则都不能生成前进目标。"""
    controller = SimpleNamespace(
        wait_for_observation=Mock(return_value=snapshot_at()),
        vehicle_status=SimpleNamespace(mode="OFFBOARD", connected=True, armed=True),
        flight_state=lambda: "IN_AIR",
    )
    context = context_for(tmp_path, controller)
    move = Mock()
    monkeypatch.setattr(flight, "move", move)
    stale = flight.forward(context, 2.0)
    assert stale["success"] and stale["commanded_distance_m"] == 0
    assert stale["depth_reason"] == "FORWARD_RGB_NOT_NEW"

    controller.wait_for_observation = lambda *, after_stamp_ns: fresh_snapshot(after_stamp_ns)
    from drone_harness.vision.depth_rules import invalid_depth_rules
    monkeypatch.setattr(flight, "compute_depth_rules", lambda *_args: invalid_depth_rules("wrong", "TEST"))
    mismatch = flight.forward(context, 2.0)
    assert mismatch["success"] and mismatch["commanded_distance_m"] == 0
    assert mismatch["depth_reason"] == "FORWARD_DEPTH_OBSERVATION_MISMATCH"
    move.assert_not_called()


def test_sim_forward_rechecks_px4_after_waiting_for_depth(tmp_path: Path, monkeypatch) -> None:
    """等新深度期间飞控模式变化时不能继续下发前进目标。"""
    status = SimpleNamespace(mode="OFFBOARD", connected=True, armed=True)

    def wait_for_observation(*, after_stamp_ns: int):
        """模拟取图时 PX4 离开允许模式。"""
        status.mode = "MANUAL"
        return fresh_snapshot(after_stamp_ns)

    controller = SimpleNamespace(wait_for_observation=wait_for_observation,
                                 vehicle_status=status, flight_state=lambda: "IN_AIR")
    context = context_for(tmp_path, controller)
    move = Mock()
    monkeypatch.setattr(flight, "move", move)
    result = flight.forward(context, 1.0)
    assert result["success"] is False and result["error"] == "PX4_STATE_INVALID"
    move.assert_not_called()


def test_repeated_observe_keeps_complete_tool_pairs_and_bounded_images(tmp_path: Path) -> None:
    """多次按需观察后仅保留最近两组完整工具调用与图像。"""
    controller = SimpleNamespace(wait_for_observation=lambda *, after_stamp_ns: fresh_snapshot(after_stamp_ns))
    context = context_for(tmp_path, controller)
    messages = [{"role": "system", "content": "test"}, {"role": "user", "content": "找门"}]
    client = FakeClient([
        SimpleNamespace(content="", tool_calls=[tool_call("observe", {"prompt": f"看方向 {index}"}, f"obs-{index}")])
        for index in range(3)
    ] + [SimpleNamespace(content="观察结束", tool_calls=[])])

    assert "观察结束" in agent_loop(client, "test-vlm", messages, context)
    final_request = client.requests[-1]
    assert [item["role"] for item in final_request] == [
        "system", "user", "assistant", "tool", "user", "assistant", "tool", "user"]
    assert [item["tool_call_id"] for item in final_request if item["role"] == "tool"] == ["obs-1", "obs-2"]
    assert sum(isinstance(item["content"], list) for item in final_request) == 2
    log_text = (tmp_path / "logs" / "session_test" / "tool_calls.jsonl").read_text()
    assert "data:image" not in log_text


def test_tool_surface_has_one_observe_and_six_flight_actions() -> None:
    """观察工具只加到原六动作旁边，不恢复旧拍照或视觉模型链。"""
    assert get_tool_definition("observe") is not None
    assert get_tool_definition("take_photo") is None
