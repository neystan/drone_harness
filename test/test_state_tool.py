"""离线验证状态查询只读、未知处理与模型上下文。"""

import json
import math
from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from drone_harness.px4.controller import Px4Controller
from drone_harness.runtime.agent_loop import agent_loop
from drone_harness.runtime.tool_dispatcher import dispatch_tool_call
from drone_harness.tools.state import get_state
from test_agent_observation_loop import FakeClient, context_for
from test_stage3_observe import tool_call


def controller_with_state():
    """建立使用真实高度和落地判断方法的离线控制器。"""
    controller = object.__new__(Px4Controller)
    controller.vehicle_status_received = True
    controller.vehicle_status = SimpleNamespace(connected=True, armed=True, mode="AUTO.LOITER")
    controller.pose_received = True
    controller.vehicle_local_position = SimpleNamespace(x=12.4, y=-3.1, z=-5.0,
                                                        heading=math.pi / 2, xy_valid=True, z_valid=True)
    controller.extended_state_received = True
    controller.extended_state = SimpleNamespace(landed_state=2)
    controller.ground_z_ned = 0.0
    return controller


def test_state_reports_only_requested_fields():
    """正确报告位置和参考高度，工具结果不含冗余字段。"""
    result = get_state(controller_with_state())
    assert result["position_ned_m"] == {"north": 12.4, "east": -3.1, "down": -5.0}
    assert result["height_above_reference_m"] == 5.0
    assert result["in_air"] is True
    assert result["mode"] == "AUTO.LOITER"
    assert set(result) == {"success", "connected", "armed", "mode", "in_air",
                           "position_ned_m", "height_above_reference_m"}
    json.dumps(result, allow_nan=False)


def test_missing_state_does_not_invent_ground_or_zero_position():
    """未接收任何消息仍正常反馈未知，不结束规划轮次。"""
    result = get_state(SimpleNamespace())
    assert result["success"]
    assert result["mode"] == "UNKNOWN"
    for key in ("in_air", "position_ned_m", "height_above_reference_m", "armed", "connected"):
        assert result[key] is None


@pytest.mark.parametrize("invalid", [float("nan"), float("inf"), True])
def test_invalid_pose_cannot_be_reported_as_position(invalid):
    """坏坐标不进入模型 JSON，飞行状态可独立报告。"""
    controller = controller_with_state()
    controller.vehicle_local_position.z = invalid
    result = get_state(controller)
    assert result["position_ned_m"] is None
    assert result["height_above_reference_m"] is None
    assert result["in_air"] is True
    json.dumps(result, allow_nan=False)


def test_disconnected_state_and_unknown_reference_are_not_current_facts():
    """断连不沿用旧空中结论；未记录地面时不编造参考高度。"""
    controller = controller_with_state()
    controller.ground_z_ned = None
    assert get_state(controller)["height_above_reference_m"] is None
    controller.extended_state.landed_state = 1
    assert get_state(controller)["in_air"] is False
    controller.vehicle_status.connected = False
    result = get_state(controller)
    assert result["in_air"] is None and result["position_ned_m"] is None


def test_reference_height_uses_recorded_ground_not_coordinate_origin():
    """非零地面参考也按地面 down 减当前 down 计算高度。"""
    controller = controller_with_state()
    controller.ground_z_ned = 2.0
    controller.vehicle_local_position.z = -3.0
    result = get_state(controller)
    assert result["position_ned_m"]["down"] == -3.0
    assert result["height_above_reference_m"] == 5.0


@pytest.mark.parametrize("mode", ["simulation", "real"])
def test_state_tool_is_read_only_without_hitl_or_observation(tmp_path, monkeypatch, mode):
    """仿真和实机查询均无需人工审批，也不取图或发送飞行指令。"""
    import drone_harness.runtime.tool_dispatcher as dispatcher

    controller = controller_with_state()
    controller.wait_for_observation = Mock(side_effect=AssertionError("不能取图"))
    context = context_for(tmp_path, controller)
    context.profile = replace(context.profile, mode=mode)
    confirm = Mock(side_effect=AssertionError("不能审批"))
    monkeypatch.setattr(dispatcher, "_confirm_flight_tool", confirm)
    result = dispatch_tool_call(context, tool_call("get_state", {}))
    assert result["success"] and result["in_air"]
    confirm.assert_not_called()
    controller.wait_for_observation.assert_not_called()


def test_state_result_enters_next_model_request(tmp_path):
    """状态结果像其他工具一样进入本轮上下文，不插入图片。"""
    context = context_for(tmp_path, controller_with_state())
    client = FakeClient([
        SimpleNamespace(content="", tool_calls=[tool_call("get_state", {})]),
        SimpleNamespace(content="目前在空中，高度为参考地面以上五米。", tool_calls=[]),
    ])
    messages = [{"role": "user", "content": "现在在哪？"}]
    agent_loop(client, "test", messages, context)
    result = json.loads(client.requests[1][-1]["content"])
    assert result["height_above_reference_m"] == 5.0
    assert client.requests[1][-1]["role"] == "tool"
