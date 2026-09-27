"""验证精简后的模型输入与仿真直接降落，不触达仿真或模型服务。"""

import json
from copy import deepcopy
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

import drone_harness.runtime.tool_dispatcher as dispatcher
from drone_harness.config.loader import load_profile
from drone_harness.llm.prompts import build_system_prompt
from drone_harness.runtime.agent_loop import _build_tool_message, agent_loop
from drone_harness.runtime.observation import build_observation_message
from drone_harness.runtime.safety import EndCurrentTurn
from drone_harness.tools import flight
from drone_harness.tools.schemas import get_tool_schemas
from drone_harness.vision.depth_rules import DepthRules, invalid_depth_rules
from test_agent_observation_loop import FakeClient, context_for
from test_depth_rules import snapshot_at
from test_real_hitl import real_context
from test_stage3_observe import tool_call


def test_system_prompt_focuses_on_navigation_not_tool_implementation(tmp_path):
    """系统规则只保留任务推进、观察顺序、绕障与完成后降落。"""
    prompt = build_system_prompt(context_for(tmp_path).profile)
    for phrase in ("长导航任务飞行规划器", "旋转", "重新观察", "深度", "确定性", "上升", "下降", "land", "不要降落"):
        assert phrase in prompt
    for removed in ("单目标", "候选完成", "观测号", "同号", "第二个视觉模型", "19 米", "1000", "不重复索取"):
        assert removed not in prompt
    assert "缺失" in prompt and "障碍" in prompt


@pytest.mark.parametrize("name,forward,vertical", [("sim", "19", "10"), ("real", "0.2", "2")])
def test_schemas_describe_actual_profile_limits(name, forward, vertical):
    """模型侧中文参数限额跟随当前 profile，实机不误用仿真数值。"""
    profile = load_profile(name, settings_path=Path(__file__).parents[1] / "settings.example.json")
    schemas = {item["function"]["name"]: item["function"] for item in get_tool_schemas(profile)}
    assert f"不超过 {forward} 米" in schemas["forward"]["parameters"]["properties"]["distance_m"]["description"]
    assert "深度" not in schemas["forward"]["description"]
    for tool in ("up", "down"):
        assert f"不超过 {vertical} 米" in schemas[tool]["parameters"]["properties"]["distance_m"]["description"]
    observe = schemas["observe"]["description"]
    assert "不重复索取深度摘要" in observe
    assert "飞行状态" in observe and "移动距离" in observe
    prompt = schemas["observe"]["parameters"]["properties"]["prompt"]["description"]
    for example in ("十字路口", "红色店招", "旋转后"):
        assert example in prompt
    assert "1000" not in prompt and "同一个模型" not in observe
    for tool in ("takeoff", "rotate", "land"):
        assert any("\u4e00" <= char <= "\u9fff" for char in schemas[tool]["description"])


def test_schema_generation_does_not_mutate_shared_definitions(tmp_path):
    """生成实机描述或修改返回值不能污染后续仿真 schema。"""
    before = deepcopy(get_tool_schemas())
    profile = context_for(tmp_path).profile
    real = replace(profile, mode="real", safety=replace(profile.safety, max_relative_move_m=.2))
    generated = get_tool_schemas(real)
    generated[0]["function"]["description"] = "changed"
    assert get_tool_schemas() == before


@pytest.mark.parametrize("mode,limit", [("simulation", "19"), ("real", "0.2")])
def test_runtime_sends_profile_specific_schema_to_model(tmp_path, mode, limit):
    """检查实际模型请求中的限额，而不只检查 schema 辅助函数。"""
    context = context_for(tmp_path)
    context.profile = replace(context.profile, mode=mode, safety=replace(
        context.profile.safety, max_relative_move_m=.2))
    create = Mock(return_value=SimpleNamespace(choices=[SimpleNamespace(
        message=SimpleNamespace(content="已收到", tool_calls=[]))]))
    client = SimpleNamespace(chat=SimpleNamespace(completions=SimpleNamespace(create=create)))
    agent_loop(client, "test", [{"role": "user", "content": "你好"}], context)
    schemas = {item["function"]["name"]: item["function"]
               for item in create.call_args.kwargs["tools"]}
    assert f"不超过 {limit} 米" in schemas["forward"]["parameters"]["properties"]["distance_m"]["description"]


def test_observation_text_hides_internal_metadata_and_preserves_pairing():
    """只删除模型文字中的编号时间，不删除程序内的同号检查。"""
    snapshot = snapshot_at()
    rules = DepthRules(snapshot.observation_id, True, 6.0, 5.0, "obstacle", "clear", "", 6.0)
    message = build_observation_message(snapshot, rules, "寻找路口")
    text = message["content"][0]["text"]
    assert "寻找路口" in text and "前方障碍距离：6.00 米" in text
    assert "本次观测的前进上限：5.00 米" in text
    assert "左前：2 米范围内有近障" in text
    assert "右前：2 米范围内未检测到近障" in text
    assert "observation_id" not in text and "stamp" not in text
    assert snapshot.observation_id not in text and str(snapshot.rgb_stamp_ns) not in text
    assert "reason=ok" not in text
    with pytest.raises(ValueError, match="do not match"):
        build_observation_message(snapshot, replace(rules, observation_id="wrong"))


def test_invalid_depth_is_unknown_not_clear_and_side_range_is_configurable():
    """无效数据不再声称前方无障碍，侧方文案沿用配置范围。"""
    invalid = invalid_depth_rules("private-id", "DEPTH_MISSING").as_text()
    assert "深度无效" in invalid and "未知" in invalid and "DEPTH_MISSING" in invalid
    assert "未检测到" not in invalid and "none_within_horizon" not in invalid
    rules = DepthRules("private-id", True, 19.9, 18.9, "clear", "obstacle", "")
    text = rules.as_text(side_obstacle_distance_m=3.0)
    assert "3 米范围" in text and "2 米范围" not in text
    assert "决策视距内未检测到障碍" in text
    assert "19.90 米" not in text  # 不把视距边界当作实体障碍。


def test_tool_message_hides_metadata_without_mutating_logged_result():
    """模型不接收观测元数据，但结果与原生 tool_call_id 完整保留。"""
    result = {"success": True, "observation_id": "rgb-private", "rgb_stamp_ns": 123,
              "depth_stamp_ns": 124, "forward_max_m": .5,
              "requested_distance_m": 3, "commanded_distance_m": .5, "clamped": True}
    before = deepcopy(result)
    message = _build_tool_message("native-call-id", result)
    visible = json.loads(message["content"])
    assert message["tool_call_id"] == "native-call-id"
    assert not ({"observation_id", "rgb_stamp_ns", "depth_stamp_ns"} & visible.keys())
    assert visible["forward_max_m"] == .5 and visible["clamped"]
    assert result == before


def test_sim_land_without_observation_or_confirmation_channel_finishes_turn(tmp_path, monkeypatch):
    """仿真降落无需观测绑定或人工通道，仍调用原降落工具并终止本轮。"""
    context = context_for(tmp_path)
    handler = Mock(return_value={"success": True, "message": "landed"})
    confirm = Mock(side_effect=AssertionError("仿真降落不应询问人工"))
    monkeypatch.setattr(flight, "land", handler)
    monkeypatch.setattr(dispatcher, "_confirm_flight_tool", confirm)
    client = FakeClient([SimpleNamespace(content="", tool_calls=[tool_call("land", {})])])
    messages = [{"role": "system", "content": "test"}, {"role": "user", "content": "完成后降落"}]
    assert "降落完成" in agent_loop(client, "test", messages, context)
    handler.assert_called_once_with(context)
    confirm.assert_not_called()
    assert len(client.requests) == 1 and messages[-1]["tool_call_id"] == "call-1"


def test_sim_land_preserves_flight_state_rejection(tmp_path, monkeypatch):
    """不再询问人工也不能绕过底层位姿检查。"""
    context = context_for(tmp_path)
    position_check = Mock(return_value=False)
    monkeypatch.setattr(flight, "_wait_for_valid_position", position_check)
    result = dispatcher.dispatch_tool_call(context, tool_call("land", {}))
    assert not result["success"] and result["error"] == "POSITION_INVALID"
    position_check.assert_called_once_with(context.controller)


def test_real_land_still_rejects_human_denial_even_when_previously_authorized(tmp_path, monkeypatch):
    """实机降落逐次批准，旧授权标志不能绕过本次拒绝。"""
    context = real_context(tmp_path)
    context.task_state.landing_authorized = True
    context.message_bus = SimpleNamespace(get_next_user_message=lambda: SimpleNamespace(content="n"),
                                          has_pending_user_message=lambda: False)
    handler = Mock(return_value={"success": True})
    monkeypatch.setattr(flight, "land", handler)
    with pytest.raises(EndCurrentTurn):
        dispatcher.dispatch_tool_call(context, tool_call("land", {}))
    handler.assert_not_called()


def test_sim_land_skips_optional_flight_hitl_setting(tmp_path, monkeypatch):
    """仿真 land 明确不审批，即使配置另行开启普通飞行动作确认。"""
    context = context_for(tmp_path)
    context.profile = replace(context.profile, safety=replace(
        context.profile.safety, human_in_the_loop_for_flight_tools=True))
    handler = Mock(return_value={"success": True})
    monkeypatch.setattr(flight, "land", handler)
    assert dispatcher.dispatch_tool_call(context, tool_call("land", {}))["success"]
    handler.assert_called_once_with(context)


def test_real_land_still_needs_bound_observation(tmp_path, monkeypatch):
    """隐藏模型侧观测号不会移除实机审批的观测绑定。"""
    context = real_context(tmp_path)
    context.observation = None
    handler = Mock(return_value={"success": True})
    monkeypatch.setattr(flight, "land", handler)
    result = dispatcher.dispatch_tool_call(context, tool_call("land", {}))
    assert result["error"] == "APPROVAL_OBSERVATION_INVALID"
    handler.assert_not_called()
