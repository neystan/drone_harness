"""离线验证四向搜索的动作顺序、联合输入和退出边界。"""

import json
from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

import drone_harness.runtime.agent_loop as loop_module
import drone_harness.runtime.skill_search as search
import drone_harness.runtime.tool_dispatcher as dispatcher
from drone_harness.runtime.agent_loop import agent_loop
from drone_harness.runtime.safety import EndCurrentTurn
from drone_harness.skills import build_skills_index, load_skills
from drone_harness.tools import flight
from test_agent_observation_loop import FakeClient, context_for
from test_stage3_observe import fresh_snapshot, tool_call


def scan_context(tmp_path, monkeypatch, events):
    """提供新帧和假转向，并记录完整执行顺序。"""
    def capture(*, after_stamp_ns, view="front"):
        """只返回本次请求之后的前视新帧。"""
        events.append("capture")
        assert view == "front"
        return fresh_snapshot(after_stamp_ns)

    def rotate(_context, direction, degrees):
        """记录固定角度并模拟动作成功。"""
        events.append((direction, degrees))
        return {"success": True, "degrees": degrees}

    monkeypatch.setattr(flight, "rotate", rotate)
    monkeypatch.setattr(search, "time", SimpleNamespace(sleep=lambda seconds: events.append(("wait", seconds))))
    return context_for(tmp_path, SimpleNamespace(wait_for_observation=capture))


def activate_call():
    """生成带当前导航疑点的 skill 原生工具调用。"""
    return tool_call("activate_skill", {"name": "four-direction-search", "prompt": "寻找仓库旁的红色卡车，核对邻近建筑。"})


def image_count(messages):
    """统计一次请求实际携带的图片数量。"""
    return sum(part.get("type") == "image_url" for message in messages
               if isinstance(message.get("content"), list) for part in message["content"])


def test_scan_is_one_joint_request_then_returns_to_ordinary_observe(tmp_path, monkeypatch):
    """四图仅出现一次，动态问题和方向齐全，后续重新观察只带一图。"""
    events = []
    context = scan_context(tmp_path, monkeypatch, events)
    client = FakeClient([
        SimpleNamespace(content="扩大搜索范围", tool_calls=[activate_call()]),
        SimpleNamespace(content="左侧有候选，邻近建筑存疑", tool_calls=[tool_call("observe", {"prompt": "核对旁边建筑"})]),
        SimpleNamespace(content="已取得新图", tool_calls=[]),
    ])
    messages = [{"role": "user", "content": "寻找仓库旁的红色卡车"}]
    agent_loop(client, "test", messages, context)
    assert events == [item for _ in range(4) for item in ("capture", ("left", 90), ("wait", 2.0))] + ["capture"]
    assert [image_count(request) for request in client.requests] == [0, 4, 1]
    group = client.requests[1][-1]["content"]
    texts = "\n".join(part.get("text", "") for part in group)
    assert "寻找仓库旁的红色卡车，核对邻近建筑。" in texts
    assert all(f"【原朝向{direction}方】" in texts for direction in search.SEARCH_DIRECTIONS)
    assert "已恢复开始朝向" in texts
    assert context.search_observations is None
    assert context.observation is not None
    results = [json.loads(message["content"]) for message in messages if message["role"] == "tool"]
    assert results[0]["scan_completed"] and results[0]["heading_restored"]
    assert results[0]["skill_content"]


@pytest.mark.parametrize("failure", ["capture", "rotate"])
def test_partial_scan_does_not_supply_four_images_or_claim_restored_heading(tmp_path, monkeypatch, failure):
    """中途失败只反馈实际进度，不请求额外转向或发布部分图组。"""
    events = []
    context = scan_context(tmp_path, monkeypatch, events)
    if failure == "capture":
        original_capture = context.controller.wait_for_observation
        # 首张也必须满足正式取图的新鲜度校验。
        def capture(*, after_stamp_ns, view="front"):
            """第一张成功，左侧视角采集失败。"""
            return original_capture(after_stamp_ns=after_stamp_ns, view=view) if not events else None
        context.controller.wait_for_observation = capture
    else:
        monkeypatch.setattr(flight, "rotate", Mock(side_effect=[
            {"success": True, "degrees": 90},
            {"success": False, "error": "HEADING_INVALID", "message": "heading invalid"},
        ]))
    result = dispatcher.dispatch_tool_call(context, activate_call())
    assert not result["success"] and not result["scan_completed"] and not result["heading_restored"]
    assert result["completed_turns"] == 1
    assert "motion_executed" not in result
    assert context.search_observations is None
    assert context.observation is None and context.depth_rules is None


@pytest.mark.parametrize("exit_kind", ["budget", "model_failure"])
def test_pending_scan_images_are_cleared_on_all_loop_exits(tmp_path, monkeypatch, exit_kind):
    """预算耗尽或服务失败也不能留下四图和扫描深度。"""
    context = scan_context(tmp_path, monkeypatch, [])
    client = FakeClient([SimpleNamespace(content="", tool_calls=[activate_call()])])
    if exit_kind == "budget":
        monkeypatch.setattr(loop_module, "MAX_TOOL_CALLS_PER_TURN", 1)
    messages = [{"role": "user", "content": "找目标"}]
    agent_loop(client, "test", messages, context)
    assert image_count(messages) == 0
    assert context.search_observations is None
    assert context.observation is None and context.depth_rules is None


def test_real_scan_keeps_each_rotate_approval_and_denial_stops_scan(tmp_path, monkeypatch):
    """实机通过原旋转审批，拒绝时不执行任何旋转。"""
    context = scan_context(tmp_path, monkeypatch, [])
    context.profile = replace(context.profile, name="real", mode="real")
    context.controller.vehicle_status = SimpleNamespace(mode="OFFBOARD", connected=True, armed=True)
    context.controller.flight_state = lambda: "IN_AIR"
    rotate = Mock()
    monkeypatch.setattr(flight, "rotate", rotate)
    monkeypatch.setattr(dispatcher, "_confirm_flight_tool", Mock(side_effect=EndCurrentTurn("用户拒绝")))
    with pytest.raises(EndCurrentTurn, match="用户拒绝"):
        dispatcher.dispatch_tool_call(context, activate_call())
    rotate.assert_not_called()
    assert context.search_observations is None and context.observation is None


def test_activation_rejects_unknown_name_and_empty_dynamic_prompt(tmp_path, monkeypatch):
    """非法启动参数不会触达采图和飞控。"""
    capture = Mock()
    context = context_for(tmp_path, SimpleNamespace(wait_for_observation=capture))
    for arguments in ({"name": "../outside", "prompt": "找目标"},
                      {"name": "four-direction-search", "prompt": "   "}):
        assert not search.activate_skill(context, arguments)["success"]
    capture.assert_not_called()


def test_skill_catalog_uses_metadata_and_rejects_invalid_resource(tmp_path):
    """目录加载检查元数据，并且索引只展示摘要而非完整正文。"""
    skill = load_skills()[0]
    assert skill.name == "four-direction-search" and skill.enabled
    assert skill.description in build_skills_index("sim")
    assert skill.body not in build_skills_index("sim")
    folder = tmp_path / "broken"
    folder.mkdir()
    (folder / "SKILL.md").write_text("---\nname: wrong\n---\n正文")
    with pytest.raises(ValueError):
        load_skills(tmp_path)
