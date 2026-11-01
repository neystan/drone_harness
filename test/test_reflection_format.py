"""验证普通观察、阶段切换与四向搜索的实际请求格式。"""

import json
from copy import deepcopy
from types import SimpleNamespace

import pytest

from drone_harness.llm.prompts import build_system_prompt
from drone_harness.runtime.agent_loop import agent_loop, append_observation
from drone_harness.runtime.navigation import navigation_messages, parse_navigation_plan
from drone_harness.tools import flight
from test_agent_observation_loop import FakeClient, context_for
from test_depth_rules import snapshot_at
from test_navigation_plan import plan_payload
from test_reflection import assessment
from test_search_skill import activate_call, image_count, scan_context
from test_stage3_observe import tool_call
from test_terminal_arrival import arrival_reply


def output_example(request):
    """从完整系统请求中读取唯一格式示例，而非动态判断记录。"""
    prompt = request[0]['content']
    marker = '在同次看图回复开头输出一行JSON：'
    assert prompt.count(marker) == 1
    return json.JSONDecoder().raw_decode(prompt.split(marker, 1)[1])[0]


def test_non_navigation_observation_keeps_five_field_format(tmp_path):
    """非导航普通观察仍携带五字段示例与当前图，不增加请求。"""
    context = context_for(tmp_path)
    history = [{'role': 'system', 'content': build_system_prompt(context.profile)},
               {'role': 'user', 'content': '看看前面的车辆'}]
    append_observation(context, history, snapshot_at())
    client = FakeClient([SimpleNamespace(content=assessment(), tool_calls=[])])
    agent_loop(client, 'test', history, context)
    assert len(client.requests) == 1 and image_count(client.requests[0]) == 1
    assert set(output_example(client.requests[0])) == {
        'target_status', 'candidate', 'support', 'conflict', 'open_issue'}


@pytest.mark.parametrize('with_system', [False, True])
def test_stage_switch_uses_one_complete_format_without_mutating_history(tmp_path, with_system):
    """同一原历史反复切换阶段，格式不累计且不改变消息及图片来源。"""
    profile = context_for(tmp_path).profile
    history = [{'role': 'user', 'content': [{'type': 'text', 'text': '原始观察'},
                {'type': 'image_url', 'image_url': {'url': 'data:image/jpeg;base64,source'}}]}]
    if with_system:
        history.insert(0, {'role': 'system', 'content': build_system_prompt(profile)})
    original = deepcopy(history)
    for terminal in (False, True, False, True):
        plan = parse_navigation_plan(plan_payload(statuses=(
            'completed' if terminal else 'in_progress',
            'in_progress' if terminal else 'pending')), '原指令')
        request = navigation_messages(history, plan, observation_current=True)
        example = output_example(request)
        assert len(example) == (6 if terminal else 5)
        assert ('arrival' in example) is terminal
        if terminal:
            assert set(example['arrival']) == {'position_status', 'evidence'}
        assert history == original and request[-1] == history[-1]
        assert '五字段JSON增加' not in request[0]['content']


@pytest.mark.parametrize('terminal', [False, True])
def test_joint_search_request_uses_same_stage_format(tmp_path, monkeypatch, terminal):
    """四向联合请求使用当前阶段完整格式，四图与请求次数保持原约定。"""
    context = scan_context(tmp_path, monkeypatch, [])
    monkeypatch.setattr(flight, 'land', lambda _context: {'success': True})
    context.navigation_enabled = True
    context.navigation_plan = parse_navigation_plan(plan_payload(statuses=(
        'completed' if terminal else 'in_progress',
        'in_progress' if terminal else 'pending')), '找卡车')
    history = [{'role': 'system', 'content': build_system_prompt(context.profile)},
               {'role': 'user', 'content': '找卡车'}]
    client = FakeClient([
        SimpleNamespace(content='', tool_calls=[activate_call()]),
        SimpleNamespace(content=arrival_reply() if terminal else assessment(), tool_calls=[tool_call('land', {})]),
    ])
    agent_loop(client, 'test', history, context)
    assert [image_count(request) for request in client.requests] == [0, 4]
    for request in client.requests:
        assert len(output_example(request)) == (6 if terminal else 5)
    assert context.search_observations is None and image_count(history) == 0
