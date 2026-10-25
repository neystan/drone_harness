"""核对推理原文的保留、回传、模型切换与工具链清理，不访问外网。"""

from copy import deepcopy
import json
from types import SimpleNamespace

import httpx2
import pytest

import drone_harness.runtime.agent_loop as loop
import drone_harness.runtime.runtime as runtime
from drone_harness.config.loader import ConfigError, load_profile
from drone_harness.runtime.navigation import compact_navigation_history
from test_agent_observation_loop import context_for
from test_request_retries import client_factory, completion, request_events


REASONING = '  private-reasoning\n先核对工具反馈，再决定下一步。  '
OFFICIAL = 'https://open.bigmodel.cn/api/paas/v4/chat/completions'


def test_current_glm_tool_reasoning_survives_next_request(tmp_path, monkeypatch, client_factory, capsys):
    """当前已核实接口的工具回复推理应原样出现在下一次请求。"""
    context = context_for(tmp_path)
    bodies = []

    def respond(request):
        """返回一轮带推理的工具回复和一次最终文字。"""
        bodies.append(json.loads(request.content))
        body = completion(tool=len(bodies) == 1)
        body['choices'][0]['message']['reasoning_content'] = REASONING
        return httpx2.Response(200, json=body)

    monkeypatch.setattr(loop, 'dispatch_tool_call', lambda *_: {'success': True, 'motion_executed': False})
    messages = [{'role': 'user', 'content': 'go'}]
    with client_factory(context, respond, base_url=OFFICIAL, model='glm-5.3-flash') as client:
        assert loop.agent_loop(client, 'glm-5.3-flash', messages, context) == '完成'
    assert all(body['reasoning_effort'] == 'high' for body in bodies)
    tool_message = next(m for m in bodies[1]['messages'] if m.get('tool_calls'))
    assert tool_message['reasoning_content'] == REASONING
    assert messages[-1]['reasoning_content'] == REASONING
    events = request_events(tmp_path)
    assert events[1]['reasoning_output_chars'] == len(REASONING)
    assert events[2]['reasoning_sent'] is True
    assert events[2]['reasoning_input_chars'] == len(REASONING)
    assert 'private-reasoning' not in capsys.readouterr().out
    for path in (tmp_path/'logs/session_test').glob('*.jsonl'):
        assert 'private-reasoning' not in path.read_text()


@pytest.mark.parametrize('mode,url,model,sent', [
    ('auto', OFFICIAL, 'glm-5.3-flash', True),
    ('auto', OFFICIAL, 'glm-5.3', True),
    ('auto', OFFICIAL, 'unknown-model', False),
    ('auto', 'https://other.invalid/v4', 'glm-5.3-flash', False),
    ('auto', 'https://open.bigmodel.cn.other.invalid/api/paas/v4', 'glm-5.3-flash', False),
    ('auto', 'https://open.bigmodel.cn/unknown-api', 'glm-5.3-flash', False),
    ('auto', 'https://open.bigmodel.cn:8443/api/paas/v4', 'glm-5.3-flash', False),
    ('on', 'https://other.invalid/v4', 'other-model', True),
    ('off', OFFICIAL, 'glm-5.3-flash', False),
])
def test_plain_reply_retention_and_provider_policy(tmp_path, client_factory, mode, url, model, sent):
    """普通回复保留原文，发送视图按端点、型号和显式开关过滤。"""
    context = context_for(tmp_path)
    bodies = []

    def respond(request):
        """模拟始终携带可选推理字段的兼容模型。"""
        bodies.append(json.loads(request.content))
        body = completion()
        body['choices'][0]['message']['reasoning_content'] = REASONING
        return httpx2.Response(200, json=body)

    messages = [{'role': 'user', 'content': 'first'}]
    with client_factory(context, respond, base_url=url, model=model, reasoning_content_mode=mode) as client:
        loop.agent_loop(client, model, messages, context)
        assert messages[-1]['reasoning_content'] == REASONING
        messages.append({'role': 'user', 'content': 'next'})
        loop.agent_loop(client, model, messages, context)
    previous = next(m for m in bodies[1]['messages'] if m['role'] == 'assistant')
    assert ('reasoning_content' in previous) is sent
    if sent:
        assert previous['reasoning_content'] == REASONING
    events = request_events(tmp_path)
    assert events[2]['reasoning_history_chars'] == len(REASONING)
    assert events[2]['reasoning_input_chars'] == (len(REASONING) if sent else 0)


@pytest.mark.parametrize('value', [None, 123, {}, '', '  exact\n'])
def test_optional_reasoning_is_not_fabricated(tmp_path, client_factory, value):
    """只保存真实字符串，缺失或错误类型不伪造，空白原样保留。"""
    context = context_for(tmp_path)

    def respond(_request):
        """返回不同类型的模型扩展字段。"""
        body = completion()
        body['choices'][0]['message']['reasoning_content'] = value
        return httpx2.Response(200, json=body)

    messages = []
    with client_factory(context, respond) as client:
        loop.agent_loop(client, 'test-model', messages, context)
    assert ('reasoning_content' in messages[-1]) is isinstance(value, str)
    if isinstance(value, str):
        assert messages[-1]['reasoning_content'] == value


def test_compaction_keeps_latest_pair_and_plain_reasoning_only():
    """旧工具推理随协议清理，纯文字推理与最新计划协议对仍保留。"""
    history = [
        {'role': 'system', 'content': 'system'}, {'role': 'user', 'content': 'go'},
        {'role': 'assistant', 'content': 'old decision', 'reasoning_content': 'old tool reasoning',
         'tool_calls': [{'id': 'old'}]},
        {'role': 'tool', 'content': '{}', 'tool_call_id': 'old'},
        {'role': 'assistant', 'content': 'plain reply', 'reasoning_content': REASONING},
        {'role': 'assistant', 'content': '', 'reasoning_content': 'reasoning only'},
        {'role': 'assistant', 'content': 'plan', 'reasoning_content': 'latest plan reasoning',
         'tool_calls': [{'id': 'plan'}]},
        {'role': 'tool', 'content': '{}', 'tool_call_id': 'plan'},
    ]
    original = deepcopy(history)
    compact_navigation_history(history, 2, observation_current=False)
    assert 'old tool reasoning' not in str(history)
    assert next(m for m in history if m['content'] == 'plain reply')['reasoning_content'] == REASONING
    assert any(m.get('reasoning_content') == 'reasoning only' for m in history)
    assert history[-2]['reasoning_content'] == 'latest plan reasoning'
    assert history[-2]['tool_calls'][0]['id'] == history[-1]['tool_call_id']
    assert original[2]['reasoning_content'] == 'old tool reasoning'


def test_retry_keeps_reasoning_images_and_provider_fields_unchanged(tmp_path, monkeypatch, client_factory):
    """新推理先保留再参与失败重试，原请求不因清理或过滤被修改。"""
    context = context_for(tmp_path)
    bodies = []

    def respond(request):
        """首轮返回工具，第二轮首次失败、重试成功。"""
        bodies.append(json.loads(request.content))
        if len(bodies) == 2:
            return httpx2.Response(500, headers={'retry-after-ms': '1'}, json={'error': {'code': '5001'}})
        body = completion(tool=len(bodies) == 1)
        body['choices'][0]['message']['reasoning_content'] = REASONING
        return httpx2.Response(200, json=body)

    invoked = []

    def dispatch(_context, call):
        """返回只查询状态的假工具结果并记录执行次数。"""
        invoked.append(call.function.name)
        return {'success': True, 'motion_executed': False}

    monkeypatch.setattr(loop, 'dispatch_tool_call', dispatch)
    images = [{'type': 'image_url', 'image_url': {'url': f'offline-image-{i}'}} for i in range(4)]
    messages = [{'role': 'user', 'content': [{'type': 'text', 'text': 'images'}, *images]}]
    with client_factory(context, respond, reasoning_content_mode='on') as client:
        loop.agent_loop(client, 'test-model', messages, context)
    assert len(bodies) == 3 and bodies[1] == bodies[2]
    assert invoked == ['forward']
    assert any(m.get('reasoning_content') == REASONING for m in bodies[1]['messages'])
    assert request_events(tmp_path)[-1]['attempt_count'] == 2


def test_interactive_turn_keeps_plain_reasoning_but_drops_old_tool_reasoning(tmp_path, monkeypatch):
    """交互跨轮只保留完整普通回复的推理，不带孤立的旧工具推理。"""
    context = context_for(tmp_path)
    inputs = iter(['first', 'next', 'exit'])
    context.message_bus = SimpleNamespace(consume_user_message=lambda: SimpleNamespace(content=next(inputs)))
    monkeypatch.setattr(runtime, '_configure_readline', lambda: None)
    calls = []

    def fake_agent(_client, _model, messages, _context):
        """第一轮生成两种回复，第二轮检查历史是否完整且无旧协议。"""
        calls.append(deepcopy(messages))
        messages.extend([
            {'role': 'assistant', 'content': 'old tool', 'reasoning_content': 'old tool reasoning',
             'tool_calls': [{'id': 'old'}]},
            {'role': 'tool', 'content': '{}', 'tool_call_id': 'old'},
            {'role': 'assistant', 'content': 'plain', 'reasoning_content': REASONING}])
        return 'plain'

    runtime._run_interactive_loop(object(), 'test', context, fake_agent, input_terminal_started=True)
    assert len(calls) == 2
    assert any(m.get('reasoning_content') == REASONING for m in calls[1])
    assert not any(m.get('reasoning_content') == 'old tool reasoning' for m in calls[1])
    assert not any(m.get('tool_calls') or m['role'] == 'tool' for m in calls[1])


@pytest.mark.parametrize('value', [None, True, 'enabled', '', [], 1])
def test_reasoning_mode_rejects_invalid_config(tmp_path, value):
    """推理回传模式只接受明确的auto/on/off。"""
    path = tmp_path/'settings.json'
    path.write_text(json.dumps({'llm': {'api_key': 'offline', 'base_url': 'https://offline.invalid',
                                       'model': 'test', 'reasoning_content_mode': value}}))
    with pytest.raises(ConfigError, match='reasoning_content_mode'):
        load_profile('sim', settings_path=path)


def test_default_mode_and_switching_request_view_do_not_mutate_history(tmp_path):
    """默认自动模式可安全切换目标视图，过滤不会破坏原始推理或图组。"""
    from dataclasses import replace
    from drone_harness.llm.messages import prepare_model_messages

    context = context_for(tmp_path)
    provider = context.profile.llm
    assert provider.reasoning_content_mode == 'auto'
    images = [{'role': 'user', 'content': [{'type': 'image_url', 'image_url': {'url': 'offline'}}]},
              {'role': 'assistant', 'content': 'reply', 'reasoning_content': REASONING}]
    original = deepcopy(images)
    official = prepare_model_messages(images, provider, 'glm-5.3-flash')
    unknown = prepare_model_messages(images, replace(provider, base_url='https://other.invalid/v1'), 'glm-5.3-flash')
    enabled = prepare_model_messages(images, replace(provider, base_url='https://other.invalid/v1',
                                                     reasoning_content_mode='on'), 'other-model')
    assert official[-1]['reasoning_content'] == enabled[-1]['reasoning_content'] == REASONING
    assert 'reasoning_content' not in unknown[-1]
    assert images == original and unknown[0] == official[0]
    assert prepare_model_messages(images, provider, 'glm-5.3-flash') == official
