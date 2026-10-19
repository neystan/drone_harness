"""通过真实SDK和内存HTTP传输核对请求重试，不访问网络或飞控。"""

import json
from dataclasses import replace
from types import SimpleNamespace

import httpx2
import openai
import pytest

import drone_harness.runtime.agent_loop as loop
from drone_harness.llm.client import create_llm_client
from test_agent_observation_loop import context_for


def completion(*, tool=False):
    """构造SDK可解析的单工具或完成响应。"""
    message = {'role': 'assistant', 'content': '继续' if tool else '完成'}
    if tool:
        message['tool_calls'] = [{'id': 'move-1', 'type': 'function', 'function': {
            'name': 'forward', 'arguments': '{"distance_m":1}'}}]
    return {'id': 'offline', 'object': 'chat.completion', 'created': 1,
            'model': 'test-model', 'request_id': 'provider-ok',
            'choices': [{'index': 0, 'finish_reason': 'tool_calls' if tool else 'stop',
                         'message': message}]}


@pytest.fixture
def client_factory(monkeypatch):
    """只替换HTTP传输，保留生产客户端的SDK参数和钩子。"""
    original = openai.DefaultHttpxClient

    def build(context, handler, max_retries=2):
        """为一次离线测试创建带原始生产钩子的客户端。"""
        def http_client(**options):
            """让所有HTTP尝试都进入内存响应器。"""
            return original(transport=httpx2.MockTransport(handler), **options)

        monkeypatch.setattr(openai, 'DefaultHttpxClient', http_client)
        context.profile = replace(context.profile, post_motion_wait_enabled=False,
                                  llm=replace(context.profile.llm, api_key='offline-test-key',
                                              base_url='https://offline.invalid/v4',
                                              model='test-model', max_retries=max_retries))
        return create_llm_client(context.profile)

    return build


def request_events(tmp_path):
    """读取一轮离线会话的逻辑请求记录。"""
    path = tmp_path/'logs/session_test/model_requests.jsonl'
    return [json.loads(line) for line in path.read_text().splitlines()]


@pytest.mark.parametrize('image_count', [0, 1, 2, 4])
def test_retry_reuses_same_payload_and_executes_motion_once(tmp_path, monkeypatch, client_factory, image_count):
    """首次500后恢复，重试图组不变且已返回动作只执行一次。"""
    context = context_for(tmp_path)
    bodies = []
    invoked = []

    def respond(request):
        """先模拟上游错误，再返回一次前进与最终文字。"""
        bodies.append(json.loads(request.content))
        if len(bodies) == 1:
            return httpx2.Response(500, headers={'retry-after-ms': '1', 'x-request-id': 'provider-failed'},
                                   json={'error': {'code': '5001', 'message': 'private-error'}})
        return httpx2.Response(200, json=completion(tool=len(bodies) == 2))

    def dispatch(_context, call):
        """记录动作执行次数而不触碰飞控。"""
        invoked.append(call.function.name)
        return {'success': True, 'motion_executed': True, 'final_position_ned': [1, 0, -3]}

    monkeypatch.setattr(loop, 'dispatch_tool_call', dispatch)
    messages = [{'role': 'user', 'content': [{'type': 'text', 'text': 'private-task'},
                *[{'type': 'image_url', 'image_url': {'url': 'private-image'}} for _ in range(image_count)]]}]
    with client_factory(context, respond) as client:
        assert loop.agent_loop(client, 'test-model', messages, context) == '完成'
    assert len(bodies) == 3 and bodies[0] == bodies[1]
    assert invoked == ['forward']
    events = request_events(tmp_path)
    assert [r['status'] for r in events] == ['started', 'completed', 'started', 'completed']
    assert events[1]['attempt_count'] == 2 and events[3]['attempt_count'] == 1
    assert [r['http_status'] for r in events[1]['attempts']] == [500, 200]
    assert events[1]['attempts'][0]['provider_request_id'] == 'provider-failed'
    assert events[1]['provider_request_id'] == 'provider-ok'
    assert events[1]['image_count'] == image_count
    assert not any(value in json.dumps(events) for value in ['private-task', 'private-image', 'private-error'])


@pytest.mark.parametrize('status,attempts', [(400, 1), (401, 1), (403, 1), (422, 1),
                                           (408, 3), (409, 3), (429, 3), (503, 3)])
def test_http_error_policy_and_exhaustion(tmp_path, monkeypatch, client_factory, status, attempts):
    """不可恢复错误立即停止，可重试错误严格耗尽预算后停止。"""
    context = context_for(tmp_path)
    called = []
    invoked = []

    def respond(request):
        """反复返回同一种带业务码的HTTP错误。"""
        called.append(request)
        return httpx2.Response(status, headers={'retry-after-ms': '1'}, json={
            'error': {'code': '1300', 'message': 'private-error', 'request_id': 'provider-error'}})

    monkeypatch.setattr(loop, 'dispatch_tool_call', lambda *_: invoked.append(True))
    with client_factory(context, respond) as client:
        assert '失败' in loop.agent_loop(client, 'test-model', [], context)
    assert len(called) == attempts and invoked == []
    events = request_events(tmp_path)
    assert [r['status'] for r in events] == ['started', 'failed']
    assert events[-1]['attempt_count'] == attempts
    assert events[-1]['http_status'] == status
    assert events[-1]['provider_error_code'] == '1300'
    assert events[-1]['provider_request_id'] == 'provider-error'
    assert 'private-error' not in json.dumps(events)


@pytest.mark.parametrize('kind', ['connection', 'timeout', 'cancel'])
def test_transport_retry_and_cancel(tmp_path, client_factory, kind):
    """连接与超时重试，KeyboardInterrupt取消只尝试一次并保留日志。"""
    context = context_for(tmp_path)
    called = []

    def respond(request):
        """只在首个HTTP尝试触发指定异常。"""
        called.append(request)
        if len(called) == 1:
            if kind == 'cancel':
                raise KeyboardInterrupt('private-cancel')
            error = httpx2.ConnectError if kind == 'connection' else httpx2.ReadTimeout
            raise error('private-transport', request=request)
        return httpx2.Response(200, json=completion())

    with client_factory(context, respond) as client:
        if kind == 'cancel':
            with pytest.raises(KeyboardInterrupt):
                loop.agent_loop(client, 'test-model', [], context)
        else:
            assert loop.agent_loop(client, 'test-model', [], context) == '完成'
    assert len(called) == (1 if kind == 'cancel' else 2)
    events = request_events(tmp_path)
    assert events[-1]['attempt_count'] == len(called)
    assert 'private-' not in json.dumps(events)


def test_zero_retries_and_malformed_success_do_not_retry(tmp_path, client_factory):
    """关闭重试后不重复发送，200结构错误也不进入HTTP重试。"""
    for status in [500, 200]:
        context = context_for(tmp_path/str(status))
        called = []

        def respond(request):
            """返回一次服务错误或缺少选择项的成功响应。"""
            called.append(request)
            return httpx2.Response(status, json={'choices': []})

        with client_factory(context, respond, max_retries=0 if status == 500 else 2) as client:
            assert '失败' in loop.agent_loop(client, 'test-model', [], context)
        assert len(called) == 1


def test_error_metadata_redacts_secret_and_unexpected_values(tmp_path, client_factory):
    """服务端错误字段含密钥或非标内容时不进入诊断日志。"""
    context = context_for(tmp_path)

    def respond(_request):
        """模拟错误对象回显密钥和内嵌图片。"""
        return httpx2.Response(400, headers={'x-request-id': 'offline-test-key'}, json={
            'error': {'code': 'offline-test-key', 'request_id': 'data:image/jpeg;base64,private-image',
                      'message': 'private-error'}})

    with client_factory(context, respond) as client:
        loop.agent_loop(client, 'test-model', [], context)
    events = request_events(tmp_path)
    assert events[-1]['provider_error_code'] is None
    assert events[-1]['provider_request_id'] is None
    assert not any(value in json.dumps(events) for value in ['offline-test-key', 'private-image', 'private-error'])


def test_pending_user_intervention_stops_before_next_http_attempt(tmp_path, client_factory):
    """首次失败期间收到用户介入时，不再实际发送第二次请求。"""
    context = context_for(tmp_path, SimpleNamespace(current_position_ned=lambda: [0, 0, -3]))
    pending = []
    called = []
    context.message_bus = SimpleNamespace(has_pending_user_message=lambda: bool(pending),
                                         get_next_user_message=lambda: pending.pop(0))

    def respond(request):
        """模拟服务错误返回前用户发送了停止指令。"""
        called.append(request)
        pending.append(SimpleNamespace(content='停止'))
        return httpx2.Response(500, headers={'retry-after-ms': '1'}, json={'error': {'code': '5001'}})

    with client_factory(context, respond) as client:
        assert loop.agent_loop(client, 'test-model', [], context) == 'tool interrupted by user input'
    assert len(called) == 1 and pending == []
    assert request_events(tmp_path)[-1]['attempt_count'] == 1


@pytest.mark.parametrize('status,hint,attempts', [(503, 'false', 1), (400, 'true', 2)])
def test_sdk_honors_explicit_server_retry_hint(tmp_path, client_factory, status, hint, attempts):
    """沿用SDK的服务端重试指示，不另写冲突的状态码循环。"""
    context = context_for(tmp_path)
    called = []

    def respond(request):
        """首次返回明确重试指示，后续尝试返回成功。"""
        called.append(request)
        if len(called) == 1:
            return httpx2.Response(status, headers={'x-should-retry': hint, 'retry-after-ms': '1'},
                                   json={'error': {'code': 'offline'}})
        return httpx2.Response(200, json=completion())

    with client_factory(context, respond) as client:
        loop.agent_loop(client, 'test-model', [], context)
    assert len(called) == attempts
