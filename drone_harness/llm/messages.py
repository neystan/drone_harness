"""保存模型推理原文，并按目标接口能力构造请求视图。"""

from typing import Any
from urllib.parse import urlparse

from drone_harness.config.schema import ProviderConfig
from drone_harness.llm.client import normalize_chat_base_url


def reasoning_content_enabled(provider: ProviderConfig, model: str) -> bool:
    """只自动启用已核实的官方组合，其他接口由显式开关决定。"""
    if provider.reasoning_content_mode != 'auto':
        return provider.reasoning_content_mode == 'on'
    endpoint = urlparse(normalize_chat_base_url(provider.base_url))
    return (endpoint.scheme == 'https' and endpoint.hostname == 'open.bigmodel.cn'
            and endpoint.port in {None, 443}
            and endpoint.path == '/api/paas/v4'
            and model in {'glm-5.3-flash', 'glm-5.3'})


def prepare_model_messages(messages: list[dict[str, Any]], provider: ProviderConfig,
                           model: str) -> list[dict[str, Any]]:
    """过滤发送视图中的推理字段，不修改保留的原始消息或图组。"""
    enabled = reasoning_content_enabled(provider, model)
    result = []
    for message in messages:
        outgoing = dict(message)
        if not (enabled and message.get('role') == 'assistant'
                and isinstance(message.get('reasoning_content'), str)):
            outgoing.pop('reasoning_content', None)
        result.append(outgoing)
    return result


def build_assistant_message(message: Any, tool_calls: list[Any]) -> dict[str, Any]:
    """统一保存普通与工具回复，仅额外保留真实字符串形式的推理。"""
    result = {'role': 'assistant', 'content': message.content or ''}
    if tool_calls:
        result['tool_calls'] = [
            {'id': call.id, 'type': 'function',
             'function': {'name': call.function.name, 'arguments': call.function.arguments}}
            for call in tool_calls]
    reasoning = getattr(message, 'reasoning_content', None)
    if isinstance(reasoning, str):
        result['reasoning_content'] = reasoning
    return result


def assistant_history_message(message: dict[str, Any]) -> dict[str, Any] | None:
    """清理旧工具协议时删除其推理，普通回复保留原文及对应推理。"""
    content = message.get('content')
    result = {'role': 'assistant', 'content': content if isinstance(content, str) else ''}
    reasoning = message.get('reasoning_content')
    if not message.get('tool_calls') and isinstance(reasoning, str):
        result['reasoning_content'] = reasoning
    return result if result['content'] or 'reasoning_content' in result else None


def reasoning_request_details(history: list[dict[str, Any]],
                              outgoing: list[dict[str, Any]]) -> dict[str, Any]:
    """仅统计内存与发送视图中的推理字符数，不提供原文给日志。"""
    retained = [message['reasoning_content'] for message in history
                if message.get('role') == 'assistant' and isinstance(message.get('reasoning_content'), str)]
    sent = [message['reasoning_content'] for message in outgoing
            if message.get('role') == 'assistant' and isinstance(message.get('reasoning_content'), str)]
    return {'reasoning_history_chars': sum(map(len, retained)),
            'reasoning_sent': bool(sent), 'reasoning_input_chars': sum(map(len, sent))}


def response_reasoning_chars(response: Any) -> int | None:
    """只统计同次SDK回复的可选推理长度，结构缺失时保持未知。"""
    choices = getattr(response, 'choices', None)
    message = getattr(choices[0], 'message', None) if isinstance(choices, (list, tuple)) and choices else None
    reasoning = getattr(message, 'reasoning_content', None)
    return len(reasoning) if isinstance(reasoning, str) else None
