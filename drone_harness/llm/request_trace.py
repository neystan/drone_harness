"""记录SDK内部HTTP尝试的允许元数据，不读取或保存请求正文。"""

from contextlib import contextmanager
from contextvars import ContextVar
from dataclasses import dataclass, field
import re
from typing import Any, Callable


def _identifier(value: Any, secret: str) -> str | None:
    """仅接受短标识符，并排除密钥回显和非标正文。"""
    if type(value) not in {str, int}:
        return None
    text = str(value)
    if secret and secret in text:
        return None
    return text if re.fullmatch(r'[A-Za-z0-9_.:-]{1,128}', text) else None


@dataclass
class RequestTrace:
    """隔离一次逻辑模型调用的HTTP尝试，不负责重试决策。"""

    api_key: str = field(repr=False)
    attempts: list[dict[str, Any]] = field(default_factory=list)
    before_retry: Callable[[], None] | None = field(default=None, repr=False)

    def details(self, *, response: Any = None, error: BaseException | None = None) -> dict[str, Any]:
        """从最终结果补充业务码和请求编号，忽略错误正文。"""
        last = self.attempts[-1] if self.attempts else {}
        body = getattr(error, 'body', None)
        body = body if isinstance(body, dict) else {}
        provider_error = body.get('error', body)
        provider_error = provider_error if isinstance(provider_error, dict) else {}
        provider_id = (last.get('provider_request_id')
                       or _identifier(getattr(response, 'request_id', None), self.api_key)
                       or _identifier(body.get('request_id'), self.api_key)
                       or _identifier(provider_error.get('request_id'), self.api_key))
        return {'attempt_count': len(self.attempts) or None,
                'attempts': [dict(attempt) for attempt in self.attempts],
                'http_status': last.get('http_status'),
                'provider_error_code': _identifier(provider_error.get('code'), self.api_key),
                'provider_request_id': provider_id}


_current_trace: ContextVar[RequestTrace | None] = ContextVar('model_request_trace', default=None)


@contextmanager
def trace_model_request(api_key: str, *, before_retry: Callable[[], None] | None = None):
    """仅在当前调用线程绑定跟踪器，所有出口均恢复原上下文。"""
    trace = RequestTrace(api_key, before_retry=before_retry)
    token = _current_trace.set(trace)
    try:
        yield trace
    finally:
        _current_trace.reset(token)


def record_http_request(_request: Any) -> None:
    """由公开HTTP钩子统计真实发送尝试，不接触消息和认证头。"""
    trace = _current_trace.get()
    if trace is not None:
        if trace.attempts and trace.before_retry is not None:
            trace.before_retry()
        trace.attempts.append({'attempt': len(trace.attempts) + 1,
                               'http_status': None, 'provider_request_id': None})


def record_http_response(response: Any) -> None:
    """只采集HTTP状态和允许的请求编号头，不消费响应体。"""
    trace = _current_trace.get()
    if trace is not None and trace.attempts:
        last = trace.attempts[-1]
        last['http_status'] = response.status_code
        last['provider_request_id'] = (_identifier(response.headers.get('x-request-id'), trace.api_key)
                                       or _identifier(response.headers.get('request-id'), trace.api_key))
