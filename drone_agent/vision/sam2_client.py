"""SAM2 Docker 追踪服务的 HTTP 客户端。"""

from __future__ import annotations

import json
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


class Sam2ClientError(RuntimeError):
    """SAM2 服务调用失败。"""


class Sam2TrackingClient:
    """封装 drone_agent 到 SAM2 Docker 服务的最小 HTTP 调用。"""

    def __init__(self, base_url: str, timeout_s: float) -> None:
        """保存服务地址和请求超时时间。"""
        self.base_url = base_url.rstrip("/")
        self.timeout_s = timeout_s

    def start(self, payload: dict[str, Any]) -> dict[str, Any]:
        """用目标框或鼠标点提示启动一次 SAM2 追踪。"""
        return self._post("/tracking/start", payload)

    def status(self, payload: dict[str, Any]) -> dict[str, Any]:
        """用当前帧更新并查询追踪状态。"""
        return self._post("/tracking/status", payload)

    def stop(self, payload: dict[str, Any]) -> dict[str, Any]:
        """停止 Docker 服务中的追踪会话。"""
        return self._post("/tracking/stop", payload)

    def _post(self, path: str, payload: dict[str, Any]) -> dict[str, Any]:
        """发送 JSON 请求并解析 JSON 响应。"""
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        request = Request(
            f"{self.base_url}{path}",
            data=body,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            with urlopen(request, timeout=self.timeout_s) as response:
                raw = response.read().decode("utf-8")
        except HTTPError as exc:
            message = exc.read().decode("utf-8", errors="replace")
            raise Sam2ClientError(f"SAM2 service HTTP {exc.code}: {message}") from exc
        except URLError as exc:
            raise Sam2ClientError(f"failed to connect SAM2 service: {exc.reason}") from exc
        except TimeoutError as exc:
            raise Sam2ClientError("SAM2 service request timed out") from exc

        try:
            result = json.loads(raw) if raw else {}
        except json.JSONDecodeError as exc:
            raise Sam2ClientError(f"SAM2 service returned non-JSON response: {raw}") from exc
        if not isinstance(result, dict):
            raise Sam2ClientError("SAM2 service response must be a JSON object")
        return result
