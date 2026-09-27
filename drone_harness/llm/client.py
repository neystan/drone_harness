"""创建唯一 OpenAI-compatible 多模态 VLM 客户端。"""

from __future__ import annotations

import os
from collections.abc import MutableMapping
from typing import Any

from drone_harness.config.schema import RuntimeProfile


PROXY_ENVIRONMENT_VARIABLES = (
    "HTTP_PROXY",
    "HTTPS_PROXY",
    "ALL_PROXY",
    "http_proxy",
    "https_proxy",
    "all_proxy",
)


def normalize_proxy_environment(
    environment: MutableMapping[str, str] | None = None,
) -> None:
    """把旧式 socks:// 代理地址转换为 HTTP 客户端支持的 socks5://。"""
    selected_environment = os.environ if environment is None else environment
    for variable_name in PROXY_ENVIRONMENT_VARIABLES:
        proxy_url = selected_environment.get(variable_name, "")
        if proxy_url.lower().startswith("socks://"):
            selected_environment[variable_name] = f"socks5://{proxy_url[8:]}"


def create_llm_client(profile: RuntimeProfile) -> Any:
    """根据单一模型配置创建支持图片与工具调用的客户端。"""
    normalize_proxy_environment()
    from openai import OpenAI

    return OpenAI(
        api_key=profile.llm.api_key,
        base_url=normalize_chat_base_url(profile.llm.base_url),
        timeout=60.0,
        max_retries=0,
    )


def normalize_chat_base_url(url: str) -> str:
    """兼容用户给出的完整 chat/completions 地址与 SDK 基础路径。"""
    normalized = url.rstrip("/")
    suffix = "/chat/completions"
    return normalized[:-len(suffix)] if normalized.endswith(suffix) else normalized
