from __future__ import annotations

from drone_harness.llm.client import normalize_proxy_environment

import os
from dataclasses import replace
from pathlib import Path

import pytest

from drone_harness.config.loader import load_profile
from drone_harness.llm.client import create_llm_client, PROXY_ENVIRONMENT_VARIABLES


@pytest.mark.parametrize("profile_name", ["sim", "real"])
def test_client_ignores_environment_proxies_without_mutation(monkeypatch, profile_name):
    """带无效代理也能创建直连客户端，环境值不改写，不发送网络请求。"""
    for name in PROXY_ENVIRONMENT_VARIABLES:
        monkeypatch.setenv(name, "invalid-proxy-scheme://127.0.0.1:1")
    monkeypatch.setenv("NO_PROXY", "")
    monkeypatch.setenv("no_proxy", "")
    before = dict(os.environ)
    profile = load_profile(profile_name, settings_path=Path(__file__).parents[1] / "settings.example.json")
    profile = replace(profile, llm=replace(profile.llm, api_key="offline-test-key", base_url="https://example.invalid/v1"))
    with create_llm_client(profile) as client:
        assert client._client.trust_env is False
        assert client.timeout == 600.0
        assert client.max_retries == 0
        assert str(client.base_url) == "https://example.invalid/v1/"
    assert dict(os.environ) == before


def test_normalize_proxy_environment_converts_legacy_socks_scheme() -> None:
    environment = {
        "ALL_PROXY": "socks://127.0.0.1:7897",
        "https_proxy": "socks://proxy.example:1080",
    }

    normalize_proxy_environment(environment)

    assert environment["ALL_PROXY"] == "socks5://127.0.0.1:7897"
    assert environment["https_proxy"] == "socks5://proxy.example:1080"


def test_normalize_proxy_environment_preserves_supported_and_unrelated_values() -> None:
    environment = {
        "HTTP_PROXY": "http://127.0.0.1:7897",
        "ALL_PROXY": "socks5h://127.0.0.1:7897",
        "UNRELATED": "socks://leave-this-alone",
    }

    normalize_proxy_environment(environment)

    assert environment == {
        "HTTP_PROXY": "http://127.0.0.1:7897",
        "ALL_PROXY": "socks5h://127.0.0.1:7897",
        "UNRELATED": "socks://leave-this-alone",
    }
