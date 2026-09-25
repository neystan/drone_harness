from __future__ import annotations

from drone_harness.llm.client import normalize_proxy_environment


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
