"""验证单模型配置与用户给出的完整接口地址兼容。"""

from __future__ import annotations

import json
from dataclasses import replace
from pathlib import Path

import pytest

from drone_harness.config.loader import ConfigError, load_profile
from drone_harness.llm.client import normalize_chat_base_url


def test_full_chat_completion_url_is_normalized_to_sdk_base() -> None:
    """SDK 会自行追加 chat/completions，避免把用户 URL 拼接两遍。"""
    assert normalize_chat_base_url(
        "https://open.bigmodel.cn/api/paas/v4/chat/completions"
    ) == "https://open.bigmodel.cn/api/paas/v4"
    assert normalize_chat_base_url("https://open.bigmodel.cn/api/paas/v4/") == (
        "https://open.bigmodel.cn/api/paas/v4"
    )


def test_legacy_parallel_model_config_is_rejected(tmp_path: Path) -> None:
    """旧视觉/检测/追踪配置不能悄悄重启第二条模型链。"""
    settings = {"llm": {"api_key": "test-only", "base_url": "https://example.test/v4",
                        "model": "test-vlm"}, "vlm": {"enabled": True}}
    path = tmp_path / "settings.json"
    path.write_text(json.dumps(settings), encoding="utf-8")
    with pytest.raises(ConfigError, match="old multi-model settings"):
        load_profile("sim", settings_path=path)


def test_single_provider_accepts_ephemeral_environment_override(tmp_path: Path, monkeypatch) -> None:
    """密钥可只放在进程环境，不要求写入仓库设置文件。"""
    settings = Path(__file__).parents[1] / "settings.example.json"
    monkeypatch.setenv("DRONE_HARNESS_LLM_API_KEY", "test-only-secret")
    monkeypatch.setenv("DRONE_HARNESS_LLM_BASE_URL", "https://example.test/v4/chat/completions")
    monkeypatch.setenv("DRONE_HARNESS_LLM_MODEL", "test-vlm")
    profile = load_profile("sim", settings_path=settings)
    assert profile.llm.api_key == "test-only-secret"
    assert profile.llm.model == "test-vlm"
    assert not hasattr(profile, "vlm")
    assert not hasattr(profile, "detector")
    assert not hasattr(profile, "tracker")


def test_client_uses_only_one_model_base_url_with_configured_retries(monkeypatch) -> None:
    """正式客户端把完整URL归一化，并向SDK传递唯一重试预算。"""
    import openai
    from drone_harness.llm.client import create_llm_client

    captured: list[dict] = []

    def fake_openai(**kwargs):
        """只记录构造参数而不访问外网。"""
        captured.append(kwargs)
        return object()

    monkeypatch.setattr(openai, "OpenAI", fake_openai)
    settings = Path(__file__).parents[1] / "settings.example.json"
    profile = load_profile("sim", settings_path=settings)
    profile = replace(profile, llm=replace(
        profile.llm, base_url="https://open.bigmodel.cn/api/paas/v4/chat/completions"))
    create_llm_client(profile)
    assert len(captured) == 1
    assert captured[0]["base_url"] == "https://open.bigmodel.cn/api/paas/v4"
    assert captured[0]["max_retries"] == 2
    assert captured[0]["http_client"].trust_env is False
    captured[0]["http_client"].close()


@pytest.mark.parametrize('value', [True, -1, 1.5, '2', None])
def test_retry_budget_rejects_invalid_config(tmp_path, value):
    """重试次数只接受非负整数，避免布尔值或字符串隐式生效。"""
    settings = {'llm': {'api_key': 'offline', 'base_url': 'https://offline.invalid',
                        'model': 'test', 'max_retries': value}}
    path = tmp_path/'settings.json'
    path.write_text(json.dumps(settings))
    with pytest.raises(ConfigError, match='max_retries'):
        load_profile('sim', settings_path=path)


@pytest.mark.parametrize('value', [None, 0, 2])
def test_retry_budget_default_and_override(tmp_path, value):
    """旧配置使用默认两次，显式零值可关闭请求重试。"""
    llm = {'api_key': 'offline', 'base_url': 'https://offline.invalid', 'model': 'test'}
    if value is not None:
        llm['max_retries'] = value
    path = tmp_path/'settings.json'
    path.write_text(json.dumps({'llm': llm}))
    assert load_profile('sim', settings_path=path).llm.max_retries == (2 if value is None else value)
