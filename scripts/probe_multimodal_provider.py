#!/usr/bin/env python3
"""用合成彩色图和假工具结果探测单一 VLM 接口，不连接飞控。"""

from __future__ import annotations

import base64
import getpass
import json
import sys
import time

import cv2
import numpy as np
from openai import OpenAI

from drone_harness.llm.client import normalize_chat_base_url, normalize_proxy_environment
from drone_harness.tools.registry import get_tool_schemas


def synthetic_image_url(bgr: tuple[int, int, int]) -> str:
    """在内存中把非敏感纯色测试图编码为 JPEG data URL。"""
    image = np.full((32, 32, 3), bgr, dtype=np.uint8)
    success, encoded = cv2.imencode(".jpg", image)
    if not success:
        raise RuntimeError("synthetic image encoding failed")
    return "data:image/jpeg;base64," + base64.b64encode(encoded.tobytes()).decode("ascii")


def main() -> int:
    """验证图片、四工具和假工具结果续聊的兼容性与耗时。"""
    key = getpass.getpass("一次性 API Key（不保存、不回显）: ").strip()
    if not key:
        print("probe_error=missing_key")
        return 1
    normalize_proxy_environment()
    client = OpenAI(
        api_key=key,
        base_url=normalize_chat_base_url("https://open.bigmodel.cn/api/paas/v4/chat/completions"),
        timeout=20.0,
        max_retries=0,
    )
    tools = get_tool_schemas()
    messages = [
        {"role": "system", "content": (
            "This is an offline API compatibility test. No drone or tool execution exists. "
            "For the first reply propose exactly one rotate function call: left, 5 degrees.")},
        {"role": "user", "content": [
            {"type": "text", "text": "Inspect this synthetic red image and propose the mock tool call."},
            {"type": "image_url", "image_url": {"url": synthetic_image_url((0, 0, 255)), "detail": "low"}},
        ]},
    ]
    try:
        started = time.monotonic()
        first = client.chat.completions.create(
            model="glm-5.3-flash", messages=messages, tools=tools,
            tool_choice="auto", temperature=0.0,
        )
        reply = first.choices[0].message
        calls = reply.tool_calls or []
        print(json.dumps({
            "first_ok": True,
            "elapsed_s": round(time.monotonic() - started, 2),
            "tool_calls": len(calls),
            "tool_names": [call.function.name for call in calls],
            "finish_reason": first.choices[0].finish_reason,
        }, ensure_ascii=False), flush=True)
        if len(calls) != 1:
            return 2
        call = calls[0]
        messages.append({
            "role": "assistant", "content": reply.content or "",
            "tool_calls": [{"id": call.id, "type": "function", "function": {
                "name": call.function.name, "arguments": call.function.arguments,
            }}],
        })
        messages.append({
            "role": "tool", "tool_call_id": call.id,
            "content": json.dumps({"success": True, "message": "mock only; no controller"}),
        })
        messages.append({"role": "user", "content": [
            {"type": "text", "text": "New image after the mock result. Name the new color; no tool call."},
            {"type": "image_url", "image_url": {"url": synthetic_image_url((0, 255, 0)), "detail": "low"}},
        ]})
        started = time.monotonic()
        second = client.chat.completions.create(
            model="glm-5.3-flash", messages=messages, tools=tools,
            tool_choice="none", temperature=0.0,
        )
        final = second.choices[0].message
        print(json.dumps({
            "second_ok": True,
            "elapsed_s": round(time.monotonic() - started, 2),
            "tool_calls": len(final.tool_calls or []),
            "has_text": bool(final.content),
            "finish_reason": second.choices[0].finish_reason,
        }, ensure_ascii=False), flush=True)
        return 0 if final.content and not final.tool_calls else 3
    except Exception as exc:
        print(json.dumps({"probe_error_type": type(exc).__name__,
                          "status_code": getattr(exc, "status_code", None)}), flush=True)
        return 1


if __name__ == "__main__":
    sys.exit(main())
