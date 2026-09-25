"""管理大模型与工具调用循环。"""

from __future__ import annotations

import json
from typing import Any

from drone_agent.logging.task_log import log_agent_message, log_task_state
from drone_agent.runtime.safety import EndCurrentTurn
from drone_agent.runtime.task_state import format_task_state_line
from drone_agent.runtime.tool_dispatcher import dispatch_tool_call
from drone_agent.tools.registry import ToolContext, get_tool_schemas


MAX_TOOL_CALLS_PER_TURN = 50


def agent_loop(
    client: Any,
    model: str,
    messages: list[dict[str, Any]],
    context: ToolContext,
) -> str:
    """执行一轮模型对话，直到得到最终回复或中断。"""
    for _ in range(MAX_TOOL_CALLS_PER_TURN):
        _record_task_state(context, "thinking")
        response = client.chat.completions.create(
            model=model,
            messages=messages,
            tools=get_tool_schemas(),
            tool_choice="auto",
            temperature=0.0,
        )

        message = response.choices[0].message
        tool_calls = message.tool_calls or []

        if not tool_calls:
            _record_task_state(context, "idle")
            assistant_text = message.content or ""
            print(f"agent> {assistant_text}")
            messages.append({"role": "assistant", "content": assistant_text})
            log_agent_message(context.profile, context.session_id, "assistant", assistant_text)
            return assistant_text

        messages.append(
            {
                "role": "assistant",
                "content": message.content or "",
                "tool_calls": [
                    {
                        "id": call.id,
                        "type": "function",
                        "function": {
                            "name": call.function.name,
                            "arguments": call.function.arguments,
                        },
                    }
                    for call in tool_calls
                ],
            }
        )

        for index, call in enumerate(tool_calls):
            try:
                tool_result = dispatch_tool_call(context, call)
            except EndCurrentTurn as exc:
                _append_turn_end_tool_results(messages, tool_calls, index, exc)
                assistant_text = str(exc)
                print(f"agent> {assistant_text}")
                log_agent_message(context.profile, context.session_id, "assistant", assistant_text)
                return assistant_text
            messages.append(
                {
                    "role": "tool",
                    "tool_call_id": call.id,
                    "content": json.dumps(tool_result, ensure_ascii=False),
                }
            )

    assistant_text = "本轮工具调用次数过多，已停止。"
    print(f"agent> {assistant_text}")
    log_agent_message(context.profile, context.session_id, "assistant", assistant_text)
    return assistant_text


def _record_task_state(context: ToolContext, phase: str) -> None:
    """更新当前阶段，并同步打印和落盘。"""
    if context.task_state is None:
        return
    if phase == "thinking":
        context.task_state.set_thinking()
    elif phase == "idle":
        context.task_state.set_idle()
    print(format_task_state_line(context.task_state))
    log_task_state(context.profile, context.session_id, context.task_state)


def _append_turn_end_tool_results(
    messages: list[dict[str, Any]],
    tool_calls: list[Any],
    stopped_index: int,
    exc: EndCurrentTurn,
) -> None:
    """补齐当前 assistant tool_calls 的 tool 响应，避免下一轮请求非法。"""
    result = exc.tool_result or {
        "success": False,
        "error": "TURN_ABORTED",
        "message": str(exc),
    }
    messages.append(_build_tool_message(tool_calls[stopped_index].id, result))

    skipped_result = {
        "success": False,
        "error": "SKIPPED_DUE_TO_TURN_END",
        "message": str(exc),
    }
    for pending_call in tool_calls[stopped_index + 1 :]:
        messages.append(_build_tool_message(pending_call.id, skipped_result))


def _build_tool_message(tool_call_id: str, result: dict[str, Any]) -> dict[str, Any]:
    """构造符合 OpenAI tool_call 协议的 tool message。"""
    return {
        "role": "tool",
        "tool_call_id": tool_call_id,
        "content": json.dumps(result, ensure_ascii=False),
    }
