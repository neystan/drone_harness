"""执行受控四向扫描并临时向主模型提供四图。"""

import json
import time
from types import SimpleNamespace
from typing import Any

from drone_harness.logging.task_log import log_observation, save_observation_image
from drone_harness.runtime.observation import encode_observation_image
from drone_harness.runtime.reflection import clear_target_assessment
from drone_harness.runtime.target_measurement import clear_target_measurement
from drone_harness.skills import load_skills


SEARCH_MARKER = "【四向搜索观察】"
SEARCH_DIRECTIONS = ("前", "左", "后", "右")


def activate_skill(context: Any, arguments: dict[str, Any]) -> dict[str, Any]:
    """按需加载 skill 正文并执行内置固定流程，保留逐动作审批。"""
    context.search_observations = None
    name, prompt = arguments.get("name"), arguments.get("prompt")
    skill = next((item for item in load_skills() if item.name == name), None)
    if skill is None or name != "four-direction-search":
        return {"success": False, "error": "SKILL_NOT_FOUND", "message": "未找到可执行的内置 skill。"}
    if not skill.enabled or context.profile.name not in skill.modes:
        return {"success": False, "error": "SKILL_UNAVAILABLE", "message": "当前模式未启用该 skill。"}
    if not isinstance(prompt, str) or not 1 <= len(prompt.strip()) <= 1000:
        return {"success": False, "error": "INVALID_SEARCH_PROMPT", "message": "搜索 prompt 须为 1 到 1000 字符的观察重点。"}
    print(f"skill> {skill.name}：{prompt.strip()}", flush=True)
    result = _scan_four_directions(context, prompt.strip())
    return {**result, "skill_name": skill.name, "skill_content": skill.body, "prompt": prompt.strip()}


def _call_tool(context: Any, name: str, arguments: dict[str, Any]) -> dict[str, Any]:
    """内部动作复用正式分发器的中断、人工审批和工具日志。"""
    from drone_harness.runtime.tool_dispatcher import dispatch_tool_call

    return dispatch_tool_call(context, SimpleNamespace(id=f"skill-{name}", function=SimpleNamespace(
        name=name, arguments=json.dumps(arguments, ensure_ascii=False))))


def _clear_observation(context: Any) -> None:
    """动作及扫描结束后清除当前位置的观察和测距缓存。"""
    context.observation = None
    context.depth_rules = None
    clear_target_assessment(context)
    clear_target_measurement(context)
    if context.task_state is not None:
        context.task_state.clear_observation()


def _scan_four_directions(context: Any, prompt: str) -> dict[str, Any]:
    """按固定顺序采四向新图，四次转向后才交还模型。"""
    snapshots = []
    turns = 0
    try:
        for direction in SEARCH_DIRECTIONS:
            observed = _call_tool(context, "observe", {"prompt": prompt, "view": "front"})
            if not observed.get("success") or context.observation is None:
                return {"success": False, "error": "SCAN_CAPTURE_FAILED", "completed_turns": turns,
                        "scan_completed": False, "heading_restored": False,
                        "message": f"{direction}方采图失败，扫描停止：{observed.get('message', '')}"}
            snapshots.append(context.observation)
            rotated = _call_tool(context, "rotate", {"direction": "left", "degrees": 90})
            _clear_observation(context)
            if not rotated.get("success"):
                return {**rotated, "scan_completed": False, "heading_restored": False,
                        "completed_turns": turns}
            turns += 1
            time.sleep(2.0)
        # 最后一次等待也响应介入，不能继续把扫描结果交给模型。
        from drone_harness.bus.intervention import interrupt_if_requested
        from drone_harness.runtime.safety import EndCurrentTurn

        interrupted = interrupt_if_requested(context, hover_on_flight_tool=True)
        if interrupted is not None:
            raise EndCurrentTurn(interrupted["message"], interrupted)
        context.search_observations = snapshots
        if context.task_state is not None:
            context.task_state.record_motion_progress("rotate", None, None, rotation_degrees=360)
        return {"success": True, "scan_completed": True, "heading_restored": True,
                "image_count": 4, "message": "四向扫描完成并恢复开始朝向，请结合本次搜索重点联合判断四图。"}
    finally:
        _clear_observation(context)


def build_search_message(context: Any, prompt: str) -> dict[str, Any]:
    """组装带方向说明和动态问题的四图消息，不提供当前深度。"""
    snapshots = context.search_observations
    context.search_observations = None
    if snapshots is None or len(snapshots) != 4:
        raise ValueError("完整四向观察缺失")
    content = [{"type": "text", "text": SEARCH_MARKER +
                "四图均为前视相机在原地旋转时拍摄，方向相对扫描开始朝向；当前已恢复开始朝向。"
                "图像只供本次联合判断，不是当前位置的新深度。\n本次搜索重点：" + prompt}]
    for direction, snapshot in zip(SEARCH_DIRECTIONS, snapshots):
        parts = [{"type": "text", "text": f"【原朝向{direction}方】"}, encode_observation_image(snapshot.rgb)]
        content.extend(parts)
        saved = save_observation_image(context.profile, context.session_id, snapshot.observation_id,
                                       {"role": "user", "content": parts})
        log_observation(context.profile, context.session_id, snapshot.observation_id,
                        snapshot.rgb_stamp_ns, snapshot.depth_stamp_ns, False, 0.0,
                        "SEARCH_HISTORY", saved, view=f"scan_{direction}")
    return {"role": "user", "content": content}


def clear_search_images(messages: list[dict[str, Any]]) -> None:
    """联合判断响应后移除四图，保留文字和普通观察图片。"""
    for index, message in enumerate(messages):
        content = message.get("content")
        if (isinstance(content, list) and content and content[0].get("type") == "text"
                and content[0].get("text", "").startswith(SEARCH_MARKER)):
            messages[index] = {**message, "content": [part for part in content if part.get("type") != "image_url"]}
