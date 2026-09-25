"""视觉模型调用与结果归一化。"""

from __future__ import annotations

import base64
import json
from pathlib import Path
from typing import Any

from drone_agent.config.schema import RuntimeProfile
from drone_agent.vision.prompts import VLM_SYSTEM_PROMPT


ANALYZE_VIEW_ACTIONS = {
    "rotate_right_search",
    "rotate_left_search",
    "move_right",
    "move_left",
    "move_forward",
    "take_photo",
    "hold_position",
}
ANALYZE_VIEW_POSITIONS = {"left", "right", "center", "upper", "lower", "none"}


def create_vlm_client(profile: RuntimeProfile) -> Any:
    """根据 profile 创建视觉模型client。"""
    from openai import OpenAI

    return OpenAI(
        api_key=profile.vlm.api_key,
        base_url=profile.vlm.base_url,
    )


def encode_image_to_data_url(image_path: Path) -> str:
    """把本地图像编码成 data URL。"""
    image_bytes = image_path.read_bytes()
    encoded = base64.b64encode(image_bytes).decode("ascii")
    return f"data:image/png;base64,{encoded}"


def build_analyze_view_prompt(target_description: str | None) -> str:
    """构造视觉分析提示词。"""
    if not target_description:
        return (
            "你正在分析无人机前视相机画面。"
            "请描述当前画面中的主要内容，并判断前方是否存在明显障碍。"
            "请严格返回 JSON 对象，必须包含以下字段："
            "scene_description, target_description, found_target, target_position, "
            "center_offset_x, center_offset_y, confidence。"
            "如果没有指定目标，则 target_description 返回 null，"
            "found_target 返回 null，target_position 返回 null，"
            "center_offset_x 返回 null，center_offset_y 返回 null，confidence 返回 null。"
        )

    return (
        "你正在分析无人机前视相机画面。"
        f"目标是：{target_description}。"
        "请判断当前画面中是否存在该目标。"
        "如果存在，请判断它位于画面的 left/right/center/upper/lower。"
        "请给出目标相对画面中心的水平和垂直偏移，归一化到 [-1, 1]。"
        "请给出识别置信度。"
        "请严格返回 JSON 对象，必须包含以下字段："
        "scene_description, target_description, found_target, target_position, "
        "center_offset_x, center_offset_y, confidence。"
    )


def extract_json_object(text: str) -> dict:
    """从模型输出中提取 JSON 对象。"""
    text = text.strip()
    if not text:
        raise ValueError("empty VLM response")
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        start = text.find("{")
        end = text.rfind("}")
        if start == -1 or end == -1 or end <= start:
            raise ValueError("VLM response does not contain a JSON object")
        return json.loads(text[start : end + 1])


def normalize_offset(value: Any) -> float | None:
    """把偏移量归一化到 [-1, 1]。"""
    if value is None:
        return None
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return max(-1.0, min(1.0, value))


def normalize_confidence(value: Any) -> float | None:
    """把置信度归一化到 [0, 1]。"""
    if value is None:
        return None
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return max(0.0, min(1.0, value))


def derive_suggested_action(
    target_description: str | None,
    found_target: bool | None,
    target_position: str | None,
    center_offset_x: float | None,
) -> str:
    """根据视觉结果推导下一步建议动作。"""
    if not target_description:
        return "hold_position"
    if not found_target:
        return "rotate_left_search"
    if target_position == "left":
        return "rotate_left_search"
    if target_position == "right":
        return "rotate_right_search"
    if target_position == "center":
        if center_offset_x is None or abs(center_offset_x) <= 0.12:
            return "take_photo"
        if center_offset_x < 0:
            return "rotate_left_search"
        return "rotate_right_search"
    return "hold_position"


def normalize_vlm_result(
    raw_result: dict,
    target_description: str | None,
    image_path: Path,
) -> dict:
    """把原始视觉模型结果整理成稳定结构。"""
    scene_description = str(raw_result.get("scene_description", "")).strip()
    position = raw_result.get("target_position")
    if position is None:
        normalized_position = None if not target_description else "none"
    else:
        normalized_position = str(position).strip().lower()
        if normalized_position not in ANALYZE_VIEW_POSITIONS:
            normalized_position = None if not target_description else "none"

    normalized_center_offset_x = normalize_offset(raw_result.get("center_offset_x"))
    normalized_center_offset_y = normalize_offset(raw_result.get("center_offset_y"))
    normalized_confidence = normalize_confidence(raw_result.get("confidence"))

    found_target = raw_result.get("found_target")
    if target_description:
        found_target = bool(found_target)
    else:
        found_target = None
        normalized_position = None

    normalized_action = derive_suggested_action(
        target_description,
        found_target,
        normalized_position,
        normalized_center_offset_x,
    )
    if normalized_action not in ANALYZE_VIEW_ACTIONS:
        normalized_action = "hold_position"

    return {
        "success": True,
        "image_path": str(image_path),
        "scene_description": scene_description,
        "target_description": target_description or None,
        "found_target": found_target,
        "target_position": normalized_position,
        "center_offset_x": normalized_center_offset_x,
        "center_offset_y": normalized_center_offset_y,
        "confidence": normalized_confidence,
        "suggested_action": normalized_action,
    }


def call_vlm(profile: RuntimeProfile, image_path: Path, prompt: str) -> dict:
    """调用视觉模型并解析返回 JSON。"""
    client = create_vlm_client(profile)
    image_data_url = encode_image_to_data_url(image_path)
    response = client.chat.completions.create(
        model=profile.vlm.model,
        temperature=0.0,
        messages=[
            {
                "role": "system",
                "content": VLM_SYSTEM_PROMPT,
            },
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": prompt},
                    {"type": "image_url", "image_url": {"url": image_data_url}},
                ],
            },
        ],
    )
    content = response.choices[0].message.content or ""
    return extract_json_object(content)


def analyze_image(
    profile: RuntimeProfile,
    image_path: Path,
    target_description: str | None,
) -> dict:
    """对指定图片执行一次完整的视觉分析。"""
    prompt = build_analyze_view_prompt(target_description)
    raw_result = call_vlm(profile, image_path, prompt)
    return normalize_vlm_result(raw_result, target_description, image_path)
