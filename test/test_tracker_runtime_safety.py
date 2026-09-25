import json
from pathlib import Path
from types import SimpleNamespace

from drone_agent.config.loader import load_profile
from drone_agent.tools import tracking as tracking_tools
from drone_agent.vision import tracking as vision_tracking
from drone_agent.vision.sam2_client import Sam2ClientError


def _write_settings(settings_path: Path, *, tracker_enabled: bool, detector_enabled: bool = False) -> None:
    """写入测试用的模型、检测器和追踪器配置。"""
    settings = {
        "llm": {
            "api_key": "llm-key",
            "base_url": "http://llm.invalid",
            "model": "llm-model",
        },
        "vlm": {"enabled": False},
        "detector": {
            "enabled": detector_enabled,
            "provider": "dinoxseek",
            "api_key": "detector-key",
            "model": "detector-model",
            "api_path": "/detect",
        },
        "tracker": {
            "enabled": tracker_enabled,
            "base_url": "http://sam2.invalid",
            "timeout_s": 0.1,
        },
    }
    settings_path.write_text(json.dumps(settings), encoding="utf-8")


def _load_profile(tmp_path: Path, *, tracker_enabled: bool, detector_enabled: bool = False):
    """加载测试用仿真 profile。"""
    settings_path = tmp_path / "settings.json"
    _write_settings(
        settings_path,
        tracker_enabled=tracker_enabled,
        detector_enabled=detector_enabled,
    )
    profile_dir = Path(__file__).resolve().parents[1] / "drone_agent" / "config" / "profiles"
    return load_profile("sim", profile_dir=profile_dir, settings_path=settings_path)


def _context(profile):
    """构造不依赖 ROS executor 的追踪工具上下文。"""
    controller = SimpleNamespace(
        latest_rgb_frame=SimpleNamespace(shape=(480, 640, 3)),
    )
    return SimpleNamespace(profile=profile, controller=controller)


def test_profile_restores_tracker_configuration(tmp_path):
    """加载 profile 后应保留 tracker 配置对象。"""
    profile = _load_profile(tmp_path, tracker_enabled=True)

    assert profile.tracker.enabled is True
    assert profile.tracker.base_url == "http://sam2.invalid"
    assert profile.tracker.timeout_s == 0.1


def test_disabled_tracker_returns_without_service_access(tmp_path):
    """禁用 tracker 时应返回结构化失败而不是访问缺失服务。"""
    profile = _load_profile(tmp_path, tracker_enabled=False)

    result = tracking_tools.sam_tracking(_context(profile), {"action": "start"})

    assert result["success"] is False
    assert result["error"] == "TRACKER_DISABLED"


def test_tracker_service_errors_do_not_escape_tool(tmp_path, monkeypatch):
    """SAM2 start 和 stop 服务异常应留在工具结果内。"""
    profile = _load_profile(tmp_path, tracker_enabled=True, detector_enabled=True)
    context = _context(profile)
    monkeypatch.setattr(
        tracking_tools,
        "save_analysis_frame",
        lambda _frame, _directory: tmp_path / "frame.png",
    )
    monkeypatch.setattr(
        tracking_tools,
        "detect_image_targets",
        lambda *_arguments: {
            "objects": [{"index": 0, "bbox_xyxy_px": [0, 0, 10, 10]}]
        },
    )

    class UnavailableClient:
        """模拟未启动的 SAM2 服务。"""

        def start(self, _payload):
            """模拟 start 请求失败。"""
            raise Sam2ClientError("connection refused")

        def stop(self, _payload):
            """模拟 stop 请求失败。"""
            raise Sam2ClientError("connection refused")

    monkeypatch.setattr(vision_tracking, "_client", lambda _profile: UnavailableClient())
    start_result = tracking_tools.sam_tracking(
        context,
        {"action": "start", "target_description": "target"},
    )

    vision_tracking._CURRENT_SESSION = vision_tracking.TrackingSession(
        track_id="track-1",
        target_description="target",
        target_index=0,
        selection_method="dinoxseek",
    )
    stop_result = tracking_tools.sam_tracking(context, {"action": "stop"})
    vision_tracking._CURRENT_SESSION = None

    assert start_result["success"] is False
    assert start_result["error"] == "SAM_TRACKING_START_FAILED"
    assert stop_result["success"] is False
    assert stop_result["error"] == "SAM2_SERVICE_UNAVAILABLE"
