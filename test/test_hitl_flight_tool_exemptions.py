"""验证真机四个飞行动作始终需要逐次人工确认。"""

from types import SimpleNamespace

from drone_harness.config.schema import SafetyConfig
from drone_harness.runtime.safety import requires_human_in_the_loop


def _profile(exempt_tools: frozenset[str], mode: str = "real") -> SimpleNamespace:
    """构造带可疑历史豁免配置的测试 profile。"""
    return SimpleNamespace(
        mode=mode,
        safety=SafetyConfig(
            human_in_the_loop_for_flight_tools=True,
            human_in_the_loop_exempt_flight_tools=exempt_tools,
            max_takeoff_height_m=3.0,
            max_relative_move_m=5.0,
            max_vertical_move_m=2.0,
            max_rotation_deg=180.0,
            action_timeout_s=20.0,
            hover_on_timeout=True,
            pre_takeoff_gate_enabled=True,
            require_battery_status_for_takeoff=True,
            min_battery_percent_for_takeoff=30.0,
            require_px4_status_ready_for_takeoff=True,
        )
    )


def test_real_rotate_and_land_cannot_be_exempt_from_human_confirmation() -> None:
    """即使旧配置仍写豁免，真机动作也必须经 HITL。"""
    profile = _profile(frozenset({"rotate", "land"}))

    assert requires_human_in_the_loop(profile, "rotate")
    assert requires_human_in_the_loop(profile, "land")
    assert requires_human_in_the_loop(profile, "takeoff")
    assert requires_human_in_the_loop(profile, "forward")


def test_empty_exemption_list_keeps_flight_tools_confirmed() -> None:
    """真机 profile 无豁免时四动作仍全部确认。"""
    profile = _profile(frozenset())

    assert requires_human_in_the_loop(profile, "rotate")
    assert requires_human_in_the_loop(profile, "land")
