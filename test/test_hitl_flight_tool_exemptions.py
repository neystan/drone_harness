"""Tests for per-profile human-in-the-loop flight-tool exemptions."""

from types import SimpleNamespace

from drone_agent.config.schema import SafetyConfig
from drone_agent.runtime.safety import requires_human_in_the_loop


def _profile(exempt_tools: frozenset[str]) -> SimpleNamespace:
    return SimpleNamespace(
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


def test_rotate_and_land_can_be_exempt_from_human_confirmation() -> None:
    profile = _profile(frozenset({"rotate", "land"}))

    assert not requires_human_in_the_loop(profile, "rotate")
    assert not requires_human_in_the_loop(profile, "land")
    assert requires_human_in_the_loop(profile, "takeoff")
    assert requires_human_in_the_loop(profile, "move")


def test_empty_exemption_list_keeps_flight_tools_confirmed() -> None:
    profile = _profile(frozenset())

    assert requires_human_in_the_loop(profile, "rotate")
    assert requires_human_in_the_loop(profile, "land")
