"""验证模型只能调用当前公开的飞行工具。"""

from types import SimpleNamespace
from unittest.mock import Mock

from drone_harness.tools.registry import get_tool_definition, get_tool_definitions
from drone_harness.tools.schemas import get_tool_schemas


def test_model_tool_surface_is_observe_plus_six_actions() -> None:
    """只新增按需观察，不恢复旧状态、视觉或任意位移工具。"""
    expected = {"observe", "takeoff", "forward", "up", "down", "rotate", "land"}
    assert {tool.name for tool in get_tool_definitions()} == expected
    assert {schema["function"]["name"] for schema in get_tool_schemas()} == expected
    for removed in (
        "move",
        "hover",
        "return_home",
        "take_photo",
        "analyze_view",
        "detect_target",
        "sam_tracking",
        "mouse_tracking",
        "activate_skill",
    ):
        assert get_tool_definition(removed) is None


def test_forward_without_observation_still_needs_new_depth(tmp_path) -> None:
    """无 observe 时若本次取不到深度，前进仍为零飞控命令。"""
    from test_agent_observation_loop import context_for

    controller = SimpleNamespace(
        wait_for_observation=Mock(return_value=None),
        vehicle_status=SimpleNamespace(mode="OFFBOARD", connected=True, armed=True),
        flight_state=lambda: "IN_AIR",
    )
    context = context_for(tmp_path, controller)
    result = get_tool_definition("forward").handler(context, {"distance_m": 0.2})
    assert result["success"] and result["commanded_distance_m"] == 0
    controller.wait_for_observation.assert_called_once()


def test_old_package_and_entry_names_are_not_packaged() -> None:
    """包元数据与脚本入口均只采用 drone_harness 命名。"""
    from pathlib import Path

    root = Path(__file__).resolve().parents[1]
    assert (root / "drone_harness" / "cli.py").is_file()
    assert not (root / "drone_agent").exists()
    assert (root / "scripts" / "drone_harness_sim").is_file()
    assert (root / "scripts" / "drone_harness_real").is_file()
    assert not (root / "scripts" / "drone_agent_sim").exists()
    assert not (root / "scripts" / "drone_agent_real").exists()
