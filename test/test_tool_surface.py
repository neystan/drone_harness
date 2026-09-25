"""验证 S2 中间态只暴露四动作，且前进严格拒绝执行。"""

from types import SimpleNamespace
from unittest.mock import Mock

from drone_harness.tools.registry import get_tool_definition, get_tool_definitions
from drone_harness.tools.schemas import get_tool_schemas


def test_model_tool_surface_is_exactly_four_actions() -> None:
    """旧状态、视觉和任意位移工具均不可再被模型调用。"""
    expected = {"takeoff", "forward", "rotate", "land"}
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


def test_forward_placeholder_issues_no_controller_command() -> None:
    """深度门尚未完成时，前进占位动作必须零飞控调用。"""
    controller = Mock()
    context = SimpleNamespace(controller=controller)
    result = get_tool_definition("forward").handler(context, {"distance_m": 0.2})
    assert result["success"] is False
    assert result["error"] == "FEATURE_NOT_READY"
    controller.assert_not_called()
    assert controller.method_calls == []


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
