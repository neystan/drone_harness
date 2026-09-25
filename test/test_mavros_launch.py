from __future__ import annotations

from pathlib import Path


LAUNCH_FILE = Path(__file__).parents[1] / "launch" / "takeoff_camera.launch.py"


def test_simulation_launch_includes_mavros_and_fcu_url() -> None:
    """验证仿真 launch 不负责启动 MAVROS 节点。"""
    source = LAUNCH_FILE.read_text(encoding="utf-8")

    assert "mavros_launch" not in source
    assert "px4.launch" not in source
    assert "fcu_url" not in source
    assert "airsim_node.launch.py" in source
    assert "MicroXRCEAgent" not in source
