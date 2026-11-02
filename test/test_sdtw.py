"""验证官方三维公式、动作筛选及历史补算，不启动仿真。"""

import json
from datetime import datetime, timedelta, timezone

import numpy as np
import pytest
from fastdtw import fastdtw

from drone_harness.testing.backfill_sdtw import backfill_batch, reconstruct_actions
from drone_harness.testing.batch_support import save_summary, write_json
from drone_harness.testing.sdtw import calculate_sdtw, is_motion_event


def test_matches_official_formula_with_original_reference_and_xyz():
    """与官方欧氏距离回调逐项对照，参考旋转重复点保留且高度参与评分。"""
    actual = [[0, 0, 0], [0, 0, 0], [5, 0, 2], [10, 0, 2]]
    reference = [[0, 0, 0, 0, 0, 0], [5, 0, 0, 0, 0, 0], [5, 0, 0, 0, 0, 1], [10, 0, 0, 0, 0, 1]]
    distance, _ = fastdtw(np.asarray(actual)[[0, 2, 3]], np.asarray(reference)[:, :3],
                         dist=lambda a, b: np.linalg.norm(np.array(b) - np.array(a), ord=2))
    score = calculate_sdtw(actual, reference, 1)
    assert score["dtw_distance_m"] == pytest.approx(distance)
    assert score["SDTW"] == pytest.approx(np.exp(-distance / (4 * 20)))
    assert score["actual_point_count"] == 3 and score["reference_point_count"] == 4
    assert score["SDTW"] < 1
    assert calculate_sdtw(reference, reference, 1)["SDTW"] == 1
    assert calculate_sdtw(actual, reference, 0)["SDTW"] == 0


@pytest.mark.parametrize("actual,reference", [([], [[0, 0, 0]]), ([[0, 0, 0]], []),
    ([[0, 0, float("nan")]], [[0, 0, 0]]), ([[0, 0, 0]], [[0, 1]]),
    ([[0, 0, 0]], [[0, 0, float("inf")]])])
def test_missing_trajectory_is_not_fabricated_as_zero(actual, reference):
    """缺失和损坏坐标即使任务失败也保持未知。"""
    assert calculate_sdtw(actual, reference, 0)["SDTW"] is None


def test_motion_filter_includes_partial_failure_and_excludes_non_motion():
    """部分移动后失败保留，拒绝、零旋转和只读工具不产生动作点。"""
    event = {"event_type": "tool_call", "tool_name": "forward", "arguments": {}, "result": {"success": True}}
    assert is_motion_event(event)
    event["result"] = {"success": False, "final_position_ned": [1, 0, 0]}
    assert is_motion_event(event)
    event["result"]["motion_executed"] = False
    assert not is_motion_event(event)
    for name in ("observe", "measure_distance", "get_state", "update_navigation_plan", "activate_skill"):
        assert not is_motion_event({**event, "tool_name": name, "result": {"success": True}})
    assert not is_motion_event({**event, "tool_name": "rotate", "arguments": {"degrees": 0}, "result": {"success": True}})


def historical_case(root):
    """构造有世界坐标和相对工具坐标差异的历史成功样本。"""
    run = root / "00001-r1-case0"
    session = run / "agent_logs/session_old"
    session.mkdir(parents=True)
    (run / "scene").mkdir()
    start = datetime(2026, 11, 1, tzinfo=timezone.utc)
    samples = [{"timestamp": (start + timedelta(seconds=i / 5)).isoformat(), "position_ned": [100 + min(i, 5), 50, -2]}
               for i in range(11)]
    (run / "scene/trajectory.jsonl").write_text("".join(json.dumps(item) + "\n" for item in samples))
    events = [{"event_type": "tool_call", "timestamp": "2026-11-01 08:00:01", "tool_name": "forward",
               "arguments": {}, "result": {"success": True, "final_position_ned": [5, 0, 0]}}]
    (session / "tool_calls.jsonl").write_text("".join(json.dumps(item) + "\n" for item in events))
    report = {"metrics": {"NE": 0, "SR": 1, "OSR": 1, "steps_taken": 1}, "final_position_ned": [105, 50, -2]}
    write_json(run / "scene/evaluation.json", report)
    row = {"case_index": 0, "episode_id": "episode-0", "scene_id": 9,
           "NE": 0, "SR": 1, "OSR": 1, "steps_taken": 1}
    write_json(run / "result.json", row)
    write_json(root / "config.json", {"data_root": str(root / "data")})
    write_json(root / "data/val_unseen.json", {"episodes": [{"episode_id": "episode-0", "scene_id": 9,
               "reference_path": [[100, 50, -2], [105, 50, -2]]}]})
    save_summary(root, [{**row, "SDTW": None, "attempt": 1, "cycle": 1}], 1, "completed")
    return run, report


def test_history_backfill_preserves_sources_and_only_changes_requested_batch(tmp_path):
    """补算使用世界坐标，替换自己的汇总并保留原始报告及旧指标证据。"""
    root = tmp_path / "batch"
    run, report = historical_case(root)
    original = (run / "scene/evaluation.json").read_bytes()
    other = tmp_path / "other/result.json"
    write_json(other, {"steps_taken": 99})
    records, alignment = reconstruct_actions(run, report)
    assert records[1]["position_ned"] == [105, 50, -2]
    assert alignment["coarse_timestamp_uncertainty_s"] == 0.5
    summary = backfill_batch(root)
    assert summary["overall"]["SDTW"] == 1
    assert summary["overall"]["SDTW_missing_count"] == 0
    assert "steps_taken" not in json.loads((run / "result.json").read_text())
    assert "steps_taken" not in (root / "results.csv").read_text()
    assert (run / "scene/evaluation.json").read_bytes() == original
    assert json.loads(other.read_text()) == {"steps_taken": 99}
    assert json.loads((root / "sdtw_backfill.json").read_text())["runs"][0]["previous_metrics"]["steps_taken"] == 1
    with pytest.raises(ValueError, match="已补算"):
        backfill_batch(root)


def test_backfill_identity_mismatch_fails_before_writes(tmp_path):
    """身份不符时不更新结果，避免把其他case的参考路径用于评分。"""
    run, _ = historical_case(tmp_path)
    row = json.loads((run / "result.json").read_text())
    row["episode_id"] = "wrong"
    write_json(run / "result.json", row)
    original = (run / "result.json").read_bytes()
    with pytest.raises(ValueError, match="身份"):
        backfill_batch(tmp_path)
    assert (run / "result.json").read_bytes() == original
    assert not (run / "sdtw.json").exists()


def test_history_clock_regression_preserves_acquisition_order(tmp_path):
    """历史墙钟小幅回拨只记录证据，不对真实位置序列重新排序。"""
    run, report = historical_case(tmp_path)
    path = run / "scene/trajectory.jsonl"
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    rows[7]["timestamp"] = rows[3]["timestamp"]
    path.write_text("".join(json.dumps(row) + "\n" for row in rows))
    records, evidence = reconstruct_actions(run, report)
    assert records[1]["position_ned"] == [105, 50, -2]
    assert evidence["pose_clock_regression_count"] == 1
    assert evidence["maximum_pose_clock_regression_s"] > 0
