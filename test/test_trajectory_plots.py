"""离线验证世界坐标轨迹、缺失数据、重复case和PNG输出。"""

import csv
import json
from pathlib import Path

import cv2
import numpy as np
import pytest
from matplotlib.figure import Figure

from drone_harness.testing import trajectory_plots as plots
from drone_harness.testing.batch_support import save_summary, write_json


def episode(index=0):
    """参考轨迹使用数据集的六维位姿，绘图只取前两维世界坐标。"""
    return {"episode_id": f"ep-{index}", "scene_id": 9 + index,
            "reference_path": [[0, 0, 0, 0, 0, 0], [10, 20, 8, 0, 0, 1]],
            "goals": [{"position": [10, 20, 8]}]}


def result(index=0, attempt=1, cycle=1):
    """提供已结算的指标和独立尝试身份。"""
    return {"case_index": index, "scene_id": 9 + index, "episode_id": f"ep-{index}",
            "attempt": attempt, "cycle": cycle, "NE": 5.0, "SR": 0, "OSR": 1,
            "steps_taken": 3, "status": "incomplete", "end_reason": "case_timeout"}


def write_attempt(root, row):
    """保存带高度变化的场景轨迹及原始评分文件。"""
    case_dir = root / f"{row['attempt']:05d}-r{row['cycle']}-case{row['case_index']}"
    write_json(case_dir / "result.json", row)
    write_json(case_dir / "scene/evaluation.json", {"goal_position_ned": [10, 20, 8],
        "final_position_ned": [13, 24, -99], "metrics": {"NE": 5.0},
        "diagnostics": {"minimum_distance_m": 3.0, "success_distance_m": 20.0}})
    (case_dir / "scene/trajectory.jsonl").write_text("\n".join(json.dumps({"position_ned": point})
        for point in [[0, 0, 0], [10, 17, 100], [13, 24, -99]]) + "\n")
    return case_dir


def test_world_xy_axes_six_dimensional_reference_and_score_are_preserved(tmp_path):
    """横轴为东、纵轴为北，高度不改变水平距离和20米圈。"""
    row = result()
    case_dir = write_attempt(tmp_path, row)
    data = plots.load_case_trajectory(case_dir, row, episode())
    assert data["NE"] == 5 and data["minimum"] == 3
    axis = Figure().subplots()
    plots._draw_trajectory(axis, data)
    assert axis.lines[0].get_xdata().tolist() == [0, 20]
    assert axis.lines[0].get_ydata().tolist() == [0, 10]
    assert axis.lines[1].get_xdata().tolist() == [0, 17, 24]
    assert axis.lines[1].get_ydata().tolist() == [0, 10, 13]
    assert axis.patches[0].center == (20, 10) and axis.patches[0].radius == 20
    assert "final 5.00m / min 3.00m" in axis.get_title()


def test_missing_path_or_trajectory_is_labelled_without_fake_motion(tmp_path):
    """启动失败与无参考路径仍出图，缺失实际轨迹不画成飞行。"""
    row = {**result(), "NE": None, "end_reason": "startup_timeout"}
    item = {**episode(), "reference_path": []}
    case_dir = tmp_path / "attempt"
    data = plots.load_case_trajectory(case_dir, row, item)
    axis = Figure().subplots()
    plots._draw_trajectory(axis, data)
    assert not axis.lines and data["minimum"] is None
    assert "final N/A / min N/A" in axis.get_title()
    assert {t.get_text() for t in axis.texts} == {"No recorded trajectory\nNo reference path"}
    path = plots.save_case_plot(case_dir, row, item)
    image = cv2.imread(str(path))
    assert image is not None and image.shape[0] > 500


def test_invalid_samples_break_the_line_and_minimum_is_horizontal(tmp_path):
    """损坏或非有限采样明确标注且断线，缺失统计时仅计算绘图距离。"""
    row = result()
    case_dir = write_attempt(tmp_path, row)
    (case_dir / "scene/evaluation.json").unlink()
    trajectory = case_dir / "scene/trajectory.jsonl"
    trajectory.write_text('{"position_ned":[10,17,100]}\ninvalid\nnull\n{"position_ned":[null,20,0]}\n{"position_ned":[13,24,0]}\n')
    data = plots.load_case_trajectory(case_dir, row, episode())
    assert data["invalid_samples"] == 3 and data["minimum"] == 3
    assert np.isnan(data["actual"][1:4]).all()
    assert np.array_equal(data["final"], [13, 24])


def test_mismatched_episode_is_rejected_before_using_the_wrong_reference(tmp_path):
    """数据集身份变化时不得把另一条参考路线画到当前case。"""
    with pytest.raises(ValueError, match="episode_id"):
        plots.load_case_trajectory(tmp_path, result(), episode(1))


def test_existing_batch_generates_each_attempt_and_one_overview_without_changing_scores(tmp_path, monkeypatch):
    """同一case多轮分别保存，补图命令只写图片且汇总能链接单图。"""
    items = [episode(0), episode(1)]
    rows = [result(index=i % 2, attempt=i + 1, cycle=i // 2 + 1) for i in range(5)]
    for row in rows:
        write_attempt(tmp_path, row)
    write_json(tmp_path / "dataset/val_unseen.json", {"episodes": items})
    write_json(tmp_path / "config.json", {"data_root": str(tmp_path / "dataset")})
    with (tmp_path / "results.csv").open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    before = {p: p.read_bytes() for p in tmp_path.rglob("*") if p.is_file()}
    titles = []
    original = plots._save_figure

    def save(figure, path):
        """同时检查实际保存的总览保留每条尝试的独立标题。"""
        if path.name == "trajectory_comparison.png":
            titles.extend(axis.get_title() for axis in figure.axes if axis.get_title())
        original(figure, path)

    monkeypatch.setattr(plots, "_save_figure", save)
    assert plots.main([str(tmp_path)]) == 0
    assert len(titles) == 5 and len(set(titles)) == 5
    assert len(list(tmp_path.glob("*/trajectory.png"))) == 5
    overview = cv2.imread(str(tmp_path / "trajectory_comparison.png"))
    assert overview is not None and overview.shape[1] > overview.shape[0]
    assert all(path.read_bytes() == content for path, content in before.items())
    save_summary(tmp_path, rows, 5, "completed")
    summary = (tmp_path / "summary.md").read_text()
    assert "[轨迹总览](trajectory_comparison.png)" in summary
    assert "00005-r3-case0/trajectory.png" in summary


def test_empty_batch_has_no_invented_overview(tmp_path):
    """尚未结算任何case时不生成代表实际尝试的图。"""
    assert plots.save_batch_plot(tmp_path, [], []) is None
    assert not list(tmp_path.glob("*.png"))
