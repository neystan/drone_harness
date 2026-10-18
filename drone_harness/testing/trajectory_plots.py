"""为已结算的case和批次绘制水平轨迹，独立于导航与评分。"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure
from matplotlib.lines import Line2D
from matplotlib.patches import Circle


def _xy(value) -> np.ndarray:
    """提取有限的世界XY坐标，无效值保留为断线点。"""
    try:
        point = np.asarray(value, dtype=float)
        if point.ndim == 1 and len(point) >= 2 and np.isfinite(point[:2]).all():
            return point[:2]
    except (TypeError, ValueError):
        pass
    return np.array([np.nan, np.nan])


def load_case_trajectory(case_dir: Path, row: dict, episode: dict) -> dict:
    """只读评测世界坐标、参考路径和评分结果，不改写成绩。"""
    if str(episode["episode_id"]) != str(row["episode_id"]):
        raise ValueError(f"case {row['case_index']} 的episode_id与数据集不一致")
    report_path = case_dir / "scene/evaluation.json"
    report = json.loads(report_path.read_text(encoding="utf-8")) if report_path.is_file() else {}
    points = []
    trajectory = case_dir / "scene/trajectory.jsonl"
    if trajectory.is_file():
        with trajectory.open(encoding="utf-8") as handle:
            for line in handle:
                try:
                    item = json.loads(line)
                    points.append(_xy(item.get("position_ned") if isinstance(item, dict) else None))
                except json.JSONDecodeError:
                    points.append(_xy(None))
    actual = np.asarray(points).reshape(-1, 2)
    valid = actual[np.isfinite(actual).all(axis=1)]
    reference = np.asarray([_xy(point) for point in episode.get("reference_path", []) or []]).reshape(-1, 2)
    goals = episode.get("goals", [])
    goal = _xy(report.get("goal_position_ned", goals[0].get("position") if goals else None))
    final = _xy(report.get("final_position_ned", valid[-1] if len(valid) else None))
    diagnostics = report.get("diagnostics", {})
    minimum = diagnostics.get("minimum_distance_m")
    if minimum is None and len(valid) and np.isfinite(goal).all():
        minimum = float(np.linalg.norm(valid - goal, axis=1).min())
    return {"row": row, "reference": reference, "actual": actual, "valid": valid,
            "goal": goal, "final": final, "NE": report.get("metrics", {}).get("NE", row.get("NE")),
            "minimum": minimum, "radius": diagnostics.get("success_distance_m", 20.0),
            "invalid_samples": len(actual) - len(valid)}


def _distance_label(value) -> str:
    """缺失距离显示N/A，避免把没有测量写成零米。"""
    try:
        return f"{float(value):.2f}m" if math.isfinite(float(value)) else "N/A"
    except (TypeError, ValueError):
        return "N/A"


def _legend_handles() -> list:
    """统一单图和总览的参考线、实际线及起终点符号。"""
    return [Line2D([], [], color="grey", linestyle="--", label="Reference"),
            Line2D([], [], color="royalblue", label="Actual"),
            Line2D([], [], color="royalblue", marker="o", linestyle="none", label="Start"),
            Line2D([], [], color="crimson", marker="x", linestyle="none", label="Final"),
            Line2D([], [], color="seagreen", marker="*", linestyle="none", label="Goal")]


def _draw_trajectory(axis, data: dict) -> None:
    """以东为横轴、北为纵轴，按相同米制比例绘制真实轨迹。"""
    row = data["row"]
    reference, actual, valid = data["reference"], data["actual"], data["valid"]
    if len(reference):
        axis.plot(reference[:, 1], reference[:, 0], "--", color="grey", linewidth=1.5)
    if len(actual):
        axis.plot(actual[:, 1], actual[:, 0], color="royalblue", linewidth=1.5)
    if len(valid):
        axis.scatter(valid[0, 1], valid[0, 0], marker="o", color="royalblue", zorder=4)
    if np.isfinite(data["final"]).all():
        axis.scatter(data["final"][1], data["final"][0], marker="x", color="crimson", zorder=5)
    if np.isfinite(data["goal"]).all():
        north, east = data["goal"]
        axis.scatter(east, north, marker="*", color="seagreen", zorder=4)
        axis.add_patch(Circle((east, north), data["radius"], color="seagreen", alpha=.10))
    notes = []
    if not len(valid):
        notes.append("No recorded trajectory")
    if not np.isfinite(reference).any():
        notes.append("No reference path")
    if data["invalid_samples"]:
        notes.append(f"Invalid samples: {data['invalid_samples']}")
    if notes:
        axis.text(.02, .98, "\n".join(notes), transform=axis.transAxes, va="top", fontsize=9)
    identity = f"Case {row['case_index']} / scene {row['scene_id']} / cycle {row['cycle']} / attempt {row['attempt']}"
    axis.set_title(identity + f"\nfinal {_distance_label(data['NE'])} / min {_distance_label(data['minimum'])}", fontsize=11)
    axis.set_xlabel("World East (m)")
    axis.set_ylabel("World North (m)")
    axis.set_aspect("equal", adjustable="datalim")
    axis.grid(alpha=.2)


def _save_figure(figure: Figure, path: Path) -> None:
    """使用无窗口渲染和原子替换保存PNG，释放绘图对象。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    try:
        FigureCanvasAgg(figure)
        figure.savefig(temporary, format="png", dpi=140)
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)
        figure.clear()


def save_case_plot(case_dir: Path, row: dict, episode: dict) -> Path:
    """每次尝试保存独立轨迹图，包括失败、超时和未飞行样本。"""
    figure = Figure(figsize=(7, 6), layout="constrained")
    axis = figure.subplots()
    _draw_trajectory(axis, load_case_trajectory(case_dir, row, episode))
    axis.legend(handles=_legend_handles(), loc="best", fontsize=9)
    path = case_dir / "trajectory.png"
    _save_figure(figure, path)
    return path


def save_batch_plot(root: Path, rows: list[dict], episodes: list[dict]) -> Path | None:
    """按尝试顺序更新一张批次总览，每条尝试独立显示且不混合场景。"""
    if not rows:
        return None
    columns = min(3, len(rows)) if len(rows) <= 9 else math.ceil(math.sqrt(len(rows)))
    height = math.ceil(len(rows) / columns)
    figure = Figure(figsize=(5 * columns, 4.8 * height))
    axes = figure.subplots(height, columns, squeeze=False).ravel()
    for axis, row in zip(axes, rows):
        case_dir = root / f"{int(row['attempt']):05d}-r{row['cycle']}-case{row['case_index']}"
        _draw_trajectory(axis, load_case_trajectory(case_dir, row, episodes[int(row["case_index"])]))
    for axis in axes[len(rows):]:
        axis.set_axis_off()
    figure.legend(handles=_legend_handles(), loc="lower center", ncol=5)
    figure.tight_layout(rect=(0, .06 if height < 3 else .025, 1, 1))
    path = root / "trajectory_comparison.png"
    _save_figure(figure, path)
    return path


def main(argv: list[str] | None = None) -> int:
    """仅为已有批次补画单图和总览，不启动仿真或重算成绩。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("batch_dir", type=Path)
    args = parser.parse_args(argv)
    root = args.batch_dir
    config = json.loads((root / "config.json").read_text(encoding="utf-8"))
    episodes = json.loads((Path(config["data_root"]) / "val_unseen.json").read_text(encoding="utf-8"))["episodes"]
    with (root / "results.csv").open(encoding="utf-8-sig", newline="") as handle:
        rows = list(csv.DictReader(handle))
    for row in rows:
        case_dir = root / f"{int(row['attempt']):05d}-r{row['cycle']}-case{row['case_index']}"
        save_case_plot(case_dir, row, episodes[int(row["case_index"])])
    save_batch_plot(root, rows, episodes)
    print(f"plots> 已保存{len(rows)}张case图及批次总览：{root}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
