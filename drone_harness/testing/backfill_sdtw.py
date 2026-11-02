"""只为指定已有批次补算动作采样 SDTW，保留原始评测、录像和日志。"""

from __future__ import annotations

import argparse
import json
import re
from datetime import datetime, timedelta, timezone
from pathlib import Path

from drone_harness.testing.batch_support import save_summary, write_json
from drone_harness.testing.sdtw import calculate_sdtw, is_motion_event, trajectory_xyz


def _timestamp(value: str) -> float:
    """旧动作日志按北京时间解析，带时区的新日志按自身时区解析。"""
    moment = datetime.fromisoformat(value)
    if moment.tzinfo is None:
        moment = moment.replace(tzinfo=timezone(timedelta(hours=8)))
    return moment.timestamp()


def reconstruct_actions(run: Path, report: dict) -> tuple[list[dict], dict]:
    """按动作结束时间匹配历史世界坐标，明确记录秒级时间戳的不确定性。"""
    source = run / "scene/trajectory.jsonl"
    sessions = sorted((run / "agent_logs").glob("session_*/tool_calls.jsonl"))
    if not source.is_file() or len(sessions) != 1:
        raise ValueError("缺少实际轨迹或无法唯一确定动作日志")
    samples = [json.loads(line) for line in source.read_text(encoding="utf-8").splitlines()]
    trajectory_xyz([item["position_ned"] for item in samples])
    stamps = [_timestamp(item["timestamp"]) for item in samples]
    regressions = [before - after for before, after in zip(stamps, stamps[1:]) if after < before]
    records = [{**samples[0], "kind": "initial"}]
    offsets = []
    coarse = 0
    previous_index = 0
    for line in sessions[0].read_text(encoding="utf-8").splitlines():
        event = json.loads(line)
        if not is_motion_event(event):
            continue
        precise = event.get("timestamp_utc")
        target = _timestamp(precise or event["timestamp"])
        if precise is None:
            # 秒级旧时间戳代表一秒区间，取中点以限制量化误差。
            target += 0.5
            coarse += 1
        # 墙钟可能小幅回拨；按文件中的真实采集顺序前进，不按时间重新排列轨迹。
        nearest = min(range(previous_index, len(stamps)), key=lambda i: abs(stamps[i] - target))
        previous_index = nearest
        offset = abs(stamps[nearest] - target)
        if offset > (1.0 if precise is None else 0.5):
            raise ValueError("动作结束附近缺少实际位置，无法可靠补算")
        offsets.append(offset)
        records.append({**samples[nearest], "kind": "motion_end", "tool_name": event["tool_name"],
                        "action_end_timestamp": precise or event["timestamp"],
                        "success": event["result"].get("success") is True,
                        "alignment_offset_s": offset})
        if event["tool_name"] == "land" and event["result"].get("success") is True:
            break
    final = report.get("final_position_ned")
    if final is None:
        raise ValueError("原始评测缺少终止位置")
    records.append({"kind": "final", "position_ned": final})
    return records, {"sampling_method": "historical_motion_time_alignment_v1",
                     "approximate": True, "coarse_timestamp_actions": coarse,
                     "coarse_timestamp_uncertainty_s": 0.5 if coarse else 0.0,
                     "maximum_alignment_offset_s": max(offsets, default=0.0),
                     "pose_clock_regression_count": len(regressions),
                     "maximum_pose_clock_regression_s": max(regressions, default=0.0),
                     "motion_event_count": len(offsets)}


def backfill_batch(root: Path) -> dict:
    """补算指定批次并替换自身汇总字段，原始scene评测及软链接目标保持原样。"""
    if (root / "sdtw_backfill.json").exists():
        raise ValueError("该批次已补算SDTW；请先检查已有记录，避免覆盖来源证据")
    config = json.loads((root / "config.json").read_text(encoding="utf-8"))
    episodes = json.loads((Path(config["data_root"]) / "val_unseen.json").read_text(encoding="utf-8"))["episodes"]
    old_summary = json.loads((root / "summary.json").read_text(encoding="utf-8"))
    old_markdown = (root / "summary.md").read_text(encoding="utf-8")
    prepared = []
    for run in sorted(root.glob("*-r*-case*")):
        result_path = run / "result.json"
        if run.is_symlink() or result_path.is_symlink():
            raise ValueError("case目录和result.json必须属于当前批次，不能通过软链接修改原始结果")
        row = json.loads(result_path.read_text(encoding="utf-8"))
        identity = re.fullmatch(r"(\d+)-r(\d+)-case(\d+)", run.name)
        if identity is None or int(identity[3]) != row["case_index"]:
            raise ValueError("case目录与结果编号不一致")
        row.update(attempt=int(identity[1]), cycle=int(identity[2]))
        episode = episodes[row["case_index"]]
        if str(episode["episode_id"]) != str(row["episode_id"]) or int(episode["scene_id"]) != int(row["scene_id"]):
            raise ValueError("case身份与数据集不一致")
        report = json.loads((run / "scene/evaluation.json").read_text(encoding="utf-8"))
        if any(report["metrics"].get(name) != row.get(name) for name in ("NE", "SR", "OSR")):
            raise ValueError("原始评测与当前结果的几何指标不一致")
        try:
            records, alignment = reconstruct_actions(run, report)
            score = calculate_sdtw([item["position_ned"] for item in records], episode.get("reference_path", []), row["SR"])
            score.update(alignment)
        except (KeyError, TypeError, ValueError, OSError) as error:
            records = []
            score = calculate_sdtw([], [], row["SR"], sampling_error=str(error))
            score.update(sampling_method="historical_motion_time_alignment_v1", approximate=True)
        old_metrics = {name: row.get(name) for name in ("NE", "SR", "OSR", "steps_taken")}
        row.pop("steps_taken", None)
        row["SDTW"] = score["SDTW"]
        prepared.append((run, row, records, score, old_metrics))
    if len(prepared) != old_summary["finished_attempts"]:
        raise ValueError("case目录数量与批次已结算次数不一致")
    evidence = {"created_at": datetime.now(timezone.utc).isoformat(), "batch_dir": str(root),
                "previous_summary": old_summary, "runs": []}
    for run, row, records, score, old_metrics in prepared:
        write_json(run / "sdtw.json", score)
        (run / "action_trajectory.jsonl").write_text("".join(json.dumps(item, ensure_ascii=False) + "\n"
                                                             for item in records), encoding="utf-8")
        write_json(run / "result.json", row)
        evidence["runs"].append({"run": run.name, "previous_metrics": old_metrics, "SDTW": score["SDTW"]})
    rows = [item[1] for item in prepared]
    save_summary(root, rows, old_summary["planned_attempts"], old_summary["status"])
    summary = json.loads((root / "summary.json").read_text(encoding="utf-8"))
    for key in ("kind", "provenance_manifest"):
        if key in old_summary:
            summary[key] = old_summary[key]
    write_json(root / "summary.json", summary)
    prefix, separator, _ = old_markdown.partition("状态：")
    if separator and prefix.strip() != "# 仿真批测结果":
        generated = (root / "summary.md").read_text(encoding="utf-8")
        generated = generated[generated.index("状态："):]
        (root / "summary.md").write_text(prefix + generated, encoding="utf-8")
    with (root / "summary.md").open("a", encoding="utf-8") as handle:
        handle.write("\nSDTW由历史动作结束时间与世界坐标轨迹对齐补算；秒级日志采用一秒区间中点，属于近似结果。"
                     "原始scene评测、录像、日志和来源映射保留；逐例计算及缺失原因见sdtw.json。\n")
        handle.write("\n| case | SDTW ↑（%） | 计算记录 |\n|---:|---:|---|\n")
        for run, row, _, score, _ in prepared:
            value = "缺失" if score["SDTW"] is None else f"{100 * score['SDTW']:.4f}"
            handle.write(f"| {row['case_index']} | {value} | [sdtw.json]({run.name}/sdtw.json) |\n")
    write_json(root / "sdtw_backfill.json", evidence)
    return summary


def main(argv: list[str] | None = None) -> int:
    """只处理命令行指定的一个批次，不自动遍历其他历史结果。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("batch_dir", type=Path)
    args = parser.parse_args(argv)
    summary = backfill_batch(args.batch_dir)
    print(json.dumps(summary["overall"], ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
