"""批测的选例、进程管理与结果汇总，不调用模型。"""

from __future__ import annotations

import csv
import codecs
import json
import os
import pty
import re
import signal
import subprocess
import time
import threading
import uuid
from pathlib import Path
from typing import Any

METRICS = ("NE", "SR", "OSR", "steps_taken")
SUPPORTED_SCENES = {9, 13, 21, 24}


def write_json(path: Path, value: Any) -> None:
    """原子保存，避免读取到半份结果。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.{uuid.uuid4().hex}.tmp")
    try:
        temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def select_indices(selection: str | list[int], episodes: list[dict], scenes: list[int]) -> list[int]:
    """按用户书写顺序展开闭区间；all 按全局索引选择指定场景。"""
    if not scenes or not set(scenes) <= SUPPORTED_SCENES:
        raise ValueError("当前仅支持 scene 9/13/21/24；scene 7 深度待修复")
    if selection == "all":
        indices = [i for i, item in enumerate(episodes) if int(item["scene_id"]) in scenes]
    elif isinstance(selection, list):
        indices = selection
    elif isinstance(selection, str):
        indices = []
        for token in re.split(r"[,\s]+", selection.strip()):
            if not re.fullmatch(r"\d+(?:-\d+)?", token):
                raise ValueError(f"无效 case 选择：{token!r}")
            ends = [int(number) for number in token.split("-")]
            if len(ends) == 1:
                indices.append(ends[0])
            else:
                step = 1 if ends[1] >= ends[0] else -1
                indices.extend(range(ends[0], ends[1] + step, step))
    else:
        raise ValueError("cases 必须为索引数组、区间字符串或 all")
    if not indices:
        raise ValueError("没有选中任何 case")
    for index in indices:
        if isinstance(index, bool) or not isinstance(index, int) or not 0 <= index < len(episodes):
            raise ValueError(f"case-index 超出范围：{index}")
        if int(episodes[index]["scene_id"]) not in scenes:
            raise ValueError(f"case {index} 不在所选可用场景中")
        instruction_text(episodes[index])
    return indices


def instruction_text(episode: dict) -> str:
    """只读取原始英文指令，不给 agent 传目标坐标和参考路径。"""
    value = episode.get("instruction", {}).get("instruction_text")
    if not isinstance(value, str) or not value.strip():
        raise ValueError("case 缺少 instruction.instruction_text")
    return value.strip()


def means(rows: list[dict]) -> dict:
    """失败也进入 SR/OSR 分母；NE 缺失单独报告，不伪装成零米。"""
    result: dict[str, Any] = {"attempts": len(rows)}
    for name in METRICS:
        values = [row[name] for row in rows if row.get(name) is not None]
        result[name] = sum(values) / len(values) if values else None
        result[name + "_valid_count"] = len(values)
    result["NE_missing_count"] = len(rows) - result["NE_valid_count"]
    successful = [row["steps_taken"] for row in rows if row.get("SR") == 1]
    result["steps_on_success"] = sum(successful) / len(successful) if successful else None
    return result


def save_summary(root: Path, rows: list[dict], planned: int, status: str) -> None:
    """每条任务结束即更新 CSV、总体、每轮、每场景和每个 case 的均值。"""
    fields = ["attempt", "cycle", "case_index", "scene_id", "episode_id", "status", "end_reason",
              *METRICS, "elapsed_s", "run_dir", "video_bytes", "runtime_status"]
    with (root / "results.csv").open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows({key: row.get(key) for key in fields} for row in rows)
    summary = {"status": status, "planned_attempts": planned, "finished_attempts": len(rows),
               "overall": means(rows)}
    for label, key in (("by_cycle", "cycle"), ("by_scene", "scene_id"), ("by_case", "case_index")):
        summary[label] = {str(value): means([row for row in rows if row[key] == value])
                          for value in sorted({row[key] for row in rows})}
    write_json(root / "summary.json", summary)
    lines = ["# 仿真批测结果", "", f"状态：{status}；已完成 {len(rows)}/{planned} 次尝试。", "",
             "| 范围 | 次数 | NE ↓ | SR ↑ | OSR ↑ | steps_taken |", "|---|---:|---:|---:|---:|---:|"]
    for label, group in [("全部", means(rows)), *[(f"循环 {k}", v) for k, v in summary["by_cycle"].items()]]:
        values = ["缺失" if group[name] is None else f"{group[name]:.4f}" for name in METRICS]
        lines.append(f"| {label} | {group['attempts']} | " + " | ".join(values) + " |")
    lines += ["", f"NE 缺失次数：{summary['overall']['NE_missing_count']}。失败尝试保留实际 OSR 和步数。",
              "SR/OSR 为 0–1 均值；steps_taken 同时包含失败尝试，成功样本均值另见 summary.json。"]
    (root / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


class ManagedProcess:
    """只管理自己启动的进程组，PX4 保留输入 PTY 避免 EOF 退出。"""

    def __init__(self, command: list[str], log_path: Path, *, env: dict, cwd: Path,
                 terminal_input: bool = False, capture_timeline: bool = False):
        self.log = log_path.open("wb")
        self.master = None
        self.reader = None
        slave = None
        if terminal_input:
            self.master, slave = pty.openpty()
        try:
            self.process = subprocess.Popen(command, cwd=cwd, env=env, stdin=slave if slave is not None else subprocess.DEVNULL,
                                            stdout=subprocess.PIPE if capture_timeline else self.log,
                                            stderr=subprocess.STDOUT, start_new_session=True)
            if capture_timeline:
                self.reader = threading.Thread(target=self._capture_console,
                    args=(log_path.with_name('console_timeline.jsonl'),), daemon=True)
                self.reader.start()
        except BaseException:
            self.log.close()
            if self.master is not None:
                os.close(self.master)
            raise
        finally:
            if slave is not None:
                os.close(slave)

    def _capture_console(self, path: Path) -> None:
        """原样保存输出，同时标记控制台块的实际接收时刻用于同步检查。"""
        decoder = codecs.getincrementaldecoder('utf-8')(errors='replace')
        with path.open('w', encoding='utf-8') as timeline:
            while chunk := os.read(self.process.stdout.fileno(), 65536):
                captured = time.monotonic_ns()
                self.log.write(chunk)
                self.log.flush()
                timeline.write(json.dumps({'captured_monotonic_ns': captured,
                    'captured_wall_ns': time.time_ns(), 'text': decoder.decode(chunk)}, ensure_ascii=False) + '\n')
                timeline.flush()
            # 明确输出已结束，录像可读取最后一批提示后再封装。
            timeline.write(json.dumps({'captured_monotonic_ns': time.monotonic_ns(),
                'captured_wall_ns': time.time_ns(), 'text': decoder.decode(b'', final=True),
                'console_closed': True}, ensure_ascii=False) + '\n')
            timeline.flush()

    def stop(self) -> None:
        """先发可清理的信号，超时再结束本进程组的剩余进程。"""
        try:
            for sig, seconds in ((signal.SIGINT, 3.0), (signal.SIGTERM, 12.0), (signal.SIGKILL, 2.0)):
                try:
                    os.killpg(self.process.pid, sig)
                except ProcessLookupError:
                    break
                deadline = time.monotonic() + seconds
                while time.monotonic() < deadline:
                    self.process.poll()
                    try:
                        os.killpg(self.process.pid, 0)
                    except ProcessLookupError:
                        return
                    time.sleep(0.1)
            self.process.wait(timeout=2)
        finally:
            if self.reader is not None:
                self.reader.join(timeout=2)
                self.process.stdout.close()
            self.log.close()
            if self.master is not None:
                os.close(self.master)
                self.master = None


def occupied_sim_ports() -> list[int]:
    """只读检查固定端口，已有仿真不在脚本的清理范围内。"""
    wanted = {4560, 41451, 14030, 14280, 14540, 14580}
    busy = set()
    for name in ("tcp", "tcp6", "udp", "udp6"):
        for line in Path("/proc/net", name).read_text().splitlines()[1:]:
            fields = line.split()
            if name.startswith("tcp") and fields[3] != "0A":
                continue
            port = int(fields[1].split(":")[-1], 16)
            if port in wanted:
                busy.add(port)
    return sorted(busy)


def pending_px4_tcp_restart() -> bool:
    """AirSim 的 PX4 TCP 监听端口不能立即复用 TIME_WAIT，等内核自然释放。"""
    for name in ("tcp", "tcp6"):
        for line in Path("/proc/net", name).read_text().splitlines()[1:]:
            fields = line.split()
            if fields[3] == "06" and int(fields[1].split(":")[-1], 16) == 4560:
                return True
    return False
