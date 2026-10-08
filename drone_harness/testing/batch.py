"""顺序运行用户选择的 AerialVLN-S case，不额外调用模型。"""

from __future__ import annotations

import argparse
import json
import math
import os
import shlex
import shutil
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

from drone_harness.testing.batch_support import (
    METRICS, ManagedProcess, instruction_text, occupied_sim_ports,
    pending_px4_tcp_restart, save_summary, select_indices, write_json,
)

REPO = Path(__file__).resolve().parents[2]
MAX_CASE_TIMEOUT_S = 30 * 60
DEFAULTS = {
    "cases": "351", "scenes": [9, 13, 21, 24], "repeat": 1,
    "data_root": "/home/stan/AirVLN_ws/DATA/data/aerialvln-s",
    "adapter_repo": "/home/stan/AirVLN_ws/AirVLN-theta-star",
    "env_root": "/home/stan/AirVLN_ws/ENVs",
    "px4_root": "/home/stan/AirVLN/px4_native",
    "px4_venv": "/home/stan/AirVLN/px4_py38",
    "airsim_python": "/home/stan/miniconda3/envs/AirVLN/bin/python",
    "ros_python": "/usr/bin/python3", "ros_domain_id": 42,
    "ros_setups": ["/opt/ros/humble/setup.bash", "/home/stan/AirVLN/mavros_ws/install/setup.bash",
                   "/home/stan/AirVLN/hw_ros2_native/ros2/install/setup.bash"],
    "output_root": "/home/stan/AirVLN/drone_harness_runtime/batches",
    "case_timeout_s": MAX_CASE_TIMEOUT_S, "startup_timeout_s": 120, "position_tolerance_m": 1.0,
    "record": True, "preview": True, "preview_fps": 20, "video_fps": 20, "video_width": 640, "video_bitrate_kbps": 500,
    "response_language": "",
    "video_max_mb": 512, "min_free_gb": 5,
}


def read_config(args: argparse.Namespace) -> dict:
    """合并配置和命令行覆盖项，并在启动前校验预算。"""
    config = dict(DEFAULTS)
    if args.config:
        supplied = json.loads(args.config.read_text(encoding="utf-8"))
        unknown = set(supplied) - set(DEFAULTS)
        if unknown:
            raise ValueError(f"未知配置项：{sorted(unknown)}")
        config.update(supplied)
    for key in ("cases", "repeat"):
        if getattr(args, key) is not None:
            config[key] = getattr(args, key)
    if args.scenes is not None:
        config["scenes"] = [int(value) for value in args.scenes.split(",")]
    if args.cases_file:
        config["cases"] = " ".join(line.split("#", 1)[0] for line in args.cases_file.read_text(encoding="utf-8").splitlines())
    for key in ("repeat", "video_width", "video_max_mb", "case_timeout_s", "startup_timeout_s", "video_bitrate_kbps"):
        if isinstance(config[key], bool) or not isinstance(config[key], int) or config[key] <= 0:
            raise ValueError(f"{key} 必须为正整数")
    if config["case_timeout_s"] > MAX_CASE_TIMEOUT_S:
        raise ValueError("每个 case 最多运行 1800 秒（30 分钟）；case_timeout_s 可调低，不能调高")
    if (not math.isfinite(config["video_fps"]) or not 1 <= config["video_fps"] <= 30
            or not math.isfinite(config["preview_fps"]) or not 1 <= config["preview_fps"] <= 30
            or not math.isfinite(config["min_free_gb"]) or not 0 < config["min_free_gb"]):
        raise ValueError("video_fps/preview_fps 应为 1–30，min_free_gb 应大于 0")
    if config["response_language"] not in {"", "zh"}:
        raise ValueError("response_language 只支持空字符串或 zh")
    if not 160 <= config["video_width"] <= 1920 or config["video_width"] % 2:
        raise ValueError("video_width 应为 160–1920 的偶数")
    for key in ("record", "preview"):
        if not isinstance(config[key], bool):
            raise ValueError(f"{key} 必须为布尔值")
    return config


def ros_environment(config: dict) -> dict[str, str]:
    """显式加载 ROS overlay，源码路径放在安装包之前。"""
    commands = []
    for value in config["ros_setups"]:
        path = Path(value)
        if not path.is_file():
            raise FileNotFoundError(path)
        commands.append("source " + shlex.quote(str(path)))
    commands.append("env -0")
    result = subprocess.run(["bash", "--noprofile", "--norc", "-c", "set -e; " + "; ".join(commands)],
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True)
    env = dict(item.decode().split("=", 1) for item in result.stdout.split(b"\0") if b"=" in item)
    env["PYTHONPATH"] = str(REPO) + os.pathsep + env.get("PYTHONPATH", "")
    env.update(ROS_DOMAIN_ID=str(config["ros_domain_id"]), PX4_SIM_HOST_ADDR="127.0.0.1", PYTHONUNBUFFERED="1")
    return env


def validate_assets(config: dict, episodes: list[dict], indices: list[int]) -> None:
    """只读检查可执行文件与场景资产，不启动仿真。"""
    paths = [Path(config["airsim_python"]), Path(config["ros_python"]),
             Path(config["adapter_repo"]) / "tools/aerialvln_px4_launch.py",
             Path(config["px4_root"]) / "build/px4_sitl_default/bin/px4"]
    paths += [Path(config["env_root"]) / f"env_{scene}/LinuxNoEditor/AirVLN.sh"
              for scene in {int(episodes[index]["scene_id"]) for index in indices}]
    for path in paths:
        if not path.is_file():
            raise FileNotFoundError(path)


def stream_agent(path: Path, offset: int) -> int:
    """把 agent 原始输出实时转发到批测终端，文件中保留完整内容。"""
    if path.is_file():
        with path.open("rb") as handle:
            handle.seek(offset)
            chunk = handle.read()
            if chunk:
                sys.stdout.write(chunk.decode("utf-8", errors="replace"))
                sys.stdout.flush()
            return handle.tell()
    return offset


def run_case(config: dict, episode: dict, index: int, root: Path, env: dict) -> dict:
    """管理单次尝试；runtime 返回、退出或超时后立即停止本条任务。"""
    root.mkdir(parents=True, exist_ok=False)
    write_json(root / "case_identity.json", {"case_index": index, "scene_id": episode["scene_id"],
                                             "episode_id": episode["episode_id"]})
    instruction = instruction_text(episode)
    (root / "original_instruction.txt").write_text(instruction + "\n", encoding="utf-8")
    if config.get("response_language") == "zh":
        instruction += "\n\n请用中文回答、描述观察结果和说明导航计划；保持上述英文导航指令的含义与路线不变。"
    (root / "instruction.txt").write_text(instruction + "\n", encoding="utf-8")
    started = time.monotonic()
    case_deadline = started + config["case_timeout_s"]
    owned: list[ManagedProcess] = []
    agent = None
    launcher = None
    recorder = None
    stop = root / "stop.json"
    runtime_result = root / "runtime_result.json"
    scene_dir = root / "scene"
    reason = "startup_error"
    halt_batch = False
    console_offset = 0

    def launch(name: str, command: list[str], *, cwd: Path = REPO, child_env: dict | None = None,
               terminal_input: bool = False) -> ManagedProcess:
        """启动一个有独立日志和进程组的组件。"""
        process = ManagedProcess(command, root / f"{name}.log", env=child_env or env,
                                 cwd=cwd, terminal_input=terminal_input, capture_timeline=name == "agent")
        owned.append(process)
        return process

    try:
        busy = occupied_sim_ports()
        if busy:
            halt_batch = True
            raise RuntimeError(f"端口已有进程占用：{busy}；请先结束手动运行")
        while pending_px4_tcp_restart():
            if time.monotonic() >= case_deadline:
                raise TimeoutError("等待 PX4 TCP 端口释放已达到本条时限")
            time.sleep(0.2)
        px4_env = dict(env)
        px4_env["PATH"] = str(Path(config["px4_venv"]) / "bin") + os.pathsep + px4_env["PATH"]
        px4_env["VIRTUAL_ENV"] = config["px4_venv"]
        launch("px4", ["make", "px4_sitl_default", "none_iris"], cwd=Path(config["px4_root"]),
               child_env=px4_env, terminal_input=True)
        adapter = Path(config["adapter_repo"])
        scene_env = dict(env)
        scene_env["PYTHONPATH"] = str(adapter) + os.pathsep + scene_env.get("PYTHONPATH", "")
        launcher = launch("scene", [config["airsim_python"], str(adapter / "tools/aerialvln_px4_launch.py"),
            "--split", "val_unseen", "--case-index", str(index), "--scene-id", str(episode["scene_id"]),
            "--data-root", config["data_root"], "--env-root", config["env_root"], "--view-mode", "Fpv",
            "--position-tolerance", str(config["position_tolerance_m"]), "--evaluate",
            "--wait-seconds", str(config["startup_timeout_s"]),
            "--run-dir", str(scene_dir), "--agent-log-root", str(root / "agent_logs"),
            "--agent-result-file", str(runtime_result), "--stop-file", str(stop),
            "--evaluation-timeout-seconds", str(config["case_timeout_s"] + config["startup_timeout_s"])],
            cwd=adapter, child_env=scene_env)
        deadline = min(case_deadline, time.monotonic() + config["startup_timeout_s"])
        while "evaluation_ready:" not in (root / "scene.log").read_text(errors="replace"):
            if launcher.process.poll() is not None:
                raise RuntimeError("场景加载或起点检查失败，详见 scene.log")
            if time.monotonic() >= deadline:
                raise TimeoutError("等待场景 evaluation_ready 超时")
            time.sleep(0.2)
        launch("mavros", ["ros2", "launch", "mavros", "px4.launch", "fcu_url:=udp://:14540@127.0.0.1:14580"])
        # 原桥接初始化会请求解锁，需等 PX4 自检就绪，不能只凭 RPC 可连接就启动。
        deadline = min(case_deadline, time.monotonic() + config["startup_timeout_s"])
        while "Ready for takeoff!" not in (root / "px4.log").read_text(errors="replace"):
            if any(item.process.poll() is not None for item in owned):
                raise RuntimeError("等待 PX4 就绪时组件退出，详见各组件日志")
            if time.monotonic() >= deadline:
                raise TimeoutError("等待 PX4 Ready for takeoff 超时")
            time.sleep(0.2)
        # 直接复用原相机 launch，避免要求重新安装 harness 的 ROS 包。
        launch("camera", ["ros2", "launch", "airsim_ros_pkgs", "airsim_node.launch.py", "host:=127.0.0.1"])
        if config["record"] or config["preview"]:
            recorder = launch("recorder", [config["ros_python"], "-m", "drone_harness.testing.recording",
                "--run-dir", str(root), "--fps", str(config["video_fps"]),
                "--preview-fps", str(config["preview_fps"]),
                "--width", str(config["video_width"]), "--max-mb", str(config["video_max_mb"]),
                "--bitrate-kbps", str(config["video_bitrate_kbps"]),
                "--min-free-gb", str(config["min_free_gb"]),
                *( ["--preview"] if config["preview"] else []),
                *( ["--record"] if config["record"] else [])], cwd=Path(config.get("source_root", REPO)))
            deadline = min(case_deadline, time.monotonic() + config["startup_timeout_s"])
            while not (root / "recorder_ready.json").is_file():
                if stop.is_file():
                    raise RuntimeError("当前任务已收到窗口停止请求")
                if recorder.process.poll() is not None:
                    halt_batch = True
                    raise RuntimeError("录像/实时预览初始化失败，详见 recorder.log")
                camera_log = root / "camera.log"
                if "process has died" in camera_log.read_text(errors="replace"):
                    raise RuntimeError("相机桥子进程退出，详见 camera.log")
                if time.monotonic() >= deadline:
                    raise TimeoutError("等待实时相机画面超时")
                time.sleep(0.2)
        if stop.is_file():
            raise RuntimeError("当前任务已收到停止请求，不启动 agent")
        if time.monotonic() >= case_deadline:
            raise TimeoutError("本条 case 已达到运行时间上限，不启动 agent")
        agent = launch("agent", [config["ros_python"], "-m", "drone_harness", "--instruction-file", str(root / "instruction.txt"),
                                "--result-file", str(runtime_result), "--log-dir", str(root / "agent_logs"),
                                "--startup-timeout-s", str(config["startup_timeout_s"])],
                       cwd=Path(config.get("source_root", REPO)))
        reason = "case_timeout"
        while not (scene_dir / "evaluation.json").is_file():
            console_offset = stream_agent(root / "agent.log", console_offset)
            if runtime_result.is_file():
                reason = json.loads(runtime_result.read_text())["end_reason"]
                break
            if agent.process.poll() is not None:
                reason = "agent_process_exited"
                break
            if stop.is_file():
                reason = json.loads(stop.read_text())["end_reason"]
                break
            if recorder is not None and recorder.process.poll() is not None:
                reason = "recorder_exited"
                halt_batch = True
                break
            if any(item.process.poll() is not None for item in owned if item not in (agent, recorder)):
                reason = "component_exited"
                break
            if time.monotonic() >= case_deadline:
                break
            if shutil.disk_usage(root).free < config["min_free_gb"] * 1024**3:
                reason = "disk_low"
                halt_batch = True
                break
            time.sleep(0.1)
    except KeyboardInterrupt:
        reason = "batch_interrupted"
        halt_batch = True
    except TimeoutError as exc:
        reason = "case_timeout" if time.monotonic() >= case_deadline else "startup_timeout"
        (root / "error.txt").write_text(f"{reason}: {exc}\n", encoding="utf-8")
        print(f"batch> case {index}：{reason}，保存结果后继续下一条。", flush=True)
    except Exception as exc:
        (root / "error.txt").write_text(f"{type(exc).__name__}: {exc}\n", encoding="utf-8")
        print(f"batch> case {index}：{type(exc).__name__}，详见 {root / 'error.txt'}", flush=True)
    finally:
        if stop.is_file():
            reason = json.loads(stop.read_text())["end_reason"]
        elif (scene_dir / "evaluation.json").is_file():
            # 评分器可能比 runtime 更早确认降落，不能沿用循环初始化的超时原因。
            reason = json.loads((scene_dir / "evaluation.json").read_text())["end_reason"]
        if reason in {"disk_low", "video_size_limit", "batch_user_stop", "recorder_error"}:
            halt_batch = True
        # runtime 正常返回时由评分器自行读取结果；提前写停止信号会误覆盖刚完成的降落。
        needs_stop_signal = (not runtime_result.is_file() and not (scene_dir / "evaluation.json").is_file()) or reason in {
            "case_timeout", "runtime_user_exit", "batch_user_stop", "disk_low", "video_size_limit",
            "recorder_error", "recorder_exited", "component_exited", "batch_interrupted"}
        if needs_stop_signal:
            write_json(stop, {"end_reason": reason})
        if agent is not None:
            # 优先让完成降落的 runtime 写完收尾；其它情况即刻停止导航线程。
            if (scene_dir / "evaluation.json").is_file():
                try:
                    agent.process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    pass
            agent.stop()
        if launcher is not None:
            try:
                launcher.process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                pass
        # 评分器结算后再通知录像结束，保留正常成功收尾。
        if not stop.is_file():
            write_json(stop, {"end_reason": reason})
        for process in reversed(owned):
            if process is not agent:
                process.stop()
        # RPC/ROS 子进程收尾后端口可能稍晚释放，避免下一条误认成已有手动仿真。
        if owned:
            settle_deadline = time.monotonic() + 75.0
            if pending_px4_tcp_restart():
                print("batch> 等待 PX4 TCP 端口冷却后再启动下一条。", flush=True)
            while (occupied_sim_ports() or pending_px4_tcp_restart()) and time.monotonic() < settle_deadline:
                time.sleep(0.2)
            if occupied_sim_ports() or pending_px4_tcp_restart():
                halt_batch = True
        stream_agent(root / "agent.log", console_offset)
    report_path = scene_dir / "evaluation.json"
    if report_path.is_file():
        report = json.loads(report_path.read_text())
    else:
        progress_path = scene_dir / "evaluation_progress.json"
        report = json.loads(progress_path.read_text()) if progress_path.is_file() else {
            "metrics": {"NE": None, "SR": 0, "OSR": 0, "steps_taken": 0}}
        report.update(status="incomplete", end_reason=reason)
        report["metrics"]["SR"] = 0
        report["recovered_from_checkpoint"] = progress_path.is_file()
        write_json(report_path, report)
    runtime = json.loads(runtime_result.read_text()) if runtime_result.is_file() else {}
    recording_path = root / "recording.json"
    recording = json.loads(recording_path.read_text()) if recording_path.is_file() else {}
    if recording.get("error"):
        halt_batch = True
    row = {"case_index": index, "scene_id": int(episode["scene_id"]), "episode_id": episode["episode_id"],
           "status": report["status"], "end_reason": report["end_reason"], **report["metrics"],
           "runtime_status": runtime.get("status", "exited_without_result"),
           "elapsed_s": round(time.monotonic() - started, 2), "run_dir": str(root),
           "video_bytes": (root / "flight.mp4").stat().st_size if (root / "flight.mp4").exists() else 0,
           "halt_batch": halt_batch}
    write_json(root / "result.json", row)
    return row


def main(argv: list[str] | None = None) -> int:
    """一个入口完成选例、循环、实时输出、录像与汇总。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path)
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument("--cases", help="如 351,397,0-350，或 all；按书写顺序执行")
    selection.add_argument("--cases-file", type=Path, help="每行 case 索引或区间，支持 # 注释")
    parser.add_argument("--scenes", help="all 的场景选择，如 9 或 9,13,21,24")
    parser.add_argument("--repeat", type=int, help="整份清单顺序循环的次数")
    parser.add_argument("--dry-run", action="store_true", help="只验证资产与顺序，不启动仿真、不调用模型")
    args = parser.parse_args(argv)
    rows = []
    try:
        config = read_config(args)
        data = json.loads((Path(config["data_root"]) / "val_unseen.json").read_text(encoding="utf-8"))
        episodes = data["episodes"]
        indices = select_indices(config["cases"], episodes, config["scenes"])
        validate_assets(config, episodes, indices)
        planned = len(indices) * config["repeat"]
        print(f"batch> 每轮 {len(indices)} 条，循环 {config['repeat']} 次，共 {planned} 次；顺序：{indices}", flush=True)
        if args.dry_run:
            print("batch> 只读预检完成，没有启动仿真或请求模型。")
            return 0
        env = ros_environment(config)
        if config["preview"] and not env.get("DISPLAY"):
            raise RuntimeError("实时窗口需要桌面 DISPLAY；请在桌面终端运行，或设置 preview=false")
        # 只校验依赖与本地配置，不请求模型、不初始化 ROS 节点。
        subprocess.run([config["ros_python"], "-c",
            "import rclpy,cv_bridge,cv2,numpy; from PIL import ImageFont; "
            "from drone_harness.config.loader import load_profile; load_profile('sim')"], env=env, check=True)
        if config["record"]:
            subprocess.run([config["ros_python"], "-c",
                "from drone_harness.testing.video import ffmpeg_executable; import subprocess; "
                "subprocess.run([ffmpeg_executable(), '-v', 'error', '-f', 'lavfi', '-i', 'color=s=16x16', "
                "'-frames:v', '1', '-c:v', 'libx264', '-f', 'null', '-'], check=True)"], env=env, check=True)
        root = Path(config["output_root"]) / datetime.now().strftime("%Y%m%d-%H%M%S-%f")
        root.mkdir(parents=True, exist_ok=False)
        write_json(root / "config.json", config)
        write_json(root / "manifest.json", {"cases": indices, "repeat": config["repeat"], "planned_attempts": planned})
        # 保存当前 dirty 差异，方便复现实验；不清理、不提交任何工作树。
        for name, repo in (("harness", REPO), ("adapter", Path(config["adapter_repo"]))):
            for suffix, command in (("head.txt", ["git", "rev-parse", "HEAD"]),
                                    ("diff.patch", ["git", "diff", "HEAD"]),
                                    ("status.txt", ["git", "status", "--short"])):
                snapshot = subprocess.run(command, cwd=repo, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                (root / f"{name}_{suffix}").write_bytes(snapshot.stdout)
        # 冻结当前源码（含 dirty 改动），长批次中编辑工作树不影响后续 case。
        source = root / "source"
        shutil.copytree(REPO / "drone_harness", source / "drone_harness",
                        ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
        adapter_source = root / "adapter_source"
        (adapter_source / "tools").mkdir(parents=True)
        for filename in ("aerialvln_case.py", "aerialvln_evaluation.py", "aerialvln_px4_launch.py"):
            shutil.copy2(Path(config["adapter_repo"]) / "tools" / filename, adapter_source / "tools" / filename)
        env["PYTHONPATH"] = str(source) + os.pathsep + env.get("PYTHONPATH", "")
        run_config = {**config, "adapter_repo": str(adapter_source), "source_root": str(source)}
        print(f"batch> 结果目录：{root}", flush=True)
        status = "running"
        save_summary(root, rows, planned, status)
        try:
            for cycle in range(1, config["repeat"] + 1):
                for index in indices:
                    if shutil.disk_usage(root).free < config["min_free_gb"] * 1024**3:
                        status = "disk_low"
                        return 2
                    attempt = len(rows) + 1
                    print(f"batch> [{attempt}/{planned}] 循环 {cycle} case {index}", flush=True)
                    row = run_case(run_config, episodes[index], index,
                                   root / f"{attempt:05d}-r{cycle}-case{index}", env)
                    row.update(attempt=attempt, cycle=cycle)
                    rows.append(row)
                    save_summary(root, rows, planned, status)
                    print("batch> " + " ".join(f"{key}={row[key]}" for key in METRICS) +
                          f" end={row['end_reason']}", flush=True)
                    if row["halt_batch"]:
                        status = "stopped"
                        return 2
            status = "completed"
            return 0
        except KeyboardInterrupt:
            status = "interrupted"
            return 130
        finally:
            save_summary(root, rows, planned, status)
            print(f"batch> {status}；汇总：{root / 'summary.md'}", flush=True)
    except Exception as exc:
        print(f"batch> {type(exc).__name__}: {exc}", file=sys.stderr, flush=True)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
