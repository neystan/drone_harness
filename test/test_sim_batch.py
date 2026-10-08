"""离线验证批测顺序、失败结算、单任务入口与录像，不启动仿真。"""

import json
import subprocess
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from drone_harness.testing import batch
from drone_harness.testing.batch_support import instruction_text, means, save_summary, select_indices, write_json
from drone_harness.testing.recording import EventTail
from drone_harness.runtime import runtime
from test_agent_observation_loop import context_for


def episodes():
    """提供不同场景与私有终点，检查 agent 只收到原始指令。"""
    return [{"scene_id": scene, "episode_id": f"ep-{i}", "instruction": {"instruction_text": f"go {i}"},
             "goals": [{"position": [999, 888, 777]}]} for i, scene in enumerate([9, 24, 13, 7])]


def test_selection_preserves_order_and_supports_long_ranges():
    data = episodes()
    assert select_indices("2,0-1,2-0", data, [9, 13, 24]) == [2, 0, 1, 2, 1, 0]
    assert select_indices("all", data, [9]) == [0]
    assert select_indices("all", data, [9, 13, 24]) == [0, 1, 2]
    many = [deepcopy(data[0]) for _ in range(351)]
    assert select_indices("0-350", many, [9]) == list(range(351))


def test_case_timeout_defaults_to_30_minutes_and_cannot_exceed_it(tmp_path):
    args = SimpleNamespace(config=None, cases=None, repeat=None, scenes=None, cases_file=None)
    assert batch.read_config(args)["case_timeout_s"] == 1800
    args.config = tmp_path / "batch.json"
    write_json(args.config, {"case_timeout_s": 3600})
    with pytest.raises(ValueError, match="30 分钟"):
        batch.read_config(args)
    write_json(args.config, {"case_timeout_s": 1200})
    assert batch.read_config(args)["case_timeout_s"] == 1200


def test_default_preview_and_video_are_20fps_and_language_is_validated(tmp_path):
    args = SimpleNamespace(config=None, cases=None, repeat=None, scenes=None, cases_file=None)
    config = batch.read_config(args)
    assert config["preview_fps"] == config["video_fps"] == 20
    args.config = tmp_path / "batch.json"
    write_json(args.config, {"response_language": "invalid"})
    with pytest.raises(ValueError, match="response_language"):
        batch.read_config(args)


@pytest.mark.parametrize("selection", ["3", "-1", "4", "0;exit", [], [True]])
def test_invalid_or_depth_excluded_selection_fails_before_launch(selection):
    with pytest.raises(ValueError):
        select_indices(selection, episodes(), [9, 13, 24])


def test_summary_keeps_failed_attempts_and_missing_ne_visible(tmp_path):
    rows = [{"cycle": 1, "case_index": 0, "scene_id": 9, "NE": 2.0, "SR": 1, "OSR": 1, "steps_taken": 10},
            {"cycle": 1, "case_index": 1, "scene_id": 24, "NE": None, "SR": 0, "OSR": 0, "steps_taken": 2},
            {"cycle": 2, "case_index": 0, "scene_id": 9, "NE": 30.0, "SR": 0, "OSR": 1, "steps_taken": 12}]
    save_summary(tmp_path, rows, 4, "stopped")
    summary = json.loads((tmp_path / "summary.json").read_text())
    assert summary["finished_attempts"] == 3
    assert summary["overall"]["SR"] == pytest.approx(1 / 3)
    assert summary["overall"]["OSR"] == pytest.approx(2 / 3)
    assert summary["overall"]["NE"] == 16
    assert summary["overall"]["NE_missing_count"] == 1
    assert summary["by_case"]["0"]["attempts"] == 2
    assert summary["by_cycle"]["1"]["SR"] == 0.5
    assert (tmp_path / "results.csv").is_file()


def test_event_tail_keeps_partial_tool_result(tmp_path):
    path = tmp_path / "tool_calls.jsonl"
    path.write_bytes(b'{"tool_name":"for')
    tail = EventTail(path)
    assert tail.read() == []
    with path.open("ab") as handle:
        handle.write(b'ward"}\n')
    assert tail.read() == [{"tool_name": "forward"}]


def test_single_runtime_uses_existing_loop_once_without_goal_leak(tmp_path):
    context = context_for(tmp_path)
    context.navigation_enabled = True
    context.controller = SimpleNamespace(vehicle_status=SimpleNamespace(connected=True),
        uav_position_is_valid=lambda: True,
        wait_for_observation=lambda **kwargs: SimpleNamespace(depth=np.ones((2, 2)), intrinsics=object(), depth_error=""))
    calls = []
    def run_agent(client, model, messages, ctx):
        calls.append(messages)
        ctx.navigation_plan = SimpleNamespace(status="incomplete")
        return "提前停止"
    result = runtime._run_single_navigation(None, context, run_agent, instruction_text(episodes()[0]))
    assert len(calls) == 1
    assert calls[0][-1] == {"role": "user", "content": "go 0"}
    assert "999" not in str(calls)
    assert result["status"] == "incomplete" and result["end_reason"] == "runtime_stopped"


def test_single_runtime_not_ready_does_not_request_model(tmp_path):
    context = context_for(tmp_path)
    context.controller = SimpleNamespace()
    result = runtime._run_single_navigation(None, context, lambda *args: pytest.fail("不应请求模型"), "go", startup_timeout_s=0)
    assert result["end_reason"] == "runtime_not_ready"


def test_single_entry_writes_result_on_exception(tmp_path, monkeypatch):
    instruction = tmp_path / "instruction.txt"
    instruction.write_text("go", encoding="utf-8")
    monkeypatch.setattr(runtime, "load_profile", lambda name: context_for(tmp_path).profile)
    def fail(*args, **kwargs):
        raise RuntimeError("offline failure")
    monkeypatch.setattr(runtime, "_start_live_runtime", fail)
    result = tmp_path / "result.json"
    assert runtime.start_single_runtime(instruction, result, tmp_path / "logs") == 1
    assert json.loads(result.read_text())["end_reason"] == "runtime_error"


@pytest.mark.parametrize("completed", [False, True])
def test_runtime_stop_finalizes_before_process_cleanup(tmp_path, monkeypatch, completed):
    events = []
    class FakeProcess:
        def __init__(self, command, log_path, **kwargs):
            self.name = log_path.stem
            self.root = log_path.parent
            self.process = self
            self.returncode = None
            log_path.write_text("evaluation_ready:\n" if self.name == "scene" else
                                "Ready for takeoff!\n" if self.name == "px4" else "", encoding="utf-8")
            if self.name == "agent":
                write_json(self.root / "runtime_result.json", {"status": "completed" if completed else "incomplete",
                    "end_reason": "runtime_completed" if completed else "runtime_stopped"})
        def poll(self):
            return self.returncode
        def wait(self, timeout):
            if self.name == "scene":
                assert (self.root / "runtime_result.json").is_file()
                assert not (self.root / "stop.json").is_file()
                write_json(self.root / "scene/evaluation.json", {
                    "status": "completed" if completed else "incomplete",
                    "end_reason": "successful_land" if completed else "runtime_stopped",
                    "metrics": {"NE": 1.0 if completed else 31.0, "SR": int(completed), "OSR": 1, "steps_taken": 7}})
                events.append("scored")
            self.returncode = 0
            return 0
        def stop(self):
            events.append("stop_" + self.name)
            self.returncode = 0
    monkeypatch.setattr(batch, "ManagedProcess", FakeProcess)
    monkeypatch.setattr(batch, "occupied_sim_ports", lambda: [])
    monkeypatch.setattr(batch, "pending_px4_tcp_restart", lambda: False)
    config = {**batch.DEFAULTS, "record": False, "preview": False, "response_language": "zh"}
    result = batch.run_case(config, episodes()[0], 0, tmp_path / "attempt", {"PATH": "/usr/bin"})
    assert (tmp_path / "attempt/original_instruction.txt").read_text().strip() == "go 0"
    assert (tmp_path / "attempt/instruction.txt").read_text().startswith("go 0\n\n请用中文回答")
    assert "999" not in (tmp_path / "attempt/instruction.txt").read_text()
    assert result["end_reason"] == ("successful_land" if completed else "runtime_stopped")
    assert result["SR"] == int(completed)
    assert result["OSR"] == 1 and result["steps_taken"] == 7
    assert not result["halt_batch"]
    assert events.index("scored") < events.index("stop_scene")


def test_evaluation_finishes_before_runtime_without_false_timeout(tmp_path, monkeypatch):
    """正常降落先被评分器确认时，停止标志不得误记为超时。"""
    class FakeProcess:
        def __init__(self, command, log_path, **kwargs):
            self.name = log_path.stem
            self.root = log_path.parent
            self.process = self
            self.returncode = None
            log_path.write_text("evaluation_ready:\n" if self.name == "scene" else
                                "Ready for takeoff!\n" if self.name == "px4" else "")
            if self.name == "agent":
                write_json(self.root / "scene/evaluation.json", {"status": "completed",
                    "end_reason": "successful_land", "metrics": {"NE": 1.0, "SR": 1, "OSR": 1, "steps_taken": 4}})
        def poll(self):
            return self.returncode
        def wait(self, timeout):
            if self.name == "agent":
                assert not (self.root / "stop.json").exists()
                write_json(self.root / "runtime_result.json", {"status": "completed", "end_reason": "runtime_completed"})
            self.returncode = 0
        def stop(self):
            self.returncode = 0
    monkeypatch.setattr(batch, "ManagedProcess", FakeProcess)
    monkeypatch.setattr(batch, "occupied_sim_ports", lambda: [])
    monkeypatch.setattr(batch, "pending_px4_tcp_restart", lambda: False)
    config = {**batch.DEFAULTS, "record": False, "preview": False}
    root = tmp_path / "attempt"
    result = batch.run_case(config, episodes()[0], 0, root, {"PATH": "/usr/bin"})
    assert result["end_reason"] == "successful_land" and result["SR"] == 1
    assert result["runtime_status"] == "completed" and not result["halt_batch"]
    assert json.loads((root / "stop.json").read_text())["end_reason"] == "successful_land"


def test_missing_final_report_recovers_saved_checkpoint(tmp_path, monkeypatch):
    """评分器提前退出时保留已保存的指标，但不能恢复成成功。"""
    class FakeProcess:
        def __init__(self, command, log_path, **kwargs):
            self.name = log_path.stem
            self.root = log_path.parent
            self.process = self
            log_path.write_text("evaluation_ready:\n" if self.name == "scene" else
                                "Ready for takeoff!\n" if self.name == "px4" else "", encoding="utf-8")
            if self.name == "scene":
                write_json(self.root / "scene/evaluation_progress.json", {
                    "status": "incomplete", "end_reason": "running",
                    "metrics": {"NE": 12.0, "SR": 1, "OSR": 1, "steps_taken": 4}})
        def poll(self):
            return 1 if self.name == "agent" else None
        def wait(self, timeout):
            return 0
        def stop(self):
            pass
    monkeypatch.setattr(batch, "ManagedProcess", FakeProcess)
    monkeypatch.setattr(batch, "occupied_sim_ports", lambda: [])
    monkeypatch.setattr(batch, "pending_px4_tcp_restart", lambda: False)
    config = {**batch.DEFAULTS, "record": False, "preview": False}
    root = tmp_path / "attempt"
    result = batch.run_case(config, episodes()[0], 0, root, {"PATH": "/usr/bin"})
    report = json.loads((root / "scene/evaluation.json").read_text())
    assert report["recovered_from_checkpoint"] is True
    assert result["end_reason"] == "agent_process_exited"
    assert result["NE"] == 12.0 and result["SR"] == 0
    assert result["OSR"] == 1 and result["steps_taken"] == 4


@pytest.mark.parametrize("end_reason", ["runtime_stopped", "case_timeout"])
def test_repeats_whole_selection_and_advances_after_failure(tmp_path, monkeypatch, end_reason):
    write_json(tmp_path / "val_unseen.json", {"episodes": episodes()})
    config = {**batch.DEFAULTS, "cases": "2,0", "repeat": 2, "data_root": str(tmp_path),
              "output_root": str(tmp_path / "batches"), "record": False, "preview": False}
    monkeypatch.setattr(batch, "read_config", lambda args: config)
    monkeypatch.setattr(batch, "validate_assets", lambda *args: None)
    monkeypatch.setattr(batch, "ros_environment", lambda *args: {})
    monkeypatch.setattr(batch.subprocess, "run", lambda *args, **kwargs: SimpleNamespace(stdout=b"", stderr=b""))
    order = []
    def run_case(config, episode, index, root, env):
        order.append(index)
        return {"case_index": index, "scene_id": int(episode["scene_id"]), "episode_id": episode["episode_id"],
                "status": "incomplete", "end_reason": end_reason, "NE": 30, "SR": 0, "OSR": 1,
                "steps_taken": 2, "halt_batch": False}
    monkeypatch.setattr(batch, "run_case", run_case)
    assert batch.main([]) == 0
    assert order == [2, 0, 2, 0]
    summary = json.loads(next((tmp_path / "batches").glob("*/summary.json")).read_text())
    assert summary["finished_attempts"] == 4 and summary["overall"]["SR"] == 0


def test_video_encoder_creates_playable_h264_without_simulation(tmp_path):
    from drone_harness.testing.video import VideoEncoder, ffmpeg_executable
    try:
        ffmpeg_executable()
    except RuntimeError:
        pytest.skip("尚未安装 FFmpeg")
    path = tmp_path / "test.mp4"
    encoder = VideoEncoder(path, 160, 120, 5, 100)
    for i in range(10):
        frame = np.zeros((120, 160, 3), np.uint8)
        frame[:, i * 10:i * 10 + 20] = 255
        encoder.write(frame)
    encoder.close()
    check = subprocess.run([ffmpeg_executable(), "-v", "error", "-i", str(path), "-f", "null", "-"], capture_output=True)
    assert check.returncode == 0, check.stderr
    assert path.stat().st_size > 1000


def test_recorder_renders_received_frames_and_tool_state_offline(tmp_path, monkeypatch):
    """用内存相机消息验证真正的叠加与编码路径，不启动 ROS 或窗口。"""
    import sys
    import time
    from drone_harness.testing import recording
    from drone_harness.testing.video import ffmpeg_executable
    try:
        ffmpeg_executable()
    except RuntimeError:
        pytest.skip("尚未安装 FFmpeg")
    write_json(tmp_path / "case_identity.json", {"case_index": 351, "scene_id": 24})
    (tmp_path / "console_timeline.jsonl").touch()
    session = tmp_path / "agent_logs/session_test"
    session.mkdir(parents=True)
    (session / "task_state.jsonl").write_text(json.dumps({"current_phase": "tool_running",
        "active_tool_name": "forward", "active_tool_arguments": {"distance_m": 5}}) + "\n")
    calls = []
    class Node:
        def create_subscription(self, msg_type, topic, callback, qos):
            self.receive = callback
        def destroy_node(self):
            calls.append("destroyed")
    node = Node()
    count = 0
    def spin_once(node, timeout_sec):
        nonlocal count
        frame = np.zeros((120, 160, 3), np.uint8)
        frame[:, count:count + 20, 1] = 200
        node.receive(SimpleNamespace(encoding="bgr8", data=frame.tobytes(), step=480, height=120, width=160))
        count += 1
        time.sleep(0.02)
        if count == 20:
            # 最后一批工具结果与停止信号同时到来，也必须进入最后一帧。
            (tmp_path / "console_timeline.jsonl").write_text(json.dumps({
                "captured_monotonic_ns": time.monotonic_ns(), "console_closed": True,
                "text": "state> tool_failed forward error=MOVE_TIMEOUT\nagent> 本轮停止。\n"}) + "\n")
            write_json(tmp_path / "stop.json", {"end_reason": "offline_test_end"})
    fake_ros = SimpleNamespace(init=lambda **kwargs: None, create_node=lambda name: node,
                              spin_once=spin_once, ok=lambda: True, shutdown=lambda: None)
    monkeypatch.setitem(sys.modules, "rclpy", fake_ros)
    monkeypatch.setitem(sys.modules, "rclpy.qos", SimpleNamespace(qos_profile_sensor_data=object()))
    monkeypatch.setitem(sys.modules, "rclpy.signals", SimpleNamespace(SignalHandlerOptions=SimpleNamespace(NO=0)))
    monkeypatch.setitem(sys.modules, "sensor_msgs", SimpleNamespace())
    monkeypatch.setitem(sys.modules, "sensor_msgs.msg", SimpleNamespace(Image=object))
    import drone_harness.config.loader as loader
    monkeypatch.setattr(loader, "load_profile", lambda name: SimpleNamespace(ros=SimpleNamespace(camera_scene_topic="/test")))
    assert recording.main(["--run-dir", str(tmp_path), "--width", "160", "--record"]) == 0
    metadata = json.loads((tmp_path / "recording.json").read_text())
    assert metadata["frames"] >= 2 and metadata["bytes"] > 1000 and metadata["error"] is None
    assert (tmp_path / "recorder_ready.json").is_file() and "destroyed" in calls
    frames = [json.loads(line) for line in (tmp_path / "frame_timeline.jsonl").read_text().splitlines()]
    assert frames[-1]["console_offset"] == (tmp_path / "console_timeline.jsonl").stat().st_size


@pytest.mark.parametrize("mode", ["normal", "crash", "timeout"])
def test_owned_process_pipeline_advances_after_runtime_exit(tmp_path, monkeypatch, mode):
    """用真实子进程替代五个组件，检查退出信号与结果持久化的实际时序。"""
    from drone_harness.testing.batch_support import ManagedProcess
    code = '''
import json,sys,time
from pathlib import Path
root=Path(sys.argv[1]); name=sys.argv[2]
if name == "scene":
    print("evaluation_ready:", flush=True)
    while not (root/"stop.json").exists() and not (root/"runtime_result.json").exists(): time.sleep(.02)
    source = root/"stop.json" if (root/"stop.json").exists() else root/"runtime_result.json"
    reason=json.loads(source.read_text())["end_reason"]
    (root/"scene").mkdir(exist_ok=True)
    (root/"scene/evaluation.json").write_text(json.dumps({"status":"incomplete","end_reason":reason,
        "metrics":{"NE":42,"SR":0,"OSR":1,"steps_taken":3}}))
elif name == "agent":
    time.sleep(.1)
    if sys.argv[3] == "normal":
        (root/"runtime_result.json").write_text(json.dumps({"status":"incomplete","end_reason":"runtime_stopped"}))
    elif sys.argv[3] == "timeout":
        time.sleep(60)
else:
    if name == "px4": print("Ready for takeoff!", flush=True)
    time.sleep(60)
'''
    children = []
    def spawn(command, log_path, **kwargs):
        process = ManagedProcess(["/usr/bin/python3", "-c", code, str(log_path.parent), log_path.stem,
                                  mode], log_path,
                                 env={"PATH": "/usr/bin"}, cwd=tmp_path)
        children.append(process)
        return process
    monkeypatch.setattr(batch, "ManagedProcess", spawn)
    monkeypatch.setattr(batch, "occupied_sim_ports", lambda: [])
    monkeypatch.setattr(batch, "pending_px4_tcp_restart", lambda: False)
    config = {**batch.DEFAULTS, "record": False, "preview": False, "case_timeout_s": 1 if mode == "timeout" else 3}
    result = batch.run_case(config, episodes()[0], 0, tmp_path / "attempt", {"PATH": "/usr/bin"})
    assert result["end_reason"] == {"normal": "runtime_stopped", "crash": "agent_process_exited", "timeout": "case_timeout"}[mode]
    assert result["NE"] == 42 and result["OSR"] == 1 and result["steps_taken"] == 3
    assert not result["halt_batch"]
    assert all(process.process.poll() is not None for process in children)
