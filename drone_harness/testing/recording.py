"""订阅实际 ROS 相机画面，同步显示原始工具输出并保存视频。"""

from __future__ import annotations

import argparse
import json
import shutil
import signal
import threading
import time
from pathlib import Path

from drone_harness.testing.batch_support import write_json


class EventTail:
    """增量读取日志完整行，不反复加载整个长任务文件。"""

    def __init__(self, path: Path):
        self.path = path
        self.offset = 0
        self.pending = b""

    def read(self) -> list[dict]:
        """保留未写完的行，下一次刷新继续读取。"""
        if not self.path.is_file():
            return []
        with self.path.open("rb") as handle:
            handle.seek(self.offset)
            chunk = handle.read()
            self.offset = handle.tell()
        lines = (self.pending + chunk).split(b"\n")
        self.pending = lines.pop()
        result = []
        for line in lines:
            try:
                value = json.loads(line)
                if isinstance(value, dict):
                    result.append(value)
            except (ValueError, UnicodeDecodeError):
                continue
        return result


def main(argv: list[str] | None = None) -> int:
    """相机后台接收，主线程显示，编码线程按时采样同一份同步合成图。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--fps", type=float, default=20)
    parser.add_argument("--preview-fps", type=float, default=20)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--bitrate-kbps", type=int, default=500)
    parser.add_argument("--max-mb", type=int, default=512)
    parser.add_argument("--min-free-gb", type=float, default=5)
    parser.add_argument("--preview", action="store_true")
    parser.add_argument("--record", action="store_true")
    args = parser.parse_args(argv)
    import cv2
    import numpy as np
    import rclpy
    from copy import copy
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import Image
    from drone_harness.config.loader import load_profile
    from drone_harness.testing.recording_display import AsyncVideoWriter, ConsolePanel, FrameBuffer, TextTail

    root = args.run_dir
    stop = root / "stop.json"
    video = root / "flight.mp4"
    info = json.loads((root / "case_identity.json").read_text())
    topic = load_profile("sim").ros.camera_scene_topic
    running = True
    writer = node = receiver = panel = None
    camera = FrameBuffer()
    receive_stop = threading.Event()
    camera_error = []
    raw_console = TextTail(root / "agent.log")
    console_timeline = EventTail(root / "console_timeline.jsonl")
    state_tail = None
    state = {"current_phase": "waiting"}
    console_offset = 0
    console_captured_ns = 0
    console_closed = False
    canvas = None
    console_lags = []
    frame_ages = []
    render_times = []
    preview_count = 0
    composed_count = 0
    first_frame_time = None
    started = time.monotonic()
    error = None
    title = f"case {info['case_index']} | camera + agent.log | N: next | Q: stop"

    def stop_requested(signum, frame) -> None:
        """外部停止只触发正常封装，不在信号回调里操作 ROS。"""
        nonlocal running
        running = False

    def read_console() -> bool:
        """正常刷新和退出收尾共用增量读取，不漏掉最后的工具结果。"""
        nonlocal console_offset, console_captured_ns, console_closed
        fresh = False
        events = console_timeline.read()
        if console_timeline.path.is_file():
            for event in events:
                text = event.get("text", "")
                panel.append(text)
                if text:
                    console_captured_ns = int(event.get("captured_monotonic_ns", 0))
                    fresh = True
                console_closed = console_closed or event.get("console_closed", False)
            console_offset = console_timeline.offset
        else:
            panel.append(raw_console.read())
            console_offset = raw_console.offset
        return fresh

    def receive(message: Image) -> None:
        """保存最新彩色帧及源时间，后台不绘字、不编码。"""
        if message.encoding not in {"bgr8", "rgb8", "bgra8", "rgba8"}:
            return
        channels = 4 if "a8" in message.encoding else 3
        raw = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
        frame = raw[:, :message.width * channels].reshape(message.height, message.width, channels)
        frame = frame[:, :, [2, 1, 0]] if message.encoding.startswith("rgb") else frame[:, :, :3]
        stamp = getattr(getattr(message, "header", None), "stamp", None)
        source_ns = int(getattr(stamp, "sec", 0)) * 1_000_000_000 + int(getattr(stamp, "nanosec", 0))
        camera.put(frame, source_ns)

    def receive_loop() -> None:
        """相机接收与界面、编码相互独立，只保留最新消息。"""
        try:
            while not receive_stop.is_set() and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.01)
        except Exception as exc:
            if not receive_stop.is_set():
                camera_error.append(f"{type(exc).__name__}: {exc}")

    previous_signals = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}
    for sig in previous_signals:
        signal.signal(sig, stop_requested)
    try:
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = rclpy.create_node("drone_harness_batch_recording")
        qos = copy(qos_profile_sensor_data)
        if hasattr(qos, "depth"):
            qos.depth = 1
        node.create_subscription(Image, topic, receive, qos)
        receiver = threading.Thread(target=receive_loop, daemon=True)
        receiver.start()
        if args.preview:
            cv2.namedWindow(title, cv2.WINDOW_NORMAL)
        period = 1 / max(args.preview_fps if args.preview else 0, args.fps if args.record else 0, 1)
        next_update = started
        next_disk_check = started
        while running and not stop.is_file():
            now = time.monotonic()
            if camera_error:
                raise RuntimeError(camera_error[0])
            if writer is not None and writer.error:
                raise RuntimeError(writer.error)
            if now >= next_disk_check:
                next_disk_check = now + 1
                reason = None
                if shutil.disk_usage(root).free < args.min_free_gb * 1024**3:
                    reason = "disk_low"
                if video.exists() and video.stat().st_size >= args.max_mb * 1024**2:
                    reason = "video_size_limit"
                if reason:
                    write_json(stop, {"end_reason": reason})
                    break
            if now < next_update:
                time.sleep(min(next_update - now, 0.005))
                continue
            next_update = max(next_update + period, now)
            snapshot = camera.get()
            if snapshot is None:
                continue
            render_started = time.perf_counter()
            frame, frame_info = snapshot
            image_height = int(frame.shape[0] * args.width / frame.shape[1]) // 2 * 2
            panel_height = max(600, image_height)
            panel_width = max(640, args.width)
            if panel is None:
                panel = ConsolePanel(panel_width, panel_height)
                panel.append("等待原始 agent 输出；上下页键回看，End 回到实时。\n")
                if args.preview:
                    cv2.resizeWindow(title, args.width + panel_width, panel_height + 64)
            fresh_console = read_console()
            if state_tail is None:
                sessions = list((root / "agent_logs").glob("session_*"))
                if len(sessions) == 1:
                    state_tail = EventTail(sessions[0] / "task_state.jsonl")
            if state_tail:
                for event in state_tail.read():
                    state = event
            canvas = np.zeros((panel_height + 64, args.width + panel_width, 3), np.uint8)
            canvas[64:64 + image_height, :args.width] = cv2.resize(frame, (args.width, image_height))
            canvas[64:, args.width:] = panel.render(live=True)
            composed_ns = time.monotonic_ns()
            age_ms = max(0, (composed_ns - frame_info["camera_receive_monotonic_ns"]) / 1e6)
            frame_ages.append(age_ms)
            if fresh_console and console_captured_ns:
                console_lags.append(max(0, (composed_ns - console_captured_ns) / 1e6))
            caption = (f"case {info['case_index']} / scene {info['scene_id']} | "
                       f"LIVE {args.preview_fps:g} / REC {args.fps:g} FPS | camera {age_ms:.0f} ms")
            cv2.putText(canvas, caption, (8, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (230, 230, 230), 1)
            cv2.putText(canvas, "N: next case   Q/Esc: stop batch   PgUp/PgDn: history   End: live",
                        (8, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (160, 220, 180), 1)
            if first_frame_time is None:
                first_frame_time = time.monotonic()
            if args.record and writer is None:
                writer = AsyncVideoWriter(video, canvas.shape[1], canvas.shape[0], args.fps, args.bitrate_kbps)
            if writer is not None:
                writer.submit(canvas, {**frame_info, "console_offset": console_offset,
                    "console_capture_monotonic_ns": console_captured_ns, "composed_monotonic_ns": composed_ns})
            composed_count += 1
            render_times.append((time.perf_counter() - render_started) * 1000)
            if args.preview:
                preview = canvas
                if panel.offset:
                    preview = canvas.copy()
                    preview[64:, args.width:] = panel.render()
                cv2.imshow(title, preview)
                preview_count += 1
                key = cv2.waitKeyEx(1)
                if key in (ord("n"), ord("q"), 27) or cv2.getWindowProperty(title, cv2.WND_PROP_VISIBLE) < 1:
                    write_json(stop, {"end_reason": "runtime_user_exit" if key == ord("n") else "batch_user_stop"})
                    break
                if key in (65365, 0x210000):
                    panel.scroll(10)
                elif key in (65366, 0x220000):
                    panel.scroll(-10)
                elif key in (65367, 0x230000):
                    panel.follow()
            if not (root / "recorder_ready.json").is_file():
                write_json(root / "recorder_ready.json", {"topic": topic, "record": args.record,
                    "preview": args.preview, "preview_fps": args.preview_fps, "video_fps": args.fps})
    except Exception as exc:
        error = f"{type(exc).__name__}: {exc}"
        write_json(stop, {"end_reason": "recorder_error"})
        print(error, flush=True)
    finally:
        if panel is not None and canvas is not None and writer is not None and not writer.error:
            # 只等终端收尾，不再继续导航；最多两秒，完整原始日志始终另存。
            drain_deadline = time.monotonic() + 2
            while console_timeline.path.is_file() and not console_closed and time.monotonic() < drain_deadline:
                read_console()
                if not console_closed:
                    time.sleep(0.01)
            read_console()
            snapshot = camera.get()
            if snapshot is not None:
                frame, frame_info = snapshot
                canvas = canvas.copy()
                canvas[64:64 + image_height, :args.width] = cv2.resize(frame, (args.width, image_height))
                canvas[64:, args.width:] = panel.render(live=True)
                composed_ns = time.monotonic_ns()
                age_ms = max(0, (composed_ns - frame_info["camera_receive_monotonic_ns"]) / 1e6)
                cv2.rectangle(canvas, (0, 0), (canvas.shape[1], 31), (0, 0, 0), -1)
                cv2.putText(canvas, f"case {info['case_index']} | FINAL | camera {age_ms:.0f} ms",
                            (8, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (230, 230, 230), 1)
                writer.submit(canvas, {**frame_info, "console_offset": console_offset,
                    "console_capture_monotonic_ns": console_captured_ns, "composed_monotonic_ns": composed_ns})
        receive_stop.set()
        if receiver is not None:
            receiver.join(timeout=2)
        if writer is not None:
            writer.close()
            error = writer.error or error
            if writer.error:
                write_json(stop, {"end_reason": "recorder_error"})
        if args.preview:
            cv2.destroyAllWindows()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        elapsed = time.monotonic() - started
        live_elapsed = max(0.001, time.monotonic() - first_frame_time) if first_frame_time else 0
        def stats(values: list[float]) -> dict:
            """统计实际延迟，不用配置帧率替代真实刷新率。"""
            if not values:
                return {}
            ordered = sorted(values)
            return {"mean": round(sum(values) / len(values), 2), "p95": round(ordered[int((len(values)-1)*0.95)], 2),
                    "max": round(max(values), 2)}
        write_json(root / "recording.json", {"codec": "h264/libx264", "fps": args.fps,
            "frames": writer.frames if writer else 0, "bitrate_kbps": args.bitrate_kbps,
            "elapsed_s": round(elapsed, 2), "error": error,
            "bytes": video.stat().st_size if video.exists() else 0, "topic": topic,
            "preview_refresh_hz": args.preview_fps,
            "actual_preview_fps": round(preview_count/live_elapsed, 2) if live_elapsed else 0,
            "actual_composition_fps": round(composed_count/live_elapsed, 2) if live_elapsed else 0,
            "actual_video_fps": round(writer.frames/live_elapsed, 2) if writer and live_elapsed else 0,
            "camera_received_frames": camera.sequence, "frame_age_ms": stats(frame_ages),
            "console_display_lag_ms": stats(console_lags), "render_ms": stats(render_times),
            "note": "原始终端追加、自动换行；相机与日志共用合成帧，逐帧对应关系见 frame_timeline.jsonl。"})
        for sig, handler in previous_signals.items():
            signal.signal(sig, handler)
    return 1 if error else 0


if __name__ == "__main__":
    raise SystemExit(main())
