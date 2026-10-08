"""验证完整终端历史、低延迟画面与独立编码，不启动仿真。"""

import json
import subprocess
import time
from pathlib import Path

import numpy as np

from drone_harness.testing.recording_display import ConsoleHistory, TextTail, wrap_cells
from drone_harness.testing.batch_support import ManagedProcess


def test_wrap_preserves_long_chinese_and_tool_arguments():
    text = 'agent> 白色建筑停车场。tool> ' + json.dumps({'subgoals': ['目标' * 50] * 5}, ensure_ascii=False)
    lines = wrap_cells(text, 40)
    assert ''.join(lines) == text
    assert len(lines) > 5
    assert all(sum(2 if ord(char) > 127 else 1 for char in line) <= 40 for line in lines)


def test_console_history_appends_and_can_scroll_back():
    history = ConsoleHistory(columns=40, max_lines=100)
    history.append('\x1b[32mstate> thinking\x1b[0m\nagent> 观察白色建筑\n')
    history.append('tool> calling forward args={"distance_m":10}\n')
    assert ''.join(history.visible(2)) == 'tool> calling forward args={"distance_m":10}'
    assert any('观察白色建筑' in line for line in history.visible(10))
    assert 'state> thinking' in history.visible(1, offset=history.line_count - 1)
    assert all('\x1b' not in line for line in history.visible(10))


def test_text_tail_preserves_partial_utf8_and_last_line(tmp_path):
    path = tmp_path / 'agent.log'
    encoded = 'agent> 观察\n'.encode()
    path.write_bytes(encoded[:-2])
    tail = TextTail(path)
    first = tail.read()
    with path.open('ab') as handle:
        handle.write(encoded[-2:])
    assert first + tail.read() == 'agent> 观察\n'
    assert tail.read() == ''


def test_owned_process_captures_original_console_with_timestamp(tmp_path):
    code = 'print("agent> 观察目标", flush=True)'
    process = ManagedProcess(['/usr/bin/python3', '-c', code], tmp_path / 'agent.log',
                             env={'PATH': '/usr/bin'}, cwd=tmp_path, capture_timeline=True)
    process.process.wait(timeout=3)
    process.stop()
    assert (tmp_path / 'agent.log').read_text() == 'agent> 观察目标\n'
    events = [json.loads(line) for line in (tmp_path / 'console_timeline.jsonl').read_text().splitlines()]
    assert ''.join(event['text'] for event in events) == 'agent> 观察目标\n'
    assert all(event['captured_monotonic_ns'] > 0 for event in events)


def test_cached_log_panel_does_not_relayout_every_frame():
    from drone_harness.testing.recording_display import ConsolePanel
    panel = ConsolePanel(640, 560)
    panel.append('agent> ' + '这是长观察描述，包含道路、建筑和停车场。' * 100 + '\n')
    first = panel.render()
    started = time.perf_counter()
    for _ in range(20):
        assert panel.render() is first
    assert time.perf_counter() - started < 0.1


def test_async_encoder_saves_20fps_and_frame_timeline(tmp_path):
    from drone_harness.testing.recording_display import AsyncVideoWriter
    from drone_harness.testing.video import ffmpeg_executable
    writer = AsyncVideoWriter(tmp_path / 'flight.mp4', 160, 120, 20, 100)
    frame = np.zeros((120, 160, 3), np.uint8)
    writer.submit(frame, {'camera_sequence': 1, 'camera_receive_monotonic_ns': time.monotonic_ns(),
                          'console_offset': 42, 'composed_monotonic_ns': time.monotonic_ns()})
    time.sleep(0.3)
    writer.close()
    assert writer.error is None and writer.frames >= 4
    events = [json.loads(line) for line in (tmp_path / 'frame_timeline.jsonl').read_text().splitlines()]
    assert len(events) == writer.frames
    assert all(event['console_offset'] == 42 for event in events)
    assert events[-1]['pts_s'] == (writer.frames - 1) / 20
    check = subprocess.run([ffmpeg_executable(), '-v', 'error', '-i', str(tmp_path / 'flight.mp4'),
                            '-f', 'null', '-'], capture_output=True)
    assert check.returncode == 0, check.stderr


def test_async_encoder_drains_last_submitted_frame_on_close(tmp_path):
    from drone_harness.testing.recording_display import AsyncVideoWriter
    writer = AsyncVideoWriter(tmp_path / 'final.mp4', 160, 120, 20, 100)
    frame = np.zeros((120, 160, 3), np.uint8)
    writer.submit(frame, {'console_offset': 1})
    time.sleep(0.08)
    writer.submit(frame, {'console_offset': 999, 'final_tool_result': True})
    writer.close()
    events = [json.loads(line) for line in (tmp_path / 'frame_timeline.jsonl').read_text().splitlines()]
    assert writer.error is None
    assert events[-1]['console_offset'] == 999 and events[-1]['final_tool_result']
