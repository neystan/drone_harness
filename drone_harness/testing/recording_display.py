"""复用原始终端输出，缓存文字面板并独立编码最新的同步画面。"""

from __future__ import annotations

import codecs
import re
import threading
import time
import unicodedata
from collections import deque
from pathlib import Path

from drone_harness.testing.video import VideoEncoder

ANSI = re.compile(r'\x1b\[[0-?]*[ -/]*[@-~]')


def wrap_cells(text: str, columns: int) -> list[str]:
    """按等宽字符单元换行，中文占两格，不截断长参数。"""
    rows, current, width = [], [], 0
    for character in text.expandtabs(4):
        size = 0 if unicodedata.combining(character) else 2 if unicodedata.east_asian_width(character) in {'W', 'F'} else 1
        if current and width + size > columns:
            rows.append(''.join(current))
            current, width = [], 0
        current.append(character)
        width += size
    if current:
        rows.append(''.join(current))
    return rows or ['']


class TextTail:
    """增量读取原始终端字节，避免跨块的中文字符被替换。"""

    def __init__(self, path: Path):
        self.path, self.offset = path, 0
        self.decoder = codecs.getincrementaldecoder('utf-8')(errors='replace')

    def read(self) -> str:
        """读取新增输出，不要求最后一行已有换行符。"""
        if not self.path.is_file():
            return ''
        with self.path.open('rb') as handle:
            handle.seek(self.offset)
            chunk = handle.read()
            self.offset = handle.tell()
        return self.decoder.decode(chunk)


class ConsoleHistory:
    """保留可回看的终端行；完整无限历史仍由 agent.log 保存。"""

    def __init__(self, columns: int, max_lines: int = 10000):
        self.columns = columns
        self.lines: deque[str] = deque(maxlen=max_lines)
        self.partial = ''

    def append(self, text: str) -> None:
        """追加全部内容，保留未换行的输出片段。"""
        parts = (self.partial + ANSI.sub('', text).replace('\r', '\n')).split('\n')
        self.partial = parts.pop()
        for line in parts:
            self.lines.extend(wrap_cells(line, self.columns))

    @property
    def line_count(self) -> int:
        """返回当前可滚动的视觉行数。"""
        return len(self.lines) + (len(wrap_cells(self.partial, self.columns)) if self.partial else 0)

    def visible(self, rows: int, offset: int = 0) -> list[str]:
        """offset 为距离最新输出的回看行数。"""
        lines = list(self.lines)
        if self.partial:
            lines += wrap_cells(self.partial, self.columns)
        end = max(0, len(lines) - offset)
        return lines[max(0, end - rows):end]


class ConsolePanel:
    """仅日志更新或滚动时绘字，普通相机帧直接复用缓存。"""

    def __init__(self, width: int, height: int):
        from PIL import ImageFont
        font_path = Path('/usr/share/fonts/opentype/noto/NotoSansMonoCJK-Regular.ttc')
        if not font_path.is_file():
            font_path = Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
        if font_path.is_file():
            try:
                # 系统 CJK 字体集合的第七索引是简体中文等宽字体，避免把普通 M 宽度当作单元。
                self.font = ImageFont.truetype(str(font_path), 16, index=7 if 'Mono' not in font_path.name else 0)
            except OSError:
                self.font = ImageFont.truetype(str(font_path), 16)
        else:
            self.font = ImageFont.load_default()
        self.width, self.height = width, height
        self.line_height = 22
        self.history = ConsoleHistory(max(1, int((width - 20) / self.font.getlength('M'))))
        self.offset = 0
        self.cached = None

    def append(self, text: str) -> None:
        """新内容追加；回看时不自动跳回最新位置。"""
        if not text:
            return
        before = self.history.line_count
        self.history.append(text)
        if self.offset:
            self.offset += max(0, self.history.line_count - before)
        self.cached = None

    def scroll(self, rows: int) -> None:
        """滚动只影响预览，不干预导航或录像的实时追尾。"""
        self.offset = max(0, min(self.history.line_count - 1, self.offset + rows))
        self.cached = None

    def follow(self) -> None:
        """回到最新输出。"""
        self.offset = 0
        self.cached = None

    def render(self, *, live: bool = False):
        """生成当前文字面板；录像始终呈现最新输出。"""
        import numpy as np
        from PIL import Image, ImageDraw
        if self.cached is not None and not (live and self.offset):
            return self.cached
        image = Image.new('RGB', (self.width, self.height), (15, 18, 22))
        draw = ImageDraw.Draw(image)
        rows = self.history.visible(self.height // self.line_height - 2, 0 if live else self.offset)
        title = '实时 agent.log | PageUp/PageDown 回看 | End 追尾'
        draw.text((8, 6), title, font=self.font, fill=(140, 200, 245))
        for number, line in enumerate(rows):
            color = (245, 245, 245)
            if 'tool_failed' in line or 'ERROR' in line:
                color = (255, 145, 145)
            elif line.startswith(('tool>', 'plan>')):
                color = (140, 225, 175)
            elif line.startswith('state>'):
                color = (150, 205, 255)
            draw.text((8, 34 + number * self.line_height), line, font=self.font, fill=color)
        frame = np.asarray(image)[:, :, ::-1].copy()
        if not (live and self.offset):
            self.cached = frame
        return frame


class FrameBuffer:
    """相机后台只覆盖最新帧，不让预览排队播放旧画面。"""

    def __init__(self):
        self.lock = threading.Lock()
        self.latest = None
        self.sequence = 0
        self.first_received_ns = 0

    def put(self, image, source_timestamp_ns: int = 0) -> None:
        """记录实际接收时刻与源相机时间戳。"""
        received = time.monotonic_ns()
        with self.lock:
            self.sequence += 1
            if not self.first_received_ns:
                self.first_received_ns = received
            self.latest = (image.copy(), {'camera_sequence': self.sequence,
                'camera_receive_monotonic_ns': received, 'camera_source_timestamp_ns': source_timestamp_ns})

    def get(self):
        """取得同一张图和对应时间元数据。"""
        with self.lock:
            return self.latest


class AsyncVideoWriter:
    """编码线程按目标帧率采样最新合成图，不阻塞实时窗口。"""

    def __init__(self, path: Path, width: int, height: int, fps: float, bitrate: int):
        self.encoder = VideoEncoder(path, width, height, fps, bitrate)
        self.timeline_path = path.with_name('frame_timeline.jsonl')
        self.fps, self.frames, self.error = fps, 0, None
        self.lock = threading.Lock()
        self.latest = None
        self.revision = 0
        self.stopping = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def submit(self, frame, metadata: dict) -> None:
        """最新图替换待采样图；画面和日志元数据一起交给编码器。"""
        with self.lock:
            self.revision += 1
            self.latest = (frame, dict(metadata), self.revision)

    def _run(self) -> None:
        """记录每个已编码画面使用的相机帧与控制台事件。"""
        import json
        deadline = None
        written_revision = 0
        try:
            with self.timeline_path.open('w', encoding='utf-8') as timeline:
                while True:
                    with self.lock:
                        latest = self.latest
                    if self.stopping.is_set() and (latest is None or latest[2] == written_revision):
                        break
                    if latest is None:
                        self.stopping.wait(0.005)
                        continue
                    if deadline is None:
                        deadline = time.monotonic()
                    frame, metadata, revision = latest
                    self.encoder.write(frame)
                    timeline.write(json.dumps({**metadata, 'frame_index': self.frames,
                        'pts_s': self.frames / self.fps, 'encoded_monotonic_ns': time.monotonic_ns()}) + '\n')
                    timeline.flush()
                    self.frames += 1
                    written_revision = revision
                    deadline += 1 / self.fps
                    self.stopping.wait(max(0, deadline - time.monotonic()))
        except Exception as exc:
            self.error = f'{type(exc).__name__}: {exc}'
        finally:
            try:
                self.encoder.close()
            except Exception as exc:
                self.error = f'{type(exc).__name__}: {exc}'

    def close(self) -> None:
        """有界等待编码收尾；不删除任何视频或轨迹。"""
        self.stopping.set()
        self.thread.join(timeout=12)
        if self.thread.is_alive():
            self.encoder.process.kill()
            self.thread.join(timeout=2)
            self.error = '编码线程收尾超时'
