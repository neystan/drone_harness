"""用 FFmpeg 将实时 BGR 帧编码为限码率 H.264。"""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path


def ffmpeg_executable() -> str:
    """优先使用系统命令，否则使用用户目录安装的 FFmpeg 二进制。"""
    executable = shutil.which("ffmpeg")
    if executable:
        return executable
    try:
        import imageio_ffmpeg
        return imageio_ffmpeg.get_ffmpeg_exe()
    except ImportError as exc:
        raise RuntimeError("缺少 FFmpeg；可安装 imageio-ffmpeg 的用户目录二进制") from exc


class VideoEncoder:
    """只编码传入的真实画面；分片 MP4 在异常退出后也能保留已完成片段。"""

    def __init__(self, path: Path, width: int, height: int, fps: float, bitrate_kbps: int):
        self.log = path.with_suffix(".ffmpeg.log").open("wb")
        try:
            self.process = subprocess.Popen([ffmpeg_executable(), "-hide_banner", "-loglevel", "warning", "-nostdin",
                "-f", "rawvideo", "-pixel_format", "bgr24", "-video_size", f"{width}x{height}",
                "-framerate", str(fps), "-i", "pipe:0", "-an", "-c:v", "libx264", "-threads", "2",
                "-preset", "veryfast", "-tune", "zerolatency", "-pix_fmt", "yuv420p",
                "-b:v", f"{bitrate_kbps}k", "-maxrate", f"{bitrate_kbps}k", "-bufsize", f"{bitrate_kbps * 2}k",
                "-g", str(max(1, int(fps * 2))), "-sc_threshold", "0",
                "-movflags", "+frag_keyframe+empty_moov+default_base_moof", "-n", str(path)],
                stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=self.log)
        except BaseException:
            self.log.close()
            raise

    def write(self, frame) -> None:
        """编码失败立刻抛出，避免静默丢失录像。"""
        if self.process.poll() is not None:
            raise RuntimeError("FFmpeg 已提前退出，详见 flight.ffmpeg.log")
        self.process.stdin.write(frame.tobytes())

    def close(self) -> None:
        """关闭输入并等待最后片段落盘，不删除任何已有视频。"""
        try:
            try:
                self.process.stdin.close()
            except BrokenPipeError:
                pass
            try:
                code = self.process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=2)
                raise RuntimeError("FFmpeg 收尾超时")
            if code not in (0, 255):
                raise RuntimeError(f"FFmpeg 编码失败：{code}")
        finally:
            self.log.close()
