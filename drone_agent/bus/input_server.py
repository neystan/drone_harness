"""提供独立输入终端到 MessageBus 的本地输入服务。"""

from __future__ import annotations

import secrets
import socketserver
import threading
from dataclasses import dataclass

from .message_bus import MessageBus


@dataclass(frozen=True)
class InputServerInfo:
    """保存输入服务连接信息。"""

    host: str
    port: int
    token: str


class InputServer:
    """在主进程中接收独立输入终端发送的用户消息。"""

    def __init__(self, message_bus: MessageBus, host: str = "127.0.0.1") -> None:
        self._message_bus = message_bus
        self._token = secrets.token_urlsafe(24)
        self._server = _ThreadedInputServer((host, 0), _InputRequestHandler)
        self._server.message_bus = message_bus
        self._server.token = self._token
        self._thread: threading.Thread | None = None

    @property
    def info(self) -> InputServerInfo:
        """返回输入终端连接主进程所需的信息。"""
        host, port = self._server.server_address
        return InputServerInfo(host=str(host), port=int(port), token=self._token)

    def start(self) -> None:
        """启动后台 socket 服务线程。"""
        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """停止输入服务并释放本地端口。"""
        self._server.shutdown()
        self._server.server_close()
        if self._thread is not None:
            self._thread.join(timeout=1.0)


class _ThreadedInputServer(socketserver.ThreadingTCPServer):
    """保存 MessageBus 和鉴权 token 的线程化 TCP 服务。"""

    allow_reuse_address = True
    daemon_threads = True

    message_bus: MessageBus
    token: str


class _InputRequestHandler(socketserver.StreamRequestHandler):
    """处理单个输入终端连接。"""

    def handle(self) -> None:
        """校验 token 后持续读取用户输入行。"""
        token = self.rfile.readline().decode("utf-8", errors="replace").strip()
        if token != self.server.token:
            self.wfile.write("AUTH_FAILED\n".encode("utf-8"))
            return
        self.wfile.write("OK\n".encode("utf-8"))
        while True:
            line = self.rfile.readline()
            if not line:
                return
            content = line.decode("utf-8", errors="replace").strip()
            if content:
                self.server.message_bus.publish_user_message(content)
