"""封装线程安全的同步消息队列。"""

from __future__ import annotations

from queue import Empty, Queue
from typing import Generic, TypeVar

T = TypeVar("T")


class SyncMessageQueue(Generic[T]):
    """提供同步场景下的消息发布和消费。"""

    def __init__(self) -> None:
        self._queue: Queue[T] = Queue()

    def publish(self, message: T) -> None:
        """发布一条消息到队列。"""
        self._queue.put(message)

    def consume(self) -> T:
        """阻塞等待并消费一条消息。"""
        return self._queue.get()

    def try_consume(self) -> T | None:
        """非阻塞消费一条消息，队列为空时返回 None。"""
        try:
            return self._queue.get_nowait()
        except Empty:
            return None

    def has_pending(self) -> bool:
        """判断当前是否存在待处理消息。"""
        return not self._queue.empty()
