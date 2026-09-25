"""定义 drone agent 的运行时消息总线。"""

from __future__ import annotations

import time
from dataclasses import dataclass

from .queue import SyncMessageQueue


@dataclass(frozen=True)
class UserMessage:
    """保存一条用户输入消息。"""

    content: str
    created_at: float


class MessageBus:
    """提供用户输入消息的发布和消费。"""

    def __init__(self) -> None:
        self._user_messages: SyncMessageQueue[UserMessage] = SyncMessageQueue()

    def publish_user_message(self, content: str) -> None:
        """发布一条用户自然语言消息。"""
        self._user_messages.publish(UserMessage(content=content, created_at=time.time()))

    def consume_user_message(self) -> UserMessage:
        """阻塞等待并消费一条用户消息。"""
        return self._user_messages.consume()

    def get_next_user_message(self) -> UserMessage | None:
        """非阻塞消费一条用户消息。"""
        return self._user_messages.try_consume()

    def has_pending_user_message(self) -> bool:
        """判断是否存在待处理用户消息。"""
        return self._user_messages.has_pending()
