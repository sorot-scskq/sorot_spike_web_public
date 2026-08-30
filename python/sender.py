"""
共通メッセージ送信モジュール（後方互換性エイリアス）。
送受信の両方に対応した communicator.py を再エクスポートする。
"""

try:
    from .communicator import (
        BaseCommunicator,
        CallbackCommunicator,
        CallbackSender,
        MessageSender,
        WebSocketCommunicator,
        WebSocketSender,
    )
except (ImportError, ValueError):
    from communicator import (
        BaseCommunicator,
        CallbackCommunicator,
        CallbackSender,
        MessageSender,
        WebSocketCommunicator,
        WebSocketSender,
    )

__all__ = [
    "BaseCommunicator",
    "CallbackCommunicator",
    "CallbackSender",
    "MessageSender",
    "WebSocketCommunicator",
    "WebSocketSender",
]
