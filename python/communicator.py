"""
共通通信モジュール（送受信対応）。

双方向メッセージ通信（送信・受信・イベント待受・リクエスト/レスポンス）を提供する。
"""

from __future__ import annotations

import json
import logging
import threading
import time
from typing import Any, Callable, Iterator, Optional, Union

logger = logging.getLogger(__name__)


class BaseCommunicator:
    """双方向通信インターフェースの基底クラス。"""

    def send(self, data: Union[dict, str]) -> None:
        """データを送信する。"""
        raise NotImplementedError

    def receive(self, timeout: Optional[float] = None) -> Optional[Union[dict, str]]:
        """データを受信する（1件）。タイムアウト時は None を返す。"""
        raise NotImplementedError

    def on_message(self, callback: Callable[[Any], None]) -> None:
        """メッセージ受信時のイベントハンドラを登録する。"""
        raise NotImplementedError


class WebSocketCommunicator(BaseCommunicator):
    """
    WebSocket を用いた双方向通信クライアント。

    - 送信: send(data)
    - 単発受信: receive(timeout)
    - 要求応答: request(data, timeout) -> レスポンスデータ
    - 継続待受: on_message(handler) + start_listening() / listen()
    """

    def __init__(self, url: str = "ws://127.0.0.1:8000/ws/hint-card", timeout: float = 2.0):
        self.url = url
        self.timeout = timeout
        self._handlers: list[Callable[[Any], None]] = []
        self._listener_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._connection = None

    def _parse_payload(self, raw_data: str) -> Union[dict, str]:
        """受信文字列を JSON 辞書または文字列としてパースする。"""
        if not isinstance(raw_data, str):
            return raw_data
        try:
            return json.loads(raw_data)
        except (ValueError, json.JSONDecodeError):
            return raw_data

    def send(self, data: Union[dict, str]) -> None:
        """データを JSON 文字列化して WebSocket 送信する。"""
        try:
            from websockets.sync.client import connect
        except ImportError:
            logger.warning("websockets が無いため送信できません: %s", self.url)
            return

        payload_str = json.dumps(data, ensure_ascii=False) if isinstance(data, dict) else str(data)
        try:
            with connect(self.url, open_timeout=self.timeout, close_timeout=self.timeout) as ws:
                ws.send(payload_str)
        except Exception:  # noqa: BLE001
            logger.exception("WebSocket 送信に失敗しました: %s", self.url)

    def receive(self, timeout: Optional[float] = None) -> Optional[Union[dict, str]]:
        """
        WebSocket サーバーからメッセージを 1件受信する。
        """
        try:
            from websockets.sync.client import connect
        except ImportError:
            logger.warning("websockets が無いため受信できません: %s", self.url)
            return None

        t = timeout if timeout is not None else self.timeout
        try:
            with connect(self.url, open_timeout=t, close_timeout=t) as ws:
                raw = ws.recv(timeout=t)
                return self._parse_payload(raw)
        except Exception:  # noqa: BLE001
            logger.exception("WebSocket 受信に失敗しました: %s", self.url)
            return None

    def request(self, data: Union[dict, str], timeout: Optional[float] = None) -> Optional[Union[dict, str]]:
        """
        データを送信し、対向からの応答メッセージを 1件待受けて返す（RPC / 要求・応答パターン）。
        """
        try:
            from websockets.sync.client import connect
        except ImportError:
            logger.warning("websockets が無いため送受信できません: %s", self.url)
            return None

        t = timeout if timeout is not None else self.timeout
        payload_str = json.dumps(data, ensure_ascii=False) if isinstance(data, dict) else str(data)
        try:
            with connect(self.url, open_timeout=t, close_timeout=t) as ws:
                ws.send(payload_str)
                raw = ws.recv(timeout=t)
                return self._parse_payload(raw)
        except Exception:  # noqa: BLE001
            logger.exception("WebSocket リクエストに失敗しました: %s", self.url)
            return None

    def on_message(self, callback: Callable[[Any], None]) -> Callable[[Any], None]:
        """メッセージ受信ハンドラを登録する（デコレータとしても利用可能）。"""
        self._handlers.append(callback)
        return callback

    def listen(self) -> Iterator[Union[dict, str]]:
        """
        メッセージを順次受け取るジェネレータ（ブロッキング待受ループ）。
        """
        try:
            from websockets.sync.client import connect
        except ImportError:
            logger.warning("websockets が無いため待受できません: %s", self.url)
            return

        while not self._stop_event.is_set():
            try:
                with connect(self.url, open_timeout=self.timeout) as ws:
                    while not self._stop_event.is_set():
                        raw = ws.recv()
                        msg = self._parse_payload(raw)
                        yield msg
            except Exception:  # noqa: BLE001
                if self._stop_event.is_set():
                    break
                logger.warning("WebSocket 切断。1秒後に再接続を試みます: %s", self.url)
                time.sleep(1.0)

    def start_listening(self) -> None:
        """バックグラウンドスレッドで受信ループを開始し、登録ハンドラへメッセージを配信する。"""
        if self._listener_thread is not None and self._listener_thread.is_alive():
            return

        self._stop_event.clear()

        def _loop():
            for msg in self.listen():
                for handler in list(self._handlers):
                    try:
                        handler(msg)
                    except Exception:  # noqa: BLE001
                        logger.exception("受信ハンドラ実行中にエラーが発生しました")

        self._listener_thread = threading.Thread(target=_loop, daemon=True)
        self._listener_thread.start()

    def stop_listening(self) -> None:
        """バックグラウンド受信を停止する。"""
        self._stop_event.set()
        if self._listener_thread is not None:
            self._listener_thread.join(timeout=1.0)
            self._listener_thread = None


class CallbackCommunicator(BaseCommunicator):
    """関数コールバックを用いた送受信クラス（PyScript やテスト環境用）。"""

    def __init__(
        self,
        send_fn: Optional[Callable[[Any], None]] = None,
        receive_fn: Optional[Callable[[], Any]] = None,
    ):
        self.send_fn = send_fn
        self.receive_fn = receive_fn
        self._handlers: list[Callable[[Any], None]] = []

    def send(self, data: Union[dict, str]) -> None:
        """送信関数を呼び出す。"""
        if self.send_fn:
            try:
                self.send_fn(data)
            except Exception:  # noqa: BLE001
                logger.exception("CallbackCommunicator 送信エラー")

    def receive(self, timeout: Optional[float] = None) -> Optional[Any]:
        """受信関数を呼び出してデータを返す。"""
        if self.receive_fn:
            try:
                return self.receive_fn()
            except Exception:  # noqa: BLE001
                logger.exception("CallbackCommunicator 受信エラー")
        return None

    def on_message(self, callback: Callable[[Any], None]) -> Callable[[Any], None]:
        """受信ハンドラを登録する。"""
        self._handlers.append(callback)
        return callback

    def dispatch(self, data: Any) -> None:
        """受信データを登録済みハンドラへ通知する（テスト・シミュレータからの模擬受信）。"""
        for handler in list(self._handlers):
            try:
                handler(data)
            except Exception:  # noqa: BLE001
                logger.exception("CallbackCommunicator ハンドラエラー")


# 後方互換性エイリアス
MessageSender = BaseCommunicator
WebSocketSender = WebSocketCommunicator
CallbackSender = CallbackCommunicator
