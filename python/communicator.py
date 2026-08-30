"""
共通通信モジュール（送受信対応）。

双方向メッセージ通信（送信・受信・イベント待受・リクエスト/レスポンス）を提供する。

【トランスポートが 2つある理由】
つなぐ相手が実機とシミュレータで違うので、下回りだけを差し替えられるように
BaseCommunicator でそろえてある。上に載る処理（ヒントカードの読み取りなど）は
どちらで動いているかを知らない。

    TcpCommunicator       実機。C++ 制御部（127.0.0.1:12345）と、PC の推論サーバ。
                          4バイトのビッグエンディアン長ヘッダ + JSON。
                          C++ 側 PythonCommServer.cpp と同じ形式。
    WebSocketCommunicator シミュレータ。ブラウザから届く先（/ws/hint-card など）。
    CallbackCommunicator  PyScript とテスト。関数を 1つ差すだけ。

【4バイト長ヘッダを共通の関数にしている理由】
この並びだけは C++ とバイト単位で一致していないと通信が成立しない。
実機のサーバ（ev3_server.py）とクライアント（pc_client.py）が別々に同じ
処理を持っていると、片方だけ直したときに気づけない。send_packet /
recv_packet の 1組だけを見ればよいようにしてある。
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



# --------------------------------------------------------------------------
# TCP（実機）
# --------------------------------------------------------------------------

#: 受け取ってよい 1メッセージの上限[byte]。C++ 側 PythonCommServer.cpp と同じ
MAX_PACKET_BYTES = 1_000_000


def send_packet(sock, payload: Union[dict, str]) -> None:
    """
    4バイトのビッグエンディアン長ヘッダを付けて送る。

    C++ 側（PythonCommServer.cpp）と同じ形式。ここを変えると通信が壊れる。
    """
    import struct

    text = json.dumps(payload, ensure_ascii=False) if isinstance(payload, dict) else str(payload)
    body = text.encode("utf-8")
    sock.sendall(struct.pack("!I", len(body)) + body)


def recv_packet(sock) -> str:
    """
    4バイトの長ヘッダを読み、その長さぶんを最後まで受け取る。

    :returns: 受け取った文字列。切断・不正な長さなら ""
    """
    import struct

    header = b""
    while len(header) < 4:
        chunk = sock.recv(4 - len(header))
        if not chunk:
            return ""  # 切断
        header += chunk

    length = struct.unpack("!I", header)[0]
    if length == 0 or length > MAX_PACKET_BYTES:
        return ""

    data = b""
    while len(data) < length:
        chunk = sock.recv(length - len(data))
        if not chunk:
            return ""  # 切断
        data += chunk

    return data.decode("utf-8")


class TcpCommunicator(BaseCommunicator):
    """
    TCP を用いた双方向通信クライアント（実機用）。

    走行体の Python から、PC の推論サーバや C++ 制御部へつなぐときに使う。
    相手がまだ起動していないことがあるので、つながるまで 1秒おきに試す。
    """

    def __init__(self, host: str = "127.0.0.1", port: int = 49661, timeout: float = 5.0):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.conn = None
        self.lock = threading.Lock()

    def connect(self, retry: bool = True) -> bool:
        """つながるまで試す。つながったら True"""
        import socket

        while True:
            try:
                self.conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.conn.connect((self.host, self.port))
                logger.info("接続しました: %s:%s", self.host, self.port)
                return True
            except Exception as e:  # noqa: BLE001
                self.conn = None
                if not retry:
                    logger.warning("接続できません: %s:%s (%s)", self.host, self.port, e)
                    return False
                logger.warning("相手が未起動。再試行します... %s", e)
                time.sleep(1)

    def send(self, data: Union[dict, str]) -> None:
        """データを送る（応答は待たない）"""
        with self.lock:
            if self.conn is None and not self.connect(retry=False):
                return
            try:
                send_packet(self.conn, data)
            except Exception:  # noqa: BLE001
                logger.exception("TCP 送信に失敗しました: %s:%s", self.host, self.port)
                self.close()

    def receive(self, timeout: Optional[float] = None) -> Optional[Union[dict, str]]:
        """メッセージを 1件受け取る。受け取れなければ None"""
        with self.lock:
            if self.conn is None and not self.connect(retry=False):
                return None
            old = self.conn.gettimeout()
            try:
                self.conn.settimeout(timeout if timeout is not None else self.timeout)
                raw = recv_packet(self.conn)
                if raw == "":
                    return None
                return self._parse_payload(raw)
            except Exception:  # noqa: BLE001
                logger.exception("TCP 受信に失敗しました: %s:%s", self.host, self.port)
                self.close()
                return None
            finally:
                try:
                    if self.conn is not None:
                        self.conn.settimeout(old)
                except Exception:  # noqa: BLE001
                    pass

    def request(self, data: Union[dict, str],
                timeout: Optional[float] = None) -> Optional[Union[dict, str]]:
        """送って、応答を 1件待つ（要求・応答パターン）"""
        with self.lock:
            if self.conn is None and not self.connect(retry=False):
                return None
            old = self.conn.gettimeout()
            try:
                self.conn.settimeout(timeout if timeout is not None else self.timeout)
                send_packet(self.conn, data)
                raw = recv_packet(self.conn)
                if raw == "":
                    raise ConnectionError("接続が切れました")
                return self._parse_payload(raw)
            except Exception:  # noqa: BLE001
                logger.exception("TCP 要求に失敗しました: %s:%s", self.host, self.port)
                self.close()
                return None
            finally:
                try:
                    if self.conn is not None:
                        self.conn.settimeout(old)
                except Exception:  # noqa: BLE001
                    pass

    def _parse_payload(self, raw: str) -> Union[dict, str]:
        try:
            return json.loads(raw)
        except (ValueError, json.JSONDecodeError):
            return raw

    def on_message(self, callback: Callable[[Any], None]) -> Callable[[Any], None]:
        """受信ハンドラを登録する。"""
        raise NotImplementedError("TcpCommunicator は receive / request を使う")

    def close(self) -> None:
        """接続を閉じる。次に使うときに繋ぎ直す"""
        try:
            if self.conn:
                self.conn.close()
        except Exception:  # noqa: BLE001
            logger.exception("TCP 接続の切断でエラー")
        finally:
            self.conn = None


# 後方互換性エイリアス
MessageSender = BaseCommunicator
WebSocketSender = WebSocketCommunicator
CallbackSender = CallbackCommunicator
