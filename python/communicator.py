"""
共通通信モジュール（送受信対応）。

双方向メッセージ通信（送信・受信・イベント待受・リクエスト/レスポンス）を提供する。

【下回りを差し替えられるようにしてある理由】
上に載る処理（ヒントカードの読み取りなど）が、どの通信でつながっているかを
知らずに済むようにするため。BaseCommunicator でそろえてある。

    TcpCommunicator       TCP でつなぐ相手がいるとき用。4バイトのビッグエンディアン
                          長ヘッダ + JSON で、C++ 側 PythonCommServer.cpp と同じ形式。
                          **走行体の本番の道ではない**（C++ とはファイル、PC とは
                          WebSocket。pc_client.py）
    CallbackCommunicator  テスト用。関数を 1つ差すだけ
    WebSocketCommunicator メッセージを1通ずつやりとりする用（枠付けなし）。
                          sorot_spike_web（シミュレータ）と同じ形。走行体が PC へ
                          送るときは枠付きなので、そちらは pc_client.py を使う

【4バイト長ヘッダを共通の関数にしている理由】
この並びだけは C++ とバイト単位で一致していないと通信が成立しない。
送る側と受ける側が別々に同じ処理を持っていると、片方だけ直したときに
気づけない。PacketCodec の1つだけを見ればよいようにしてある。
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

    **走行体では使っていない。** 走行体は TcpCommunicator でつながる。
    無線通信デバイスと WebSocket で話す必要が出たときのために残してある
    （使うには websockets を入れること）。

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
    """関数コールバックを用いた送受信クラス（テスト用）。"""

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
        """受信データを登録済みハンドラへ通知する（テストからの模擬受信）。"""
        for handler in list(self._handlers):
            try:
                handler(data)
            except Exception:  # noqa: BLE001
                logger.exception("CallbackCommunicator ハンドラエラー")



# --------------------------------------------------------------------------
# TCP（実機）
# --------------------------------------------------------------------------

class PacketCodec:
    """
    4バイトのビッグエンディアン長ヘッダ ＋ UTF-8 の JSON。

    **C++ 側（PythonCommServer.cpp）とバイト単位で一致していないと通信が成立しない。**
    ここ1つにまとめてあるのは、送る側と受ける側が別々に同じ処理を持つと、片方だけ
    直したときに気づけないため。

        pack / unpack … WebSocket のフレーム1通ぶん（pc_client.py）
        send / recv   … TCP のストリーム（TcpCommunicator）
    """

    #: 受け取ってよい 1メッセージの上限[byte]。C++ 側 PythonCommServer.cpp と同じ
    MAX_BYTES = 1_000_000

    @classmethod
    def pack(cls, payload: Union[dict, str]) -> bytes:
        """長ヘッダを付けたバイト列にする"""
        import struct

        text = (json.dumps(payload, ensure_ascii=False)
                if isinstance(payload, dict) else str(payload))
        body = text.encode("utf-8")
        return struct.pack("!I", len(body)) + body

    @classmethod
    def unpack(cls, raw: bytes) -> Optional[str]:
        """
        長ヘッダ付きのバイト列から中身を取り出す（WebSocket のフレーム1通ぶん）。

        ストリーム（TCP）と違って切れ目は運ぶ側が持っているので、ヘッダと中身の
        長さが合っているかだけを見る。

        :returns: 中身の文字列。形が違えば None
        """
        import struct

        if not isinstance(raw, (bytes, bytearray)):
            return None
        if len(raw) < 4:
            logger.warning("フレームが短すぎます（%dバイト）", len(raw))
            return None

        length = struct.unpack("!I", bytes(raw[:4]))[0]
        body = bytes(raw[4:])
        if length != len(body):
            # ヘッダを信じて読む。送り側が枠付けを忘れていると、ここで気づける
            logger.warning("長ヘッダ（%d）と中身（%dバイト）が合いません", length, len(body))
            body = body[:length]
        return body.decode("utf-8", errors="replace")

    @classmethod
    def send(cls, sock, payload: Union[dict, str]) -> None:
        """長ヘッダを付けて送る（TCP）"""
        sock.sendall(cls.pack(payload))

    @classmethod
    def recv(cls, sock) -> str:
        """
        長ヘッダを読み、その長さぶんを最後まで受け取る（TCP）。

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
        if length == 0 or length > cls.MAX_BYTES:
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
                PacketCodec.send(self.conn, data)
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
                raw = PacketCodec.recv(self.conn)
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
                PacketCodec.send(self.conn, data)
                raw = PacketCodec.recv(self.conn)
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
