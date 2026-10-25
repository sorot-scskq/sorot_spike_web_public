"""
無線通信デバイス（PC）へ WebSocket でつなぐクライアント。

**ソケット（TCP 49661）から WebSocket（8765）へ変えた。** PC 側
（PC-System_2026）が WebSocket で待ち受けるようになったため
（`config/socket_config.py` の `COMM_MODE = 'WS_TCP'`）。

運ぶものは TCP のときと同じで、**4バイトの長さヘッダ＋UTF-8 の JSON を
バイナリフレームで**送る。WebSocket 自身がメッセージの切れ目を持つので
ヘッダは本来要らないが、PC 側は実機とシミュレータで同じ道を通すために
この形を見ている。外すと向こうで警告になる（`communicator.PacketCodec`）。

    走行体 ──ws://<PC>:8765──▶ PC-System_2026（WebSocketTcpCommunicator）

【受信を別スレッドで回す理由】
PC は頼んでいないものも送ってくる。走行経路（シナリオ）は要求の応答として
返るが、接続した時点で送ってくる相手もいる（`tools/ws_debug_server`）。
要求の往復の中でしか読まないと、そういう便を取りこぼす。受信は1本の
スレッドが常に読み、

    要求の応答（`completed_req_id` が要求と一致）  待っている側へ渡す
    それ以外                                      on_message へ流す

と振り分ける。走行経路の登録は on_message 側（`Common/route_register.py`）。

【PC が居なくても走行は続ける】
つながらないときは例外を返すだけで、繋がるまで待たない。走行体の中だけで
完結する判定（ボトルの色）を止めないため。`connect()` だけは繋がるまで
試すので、起動時の温めのように別スレッドから呼ぶこと。
"""

import json
import logging
import queue
import threading
import time
from typing import Any, Callable, List, Optional

import config
from communicator import PacketCodec

logger = logging.getLogger(__name__)


class PCClient:
    """PC の推論サーバ（無線通信デバイス）と JSON をやり取りする。"""

    #: 繋ぎ直すまでの間隔[s]
    RECONNECT_INTERVAL = 1.0

    def __init__(self, url: Optional[str] = None, open_timeout: Optional[float] = None,
                 connector: Optional[Callable] = None):
        """
        :param url: 繋ぎ先。省略すると config.PC_WS_URL
        :param open_timeout: 繋ぐのを諦めるまで[s]
        :param connector: 繋ぐ手だて。差し替えるとテストから使える
                          （既定は websockets.sync.client.connect）
        """
        self.url = url or config.PC_WS_URL
        self.open_timeout = (config.PC_WS_OPEN_TIMEOUT
                             if open_timeout is None else open_timeout)
        self._connector = connector
        self._conn = None
        #: 要求は1件ずつ。応答がどの要求のものか分からなくなるのを避ける
        self._send_lock = threading.Lock()
        self._conn_lock = threading.Lock()
        self._responses: queue.Queue = queue.Queue()
        self._listeners: List[Callable[[Any], None]] = []
        self._reader: Optional[threading.Thread] = None
        self._closed = threading.Event()
        logger.info("PC側（無線通信デバイス）の繋ぎ先: %s", self.url)

    # --- 受け取り口 -----------------------------------------------------------

    def on_message(self, fn: Callable[[Any], None]) -> Callable[[], None]:
        """
        届いた JSON を受け取る口を足す。戻り値を呼ぶと外せる。

        **要求の応答も、頼んでいない便も、ここを通る。** 届いたものが使えるか
        （走行経路が入っているか）を決めるのは受け取る側。
        """
        if not callable(fn):
            return lambda: None
        self._listeners.append(fn)

        def off() -> None:
            if fn in self._listeners:
                self._listeners.remove(fn)

        return off

    # --- 接続 -----------------------------------------------------------------

    @property
    def is_connected(self) -> bool:
        return self._conn is not None

    def connect(self, retry: bool = True) -> bool:
        """
        繋ぐ。**繋がるまで試すので、走行を止めない場所から呼ぶこと。**

        :returns: 繋がったら True
        """
        while not self._closed.is_set():
            if self._ensure_connected() is not None:
                return True
            if not retry:
                return False
            self._closed.wait(self.RECONNECT_INTERVAL)
        return False

    def _ensure_connected(self):
        """繋がっていなければ1回だけ試す。繋がらなければ None"""
        with self._conn_lock:
            if self._conn is not None:
                return self._conn
            conn = self._open()
            if conn is None:
                return None
            self._conn = conn
            self._reader = threading.Thread(target=self._read_loop, args=(conn,),
                                            name="pc-recv", daemon=True)
            self._reader.start()
            return conn

    def _open(self):
        """WebSocket を1本開く。開けなければ None"""
        connect = self._connector
        if connect is None:
            try:
                from websockets.sync.client import connect
            except ImportError:
                logger.error("websockets が入っていないため PC へつなげません"
                             "（Common/setup_venv.sh を実行するか pip install websockets）")
                return None
        try:
            conn = connect(self.url, open_timeout=self.open_timeout,
                           max_size=PacketCodec.MAX_BYTES)
        except Exception as e:  # noqa: BLE001 — 繋がらなくても走行は続ける
            logger.warning("PC へつなげません(%s): %s", self.url, e)
            return None
        logger.info("PC側Pythonと接続しました: %s", self.url)
        return conn

    def close(self) -> None:
        """繋ぎを閉じる。走行の終わりに呼ぶ"""
        self._closed.set()
        self._drop(self._conn)

    def _drop(self, conn) -> None:
        """いまの繋ぎを捨てる。次に要求したときに繋ぎ直す"""
        if conn is None:
            return
        with self._conn_lock:
            if self._conn is conn:
                self._conn = None
        try:
            conn.close()
        except Exception:  # noqa: BLE001
            logger.debug("PC との繋ぎを閉じるときに例外が出ましたが、無視します")

    # --- 受信 -----------------------------------------------------------------

    def _read_loop(self, conn) -> None:
        """届いたものを読み続ける。**このスレッドだけが recv を呼ぶ**"""
        while not self._closed.is_set():
            try:
                raw = conn.recv()
            except Exception as e:  # noqa: BLE001 — 切断も含む
                if not self._closed.is_set():
                    logger.warning("PC との接続が切れました: %s", e)
                break
            message = self._decode(raw)
            if message is not None:
                self._dispatch(message)
        self._drop(conn)

    def _decode(self, raw) -> Optional[dict]:
        """フレーム1通を JSON の辞書にする。読めなければ None"""
        if isinstance(raw, str):
            # 長ヘッダの無いテキスト。デバッグ用のサーバ（ws_debug_server）がこれ
            text = raw
        else:
            text = PacketCodec.unpack(raw)
        if text is None:
            return None
        try:
            message = json.loads(text)
        except ValueError as e:
            logger.warning("PC から届いたものを JSON として読めません: %s", e)
            return None
        if not isinstance(message, dict):
            logger.warning("PC から届いたものが JSON オブジェクトではありません")
            return None
        return message

    def _dispatch(self, message: dict) -> None:
        """届いたものを受け取り口へ流し、応答として待っている側にも渡す"""
        logger.info("PCから受信: %s", self._brief(message))
        for fn in list(self._listeners):
            try:
                fn(message)
            except Exception:  # noqa: BLE001 — 受け取り側で失敗しても読み続ける
                logger.exception("PC から届いたものの受け取りで失敗しました")
        self._responses.put(message)

    @staticmethod
    def _brief(message: dict) -> str:
        """ログ用。長い項目（画像やコマンド列）は長さだけにする"""
        brief = {}
        for key, value in message.items():
            if isinstance(value, str) and len(value) > 120:
                brief[key] = "<%d文字>" % len(value)
            elif isinstance(value, (list, dict)):
                brief[key] = "<%d件>" % len(value)
            else:
                brief[key] = value
        return json.dumps(brief, ensure_ascii=False)

    # --- 要求 -----------------------------------------------------------------

    def send_and_receive(self, payload: dict, timeout: Optional[float] = None) -> dict:
        """
        要求を送り、その応答を待つ。

        応答かどうかは `completed_req_id` で見る。待っているあいだに届いた
        別の便（頼んでいない経路など）は読み飛ばす。読み飛ばしても on_message
        には流れているので、取りこぼしにはならない。

        :raises ConnectionError: 繋がらない・送れない
        :raises TimeoutError: 制限時間までに応答が来ない
        """
        timeout = config.PC_TIMEOUT if timeout is None else timeout
        with self._send_lock:
            conn = self._ensure_connected()
            if conn is None:
                raise ConnectionError("PC へつながっていません: %s" % self.url)

            # 前の要求の取りこぼしや、頼んでいない便を残さない
            self._drain()

            request_id = payload.get("request_id")
            try:
                conn.send(PacketCodec.pack(payload))
            except Exception as e:  # noqa: BLE001
                self._drop(conn)
                raise ConnectionError("PC へ送れませんでした: %s" % e) from e
            logger.info("送信しました: %s", self._brief(payload))

            deadline = time.time() + timeout
            while True:
                remaining = deadline - time.time()
                if remaining <= 0:
                    raise TimeoutError(
                        "PCからの応答がタイムアウト時間（%s秒）内に受信できませんでした。"
                        % timeout)
                try:
                    message = self._responses.get(timeout=remaining)
                except queue.Empty:
                    continue
                if self._is_response_to(message, request_id):
                    return message
                logger.info("この要求(request_id=%s)の応答ではないので読み飛ばします: "
                            "completed_req_id=%s", request_id,
                            message.get("completed_req_id"))

    @staticmethod
    def _is_response_to(message: dict, request_id) -> bool:
        """届いたものが、その要求の応答か。

        `completed_req_id` が無い相手（デバッグ用のサーバ）は応答として扱う。
        """
        completed = message.get("completed_req_id")
        if completed is None or request_id is None:
            return True
        try:
            return int(completed) == int(request_id)
        except (TypeError, ValueError):
            return False

    def _drain(self) -> None:
        """溜まっている便を捨てる（受け取り口には既に流してある）"""
        while True:
            try:
                stale = self._responses.get_nowait()
            except queue.Empty:
                return
            logger.debug("待っていない便を捨てます: %s", self._brief(stale))
