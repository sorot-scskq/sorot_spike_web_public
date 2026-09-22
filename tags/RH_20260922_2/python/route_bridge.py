"""
走行経路の受け取りを、シミュレータのブラウザ内で動かす（PyScript）。

走行体で動く処理そのものは src/RoughSpot/Python/2026/route_receiver.py にある
（毎年変わらない土台は src/RoughSpot/Python/Common/wireless_device.py）。
ここはそれをブラウザで動かすための配線だけを持つ。

    実機          : TcpCommunicator      → route_receiver → C++ へシナリオを渡す
    シミュレータ  : ブラウザの WebSocket → route_receiver → JS がシナリオを挿し込む

【役割の分け方】
    Python … いつ要求するか、届いた JSON が使えるか、どの区分の経路か
    JS     … 受け取ったコマンド列を走行中のシナリオへ挿し込む
             （sim/dom-ui/route-receiver-client.js）

【ブラウザの WebSocket を包んでいる理由】
Common/communicator.py の WebSocketCommunicator は `websockets` を使うが、
ブラウザには入っていない。送受信の口だけをここで js.WebSocket に差し替える。
送信と受信が別々に起きるので、応答を待つ receive_route() ではなく、
request_route() で送っておき、届いたら handle_message() へ流す。

【全体を関数の中に入れている理由】
PyScript には、この形の Python を扱ううえで気をつけることが 2つある。

  1. 同じスクリプトが **2回実行される**ことがある。素直に書くと受け取り器が
     2つでき、1回の応答でシナリオが二重に挿し込まれる（実際に起きた）。
     window.__simPython.route があるかどうかで 2回目を弾く。
  2. 複数の <script type="py"> が **同じ名前空間**で動く。トップレベルに置いた
     関数や変数は、あとから読み込まれた別のスクリプトに上書きされる（実際、
     走行経路の宛先がヒントカード側の宛先に化けた）。関数の中に入れておけば
     ぶつからない。
"""

import json
import traceback

from js import Object, window
from pyodide.ffi import create_proxy, to_js as _to_js
from pyodide.http import pyfetch

#: 無線通信デバイスの既定の宛先（tools/ws_debug_server）
ROUTE_DEFAULT_WS_URL = "ws://127.0.0.1:8765"


async def _route_setup():
    """走行経路の受け取りをブラウザに載せる。読み込み時に 1回だけ呼ぶ"""

    def to_obj(value):
        """dict は JS の Map ではなく普通のオブジェクトにする（JS から見やすくするため）"""
        return _to_js(value, dict_converter=Object.fromEntries)

    # -- 走行体側の実装を取り込む（書き写さず、実機と同じ 1本を読む）--------
    for name in ("communicator.py", "wireless_device.py", "route_receiver.py"):
        response = await pyfetch(f"python/{name}")
        with open(name, "w") as out:
            out.write(await response.string())

    from route_receiver import (
        IDENTIFIER_ROUTE,
        SENARIO_KBN_GARAGE,
        RouteReceiver,
        senario_kbn_label,
    )

    def bridge():
        """JS 側の口（sim/dom-ui/route-receiver-client.js）"""
        return getattr(window, "__simRoute", None)

    def ws_url():
        js_side = bridge()
        url = getattr(js_side, "wsUrl", None) if js_side is not None else None
        return url if isinstance(url, str) and url != "" else ROUTE_DEFAULT_WS_URL

    # -- 送り口（ブラウザの WebSocket）-------------------------------------

    class BrowserWebSocket:
        """
        ブラウザの WebSocket を、Common/communicator.py と同じ形で使えるようにする。

        繋ぐのは経路が要るとき（LAP ゲート通過時）だけ。走行の最初から繋いで
        おくと、サーバを立てずに走らせたときにコンソールが接続エラーで埋まる。
        """

        def __init__(self):
            self.socket = None
            self.queued = []
            self.connected = False
            self.failed = False

        def connect(self):
            """繋ぐ。すでに繋いでいる／繋ぎにいっているなら何もしない"""
            if self.socket is not None and self.socket.readyState <= 1:  # 0=接続中 1=接続済
                return
            url = ws_url()
            self.socket = window.WebSocket.new(url)
            self.socket.addEventListener("open", create_proxy(self._on_open))
            self.socket.addEventListener("message", create_proxy(self._on_message))
            self.socket.addEventListener("error", create_proxy(self._on_error))
            self.socket.addEventListener("close", create_proxy(self._on_close))
            window.console.log("[PyScript] 走行経路の受け取り: 接続します " + url)

        def _on_open(self, _event):
            self.connected = True
            self.failed = False
            # 繋がる前に送ろうとしたぶんを流す
            for text in self.queued:
                self.socket.send(text)
            self.queued = []

        def _on_message(self, event):
            try:
                message = json.loads(event.data)
            except Exception:  # noqa: BLE001 — JSON 以外が届いても走行は続ける
                window.console.warn("[PyScript] 走行経路: JSON として読めません")
                return
            try:
                receiver.handle_message(message)
            except Exception:  # noqa: BLE001
                window.console.warn(
                    "[PyScript] 走行経路の処理で失敗: " + traceback.format_exc()
                )

        def _on_error(self, _event):
            if not self.failed:
                self.failed = True
                window.console.info(
                    "[PyScript] 走行経路の受け取り先に繋がりません（" + ws_url() + "）。"
                    "走行は続けます。使うには tools/ws_debug_server を起動してください"
                )
            js_side = bridge()
            if js_side is not None and hasattr(js_side, "onError"):
                js_side.onError(ws_url())

        def _on_close(self, _event):
            self.connected = False

        # --- BaseCommunicator と同じ形 -----------------------------------

        def send(self, data):
            """送る。まだ繋がっていなければ、繋がってから送る"""
            text = json.dumps(data, ensure_ascii=False) if isinstance(data, dict) else str(data)
            self.connect()
            if self.socket is not None and self.socket.readyState == 1:
                self.socket.send(text)
            else:
                self.queued.append(text)

        def receive(self, timeout=None):
            """ブラウザでは使わない。届いたものは message イベントから流れる"""
            return None

        def request(self, data, timeout=None):
            """ブラウザでは応答を待てない。send して message イベントで受ける"""
            self.send(data)
            return None

    communicator = BrowserWebSocket()
    receiver = RouteReceiver(communicator=communicator, senario_kbn=SENARIO_KBN_GARAGE)

    # -- 受け取ったシナリオを JS へ渡す ------------------------------------

    def apply_scenario(scenario, label):
        """
        受け取ったコマンド列を JS へ渡す。挿し込むのは JS の仕事。

        走行中に受け取るので、入れ替えではなく**挿し込み**であること。
        入れ替えると走行が最初からやり直しになる。
        """
        js_side = bridge()
        if js_side is None or not hasattr(js_side, "applyScenario"):
            window.console.warn("[PyScript] 走行経路: JS 側の受け口がありません")
            return
        js_side.applyScenario(to_obj(scenario), label)

    receiver.set_applier(apply_scenario)

    # -- JS から呼ぶ口 -----------------------------------------------------

    def lap_gate_passed(senario_kbn=None):
        """
        LAP ゲートを通過したことを知らせる。ここで経路を要求する。

        要求は 1回だけ。何度呼ばれても 2回目以降は送らない。
        :returns: 要求を送ったら True
        """
        try:
            kbn = int(senario_kbn) if senario_kbn is not None else None
            return receiver.notify_lap_gate_passed(kbn)
        except Exception:  # noqa: BLE001 — 通信で落ちても走行は続ける
            window.console.warn(
                "[PyScript] 走行経路の要求で失敗: " + traceback.format_exc()
            )
            return False

    def get_state():
        """いまの状態。画面表示用"""
        kbn = receiver.get_senario_kbn()
        return to_obj({
            "requested": receiver.requested,
            "received": receiver.has_received(),
            "senarioKbn": kbn,
            "label": senario_kbn_label(kbn) if kbn is not None else None,
            "commandCount": len(receiver.get_scenario()),
            "connected": communicator.connected,
        })

    def reset():
        """走行やり直し用"""
        receiver.reset()

    def connect():
        """
        先に繋ぐ（走行開始のとき）。

        ふだんは経路が要るとき（LAPゲート通過時）に初めて繋ぐが、PC-System と
        結合するとキャリブレーションの時点で接続が要る。走り出してから繋ぐのでは
        間に合わないので、走行開始でここを呼ぶ。

        WS モードの PC-System は接続した瞬間に経路（id=3）を送ってくるので、
        受け取り口と同じ接続でなければならない。別に張ると、welcome の経路を
        取りこぼす。
        """
        try:
            communicator.connect()
            return True
        except Exception:  # noqa: BLE001 — 繋がらなくても走行は続ける
            window.console.warn(
                "[PyScript] 走行経路の受け取り先へ繋げません: " + traceback.format_exc()
            )
            return False

    # window.__simPython は Python 側の窓口をまとめる入れ物。認識処理ごとに
    # 別々の PyScript が足していくので、丸ごと代入せず自分の枠だけを足す
    if not hasattr(window, "__simPython") or window.__simPython is None:
        window.__simPython = to_obj({})
    window.__simPython.route = to_obj({
        "lapGatePassed": create_proxy(lap_gate_passed),
        "connect": create_proxy(connect),
        "getState": create_proxy(get_state),
        "reset": create_proxy(reset),
    })
    window.console.log(
        "[PyScript] 走行経路の受け取り（route_receiver.py, identifier="
        + str(IDENTIFIER_ROUTE) + "）を読み込みました"
    )


# 同じスクリプトが 2回実行されることがある。2回目は何もしない
# （受け取り器が 2つできると、1回の応答でシナリオが二重に挿し込まれる）
if getattr(getattr(window, "__simPython", None), "route", None) is None:
    await _route_setup()
else:
    window.console.log("[PyScript] 走行経路の受け取りは読み込み済み。二重実行を飛ばします")
