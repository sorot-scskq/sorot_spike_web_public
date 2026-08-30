"""
ヒントカードの読み取りを、シミュレータのブラウザ内で動かす（PyScript）。

走行体で動く読み取りそのものは src/RoughSpot/Python/2026/hint_card_reader.py にある。ここは
そのコードをブラウザで動かすための配線だけを持つ。

    実機          : cv2.VideoCapture      → hint_card_reader → WebSocket
    シミュレータ  : #cameraView の canvas → hint_card_reader → 画面と WebSocket

【シミュレータで中身を補う理由】
コースに描いている二次元コードは本物のコードではない。仮に本物にしても、
前面カメラは 240x144 で、5cm のカードは距離によらず 24px 前後にしかならない
（sim/camera-model.js はほぼ平行投影）。版1 の二次元コードでも 21 モジュール
あるので、1px/モジュールを割って読めない。
そこで OpenCV には「カードが画素として写っているか」までをやらせ、中身は
シミュレータ側（JS）から補う。近づいて正面に構えないと読めない、という
走行側の段取りは写っているかどうかで決まるので、そこは実機と同じ条件になる。
"""

import json
import traceback

import pyodide_js
from js import Object, document, window
from pyodide.ffi import create_proxy, to_js as _to_js
from pyodide.http import pyfetch

# --------------------------------------------------------------------------
# 依存の用意
#
# パッケージも走行体側のコードも、ここで自分で用意する。
# <script type="py"> の config に書く手もあるが、PyScript はページに複数の
# Python があると最初の config だけを使う。年ごとに認識処理が増減するこの
# シミュレータでは、どれが最初になるか決められないので config に頼らない。
# --------------------------------------------------------------------------

# 二次元コードの読み取りは OpenCV。実機と同じ処理を通すため入れる
await pyodide_js.loadPackage(["numpy", "opencv-python"])

import cv2  # noqa: E402
import numpy as np  # noqa: E402

# 走行体側の実装および共通モジュールを取り込む
for _mod_name in ("camera.py", "qr_decoder.py", "sender.py", "communicator.py", "hint_card_reader.py"):
    _resp = await pyfetch(f"python/{_mod_name}")
    with open(_mod_name, "w") as _f:
        _f.write(await _resp.string())

from camera import RobotCamera  # noqa: E402
from hint_card_reader import HintCard, HintCardReader  # noqa: E402

# 前面カメラの canvas（index.html）
CANVAS_ID = "cameraView"
FRAME_IMAGE_PATH = "sim_camera_frame.png"

# 読んだ内容の送り先。ページ側で window.__simHintCard.wsUrl を書けば変えられる
DEFAULT_WS_URL = "ws://127.0.0.1:8000/ws/hint-card"


def to_js(value):
    """dict は JS の Map ではなく普通のオブジェクトにする（JS から見やすくするため）"""
    return _to_js(value, dict_converter=Object.fromEntries)


# 画像ファイル経由で映像を取得する RobotCamera インスタンスを渡して初期化
reader = HintCardReader(camera=RobotCamera(source=FRAME_IMAGE_PATH))


# --------------------------------------------------------------------------
# 映像の取り込み（canvas → 画像ファイルへ保存）
# --------------------------------------------------------------------------


def save_canvas_frame(file_path=FRAME_IMAGE_PATH):
    """前面カメラの canvas を 1枚読み、画像ファイルとして保存する。"""
    canvas = document.getElementById(CANVAS_ID)
    if canvas is None:
        return False
    width = int(canvas.width)
    height = int(canvas.height)
    if width <= 0 or height <= 0:
        return False

    ctx = canvas.getContext("2d", to_js({"willReadFrequently": True}))
    image = ctx.getImageData(0, 0, width, height)
    # Uint8ClampedArray → numpy → BGR 画像
    rgba = np.frombuffer(bytearray(image.data.to_py()), dtype=np.uint8)
    rgba = rgba.reshape((height, width, 4))
    bgr = cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGR)
    cv2.imwrite(file_path, bgr)
    return True


# --------------------------------------------------------------------------
# 中身を補う（シミュレータだけの道）
# --------------------------------------------------------------------------


#: 直近に画素から見つかったカードらしい四角の数（画面に出すためだけのもの）
_panel_count = 0


def resolve_panels(panels):
    """
    OpenCV が見つけたカードの位置を JS へ渡し、そこにあるカードの中身をもらう。

    どのカードかを決めるのは配置を知っている側の仕事なので、JS に任せる。
    写っているかどうかは、ここへ来る前に画素から決まっている。
    """
    global _panel_count
    _panel_count = len(panels)

    bridge = getattr(window, "__simHintCard", None)
    if bridge is None or not hasattr(bridge, "resolvePanels"):
        return None
    text = bridge.resolvePanels(to_js(panels))
    return text if isinstance(text, str) and text.strip() != "" else None


# --------------------------------------------------------------------------
# 送信
# --------------------------------------------------------------------------

_socket = None

# 送り先が居ないと分かったら、以後は送りにいかない。
# サーバを立てずに走らせることのほうが多く、毎回の失敗でログが読めなくなるため
_ws_unavailable = False


def _on_ws_error(_event):
    global _ws_unavailable
    if not _ws_unavailable:
        _ws_unavailable = True
        window.console.info(
            "[PyScript] ヒントカードの送信先に繋がりません（" + _ws_url() + "）。"
            "読み取りは続けます。送るには tools/remote_control_server を起動してから"
            "ページを開き直してください"
        )


def _ws_url():
    bridge = getattr(window, "__simHintCard", None)
    url = getattr(bridge, "wsUrl", None) if bridge is not None else None
    return url if isinstance(url, str) and url != "" else DEFAULT_WS_URL


def send_card(payload):
    """
    読んだ内容を送る。

    ブラウザなので `websockets` は使えない。同じ形の JSON を js.WebSocket で送る。
    送り先が居なくても走行は続けたいので、失敗は記録するだけにする。
    実機では hint_card_reader.WebSocketSender が同じ JSON を送る。
    """
    text = json.dumps(payload, ensure_ascii=False)

    bridge = getattr(window, "__simHintCard", None)
    if bridge is not None and hasattr(bridge, "onSent"):
        bridge.onSent(to_js(payload))

    global _socket, _ws_unavailable
    if _ws_unavailable:
        return  # 送り先が居ないと分かっている。毎回試すとログが埋まる
    try:
        if _socket is None or _socket.readyState > 1:  # 0=CONNECTING 1=OPEN
            _socket = window.WebSocket.new(_ws_url())
            _socket.addEventListener("error", create_proxy(_on_ws_error))
        if _socket.readyState == 1:
            _socket.send(text)
        else:
            # まだ開いていない。開いたら送る
            def _on_open(_e, body=text, sock=_socket):
                sock.send(body)

            _socket.addEventListener("open", create_proxy(_on_open))
    except Exception:  # noqa: BLE001 — 送れなくても読んだ事実は残す
        window.console.warn("[PyScript] ヒントカードの送信に失敗: " + traceback.format_exc())


# --------------------------------------------------------------------------
# JS から呼ぶ口
# --------------------------------------------------------------------------


#: 次に映像を読む時刻[ms]
_next_read_at = 0.0

#: 映像を読む間隔[ms]。毎フレーム（60回/秒）OpenCV に掛けると重い。
#: 走行体も毎周期は読まないので、これで実機の使い方にも近くなる
READ_INTERVAL_MS = 200


def read_once():
    """
    毎フレーム JS から呼ばれる。読めたら送り、いまの状態を返す。

    実際に映像を読むのは READ_INTERVAL_MS おきで、2枚とも読み終えたら
    それもやめる。呼び出し側は間引きを気にせず、毎フレーム呼んでよい。

    :returns: {'card1': str|None, 'card2': str|None, 'placement': str,
               'newCard': str|None, 'panels': int} を JS のオブジェクトにしたもの
    """
    global _next_read_at, _panel_count

    new_card = None
    now = window.performance.now()
    if not reader.has_read_all() and now >= _next_read_at:
        _next_read_at = now + READ_INTERVAL_MS
        _panel_count = 0
        try:
            if save_canvas_frame():
                card = reader.read()
                new_card = card["card"] if card else None
        except Exception:  # noqa: BLE001 — 認識で落ちても走行は続ける
            window.console.warn(
                "[PyScript] ヒントカードの読み取りで失敗: " + traceback.format_exc()
            )

    c1 = reader.get_card(HintCard.CARD1)
    c2 = reader.get_card(HintCard.CARD2)
    return to_js({
        "card1": c1["text"] if c1 else None,
        "card2": c2["text"] if c2 else None,
        "placement": reader.get_placement_string(),
        "newCard": new_card,
        "panels": _panel_count,
    })


def reset():
    """走行やり直し用。読み取り結果を捨てる"""
    global _next_read_at, _panel_count
    reader.reset()
    _next_read_at = 0.0
    _panel_count = 0


reader.set_panel_resolver(resolve_panels)
reader.set_sender(send_card)

# JS からはこの 1か所だけを見る（sim/gate/hint-card-camera.js）
window.__simPython = to_js({
    "hintCard": {
        "read": create_proxy(read_once),
        "reset": create_proxy(reset),
    },
})
window.console.log("[PyScript] ヒントカードの読み取り（hint_card_reader.py）を読み込みました")
