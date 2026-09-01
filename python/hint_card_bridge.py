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

【名前がぶつからないようにすること】
PyScript の <script type="py"> は、複数あっても **同じ Python の名前空間**で動く。
同じ名前の関数や定数を別のファイルで定義すると、あとから読み込まれたほうで
上書きされる。実際、走行経路の受け取りがヒントカード側の宛先へ繋ぎにいく
不具合が出た。ここで定義するものは、このファイル専用の接頭辞を付けること。
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
async def _hint_load_modules():
    """
    走行体側の実装を取り込む。

    取り込みは関数の中で行う。PyScript は複数の Python を**同じ名前空間**で
    動かすので、ループ変数を素のトップレベルに置くと、別の Python が
    await している隙に書き換えてしまう。
    """
    for name in ("camera.py", "qr_decoder.py", "communicator.py", "hint_card_reader.py"):
        response = await pyfetch(f"python/{name}")
        with open(name, "w") as out:
            out.write(await response.string())


await _hint_load_modules()

from camera import RobotCamera  # noqa: E402
from qr_decoder import QrCodeDecoder  # noqa: E402
from hint_card_reader import HintCard, HintCardReader  # noqa: E402

# 読み取りに使う canvas が取れなかったときの逃げ先（表示用・index.html）
CANVAS_ID = "cameraView"

# 読んだ内容の送り先。PC 側（PC-System_2026）の WebSocket と同じ口で、
# 走行経路の受け取り（route_bridge.py）と同じ宛先。
# ページ側で window.__simHintCard.wsUrl を書けば変えられる
HINT_DEFAULT_WS_URL = "ws://127.0.0.1:8765"


def to_js(value):
    """dict は JS の Map ではなく普通のオブジェクトにする（JS から見やすくするため）"""
    return _to_js(value, dict_converter=Object.fromEntries)


# 映像は canvas から直に渡す。RobotCamera.set_frame_source は、まさに
# シミュレータのために camera.py が用意している口（下の _hint_grab_frame で差す）
class _PanelOnlyDecoder(QrCodeDecoder):
    """
    板を見つけるところまでで止める読み取り。

    ブラウザの OpenCV（Pyodide の opencv-python）は二次元コードを**復号できない**。
    検出はするが、復号は必ず空文字を返す。自分で作ったコードを自分に読ませても
    同じなので、映像の写り方の問題ではなく、復号器（quirc）を外して作られた
    ものだと分かる。

        cv2.QRCodeEncoder_create().encode(...) → detectAndDecode()
        → detected=True, decoded=""

    そのため decode_frame の 4段（カラー・グレー・二値化・拡大）は、シミュレータ
    では必ず失敗する。板 1枚ぶんの切り抜きでも 1回 70ms 掛かり、カードを向けて
    いるあいだ走行が引っかかる。読めないと分かっているものに掛ける時間なので、
    ここで止める。

    板を探すところ（find_card_panels）は動くので、そのまま使う。中身は
    シミュレータ側（JS の resolvePanels）が答える。実機の OpenCV は復号器を
    持っているので、走行体側のコード（qr_decoder.py）はそのままでよい。
    """

    def decode(self, frame, panels=None):
        return None


def _cv2_can_decode_qr():
    """この OpenCV が二次元コードを復号できるか。自分で作って自分で読んでみる"""
    try:
        code = cv2.QRCodeEncoder_create().encode("25,35")
        n = code.shape[0]
        big = cv2.resize(code, (n * 8, n * 8), interpolation=cv2.INTER_NEAREST)
        padded = cv2.copyMakeBorder(big, 32, 32, 32, 32, cv2.BORDER_CONSTANT, value=255)
        decoded, _points, _ = cv2.QRCodeDetector().detectAndDecode(
            cv2.cvtColor(padded, cv2.COLOR_GRAY2BGR)
        )
        return decoded == "25,35"
    except Exception:  # noqa: BLE001 — 判定に失敗したら「読めない」側に倒す
        return False


reader = HintCardReader(camera=RobotCamera())

if not _cv2_can_decode_qr():
    reader.decoder = _PanelOnlyDecoder()
    window.console.info(
        "[PyScript] この OpenCV は二次元コードを復号できません（Pyodide の "
        "opencv-python は復号器を持たない）。板の検出までにして、中身は "
        "シミュレータ側で補います"
    )


# --------------------------------------------------------------------------
# 映像の取り込み（canvas → 画像ファイルへ保存）
# --------------------------------------------------------------------------


def _hint_read_canvas():
    """
    画素を読む canvas を返す。

    JS 側が読み取り用の高解像度 canvas を持っていればそれを使う
    （sim/gate/hint-card-camera.js の window.__simHintCard.canvas）。
    表示用の 240x144 では 5cm のカードが 24px にしかならず、二次元コードの
    模様として写らない。加えて resolve_panels が返す位置もこの canvas の
    座標なので、別の canvas を読むと縮尺が食い違って一致しなくなる。
    """
    bridge = getattr(window, "__simHintCard", None)
    canvas = getattr(bridge, "canvas", None) if bridge is not None else None
    if canvas is not None:
        return canvas
    return document.getElementById(CANVAS_ID)


def _hint_capture_frame():
    """
    読み取り用の映像を、今の景色で描き直してもらう。

    高解像度の映像は毎フレームは描かれていない。1枚が重いので、画素を見る
    この瞬間にだけ JS へ描かせる（sim/canvas.js の captureReadView）。
    """
    bridge = getattr(window, "__simHintCard", None)
    capture = getattr(bridge, "captureFrame", None) if bridge is not None else None
    if capture is None:
        return True   # 描き直す口が無い＝表示用をそのまま読む
    return bool(capture())


def _hint_grab_frame():
    """
    canvas を 1枚読み、OpenCV が扱う BGR の配列にして返す。

    RobotCamera.set_frame_source に差す。camera.py がシミュレータ用に
    用意している口で、実機は同じ RobotCamera を v4l2 の撮影ファイルで使う。

    【PNG を経由しない理由】
    以前はここで cv2.imwrite し、RobotCamera(source=パス) に読み直させていた。
    960x576 だと PNG の符号化と復号だけで 1回 100ms を超える。読み取りは
    200ms おきなので、それだけで走行が止まる。配列のまま渡せば往復が要らない。

    :returns: BGR の配列。読めなければ None
    """
    if not _hint_capture_frame():
        return None
    canvas = _hint_read_canvas()
    if canvas is None:
        return None
    width = int(canvas.width)
    height = int(canvas.height)
    if width <= 0 or height <= 0:
        return None

    ctx = canvas.getContext("2d", to_js({"willReadFrequently": True}))
    image = ctx.getImageData(0, 0, width, height)
    # Uint8ClampedArray → numpy → BGR 画像
    rgba = np.frombuffer(bytearray(image.data.to_py()), dtype=np.uint8)
    rgba = rgba.reshape((height, width, 4))
    return cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGR)


reader.camera.set_frame_source(_hint_grab_frame)


# --------------------------------------------------------------------------
# 中身を補う（シミュレータだけの道）
# --------------------------------------------------------------------------


#: 直近に画素から見つかったカードらしい四角の数（画面に出すためだけのもの）
_hint_panel_count = 0


def resolve_panels(panels):
    """
    OpenCV が見つけたカードの位置を JS へ渡し、そこにあるカードの中身をもらう。

    どのカードかを決めるのは配置を知っている側の仕事なので、JS に任せる。
    写っているかどうかは、ここへ来る前に画素から決まっている。
    """
    global _hint_panel_count
    _hint_panel_count = len(panels)

    bridge = getattr(window, "__simHintCard", None)
    if bridge is None or not hasattr(bridge, "resolvePanels"):
        return None
    text = bridge.resolvePanels(to_js(panels))
    return text if isinstance(text, str) and text.strip() != "" else None


# --------------------------------------------------------------------------
# 送信
# --------------------------------------------------------------------------

_hint_socket = None

# 送り先が居ないと分かったら、以後は送りにいかない。
# サーバを立てずに走らせることのほうが多く、毎回の失敗でログが読めなくなるため
_hint_ws_unavailable = False


def _on_hint_ws_error(_event):
    global _hint_ws_unavailable
    if not _hint_ws_unavailable:
        _hint_ws_unavailable = True
        window.console.info(
            "[PyScript] ヒントカードの送信先に繋がりません（" + _hint_ws_url() + "）。"
            "読み取りは続けます。送るには tools/remote_control_server を起動してから"
            "ページを開き直してください"
        )


def _hint_ws_url():
    bridge = getattr(window, "__simHintCard", None)
    url = getattr(bridge, "wsUrl", None) if bridge is not None else None
    return url if isinstance(url, str) and url != "" else HINT_DEFAULT_WS_URL


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

    global _hint_socket, _hint_ws_unavailable
    if _hint_ws_unavailable:
        return  # 送り先が居ないと分かっている。毎回試すとログが埋まる
    try:
        if _hint_socket is None or _hint_socket.readyState > 1:  # 0=CONNECTING 1=OPEN
            _hint_socket = window.WebSocket.new(_hint_ws_url())
            _hint_socket.addEventListener("error", create_proxy(_on_hint_ws_error))
        if _hint_socket.readyState == 1:
            _hint_socket.send(text)
        else:
            # まだ開いていない。開いたら送る
            def _on_open(_e, body=text, sock=_hint_socket):
                sock.send(body)

            _hint_socket.addEventListener("open", create_proxy(_on_open))
    except Exception:  # noqa: BLE001 — 送れなくても読んだ事実は残す
        window.console.warn("[PyScript] ヒントカードの送信に失敗: " + traceback.format_exc())


# --------------------------------------------------------------------------
# JS から呼ぶ口
# --------------------------------------------------------------------------


#: 次に映像を読む時刻[ms]
_hint_next_read_at = 0.0

#: 映像を読む間隔[ms]。毎フレーム（60回/秒）OpenCV に掛けると重い。
#: 走行体も毎周期は読まないので、これで実機の使い方にも近くなる
HINT_READ_INTERVAL_MS = 200


def read_once():
    """
    毎フレーム JS から呼ばれる。読めたら送り、いまの状態を返す。

    実際に映像を読むのは前回を終えてから HINT_READ_INTERVAL_MS 後で、2枚とも
    読み終えたらそれもやめる。呼び出し側は間引きを気にせず、毎フレーム呼んでよい。

    :returns: {'card1': str|None, 'card2': str|None, 'placement': str,
               'newCard': str|None, 'panels': int} を JS のオブジェクトにしたもの
    """
    global _hint_next_read_at, _hint_panel_count

    new_card = None
    now = window.performance.now()
    if not reader.has_read_all() and now >= _hint_next_read_at:
        _hint_panel_count = 0
        try:
            card = reader.read()
            new_card = card["card"] if card else None
        except Exception:  # noqa: BLE001 — 認識で落ちても走行は続ける
            window.console.warn(
                "[PyScript] ヒントカードの読み取りで失敗: " + traceback.format_exc()
            )
        finally:
            # 次の時刻は「終わってから」数える。始めた時刻から数えると、
            # 1回が間隔より長くなったときに切れ目なく走り続けることになる。
            #
            # 読み取りは同期で走るので、走っているあいだは描画もタスクも
            # 止まる。1回が間隔より長い（実測で 140ms 前後）と、止まって
            # いる時間の割合が間隔だけでは決まらないため、掛かった時間の
            # ぶんも空ける。こうすると止まっている割合は半分を超えない。
            _hint_elapsed = window.performance.now() - now
            _hint_next_read_at = window.performance.now() + max(
                HINT_READ_INTERVAL_MS, _hint_elapsed
            )

    c1 = reader.get_card(HintCard.CARD1)
    c2 = reader.get_card(HintCard.CARD2)
    return to_js({
        "card1": c1["text"] if c1 else None,
        "card2": c2["text"] if c2 else None,
        "placement": reader.get_placement_string(),
        "newCard": new_card,
        "panels": _hint_panel_count,
    })


def hint_reset():
    """走行やり直し用。読み取り結果を捨てる"""
    global _hint_next_read_at, _hint_panel_count
    reader.reset()
    _hint_next_read_at = 0.0
    _hint_panel_count = 0


reader.set_panel_resolver(resolve_panels)
reader.set_sender(send_card)

# JS からはこの 1か所だけを見る（sim/gate/hint-card-camera.js）。
# window.__simPython は Python 側の窓口をまとめる入れ物で、認識処理ごとに
# 別々の PyScript が足していく。丸ごと代入すると、先に載った別の処理
# （走行経路の受け取りなど）を消してしまうので、自分の枠だけを足す。
if not hasattr(window, "__simPython") or window.__simPython is None:
    window.__simPython = to_js({})
window.__simPython.hintCard = to_js({
    "read": create_proxy(read_once),
    "reset": create_proxy(hint_reset),
})
window.console.log("[PyScript] ヒントカードの読み取り（hint_card_reader.py）を読み込みました")
