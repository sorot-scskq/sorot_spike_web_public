"""
走行体の Python を、ブラウザの中で走行体のコードのまま動かす（シナリオの SCRIPT の受け口）。

走行体では、C++ 部が SCRIPT のパスを送ってくると、Python 部（Common/ev3_python.py の
handle_request）が次の2行を実行する。

    RobotPose.get_instance().update_from_request(req)
    ScriptRunner.get_instance().run(script, request_id)

ScriptRunner は `<年>/recog/<名前>.py` を読み、その run(request_id) を呼ぶ。シミュレータでも
**この2行をそのまま呼ぶ。** パスごとの対応表は書かない。走行体が recog にファイルを足せば、
ビルドし直すだけで動く（一覧はビルド時に作る。vite.config.js の robotManifest）。

【シミュレータが用意するもの（走行体の「機械」に当たるもの）】
走行体のコードは変えず、走行体が外から差せるように作ってある口へ差すだけにする。

  カメラ         RobotCamera.set_frame_source に、前面カメラの読み取り用の画を渡す
                 （走行体にシミュレータ用として元からある口。Common/camera.py）
  カメラの取付   et_sumo.ROBOT_CAMERA を、シミュレータのカメラ（SIM_CAMERA）の値にする。
                 シミュレータのカメラは実機と高さ・俯角・画角が違うため
  スレッド       ブラウザでは立てられない。threading.Thread / Timer の start を、JS の
                 タイマーから JSPI（callPromising）で呼ぶ形に差し替える。Pyodide 0.28 以降は、
                 そうして始めた Python の中の time.sleep() が、そこで一時停止してブラウザに
                 順番を返す。走行体の「撮る → sleep → 撮る」ループが、ページを止めずに回る。
                 Event.wait() も短く sleep しながら待つ形にする（下の _rr_install_threads）
  C++ との口     リモートコントロール（FUNCNO 18）は、走行体では /dev/shm のファイルで
                 やりとりする。シミュレータでは RemoteControlBridge（window.REMOTECONTROL）
                 から RemoteControl.handle_observation を直に呼ぶ（sim/pyscript/et_sumo_bridge.py と同じ）

【できないこと】
スレッドどうしが Lock を握ったまま sleep して取り合うと、交代できずに止まる（本物のスレッドは
無く、交代するのは sleep / Event.wait の所だけのため）。認識1枚の処理（OpenCV）の間も交代しない。

【PyScript の名前空間】
他の bridge と衝突しないよう、グローバルは _rr_ 接頭辞を付ける。JS からは window.__simRobot を使う。
"""

import json
import logging
import os
import sys
import threading
import time
import traceback

import pyodide_js
from js import JSON, Object, console, setTimeout, window
from pyodide.code import run_js
from pyodide.ffi import create_once_callable, create_proxy
from pyodide.ffi import to_js as _to_js
from pyodide.http import pyfetch


async def _sim_ffi_load():
    """JS の null を None にそろえる共通の関数（sim/pyscript/sim_ffi.py）を取り込む"""
    response = await pyfetch("python/sim_ffi.py", cache="no-store")
    with open("sim_ffi.py", "w") as out:
        out.write(await response.string())


await _sim_ffi_load()
from sim_ffi import js_value  # noqa: E402

await pyodide_js.loadPackage(['numpy', 'opencv-python'])

import cv2  # noqa: E402
import numpy as np  # noqa: E402

#: 走行体の RoughSpot/Python に当たる置き場（ブラウザの中のファイル）。SCRIPT はここからの相対
_RR_ROOT = os.path.join(os.getcwd(), 'robot_python')
#: 走行体の SubScenario の置き場（config_2026 の BOTTLE_ROUTE_FILES / SCENARIO_FILE と同じ）
_RR_SUBSCENARIO = os.path.join('sorot_spike', 'SubScenario')


def _rr_to_js(value):
    return _to_js(value, dict_converter=Object.fromEntries)


async def _rr_fetch_text(url):
    response = await pyfetch(url, cache='no-store')
    if not response.ok:
        raise RuntimeError(f'{url} を取得できませんでした（HTTP {response.status}）')
    return await response.string()


async def _rr_load_robot_python():
    """
    走行体の Python を置く。

    モジュール（Common/・2026/・2025/）は、走行体と同じく平らに import できる所（いまの
    フォルダ）へ。SCRIPT（<年>/recog/）は、走行体と同じ並びで _RR_ROOT の下へ。
    """
    manifest = json.loads(await _rr_fetch_text('python/robot-python.json'))
    for name in manifest['modules']:
        with open(name, 'w') as out:
            out.write(await _rr_fetch_text(f'python/{name}'))
    for rel in manifest['scripts']:
        dest = os.path.join(_RR_ROOT, rel)
        os.makedirs(os.path.dirname(dest), exist_ok=True)
        with open(dest, 'w') as out:
            out.write(await _rr_fetch_text(f'python/robot/{rel}'))
    # 走行中に差し替える運び先など。走行体と同じ所（./sorot_spike/SubScenario/）へ
    os.makedirs(_RR_SUBSCENARIO, exist_ok=True)
    for name in manifest.get('subScenario', []):
        with open(os.path.join(_RR_SUBSCENARIO, name), 'w', encoding='utf-8') as out:
            out.write(await _rr_fetch_text(f'python/robot/SubScenario/{name}'))
    return manifest


_rr_manifest = await _rr_load_robot_python()

# --- スレッド（ブラウザでは立てられない）-------------------------------------------


def _rr_later(fn, delay_sec=0.0):
    """fn を JS のタイマーで1回呼ぶ。落ちても走行は止めない（走行体のスレッドと同じ）"""
    def run():
        try:
            fn()
        except Exception as exc:  # noqa: BLE001
            console.error(f'robot_runtime: スレッドの中でエラー: {exc}\n{traceback.format_exc()}')
    setTimeout(create_once_callable(run), max(0, int(delay_sec * 1000)))


#: JS 側から Python の関数を JSPI で呼ぶ（callPromising）。こう呼んだ Python の中でだけ、
#: time.sleep() が一時停止してブラウザに順番を返せる
_rr_call_promising = run_js(
    "(fn, name) => fn.callPromising().catch((e) => console.error('robot_runtime: スレッド ' + name, e))"
    ".finally(() => fn.destroy())")


def _rr_can_switch():
    """いまの Python が一時停止できるか（JSPI で呼ばれた中か）"""
    try:
        from pyodide.ffi import can_run_sync
        return bool(can_run_sync())
    except Exception:  # noqa: BLE001 — 古い Pyodide
        return False


def _rr_spawn(fn, name, delay_sec=0.0):
    """fn を、一時停止できる「スレッド」として走らせる。落ちても走行は止めない"""
    def body():
        try:
            fn()
        except Exception as exc:  # noqa: BLE001
            console.error(f'robot_runtime: スレッド {name} でエラー: {exc}\n{traceback.format_exc()}')

    proxy = create_proxy(body)
    setTimeout(create_once_callable(lambda: _rr_call_promising(proxy, name)), max(0, int(delay_sec * 1000)))


def _rr_install_threads():
    """
    threading.Thread / Timer / Event を、ブラウザで動く形にする。

    立てられるならそのまま立てる（いまの Pyodide では立てられず RuntimeError になる）。
    立てられなければ、JSPI で呼ぶ「スレッド」として走らせる。その中の time.sleep() と
    Event.wait() で、ほかのスレッドとブラウザに順番が回る。
    """
    original_start = threading.Thread.start
    if getattr(original_start, '_rr_patched', False):
        return
    original_is_alive = threading.Thread.is_alive
    original_wait = threading.Event.wait

    def start(self):
        try:
            return original_start(self)
        except RuntimeError:
            pass
        self._rr_alive = True

        def run(thread=self):
            try:
                if isinstance(thread, threading.Timer):
                    if not thread.finished.is_set():
                        thread.function(*thread.args, **thread.kwargs)
                        thread.finished.set()
                else:
                    thread.run()
            finally:
                thread._rr_alive = False

        delay = float(self.interval) if isinstance(self, threading.Timer) else 0.0
        _rr_spawn(run, self.name, delay)

    def is_alive(self):
        if hasattr(self, '_rr_alive'):
            return self._rr_alive
        return original_is_alive(self)

    def wait(self, timeout=None):
        """
        立っていなければ、短く sleep しながら待つ。sleep のたびに順番が回るので、
        立てる側（別の「スレッド」やブラウザ）が動ける。一時停止できない所では元の待ち方
        """
        if self.is_set():
            return True
        if not _rr_can_switch():
            return original_wait(self, timeout)
        end = None if timeout is None else time.monotonic() + float(timeout)
        while not self.is_set():
            remaining = None if end is None else end - time.monotonic()
            if remaining is not None and remaining <= 0:
                return False
            time.sleep(0.01 if remaining is None else min(0.01, remaining))
        return True

    start._rr_patched = True
    threading.Thread.start = start
    threading.Thread.is_alive = is_alive
    threading.Event.wait = wait


_rr_install_threads()

import et_sumo  # noqa: E402
import remote_control  # noqa: E402
from camera import RobotCamera  # noqa: E402
from robot_pose import RobotPose  # noqa: E402
from script_runner import ScriptRunner  # noqa: E402
from state import ResultStore  # noqa: E402
import config as _rr_config  # noqa: E402

#: その年の設定（config_2026）。走行体の ev3_python も load_season で同じものを読む
_rr_season = _rr_config.load_season()

# --- カメラ ------------------------------------------------------------------------


def _rr_frame_source():
    """
    前面カメラの読み取り用の画（960x576）を BGR で返す。無ければ None。

    **撮る前に、直前の1枚を描くのにかかった時間と同じだけ順番を返す。** 走行体の見張りの
    ループは「1枚見る → 間隔の残りだけ待つ」。ブラウザでは画を描くのが重く（ページが混むと
    1枚 100〜300ms。走行体は全部で 30ms）、残りが無くて一度も待たないまま回り続け、ページが
    止まる。実機でもカメラは次の映像が来るまで待つ所なので、ここで返す。描くのと同じだけ返せば、
    どれだけ重くてもページは半分の時間は動ける（上限 0.3 秒）。一時停止できない所では何もしない
    """
    if _rr_can_switch():
        time.sleep(min(0.3, max(0.001, _rr_frame_clock['grab'])))
    t0 = time.monotonic()
    try:
        return _rr_grab_frame()
    finally:
        _rr_frame_clock['grab'] = time.monotonic() - t0


#: 直前の1枚を描くのにかかった時間[s]
_rr_frame_clock = {'grab': 0.0}


def _rr_grab_frame(size=None):
    """
    前面カメラの読み取り用の画を、今の景色で描き直して BGR で返す。

    size=(幅, 高さ) を渡すと、その大きさで描く（sim/canvas.js の captureView）。色だけを
    見る読み取りは小さい絵で足りるので、描くのも調べるのも軽くなる。

    画面の「カメラON」を外しているときは None（カメラが付いていない走行体と同じ）。同じシナリオの
    まま「カメラで直す／直さない」を見比べるため
    """
    if js_value(getattr(window, '__cameraEnabled', None)) is False:
        return None
    manager = js_value(getattr(window, 'canvasManager', None))
    if manager is None:
        return None
    if size is None:
        if not manager.captureReadView():
            return None
        canvas = manager.readCanvas
    else:
        canvas = js_value(manager.captureView(int(size[0]), int(size[1])))
        if canvas is None:
            return None
    width, height = int(canvas.width), int(canvas.height)
    ctx = canvas.getContext('2d', _rr_to_js({'willReadFrequently': True}))
    data = ctx.getImageData(0, 0, width, height).data.to_bytes()
    rgba = np.frombuffer(data, dtype=np.uint8).reshape((height, width, 4))
    return cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGR)


RobotCamera.shared().set_frame_source(_rr_frame_source)

#: キャリーボトルの色を見るときの絵の大きさ。読み取り用（960x576）の縦横半分＝画素は4分の1。
#: 色は ROI に占める割合で決まるので（Common/bottle_color_monitor.py）、大きさに依らない
_RR_COLOR_VIEW_SIZE = (480, 288)

#: 色を見る絵を、直前に描くのにかかった時間[s]
_rr_color_clock = {'grab': 0.0}


def _rr_color_frame_source():
    """
    キャリーボトルの色を見るための小さい絵を BGR で返す。無ければ None。
    _rr_frame_source と同じく、撮る前に直前の1枚を描いたぶんだけ順番を返す
    """
    if _rr_can_switch():
        time.sleep(min(0.3, max(0.001, _rr_color_clock['grab'])))
    t0 = time.monotonic()
    try:
        return _rr_grab_frame(_RR_COLOR_VIEW_SIZE)
    finally:
        _rr_color_clock['grab'] = time.monotonic() - t0


def _rr_install_lazy_bottle_camera(task_cls, reader_cls, camera_cls, idle_sleep=0.1, color_frame=None):
    """
    キャリーボトルの色の読み取り（BottleColorTask）を、判別中だけ撮る形にする。

    【なぜ要るか】
    走行体は走るたびに色の読み取りを起こし（config_2026.start_recognizers）、走り終わるまで
    30Hz でカメラを読み続ける。実機ではカメラが映像を流しているので読むのはほぼただだが、
    シミュレータでは1枚ごとに読み取り用の画（960x576）を描き直し、OpenCV に掛ける。
    色が意味を持つのは判別中（SCRIPT の bottle_color.py が judge() を呼んでから、決まるか
    打ち切るまで）だけで、それ以外の読みはすべて捨てられる（BottleColorTask._decide）。
    これでページの時間の半分近くを使っていた。

    【差し替えること（走行体のコードは変えない）】
      1. 色の読み取り器（BottleColorReader.read）… 判別中でなければ撮らずに「見えていない」を
         返し、少し休む（休まないと読み取りのループが空回りする）
      2. 最後の1枚（RobotCamera.latest_frame）… ヒントカードと向きの打ち直しは、色の読み取りが
         回っているあいだ自分では撮らず、その最後の1枚を借りる。判別中でなければ最後の1枚は
         古いままなので、借りる代わりに今の景色で1枚撮る。実機で2か所から読まないのは
         VideoCapture が壊れるからで、シミュレータの取り込み口は関数なので2か所から呼べる
      3. 判別中の色の読み取りは、色を見るためだけの小さい絵（color_frame）を見る。
         このとき最後の1枚は色の読み取りでは入れ替わらないので、借りる側はいつも今の景色で撮る

    :param task_cls: BottleColorTask
    :param reader_cls: BottleColorReader
    :param camera_cls: RobotCamera
    :param idle_sleep: 判別中でないときに1周で休む時間[s]
    :param color_frame: 判別中に色を見る絵を返す関数。None ならカメラから撮る（大きい絵）
    """
    if getattr(reader_cls.read, '_rr_lazy', False):
        return

    def judging():
        task = task_cls._instance
        return task is not None and bool(getattr(task, '_judging', False))

    original_read = reader_cls.read

    def read(self, frame=None):
        if frame is None:
            if not judging():
                time.sleep(idle_sleep)
                return 'none'
            if color_frame is not None:
                frame = color_frame()
                if frame is None:
                    return 'none'
        return original_read(self, frame)

    original_latest = camera_cls.latest_frame

    def latest_frame(self):
        if getattr(self, '_frame_source', None) is not None and (color_frame is not None or not judging()):
            return self.grab()
        return original_latest(self)

    read._rr_lazy = True
    reader_cls.read = read
    camera_cls.latest_frame = latest_frame


try:
    from bottle_color_task import BottleColorTask as _RrBottleColorTask  # noqa: E402
    from bottle_color_reader import BottleColorReader as _RrBottleColorReader  # noqa: E402
    _rr_install_lazy_bottle_camera(_RrBottleColorTask, _RrBottleColorReader, RobotCamera,
                                   color_frame=_rr_color_frame_source)
except ImportError:
    pass    # ボトルデリバリーの無い年


#: 重い処理の1回のあと、掛かった時間の何倍だけ順番を返すか。3倍なら、ページが止まるのは
#: 4分の1以下になる（2倍で 750mm 先の読めないカードを試し続けたとき 4秒のうち 1.4秒止まった）。
#: ヒントカードは止まる行の中で読むので、試す回数が減っても読める（近ければ1枚目で読める）
_RR_HEAVY_YIELD_RATIO = 3.0

#: 1回で返す時間の上限[s]。読み取りが極端に重い1枚のあと、長く止まりすぎないように
_RR_HEAVY_YIELD_MAX = 0.5


def _rr_install_heavy_yield(cls, name, ratio=_RR_HEAVY_YIELD_RATIO, max_sleep=_RR_HEAVY_YIELD_MAX,
                            sleep=None, clock=None):
    """
    cls.name（重い1回の処理）のあとで、掛かった時間に比例して順番を返す形にする。

    【なぜ要るか】
    走行体のヒントカードの読み取り（2026/hint_card_capture.py）は、1枚試すごとに 0.03秒しか
    休まない（FRAME_INTERVAL）。実機は読み取りが別のスレッドで、走行は止まらない。ブラウザでは
    ページと同じスレッドで、OpenCV の1回（読めない絵で約 80ms、ときに 350ms）のあいだページが
    止まる。読める距離に来る前から試し続けるので、キャリーボトルを取ったあとの区間で、
    4秒のうち 2.8秒ページが止まっていた。

    1回のあと、その ratio 倍だけ sleep する（一時停止できる所＝「スレッド」の中だけ）。
    1回の重さは変わらないので、1回が長いときの引っかかりは残る。

    :param cls: 差し替えるクラス（HintCardCapture）
    :param name: 差し替えるメソッドの名前（_decode）
    :param sleep: 待ち方（テスト用。省略時 time.sleep）
    :param clock: 時計（テスト用。省略時 time.monotonic）
    """
    original = getattr(cls, name)
    if getattr(original, '_rr_heavy_yield', False):
        return
    do_sleep = sleep or time.sleep
    now = clock or time.monotonic

    def wrapped(self, *args, **kwargs):
        t0 = now()
        try:
            return original(self, *args, **kwargs)
        finally:
            spent = now() - t0
            if spent > 0 and _rr_can_switch():
                do_sleep(min(max_sleep, spent * ratio))

    wrapped._rr_heavy_yield = True
    setattr(cls, name, wrapped)


try:
    from hint_card_capture import HintCardCapture as _RrHintCardCapture  # noqa: E402
    _rr_install_heavy_yield(_RrHintCardCapture, '_decode')
    # 読めた絵・読めなかった絵を残さない。走行体では後から見て切り分けるためのものだが、
    # シミュレータではブラウザのメモリに溜まるだけで、誰も見ない。読めない間は 0.1秒おきに
    # 960x576 を JPEG にして書くので、そのたびにページが止まっていた
    _RrHintCardCapture.SNAPSHOT_ENABLE = 0
except ImportError:
    pass    # ETラリーの無い年


# --- 二次元コードの読み取り器（zxing-cpp）---------------------------------------------


def _rr_make_zxing_shim(read_texts):
    """
    走行体の zxingcpp（zxing-cpp の Python 版）の代わりになるモジュールを作る。

    【なぜ要るか】
    走行体は二次元コードを zxing-cpp で読む（Common/qr_decoder.py。ヒントカードは ZXING_ONLY で
    zxing-cpp だけで読む）。ブラウザの Python には zxing-cpp が無く、シミュレータだけ OpenCV の
    重い道に落ちて、1回 0.1〜0.4秒ページが止まっていた。zxing-cpp そのものを WebAssembly にした
    zxing-wasm（sim/vision/zxing-reader.js）で読み、走行体のコードからは zxing-cpp に見せる。

    走行体が使うのは read_barcodes(白黒かBGRの絵, formats=BarcodeFormat.QRCode) と、結果の .text だけ。
    その2つだけを持たせる（ほかの書式は読まない。二次元コードしか使っていない）。

    :param read_texts: (RGBA の bytes, 幅, 高さ) → 読めた文字列のリスト
    """
    import types

    module = types.ModuleType('zxingcpp')

    class BarcodeFormat:
        QRCode = 'QRCode'

    class Barcode:
        """読めた1つ。走行体は .text だけを見る"""

        def __init__(self, text):
            self.text = text
            self.format = BarcodeFormat.QRCode
            self.valid = True

    def read_barcodes(image, formats=None, **_options):
        if image is None or getattr(image, 'size', 0) == 0:
            return []
        height, width = int(image.shape[0]), int(image.shape[1])
        rgba = np.empty((height, width, 4), dtype=np.uint8)
        if image.ndim == 2:
            rgba[:, :, 0] = image
            rgba[:, :, 1] = image
            rgba[:, :, 2] = image
        else:
            # OpenCV の並び（BGR）を RGB へ
            rgba[:, :, 0] = image[:, :, 2]
            rgba[:, :, 1] = image[:, :, 1]
            rgba[:, :, 2] = image[:, :, 0]
        rgba[:, :, 3] = 255
        return [Barcode(text) for text in read_texts(rgba.tobytes(), width, height) if text]

    module.BarcodeFormat = BarcodeFormat
    module.Barcode = Barcode
    module.read_barcodes = read_barcodes
    return module


def _rr_zxing_read_texts(rgba_bytes, width, height):
    """
    zxing-wasm で読む（JS の約束が済むまで、この「スレッド」だけ一時停止して待つ）。

    一時停止できない所（JSPI の外）では待てないので読まない。走行体の読み取り
    （ヒントカード・床のコード）は「スレッド」の中で動くので、そちらでは読める
    """
    if not _rr_can_switch():
        return []
    from pyodide.ffi import run_sync
    found = run_sync(window.__simZxing.read(_to_js(rgba_bytes), int(width), int(height)))
    return [str(text) for text in found.to_py()]


async def _rr_prepare_zxing():
    """zxing-wasm を読み込めたら、zxingcpp の代わりを置く。読めなければ OpenCV のまま"""
    bridge = js_value(getattr(window, '__simZxing', None))
    if bridge is None:
        return False
    try:
        if not bool(await bridge.prepare()):
            return False
    except Exception as exc:  # noqa: BLE001 — 読めなくても OpenCV で読める
        console.warn(f'robot_runtime: zxing-cpp（wasm）を用意できませんでした: {exc}')
        return False
    sys.modules['zxingcpp'] = _rr_make_zxing_shim(_rr_zxing_read_texts)
    return True


_rr_zxing_ready = await _rr_prepare_zxing()

try:
    import bottle_color_reader as _rr_bottle_color_reader  # noqa: E402
    # 【デバッグ】の定期保存（1秒おき）も同じ理由で止める。色が決まった瞬間の1枚は残す
    _rr_bottle_color_reader.DEBUG_SNAPSHOT_SEC = 0
except ImportError:
    pass    # ボトルデリバリーの無い年


def _rr_use_sim_camera(module, name='ROBOT_CAMERA', sim_name='SIM_CAMERA'):
    """
    走行体のカメラの取り付けを、シミュレータのカメラの値にする。

    走行体のコードは ROBOT_CAMERA を既定の引数として持っている（attach(geometry=ROBOT_CAMERA)）。
    既定の引数は同じ物を指しているので、**その物の中身を書き替える**（凍った dataclass でも
    object.__setattr__ なら書ける）。走行体のコードは変えない。
    """
    robot = getattr(module, name)
    sim = getattr(module, sim_name)
    for field in robot.__dataclass_fields__:
        object.__setattr__(robot, field, getattr(sim, field))


_rr_use_sim_camera(et_sumo)

import heading_reader  # noqa: E402
from grid_heading import CameraConstants  # noqa: E402


def _rr_configure_camera(camera_json):
    """
    走行体の前面カメラの定数（heading_reader.ROBOT_CAMERA）を、シミュレータのカメラの値にする。
    ゲートの見張り（gate_finder）や向き（heading_task）はこれを使う。JS が sim/camera-model.js の値を渡す。

    既定の引数などで同じ物を指しているので、その物の中身を書き替える。
    """
    sim = CameraConstants.from_json(json.loads(camera_json))
    for field in heading_reader.ROBOT_CAMERA.__dataclass_fields__:
        object.__setattr__(heading_reader.ROBOT_CAMERA, field, getattr(sim, field))
    return True

# --- SCRIPT を走らせる（走行体の ev3_python.handle_request と同じ2行）-------------------

ScriptRunner._instance = ScriptRunner(root=_RR_ROOT)


def _rr_run(script, request_json):
    """
    SCRIPT を1本走らせる。走行体の Python 部が要求を受けたときと同じ。

    :param request_json: 走行体の要求と同じ JSON（script / request_id / x / y / heading / sno / cno）
    :returns: 呼び出しを起こせたら True
    """
    try:
        req = json.loads(request_json) if request_json else {'script': script}
        RobotPose.get_instance().update_from_request(req)
        started = ScriptRunner.get_instance().run(script, int(req.get('request_id', 0)))
        # スクリプトが力士寄せなどの制御関数を差したら、C++ の口（RemoteControlBridge）へつなぐ。
        # スクリプトはタイマーで走るので、その後に見る
        _rr_later(_rr_connect_remote_control)
        return bool(started)
    except Exception as exc:  # noqa: BLE001 — 走行は止めない
        console.error(f'robot_runtime: SCRIPT {script} の実行に失敗: {exc}\n{traceback.format_exc()}')
        return False


# --- リモートコントロール（FUNCNO 18）の口 ------------------------------------------

_rr_state = {'proxy': None}


def _rr_on_observation(js_observation):
    """RemoteControlBridge から毎周期呼ばれる。走行体の Python と同じ応答を返す"""
    request = json.loads(JSON.stringify(js_observation))
    response = remote_control.RemoteControl.get_instance().handle_observation(request)
    return _rr_to_js(response)


def _rr_connect_remote_control():
    """走行体の RemoteControl に制御関数が差さっていれば、RemoteControlBridge につなぐ"""
    if remote_control.RemoteControl.get_instance()._controller is None:
        return False
    bridge = js_value(getattr(window, 'REMOTECONTROL', None))
    if bridge is None:
        console.error('robot_runtime: RemoteControlBridge（window.REMOTECONTROL）がありません')
        return False
    if _rr_state['proxy'] is None:
        _rr_state['proxy'] = create_proxy(_rr_on_observation)
    bridge.setController(_rr_state['proxy'])
    return True


def _rr_reset():
    """
    走らせ直すとき。**走行体は走るたびに Python 部を起動し直す**ので、持ち越すものは無い。

    シミュレータは Python を読み直さない代わりに、走行体のモジュールにある「1つだけ持つ物」
    （get_instance / reset_instance を持つクラス）を全部作り直す。見張りのスレッドは
    reset_instance の中で止まる（GateCheckTask.reset_instance → stop）
    """
    bridge = js_value(getattr(window, 'REMOTECONTROL', None))
    if bridge is not None and _rr_state['proxy'] is not None:
        bridge.clearController()
    for name in _rr_manifest['modules']:
        module = sys.modules.get(name[:-3])
        if module is None:
            continue
        for value in list(vars(module).values()):
            if (isinstance(value, type) and value.__module__ == module.__name__
                    and callable(getattr(value, 'reset_instance', None))):
                try:
                    value.reset_instance()
                except Exception as exc:  # noqa: BLE001
                    console.warn(f'robot_runtime: {name} の {value.__name__} を作り直せません: {exc}')
    ScriptRunner._instance = ScriptRunner(root=_RR_ROOT)
    _rr_connect_pc()
    return True


# --- PC へ繋ぐ口（走行体の Common/pc_client.py の PCClient）----------------------------


class _RrPcClient:
    """
    走行体の PCClient と同じ口（send_and_receive）。

    走行体では Python 部が PC-System へ直に繋ぐ。シミュレータでは PC への接続を JS
    （sim/dom-ui/pc-system-client.js）が持っているので、そこから送り、応答が届くまで
    sleep しながら待つ（呼ぶのはスレッドの中。ヒントカードの読み取りなど）
    """

    POLL_SEC = 0.05

    def send_and_receive(self, payload, timeout=None):
        api = js_value(getattr(window, '__simPcSystem', None))
        if api is None:
            raise ConnectionError('PC へ送る口（window.__simPcSystem）がありません')
        request_id = js_value(api.sendFromRobot(json.dumps(payload, ensure_ascii=False)))
        if request_id is None:
            raise ConnectionError('PC-System へつながっていません')
        timeout = _rr_config.PC_TIMEOUT if timeout is None else timeout
        deadline = time.time() + timeout
        while time.time() < deadline:
            got = js_value(api.takeResponse(request_id))
            if got is not None:
                return json.loads(got)
            time.sleep(self.POLL_SEC)
        raise TimeoutError('PCからの応答がタイムアウト時間（%s秒）内に受信できませんでした。' % timeout)


_rr_pc_client = _RrPcClient()


def _rr_connect_pc():
    """PC へ繋ぐ口を渡す。走行体の ev3_python の起動時と同じく、その年の set_pc_client を呼ぶ"""
    hook = getattr(_rr_season, 'set_pc_client', None)
    if callable(hook):
        hook(_rr_pc_client)


_rr_connect_pc()


# --- 走行体の Python 部の起動（走行体の ev3_python の start_recognizers）------------------


def _rr_start(scenario_json):
    """
    走り出すとき。走行体は走るたびに Python 部を起動し、その年の認識処理を起こす
    （ev3_python → config_2026.start_recognizers。キャリーボトルの色の読み取り、
    ヒントカードが読めたら区間を終わらせる制御関数など）。

    :param scenario_json: 走らせるシナリオ。走行体の SubScenario/custom.json にあたる
                          （SCENARIO_FILE。打ち直しの行を探すのに読む）
    """
    try:
        if scenario_json:
            with open(os.path.join(_RR_SUBSCENARIO, 'custom.json'), 'w', encoding='utf-8') as out:
                out.write(scenario_json)
        hook = getattr(_rr_season, 'start_recognizers', None)
        if callable(hook):
            hook()
        _rr_connect_remote_control()
        return True
    except Exception as exc:  # noqa: BLE001 — 走行体と同じく、起こせなくても走行は止めない
        console.error(f'robot_runtime: その年の認識処理を起こせませんでした: {exc}\n{traceback.format_exc()}')
        return False


# --- C++ へのシナリオ更新（走行体の /dev/shm/sorot_scenario_update.json）---------------


def _rr_take_scenario_update():
    """
    Python 部が置いたシナリオ更新を引き取る。無ければ空文字。

    走行体の C++（PythonCommServer::takeScenarioUpdate）と同じく、置き場を .taken へ
    rename してから読み、消す。置いたのは ScenarioMailbox（ボトルの色の運び先など）
    """
    path = _rr_config.COMM_SCN_FILE
    taken = path + '.taken'
    try:
        os.replace(path, taken)
    except OSError:
        return ''           # 無い（ほとんどの周期はこれ）
    try:
        with open(taken, encoding='utf-8') as f:
            return f.read()
    finally:
        try:
            os.remove(taken)
        except OSError:
            pass


# --- C++ が読む判定結果（走行体の ev3_python.build_result と同じもの）----------------


def _rr_result():
    """
    いまの判定結果の写し。走行体の Python 部が C++ の読める所へ置くものと同じ
    （ResultStore の写しに、その年の項目 config_2026.fill_result を足す）。
    JS の ResultRead.run（C++ の ResultRead::run と同じ）が 100ms ごとに読む
    """
    try:
        snapshot = ResultStore.get_instance().snapshot()
        fill = getattr(_rr_season, 'fill_result', None)
        if callable(fill):
            fill(snapshot)
        return json.dumps(snapshot, ensure_ascii=False, default=str)
    except Exception as exc:  # noqa: BLE001 — 読めなくても走行は続ける
        console.error(f'robot_runtime: 判定結果を組めません: {exc}')
        return ''


# --- C++ が置く自己位置（走行体の /dev/shm/sorot_pose.json）-----------------------------


def _rr_put_pose(pose_json):
    """
    自己位置を置く。走行体の C++ が 100ms ごとに置くもの（PythonCommServer::putPose）と同じ置き場・
    同じ置き方（一時ファイル＋rename）。Python 部は Common/pose.py の PoseReader で読む。
    JS の src/RoughSpot/Cpp/PoseReport.js が呼ぶ（座標は走行体の座標系に直してある）
    """
    try:
        path = _rr_config.COMM_POSE_FILE
        os.makedirs(os.path.dirname(path), exist_ok=True)
        tmp = path + '.tmp'
        with open(tmp, 'w', encoding='utf-8') as out:
            out.write(pose_json)
        os.replace(tmp, path)
        # 新しさは置いた時刻で見る（PoseReader.read_fresh）。ブラウザのファイルは秒より細かく
        # 持たないことがあるので、いまの時刻を入れ直す
        now = time.time()
        os.utime(path, (now, now))
        return True
    except Exception as exc:  # noqa: BLE001 — 置けなくても走行は続ける
        console.warn(f'robot_runtime: 自己位置を置けません: {exc}')
        return False


def _rr_pose_now():
    """確かめ用。Python 部から見た自己位置（走行体の PoseReader.read_fresh と、置き場の古さ）"""
    from pose import PoseReader
    try:
        age = time.time() - os.stat(_rr_config.COMM_POSE_FILE).st_mtime
    except OSError:
        age = None
    found = PoseReader.read_fresh()
    return json.dumps({
        'age_s': age,
        'fresh': None if found is None else {'x': found.x, 'y': found.y, 'heading_deg': found.heading_deg,
                                             'sno': found.sno, 'cno': found.cno},
        'request': RobotPose.get_instance().snapshot(),
    }, ensure_ascii=False, default=str)


# --- 走行体の流れの入口（C++ や PC から来るもの）--------------------------------------


def _rr_route_registered(senario_kbn, rows_json):
    """PC から経路が届いて置いたとき。走行体の route_register が呼ぶ config_2026.on_route_registered と同じ"""
    hook = getattr(_rr_season, 'on_route_registered', None)
    if callable(hook):
        hook(int(senario_kbn), json.loads(rows_json))
    return True


def _rr_watch_gates(seconds):
    """ゲートを見張る（画面の「ゲートを見張る」から）。走行体の GateCheckTask.watch と同じ。None なら走り終わりまで"""
    from gate_check_task import GateCheckTask
    return bool(GateCheckTask.get_instance().watch(0, None if seconds is None else float(seconds)))


def _rr_set_switch(path, on):
    """走行体の置き場のつまみ（例 gate_steer.ENABLE_SWITCH）を書く。1行目に 1 か 0"""
    directory = os.path.dirname(path)
    if directory:
        os.makedirs(directory, exist_ok=True)
    with open(path, 'w', encoding='utf-8') as out:
        out.write('1\n' if on else '0\n')
    return True


# --- 走行体のログを JS の出力欄へ ------------------------------------------------------


class _RrLogToJs(logging.Handler):
    def emit(self, record):
        try:
            sink = js_value(getattr(window, '__simRobotLog', None))
            text = self.format(record)
            if sink is not None:
                sink(record.levelname, record.name, text)
            else:
                console.log(text)
        except Exception:  # noqa: BLE001
            pass


_rr_handler = _RrLogToJs()
_rr_handler.setFormatter(logging.Formatter('%(message)s'))
for _rr_name in ('script_runner', 'et_sumo', 'remote_control', 'gate_check_task', 'gate_steer',
                 'heading_task', 'grid_position', 'heading_fix_task', 'coord_correct',
                 'floor_code', 'floor_code_observer', 'floor_code_gate', 'scenario_rows',
                 'ground_plane', 'hint_card_capture', 'qr_decoder', 'handler',
                 'bottle_color_task', 'bottle_color_reader', 'hint_card_advance', 'scenario_update',
                 'config_2026'):
    _rr_logger = logging.getLogger(_rr_name)
    _rr_logger.addHandler(_rr_handler)
    _rr_logger.setLevel(logging.INFO)
    _rr_logger.propagate = False

# --- 画面の記録（シミュレータだけ）----------------------------------------------------


def _rr_watch_heading():
    """
    向きを測るたびに、測った中身を画面へ知らせる（window.__simOnHeadingMeasured）。
    答え合わせ（本当の向きと並べる）と、走り終わりの数（撮った・測れた）に使う。
    走行体の HeadingTask.measure はそのまま呼ぶ。戻ったあとに last を見て渡すだけ
    """
    from heading_task import HeadingTask
    original = HeadingTask.measure
    if getattr(original, '_rr_watched', False):
        return

    def measure(self, request_id=0):
        result = original(self, request_id)
        try:
            hook = js_value(getattr(window, '__simOnHeadingMeasured', None))
            if hook is not None:
                hook(json.dumps(result if result is not None else (self.last or {'ok': False}),
                                ensure_ascii=False, default=str))
        except Exception:  # noqa: BLE001 — 画面の記録が転んでも走行は止めない
            pass
        return result

    measure._rr_watched = True
    HeadingTask.measure = measure


_rr_watch_heading()


def _rr_probe():
    """
    「スレッド」の中で一時停止できるかを確かめる（JSPI が効いているか）。結果は window.__simRobotProbe
    """
    def body():
        import sys as _sys
        result = {'can_run_sync': _rr_can_switch(), 'pyodide': getattr(_sys.modules.get('pyodide'), '__version__', '?')}
        # sleep の前後で、JS のタイマーが何回回ったか（JS 側で window.__simRobotTicks を数えておく）
        before = int(getattr(window, '__simRobotTicks', 0) or 0)
        t0 = time.monotonic()
        time.sleep(0.5)
        result['sleep_ms'] = round((time.monotonic() - t0) * 1000)
        result['js_ticks_during_sleep'] = int(getattr(window, '__simRobotTicks', 0) or 0) - before
        # Event.wait（差し替えたほう）でも見る
        ev = threading.Event()
        before = int(getattr(window, '__simRobotTicks', 0) or 0)
        ev.wait(0.5)
        result['js_ticks_during_wait'] = int(getattr(window, '__simRobotTicks', 0) or 0) - before
        window.__simRobotProbe = _rr_to_js(result)
    _rr_spawn(body, 'probe')
    return True


window.__simRobot = _rr_to_js({
    'probe': _rr_probe,
    'run': _rr_run,
    'reset': _rr_reset,
    'configureCamera': _rr_configure_camera,
    'result': _rr_result,
    'putPose': _rr_put_pose,
    'start': _rr_start,
    'takeScenarioUpdate': _rr_take_scenario_update,
    'poseNow': _rr_pose_now,
    'routeRegistered': _rr_route_registered,
    'watchGates': _rr_watch_gates,
    'setSwitch': _rr_set_switch,
    'scripts': _rr_to_js(_rr_manifest['scripts']),
})

console.log(f'PyScript: robot_runtime_bridge.py が走行体の Python を載せました'
            f'（モジュール {len(_rr_manifest["modules"])} 本・SCRIPT {len(_rr_manifest["scripts"])} 本）')
