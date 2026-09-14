"""
走行体上の Python のリモートコントロール（ET相撲の力士寄せ）を、シミュレータのブラウザ内
（PyScript）で動かす。

実機では C++ が観測をファイルに置き、ラズパイの Python（remote_control.py）が応答を置く。
シミュレータにはそのやりとりが無いので、RemoteControlBridge（JS）が毎周期この
handle_observation を直接呼ぶ。応答は実機と同じく次の周期に取り込まれる。

動かすのは実機と同じファイル（書き換えない）。

  Common/remote_control.py   リモート側の窓口（観測 → 応答 {"messages": [...]}）
  Common/bottle_locator.py   力士ボトルを映像から探す
  2026/et_sumo.py            力士寄せの段取り

ここで差し替えるのは、実機でしか用意できない2つだけ。

  映像の入り口   実機: 前面カメラ（RobotCamera.shared）
                シミュレータ: 前面カメラの読み取り用の画（sim/canvas.js の captureReadView。
                認識結果の文字や枠が描かれていない。表示用の #cameraView は、水色の文字が
                ボトルの胴と同じ色に入って位置がずれるので読まない）
  後で呼ぶ口     実機: 別スレッド（threading.Timer）
                シミュレータ: setTimeout（Pyodide にはスレッドが無い）

【PyScript の名前空間】
  他の bridge と衝突しないよう、グローバルは _es_ 接頭辞を付ける。
  JS からは window.simEtSumo.* を使う（window.__simPython を作り直す bridge があり、
  そこに置くと消えることがあるため）。
"""

import json
import math
import traceback
from collections import deque

import pyodide_js
from js import JSON, Object, console, performance, setTimeout, window
from pyodide.ffi import create_once_callable, create_proxy
from pyodide.ffi import to_js as _to_js
from pyodide.http import pyfetch

# OpenCV（位置特定で使う）
await pyodide_js.loadPackage(['numpy', 'opencv-python'])

import cv2  # noqa: E402
import numpy as np  # noqa: E402


def _es_to_js(value):
    return _to_js(value, dict_converter=Object.fromEntries)


async def _es_load_modules():
    # /python/ で配られている（vite.config.js の PYTHON_SOURCE_DIRS）
    for name in ('config.py', 'bottle_locator.py', 'remote_control.py', 'et_sumo.py'):
        # 書き換えた Python を読み直させるため、ブラウザのキャッシュを使わない
        response = await pyfetch(f'python/{name}', cache='no-store')
        if not response.ok:
            raise RuntimeError(f'python/{name} を取得できませんでした（HTTP {response.status}）')
        with open(name, 'w') as out:
            out.write(await response.string())


await _es_load_modules()

import et_sumo  # noqa: E402
import remote_control  # noqa: E402

# --- 実機でしか用意できないものの差し替え ---------------------------------------

_es_frame = {'at': -1e9, 'bgr': None}
# 撮影は1回に1枚。同じ周期の中で二度読むことは無いが、念のため短い間は使い回す
_ES_FRAME_REUSE_MS = 30


def _es_frame_source():
    """前面カメラの読み取り用の画（960x576）を BGR で返す。縮めるのは et_sumo 側"""
    now = performance.now()
    if now - _es_frame['at'] < _ES_FRAME_REUSE_MS:
        return _es_frame['bgr']
    bgr = None
    manager = getattr(window, 'canvasManager', None)
    if manager is not None and manager.captureReadView():
        canvas = manager.readCanvas
        width, height = int(canvas.width), int(canvas.height)
        ctx = canvas.getContext('2d', _es_to_js({'willReadFrequently': True}))
        data = ctx.getImageData(0, 0, width, height).data.to_py()
        rgba = np.frombuffer(bytearray(data), dtype=np.uint8).reshape((height, width, 4))
        bgr = cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGR)
    _es_frame['at'] = now
    _es_frame['bgr'] = bgr
    return bgr


def _es_run_later(fn, delay_sec):
    def run():
        try:
            fn()
        except Exception as exc:  # 撮影が転んでも走行は止めない（実機と同じ）
            console.error(f'et_sumo: 撮影でエラー: {exc}\n{traceback.format_exc()}')
    setTimeout(create_once_callable(run), int(delay_sec * 1000))


# --- 撮影と指令の記録（実機なら ev3_python.log に出るもの）----------------------

_es_history = deque(maxlen=300)
_es_state = {'proxy': None, 'last_cmd': None}


def _es_note(kind, **fields):
    _es_history.append({'t': round(performance.now() / 1000, 2), 'kind': kind, **fields})


def _es_trace_locator(control):
    read = control.locator.read

    def traced():
        pose = read()
        if pose.get('found'):
            _es_note('photo', found=True, distance_mm=round(pose['distance_mm']),
                     bearing_deg=round(math.degrees(pose['bearing_rad']), 1))
        else:
            _es_note('photo', found=False)
        return pose

    control.locator.read = traced


def _es_on_observation(js_observation):
    """RemoteControlBridge から毎周期呼ばれる。実機の Python と同じ応答を返す"""
    request = json.loads(JSON.stringify(js_observation))
    response = remote_control.RemoteControl.get_instance().handle_observation(request)
    for msg in response.get('messages', []):
        if msg.get('type') != 'cmd':
            _es_note(msg.get('type'), **{k: v for k, v in msg.items() if k != 'commands'})
            continue
        key = (round(msg.get('forward', 0), 1), round(msg.get('turn', 0)), msg.get('status'))
        if key != _es_state['last_cmd']:
            _es_state['last_cmd'] = key
            _es_note('cmd', forward=msg.get('forward'), turn=msg.get('turn'), status=msg.get('status'),
                     sno=request.get('sno'), odo_mm=round((request.get('coordinate') or {}).get('distance', 0)))
    return _es_to_js(response)


def _es_start():
    """力士寄せを差し直し、RemoteControlBridge に差す。走らせるたびに段取りを頭から始める"""
    bridge = window.REMOTECONTROL
    if bridge is None:
        console.error('et_sumo: RemoteControlBridge（window.REMOTECONTROL）がありません')
        return False
    remote_control.RemoteControl.reset_instance()
    control = et_sumo.attach(frame_source=_es_frame_source, geometry=et_sumo.SIM_CAMERA,
                             run_later=_es_run_later)
    _es_trace_locator(control)
    _es_history.clear()
    _es_state['last_cmd'] = None
    if _es_state['proxy'] is None:
        _es_state['proxy'] = create_proxy(_es_on_observation)
    bridge.setController(_es_state['proxy'])
    console.log('PyScript: et_sumo（力士寄せ）を RemoteControl に差しました')
    return True


def _es_restart():
    """差さっているときだけ、力士寄せを作り直す（走り直すとき。sim/dom-ui/scenario-state.js）"""
    bridge = window.REMOTECONTROL
    if bridge is None or _es_state['proxy'] is None or not bridge.hasController():
        return False
    return _es_start()


def _es_stop():
    bridge = window.REMOTECONTROL
    if bridge is not None:
        bridge.clearController()
    remote_control.RemoteControl.reset_instance()
    return True


def _es_history_json():
    return json.dumps(list(_es_history), ensure_ascii=False)


def _es_state_json():
    """力士寄せの内部状態（止まったときに、どこで待っているかを見る）"""
    control = remote_control.RemoteControl.get_instance()._controller
    if control is None:
        return json.dumps({'controller': None})
    keys = ('_state', '_cmd_seq', '_shooting', '_shot_done', '_seen', '_start_distance',
            '_step_mm', '_bearing_deg', '_start_direction', '_target_rad', '_turn', '_scan_index')
    return json.dumps({k.lstrip('_'): getattr(control, k, None) for k in keys}, ensure_ascii=False, default=str)


window.simEtSumo = _es_to_js({
    'start': _es_start,
    'restart': _es_restart,
    'stop': _es_stop,
    'history': _es_history_json,
    'state': _es_state_json,
})

console.log('PyScript: et_sumo_bridge.py のバインドが完了しました。')
