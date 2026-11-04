"""
ET ラリーのグリッドに対する向きのずれを、シミュレータのブラウザ内（PyScript + OpenCV）で測る。

動かすのは走行体と同じファイル（書き換えない）。

  2026/grid_heading.py     映像とカメラの定数から向きのずれを測る
  2026/heading_reader.py   映像を 1枚もらって測る口

ここで差し替えるのは映像の入り口だけ。

  実機          前面カメラ（RobotCamera.shared().latest_frame()）
  シミュレータ  前面カメラの読み取り用の画（sim/canvas.js の captureReadView。認識結果の
                文字や枠が描かれていない。表示用の #cameraView は重ね描きが入るので読まない）

カメラの定数は JS から configure で渡す（sim/camera-model.js が持っている値。画角を
変えたときにここを直し忘れないように）。

【PyScript の名前空間】
  他の bridge と衝突しないよう、グローバルは _gh_ 接頭辞を付ける。
  JS からは window.__simPython.gridHeading.* を使う。結果は JSON 文字列で返す
  （Python の辞書を PyProxy のまま渡すと、JS から hit.found のように読めないため）。
"""

import json
import traceback

import pyodide_js
from js import Object, console, window
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


def _gh_to_js(value):
    return _to_js(value, dict_converter=Object.fromEntries)


async def _gh_load_modules():
    # /python/ で配られている（vite.config.js の PYTHON_SOURCE_DIRS）
    # 使うのは測るところ（grid_heading / heading_reader）だけだが、同じページの走行体の
    # Python（robot_runtime_bridge.py）と同じ1そろいを置いておく。**1本でも足りないと
    # ブラウザでだけ ImportError になる**（母艦の pytest は src/ を直に読むので気づけない）
    for name in ('config.py', 'opencv.py', 'state.py', 'robot_pose.py',
                 'pose.py', 'pose_reader.py', 'scenario_update.py',
                 'scenario_rows.py', 'route_plan.py', 'route_correct.py',
                 'route_numbering.py', 'route_follower.py', 'route_register.py',
                 'route_place.py', 'floor_code.py', 'floor_code_heading.py',
                 'grid_position.py',
                 'grid_heading.py', 'heading_reader.py', 'heading_task.py'):
        # 書き換えた Python を読み直させるため、ブラウザのキャッシュを使わない
        response = await pyfetch(f'python/{name}', cache='no-store')
        if not response.ok:
            raise RuntimeError(f'python/{name} を取得できませんでした（HTTP {response.status}）')
        with open(name, 'w') as out:
            out.write(await response.string())


await _gh_load_modules()

import grid_heading  # noqa: E402
import heading_reader  # noqa: E402


def _gh_frame_source():
    """前面カメラの読み取り用の画を、今の景色で描き直して BGR で返す"""
    manager = js_value(getattr(window, 'canvasManager', None))
    if manager is None or not manager.captureReadView():
        return None
    canvas = manager.readCanvas
    width, height = int(canvas.width), int(canvas.height)
    ctx = canvas.getContext('2d', _gh_to_js({'willReadFrequently': True}))
    data = ctx.getImageData(0, 0, width, height).data.to_py()
    rgba = np.frombuffer(bytearray(data), dtype=np.uint8).reshape((height, width, 4))
    return cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGR)


#: 測るところ（走行体の HeadingReader）。カメラの定数の差し替え（configure）で触る。
#: 段取り（HeadingTask）は通さない。判定結果の置き場へ載せないため（_gh_measure）
_gh_reader = heading_reader.HeadingReader(frame_source=_gh_frame_source,
                                          camera=heading_reader.SIM_CAMERA)


def _gh_configure(camera_json):
    """カメラの定数を差し替える（JS の sim/camera-model.js の値）"""
    _gh_reader.camera = grid_heading.CameraConstants.from_json(json.loads(camera_json))
    return True


def _gh_measure():
    """1枚撮って測る。戻りは測った中身（JSON 文字列）

    **判定結果の置き場へは載せない。** これは画面の重ね描き（答え合わせ）と補正のボタン用。
    走行中に向きを直すのは、シナリオの SCRIPT「2026/recog/grid_heading.py」を走行体の
    Python のまま走らせたほう（sim/pyscript/robot_runtime_bridge.py）。Python は1つの
    ページで1つなので置き場（state.ResultStore）も共有になる。ここが載せると、画面で
    測っただけの値が向きの補正として当たり、走行体の通し番号（heading_seq）も食い違う。
    """
    try:
        return json.dumps(_gh_reader.read() or {'ok': False, 'reason': '測れない'},
                          ensure_ascii=False, default=str)
    except Exception as exc:  # 測るのに失敗しても画面は止めない
        console.error(f'grid_heading: 測るのに失敗: {exc}\n{traceback.format_exc()}')
        return json.dumps({'ok': False, 'reason': f'エラー: {exc}', 'deviation_deg': None, 'source': ''},
                          ensure_ascii=False)


# window.__simPython は認識処理ごとの窓口をまとめる（上書きしない）
if not hasattr(window, '__simPython') or js_value(window.__simPython) is None:
    window.__simPython = _gh_to_js({})
window.__simPython.gridHeading = _gh_to_js({
    'measure': _gh_measure,
    'configure': _gh_configure,
})

console.log('PyScript: grid_heading_bridge.py のバインドが完了しました。')
