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

await pyodide_js.loadPackage(['numpy', 'opencv-python'])

import cv2  # noqa: E402
import numpy as np  # noqa: E402


def _gh_to_js(value):
    return _to_js(value, dict_converter=Object.fromEntries)


async def _gh_load_modules():
    # /python/ で配られている（vite.config.js の PYTHON_SOURCE_DIRS）
    for name in ('grid_heading.py', 'heading_reader.py'):
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
    manager = getattr(window, 'canvasManager', None)
    if manager is None or not manager.captureReadView():
        return None
    canvas = manager.readCanvas
    width, height = int(canvas.width), int(canvas.height)
    ctx = canvas.getContext('2d', _gh_to_js({'willReadFrequently': True}))
    data = ctx.getImageData(0, 0, width, height).data.to_py()
    rgba = np.frombuffer(bytearray(data), dtype=np.uint8).reshape((height, width, 4))
    return cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGR)


_gh_reader = heading_reader.HeadingReader(frame_source=_gh_frame_source, camera=heading_reader.SIM_CAMERA)


def _gh_configure(camera_json):
    """カメラの定数を差し替える（JS の sim/camera-model.js の値）"""
    _gh_reader.camera = grid_heading.CameraConstants.from_json(json.loads(camera_json))
    return True


def _gh_read():
    """1枚撮って測る。結果は JSON 文字列"""
    try:
        return json.dumps(_gh_reader.read(), ensure_ascii=False)
    except Exception as exc:  # 測るのに失敗しても画面は止めない
        console.error(f'grid_heading: 測るのに失敗: {exc}\n{traceback.format_exc()}')
        return json.dumps({'ok': False, 'reason': f'エラー: {exc}', 'deviation_deg': None, 'source': ''},
                          ensure_ascii=False)


# window.__simPython は認識処理ごとの窓口をまとめる（上書きしない）
if not hasattr(window, '__simPython') or window.__simPython is None:
    window.__simPython = _gh_to_js({})
window.__simPython.gridHeading = _gh_to_js({
    'read': _gh_read,
    'configure': _gh_configure,
})

console.log('PyScript: grid_heading_bridge.py のバインドが完了しました。')
