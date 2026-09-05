"""
力士ボトル位置特定を、シミュレータのブラウザ内で OpenCV + PyScript で動かす。

走行体で動く本体は src/RoughSpot/Python/Common/bottle_locator.py。
ここは canvas から画素を取り、判定結果を JS へ渡す配線だけを持つ。

【PyScript の名前空間】
  他の bridge と衝突しないよう、グローバル関数は _bl_ 接頭辞を付ける。
  JS からは window.__simPython.bottleLocate.* を使う。
"""

import json
import traceback

import pyodide_js
from js import Object, alert, console, document, window
from pyodide.ffi import to_js as _to_js
from pyodide.http import pyfetch

# OpenCV（実機と同じ処理を通す）
await pyodide_js.loadPackage(['numpy', 'opencv-python'])

import cv2  # noqa: E402
import numpy as np  # noqa: E402


async def _bl_load_modules():
    response = await pyfetch('python/bottle_locator.py')
    with open('bottle_locator.py', 'w') as out:
        out.write(await response.string())


await _bl_load_modules()

from bottle_locator import (  # noqa: E402
    BottleLocator,
    compute_lateral_score,
    find_bottle_in_frame,
    lateral_zone,
)

CANVAS_ID = 'cameraView'


def _bl_to_js(value):
    return _to_js(value, dict_converter=Object.fromEntries)


def _bl_get_canvas_frame(canvas_id=CANVAS_ID):
    canvas = document.getElementById(canvas_id)
    if canvas is None:
        return None
    width = int(canvas.width)
    height = int(canvas.height)
    if width <= 0 or height <= 0:
        return None
    ctx = canvas.getContext('2d', _bl_to_js({'willReadFrequently': True}))
    image = ctx.getImageData(0, 0, width, height)
    return {'data': image.data, 'width': width, 'height': height}


def _bl_read_frame():
    """前面カメラから 1 枚読み、OpenCV で力士ボトルを探す。"""
    frame = _bl_get_canvas_frame()
    if not frame:
        return {'found': False, 'error': 'canvas_unavailable'}
    hit = find_bottle_in_frame(frame, {'min_label_ratio': 0.05})
    return hit


def _bl_detect_and_alert(event=None):
    """デバッグ用: 判定結果をアラート表示する。"""
    try:
        hit = _bl_read_frame()
        if hit.get('error'):
            alert('前面カメラ (cameraView) の映像を取得できませんでした。')
            return
        if not hit.get('found'):
            alert(
                '【PyScript OpenCV 力士ボトル位置特定】\n\n'
                '■ 結果: 見つかりませんでした\n\n'
                '・「カメラON」にチェックを入れ、前面カメラに映像があるか確認\n'
                '・力士ボトル（黒ラベル）が写っているか確認'
            )
            return

        zone_label = {'left': '左', 'center': '中央', 'right': '右'}.get(hit['zone'], hit['zone'])
        msg = (
            '【PyScript OpenCV 力士ボトル位置特定】\n\n'
            f"■ 検出: あり\n"
            f"■ 左右位置: {zone_label}\n"
            f"■ 位置スコア: {hit['lateral_score']:.1f} （中央=0, 左=+100, 右=-100）\n"
            f"■ 重心 (u, v): ({hit['u_center']:.1f}, {hit['v_base']:.1f})\n"
            f"■ サイズ: {hit['width_px']} x {hit['height_px']} px\n"
            f"■ 黒ラベル比率: {hit['label_ratio'] * 100:.1f}%"
        )
        alert(msg)
        console.log(f"PyScript BottleLocate: {json.dumps(hit)}")
    except Exception as exc:
        console.error(f"PyScript bottle_locate_bridge detect error: {exc}\n{traceback.format_exc()}")
        alert(f'位置特定中にエラーが発生しました:\n{exc}')


def _bl_reset():
    pass


# window.__simPython は認識処理ごとの窓口をまとめる（上書きしない）
if not hasattr(window, '__simPython') or window.__simPython is None:
    window.__simPython = _bl_to_js({})
window.__simPython.bottleLocate = _bl_to_js({
    'read': _bl_read_frame,
    'reset': _bl_reset,
    'computeLateralScore': compute_lateral_score,
    'lateralZone': lateral_zone,
})

window.pyDetectBottleLocate = _bl_detect_and_alert

btn = document.getElementById('pyDetectBottleLocateButton')
if btn:
    btn.onclick = _bl_detect_and_alert

console.log('PyScript: bottle_locate_bridge.py (OpenCV) のバインドが完了しました。')
