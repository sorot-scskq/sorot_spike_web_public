"""
前面カメラで、ET ラリーのグリッドに対する向きのずれを測る（走行体の Python）。

測る処理そのものは grid_heading.py。ここは「映像を 1枚もらって測り、結果を覚えておく」口。
いまは測って知らせるだけで、走りは変えない（ずれを把握する段階）。

【映像の入り口は差し替える】
  走行体        RobotCamera.shared().latest_frame()（自分で read() しないこと）
  シミュレータ  sim/pyscript/grid_heading_bridge.py が前面カメラの読み取り用の画を渡す

【カメラの定数】
  SIM_CAMERA   シミュレータ（sim/camera-model.js と同じ値）。シミュレータでは JS から渡し直す
  ROBOT_CAMERA 実機。**まだ測っていない。仮の値**。俯角は 0.1° の桁で合わせないと、黒線で
               測ったときに 1°近く外れる（docs/ETラリー_カメラで向きを補正する検証.md §4.4）

使い方

    reader = HeadingReader(frame_source=camera.latest_frame, camera=ROBOT_CAMERA)
    result = reader.read()
    if result['ok']:
        print(result['deviation_deg'])   # 走行体の向き − いちばん近いグリッドの向き
"""

from __future__ import annotations

import time
from dataclasses import asdict
from typing import Callable, Optional

try:
    from grid_heading import CameraConstants, Options, estimate_heading
except ImportError:  # パッケージとして読み込まれた場合
    from .grid_heading import CameraConstants, Options, estimate_heading

#: シミュレータの前面カメラ（読み取り用の画 960x576）
SIM_CAMERA = CameraConstants(model='sim', width=960, height=576, hfov_deg=70.0, vfov_deg=36.0,
                             eye_height_mm=170.0, tilt_deg=20.0)

#: 実機の前面カメラ。1280x720・視点 170mm は実機の値。画角と俯角は**未実測の仮の値**
ROBOT_CAMERA = CameraConstants(model='pinhole', width=1280, height=720, hfov_deg=70.0, vfov_deg=42.0,
                               eye_height_mm=170.0, tilt_deg=20.0)


class HeadingReader:
    """
    映像を 1枚もらって、グリッドに対する向きのずれを測る。

    :param frame_source: BGR の画像（numpy）を返す関数。撮れなければ None
    :param camera: カメラの定数。映像の大きさが違えば、画角はそのままで合わせて使う
    :param options: 測り方の細かい設定（grid_heading.Options）
    """

    def __init__(self, frame_source: Callable[[], object], camera: CameraConstants = SIM_CAMERA,
                 options: Optional[Options] = None, clock: Callable[[], float] = time.monotonic):
        self.frame_source = frame_source
        self.camera = camera
        self.options = options or Options()
        self._clock = clock
        self.last: Optional[dict] = None

    def read(self) -> dict:
        """
        1枚撮って測る。

        :returns: grid_heading.HeadingResult を辞書にしたもの（points は除く）に、
                  elapsed_ms（測るのにかかった時間）と width / height（映像の大きさ）を足したもの
        """
        frame = self.frame_source()
        if frame is None:
            result = {'ok': False, 'reason': '映像が無い', 'deviation_deg': None, 'source': ''}
            self.last = result
            return result
        height, width = frame.shape[:2]
        camera = self.camera.with_size(width, height)
        started = self._clock()
        found = asdict(estimate_heading(frame, camera, self.options))
        found.pop('points', None)
        found['elapsed_ms'] = round((self._clock() - started) * 1000, 1)
        found['width'] = width
        found['height'] = height
        self.last = found
        return found
