"""
床の二次元コードで、グリッドに対する向きのずれを測る（走行体の Python）。

測る処理そのものは `Common/floor_code_heading.py`。ここは「映像を 1枚もらって測り、
`heading_reader.HeadingReader` と**同じ形の値**を返す」口。同じ形にしてあるので、
`heading_task.HeadingTask` にそのまま差せる。

    reader = FloorCodeReader(frame_source=camera.latest_frame)
    result = reader.read()
    if result['ok']:
        print(result['deviation_deg'])   # 走行体の向き − いちばん近いグリッドの向き

【grid_heading（灰色の丸と黒線）との違い】
見ているものが違う。**こちらは二次元コードのファインダパターン3個だけ**を見る。

    灰色の丸  実機のコントラストは 6.3%。布の陰影（7.0%）より薄く、切り分けられない
    黒線      ラリーのエリアには格子と平行な線が無く、走行ラインの曲線を拾うことがある
    **QR**    **コントラスト 78%。桁違い**

2026-09-22 の実機80枚で、grid_heading が 9枚（11%）、こちらが 40枚（50%）。
**grid_heading が誤った3枚（+8.93 / +10.59 / +12.80°）は、こちらは3枚とも
「測れない」を返した**（黒線の退避路が無いため、答えを作りようがない）。

【走行を止めないこと】
`heading_task` と同じ約束。カメラが無い・校正が無い・測れない、のいずれでも
例外を投げず `ok=False` を返す。
"""

from __future__ import annotations

import logging
import math
import time
from typing import Any, Callable, Dict, Optional

try:
    from grid_heading import _to_deviation
except ImportError:  # パッケージとして読み込まれた場合
    from .grid_heading import _to_deviation

from floor_code_heading import FloorCodeHeading, GroundPlaneAdapter, Options
from ground_plane import GroundPlane

logger = logging.getLogger(__name__)

#: コードの上辺が、コース座標系でどちらを向いているか[deg]。
#:
#: **暫定値。** 2026-09-22 の実機6枚で、grid_heading の答えと突き合わせて出した
#: （差の平均 -91.24°、ばらつき 0.45°）。使ったのは D の暫定の行列
#: （`2026/ground_homography.json`、走行中の1枚から作ったもの）。
#:
#: **取り直しが要る（2026-09-22）。** カメラの幾何が解き直された
#: （高さ 220→205mm、俯角 20.0→26.6°、画角 49.0→56.1°）。grid_heading 側も
#: 新しい幾何で動くので、両者の関係が変わる。**実機6枚で測り直すこと。**
#: 45度刻みへ畳むので、**90度ちがっても結果は同じ**（-91.24 と -1.24 は同じ）。
#: **2026-09-23 16時台に -91.24 → -89.0 へ取り直した。** 上の「取り直しが要る」に対応。
#: 作り直したホモグラフィのうえで、目印で測れた45枚のうち両方測れた37枚と突き合わせ、
#: **差の中央値が 0 になる値**を探した（軸を X 増やすと差が X 増える、素直な関係）。
#:
#:   軸 -91.24（それまで）  差 -2.23°
#:   軸 -89.24             差 -0.23°
#:   **軸 -89.0             差 ≒ 0**      ← 採った
#:   軸 -88.24             差 +0.77°
#:
#: **散らばりは 1.12° で、軸をどこにしても変わらない**（一定のずれだけが動く）。
#: その 1.12° は、ホモグラフィを作り直して 3.26° から下がった値
CODE_AXIS_COURSE_DEG = -89.0


class FloorCodeReader:
    """
    映像を 1枚もらって、床の二次元コードでグリッドに対する向きのずれを測る。

    :param frame_source: BGR の画像（numpy）を返す関数。撮れなければ None
    :param plane: 画像 → 床 の変換。省くと `GroundPlane.load()` から作る
    :param options: 測り方の細かい設定（`floor_code_heading.Options`）
    :param code_axis_course_deg: コードの上辺がコース座標系で向いている角度
    """

    def __init__(self, frame_source: Callable[[], Any],
                 plane: Optional[Any] = None,
                 options: Optional[Options] = None,
                 code_axis_course_deg: float = CODE_AXIS_COURSE_DEG,
                 clock: Callable[[], float] = time.monotonic):
        self.frame_source = frame_source
        self._clock = clock
        self.options = options or Options()
        self.options.code_axis_course_deg = code_axis_course_deg
        self._plane = plane if plane is not None else self._load_plane()
        self._reader = FloorCodeHeading(self._plane, self.options, clock=clock)
        self.last: Optional[dict] = None
        #: 最後に測った1枚。デバッグ画像として残すために覚えておく（heading_task.py）
        self.last_frame = None

    @staticmethod
    def _load_plane() -> Any:
        """
        校正ファイルから床平面を作る。

        **`shared()` ではなく `load()` を使う。** 床のコードを的にして校正すると、
        走行体の置き方が入るまで `frame` が 'grid' のままで `shared()` は None を返す
        （格子座標で位置を打ち直すと全部ずれるための歯止め）。
        **向きだけなら 'grid' のままでよい**（一定のずれが乗るだけで、
        `code_axis_course_deg` で吸収できる）。
        """
        try:
            ground = GroundPlane.load()
        except Exception:  # noqa: BLE001 — 校正が無くても走行は続ける
            logger.warning("床平面の校正を読めませんでした", exc_info=True)
            ground = None
        if ground is None:
            logger.warning("床平面の校正がありません。床のコードでは向きを測れません"
                           "（Docs/ground_homography.md）")
        return GroundPlaneAdapter(ground)

    def read(self) -> dict:
        """
        1枚撮って測る。

        :returns: `heading_reader.HeadingReader.read()` と同じ形の辞書。
                  `deviation_deg` は「走行体の向き − いちばん近いグリッドの向き
                  （45度刻み）」で、**grid_heading と同じ意味**
        """
        frame = self.frame_source()
        self.last_frame = frame
        started = self._clock()
        found = self._reader.measure(frame)

        result: Dict[str, Any] = {
            "ok": False, "deviation_deg": None, "reason": found.reason,
            "source": "floorcode", "markers": found.finders,
            "elapsed_ms": round((self._clock() - started) * 1000, 1),
            "width": found.width, "height": found.height,
            "codes": len(found.codes), "frame": found.frame,
            "calibrated": found.calibrated,
        }
        if not found.ok:
            self.last = result
            return result

        code = found.codes[0]
        system, deviation = _to_deviation(code.heading_deg)
        result.update({
            "ok": True, "deviation_deg": round(deviation, 4), "system": system,
            "heading_deg": round(code.heading_deg, 4),
            "sigma_deg": code.sigma_deg, "module_px": code.module_px,
            "right_angle_deg": code.right_angle_deg,
            "x_mm": round(code.x_mm, 1), "y_mm": round(code.y_mm, 1),
        })
        if len(found.codes) >= 2:
            # **同じ絵の2枚は同じ向きのはず。** 食い違いはそのまま誤差
            ordered = sorted(found.codes, key=lambda c: c.module_px, reverse=True)
            gap = (ordered[-1].heading_deg - ordered[0].heading_deg + 180.0) % 360.0 - 180.0
            result["code_gap_deg"] = round(gap, 3)
        self.last = result
        return result
