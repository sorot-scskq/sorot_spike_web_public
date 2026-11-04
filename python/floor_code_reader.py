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

from floor_code_heading import CameraFloorPlane, FloorCodeHeading, GroundPlaneAdapter, Options
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

#: 床に戻すのに、カメラの幾何（高さ・画角は確定、俯角は1枚ごとにコードの形から合わせる）を使うか。
#: False なら、これまでの床平面（`ground_homography.json`、'grid'）を使う。
#:
#: **これまでの床平面は、作った時の俯角で固まっている。** 柱に当たって俯角が動くと奥行きだけが
#: 伸び縮みし、走行体が格子に斜め（15〜30°）のとき向きが外れた（5°超えが 40%）。
#: 俯角を合わせ直すと 0%（1枚ごと）（2026-10-31、`floor_code_heading.TiltFitter`）
USE_CAMERA_GEOMETRY = True

#: カメラの幾何で読むときの、コードの上辺の向き[deg]。`CODE_AXIS_COURSE_DEG` のカメラの幾何版。
#: 床は走行体から見た座標（x 前・y 左）になるので、ここに入るのは**カメラの取り付けの左右の向き**だけ。
#:
#: **暫定値（2026-10-31）。** grid_heading（黒線・目印）の答えが名前に入った画像と突き合わせた：
#: 9月の47枚で -0.12°（ばらつき 中央 1.39°）、10月の2枚で -0.83°。どちらもばらつきの中なので 0 にした。
#: **撮影した画像で測り直すこと。** 45°刻みへ畳むので 45°ちがっても同じ答え
CAMERA_CODE_AXIS_COURSE_DEG = 0.0

#: 平均から何度より外れたコードを除くか。
#: 同じ絵のコードどうしは同じ向きのはず。3個のコードの線2本どうしの差は中央 2.4° なので、
#: その倍に近い 4° を超えたら、そのコードの読み違いとみなす
OUTLIER_DEG = 4.0


class HeadingAverage:
    """
    同じ1枚に写ったコードの向きを平均する。**向きは90°ごとに扱う**
    （ファインダ2個のコードは90°ごとにしか決まらず、使う側も格子からのずれしか見ないため）。

    2026-10-31、手元の画像で自己位置の回った量と比べた（コードが2つ以上写った78枚）。
    いちばん近い1個だけ 中央 1.46°・9割 4.12° → 平均 1.19°・3.98°・最大 8.8→6.1°。

    :param outlier_deg: 平均からこれより外れたコードを除いて、平均し直す
    """

    def __init__(self, outlier_deg: float = OUTLIER_DEG):
        self.outlier_deg = outlier_deg

    def average(self, headings_deg: list) -> Dict[str, Any]:
        """
        :returns: mean_deg（±45°に畳んだ平均）, used, dropped, spread_deg（使ったものの平均からの最大の外れ）, fallback。
                  **全部が外れたとき（2枚が 8° 以上食い違うなど）は平均を作らず、先頭（いちばん近いコード）を返し
                  fallback を True にする**
        """
        first = self._mean(headings_deg)
        kept = [h for h in headings_deg if abs(self._wrap90(h - first)) <= self.outlier_deg]
        if not kept:
            return {"mean_deg": self._wrap90(headings_deg[0]), "used": 1,
                    "dropped": len(headings_deg) - 1, "spread_deg": 0.0, "fallback": True}
        mean = self._mean(kept)
        spread = max(abs(self._wrap90(h - mean)) for h in kept)
        return {"mean_deg": mean, "used": len(kept), "dropped": len(headings_deg) - len(kept),
                "spread_deg": spread, "fallback": False}

    def _mean(self, headings_deg: list) -> float:
        """90°周期の平均（4倍角で平均して4で割る）"""
        sine = sum(math.sin(math.radians(4.0 * h)) for h in headings_deg)
        cosine = sum(math.cos(math.radians(4.0 * h)) for h in headings_deg)
        return self._wrap90(math.degrees(math.atan2(sine, cosine)) / 4.0)

    @staticmethod
    def _wrap90(degrees: float) -> float:
        return (degrees + 45.0) % 90.0 - 45.0


class FloorCodeReader:
    """
    映像を 1枚もらって、床の二次元コードでグリッドに対する向きのずれを測る。

    :param frame_source: BGR の画像（numpy）を返す関数。撮れなければ None
    :param plane: 画像 → 床 の変換。省くと `GroundPlane.load()` から作る
    :param options: 測り方の細かい設定（`floor_code_heading.Options`）
    :param code_axis_course_deg: コードの上辺がコース座標系で向いている角度。省くと床平面に合わせて
                                 `CAMERA_CODE_AXIS_COURSE_DEG` か `CODE_AXIS_COURSE_DEG`
    :param use_camera_geometry: `plane` を省いたとき、カメラの幾何で床に戻すか（`USE_CAMERA_GEOMETRY`）
    :param average_codes: 写ったコード（ファインダ2・3個）の向きを平均して `deviation_deg` にするか。
                          False なら、これまでどおりいちばん近いコードだけを使う
    """

    def __init__(self, frame_source: Callable[[], Any],
                 plane: Optional[Any] = None,
                 options: Optional[Options] = None,
                 code_axis_course_deg: Optional[float] = None,
                 clock: Callable[[], float] = time.monotonic,
                 average_codes: bool = True,
                 averager: Optional[HeadingAverage] = None,
                 use_camera_geometry: bool = USE_CAMERA_GEOMETRY):
        self.frame_source = frame_source
        self._clock = clock
        self.average_codes = average_codes
        self._averager = averager or HeadingAverage()
        self.options = options or Options()
        if plane is None:
            plane = self._camera_plane() if use_camera_geometry else self._load_plane()
        self._plane = plane
        if code_axis_course_deg is None:
            code_axis_course_deg = (CAMERA_CODE_AXIS_COURSE_DEG if isinstance(plane, CameraFloorPlane)
                                    else CODE_AXIS_COURSE_DEG)
        self.options.code_axis_course_deg = code_axis_course_deg
        self._reader = FloorCodeHeading(self._plane, self.options, clock=clock)
        self.last: Optional[dict] = None
        #: 最後に測った1枚。デバッグ画像として残すために覚えておく（heading_task.py）
        self.last_frame = None
        #: 最後に測った結果（`FloorHeadingResult`）。コードの位置を使う側（`qr_steer.py`）が読む
        self.last_found = None

    @staticmethod
    def _camera_plane() -> CameraFloorPlane:
        """
        走行体のカメラの定数（`heading_reader.ROBOT_CAMERA`。部屋の校正があればそれ）から作る。
        俯角は校正の値から始まり、1枚ごとにコードの形から合わせ直す
        """
        try:
            from heading_reader import ROBOT_CAMERA
        except ImportError:  # パッケージとして読み込まれた場合
            from .heading_reader import ROBOT_CAMERA
        plane = CameraFloorPlane.from_constants(ROBOT_CAMERA)
        logger.info("床のコードはカメラの幾何で読みます（高さ %.0fmm・俯角 %.2f° から合わせる）",
                    plane.eye_height_mm, plane.base_tilt_deg)
        return plane

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
                  （45度刻み）」で、**grid_heading と同じ意味**。
                  コードが2つ以上写ったときは平均から出す（`average_codes`）。
                  heading_deg / sigma_deg / x_mm などは、いちばん近いコードの値
        """
        frame = self.frame_source()
        self.last_frame = frame
        started = self._clock()
        found = self._reader.measure(frame)
        self.last_found = found

        result: Dict[str, Any] = {
            "ok": False, "deviation_deg": None, "reason": found.reason,
            "source": "floorcode", "markers": found.finders,
            "elapsed_ms": round((self._clock() - started) * 1000, 1),
            "width": found.width, "height": found.height,
            "codes": len(found.codes), "frame": found.frame,
            "calibrated": found.calibrated,
            "tilt_deg": found.tilt_deg, "tilt_source": found.tilt_source,
            "tilt_cost": found.tilt_cost,
        }
        if found.tilt_deg is not None:
            logger.info("床のコード: 俯角 %.2f°（%s）コード%d枚", found.tilt_deg,
                        found.tilt_source, len(found.codes))
        if not found.ok:
            self.last = result
            return result

        code = found.codes[0]
        _, nearest = _to_deviation(code.heading_deg)
        if self.average_codes and len(found.codes) >= 2:
            averaged = self._averager.average([c.heading_deg for c in found.codes])
            if averaged["fallback"]:
                logger.warning("床のコード%d枚の向きが %.1f° 以上食い違うため、平均せず近い1枚を使います",
                               len(found.codes), 2 * self._averager.outlier_deg)
            system, deviation = _to_deviation(averaged["mean_deg"])
            result.update({"codes_used": averaged["used"], "codes_dropped": averaged["dropped"],
                           "spread_deg": round(averaged["spread_deg"], 3),
                           "average_fallback": averaged["fallback"]})
        else:
            system, deviation = _to_deviation(code.heading_deg)
            result.update({"codes_used": 1, "codes_dropped": 0, "spread_deg": 0.0,
                           "average_fallback": False})
        result.update({
            "ok": True, "deviation_deg": round(deviation, 4), "system": system,
            "nearest_deviation_deg": round(nearest, 4),
            "heading_deg": round(code.heading_deg, 4),
            "sigma_deg": code.sigma_deg, "module_px": code.module_px,
            "right_angle_deg": code.right_angle_deg,
            "x_mm": round(code.x_mm, 1), "y_mm": round(code.y_mm, 1),
            "finders_used": code.finders,
        })
        if len(found.codes) >= 2:
            # **同じ絵の2枚は同じ向きのはず。** 食い違いはそのまま誤差。
            # ファインダ2個のコードは90°ごとにしか決まらないので、そのときは90°で畳む
            ordered = sorted(found.codes, key=lambda c: c.module_px, reverse=True)
            period = 360.0 if all(c.finders == 3 for c in found.codes) else 90.0
            gap = ((ordered[-1].heading_deg - ordered[0].heading_deg + period / 2) % period
                   - period / 2)
            result["code_gap_deg"] = round(gap, 3)
        self.last = result
        return result
