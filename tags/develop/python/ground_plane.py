"""
画像の座標[px]から「走行体から見た床[mm]」を出す（平面ホモグラフィ）。

床の位置補助コードを見て自己位置を打ち直すための土台
（`Docs/position_correction_design.md` 5.4、手順は `Docs/ground_homography.md`）。

【なぜカメラの内部パラメータを使わないか】
見たいものが**床という1枚の平面の上にしか無い**ため。平面と平面の対応は
3x3 の行列ひとつ（ホモグラフィ）で書け、焦点距離も歪みも取り付け角も
その中に溶けている。**床にメジャーで測った4点があれば求まる**ので、
チェッカーボードを撮り集める内部パラメータ校正が丸ごと要らない。

一度求めれば固定。**カメラを動かしたら測り直す**（台の緩みも含む）。

【座標系】走行体から見た床。C++ の自己位置と同じ向きにそろえてある。

    x = 前方[mm]   y = 左[mm]   方位 = 時計回りが正

`Device/HWMonitor.cpp` の `calcCoordinates` が `x += d*cos, y -= d*sin` で積み、
`Tools/plot_trajectory.py` も同じ。**ここだけ左右を逆にすると、補正が
毎回ずれる向きに効いて誤差が育つ。** 符号を変えるときは3か所まとめて。

【地平線より上は床ではない】
画像の上のほうは床の無限遠より先（壁や人）を写している。そこを変換すると
**遠方の巨大な値や、符号が逆の値**が出る。この実装は分母 w<=0 を None で返す。
呼ぶ側は None を「床ではない」として捨てること。

【実行時に要るのは numpy だけ】
求めるのに cv2.findHomography を使う（`calibrate`）が、**使うときは行列を
掛けるだけ**なので numpy があれば動く。走行中の経路に cv2 を増やさない。
"""

from __future__ import annotations

import json
import logging
import os
from typing import Iterable, List, Optional, Sequence, Tuple

try:
    from opencv import OpenCv
except ImportError:                      # パッケージとして読み込まれた場合
    from .opencv import OpenCv

logger = logging.getLogger(__name__)

#: 床座標がこの値より手前（分母がこれ以下）なら地平線の向こうとみなす
_EPS = 1e-9

Pair = Tuple[Tuple[float, float], Tuple[float, float]]


class GroundPlane:
    """画像[px] ←→ 床[mm] を行き来する。行列は校正で決まり、以後は固定"""

    #: 校正ファイルの既定の置き場（年のフォルダ）。config_2026.py が指す
    DEFAULT_FILENAME = "ground_homography.json"

    _shared: Optional["GroundPlane"] = None
    _shared_loaded = False

    def __init__(self, matrix, image_size, meta: Optional[dict] = None):
        """
        :param matrix: 3x3。画像[px] → 床[mm]
        :param image_size: 校正したときの (幅, 高さ)[px]
        :param meta: 校正の素性（日付・残差・取り付けのメモ）。使わないが残す
        """
        numpy = OpenCv.numpy()
        if numpy is None:
            raise RuntimeError("numpy が無いのでホモグラフィを扱えません")
        self._np = numpy
        self.matrix = numpy.asarray(matrix, dtype=float).reshape(3, 3)
        self.image_size = (int(image_size[0]), int(image_size[1]))
        self.meta = dict(meta or {})
        self._inverse = None

    # --- 求める ---------------------------------------------------------------

    @classmethod
    def calibrate(cls, pairs: Sequence[Pair], image_size,
                  meta: Optional[dict] = None) -> Optional["GroundPlane"]:
        """
        対応点から行列を求める。4点以上。求まらなければ None。

        **cv2.findHomography はここでしか使わない。** 最小二乗なので5点以上
        渡せばそのぶん均される。外れ値を弾く RANSAC は使わない
        （点が4〜8個しかなく、弾かれた点に気づけないほうが危ない。
        外れ値は `residuals_mm` の数字で人が見て落とす）。

        :param pairs: ((u, v), (x_mm, y_mm)) の並び
        """
        cv2 = OpenCv.cv2()
        numpy = OpenCv.numpy()
        if cv2 is None or numpy is None:
            logger.warning("OpenCV が無いので校正できません")
            return None
        if len(pairs) < 4:
            logger.warning("対応点が %d 個しかありません（4点以上）", len(pairs))
            return None

        src = numpy.array([p[0] for p in pairs], dtype=float)
        dst = numpy.array([p[1] for p in pairs], dtype=float)
        matrix, _ = cv2.findHomography(src, dst, 0)
        if matrix is None:
            logger.warning("ホモグラフィが求まりませんでした（4点が一直線に並んでいないか）")
            return None

        # **符号をそろえる。** 行列は定数倍の自由度があり、全体が -1 倍で
        # 出てくることがある。そのままだと床の点まで分母が負になり、
        # 地平線の判定（w<=0 は床ではない）が丸ごと裏返る。校正点は必ず床の
        # 上にあるので、そこで分母が正になるように向きを決める。
        w = matrix[2, 0] * src[:, 0] + matrix[2, 1] * src[:, 1] + matrix[2, 2]
        if float(numpy.mean(w)) < 0:
            matrix = -matrix

        return cls(matrix, image_size, meta)

    # --- 読み書き -------------------------------------------------------------

    @classmethod
    def default_path(cls) -> str:
        """既定の校正ファイル。config_2026.py の GROUND_HOMOGRAPHY_FILE を優先する"""
        try:
            import config
        except ImportError:                 # パッケージとして読み込まれた場合
            from . import config            # type: ignore
        season = config.load_season()
        path = getattr(season, "GROUND_HOMOGRAPHY_FILE", None)
        if path:
            return path
        return os.path.join(config.season_dir(), cls.DEFAULT_FILENAME)

    @classmethod
    def load(cls, path: Optional[str] = None) -> Optional["GroundPlane"]:
        """
        校正ファイルを読む。**無ければ None を返す（例外にしない）。**

        走行中に呼ばれるため。校正がまだでも走行は続けたい（補正だけ止まる）。
        """
        path = path or cls.default_path()
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
        except FileNotFoundError:
            logger.warning("床のホモグラフィが未校正です（%s）。"
                           "Tools/calibrate_ground_homography.py で作ります", path)
            return None
        except (OSError, ValueError) as e:
            logger.warning("床のホモグラフィを読めません（%s）: %s", path, e)
            return None

        matrix = data.get("matrix")
        size = data.get("image_size")
        if not matrix or not size:
            logger.warning("床のホモグラフィの中身が足りません（matrix / image_size）: %s", path)
            return None
        meta = {k: v for k, v in data.items() if k not in ("matrix", "image_size")}
        try:
            return cls(matrix, size, meta)
        except (RuntimeError, ValueError) as e:
            logger.warning("床のホモグラフィを組み立てられません: %s", e)
            return None

    @property
    def frame(self) -> str:
        """
        行き先の座標系。`"robot"`（走行体から見た床）か `"grid"`（コースの格子）。

        **床のコードを的にして校正すると、まず格子座標で求まる**
        （`Tools/calibrate_ground_homography.py --from-codes`）。走行体が
        格子のどこに置いてあったかを渡すまでは `"grid"` のままで、位置の
        打ち直しには使えない。書いていない古いファイルは `"robot"` とみなす。
        """
        return str(self.meta.get("frame", "robot"))

    @classmethod
    def shared(cls, path: Optional[str] = None) -> Optional["GroundPlane"]:
        """
        一度だけ読んで使い回す。固定の行列なので読み直す理由が無い。

        **走行体の座標系でなければ None を返す。** 格子座標の行列をそのまま
        自己位置の補正に使うと、原点も向きも違うまま打ち直すことになる。
        道具から使うときは `load()`（座標系を問わない）のほう。
        """
        if not cls._shared_loaded:
            plane = cls.load(path)
            if plane is not None and plane.frame != "robot":
                logger.warning(
                    "床のホモグラフィが %s 座標のままです（走行体の置き方が未指定）。"
                    "補正には使えません。--origin を渡して校正し直してください",
                    plane.frame)
                plane = None
            cls._shared = plane
            cls._shared_loaded = True
        return cls._shared

    @classmethod
    def reset_shared(cls) -> None:
        """読み直させる（テストと、校正し直した直後用）"""
        cls._shared = None
        cls._shared_loaded = False

    def save(self, path: str, extra: Optional[dict] = None) -> None:
        """校正ファイルを書く。**人が読んで直せる形にしておく**（JSON、桁は丸めない）"""
        data = dict(self.meta)
        data.update(extra or {})
        data["image_size"] = list(self.image_size)
        data["matrix"] = [[float(v) for v in row] for row in self.matrix]
        tmp = path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
            f.write("\n")
        os.replace(tmp, path)

    # --- 使う -----------------------------------------------------------------

    def to_floor(self, points_px: Iterable[Sequence[float]],
                 image_size=None) -> List[Optional[Tuple[float, float]]]:
        """
        画像[px] → 床[mm]。**地平線より先の点は None**（床ではないので捨てる）。

        :param image_size: いまの映像の大きさ。校正時と違えば合わせて縮める
        """
        np = self._np
        pts = np.asarray(list(points_px), dtype=float).reshape(-1, 2)
        pts = self._rescale(pts, image_size)
        ones = np.ones((len(pts), 1))
        hom = np.hstack([pts, ones]) @ self.matrix.T
        out: List[Optional[Tuple[float, float]]] = []
        for x, y, w in hom:
            out.append(None if w <= _EPS else (float(x / w), float(y / w)))
        return out

    def to_image(self, points_mm: Iterable[Sequence[float]],
                 image_size=None) -> List[Optional[Tuple[float, float]]]:
        """床[mm] → 画像[px]。カメラの後ろ側になる点は None"""
        np = self._np
        if self._inverse is None:
            self._inverse = np.linalg.inv(self.matrix)
        pts = np.asarray(list(points_mm), dtype=float).reshape(-1, 2)
        ones = np.ones((len(pts), 1))
        hom = np.hstack([pts, ones]) @ self._inverse.T
        out: List[Optional[Tuple[float, float]]] = []
        scale = self._scale(image_size)
        for u, v, w in hom:
            if abs(w) <= _EPS:
                out.append(None)
            else:
                out.append((float(u / w) * scale[0], float(v / w) * scale[1]))
        return out

    def mm_per_pixel(self, point_px: Sequence[float]) -> Optional[float]:
        """
        その画素を 1px 取り違えると床で何mm ずれるか（いちばん悪い向きで）。

        **遠いほど大きくなる。** どこまで使えるかはこの数字で決まるので、
        検算（`Tools/check_ground_homography.py`）と同じものをここに置く。
        """
        u, v = float(point_px[0]), float(point_px[1])
        base = self.to_floor([(u, v)])[0]
        if base is None:
            return None
        worst = 0.0
        for du, dv in ((0.5, 0.0), (0.0, 0.5)):
            a = self.to_floor([(u - du, v - dv)])[0]
            b = self.to_floor([(u + du, v + dv)])[0]
            if a is None or b is None:
                return None
            worst = max(worst, ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5)
        return worst

    def horizon_row(self, u: Optional[float] = None) -> float:
        """
        床が無限遠になる画像の行 v。**これより上は床ではない。**

        分母 w = h20*u + h21*v + h22 = 0 を v について解く。
        h21 が 0 のとき（カメラが傾いていない＝床が写らない）は inf。
        """
        h20, h21, h22 = self.matrix[2]
        if abs(h21) <= _EPS:
            return float("inf")
        u = self.image_size[0] / 2.0 if u is None else float(u)
        return float(-(h20 * u + h22) / h21)

    def residuals_mm(self, pairs: Sequence[Pair]) -> List[Optional[float]]:
        """対応点それぞれの、変換結果と実測の開き[mm]"""
        got = self.to_floor([p[0] for p in pairs])
        out: List[Optional[float]] = []
        for (_, mm), g in zip(pairs, got):
            if g is None:
                out.append(None)
            else:
                out.append(((g[0] - mm[0]) ** 2 + (g[1] - mm[1]) ** 2) ** 0.5)
        return out

    # --- 解像度合わせ ---------------------------------------------------------

    def _scale(self, image_size) -> Tuple[float, float]:
        """いまの映像 → 校正時 の倍率"""
        if image_size is None:
            return (1.0, 1.0)
        return (self.image_size[0] / float(image_size[0]),
                self.image_size[1] / float(image_size[1]))

    def _rescale(self, pts, image_size):
        """
        校正時と違う解像度で撮った点を、校正時の画素へ直す。

        **行列は校正した解像度に紐づいている。** 1280x720 で校正して
        640x360 の映像を渡すと、直さなければ床の座標がおよそ倍ずれる。
        """
        if image_size is None:
            return pts
        sx, sy = self._scale(image_size)
        if sx == 1.0 and sy == 1.0:
            return pts
        return pts * self._np.array([sx, sy])

    def __repr__(self) -> str:
        return "GroundPlane(%s, image_size=%dx%d, horizon_v=%.1f)" % (
            self.frame, self.image_size[0], self.image_size[1], self.horizon_row())
