"""
床の二次元コードから測った向きを、打ち直せる形に整える（2026・ETラリー）。

測るのは別のモジュール（床のコードを見つけて向きを出す側）で、ここがするのは

    1. 1枚の映像から「見えたコードごとの観測」を受け取る
    2. **誤対応の防波堤**（`Docs/position_correction_design.md` 6章）
       - 2枚以上見えたら、250mm 格子に乗っているかを確かめ、合わなければ**両方捨てる**
       - コードどうしの向きが食い違えば捨てる
    3. 残った観測から、格子の向きに対するずれ（deviation）を出す

の3つ。**自己位置を触らない**（打ち直しは `Common/coord_correct.py`）。

【観測の座標系】（セッション D と取り決めた）
    x = 前方[mm]、y = 左[mm]、方位は時計回りが正。C++ の自己位置と同じ
    heading_deg = 走行体の向き − コードの向き

床のコードは**全部同じ向きに貼ってある**（2026-09-20 に確認）ので、コードの向き＝
コース座標系の向きになる。どのコードかが分からなくても向きは決まる（設計 5.3）。

【ずれの出し方】
経路は格子の上を 45度刻みで進むので、**本来の向きは必ず 45度の倍数**。
測った向きといちばん近い 45度の倍数との差が、そのまま向きのずれになる。
自己位置の向き（走行体が思っている向き）にこのずれを足したものが、本当の向き。
"""

import logging
import math
from typing import Any, Dict, List, Optional, Sequence

logger = logging.getLogger(__name__)

#: コースの格子の間隔[mm]（公式コース図から実測。`Docs/et_rally_route.md` 6.2）
GRID_PITCH_MM = 250.0

#: 格子に乗っているとみなす許容[mm]。**格子の 1/4 より小さくすること**
#: （半マスずれた組を通すと、誤対応を確かめる意味が無くなる）
GRID_TOLERANCE_MM = 60.0

#: コードどうしの向きが食い違ってよい範囲[度]
HEADING_AGREE_DEG = 3.0

#: 本来の向きの刻み[度]。経路は 45度刻みで格子を渡る
HEADING_STEP_DEG = 45.0

#: これより遠くに写ったコードは使わない[mm]
#:
#: **向きの精度は距離とともに悪くなる**（床は奥へ潰れて写るので、画像の1画素が
#: 奥行きでは数mm になる）。ただし**近ければよいわけでもない。**
#: いまの取り付け（高さ 220mm・俯角 20.0°・画角 49.0°/27.6°）では、
#: **見える床の帯は 329〜2025mm** で、それより近くは画角に入らない。
#:
#:     450mm  148x65px  σ 0.88°
#:     600mm  114x40px  σ 1.45°
#:     800mm   87x24px  σ 2.43°
#:    1000mm   71x16px  σ 3.65°   ← MAX_SIGMA_DEG で落ちる
#:
#: **実際に落とすのは σ（MAX_SIGMA_DEG）のほう。** ここは「遠すぎて論外」を
#: 切るだけの広い上限にしてある。取り付けを変えたら測り直すこと
MAX_CODE_RANGE_MM = 900.0

#: 測る側が出した精度の目安が、これより悪いコードは使わない[度]
#:
#: 合成画像では 0.2〜0.6° が出ている（測る側の `sigma_deg`）。大きく外れるのは
#: 腕が短く写ったとき（斜めから見た・遠い）なので、そこで落とす。
#: 出してこない測り方（sigma が無い）のときは通す
MAX_SIGMA_DEG = 3.0


class FloorCodeObservation:
    """床のコード1枚ぶんの観測"""

    def __init__(self, heading_deg: float, x_mm: float, y_mm: float,
                 sigma_deg: Optional[float] = None, code_id: str = ""):
        #: 走行体の向き − コードの向き[度]（時計回りが正）
        self.heading_deg = float(heading_deg)
        #: コードの位置（走行体から見て前[mm]）
        self.x_mm = float(x_mm)
        #: 同（左[mm]）
        self.y_mm = float(y_mm)
        #: 測りのばらつきの見積り[度]。分からなければ None
        self.sigma_deg = None if sigma_deg is None else float(sigma_deg)
        #: 読めた中身（`A1`〜`D4`）。読めていなければ空。**照合に使う**
        self.code_id = str(code_id or "")

    @classmethod
    def from_any(cls, value: Any) -> Optional["FloorCodeObservation"]:
        """辞書でもオブジェクトでも受け取る（測る側の形に合わせやすくするため）"""
        def pick(*names):
            for name in names:
                if isinstance(value, dict):
                    if name in value:
                        return value[name]
                elif hasattr(value, name):
                    return getattr(value, name)
            return None

        heading = pick("heading_deg", "heading")
        x = pick("x_mm", "x", "d_mm")
        y = pick("y_mm", "y", "l_mm")
        if heading is None or x is None or y is None:
            return None
        try:
            return cls(float(heading), float(x), float(y), pick("sigma_deg", "sigma"),
                       pick("code_id", "text") or "")
        except (TypeError, ValueError):
            return None

    def __repr__(self) -> str:
        return "FloorCode(%.2f°, 前%.0f 左%.0f)" % (self.heading_deg, self.x_mm, self.y_mm)


class HeadingMeasurement:
    """決まった向きのずれ（採れなかった理由も持つ）"""

    def __init__(self, ok: bool, deviation_deg: Optional[float] = None,
                 codes: int = 0, spread_deg: Optional[float] = None,
                 reason: str = "", frames: int = 1):
        self.ok = ok
        #: いちばん近い 45度の倍数からのずれ[度]。自己位置の向きに足すと本当の向き
        self.deviation_deg = deviation_deg
        #: 1枚の中で使えたコードの枚数
        self.codes = codes
        #: ずれの開き[度]。1枚の中ならコードどうし、重ねたときは映像どうし
        self.spread_deg = spread_deg
        #: 重ねた映像の枚数
        self.frames = frames
        #: 採れなかった理由
        self.reason = reason

    def note(self) -> str:
        """ログに出す一言"""
        if not self.ok:
            return self.reason
        note = "ずれ %+.2f°・映像%d枚・コード%d枚" % (
            self.deviation_deg, self.frames, self.codes)
        if self.spread_deg is not None:
            note += "・開き %.2f°" % self.spread_deg
        return note


def fold_to_step(deg: float, step: float = HEADING_STEP_DEG) -> float:
    """いちばん近い `step` の倍数からのずれ[度]（±step/2 に畳む）"""
    return float(deg) - round(float(deg) / step) * step


def distance_mm(a: FloorCodeObservation, b: FloorCodeObservation) -> float:
    """2枚のコードの間隔[mm]（走行体から見たまま）。

    **向きで回さない。** 回すなら2枚に同じ角度を掛けなければならないが、
    読み取りが返す四隅の回り方は1枚ごとに変わる（実機で C1 だけ 180° 回って
    返ってきた）。**間隔は回しても変わらない**ので、そもそも回さずに測る。
    """
    return math.hypot(b.x_mm - a.x_mm, b.y_mm - a.y_mm)


def lattice_distances_mm(pitch: float = GRID_PITCH_MM, limit: int = 4) -> list:
    """格子の上の2点が取りうる間隔[mm]（近いほうから）。

    縦横だけでなく斜めもある（250・354・500・559…）。**どちらの文字がどの軸かを
    決めなくてよい**ので、貼り方の向きに依らない。
    """
    out = {pitch * math.hypot(i, j)
           for i in range(limit + 1) for j in range(limit + 1) if i or j}
    return sorted(out)


def off_grid_mm(a: FloorCodeObservation, b: FloorCodeObservation,
                pitch: float = GRID_PITCH_MM) -> float:
    """2枚のコードの間隔が、格子で取りうる間隔からどれだけ外れているか[mm]"""
    seen = distance_mm(a, b)
    return min(abs(seen - d) for d in lattice_distances_mm(pitch))


class FloorCodeGate:
    """
    見えたコードから、格子の向きに対するずれを決める。

    **状態を持たない。** 1枚ごとに完結する。
    """

    def __init__(self, pitch_mm: float = GRID_PITCH_MM,
                 tolerance_mm: float = GRID_TOLERANCE_MM,
                 agree_deg: float = HEADING_AGREE_DEG,
                 max_range_mm: float = MAX_CODE_RANGE_MM,
                 max_sigma_deg: float = MAX_SIGMA_DEG):
        self.pitch_mm = pitch_mm
        self.tolerance_mm = tolerance_mm
        self.agree_deg = agree_deg
        self.max_range_mm = max_range_mm
        self.max_sigma_deg = max_sigma_deg

    def _off_expected_mm(self, a: FloorCodeObservation,
                         b: FloorCodeObservation) -> Optional[float]:
        """
        2個のコードの並びが、あるべき配置からどれだけ外れているか[mm]。

        **中身（ID）が読めていれば、そちらで照らす。** どのコードかが分かるので
        「格子で取りうる間隔のどれか」ではなく**ただ1つの間隔**と比べられる。
        同じ中身が2個見えていたら取り違えなので None（呼ぶ側が全部捨てる）。

        読めていなければ、格子で取りうる間隔（250・354・500…）と比べる。

        **どちらも「間隔」だけを見る。** 向きで回してから比べると、読み取りが
        返す四隅の回り方の違い（実機で 180° 回った例がある）がそのまま位置の
        食い違いになる。間隔は回しても変わらない。
        """
        if a.code_id and b.code_id:
            if a.code_id == b.code_id:
                return None
            expected = id_distance_mm(a.code_id, b.code_id, self.pitch_mm)
            if expected is not None:
                return abs(distance_mm(a, b) - expected)
        return off_grid_mm(a, b, self.pitch_mm)

    def decide(self, observations: Sequence[Any]) -> HeadingMeasurement:
        """
        観測のリストから向きのずれを決める。

        :param observations: `FloorCodeObservation`、またはそれに直せる辞書
        """
        seen = [o for o in (FloorCodeObservation.from_any(v) for v in observations)
                if o is not None]
        if not seen:
            return HeadingMeasurement(False, reason="床のコードが見つからない")

        codes = [o for o in seen if math.hypot(o.x_mm, o.y_mm) <= self.max_range_mm]
        if not codes:
            near = min(math.hypot(o.x_mm, o.y_mm) for o in seen)
            return HeadingMeasurement(
                False, reason="いちばん近いコードでも %.0fmm 先（上限 %.0fmm）"
                              % (near, self.max_range_mm))

        codes = [o for o in codes
                 if o.sigma_deg is None or o.sigma_deg <= self.max_sigma_deg]
        if not codes:
            return HeadingMeasurement(
                False, reason="どのコードも精度の目安が %.1f° より悪い" % self.max_sigma_deg)

        if len(codes) == 1:
            dev = fold_to_step(codes[0].heading_deg)
            return HeadingMeasurement(True, dev, 1)

        # 向きが食い違うコードがあれば、どれが正しいか決められない。全部捨てる
        devs = [fold_to_step(o.heading_deg) for o in codes]
        spread = max(devs) - min(devs)
        if spread > self.agree_deg:
            return HeadingMeasurement(
                False, reason="コード%d枚の向きが %.2f° 食い違う（上限 %.1f°）"
                              % (len(codes), spread, self.agree_deg))

        # **250mm 格子に乗っていなければ、コードを取り違えている。** 全部捨てる
        worst = 0.0
        for i in range(len(codes)):
            for j in range(i + 1, len(codes)):
                off = self._off_expected_mm(codes[i], codes[j])
                if off is None:
                    return HeadingMeasurement(
                        False, reason="同じ中身（%s）のコードが2個見えている"
                                      % codes[i].code_id)
                worst = max(worst, off)
                if off > self.tolerance_mm:
                    return HeadingMeasurement(
                        False, reason="コードの並びが %.0fmm 格子に合わない"
                                      "（外れ %.0fmm・上限 %.0fmm）"
                                      % (self.pitch_mm, off, self.tolerance_mm))

        dev = sum(devs) / len(devs)
        logger.debug("床のコード%d枚: ずれ %+.2f°・開き %.2f°・格子の外れ %.0fmm",
                     len(codes), dev, spread, worst)
        return HeadingMeasurement(True, dev, len(codes), spread)


def id_distance_mm(a: str, b: str, pitch: float = GRID_PITCH_MM) -> Optional[float]:
    """
    中身（`A1`〜`D4`）から決まる、2個のコードの間隔[mm]。分からなければ None。

    **どちらの文字がどの軸かを決めなくてよい。** 距離だけを見るので、
    貼り方の向き（行と列の取り方）に依らない。同じ中身なら 0 を返す。
    """
    if len(a) != 2 or len(b) != 2:
        return None
    if not (a[0].isalpha() and b[0].isalpha() and a[1].isdigit() and b[1].isdigit()):
        return None
    d_letter = ord(a[0].upper()) - ord(b[0].upper())
    d_number = int(a[1]) - int(b[1])
    return pitch * math.hypot(d_letter, d_number)


def measurement_to_heading(pose_heading_deg: float,
                           measurement: HeadingMeasurement) -> Optional[float]:
    """
    測ったずれと自己位置の向きから、打ち直す向き[度]を作る。

    走行体は「計画どおりの向きで走っているつもり」なので、自己位置の向きは
    本来の向き（45度の倍数）を指している。実際にはそこから `deviation_deg` だけ
    ずれているので、**足したものが本当の向き**になる。
    """
    if not measurement.ok or measurement.deviation_deg is None:
        return None
    return float(pose_heading_deg) + measurement.deviation_deg
