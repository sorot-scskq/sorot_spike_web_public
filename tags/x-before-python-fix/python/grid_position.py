"""
床の二次元コードの見え方から、自己位置の「位置」の打ち直し量を出す（2026・ETラリー）。

`grid_heading.py` が 1枚の画像から出した「格子の中のどこに居るか」（`offset_mm`）を、
自己位置といちばん近いコードに当てて、位置のずれ（dx, dy）にする。
C++ はこれを `ACAF 0x07` で打ち直す（`Docs/position_correction_design.md` 5.5）。

【向き → 位置 の順。逆にしてはいけない】
向きがずれていると、格子の軸を走行体の向きから起こす段で位置が大きく外れる
（見えている目印は最大 1.1m 先なので、2度で 38mm）。そこでここでは
**走行体の向きを使わず、当てはめた格子の軸そのものを使う**。向きが先に合っている
ほど「いちばん近いコード」の取り違えも減るので、測った向きのずれが
`MAX_DEVIATION_DEG` より大きい回は位置を出さない。

【誤対応が最大のリスク】
復号していないので「どのコードを見ているか」は分からない。自己位置がいちばん近い
コードを見ていると仮定して当てるしかなく、**自己位置が半マス以上ずれていれば
隣のコードに当ててしまう**。当てた結果のずれが格子の 1/3（83mm）を超えたら採らない
（`MAX_OFFSET_MM`）。超えるということは、どのコードか決めきれていないということ。

【復号できれば、この当て推量が要らなくなる（まだ使っていない）】
コードの中身（`A1`〜`D4`）が読めれば、どのコードかが**決まる**ので最寄り判定が要らない
（`Docs/position_correction_design.md` 5.5 の (a)）。`Common/floor_code.py` が読めるが、
実機の1枚で読めたのは 4個中 2個（1920x1080・zxing-cpp）で、走行体の取り込みは
1280x720、zxing-cpp も未導入。**1枚で2個読めないと格子の向きが決まらない**ので、
ID で当てる作りにはまだできない。

いまは**読めた ID を記録するだけ**にしてある（`code_ids`）。実機で「1枚に何個読めるか」を
数えれば、(a) に移れるかが決まる。手順は `Docs/position_correction_field_check.md`。

【座標系】
C++ の自己位置と同じ（`Device/HWMonitor.cpp`）。前へ d 進むと
x += d·cos(heading)、y -= d·sin(heading)。heading は度で、右回りが正。

【格子の置き場（アンカー）】
打ち直すには「自己位置の座標系のどこに格子があるか」が要る。自己位置の原点は
走り出した場所なので、格子との関係は置き方で決まる。

    learn（既定）  最初のよく写った 1回で格子の位置と向きを決め、以後はそこからのずれを直す
    fixed          走り出しが格子点の上（`ANCHOR_X` / `ANCHOR_Y`）で、格子の軸と平行
                   （`ANCHOR_ANGLE_DEG`）だったものとして直す

**既定を learn にしてあるのは、fixed だと外したときに何も起きないため。**
走り出しが格子点から 83mm 以上ずれていると、fixed ではすべての回が上限に掛かり、
1回も打ち直さないまま「効かなかった」ように見える。learn は 1回目を基準にするので、
置き方によらず**それ以降の積もりを必ず取る**（置き方そのもののずれは取れない）。
"""

import logging
import math
import threading
from typing import Any, Dict, Optional

try:
    from grid_heading import GRID_STEP_MM
except ImportError:  # パッケージとして読み込まれた場合
    from .grid_heading import GRID_STEP_MM

from route_register import SwitchFile

logger = logging.getLogger(__name__)

#: 位置を打ち直すか（置き場が無いときの既定）。
#:
#: **2026-09-23 は False。** 実機では一度も動いていない。現地で A/B を取ってから
#: True にすること。**走行体では下の置き場で切り替える**（ソースを直に書き換えると
#: `git pull` で戻る。引き継ぎ §0 の失敗）。
ENABLE = False

#: `ENABLE` を走行体で切り替える置き場（1行目に 1 か 0）。git の管理外なので pull で戻らない
ENABLE_SWITCH = "./sorot_spike/position_fix.txt"

#: 当てた結果のずれの上限[mm]。**格子の 1/3**。超えたら誤対応とみなして採らない
#:
#: いちばん近いコードに当てる都合で、ずれは必ず格子の半分（125mm）以内に収まる。
#: 125mm 近くまで出ているということは「隣のコードかもしれない」ということなので、
#: そこまで待たずに切る（`Docs/position_correction_design.md` 6）。
MAX_OFFSET_MM = GRID_STEP_MM / 3.0

#: 向きの打ち直し量の上限[度]。超えたら測り間違いとみなして位置ごと採らない
MAX_HEADING_DEG = 10.0

#: 測った向きのずれがこれを超える回は、位置を出さない[度]
#:
#: 向きが合っていない間は、いちばん近いコードの取り違えが起きやすい。
#: 向きは `Utillity/HeadingHold.h` が 1回 3度ずつ直すので、数回で下回る。
MAX_DEVIATION_DEG = 6.0

#: 位置を出すのに要る目印の数。向き（3個）より多くする
#:
#: 目印が 3個だと格子の当てはめの自由度（φ・倍率・平行移動2つ）に対して余裕が無く、
#: 平行移動が目印の拾い間違いにそのまま引きずられる。
MIN_MARKERS = 4

#: 格子の置き場の決め方。"learn" か "fixed"
ANCHOR_MODE = "learn"

#: "learn" で格子の置き場を決めるのに要る目印の数。1回目を間違えると以後ずっとずれる
ANCHOR_MIN_MARKERS = 5

#: "fixed" のときの格子点の位置[mm]（走り出しが格子点の上なら 0, 0）
ANCHOR_X = 0.0
ANCHOR_Y = 0.0

#: 格子の軸が自己位置の x 軸から何度回っているか[度]
ANCHOR_ANGLE_DEG = 0.0


def wrap_deg(deg: float) -> float:
    """角度を (-180, 180] へ"""
    return (deg + 180.0) % 360.0 - 180.0


def fold(value: float, pitch: float) -> float:
    """格子の間隔で折り返した余り。(-pitch/2, pitch/2]"""
    return (value + pitch / 2.0) % pitch - pitch / 2.0


class GridAxes:
    """自己位置の座標系から見た格子の軸

    第1軸 e1 と第2軸 e2。走行体の (前, 右) と同じ組み方なので、
    向き angle_deg の走行体の前が e1、右が e2 になる。
    """

    def __init__(self, angle_deg: float):
        self.angle_deg = angle_deg
        rad = math.radians(angle_deg)
        self.e1 = (math.cos(rad), -math.sin(rad))
        self.e2 = (-math.sin(rad), -math.cos(rad))

    def to_axes(self, x: float, y: float):
        """自己位置の座標系のベクトルを、格子の軸に沿った成分へ"""
        return (x * self.e1[0] + y * self.e1[1], x * self.e2[0] + y * self.e2[1])

    def to_xy(self, a1: float, a2: float):
        """格子の軸に沿った成分を、自己位置の座標系のベクトルへ"""
        return (a1 * self.e1[0] + a2 * self.e2[0], a1 * self.e1[1] + a2 * self.e2[1])


class GridAnchor:
    """自己位置の座標系のどこに格子があるか（格子点 1つの位置と、軸の向き）"""

    def __init__(self, x: float = ANCHOR_X, y: float = ANCHOR_Y,
                 angle_deg: float = ANCHOR_ANGLE_DEG, pitch: float = GRID_STEP_MM,
                 known: bool = True):
        self.x = x
        self.y = y
        self.pitch = pitch
        self.known = known
        self.axes = GridAxes(angle_deg)

    @property
    def angle_deg(self) -> float:
        return self.axes.angle_deg

    def snap_axis_deg(self, axis_deg: float) -> float:
        """格子の軸の向きを、90度刻みでアンカーの向きに合わせ直す

        当てはめが返す軸は 4通りのどれか（格子は 90度回しても同じ）。
        アンカーの向きにいちばん近いものを選ぶ。
        """
        return self.angle_deg + 90.0 * round(wrap_deg(axis_deg - self.angle_deg) / 90.0)

    def residual(self, x: float, y: float):
        """その点がいちばん近い格子点からどれだけ外れているか（格子の軸に沿った成分）"""
        a1, a2 = self.axes.to_axes(x - self.x, y - self.y)
        return (fold(a1, self.pitch), fold(a2, self.pitch))

    def learn(self, x: float, y: float, angle_deg: Optional[float] = None) -> None:
        """その点を格子点だったことにし、格子の向きもそのときの見え方に合わせる"""
        self.x = x
        self.y = y
        if angle_deg is not None:
            self.axes = GridAxes(fold(angle_deg, 90.0))
        self.known = True


class PositionFix:
    """1回ぶんの打ち直し量。採らなかったときは ok が False で、reason に理由が入る"""

    def __init__(self, ok: bool, dx: float = 0.0, dy: float = 0.0, dheading_deg: float = 0.0,
                 reason: str = "", along: tuple = (0.0, 0.0), anchored: bool = False,
                 code_ids: Optional[list] = None):
        self.ok = ok
        self.dx = dx
        self.dy = dy
        self.dheading_deg = dheading_deg
        self.reason = reason
        self.along = along
        self.anchored = anchored
        #: その1枚で復号できた床のコードの中身（`A1`〜`D4`）。**当てるのには使っていない**
        self.code_ids = list(code_ids or [])

    @property
    def distance_mm(self) -> float:
        return math.hypot(self.dx, self.dy)

    def as_dict(self) -> Dict[str, Any]:
        return {"ok": self.ok, "dx": round(self.dx, 2), "dy": round(self.dy, 2),
                "dheading_deg": round(self.dheading_deg, 3), "reason": self.reason,
                "along": [round(self.along[0], 2), round(self.along[1], 2)],
                "anchored": self.anchored, "code_ids": list(self.code_ids)}


class GridPositionFixer:
    """格子の当てはめと自己位置から、位置の打ち直し量を出す"""

    _instance: Optional["GridPositionFixer"] = None
    _instance_lock = threading.Lock()

    @classmethod
    def get_instance(cls) -> "GridPositionFixer":
        with cls._instance_lock:
            if cls._instance is None:
                cls._instance = cls()
            return cls._instance

    @classmethod
    def reset_instance(cls) -> None:
        """共有している1つを捨てる（テスト用）"""
        with cls._instance_lock:
            cls._instance = None

    def __init__(self, anchor: Optional[GridAnchor] = None, mode: str = None):
        self.mode = ANCHOR_MODE if mode is None else mode
        if anchor is not None:
            self.anchor = anchor
        else:
            self.anchor = GridAnchor(known=(self.mode != "learn"))
        #: 直近の結果（確かめる用）
        self.last: Optional[PositionFix] = None

    def fix(self, result: Dict[str, Any], pose: Dict[str, Any],
            code_ids: Optional[list] = None) -> PositionFix:
        """
        :param result:   `grid_heading.estimate_heading()` を辞書にしたもの
        :param pose:     撮った時点の自己位置（`Common/pose.py`）。x, y, heading[度]
        :param code_ids: その1枚で復号できたコードの中身。**記録するだけで、当てるのには使わない**
        :returns: 打ち直し量。採らないときは ok が False
        """
        fix = self._judge(result or {}, pose or {})
        fix.code_ids = list(code_ids or [])
        self.last = fix
        return fix

    def _judge(self, result: Dict[str, Any], pose: Dict[str, Any]) -> PositionFix:
        if not SwitchFile.read(ENABLE_SWITCH, ENABLE):
            return PositionFix(False, reason="位置の打ち直しは止めてある")
        if not result.get("ok"):
            return PositionFix(False, reason="向きが測れていない")
        offset = result.get("offset_mm")
        axis_deg = result.get("axis_deg")
        if offset is None or axis_deg is None:
            # 黒線だけで測った回。格子のどこに居るかは分からない
            return PositionFix(False, reason="目印で測った回ではない（%s）" % result.get("source"))
        markers = result.get("markers") or 0
        if markers < MIN_MARKERS:
            return PositionFix(False, reason="目印が %d 個では位置を出さない" % markers)
        deviation = result.get("deviation_deg")
        if deviation is None or abs(deviation) > MAX_DEVIATION_DEG:
            # 向きが先。合っていないうちに位置を当てると、隣のコードに当たる
            return PositionFix(False, reason="向きのずれ %s度 がまだ大きい" % deviation)

        for key in ("x", "y", "heading"):
            if not isinstance(pose.get(key), (int, float)):
                return PositionFix(False, reason="自己位置が届いていない")
        x, y, heading = float(pose["x"]), float(pose["y"]), float(pose["heading"])

        # 格子の軸を、走行体の向きではなく当てはめから起こす（向き → 位置 の順）。
        # 当てはめの軸は 90度刻みでどれになるか決まらないので、アンカーに近いものを採る
        seen = heading + float(axis_deg)
        anchoring = not self.anchor.known
        if anchoring and markers < ANCHOR_MIN_MARKERS:
            return PositionFix(False, reason="格子を置くには目印が %d 個では足りない" % markers)
        axis = seen if anchoring else self.anchor.snap_axis_deg(seen)
        axes = GridAxes(axis)

        # 走行体が格子の中のどこに居るかから、見ている格子点の居場所を逆算する
        t1, t2 = float(offset[0]), float(offset[1])
        node_dx, node_dy = axes.to_xy(t1, t2)
        node_x, node_y = x - node_dx, y - node_dy

        anchored = False
        if anchoring:
            # 1回目。ここを基準にする。この回の打ち直し量は 0 になる
            self.anchor.learn(node_x, node_y, axis)
            anchored = True

        # いちばん近い格子点に当てる。外れの分だけ自己位置を動かす
        r1, r2 = self.anchor.residual(node_x, node_y)
        if abs(r1) > MAX_OFFSET_MM or abs(r2) > MAX_OFFSET_MM:
            return PositionFix(False, along=(-r1, -r2), anchored=anchored,
                               reason="格子から %.0f, %.0f mm 外れている（上限 %.0f）"
                                      % (r1, r2, MAX_OFFSET_MM))

        # 外れはアンカーの軸に沿って測ったので、戻すのもアンカーの軸で。
        # 当てはめの軸（axes）は 90度刻みで入れ替わりうるので、混ぜると x と y が入れ替わる
        dx, dy = self.anchor.axes.to_xy(-r1, -r2)
        dheading = wrap_deg(axis - float(axis_deg) - heading)
        if abs(dheading) > MAX_HEADING_DEG:
            return PositionFix(False, along=(-r1, -r2), anchored=anchored,
                               reason="向きの打ち直しが %.1f度 と大きい" % dheading)

        return PositionFix(True, dx=dx, dy=dy, dheading_deg=dheading,
                           along=(-r1, -r2), anchored=anchored)
