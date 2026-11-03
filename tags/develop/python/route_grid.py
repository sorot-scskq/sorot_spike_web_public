"""
PC が経路に書いてきた mm から、**格子がどちらを向いているか**を出す（2026・ETラリー）。

位置の打ち直し（`2026/grid_position.py`）は「格子が自己位置の座標系のどこに、どの向きで
あるか」を知っている必要がある。これまで経路は相対の指示だけだったので、
**1回目の観測から向きも位置も学習していた**。PC が 2026-09-23 から各行の始点・終点を
mm で書いてくるようになったので、**向きだけはそこから正確に出せる**。

    "直進 250mm[G5.5-1.5→G5-1.5][mm:G1-1 1153,123→983,123]"
                                      ↑ この2点を結ぶ向きがコースの向き

【向きだけ。位置は観測から取る】
経路が言うのは「その行でどこからどこへ行くはずか」であって、**いま実際にどこに居るかでは
ない**。位置まで経路から取ると、観測ではなく計画で自己位置を上書きすることになる
（`Docs/position_correction_design.md` 8章が採らないとしているもの）。
**向きは行全体の向きなので、行のどこに居るかに関係なく決まる。** そこだけ使う。

【座標系の変換】
PC は 原点 G1-1・X+ が東・**Y+ が南**。走行体の自己位置は 走り出しが原点・**y+ が左**で、
向きは `x += d·cos(h)`, `y -= d·sin(h)`（`Device/HWMonitor.cpp`）。

**Y の符号が逆なので、PC の (dX, dY) の atan2 がそのまま走行体の向きの流儀になる。**
走行体の向きは「y が上向きの図で時計回りが正」で、PC の Y+ が南＝下向きだから。

    コースの向き[度] = atan2(dY_pc, dX_pc)

これと、そのとき走行体が思っている向きの差が、**自己位置の座標系から見た格子の向き**。

【行の対応が合っているかを必ず確かめる】
経路は C++ が番号を振り直して挿し込むので、**行の番号と経路の配列の添字が一致する保証が
無い**。取り違えたまま向きを決めると、格子がまるごと回った状態で誤対応が始まる。

**`SCD` と mm の距離を突き合わせて確かめる**（`route_place.SCD_TOLERANCE_MM` = 2mm）。
合わなければ「読めなかった」ことにする。**スタートの寄せを吸収した行も、この検算で自然に
落ちる**（量も吸収する行も経路によって変わる。`Common/route_place.py` の説明を参照）。
**どの行かを決め打ちしないこと。**

【R コースは列が鏡】
経路計算は L の格子で行い、R では旋回の左右だけを反転している。**mm も論理座標（L 基準）**
なので、R コースでは東西が鏡になる。`Common/route_place.py` の説明を参照。
向きは「差の atan2」なので、**鏡になると符号が変わる**。`MIRROR_COLUMNS` で反転する。
"""

import logging
import math
import threading
from typing import Any, List, Optional

try:
    from route_place import SCD_TOLERANCE_MM, PlaceTag
except ImportError:  # パッケージとして読み込まれた場合
    from ..Common.route_place import SCD_TOLERANCE_MM, PlaceTag

logger = logging.getLogger(__name__)

#: 経路から格子の向きを取るか。**やめるときはここを False にする**（再ビルド不要）
ENABLE = True

#: R コースで列を反転するか。**PC の mm は L 基準の論理座標**
#:
#: `Common/route_place.py` の「R コースは列が鏡」を参照。走行体は R でも経路を反転しない
#: （反転しているのは PC 側の旋回の左右だけ）。**Lコースなら False のまま。**
MIRROR_COLUMNS = False

#: 向きを出すのに要る行の長さ[mm]。短い行は向きが定まらない
#:
#: 端点の mm は整数に丸められているので、短い行ほど角度の誤差が大きい。
#: 100mm なら丸めの 1mm で 0.6度。それ以下は使わない
MIN_LENGTH_MM = 100.0

#: 何行ぶんの向きが揃えば信じるか
#:
#: 1行だけだと、行の取り違えに気づけない。**別々の行から同じ向きが出ることを確かめる**
MIN_ROWS = 2

#: 揃っているとみなす幅[度]。経路の行はすべて 45度刻みなので、実際には 1度も開かない
AGREE_DEG = 2.0


def wrap_deg(deg: float) -> float:
    """角度を (-180, 180] へ"""
    return (deg + 180.0) % 360.0 - 180.0


def fold90(deg: float) -> float:
    """格子は 90度回しても同じなので (-45, 45] へ畳む"""
    return (deg + 45.0) % 90.0 - 45.0


class RouteRowGeometry:
    """経路の1行ぶん。mm で始点と終点が分かっている行だけを持つ"""

    def __init__(self, index: int, tag: PlaceTag, scd: Optional[float]):
        self.index = index
        self.tag = tag
        self.scd = scd

    @property
    def usable(self) -> bool:
        """向きを出すのに使えるか

        **`SCD` と mm の距離が合うことを確かめる。** 行の対応が取り違えていれば、ここで落ちる。
        """
        if not self.tag.has_mm:
            return False
        length = self.tag.length_mm
        if length is None or length < MIN_LENGTH_MM:
            return False
        if self.scd is None or self.scd == 0:
            return False
        return abs(abs(self.scd) - length) <= SCD_TOLERANCE_MM

    @property
    def course_heading_deg(self) -> float:
        """その行が向いているコースの向き[度]（走行体の流儀。時計回りが正）

        **PC の Y+ が南**なので、(dX, dY) の atan2 がそのまま走行体の流儀になる。
        後退の行は進む向きと逆を向いているが、**格子の向きは 90度で畳む**ので影響しない。
        """
        dx = float(self.tag.to_mm[0] - self.tag.frm_mm[0])
        dy = float(self.tag.to_mm[1] - self.tag.frm_mm[1])
        if MIRROR_COLUMNS:
            dx = -dx
        return math.degrees(math.atan2(dy, dx))


class RouteGrid:
    """PC の経路から出した「自己位置の座標系から見た格子の向き」。走行体で1つを共有する"""

    _instance: Optional["RouteGrid"] = None
    _instance_lock = threading.Lock()

    @classmethod
    def get_instance(cls) -> "RouteGrid":
        with cls._instance_lock:
            if cls._instance is None:
                cls._instance = cls()
            return cls._instance

    @classmethod
    def reset_instance(cls) -> None:
        """共有している1つを捨てる（テスト用）"""
        with cls._instance_lock:
            cls._instance = None

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._rows: List[RouteRowGeometry] = []
        #: 出せた向き[度]。まだ出せていなければ None
        self.axis_deg: Optional[float] = None
        #: 何行から出したか
        self.rows_used = 0

    # --- 経路を受け取る -------------------------------------------------------

    def set_route(self, scenario: Any) -> int:
        """登録された経路を覚える。**走りは変えない。**

        :returns: mm が読めて向きに使える行の数
        """
        rows: List[RouteRowGeometry] = []
        for i, row in enumerate(scenario or []):
            if not isinstance(row, dict):
                continue
            tag = PlaceTag.parse(row.get("Comment"))
            if tag is None:
                continue
            scd = (row.get("SwitchInfo") or {}).get("SCD")
            geom = RouteRowGeometry(i, tag, float(scd) if isinstance(scd, (int, float)) else None)
            if geom.usable:
                rows.append(geom)

        with self._lock:
            self._rows = rows
            self.axis_deg = None
            self.rows_used = 0
        return len(rows)

    # --- 向きを出す -----------------------------------------------------------

    def learn(self, cno: int, heading_deg: float) -> Optional[float]:
        """いま走っている行と、そのとき思っている向きから、格子の向きを詰める。

        **別々の行から同じ向きが出たときだけ信じる**（行の取り違えに気づくため）。

        :param cno: いまのコマンドセット番号（経路の行は 1 から並ぶ）
        :param heading_deg: いま x/y を積むのに使っている向き[度]
        :returns: 出せた格子の向き[度]。まだなら None
        """
        if not ENABLE:
            return None
        with self._lock:
            rows = list(self._rows)
        geom = next((g for g in rows if g.index == cno - 1), None)
        if geom is None:
            return None

        seen = fold90(wrap_deg(heading_deg - geom.course_heading_deg))
        with self._lock:
            if self.axis_deg is None:
                self._pending = getattr(self, "_pending", [])
                self._pending.append((cno, seen))
                agree = [s for _, s in self._pending
                         if abs(fold90(s - self._pending[0][1])) <= AGREE_DEG]
                if len(agree) >= MIN_ROWS and len({c for c, _ in self._pending}) >= MIN_ROWS:
                    self.axis_deg = sum(agree) / len(agree)
                    self.rows_used = len(agree)
                    logger.info("経路から格子の向きを決めました: %+.2f度（%d行から）",
                                self.axis_deg, self.rows_used)
            return self.axis_deg

    @property
    def known(self) -> bool:
        with self._lock:
            return self.axis_deg is not None

    def summary(self) -> str:
        with self._lock:
            if self.axis_deg is None:
                return "経路から格子の向きは出せていない（mm 付きの行 %d）" % len(self._rows)
            return "経路から見た格子の向き %+.2f度（%d行）" % (self.axis_deg, self.rows_used)
