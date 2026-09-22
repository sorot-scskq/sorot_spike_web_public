"""
経路（相対の指示の列）から「各行の到達目標姿勢」を作る。

PC が返す経路は**相対の指示だけ**で（「320mm 進む」「45度回る」）、
どこに居るべきかはどこにも書かれていない（`Docs/position_correction_design.md` 3章）。
そのため自己位置をいくら正しくしても、走り方を直す材料が無い。

ここは入り口の姿勢に指示を積み、**節点（行を走り終えた時点の姿勢）の列**を作る。

    nodes[0]       入り口（経路の1行目を始める姿勢）
    nodes[i]       i行目を走り終えた姿勢 ＝ i+1行目を始める姿勢
    nodes[len]     終点

走行中の補正（`route_correct.py` / `route_follower.py`）と、走行後の突き合わせ
（`Tools/compare_route.py`）が同じここを使う。**2通り書くと必ず食い違う。**

【座標系】C++ の自己位置（`Device/HWMonitor.cpp` の `calcCoordinates`）と同じ。

    x += d * cos(heading)
    y -= d * sin(heading)        ← **y は引く。** 時計回りが正の方位で、y は左が正
    heading += SCR               SCR は時計回りが正（PC の「左45度回転」は SCR -45）

`Tools/plot_trajectory.py` の再計算、`RoughSpot/Cpp/PoseReport.cpp` が置く
`head_deg` もこれ。**符号を変えるときは3か所まとめて。**
"""

import math

#: 走行方法（C++ の RunMethod.h と同じ値）
FUNCNO_REVOLUTION = 3
FUNCNO_LINEAR = 5
FUNCNO_HEADING_LINEAR = 19

#: 切替条件のビット（SwitchMethod/SwitchiType.h）
SW_DIR = 32
SW_DISTANCE = 128


def wrap_deg(deg):
    """角度を ±180 に畳む"""
    while deg > 180.0:
        deg -= 360.0
    while deg < -180.0:
        deg += 360.0
    return deg


class Pose:
    """ある時点の姿勢。**作り直して返す**（積むときに元を壊さない）"""

    __slots__ = ("x", "y", "heading_deg")

    def __init__(self, x=0.0, y=0.0, heading_deg=0.0):
        self.x = float(x)
        self.y = float(y)
        self.heading_deg = float(heading_deg)

    def moved(self, mm):
        """向いたまま進む[mm]。負なら後退（**y は引く**。モジュールの説明を参照）"""
        rad = math.radians(self.heading_deg)
        return Pose(self.x + mm * math.cos(rad), self.y - mm * math.sin(rad), self.heading_deg)

    def turned(self, deg):
        """その場で回る[度]。時計回りが正"""
        return Pose(self.x, self.y, wrap_deg(self.heading_deg + deg))

    def distance_to(self, other):
        return math.hypot(other.x - self.x, other.y - self.y)

    def heading_to(self, other, backward=False):
        """そこへ向かう方位[度]。`backward` なら後退で向かう向き（真後ろ）"""
        dx, dy = other.x - self.x, other.y - self.y
        if backward:
            dx, dy = -dx, -dy
        return math.degrees(math.atan2(-dy, dx))

    def gap_to(self, other):
        """自分の向きから見た開き `(前方, 左)`[mm]

        前方＝進みすぎ/足りない、左＝横のずれ。**直すところが違う**ので分ける
        （前方は距離 `SCD`、横は旋回 `SCR` でしか直せない）。
        """
        rad = math.radians(self.heading_deg)
        dx, dy = other.x - self.x, other.y - self.y
        return (dx * math.cos(rad) - dy * math.sin(rad),
                dx * math.sin(rad) + dy * math.cos(rad))

    def __repr__(self):
        return "Pose(%.1f, %.1f, %.1f度)" % (self.x, self.y, self.heading_deg)


class RouteRow:
    """経路の1行。走らせ方と、そこで狙った量を見る

    **狙った量は書き換えられる**（走行中の補正が `SCD`/`SCR` を計算し直す）。
    書き換えても `original_*` は残す。安全弁（元の値からどれだけ離れたか）は
    いつも元の値と比べること。積み重ねで少しずつ流れていくのを防ぐため。
    """

    def __init__(self, raw):
        run = raw.get("RunInfo") or {}
        switch = raw.get("SwitchInfo") or {}
        self.raw = raw
        self.comment = raw.get("Comment", "")
        self.funcno = int(run.get("FUNCNO") or 0)
        self.scjfn = int(switch.get("SCJFN") or 0)
        self.distance_mm = float(switch.get("SCD") or 0.0)
        self.turn_deg = float(switch.get("SCR") or 0.0)
        self.original_distance_mm = self.distance_mm
        self.original_turn_deg = self.turn_deg

    @property
    def is_turn(self):
        return self.funcno == FUNCNO_REVOLUTION and (self.scjfn & SW_DIR) != 0

    @property
    def is_straight(self):
        """直進と後退。**PC の経路には後退が2割ほど入る**（SCD が負、FWD も負）"""
        return (self.funcno in (FUNCNO_LINEAR, FUNCNO_HEADING_LINEAR)
                and (self.scjfn & SW_DISTANCE) != 0 and self.distance_mm != 0)

    @property
    def kind(self):
        if self.is_turn:
            return "旋回"
        if self.is_straight:
            return "後退" if self.distance_mm < 0 else "直進"
        return "その他"

    @property
    def keeps_heading(self):
        """この行を走っても車体の向きが変わらないか

        **打ち直す向きは「測った瞬間の向き」なので、効くまでに向きが変われば
        嘘になる。** 直進（FUNCNO 5 / 19）で `TRN` が 0 の行だけを「変わらない」
        とみなし、旋回・カーブ・ライントレースは変わる側へ倒す。
        """
        run = self.raw.get("RunInfo") or {}
        return (self.funcno in (FUNCNO_LINEAR, FUNCNO_HEADING_LINEAR)
                and not run.get("TRN", 0))

    @property
    def planned(self):
        """この行で狙った量（旋回なら度、直進なら mm）"""
        return self.turn_deg if self.is_turn else self.distance_mm

    @property
    def original_planned(self):
        return self.original_turn_deg if self.is_turn else self.original_distance_mm

    def advance(self, pose):
        """この行を計画どおり走ったあとの姿勢。走らせ方が読めない行は動かない"""
        if self.is_turn:
            return pose.turned(self.turn_deg)
        if self.is_straight:
            return pose.moved(self.distance_mm)
        return pose

    def set_distance(self, mm):
        self.distance_mm = float(mm)

    def set_turn(self, deg):
        self.turn_deg = float(deg)

    def emit(self, sno, cno, prv_sno, prv_cno):
        """走行中の差し替えに置く1行を作る（番号ごと丸ごと上書きされる）

        **C++ は同じ SNO/CNO の行を丸ごと置き換える**（`Scenario::insertCommandSet`）。
        欠けたキーは 0 になるので、元の行を全部写してから変えたところだけ直す。

        旋回の向きは `TRN` の符号で決まる（`RunMethod/RevolutionRunning.cpp`）。
        `SCR` の符号を変えたのに `TRN` を置いていくと、**逆へ回って止まらない。**

        `SCR` / `SCD` は `int16_t` なので整数で置く（C++ 側は小数を切り捨てる）。
        """
        row = dict(self.raw)
        run = dict(row.get("RunInfo") or {})
        switch = dict(row.get("SwitchInfo") or {})

        if self.is_turn:
            switch["SCR"] = int(round(self.turn_deg))
            trn = int(run.get("TRN") or 0)
            if trn != 0:
                run["TRN"] = abs(trn) if self.turn_deg >= 0 else -abs(trn)
        else:
            switch["SCD"] = int(round(self.distance_mm))

        row["RunInfo"] = run
        row["SwitchInfo"] = switch
        row["SNO"] = sno
        row["CNO"] = cno
        row["PRVSNO"] = prv_sno
        row["PRVCNO"] = prv_cno
        return row


def is_terminator(raw):
    """終了コマンド（SNO 0 かつ CNO 0。この行は実行されない）か、走らせ方が無い行か

    実物の終了コマンドは `FUNCNO 1` を持っている（番号だけで見分ける）。
    番号を書かない経路もあるので、**両方の番号がある行だけ**見る。
    """
    if "SNO" in raw and "CNO" in raw:
        if int(raw.get("SNO") or 0) == 0 and int(raw.get("CNO") or 0) == 0:
            return True
    return int((raw.get("RunInfo") or {}).get("FUNCNO") or 0) == 0


class PlannedRoute:
    """経路と、そこから作った節点の列

    **C++ の `RoutePlan`（RouteInserter.h）とは別物。** あちらは「どこへ・どの番号で
    挿し込むか」で、こちらは「どこに居るべきか」。
    """

    def __init__(self, rows):
        self.rows = list(rows)

    @classmethod
    def from_rows(cls, raw_rows):
        """PC の応答の `scenario`（コマンドセットの配列）から作る"""
        return cls([RouteRow(r) for r in (raw_rows or []) if not is_terminator(r)])

    @classmethod
    def from_message(cls, data):
        """PC の応答（`scenario` キー）でも、行の配列でも読む"""
        rows = data.get("scenario") if isinstance(data, dict) else data
        return cls.from_rows(rows)

    def __len__(self):
        return len(self.rows)

    def __iter__(self):
        """行をそのまま並べる（`Tools/overlay_course.py` が行と区間を突き合わせる）"""
        return iter(self.rows)

    def __getitem__(self, index):
        return self.rows[index]

    def nodes(self, entry):
        """入り口の姿勢から積んだ節点の列。長さは行数 + 1"""
        out = [entry]
        pose = entry
        for row in self.rows:
            pose = row.advance(pose)
            out.append(pose)
        return out

    def advance_through(self, pose, start, stop):
        """`start` 行から `stop` 行の手前までを計画どおり走ったあとの姿勢"""
        for row in self.rows[start:stop]:
            pose = row.advance(pose)
        return pose
