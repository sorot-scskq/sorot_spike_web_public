"""
PC が経路の `Comment` の末尾に書いてくる「場所」を読む。

PC-System_2026 が 2026-09-23 に足した。**新しいキーは増えていない。**
`Comment` の末尾に `[始点→終点]`、ゲートを通る行だけ `[始点→終点 色]` が付く。
**2026-09-23 に mm が2つ目の組として足された**（`[mm:G1-1 始点X,始点Y→終点X,終点Y]`）。

    "直進 250mm[G5.5-1.5→G5-1.5][mm:G1-1 1153,123→983,123]"
    "直進 614mm（ゲートを確認）[11→G1-3 RED][mm:G1-1 307,430→307,798]"
    "斜め直進 348mm[13→24]"                       ← mm を切っているとこの形

**mm の組があるときは、それが末尾になる。** 場所の組を末尾で探すと mm の組に当たって
**静かに間違う**（`frm="mm:G1-1 307,430"` と読めてしまう）ので、**先に mm を外してから
場所を読む**こと。

**読むだけ。走りは変えない。** PC 側は走行方法も切替条件も変えていないので、
この行を読まなくても走行は今までどおり進む（読まなければ素通りする）。
用途はまだ決まっていないため、**自己位置の打ち直しや切替の判断には使わない**
（使うならユーザーに確認してから。`Docs/handoff_20260922.md`）。

【座標は mm ではない。コースの格子】

    11〜44      スマートエリアのマスの中心。1桁目が列 X、2桁目が行 Y
    G<列>-<行>  支柱が立つ 5x5 のゲートポジション。半マス刻みなので G1.5-3 もある

PC 側の連続座標は「原点がゲートポジション G1-1、X+ が東、**Y+ が南**」。

    ゲートポジション G<列>-<行>  = (列-1, 行-1)
    マスの中心 <列><行>          = (列-0.5, 行-0.5)

**走行体の自己位置系とは別物。** あちらは走り出しが原点の mm で、y は左が正
（`Device/HWMonitor.cpp`）。**Y の符号が逆で、単位も違う。**
1マス 245.75mm だが、東西のコース線だけ実測に合わせた特別扱い（東 170mm /
西 150mm）があるので、**単純に 245.75 を掛けても mm にならない。**
**換算はこちらでやらない。** 2026-09-23 に PC 側が mm を併記するようにしたので、
そちらを読む（変換の唯一の実装は PC 側の `CreateRouteV2.cell_center()`）。

【mm の組】

    [mm:G1-1 始点X,始点Y→終点X,終点Y]

先頭の `mm:G1-1` が**座標系の名前**。原点が変われば PC 側がこの印を変えるので、
**知らない印なら読まずに落とす**（値域の検査と同じ考え）。値は整数 mm で、
**マイナスが出る**（青丸ゴールが `-150,123`）。2桁や `G` 記法には出なかった符号。

**1行目だけ `SCD` と mm の距離が合わない。** 走行体は論理上のスタート点より
`START_OFFSET_MM = 80mm` 外側（東）に置かれるが、mm は論理座標をそのまま出している。
**2行目からは 2mm 以内で一致する**ので、経路の検算に使える
（斜めの行で丸めの差が 1.52mm 出るため、1mm では狭すぎる）。

【R コースは列が鏡】

経路計算は L の格子で行い、R では旋回の左右だけを反転している。つまり Comment の
`12` や `G2-3` は**論理座標（L 基準）**で、**R コースの床に貼ってある位置補助コードとは
列が鏡**になる。床のコードと突き合わせるなら、R のとき列を反転すること。
"""

import re
from typing import List, Optional


#: `Comment` の末尾の `[始点→終点]` / `[始点→終点 色]`
#:
#: 矢印は全角の U+2192、括弧は半角。**必ず末尾**に付く（終了コマンドには付かない）。
#: 始点は空白を含みうるが、終点と色は空白で区切られる
PLACE_PATTERN = re.compile(
    r"\[(?P<frm>[^\[\]→]+)→(?P<to>[^\[\]→ ]+)(?: (?P<gates>[A-Z,]+))?\]$")

#: マスの中心の2桁
#:
#: **`11`〜`44` の 16通りちょうど**（スマートエリアの 4x4）。2桁になるのは列も行も
#: 整数で 1〜4 のときだけで、それ以外は `G` 記法になる（PC 側で総当たり確認済み）。
#: スマートエリアの外のマスの中心（スタート `G5.5-1.5`、青丸 `G0.5-1.5`）は、
#: 床にコードが貼られていないので 2桁にしない。
#: **広く取らないこと。** `55` のような2桁が来たら書式が変わった合図
CELL_PATTERN = re.compile(r"^(?P<col>[1-4])(?P<row>[1-4])$")

#: ゲートポジション（`G5.5-1.5`）。半マス刻みがある
#:
#: **負の値は出ない**（走行可能範囲の下限がちょうど 0 で止まる）。`G0-0` や
#: `G0-2.5` は実在する（西・北の外周レーン）ので、**列 0 や行 0 を異常として
#: 弾かないこと**
GATE_PATTERN = re.compile(r"^G(?P<col>\d+(?:\.\d+)?)-(?P<row>\d+(?:\.\d+)?)$")

#: mm の組（`[mm:G1-1 307,430→307,798]`）。**あるときは必ず Comment の末尾**
#:
#: 先頭の `mm:<印>` が座標系の名前。値は整数で、**マイナスが出る**
MM_PATTERN = re.compile(
    r"\[mm:(?P<frame>[^\s\[\]]+) "
    r"(?P<fx>-?\d+),(?P<fy>-?\d+)\u2192(?P<tx>-?\d+),(?P<ty>-?\d+)\]$")

#: こちらが意味を知っている座標系。**これ以外の印なら mm を読まない**
#:
#: 原点はゲートポジション G1-1、X+ が東、**Y+ が南**。
#: 走行体の自己位置（走り出しが原点、Y+ が北）とは原点も Y の符号も違う
MM_FRAME = "G1-1"

#: mm の距離と `SCD` の食い違いに許す幅[mm]
#:
#: **端点2つと `SCD` をそれぞれ別に丸めている**ので、丸めの差が積もる。
#: 半マス 122.875mm が 123mm に寄るぶんが、**斜めでは両軸に乗る**。
#:
#:     22 → 33（1マス斜め）  mm 369,369 → 614,614 の距離 346.48
#:                           SCD 348（245.75×√2 = 347.54 を丸めた）
#:                           差 1.52mm
#:
#: 真横・真縦なら 0mm に収まる。**「斜めなら出る」ではなく「2↔3 をまたぐ斜めなら出る」**
#: （出どころは PC 側の分析）。マス中心の mm は 123 / 369 / 614 / 860 で、**隣どうしの
#: 差が一定ではない**（1→2 が 246、2→3 が 245、3→4 が 246）。245.75 の半端がマスに
#: よって上下に寄るため、**両軸とも 2↔3 をまたぐ斜めだけが 245,245 になる**。
#:
#:     22↔33 と 23↔32 の各向き（4通りだけ）  1.52mm
#:     片方の軸だけ 2↔3                      0.81mm
#:     1↔2 どうし・3↔4 どうし                0.1mm
#:
#: **配置によって出たり出なかったりする。** 実機の6配置で試して、斜めが出たのは
#: 1配置だけだった（`G3-1,G3-2` / `G1-3,G2-3` / `G4-5,G5-5` で `32→23` が出る）。
#: **「手元の配置で通ったから 1mm でよい」と判断しないこと。**
#: **2026-09-23 に 1.0 から 2.0 へ直した**（PC 側も同じ穴を踏んでいて、
#: 向こうの検算も 2mm に直された）。**1mm にしないこと。**
SCD_TOLERANCE_MM = 2.0

#: ゲートポジションの値域（PC 側が走行可能範囲を総当たりして出した。2026-09-23）
#:
#: **外れたら書式か経路の作りが変わった合図**なので、読めなかったことにする。
#: 走りには使っていないので、弾いても実害は「ログに出ない」だけ
GATE_COL_MAX = 5.5
GATE_ROW_MAX = 6.0
#: 半マス刻み。これ以外の端数が来たら書式が変わっている
GATE_STEP = 0.5


class PlacePoint:
    """場所ひとつ。`11`（マスの中心）か `G2.5-0.5`（ゲートポジション）"""

    #: マスの中心
    CELL = "cell"
    #: 支柱が立つゲートポジション
    GATE = "gate"

    def __init__(self, text: str, kind: str, col: float, row: float):
        self.text = text
        self.kind = kind
        #: PC 側の連続座標[マス]。原点は G1-1、X+ が東、**Y+ が南**
        self.col = col
        self.row = row

    @classmethod
    def parse(cls, text: str) -> Optional["PlacePoint"]:
        """読めなければ None を返す（**読めなくても走行は続く**）"""
        text = (text or "").strip()
        m = CELL_PATTERN.match(text)
        if m:
            # マスの中心は支柱の格子に乗らないので半マスずれる
            return cls(text, cls.CELL, int(m["col"]) - 0.5, int(m["row"]) - 0.5)
        m = GATE_PATTERN.match(text)
        if m:
            col, row = float(m["col"]), float(m["row"])
            if not (0.0 <= col <= GATE_COL_MAX and 0.0 <= row <= GATE_ROW_MAX):
                return None
            if col % GATE_STEP or row % GATE_STEP:
                return None
            return cls(text, cls.GATE, col - 1.0, row - 1.0)
        return None

    def __eq__(self, other) -> bool:
        return (isinstance(other, PlacePoint) and self.kind == other.kind
                and self.col == other.col and self.row == other.row)

    def __repr__(self) -> str:
        return "PlacePoint(%s %s %.1f,%.1f)" % (self.text, self.kind, self.col, self.row)


class PlaceTag:
    """`Comment` の末尾から読んだ、その行の始点・終点と通るゲートの色"""

    def __init__(self, frm_text: str, to_text: str, gates: List[str],
                 frm_mm=None, to_mm=None):
        self.frm_text = frm_text
        self.to_text = to_text
        #: 通るゲートの色（`RED` / `BLUE` / `YELLOW`）。1行で2本通ることがある
        self.gates = gates
        #: 読めた場合だけ入る。読めなければ None（書式が変わっても落ちないため）
        self.frm = PlacePoint.parse(frm_text)
        self.to = PlacePoint.parse(to_text)
        #: PC が併記した mm（原点 G1-1、X+ 東、**Y+ 南**）。無ければ None
        self.frm_mm = frm_mm
        self.to_mm = to_mm

    @classmethod
    def parse(cls, comment) -> Optional["PlaceTag"]:
        """`Comment` から場所を読む。付いていなければ None"""
        if not isinstance(comment, str):
            return None
        text = comment.strip()

        # **先に mm を外す。** mm があるとそちらが末尾になるので、場所を末尾で
        # 探すと mm の組に当たって静かに間違う（frm="mm:G1-1 307,430" と読める）
        frm_mm = to_mm = None
        mm = MM_PATTERN.search(text)
        if mm:
            text = text[:mm.start()].rstrip()
            if mm["frame"] == MM_FRAME:
                frm_mm = (int(mm["fx"]), int(mm["fy"]))
                to_mm = (int(mm["tx"]), int(mm["ty"]))
            # 知らない座標系なら、外すだけで中身は使わない（原点が変わった合図）

        m = PLACE_PATTERN.search(text)
        if not m:
            return None
        gates = [g for g in (m["gates"] or "").split(",") if g]
        return cls(m["frm"].strip(), m["to"].strip(), gates, frm_mm, to_mm)

    @property
    def known(self) -> bool:
        """始点も終点も座標として読めたか"""
        return self.frm is not None and self.to is not None

    @property
    def has_mm(self) -> bool:
        """こちらが意味を知っている座標系の mm が付いているか"""
        return self.frm_mm is not None and self.to_mm is not None

    @property
    def length_mm(self) -> Optional[float]:
        """区間の長さ[mm]。**`SCD` と突き合わせる検算に使える**

        **1行目だけ合わない。** 走行体は論理上のスタート点より 80mm 外側に置かれるが、
        mm は論理座標をそのまま出しているため（`START_OFFSET_MM`）。2行目からは 2mm 以内
        """
        if not self.has_mm:
            return None
        dx = self.to_mm[0] - self.frm_mm[0]
        dy = self.to_mm[1] - self.frm_mm[1]
        return (dx * dx + dy * dy) ** 0.5

    def __repr__(self) -> str:
        tail = (" " + ",".join(self.gates)) if self.gates else ""
        return "PlaceTag(%s→%s%s)" % (self.frm_text, self.to_text, tail)


class RoutePlaces:
    """経路1本ぶんの場所。**記録と突き合わせのためだけに持つ**"""

    def __init__(self, tags, scd=None):
        #: 行と同じ並び。付いていない行は None
        self.tags = tags
        #: 行と同じ並びの `SCD`（検算用）。無ければ None
        self.scd = scd or [None] * len(tags)

    @classmethod
    def read(cls, scenario) -> "RoutePlaces":
        """経路の各行の `Comment` から場所を読む"""
        rows = list(scenario or [])
        return cls([PlaceTag.parse((r or {}).get("Comment")) for r in rows],
                   [((r or {}).get("SwitchInfo") or {}).get("SCD") for r in rows])

    def scd_mismatches(self, tolerance_mm: float = SCD_TOLERANCE_MM) -> List[int]:
        """mm の距離と `SCD` が食い違う行（0 始まり）

        **PC が計算した2つを突き合わせる。** 場所と距離は別々に作られているので、
        片方だけ壊れたらここで出る。走りには使っていないので、出ても走行は続く。

        **1行目は除く。** 走行体は論理上のスタート点より 80mm 外側に置かれるが、
        mm は論理座標をそのまま出しているため、必ず 80mm ずれる（PC 側の仕様）。

        許容は `SCD_TOLERANCE_MM`（2mm）。**斜めの行で丸めの差が 1.52mm 出る**ので
        1mm にしてはいけない。
        """
        out = []
        for i, (t, scd) in enumerate(zip(self.tags, self.scd)):
            if i == 0 or t is None or not t.has_mm or scd is None:
                continue
            if abs(t.length_mm - abs(scd)) > tolerance_mm:
                out.append(i)
        return out

    @property
    def count(self) -> int:
        """場所が付いていた行の数"""
        return sum(1 for t in self.tags if t is not None)

    @property
    def gate_rows(self) -> List[int]:
        """ゲートを通る行の番号（0 始まり）"""
        return [i for i, t in enumerate(self.tags) if t is not None and t.gates]

    def summary(self) -> str:
        """ログ1行ぶん。**付いていなければ、そう言う**（PC 側が未対応か古い）"""
        if self.count == 0:
            return "場所の情報は付いていない（PC 側が未対応か、書き足しを切っている）"
        first = next(t for t in self.tags if t is not None)
        last = next(t for t in reversed(self.tags) if t is not None)
        gates = [g for t in self.tags if t is not None for g in t.gates]
        unreadable = sum(1 for t in self.tags if t is not None and not t.known)
        text = "%d行に場所が付いていた（%s → %s）" % (
            self.count, first.frm_text, last.to_text)
        if gates:
            # **色ごとの数にまとめる。** 全部並べるとログが1行で溢れる
            # （本番の経路は3周ぶんあるので同じ色が何度も出る）
            order = []
            for g in gates:
                if g not in order:
                    order.append(g)
            text += "、ゲート %d本（%s）" % (
                len(gates), " ".join("%s%d" % (g, gates.count(g)) for g in order))
        if unreadable:
            text += "、**座標として読めない行が %d 件**" % unreadable

        with_mm = sum(1 for t in self.tags if t is not None and t.has_mm)
        if with_mm:
            text += "、mm %d行" % with_mm
            bad = self.scd_mismatches()
            if bad:
                # **PC が計算した2つが食い違っている。** 片方だけ壊れた合図
                text += "（**SCD と合わない行が %d 件: %s**）" % (
                    len(bad), " ".join(str(i) for i in bad[:5]))
        return text
