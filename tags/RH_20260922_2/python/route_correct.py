"""
計画との開きから、**まだ走っていない先の行**の `SCD` / `SCR` を計算し直す。

ここはハードウェアもファイルも触らない。母艦でそのまま試せる
（`RoughSpot/Python/tests/test_route_correct.py`）。置く側は `route_follower.py`。

【直し方：先の「旋回 → 直進 → 旋回」で節点へ向け直す】

    いま(A)           …走ってしまう行…   旋回t    直進s…     旋回t2
      │                                    ↓        ↓          ↓
      └─ 計画との開き ──▶ 先読み(P) ──▶ 向きを変え ─▶ 節点へ ─▶ 計画の向きに戻す

1. **旋回 t** … 節点（直進の行き先）へ**まっすぐ向く**角度に直す
2. **直進 s** … そこまでの**距離**に直す（続く同符号の直進はまとめて比で伸縮）
3. **旋回 t2** … 直進で傾いたぶんを足し引きし、**計画の向きへ戻す**

この3つで、横のずれ・前後のずれ・向きのずれが1回でそろう。旋回は**その場**で
回るので位置を変えない（`FUNCNO 3`）。だから2の距離は1の角度に影響されない。

【直さないとき（そのほうがましなとき）】

* 開きが小さい（`MIN_GAP_MM` / `MIN_HEAD_DEG`）… 毎行直すと振動する
* 開きが大きすぎる（`MAX_TRUST_MM` / `MAX_TRUST_DEG`）… 自己位置のほうが壊れている
* 直した値が元から離れすぎる（`MAX_DISTANCE_RATIO` / `MAX_TURN_ADJUST_DEG`）
  … **経路が壊れるよりは、ずれたまま走るほうがましである**
* 先に「旋回＋直進」の形が無い … 直す場所が無い

【どこまで細かく直せるか】
**`SCR` も `SCD` も `int16_t`**（`Scenario/CommandSet.h`）なので、角度は1度・距離は1mm
刻みでしか指示できない。300mm の脚なら 1度 ≒ 5mm が丸めで残る。旋回1回の残差
（±0.5度。#51）と同じくらいなので、**mm まで詰める道具ではない。**
「格子1マス（250mm）よりずっと小さく保つ」ための道具である。

安全弁は**いつも PC が返した元の値と比べる**（`RouteRow.original_*`）。直した値を
基準にすると、少しずつ直すたびに経路が流れていく。
"""

import logging

from route_plan import wrap_deg

logger = logging.getLogger(__name__)

#: 何行先から直してよいか。**実機で測ること**（`Docs/route_plan_follow.md`）。
#: 置いてから C++ が取り込むまで最大 100ms、そこから行が始まるまでにも時間がいる。
#: 1 は「いま走っている行の次」で、C++ 側の下限（`Scenario::canInsertWhileRunning`
#: は実行中の行より後ろしか入れない）ぎりぎり。既定は1行ぶん余裕を持たせる
LEAD_ROWS = 2

#: 直せる形（旋回＋直進）をどこまで先に探すか[行]
SEARCH_ROWS = 10

#: これより小さい開きは直さない（振動させない）
#:
#: **実機の3走行24回の判断から決めた**（2026-09-21、Lコース）。
#:
#:     開き 60mm 超    4回改善 / 1回悪化   ← ここだけがほぼ確実に効く
#:     開き 30〜60mm   4回改善 / 4回悪化   五分五分
#:     開き 30mm 未満  1回改善 / 6回悪化   **直すと悪くなる**
#:
#: 向きの下限は**空走の角度より上げること**（6.1.1）。行の頭で読む向きは、
#: 前の旋回の空走が残っていて 3〜6度 偏る。1.5 だとその偽の傾きで書き直してしまう
MIN_GAP_MM = 60.0
MIN_HEAD_DEG = 7.0

#: これより大きい開きは**自己位置のほうを疑う**。直さない
#: （`Docs/position_correction_design.md` 6章の防波堤）
MAX_TRUST_MM = 600.0
MAX_TRUST_DEG = 45.0

#: 安全弁。元の値からこれ以上離れる書き直しは採らない
MAX_TURN_ADJUST_DEG = 15.0
MAX_DISTANCE_RATIO = 0.30


class Edit:
    """1行ぶんの書き直し"""

    def __init__(self, index, field, old, new):
        self.index = index
        self.field = field          # "SCR" か "SCD"
        self.old = old
        self.new = new

    @property
    def delta(self):
        return self.new - self.old

    def __repr__(self):
        fmt = "%d:%s %.1f>%.1f" if self.field == "SCR" else "%d:%s %.0f>%.0f"
        return fmt % (self.index, self.field, self.old, self.new)


class Decision:
    """1回ぶんの判断。**採らなかったときも理由ごと残す**（走行後に効いたかを見るため）"""

    def __init__(self, row_index, gap_mm, along_mm, cross_mm, head_deg,
                 edits=None, reason=""):
        self.row_index = row_index
        self.gap_mm = gap_mm
        self.along_mm = along_mm
        self.cross_mm = cross_mm
        self.head_deg = head_deg
        self.edits = edits or []
        self.reason = reason

    @property
    def accepted(self):
        return bool(self.edits)

    @property
    def rows(self):
        return [e.index for e in self.edits]

    @property
    def last_row(self):
        """書き直した最後の行。この行を走り終えた時点で開きが縮んでいるはず"""
        return max(self.rows) if self.edits else self.row_index

    def apply(self, plan):
        """決めた値を経路へ入れる（次の判断はこの値を前提に先を読む）

        **置く行を組むより先に呼ぶこと。** 行は経路の値をそのまま写すので、
        入れる前に組むと**直す前の値が置かれる**（何も変わらない行を置いてしまう）。
        """
        self._write(plan, "new")

    def revert(self, plan):
        """入れた値を戻す（置けなかったとき）

        置けていないのに経路を直したことにすると、以後の判断が「直った前提」で
        先を読み、ずれが取り返せなくなる。
        """
        self._write(plan, "old")

    def _write(self, plan, which):
        for edit in self.edits:
            row = plan.rows[edit.index]
            value = edit.new if which == "new" else edit.old
            if edit.field == "SCR":
                row.set_turn(value)
            else:
                row.set_distance(value)

    def describe(self):
        return "|".join(repr(e) for e in self.edits)


class Window:
    """直すのに使う「旋回 → 直進… → 旋回」のかたまり"""

    def __init__(self, turn, straights, next_turn):
        self.turn = turn                # 旋回の行番号
        self.straights = straights      # 直進の行番号（1つ以上、同じ符号）
        self.next_turn = next_turn      # 続く旋回の行番号。無ければ None

    @property
    def goal_index(self):
        """節点の番号（直進を走り終えた時点）"""
        return self.straights[-1] + 1


class RouteCorrector:
    """開きを見て、先の行をどう直すかを決める。**状態を持たない**"""

    def __init__(self, lead_rows=None, search_rows=None):
        self.lead_rows = LEAD_ROWS if lead_rows is None else lead_rows
        self.search_rows = SEARCH_ROWS if search_rows is None else search_rows

    # --- 直せる形を探す -------------------------------------------------------

    def find_window(self, plan, start):
        """`start` 行から先で、最初に使える「旋回 → 直進… → 旋回」を探す

        直進が続くときはまとめて1本の脚として扱う。**符号が混ざる並び
        （直進のあとすぐ後退）は使えない**。まとめて比で伸縮しても、行って
        戻るぶんが打ち消し合って距離が変えられないため。
        """
        rows = plan.rows
        limit = min(len(rows), start + self.search_rows)
        for t in range(start, limit):
            if not rows[t].is_turn or self.after_backward(rows, t):
                continue
            straights = []
            sign = 0
            k = t + 1
            while k < len(rows) and rows[k].is_straight:
                row_sign = 1 if rows[k].distance_mm > 0 else -1
                if sign == 0:
                    sign = row_sign
                elif row_sign != sign:
                    break
                straights.append(k)
                k += 1
            if not straights:
                continue
            next_turn = k if (k < len(rows) and rows[k].is_turn
                              and not self.after_backward(rows, k)) else None
            return Window(t, straights, next_turn)
        return None

    @staticmethod
    def after_backward(rows, index):
        """その旋回は後退の直後か。**直後なら書き直さない**

        実機で2走行続けて、**後退 840mm の直後の旋回だけが壊れた**
        （2026-09-21。-45度 を -37/-38度 へ短くした行で、実際は -8度 しか回らず、
        そこから終点の開きが 177mm へ跳ねた）。

        起きていること:

        1. 後退の直後の旋回で車輪が滑り、**エンコーダがジャイロの 3.6倍**数える
        2. `TurnAngle` の滑り検出は**ジャイロが 10度 回ってから**始まる
           （`TURN_SLIP_CHECK_DEG`）。ジャイロ 8.3度 で終わったので一度も走っていない
        3. 水増しされたエンコーダ角が、**短くした目標に早く届いて**止まる

        元の -45度 なら、届くまでにジャイロが 10度 を超えて切替が働く。
        **短くする書き直しが、この隙間へ入れてしまう。**
        直すべきは C++ 側（`TURN_SLIP_CHECK_DEG` を下げる）だが、それまではここで避ける。
        """
        return index > 0 and rows[index - 1].is_straight and rows[index - 1].distance_mm < 0

    # --- 判断 -----------------------------------------------------------------

    def decide(self, plan, nodes, row_index, pose):
        """
        いま `row_index` 行の頭に居て、姿勢が `pose` だったときに何をするか。

        :param plan:      経路（`route_plan.PlannedRoute`）。**いま入っている値**を見る
        :param nodes:     入り口から積んだ**元の**節点の列（目指す先）
        :param row_index: いま始まった行の番号（0 始まり）
        :param pose:      いまの自己位置（補正後）
        :returns:         `Decision`
        """
        goal_now = nodes[row_index]
        along, cross = goal_now.gap_to(pose)
        gap = goal_now.distance_to(pose)
        head = wrap_deg(pose.heading_deg - goal_now.heading_deg)

        def refuse(reason):
            return Decision(row_index, gap, along, cross, head, reason=reason)

        if gap > MAX_TRUST_MM or abs(head) > MAX_TRUST_DEG:
            return refuse("開きが大きすぎる（自己位置を疑う）")
        if gap < MIN_GAP_MM and abs(head) < MIN_HEAD_DEG:
            return refuse("開きが小さい")

        window = self.find_window(plan, row_index + self.lead_rows)
        if window is None:
            return refuse("先に旋回＋直進が無い")

        return self._solve(plan, nodes, row_index, pose, window, gap, along, cross, head)

    def _solve(self, plan, nodes, row_index, pose, window, gap, along, cross, head):
        """節点へ向け直す角度と距離を出し、安全弁にかける"""
        rows = plan.rows
        turn_row = rows[window.turn]
        straight_rows = [rows[i] for i in window.straights]

        def refuse(reason):
            return Decision(row_index, gap, along, cross, head, reason=reason)

        # 旋回の頭に居るはずの姿勢。ここまでの行は直せないので、計画どおり走るとみなす
        start = plan.advance_through(pose, row_index, window.turn)
        goal = nodes[window.goal_index]

        total = sum(r.distance_mm for r in straight_rows)
        if total == 0:
            return refuse("直進の距離が 0")
        backward = total < 0

        needed = start.distance_to(goal) * (-1.0 if backward else 1.0)
        if needed == 0:
            return refuse("向け直す先が同じ場所")

        # **`SCR` も `SCD` も int16**（Scenario/CommandSet.h）。C++ 側は小数を
        # 切り捨てるので、ここで丸めておく。丸めた値で先を読まないと、計画と
        # 実際に走る値が少しずつ食い違う
        new_turn = float(round(wrap_deg(start.heading_to(goal, backward) - start.heading_deg)))
        scale = needed / total

        edits = []
        if abs(wrap_deg(new_turn - turn_row.original_turn_deg)) > MAX_TURN_ADJUST_DEG:
            return refuse("旋回の直しが大きすぎる（%+.1f度）"
                          % wrap_deg(new_turn - turn_row.original_turn_deg))
        edits.append(Edit(window.turn, "SCR", turn_row.turn_deg, new_turn))

        for index, row in zip(window.straights, straight_rows):
            new_distance = float(round(row.distance_mm * scale))
            limit = abs(row.original_distance_mm) * MAX_DISTANCE_RATIO
            if abs(new_distance - row.original_distance_mm) > limit:
                return refuse("距離の直しが大きすぎる（%+.0f%%）"
                              % ((new_distance / row.original_distance_mm - 1.0) * 100.0))
            edits.append(Edit(index, "SCD", row.distance_mm, new_distance))

        # 直進で傾いたぶんを、続く旋回で計画の向きへ戻す
        after = start.turned(new_turn).moved(needed)
        residual = wrap_deg(nodes[window.goal_index].heading_deg - after.heading_deg)
        if window.next_turn is None:
            # 戻す先が無い。**向きを今より悪くするなら手を出さない**
            if abs(residual) > abs(head):
                return refuse("向きを戻す旋回が先に無い")
        else:
            next_row = rows[window.next_turn]
            new_next = float(round(wrap_deg(
                nodes[window.next_turn + 1].heading_deg - after.heading_deg)))
            if abs(wrap_deg(new_next - next_row.original_turn_deg)) > MAX_TURN_ADJUST_DEG:
                return refuse("戻す旋回の直しが大きすぎる（%+.1f度）"
                              % wrap_deg(new_next - next_row.original_turn_deg))
            edits.append(Edit(window.next_turn, "SCR", next_row.turn_deg, new_next))

        return Decision(row_index, gap, along, cross, head, edits=edits)
