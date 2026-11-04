"""
走行中に自己位置を打ち直す（CoordCorrectInfo の挿し込み）。

観測（床の二次元コードなど）で分かった向きを、走行中の自己位置へ反映させる。
**C++ は触らない。** シナリオの行には昔から座標補正の口があり、走行中の差し替えも
同じ読み込みを通るので、`CoordCorrectInfo` を書いた行を置けば打ち直せる
（`Docs/position_correction_design.md` 4章、`Tests/cpp/test_coord_correct.cpp`）。

    観測 ──▶ CoordCorrector ──▶ /dev/shm/sorot_scenario_update.json
                                     ↓ 100ms ごと
                                C++ ResultRead → ScenarioControl → Scenario
                                     ↓ その行を読んだとき
                                HWMonitor::correctCoordinateInfo

【先の行にしか書けない】
差し替えは同じ SNO/CNO の行を上書きするので、**すでに読み終わった行は変わらない**。
何行先に書けば間に合うかは実機で測る（`Docs/heading_correction.md`）。

【必ず「行の丸ごと」を書く】
差し替えは行を置き換える。補正だけを書くと走り方が消える。書き込む先の行の中身は
`scenario_rows.ScenarioRows` が持っていて、ここはそれに `CoordCorrectInfo` を足すだけ。

【経路の入り口では、実行順での次は「経路の先頭」】
`SW_ROUTE` の印の直後には、走行中に受け取った経路が挿し込まれる。ファイルの並びで
「1行先」を選ぶと、**経路を走り終えたあとの行**に書くことになり、走る前の向きで
打ち直してしまう（2026-09-21、実機で大回りになった）。印の行では経路の先頭を狙う。

【経路の中の行は `RouteFollower` に組ませる】
ETラリーの経路の行は、走行中に `SCD`/`SCR` も書き直される（`route_follower.py`）。
**同じ行を2か所で組み立てると、あとから置いたほうが前のぶんを消す**
（`Scenario::insertCommandSet` は行を丸ごと置き換える）。そこで、いま走っている行が
経路の中なら `route_follower.set_coord_correct()` に任せ、外（`custom.json` の行）だけ
ここで組む（`Docs/route_plan_follow.md` 6.4）。

【当て方は2つある。行に書けないときは「その場で当てる」】
行に書く道は、**書き込める先の行があるときしか使えない**。実機では
`29/2`（`SW_ROUTE` で経路の到着を待つ行）で測るが、そこは

    custom.json の SNO 29 は CNO 1 と 2 しか無く、29/3 が存在しない
    29/2 の次は走行中に挿し込まれる経路（SNO 30）で、静的な行としては無い
    29/2 は経路の到着を待っている行なので、経路もまだ手元に無い

**書ける先がどちらにも無く、2026-09-21 から 29回すべて見送られていた。**

そこで、行に書けないときは**その場で当てる**（F が用意した道）。判定結果に
ずれを載せて `coord_seq` を増やすと、C++ が 100ms 以内に拾って自己位置へ打ち直す
（`Utillity/CoordCorrect.h` → `Task/RunControl::correctCoordinate`）。

    行に書ける   → これまでどおり行に書く（その行を読んだときに効く）
    書けない     → その場で当てる（次の 100ms で効く）

**順序を変えないこと。** 行に書けるなら、そちらのほうが「いつ効くか」がはっきりする。

【打ち直したことは必ずログに残す】
残さないと、走行後に「補正が効いたのか、たまたま合ったのか」が分からない
（`Docs/position_correction_design.md` 6章）。採った回も見送った回も理由ごと出す。
数えた回数は `Tools/summarize_heading_fix.py` が集計する。
"""

import logging
import threading
from typing import Any, Dict, Optional

import route_follower
from scenario_rows import RowTarget, ScenarioRows
from scenario_update import ScenarioMailbox
from state import ResultStore

logger = logging.getLogger(__name__)

#: 既定値としての「渡されなかった」印（None は「使わない」の意味で使うため）
_UNSET = object()

#: 絶対座標補正フラグ（C++ の CommandSet.h と同じビット）
ACAF_X = 0x01
ACAF_Y = 0x02
ACAF_HEADING = 0x04

#: その場で当てるときの置き場（`Common/state.py`）。
#: **C++ は `coord_seq` が増えたときだけ取り込む**（`RoughSpot/Cpp/ResultRead.cpp`）
COORD_KEYS = ("coord_dx", "coord_dy", "coord_dr", "coord_seq")

#: 自己位置との差がこれを超えたら採用しない[度]
#:
#: **「観測は常に正しい」と思った瞬間に破綻する。** 別の模様を二次元コードと
#: 見た、ゲートの柱に隠れて隅を取り違えた、といった測り間違いは必ず起きる。
#: **5° は現地の実測から決めた**（2026-09-22）。その日の測定値は
#: -0.17 / +0.45 / +0.52 / -0.19 / +0.48° と小さくそろう一方、外れ値が
#: +6.75 / +7.79° と飛んだ。同じ走行で旋回2回が 7° 大きく回っており、
#: 誤測定が当たっていた疑いが濃い（走らせ方を直す側の `HEADING_CORRECT_MAX_DEG` も
#: 15° → 5° に下げてある）。**5° なら、良い値を通して外れ値だけ落とせる。**
#:
#: C++ 側の `COORD_CORRECT_MAX_DEG`（`Utillity/CoordCorrect.h`）は 10° で、
#: **そこを超える値は送っても黙って捨てられる。** ここはそれより厳しくしてあるので、
#: 捨てた理由が必ずログに残る。
#:
#: **大きなずれを直したいなら上げる**（積もった向きの誤りは 10° を超えうる。
#: C の観測は 11° のずれを捕まえた実績がある）。ただし外れ値も通ることになる
MAX_CORRECTION_DEG = 5.0


def wrap_deg(deg: float) -> float:
    """角度を -180 < x <= 180 に畳む"""
    x = (float(deg) + 180.0) % 360.0 - 180.0
    return 180.0 if x == -180.0 else x


class CoordCorrector:
    """
    自己位置の打ち直しを走行中のシナリオへ挿し込む。走行体で1つを共有する。

    **止めない。** 書き込む先が無い・差が大きい・置けない、のどれでも走行は続く。
    打ち直せなければ、これまでどおり推定だけで走るだけ。
    """

    _instance: Optional["CoordCorrector"] = None
    _instance_lock = threading.Lock()

    @classmethod
    def shared(cls) -> "CoordCorrector":
        with cls._instance_lock:
            if cls._instance is None:
                cls._instance = cls()
            return cls._instance

    @classmethod
    def reset_shared(cls) -> None:
        """共有している1つを捨てる（テスト用）"""
        with cls._instance_lock:
            cls._instance = None

    def __init__(self, rows: Optional[ScenarioRows] = None, mailbox=ScenarioMailbox,
                 follower: Any = _UNSET):
        self._lock = threading.Lock()
        self._rows = rows or ScenarioRows.shared()
        self._mailbox = mailbox
        #: 経路の行を持っているもの。None にすると経路の中でも自分で組む（テスト用）
        self._follower = (route_follower.RouteFollower.get_instance()
                          if follower is _UNSET else follower)
        #: 打ち直した回数（行に書いたぶん＋その場で当てたぶん）
        self.applied = 0
        #: そのうち、その場で当てたぶん
        self.applied_in_place = 0
        #: その場で当てるときの通し番号（C++ は増えた回だけ取り込む）
        self._in_place_seq = 0
        #: 見送った回数
        self.skipped = 0
        #: 動かした角度の合計[度]（絶対値）。「何度ずらしたか」を走行後に見るため
        self.total_shift_deg = 0.0
        #: 直近の打ち直し（確かめる用）
        self.last: Optional[Dict[str, Any]] = None

    def correct_heading(self, pose_heading_deg: float, measured_heading_deg: float,
                        sno: int, cno: int, steps: int, note: str = "") -> bool:
        """
        向きだけを打ち直す（ACAF 0x04）。位置（CAPX/CAPY）は触らない。

        :param pose_heading_deg:     いま走行体が思っている向き[度]（要求に載ってきた値）
        :param measured_heading_deg: 観測から求めた本当の向き[度]。同じ座標系で渡すこと
        :param sno: いま走っている行のシナリオ番号
        :param cno: いま走っている行のコマンドセット番号
        :param steps: 何行先に書くか
        :param note: ログに残す一言（観測の中身など）
        :returns: 置けたら True
        """
        shift = wrap_deg(measured_heading_deg - pose_heading_deg)
        if abs(shift) > MAX_CORRECTION_DEG:
            self._skip("自己位置との差 %+.2f° が上限 %.1f° を超える（%s）"
                       % (shift, MAX_CORRECTION_DEG, note or "観測"))
            return False

        # **いまの向きの近くに書く（-180〜180 へ畳まない）。**
        # C++ の mCoordHeading は回った角を足し続けた値で、一周を越えても畳まれない。
        # 畳んだ値を書くと向きが 360° 飛び、その周期の TurnSpeed（先読み停止が使う）が
        # 出鱈目になる。打ち直しで動かすのは shift の分だけにする
        capr = int(round(float(pose_heading_deg) + shift))

        target = self._route_target(sno, cno, steps, capr)
        if target is False:
            return False                    # 経路の中で、載せられなかった
        if target is None:
            target = self._rows.plan_target(sno, cno, steps)
            if target is None:
                # **行に書けない。その場で当てる**（29/2 のように先の行が無い場所）
                return self._apply_in_place(shift, sno, cno, pose_heading_deg, note)
            target.row["CoordCorrectInfo"] = {"ACAF": ACAF_HEADING, "CAPR": capr}
            if not self._mailbox.place_rows([target.row]):
                self._skip("置き場へ書けなかった")
                return False

        with self._lock:
            self.applied += 1
            self.total_shift_deg += abs(shift)
            self.last = {
                "sno": target.sno, "cno": target.cno, "source": target.source,
                "pose_heading": float(pose_heading_deg),
                "measured_heading": float(measured_heading_deg),
                "shift_deg": shift, "capr": capr, "note": note,
            }
            applied, total = self.applied, self.total_shift_deg

        # **この1行が無いと、走行後に効いたかどうかが分からない。**
        # 書式を変えるときは Tools/summarize_heading_fix.py も直すこと
        logger.info("向きを打ち直しました: %d/%d へ CAPR=%d"
                    "（自己位置 %.1f° を %+.2f° 動かす・%d行先・%s）"
                    "%d回目・累計 %.1f°%s",
                    target.sno, target.cno, capr,
                    float(pose_heading_deg), shift, steps, target.source,
                    applied, total, ("・" + note) if note else "")
        return True

    def _apply_in_place(self, shift: float, sno: int, cno: int,
                        pose_heading_deg: float, note: str) -> bool:
        """
        いる場所で自己位置を打ち直す（行に書けないとき）。

        判定結果に**ずれ**（絶対値ではない）を載せて `coord_seq` を増やすと、
        C++ が 100ms 以内に拾って打ち直す。x/y は触らないので 0 を入れる。

        **旋回の最中は当てない。** `ACAF 0x04` は `driveDirection` も置き換えるため、
        その行の角度の判定が狂う（C++ 側でも止めているが、送る前に止める）。
        """
        keeps = self._rows.keeps_heading_at(sno, cno)
        if keeps is False:
            self._skip("いまの行（%d/%d）は向きが変わる行なので、その場では当てない%s"
                       % (sno, cno, ("・" + note) if note else ""))
            return False

        store = ResultStore.get_instance()
        with self._lock:
            self._in_place_seq += 1
            seq = self._in_place_seq
        store.set("coord_dx", 0.0)
        store.set("coord_dy", 0.0)
        store.set("coord_dr", round(float(shift), 2))
        store.set("coord_seq", seq)

        with self._lock:
            self.applied += 1
            self.applied_in_place += 1
            self.total_shift_deg += abs(shift)
            self.last = {
                "sno": sno, "cno": cno, "source": "inplace",
                "pose_heading": float(pose_heading_deg),
                "measured_heading": float(pose_heading_deg) + shift,
                "shift_deg": shift, "capr": None, "note": note, "seq": seq,
            }
            applied, total = self.applied, self.total_shift_deg

        # **書式を変えるときは Tools/summarize_heading_fix.py も直すこと**
        logger.info("向きを打ち直しました: %d/%d でその場"
                    "（自己位置 %.1f° を %+.2f° 動かす・seq %d・inplace）"
                    "%d回目・累計 %.1f°%s",
                    sno, cno, float(pose_heading_deg), shift, seq,
                    applied, total, ("・" + note) if note else "")
        return True

    def _route_target(self, sno: int, cno: int, steps: int, capr: int):
        """経路の中なら `RouteFollower` に組ませる

        :returns: 載せたら `RowTarget`、載せられなかったら False、
                  経路の外なら None（呼ぶ側が `ScenarioRows` で組む）
        """
        follower = self._follower
        if follower is None:
            return None

        if follower.route_row_index((sno, cno)) is None:
            # 経路の外。ただし**経路の入り口の印**なら、実行順での次は経路の先頭
            if not (self._rows.is_route_marker(sno, cno)
                    and getattr(follower, "plan", None) is not None):
                return None

        row_index = follower.next_heading_row(steps, current_key=(sno, cno))
        if row_index is None:
            self._skip("経路の %d 行先までに向きの変わる行がある（旋回を挟むと値が嘘になる）"
                       % steps)
            return False
        if not follower.set_coord_correct(row_index, ACAF_HEADING, capr=capr,
                                          current_key=(sno, cno)):
            self._skip("経路の行へ載せられなかった（%d行目）" % row_index)
            return False

        numbers = follower.numbering.numbers_for(row_index)
        return RowTarget({"SNO": numbers[0], "CNO": numbers[1]}, "route")

    def _skip(self, why: str) -> None:
        with self._lock:
            self.skipped += 1
            skipped = self.skipped
        logger.info("向きの打ち直しを見送りました（%d回目）: %s", skipped, why)

    def summary(self) -> Dict[str, Any]:
        """走行後に見る数（打ち直した回数・見送った回数・動かした合計）"""
        with self._lock:
            return {"applied": self.applied,
                    "applied_in_place": self.applied_in_place,
                    "applied_in_row": self.applied - self.applied_in_place,
                    "skipped": self.skipped,
                    "total_shift_deg": round(self.total_shift_deg, 2),
                    "last": dict(self.last) if self.last else None}
