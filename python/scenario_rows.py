"""
走行中のシナリオの行を、Python 側でも持っておく。

**走行中の差し替えは、行を丸ごと置き換える。** 打ち直し（CoordCorrectInfo）だけを
書いた行を置くと、その行の走り方（FUNCNO / FWD / 切替条件）が 0 になって走らなくなる
（`Tests/cpp/test_coord_correct.cpp` の「行は丸ごと置き換わる」）。
そこで **書き込む先の行の中身をここで持っておき、写してから補正を足す。**

行の出どころは2つある。

    シナリオのファイル   SubScenario/custom.json。番号（SNO/CNO）はファイルのまま
    PC から届いた経路   ETラリー。**番号は走行中に C++ が振り直す**（RouteInserter）

経路の番号は Python からは見えないので、SCRIPT の要求に載ってくる SNO/CNO から
学ぶ（`note_script_row`）。1回学べば、あとは足し算で分かる。学んだ番号で引いた行が
SCRIPT を持っていなければ、学び方が狂っているとみなして打ち直しをやめる。

**「観測した時点より先の行」にしか書けない。** C++ は走行中、実行位置より前への
差し替えを弾く（`Scenario::canInsertWhileRunning`）。何行先に書けば間に合うかは
実機で測る（`Docs/heading_correction.md`）。
"""

import json
import logging
import threading
from typing import Any, Dict, List, Optional, Tuple

logger = logging.getLogger(__name__)

#: 向きが変わらない走行方法（直進・向き保持直進）。C++ の RunMethod.h と同値
FUNCNO_LINEAR = 5
FUNCNO_HEADING_LINEAR = 19

#: コマンドセット番号（CNO）の上限。C++ は uint8_t で持つ
MAX_CNO = 255

#: 経路の入り口の印（SwitchMethod.h の SW_ROUTE）。
#: **この行の直後に、受け取った経路が挿し込まれる**（RouteInserter）
SW_ROUTE = 4096


def keeps_heading(row: Any) -> bool:
    """
    その行を走っても車体の向きが変わらないか。

    向きを打ち直す値は「測った瞬間の向き」なので、**測ってから打ち直すまでに
    向きが変わると嘘になる。** 直進（FUNCNO 5 / 19）で TRN が 0 の行だけを
    「変わらない」とみなす。ライントレースもカーブも旋回も、変わる側に倒す。
    """
    run = row.get("RunInfo") if isinstance(row, dict) else None
    if not isinstance(run, dict):
        return False
    if run.get("FUNCNO") not in (FUNCNO_LINEAR, FUNCNO_HEADING_LINEAR):
        return False
    return not run.get("TRN", 0)


class RowTarget:
    """打ち直しを書き込む先の行"""

    def __init__(self, row: Dict[str, Any], source: str):
        #: 書き込む行（写し。呼び出し側が CoordCorrectInfo を足してよい）
        self.row = row
        #: どこから取った行か（"file" / "route"）。ログに出す
        self.source = source

    @property
    def sno(self) -> int:
        return self.row.get("SNO", 0)

    @property
    def cno(self) -> int:
        return self.row.get("CNO", 0)

    def __repr__(self) -> str:
        return "RowTarget(%d/%d, %s)" % (self.sno, self.cno, self.source)


class ScenarioRows:
    """
    いま走っているシナリオの行。走行体で1つを共有する（`shared()`）。

    **番号が分かる行しか返さない。** 行き先の番号がシナリオに無いと、C++ は
    更新ではなく末尾への追加をする（同テストの「無い番号へ書くと末尾に足されてしまう」）。
    """

    _instance: Optional["ScenarioRows"] = None
    _instance_lock = threading.Lock()

    @classmethod
    def shared(cls) -> "ScenarioRows":
        with cls._instance_lock:
            if cls._instance is None:
                cls._instance = cls()
            return cls._instance

    @classmethod
    def reset_shared(cls) -> None:
        """共有している1つを捨てる（テスト用）"""
        with cls._instance_lock:
            cls._instance = None

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._file_rows: List[Dict[str, Any]] = []
        self._file_at: Dict[Tuple[int, int], int] = {}
        self._route_rows: List[Dict[str, Any]] = []
        self._route_script = ""
        #: 経路の CNO と並び順の差。走行中に学ぶ（None なら未学習）
        self._route_offset: Optional[int] = None
        #: 学び方が狂ったら二度と使わない
        self._route_lost = False

    # --- 行を覚える -----------------------------------------------------------

    def load_file(self, path: str) -> int:
        """
        シナリオの JSON を読む（走り出す前に1回）。

        :returns: 読めた行数。読めなければ 0（打ち直しをしないだけで走行は続く）
        """
        try:
            with open(path, encoding="utf-8") as f:
                rows = json.load(f)
        except (OSError, ValueError) as e:
            logger.warning("シナリオを読めないので打ち直しはしません（%s）: %s", path, e)
            return 0

        if not isinstance(rows, list):
            logger.warning("シナリオの形が違います（配列ではない）: %s", path)
            return 0

        with self._lock:
            self._file_rows = [r for r in rows if isinstance(r, dict)]
            self._file_at = {}
            for i, row in enumerate(self._file_rows):
                key = (row.get("SNO", 0), row.get("CNO", 0))
                self._file_at.setdefault(key, i)
        logger.info("シナリオの行を覚えました（%d 行）: %s", len(self._file_rows), path)
        return len(self._file_rows)

    def set_route(self, rows: List[Dict[str, Any]], script: str) -> None:
        """
        PC から届いた経路を覚える（`route_register` が置くのと同じ中身）。

        :param rows: コマンドセットの配列（C++ へ渡すのと同じもの）
        :param script: 観測の行に書いてある SCRIPT のパス。番号を学ぶ手がかりにする
        """
        with self._lock:
            self._route_rows = [r for r in rows if isinstance(r, dict)]
            self._route_script = script
            self._route_offset = None
            self._route_lost = False
        logger.info("経路の行を覚えました（%d 行）", len(self._route_rows))

    # --- 番号を学ぶ -----------------------------------------------------------

    def note_script_row(self, sno: int, cno: int) -> None:
        """
        SCRIPT が呼ばれた行の番号を知らせる（要求に載ってくる SNO/CNO）。

        ファイルに無い番号なら、走行中に挿し込まれた経路の行とみなし、
        **並び順との差（オフセット）を1回だけ学ぶ。**
        """
        with self._lock:
            if (sno, cno) in self._file_at or self._route_lost or not self._route_rows:
                return
            if self._route_offset is not None:
                return
            hits = self._script_indices()
            if not hits:
                return
            self._route_offset = cno - hits[0]
            logger.info("経路の番号を学びました: SNO=%d CNO=%d は経路の %d 行目",
                        sno, cno, hits[0] + 1)

    def _script_indices(self) -> List[int]:
        """経路のうち、観測の SCRIPT が書かれている行の並び順"""
        if not self._route_script:
            return []
        out = []
        for i, row in enumerate(self._route_rows):
            run = row.get("RunInfo")
            if isinstance(run, dict) and run.get("SCRIPT") == self._route_script:
                out.append(i)
        return out

    # --- 書き込む先を決める ---------------------------------------------------

    def plan_target(self, sno: int, cno: int, steps: int,
                    require_straight: bool = True) -> Optional[RowTarget]:
        """
        いま走っている行（sno/cno）から `steps` 先の行を、書き込める形で返す。

        返す行は**その行の中身をそのまま写したもの**なので、呼び出し側は
        CoordCorrectInfo を足して置くだけでよい。

        次のいずれかなら None（打ち直さない）。

            - いまの行が分からない（ファイルにも経路にも無い）
            - `steps` 先が無い（シナリオの終わり、または CNO が 255 を超える）
            - いまの行から書き込む先までに向きの変わる行がある
              （旋回・カーブ・ライントレース。**書き込む先の行そのものも含む**）

        :param steps: 何行先に書くか（1 以上）。実機で測って決める
        :param require_straight: 途中に向きの変わる行があれば諦めるか
        """
        if steps < 1:
            logger.error("打ち直しは先の行にしか書けません（steps=%d）", steps)
            return None

        with self._lock:
            target = self._plan_from_file(sno, cno, steps, require_straight)
            if target is not None:
                return target
            return self._plan_from_route(sno, cno, steps, require_straight)

    def keeps_heading_at(self, sno: int, cno: int) -> Optional[bool]:
        """その行を走っても向きが変わらないか。**知らない行なら None**

        その場で打ち直す側が使う（旋回の最中に打ち直すと、その行の止め方が狂う）。
        """
        with self._lock:
            at = self._file_at.get((sno, cno))
            if at is not None:
                return keeps_heading(self._file_rows[at])
            if self._route_offset is not None and self._route_rows:
                index = cno - self._route_offset
                if 0 <= index < len(self._route_rows):
                    return keeps_heading(self._route_rows[index])
        return None

    def is_route_marker(self, sno: int, cno: int) -> bool:
        """その行が経路の入り口の印か（`SW_ROUTE`）。

        **印の直後には、走行中に経路が挿し込まれる。** ファイルの並びでいう
        「1行先」は、実行順では経路を走り終えたあとの行になる。
        """
        with self._lock:
            at = self._file_at.get((sno, cno))
            if at is None:
                return False
            switch = self._file_rows[at].get("SwitchInfo")
            return isinstance(switch, dict) and bool(switch.get("SCJFN", 0) & SW_ROUTE)

    def _plan_from_file(self, sno: int, cno: int, steps: int,
                        require_straight: bool) -> Optional[RowTarget]:
        at = self._file_at.get((sno, cno))
        if at is None:
            return None

        switch = self._file_rows[at].get("SwitchInfo")
        if isinstance(switch, dict) and (switch.get("SCJFN", 0) & SW_ROUTE):
            # **印の行では、ファイルの並びで書かない。** 実行順では、この直後に
            # 受け取った経路の数十行が入る。並びで選ぶと「経路を走り終えたあとの行」に
            # 書くことになり、走る前の向きで打ち直してしまう
            # （2026-09-21、実機で大回りになった）。
            #
            # **経路がまだ届いていなくても同じ。** 印の行は SCT のあいだ経路を待つ行で、
            # SCRIPT はその入り口で走る。書く時点で届いていなくても、実行が次へ進む
            # までに届けば挿し込まれる。届くかどうかで書き先を変えると、間に合わせを
            # 賭けることになる
            logger.info("経路の入り口（%d/%d）なので、ファイルの並びでは書きません"
                        "（経路の行は route_follower が組みます）", sno, cno)
            return None
        to = at + steps
        if to >= len(self._file_rows):
            logger.info("打ち直しを書く先がありません（%d/%d の %d 行先はシナリオの外）",
                        sno, cno, steps)
            return None
        if require_straight and not self._straight_between(self._file_rows, at, to):
            logger.info("打ち直しをやめます（%d/%d の %d 行先までに向きの変わる行がある）",
                        sno, cno, steps)
            return None
        return RowTarget(json.loads(json.dumps(self._file_rows[to])), "file")

    def _plan_from_route(self, sno: int, cno: int, steps: int,
                         require_straight: bool) -> Optional[RowTarget]:
        if self._route_lost or self._route_offset is None or not self._route_rows:
            return None

        at = cno - self._route_offset
        if not (0 <= at < len(self._route_rows)):
            self._lose_route("番号が経路の外（CNO=%d → %d 行目）" % (cno, at + 1))
            return None

        # 学んだ番号で引いた行が、観測の SCRIPT を持っていなければ学び方が狂っている
        run = self._route_rows[at].get("RunInfo")
        if not isinstance(run, dict) or run.get("SCRIPT") != self._route_script:
            self._lose_route("CNO=%d が指す %d 行目に SCRIPT が無い" % (cno, at + 1))
            return None

        to = at + steps
        if to >= len(self._route_rows):
            logger.info("打ち直しを書く先がありません（経路の %d 行目の %d 行先は経路の外）",
                        at + 1, steps)
            return None
        if cno + steps > MAX_CNO:
            logger.info("打ち直しを書く先がありません（CNO が %d を超える）", MAX_CNO)
            return None
        if require_straight and not self._straight_between(self._route_rows, at, to):
            logger.info("打ち直しをやめます（経路の %d 行目から %d 行先までに向きの変わる行がある）",
                        at + 1, steps)
            return None

        row = json.loads(json.dumps(self._route_rows[to]))
        # 経路の番号は C++ が振り直している。学んだオフセットで書き戻す
        row["SNO"] = sno
        row["CNO"] = cno + steps
        row["PRVSNO"] = sno
        row["PRVCNO"] = cno + steps - 1
        return RowTarget(row, "route")

    @staticmethod
    def _straight_between(rows: List[Dict[str, Any]], at: int, to: int) -> bool:
        """いまの行から書き込む先まで、ひとつも向きが変わらないか。

        2つの理由で要る（`route_follower._keeps_heading_through` と同じ規則）。

        * **測ってから効くまでに向きが変われば、打ち直す値が嘘になる**
        * **書き込む先の行そのものも旋回であってはいけない。** `ACAF 0x04` は
          `driveDirection` も置き換えるが、旋回はその行に入った時点の
          `driveDirection` を基準に止める。しかも打ち直しは StatusMonitor へ、
          走り方は Navigator へ**別のタスクに**送られる
          （`ScenarioControl::nextScenario`）ので、同じ行でもどちらが先に効くかは
          決まっていない
        """
        return all(keeps_heading(rows[i]) for i in range(at, to + 1))

    def _lose_route(self, why: str) -> None:
        """経路の番号が信用できなくなったら、経路への打ち直しをやめる"""
        if not self._route_lost:
            logger.error("経路の番号が合わないので、経路への打ち直しをやめます（%s）", why)
        self._route_lost = True
        self._route_offset = None
