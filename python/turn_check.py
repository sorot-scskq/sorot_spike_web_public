"""
格子の中で旋回するたびに、止まって床の二次元コードで向きを測り、その場で直してから進む（2026・ETラリー）。

【なぜ】
旋回は1回あたり σ0.7°（実機の実測。`Docs/spin_turn.md`）ずれ、ジャイロにもエンコーダにも見えない。
直進の行の頭で測って向き保持の目標を直す道（`heading_task`）はあるが、

* 測るのは1行に1回で、走り出しの絵（向きがまだ動いている）しか撮れない
* 前の補正を向き保持が追っている間に撮った絵は捨てる（1行が短いと半分ほど捨てる）

ので、格子1マス（246mm）ごとに旋回する経路（点検 X）では、旋回のたびに積もるずれに追いつけない。
入口の QR角度補正（`qr_turn_correction.py`）と同じことを、格子の中の旋回のたびにやる。

【どうやって】
直進の行の頭（`recog/grid_heading.py`）で先を見て、**次の行が旋回で、その次が格子の中の直進**
なら、旋回の行の直後に2行を差し込む（`Scenario.insertCommandSet` は、知らない番号の行を
PRVSNO/PRVCNO の直後へ入れる）。

    旋回（ファイルの行）
      └ 止まって測る行   CNO = 100 + 旋回の CNO     SCRIPT qr_turn_correction.py、SCJFN 2049
          └ 直しの旋回    CNO = 101 + 旋回の CNO     測ったずれの分だけ回す行に差し替わる（ずれが無ければ 0.1 秒待つだけ）

測るのも、抜けさせるのも、直しの旋回に差し替えるのも、入口の QR角度補正の仕組みをそのまま使う
（`QrTurnCorrection.register_wait_row`）。測る時間だけ短くする（`SAMPLE_SECONDS`）。

【引き換え】
旋回のたびに止まるので遅くなる（1回あたり 0.5〜1 秒）。点検 X は 1組に旋回が 80回ほどある。
入れるかどうかは置き場（`ENABLED_SWITCH`）で切り替える。既定は切。
"""

import logging
import threading
import time
from typing import Any, Dict, Optional

from route_register import SwitchFile
from scenario_update import ScenarioMailbox

logger = logging.getLogger(__name__)

#: 入れるか（置き場が無いときの既定）
ENABLED = False

#: `ENABLED` を走行体で切り替える置き場（1行目に 1 か 0）
ENABLED_SWITCH = "./sorot_spike/turn_check.txt"

#: 止まって測る行の SCRIPT（入口の QR角度補正と同じもの）
SCRIPT_PATH = "2026/recog/qr_turn_correction.py"

#: 格子の中の直進の行の SCRIPT。次の直進がこれなら、格子の中の旋回とみなす
GRID_SCRIPT = "2026/recog/grid_heading.py"

#: 差し込む行の番号のずらし。CNO は 255 まで（`scenario_rows.MAX_CNO`）
CNO_OFFSET = 100

#: 止まって測る時間[s]。入口（1.2 秒）より短くする。止まっているので数枚で足りる
SAMPLE_SECONDS = 0.4

#: 止まって測る行の時間切れ[ms]。測れなくてもこれで先へ進む
WAIT_TIMEOUT_MS = 3000

#: 置き場が引き取られるのを待つ上限[s]
PLACE_TIMEOUT_SECONDS = 0.5

#: 旋回の切替条件（SwitchMethod の SW_DIR）
SW_DIR = 32


def is_turn(row: Optional[Dict[str, Any]]) -> bool:
    """その場旋回の行か（FUNCNO 3 で、角度で止まる）"""
    if not isinstance(row, dict):
        return False
    run = row.get("RunInfo") or {}
    switch = row.get("SwitchInfo") or {}
    return run.get("FUNCNO") == 3 and (int(switch.get("SCJFN", 0) or 0) & SW_DIR) != 0


def is_grid_straight(row: Optional[Dict[str, Any]]) -> bool:
    """格子の中の直進の行か（向きを測る SCRIPT が付いた、前へ進む行）"""
    if not isinstance(row, dict):
        return False
    run = row.get("RunInfo") or {}
    return str(run.get("SCRIPT", "")).strip() == GRID_SCRIPT and (run.get("FWD", 0) or 0) > 0


def wait_row(sno: int, turn_cno: int) -> Dict[str, Any]:
    """止まって測る行。測り終えると QR角度補正の制御が抜けさせる（SCJFN 2049）"""
    cno = CNO_OFFSET + turn_cno
    return {
        "Comment": "旋回のあと止まって向きを測る（turn_check）",
        "SNO": sno, "CNO": cno, "PRVSNO": sno, "PRVCNO": turn_cno,
        "RunInfo": {"SCRIPT": SCRIPT_PATH, "FUNCNO": 5, "FWD": 0, "TRN": 0,
                    "KP": 0, "KI": 0, "KD": 0, "NOBLNCE": 1},
        "SwitchInfo": {"SCJFN": 2049, "SCT": WAIT_TIMEOUT_MS, "SCD": 0, "SCR": 0,
                       "SCC": 0, "SCV": 0, "SCX": 0, "SCY": 0},
        "CoordCorrectInfo": {},
    }


def placeholder_row(sno: int, turn_cno: int) -> Dict[str, Any]:
    """直しの旋回に差し替わる行。ずれが無ければ 0.1 秒待つだけ"""
    cno = CNO_OFFSET + turn_cno + 1
    return {
        "Comment": "旋回のあとの直しの旋回に差し替わる行（turn_check）",
        "SNO": sno, "CNO": cno, "PRVSNO": sno, "PRVCNO": cno - 1,
        "RunInfo": {"FUNCNO": 5, "FWD": 0, "TRN": 0, "KP": 0, "KI": 0, "KD": 0, "NOBLNCE": 1},
        "SwitchInfo": {"SCJFN": 1, "SCT": 100, "SCD": 0, "SCR": 0,
                       "SCC": 0, "SCV": 0, "SCX": 0, "SCY": 0},
        "CoordCorrectInfo": {},
    }


class TurnCheck:
    """差し込んだ行を覚えておき、同じ旋回に2回差し込まない"""

    _instance: Optional["TurnCheck"] = None
    _instance_lock = threading.Lock()

    @classmethod
    def get_instance(cls) -> "TurnCheck":
        with cls._instance_lock:
            if cls._instance is None:
                cls._instance = cls()
            return cls._instance

    @classmethod
    def reset_instance(cls) -> None:
        with cls._instance_lock:
            cls._instance = None
        from qr_turn_correction import QrTurnCorrection
        QrTurnCorrection.clear_registered_wait_rows()

    def __init__(self, mailbox=ScenarioMailbox, rows=None, clock=time.monotonic, sleep=time.sleep):
        self._mailbox = mailbox
        self._rows = rows
        self._clock = clock
        self._sleep = sleep
        self._lock = threading.Lock()
        #: 差し込んだ旋回 (SNO, CNO)
        self.placed = set()

    @staticmethod
    def enabled() -> bool:
        return bool(SwitchFile.read(ENABLED_SWITCH, ENABLED))

    def _scenario(self):
        if self._rows is not None:
            return self._rows
        from scenario_rows import ScenarioRows
        return ScenarioRows.shared()

    def plan_after(self, sno: int, cno: int) -> bool:
        """
        直進の行（sno, cno）の頭で呼ぶ。次が格子の中の旋回なら、その直後に止まって測る行を差し込む。

        :returns: 差し込んだら True
        """
        if not self.enabled():
            return False
        rows = self._scenario()
        turn_cno = cno + 1
        if not is_turn(rows.row_at(sno, turn_cno)) or not is_grid_straight(rows.row_at(sno, turn_cno + 1)):
            return False
        if CNO_OFFSET + turn_cno + 1 > 255:
            return False
        with self._lock:
            if (sno, turn_cno) in self.placed:
                return False
            if not self._place([wait_row(sno, turn_cno), placeholder_row(sno, turn_cno)]):
                logger.info("旋回のあとの向きの確かめを差し込めませんでした（%d/%d）", sno, turn_cno)
                return False
            self.placed.add((sno, turn_cno))
        from qr_turn_correction import QrTurnCorrection
        QrTurnCorrection.register_wait_row(sno, CNO_OFFSET + turn_cno, CNO_OFFSET + turn_cno + 1,
                                           sample_seconds=SAMPLE_SECONDS)
        logger.info("旋回 %d/%d のあとに、止まって向きを測る行を差し込みました（%d/%d）",
                    sno, turn_cno, sno, CNO_OFFSET + turn_cno)
        return True

    def _place(self, rows) -> bool:
        deadline = self._clock() + PLACE_TIMEOUT_SECONDS
        while self._mailbox.pending() and self._clock() < deadline:
            self._sleep(0.02)
        if self._mailbox.pending():
            return False
        return bool(self._mailbox.place_rows(rows))
