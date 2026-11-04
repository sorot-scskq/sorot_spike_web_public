"""
止まって床コードを読み、次の行へ補正旋回を設定する。

止まって測る行と、補正旋回に差し替える行の組は `WAIT_ROWS`。
- 20/14 → 20/15：以前の置き場所
- 29/2 → 29/3：ETラリーの入口（経路を頼んだあと、経路の入り口の手前。2026-10-31）

どの組かは、SCRIPT が呼ばれた行の番号（要求に載る SNO/CNO）で決める。
"""

import logging
import statistics
import time
from typing import Optional

from camera import RobotCamera
from floor_code_reader import FloorCodeReader
from remote_control import Command, Status
from robot_pose import RobotPose
from scenario_update import ScenarioMailbox

logger = logging.getLogger(__name__)


#: シナリオの SCRIPT に書く名前（`wait_rows` が待ちの行を探すのに使う）
SCRIPT_PATH = "2026/recog/qr_turn_correction.py"


class QrTurnCorrection:
    SNO = 20
    WAIT_CNO = 14
    TURN_CNO = 15
    #: 止まって測る行 →（同じ SNO の）補正旋回に差し替える CNO
    WAIT_ROWS = {(20, 14): 15, (29, 2): 3}
    SAMPLE_SECONDS = 1.2
    SAMPLE_INTERVAL = 0.05
    MIN_SAMPLES = 2
    MAX_SPREAD_DEG = 10.0
    MAX_CORRECTION_DEG = 22.5
    TURN_POWER = 70
    UPDATE_SETTLE_SECONDS = 0.2
    UPDATE_TIMEOUT_SECONDS = 1.0

    _instance: Optional["QrTurnCorrection"] = None

    @classmethod
    def shared(cls) -> "QrTurnCorrection":
        if cls._instance is None:
            cls._instance = cls()
        return cls._instance

    @classmethod
    def reset_shared(cls) -> None:
        cls._instance = None

    def __init__(self, reader=None, mailbox=ScenarioMailbox):
        self.reader = reader
        self.mailbox = mailbox
        self.done = False
        self.fallback = None
        #: 測り終えた待ちの行。待ちの行ごとに覚える（別の行の「済み」で抜けないように）
        self.done_rows = set()
        #: いま測っている待ちの行 (SNO, CNO)
        self.active = (self.SNO, self.WAIT_CNO)

    @staticmethod
    def correction_row(scr: int, sno: int = SNO, wait_cno: int = WAIT_CNO,
                       turn_cno: int = TURN_CNO) -> dict:
        """補正旋回の行（turn_cno の完全な置換行）。旋回符号はSCRと同じ向きにする。"""
        power = 0 if scr == 0 else (QrTurnCorrection.TURN_POWER if scr > 0
                                    else -QrTurnCorrection.TURN_POWER)
        return {
            "Comment": "QR角度補正旋回",
            "SNO": sno,
            "CNO": turn_cno,
            "PRVSNO": sno,
            "PRVCNO": wait_cno,
            "RunInfo": {"FUNCNO": 3, "FWD": 0, "TRN": power,
                        "KP": 0, "KI": 0, "KD": 0, "NOBLNCE": 1},
            "SwitchInfo": {"SCJFN": 33, "SCT": 4000, "SCD": 0, "SCR": scr,
                            "SCC": 0, "SCV": 0, "SCX": 0, "SCY": 0},
            "CoordCorrectInfo": {},
        }

    def request(self, request_id: int = 0) -> None:
        """SCRIPTから呼ばれる。完了までは待ちの行のRemote制御が待つ。"""
        self.done = False
        self.fallback = None
        self.active = self._asked_row()
        self.done_rows.discard(self.active)
        try:
            deviations = self._measure()
            if len(deviations) < self.MIN_SAMPLES:
                raise ValueError("床コードの向きを測れませんでした")
            spread = max(deviations) - min(deviations)
            if spread > self.MAX_SPREAD_DEG:
                raise ValueError("測定値のばらつき %.2f° が大きすぎます" % spread)

            deviation = statistics.median(deviations)
            if abs(deviation) > self.MAX_CORRECTION_DEG:
                raise ValueError("補正量 %+.2f° が上限を超えています" % deviation)

            scr = int(round(-deviation))
            if not self._update_row(scr):
                raise RuntimeError("%d/%d の更新に失敗しました" % (self.active[0], self._turn_cno()))
            logger.info("QR角度補正（%d/%d）: ずれ %+.2f°、追加旋回 SCR=%d° (%d枚)",
                        self.active[0], self.active[1], deviation, scr, len(deviations))
            if scr != 0:
                # 旋回の行のあと、向き保持の行で目標まで回し切るので、後は2回撮る
                from correction_photo import CorrectionPhoto
                CorrectionPhoto.shared().record(getattr(self, "last_frame", None), "qr_turn",
                                                deviation, after_seconds=(1.0, 2.5))
        except Exception as error:  # noqa: BLE001 - 補正なしで安全に次へ進む
            self.fallback = str(error)
            logger.warning("QR角度補正を行いません: %s", error)
        finally:
            self.done = True
            self.done_rows.add(self.active)

    def wait_rows(self) -> dict:
        """
        止まって測る行 → 補正旋回に差し替える行の CNO。

        決め打ちの `WAIT_ROWS` に、シナリオのファイルで SCRIPT にこのスクリプトを書いた行を足す
        （差し替える行はその次の CNO）。点検用のシナリオ（`Tools/make_rally_drive_check.py`）は
        入口を何度も通るので、29/2 だけでは足りない。
        """
        rows = dict(self.WAIT_ROWS)
        try:
            from scenario_rows import ScenarioRows
            for sno, cno in ScenarioRows.shared().rows_with_script(SCRIPT_PATH):
                rows.setdefault((sno, cno), cno + 1)
        except Exception:  # noqa: BLE001 — 読めなくても決め打ちの行では動く
            logger.warning("QR角度補正: シナリオから待ちの行を拾えませんでした", exc_info=True)
        return rows

    def control(self, obs):
        """対象の停止行だけ完了を管理し、それ以外は既存制御へ渡す。"""
        if (obs.sno, obs.cno) in self.wait_rows():
            if (obs.sno, obs.cno) not in self.done_rows:
                return Command(status=Status.NG)
            return Command(status=Status.OK)
        return None

    def _asked_row(self) -> tuple:
        """SCRIPT が呼ばれた行。要求に番号が無い・知らない行なら 20/14 として扱い、そう言う"""
        asked = RobotPose.get_instance().snapshot()
        row = (asked.get("sno"), asked.get("cno"))
        waits = self.wait_rows()
        if row in waits:
            return row
        logger.warning("QR角度補正: 呼ばれた行 %s/%s は待ちの行（%s）でないので %d/%d として扱います",
                       row[0], row[1], ", ".join("%d/%d" % r for r in waits),
                       self.SNO, self.WAIT_CNO)
        return (self.SNO, self.WAIT_CNO)

    def _turn_cno(self) -> int:
        return self.wait_rows()[self.active]

    def _measure(self) -> list:
        reader = self.reader
        if reader is None:
            camera = RobotCamera.shared()
            reader = FloorCodeReader(frame_source=camera.latest_frame)

        deviations = []
        self.last_frame = None
        deadline = time.monotonic() + self.SAMPLE_SECONDS
        while time.monotonic() < deadline:
            result = reader.read()
            if result.get("ok"):
                deviations.append(float(result["deviation_deg"]))
                self.last_frame = getattr(reader, "last_frame", None)
            time.sleep(self.SAMPLE_INTERVAL)
        return deviations

    def _update_row(self, scr: int) -> bool:
        if scr == 0:
            return True
        deadline = time.monotonic() + self.UPDATE_TIMEOUT_SECONDS
        while self.mailbox.pending() and time.monotonic() < deadline:
            time.sleep(0.02)
        if self.mailbox.pending():
            return False
        if not self.mailbox.place_rows([self.correction_row(scr, self.active[0], self.active[1],
                                                            self._turn_cno())]):
            return False

        deadline = time.monotonic() + self.UPDATE_TIMEOUT_SECONDS
        while self.mailbox.pending() and time.monotonic() < deadline:
            time.sleep(0.02)
        if self.mailbox.pending():
            return False
        time.sleep(self.UPDATE_SETTLE_SECONDS)
        return True


class QrTurnController:
    """QR補正待ちと、既存のヒントカード制御を1つの口で扱う。"""

    def __init__(self, fallback=None):
        self.turn = QrTurnCorrection.shared()
        self.fallback = fallback

    def control(self, obs):
        command = self.turn.control(obs)
        if command is not None:
            return command
        return self.fallback(obs) if self.fallback is not None else None


def run(request_id: int) -> None:
    QrTurnCorrection.shared().request(request_id)