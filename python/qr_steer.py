"""
床の二次元コード（ゲート位置補助情報）の真ん中を通るように、横位置を直す（2026・ETラリー）。

【なぜコードの真ん中か】
床のコード（A1〜D4）はマスの真ん中にあり、ゲートは格子点2つの間にある（PC の経路 `G3-5,G4-5` など）。
**コードの真ん中から隣のコードの真ん中へまっすぐ進めば、ゲートの真ん中を通る。**
赤のボトルの回は ETラリーの入口で横に 110〜180mm ずれ、青ゲートの柱に当たっていた（2026-10-31）。

【やり方：振って、行の中で必ず戻す】
横へ平行移動する走り方が無いので、向き保持（FUNCNO 19）の目標を振って寄せる。

  振る   狙うコードがその行の中にあり、行の残りが `tilt_min_remain_mm()`（速さで決まる）以上あるとき。
         コードの真ん中へ向ける。格子に対して `MAX_TILT_DEG` で頭打ち
  戻す   コードの手前 `SQUARE_UP_MM` 以内・行の残りが少ない・狙うコードが決まらないとき。
         格子に沿った正面へ戻す（出す値＝格子に対する向きのずれ）

**振った分を行の終わりまでに戻さないと、目標が傾いたまま次の旋回へ持ち越され、経路全体が回る**
（C++ の旋回の目標は、その時点の目標から測るため）。2026-10-31 18:20 の走行で、30/5 で振った約 6° が残り、
30/7 で目標から +10.4° ずれた。そのため「戻す」を優先し、振るのは戻す余裕があるときだけにした。

止まっている行（FWD 0）では出さない。止まって測る 29/2 で振ってしまい、入口の QR角度補正と二重に直していた。

【出す先】
`HeadingTask.publish()`（向きを直す口は1本）。出す値は「いまの向き − 向けたい向き」[度]（時計回りが正）。
C++ は 1回 3°ずつ当て、10° を超える値は捨てる（`HEADING_CORRECT_STEP_DEG` / `_MAX_DEG`。変えない）。
**SCRIPT の無い行では C++ が受け取らない**（目標の控えを取らない）ので、そういう行では出さない。

【狙うコードの取り違え】
コードは 250mm おきに並ぶので、横に 125mm 以上ずれると隣の列のコードのほうが近く見える。
- ゲートが見えていれば、ゲートの真ん中の横位置に近いコードを狙う（ゲートの真ん中はコードの列の上にある）。
  ただし斜めから見たゲート（`GATE_MAX_APPROACH_DEG` 超え）は横位置が信じられないので使わない
- 見えていなければ、横のずれが `UNAMBIGUOUS_MM` 以内のコードだけを狙う
- **斜めの行（格子に 45°）では振らない。** 線の上のコードと隣のコードが横に 177mm しか離れず、
  線から 80〜100mm ずれていると左右どちらのコードも同じくらいに見えて取り違える
  （2026-11-03、30/5 で隣のコードへ寄せて黄の足に当たった）

【走る場所】
ゲートの確認（`gate_check_task`）の流れに相乗りする（カメラを2つの流れから読むと壊れるため）。
ゲートの確認が動いている間だけ働く。止めるときは `qr_steer.txt` に 0。
"""

from __future__ import annotations

import logging
import math
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, List, Optional, Tuple

from route_register import SCJFN_DISTANCE, SwitchFile

try:
    from gate_finder import CAMERA_TO_AXLE_FORWARD_MM
    from gate_steer import MAX_APPROACH_DEG as GATE_MAX_APPROACH_DEG
except ImportError:                         # パッケージとして読み込まれた場合
    from .gate_finder import CAMERA_TO_AXLE_FORWARD_MM
    from .gate_steer import MAX_APPROACH_DEG as GATE_MAX_APPROACH_DEG

logger = logging.getLogger(__name__)

#: 横の補正をするか（既定）。走行体で切り替えるなら `ENABLED_SWITCH` に 1 か 0
ENABLED = True
ENABLED_SWITCH = "./sorot_spike/qr_steer.txt"

#: 狙うコードを探す範囲[mm]（車軸から前へ）。手前はカメラに写らず、奥は四角が潰れて位置が荒れる。
#: 2026-10-31 の走行では、前方 300〜700mm にコードが写っていた
MIN_AHEAD_MM = 150.0
MAX_AHEAD_MM = 900.0

#: これより近いコードに向かうときは振らずに正面へ戻す[mm]。
#: 斜めのまま通ると通れる幅が 1°あたり 3.8mm 減る（`gate_steer.py`）
SQUARE_UP_MM = 250.0

#: 向き保持が、動かした目標に追いつくまでの時間[秒]。振ってよい行の残りの式に使う。
#: 2026-11-03 の記録で、半分追いつくまで中央 0.15秒・長いと 0.66秒
HOLD_CATCHUP_S = 0.7

#: 行が終わるまでに、まっすぐに戻っている余裕[mm]
END_MARGIN_MM = 50.0

#: 速さを出すのに、自己位置をためる時間[秒]。これより短い間しか見ていなければ速さは分からない
SPEED_WINDOW_S = 0.3

#: 格子に対して振る角度の上限[度]。戻しきれずに残っても、次の行の始まりの向きの補正
#: （C++ は 10° を超えると捨てる）で戻せる大きさにする。8° では残りと格子のずれで 10° を超えた
MAX_TILT_DEG = 5.0

#: 出す値の上限[度]。C++ の `HEADING_CORRECT_MAX_DEG`（10°）を超えると丸ごと捨てられるので、手前で止める
MAX_PUBLISH_DEG = 9.5

#: ゲートが見えていないとき、狙ってよい横のずれ[mm]。コードの間隔 250mm の半分より小さくする
UNAMBIGUOUS_MM = 110.0

#: ゲートが見えているとき、コードがゲートの真ん中の列にあるとみなす横の差[mm]
GATE_MATCH_MM = 100.0

#: 出す間隔[秒]。C++ が 3°動かした目標に、向き保持が追いつく前に次を出すと振りすぎる。
#: ほかの流れが出した値との間隔は、出す口（`HeadingTask.publish`）が見る
PUBLISH_INTERVAL = 1.0

def tilt_min_remain_mm(speed_mm_s: float) -> float:
    """
    振ってよい行の残り[mm]。振った分を行の中で戻し切るのに要る距離。

      速さ ×（戻すまでの待ち `PUBLISH_INTERVAL` ＋ 向き保持が追いつく `HOLD_CATCHUP_S`）＋ `END_MARGIN_MM`

    FWD 70（約 307mm/秒）で約 570mm、FWD 80（約 366mm/秒）で約 670mm。
    """
    return float(speed_mm_s) * (PUBLISH_INTERVAL + HOLD_CATCHUP_S) + END_MARGIN_MM


#: 速さをまだ測れていないとき（行に入った直後）、指令の速さ（FWD）から決めるか。
#:
#: 2026-11-04 の点検 X（20:33）では、寄せなかった 229 回のうち 56 回が「速さがまだ分からない」だった
#: （ET_2026_REHEARSAL#94）。行の頭はちょうど床のコードが前に見えている所なので、ここで決めたい。
SPEED_FROM_FWD = True

#: `SPEED_FROM_FWD` を走行体で切り替える置き場（1行目に 1 か 0）
SPEED_FROM_FWD_SWITCH = "./sorot_spike/qr_steer_speed_fwd.txt"


def speed_from_fwd(fwd: float) -> Optional[float]:
    """
    指令の速さ（FWD）から、走る速さ[mm/秒]を見積もる。前へ進まないなら None。

    実機の記録で FWD 70 が約 307mm/秒、80 が約 366mm/秒（`tilt_min_remain_mm` の説明と同じ値）。
    その2点を通る直線で見積もる。測れるようになれば、測った値を使う
    """
    fwd = abs(float(fwd or 0))
    if fwd <= 0:
        return None
    return max(50.0, 307.0 + (fwd - 70.0) * (366.0 - 307.0) / 10.0)


#: 振った分を**行の中で戻しきれなくても**振るか（置き場が無いときの既定）。
#:
#: 経路の多くの行は、振って戻すのに要る距離（FWD 70 で約 570mm）より短く、2026-11-04 の点検 X では
#: 「行の残りが足りない」で 56 回寄せなかった。入れると、振りが効く距離（向き保持が追いつくまで
#: `HOLD_CATCHUP_S` ＋ `END_MARGIN_MM`）が残っていれば振る。戻しきれなかった分は旋回の目標に残るが、
#: 次の直進の行の頭で測る向きの補正（`heading_task`。この行の振りと同じ口）で戻る。
CARRY_TILT = False

#: `CARRY_TILT` を走行体で切り替える置き場（1行目に 1 か 0）
CARRY_TILT_SWITCH = "./sorot_spike/qr_steer_carry.txt"


def tilt_effective_remain_mm(speed_mm_s: float) -> float:
    """振りが効く（向き保持が追いつく）のに要る行の残り[mm]。戻す分は数えない"""
    return float(speed_mm_s) * HOLD_CATCHUP_S + END_MARGIN_MM


#: 床のコードを読む間隔[秒]。ゲートの確認は 0.15秒おきに映像を渡してくるが、毎枚読むと CPU を食う
READ_INTERVAL = 0.3

#: これより小さいずれは出さない[度]。細かく出し続けると、行の始まりの向きの補正と打ち消し合う
MIN_PUBLISH_DEG = 0.5


@dataclass
class Aim:
    """1枚から決めた、出す値と理由"""

    #: 出す値（いまの向き − 向けたい向き）[度]。出さないなら None
    deviation_deg: Optional[float]
    #: 決めた理由（ログ用）
    reason: str
    #: 狙ったコード（前・左[mm]、車軸から）。狙わなかったら None
    target: Optional[tuple] = None
    #: 振る（'tilt'）か戻す（'square'）か
    mode: str = ""


class QrAim:
    """見えたコードと格子のずれから、出す値を決める（走りには触らない。試験しやすいように分けてある）"""

    def decide(self, codes: List[Any], grid_deviation_deg: Optional[float],
               gate_lateral_left_mm: Optional[float] = None,
               remaining_mm: Optional[float] = None,
               speed_mm_s: Optional[float] = None, diagonal: bool = False,
               carry: bool = False) -> Aim:
        """
        :param codes: 見えたコード（`x_mm` 前・`y_mm` 左[mm]、カメラの真下から）
        :param grid_deviation_deg: 格子に対する走行体の向きのずれ[度]（時計回りが正）
        :param gate_lateral_left_mm: 見えているゲートの真ん中の横位置[mm]（左が正）。見えていなければ None
        :param remaining_mm: この行の残りの距離[mm]。分からなければ None（振らずに戻すだけ）
        :param speed_mm_s: いまの速さ[mm/秒]。分からなければ None（振らずに戻すだけ）
        :param diagonal: 格子に対して斜め（45°）に走っているか。斜めなら振らずに戻すだけ
        """
        if grid_deviation_deg is None:
            return Aim(None, "格子に対する向きが測れていない")
        g = float(grid_deviation_deg)

        ahead = []
        for code in codes:
            forward = float(code.x_mm) + CAMERA_TO_AXLE_FORWARD_MM
            if MIN_AHEAD_MM <= forward <= MAX_AHEAD_MM:
                ahead.append((forward, float(code.y_mm)))
        target = self._pick(ahead, gate_lateral_left_mm) if ahead else None

        why_square = ("斜めの行（隣のコードが横に 177mm しか離れず取り違える）" if diagonal
                      else self._why_square(target, remaining_mm, bool(ahead), speed_mm_s, carry))
        if why_square:
            return self._square(g, why_square, target)

        forward, left = target
        # 狙う向き（時計回りが正）を格子から測る。左のコードへは反時計回り＝負
        aim_rel_grid = g - math.degrees(math.atan2(left, forward))
        want_rel_grid = max(-MAX_TILT_DEG, min(MAX_TILT_DEG, aim_rel_grid))
        return Aim(self._clip(g - want_rel_grid), "", target=(round(forward), round(left)), mode="tilt")

    @staticmethod
    def _why_square(target: Optional[tuple], remaining_mm: Optional[float], any_ahead: bool,
                    speed_mm_s: Optional[float] = None, carry: bool = False) -> str:
        """振らずに戻す理由。振ってよければ空文字"""
        if target is None:
            return ("狙うコードを決められない（横 %.0fmm を超え、ゲートで確かめられない）" % UNAMBIGUOUS_MM
                    if any_ahead else "前方 %.0f〜%.0fmm にコードが無い" % (MIN_AHEAD_MM, MAX_AHEAD_MM))
        if remaining_mm is None:
            return "行の残りが分からない"
        if speed_mm_s is None:
            return "速さがまだ分からない"
        # 戻しきれなくても振るときは、振りが効く距離だけあればよい（CARRY_TILT）
        need = tilt_effective_remain_mm(speed_mm_s) if carry else tilt_min_remain_mm(speed_mm_s)
        if remaining_mm < need:
            return "行の残り %.0fmm（戻すには %.0fmm 要る・速さ %.0fmm/秒）" % (remaining_mm, need, speed_mm_s)
        if target[0] <= SQUARE_UP_MM:
            return "コードの手前 %.0fmm" % target[0]
        if target[0] > remaining_mm + SQUARE_UP_MM:
            return "狙うコードがこの行の先（前 %.0fmm・行の残り %.0fmm）" % (target[0], remaining_mm)
        return ""

    def _square(self, g: float, why: str, target: Optional[tuple]) -> Aim:
        """格子に沿った正面へ戻す（出す値＝格子に対する向きのずれ）"""
        return Aim(self._clip(g), why, target=None if target is None else (round(target[0]), round(target[1])),
                   mode="square")

    @staticmethod
    def _clip(deviation: float) -> float:
        return round(max(-MAX_PUBLISH_DEG, min(MAX_PUBLISH_DEG, deviation)), 3)

    @staticmethod
    def _pick(ahead: List[tuple], gate_left: Optional[float]) -> Optional[tuple]:
        """通るべき列のコードを選ぶ。ゲートが見えていればその横位置に、なければ正面に近いもの"""
        if gate_left is not None:
            best = min(ahead, key=lambda c: abs(c[1] - gate_left))
            return best if abs(best[1] - gate_left) <= GATE_MATCH_MM else None
        best = min(ahead, key=lambda c: abs(c[1]))
        return best if abs(best[1]) <= UNAMBIGUOUS_MM else None


class QrSteer:
    """走行中に、ゲートの確認の流れから1枚ずつ呼ばれて、横の補正を出す。走行体で1つだけ使う"""

    _instance: Optional["QrSteer"] = None
    _instance_lock = threading.Lock()

    @classmethod
    def get_instance(cls) -> "QrSteer":
        with cls._instance_lock:
            if cls._instance is None:
                cls._instance = cls()
            return cls._instance

    @classmethod
    def reset_instance(cls) -> None:
        """共有している1つを捨てる（テスト用）"""
        with cls._instance_lock:
            cls._instance = None

    def __init__(self, reader: Any = None, publish: Optional[Callable[..., Any]] = None,
                 pose_source: Optional[Callable[[], Any]] = None,
                 rows: Any = None, clock: Callable[[], float] = time.monotonic,
                 route_row: Optional[Callable[[Tuple[int, int]], Optional[dict]]] = None,
                 speed: Optional[Callable[[], Optional[float]]] = None):
        self._frame = None
        self._reader = reader
        self._publish = publish
        self._pose_source = pose_source
        self._rows = rows
        self._clock = clock
        self._last_publish = -1e9
        self._last_read = -1e9
        self._route_row = route_row
        self._told = set()
        #: いまの行 (SNO, CNO) と、その行に入ったときの走行距離[mm]（行の残りを出すため）
        self._row_key: Optional[Tuple[int, int]] = None
        self._row_start_dist = 0.0
        #: いまの行で見た（時刻, 走行距離）。速さを出すため
        self._samples: List[Tuple[float, float]] = []
        self._speed = speed

    def enabled(self) -> bool:
        return SwitchFile.read(ENABLED_SWITCH, ENABLED)

    def consider(self, frame: Any, gate: Any = None) -> Optional[float]:
        """
        1枚見て、出すべきなら出す。

        :param frame: ゲートの確認が取った映像
        :param gate: その映像でいちばん近いゲート（`GateSighting`）。無ければ None
        :returns: 出した値[度]。出さなければ None
        """
        if frame is None or not self.enabled():
            return None
        pose = self._pose()
        if pose is None:
            return None
        travelled = self._travelled(pose)
        row = self._row_at(pose.sno, pose.cno)
        if not self._steerable(row, pose.sno, pose.cno):
            return None
        now = self._clock()
        if now - self._last_publish < PUBLISH_INTERVAL or now - self._last_read < READ_INTERVAL:
            return None
        self._last_read = now
        captured_at = time.monotonic()

        reader = self._reader_for()
        self._frame = frame
        result = reader.read()
        found = getattr(reader, "last_found", None)
        if not result.get("ok") or found is None:
            return None
        if getattr(found, "frame", "robot") != "robot":
            self._tell_once("frame", "床のコードの位置が走行体から見た座標でないので、横の補正はしません")
            return None

        gate_left = self._gate_left(gate)
        aim = QrAim().decide(found.codes, result.get("deviation_deg"), gate_left,
                             remaining_mm=self._row_remaining(row, travelled),
                             speed_mm_s=self._speed_now(row),
                             diagonal=(result.get("system") == 45),
                             carry=self._carry_tilt())
        if aim.deviation_deg is None:
            logger.debug("横の補正なし（%d/%d）: %s", pose.sno, pose.cno, aim.reason)
            return None
        if abs(aim.deviation_deg) < MIN_PUBLISH_DEG:
            return None
        if self._publish_now(aim.deviation_deg, captured_at) is None:
            return None
        self._last_publish = self._clock()
        self._photo(frame, aim.deviation_deg)
        if aim.mode == "tilt":
            logger.info("横の補正 %+.2f°（%d/%d・振る・コード 前%.0f 左%+.0fmm・格子のずれ %+.2f°%s）",
                        aim.deviation_deg, pose.sno, pose.cno, aim.target[0], aim.target[1],
                        result.get("deviation_deg"), "・ゲートで確かめた" if gate_left is not None else "")
        else:
            logger.info("横の補正 %+.2f°（%d/%d・戻す・%s）", aim.deviation_deg, pose.sno, pose.cno, aim.reason)
        return aim.deviation_deg

    # -- まわり ---------------------------------------------------------------

    def _travelled(self, pose: Any) -> float:
        """いまの行に入ってから進んだ距離[mm]。行が変わったら数え直す（速さの記録も）"""
        key = (pose.sno, pose.cno)
        dist = float(getattr(pose, "dist", 0.0) or 0.0)
        now = self._clock()
        if key != self._row_key:
            self._row_key = key
            self._row_start_dist = dist
            self._samples = []
        self._samples.append((now, dist))
        self._samples = [(t, d) for t, d in self._samples if now - t <= 3 * SPEED_WINDOW_S]
        return dist - self._row_start_dist

    def _speed_now(self, row: Optional[dict]) -> Optional[float]:
        """いまの速さ[mm/秒]。測れていなければ、指令の速さ（FWD）から見積もる（`SPEED_FROM_FWD`）"""
        if self._speed is not None:
            return self._speed()
        measured = self._measured_speed()
        if measured is not None or row is None or not SwitchFile.read(SPEED_FROM_FWD_SWITCH, SPEED_FROM_FWD):
            return measured
        return speed_from_fwd((row.get("RunInfo") or {}).get("FWD", 0))

    @staticmethod
    def _carry_tilt() -> bool:
        return bool(SwitchFile.read(CARRY_TILT_SWITCH, CARRY_TILT))

    def _measured_speed(self) -> Optional[float]:
        """いまの行の、直近 `SPEED_WINDOW_S` 秒以上の自己位置から出した速さ[mm/秒]。足りなければ None"""
        if not self._samples:
            return None
        t1, d1 = self._samples[-1]
        older = [(t, d) for t, d in self._samples if t1 - t >= SPEED_WINDOW_S]
        if not older:
            return None
        t0, d0 = older[-1]
        return abs(d1 - d0) / (t1 - t0)

    @staticmethod
    def _row_remaining(row: Optional[dict], travelled: float) -> Optional[float]:
        """行の残りの距離[mm]。距離で終わる行でなければ None"""
        if row is None:
            return None
        sw = row.get("SwitchInfo") or {}
        if not (int(sw.get("SCJFN", 0) or 0) & SCJFN_DISTANCE):
            return None
        return abs(float(sw.get("SCD", 0) or 0)) - travelled

    def _steerable(self, row: Optional[dict], sno: int, cno: int) -> bool:
        """前へ進む直進で、SCRIPT のある行（C++ が目標の控えを取る行）か"""
        if row is None:
            self._tell_once(("norow", sno, cno),
                            "横の補正を出しません（%d/%d の行の中身が分からない。経路の番号をまだ学んでいない）"
                            % (sno, cno))
            return False
        from scenario_rows import keeps_heading
        run = row.get("RunInfo") or {}
        if not keeps_heading(row) or not (run.get("FWD", 0) or 0) > 0:
            return False
        if not str(run.get("SCRIPT", "")).strip():
            self._tell_once(("noscript", sno, cno),
                            "横の補正を出しません（%d/%d に SCRIPT が無く、C++ が受け取らない）" % (sno, cno))
            return False
        return True

    @staticmethod
    def _gate_left(gate: Any) -> Optional[float]:
        """ゲートの真ん中の横位置（左が正）。並んで見えている・斜めから見ているときは使わない"""
        if gate is None or getattr(gate, "offset_mm", None) is None:
            return None
        approach = getattr(gate, "approach_deg", None)
        if approach is None or not math.isfinite(approach) or abs(approach) > GATE_MAX_APPROACH_DEG:
            return None
        lateral = getattr(gate, "lateral_mm", None)
        if lateral is None or not math.isfinite(lateral):
            return None
        return -float(lateral)          # gate_finder は右が正

    def _reader_for(self) -> Any:
        if self._reader is None:
            from floor_code_reader import FloorCodeReader
            self._reader = FloorCodeReader(frame_source=lambda: self._frame)
        else:
            self._reader.frame_source = lambda: self._frame
        return self._reader

    def _pose(self) -> Any:
        if self._pose_source is not None:
            return self._pose_source()
        from pose import PoseReader
        return PoseReader.read_fresh()

    def _row_at(self, sno: int, cno: int) -> Optional[dict]:
        """いまの行。経路を書き直していれば、書き直した後の行（SCD が変わる）"""
        rewritten = self._rewritten_row(sno, cno)
        if rewritten is not None:
            return rewritten
        rows = self._rows
        if rows is None:
            from scenario_rows import ScenarioRows
            rows = ScenarioRows.shared()
        return rows.row_at(sno, cno)

    def _rewritten_row(self, sno: int, cno: int) -> Optional[dict]:
        if self._route_row is not None:
            return self._route_row((sno, cno))
        from route_follower import RouteFollower
        return RouteFollower.get_instance().row_now((sno, cno))

    def _publish_now(self, deviation_deg: float, captured_at: float) -> Optional[int]:
        """出す。出す口が捨てたら None（ほかの流れが出した直後）"""
        if self._publish is not None:
            return self._publish(deviation_deg, source="qr_steer", markers=0, elapsed_ms=0.0,
                                 captured_at=captured_at)
        from heading_task import HeadingTask
        return HeadingTask.get_instance().publish(deviation_deg, source="qr_steer", markers=0,
                                                  elapsed_ms=0.0, captured_at=captured_at)

    def _photo(self, frame: Any, deviation_deg: float) -> None:
        """当てた前後の写真を残す（`correction_photo`）"""
        if self._publish is not None:          # テストで出す先を差し替えているときは残さない
            return
        from correction_photo import CorrectionPhoto
        CorrectionPhoto.shared().record(frame, "qr_steer", deviation_deg)

    def _tell_once(self, key: Any, message: str) -> None:
        if key not in self._told:
            self._told.add(key)
            logger.info("%s", message)
