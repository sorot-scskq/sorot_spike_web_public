"""
ET相撲（2026）の力士寄せ。走行体上の Python で、リモートコントロール走行の区間を走らせる。

土俵の力士ボトルへ、前面カメラで見ながら寄る。押し出せる近さと向きに来たら
Status.OK を返し、押し出しはシナリオの次のコマンドに任せる。

    撮る   … 止まって少し待ってから、力士ボトルがどこに見えるかを読む
    寄る   … 残りの距離の半分だけ寄る（1回 MAX_STEP_MM まで。APPROACH_STOP_AT_MM で止まる）
    首振り … 見えなければ 左 → 右 → 中央 と振って撮り直す。一巡しても見えなければ少し前へ出る
    向き直り … 近く（APPROACH_DONE_MM 以内）でずれていれば、前へ出ずに向きだけ直す
    OK     … 近くで正面（ALIGN_DEG 以内）に見えた

**掴んだかどうかは見ない。** アームが触れたかを確かめる手段は実機にも無い。

【シナリオの書き方】
    { "SNO": 40, "CNO": 1, "RunInfo": { "FUNCNO": 18, "FWD": 40, "TRN": 40, "NOBLNCE": 1 },
      "SwitchInfo": { "SCJFN": 2048 } }
FWD は寄るときの速さ、TRN は首振りの強さ。この区間の OK で次（押し出し）へ進む。

【差し方】
走行体では ev3_python の起動後に attach() を呼ぶ（config_2026 から呼ぶ想定）。
リモート側の制御関数は走行体に1つだけ差せる（remote_control.py）。ETラリーでも
リモートコントロールを使うなら、sno を見て振り分ける親の制御関数を作ること。
この制御関数は、担当の SNO 以外の区間では指令を出さない（None を返す）。

シミュレータでは sim/pyscript/et_sumo_bridge.py が同じファイルを読み込み、映像の入り口と
「後で呼ぶ」口だけを差し替えて attach() する。

【カメラ】
実機は 1280x720 を連続で取り込む（camera.py の RobotCamera.shared）。そのまま探すと重いので、
横 PROCESS_WIDTH に縮めてから探す。シミュレータの映像も同じ幅に縮めるので、しきい値
（塊とみなす画素数など）は実機とシミュレータで同じものを使える。

地面の写し方（CameraGeometry）はカメラの取り付けで決まる。**実機の値はまだ測っていない。**
SIM_CAMERA はシミュレータの前面カメラ（sim/camera-model.js）の値。
"""

from __future__ import annotations

import logging
import math
import threading
from dataclasses import dataclass
from typing import Any, Callable, Optional

from bottle_locator import BottleLocator
from remote_control import Command, Observation, Status, set_controller

logger = logging.getLogger(__name__)

# --- 段取りのしきい値 --------------------------------------------------------
#: 担当する SNO（シナリオ番号の設計で ET相撲 は 40）
SUMO_SNO = 40
#: これより近くで正面に見えたら寄せ終わり。近すぎると足元が画面の下へ外れて写らない
#: （シミュレータの前面カメラは画面の一番下でも 219mm 先）。写る範囲の内側に置く
APPROACH_DONE_MM = 280.0
#: 寄るときはここで止まるように詰める。半分ずつ寄ると写らない近さまで入ってしまう
APPROACH_STOP_AT_MM = 250.0
#: 寄せ終わりとみなす向きのずれ[度]（車軸から見て）。250mm 先で 5度なら横に約 22mm
ALIGN_DEG = 5.0
#: 近くで向きだけ直すときの旋回量。首振りと同じ強さだと行き過ぎる
ALIGN_TURN = 20.0
#: 1回に寄る上限[mm]
MAX_STEP_MM = 150.0
#: 見つからないときに前へ出る距離[mm]
SEEK_MM = 50.0
#: 止まってから撮るまで待つ時間[s]。止まりきる前に撮るとぶれた映像になる
SETTLE_SEC = 0.2
#: 見つからないときの首振り[度]。左 → 右端 → 中央 の順
SCAN_SIDE_DEG = 35.0
SCAN_SEQUENCE_DEG = (SCAN_SIDE_DEG, -2.0 * SCAN_SIDE_DEG, SCAN_SIDE_DEG)

# --- カメラ ------------------------------------------------------------------
#: 探す前に縮める横幅[px]。実機 1280x720 → 320x180
PROCESS_WIDTH = 320
#: 塊とみなす画素数。bottle_locator の既定（40）は 240x144 の映像で決めた値なので、
#: 面積の比で合わせる
BASE_MIN_PIXELS = 40
BASE_AREA = 240 * 144


@dataclass(frozen=True)
class CameraGeometry:
    """前面カメラの取り付け。画面の位置を、走行体から見た距離・左右のずれに直すのに使う"""

    #: 横・縦の画角[度]
    hfov_deg: float
    vfov_deg: float
    #: レンズの高さ[mm]
    height_mm: float
    #: 下向きの傾き[度]
    tilt_deg: float
    #: レンズが車軸（その場旋回の中心）より前にある距離[mm]
    forward_mm: float
    #: これより遠い地面は見えないものとして扱う[mm]
    far_mm: float = 5000.0


#: シミュレータの前面カメラ（sim/camera-model.js。水平 70度・垂直 36度、高さ 34px・前後 18px を 0.2px/mm で直した値）
SIM_CAMERA = CameraGeometry(hfov_deg=70.0, vfov_deg=36.0, height_mm=170.0, tilt_deg=20.0,
                            forward_mm=90.0)
#: 実機の前面カメラ。**まだ測っていないので、シミュレータの値を仮に置いている**
ROBOT_CAMERA = SIM_CAMERA


def ground_projection(geometry: CameraGeometry) -> Callable[[float, float, int, int], Optional[dict]]:
    """画面の (u, v) を、カメラから見た距離・左右のずれ[mm]に直す関数を作る"""
    tan_half_h = math.tan(math.radians(geometry.hfov_deg) / 2.0)
    tan_half_v = math.tan(math.radians(geometry.vfov_deg) / 2.0)

    def project(u: float, v: float, width: int, height: int) -> Optional[dict]:
        if width <= 0 or height <= 0:
            return None
        focal = (height / 2.0) / tan_half_v
        down = math.radians(geometry.tilt_deg) + math.atan((v - height / 2.0) / focal)
        distance = geometry.far_mm if down <= 1e-4 else min(geometry.far_mm, geometry.height_mm / math.tan(down))
        half_lateral = distance * tan_half_h
        lateral = ((u + 0.5) / width - 0.5) * 2.0 * half_lateral
        return {"distance_mm": distance, "lateral_mm": lateral}

    return project


def pivot_deg(distance_mm: float, bearing_deg: float, forward_mm: float) -> float:
    """
    カメラから見た向きを、車軸から見た向きに直す。

    カメラは車軸より前にあるので、近い物ほどカメラから見た角度が大きく出る。
    その角度のまま回ると回りすぎて、左右に振れ続ける。
    """
    b = math.radians(bearing_deg)
    return math.degrees(math.atan2(distance_mm * math.sin(b), distance_mm * math.cos(b) + forward_mm))


def _wrap_pi(rad: float) -> float:
    return (rad + math.pi) % (2.0 * math.pi) - math.pi


def shrink_for_processing(frame: Any) -> Any:
    """映像を横 PROCESS_WIDTH に縮める（縦横比は保つ）。縮める必要が無ければそのまま"""
    if frame is None:
        return None
    import cv2  # 使うときだけ読む（RoughSpot/README.md の決まり）
    height, width = frame.shape[:2]
    if width <= PROCESS_WIDTH:
        return frame
    size = (PROCESS_WIDTH, max(1, round(height * PROCESS_WIDTH / width)))
    return cv2.resize(frame, size, interpolation=cv2.INTER_AREA)


def make_locator(frame_source: Callable[[], Any], geometry: CameraGeometry) -> BottleLocator:
    """力士ボトルの探索器。黒ラベルでキャリーボトル（赤・青・黄）と見分ける"""
    ratio = (PROCESS_WIDTH * PROCESS_WIDTH * 9 / 16) / BASE_AREA
    locator = BottleLocator({"min_label_ratio": 0.05,
                             "min_pixels": max(1, round(BASE_MIN_PIXELS * ratio))})
    locator.set_frame_source(lambda: shrink_for_processing(frame_source()))
    locator.set_ground_projection(ground_projection(geometry))
    return locator


def _robot_frame_source() -> Callable[[], Any]:
    """実機の前面カメラ（連続取り込みの最新の1枚）"""
    from camera import RobotCamera
    return lambda: RobotCamera.shared().latest_frame()


def _run_later_with_thread(fn: Callable[[], None], delay_sec: float) -> None:
    """実機: 別スレッドで待ってから呼ぶ。制御関数（5ms ごと）を待たせない"""
    timer = threading.Timer(delay_sec, fn)
    timer.daemon = True
    timer.start()


class SumoApproach:
    """
    力士寄せの制御関数（remote_control の set_controller に差す）。

    撮影と位置の読み取りは重いので、制御関数の中ではやらない。run_later で後から
    呼んでもらい、制御関数は「もう読めたか」を見るだけにする。
    """

    def __init__(self, locator: BottleLocator, geometry: CameraGeometry,
                 run_later: Callable[[Callable[[], None], float], None] = _run_later_with_thread,
                 sno: int = SUMO_SNO):
        self.locator = locator
        self.geometry = geometry
        self.run_later = run_later
        self.sno = sno
        self._lock = threading.Lock()
        self._seen: Optional[dict] = None
        self._shooting = False
        self._shot_done = False
        self._state = "shoot"
        self._cmd_seq: Optional[int] = None
        self._tick = 0
        self._start_distance = 0.0
        self._step_mm = 0.0
        self._bearing_deg = 0.0
        self._start_direction = 0.0
        self._target_rad = 0.0
        self._turn = 0.0
        self._scan_index = 0

    # --- 撮影（run_later から呼ばれる）---------------------------------------

    def _shoot(self) -> None:
        pose = self.locator.read()
        seen = None
        if pose.get("found"):
            seen = {"distance_mm": float(pose["distance_mm"]),
                    "bearing_deg": math.degrees(float(pose["bearing_rad"]))}
        with self._lock:
            self._seen = seen
            self._shot_done = True
            self._shooting = False

    # --- 状態の移り変わり ------------------------------------------------------

    def _start_turn(self, obs: Observation, target_deg: float, turn: float) -> Command:
        self._state = "turn"
        self._start_direction = float(obs.coordinate.get("direction", 0.0))
        self._target_rad = math.radians(target_deg)
        self._turn = abs(turn)
        return Command(0, 0, Status.NG)

    def _start_scan_or_seek(self, obs: Observation, turn_gain: float) -> Command:
        if self._scan_index < len(SCAN_SEQUENCE_DEG):
            deg = SCAN_SEQUENCE_DEG[self._scan_index]
            self._scan_index += 1
            logger.info("力士ボトルが見えない → 首振り %+g度 (%d/%d)", deg, self._scan_index, len(SCAN_SEQUENCE_DEG))
            return self._start_turn(obs, deg, turn_gain)
        self._scan_index = 0
        self._step_mm = SEEK_MM
        self._start_distance = float(obs.coordinate.get("distance", 0.0))
        self._bearing_deg = 0.0
        self._state = "move"
        logger.info("首振りでも見えない → %gmm 前へ出る", SEEK_MM)
        return Command(0, 0, Status.NG)

    def __call__(self, obs: Observation) -> Optional[Command]:
        if obs.sno != self.sno:
            return None     # 担当の区間ではない

        # 区間に入り直したら、撮影から始める。通し番号が同じでも、区間の中の周期番号（tick）が
        # 戻っていれば入り直している（走行体を起動し直すと、通し番号はまた同じ値から振られる）
        tick = int(obs.tick or 0)
        restarted = tick < self._tick
        self._tick = tick
        if obs.cmd_seq != self._cmd_seq or restarted:
            self._cmd_seq = obs.cmd_seq
            self._state = "shoot"
            self._scan_index = 0
            with self._lock:
                self._seen = None
                self._shooting = False
                self._shot_done = False

        fwd = float(obs.run_info.get("FWD") or 40.0)
        turn_gain = abs(float(obs.run_info.get("TRN") or 40.0))
        distance_now = float(obs.coordinate.get("distance", 0.0))

        if self._state == "shoot":
            with self._lock:
                done = self._shot_done
                ask = not done and not self._shooting
                if ask:
                    self._shooting = True
                if done:
                    self._shot_done = False
                    seen, self._seen = self._seen, None
            if not done:
                # 頼むのはロックの外で。すぐ呼ばれる run_later だと、_shoot が同じロックで止まる
                if ask:
                    self.run_later(self._shoot, SETTLE_SEC)
                return Command(0, 0, Status.NG)     # 止まったまま待つ

            if seen is None:
                return self._start_scan_or_seek(obs, turn_gain)

            self._scan_index = 0
            distance = seen["distance_mm"]
            bearing = seen["bearing_deg"]
            if distance <= APPROACH_DONE_MM:
                pivot = pivot_deg(distance, bearing, self.geometry.forward_mm)
                if abs(pivot) <= ALIGN_DEG:
                    logger.info("力士ボトルが近くで正面に見えた（%.0fmm, %+.1f度）→ OK", distance, pivot)
                    return Command(0, 0, Status.OK)
                logger.info("近い（%.0fmm）→ 向きだけ直す %+.1f度", distance, pivot)
                return self._start_turn(obs, pivot, min(turn_gain, ALIGN_TURN))

            self._step_mm = min(distance / 2.0, MAX_STEP_MM, distance - APPROACH_STOP_AT_MM)
            self._start_distance = distance_now
            self._bearing_deg = bearing
            self._state = "move"
            return Command(0, 0, Status.NG)

        if self._state == "turn":
            progressed = _wrap_pi(float(obs.coordinate.get("direction", 0.0)) - self._start_direction)
            done = progressed >= self._target_rad if self._target_rad >= 0.0 else progressed <= self._target_rad
            if done:
                self._state = "shoot"
                return Command(0, 0, Status.NG)
            return Command(0.0, self._turn if self._target_rad >= 0.0 else -self._turn, Status.NG)

        # move: 寄る／探して前へ出る
        if distance_now - self._start_distance >= self._step_mm:
            self._state = "shoot"
            return Command(0, 0, Status.NG)
        turn = max(-turn_gain, min(turn_gain, self._bearing_deg * 2.0))
        return Command(fwd, turn, Status.NG)


def attach(frame_source: Optional[Callable[[], Any]] = None,
           geometry: CameraGeometry = ROBOT_CAMERA,
           run_later: Callable[[Callable[[], None], float], None] = _run_later_with_thread,
           sno: int = SUMO_SNO) -> SumoApproach:
    """
    力士寄せを remote_control に差す。差した制御関数を返す。

    :param frame_source: 映像（BGR の numpy 配列）を返す関数。省略すると実機の前面カメラ
    :param geometry: カメラの取り付け
    :param run_later: fn を delay_sec 秒後に呼ぶ関数。省略すると別スレッド
    :param sno: 担当する SNO
    """
    source = frame_source or _robot_frame_source()
    control = SumoApproach(make_locator(source, geometry), geometry, run_later, sno)
    set_controller(control)
    logger.info("ET相撲の力士寄せを差しました（SNO %d）", sno)
    return control
