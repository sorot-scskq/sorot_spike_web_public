"""
ET相撲（2026）の力士寄せ。緑の場所を避ける版。

et_sumo.py の SumoApproach を継承し、段取り（撮る → 寄る → 近くで正面なら OK）はそのまま使う。
足すのは「緑を避ける」ことだけ。コースの緑の場所には障害物があり、力士ボトルを押し込むと
倒れるおそれがあり、走行体も当たるおそれがある。

    押し出し     … 近くで正面に見えたら、シナリオに任せず自分で押し出してから OK を出す。
                   押す長さは、押し出す道筋（ボトルの向こう側）に写った緑の手前までに抑える。
                   土俵の近くは緑が迫っていて（中心から上 215mm・左 510mm）、決まった長さを押すと
                   緑に入るため。緑の手前までで土俵を出るのに足りなければ、ボトルのまわりを
                   少し回り込んで向きを変え、見直す（緑の多い側と反対へ振れるように回る）。
    前へ出る     … 走行体が緑の縁をかすめるのは許す（アームの先が 40mm 入るまで。帯は中心の ±45mm）。
                   寄るときは、その場で向きを合わせてからまっすぐ前へ出る（曲がりながら寄ると、
                   まっすぐ前の帯しか見ていない緑の判断をすり抜けて緑の角を横切るため）。
                   行く手に緑が写っていれば、その手前で止める。止める余地が無ければ、
                   緑の少ない側へ向きを変える。

判断は前面カメラの映像だけで行う（コースの地図は持たない）。実機でもそのまま使える。

【見えないところ】
カメラは画面の一番下でも 219mm 先（車軸から 309mm 先）までしか写らない。アームの先（125mm）との
あいだは見えないので、そこへ入る緑は、前に撮ったときに見えていた分でしか避けられない。
押し出したあとのバック（シナリオの次のコマンド）は、押し出しで通った道を戻るだけなので見ない。

【シナリオ】
押し出しはこの中で行うので、シナリオには押し出しを置かない（13_ET相撲単体 / 99_Lコース の
40/1 力士寄せ → 40/2 バック → 40/3 旋回）。et_sumo.py（押し出さない版）を使うときは、
シナリオに押し出しを戻すこと。

【差し方】
et_sumo.py と同じ。attach() を呼ぶ（シミュレータは sim/pyscript/et_sumo_bridge.py が呼ぶ）。
"""

from __future__ import annotations

import logging
import math
from typing import Any, Callable, Optional

import et_sumo
from et_sumo import (
    ROBOT_CAMERA,
    SETTLE_SEC,
    SUMO_SNO,
    CameraGeometry,
    SumoApproach,
    _robot_frame_source,
    _run_later_with_thread,
    _wrap_pi,
    make_locator,
)
from remote_control import Command, Observation, Status, set_controller

logger = logging.getLogger(__name__)

# --- 緑の見分け方 -------------------------------------------------------------
#: 緑とみなす色の範囲（OpenCV の HSV。H は 0〜180）。コースの緑は RGB(20,158,78) 前後
GREEN_HSV_LOW = (35, 80, 60)
GREEN_HSV_HIGH = (90, 255, 255)
#: 緑があるとみなす、1行あたりの緑の画素数（ノイズを拾わない）
GREEN_ROW_PIXELS = 2

# --- 走行体の大きさ -----------------------------------------------------------
#: アームの先端（車軸から前）[mm]
ARM_TIP_MM = 125.0
#: 前へ出るときに見る帯の半幅[mm]。
#: 走行体が緑の縁をかすめるのは許す（力士ボトルは緑に入れない）。車体の半幅（74mm）ではなく、
#: 中心付近だけを見る。車体の端が緑に重なる向きも選べる
BAND_HALF_WIDTH_MM = 45.0
#: 押し出したボトルが通る道筋の半幅[mm]（ボトルの半径 33 + 余裕）。ここに緑が写れば押さない
PUSH_PATH_HALF_WIDTH_MM = 100.0
#: 押し出す道筋のまわりとして見る帯の半幅[mm]。
#: 押し出す直前は力士ボトルが手前（250mm）にあり、その奥の真ん中はボトルに隠れて写らない
#: （900mm 先では左右 ±90mm ほどが隠れる）。隠れた所を横切る緑は、道筋の外の左右の両側に
#: 同じ遠さで写るので、そこまで広げて見る
PUSH_HALF_WIDTH_MM = 250.0
#: 行く手の緑に、アームの先がこれだけ入るところまでは進んでよい[mm]。
#: 負の余裕として使う（緑の縁をかすめるのは許す）。力士ボトルの押し出しには使わない
STOP_MARGIN_MM = -40.0

# --- 押し出し ------------------------------------------------------------------
#: 押している力士ボトルの中心は、走行体の車軸よりこれだけ前にある[mm]（車体の前面 75 + ボトルの半径 33）
BOTTLE_CENTER_AHEAD_MM = 75.0 + 33.0
#: ボトルの半径[mm]
BOTTLE_RADIUS_MM = 33.0
#: 押し出したボトルの縁と緑とのあいだに残す余裕[mm]
PUSH_GREEN_MARGIN_MM = 60.0
#: ボトルを動かしたい距離[mm]。土俵（半径 187mm）の中のたいていの場所から、外へ出せる長さ
TARGET_BOTTLE_TRAVEL_MM = 500.0
#: ボトルを最低これだけは動かす[mm]。緑の手前までで、これに届かない向きには押さない（回り込む）
MIN_BOTTLE_TRAVEL_MM = 350.0
#: 押す長さの上限（車軸が進む距離）[mm]
PUSH_MAX_MM = 800.0
#: 押すときの速さ
PUSH_FWD = 40.0
#: 押し出す道筋として緑を見る遠さ（車軸から）[mm]
PUSH_REACH_MM = PUSH_MAX_MM + BOTTLE_CENTER_AHEAD_MM + BOTTLE_RADIUS_MM + PUSH_GREEN_MARGIN_MM
#: 行く手を見る遠さ[mm]（カメラから）
LOOK_AHEAD_MM = 1500.0

# --- 回り込み ------------------------------------------------------------------
#: 1回に回り込む角度[度]。押し出す向きがこれだけ変わる。
#: 大きいと横へ出る距離が長くなり、土俵の上の緑に迫られてすぐふさがる
ORBIT_DEG = 15.0
#: 回り込む回数の上限。これを超えたら、緑の手前まで押せるだけ押す（止まり続けるよりよい）
MAX_ORBITS = 4
#: 横へ出る先に緑があるとき、この長さ以上を横へ出られるなら、緑の手前まで短く出る[mm]
MIN_SIDESTEP_MM = 40.0
#: 行く手の緑を避けて向きを変えたあと、その向きのまま進む長さ[mm]。
#: 向きを変えただけで探し直すと、またボトルのほうを向いて同じ緑に阻まれ、回り続ける
DETOUR_MM = 150.0
#: 写るいちばん手前がもう緑のとき、緑はそれよりこれだけ手前から始まるとみなす[mm]
BLIND_GUESS_MM = 60.0
#: 見えずに首振りを始めるとき、手前が緑なら先に下がる距離[mm]
ESCAPE_BACK_MM = 120.0
#: 行く手の緑を避けて向きを変える 1回の角度[度]。狭い通路では大きく変えると反対側の緑に阻まれる
DETOUR_TURN_DEG = 15.0
#: 向きを変えて試す回数の上限（DETOUR_TURN_DEG × これ で、最大 90度まで振る）
DETOUR_TRIES = 6
#: 回り込むときの速さ
ORBIT_FWD = 60.0
#: 回り込み・迂回で向きを変えるときの旋回の強さ（シナリオの TRN より強くして時間を縮める）
ORBIT_TURN = 60.0
#: 回り込む前に下がる距離[mm]。横へ出る道筋を、ボトルの向こうに迫る緑から離すため。
#: 下がる道は寄ってきた道なので、見なくても緑は無い
ORBIT_BACK_MM = 100.0
#: 寄る前に、その場で向きを合わせる角度のしきい値[度]（車軸から見て）
STRAIGHT_ALIGN_DEG = 4.0
#: 写る地面のいちばん手前と、いちばん近い緑との差がこれより小さければ、見えない手前にも緑があるとみなす
BLIND_MARGIN_MM = 15.0


def _hsv_green_mask(frame: Any):
    import cv2  # 使うときだけ読む（RoughSpot/README.md の決まり）
    import numpy as np
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    return cv2.inRange(hsv, np.array(GREEN_HSV_LOW, dtype=np.uint8),
                       np.array(GREEN_HSV_HIGH, dtype=np.uint8)) > 0


def _row_ground(geometry: CameraGeometry, height: int):
    """行ごとの (カメラからの距離[mm], その距離での左右の半幅[mm])。地面と交わらない行は距離 None"""
    tan_half_h = math.tan(math.radians(geometry.hfov_deg) / 2.0)
    focal = (height / 2.0) / math.tan(math.radians(geometry.vfov_deg) / 2.0)
    rows = []
    for v in range(height):
        down = math.radians(geometry.tilt_deg) + math.atan((v + 0.5 - height / 2.0) / focal)
        if down <= 1e-4:
            rows.append((None, 0.0))
            continue
        distance = geometry.height_mm / math.tan(down)
        rows.append((distance, distance * tan_half_h))
    return rows


def analyze_green(frame: Any, geometry: CameraGeometry, push_far_mm: float,
                  half_width_mm: float = BAND_HALF_WIDTH_MM,
                  push_half_width_mm: float = PUSH_HALF_WIDTH_MM,
                  push_path_half_width_mm: float = PUSH_PATH_HALF_WIDTH_MM,
                  look_ahead_mm: float = LOOK_AHEAD_MM) -> Optional[dict]:
    """
    映像から、走行体の正面にどこまで緑が無いかを見る。距離はすべて車軸から[mm]。

    **割合では見ない。** 遠くの地面は画面の数行にしか写らないので、割合にすると遠くの緑は
    ほぼ 0 になって見落とす。「その帯に緑が 1行でも写ったいちばん近い距離」を返す。

    :param push_far_mm: 押し出す道筋として見る遠さ（車軸から）
    :returns:
        ahead_mm          前へ出る帯（±half_width_mm）でいちばん近い緑。無ければ None
        push_green_mm     押し出す道筋をふさぐ、いちばん近い緑。無ければ None。
                          道筋（±push_path_half_width_mm）に緑が写った行か、道筋の外の左右の両側に
                          緑が写った行（隠れた真ん中を横切っている）をふさいでいるとみなす。
                          片側だけの緑（道筋と平行な縁など）ではふさがない
        push_left/right   押し出す帯の左半分・右半分の緑の画素数
        push_left_mm/right_mm  押し出す帯の左半分・右半分で、いちばん近い緑。無ければ None
        visible_from_mm   写る地面のいちばん手前
        blind_green       写るいちばん手前がもう緑（その手前の見えない所にも緑があるかもしれない）
    """
    if frame is None:
        return None
    height, width = frame.shape[:2]
    green = _hsv_green_mask(frame)
    rows = _row_ground(geometry, height)
    center = width / 2.0
    forward = geometry.forward_mm

    def band(v: int, half_mm: float, half_lat: float):
        half_px = half_mm / (2.0 * half_lat) * width
        lo = max(0, int(round(center - half_px)))
        hi = min(width, int(round(center + half_px)))
        return lo, hi

    ahead_mm = push_green_mm = visible_from = None
    left_mm = right_mm = None
    left = right = 0
    for v in range(height - 1, -1, -1):          # 手前（画面の下）から
        distance, half_lat = rows[v]
        if distance is None or half_lat <= 0:
            continue
        axle = distance + forward
        if visible_from is None:
            visible_from = axle
        if distance <= look_ahead_mm and ahead_mm is None:
            lo, hi = band(v, half_width_mm, half_lat)
            if hi > lo and int(green[v, lo:hi].sum()) >= GREEN_ROW_PIXELS:
                ahead_mm = axle
        if axle <= push_far_mm:
            lo, hi = band(v, push_half_width_mm, half_lat)
            if hi > lo:
                row = green[v, lo:hi]
                mid = max(0, min(row.size, int(round(center)) - lo))
                row_left = int(row[:mid].sum())
                row_right = int(row[mid:].sum())
                left += row_left
                right += row_right
                if push_green_mm is None:
                    plo, phi = band(v, push_path_half_width_mm, half_lat)
                    in_path = int(green[v, plo:phi].sum()) if phi > plo else 0
                    outer_left = int(green[v, lo:max(lo, plo)].sum())
                    outer_right = int(green[v, min(hi, phi):hi].sum())
                    if in_path >= GREEN_ROW_PIXELS or (outer_left >= GREEN_ROW_PIXELS and outer_right >= GREEN_ROW_PIXELS):
                        push_green_mm = axle
                if left_mm is None and row_left >= GREEN_ROW_PIXELS:
                    left_mm = axle
                if right_mm is None and row_right >= GREEN_ROW_PIXELS:
                    right_mm = axle

    return {
        "ahead_mm": ahead_mm,
        "push_green_mm": push_green_mm,
        "push_left": left,
        "push_right": right,
        "push_left_mm": left_mm,
        "push_right_mm": right_mm,
        "visible_from_mm": visible_from,
        "blind_green": (ahead_mm is not None and visible_from is not None
                        and ahead_mm - visible_from <= BLIND_MARGIN_MM),
    }


class SumoApproachAvoidGreen(SumoApproach):
    """
    緑を避ける力士寄せ。SumoApproach の段取りに、押し出す向きの見直しと、行く手の緑の手前で
    止めることを足す。

    :param green_source: 撮った映像から analyze_green の結果を返す関数（テストで差し替える）。
                         省略すると、探索器の映像の入り口から撮って調べる
    """

    def __init__(self, locator, geometry: CameraGeometry,
                 run_later: Callable[[Callable[[], None], float], None] = _run_later_with_thread,
                 sno: int = SUMO_SNO,
                 green_source: Optional[Callable[[Any], Optional[dict]]] = None):
        super().__init__(locator, geometry, run_later, sno)
        self.green_source = green_source or self._green_from_frame
        #: 直近に撮ったときの緑の具合
        self._green: Optional[dict] = None
        #: 直近に見えた力士ボトルの距離[mm]（回り込む大きさを決める）
        self._last_seen_mm: Optional[float] = None
        #: 回り込みの手順。None なら SumoApproach の段取りに任せている
        self._plan: Optional[list] = None
        self._plan_start_direction = 0.0
        self._plan_start_distance = 0.0
        self._plan_step_started = False
        self._orbits = 0
        self._orbit_side = 0
        self._look_pending = False
        #: 押し出しを終えた（この区間ではもう何もしない）
        self._pushed = False

    # --- 撮影 -----------------------------------------------------------------

    def _green_from_frame(self, frame: Any) -> Optional[dict]:
        return analyze_green(frame, self.geometry, PUSH_REACH_MM)

    def _shoot(self) -> None:
        # 位置特定と緑の見分けは、同じ 1枚で行う
        source = getattr(self.locator, "frame_source", None)
        frame = source() if callable(source) else None
        green = self.green_source(frame)
        if callable(source):
            self.locator.frame_source = lambda: frame
        try:
            super()._shoot()
        finally:
            if callable(source):
                self.locator.frame_source = source
        with self._lock:
            self._green = green
            seen = self._seen
        if seen is not None:
            self._last_seen_mm = seen["distance_mm"]

    # --- 回り込みの手順 ---------------------------------------------------------

    def _start_plan(self, steps: list) -> Command:
        self._plan = steps
        self._plan_step_started = False
        return Command(0, 0, Status.NG)

    def _orbit_plan(self, side: int) -> list:
        """
        side=+1 で右へ、-1 で左へ回り込む手順。

        少し下がってから右を向き（side*90度）、行く手に緑が無いかを見てから横へ出て、
        ボトルのほうを向き直す。横へ出る距離は、押し出す向きが ORBIT_DEG だけ変わる長さ
        （ボトルまでの距離 × tan）。向き直したあとは SumoApproach の段取りに戻り、撮り直して寄り直す。
        """
        reach = (self._last_seen_mm or et_sumo.APPROACH_STOP_AT_MM) + self.geometry.forward_mm + ORBIT_BACK_MM
        sideways = reach * math.tan(math.radians(ORBIT_DEG))
        return [
            ("back", ORBIT_BACK_MM),
            ("turn", side * 90.0, ORBIT_TURN),
            ("look", sideways, side, reach),
            ("move", sideways),
            ("turn", -side * (90.0 + ORBIT_DEG), ORBIT_TURN),
        ]

    def push_length(self, green: Optional[dict]) -> tuple:
        """
        押す長さ（車軸が進む距離）を決める。(押す長さ, 足りるか) を返す。

        ボトルは車軸の BOTTLE_CENTER_AHEAD_MM 前で押される。いまボトルの中心は
        「見えた距離 + カメラの前後位置」にあるので、車軸が T 進むとボトルは
        T + BOTTLE_CENTER_AHEAD_MM − いまの位置 だけ動く。
        緑が見えていれば、ボトルの縁が緑の PUSH_GREEN_MARGIN_MM 手前で止まる長さまでにする。
        """
        bottle_now = (self._last_seen_mm or et_sumo.APPROACH_STOP_AT_MM) + self.geometry.forward_mm
        to_move = lambda travel: bottle_now - BOTTLE_CENTER_AHEAD_MM + travel  # noqa: E731
        longest = PUSH_MAX_MM
        green_at = green.get("push_green_mm") if green else None
        if green_at is not None:
            longest = min(longest, green_at - BOTTLE_CENTER_AHEAD_MM - BOTTLE_RADIUS_MM - PUSH_GREEN_MARGIN_MM)
        length = min(longest, to_move(TARGET_BOTTLE_TRAVEL_MM))
        return max(0.0, length), longest >= to_move(MIN_BOTTLE_TRAVEL_MM)

    @staticmethod
    def _blind_ahead(green: Optional[dict]) -> Optional[float]:
        """
        横へ出る・迂回して進むときの、行く手の緑までの距離（車軸から）。

        写るいちばん手前がもう緑なら、見えない手前にも緑があるかもしれない。走行体が緑の縁を
        かすめるのは許しているので、そのときは「写るいちばん手前の BLIND_GUESS_MM 手前から緑」と
        みなす（0 とみなすと、緑の縁に沿っては一歩も横へ出られず、反対側へ回り込み直していた）
        """
        if not green:
            return None
        if green.get("blind_green"):
            return max(0.0, (green.get("visible_from_mm") or 0.0) - BLIND_GUESS_MM)
        return green.get("ahead_mm")

    @staticmethod
    def choose_orbit_side(green: dict) -> int:
        """
        回り込む側を決める。+1 で右、-1 で左。

        押す向きを、緑が遠い側へ振りたい。右へ回り込むと押す向きは左へ振れ、左へ回り込むと右へ振れる。
        **画素の多さでは決めない。** 遠くの広い緑は画素が多く、ボトルのすぐ向こうの近い緑は
        ボトルに隠れて画素が少ない。画素で決めると、近い緑の側へ押す向きを振ってしまう。
        """
        far = float("inf")
        left = green.get("push_left_mm")
        right = green.get("push_right_mm")
        left = far if left is None else left
        right = far if right is None else right
        if left == right:
            # 距離で決まらなければ、画素の少ない側へ振る
            return 1 if green.get("push_left", 0) < green.get("push_right", 0) else -1
        # 左の緑が遠い → 押す向きを左へ振りたい → 右へ回り込む
        return 1 if left > right else -1

    def _back_to_approach(self) -> Command:
        """回り込みを終えて、撮影から段取りをやり直す"""
        self._plan = None
        self._state = "shoot"
        with self._lock:
            self._seen = None
            self._shooting = False
            self._shot_done = False
        return Command(0, 0, Status.NG)

    def _run_plan(self, obs: Observation, turn_gain: float) -> Command:
        if not self._plan:
            return self._back_to_approach()
        step = self._plan[0]
        kind = step[0]
        direction = float(obs.coordinate.get("direction", 0.0))
        distance = float(obs.coordinate.get("distance", 0.0))

        if not self._plan_step_started:
            self._plan_step_started = True
            self._plan_start_direction = direction
            self._plan_start_distance = distance
            self._look_pending = False

        def next_step() -> Command:
            self._plan.pop(0)
            self._plan_step_started = False
            return Command(0, 0, Status.NG)

        if kind == "turn":
            target = math.radians(step[1])
            progressed = _wrap_pi(direction - self._plan_start_direction)
            done = progressed >= target if target >= 0.0 else progressed <= target
            if done:
                return next_step()
            power = step[2] if len(step) > 2 else turn_gain
            return Command(0.0, power if target >= 0.0 else -power, Status.NG)

        if kind == "move":
            if distance - self._plan_start_distance >= step[1]:
                return next_step()
            return Command(ORBIT_FWD, 0.0, Status.NG)

        if kind == "back":
            if self._plan_start_distance - distance >= step[1]:
                return next_step()
            return Command(-ORBIT_FWD, 0.0, Status.NG)

        if kind == "push":
            if distance - self._plan_start_distance >= step[1]:
                # 押し終えた。この区間はここまで（シナリオのバックへ）
                logger.info("%.0fmm 押し出した → OK", step[1])
                self._plan = []
                self._plan_step_started = False
                self._pushed = True
                return Command(0, 0, Status.OK)
            return Command(PUSH_FWD, 0.0, Status.NG)

        if kind == "probe":
            # 撮って、行く手の緑の手前まで（最大 step[1]）進む。
            # 進めなければ、同じ側へさらに向きを変えて試す（step[2]: 側、step[3]: 残りの回数）
            with self._lock:
                done = self._shot_done
                ask = not done and not self._shooting
                if ask:
                    self._shooting = True
                if done:
                    self._shot_done = False
                    self._seen = None
                    green = self._green
            if not done:
                if ask:
                    self.run_later(self._shoot, SETTLE_SEC)
                return Command(0, 0, Status.NG)
            ahead = self._blind_ahead(green)
            allowed = step[1] if ahead is None else min(step[1], ahead - ARM_TIP_MM - STOP_MARGIN_MM)
            if allowed >= MIN_SIDESTEP_MM:
                logger.info("避けた向きのまま %.0fmm 進む", allowed)
                self._plan[0] = ("move", allowed)
                self._plan_step_started = False
                return Command(0, 0, Status.NG)
            side, tries = step[2], step[3]
            if tries > 1:
                self._plan[0:1] = [("turn", side * DETOUR_TURN_DEG), ("probe", step[1], side, tries - 1)]
                self._plan_step_started = False
                return Command(0, 0, Status.NG)
            return next_step()

        # look: 横へ出る前に、行く手に緑が無いかを撮って見る
        sideways, side, reach = step[1], step[2], step[3]
        need_mm = sideways + ARM_TIP_MM + STOP_MARGIN_MM
        with self._lock:
            done = self._shot_done
            ask = not done and not self._shooting
            if ask:
                self._shooting = True
            if done:
                self._shot_done = False
                self._seen = None
                green = self._green
        if not done:
            if ask:
                self.run_later(self._shoot, SETTLE_SEC)
            return Command(0, 0, Status.NG)
        ahead = self._blind_ahead(green)
        if ahead is not None and ahead < need_mm:
            fit = ahead - ARM_TIP_MM - STOP_MARGIN_MM
            if fit >= MIN_SIDESTEP_MM:
                # 緑の手前まで短く横へ出る。向き直す角度もそのぶん小さくする
                turned = math.degrees(math.atan2(fit, reach))
                logger.info("横へ出る先 %.0fmm に緑 → %.0fmm だけ横へ出る（%.0f度ぶん）", ahead, fit, turned)
                self._plan[1] = ("move", fit)
                self._plan[2] = ("turn", -side * (90.0 + turned), ORBIT_TURN)
                return next_step()
            # この側は緑がある。向きを戻して、反対側へ回り込む（反対側も駄目なら、あきらめて押す）
            logger.info("回り込む先（%s）に緑がある（%.0fmm 先）", "右" if side > 0 else "左", ahead)
            if self._orbit_side == side:
                self._orbit_side = -side
                # 下がるのは済んでいる。向きを戻して、反対側へ横に出る
                self._plan = [("turn", -side * 90.0, ORBIT_TURN)] + [st for st in self._orbit_plan(-side) if st[0] != "back"]
            else:
                self._orbits = MAX_ORBITS
                self._plan = [("turn", -side * 90.0, ORBIT_TURN)]
            self._plan_step_started = False
            return Command(0, 0, Status.NG)
        return next_step()

    # --- 段取りに足すこと -------------------------------------------------------

    def _reset_avoid(self) -> None:
        self._plan = None
        self._pushed = False
        self._orbits = 0
        self._orbit_side = 0
        self._green = None

    def __call__(self, obs: Observation) -> Optional[Command]:
        if obs.sno != self.sno:
            return None

        # 区間に入り直したら、回り込みの途中も捨てる（判断の仕方は SumoApproach と同じ）
        tick = int(obs.tick or 0)
        if obs.cmd_seq != self._cmd_seq or tick < self._tick:
            self._reset_avoid()

        turn_gain = abs(float(obs.run_info.get("TRN") or 40.0))
        if self._pushed:
            self._tick = tick
            return Command(0, 0, Status.OK)       # 押し終えた。次のコマンドへ移るまで OK を返し続ける
        if self._plan is not None:
            self._tick = tick
            return self._run_plan(obs, turn_gain)

        state_before = self._state
        scan_before = self._scan_index
        command = super().__call__(obs)
        if command is None:
            return None

        with self._lock:
            green = self._green

        # 見えずに首振りを始めるとき、写るいちばん手前がもう緑なら、先に来た道を下がって緑から離れる。
        # 緑のすぐ脇では、どちらを向いても手前が緑に見えて、その場で首振りを続けてしまう
        if state_before == "shoot" and self._state == "turn" and scan_before == 0 \
                and green is not None and green.get("blind_green"):
            logger.info("見えず、すぐ前が緑 → 首振りの前に %.0fmm 下がる", ESCAPE_BACK_MM)
            return self._start_plan([("back", ESCAPE_BACK_MM)])

        # 近くで正面に見えた。緑の手前まで押して、土俵を出るのに足りるかを見る
        if command.status == Status.OK:
            length, enough = self.push_length(green)
            if enough or self._orbits >= MAX_ORBITS:
                if not enough:
                    logger.warning("回り込んでも、緑の手前までで土俵を出せる向きが見つからない → %.0fmm だけ押す", length)
                logger.info("押し出す（%.0fmm、押し出す先の緑 %s）", length,
                            "無し" if not green or green.get("push_green_mm") is None
                            else "%.0fmm" % green["push_green_mm"])
                return self._start_plan([("push", length)])
            self._orbits += 1
            if self._orbit_side == 0:
                # 回り込む側は、区間の最初に一度だけ決める。撮るたびに比べ直すと、回り込むたびに
                # 見え方が変わって左右へ行ったり来たりし、向きが変わらない。
                self._orbit_side = self.choose_orbit_side(green)
            logger.info("押し出す先 %.0fmm に緑（左 %d / 右 %d 画素）→ %sへ回り込む（%d回目）",
                        green["push_green_mm"] or -1, green["push_left"], green["push_right"],
                        "右" if self._orbit_side > 0 else "左", self._orbits)
            return self._start_plan(self._orbit_plan(self._orbit_side))

        # 寄ることにしたとき、向きがずれていれば、まずその場で向きを合わせる（まっすぐ前へ出るため）
        if state_before == "shoot" and self._state == "move" and self._last_seen_mm is not None \
                and abs(self._bearing_deg) > 0.0:
            pivot = et_sumo.pivot_deg(self._last_seen_mm, self._bearing_deg, self.geometry.forward_mm)
            if abs(pivot) > STRAIGHT_ALIGN_DEG:
                return self._start_turn(obs, pivot, min(turn_gain, et_sumo.ALIGN_TURN if abs(pivot) < 15 else turn_gain))
            self._bearing_deg = 0.0             # 向きは合っている。曲がらずに進む
            # 向きが合っているので、途中で止まって撮り直さず、止まる位置まで一度に進む
            self._step_mm = max(self._step_mm, self._last_seen_mm - et_sumo.APPROACH_STOP_AT_MM)

        # 寄る・前へ出ることにしたときは、行く手の緑の手前で止める
        if state_before == "shoot" and self._state == "move" and green is not None:
            ahead = green.get("ahead_mm")
            if ahead is not None:
                # 写るいちばん手前がもう緑なら、見えない手前にも緑があるかもしれない。進まない
                allowed = -1.0 if green.get("blind_green") else ahead - ARM_TIP_MM - STOP_MARGIN_MM
                if allowed < self._step_mm:
                    if allowed >= 20.0:
                        logger.info("行く手に緑（%.0fmm 先）→ %.0fmm だけ進む", ahead, allowed)
                        self._step_mm = allowed
                    else:
                        # 進む余地が無い。緑が遠い側へ向きを変え、その向きのまま少し進んでから探し直す
                        side = float(-self.choose_orbit_side(green))
                        logger.info("すぐ前に緑（%.0fmm 先）→ %s へ向きを変えて進む", ahead, "右" if side > 0 else "左")
                        return self._start_plan([("turn", side * DETOUR_TURN_DEG),
                                                 ("probe", DETOUR_MM, side, DETOUR_TRIES)])
        return command


def attach(frame_source: Optional[Callable[[], Any]] = None,
           geometry: CameraGeometry = ROBOT_CAMERA,
           run_later: Callable[[Callable[[], None], float], None] = _run_later_with_thread,
           sno: int = SUMO_SNO) -> SumoApproachAvoidGreen:
    """緑を避ける力士寄せを remote_control に差す。引数は et_sumo.attach と同じ"""
    source = frame_source or _robot_frame_source()
    control = SumoApproachAvoidGreen(make_locator(source, geometry), geometry, run_later, sno)
    set_controller(control)
    logger.info("ET相撲の力士寄せ（緑を避ける）を差しました（SNO %d）", sno)
    return control
