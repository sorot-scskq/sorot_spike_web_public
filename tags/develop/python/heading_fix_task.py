"""
走行中に、床の二次元コードで測った向きへ自己位置を打ち直す（2026・ETラリー）。

シナリオの行に書いた SCRIPT（`2026/recog/heading_fix.py`）から呼ばれる。

    カメラ ──▶ 床のコードを測る ──▶ FloorCodeHeading（防波堤）
                                        ↓
                          CoordCorrector ──▶ 先の行へ CoordCorrectInfo
                                        ↓ C++ がその行を読んだとき
                          HWMonitor::correctCoordinateInfo（向きだけ）

【走行を止めないこと】
カメラが無い・コードが見えない・書き込む先が無い、のどれでも走行は続く。
打ち直されないだけで、これまでどおり推定で走る。ここで例外を投げると、C++ が
判定結果を取りにいくたびに詰まる。

【直す量が違う2つの補正を、1回の観測でまかなう】
向き保持直進のカメラ補正（`Docs/heading_hold.md`）は**車体の向きそのもの**を
目標へ寄せる（ジャイロの積分角 YawAngle）。こちらが直すのは**自己位置の向き**
（driveDirection と mCoordHeading）で、x/y の積み方に効く。**別の量なので同時に
使ってよい。**

**SCRIPT は1行に1つしか書けない**ので、以前はどちらかを選ぶ必要があった。
**測る量は同じ**（格子に対する向きのずれ）なので、いまは1回の観測から両方へ配る。

    床のコードで測れた  → 自己位置を打ち直す ＋ 走らせ方の目標も直す
    測れなかった        → 目印・黒線で測り直し、走らせ方の目標だけ直す

床のコードは ETラリーの区画にしか無いので、見えない場所ではこれまでどおりになる。

【測る側との取り決め】
`FloorCodeObservation.heading_deg` は**コース座標系での走行体の向き**
（`floor_code_observer.CODE_AXIS_COURSE_DEG` が 0 のときは格子基準のずれ）。
45度刻みには畳まれていないので、畳むのはこちら（`floor_code_gate.fold_to_step`）。
**`CODE_AXIS_COURSE_DEG` には 45 の倍数だけを入れること。** 半端な値を入れると
「いちばん近い 45度の倍数」が本来の向きと合わなくなる。

【位置は打ち直さない】
向きだけ（ACAF 0x04）。位置は「どのコードか」が分からないと決められないので、
向きが落ち着いてから別に作る（`Docs/position_correction_design.md` 5.5）。
"""

import logging
import threading
import time
from typing import Any, Callable, List, Optional

from coord_correct import CoordCorrector
from pose import PoseReader
from robot_pose import RobotPose
from scenario_rows import ScenarioRows

try:
    from floor_code_gate import (
        FloorCodeGate, HeadingMeasurement, measurement_to_heading)
except ImportError:  # パッケージとして読み込まれた場合
    from .floor_code_gate import (
        FloorCodeGate, HeadingMeasurement, measurement_to_heading)

logger = logging.getLogger(__name__)

#: 校正が無いときに使う「カメラの幾何で床へ戻す」印
CAMERA_GEOMETRY = "camera-geometry"


#: 測ったずれを、走らせ方を直す側（向き保持の目標）にも載せるか
#:
#: **SCRIPT は1行に1つしか書けない。** これまでは「自己位置を打ち直す」と
#: 「走らせ方を直す」のどちらかを選ぶ必要があった（`route_register.USE_HEADING_FIX`）。
#: **直す量は別でも、測る量は同じ**（格子に対する向きのずれ）なので、1回の観測から
#: 両方へ配れる。測り直さないので時間も増えない。
#:
#: 載せ先は `heading_task.HeadingTask.publish()`（通し番号を1つに保つため）。
#: C++ 側は `HEADING_CORRECT_MAX_DEG`（5°）と 1回 3° の上限で受ける
PUBLISH_HEADING_DEV = True

#: 床のコードで測れなかったとき、目印・黒線で測る側（`heading_task`）へ回すか
#:
#: **床のコードは ETラリーの区画にしか無い。** 見えない場所では、これまでどおり
#: 目印・黒線で測って走らせ方だけ直す（自己位置の打ち直しはしない）。
#: 2026-09-21 の実機では「黒線は使える／目印の格子は使えない」
#: （`Docs/heading_from_camera_field_20260921.md`）
FALLBACK_TO_MARKERS = True

#: 打ち直しをするか。**False にすれば、シナリオに SCRIPT が書いてあっても何もしない**
#:
#: シナリオから SCRIPT を消すのが本筋だが、実機で困ったときに**1か所で止められる**
#: 手立てを残しておく（再ビルド不要）。走行を止めずに切り分けるためのもの
ENABLED = True

#: 何行先の行に打ち直しを書くか
#:
#: **実機で測って決める。** 差し替えは同じ SNO/CNO を上書きするので、C++ が
#: すでに読み終わった行に書いても効かない。1 で間に合うか、2 以上が要るかは
#: 区間の長さと速さによる（測り方は `Docs/heading_correction.md`）。
LOOKAHEAD_ROWS = 1

#: 1回の打ち直しで重ねる枚数
#:
#: **1枚では足りないが、何枚も撮る時間も無い。**
#: 床は奥へ潰れて写るので 1枚の向きは 1°前後ばらつく（`Docs/ground_homography.md` 3章）。
#: 一方、復号は 1280x720 で **1枚 215ms**（`Docs/handoff_floor_code_20260921.md` 3章）で、
#: 走行中にコードが見えている帯は 0.2〜0.3秒しかない。**重ねられるのは数枚**。
#: 枚数を増やすほど確かになるが、そのぶん走ってから書き込むので LOOKAHEAD_ROWS が要る。
SAMPLE_FRAMES = 4

#: 重ねるのをやめる時間[s]。これを過ぎたら、集まったぶんで決める
SAMPLE_SECONDS = 0.8

#: 決めるのに要る最低の枚数。これに満たなければ打ち直さない
#:
#: **1 にしてある。** 1280x720 では復号が 1枚 215ms、コードが見えている帯は 0.2〜0.3秒
#: なので、**1回の通過で試せるのは 0.9〜1.4回＝多くて1枚**
#: （`Docs/handoff_floor_code_20260921.md` 3章）。2枚を要ると、ほとんどの通過で
#: 打ち直さないことになる。1枚の向きは 1°前後ばらつくが、旋回1回の残差 ±0.5° が
#: 積もるのを止める役には立つし、外れ値は残りの歯止め（10°の上限・形の検査・
#: 中身の照合）で落ちる。
#:
#: **上げたければ、先に1枚あたりの時間を削ること。** 枚数は時間で決まるので、
#: 枚数だけ要求しても打ち直しが止まるだけになる
MIN_SAMPLES = 1

#: 1枚撮るごとの待ち[s]。復号のほうがずっと遅いので、待ちは要らない
SAMPLE_INTERVAL = 0.0

#: 重ねた枚数のばらつきの上限[度]。これより散らばっていたら測り間違いを疑って捨てる
SAMPLE_SPREAD_DEG = 4.0


class HeadingFixTask:
    """SCRIPT で指示された区間で、向きを測って自己位置へ打ち直す。"""

    _instance: Optional["HeadingFixTask"] = None
    _instance_lock = threading.Lock()

    @classmethod
    def get_instance(cls) -> "HeadingFixTask":
        """走行体で共有する1つを返す。無ければ作る"""
        with cls._instance_lock:
            if cls._instance is None:
                cls._instance = cls()
            return cls._instance

    @classmethod
    def reset_instance(cls) -> None:
        """共有している1つを捨てる（テスト用）"""
        with cls._instance_lock:
            cls._instance = None

    def __init__(self, frame_source: Optional[Callable[[], Any]] = None,
                 detector: Optional[Callable[[Any, Any], Any]] = None,
                 plane_source: Optional[Callable[[], Any]] = None):
        self._lock = threading.Lock()
        self._frame_source = frame_source
        self._detector = detector
        self._plane_source = plane_source
        self._judge = FloorCodeGate()
        #: 測った回数（打ち直せた回数は CoordCorrector が数える）
        self.measured = 0
        #: 直近の測り（確かめる用）
        self.last = None
        #: 同じ理由を何度も出さないための覚え
        self._told = set()

    # --- SCRIPT から呼ばれる --------------------------------------------------

    def fix(self, request_id: int = 0) -> bool:
        """
        何枚か撮って向きを測り、通れば先の行へ打ち直しを書く。

        :returns: 打ち直しを置けたら True
        """
        if not ENABLED:
            self._tell_once("disabled", "向きの打ち直しは止めてあります"
                                        "（heading_fix_task.ENABLED = False）")
            return False

        with self._lock:
            #: 行の番号は要求にしか載っていない（どの行を走っているか）
            asked = RobotPose.get_instance().snapshot()
            if "sno" not in asked or "cno" not in asked:
                self._tell_once("pose", "区間の番号が要求に載っていないので打ち直せません"
                                        "（C++ が古い版かもしれません）")
                return False

            ScenarioRows.shared().note_script_row(asked["sno"], asked["cno"])

            plane = self._ground_plane()
            if plane is None:
                self._tell_once("plane", "床へ戻す手立てが無いので打ち直しません")
                return False

            measurement = self._sample(plane)
            self.measured += 1
            self.last = measurement
            if not measurement.ok:
                logger.info("床のコードから向きを決められませんでした（%s）", measurement.reason)
                return self._fallback_to_markers(request_id)

            # **同じ観測から、走らせ方を直す側にも配る。**
            # 直す量は別（あちらは車体の向き、こちらは自己位置の向き）だが、測る量は同じ
            if PUBLISH_HEADING_DEV:
                self._publish_heading_dev(measurement)

            #: **向きは撮り終えた時点の値を読む。** 要求に載ってくるのは区間に入った
            #: 瞬間の値で、重ねているあいだ（最大 SAMPLE_SECONDS）に古くなる
            pose_heading = self._pose_heading(asked)
            if pose_heading is None:
                return False

            heading = measurement_to_heading(pose_heading, measurement)
            return CoordCorrector.shared().correct_heading(
                pose_heading_deg=pose_heading,
                measured_heading_deg=heading,
                sno=asked["sno"], cno=asked["cno"],
                steps=LOOKAHEAD_ROWS,
                note=measurement.note())

    def _pose_heading(self, asked: dict):
        """
        打ち直す基準にする「いま思っている向き」[度]。

        **置き場（/dev/shm/sorot_pose.json）の新しい値を先に使う。**
        古ければ、そのぶん打ち直す値がずれるので使わない（`PoseReader.read_fresh`）。
        置き場が無い古い C++ のときだけ、要求に載ってきた値へ落ちる。
        """
        pose = PoseReader.read_fresh()
        if pose is not None:
            return pose.heading_deg
        if "heading" in asked:
            self._tell_once("pose_file", "自己位置の置き場が新しくないので、"
                                         "区間に入った時点の向きで打ち直します")
            return asked["heading"]
        logger.info("いま思っている向きが分からないので打ち直しません")
        return None

    def _publish_heading_dev(self, measurement) -> None:
        """測ったずれを、走らせ方を直す側へ載せる（失敗しても打ち直しは続ける）"""
        try:
            from heading_task import HeadingTask
            HeadingTask.get_instance().publish(
                measurement.deviation_deg, source="floor_code",
                markers=measurement.codes)
        except Exception:  # noqa: BLE001 — 載せられなくても自己位置の打ち直しは続ける
            logger.exception("向きのずれを走らせ方の側へ載せられませんでした")

    def _fallback_to_markers(self, request_id: int) -> bool:
        """
        床のコードで測れなかったとき、目印・黒線で測る側へ回す。

        **走らせ方だけ直す**（自己位置は打ち直さない）。床のコードが見えない場所でも、
        これまでの補正が効いたままになる。
        """
        if not (FALLBACK_TO_MARKERS and PUBLISH_HEADING_DEV):
            return False
        try:
            from heading_task import HeadingTask
            HeadingTask.get_instance().measure(request_id)
        except Exception:  # noqa: BLE001 — 測れなくても走行は続ける
            logger.exception("目印・黒線での測定に失敗しました")
        return False

    # --- 重ねて測る -----------------------------------------------------------

    def _sample(self, plane: Any) -> HeadingMeasurement:
        """
        何枚か撮って、ずれの中央値を採る。

        **1枚では精度が足りない**（`SAMPLE_FRAMES` の説明）。散らばりが大きい回は
        測り間違いが混ざっているとみなして捨てる。中央値にしてあるのは、外れが
        1枚混ざっても動かないため。
        """
        devs: List[float] = []
        codes = 0
        last_reason = "映像が取れない"
        deadline = time.time() + SAMPLE_SECONDS

        while len(devs) < SAMPLE_FRAMES and time.time() < deadline:
            frame = self._frame()
            if frame is None:
                break

            observations = self._observe(frame, plane)
            if observations is None:
                return HeadingMeasurement(False, reason="測るモジュールが使えない")

            one = self._judge.decide(observations)
            if one.ok:
                devs.append(one.deviation_deg)
                codes = max(codes, one.codes)
            else:
                last_reason = one.reason
            time.sleep(SAMPLE_INTERVAL)

        if len(devs) < MIN_SAMPLES:
            return HeadingMeasurement(
                False, codes=codes,
                reason="%d枚しか測れない（%d枚要る）・最後の理由: %s"
                       % (len(devs), MIN_SAMPLES, last_reason))

        spread = max(devs) - min(devs)
        if len(devs) >= 2 and spread > SAMPLE_SPREAD_DEG:
            # **散らばりは2枚以上そろったときだけ見られる。** 1枚のときは
            # ここでは落とせないので、残りの歯止めに任せる
            return HeadingMeasurement(
                False, codes=codes,
                reason="%d枚のずれが %.2f° 散らばっている（上限 %.1f°）"
                       % (len(devs), spread, SAMPLE_SPREAD_DEG))

        ordered = sorted(devs)
        middle = len(ordered) // 2
        dev = (ordered[middle] if len(ordered) % 2
               else (ordered[middle - 1] + ordered[middle]) / 2.0)
        return HeadingMeasurement(True, dev, codes, spread, frames=len(devs))

    # --- 材料をそろえる -------------------------------------------------------

    def _observe(self, frame: Any, plane: Any) -> Optional[List[Any]]:
        """
        床のコードを測る（測るのは `Common/floor_code_heading.py`）。

        :returns: 見えたコードのリスト。測る側が居なければ None（打ち直しをやめる）
        """
        detector = self._detector
        if detector is None:
            detector = self._load_detector(plane)
        if detector is None:
            return None

        try:
            result = detector(frame, plane)
        except Exception:  # noqa: BLE001 — 測れなくても走行は続ける
            logger.exception("床のコードの測定に失敗しました")
            return None

        if result is None:
            return []
        codes = getattr(result, "codes", None)
        if codes is None and isinstance(result, dict):
            codes = result.get("codes")
        if codes is None:
            codes = result if isinstance(result, (list, tuple)) else []
        return list(codes)

    def _load_detector(self, plane: Any) -> Optional[Callable[[Any, Any], Any]]:
        """
        測る側を組み立てる。

        読むのは `Common/floor_code.py`（B の成果。中身と四隅を返す）、
        観測へ直すのは `floor_code_observer.py`。**床平面が無ければ
        カメラの幾何**（`heading_reader.ROBOT_CAMERA`）で床へ戻す。
        """
        try:
            from floor_code_observer import FloorCodeObserver
        except ImportError:
            self._tell_once("detector", "床のコードを観測に直すモジュール"
                                        "（2026/floor_code_observer.py）が無いので打ち直しません")
            return None

        observer = FloorCodeObserver(plane=None if plane is CAMERA_GEOMETRY else plane)
        self._detector = lambda frame, _plane: observer.observe(frame)
        return self._detector

    def _ground_plane(self) -> Any:
        """
        床へ戻す道。

        **校正（`GroundPlane`）があればそれを使う。** メジャーで測った4点から
        作るので確か。無ければ `CAMERA_GEOMETRY` を返し、カメラの高さ・俯角から
        戻す（現地の実測 220mm / 41.3°。**画角はまだ仮**）。

        つまり**校正が無くても打ち直しは動く。** 確かさは落ちるので、校正ができたら
        そちらへ寄せること（`Docs/ground_homography.md`）。
        """
        if self._plane_source is not None:
            return self._plane_source()
        try:
            from ground_plane import GroundPlane
        except ImportError:
            self._tell_once("ground_plane", "床の校正（ground_plane.py）が無いので、"
                                            "カメラの高さと俯角で床へ戻します")
            return CAMERA_GEOMETRY
        plane = GroundPlane.shared()
        if plane is None:
            self._tell_once("uncalibrated", "床がまだ校正されていないので、"
                                            "カメラの高さと俯角で床へ戻します")
            return CAMERA_GEOMETRY
        return plane

    def _frame(self) -> Any:
        """いまの映像。色の読み取りが回っていれば、その最後の1枚を借りる"""
        if self._frame_source is not None:
            try:
                return self._frame_source()
            except Exception:  # noqa: BLE001
                logger.exception("映像の取り込みに失敗しました")
                return None

        from bottle_color_task import BottleColorTask
        from camera import RobotCamera

        camera = RobotCamera.shared()
        if BottleColorTask.get_instance().is_reading():
            # 同時に read() すると壊れる（camera.py の RobotCamera を参照）
            return camera.latest_frame()
        return camera.grab_latest()

    def _tell_once(self, key: str, message: str) -> None:
        """同じ理由は1回だけ出す（毎区間で出すとログが埋まる）"""
        if key not in self._told:
            self._told.add(key)
            logger.warning("%s", message)
        else:
            logger.debug("%s", message)
