"""
ET ラリーの走行中、前方カメラでゲートを見張る（2026・ETラリー）。

    カメラ ──▶ GateFinder ──▶ GateCheckTask ──▶ ログ（通れる／当たる）
                                    └──▶ RoughSpot/PhotoImage/gate/ に絵

【なぜ要るか】
経路は開ループで、自己位置の指標は**実際の曲がりを隠す**（制御と推定が同じジャイロを
見ているため。`Docs/handoff_20260922.md` 4章）。ゲートは動かない外の基準なので、
「通れる位置にいたか」は記録の外から確かめられる。

【走りは変えない】
測って残すだけ。**当たると分かっても止めないし、曲げない。** 直す仕組み（自己位置の
打ち直し・経路の書き直し）は別にあり、まずは測れることを確かめる段階。

【起こすのは PC が差した SCRIPT】
経路の行を作っているのは PC なので、**SCRIPT を差すのも PC の仕事**
（PC-System_2026 の `CreateRouteV2.gate_check_indices`。ゲートを前向きに通り、
同じ行に助走のある行にだけ差す）。走行体はその行に入ってから WATCH_SECONDS だけ見張る。

PC が古くて SCRIPT を差してこないときは、経路が届いた時点で走り終わりまで見張る側へ
落ちる（`config_2026.on_route_registered`）。手引きは `Docs/gate_check.md`。
"""

import logging
import os
import statistics
import threading
import time
from typing import Any, Callable, Dict, List, Optional

from bottle_color import BottleColor
from opencv import OpenCv
from robot_pose import RobotPose
from gate_steer import GateSteer
from route_register import SwitchFile

try:
    from gate_finder import GateFinder, GateSighting, margin_mm
    from heading_reader import ROBOT_CAMERA
except ImportError:  # パッケージとして読み込まれた場合
    from .gate_finder import GateFinder, GateSighting, margin_mm
    from .heading_reader import ROBOT_CAMERA

logger = logging.getLogger(__name__)

#: このタスクを起こす SCRIPT のパス（PC が経路の行に差してくるもの）。
#: **PC-System_2026 の `config/rough_spot_config.py` の ET_RALLY_GATE_CHECK_SCRIPT と
#: 同じにすること。** 届いた経路に入っているかを見分けるのに使う
GATE_CHECK_SCRIPT = "2026/recog/gate_check.py"

#: 見張るか。**False にすれば、シナリオに SCRIPT が書いてあっても何もしない**
ENABLED = True

#: git の管理外で上から止める置き場（走行体は checkout なので、ソースを書き換えても戻る）
ENABLED_SWITCH = "./sorot_spike/gate_check.txt"

#: 何秒おきに1枚見るか。
#:
#: **走行体では1枚 29.5ms**（1280x720 を半分に縮めて。asp を動かしたまま実測、
#: 2026-09-22）。母艦の 3ms とは1桁違うので、母艦の数字で決めないこと。
#:
#: 0.15秒おきなら CPU 1個の2割。ゲートが見えるのは 0.25〜0.95m の帯で、経路の
#: 直進は 300mm/s 前後だから、通り抜けるまで 2.3秒 ＝ **15枚見える。**
#: 判定に使うのはいちばん近い1枚だけなので、これで足りる。
#:
#: **短くするほど、測っている当人が走りを変えてしまう。** 追い込んでいるのは
#: ミリ単位の話なので、見張りが CPU を取ると比べる相手が動く
SAMPLE_INTERVAL = 0.15

#: 見えなくなってから「通り過ぎた」と決めるまで[s]
GONE_SECONDS = 1.0

#: SCRIPT で起こされたとき、何秒見張るか。
#:
#: **ゲートの近くだけ見る。** PC はゲートを通る行にだけ SCRIPT を差すので
#: （PC-System_2026 の `CreateRouteV2.gate_check_indices`）、その行に入ってから
#: 通り抜けるまでを覆えばよい。助走 375mm ＋ 通過 375mm を FWD 70 で走って 2秒前後、
#: 長い行でも 1106mm ＝ 3秒。余裕を見て 5秒。
#:
#: 同じ行をもう一度通ったり、次のゲートの行に入れば、そのたびに延びる。
#: **走り終わりまで回しっぱなしにしないのは CPU のため**（1枚 29.5ms）
WATCH_SECONDS = 5.0

#: 絵を残す先
SNAPSHOT_DIR = "./sorot_spike/RoughSpot/PhotoImage/gate"

#: 0 にすると残さない。**本番は 0 にすること**
SNAPSHOT_ENABLE = 1

#: 残す枚数の上限。古いものから消す
SNAPSHOT_KEEP = 60

#: 判定に使う、近い側の枚数。**1枚では外れ値を掴む**
#:
#: 2026-09-23 の据え置き実測（静止したゲート、1000mm、449枚）で、距離が 13mm 刻みに
#: 量子化されるため、柱2本の距離差が振れて `approach` が -8.2〜+29.2° までばらついた。
#: 余裕は傾き1度あたり 3.8mm 効くので、**同じゲートに対して margin が -42〜-108mm** と
#: 60mm 以上動いた。中央値を採れば -50mm 付近で落ち着く。
VERDICT_SAMPLES = 5

#: 判定に使ってよい斜めの大きさ[deg]。**これを超える測りは採らない**
#:
#: 2026-09-23 の走行（50件）で、間隔（正 258mm）の出かたが角度ではっきり分かれた。
#:
#:     斜め  0〜10°  17件  中央値 254mm（倍率 0.98）   ← 使える
#:     斜め 10〜25°   7件  中央値 273mm（倍率 1.06）
#:     斜め 25°以上  26件  中央値 233mm（204〜307）    ← ばらばら
#:
#: 斜めから見ると奥の柱の足元が当たらなくなる。**そのとき offset も信じられない。**
#: 走行中は「通り抜けるゲート」以外も視界に入る（真横のゲートなど）ので、
#: ここで落とさないと、通りもしないゲートの判定が記録に混ざる。
TRUST_APPROACH_DEG = 15.0


class GateTrack:
    """ゲート1つを見つけてから見えなくなるまで"""

    def __init__(self, color: str):
        self.color = color
        self.seen = 0
        self.first_distance_mm: Optional[float] = None
        self.last_at = 0.0
        #: 見えた測りを全部覚える（判定は近い側の中央値。絵は最寄りの1枚だけ持つ）
        self.sightings: List[GateSighting] = []
        #: いちばん近くで測った1回。**判定にはこれを使う**（近いほど当たる）
        self.nearest: Optional[GateSighting] = None
        self.nearest_pose: Optional[Dict[str, Any]] = None
        self.nearest_frame: Any = None

    def add(self, sighting: GateSighting, pose: Optional[Dict[str, Any]],
            frame: Any, now: float) -> None:
        self.seen += 1
        self.last_at = now
        self.sightings.append(sighting)
        if self.first_distance_mm is None:
            self.first_distance_mm = sighting.distance_mm
        if self.nearest is None or sighting.distance_mm < self.nearest.distance_mm:
            self.nearest = sighting
            self.nearest_pose = pose
            self.nearest_frame = frame

    def consensus(self) -> Optional[GateSighting]:
        """
        判定に使う1つ。**近い側 VERDICT_SAMPLES 枚の中央値**。

        1枚だけだと、距離の量子化で `approach` が数度ふらつき、余裕が 60mm 動く
        （`VERDICT_SAMPLES` の注記）。中央値なら外れ値を落とせる。
        """
        usable = [s for s in self.sightings
                  if s.offset_mm is not None
                  and abs(s.approach_deg or 0.0) <= TRUST_APPROACH_DEG]
        if not usable:
            return None                  # 斜めからしか見ていない。判じない
        near = sorted(usable, key=lambda s: s.distance_mm)[:VERDICT_SAMPLES]
        if len(near) < 2:
            return near[0]

        def median(pick):
            return statistics.median([pick(s) for s in near])

        offset = median(lambda s: s.offset_mm)
        approach = median(lambda s: s.approach_deg or 0.0)
        span = median(lambda s: s.span_mm)
        return GateSighting(
            color=self.color, distance_mm=median(lambda s: s.distance_mm),
            lateral_mm=median(lambda s: s.lateral_mm), span_mm=span,
            offset_mm=offset, margin_mm=margin_mm(offset, approach),
            approach_deg=approach, posts=near[0].posts)

    def verdict(self) -> str:
        """ログ1行ぶんのまとめ"""
        name = BottleColor.NAMES.get(self.color, self.color)
        decided = self.consensus()
        if decided is None:
            測れた = [s for s in self.sightings if s.offset_mm is not None]
            if not 測れた:
                # ゲートと並んで走っていた（通り抜けの位置が決まらない）
                if self.nearest is not None:
                    return "%s ゲート: 通れるか決まりません（%s・%d枚）" % (
                        name, self.nearest.summary(), self.seen)
                return "%s ゲートは測れませんでした" % name
            return "%s ゲート: **斜めからしか見ていないので判じません**（%d枚・最小 %+.1f°）" % (
                name, self.seen, min((s.approach_deg or 0.0 for s in 測れた), key=abs))
        passes = decided.will_pass
        if passes is None:
            judged = "通れるか決まりません"
        elif passes:
            judged = "通れます"
        else:
            judged = "**柱に当たります**"
        return "%s ゲート: %s（%s・%d枚・%.0fmm 先で見え始め・%s・%s）" % (
            name, judged, decided.summary(), self.seen,
            self.first_distance_mm or 0.0, self.place(), self.arrival())

    def arrival(self) -> str:
        """
        **いちばん近くで見た1枚の姿勢**（＝ゲートへ入る直前の姿勢）。

        寄せ（`gate_steer`）が「振って、戻す」を終えられたかは、これでしか分からない。
        判定に使う値は近い側5枚の中央値なので、**戻し切ったかは最後の1枚を見る。**
        ゲートが画角に収まるのは 271mm までなので、そこが最後になる。
        """
        測れた = [s for s in self.sightings if s.offset_mm is not None]
        if not 測れた:
            return "到着NA"
        last = min(測れた, key=lambda s: s.distance_mm)
        return "到着 %.0fmm 横%+.0fmm 斜め%+.1f°" % (
            last.distance_mm, last.lateral_mm, last.approach_deg or 0.0)

    def place(self) -> str:
        """いちばん近くで測ったときの自己位置。**どの周のどのゲートかを後から突き合わせる"""
        pose = self.nearest_pose or {}
        if "sno" not in pose or "cno" not in pose:
            return "区間NA"
        if "x" not in pose or "y" not in pose:
            return "%d/%d" % (pose["sno"], pose["cno"])
        return "%d/%d x%+.0f y%+.0f" % (pose["sno"], pose["cno"], pose["x"], pose["y"])


class GateCheckTask:
    """走行中にゲートを見張る。走行体で1つだけ使う"""

    _instance: Optional["GateCheckTask"] = None
    _instance_lock = threading.Lock()

    @classmethod
    def get_instance(cls) -> "GateCheckTask":
        with cls._instance_lock:
            if cls._instance is None:
                cls._instance = cls()
            return cls._instance

    @classmethod
    def reset_instance(cls) -> None:
        """共有している1つを捨てる（テスト用）"""
        with cls._instance_lock:
            if cls._instance is not None:
                cls._instance.stop()
            cls._instance = None

    def __init__(self, frame_source: Optional[Callable[[], Any]] = None,
                 finder: Optional[GateFinder] = None,
                 clock: Callable[[], float] = time.monotonic):
        self._frame_source = frame_source
        self._finder = finder or GateFinder(ROBOT_CAMERA)
        self._clock = clock
        self._lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._tracks: Dict[str, GateTrack] = {}
        #: 見張りをやめる時刻。**None なら期限なし**（0.0 は「もう切れている」）
        self._deadline: Optional[float] = 0.0
        #: 通り過ぎたゲートの記録（走り終わりにまとめて出す）
        self.passed: List[GateTrack] = []
        self.snapshot_dir = SNAPSHOT_DIR

    # -- 入り口 ---------------------------------------------------------------

    def watch(self, request_id: int = 0, seconds: Optional[float] = WATCH_SECONDS) -> bool:
        """
        見張りを起こす。

        すでに動いていれば、見張る期限だけ延ばす（同じ行をもう一度通っても増えない）。

        :param seconds: 何秒見張るか。**None なら走り終わりまで**（PC が SCRIPT を
            差してこないときの保険。`config_2026.on_route_registered`）
        :returns: 新しく起こしたら True
        """
        if not SwitchFile.read(ENABLED_SWITCH, ENABLED):
            logger.info("ゲートの確認は止めてあります（%s）", ENABLED_SWITCH)
            return False
        with self._lock:
            self._extend(seconds)
            if self._thread is not None and self._thread.is_alive():
                return False
            self._stop.clear()
            self._thread = threading.Thread(target=self._loop, name="gate-check", daemon=True)
            self._thread.start()
        logger.info("ゲートの確認を始めました（%s）",
                    "走り終わりまで" if seconds is None else "%.0f秒" % seconds)
        return True

    def _extend(self, seconds: Optional[float]) -> None:
        """見張る期限を延ばす。**ロックの中から呼ぶ**"""
        if seconds is None:
            self._deadline = None           # 期限なしが勝つ（保険のほうを消さない）
        elif self._deadline is not None:
            self._deadline = max(self._deadline, self._clock() + seconds)

    def stop(self) -> None:
        """見張りをやめ、見えたままのゲートも記録へ落とす"""
        self._stop.set()
        with self._lock:
            self._deadline = 0.0
            for track in list(self._tracks.values()):
                self._close(track)
            self._tracks.clear()

    def summary(self) -> List[str]:
        """走り終わりのまとめ。通った順に1行ずつ"""
        return [t.verdict() for t in self.passed]

    # -- 中身 -----------------------------------------------------------------

    def _loop(self) -> None:
        """一定の間で1枚ずつ見る。**例外で落ちないこと**（走行が続くため）"""
        while not self._stop.is_set() and not self._expired():
            started = time.perf_counter()
            try:
                self._look()
            except Exception:  # noqa: BLE001 — 見られなくても走行は続ける
                logger.exception("ゲートの確認に失敗しました")
            remaining = SAMPLE_INTERVAL - (time.perf_counter() - started)
            if remaining > 0:
                self._stop.wait(remaining)
        # 期限が来たときも、見えたままのゲートを記録へ落とす
        with self._lock:
            for track in list(self._tracks.values()):
                self._close(track)
            self._tracks.clear()

    def _expired(self) -> bool:
        """見張る期限が来たか"""
        with self._lock:
            return self._deadline is not None and self._clock() >= self._deadline

    def _look(self) -> None:
        """1枚見て、見えたゲートを覚える。見えなくなったものは記録へ落とす"""
        frame = self._frame()
        now = self._clock()
        result = self._finder.find(frame)
        pose = RobotPose.get_instance().snapshot()

        with self._lock:
            for sighting in result.gates:
                track = self._tracks.get(sighting.color)
                if track is None:
                    track = GateTrack(sighting.color)
                    self._tracks[sighting.color] = track
                    logger.info("ゲートが見えました: %s", sighting.summary())
                track.add(sighting, pose, frame, now)

            for color, track in list(self._tracks.items()):
                if now - track.last_at >= GONE_SECONDS:
                    self._close(track)
                    del self._tracks[color]

        # **ロックの外で呼ぶ。** self._lock は素の Lock で再入できず、
        # consider() が GateCheckTask を呼び返すと固まる（J の指摘）。
        # 寄せるのは**いちばん近い1つ**だけ。奥のゲートに合わせない
        try:
            GateSteer.get_instance().consider(result.nearest)
        except Exception:   # noqa: BLE001 — 寄せられなくても見張りは続ける
            logger.exception("ゲートへ寄せるのに失敗しました")

    def _close(self, track: GateTrack) -> None:
        """見えなくなったゲートを記録へ落とす。**ロックの中から呼ぶ**"""
        self.passed.append(track)
        logger.info("%s", track.verdict())
        self._save_snapshot(track)
        track.nearest_frame = None      # 絵は残したので手放す

    def _frame(self) -> Any:
        """いまの映像。色の読み取りが回っていれば、その最後の1枚を借りる"""
        if self._frame_source is not None:
            return self._frame_source()

        from bottle_color_task import BottleColorTask
        from camera import RobotCamera

        camera = RobotCamera.shared()
        if BottleColorTask.get_instance().is_reading():
            # 同時に read() すると壊れる（camera.py の RobotCamera を参照）
            return camera.latest_frame()
        return camera.grab_latest()

    # -- 絵 -------------------------------------------------------------------

    def _save_snapshot(self, track: GateTrack) -> Optional[str]:
        """いちばん近くで測った1枚を、測った値を重ねて残す。**失敗しても走行は止めない**"""
        if not SNAPSHOT_ENABLE or track.nearest is None or track.nearest_frame is None:
            return None
        try:
            cv2 = OpenCv.cv2()
            if cv2 is None:
                return None
            os.makedirs(self.snapshot_dir, exist_ok=True)
            path = os.path.join(self.snapshot_dir, self._snapshot_name(track))
            if not cv2.imwrite(path, self._overlay(cv2, track)):
                logger.warning("ゲートの絵を書けませんでした: %s", path)
                return None
            self._prune()
            return path
        except Exception as e:  # noqa: BLE001 — 残せなくても走行は続ける
            logger.warning("ゲートの絵を残せませんでした: %s", e)
            return None

    @staticmethod
    def _snapshot_name(track: GateTrack, when: Optional[float] = None) -> str:
        """
        絵のファイル名。

            gate_20260923_101530_123_red_s31c12_x+1234_y+0568_off+035_mar+016.jpg
                                      ↑色   ↑区間  ↑自己位置     ↑ずれ  ↑余裕[mm]
        """
        now = time.time() if when is None else when
        stamp = "%s_%03d" % (time.strftime("%Y%m%d_%H%M%S", time.localtime(now)),
                             int((now % 1) * 1000))
        pose = track.nearest_pose or {}
        if "sno" in pose and "cno" in pose:
            section = "s%dc%d" % (pose["sno"], pose["cno"])
        else:
            section = "sNAcNA"
        if "x" in pose and "y" in pose:
            place = "x%+05d_y%+05d" % (round(pose["x"]), round(pose["y"]))
        else:
            place = "xNA_yNA"
        sighting = track.nearest
        if sighting is not None and sighting.offset_mm is not None:
            measured = "off%+04d_mar%+04d" % (round(sighting.offset_mm),
                                              round(sighting.margin_mm or 0.0))
        else:
            measured = "offNA"
        return "gate_%s_%s_%s_%s_%s.jpg" % (stamp, track.color, section, place, measured)

    def _overlay(self, cv2, track: GateTrack):
        """柱の枠と足元、測った値を重ねた絵。現地でしきい値を合わせるのに要る"""
        image = track.nearest_frame.copy()
        sighting = track.nearest
        if sighting is None:
            return image
        # 柱の枠は縮めた画の画素なので、元の大きさへ戻して描く
        scale = 1.0 / float(self._finder.options['scale'] or 1.0)
        for post in sighting.posts:
            x, y, w, h = (int(v * scale) for v in post.bbox)
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            foot = (int(post.foot_px[0] * scale), int(post.foot_px[1] * scale))
            cv2.circle(image, foot, 5, (0, 0, 255), -1)
        # OpenCV は日本語を描けないので、重ねる文字は英数字だけにする
        cv2.putText(image, sighting.label(), (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        return image

    def _prune(self) -> None:
        """古い絵から消す。走行を重ねてもディスクを埋めない"""
        try:
            files = sorted(f for f in os.listdir(self.snapshot_dir)
                           if f.startswith("gate_") and f.endswith(".jpg"))
            for f in files[:-SNAPSHOT_KEEP]:
                os.remove(os.path.join(self.snapshot_dir, f))
        except Exception:  # noqa: BLE001 — 消せなくても走行は続ける
            pass
