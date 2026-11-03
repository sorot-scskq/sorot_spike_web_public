"""
ET ラリーの走行中、前方カメラでゲートを見張る（2026・ETラリー）。

    カメラ ──▶ GateFinder ──▶ GateCheckTask ──▶ ログ（通れる／当たる）
                                    └──▶ RoughSpot/PhotoImage/gate/ に絵

【なぜ要るか】
経路は開ループで、自己位置の指標は**実際の曲がりを隠す**（制御と推定が同じジャイロを
見ているため。`Docs/handoff_20260922.md` 4章）。ゲートは動かない外の基準なので、
「通れる位置にいたか」は記録の外から確かめられる。

【走りを変えるのは寄せだけ】
ここは測って残す。**止めない。** 向きを振るのは寄せ（`gate_steer.py`、`gate_steer.txt`）
で、見えたいちばん近いゲートをここから渡す。

【柱が1本でも判定する】
ずれて近づくと、相方の柱が先に視野の外へ出る（中心から 130mm ずれていれば 500mm 手前で）。
**当たりそうなときほど1本しか写らない**ので、1本でも判定する。

  * 見えた柱が車体の通り道に入っていれば、**左右どちらの柱でも当たる**
  * 左右は**経路の計画と自己位置から決める**（`route_follower.gate_hint`）。決まれば
    写っていない相方を 254mm 先に置いて、2本のときと同じに測り、寄せにも渡す

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
import route_follower
from route_register import SwitchFile

try:
    from gate_finder import (GateFinder, GateSighting, POST_SPAN_MM, margin_mm,
                             resolve_single)
    from heading_reader import ROBOT_CAMERA
except ImportError:  # パッケージとして読み込まれた場合
    from .gate_finder import (GateFinder, GateSighting, POST_SPAN_MM, margin_mm,
                              resolve_single)
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

#: 見つからなかったコマを何枚残すか（1回の見張りあたり）
#:
#: **2026-09-25 の実機で、ゲート確認の区間 33コマのうち 30コマ（91%）で
#: 柱の組を作れていなかった。** 据え置きでは 300〜1200mm でほぼ全コマ
#: 見つかるので、**走行中だけの理由がある**（ぶれ・視野の端・近すぎ・露出）。
#: **成功した1枚しか残していないと、その理由が分からない。**
#:
#: 0 で残さない。走りは変えない（保存に失敗しても走行は続く）
MISS_SNAPSHOT_KEEP = 12

#: 判定に使う、近い側の枚数。**1枚では外れ値を掴む**
#:
#: 2026-09-23 の据え置き実測（静止したゲート、1000mm、449枚）で、距離が 13mm 刻みに
#: 量子化されるため、柱2本の距離差が振れて `approach` が -8.2〜+29.2° までばらついた。
#: 余裕は傾き1度あたり 3.8mm 効くので、**同じゲートに対して margin が -42〜-108mm** と
#: 60mm 以上動いた。中央値を採れば -50mm 付近で落ち着く。
VERDICT_SAMPLES = 5

#: 判定を出すのに要る最小の枚数。
#:
#: **中央値は枚数が無ければ効かない。** 2026-09-25 の実データ:
#:
#:   **4枚** → 間隔 253mm（正 254。ほぼ完璧）
#:   **2枚** → 間隔 276mm（+22mm）
#:
#: 以前は1枚でも判定を出していた。**足りないときは黙るのではなく、
#: 「枚数が足りない」と書いた判定を出す**（見えたこと自体は残したいので）
MIN_VERDICT_FRAMES = 3

#: 判定に使う1枚の、斜めの上限[度]。
#:
#: **2026-09-25 に 15 → 5 へ締めて、走行の実データで否定されたので 10 に戻した。**
#:
#:   21:01  斜め **7.1°**  間隔 **253mm**（正 254）  **4枚**  ← 当たっていたほう
#:   21:30  斜め  4.4°   間隔  276mm（+22mm）      **2枚**  ← 外れていたほう
#:
#: **5° にすると、当たっていた 7.1° を捨てて、外れた 4.4° を残す。** 据え置きで
#: 「斜め 3.5° で間隔が 9mm 動く」を見て締めたが、**走行中に効いているのは
#: 枚数のほう**だった（`MIN_VERDICT_FRAMES`）。
#:
#: 10° にしてあるのは、実データで弾きたいものが **-79°/-81°**（ゲートと並んで
#: 見ている）と **9.8/11.1/11.6°** の2群に割れているため。前者は必ず捨て、
#: 後者は際どいので残さない
TRUST_APPROACH_DEG = 10.0


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
        #: 柱が1本しか見えなかった測り（`single=True`）。組の測りとは分けて持つ
        self.singles: List[GateSighting] = []
        self.nearest_single: Optional[GateSighting] = None
        self.nearest_single_pose: Optional[Dict[str, Any]] = None
        self.nearest_single_frame: Any = None
        #: 「柱1本が進路上」をもう言ったか（1つのゲートで1回だけ言う）
        self.hit_warned = False

    def add(self, sighting: GateSighting, pose: Optional[Dict[str, Any]],
            frame: Any, now: float) -> None:
        self.seen += 1
        self.last_at = now
        if self.first_distance_mm is None:
            self.first_distance_mm = sighting.distance_mm
        if sighting.single:
            self.singles.append(sighting)
            if (self.nearest_single is None
                    or sighting.distance_mm < self.nearest_single.distance_mm):
                self.nearest_single = sighting
                self.nearest_single_pose = pose
                self.nearest_single_frame = frame
            return
        self.sightings.append(sighting)
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

    def single_consensus(self) -> Optional[GateSighting]:
        """
        柱1本の測りから出す判定。**近い側 VERDICT_SAMPLES 枚の中央値。**

        左右を地図で決められた測りがあれば、そちらだけを使う（2本のときと同じ量が出る）。
        決められていなければ「通り道に入っているか」だけを言う。**過半の枚で入っていれば
        当たる**。入っていなければ、相方が通り道に入っているかもしれないので決めない。
        """
        resolved = [s for s in self.singles if s.offset_mm is not None]
        pool = resolved or self.singles
        if not pool:
            return None
        near = sorted(pool, key=lambda s: s.distance_mm)[:VERDICT_SAMPLES]

        def median(pick, among=near):
            return statistics.median([pick(s) for s in among])

        if resolved:
            offset = median(lambda s: s.offset_mm)
            approach = median(lambda s: s.approach_deg or 0.0)
            return GateSighting(
                color=self.color, distance_mm=median(lambda s: s.distance_mm),
                lateral_mm=median(lambda s: s.lateral_mm), span_mm=POST_SPAN_MM,
                offset_mm=offset, margin_mm=margin_mm(offset, approach),
                approach_deg=approach, posts=near[0].posts, single=True)
        hits = [s for s in near if s.margin_mm is not None]
        margin = median(lambda s: s.margin_mm, hits) if len(hits) * 2 > len(near) else None
        return GateSighting(
            color=self.color, distance_mm=median(lambda s: s.distance_mm),
            lateral_mm=median(lambda s: s.posts[0].lateral_mm), span_mm=0.0,
            offset_mm=None, margin_mm=margin, approach_deg=None,
            posts=near[0].posts, single=True)

    def approaching_hit(self) -> Optional[GateSighting]:
        """
        **柱1本が進路上にあり、近づいている**なら、その測り。

        直近2枚が「当たる」で、しかも距離が縮んでいるときだけ言う（1枚では外れ値を掴む）。
        """
        if len(self.singles) < 2:
            return None
        before, last = self.singles[-2], self.singles[-1]
        if last.will_pass is not False or before.will_pass is not False:
            return None
        if last.distance_mm >= before.distance_mm:
            return None
        return last

    def _single_verdict(self, name: str, decided: GateSighting) -> str:
        """柱1本の測りだけで出す判定の1行"""
        passes = decided.will_pass
        if passes is None:
            judged = "通れるか決まりません"
        elif passes:
            judged = "通れます"
        else:
            judged = "**柱に当たります**"
        how = ("柱1本・左右は地図で決めた" if decided.offset_mm is not None
               else "柱1本・左右は決めていない")
        thin = ("・**%d枚しかないので信用できません**" % len(self.singles)
                if len(self.singles) < MIN_VERDICT_FRAMES else "")
        return "%s ゲート: %s%s（%s・%s・%d枚・%.0fmm 先で見え始め・%s）" % (
            name, judged, thin, how, decided.summary(), len(self.singles),
            self.first_distance_mm or 0.0, self.place(self.nearest_single_pose))

    def verdict(self) -> str:
        """ログ1行ぶんのまとめ"""
        name = BottleColor.NAMES.get(self.color, self.color)
        decided = self.consensus()
        single = self.single_consensus()
        if decided is None and single is not None:
            return self._single_verdict(name, single)
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
        thin = ("・**%d枚しかないので信用できません**" % len(self.sightings)
                if len(self.sightings) < MIN_VERDICT_FRAMES else "")
        # 2本で判じたあと、もっと近くで柱1本しか見えなかったなら、その結果も添える
        after = ""
        if single is not None and single.will_pass is not None:
            after = "・近くの柱1本では%s" % ("通れる" if single.will_pass else "**当たる**")
        return "%s ゲート: %s%s（%s・%d枚・%.0fmm 先で見え始め・%s・%s%s）" % (
            name, judged, thin, decided.summary(), len(self.sightings),
            self.first_distance_mm or 0.0, self.place(), self.arrival(), after)

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

    def place(self, pose: Optional[Dict[str, Any]] = None) -> str:
        """いちばん近くで測ったときの自己位置。**どの周のどのゲートかを後から突き合わせる"""
        pose = pose or self.nearest_pose or {}
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
                 clock: Callable[[], float] = time.monotonic,
                 hint_source: Optional[Callable[[float], Any]] = None):
        self._frame_source = frame_source
        #: 柱1本の左右を決める見込み（`route_follower.gate_hint`）。
        #: 前方 d[mm] を渡すと (中心の横位置, 傾き) か None を返す
        self._hint_source = hint_source or route_follower.gate_hint
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
        #: ゲートが見つかったコマの数
        self._frames_seen = 0
        #: 見つからなかったコマを、見えた柱の本数で分けた数（0本 / 1本 / 2本以上）
        self._frames_miss = [0, 0, 0]
        #: 残した「見つからなかったコマ」の枚数
        self._miss_saved = 0

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
        self._log_tally()

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
        self._tally(result, frame, pose, now)
        singles = [self._resolve(s) for s in getattr(result, "singles", None) or []]

        with self._lock:
            for sighting in list(result.gates) + singles:
                track = self._tracks.get(sighting.color)
                if track is None:
                    track = GateTrack(sighting.color)
                    self._tracks[sighting.color] = track
                    logger.info("ゲートが見えました: %s%s",
                                "（柱1本）" if sighting.single else "", sighting.summary())
                track.add(sighting, pose, frame, now)
                hit = track.approaching_hit() if sighting.single else None
                if hit is not None and not track.hit_warned:
                    track.hit_warned = True
                    logger.warning("**柱1本が進路上にあり、近づいています。当たります**: %s",
                                   hit.summary())

            for color, track in list(self._tracks.items()):
                if now - track.last_at >= GONE_SECONDS:
                    self._close(track)
                    del self._tracks[color]

        # **ロックの外で呼ぶ。** self._lock は素の Lock で再入できず、
        # consider() が GateCheckTask を呼び返すと固まる（J の指摘）。
        # 寄せるのは**いちばん近い1つ**だけ。奥のゲートに合わせない
        # 柱1本でも、左右が決まっていれば寄せに渡す（決まっていなければ振る向きが無い）
        steerable = list(result.gates) + [s for s in singles if s.offset_mm is not None]
        nearest = min(steerable, key=lambda g: g.distance_mm) if steerable else None
        try:
            GateSteer.get_instance().consider(nearest)
        except Exception:   # noqa: BLE001 — 寄せられなくても見張りは続ける
            logger.exception("ゲートへ寄せるのに失敗しました")

    def _resolve(self, sighting: GateSighting) -> GateSighting:
        """柱1本の測りの左右を、経路の計画から決める。決められなければそのまま返す"""
        try:
            hint = self._hint_source(sighting.distance_mm)
        except Exception:  # noqa: BLE001 — 決められなくても見張りは続ける
            logger.exception("ゲートの中心を計画から見込めませんでした")
            return sighting
        if hint is None:
            return sighting
        center_mm, tilt_deg = hint
        return resolve_single(sighting, center_mm, tilt_deg)

    def _tally(self, result, frame, pose, now: float) -> None:
        """
        **見つからなかったコマを数える。** 見つかった1枚だけでは理由が分からない。

        柱が何本見えたかで、落ちている段が分かれる。

            0本      そもそも柱として拾えていない（明るさ・ぶれ・視野の外）
            1本      片方しか見えない（相方が視野の外か、隠れている）
            2本以上  拾えているのに**組にならない**（間隔の門・高さの門）
        """
        try:
            posts = len(getattr(result, "posts", None) or [])
            if result.gates:
                self._frames_seen += 1
                return
            self._frames_miss[min(posts, 2)] += 1
            if MISS_SNAPSHOT_KEEP and self._miss_saved < MISS_SNAPSHOT_KEEP:
                self._miss_saved += 1
                self._save_miss(frame, pose, posts, now)
        except Exception:  # noqa: BLE001 — 数えられなくても見張りは続ける
            pass

    def _save_miss(self, frame, pose, posts: int, now: float) -> None:
        """見つからなかったコマを1枚残す。**失敗しても走行は止めない**"""
        try:
            cv2 = OpenCv.cv2()
            if cv2 is None or frame is None:
                return
            d = os.path.join(self.snapshot_dir, "miss")
            os.makedirs(d, exist_ok=True)
            # **`now` は単調時計なので日付にしない**（1970年になる）。名前は壁時計で付ける
            wall = time.time()
            stamp = "%s_%03d" % (time.strftime("%Y%m%d_%H%M%S", time.localtime(wall)),
                                 int((wall % 1) * 1000))
            place = ("x%+05d_y%+05d" % (round(pose.get("x", 0)), round(pose.get("y", 0)))
                     if pose else "xNA_yNA")
            cv2.imwrite(os.path.join(d, "miss_%s_%s_p%d.jpg" % (stamp, place, posts)), frame)
        except Exception:  # noqa: BLE001 — 残せなくても走行は続ける
            pass

    def _log_tally(self) -> None:
        """見張りを終えるときに1行だけ出す。**毎コマ出すと周期を壊す**"""
        total = self._frames_seen + sum(self._frames_miss)
        if not total:
            return
        logger.info("ゲートを見た %dコマ: **見つかった %d（%.0f%%）** / "
                    "柱0本 %d / 柱1本 %d / **柱2本以上だが組にならない %d**",
                    total, self._frames_seen, 100.0 * self._frames_seen / total,
                    self._frames_miss[0], self._frames_miss[1], self._frames_miss[2])
        # **柱0本のコマはゲートが視野に無いことが多い**（通り過ぎたあと・別の物を
        # 見ている）。それを分母に入れると率が意味を持たないので、並べて出す
        with_post = self._frames_seen + self._frames_miss[1] + self._frames_miss[2]
        if with_post:
            logger.info("  柱が見えていた %dコマのうち **組にできた %d（%.0f%%）**"
                        "（柱1本 %d は相方が視野の外）",
                        with_post, self._frames_seen, 100.0 * self._frames_seen / with_post,
                        self._frames_miss[1])
        self._frames_seen = 0
        self._frames_miss = [0, 0, 0]
        self._miss_saved = 0

    def _close(self, track: GateTrack) -> None:
        """見えなくなったゲートを記録へ落とす。**ロックの中から呼ぶ**"""
        self.passed.append(track)
        logger.info("%s", track.verdict())
        self._save_snapshot(track)
        self._save_snapshot(track, single=True)
        track.nearest_frame = None      # 絵は残したので手放す
        track.nearest_single_frame = None

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

    def _save_snapshot(self, track: GateTrack, single: bool = False) -> Optional[str]:
        """
        いちばん近くで測った1枚を、測った値を重ねて残す。**失敗しても走行は止めない**

        :param single: 柱1本の測りのほうを残す（組の絵とは別の1枚）
        """
        sighting = track.nearest_single if single else track.nearest
        frame = track.nearest_single_frame if single else track.nearest_frame
        pose = track.nearest_single_pose if single else track.nearest_pose
        if not SNAPSHOT_ENABLE or sighting is None or frame is None:
            return None
        try:
            cv2 = OpenCv.cv2()
            if cv2 is None:
                return None
            os.makedirs(self.snapshot_dir, exist_ok=True)
            path = os.path.join(self.snapshot_dir,
                                self._snapshot_name(track, sighting=sighting, pose=pose))
            if not cv2.imwrite(path, self._overlay(cv2, frame, sighting)):
                logger.warning("ゲートの絵を書けませんでした: %s", path)
                return None
            self._prune()
            return path
        except Exception as e:  # noqa: BLE001 — 残せなくても走行は続ける
            logger.warning("ゲートの絵を残せませんでした: %s", e)
            return None

    @staticmethod
    def _snapshot_name(track: GateTrack, when: Optional[float] = None,
                       sighting: Optional[GateSighting] = None,
                       pose: Optional[Dict[str, Any]] = None) -> str:
        """
        絵のファイル名。

            gate_20260923_101530_123_red_s31c12_x+1234_y+0568_off+035_mar+016.jpg
                                      ↑色   ↑区間  ↑自己位置     ↑ずれ  ↑余裕[mm]

        柱1本の測りの絵は末尾に `_1post` が付く（左右が決まらず当たるなら `offNA_hit_1post`）
        """
        now = time.time() if when is None else when
        stamp = "%s_%03d" % (time.strftime("%Y%m%d_%H%M%S", time.localtime(now)),
                             int((now % 1) * 1000))
        sighting = sighting if sighting is not None else track.nearest
        pose = (pose if pose is not None else track.nearest_pose) or {}
        if "sno" in pose and "cno" in pose:
            section = "s%dc%d" % (pose["sno"], pose["cno"])
        else:
            section = "sNAcNA"
        if "x" in pose and "y" in pose:
            place = "x%+05d_y%+05d" % (round(pose["x"]), round(pose["y"]))
        else:
            place = "xNA_yNA"
        if sighting is not None and sighting.offset_mm is not None:
            measured = "off%+04d_mar%+04d" % (round(sighting.offset_mm),
                                              round(sighting.margin_mm or 0.0))
        else:
            measured = "offNA"
        if sighting is not None and sighting.single:
            if sighting.offset_mm is None and sighting.will_pass is False:
                measured += "_hit"
            measured += "_1post"
        return "gate_%s_%s_%s_%s_%s.jpg" % (stamp, track.color, section, place, measured)

    def _overlay(self, cv2, frame, sighting: Optional[GateSighting]):
        """
        柱の枠と足元、測った値を重ねた絵。現地でしきい値を合わせるのに要る。

        柱は見つけ方で描き分ける。**相方探しで拾った柱は枠が 2px しかない**ので、
        枠ではなく水色の輪で描く（赤い点だけだと、拾えていないように見える）。
        柱1本から置いた相方（`assumed`）は写っていないので描かない。
        """
        image = frame.copy()
        if sighting is None:
            return image
        # 柱の枠は縮めた画の画素なので、元の大きさへ戻して描く
        scale = 1.0 / float(self._finder.options['scale'] or 1.0)
        for post in sighting.posts:
            kind = getattr(post, "kind", "blob")
            if kind == "assumed":
                continue
            foot = (int(post.foot_px[0] * scale), int(post.foot_px[1] * scale))
            if kind == "partner":
                cv2.circle(image, foot, 22, (255, 255, 0), 3)
            else:
                x, y, w, h = (int(v * scale) for v in post.bbox)
                color = (0, 165, 255) if sighting.single else (0, 255, 0)
                cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)
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
