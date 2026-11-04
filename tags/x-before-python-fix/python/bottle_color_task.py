"""
キャリーボトルのラベル色を、走りながら読み続ける（2026・ボトルデリバリー）。

ボトルデリバリーは、拾ったボトルのラベル色と同じ色のゾーンへ運ぶ（規約 5.16）。
運び先は走ってみるまで決まらないので、走行体が自分で色を読み、シナリオが
その色で飛び先を変える。

    カメラ ──▶ BottleColorReader ──▶ BottleColorTask ──▶ CMD_GET_RESULT の bottle_color
                                                              │
                                                  C++ ResultRead ──▶ VarASM（SCV 10〜13）

走行体で1つだけ使う。呼ぶ側は `BottleColorTask.get_instance()` から取る。

【なぜ「見えているうちに決める」のか】
抱えた時点ではもう読めない。ボトルがカメラに近すぎて、ラベルが画角の下へ
外れるためで、読めるのは 449〜249mm 手前だけ。

そこで「抱えたか」ではなく「いつ判別するか」をシナリオに書いてもらう
（RunInfo.SCRIPT に 2026/recog/bottle_color.py）。ボトルへ寄る区間に書けば、
ラベルが見えているうちに判別が走る。把持を返すセンサが無いことは問題にならない。

色が決まったら、その色の運び先で走行中のシナリオを差し替える
（Common/scenario_update.py）。

【青は「多めに見えたとき」だけ採る】
コースの青い目印（黒線の途中にある）が ROI に入る。目印はラベルよりずっと
大きく写るので、赤・黄と同じ扱いにすると目印で「青」と決まってしまう。
そこで青だけ 2つの条件を重くしてある。

    占有率の下限  0.02 → 0.03（BottleColorClassifier の minCoverageByColor）
    確定に要る枚数  3 → 6（JUDGE_STABLE_FRAMES_BY_COLOR）

赤・黄が同時に見えているあいだは赤・黄を先に採る（classifier の colorPriority）。
それでも JUDGE_TIMEOUT のあいだに何も決まらなければ、これまでどおり
青の運び先を置く（UNDECIDED_COLOR）。

【走行を止めないこと】
カメラが無い・開けない・OpenCV が入っていない、のいずれでも走行は続ける。
色が分からなければ「未判定」のまま返り、シナリオは既定の置き場へ運ぶ。
ここで例外を投げて Python 部を落とすと、C++ 側が判定結果を取りにいくたびに詰まる。
"""

import logging
import threading
import time
from typing import Optional

from scenario_update import ScenarioMailbox
from bottle_color import BottleColor
from bottle_color_monitor import BottleColorMonitor
from bottle_color_reader import BottleColorReader

logger = logging.getLogger(__name__)


class BottleColorTask:
    """ラベル色の読み取りを回し、SCRIPT で指示された区間で色を確定させる。"""

    #: 1周の長さの上限[s]。**これより速く回さない、というだけの値。**
    #:
    #: 以前は毎回この時間を「足して」いた。足すと待っているあいだに映像が溜まり、
    #: 次に読むのが古い映像になる（2026-09-08 実測。足すと 18.8Hz で read() が
    #: 11.5ms＝溜まったぶんを即座に受け取る。足さなければ 29.9Hz で read() が
    #: 26ms 待つ＝常に最新）。いまは「足りないぶんだけ待つ」に変えてある。
    #:
    #: カメラから読めているときは read() が1フレーム分（MJPG 1280x720 で 33ms）
    #: ブロックするので、ここは実質効かない。効くのはカメラが無いときと、
    #: 取り込み口を差したとき（テスト）の空回り止めとして。
    MAX_INTERVAL = 1.0 / 30.0

    #: これ未満の端数は待たない[s]。Event.wait は短い時間を渡しても正確には
    #: 起きないため、端数を渡すとかえって1周が伸びる（上の実測を参照）
    MIN_SLEEP = 0.005

    #: カメラを開けなかったときに、次に試すまでの間隔[s]
    REOPEN_INTERVAL = 2.0

    #: 同じ色が何回続けて見えたら確定するか。30Hz なので 3回で 0.1秒
    JUDGE_STABLE_FRAMES = 3

    #: 色ごとの、確定に要る枚数（ここに無い色は JUDGE_STABLE_FRAMES）。
    #:
    #: **青だけ多くする。** 黒線の青い目印が ROI をよぎるあいだの数枚で
    #: 決めてしまわないため。6枚＝0.2秒。ラベルが見えるのは 449〜249mm 手前の
    #: 200mm ぶんで、FWD 60 なら 0.2秒に進むのは約12mm。延ばす余地はある
    JUDGE_STABLE_FRAMES_BY_COLOR = {BottleColor.BLUE: 6}

    #: 判定に使う色。**青も見るが、占有率と枚数の条件を重くしてある**
    #: （コースの青い目印をラベルと取り違えるため。上の説明を参照）
    JUDGE_COLORS = (BottleColor.RED, BottleColor.YELLOW, BottleColor.BLUE)

    #: 判別を始めてから打ち切るまでの時間[s]。
    #: 打ち切らないと区間を出ても判別が続き、あとで見えた赤・黄のもので決まってしまう
    JUDGE_TIMEOUT = 5.0

    #: 打ち切ったときに運ぶ色。**青のまま。**
    #: 青を判定するようになっても、条件を重くしたぶん「青なのに決まらない」は
    #: 起こりうる。決まらなかったときの逃げ道は引き続き青の運び先にする
    #: （held_color() は 'none' のままなので、決めたのか打ち切ったのかは区別できる）
    UNDECIDED_COLOR = BottleColor.BLUE

    #: 起動した時点から判別しておくか。
    #:
    #: **キャリーボトルは 1本しかない。** 走行中に見えるラベルはその 1本だけなので、
    #: 「どのボトルか」を指示してもらう必要がない。見えた時点で決めてよい。
    #:
    #: True にすると、シナリオの SCRIPT を待たずに読み取りの開始と同時に判別へ入る。
    #: **要求の往復（最大 40ms）と、SCRIPT の区間へ入るのを待つ時間がまるごと消える。**
    #: LAPゲートを最速で駆け抜けながら読むときは、区間に入ってから数え始めていては
    #: 通り過ぎてしまう。
    #:
    #: **既定は False。** True にすると走り出す前から判別が生きるため、コースの後ろに
    #: 立つ人の赤い服や、床の模様を「ボトルの色」として確定してしまう。実際、
    #: 3枚（0.1秒）続けて見えれば決まってしまい、そのあと本物のラベルを見ても変わらない。
    #: シミュレータには「物理的に抱えている間だけ確定する」歯止めがあるが、実機には無い。
    #:
    #: **判別する区間はシナリオで限定すること。** ラベルが見えるのは 449〜249mm 手前
    #: なので、その手前から SCRIPT を書けば、要求の往復（最大40ms）は十分に吸収される。
    ALWAYS_JUDGE = False

    _instance: Optional["BottleColorTask"] = None
    _instance_lock = threading.Lock()

    @classmethod
    def get_instance(cls) -> "BottleColorTask":
        """走行体で共有する1つを返す。無ければ作る"""
        with cls._instance_lock:
            if cls._instance is None:
                cls._instance = cls()
            return cls._instance

    @classmethod
    def reset_instance(cls) -> None:
        """共有している1つを止めて捨てる（テスト用）"""
        with cls._instance_lock:
            old, cls._instance = cls._instance, None
        if old is not None:
            old.stop()

    def __init__(self):
        self._reader: Optional[BottleColorReader] = None
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self._stop = threading.Event()

        #: 判別中か（SCRIPT で judge() が呼ばれてから、色が決まるか打ち切るまで）
        self._judging = False
        #: 決まった色。次の judge() まで変わらない
        self._decided: Optional[str] = None
        #: 色が決まったときに運び先のシナリオを差し替えるか（点検では False）
        self._replace_route = True
        #: 確定待ちの色と、その色が続けて見えた回数
        self._stable_color: Optional[str] = None
        self._stable_count = 0
        #: 判別を始めてから色が見えた回数（確定までにどれだけ余裕があったか）
        self._seen_count = 0
        #: judge() を呼ぶたびに増やす。前の判別の打ち切りが遅れて来ても無視するため
        self._generation = 0
        #: 打ち切りのタイマー
        self._timer: Optional[threading.Timer] = None

    def start(self) -> bool:
        """
        読み取りを起こす。すでに動いていれば何もしない。

        :returns: 起こしたら True
        """
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                return False
            self._stop.clear()
            self._reader = self._make_reader()
            self._thread = threading.Thread(target=self._loop, name="bottle-color", daemon=True)
            self._thread.start()
        logger.info("キャリーボトルの色の読み取りを開始しました")

        if self.ALWAYS_JUDGE:
            # シナリオの SCRIPT を待たずに判別へ入る。ボトルは1本なので、
            # 見えた時点で決めてよい（上の ALWAYS_JUDGE を参照）
            self.judge(request_id=0)
        return True

    def _make_reader(self) -> BottleColorReader:
        """判定に使う色（JUDGE_COLORS）だけを数える読み取り器を作る"""
        monitor = BottleColorMonitor(options={'colors': list(self.JUDGE_COLORS)})
        return BottleColorReader(monitor=monitor)

    def stop(self) -> None:
        """読み取りを止める。走行の終わりに呼ぶ（呼ばなくても daemon なので落ちる）"""
        self._stop.set()
        with self._lock:
            self._cancel_timer()
            if self._reader is not None:
                self._reader.close()

    def _loop(self) -> None:
        """カメラから 1枚ずつ読んで、見えている色を覚えさせる

        **読めているあいだは待たない。** read() がカメラの速度でブロックするので、
        それが周期になる（MJPG 1280x720 で約33ms＝30Hz）。ここで待ちを足すと、
        待っているあいだに届いたぶんが溜まり、次に読むのは古い映像になる。

        映像が取れなかったときだけ間を置く。カメラが開けていないのに全速で
        回すと、失敗だけを秒間数千回繰り返すことになるため。
        """
        while not self._stop.is_set():
            t0 = time.perf_counter()
            try:
                self._decide(self._reader.read())
            except Exception as e:  # noqa: BLE001 — 読めなくても走行は続ける
                logger.warning("ラベル色の読み取りに失敗しました: %s", e)
                # 続けて失敗し続けるとログが埋まるので、少し待ってから戻る
                self._stop.wait(self.REOPEN_INTERVAL)
                continue

            # 足りないぶんだけ待つ。**一定時間の待ちを毎回足してはいけない。**
            #
            # カメラから読めているときは read() が1フレーム分ブロックするので、
            # ここは残り 0 以下になり、待たずに次へ行く（実測 29.9Hz）。
            # 毎回 1/30 秒を足すと、待っているあいだに届いたぶんが溜まり、
            # 次に読むのは古い映像になる（実測 18.8Hz、read() が 11.5ms で返る）。
            #
            # 待ちが要るのは、read() が待たずに返る場合。カメラを開けていない
            # ときと、取り込み口を差してあるとき（テスト）で、どちらも待たないと
            # 全速で空回りする。
            # **端数では待たない。** Event.wait は数ミリ秒を渡しても、その通りには
            #  起きない。1周 32ms のところへ残り 1ms を渡したら 46ms になった
            #  （2026-09-08 実測、31Hz のはずが 21Hz）。まとまった時間のときだけ待つ
            remaining = self.MAX_INTERVAL - (time.perf_counter() - t0)
            if remaining >= self.MIN_SLEEP:
                self._stop.wait(remaining)

    def judge(self, request_id: int = 0, replace_route: bool = True) -> None:
        """
        ラベル色の判別を始める。シナリオの SCRIPT（2026/recog/bottle_color.py）から呼ぶ。

        **その場では決めない。** 呼ばれた時点ではまだ遠くてラベルが小さいことも、
        逆に近すぎて画角から外れていることもある。判別中にしておき、読み取りの
        周期で同じ色が続けて見えたときに確定させる（下の _decide）。
        JUDGE_TIMEOUT のあいだに決まらなければ打ち切る（下の _expire）。

        前の判別で決まっていた色はここで捨てる。次のボトルに持ち越さないため。

        :param replace_route: 色が決まったときに運び先のシナリオを差し替えるか。
                          点検のシナリオには運び先が無いので False にする
                          （2026/recog/bottle_color_check.py）
        """
        with self._lock:
            self._judging = True
            self._replace_route = replace_route
            self._decided = None
            self._stable_color = None
            self._stable_count = 0
            self._seen_count = 0
            self._generation += 1
            self._cancel_timer()
            self._timer = threading.Timer(self.JUDGE_TIMEOUT, self._expire,
                                          args=(self._generation,))
            self._timer.daemon = True
            self._timer.start()
        logger.info("ラベル色の判別を開始しました (request_id=%s)", request_id)

    def _decide(self, color: str) -> None:
        """
        読み取り 1回分を判別に反映する。**判別中だけ働く。**

        ちらつきで決めてしまわないよう、同じ色が続けて見えたときだけ確定する
        （30Hz なので 3回で 0.1秒。青だけ 6回＝0.2秒。_stable_frames）。
        """
        if color not in self.JUDGE_COLORS:
            color = BottleColor.NONE        # 判定しない色は、見えていないのと同じ

        with self._lock:
            if not self._judging:
                return
            if color == BottleColor.NONE:
                self._stable_color = None
                self._stable_count = 0
                return
            self._seen_count += 1
            if color == self._stable_color:
                self._stable_count += 1
            else:
                self._stable_color = color
                self._stable_count = 1
            if self._stable_count < self._stable_frames(color):
                return
            self._judging = False
            self._decided = color
            self._cancel_timer()
            seen_frames = self._seen_count
            replace_route = self._replace_route

        # **何枚見えて決まったかを残す。** 速く走るほど写る枚数が減る。
        # ここが JUDGE_STABLE_FRAMES ぎりぎりなら、その速度では余裕が無い
        logger.info("ラベル色を確定しました: %s（%d枚見えて確定／必要 %d枚）",
                    color, seen_frames, self._stable_frames(color))

        # **決めた瞬間の1枚を残す。** 定期保存は1秒ごとなので、0.1秒で決まる瞬間を
        # 取り逃がす。背景の色で決めてしまったときに、後から確かめられるようにする
        if self._reader is not None:
            try:
                self._reader.save_snapshot(color)
            except Exception as e:  # noqa: BLE001 — 残せなくても走行は続ける
                logger.warning("確定時の画像を残せませんでした: %s", e)

        if replace_route:
            self._place_route(color)

    @classmethod
    def _stable_frames(cls, color: str) -> int:
        """その色を確定させるのに要る枚数。色ごとの指定が無ければ JUDGE_STABLE_FRAMES"""
        return cls.JUDGE_STABLE_FRAMES_BY_COLOR.get(color, cls.JUDGE_STABLE_FRAMES)

    def _expire(self, generation: int) -> None:
        """
        JUDGE_TIMEOUT のあいだに色が決まらなかった。判別を打ち切り、UNDECIDED_COLOR へ運ぶ。

        :param generation: タイマーを起こしたときの判別の番号。いまの判別と違えば何もしない
        """
        with self._lock:
            if not self._judging or generation != self._generation:
                return
            self._judging = False
            self._timer = None
            replace_route = self._replace_route

        logger.warning("%.1f秒で色が決まらなかったため判別を打ち切りました", self.JUDGE_TIMEOUT)
        if replace_route:
            self._place_route(self.UNDECIDED_COLOR)

    def _cancel_timer(self) -> None:
        """打ち切りのタイマーを止める。_lock を持って呼ぶ"""
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None

    def _place_route(self, color: str) -> None:
        """色の運び先へ、走行中のシナリオを差し替える。置けなくても走行は続く"""
        route = self._routes().get(color)
        if route is None:
            logger.warning("%s の運び先が設定されていません（既定のルートを走ります）", color)
            return
        logger.info("%s の運び先を置きます", color)
        ScenarioMailbox.place(route)

    def _routes(self) -> dict:
        """色 → 運び先シナリオのパス（config_2026 の BOTTLE_ROUTE_FILES）"""
        try:
            import config_2026
            return getattr(config_2026, "BOTTLE_ROUTE_FILES", {})
        except ImportError:
            return {}

    def is_reading(self) -> bool:
        """
        映像の読み取りが回っているか。

        ヒントカードの読み取り（`2026/hint_card_capture.py`）が、自分でカメラを
        読みにいってよいかを決めるのに見る。回っているあいだは、その最後の1枚を
        借りる（`VideoCapture` を触るのは1つだけにする。`camera.py` を参照）。
        """
        return self._thread is not None and self._thread.is_alive()

    def held_color(self) -> str:
        """
        確定している色。まだなら 'none'（打ち切って青へ運んだときも 'none'）。

        C++ 側（ResultRead）はこの名前を CarryBottleColor の番号へ写す。
        番号ではなく名前で渡すのは、色の呼び名を Python と C++ の 1か所ずつに
        留めるため。
        """
        return self._decided or BottleColor.NONE

    def fill_result(self, result: dict) -> None:
        """判定結果の応答へ、確定している色を足す"""
        result["bottle_color"] = self.held_color()
