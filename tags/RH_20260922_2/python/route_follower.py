"""
受け取った経路を、**自己位置を見ながら走らせる**（開ループをやめる）。

    PC の経路 ──▶ route_register ──▶ C++ へ渡す（SW_ROUTE の印の直後へ入る）
                        │
                        └─▶ RouteFollower.begin()
                                │  入り口の姿勢から節点（居るべき場所）を作る
                                ▼
              /dev/shm/sorot_pose.json（100ms） ──▶ 行が変わるたびに開きを見る
                                                       │ 大きければ
                                                       ▼
                                  先の行の SCD / SCR を計算し直して置く
                                  /dev/shm/sorot_scenario_update.json

いままで経路は `SCD`（距離）と `SCR`（角度）だけの**開ループ**で、自己位置を
打ち直しても走り方は何も変わらなかった（`Docs/position_correction_design.md` 5.7 穴2）。
ここが、直した自己位置と走り方をつなぐ1本目の道である。

【仕事を分けてある】

| 何を | どこで |
| --- | --- |
| 居るべき場所を作る | `route_plan.py`（走行後の `Tools/compare_route.py` と同じもの） |
| どう直すかを決める | `route_correct.py`（母艦で試せる。ファイルも時計も触らない） |
| 何番の行かを知る | `route_numbering.py` |
| 自己位置を読む | `pose.py`（置く側が持つ）＋ `pose_reader.py`（形を合わせるだけ） |
| **見張って置く（ここ）** | 別スレッド。読み・判断・置き・記録をつなぐだけ |

【行の組み立ては、ここに一本化する】
**C++ は同じ SNO/CNO の行を丸ごと置き換える。** 向きの打ち直し・位置の打ち直し・
走り方の書き直しが別々に行を組み立てると、あとから置いたほうが前のぶんを消す。
打ち直す側は `set_coord_correct()` を呼ぶこと（`Docs/route_plan_follow.md` 6.4）。

【止まらないこと】
自己位置が置かれない C++（古い版）でも、経路が予定と違う形でも、**黙って
何もしないだけ**で経路はそのまま走る。補正は「効けば良くなる」もので、
「無いと走れない」ものにはしない。

【記録】
判断は採ったものも見送ったものも `route_rewrite.csv` に残す。走行後に
`Tools/compare_route.py --rewrites route_rewrite.csv` で、**書き直しで開きが
縮んだかを数字で見る**。残さないと「効いたのか、たまたま合ったのか」が
永久に分からない（`Docs/position_correction_design.md` 6章）。
"""

import csv
import json
import logging
import os
import threading
import time

from pose_reader import PoseSnapshotReader, POSE_POLL_INTERVAL
import route_correct
from route_correct import RouteCorrector
from route_numbering import RouteNumbering, SCENARIO_PATH
from route_plan import PlannedRoute
from scenario_update import ScenarioMailbox

logger = logging.getLogger(__name__)

#: 書き直してから、次に判断するまでに空ける行数。
#: **毎行直すと振動する。** 直した区間（旋回＋直進＋旋回）を走り終えてからでないと、
#: 直した結果が自己位置に現れない
MIN_ROWS_BETWEEN = 4

#: 書き直しの記録。走行体では asp と同じ場所（trajectory.csv の隣）へ置く
REWRITE_LOG_PATH = "./sorot_spike/route_rewrite.csv"

#: 走った経路をそのまま残す場所。
#:
#: **走行後の突き合わせに要る。** `/dev/shm/sorot_route.json` は C++ が引き取ると
#: 消えるので、走り終わったあとには残っていない。ここに残しておけば
#: `Tools/compare_route.py --route route_used.json` がそのまま通る
#: （シナリオ（custom.json）を渡しても比べ物にならない。あれは経路ではない）
ROUTE_USED_PATH = "./sorot_spike/route_used.json"

#: 向きを載せられる行を、どこまで先まで探すか[行]
SEARCH_HEADING_ROWS = 10

#: 自己位置の打ち直しのビット（`Scenario/CommandSet.h` の ACAF）
ACAF_X = 0x01
ACAF_Y = 0x02
ACAF_HEADING = 0x04

#: 記録の列。`Tools/compare_route.py` が読む。
#: **`t_us` は走行体の時計[us]**（`get_tim`。trajectory.csv の `t_ms` と同じ値で、
#: あちらは ms.us に割ってから出している）。突き合わせるときに使う
#:
#: **`x` / `y` / `head` は、H がその判断に実際に使った姿勢。**
#: 走行後に `trajectory.csv` と突き合わせて「同じものを見ていたか」を確かめる。
#: 2026-09-21、判断時の開きが走行後の分析と食い違ったが、記録の姿勢を流し込むと
#: 一致したため、**受け取っていた姿勢そのものを疑う**ところで止まっていた
LOG_COLUMNS = ["kind", "t_us", "sno", "cno", "row",
               "gap_mm", "along_mm", "cross_mm", "head_deg", "rows", "detail",
               "x", "y", "head"]


def beside_log(path):
    """走行体で書ける場所に直す（asp と同じ場所が無ければカレントへ）"""
    if os.path.isdir(os.path.dirname(path) or "."):
        return path
    return "./" + os.path.basename(path)


def mark_open_loop(note=""):
    """
    **書き直しをしない走行にも、記録の区切りを入れる。**

    入れないと `Tools/compare_route.py --rewrites` が前の走行の節を拾い、
    書き直していない走行を書き直した扱いで比べる（2026-09-21 に発生。
    元の値と差し替えた値が入れ替わり、行ごとの差が 50mm ずれて見えた）。
    """
    log = RewriteLog()
    label = "開ループ %s" % time.strftime("%Y-%m-%d %H:%M:%S")
    if note:
        label += " " + note
    if not log.open(label):
        return False
    log.close()
    return True


def save_route(rows, path=None):
    """走った経路をそのまま残す。**書けなくても走行は止めない**"""
    path = beside_log(path or ROUTE_USED_PATH)
    try:
        with open(path, "w", encoding="utf-8") as f:
            json.dump({"scenario": rows}, f, ensure_ascii=False)
    except (OSError, ValueError) as e:
        logger.warning("走った経路を残せません(%s): %s", path, e)
        return None
    logger.info("走った経路を残しました: %s（%d行）", path, len(rows))
    return path


class RewriteLog:
    """書き直しの記録（CSV）。**書けなくても走行は止めない**"""

    def __init__(self, path=None):
        self.path = beside_log(path or REWRITE_LOG_PATH)
        self._writer = None
        self._file = None

    def open(self, label):
        """走行の区切り（`#` の行）を入れて書き始める。trajectory.csv と同じ形"""
        try:
            self._file = open(self.path, "a", encoding="utf-8", newline="")
        except OSError as e:
            logger.warning("書き直しの記録を開けません(%s): %s", self.path, e)
            return False
        self._writer = csv.writer(self._file)
        self._file.write("# %s\n" % label)
        self._writer.writerow(LOG_COLUMNS)
        self._file.flush()
        return True

    def close(self):
        if self._file is not None:
            try:
                self._file.close()
            except OSError:
                pass
        self._file = None
        self._writer = None

    def write(self, kind, snapshot, row=None, decision=None, detail=""):
        if self._writer is None:
            return
        try:
            self._writer.writerow([
                kind,
                snapshot.t_ms, snapshot.sno, snapshot.cno,
                "" if row is None else row,
                "" if decision is None else "%.1f" % decision.gap_mm,
                "" if decision is None else "%.1f" % decision.along_mm,
                "" if decision is None else "%.1f" % decision.cross_mm,
                "" if decision is None else "%.2f" % decision.head_deg,
                "" if decision is None else ";".join(str(i) for i in decision.rows),
                detail,
                "%.1f" % snapshot.pose.x, "%.1f" % snapshot.pose.y,
                "%.2f" % snapshot.pose.heading_deg,
            ])
            self._file.flush()
        except (OSError, ValueError) as e:
            logger.warning("書き直しを記録できません: %s", e)


class RouteFollower:
    """経路を自己位置と突き合わせ、先の行を書き直す。走行体で1つだけ動く"""

    _instance = None
    _instance_lock = threading.Lock()

    @classmethod
    def get_instance(cls):
        with cls._instance_lock:
            if cls._instance is None:
                cls._instance = cls()
            return cls._instance

    @classmethod
    def reset_instance(cls):
        """共有している1つを捨てる（テスト用）"""
        with cls._instance_lock:
            if cls._instance is not None:
                cls._instance.stop()
            cls._instance = None

    def __init__(self, reader=None, corrector=None, mailbox=None, log=None,
                 route_used_path=None):
        self.reader = reader or PoseSnapshotReader()
        self.corrector = corrector or RouteCorrector()
        self.mailbox = mailbox or ScenarioMailbox
        self.log = log or RewriteLog()
        self.route_used_path = route_used_path
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._loop_thread = None
        self._clear()

    def _clear(self):
        self.plan = None
        self.numbering = None
        #: 入り口の姿勢から積んだ節点。**元の経路で積む**（目指す先は動かさない）
        self.nodes = None
        self._last_key = None
        self._entered = False
        self._finished = False
        self._next_decision_row = 1
        self._marker_pose = None        # 印の行で見た姿勢。これを経路の原点にする
        self._pending = None            # 置いたが、まだその行に入っていない書き直し
        self._pending_at = 0.0          # 置いた時刻（こちらの時計。単位の取り違えを避ける）

    # --- 走り出し -------------------------------------------------------------

    def begin(self, scenario_rows, rough_spot_id, scenario_path=None):
        """
        経路を C++ へ渡した直後に呼ぶ（`route_register` から）。

        **ここでは見張りのスレッドを起こさない**（母艦のテストが1回ずつ進められる
        ように分けてある）。走行体からはモジュールの `begin()` を呼ぶこと。

        :param scenario_rows: C++ へ渡したのと**同じ**コマンドの配列
                              （向き保持直進への差し替え後のもの）
        :param rough_spot_id: 難所の識別値（`SW_ROUTE` の `SCV`）
        :returns: 見張る用意ができたら True
        """
        plan = PlannedRoute.from_rows(scenario_rows)
        if len(plan) == 0:
            return False

        numbering = RouteNumbering.predict_from_file(
            scenario_path or SCENARIO_PATH, rough_spot_id)
        if numbering is None:
            # 印が読めなくても、走り出しの観測（印の次の行）で拾える見込みはある。
            # ただし印そのものが分からないので、そこは諦める
            logger.info("経路の入り口（SW_ROUTE の印）が手元のシナリオに見つかりません。"
                        "経路の書き直しはしません")
            return False

        with self._lock:
            self._clear()
            self.plan = plan
            self.numbering = numbering
            self.log.open("経路の書き直し %s %s"
                          % (time.strftime("%Y-%m-%d %H:%M:%S"), self.settings()))
        logger.info("経路を見張ります: %d行 %s  %s", len(plan), numbering, self.settings())
        return True

    def settings(self):
        """**その走行で実際に効いたしきい値。**

        ファイルを見ても、走り出したあとに書き換えたのか、古い `.pyc` を読んだのかが
        分からない（現地で1度ずつ両方に引っかかった）。走行ごとの記録に残す。
        """
        return ("しきい値 開き%.0fmm/向き%.1f度 先読み%d行 間隔%d行"
                % (route_correct.MIN_GAP_MM, route_correct.MIN_HEAD_DEG,
                   self.corrector.lead_rows, MIN_ROWS_BETWEEN))

    def stop(self):
        """見張るスレッドを止める（走行の終わりとテスト用）"""
        self._stop.set()
        thread = self._loop_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)
        self._loop_thread = None
        self.log.close()

    def start_file_loop(self, interval=None):
        """自己位置を見張る別スレッドを起こす。起こしたら True"""
        if self._loop_thread is not None and self._loop_thread.is_alive():
            return True
        self._stop.clear()
        wait = POSE_POLL_INTERVAL if interval is None else interval

        def _run():
            while not self._stop.is_set():
                try:
                    self.poll_once()
                except Exception as e:      # noqa: BLE001 — 補正が落ちても走行は止めない
                    logger.error("経路の書き直しに失敗しました: %s", e)
                self._stop.wait(wait)

        self._loop_thread = threading.Thread(target=_run, name="route-follow", daemon=True)
        self._loop_thread.start()
        return True

    # --- 見張り ---------------------------------------------------------------

    def poll_once(self):
        """自己位置を1回読む。**行が変わったときだけ**考える

        :returns: 行が変わって判断まで進んだら True
        """
        if self.plan is None or self._finished:
            return False
        snapshot = self.reader.read()
        if snapshot is None:
            return False

        with self._lock:
            key = snapshot.key
            if key == self._last_key:
                return False
            if key == (self.numbering.marker_sno, self.numbering.marker_cno):
                # **印の行は止まって経路を待っている。** ここの姿勢が経路の原点
                self._marker_pose = snapshot.pose
            previous, self._last_key = self._last_key, key
            return self._on_row_start(previous, snapshot)

    def _on_row_start(self, previous, snapshot):
        """行が変わった。経路の中なら開きを見る"""
        numbering = self.numbering

        # 印の次に走り出した行が経路の先頭。**予測より観測を優先する**
        if not numbering.observed and previous == numbering.marker_key:
            numbering.sno, numbering.first_cno = snapshot.key
            numbering.observed = True
            logger.info("経路の番号を観測で確かめました: %s", numbering)

        index = numbering.row_index(snapshot.key, len(self.plan))
        if index is None:
            if self._entered:
                self._finish(snapshot)
            return False

        self._entered = True
        if self.nodes is None:
            return self._enter(index, snapshot)

        self._check_arrival(index, snapshot)
        if index < self._next_decision_row:
            return False
        return self._consider(index, snapshot)

    def _enter(self, index, snapshot):
        """経路に入った。ここの姿勢が計画の原点になる"""
        if index != 0:
            # 先頭行を見逃した（自己位置が置かれ始めるのが遅かった等）。
            # 途中の姿勢を原点にすると「いまが計画どおり」と決めつけることになる
            logger.info("経路の先頭（%d行目）を見ていないので、書き直しはしません", index)
            self._finished = True
            return False
        entry = self._entry_pose(snapshot)
        self.nodes = self.plan.nodes(entry)
        self.log.write("start", snapshot, row=0,
                       detail="経路%d行 入り口 %s（%s）"
                              % (len(self.plan), entry,
                                 "印の行" if self._marker_pose is not None else "1行目の途中"))
        logger.info("経路の入り口: %s", entry)
        return False

    def _entry_pose(self, snapshot):
        """経路の原点にする姿勢

        **印の行（`SW_ROUTE`）で見た姿勢を使う。** そこは `FWD 0` で止まって
        経路を待っているので姿勢が汚れない。経路の1行目はその姿勢から始まる。

        1行目に入ってから読むと、**自己位置は 100ms ごとにしか置かれない**ぶん
        遅れる。1行目が旋回だと、その間に向きが数度 回っている。**原点の向きが
        ずれると経路全体がその角度だけ回り**、始点から 3m の行では
        3m × sin(3.6度) ＝ 188mm ずれる（2026-09-21 実機。H が見ていた開きが
        走行後の分析と 27mm 対 106mm で食い違った原因）。

        印の行を見ていなければ、これまでどおり1行目の姿勢で我慢する。
        """
        return self._marker_pose if self._marker_pose is not None else snapshot.pose

    @staticmethod
    def now():
        """こちらの時計[s]。**走行体の `t_ms` は使わない**

        置く側（`PoseSample.cpp`）が `get_tim()` の値をそのまま入れていて、
        その単位は**マイクロ秒**である（軌跡ログは同じ値を `tUs` として持ち、
        1000 で割ってから出している。`TrajectoryLogger.cpp`）。余裕の桁が
        1000倍になるので、こちらで測る。
        """
        return time.monotonic()

    def _check_arrival(self, index, snapshot):
        """置いた書き直しが、その行に**間に合ったか**を記録する

        余裕[ms] = その行に入ったと**気づいた**時刻 − 置いた時刻。自己位置は
        100ms ごとに置かれ、こちらは 50ms ごとに見るので、実際の余裕より
        最大 150ms ほど長く出る。**秒の単位で見ること。**
        """
        pending = self._pending
        if pending is None or index < pending.rows[0]:
            return
        self.log.write("arrive", snapshot, row=pending.rows[0],
                       detail="余裕 %dms" % round((self.now() - self._pending_at) * 1000.0))
        self._pending = None

    def _consider(self, index, snapshot):
        """開きを見て、必要なら先の行を書き直す"""
        decision = self.corrector.decide(self.plan, self.nodes, index, snapshot.pose)
        if not decision.accepted:
            self._next_decision_row = index + 1
            self.log.write("skip", snapshot, row=index, decision=decision,
                           detail=decision.reason)
            return True

        # **入れてから組む。** 行は経路の値を写すので、順を逆にすると
        # 直す前の値が置かれる
        decision.apply(self.plan)
        rows = self._emit(decision)
        if not self.mailbox.place_rows(rows):
            decision.revert(self.plan)
            self._next_decision_row = index + 1
            self.log.write("skip", snapshot, row=index, decision=decision,
                           detail="置けなかった（前の更新が残っている）")
            return True

        self._pending = decision
        self._pending_at = self.now()
        self._next_decision_row = max(decision.last_row + 1, index + MIN_ROWS_BETWEEN)
        self.log.write("apply", snapshot, row=index, decision=decision,
                       detail=decision.describe())
        logger.info("経路を書き直しました: %d行目で開き %.0fmm 向き %+.1f度 → %s",
                    index, decision.gap_mm, decision.head_deg, decision.describe())
        return True

    # --- 他の補正から呼ぶ口（行の組み立てはここに一本化する） -----------------

    def route_row_index(self, key):
        """`(SNO, CNO)` が経路の何行目か。経路の外なら None

        打ち直す側は要求に載ってきた `SNO`/`CNO` を持っている。**それが経路の中なら
        行の組み立てをこちらへ寄せる**（外なら `custom.json` の行なので、こちらは
        何も知らない）。
        """
        with self._lock:
            if self.plan is None or self.numbering is None:
                return None
            return self.numbering.row_index(key, len(self.plan))

    def rows_ahead(self, count, current_key=None):
        """いま走っている行の `count` 行先。経路に入る前は先頭からの番号

        打ち直す側は「何行先に書くか」で考える（`LOOKAHEAD_ROWS`）ので、
        そこから経路の行番号へ直すのはこちらが持つ。

        :param current_key: いま走っている `(SNO, CNO)`。渡せば自己位置に頼らない
        """
        with self._lock:
            current = self._current_row(current_key)
        return (0 if current is None else current) + count

    def next_heading_row(self, min_ahead=1, search=None, current_key=None):
        """向きの打ち直しを載せられる、いちばん近い行。無ければ None

        **いまの行からその行まで、ひとつも向きが変わってはいけない**
        （`_keeps_heading_through`）。呼ぶ側が「何行先」だけで決めると、
        たいてい旋回に当たって載せられない。ここで探す。

        :param min_ahead: 最低でも何行先か（間に合う行数。実機で測る）
        """
        with self._lock:
            if self.plan is None:
                return None
            current = self._current_row(current_key)
            first = (0 if current is None else current + 1) + max(0, min_ahead - 1)
            limit = min(len(self.plan), first + (search or SEARCH_HEADING_ROWS))
            for index in range(first, limit):
                if self._keeps_heading_through(index, current):
                    return index
        return None

    def set_coord_correct(self, row_index, acaf, capx=None, capy=None, capr=None,
                          current_key=None):
        """
        自己位置の打ち直し（`CoordCorrectInfo`）を、経路の行に載せて置く。

        **同じ SNO/CNO の行は丸ごと置き換わる**（`Scenario::insertCommandSet`）。
        向きの打ち直し・位置の打ち直し・走り方の書き直しが**別々に行を組み立てると
        互いに消し合う**ので、組み立てはここに寄せる。ここで置けば、その行に
        入っている `SCD`/`SCR` の書き直しもそのまま残る。

        何度呼んでもよい。`ACAF` のビットは足し合わせ、渡した値だけ書き換える
        （向きと位置が別々のときに来ても、1つの行に両方載る）。

        **渡した値はそのまま置く。** 絶対値（`CAPX`/`CAPY`/`CAPR`）なので、
        その行に着くころに合っているかは呼ぶ側の責任になる。

        :param row_index: 経路の行番号（`rows_ahead()` で作る）
        :param acaf:      直す項目のビット（`ACAF_X` / `ACAF_Y` / `ACAF_HEADING`）
        :param current_key: いま走っている `(SNO, CNO)`。渡せば自己位置に頼らない
        :returns: 置けたら True
        """
        with self._lock:
            row = self._coord_target(row_index, acaf, current_key)
            if row is None:
                return False

            correct = dict(row.raw.get("CoordCorrectInfo") or {})
            correct["ACAF"] = int(correct.get("ACAF") or 0) | int(acaf)
            for key, value in (("CAPX", capx), ("CAPY", capy), ("CAPR", capr)):
                if value is not None:
                    correct[key] = int(round(value))
            row.raw["CoordCorrectInfo"] = correct

            numbers = self.numbering.numbers_for(row_index)
            if not self.mailbox.place_rows([row.emit(*numbers)]):
                logger.info("打ち直しを置けませんでした（前の更新が残っている）: %d行目",
                            row_index)
                return False

        logger.info("自己位置の打ち直しを %d行目（%d/%d）へ載せました: %s",
                    row_index, numbers[0], numbers[1], correct)
        return True

    def _coord_target(self, row_index, acaf, current_key=None):
        """打ち直しを載せてよい行か。だめなら None（理由をログに出す）"""
        if self.plan is None or self.numbering is None:
            logger.info("経路を見ていないので打ち直しを載せられません")
            return None
        if not 0 <= row_index < len(self.plan):
            logger.info("経路の外なので打ち直しを載せられません: %d行目", row_index)
            return None

        current = self._current_row(current_key)
        if self._finished or (current is not None and row_index <= current):
            logger.info("いま走っている行（%s）より先ではないので載せません: %d行目",
                        current, row_index)
            return None

        if (int(acaf) & ACAF_HEADING) != 0 and not self._keeps_heading_through(row_index, current):
            logger.info("いまの行から %d行目までに向きの変わる行があるので載せません",
                        row_index)
            return None
        return self.plan.rows[row_index]

    def _keeps_heading_through(self, row_index, current):
        """いまの行から `row_index` まで、ひとつも向きが変わらないか

        2つの理由で要る。

        * **測ってから効くまでに向きが変われば、打ち直す値が嘘になる**
          （旋回・カーブ・ライントレースを挟んだら諦める）
        * **`row_index` そのものも旋回であってはいけない。** `ACAF 0x04` は
          `driveDirection` も置き換えるが、旋回はその行に入った時点の
          `driveDirection` を基準に止める。しかも打ち直しは StatusMonitor へ、
          走り方は Navigator へ**別のタスクに**送られる（`ScenarioControl::nextScenario`）
          ので、同じ行でもどちらが先に効くかは決まっていない
        """
        first = 0 if current is None else current
        return all(self.plan.rows[i].keeps_heading for i in range(first, row_index + 1))

    def _current_row(self, current_key=None):
        """いま走っている経路の行番号。経路の外なら None

        呼ぶ側が `(SNO, CNO)` を知っていればそれを使う。自己位置が置かれない
        C++ でも、打ち直しの行き先は決められる
        """
        key = current_key if current_key is not None else self._last_key
        if key is None or self.numbering is None:
            return None
        return self.numbering.row_index(key, len(self.plan))

    def _emit(self, decision):
        """書き直す行を、番号ごと丸ごと組む"""
        out = []
        for row_index in sorted(set(decision.rows)):
            sno, cno, prv_sno, prv_cno = self.numbering.numbers_for(row_index)
            out.append(self.plan.rows[row_index].emit(sno, cno, prv_sno, prv_cno))
        return out

    def _finish(self, snapshot):
        """経路を走り終えた（別のシナリオ番号へ移った）"""
        self._finished = True
        self.log.write("end", snapshot, detail="経路を出ました")
        self.log.close()
        logger.info("経路を走り終えました")


# --------------------------------------------------------------------------
# 走行体で使う1つへの入り口（`route_register` から呼ぶ）
# --------------------------------------------------------------------------


def begin(scenario_rows, rough_spot_id, scenario_path=None):
    """経路を渡した直後に、見張りを起こす（用意して、別スレッドを回し始める）"""
    follower = RouteFollower.get_instance()
    if not follower.begin(scenario_rows, rough_spot_id, scenario_path):
        return False
    return follower.start_file_loop()


def stop():
    RouteFollower.get_instance().stop()


def rows_ahead(count):
    """いま走っている行の `count` 行先（打ち直す側が「何行先」で考えるため）"""
    return RouteFollower.get_instance().rows_ahead(count)


def route_row_index(key):
    """`(SNO, CNO)` が経路の何行目か。経路の外なら None"""
    return RouteFollower.get_instance().route_row_index(key)


def next_heading_row(min_ahead=1, current_key=None):
    """向きの打ち直しを載せられる、いちばん近い行（`RouteFollower.next_heading_row`）"""
    return RouteFollower.get_instance().next_heading_row(
        min_ahead, current_key=current_key)


def set_coord_correct(row_index, acaf, capx=None, capy=None, capr=None, current_key=None):
    """自己位置の打ち直しを経路の行へ載せて置く（`RouteFollower.set_coord_correct`）"""
    return RouteFollower.get_instance().set_coord_correct(
        row_index, acaf, capx, capy, capr, current_key)
