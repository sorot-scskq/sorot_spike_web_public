"""
走行体 Python の親。C++ 部が置いた要求を読み、認識や PC とのやりとりへ回す。

    C++（PythonCommServer） ── /dev/shm/sorot_comm_req.json ──▶ Ev3Python.run()
                           ◀─ /dev/shm/sorot_comm_res.json ──

**C++ 部とはソケットで繋がない。** 同じ機械の中の話で、運ぶのは識別値ひとつと
判定結果の写しだけなので、ファイルを置き合うほうが素直で強い。接続状態が無いため、
どちらが先に起動しても、途中で落ちても、生きている側は動き続ける。

**PC（無線通信デバイス）とは WebSocket。** 別の機械なので通信が要る
（`pc_client.py`。以前は TCP 49661 だった）。

リモートコントロール走行のやりとり（10ms 周期）も同じくファイルで、このループとは
別のスレッドが受け持つ（`RemoteControl.start_file_loop`）。
"""
import json
import os
import subprocess
import threading
import time
import logging
from config import (
    load_season, setup_logging, PC_WS_URL,
    COMM_REQ_FILE, COMM_RES_FILE, COMM_RES_TMP,
    COMM_POLL_INTERVAL, COMM_RESULT_REFRESH,
    JUDG_MINIFIGURE, JUDG_PLARAIL, JUDG_BOTTLE, JUDG_GATE,
    JUDG_QR1, JUDG_QR2, JUDG_ET_RALLY, CMD_GET_RESULT
)
from pc_client import PCClient
from state import ResultStore
import route_register
from robot_pose import RobotPose
from script_runner import ScriptRunner
from handler import PcExchange
import remote_control

# ログ初期設定の実行
setup_logging()
# その年の設定（使う識別値と待ち時間）を重ねる。
# 差し込み口（start_recognizers / handle_request / fill_result）もここから引く。
# 共通側が年のフォルダを名指しで import しないための一本道（config.py を参照）
SEASON = load_season()
logger = logging.getLogger(__name__)


class Ev3Python:
    """
    C++ 部からの要求を捌く親。走行体で1つだけ動く。

    受け渡しのファイルと周期は作るときに差し替えられる（テストは一時ディレクトリへ
    向ける）。**相手の生死に関わらず回り続ける**ので、C++ 部が落ちても上がり直せる。
    """

    #: C++ 部（asp）の生存を確かめる間隔[s]。毎周期 pgrep を呼ぶと重い
    ASP_CHECK_INTERVAL = 5.0

    #: 起動直後は確かめない[s]。asp が立ち上がりきる前に終わってしまわないため
    ASP_GRACE = 15.0

    #: 識別値 → 撮影回数。PC へ画像を送る判定はここを通る
    CAPTURES = {
        JUDG_MINIFIGURE: 1,     # ミニフィグ判定
        JUDG_PLARAIL: 5,        # プラレール判定（5回連続で撮る）
        JUDG_BOTTLE: 1,         # ボトル判定
        JUDG_GATE: 1,           # キャリーゲート判定
        JUDG_QR1: 1,            # ヒントカード1
        JUDG_QR2: 1,            # ヒントカード2
        JUDG_ET_RALLY: 1,       # ETラリー（PC 側が経路を計算して返す）
    }

    def __init__(self, req_file=None, res_file=None, res_tmp=None,
                 poll_interval=None, result_refresh=None):
        self.req_file = req_file or COMM_REQ_FILE
        self.res_file = res_file or COMM_RES_FILE
        self.res_tmp = res_tmp or COMM_RES_TMP
        self.poll_interval = COMM_POLL_INTERVAL if poll_interval is None else poll_interval
        self.result_refresh = COMM_RESULT_REFRESH if result_refresh is None else result_refresh
        #: その年の設定（差し込み口をここから引く）
        self.season = SEASON
        #: PC へ繋ぐ口。run() が作る
        self.pc_client = None

    # --- その年の差し込み口 ---------------------------------------------------

    def season_hook(self, name):
        """その年の差し込み口。無ければ None"""
        fn = getattr(self.season, name, None)
        return fn if callable(fn) else None

    # --- C++ との受け渡し -----------------------------------------------------

    def read_request(self, last_req_id):
        """C++ が置いた要求を読む。

        要求は 2通りある。

            {"script": "2026/recog/bottle_color.py", "request_id": 3,
             "x": 1234.5, "y": 567.8, "heading": 135.2, "sno": 30, "cno": 12}
                                                                    スクリプトを実行させる
                                                                    （座標はデバッグ画像の名前用）
            {"identifier": 2, "request_id": 3}                          共通の画像判定（従来）

        :returns: 新しい要求ならその中身（辞書）。無ければ None
        """
        try:
            with open(self.req_file, encoding="utf-8") as f:
                req = json.load(f)
        except FileNotFoundError:
            return None
        except (OSError, ValueError) as e:
            # 書きかけを読むことは無い（C++ 側が rename で置くため）。
            # それでも壊れていたら次の周期で読み直す
            logger.warning("要求ファイルを読めません: %s", e)
            return None

        request_id = req.get("request_id", 0)
        if request_id == last_req_id:
            return None                     # まだ同じ要求。処理済み
        return req

    def build_result(self):
        """いまの判定結果の写しを組む"""
        snapshot = ResultStore.get_instance().snapshot()
        # その年だけの項目を足す（走行体が自分で判定したものなど）
        fill_result = self.season_hook("fill_result")
        if fill_result is not None:
            try:
                fill_result(snapshot)
            except Exception as e:  # noqa: BLE001 — 積めなくても走行は続ける
                logger.error("その年の判定結果を積めませんでした: %s", e)
        return snapshot

    def write_result(self, snapshot):
        """判定結果を C++ が読めるところへ置く。

        一時ファイルへ書いてから rename する。rename は同じファイルシステム内では
        不可分なので、C++ 側が書きかけを読むことがない。ロックが要らないのはこのため。

        **書けなくても走行は止めない。**
        """
        body = json.dumps(snapshot)
        try:
            with open(self.res_tmp, "w", encoding="utf-8") as f:
                f.write(body)
            os.replace(self.res_tmp, self.res_file)
        except OSError as e:
            logger.warning("判定結果を書けません(%s): %s", self.res_file, e)

    # --- 相手の生死 -----------------------------------------------------------

    def asp_alive(self):
        """
        C++ 部（asp）が動いているか。

        **こちらは asp から起動される。** `&` を付けた子プロセスなので、asp が
        終わっても道連れにならず、そのまま残り続ける。次の走行で asp を起動すると
        古いこちらが残っていて、シリアルの取り合いで
        「port =X getdevice time out」になる（起動時の掃除で対処していたが、
        そもそも残さないほうがよい）。

        pgrep が無い・呼べないときは「生きている」とみなす。
        確かめられないことを理由に勝手に終わらない。
        """
        try:
            done = subprocess.run(["pgrep", "-x", "asp"],
                                  stdout=subprocess.DEVNULL,
                                  stderr=subprocess.DEVNULL)
            return done.returncode == 0
        except Exception:       # noqa: BLE001 — pgrep が無い環境
            return True

    # --- 起動 -----------------------------------------------------------------

    def start_pc_client(self):
        """
        PC へ繋ぐ口を作る。

        **ここで接続を待たない。** `connect()` は繋がるまで再試行するため、PC を
        立てていないと C++ 部の要求を1つも捌けなくなる。ボトルの色の判別のように
        走行体の中だけで完結する要求まで止まり、C++ 側からは黙って結果が来ない
        だけに見える。温めるのは別スレッドに任せ、ループは先へ進める。
        """
        self.pc_client = PCClient(PC_WS_URL)
        threading.Thread(target=self.pc_client.connect, name="pc-connect",
                         daemon=True).start()

        # 無線通信デバイスから受け取ったシナリオ（ETラリーの経路）を登録する。
        # **要求の応答も、頼んでいない便も、同じここを通る。** PC は接続した時点で
        # 経路を送ってくることもあるので、要求の往復の中だけで見ると取りこぼす
        self.pc_client.on_message(route_register.RouteRegister.get_instance().handle_message)
        return self.pc_client

    def start_season(self):
        """
        その年の認識処理を起こす（走行体のカメラを使うものなど）。

        ヒントカードの内容を PC へ送るのに使う口も、ここで渡しておく。
        **失敗しても走行は止めない。**
        """
        set_pc_client = self.season_hook("set_pc_client")
        if set_pc_client is not None:
            set_pc_client(self.pc_client)

        start_recognizers = self.season_hook("start_recognizers")
        if start_recognizers is not None:
            try:
                start_recognizers()
            except Exception as e:  # noqa: BLE001
                logger.error("その年の認識処理を起こせませんでした: %s", e)

    # --- 本体 -----------------------------------------------------------------

    def run(self):
        """
        要求を捌き続ける。**C++ 部が居なくなったら戻る。**

        残ると次の走行でカメラとシリアルを取り合い、「port =X getdevice time out」
        で起動できなくなる。
        """
        logger.info("Python起動")

        # 前回の走行の要求が残っていると、いきなり拾ってしまう
        try:
            os.remove(self.req_file)
        except OSError:
            pass

        # C++ 側が起動を待っている。まず空の結果を置いて、動いていることを示す
        self.write_result(self.build_result())

        self.start_pc_client()
        self.start_season()

        # リモートコントロール走行のやりとり。**このループとは別のスレッドで回す。**
        # 10ms 周期で観測が置き直されるので、撮影（capture_and_send）の待ちに
        # 巻き込まれると指令が保持時間（200ms）を過ぎて走行体が止まる
        remote_control.start_file_loop()

        last_req_id = 0
        last_written = None
        last_write_at = 0.0
        started_at = time.time()
        last_asp_check = started_at

        while True:
            try:
                now = time.time()
                if (now - last_asp_check >= self.ASP_CHECK_INTERVAL
                        and now - started_at >= self.ASP_GRACE):
                    last_asp_check = now
                    if not self.asp_alive():
                        logger.info("C++部（asp）が居なくなりました。Python部を終了します")
                        break

                req = self.read_request(last_req_id)

                # 中身が変わったとき、または一定時間ごとに置き直す。
                # 変わらなくても py_time を更新して、生きていることを示す
                snapshot = self.build_result()
                now = time.time()
                if snapshot != last_written or now - last_write_at >= self.result_refresh:
                    snapshot["py_time"] = int(now)
                    self.write_result(snapshot)
                    last_written = {k: v for k, v in snapshot.items() if k != "py_time"}
                    last_write_at = now

                if req is None:
                    time.sleep(self.poll_interval)
                    continue

                # 捌く前に覚える。捌くほうで失敗しても、同じ要求を拾い直さない
                last_req_id = self._request_id(req)
                self.handle_request(req)

            except Exception as e:  # noqa: BLE001 — 1件の失敗でループを止めない
                logger.error("通信エラー: %s", e)

        # 終了時の後始末（PC 側のみ。C++ 部とはファイルなので閉じるものが無い）
        self.pc_client.close()

    def handle_request(self, req):
        """要求1件を捌く。スクリプト指定と識別値のどちらでも来る"""
        request_id = self._request_id(req)

        # シナリオの RunInfo.SCRIPT で指定されたスクリプトを実行する。
        # 認識を足すときに共通側も C++ も触らずに済むよう、パスで呼ぶ
        script = req.get("script")
        if script:
            # 要求に載っている自己位置を覚える（デバッグ画像の名前に使う。robot_pose.py）
            RobotPose.get_instance().update_from_request(req)
            ScriptRunner.get_instance().run(script, request_id)
            return

        identifier = req.get("identifier")
        logger.info("C++部から受信: 識別値=%s, request_id=%s", identifier, request_id)

        # 判定結果の問い合わせ（99）はもう来ない。C++ 部は結果ファイルを
        # 直接読むため、往復が要らなくなった。古いシナリオ対策で無視する
        if identifier == CMD_GET_RESULT:
            return

        repeat = self.CAPTURES.get(identifier)
        if repeat is not None:
            PcExchange.capture_and_send(identifier, self.pc_client,
                                        request_id=request_id, repeat=repeat)
            return

        # 共通が知らない識別値は、その年へ回す（走行体の中だけで完結する要求など。
        # PC へは送らない）
        handle_request = self.season_hook("handle_request")
        if handle_request is None or not handle_request(identifier, request_id):
            logger.warning("未知の識別値を受信しました: %s", identifier)

    @staticmethod
    def _request_id(req):
        return req.get("request_id", 0) if isinstance(req, dict) else 0


def main():
    """走行体の入口（C++ 部が `&` を付けて起動する）"""
    Ev3Python().run()


if __name__ == "__main__":
    main()
