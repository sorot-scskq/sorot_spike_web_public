"""
ヒントカードを撮って読む（2026・ETラリー）。

ET ラリーのゲート配置は、コース上に立ててある 2枚のヒントカードを読まないと
分からない（規約 7.3.1、表 7-2）。カードは 5cm 四方の二次元コードで、走行体の
正面を向いて台に立っている。

シナリオのコマンドに書いて使う（`RunInfo.SCRIPT`）。

    "RunInfo": {"SCRIPT": "2026/recog/hint_card1.py", "FUNCNO": 5, "FWD": 0}

【1枚で諦めない】
**以前は最後の1枚を読むだけだった。** 走行体が止まっていても、ぶれ・光の向き・
露出で読めない枚がある。1枚で諦めると「読めなかった」で確定してしまうので、
`READ_TIMEOUT` のあいだ、新しい映像が来るたびに読み直す。

読むのは、まず**カードが写っている所だけ**を切り抜いて、拡大してから
（`qr_decoder`）。カードは映像のごく一部にしか写らないので、映像まるごとを
回すより桁違いに速く、小さく写っても読める。それでも読めないときだけ、
ときおり映像まるごとも試す。

【カメラは走行体に1台しか無い】
色の読み取り（`BottleColorTask`）が 30Hz で開きっぱなしにしているので、
**自分では撮りにいかない。** その最後の1枚を借りる。読み取りが動いていない
ときだけ自分で取る（`camera.grab_latest`）。

【寄りながら読む】
**止まってから読み始めるのでは遅い。** 読めるかどうかはコードが何画素で
写るかでほぼ決まり（`qr_decoder.py`）、止まった1か所の絵を100枚見ても
同じ絵しか来ない。接近の区間から `SCRIPT` を書けば、近づくにつれて大きく
写った絵を何十枚も試せるうえ、照り返しの位置も動く。

そのため、**同じカードの `SCRIPT` がもう一度来たら、読み直さずに粘る時間を
延ばす。** 接近の行と止まる行の両方に書いておけば、接近で読めなくても
止まった所まで続けて粘る。すでに読めていれば何もしない（読み直して
失敗すると、読めた結果を「読めなかった」で上書きしてしまう）。

【次のカードの読み取りが始まったら打ち切る】
1枚目を粘っているあいだにシナリオが2枚目の区間へ進むことがある。そのまま
粘らせると、1枚目の読み取りが2枚目のカードを読んでしまい、同じ内容が
2回登録される。**別のカードの** `capture()` が呼ばれた時点で前の読み取りは捨てる。

【読めなければ撮り直す】
`READ_TIMEOUT` で読めなければ、すぐには諦めず、`RETRY_TIMEOUT` のあいだ新しい
映像を読み直す（`RETRY_COUNT` 回）。3秒＋1.5秒×3回＝7.5秒で、止まる行（8秒）の中で終わる。
ピントが合うまでに時間がかかる映像でも、回数を重ねれば合った絵に当たる。
接近が長引いて止まる前に期限が来たときも、止まってからの映像で読める。

**読み方は変えない。** CLAHE・適応的二値化で読み直す案は、合成画像で読めなかった
66枚のうち1枚しか拾えず、1枚 700ms 級と重いので入れていない。

【読めなかった絵も残す】
走行体に画面は無いので、走り終わってから「カードが写っていなかった」のか
「写っていたが小さい・白飛びで読めなかった」のかを、絵を見ないと切り分けられない。
読めた1枚だけでなく、**読めない間の絵も `UNREAD_SNAPSHOT_SEC` おきに残す**
（`RoughSpot/PhotoImage/hint_card/`）。読めた絵とは別の枠で、カードごとに数えるので、
読めた走行やカード2の絵に押し出されない。**本番は `SNAPSHOT_ENABLE` を 0 にする。**

読めた文字列は判定結果の qr1 / qr2 に載り、そのまま PC へ送られる。どちらの
カードとして覚えるかは中身で決まる（`2026/hint_card.py`、`PcExchange.qr_key`）。
"""

import logging
import os
import threading
import time
from typing import Any, Callable, Optional

from scenario_update import ScenarioMailbox

logger = logging.getLogger(__name__)


class HintCardCapture:
    """ヒントカードを読み、読めた文字列を PC へ送る。"""

    #: 1枚のカードを読むのに粘る時間[s]。
    #: シナリオ側の待ち（`SCT`）より長くてよい。別スレッドで読むので走行は進む
    READ_TIMEOUT = 3.0

    #: 次の映像を見るまでの待ち[s]。カメラは 30Hz なので、それ以上速く見ても同じ
    FRAME_INTERVAL = 0.03

    #: 切り抜きで読めないとき、何回に1回は映像まるごとを試すか。
    #: まるごとは 250ms 級に重いので、毎回はやらない
    FULL_FRAME_EVERY = 10

    #: zxing-cpp があれば、それだけで読む（OpenCV の読み直しをしない）。
    #: False に戻すと、読めない絵1枚に走行体で約2秒かかる（2026-09-26 scsk2）
    ZXING_ONLY = True

    #: 読めなかったときに撮り直す回数
    RETRY_COUNT = 5

    #: 撮り直しで新しい映像を読む時間[s]。
    #: READ_TIMEOUT + RETRY_TIMEOUT × RETRY_COUNT が止まる行の SCT に収まるようにする
    RETRY_TIMEOUT = 1.5

    #: 読んだ映像を残す場所（走行体から見た相対パス）
    SNAPSHOT_DIR = "./sorot_spike/RoughSpot/PhotoImage/hint_card"

    #: 0 にすると残さない。**本番は 0 にすること**
    SNAPSHOT_ENABLE = 1

    #: 読めた絵を残す枚数。これを超えた古いものから消す
    SNAPSHOT_KEEP = 20

    #: 読めなかった絵を残す枚数（**カードごと**）。読めた絵とも別に数える。
    #: まとめて数えると、カード2の絵がカード1の寄りながらの絵を押し出す
    #: （40枚をまとめて数えていたとき、カード1は最後の5枚しか残らなかった）。
    #: 粘る時間は最長 10.5秒なので、0.1秒おきなら 105枚で全部残る
    UNREAD_KEEP = 120

    #: 読めない間、何秒おきに1枚残すか。0 にすると最後の1枚だけ残す。
    #: 旋回中は 0.3秒でカードが画面の端から端へ飛ぶので、0.1秒にする。
    #: カメラは 30Hz なので、毎枚残すと書き込みが追いつかない
    UNREAD_SNAPSHOT_SEC = 0.1

    _instance: Optional["HintCardCapture"] = None
    _instance_lock = threading.Lock()

    @classmethod
    def get_instance(cls) -> "HintCardCapture":
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

    def __init__(self):
        self._lock = threading.Lock()
        #: 別のカードの capture() で増やす。前の読み取りを打ち切るために見る
        self._generation = 0
        #: いま粘っているカード（識別値）。無ければ None
        self._reading: Optional[int] = None
        #: 粘るのをやめる時刻。同じカードの SCRIPT が来るたびに延ばす
        self._deadline = 0.0
        #: 最後に来た要求番号。完了の通知はこれに対して返す
        self._request_id = 0
        #: すでに読めたカード（識別値）。読み直さない
        self._done: set = set()
        self._pc_client = None
        self._decoder = None
        self._frame_source: Optional[Callable[[], Any]] = None
        #: 読めない絵を最後に残した時刻（UNREAD_SNAPSHOT_SEC の間引きに使う）
        self._last_unread_at = 0.0

    # --- 差し替え口 -----------------------------------------------------------

    def set_pc_client(self, pc_client) -> None:
        """PC へ繋ぐ口を受け取る（`ev3_python` の起動時に渡される）"""
        self._pc_client = pc_client

    def set_decoder(self, decoder) -> None:
        """二次元コードを読むものを差す（テスト用）"""
        self._decoder = decoder

    def set_frame_source(self, fn) -> None:
        """映像の取り込み口を差す（テスト用）"""
        self._frame_source = fn if callable(fn) else None

    # --- 読み取り -------------------------------------------------------------

    def capture(self, identifier: int, request_id: int = 0) -> None:
        """
        ヒントカードを読む。シナリオの SCRIPT から呼ぶ。

        **待たずに返る。** 読み取りは別スレッドで走るので、シナリオは次の
        コマンドへ進める。

        同じカードで呼び直されたら、読み直さずに粘る時間を延ばす（接近の行と
        止まる行の両方に書くため）。すでに読めていれば何もしない。

        :param identifier: JUDG_QR1 / JUDG_QR2（読めた文字列の中身が優先される）
        """
        with self._lock:
            if identifier in self._done:
                logger.info("ヒントカード（識別値 %d）はもう読めているので読み直しません",
                            identifier)
                return
            self._request_id = request_id
            if self._reading == identifier:
                self._deadline = time.time() + self.READ_TIMEOUT
                logger.info("ヒントカード（識別値 %d）を続けて粘ります（あと %.1f秒）",
                            identifier, self.READ_TIMEOUT)
                return
            self._generation += 1
            generation = self._generation
            self._reading = identifier
            self._deadline = time.time() + self.READ_TIMEOUT
            self._last_unread_at = 0.0      # カードが変わったら1枚目から残す
        threading.Thread(target=self._work, args=(generation, identifier),
                         name="hint-card", daemon=True).start()

    def _finish(self, generation: int, identifier: int, read: bool) -> Optional[int]:
        """粘るのをやめる。打ち切られていなければ、返すべき要求番号を返す"""
        with self._lock:
            if self._generation != generation:
                return None
            self._reading = None
            if read:
                self._done.add(identifier)
            return self._request_id

    def _work(self, generation: int, identifier: int) -> None:
        """読めるまで粘る。読めなければ撮り直し、それでもだめなら諦める"""
        from handler import PcExchange

        result = self._read_until_deadline(generation, identifier)
        if result is None:
            return
        text, frame, attempts = result

        for retry in range(1, self.RETRY_COUNT + 1):
            if text:
                break
            with self._lock:
                if self._generation != generation:
                    return
                self._deadline = max(self._deadline, time.time() + self.RETRY_TIMEOUT)
            logger.info("ヒントカードを読めないので撮り直します（%d/%d回目、%.1f秒）",
                        retry, self.RETRY_COUNT, self.RETRY_TIMEOUT)
            result = self._read_until_deadline(generation, identifier, start=attempts)
            if result is None:
                return
            text, last, attempts = result
            frame = last if last is not None else frame

        if text:
            request_id = self._finish(generation, identifier, read=True)
            if request_id is None:
                return                          # 読めた直後に打ち切られた
            logger.info("ヒントカードを読めました（%d枚目）: %s", attempts, text)
            self._save(frame, text, identifier, attempts)
            PcExchange.send_decoded_qr(identifier, text, self._pc_client, request_id)
            self._place_qr_route(identifier)
            return

        request_id = self._finish(generation, identifier, read=False)
        if request_id is None:
            return
        logger.warning("ヒントカードを読めませんでした（%d枚試し、%d回撮り直した）。"
                       "カードが写っているか、止まる位置を確かめてください",
                       attempts, self.RETRY_COUNT)
        if frame is not None:
            self._save(frame, None, identifier, attempts)
        PcExchange.record_decode_error(identifier, request_id)

    def _read_until_deadline(self, generation: int, identifier: int = 0, start: int = 0):
        """
        `_deadline`（延びることがある）まで新しい映像を読む。

        読めなかった絵も `UNREAD_SNAPSHOT_SEC` おきに残す。**走行体には画面が
        無いので、あとから絵を見ないと「カードが写っていない」のか「写っているが
        読めない」のかが分からない。** 読めた絵だけ残すと、肝心の読めなかった
        ときに何を見ていたかが残らない。

        :param identifier: 読んでいるカード（JUDG_QR1 / JUDG_QR2）。名前に入れる
        :param start: それまでに試した枚数（まるごと読む間隔を続けて数える）
        :returns: (読めた文字列 or None, 最後に見た絵, 試した枚数の累計)。打ち切られたら None
        """
        attempts = start
        last_frame = None

        while time.time() < self._deadline:
            if self._generation != generation:
                logger.info("ヒントカードの読み取りを打ち切りました"
                            "（次のカードの読み取りが始まった）")
                return None

            frame = self._frame()
            if frame is None or frame is last_frame:
                # 新しい映像が来るまで待つ。同じ1枚を読み直しても結果は変わらない
                time.sleep(self.FRAME_INTERVAL)
                continue

            last_frame = frame
            attempts += 1
            text = self._decode(frame, attempts)
            if text:
                return text, frame, attempts

            self._save_unread(frame, identifier, attempts)
            time.sleep(self.FRAME_INTERVAL)

        if self._generation != generation:
            return None
        return None, last_frame, attempts

    def _frame(self) -> Any:
        """いまの映像。色の読み取りが回っていれば、その最後の1枚を借りる"""
        if self._frame_source is not None:
            try:
                return self._frame_source()
            except Exception:  # noqa: BLE001 — 映像が取れなくても走行は続ける
                logger.exception("映像の取り込みに失敗しました")
                return None

        from bottle_color_task import BottleColorTask
        from camera import RobotCamera

        camera = RobotCamera.shared()
        if BottleColorTask.get_instance().is_reading():
            # **自分では読みにいかない。** VideoCapture を2つのスレッドから
            # 同時に read() すると壊れる（camera.py の RobotCamera を参照）
            return camera.latest_frame()
        return camera.grab_latest()

    def _decode(self, frame: Any, attempts: int) -> Optional[str]:
        """映像1枚から二次元コードを読む。読めなければ None"""
        decoder = self._decoder
        if decoder is None:
            from qr_decoder import QrCodeDecoder
            decoder = QrCodeDecoder()
            self._decoder = decoder

        try:
            # zxing-cpp があれば、それだけで読む。OpenCV の読み直しは読めない絵に
            # 走行体で1枚約2秒かかり、30Hz の映像を数秒に1枚しか試せなくなる
            text = decoder.decode(frame, rects=decoder.find_card_rects(frame),
                                  zxing_only=self.ZXING_ONLY)
            if text:
                return text
            # 切り抜きで読めない。カード以外の白いものを見ていることがあるので、
            # ときおり映像まるごとも試す（重いので毎回はやらない）
            if attempts % self.FULL_FRAME_EVERY == 0:
                return decoder.decode(frame, zxing_only=self.ZXING_ONLY)
        except Exception:  # noqa: BLE001 — 読めなくても走行は続ける
            logger.exception("二次元コードの読み取りに失敗しました")
        return None

    # --- 残す -----------------------------------------------------------------

    def _save(self, frame: Any, text: Optional[str],
              identifier: int = 0, attempts: int = 0) -> Optional[str]:
        """読んだ映像を1枚残す。読めなかったときも残す（何を見ていたかが分かる）

            hint_20260919_121530_123_c1_n23_read.jpg
            hint_20260919_121530_456_c1_n41_unread.jpg
                    ↑日時_ミリ秒      ↑カード ↑何枚目  ↑読めた/読めなかった

        :param identifier: JUDG_QR1 / JUDG_QR2。どちらのカードを読んでいたか
        :param attempts: 何枚目の映像か。撮り直しをまたいで数える
        :returns: 書けたファイルのパス。書けなければ None
        """
        if not self.SNAPSHOT_ENABLE:
            return None
        try:
            from opencv import OpenCv
            cv2 = OpenCv.cv2()
            if cv2 is None:
                return None

            os.makedirs(self.SNAPSHOT_DIR, exist_ok=True)
            path = os.path.join(self.SNAPSHOT_DIR, self._name(text, identifier, attempts))
            if not cv2.imwrite(path, frame):
                logger.warning("画像を書けませんでした: %s", path)
                return None
            self._prune("_read.jpg", self.SNAPSHOT_KEEP)
            self._prune("_unread.jpg", self.UNREAD_KEEP, self._card(identifier))
            return path
        except Exception:  # noqa: BLE001 — 残せなくても走行は続ける
            logger.exception("ヒントカードの画像を残せませんでした")
            return None

    def _save_unread(self, frame: Any, identifier: int, attempts: int) -> Optional[str]:
        """読めなかった映像を、UNREAD_SNAPSHOT_SEC おきに1枚残す

        毎枚残すと 30Hz ぶんの書き込みになり、読み取り自体が遅くなる。
        間引いても、寄りながら読むあいだの絵が何枚か残れば切り分けには足りる。
        """
        if not self.SNAPSHOT_ENABLE or not self.UNREAD_SNAPSHOT_SEC:
            return None
        now = time.time()
        if (now - self._last_unread_at) < self.UNREAD_SNAPSHOT_SEC:
            return None
        self._last_unread_at = now
        return self._save(frame, None, identifier, attempts)

    def _name(self, text: Optional[str], identifier: int, attempts: int) -> str:
        """残す絵の名前。カードと何枚目かを入れ、読めたかどうかで終わりを変える"""
        now = time.time()
        stamp = "%s_%03d" % (time.strftime("%Y%m%d_%H%M%S", time.localtime(now)),
                             int((now % 1) * 1000))
        return "hint_%s_%s_n%d_%s.jpg" % (stamp, self._card(identifier), attempts,
                                          "read" if text else "unread")

    @staticmethod
    def _card(identifier: int) -> str:
        """名前に入れるカードの印（c1 / c2）"""
        from config import JUDG_QR1, JUDG_QR2

        return {JUDG_QR1: "c1", JUDG_QR2: "c2"}.get(identifier, "cNA")

    def _prune(self, suffix: str = "_read.jpg", keep: int = SNAPSHOT_KEEP,
               card: Optional[str] = None) -> None:
        """古い画像から消す。SD カードを埋めないため

        **読めた絵と読めなかった絵は別々に数える。** ひとまとめにすると、
        読めた走行を重ねるだけで、見たかった「読めなかった絵」が消える。

        :param card: 指定すると、そのカード（c1 / c2）の絵だけを数える
        """
        try:
            files = sorted(f for f in os.listdir(self.SNAPSHOT_DIR)
                           if f.startswith("hint_") and f.endswith(suffix)
                           and (card is None or ("_%s_" % card) in f))
            for name in files[:-keep]:
                os.remove(os.path.join(self.SNAPSHOT_DIR, name))
        except OSError:
            pass

    # --- 読めた文字列での差し替え ---------------------------------------------

    def _place_qr_route(self, identifier: int) -> None:
        """読めた文字列に対応する運び先があれば、シナリオを差し替える"""
        from config import JUDG_QR1
        from state import ResultStore

        try:
            import config_2026
            routes = getattr(config_2026, "QR_ROUTE_FILES", {})
        except ImportError:
            return
        if not routes:
            return

        key = "qr1" if identifier == JUDG_QR1 else "qr2"
        text = ResultStore.get_instance().get(key, "")
        route = routes.get(text)
        if route is not None:
            ScenarioMailbox.place(route)
