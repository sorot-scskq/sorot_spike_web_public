"""
走行体のカメラモジュール。

**取り込みは OpenCV に一本化してある。カメラは走行体に1台しか無い。**

    連続取込 : カメラを開きっぱなしにする（RobotCamera）。走りながら読むもの
               （ラベル色・ヒントカード）はこちら。1枚あたりのコストが下がり、
               短い区間でも複数枚拾える
    1枚撮り  : その同じカメラから1枚取ってファイルへ（RobotCamera.capture_still）。
               PC へ画像を送る判定はこちら

**別々に開いてはいけない。** 1台しか無いので、2つ目の VideoCapture は
デバイスが使用中で失敗する。`RobotCamera.shared()` で同じ1台を借りること。

**VideoCapture を触るのは連続取り込みだけ。** 撮影は自分で read() せず、
連続取り込みが取ったばかりの1枚（latest_frame）を使う。2つのスレッドから
同時に read() すると壊れるため、読む側をひとつに絞ってある。
連続取り込みが回っていない年（色を読まない年）は、誰も読んでいないので
撮影が自分で1枚取る。

以前は1枚撮りだけ v4l2-ctl を subprocess で起動していた。プロセス生成と
デバイス open が毎回かかるうえ、連続取込と両立しないため、やめた。

なお set_frame_source は映像の取り込み口を外から差すためのもので、テストと、
sorot_spike_web（シミュレータ）が canvas の画素を渡すのに使う。
"""

from __future__ import annotations

import base64
import logging
import os
import threading
from typing import Any, Optional, Union

try:
    from opencv import OpenCv
except ImportError:  # パッケージとして読み込まれた場合
    from .opencv import OpenCv

logger = logging.getLogger(__name__)


class RobotCamera:
    """
    走行体のカメラから映像を取得する。

    物理カメラ（デバイス番号 0 等）のほか、画像ファイル（.png, .jpg）や
    動画ファイルからの読み込みに対応する。テストは set_frame_source で
    映像を差し込める。

    **走行体のカメラは1台。** 共有する1つは `shared()` で借りる。
    """

    #: 走行体で共有する1台（開けるのは1つのプロセスの1つの VideoCapture だけ）
    _shared: Optional["RobotCamera"] = None
    _shared_lock = threading.Lock()

    # --- 共有する1台 -----------------------------------------------------------

    @classmethod
    def shared(cls, width: int = None, height: int = None) -> "RobotCamera":
        """
        走行体のカメラ（1台）を借りる。

        初回に開いた解像度がそのまま使われる。色の読み取りと撮影が同じ映像を見る。

        :param width: 初回だけ効く。省略時は config の CAMERA_WIDTH
        :param height: 同上
        """
        with cls._shared_lock:
            if cls._shared is None:
                try:
                    from config import CAMERA_WIDTH, CAMERA_HEIGHT
                except ImportError:      # パッケージとして読み込まれた場合
                    from .config import CAMERA_WIDTH, CAMERA_HEIGHT
                cls._shared = cls(width=width or CAMERA_WIDTH,
                                  height=height or CAMERA_HEIGHT)
            return cls._shared

    @classmethod
    def release_shared(cls) -> None:
        """借りていたカメラを返す（テストと終了処理用）"""
        with cls._shared_lock:
            old, cls._shared = cls._shared, None
        if old is not None:
            old.close()

    # --- 1枚撮り ---------------------------------------------------------------

    @classmethod
    def capture_still(cls, path: str) -> bool:
        """
        いまの映像を1枚、ファイルへ書く。撮れたら True。

        **v4l2-ctl の1枚撮りをやめて OpenCV にした。** 以前は撮影のたびに
        v4l2-ctl を subprocess で起動しており、プロセス生成とデバイス open が
        毎回かかっていた。色の読み取りで既にカメラを開いている以上、その1台から
        取れば起動も open も要らない。

        **撮れなくても走行は止めない。** False を返すので、呼び側は模擬画像へ回る。
        """
        cv2 = OpenCv.cv2()
        if cv2 is None:
            logger.warning("OpenCV が無いため撮影できません")
            return False

        camera = cls.shared()

        # **自分では読みにいかない。** 連続取り込み（色の読み取り）が 30Hz で
        # 回しているので、その最後の1枚を使う。VideoCapture を2つのスレッドから
        # 同時に read() すると壊れるため、触るのは1つだけにしておく。
        # 30Hz なら最新でも 33ms 前のもので、以前の v4l2-ctl の1枚撮り
        # （プロセス起動ぶん遅れる）より新しい。
        frame = camera.latest_frame()
        if frame is None:
            # 連続取り込みが回っていない。誰も読んでいないので自分で1枚取る
            frame = camera.grab_latest()
        if frame is None:
            logger.warning("映像を取れませんでした（カメラを開けているか確認）")
            return False

        try:
            os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
            if not cv2.imwrite(path, frame):
                logger.warning("画像を書けませんでした: %s", path)
                return False
        except OSError as e:
            logger.warning("画像を書けませんでした(%s): %s", path, e)
            return False

        logger.info("撮影しました: %s", path)
        return True

    # --- 画像の受け渡し ---------------------------------------------------------

    @staticmethod
    def encode_base64(path: str) -> str:
        """撮った画像を Base64 にする。PC へはこの形で送る"""
        if not os.path.exists(path) or os.path.getsize(path) == 0:
            logger.warning("画像ファイルが存在しないか空です: %s", path)
            return ""

        with open(path, "rb") as img_file:
            return base64.b64encode(img_file.read()).decode("utf-8")

    @staticmethod
    def decode_qr(path: str):
        """画像ファイルから二次元コードを読む（`QrCodeDecoder.decode_file` と同じ）"""
        try:
            from qr_decoder import QrCodeDecoder
        except ImportError:
            from .qr_decoder import QrCodeDecoder
        return QrCodeDecoder().decode_file(path)

    # --- 1台ぶんの取り込み -----------------------------------------------------

    def __init__(self, source: Union[int, str] = 0, width: int = 640, height: int = 480):
        self.source = source
        self.width = width
        self.height = height
        self._capture = None
        self._frame_source = None
        # 最後に取れた1枚。撮影（capture_still）はこれを使う。
        # **VideoCapture を触るのは1つだけにする。** 同時に read() すると壊れるため、
        # 連続取り込みが回っているあいだ、撮影は自分で読みにいかない
        self._last_frame = None

    def set_frame_source(self, fn) -> None:
        """
        映像の取り込み口を差す。差してあるあいだはカメラを開かない。

        テストはここに映像を返す関数を差す。
        """
        self._frame_source = fn if callable(fn) else None

    def _is_image_file(self) -> bool:
        """指定されたソースが静止画ファイルかどうかを判定する。"""
        if isinstance(self.source, str):
            lower = self.source.lower()
            return lower.endswith((".png", ".jpg", ".jpeg", ".bmp", ".webp"))
        return False

    def open(self) -> bool:
        """カメラまたは動画ファイルを開く。開けたら True"""
        if self._frame_source is not None:
            return True
        cv2 = OpenCv.cv2()
        if cv2 is None:
            logger.warning("OpenCV が無いためカメラを開けません")
            return False
        if self._is_image_file():
            return True
        if self._capture is not None and self._capture.isOpened():
            return True
        cap = cv2.VideoCapture(self.source)
        if not cap.isOpened():
            logger.warning("カメラソース %s を開けませんでした", self.source)
            return False
        if isinstance(self.source, int):
            # MJPG（カメラ内で圧縮したもの）を要求する。
            #
            # 指定しないとドライバは無圧縮の YUYV を返す。解像度は要求どおり
            # 出るが、YUYV は 800x600 以上で 30fps を出せない（カメラの対応表。
            # 1280x720 で 10fps、1280x960 以上で 5fps が上限）。
            #
            # **MJPG は速さでも CPU でも YUYV に勝つ。** 伸張の手間より、YUYV の
            # YUV→BGR 変換のほうが高い。2026-09-08 実測（30Hz を維持したまま、
            # read + 色判定 1周期あたりの CPU）:
            #
            #   640x480    MJPG 19.2% / YUYV 33.7%（どちらも 30Hz 出る）
            #   1280x720   MJPG 65.1% / YUYV は 9.7Hz しか出ない
            #
            # 解像度は 1280x960 まで 30Hz を維持できる。1920x1080 だけは 25.4Hz
            # に落ち、CPU も 1コアを超える（111.8%）。
            #
            # **解像度より先に指定すること。** 後から指定するとドライバが
            # 解像度を選び直し、要求した大きさに戻らないことがある。
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            # 溜め込みは 2枚にする。**1枚にしてはいけない。**
            #
            # 1枚だと、こちらが1枚を持っているあいだドライバに置き場が無く、
            # 次のフレームを取りこぼす。実測でちょうど半分になった
            # （2026-09-08、MJPG 1280x720。BUFFERSIZE=1 で 14.9fps、
            # 2 でも 4 でも 29.9fps）。以前ここが 1 だったため、30fps 出る
            # カメラを 15fps で使っていた。
            #
            # 2 あれば取りこぼさない。古い映像を掴まないための仕組みは
            # 溜め込みを減らすことではなく、**読む側が待たずに読み続けること**
            # （grab_latest と BottleColorTask._loop のコメントを参照）。
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)
            # **つまみを書く前に1枚読む。**
            #
            # OpenCV は最初に読まれるまで撮影を始めない（VIDIOC_STREAMON）。
            # 始まる前に書いた露出・ゲインは撮影開始時に捨てられ、カメラは
            # 白飛びしたまま二度と戻らない。書けたかを読み直しても要求どおりの
            # 値が返るので気づけない（2026-09-19 実測。開けて即 manual(60) を
            # 書くと 120枚・4.5秒のあいだ明るさ 251・白飛び 85% のまま。
            # 1枚読んでから同じ値を書くと 2枚目から 182・0% に変わる）。
            #
            # 撮影を始めるだけなので、伸張の要らない grab() で足りる。
            cap.grab()
            self._tune(cap)
        self._capture = cap
        return True

    #: つまみを決めたあと、捨てる枚数。
    #:
    #: **書いた直後の数枚は前の設定の絵。** 実機で、露出を短くしたのに白飛びした
    #: 絵がしばらく返った（2026-09-21）。走り出しの1枚目でボトルの色を誤るのを防ぐ。
    TUNE_FLUSH_FRAMES = 6

    def _tune(self, cap) -> None:
        """
        シャッター・ゲインを config のとおりに決める。

        **既定（すべて None）では何もしない＝自動露出のまま。**
        床の二次元コードを読むときに要る（`Common/camera_tuning.py`）。

        **ここは本番の走行でも通る。** 点検の道具（`Tools/read_floor_code.py`）と
        同じ `CameraTuning.manual()` を呼ぶので、自動を切ってから書く順序も、
        撮影を始めてから書く順序（呼ぶ側の `cap.grab()`）も共通に効く。

        :param cap: 開いたばかりの VideoCapture。書いたあとの数枚を捨てるのに使う
        """
        try:
            from config import CAMERA_FOCUS, CAMERA_EXPOSURE, CAMERA_GAIN
        except ImportError:      # パッケージとして読み込まれた場合
            from .config import CAMERA_FOCUS, CAMERA_EXPOSURE, CAMERA_GAIN
        if CAMERA_FOCUS is None and CAMERA_EXPOSURE is None and CAMERA_GAIN is None:
            return
        try:
            from camera_tuning import CameraTuning
        except ImportError:
            from .camera_tuning import CameraTuning
        device = "/dev/video%d" % self.source if isinstance(self.source, int) else None
        if device is None:
            return
        CameraTuning(device).manual(focus=CAMERA_FOCUS, exposure=CAMERA_EXPOSURE,
                                    gain=CAMERA_GAIN)
        for _ in range(self.TUNE_FLUSH_FRAMES):
            if not cap.grab():          # デコードせずに捨てる
                break
        logger.info("カメラのつまみを決めました: 露出=%s ゲイン=%s（焦点=%s。固定焦点）",
                    CAMERA_EXPOSURE, CAMERA_GAIN, CAMERA_FOCUS)

    def grab(self) -> Any:
        """映像を 1枚取得する。取得できなければ None"""
        frame = self._grab()
        if frame is not None:
            self._last_frame = frame
        return frame

    def latest_frame(self) -> Any:
        """最後に取れた1枚。まだ取っていなければ None

        連続取り込みが回っているあいだは、これが 30Hz で入れ替わる。
        撮影はこれを使うので、VideoCapture を触るのは連続取り込みだけで済む。
        """
        return self._last_frame

    def _grab(self) -> Any:
        if self._frame_source is not None:
            try:
                return self._frame_source()
            except Exception:  # noqa: BLE001 — 映像が取れなくても走行は続ける
                logger.exception("映像の取り込みに失敗しました")
                return None

        cv2 = OpenCv.cv2()
        if cv2 is None:
            return None

        # 静止画ファイルの場合は常に最新のファイルを読み込む
        if self._is_image_file():
            return cv2.imread(str(self.source))

        # カメラデバイスまたは動画ファイルから読み込む
        if not self.open():
            return None
        ok, frame = self._capture.read()
        return frame if ok else None

    def grab_latest(self, drop: int = 0) -> Any:
        """
        いちばん新しい映像を取る。

        **既定では読み捨てない（drop=0）。**

        以前は「CAP_PROP_BUFFERSIZE が効かないドライバへの保険」として 2枚
        読み捨てていたが、これは害のほうが大きかった。1枚読むのに1フレーム
        待つので、捨てる2枚がそのまま待ち時間になる（2026-09-08 実測で
        1回 360ms、実効 2.7Hz）。

        古い映像を掴む本当の原因は溜め込みではなく、**読む側が周期待ちを
        入れて読み落とすこと**だった。待ちを入れると行列ができ、read() が
        即座に返って古い映像を渡す。待たずに読み続ければ read() 自身が
        カメラの速度でブロックし、常に最新の1枚が来る（実測 29.8Hz、
        read() は 26ms 待つ＝新しい）。呼ぶ側は BottleColorTask._loop。

        取り込み口を差してあるとき（シミュレータ・テスト）と静止画のときは、
        そもそも溜まらないので grab() と同じ。

        :param drop: 読み捨てる枚数。読む側が待ちを入れざるを得ない場合だけ
                     指定する。1枚につき1フレーム分の待ちが増える
        """
        if self._frame_source is not None or self._is_image_file():
            return self.grab()
        if not self.open():
            return None
        for _ in range(max(0, drop)):
            self._capture.grab()      # デコードせずに捨てる（read より軽い）
        return self.grab()

    def close(self) -> None:
        """カメラを閉じる。"""
        self._last_frame = None
        if self._capture is not None:
            self._capture.release()
            self._capture = None
