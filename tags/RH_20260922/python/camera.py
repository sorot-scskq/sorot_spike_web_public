"""
走行体共通のカメラモジュール。

映像の取り方が実機とシミュレータで違うので、両方をここに置いて使い分ける。

    実機          : v4l2-ctl で 1枚撮ってファイルへ（execute_camera_command）
                    または OpenCV でカメラを開く（RobotCamera）
    シミュレータ  : PyScript が canvas から作った配列を RobotCamera に渡す

【遅延 import にしている理由】
このモジュールは PyScript（Pyodide）からも読み込む。ブラウザには subprocess が
無く、OpenCV も後から読み込むため、モジュールの先頭で import すると
読み込んだ時点で落ちる。実機でも同じことが起きた実績がある（走行体に OpenCV が
無い環境で ev3_python 全体が起動できず、C++ 側が「接続失敗、再試行します」を
出し続けた）。使う関数の中で import する。
"""

from __future__ import annotations

import base64
import logging
import os
from typing import Any, Union

logger = logging.getLogger(__name__)


def _cv2():
    """OpenCV。無ければ None（映像を扱わない処理は続けられるようにする）"""
    try:
        import cv2
        return cv2
    except ImportError:
        return None


# --------------------------------------------------------------------------
# 実機: 外部コマンドで 1枚撮る
# --------------------------------------------------------------------------


def execute_camera_command(command: str) -> bool:
    """
    撮影コマンド（v4l2-ctl など）を実行する。成功したら True。

    設定は Common/config.py の CAMERA_COMMAND / IMAGE_PATH を使う。
    """
    import subprocess

    try:
        subprocess.run(
            command,
            shell=True,
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        logger.info("カメラコマンド実行完了")
        return True
    except subprocess.CalledProcessError as e:
        # 環境のエンコーディングに合わせてデコードを試行 (Windows=cp932, Linux=utf-8)
        err_msg = ""
        for encoding in ("utf-8", "cp932", "shift_jis"):
            try:
                err_msg = e.stderr.decode(encoding)
                break
            except Exception:
                continue
        if not err_msg:
            err_msg = e.stderr.decode(errors="replace")

        logger.error("カメラコマンド実行失敗: %s", err_msg.strip())
        return False
    except Exception as e:  # noqa: BLE001 — subprocess が無い環境（ブラウザ）を含む
        logger.error("カメラコマンドを実行できません: %s", e)
        return False


def encode_image_to_base64(path: str) -> str:
    """撮った画像を Base64 にする。PC の推論サーバへはこの形で送る"""
    if not os.path.exists(path) or os.path.getsize(path) == 0:
        logger.warning("画像ファイルが存在しないか空です: %s", path)
        return ""

    with open(path, "rb") as img_file:
        return base64.b64encode(img_file.read()).decode("utf-8")


# --------------------------------------------------------------------------
# OpenCV でカメラ・画像・動画から取る
# --------------------------------------------------------------------------


class RobotCamera:
    """
    走行体のカメラから映像を 1枚取得するクラス。

    物理カメラ（デバイス番号 0 等）のほか、画像ファイル（.png, .jpg）や
    動画ファイルからの読み込みに対応する。シミュレータは set_frame_source で
    canvas から作った配列を差す。
    """

    def __init__(self, source: Union[int, str] = 0, width: int = 640, height: int = 480):
        self.source = source
        self.width = width
        self.height = height
        self._capture = None
        self._frame_source = None

    def set_frame_source(self, fn) -> None:
        """
        映像の取り込み口を差す。差してあるあいだはカメラを開かない。

        シミュレータ（PyScript）はここに canvas を読む関数を差す。
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
        cv2 = _cv2()
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
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            # 溜め込みを 1枚にする。
            #
            # read() は「次の1枚」を返すので、取り込みが撮影より遅いと古い順に
            # 溜まったものが返る。走行体は動いているので、これは「さっき居た
            # 場所の映像」を見ることになる。
            #
            # キャリーボトルの色は、ボトルの 449mm〜249mm 手前でしか読めない
            # （それより近いとラベルが画角の下へ外れる）。この区間は 200mm しか
            # なく、通過に 0.5秒ほど。数枚ぶん遅れるだけで区間の外の映像を見る。
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self._capture = cap
        return True

    def grab(self) -> Any:
        """映像を 1枚取得する。取得できなければ None"""
        if self._frame_source is not None:
            try:
                return self._frame_source()
            except Exception:  # noqa: BLE001 — 映像が取れなくても走行は続ける
                logger.exception("映像の取り込みに失敗しました")
                return None

        cv2 = _cv2()
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

    def grab_latest(self, drop: int = 2) -> Any:
        """
        溜まっているぶんを捨てて、いちばん新しい映像を取る。

        CAP_PROP_BUFFERSIZE はドライバによっては効かない。効かなかったときの
        ための保険として、読み捨ててから 1枚取る。捨てるのは待ち時間なので、
        必要なだけにとどめる（既定 2枚）。

        取り込み口を差してあるとき（シミュレータ）と静止画のときは、そもそも
        溜まらないので grab() と同じ。

        :param drop: 読み捨てる枚数
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
        if self._capture is not None:
            self._capture.release()
            self._capture = None


# 二次元コードの読み取りは qr_decoder.py にある。
# handler.py が `from camera import decode_qr_code` で使っているため、
# 呼び出し側を変えずに済むよう、ここからも見えるようにしておく。
def decode_qr_code(path: str):
    """画像ファイルから二次元コードを読む（QrCodeDecoder().decode_file と同じ）"""
    try:
        from qr_decoder import QrCodeDecoder
    except ImportError:
        from .qr_decoder import QrCodeDecoder
    return QrCodeDecoder().decode_file(path)
