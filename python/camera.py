"""
走行体共通のカメラキャプチャモジュール。

OpenCV を用いて物理カメラデバイス（0 など）または画像ファイル/動画ファイルから映像を取得する。
"""

from __future__ import annotations

import logging
from typing import Any, Optional, Union

logger = logging.getLogger(__name__)

try:
    import cv2
except ImportError:
    cv2 = None


class RobotCamera:
    """
    走行体のカメラから映像を 1枚取得するクラス。

    物理カメラ（デバイス番号 0 等）のほか、画像ファイル（.png, .jpg）や
    動画ファイルからの読み込みに対応する。
    """

    def __init__(self, source: Union[int, str] = 0, width: int = 640, height: int = 480):
        self.source = source
        self.width = width
        self.height = height
        self._capture = None

    def _is_image_file(self) -> bool:
        """指定されたソースが静止画ファイルかどうかを判定する。"""
        if isinstance(self.source, str):
            lower = self.source.lower()
            return lower.endswith((".png", ".jpg", ".jpeg", ".bmp", ".webp"))
        return False

    def open(self) -> bool:
        """カメラまたは動画ファイルを開く。開けたら True"""
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
        self._capture = cap
        return True

    def grab(self) -> Any:
        """映像を 1枚取得する。取得できなければ None"""
        if cv2 is None:
            return None

        # 静止画ファイルの場合は常に最新のファイルを読み込む
        if self._is_image_file():
            frame = cv2.imread(str(self.source))
            return frame

        # カメラデバイスまたは動画ファイルから読み込む
        if not self.open():
            return None
        ok, frame = self._capture.read()
        return frame if ok else None

    def close(self) -> None:
        """カメラを閉じる。"""
        if self._capture is not None:
            self._capture.release()
            self._capture = None
