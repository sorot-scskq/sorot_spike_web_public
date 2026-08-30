"""
共通の二次元コード読み取りおよびパネル検出モジュール。
"""

from __future__ import annotations

import logging
from typing import Optional

logger = logging.getLogger(__name__)

try:
    import cv2
except ImportError:
    cv2 = None

try:
    import numpy as np
except ImportError:
    np = None


class QrCodeDecoder:
    """
    OpenCV を用いて二次元コードを検出・デコードするクラス。
    """

    def __init__(self):
        self._detector = None

    def _get_detector(self):
        if self._detector is None:
            if cv2 is None:
                return None
            self._detector = cv2.QRCodeDetector()
        return self._detector

    def decode(self, frame) -> Optional[str]:
        """映像から二次元コードの文字列を読む。読めなければ None"""
        detector = self._get_detector()
        if detector is None or frame is None:
            return None
        try:
            text, points, _ = detector.detectAndDecode(frame)
        except Exception:  # noqa: BLE001
            logger.exception("二次元コードの読み取りに失敗しました")
            return None
        if isinstance(text, str) and text.strip() != "":
            return text
        return None


def find_card_panels(frame, min_side: int = 8, max_side_ratio: float = 0.9) -> list:
    """
    映像の中から「カードらしい四角」を探す。

    白地に黒い模様が乗った、おおむね正方形の板を探す。二次元コードとして
    読めるだけの解像度が無いときに、カードが写っているかどうかだけを
    画素から判断するために使う。

    :returns: [{'x','y','w','h','area','fill'}] を大きい順に。'fill' は
              四角の中の暗い画素の割合。
    """
    if cv2 is None or np is None or frame is None:
        return []

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
    h, w = gray.shape[:2]
    # 白い板を取り出す
    _, mask = cv2.threshold(gray, 170, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    found = []
    for contour in contours:
        x, y, cw, ch = cv2.boundingRect(contour)
        if cw < min_side or ch < min_side:
            continue
        if cw > w * max_side_ratio and ch > h * max_side_ratio:
            continue
        aspect = cw / float(ch)
        if aspect < 0.6 or aspect > 1.7:
            continue
        roi = gray[y:y + ch, x:x + cw]
        dark = float((roi < 110).sum()) / float(roi.size)
        if dark < 0.05:
            continue
        found.append({"x": x, "y": y, "w": cw, "h": ch, "area": cw * ch, "fill": dark})

    found.sort(key=lambda p: p["area"], reverse=True)
    return found
