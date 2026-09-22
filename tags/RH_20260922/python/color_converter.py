"""
共通色空間変換モジュール。
RGB/HSV 等の色空間変換ユーティリティを提供する。
"""

from typing import Dict


class ColorConverter:
    """色空間の変換ユーティリティクラス。"""

    @staticmethod
    def rgb_to_hsv(r: float, g: float, b: float) -> Dict[str, float]:
        """
        RGB (0〜255) を HSV (h: 0〜360, s: 0〜1, v: 0〜1) に変換する。
        OpenCV の cvtColor(COLOR_BGR2HSV) と同等の定義。
        """
        rn = r / 255.0
        gn = g / 255.0
        bn = b / 255.0
        cmax = max(rn, gn, bn)
        cmin = min(rn, gn, bn)
        delta = cmax - cmin

        h = 0.0
        if delta > 0:
            if cmax == rn:
                h = 60.0 * (((gn - bn) / delta) % 6)
            elif cmax == gn:
                h = 60.0 * ((bn - rn) / delta + 2)
            else:
                h = 60.0 * ((rn - gn) / delta + 4)
        if h < 0:
            h += 360.0

        s = 0.0 if cmax == 0 else delta / cmax
        v = cmax
        return {'h': h, 's': s, 'v': v}
