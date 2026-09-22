"""
二次元コードの読み取りと、カード領域の検出。

【4段構えで読む理由】
走行体のカメラは、光の当たり方・ぶれ・距離で写りが変わる。素直に
detectAndDecode を 1回呼ぶだけでは実機で読み落とすため、手を変えて 4回試す。
実機で効いた順に並べてある。

    1. カラー画像のまま
    2. グレースケール化（色かぶりを落とす）
    3. Otsu の二値化（コントラストを立てる）
    4. 2倍に拡大（ぼやけ・潰れを補う）

【遅延 import にしている理由】
このモジュールは PyScript（Pyodide）からも読み込む。OpenCV はページを開いた
あとに読み込むので、モジュールの先頭で import すると None のまま固定されて
しまう。使うときに取りにいく。
"""

from __future__ import annotations

import logging
import os
from typing import Optional

logger = logging.getLogger(__name__)


#: 4段目の拡大で作ってよい画像の長辺[px]。
#:
#: 拡大は「小さく写って潰れたコードを補う」ための手当てで、元が大きいときは
#: 情報が増えないまま画素数だけ 4倍になる。実測（PyScript, Pyodide）では
#: 960x576 を 1920x1152 にすると、この 1段だけで 120ms かかり、4段の合計
#: 250ms の半分近くを占めていた。
#:
#: 実機のカメラは 640x480（camera.py の RobotCamera 既定）なので 2倍でも
#: 1280 に収まり、これまでどおり拡大して読む。シミュレータの読み取り用
#: （960x576）はもともと二次元コードが読める大きさで描いているため、
#: ここで止めても読めるものが読めなくなることはない。
MAX_UPSCALED_LONG_SIDE = 1280


#: 板を切り抜くときに足す余白[px]。
#:
#: 二次元コードは周りに余白（クワイエットゾーン）が無いと読めない。板の
#: 外形ちょうどで切ると、白い縁が入らずに読めなくなることがあるため足す。
PANEL_CROP_MARGIN = 8


def _cv2():
    """OpenCV。無ければ None"""
    try:
        import cv2
        return cv2
    except ImportError:
        return None


def _np():
    """NumPy。無ければ None"""
    try:
        import numpy as np
        return np
    except ImportError:
        return None


class QrCodeDecoder:
    """
    OpenCV を用いて二次元コードの検出・デコードおよびカード領域検出を行うクラス。

    【4段構えで読む理由】
    走行体のカメラは、光の当たり方・ぶれ・距離で写りが変わる。素直に
    detectAndDecode を 1回呼ぶだけでは実機で読み落とすため、手を変えて 4回試す。
    実機で効いた順に並べてある。

        1. カラー画像のまま
        2. グレースケール化（色かぶりを落とす）
        3. Otsu の二値化（コントラストを立てる）
        4. 2倍に拡大（ぼやけ・潰れを補う。元が大きいときは拡大しない）
    """

    def __init__(self) -> None:
        self._detector = None

    def _get_detector(self, cv2):
        """QRCodeDetector のインスタンスを取得またはキャッシュから返す。"""
        if self._detector is None:
            try:
                self._detector = cv2.QRCodeDetector()
            except Exception:
                return None
        return self._detector

    def decode_frame(self, frame) -> Optional[str]:
        """
        映像（BGR の配列）から二次元コードの文字列を読む。読めなければ None。

        手を変えて 4回試す（クラスの説明を参照）。
        """
        cv2 = _cv2()
        if cv2 is None or frame is None:
            return None

        try:
            detector = self._get_detector(cv2)
            if detector is None:
                detector = cv2.QRCodeDetector()

            # 1. カラー画像のまま
            data, _bbox, _ = detector.detectAndDecode(frame)
            if data:
                logger.info("二次元コードを読めました (カラー): %s", data)
                return data

            # 2. グレースケール化
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
            found = self._decode_multi(detector, gray, "グレースケール")
            if found:
                return found

            # 3. Otsu の二値化でコントラストを立てる
            _, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
            found = self._decode_multi(detector, thresh, "二値化")
            if found:
                return found

            # 4. 2倍に拡大してぼやけ・潰れを補う。
            #    元がもう大きいときは拡大しない（MAX_UPSCALED_LONG_SIDE）
            h, w = gray.shape[:2]
            if max(h, w) * 2 <= MAX_UPSCALED_LONG_SIDE:
                resized = cv2.resize(gray, (w * 2, h * 2), interpolation=cv2.INTER_CUBIC)
                found = self._decode_multi(detector, resized, "2倍拡大")
                if found:
                    return found

            return None
        except Exception:  # noqa: BLE001 — 読めなくても走行は続ける
            logger.exception("二次元コードの読み取りに失敗しました")
            return None

    def decode(self, frame, panels=None) -> Optional[str]:
        """
        映像から二次元コードの文字列を読む。読めなければ None。

        :param panels: find_card_panels の戻り値。渡すと、その板のところだけを
                       切り抜いて読む。

        【切り抜いて読む理由】
        4段の読み取りは、コードが写っている大きさに関係なく映像の全画素ぶんの
        時間が掛かる。板は映像のごく一部なので、そこだけ切り出せば同じ4段でも
        桁違いに速い。シミュレータ（960x576, PyScript）の実測で 250ms → 数ms。

        板が1つも見つからないときは、これまでどおり映像まるごとを読む。板の
        検出は「白い板に黒い模様」を前提にしているので、そう写らない撮り方
        （コードだけが画面いっぱい等）でも読めるようにしておく。
        """
        if panels:
            for panel in panels:
                text = self.decode_frame(self._crop_panel(frame, panel))
                if isinstance(text, str) and text.strip() != "":
                    return text
            return None

        text = self.decode_frame(frame)
        if isinstance(text, str) and text.strip() != "":
            return text
        return None

    def _crop_panel(self, frame, panel):
        """板のところを、余白を付けて切り抜く。切り出せなければ元の映像"""
        if frame is None:
            return frame
        try:
            h, w = frame.shape[:2]
            x0 = max(0, int(panel["x"]) - PANEL_CROP_MARGIN)
            y0 = max(0, int(panel["y"]) - PANEL_CROP_MARGIN)
            x1 = min(w, int(panel["x"]) + int(panel["w"]) + PANEL_CROP_MARGIN)
            y1 = min(h, int(panel["y"]) + int(panel["h"]) + PANEL_CROP_MARGIN)
        except (KeyError, TypeError, ValueError):
            return frame
        if x1 - x0 < 1 or y1 - y0 < 1:
            return frame
        return frame[y0:y1, x0:x1]

    def _decode_multi(self, detector, image, label: str) -> Optional[str]:
        """detectAndDecodeMulti で読み、最初に中身のあったものを返す"""
        retval, decoded_info, _points, _straight = detector.detectAndDecodeMulti(image)
        if not retval:
            return None
        for info in decoded_info:
            if info:
                logger.info("二次元コードを読めました (%s): %s", label, info)
                return info
        return None

    def decode_file(self, path: str) -> Optional[str]:
        """
        画像ファイルから二次元コードを読む（実機の撮影 → 読み取りの道）。

        :returns: 読めた文字列。読めなければ ""。OpenCV が無ければ None
        """
        cv2 = _cv2()
        if cv2 is None:
            logger.error("OpenCVが入っていないためQRデコードできません"
                         "（sudo apt-get install python3-opencv）")
            return None

        if not os.path.exists(path) or os.path.getsize(path) == 0:
            logger.warning("画像ファイルが存在しないか空です: %s", path)
            return ""

        img = cv2.imread(path)
        if img is None:
            logger.warning("画像の読み込みに失敗しました: %s", path)
            return ""

        found = self.decode_frame(img)
        if found:
            return found
        logger.warning("すべての手法で二次元コードが検出されませんでした")
        return ""

    def find_card_panels(self, frame, min_side: int = 8, max_side_ratio: float = 0.9) -> list:
        """
        映像の中から「カードらしい四角」を探す。

        白地に黒い模様が乗った、おおむね正方形の板を探す。二次元コードとして
        読めるだけの解像度が無いときに、カードが写っているかどうかだけを
        画素から判断するために使う。

        :returns: [{'x','y','w','h','area','fill'}] を大きい順に。'fill' は
                  四角の中の暗い画素の割合。
        """
        cv2 = _cv2()
        np = _np()
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


