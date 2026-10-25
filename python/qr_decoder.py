"""
二次元コードの読み取りと、カードが写っている場所（カード領域）の検出。

【小さく写ったら読めない。だから「探す」と「読む」を分ける】
5cm のカードは 1280x720 のごく一部にしか写らない。二次元コードは 25モジュール
角なので、カードが 120px 未満で写ると 1モジュールが 4px を切り、どんな手を
使っても読めなくなる（合成画像 252枚での実測：120px で 91%、90px で 72%、
70px で 58%、50px では 0%）。

そこで

    1. 探す … 読まずに場所だけ見つける（`detect`）。読むより通りやすい
    2. 読む … 見つけた所を切り抜き、400px / 800px へ拡大してから読む

拡大は情報を増やさないが、OpenCV の検出は 1モジュールあたりの画素が少ないと
外すため、補間で埋めると通る。切り抜きは小さいので拡大しても速い。

この2段で、合成画像 252枚の読み取りは 48% → 71%（1枚あたり 47ms）になった。
**残りはカードが小さすぎる・白飛び・照り返しで、算法では埋まらない。**
詰めるべきは写し方（`Docs/et_rally_route.md`）。

【zxing-cpp があれば先に試す】
OpenCV の検出は、実機の映像（透明ボトルに貼ったカード、少しぼけた 100px 前後の
写り）を落とすことがある。実機で読めなかった1枚（ET_2026_REHEARSAL #10、
2026-09-16）は、本番の道では 1.9秒かけて読めず、zxing-cpp は映像まるごとを
4ms で読めた。入っていれば先に試し、読めなければ従来の道へ進む。
**入っていなくても動く**（`Common/setup_venv.sh` が入れる。無ければ 1回だけログに出す）。

【遅延 import にしている理由】
走行体に OpenCV が入っていない環境で、モジュールの先頭の import が失敗して
ev3_python 全体が起動できなくなるのを避ける。使うときに取りにいき、
無ければ読み取りを諦めて走行は続ける。
"""

from __future__ import annotations

import logging
import os
from typing import Optional

logger = logging.getLogger(__name__)


#: 映像まるごとを読むときの拡大で、作ってよい画像の長辺[px]。
#:
#: 960x576 を 1920x1152 にすると、この 1段だけで 120ms かかった
#: （sorot_spike_web での実測）。長辺がこの値を超えていれば拡大しない。
MAX_UPSCALED_LONG_SIDE = 1280

#: 切り抜きを読むときに、拡大して試す長辺[px]。
#:
#: 小さく写ったコードは、素の大きさでは OpenCV が検出を外す。切り抜きは
#: 小さいので、ここまで拡大しても数msで済む。
DECODE_LONG_SIDES = (400, 800)

#: カード領域を切り抜くときに足す余白[px]。
#:
#: 二次元コードは周りに余白（クワイエットゾーン）が無いと読めない。外形
#: ちょうどで切ると白い縁が入らず読めなくなることがあるため足す。
CARD_CROP_MARGIN = 8

#: `detect` で見つけた枠に足す余白（枠の長辺に対する割合）。
#: 検出した四隅はコードの外周ぎりぎりなので、余白を足さないと読めない。
LOCATE_MARGIN_RATIO = 0.25

#: 白いカードとみなす明るさ。これで見つからなければ Otsu で切り直す
CARD_WHITE_LEVEL = 170


try:
    from opencv import OpenCv
except ImportError:  # パッケージとして読み込まれた場合
    try:
        from .opencv import OpenCv
    except (ImportError, ValueError):
        from Common.opencv import OpenCv


class QrCodeDecoder:
    """
    OpenCV を用いて二次元コードの検出・デコードおよびカード領域の検出を行うクラス。

    読む手は、効いた順に並べてある（モジュールの説明を参照）。

        1. 切り抜きを 400px / 800px へ拡大して読む
        2. 鮮鋭化してから同じく読む（ぼけの補い）
        3. Otsu の二値化（コントラストを立てる）
    """

    def __init__(self) -> None:
        self._detector = None
        self._aruco = None
        self._aruco_tried = False
        self._zxing = None
        self._zxing_tried = False

    def _get_zxing(self):
        """zxing-cpp を返す。入っていなければ None（1回だけ知らせる）"""
        if not self._zxing_tried:
            self._zxing_tried = True
            try:
                import zxingcpp
                self._zxing = zxingcpp
            except Exception:  # noqa: BLE001 — 無くても OpenCV で読む
                self._zxing = None
                logger.info("zxing-cpp が無いため OpenCV だけで読みます"
                            "（Common/setup_venv.sh で入ります。実機の映像で読み取りが上がります）")
        return self._zxing

    def _read_zxing(self, cv2, image, label: str) -> Optional[str]:
        """zxing-cpp で読む。無い・読めないなら None。映像まるごとでも数msで済む"""
        zxing = self._get_zxing()
        if zxing is None or image is None or getattr(image, "size", 0) == 0:
            return None
        try:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
            for found in zxing.read_barcodes(gray, formats=zxing.BarcodeFormat.QRCode):
                if found.text:
                    logger.info("二次元コードを読めました (zxing・%s): %s", label, found.text)
                    return found.text
        except Exception:  # noqa: BLE001 — 読めなくても従来の道へ進む
            logger.debug("zxing-cpp での読み取りに失敗しました", exc_info=True)
        return None

    def _get_detector(self, cv2):
        """QRCodeDetector のインスタンスを取得またはキャッシュから返す。"""
        if self._detector is None:
            try:
                self._detector = cv2.QRCodeDetector()
            except Exception:
                return None
        return self._detector

    def _get_aruco(self, cv2):
        """
        QRCodeDetectorAruco を返す。無い OpenCV なら None。

        **こちらの方が見つける力が強い**（合成画像で場所を当てた割合 83% 対 51%、
        1枚 8.8ms）。OpenCV 4.7 以降にしか無いので、無ければ従来の検出で進む。
        """
        if self._aruco is None and not self._aruco_tried:
            self._aruco_tried = True
            try:
                self._aruco = cv2.QRCodeDetectorAruco()
            except Exception:
                self._aruco = None
                logger.info("QRCodeDetectorAruco が無いため従来の検出で読みます"
                            "（OpenCV 4.7 以降で読み取りが上がります）")
        return self._aruco

    def _detectors(self, cv2) -> list:
        """使える検出器を、効いた順に並べて返す"""
        found = []
        aruco = self._get_aruco(cv2)
        legacy = self._get_detector(cv2)
        if legacy is not None:
            found.append(legacy)
        if aruco is not None:
            found.append(aruco)
        return found

    # --- 読む -----------------------------------------------------------------

    def decode(self, frame, rects=None, zxing_only: bool = False) -> Optional[str]:
        """
        映像から二次元コードの文字列を読む。読めなければ None。

        :param rects: `find_card_rects` の戻り値。渡すと「まず場所を探して、
                      そこだけを拡大して読む」道を通る（速くて当たりやすい）。
                      省略すると映像まるごとを読む。

        場所を探す順は

            1. `detect` でコードの枠を見つける（カードが白く写らなくても通る）
            2. 見つからなければ、渡されたカード領域を大きい順に3つ試す

        **映像まるごとには落とさない。** 1920x1080 では 250ms 級で、粘るあいだに
        試せる枚数が減る。まるごとを試すのは呼ぶ側が間隔を決める
        （`HintCardCapture.FULL_FRAME_EVERY`）。

        :param zxing_only: zxing-cpp があれば、それだけで読む（OpenCV の読み直しを
                           しない）。読めない絵に走行体で1枚約2秒かかり、旋回中に
                           カードが横切る間に1回しか試せなかった（2026-09-26）。
                           zxing-cpp が無ければ、この指定は効かず従来どおり読む
        """
        cv2 = OpenCv.cv2()
        if cv2 is None or frame is None:
            return None

        # zxing-cpp があれば、まず映像まるごとを読む（数ms。場所探しより先でよい）
        text = self._read_zxing(cv2, frame, "映像まるごと")
        if text:
            return text

        if zxing_only and self._get_zxing() is not None:
            for rect in (rects or [])[:3]:
                crop = self._crop_rect(frame, rect)
                gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY) if crop.ndim == 3 else crop
                text = self._read_zxing(cv2, self._upscale(cv2, gray, DECODE_LONG_SIDES[-1]),
                                        "切り抜き拡大")
                if text:
                    return text
            return None

        if rects is None:
            text = self.decode_frame(frame)
            return text if isinstance(text, str) and text.strip() != "" else None

        box = self._locate(cv2, frame)
        if box is not None:
            text = self._read_crop(cv2, self._crop_box(frame, box))
            if text:
                return text

        for rect in rects[:3]:
            text = self._read_crop(cv2, self._crop_rect(frame, rect))
            if text:
                return text
        return None

    def decode_frame(self, frame) -> Optional[str]:
        """
        映像（BGR の配列）まるごとから二次元コードの文字列を読む。読めなければ None。

        カード領域が1つも見つからない撮り方（コードだけが画面いっぱい等）の
        ための逃げ道。全画素を相手にするので重い。
        """
        cv2 = OpenCv.cv2()
        if cv2 is None or frame is None:
            return None

        try:
            detectors = self._detectors(cv2)
            if not detectors:
                return None

            # 1. カラー画像のまま
            for detector in detectors:
                data, _bbox, _ = detector.detectAndDecode(frame)
                if data:
                    logger.info("二次元コードを読めました (カラー): %s", data)
                    return data

            # 2. グレースケール化
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
            found = self._decode_multi(detectors, gray, "グレースケール")
            if found:
                return found

            # 3. Otsu の二値化でコントラストを立てる
            _, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
            found = self._decode_multi(detectors, thresh, "二値化")
            if found:
                return found

            # 4. 2倍に拡大してぼやけ・潰れを補う。
            #    元がもう大きいときは拡大しない（MAX_UPSCALED_LONG_SIDE）
            h, w = gray.shape[:2]
            if max(h, w) * 2 <= MAX_UPSCALED_LONG_SIDE:
                resized = cv2.resize(gray, (w * 2, h * 2), interpolation=cv2.INTER_CUBIC)
                found = self._decode_multi(detectors, resized, "2倍拡大")
                if found:
                    return found

            return None
        except Exception:  # noqa: BLE001 — 読めなくても走行は続ける
            logger.exception("二次元コードの読み取りに失敗しました")
            return None

    def _read_crop(self, cv2, crop) -> Optional[str]:
        """
        切り抜きを読む。拡大 → 鮮鋭化 → 二値化の順に手を変える。

        切り抜きは小さいので、ここまでやっても1枚あたり数十msで済む。
        """
        if crop is None or getattr(crop, "size", 0) == 0:
            return None
        detectors = self._detectors(cv2)
        if not detectors:
            return None
        try:
            gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY) if crop.ndim == 3 else crop

            # zxing-cpp があれば先に。素の切り抜きと、いちばん大きい拡大の2つ
            for image, label in ((gray, "切り抜き"),
                                 (self._upscale(cv2, gray, DECODE_LONG_SIDES[-1]), "切り抜き拡大")):
                found = self._read_zxing(cv2, image, label)
                if found:
                    return found

            for variant, label in ((gray, "拡大"), (self._unsharp(cv2, gray), "鮮鋭化")):
                seen = 0
                for long_side in DECODE_LONG_SIDES:
                    image = self._upscale(cv2, variant, long_side)
                    if image.shape[0] == seen:
                        continue                    # もう拡大できていて同じ画像
                    seen = image.shape[0]
                    found = self._decode_multi(detectors, image, label)
                    if found:
                        return found

            _, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
            return self._decode_multi(detectors, self._upscale(cv2, thresh, DECODE_LONG_SIDES[0]),
                                      "二値化")
        except Exception:  # noqa: BLE001 — 読めなくても走行は続ける
            logger.exception("切り抜きの読み取りに失敗しました")
            return None

    def _decode_multi(self, detectors, image, label: str) -> Optional[str]:
        """detectAndDecode（単・複）で読み、最初に中身のあったものを返す"""
        for detector in detectors:
            try:
                data, _bbox, _ = detector.detectAndDecode(image)
                if data:
                    logger.info("二次元コードを読めました (%s): %s", label, data)
                    return data
                retval, decoded_info, _points, _straight = detector.detectAndDecodeMulti(image)
            except Exception:
                continue
            if not retval:
                continue
            for info in decoded_info:
                if info:
                    logger.info("二次元コードを読めました (%s): %s", label, info)
                    return info
        return None

    def _upscale(self, cv2, gray, long_side: int):
        """長辺が long_side になるまで拡大する。足りていればそのまま"""
        h, w = gray.shape[:2]
        if max(h, w) >= long_side:
            return gray
        factor = long_side / float(max(h, w))
        return cv2.resize(gray, (max(1, int(w * factor)), max(1, int(h * factor))),
                          interpolation=cv2.INTER_CUBIC)

    def _unsharp(self, cv2, gray):
        """ぼけを補う（ぼかした画像を引く）。小さく写った回で効いた"""
        blurred = cv2.GaussianBlur(gray, (0, 0), 1.2)
        return cv2.addWeighted(gray, 1.8, blurred, -0.8, 0)

    # --- 探す -----------------------------------------------------------------

    def locate_code(self, frame):
        """
        読まずに、コードそのものの枠を返す (x0, y0, x1, y1)。無ければ None。

        **読むより見つける方が通りやすい**（合成画像で 83%。白いカードを探す
        `find_card_rects` は 51% で、白い別のものを拾うこともある）。

        枠の大きさは「読めるかどうか」をそのまま表すので、写し方を詰めるときの
        目安に使える（`Tools/read_hint_card.py`）。
        """
        cv2 = OpenCv.cv2()
        if cv2 is None or frame is None:
            return None
        return self._code_box(cv2, frame)

    def _code_box(self, cv2, frame):
        """コードの四隅を囲む枠。余白は足さない"""
        np = OpenCv.numpy()
        if np is None:
            return None
        for detector in self._detectors(cv2):
            try:
                ok, points = detector.detect(frame)
            except Exception:
                continue
            if not ok or points is None:
                continue
            corners = np.asarray(points, dtype=float).reshape(-1, 2)
            if corners.size < 8:
                continue
            return (int(corners[:, 0].min()), int(corners[:, 1].min()),
                    int(corners[:, 0].max()), int(corners[:, 1].max()))
        return None

    def _locate(self, cv2, frame):
        """コードの枠に、読むための余白（クワイエットゾーン）を足して返す"""
        box = self._code_box(cv2, frame)
        if box is None:
            return None
        x0, y0, x1, y1 = box
        margin = max(CARD_CROP_MARGIN, int(max(x1 - x0, y1 - y0) * LOCATE_MARGIN_RATIO))
        h, w = frame.shape[:2]
        return (max(0, x0 - margin), max(0, y0 - margin),
                min(w, x1 + margin), min(h, y1 + margin))

    def _crop_box(self, frame, box):
        """(x0, y0, x1, y1) で切り抜く"""
        x0, y0, x1, y1 = box
        if x1 - x0 < 1 or y1 - y0 < 1:
            return frame
        return frame[y0:y1, x0:x1]

    def _crop_rect(self, frame, rect):
        """カード領域のところを、余白を付けて切り抜く。切り出せなければ元の映像"""
        if frame is None:
            return frame
        try:
            h, w = frame.shape[:2]
            x0 = max(0, int(rect["x"]) - CARD_CROP_MARGIN)
            y0 = max(0, int(rect["y"]) - CARD_CROP_MARGIN)
            x1 = min(w, int(rect["x"]) + int(rect["w"]) + CARD_CROP_MARGIN)
            y1 = min(h, int(rect["y"]) + int(rect["h"]) + CARD_CROP_MARGIN)
        except (KeyError, TypeError, ValueError):
            return frame
        return self._crop_box(frame, (x0, y0, x1, y1))

    def find_card_rects(self, frame, min_side: int = 8, max_side_ratio: float = 0.9) -> list:
        """
        映像の中から「カードらしい四角」を探す。

        白地に黒い模様が乗った、おおむね正方形の領域を探す。二次元コードとして
        読めるだけの解像度が無いときに、カードが写っているかどうかだけを
        画素から判断するために使う。

        **暗い場面では固定のしきい値で取り出せない。** そのときは Otsu で
        場面に合わせて切り直す（実測：平均輝度27の画像は固定170で0個、Otsu で
        見つかって読めた）。

        :returns: [{'x','y','w','h','area','fill'}] を大きい順に。'fill' は
                  四角の中の暗い画素の割合。
        """
        cv2 = OpenCv.cv2()
        np = OpenCv.numpy()
        if cv2 is None or np is None or frame is None:
            return []

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame

        found = self._rects_at(cv2, gray, CARD_WHITE_LEVEL, min_side, max_side_ratio)
        if not found:
            found = self._rects_at(cv2, gray, None, min_side, max_side_ratio)
        return found

    def _rects_at(self, cv2, gray, white_level, min_side, max_side_ratio) -> list:
        """白いところを取り出して四角を拾う。white_level が None なら Otsu で切る"""
        h, w = gray.shape[:2]
        if white_level is None:
            _, mask = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
        else:
            _, mask = cv2.threshold(gray, white_level, 255, cv2.THRESH_BINARY)
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

    # --- ファイルから ----------------------------------------------------------

    def decode_file(self, path: str) -> Optional[str]:
        """
        画像ファイルから二次元コードを読む（実機の撮影 → 読み取りの道）。

        :returns: 読めた文字列。読めなければ ""。OpenCV が無ければ None
        """
        cv2 = OpenCv.cv2()
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

        found = self.decode(img, self.find_card_rects(img))
        if not found:
            found = self.decode_frame(img)      # 1枚撮りは重さより読めることを採る
        if found:
            return found
        logger.warning("すべての手法で二次元コードが検出されませんでした")
        return ""
