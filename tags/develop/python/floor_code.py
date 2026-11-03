"""
床の二次元コード（規約 7.3.2 の位置補助情報。50mm 四方、中身は `A1`〜`D4`）を読む。

【ヒントカードとは別物】
ヒントカードは正面から見た 25モジュール角で、効くのは「寄る」ことだった
（`Common/qr_decoder.py`）。床のコードは**浅い角度で見下ろす**ので、寄っても
縦だけが潰れる。同じ読み方では通らない。

【要点：縦に引き伸ばしてから読む】
床を斜めから見るとコードは縦方向に縮む（実機の1枚で 2.3〜3 : 1）。モジュールが
正方形でなくなるため、OpenCV も zxing-cpp も**そのままでは復号できない**。
**映像を縦に3倍前後へ引き伸ばすと読める。**

**これが効くのは遠くのコード（潰れが強い）だけ。** 至近で撮った1枚は潰れが 2.3:1 と
軽く、落ちる原因はぼけだった（Laplacian 6）。そちらは焦点とシャッターの話で、
引き伸ばしでは救えない（`Common/camera_tuning.py`、`Docs/position_correction_design.md` 5.1）。

実機の1枚（1920x1080、コース上の4個が写ったもの）での実測。

    そのまま           0個
    縦3.0倍 → `A2` `B2` の2個（58ms）
    縦3.5倍 / 4.0倍 → 同じ2個

**残りの2個は読めない。** そちらは照り返しで白っぽくなっている側にあり、
縦倍率を 1.5〜6.5 まで 0.25 刻みで振っても、鮮鋭化・CLAHE・大津を噛ませても
1回も読めなかった。**ぼけではなく、コントラストが無い。**

【前処理は足さない（zxing-cpp のとき）】
鮮鋭化・CLAHE・大津は、読める枚数を増やさずに時間だけ増やした（CLAHE は
2倍以上かかる）。**素のグレースケールを縦に伸ばすだけが一番良い。**

【zxing-cpp が無ければ OpenCV で読む。ただし落ちる】
同じ1枚で比べると差が大きい。

    zxing-cpp   2個（`A2` `B2`）  縦3倍で 51ms、素のグレースケールのまま
    OpenCV      1個（`B2` だけ）  70〜270ms、**大津で二値化しないと読めない**

**走行体には zxing-cpp が入っていない**（2026-09-20 に確認）。
`Common/setup_venv.sh` をやり直すこと。入るまでは OpenCV で半分しか読めない。

【解像度は効く。そして本番はいま足りていない】
同じ1枚を落として試すと、1920x1080 で2個、1280x720 では1個しか読めない。

**その1枚が 1920x1080 なのは、当時の1枚撮りが v4l2-ctl の撮影専用経路
（`config.CAMERA_COMMAND`）だったから。** 設定（`CAMERA_WIDTH/HEIGHT`）は
当時も今も 1280x720 で、そちらは色の読み取り（30Hz）に合わせた値。
**1枚撮りは 2026-09-08 に連続取り込みと同じ1台・同じ解像度へ移った**ので、
**いま撮り直すと 1280x720 ＝ 読める数が半分になる。**

床のコードを当てにするなら、取り込みの解像度を上げるかを決めること。
1920x1080 は 25.4Hz まで落ち CPU が1コアを超える（CLAUDE.md）。
1280x960 なら 30Hz を維持できるが、**4:3 が縦の画素密度を上げるとは限らない**
（多くの UVC は 16:9 が 4:3 の縦を切り出したもので、増えるのは画角）。実機で測ること。
"""

from __future__ import annotations

import logging
from typing import List, Optional, Tuple

logger = logging.getLogger(__name__)


#: 縦に引き伸ばす倍率。手前ほど強く潰れるので、当たった順ではなく全部試す。
#:
#: 実機の1枚では 3.0 / 3.5 / 4.0 が当たり、2.0 は1個だけ、4.5 以上は落ちた。
STRETCH_FACTORS = (3.0, 3.5, 4.0, 2.5)

#: 引き伸ばしたあとに作ってよい画像の高さ[px]。
#:
#: 1280x720 を縦4倍にすると 1280x2880。これ以上は復号が伸びるだけで当たらない。
MAX_STRETCHED_HEIGHT = 4400

#: 床のコードとして扱う中身。コース図の格子の呼び名（規約 7.3.2）。
#: これに合わないものは、ヒントカードなど別のコードとみなして捨てる。
ID_COLUMNS = "ABCD"
ID_ROWS = "1234"


try:
    from opencv import OpenCv
except ImportError:  # パッケージとして読み込まれた場合
    try:
        from .opencv import OpenCv
    except (ImportError, ValueError):
        from Common.opencv import OpenCv


class FloorCodeSighting:
    """読めた床のコード1個。座標は**元の映像**の画素で持つ"""

    def __init__(self, text: str, corners: List[Tuple[float, float]], stretch: float):
        #: 中身（`A1`〜`D4`）
        self.text = text
        #: 四隅（左上・右上・右下・左下）。元の映像の画素
        self.corners = corners
        #: 読めたときの縦倍率。写りの潰れ具合の目安になる
        self.stretch = stretch

    @property
    def center(self) -> Tuple[float, float]:
        """四隅の重心。どこに写っていたか"""
        xs = [p[0] for p in self.corners]
        ys = [p[1] for p in self.corners]
        return (sum(xs) / len(xs), sum(ys) / len(ys))

    @property
    def width_px(self) -> float:
        """映像の横方向に何px 占めるか（四隅を囲む枠）"""
        xs = [p[0] for p in self.corners]
        return max(xs) - min(xs)

    @property
    def height_px(self) -> float:
        """映像の縦方向に何px 占めるか。横との比が潰れ具合"""
        ys = [p[1] for p in self.corners]
        return max(ys) - min(ys)

    @property
    def compression(self) -> float:
        """
        潰れ具合（横 ÷ 縦）。床を浅く見るほど、また遠いほど大きくなる。

        **カメラの取り付け角の目安になる**（潰れ ≒ 1/sin(見下ろす角)）。
        実機の1枚では手前 3.67:1（約16°）、奥 4.58:1（約13°）だった。
        """
        height = self.height_px
        return (self.width_px / height) if height else 0.0

    def __repr__(self) -> str:
        cx, cy = self.center
        return ("FloorCodeSighting(%r, center=(%.0f, %.0f), %.0fx%.0fpx, stretch=%.1f)"
                % (self.text, cx, cy, self.width_px, self.height_px, self.stretch))


class FloorCodeDecoder:
    """
    映像から床の二次元コードを読む。

    読み方は1つだけ（モジュールの説明を参照）。

        グレースケール → 縦に引き伸ばす → 映像まるごと読む

    読むのは zxing-cpp。無ければ OpenCV に落ちるが、**読める数が半分になり
    数倍遅くなる**ので、`Common/setup_venv.sh` で入れること。
    """

    def __init__(self, stretch_factors=STRETCH_FACTORS) -> None:
        self.stretch_factors = tuple(stretch_factors)
        self._zxing = None
        self._zxing_tried = False
        self._detector = None

    # --- 読む -----------------------------------------------------------------

    def read(self, frame) -> List[FloorCodeSighting]:
        """
        映像から読めた床のコードを全部返す。読めなければ空。

        倍率を変えて何度も読むので、**同じコードが複数の倍率で当たる**。
        中身ごとに、いちばん大きく写っていた1件だけを残す。
        """
        cv2 = OpenCv.cv2()
        if cv2 is None or frame is None:
            return []

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        height = gray.shape[0]
        zxing = self._get_zxing()

        best = {}
        for stretch in self.stretch_factors:
            if height * stretch > MAX_STRETCHED_HEIGHT:
                continue
            stretched = cv2.resize(gray, None, fx=1.0, fy=stretch,
                                   interpolation=cv2.INTER_CUBIC)
            if zxing is not None:
                found = self._read_zxing(zxing, stretched, stretch)
            else:
                found = self._read_opencv(cv2, stretched, stretch)
            for sighting in found:
                old = best.get(sighting.text)
                if old is None or sighting.width_px > old.width_px:
                    best[sighting.text] = sighting
        return sorted(best.values(), key=lambda s: -s.width_px)

    def read_ids(self, frame) -> List[str]:
        """読めた中身だけを返す。どこに写っていたかが要らないとき"""
        return [s.text for s in self.read(frame)]

    def _read_zxing(self, zxing, stretched, stretch: float) -> List[FloorCodeSighting]:
        """素のグレースケールのまま読む。前処理は当たりを増やさない"""
        try:
            results = zxing.read_barcodes(stretched, formats=zxing.BarcodeFormat.QRCode)
        except Exception:  # noqa: BLE001 — 読めなくても走行は続ける
            logger.debug("床のコードの読み取りに失敗しました（縦%.1f倍）", stretch, exc_info=True)
            return []
        found = []
        for result in results:
            corners = [(float(p.x), float(p.y) / stretch) for p in
                       (result.position.top_left, result.position.top_right,
                        result.position.bottom_right, result.position.bottom_left)]
            found.extend(self._accept(result.text, corners, stretch))
        return found

    def _read_opencv(self, cv2, stretched, stretch: float) -> List[FloorCodeSighting]:
        """
        zxing-cpp が無いときの逃げ道。

        **大津で二値化してから渡す。** 素のままでは1個も読めなかった。
        """
        detector = self._get_detector(cv2)
        if detector is None:
            return []
        _, binary = cv2.threshold(stretched, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
        try:
            ok, texts, points, _ = detector.detectAndDecodeMulti(binary)
        except Exception:  # noqa: BLE001 — 読めなくても走行は続ける
            logger.debug("床のコードの読み取りに失敗しました（縦%.1f倍）", stretch, exc_info=True)
            return []
        if not ok or points is None:
            return []
        found = []
        for text, quad in zip(texts, points):
            corners = [(float(x), float(y) / stretch) for x, y in quad]
            found.extend(self._accept(text, corners, stretch))
        return found

    def _accept(self, text, corners, stretch: float) -> List[FloorCodeSighting]:
        """中身が格子の名前なら1件、違えば空（ヒントカードを拾わない）"""
        text = (text or "").strip()
        if not self.is_floor_id(text):
            return []
        return [FloorCodeSighting(text, corners, stretch)]

    # --- 見分け ---------------------------------------------------------------

    @staticmethod
    def is_floor_id(text: str) -> bool:
        """床のコードの中身か（`A1`〜`D4`）。ヒントカードを拾わないための歯止め"""
        return (len(text) == 2
                and text[0] in ID_COLUMNS
                and text[1] in ID_ROWS)

    # --- zxing-cpp ------------------------------------------------------------

    def _get_zxing(self):
        """zxing-cpp。無ければ None（1回だけ知らせる）"""
        if not self._zxing_tried:
            self._zxing_tried = True
            try:
                import zxingcpp
                self._zxing = zxingcpp
            except Exception:  # noqa: BLE001 — 無くても OpenCV で読む
                self._zxing = None
                logger.warning("zxing-cpp が無いため OpenCV で読みます。"
                               "床のコードは読める数が半分になります"
                               "（Common/setup_venv.sh で入ります）")
        return self._zxing

    def _get_detector(self, cv2):
        """OpenCV の検出器。`QRCodeDetectorAruco` があればそちら（速い）"""
        if self._detector is None:
            for name in ("QRCodeDetectorAruco", "QRCodeDetector"):
                try:
                    self._detector = getattr(cv2, name)()
                    break
                except Exception:  # noqa: BLE001 — 古い OpenCV には無い
                    continue
        return self._detector

    # --- 写りを測る -----------------------------------------------------------

    @staticmethod
    def sharpness(frame) -> Optional[float]:
        """
        ぼけ具合（Laplacian の分散）。大きいほどシャープ。

        **焦点とシャッターを追い込むときの目盛り。** 実機で 6 まで落ちた1枚は
        復号できず、285 の1枚は読めた（`Docs/position_correction_design.md` 5.1）。
        """
        cv2 = OpenCv.cv2()
        if cv2 is None or frame is None:
            return None
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        return float(cv2.Laplacian(gray, cv2.CV_64F).var())
