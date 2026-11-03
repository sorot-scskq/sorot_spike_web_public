"""
キャリーボトルのラベル色の認識。
前面カメラの映像から、目の前にあるキャリーボトルが赤・青・黄のどれかを判定する。

映像の形が 2通りあるので、判定も 2通り持っている。

    OpenCV の画像 (h, w, 3|4) … 走行体。cv2 + numpy でまとめて数える
    平らな RGBA 配列          … それ以外。画素を1つずつ見る

どちらもしきい値と色相の範囲は同じ（BottleColor.HUE_RANGES）なので、
詰めた値はそのまま両方に効く。

【走行体で 1画素ずつ見てはいけない】
ROI は 640x480 の 0.4x0.45 で 5万画素を超える。Python のループで回すと
1枚あたり数百 ms かかり、ラベルが読める区間（200mm・約0.5秒）を数枚しか
拾えない。cv2.cvtColor と numpy の比較でまとめて数えれば 1枚が数 ms で済む。
"""

import math
from typing import Any, Callable, Dict, Optional, Tuple

try:
    from bottle_color import BottleColor
    from color_converter import ColorConverter
except ImportError:
    try:
        from .bottle_color import BottleColor
        from .color_converter import ColorConverter
    except (ImportError, ValueError):
        from Common.bottle_color import BottleColor
        from Common.color_converter import ColorConverter


try:
    from opencv import OpenCv
except ImportError:  # パッケージとして読み込まれた場合
    try:
        from .opencv import OpenCv
    except (ImportError, ValueError):
        from Common.opencv import OpenCv


class BottleLineAnchor:
    """
    黒線を画面の下から辿り、線が途切れた所（＝ボトルの足元）を返す。

    **キャリーボトルは黒線の上に置いてある。** 走行体はエッジ走行で左右に
    ぶれるので、画面の決まった位置に枠を置くと、ボトルが枠の端に寄り、
    代わりにコーンや床の矢印が枠に入る（REHEARSAL #27 / #29）。
    線を基準にすれば、ぶれてもボトルは枠の中央に来る。

    帯（画面の高さの 1/36）ごとに暗い列の塊を探し、1つ下の帯の位置に近いものを
    上へ繋いでいく。線の途中の青い目印は線の一部として辿る（MARKER_*）。
    灰色の丸などの途切れは線幅の GAP_PER_WIDTH 倍まで飛び越える。
    ボトルの色の部分は線幅の3倍ほどの高さに写るので、そこで止まる。
    """

    #: 間引き（縦横何画素に1つ見るか）。幅 640 未満の映像では間引かない
    STEP = 2
    #: 帯の高さ（画面の高さに対する割合）
    BAND = 1.0 / 36.0
    #: 黒とみなす明るさの上限（0〜255）。実機の線は 20 前後、床の白は 200 超。
    #: **グレーではなく、いちばん明るいチャンネルで比べる。** 濃い青はグレーにすると
    #: 60 ほどになり、ボトルのラベルや目印まで線として辿ってしまう
    DARK_MAX = 70
    #: 帯の中で、その列が暗い行の割合の下限
    FILL = 0.5
    #: 線の幅として採る範囲（画面の幅に対する割合）。
    #: 1280 幅で手前 70px・奥 30px ほど。全面が暗い（カメラの覆いなど）は採らない
    MIN_WIDTH = 0.01
    MAX_WIDTH = 0.15
    #: 線の始まりは画面の下からここまでで見つける（高さに対する割合）
    SEED_SPAN = 0.3
    #: 1帯で動いてよい横の量の下限（幅に対する割合）。線の幅の半分のほうが大きければそちら。
    #: 広げると、ボトルの縁や影の暗い所へ乗り移る（実機の画像で 30px 横へ跳んだ）
    MAX_SHIFT = 0.01
    #: 1帯で細くなってよい割合の下限。線は奥へ向けて少しずつしか細らない
    #: （1帯あたり 1px ほど）。急に細いものはボトルの足元の切れ端
    MIN_WIDTH_RATIO = 0.7
    #: 1帯で太ってよい割合の上限。線は奥へ向けて細るだけなので、急に太いものは
    #: 線ではない。青ボトルの水は線の3倍ほどの幅があり、ここで止まる
    MAX_WIDTH_RATIO = 1.5
    #: 線の途中の青い目印を、線の一部として辿るための色の条件（OpenCV の H は 0〜179）。
    #: 手前の目印は高さ 70px ほどに写り、MAX_GAP では飛び越えられない
    #: （scsk3 の赤ボトルの画像で、目印の下端を足元と取り違えた）
    MARKER_HUE = (90, 130)
    MARKER_MIN_SAT = 128
    MARKER_MIN_VAL = 80
    #: 飛び越える途切れの長さ（その所の線幅に対する倍率）。
    #: 画面の高さで決めると、手前ほど大きく写る丸（線幅の約1.1倍）を越えられず、
    #: 丸の下端を足元と取り違えた（scsk3 の赤ボトルの画像）
    GAP_PER_WIDTH = 2.0
    #: 飛び越える途切れの長さの上限（高さに対する割合）
    MAX_GAP = 0.12

    @staticmethod
    def _runs(cols) -> list:
        """真の列が続く区間 [(開始, 終わり+1), ...]"""
        np = OpenCv.numpy()
        edges = np.diff(np.concatenate(([0], cols.astype(np.int8), [0])))
        starts = np.where(edges == 1)[0]
        ends = np.where(edges == -1)[0]
        return list(zip(starts.tolist(), ends.tolist()))

    def find(self, image: Any) -> Optional[Tuple[int, int]]:
        """
        :returns: (線の x, 線が途切れた y) [px]。線が見つからなければ None
        """
        cv2 = OpenCv.cv2()
        np = OpenCv.numpy()
        # 縦横を間引いて見る。線は太いので足りる。全画素だと走行体で 30Hz を割る
        step = self.STEP if image.shape[1] >= 640 else 1
        bgr = np.ascontiguousarray(image[::step, ::step, :3])
        height, width = bgr.shape[:2]
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        marker = cv2.inRange(hsv, (self.MARKER_HUE[0], self.MARKER_MIN_SAT, self.MARKER_MIN_VAL),
                             (self.MARKER_HUE[1], 255, 255))
        black = cv2.inRange(bgr, (0, 0, 0), (self.DARK_MAX,) * 3)
        dark = cv2.bitwise_or(black, marker) > 0

        band = max(1, int(round(height * self.BAND)))
        min_w = max(2, int(width * self.MIN_WIDTH))
        max_w = int(width * self.MAX_WIDTH)
        max_gap_cap = int(height * self.MAX_GAP)
        seed_top = height - int(height * self.SEED_SPAN)

        x = None
        line_w = 0
        top = None
        gap = 0
        y1 = height
        while y1 - band >= 0:
            y0 = y1 - band
            cols = dark[y0:y1].mean(axis=0) >= self.FILL
            runs = [(s, e) for s, e in self._runs(cols) if min_w <= e - s <= max_w]
            y1 = y0
            if x is None:
                if runs:
                    s, e = max(runs, key=lambda r: r[1] - r[0])
                    x, line_w, top = (s + e) / 2.0, e - s, y0
                elif y0 < seed_top:
                    return None
                continue

            shift = max(line_w / 2.0, width * self.MAX_SHIFT)
            near = [r for r in runs if abs((r[0] + r[1]) / 2.0 - x) <= shift
                    and line_w * self.MIN_WIDTH_RATIO <= (r[1] - r[0])
                    <= line_w * self.MAX_WIDTH_RATIO]
            if near:
                s, e = min(near, key=lambda r: abs((r[0] + r[1]) / 2.0 - x))
                x, line_w, top = (s + e) / 2.0, e - s, y0
                gap = 0
            else:
                gap += band
                if gap > min(max_gap_cap, max(2 * band, line_w * self.GAP_PER_WIDTH)):
                    break
        return int(round(x * step)), int(top * step)


class BottleColorClassifier:
    """
    カメラ映像の画素配列からキャリーボトル色を判定する分類器。
    """

    @staticmethod
    def _as_image(frame_data: Any):
        """
        OpenCV の画像として扱えるか。扱えなければ None（平らな配列として見る）。

        走行体のカメラは (h, w, 3) の BGR を返す。これを平らな RGBA として
        frame_data[(y * width + x) * 4] で読むと、画素ではなく行を拾ってしまう。
        """
        shape = getattr(frame_data, 'shape', None)
        if shape is None or len(shape) != 3 or shape[2] not in (3, 4):
            return None
        if OpenCv.cv2() is None or OpenCv.numpy() is None:
            return None
        return frame_data

    @classmethod
    def _count_colors_image(cls, image: Any, x0: int, y0: int, x1: int, y1: int,
                            opt: Dict[str, Any]) -> Optional[Dict[str, int]]:
        """
        OpenCV の画像から、ROI 内の色ごとの画素数を数える。

        色相の範囲としきい値は平らな配列のときと同じものを使う。OpenCV の H は
        0〜179（度の半分）なので 2倍して度に戻す。S と V は 0〜255 なので 0〜1 にする。
        """
        cv2 = OpenCv.cv2()
        np = OpenCv.numpy()
        if cv2 is None or np is None:
            return None
        try:
            roi = image[y0:y1, x0:x1]
            if roi.size == 0:
                return None
            if roi.shape[2] == 4:
                roi = cv2.cvtColor(roi, cv2.COLOR_BGRA2BGR)
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            hue = hsv[:, :, 0].astype(np.float32) * 2.0
            sat = hsv[:, :, 1].astype(np.float32) / 255.0
            val = hsv[:, :, 2].astype(np.float32) / 255.0

            bright = (sat >= opt['minSaturation']) & (val >= opt['minValue'])

            counts = {'red': 0, 'blue': 0, 'yellow': 0}
            blobs = {'red': 0, 'blue': 0, 'yellow': 0}
            for r in BottleColor.HUE_RANGES:
                if r['color'] not in opt['colors']:
                    continue
                if r['from'] <= r['to']:
                    inside = (hue >= r['from']) & (hue <= r['to'])
                else:
                    # 赤は 0度をまたぐ
                    inside = (hue >= r['from']) | (hue <= r['to'])
                mask = inside & bright
                counts[r['color']] = int(np.count_nonzero(mask))
                blobs[r['color']] = cls._largest_blob(cv2, np, mask)
            return counts, blobs
        except Exception:
            return None

    @staticmethod
    def _largest_blob(cv2, np, mask) -> int:
        """
        その色の画素のうち、**いちばん大きな一続きの塊**の画素数。

        数を数えるだけでは、コースの後ろに立つ人の服や、床に散った色を
        ボトルのラベルと区別できない。ラベルは一続きの面なので、
        ばらばらに散った同じ画素数より塊が大きくなる。

        数えられなければ全体の画素数を返す（判定そのものは止めない）。
        """
        try:
            n, _labels, stats, _c = cv2.connectedComponentsWithStats(
                mask.astype(np.uint8), 8)
            if n <= 1:
                return 0
            # 0番は背景。1番以降のうち最大
            return int(stats[1:, cv2.CC_STAT_AREA].max())
        except Exception:
            return int(np.count_nonzero(mask))

    @staticmethod
    def _min_coverage(color: str, opt: Dict[str, Any]) -> float:
        """その色を採るのに要る占有率。色ごとの指定が無ければ共通の minCoverage"""
        return opt.get('minCoverageByColor', {}).get(color, opt['minCoverage'])

    @classmethod
    def _decide(cls, counts: Dict[str, int], total: int, opt: Dict[str, Any],
                blobs: Optional[Dict[str, int]] = None) -> Dict[str, Any]:
        """
        しきい値を満たした色のうち、優先順（colorPriority）の前のものを採る。
        満たす色が無ければ「見えていない」。

        **塊が分かるときは、塊の大きさで比べる。** ROI にはコースの後ろが写る。
        立っている人の服や床の模様は、数だけなら勝てても一続きにはならない。
        ラベルは面なので、散らばったものより塊が大きくなる（`_largest_blob`）。
        blobs が None のとき（平らな RGBA の経路）は数で比べる。シミュレータは
        背景に人が立たないため、そちらは数のままでよい。

        **大きさだけで選んではいけない。** 黒線にある青い目印はラベルより
        ずっと大きく写るので、青も数えると赤・黄のラベルが必ず負ける。
        同時にしきい値を満たしたときは colorPriority の前にあるほうを採る。
        """
        values = blobs if blobs else counts
        priority = list(opt.get('colorPriority', ()))

        def rank(color: str):
            index = priority.index(color) if color in priority else len(priority)
            return (index, -values.get(color, 0))

        def coverage_of(color: str) -> float:
            return (values.get(color, 0) / total) if total > 0 else 0.0

        def solid_of(color: str) -> float:
            # その色の画素が散らばっていないか。半分以上が一続きなら面とみなす
            count = counts.get(color, 0)
            return (values.get(color, 0) / count) if count > 0 else 0.0

        chosen = BottleColor.NONE
        for color in sorted(values, key=rank):
            if values.get(color, 0) <= 0:
                continue
            if coverage_of(color) < cls._min_coverage(color, opt):
                continue
            if blobs and solid_of(color) < opt.get('minSolidRatio', 0.0):
                continue            # 散らばっている。ラベルではなく背景とみなす
            chosen = color
            break

        # 採れなかったときは、いちばん大きかった色の数字を残す（落ちた理由を追うため）
        largest = max(values, key=lambda c: values.get(c, 0))
        reported = chosen
        if reported == BottleColor.NONE and values.get(largest, 0) > 0:
            reported = largest

        result = {'color': chosen, 'coverage': coverage_of(reported), 'counts': counts}
        if blobs:
            result['blobs'] = blobs
            result['solid'] = solid_of(reported)
        return result

    DEFAULT_RECOGNITION: Dict[str, Any] = {
        'minSaturation': 0.35,
        'minValue': 0.18,
        #: 枠のうち、その色が占める割合の下限。
        #:
        #: **0.06 から下げた。** 縦を画面いっぱいに広げて分母が倍になったのと、
        #: 実機で青が他の色の 1/3 しか出ないため（30cm で赤・黄 30000画素に対し
        #: 青 9000画素。2026-09-09）。0.02 なら青 2.4% が通り、赤 8.1% は余裕。
        #:
        #: 下げると背景を拾いやすくなるが、下の minSolidRatio が
        #: 「散らばっているものは採らない」で受け止める。
        'minCoverage': 0.02,
        #: 色ごとの下限（ここに無い色は上の minCoverage）。
        #:
        #: **青だけ高くしてある。** 黒線の途中にある青い目印が ROI に入るため、
        #: 赤・黄と同じ下限だと目印をラベルとして採ってしまう。
        #:
        #: 上げられる幅は狭い。実機のラベルは 30cm で 9000画素＝占有率 2.4%
        #: しか出ない（2026-09-09）ので、0.03 を超えると本物の青ラベルが
        #: 遠いうちは通らなくなる。**目印をまだ拾うなら、ここを上げる前に
        #: ROI を下げるか、判別する区間を目印より先にずらすこと。**
        'minCoverageByColor': {BottleColor.BLUE: 0.03},
        #: その色の画素のうち、いちばん大きな塊が占める割合の下限。
        #: **散らばった背景を弾くためのもの。** ラベルは一続きの面なので、
        #: ほとんどが1つの塊になる。服のしわや影で割れることを見込んで半分にする。
        #: 塊を数えられる経路（OpenCV）でだけ効く
        'minSolidRatio': 0.5,
        #: 判定に使う色。ここに無い色の画素は数えない（背景と同じ扱い）
        'colors': [BottleColor.RED, BottleColor.YELLOW, BottleColor.BLUE],
        #: 同時にしきい値を満たしたときの優先順。**青はいちばん後ろ。**
        #: 黒線の青い目印はラベルより大きく写るので、大きさで比べると
        #: 赤・黄のラベルが負ける（`_decide`）
        'colorPriority': [BottleColor.RED, BottleColor.YELLOW, BottleColor.BLUE],
        #: 見る範囲（画面に対する割合）。**黒線が見つからないときだけ使う**（下の lineRoi）。
        #:
        #: **縦は画面いっぱい。** ボトルは距離で写る高さが変わる（遠いほど上）。
        #: 縦を切ると、その距離のときだけラベルが枠から外れる。横は 30〜70% に
        #: 絞ってあり、真正面のボトルだけを見る。
        #:
        #: **縦を広げたぶん、占有率の分母が倍になっている。** 同じ画素数でも
        #: 占有率は半分になるので、minCoverage を下げてある（下記）。
        #:
        #: シミュレータ（sim/bottle/bottle-camera.js）とは別の値。カメラも画角も
        #: 違うため。しきい値と色相は引き続き同じものを使う。
        'roi': {'x': 0.3, 'y': 0.0, 'width': 0.4, 'height': 1.0},
        #: 黒線を基準に枠を置く（BottleLineAnchor）。線が見つからなければ上の roi。
        #:
        #: width  … 枠の幅（画面の幅に対する割合）。線を中心に置く。**上の roi より狭い**
        #:           のは、ぶれを線で吸収できるぶん端を削れるから。枠に入っていた
        #:           コーン・床の矢印・右の桃色の帯を外す。**25% より狭めない。**
        #:           250mm 手前でボトルは約 330px（1280 幅の 26%）に写る
        #: below  … 線が途切れた所から下へ、どこまで枠に入れるか（高さに対する割合）。
        #:           ボトルより手前の線の上にある青い目印を外す
        #: above  … 線が途切れた所から上へ、どこまで枠に入れるか（足元の y に対する割合）。
        #:           奥のコーンを外す（青ボトルを赤と確定した。2026-09-26 scsk3）。
        #:           ボトルは足元の y とほぼ同じ高さに写り（地平線が画面の上端近く）、
        #:           色の付いた部分は足元から上へ 0.5〜0.62。奥のコーンの下端は
        #:           足元 y=400 のとき 0.81 にある
        #:
        #: **占有率の分母は上の roi の面積のまま。** 枠を狭めて分母まで小さくすると、
        #: 同じ画素数で占有率が上がり、実機で合わせた minCoverage の意味が変わるため
        'lineRoi': {'enable': True, 'width': 0.3, 'below': 0.05, 'above': 0.8},
        #: 線基準の枠で見ているときの、色ごとの占有率の下限（minCoverageByColor の代わり）。
        #:
        #: **青も赤・黄と同じ 0.02。** 青を重くしていたのは黒線の青い目印のためで、
        #: 線基準の枠ではボトルより手前の目印は枠の外になる（lineRoi.below）。
        #: 青ボトルは 0.03 を待つあいだに近づき、0.038〜0.052 でやっと確定していた
        #: （赤・黄は 0.022 前後。scsk3 の確定画像 40枚）。青でないボトルの回に
        #: 枠へ入る青は 0.005〜0.009
        'minCoverageByColorOnLine': {},
        #: 線基準の枠で見ているときの優先順（colorPriority の代わり）。
        #:
        #: **空＝塊の大きさで選ぶ。** 赤・黄を先に採るのは、目印の青がラベルより
        #: 大きく写るからで、線基準の枠では手前の目印が外にある。逆に優先順のままだと、
        #: 奥のコーン（赤）が少し入っただけで青ボトルを赤と確定する（2026-09-26 scsk3）
        'colorPriorityOnLine': [],
    }

    def __init__(self, default_options: Optional[Dict[str, Any]] = None):
        self.default_options = {**self.DEFAULT_RECOGNITION, **(default_options or {})}
        self.line_anchor = BottleLineAnchor()

    def _roi_on_line(self, image: Any, width: int, height: int, y0: int, y1: int,
                     opt: Dict[str, Any]) -> Optional[Tuple[int, int, int, int]]:
        """黒線を基準にした枠 (x0, y0, x1, y1)。線が見つからなければ None"""
        line = opt.get('lineRoi') or {}
        if not line.get('enable'):
            return None
        try:
            found = self.line_anchor.find(image)
        except Exception:
            return None
        if found is None:
            return None
        x, top = found
        half = int(width * line['width'] / 2)
        x0 = max(0, x - half)
        x1 = min(width, x + half)
        y0 = max(y0, top - int(top * line.get('above', 1.0)))
        y1 = min(y1, top + int(height * line['below']))
        if x1 <= x0 or y1 <= y0:
            return None
        return x0, y0, x1, y1

    def classify(self, frame_data: Any, width: int, height: int, options: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """
        映像の画素配列 (RGBA または RGB) からキャリーボトルの色を判定する。
        """
        opt = {**self.default_options, **(options or {})}
        roi = {**self.default_options['roi'], **(options.get('roi', {}) if options else {})}

        empty = {
            'color': BottleColor.NONE,
            'coverage': 0.0,
            'counts': {'red': 0, 'blue': 0, 'yellow': 0},
        }

        image = self._as_image(frame_data)
        if image is None and not frame_data:
            return empty
        if width <= 0 or height <= 0:
            return empty

        x0 = max(0, int(math.floor(width * roi['x'])))
        y0 = max(0, int(math.floor(height * roi['y'])))
        x1 = min(width, int(math.ceil(width * (roi['x'] + roi['width']))))
        y1 = min(height, int(math.ceil(height * (roi['y'] + roi['height']))))
        total = (x1 - x0) * (y1 - y0)
        if total <= 0:
            return empty

        if image is not None:
            box = self._roi_on_line(image, width, height, y0, y1, opt)
            source = 'line' if box else 'fixed'
            box = box or (x0, y0, x1, y1)
            if source == 'line':
                if 'minCoverageByColorOnLine' in opt:
                    opt = {**opt, 'minCoverageByColor': opt['minCoverageByColorOnLine']}
                if 'colorPriorityOnLine' in opt:
                    opt = {**opt, 'colorPriority': opt['colorPriorityOnLine']}
            counted = self._count_colors_image(image, *box, opt)
            if counted is None:
                return empty
            counts, blobs = counted
            result = self._decide(counts, total, opt, blobs)
            result['roi'] = box
            result['roi_source'] = source
            return result

        counts = {'red': 0, 'blue': 0, 'yellow': 0}

        for y in range(y0, y1):
            for x in range(x0, x1):
                offset = (y * width + x) * 4
                r = frame_data[offset]
                g = frame_data[offset + 1]
                b = frame_data[offset + 2]

                hsv = ColorConverter.rgb_to_hsv(r, g, b)
                if hsv['s'] < opt['minSaturation'] or hsv['v'] < opt['minValue']:
                    continue

                color = BottleColor.from_hue(hsv['h'])
                if color and color in opt['colors']:
                    counts[color] += 1

        return self._decide(counts, total, opt)


# 「最後に見えた色」を使ってよい古さ[フレーム]。
#
# 30fps で 60フレーム = 2秒。ボトルへ寄せてから抱えるまでは 1秒ほどなので、
# 寄せながら見えていた色は充分に届く。それより古いものは、別の場面で見た色を
# 誤って採る恐れがあるので使わない。
LAST_SEEN_MAX_AGE = 60


class BottleColorMonitor:
    """
    カメラ映像からキャリーボトルの色を読み、抱えた色を保持・管理するモニタクラス。
    """

    def __init__(self, classifier: Optional[BottleColorClassifier] = None, options: Optional[Dict[str, Any]] = None):
        self.classifier = classifier or BottleColorClassifier(options)
        self.options = options
        self.color_info: Dict[str, Any] = {
            'color': BottleColor.NONE,
            'coverage': 0.0,
            'counts': {'red': 0, 'blue': 0, 'yellow': 0},
            'held': BottleColor.NONE,
            # 最後に見えた色と、それが何フレーム前か。
            # 抱えた瞬間はラベルがカメラに写らない（近すぎて画角の下に外れる）ので、
            # 「いま見えている色」だけでは永久に判定できない。近づきながら見えて
            # いた色を覚えておき、抱えたときにそれを採用する。
            'last_seen': BottleColor.NONE,
            'last_seen_age': 0,
        }

    def set_options(self, options: Dict[str, Any]):
        """しきい値オプションを上書きする。"""
        self.options = {**(self.options or {}), **options}

    def read(self, frame_data: Any = None, width: int = 0, height: int = 0) -> str:
        """映像を1枚読み取り、色を判定する。"""
        if frame_data is None or width <= 0 or height <= 0:
            return self.color_info['color']

        try:
            result = self.classifier.classify(
                frame_data,
                width,
                height,
                self.options,
            )
            self.color_info['color'] = result['color']
            self.color_info['coverage'] = result['coverage']
            self.color_info['counts'] = result['counts']
            # どこを見て決めたか（画像に枠を描く・線を見失っていないかを追う）
            self.color_info['roi'] = result.get('roi')
            self.color_info['roi_source'] = result.get('roi_source')
            if result['color'] != BottleColor.NONE:
                self.color_info['last_seen'] = result['color']
                self.color_info['last_seen_age'] = 0
            else:
                self.color_info['last_seen_age'] += 1
            return result['color']
        except Exception:
            return self.color_info['color']

    def hold(self) -> str:
        """
        抱えた色を覚える（初回のみ）。

        いま見えていればそれを、見えていなければ「最後に見えた色」を採る。

        抱えた瞬間、ボトルはカメラに近すぎてラベルが画角の下に外れる
        （シミュレータの実測で、写るのは 180mm 以遠。アームで挟む位置は
        54〜125mm）。「いま見えている色」しか採らないと、抱えたあとは
        いつまでも判定できず、未判定の逃げ道（SCV 13）へ落ちる。

        古すぎる記憶は使わない。ボトルから離れたまま抱えたことになった
        （すり抜けなど）ときに、まったく別のときに見た色を採らないため。
        """
        if self.color_info['held'] != BottleColor.NONE:
            return self.color_info['held']

        if self.color_info['color'] != BottleColor.NONE:
            self.color_info['held'] = self.color_info['color']
        elif (self.color_info['last_seen'] != BottleColor.NONE
                and self.color_info['last_seen_age'] <= LAST_SEEN_MAX_AGE):
            self.color_info['held'] = self.color_info['last_seen']
        return self.color_info['held']

    def release(self):
        """抱えた色をリセットする。"""
        self.color_info['held'] = BottleColor.NONE

    def get_held_color(self) -> str:
        """現在抱えている色を返す。"""
        return self.color_info['held']

    def get_color_info(self) -> Dict[str, Any]:
        """判定結果情報のコピーを返す。"""
        return {
            'color': self.color_info['color'],
            'coverage': self.color_info['coverage'],
            'counts': {**self.color_info['counts']},
            'held': self.color_info['held'],
            'roi': self.color_info.get('roi'),
            'roi_source': self.color_info.get('roi_source'),
        }

    def reset(self):
        """全状態を初期状態にリセットする。"""
        self.color_info['color'] = BottleColor.NONE
        self.color_info['coverage'] = 0.0
        self.color_info['counts'] = {'red': 0, 'blue': 0, 'yellow': 0}
        self.color_info['held'] = BottleColor.NONE
        self.color_info['last_seen'] = BottleColor.NONE
        self.color_info['last_seen_age'] = 0
        self.color_info['roi'] = None
        self.color_info['roi_source'] = None
