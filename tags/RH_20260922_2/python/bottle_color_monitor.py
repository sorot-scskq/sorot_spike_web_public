"""
キャリーボトルのラベル色の認識（Python 版）。
前面カメラの映像から、目の前にあるキャリーボトルが赤・青・黄のどれかを判定する。

画像データ（RGBA / BGR 画素配列）を入力として受け取る。
"""

import math
from typing import Any, Callable, Dict, Optional

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


class BottleColorClassifier:
    """
    カメラ映像の画素配列からキャリーボトル色を判定する分類器。
    """

    DEFAULT_RECOGNITION: Dict[str, Any] = {
        'minSaturation': 0.35,
        'minValue': 0.18,
        'minCoverage': 0.06,
        'roi': {'x': 0.3, 'y': 0.42, 'width': 0.4, 'height': 0.45},
    }

    def __init__(self, default_options: Optional[Dict[str, Any]] = None):
        self.default_options = {**self.DEFAULT_RECOGNITION, **(default_options or {})}

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

        if not frame_data or width <= 0 or height <= 0:
            return empty

        x0 = max(0, int(math.floor(width * roi['x'])))
        y0 = max(0, int(math.floor(height * roi['y'])))
        x1 = min(width, int(math.ceil(width * (roi['x'] + roi['width']))))
        y1 = min(height, int(math.ceil(height * (roi['y'] + roi['height']))))
        total = (x1 - x0) * (y1 - y0)
        if total <= 0:
            return empty

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
                if color:
                    counts[color] += 1

        best = BottleColor.NONE
        best_count = 0
        for color, count in counts.items():
            if count > best_count:
                best = color
                best_count = count

        coverage = (best_count / total) if total > 0 else 0.0
        if coverage < opt['minCoverage']:
            return {'color': BottleColor.NONE, 'coverage': coverage, 'counts': counts}

        return {'color': best, 'coverage': coverage, 'counts': counts}


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
        }

    def reset(self):
        """全状態を初期状態にリセットする。"""
        self.color_info['color'] = BottleColor.NONE
        self.color_info['coverage'] = 0.0
        self.color_info['counts'] = {'red': 0, 'blue': 0, 'yellow': 0}
        self.color_info['held'] = BottleColor.NONE
        self.color_info['last_seen'] = BottleColor.NONE
        self.color_info['last_seen_age'] = 0
