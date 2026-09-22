"""
ボトル色定義モジュール。
ロボコン競技における各種ボトル（キャリーボトル・力士ボトルなど）の色定数と色相判定を提供する。
"""

from typing import Any, Dict, List, Optional


class BottleColor:
    """認識できるボトルのラベル色定数。"""
    RED = 'red'
    BLUE = 'blue'
    YELLOW = 'yellow'
    NONE = 'none'

    # 日本語名とカラーコード表示用
    NAMES: Dict[str, str] = {
        RED: '赤 (RED)',
        BLUE: '青 (BLUE)',
        YELLOW: '黄 (YELLOW)',
        NONE: '未検出 (NONE)',
    }

    # ラベル色の色相[度]の範囲（赤は 0度をまたぐため 335〜22）
    HUE_RANGES: List[Dict[str, Any]] = [
        {'color': RED, 'from': 335, 'to': 22},
        {'color': YELLOW, 'from': 38, 'to': 75},
        {'color': BLUE, 'from': 195, 'to': 260},
    ]

    @classmethod
    def from_hue(cls, hue: float) -> Optional[str]:
        """色相[度]から該当するボトル色を判定して返す。"""
        for r in cls.HUE_RANGES:
            if r['from'] <= r['to']:
                inside = r['from'] <= hue <= r['to']
            else:
                inside = hue >= r['from'] or hue <= r['to']
            if inside:
                return r['color']
        return None
