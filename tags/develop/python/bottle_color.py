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
    #
    # 実物のラベルを測った値（2026-09-09、3色を同じ照明で撮影）
    #   赤 6度  青 206度  黄 46度
    #
    # **青の下限は 180 まで下げてある。** 実測 206度に対して下限 195度では
    # 余裕が 11度しかなく、カメラのホワイトバランスが暖色寄りに振れるだけで
    # 範囲から落ちる。実機で赤・黄が 30000画素出るのに青だけ 9000画素という
    # 差が出た（同じ距離・同じ大きさのラベル）。シアン(180)まで含むが、
    # 緑(120)とは離れているので取り違えない。
    #
    # 黄の下限 38度も実測 46度に対して余裕が小さいが、実機で問題が出ていない
    # ため触っていない。赤の上限 22度と近すぎるのも理由（オレンジを挟む）。
    HUE_RANGES: List[Dict[str, Any]] = [
        {'color': RED, 'from': 335, 'to': 22},
        {'color': YELLOW, 'from': 38, 'to': 75},
        {'color': BLUE, 'from': 180, 'to': 260},
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
