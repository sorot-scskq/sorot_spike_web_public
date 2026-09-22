"""
2026年度の設定。共通部分は Common/config.py にある。

ここに書くのは、その年に使う識別値の顔ぶれと、待ち時間だけ
（今年の難所: ボトルデリバリー・ET相撲・ET ラリー）。接続先・撮影のパス・ログ設定・識別値の採番は共通。

    from config import *                # 共通
    from config_2026 import USED_JUDG, TIMEOUT_CONFIG

【ファイル名に年が入っている理由】
シミュレータは Python を /python/<ファイル名> という**平らな置き場**へ配る
（vite.config.js の python-assets）。年ごとのフォルダに同じ名前のファイルが
あると、あとの年で上書きされて片方が消える。名前は全体で 1つにすること。
"""

from config import JUDG_BOTTLE, JUDG_ET_RALLY, JUDG_GATE, JUDG_QR1, JUDG_QR2

#: 今年つかう画像識別値
USED_JUDG = (
    JUDG_BOTTLE,
    JUDG_GATE,
    JUDG_QR1,
    JUDG_QR2,
    JUDG_ET_RALLY,
)

#: 要求区分(prefix)ごとのタイムアウト設定（秒）
TIMEOUT_CONFIG = {
    JUDG_BOTTLE: 3.0,  # ボトル判定
    JUDG_GATE: 3.0,  # ゲート判定
    JUDG_QR1: 2.0,  # QR1復号
    JUDG_QR2: 3.0,  # QR2復号
    JUDG_ET_RALLY: 5.0,  # ET ラリーの経路計算（PC 側で最短経路を組み立てる）
}
