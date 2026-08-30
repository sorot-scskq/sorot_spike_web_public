"""
走行体 Python の共通設定（接続先・パス・共通の識別値・ログ）。

毎年変わらないものだけを置く。年ごとに変わるぶんは年のフォルダにある
（2025/config_2025.py、2026/config_2026.py）。

    共通（ここ）  接続先、撮影のパスとコマンド、識別値の採番、ログ設定
    年ごと        その年に使う識別値の顔ぶれと、要求区分ごとのタイムアウト

【識別値の番号について】
JUDG_* が正（0始まり）。C++ 側に PhotoObject という 1始まりの enum もあるが、
C++ の実装（PythonCommServer.cpp の sendJsonCommand と ResultRead.cpp の
getLatestRequestId(0..3)）はどちらも 0始まりを使っており、PhotoObject は
どこからも参照されていない。使うのはこちら。

なお、無線通信デバイス（WebSocket）の identifier とは**別物**である。
同じ番号でも意味が違うので混ぜないこと（2026/route_receiver.py を参照）。

【パスについて】
IMAGE_PATH / CAMERA_COMMAND は走行体（Raspberry Pi）で ./ から見たパス。
シミュレータでは撮影しない（canvas の画素を直接読む）ので使わない。
"""

import logging

# --------------------------------------------------------------------------
# 画像識別値（PC 側と同値にすること）
#
# 番号の割り当ては毎年同じ。年で変わるのは「どれを使うか」と「どれだけ待つか」
# なので、それは年のフォルダ（2025/config_2025.py、2026/config_2026.py）で決める。
# 使わない年があっても番号は空けておくこと。詰め直すと PC 側とずれる。
#
# ここは PC 側（PC-System_2026/config/image_config.py）と同じ値にすること。
# ずれると、走行体が頼んだのと違う判定が走る。
# --------------------------------------------------------------------------
JUDG_MINIFIGURE = 0  # ミニフィグ判定（2025 で使用）
JUDG_PLARAIL = 1  # プラレール判定（2025 で使用）
JUDG_BOTTLE = 2  # ボトル判定
JUDG_GATE = 3  # キャリーゲート判定
JUDG_QR1 = 4  # QRコード1判定（2026 で使用）
JUDG_QR2 = 5  # QRコード2判定（2026 で使用）
JUDG_ET_RALLY = 6  # ET ラリー（経路の計算・生成）
CMD_GET_RESULT = 99  # 推論結果取得要求（定期的な問い合わせ用）

DECODE_ERROR_VAL = "DECODE_ERROR"  # QRデコード失敗時のエラー応答値

# --------------------------------------------------------------------------
# 接続先
# --------------------------------------------------------------------------
CXX_HOST = "127.0.0.1"
CXX_PORT = 12345

# PC_HOST = "133.17.164.44"
PC_HOST = "127.0.0.1"
PC_PORT = 49661

# 要求区分(prefix)ごとのタイムアウト設定（秒）。
# どの年でも使う判定ぶんだけを既定として持ち、その年ぶんは load_season() で重ねる
TIMEOUT_CONFIG = {
    JUDG_BOTTLE: 3.0,  # ボトル判定
    JUDG_GATE:   3.0,  # ゲート判定
}
PC_TIMEOUT = 5.0  # デフォルトのタイムアウト時間（秒）

# --------------------------------------------------------------------------
# その年の設定
# --------------------------------------------------------------------------

#: その年の設定モジュール。★毎年ここだけ変える
SEASON_CONFIG_MODULE = "config_2026"

def load_season():
    """
    その年の設定（使う識別値と待ち時間）を重ねる。走行体の起動時に 1回呼ぶ。

    年ごとの値をここから直接持たないのは、共通部分が特定の年を知ってしまうと
    翌年に共通側まで書き換えることになるため。参照は「共通 ← 年」の一方通行に
    しておき、つなぐのは SEASON_CONFIG_MODULE の 1行だけにする。

    :returns: 読み込んだ年の設定モジュール。無ければ None（共通の既定で動く）
    """
    try:
        season = __import__(SEASON_CONFIG_MODULE)
    except ImportError:
        logging.getLogger(__name__).warning(
            "その年の設定 %s が見つかりません。共通の既定で動きます",
            SEASON_CONFIG_MODULE,
        )
        return None
    TIMEOUT_CONFIG.update(getattr(season, "TIMEOUT_CONFIG", {}))
    return season

# --------------------------------------------------------------------------
# 撮影
# --------------------------------------------------------------------------
IMAGE_PATH = "./sorot_spike/RoughSpot/PhotoImage/single/image.jpg"
CAMERA_COMMAND = (
    "v4l2-ctl --device=/dev/video0 "
    "--set-fmt-video=width=1920,height=1080,pixelformat=MJPG "
    "--stream-mmap=3 --stream-count=1 "
    "--stream-to=./sorot_spike/RoughSpot/PhotoImage/single/image.jpg"
)


# ログ設定
def setup_logging():
    import os
    root_logger = logging.getLogger()
    root_logger.setLevel(logging.DEBUG)

    # ロギング用フォルダの存在確認（存在しない場合はカレントフォルダへ出力するフォールバック）
    log_dir = "./sorot_spike"
    if not os.path.exists(log_dir):
        log_path = "./ev3_python.log"
    else:
        log_path = "./sorot_spike/ev3_python.log"

    # 共通のフォーマッター (%(name)s を含めることでモジュール名を出力可能に)
    formatter = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")

    # ファイルハンドラー (ファイル書き出し用)
    file_handler = logging.FileHandler(log_path, encoding="utf-8")
    file_handler.setLevel(logging.DEBUG)
    file_handler.setFormatter(formatter)
    root_logger.addHandler(file_handler)

    # ストリームハンドラー (コマンドプロンプト画面出力用)
    import sys
    stream_handler = logging.StreamHandler(sys.stdout)
    stream_handler.setLevel(logging.DEBUG)
    stream_handler.setFormatter(formatter)
    root_logger.addHandler(stream_handler)
