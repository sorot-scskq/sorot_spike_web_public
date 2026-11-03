"""
ヒントカード2 を撮って読む。

シナリオのコマンドに書いて使う。

    "RunInfo": {"SCRIPT": "2026/recog/hint_card2.py", "FUNCNO": 5, "FWD": 0, "NOBLNCE": 1}

**カードへ寄る区間から書くこと。** 止まる行にも書いておくと、読み直さずに粘る
時間が延びる（`2026/hint_card_capture.py`）。近づくほど大きく写るので、止まってから
読み始めるより当たりやすい。色の読み取りが開いているカメラの映像を借りる。

復号した文字列は判定結果の qr2 に載り、そのまま PC へ送られる（本番のヒント2 は
暗号文なので、復号は PC 側でする）。文字列ごとの運び先を
config_2026.QR_ROUTE_FILES に書いておくと、走行中のシナリオが差し替わる。
"""
from config import JUDG_QR2
from hint_card_capture import HintCardCapture


def run(request_id):
    HintCardCapture.get_instance().capture(JUDG_QR2, request_id)
