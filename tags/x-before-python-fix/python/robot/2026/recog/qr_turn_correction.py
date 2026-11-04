"""停止中に床コードを読み、後続の補正旋回を設定する。"""

from qr_turn_correction import QrTurnCorrection


def run(request_id: int) -> None:
    QrTurnCorrection.shared().request(request_id)
