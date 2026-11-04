"""
ゲートの確認を、走り終わりまで回しっぱなしにする（点検用のシナリオで使う）。

ふだんは PC から経路が届いたときに起きる（`config_2026.on_route_registered`）。
PC を使わずに経路の形を手で書いたシナリオ（`SubScenario/custom_Left_A〜D.json`）では
経路が届かないので、最初の行にこれを書いて起こす。床のコードを使う横の補正（`qr_steer`）も
この流れに乗っているので、これが無いと本番と同じ補正が働かない。

**`FUNCNO 7〜10 / 15` の行に書いてはいけない**（`2026/recog/gate_check.py` と同じ）。
"""
from gate_check_task import GateCheckTask


def run(request_id):
    GateCheckTask.get_instance().watch(request_id, seconds=None)
