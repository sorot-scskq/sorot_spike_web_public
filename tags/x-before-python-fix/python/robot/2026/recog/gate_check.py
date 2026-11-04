"""
走行中、前方カメラで ET ラリーのゲートを見張る。

**この行は PC が差す。** 経路の行を作っているのは PC なので、ゲートを通る行への
差し込みも PC の仕事（PC-System_2026 の `CreateRouteV2.gate_check_indices`）。

    {"SNO":30,"CNO":15,
     "RunInfo":{"SCRIPT":"2026/recog/gate_check.py","FUNCNO":19,"FWD":70,"ARMANG":-150},
     "SwitchInfo":{"SCJFN":128,"SCD":1106}}

その行に入ってから `gate_check_task.WATCH_SECONDS` のあいだ、ゲートが見えるたびに
「いまの姿勢のまま進んで通れるか」をログに残す（`RoughSpot/PhotoImage/gate/` に
測った1枚も残る）。**走りは変えない。**

**`FUNCNO 7〜10 / 15` の行に書いてはいけない**（要求の置き場を取り合って、SCRIPT が
届かない）。点検用のシナリオで手で書くときも同じ。

まとめは `Tools/summarize_gate_check.py`。手引きは `Docs/gate_check.md`。
"""

from gate_check_task import GateCheckTask


def run(request_id):
    GateCheckTask.get_instance().watch(request_id)
