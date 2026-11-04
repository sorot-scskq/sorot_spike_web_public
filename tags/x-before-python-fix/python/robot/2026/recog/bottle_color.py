"""
キャリーボトルのラベル色を判別する（ボトルデリバリー）。

シナリオのコマンドに書いて使う。

    {"Comment":"ボトルへ寄る","SNO":1,"CNO":1,"PRVSNO":0,"PRVCNO":0,
     "RunInfo":{"SCRIPT":"2026/recog/bottle_color.py",
                "FUNCNO":5,"FWD":30,"TRN":0,"NOBLNCE":1},
     "SwitchInfo":{"SCJFN":128,"SCD":250},"CoordCorrectInfo":{}}

**ラベルが見える距離（ボトルの 449〜249mm 手前）を走る区間に書くこと。**
抱えたあとはボトルがカメラに近すぎてラベルが画角の下へ外れる。

色が決まると、その色の運び先へ走行中のシナリオが差し替わる
（config_2026.BOTTLE_ROUTE_FILES）。決まらなければ何も起きず、
シナリオに書いてある既定のルートをそのまま走る。
"""
from bottle_color_task import BottleColorTask


def run(request_id):
    BottleColorTask.get_instance().judge(request_id)
