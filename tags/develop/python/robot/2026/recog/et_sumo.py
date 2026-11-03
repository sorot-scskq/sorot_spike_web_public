"""
ET相撲の力士寄せを、その区間に入った時点で差し直す。

シナリオのコマンドに書いて使う。

    {"SNO":40,"CNO":2,
     "RunInfo":{"SCRIPT":"2026/recog/et_sumo.py",
                "FUNCNO":18,"FWD":40,"TRN":40,"NOBLNCE":1},
     "SwitchInfo":{"SCJFN":2049,"SCT":20000},"CoordCorrectInfo":{}}

**ここでしか差していない。** 以前は起動時にも差していたが
（config_2026.start_recognizers）、いまはコメントアウトしてある。

こうしている理由は2つ。

* シナリオを読んだだけで、この区間が力士寄せであることが分かる
* 区間に入るたびに作り直すので、前の周回の途中状態を持ち越さない

**その代わり、区間に入ってから差さるまで最大 40ms かかる。** その間は制御関数が
無いので C++ は指令を受け取れず (0,0) を出す（RunMethod/RemoteControl.cpp の末尾）。
FWD 40 で進むぶんは数 mm だが、気になるなら config_2026 のコメントアウトを
生かせば埋まる（二重に差しても後勝ちで無害）。
"""
from et_sumo import attach


def run(request_id):
    attach()
