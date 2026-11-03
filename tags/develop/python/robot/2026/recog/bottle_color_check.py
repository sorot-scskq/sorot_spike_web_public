"""
キャリーボトルのラベル色を判別する（点検用。運び先は差し替えない）。

`bottle_color.py` と同じく色を判別するが、**シナリオを差し替えない**。
色が読めているかどうかだけを、耳（SCBEEP 2）と残った画像で確かめるためのもの。

    "RunInfo": {"SCRIPT": "2026/recog/bottle_color_check.py", "FUNCNO": 1, "FWD": 40, ...}

点検のシナリオには運び先（SNO 2 など）が無いので、差し替えを走らせると
行き場のないコマンドが末尾に足されてしまう。それを避けるために分けてある。

本番のシナリオでは `bottle_color.py` を使うこと。
"""
from bottle_color_task import BottleColorTask


def run(request_id):
    BottleColorTask.get_instance().judge(request_id, replace_route=False)
