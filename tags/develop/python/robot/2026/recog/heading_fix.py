"""
床の二次元コードで測った向きへ、走行中の自己位置を打ち直す。

ETラリーの直進の行に書いて使う。**旋回の直後の直進**が向いている
（旋回の残差がそのまま以後の直進に乗るため。`Docs/position_correction_design.md` 2章）。

    {"RunInfo": {"SCRIPT": "2026/recog/heading_fix.py", "FUNCNO": 19, "FWD": 70, ...},
     "SwitchInfo": {"SCJFN": 128, "SCD": 340}}

測れたぶんは、いまの行から数行先の行へ `CoordCorrectInfo`（ACAF 0x04 / CAPR）として
書き込まれ、C++ がその行を読んだときに向きが打ち直る。**何行先に書くかは
`heading_fix_task.LOOKAHEAD_ROWS`**（実機で測って決める。`Docs/heading_correction.md`）。

測れなければ何も起きない。
"""

from heading_fix_task import HeadingFixTask


def run(request_id):
    HeadingFixTask.get_instance().fix(request_id)
