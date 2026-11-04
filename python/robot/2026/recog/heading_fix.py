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

**止めた（2026-10-31）。** QR を復号して四隅から向きを出す旧い道で、走行中に誤った値を当てた。
シナリオに書いてあっても何もしない（`route_register.heading_script` の説明）。
"""

import logging

logger = logging.getLogger(__name__)


def run(request_id):
    logger.info("heading_fix は止めてあります（2026/recog/heading_fix.py）。向きは heading_task で測ります")
