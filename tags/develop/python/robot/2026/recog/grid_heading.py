"""
前面カメラで、ET ラリーのグリッドに対する向きのずれを測る。

経路の直進の行に書いて使う。書くのは `Common/route_register.py`（PC から届いた経路を
置く前に、旋回の直後の直進の行へ付ける）。

    {"RunInfo": {"SCRIPT": "2026/recog/grid_heading.py", "FUNCNO": 19, "FWD": 70, ...},
     "SwitchInfo": {"SCJFN": 128, "SCD": 340}}

測った値は判定結果の heading_dev / heading_seq に載る。C++ がそれを読んで、
向き保持直進（FUNCNO 19）の目標の向きを直す（`Docs/heading_hold.md`）。
"""

from heading_task import HeadingTask


def run(request_id):
    HeadingTask.get_instance().measure(request_id)
