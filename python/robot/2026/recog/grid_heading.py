"""
前面カメラで、ET ラリーのグリッドに対する向きのずれを測る。

経路の直進の行に書いて使う。書くのは `Common/route_register.py`（PC から届いた経路を
置く前に、旋回の直後の直進の行へ付ける）。

    {"RunInfo": {"SCRIPT": "2026/recog/grid_heading.py", "FUNCNO": 19, "FWD": 70, ...},
     "SwitchInfo": {"SCJFN": 128, "SCD": 340}}

測った値は判定結果の heading_dev / heading_seq に載る。C++ がそれを読んで、
向き保持直進（FUNCNO 19）の目標の向きを直す（`Docs/heading_hold.md`）。

**この SCRIPT が呼ばれた行の番号で、経路の番号を学ぶ**（`ScenarioRows.note_script_row`）。
学ばないと経路の行の中身が引けず、横の補正（`qr_steer.py`）が働かない。
以前は heading_fix が学んでいた（2026-10-31 に止めた）。
"""

from heading_task import HeadingTask
from robot_pose import RobotPose
from scenario_rows import ScenarioRows


def run(request_id):
    asked = RobotPose.get_instance().snapshot()
    if "sno" in asked and "cno" in asked:
        ScenarioRows.shared().note_script_row(asked["sno"], asked["cno"])
    HeadingTask.get_instance().measure(request_id)
