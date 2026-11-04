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

import time

import heading_task
from heading_task import HeadingTask
from pose import PoseReader
from robot_pose import RobotPose
from route_register import SwitchFile
from scenario_rows import ScenarioRows

#: その行を走っている間、向きを**繰り返し**測るか（置き場が無いときの既定）。
#:
#: 既定は行の頭の1回だけ。格子1マス（246mm）ごとに旋回する経路（点検 X）では、旋回のたびに積もる
#: 向きのずれに、1行1回（しかも前の補正を追っている間に撮った絵は捨てる）では追いつけなかった。
#: 入れると、行が変わるまで `REPEAT_INTERVAL` ごとに測り直す。走る速さは変わらない。
REPEAT = False

#: `REPEAT` を走行体で切り替える置き場（1行目に 1 か 0）
REPEAT_SWITCH = "./sorot_spike/heading_repeat.txt"

#: 測り直す間隔[s]。1回の測定（約 0.3 秒）に足して待つ
REPEAT_INTERVAL = 0.1

#: 1行で測り直す長さの上限[s]。行が終わったのに気づけないときの歯止め
REPEAT_MAX_SECONDS = 5.0

#: 繰り返し測るときの「前に出した値を追い終えるのを待つ時間」[s]（`heading_task.PUBLISH_SETTLE_SECONDS`）。
#: 1.0 秒のままだと、1行（0.5〜1.3 秒）の間に2回目を出せない。向き保持は 1回 3度までしか動かさないので、
#: 0.5 秒あれば追いつく
REPEAT_SETTLE_SECONDS = 0.5


def run(request_id):
    asked = RobotPose.get_instance().snapshot()
    if "sno" in asked and "cno" in asked:
        ScenarioRows.shared().note_script_row(asked["sno"], asked["cno"])
    if "sno" in asked and "cno" in asked:
        # 次が格子の中の旋回なら、止まって向きを測る行を差し込む（turn_check.py。既定は切）
        from turn_check import TurnCheck
        TurnCheck.get_instance().plan_after(asked["sno"], asked["cno"])
    task = HeadingTask.get_instance()
    repeat = bool(SwitchFile.read(REPEAT_SWITCH, REPEAT))
    heading_task.PUBLISH_SETTLE_SECONDS = REPEAT_SETTLE_SECONDS if repeat else 1.0
    task.measure(request_id)
    if not repeat or "sno" not in asked or "cno" not in asked:
        return
    row = (asked["sno"], asked["cno"])
    end = time.monotonic() + REPEAT_MAX_SECONDS
    while time.monotonic() < end:
        time.sleep(REPEAT_INTERVAL)
        pose = PoseReader.read_fresh()
        if pose is None or (pose.sno, pose.cno) != row:
            break
        task.measure(request_id)
