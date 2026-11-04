"""
床の二次元コードで、ET ラリーのグリッドに対する向きのずれを測る。

経路の直進の行に書いて使う。`2026/recog/grid_heading.py` と**同じ口・同じ意味の値**を
返すので、そのまま差し替えられる。

    {"RunInfo": {"SCRIPT": "2026/recog/floor_code_heading.py", "FUNCNO": 19, "FWD": 70},
     "SwitchInfo": {"SCJFN": 128, "SCD": 340}}

測った値は判定結果の heading_dev / heading_seq に載る。C++ がそれを読んで、
向き保持直進（FUNCNO 19）の目標の向きを直す（`Docs/heading_hold.md`）。

【grid_heading との違い】
見ているものが違う。**こちらは二次元コードのファインダパターン3個だけ**を見る
（灰色の丸も黒線も見ない）。実機80枚で 9枚→40枚。**grid_heading が誤った3枚は
3枚とも「測れない」を返す。** 詳細は `Docs/floor_code_heading.md`。

**床平面の校正（`2026/ground_homography.json`）が要る。** 無ければ測れないだけで、
走行は止まらない。
"""

from floor_code_reader import FloorCodeReader
from heading_task import HeadingTask

#: 走行体で共有する1つ。**grid_heading の HeadingTask とは別に持つ**
#: （読み取り道具が違うため）。判定結果（heading_dev / heading_seq）は同じ所へ載る
_task = None


def get_task():
    """床のコードで測る側のタスク。無ければ作る"""
    global _task
    if _task is None:
        task = HeadingTask(reader=None)
        task._reader = FloorCodeReader(frame_source=task._frame)
        _task = task
    return _task


def reset_task():
    """共有している1つを捨てる（テスト用）"""
    global _task
    _task = None


def run(request_id):
    get_task().measure(request_id)
