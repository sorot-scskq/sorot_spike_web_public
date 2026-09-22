"""
`Common/pose.py` が読んだ自己位置を、経路の書き直しが使う形にする。

**読むところは持たない。** 置き場（`/dev/shm/sorot_pose.json`）を読む決まりは
`pose.py` が持っている。ここが足すのは2つだけ。

* 姿勢を `route_plan.Pose` にする（計画を積むのと同じ型・同じ座標系）
* 走っている行（`sno`/`cno`）と1つの組にする

【向きは `head_deg`】
x/y を積むのに実際に使った方位（#80）。`dir_deg`（エンコーダ由来）はライントレース中に
実際より小さく出る。`pose.py` の `heading_deg` が同じ決め方（`head_deg`、無ければ
`dir_deg`）なので、それをそのまま使う。

【古いものは使わない】
C++ が止まってもファイルは残る。**間違えると事故になる使い方なので `read_fresh()`**
（既定 `config.POSE_STALE_SEC` ＝ 0.3秒 ＝ 3周期）。`read()` は古さを見ない。
"""

import logging

from pose import PoseReader
from route_plan import Pose

logger = logging.getLogger(__name__)

#: 見にいく間隔[s]。C++ は 100ms ごとに置くので、それより短くする
POSE_POLL_INTERVAL = 0.05


class PoseSnapshot:
    """ある時点の自己位置ひとそろい（姿勢と、走っている行）"""

    __slots__ = ("pose", "dir_deg", "distance", "sno", "cno", "t_ms")

    def __init__(self, pose, dir_deg, distance, sno, cno, t_ms):
        self.pose = pose
        self.dir_deg = dir_deg
        self.distance = distance
        self.sno = sno
        self.cno = cno
        self.t_ms = t_ms

    @property
    def key(self):
        """いま走っている行（シナリオNo・コマンドNo）"""
        return (self.sno, self.cno)

    def __repr__(self):
        return "PoseSnapshot(%d/%d %s t=%dms)" % (self.sno, self.cno, self.pose, self.t_ms)


class PoseSnapshotReader:
    """`pose.PoseReader` を包んで `PoseSnapshot` にする。**古ければ None**"""

    def __init__(self, reader=PoseReader, max_age=None):
        self.reader = reader
        #: 許す古さ[s]。None なら `config.POSE_STALE_SEC`（0.3秒＝3周期）
        self.max_age = max_age

    def read(self):
        found = self.reader.read_fresh(self.max_age)
        if found is None:
            return None
        return PoseSnapshot(
            pose=Pose(found.x, found.y, found.heading_deg),
            dir_deg=found.dir_deg, distance=found.dist,
            sno=found.sno, cno=found.cno, t_ms=found.t_ms)
