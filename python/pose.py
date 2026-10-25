"""
走行中の自己位置を C++ から読む。

C++ が 100ms ごとに置いた最新の1件を読むだけ。頼んで待つ経路は無い。

    C++（app.cpp の result_reader_task）
      → /dev/shm/sorot_pose.json（最新だけ。一時ファイル＋rename）
                                    PoseReader.read() ──▶ Pose

**リモートの観測（remote_control.py）とは別物。** あちらはリモートコントロール
走行（FUNCNO 18）の区間でしか置かれない。こちらは走行中いつでも置かれる
（Docs/position_correction_design.md 5.7 の「穴1」）。

【向きは2つある。混ぜないこと】

    dir_deg   エンコーダ由来の進行方向。ライントレース中は実際より小さく出る
    head_deg  x/y を積むのに実際に使った方位（#80 以降はジャイロ由来）

**座標と突き合わせるときに要るのは head_deg。** `heading_deg` はその head_deg を
返す（無ければ dir_deg で代用する）。Tools/compare_route.py と同じ決め方。

なお `gyro_deg`（ジャイロの積分値）は**絶対方位の基準にならない**。直進中に
実在しない回転を積む（#54）。回転「量」の目安としてだけ見ること。

【古い値を使わないこと】
C++ が止まってもファイルは残る。`read()` は取れたものをそのまま返すので、
**補正のように間違えると事故になる使い方では `read_fresh()` を使う**
（既定 0.3秒＝3周期。それより古ければ None）。
"""

import json
import logging
import os
import time
from typing import Any, Dict, Optional

try:
    import config
except ImportError:  # パッケージとして読み込まれた場合
    from . import config

logger = logging.getLogger(__name__)


class Pose:
    """
    ある時点の自己位置ひとそろい。**角度はすべて度[deg]、長さは mm。**

    C++ が置いた JSON をそのまま持つ。欠けたキーは 0 として読む
    （C++ 側の JSON パーサと同じ振る舞い。古い asp と混ざっても落ちない）。
    """

    def __init__(self, raw: Dict[str, Any], age: float = 0.0):
        self.raw = raw
        #: 置かれてから読むまでの経過[s]。ファイルの更新時刻から求める
        self.age = age
        self.x = float(raw.get("x") or 0.0)
        self.y = float(raw.get("y") or 0.0)
        self.dir_deg = float(raw.get("dir_deg") or 0.0)
        self.head_deg = float(raw.get("head_deg") or 0.0)
        self.dist = float(raw.get("dist") or 0.0)
        self.gyro_deg = float(raw.get("gyro_deg") or 0.0)
        self.sno = int(raw.get("sno") or 0)
        self.cno = int(raw.get("cno") or 0)
        #: 走行体の起動からの経過[ms]。**進んでいなければ C++ が止まっている**
        self.t_ms = int(raw.get("t_ms") or 0)

    @property
    def heading_deg(self) -> float:
        """
        座標と突き合わせるときに使う方位[deg]。

        head_deg（x/y を積むのに使った方位）があればそれ。無ければ dir_deg。
        Tools/compare_route.py と同じ決め方にしてある
        """
        if "head_deg" in self.raw:
            return self.head_deg
        return self.dir_deg

    def is_fresh(self, max_age: Optional[float] = None) -> bool:
        """置かれてからの経過が max_age[s] 以内か（既定 config.POSE_STALE_SEC）"""
        limit = config.POSE_STALE_SEC if max_age is None else max_age
        return self.age <= limit

    def __repr__(self) -> str:
        return ("Pose(x=%.1f, y=%.1f, heading=%.2f, dist=%.1f, "
                "sno=%d, cno=%d, age=%.3f)"
                % (self.x, self.y, self.heading_deg, self.dist,
                   self.sno, self.cno, self.age))


class PoseReader:
    """
    C++ が置いた自己位置を読む口。

    **状態を持たない。** 生成せずにクラスのまま呼ぶ（`PoseReader.read()`）。
    置き場のパスはクラスの属性にしてあるので、テストは一時ディレクトリへ向けられる。
    """

    #: C++ が置く場所。読むだけで、消さない（最新だけが要るので上書きされてよい）
    POSE_FILE = config.COMM_POSE_FILE

    @classmethod
    def read(cls) -> Optional[Pose]:
        """
        いちばん新しい自己位置を読む。**古さは見ない。**

        :returns: 読めたら Pose。ファイルが無い／壊れていれば None
        """
        try:
            stat = os.stat(cls.POSE_FILE)
            with open(cls.POSE_FILE, encoding="utf-8") as f:
                raw = json.load(f)
        except FileNotFoundError:
            return None              # まだ置かれていない（起動直後）
        except (OSError, ValueError) as e:
            #書く側は一時ファイル＋rename なので、書きかけを読むことは無い。
            #ここへ来るのは中身の形が変わったときなので、気づけるように残す
            logger.warning("自己位置を読めません（%s）: %s", cls.POSE_FILE, e)
            return None

        if not isinstance(raw, dict):
            logger.warning("自己位置の形が違います（%s）: %r", cls.POSE_FILE, raw)
            return None

        age = max(0.0, time.time() - stat.st_mtime)
        return Pose(raw, age)

    @classmethod
    def read_fresh(cls, max_age: Optional[float] = None) -> Optional[Pose]:
        """
        新しいときだけ自己位置を返す。

        **補正のように、間違えると事故になる使い方はこちら。** C++ が止まっても
        ファイルは残るので、`read()` だけでは古い値を掴んだことに気づけない。

        :param max_age: 許す古さ[s]（既定 config.POSE_STALE_SEC）
        :returns: 新しければ Pose、古ければ／読めなければ None
        """
        pose = cls.read()
        if pose is None:
            return None
        if not pose.is_fresh(max_age):
            logger.warning("自己位置が %.3f秒 古いので使いません", pose.age)
            return None
        return pose
