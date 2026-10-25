"""
C++ から届いた「そのときの自己位置」を覚えておく（走行体の Python）。

Python 部は自己位置を持っていない。観測（`/dev/shm/sorot_remote_obs.json`）が置かれるのは
リモートコントロール走行のときだけなので、ETラリーの直進中には読めない。
そこで **SCRIPT の要求に座標・向き・区間番号を載せてもらい**、ここに覚えておく
（C++ 側は `RoughSpot/Cpp/PythonCommServer.cpp` の `sendScriptRequest`）。

    {"script": "2026/recog/grid_heading.py", "request_id": 7,
     "x": 1234.5, "y": 567.8, "heading": 135.2, "sno": 30, "cno": 12}

用途はデバッグ画像の名前付け（`Docs/heading_camera_field_check.md`）。
**走行の判断には使わない。** 届くのは要求を出した瞬間の値で、撮った瞬間の値ではない。

古い C++（座標を載せない版）と混ぜても動く。そのときは「分からない」として扱う。
"""

import logging
import threading
from typing import Any, Dict, Optional

logger = logging.getLogger(__name__)


class RobotPose:
    """SCRIPT の要求に載ってきた自己位置。走行体で1つを共有する"""

    _instance: Optional["RobotPose"] = None
    _instance_lock = threading.Lock()

    @classmethod
    def get_instance(cls) -> "RobotPose":
        with cls._instance_lock:
            if cls._instance is None:
                cls._instance = cls()
            return cls._instance

    @classmethod
    def reset_instance(cls) -> None:
        """共有している1つを捨てる（テスト用）"""
        with cls._instance_lock:
            cls._instance = None

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._pose: Dict[str, Any] = {}

    def update_from_request(self, req: Dict[str, Any]) -> Dict[str, Any]:
        """
        要求の JSON から座標・向き・区間番号を取り出して覚える。

        **足りない項目は覚えない。** 座標を載せない古い C++ でも落ちない。

        :returns: 覚えた中身（読み取り用の写し）
        """
        pose: Dict[str, Any] = {}
        for key in ("x", "y", "heading"):
            value = req.get(key)
            if isinstance(value, (int, float)) and not isinstance(value, bool):
                pose[key] = float(value)
        for key in ("sno", "cno", "request_id"):
            value = req.get(key)
            if isinstance(value, int) and not isinstance(value, bool):
                pose[key] = value

        with self._lock:
            self._pose = pose
        return dict(pose)

    def snapshot(self) -> Dict[str, Any]:
        """いま覚えている自己位置の写し。何も届いていなければ空"""
        with self._lock:
            return dict(self._pose)

    def clear(self) -> None:
        with self._lock:
            self._pose = {}
