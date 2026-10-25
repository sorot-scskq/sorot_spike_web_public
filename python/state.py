"""
直近の判定結果（スレッド間で共有する）。

書くのは PC と話すスレッド（`handler.py`）と、走行体で認識しているもの
（`2026/bottle_color_task.py`、`2026/hint_card_capture.py`）。読むのは
`ev3_python.py` で、判定結果ファイル（/dev/shm/sorot_comm_res.json）へ書き出し、
C++ が 100ms ごとに読む。

番兵の 100 は「まだ結果が無い」。C++ 側（RoughSpot/Cpp/ResultRead.h の
RESULT_UNKNOWN）と同じ値にすること。
"""

import threading
from typing import Any, Iterable, Optional


class ResultStore:
    """
    直近の判定結果。走行体で1つだけ使う（`get_instance()`）。

    **中身を直接は触らせない。** 読むときは写し（`snapshot`）を渡し、書くときは
    項目ごと（`set`）か辞書ごと（`replace`）に呼ぶ。複数のスレッドが同じ辞書を
    書き換えると、読み出しの途中の中途半端な状態が C++ へ渡ることがある。
    """

    #: 初期値。キーの顔ぶれは C++（ResultRead）と Python で同じにすること
    DEFAULT = {
        "completed_req_id": 0,
        "minifigure": 100,
        "plarail": 100,
        "bottle": 100,
        "gate": 100,
        "qr1": "",
        "qr2": "",
        # ETラリー：前面カメラで測った、グリッドに対する向きのずれ[度]と、測れた回数。
        # C++ は heading_seq が増えたときだけ取り込む（Docs/heading_hold.md）
        "heading_dev": 0.0,
        "heading_seq": 0,
        # ETラリー：床の二次元コードで出した、自己位置の打ち直し量[mm]と[度]。
        # C++ は coord_seq が増えたときだけ取り込み、ACAF 0x07 で打ち直す
        # （Docs/position_correction_design.md）
        "coord_dx": 0.0,
        "coord_dy": 0.0,
        "coord_dr": 0.0,
        "coord_seq": 0,
    }

    _instance: Optional["ResultStore"] = None
    _instance_lock = threading.Lock()

    @classmethod
    def get_instance(cls) -> "ResultStore":
        """走行体で共有する1つを返す。無ければ作る"""
        with cls._instance_lock:
            if cls._instance is None:
                cls._instance = cls()
            return cls._instance

    @classmethod
    def reset_instance(cls) -> None:
        """共有している1つを捨てる（テスト用）"""
        with cls._instance_lock:
            cls._instance = None

    def __init__(self):
        self._lock = threading.Lock()
        self._result = dict(self.DEFAULT)

    # --- 読む -----------------------------------------------------------------

    def snapshot(self) -> dict:
        """
        いまの中身の写し。

        写しなので、持ち出したあとで書き換えても共有のほうは変わらない
        （`ev3_python` が py_time を足して書き出すのはこれ）。
        """
        with self._lock:
            return dict(self._result)

    def get(self, key: str, default: Any = None) -> Any:
        """1項目だけ読む"""
        with self._lock:
            return self._result.get(key, default)

    # --- 書く -----------------------------------------------------------------

    def set(self, key: str, value: Any) -> None:
        """1項目だけ書く（ほかの項目は残る）"""
        with self._lock:
            self._result[key] = value

    def complete(self, request_id: int) -> None:
        """
        その要求が終わったことを C++ へ知らせる（`completed_req_id`）。

        失敗したときも知らせること。知らせないと、C++ は結果を待ち続ける。
        """
        self.set("completed_req_id", request_id)

    def replace(self, latest: dict, keep: Iterable[str] = ()) -> dict:
        """
        辞書ごと差し替える。

        :param keep: **空で来たぶんだけ前の値を引き継ぐ項目。** 差し替えは辞書ごと
                     なので、放っておくと別の判定結果で消えてしまうものがある
                     （ヒントカードは2枚そろってから経路を頼むので、PC の応答が
                     片方しか持っていないと「そろっていない」に戻る）
        :returns: 差し替えたあとの中身（写し）
        """
        latest = dict(latest)
        with self._lock:
            for key in keep:
                if not latest.get(key):
                    latest[key] = self._result.get(key, "")
            self._result = latest
            return dict(self._result)

    def clear(self) -> None:
        """初期値へ戻す（走行やり直し用）"""
        with self._lock:
            self._result = dict(self.DEFAULT)
