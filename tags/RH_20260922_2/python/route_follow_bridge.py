"""
経路の書き直しを、シミュレータのブラウザ内で動かす（PyScript）。

走行体で動く処理そのものは src/RoughSpot/Python/Common/route_follower.py にある
（判断は route_correct.py、計画の積分は route_plan.py）。ここはそれをブラウザで
動かすための配線だけを持つ。

    実機          : C++ が /dev/shm へ置く自己位置 → RouteFollower → /dev/shm へ差し替え
    シミュレータ  : JS が渡す自己位置             → RouteFollower → JS が行を差し替える

【なぜ差し替えられるか】
`RouteFollower(reader=..., mailbox=...)` に依存注入の口がある。継ぎ目は2つだけ。

    reader.read()          いまの自己位置（PoseSnapshot）。無ければ None
    mailbox.place_rows()   書き直した行を渡す。置けたら True

【実機との違いは3つだけ】
  1. 自己位置はファイルではなく JS からもらう（ブラウザにファイルシステムが無い）
  2. 書き直した行はファイルではなく JS へ返す
  3. シナリオは JS からもらい、Pyodide のメモリ上のファイルに書いてから渡す
     （`begin()` がファイルのパスを取る作りのため。走行体のコードは変えない）

【見張りのスレッドは起こさない】
ブラウザに本物のスレッドが無い。JS が毎フレーム `poll()` を呼ぶ。
実機の `start_file_loop()` は使わない。

【全体を関数の中に入れている理由】
route_bridge.py と同じ。PyScript は同じスクリプトを2回実行することがあり、
複数の <script type="py"> が同じ名前空間で動く。
"""

import json
import traceback

from js import Object, window
from pyodide.ffi import to_js as _to_js
from pyodide.http import pyfetch

#: 走行体の実装。書き写さず、実機と同じ1本を読む（/python/ で配られる）
ROUTE_FOLLOW_SOURCES = (
    "config.py",
    "route_plan.py",
    "route_correct.py",
    "route_numbering.py",
    "pose.py",
    "pose_reader.py",
    "scenario_update.py",
    "route_follower.py",
)

#: シナリオを置く先（Pyodide のメモリ上。`begin()` にパスで渡すため）
SCENARIO_MEMFS_PATH = "sim_scenario.json"


async def _route_follow_setup():
    """経路の書き直しをブラウザに載せる。読み込み時に1回だけ呼ぶ"""

    def to_obj(value):
        """dict は JS の Map ではなく普通のオブジェクトにする"""
        return _to_js(value, dict_converter=Object.fromEntries)

    def bridge():
        """JS 側の口（sim/dom-ui/route-follow-client.js）"""
        return getattr(window, "__simRouteFollow", None)

    # -- 2回目の実行を弾く -------------------------------------------------
    existing = bridge()
    if existing is not None and getattr(existing, "ready", False):
        return

    # -- 走行体側の実装を取り込む -----------------------------------------
    for name in ROUTE_FOLLOW_SOURCES:
        response = await pyfetch(f"python/{name}")
        with open(name, "w") as out:
            out.write(await response.string())

    from pose_reader import PoseSnapshot
    from route_plan import Pose
    from route_follower import RouteFollower

    class SimPoseReader:
        """JS が渡す自己位置を `PoseSnapshot` にする。無ければ None

        **古さは見ない。** 実機の `read_fresh()` は C++ が止まったことに気づく
        ためのものだが、シミュレータには止まる相手がいない。
        """

        def read(self):
            js_side = bridge()
            if js_side is None:
                return None
            try:
                found = js_side.pose()
            except Exception:                       # noqa: BLE001 — 走行は止めない
                traceback.print_exc()
                return None
            if found is None:
                return None
            return PoseSnapshot(
                pose=Pose(float(found.x), float(found.y), float(found.heading_deg)),
                dir_deg=float(found.dir_deg), distance=float(found.dist),
                sno=int(found.sno), cno=int(found.cno), t_ms=int(found.t_ms))

    class SimMailbox:
        """書き直した行を JS へ返す。実機の `ScenarioMailbox` の代わり

        **1通ずつという制約が無い。** 実機は C++ が引き取るまで次を置けないが、
        JS は同期で受け取れるので `pending()` は常に False。
        """

        @classmethod
        def place_rows(cls, rows):
            js_side = bridge()
            if js_side is None:
                return False
            try:
                return bool(js_side.applyRows(to_obj(list(rows))))
            except Exception:                       # noqa: BLE001
                traceback.print_exc()
                return False

        @classmethod
        def pending(cls):
            return False

    follower = RouteFollower(reader=SimPoseReader(), mailbox=SimMailbox)

    # -- JS から呼ぶ口 -----------------------------------------------------
    def begin(scenario_rows_json, rough_spot_id, scenario_json):
        """経路を渡した直後に呼ぶ。戻りは見張る用意ができたか

        :param scenario_rows_json: C++ へ渡したのと**同じ**コマンドの配列（JSON 文字列）
        :param rough_spot_id: 難所の識別値（`SW_ROUTE` の `SCV`）
        :param scenario_json: 走行中のシナリオ全体（JSON 文字列）。番号の予測に要る
        """
        try:
            rows = json.loads(scenario_rows_json)
            with open(SCENARIO_MEMFS_PATH, "w", encoding="utf-8") as out:
                out.write(scenario_json)
            return bool(follower.begin(rows, int(rough_spot_id),
                                       scenario_path=SCENARIO_MEMFS_PATH))
        except Exception:                           # noqa: BLE001
            traceback.print_exc()
            return False

    def poll():
        """自己位置を1回読む。**行が変わったときだけ**考える

        JS が毎フレーム呼ぶ。戻りは、行が変わって判断まで進んだか。
        """
        try:
            return bool(follower.poll_once())
        except Exception:                           # noqa: BLE001
            traceback.print_exc()
            return False

    def stop():
        """見張りをやめる（走行の終わり）"""
        try:
            follower.stop()
            return True
        except Exception:                           # noqa: BLE001
            traceback.print_exc()
            return False

    def settings():
        """その走行で実際に効いたしきい値。記録に残すため"""
        try:
            return str(follower.settings())
        except Exception:                           # noqa: BLE001
            return ""

    js_side = bridge()
    if js_side is None:
        window.__simRouteFollow = to_obj({})
        js_side = window.__simRouteFollow
    js_side.begin = begin
    js_side.poll = poll
    js_side.stop = stop
    js_side.settings = settings
    js_side.ready = True
    print("[route_follow] 経路の書き直しを載せました:", settings())


_route_follow_setup()
