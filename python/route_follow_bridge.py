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


async def _sim_ffi_load():
    """JS の null を None にそろえる共通の関数（sim/pyscript/sim_ffi.py）を取り込む"""
    response = await pyfetch("python/sim_ffi.py", cache="no-store")
    with open("sim_ffi.py", "w") as out:
        out.write(await response.string())


await _sim_ffi_load()
from sim_ffi import js_value  # noqa: E402

#: 走行体の実装。書き写さず、実機と同じ1本を読む（/python/ で配られる）
#:
#: scenario_rows / coord_correct は**自己位置の向きの打ち直し**（2026/recog/heading_fix.py）の
#: 書き込む先。heading_fix 自体は走行体の Python のまま走る（sim/pyscript/robot_runtime_bridge.py）。
#: 同じページの Python は1つなので、ここで座らせた CoordCorrector / ScenarioRows / RouteFollower を
#: 走行体の HeadingFixTask がそのまま使う。経路の書き直しと同じ RouteFollower で行を組むため
#: （同じ行を2か所で組み立てると、あとから置いたほうが前のぶんを消す。
#: Docs/position_correction_design.md 5.8）。
ROUTE_FOLLOW_SOURCES = (
    "config.py",
    "route_plan.py",
    "route_correct.py",
    "route_numbering.py",
    "pose.py",
    "pose_reader.py",
    "scenario_update.py",
    "route_follower.py",
    "state.py",
    "robot_pose.py",
    "scenario_rows.py",
    "coord_correct.py",
)

#: 自己位置の向きを打ち直す行に書く SCRIPT。実機と同じパス
HEADING_FIX_SCRIPT = "2026/recog/heading_fix.py"

#: シナリオを置く先（Pyodide のメモリ上。`begin()` にパスで渡すため）
SCENARIO_MEMFS_PATH = "sim_scenario.json"


async def _route_follow_setup():
    """経路の書き直しをブラウザに載せる。読み込み時に1回だけ呼ぶ"""

    def to_obj(value):
        """dict は JS の Map ではなく普通のオブジェクトにする"""
        return _to_js(value, dict_converter=Object.fromEntries)

    def bridge():
        """JS 側の口（sim/dom-ui/route-follow-client.js）"""
        return js_value(getattr(window, "__simRouteFollow", None))

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
    from robot_pose import RobotPose
    from scenario_rows import ScenarioRows
    from coord_correct import CoordCorrector

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
                found = js_value(js_side.pose())
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
    # **走行体で1つを共有する口に、こちらで組んだものを座らせる。**
    # 打ち直し側（coord_correct.py）は RouteFollower.get_instance() を見にいくので、
    # 座らせないと別の（ファイルを読む）RouteFollower が作られ、経路の行が
    # 見つからないまま custom.json の行へ書こうとする
    RouteFollower._instance = follower           # noqa: SLF001 — 実機の共有口に座らせる


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
            started = bool(follower.begin(rows, int(rough_spot_id),
                                          scenario_path=SCENARIO_MEMFS_PATH))
            # 打ち直しの材料もここでそろえる。走り直しのたびに作り直す
            #
            # **経路の行は走行中に番号を振り直される。** Python からは見えないので、
            # SCRIPT の要求に載ってくる SNO/CNO から学ぶ（scenario_rows.note_script_row）。
            # その手がかりが「経路のどの行に heading_fix が書いてあるか」なので、
            # 経路の行そのものと SCRIPT のパスを渡しておく
            ScenarioRows.reset_shared()
            sheet = ScenarioRows.shared()
            sheet.load_file(SCENARIO_MEMFS_PATH)
            sheet.set_route(rows, HEADING_FIX_SCRIPT)
            CoordCorrector.reset_shared()
            # **座らせ直す。** 走り直すたびに、走行体の Python（robot_runtime_bridge.py の
            # _rr_reset）が RouteFollower.reset_instance で共有口を空にする。空のまま
            # CoordCorrector を作ると、経路を知らない別の RouteFollower を掴み、
            # 打ち直しの行を経路の書き直しと別々に組んでしまう（あとから置いたほうが前のぶんを消す）
            RouteFollower._instance = follower       # noqa: SLF001 — 実機の共有口に座らせる
            # 置き場は JS。実機の ScenarioMailbox（/dev/shm）の代わり
            CoordCorrector._instance = CoordCorrector(  # noqa: SLF001 — 実機の共有口に座らせる
                rows=sheet, mailbox=SimMailbox)
            RobotPose.get_instance().clear()
            return started
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
        js_side = js_value(window.__simRouteFollow)
    js_side.begin = begin
    js_side.poll = poll
    js_side.stop = stop
    js_side.settings = settings
    js_side.ready = True
    print("[route_follow] 経路の書き直しを載せました:", settings())


_route_follow_setup()
