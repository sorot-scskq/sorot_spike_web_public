"""
無線通信デバイス（PC）から受け取ったシナリオを登録する。

PC は「この先どう走るか」をシナリオのコマンド列で返してくる（ETラリーの経路）。
受け取ったら C++ の置き場へ置き、C++ が 100ms ごとに引き取って走行中のシナリオへ
挿し込む（`Docs/et_rally_route.md`）。

    PC の応答 ──▶ RouteRegister ──▶ /dev/shm/sorot_route.json ──▶ C++ ResultRead

【頼んでから届いたものだけを登録する】
PC は一度作った経路を覚えていて、**別の要求の応答にも同じものを載せてくる**
（PC-System_2026 の `AppStateManager.get_result_payload`）。走り直して繋ぎ直すと、
前の走行で作った経路が、ヒントカードを読む前に届く。それを登録すると古い
ゲート配置のまま走ることになる。

そこで、この走行で経路を頼んだあと（`expect`）に届いたものだけを登録し、
1回登録したらそこで止める。次に頼めばまた受け取れる。

【ETラリーの直進は向き保持直進に替える】
PC は直進を FUNCNO 5 で送ってくる。FUNCNO 5 は行ごとにエンコーダの基準を取り直すので、
直進で曲がった分が積もる（2026-09-15 の実機で ETラリー全体で約45°）。ETラリーの経路の
直進（FUNCNO 5、FWD≠0、TRN 0）だけを、ジャイロの向きを保つ FUNCNO 19 に替えてから置く
（`Docs/heading_hold.md`）。**C++ に FUNCNO 19 が無い版へ置くと、その行は走らない。**
やめるときは `USE_HEADING_HOLD` を False にする。

【渡したあとは自己位置と突き合わせる】
経路は相対の指示だけなので、そのままでは開ループで走る。渡した行から
「各行の到達目標姿勢」を作り、走行中に先の行を書き直すのが `route_follower.py`
（`Docs/route_plan_follow.md`）。**やめるときは `FOLLOW_PLAN` を False にする**
（走行体では `sorot_spike/follow_plan.txt` に `0`）。

【旋回の直後の直進で、カメラで向きを測る】
向き保持直進はジャイロの向きを保つだけで、ジャイロ自体のずれ（走行ごとに ±1%）は
直せない。旋回の直後の直進の行に SCRIPT（2026/recog/grid_heading.py）を書いておくと、
その区間で前面カメラが 1枚撮ってグリッドに対する向きを測り、C++ が目標の向きを直す。
測れなければ何も起きない。やめるときは `USE_HEADING_CAMERA` を False にする。

【後退の直後の旋回の前に、短い停止を挟む】
先読み停止は目標の手前で電源を切るので、行が終わった時点でまだ後ろへ動いている。
**その場旋回は左右を逆に回すので前後の慣性を止めない**ため、走行体は回りながら
後ろへ滑る。滑る量は旋回にかかる時間に比例し、実機では -860mm の後退の直後の
90度旋回で +25〜31mm 行き過ぎた（`Docs/handoff_20260922.md` 3.5）。後退そのものは
旋回を外すと ±5mm で正確なので、**間に止まる行を1つ挟んで慣性を落とす。**

**挟むのは PC 側（PC-System_2026 の `CreateRouteV2.convert_route_to_scenario`）で、
ここは既定で止めてある。** PC 側を配れないときだけ
`sorot_spike/pause_after_backward.txt` に `1` を書いて使う。両方効いても二重には
ならない（後退の次の行が旋回のときだけ挟むので、PC が挟んでいれば当たらない）。

【シナリオ区分が無い応答を補う】
C++ は「どの難所の経路か」で挿し込み先を決める（`SW_ROUTE` の `SCV`）。
ところが PC-System_2026 の応答には `senario_kbn` が入っていない。頼んだときの
難所を覚えておき、入っていなければそれで補う。
"""

import json
import logging
import threading
from typing import Any, Optional

import route_follower

from scenario_rows import ScenarioRows
from route_place import RoutePlaces
from scenario_update import ScenarioMailbox

try:
    from config import SENARIO_KBN_ET_RALLY, season_hook
except ImportError:  # パッケージとして読み込まれた場合
    from .config import SENARIO_KBN_ET_RALLY, season_hook

logger = logging.getLogger(__name__)

#: 応答の status。1 = 処理成功、0 = 未処理、-1 = 失敗
STATUS_OK = 1

#: ETラリーの直進を向き保持直進（FUNCNO 19）に替えるか
#:
#: **False にすると FUNCNO 5（エンコーダによる方向補正）で走る。** 向き保持は
#: ジャイロの向きを保つが、ジャイロは直進中に実在しない回転を積む（#54、±5°/80cm）。
#: それを打ち消そうとして走行体が実際に曲がる。しかも自己位置も同じジャイロで
#: 積むので、**曲がったことが記録から消える**（2026-09-22、Lコース実機）。
USE_HEADING_HOLD = True

#: 旋回の最中に走行体が前後へ動く量[mm]。**直前の直進の向きと、旋回の向きで決まる。**
#:
#: その場旋回は左右どちらかの車輪を逆転させる。逆転する側はガタを取り終えるまで
#: 動かないので、片輪だけが先に回る。その差が左右の平均（走行距離）に出る。
#: **純粋なその場旋回なら 0 のはず**で、0 でないぶんだけ走行体は前後に動いている。
#:
#: 2026-09-22 のLコース実機、通し走行1本の旋回28回から。ばらつきは 1〜10mm:
#:
#:     直前   旋回の符号   回数   平均[mm]
#:     前進      ＋         8      -1.0
#:     前進      −         13     +23.0
#:     後退      ＋         3     -51.8
#:     後退      −         4     -14.5
#:
#: **旋回の角度（45°と90°）ではほとんど変わらない**（前進−で +24.4 対 +21.7）。
#: 滑る時間に比例しないので、慣性ではない。実際、後退と旋回の間に 250ms の停止を
#: 挟んでも変わらなかった。
#:
#: **2026-09-22 の実機2本で決めた値。**
#:
#: 旋回中の並進そのものは、最初に測った値と2本とも再現した（-0.4/+25.5/-51.9/-13.4、
#: -2.0/+23.4/-48.9/-13.8 に対し、最初の実測は -1.0/+23.0/-51.8/-14.5）。**モデルは正しい。**
#:
#: ここに入れるのは「旋回中の並進」そのものではなく、**並進と先読み停止の差**である。
#:
#:     行は  SCD − 先読み(≈22mm)  で切り替わる
#:     その後  旋回中の並進  が起きる
#:     合計 = SCD − 先読み + 並進
#:     よって詰めるべき量 = 並進 − 先読み
#:
#: 実際、並進から 22mm を引くと -23 / +1 / -29.8 / +7.5 となり、実機2本から
#: 逆算した -25.9 / +4.6 / -29.7 / +1.1 とほぼ一致する。**独立した2つの筋が
#: 同じ答えを指している。**
#:
#: 筋を通すなら、旋回が続く行では先読み停止そのものをやめるべき（C++）。
#: ここは実測で合わせた暫定。**速度を変えたら測り直すこと**（先読みは速度に比例する）。
#:
#:
#: 旋回中の並進そのものは、最初に測った値と再現した（実測 -0.4 / +25.5 / -51.9 / -13.4
#: に対し、表は -1.0 / +23.0 / -51.8 / -14.5）。**モデルは正しい。**
#:
#: ところが命令に対する合計は、全部の行で一律 17〜27mm 足りなかった。**先読み停止の
#: ぶんが返ってきていない。**
#:
#:     行は  SCD − 先読み(≈22mm)  で切り替わる
#:     その後  旋回中の並進  が起きる
#:     合計 = SCD − 先読み + 並進
#:     補正で SCD' = SCD − 並進 にしたので、合計 = SCD − 先読み
#:
#: **先読み停止は「空走で着地する」前提だが、実際に起きるのは空走ではなく旋回中の
#: 並進だった。** その並進を別途相殺したので、先読みで引いたぶんが宙に浮く。
#: そこで、**旋回中の並進と先読みの両方**を含む値にしてある（残差で実測）。
#:
#: 筋を通すなら、旋回が続く行では先読み停止そのものをやめるべき（C++）。
#: ここは実測で合わせた暫定。**速度を変えたら測り直すこと**（先読みは速度に比例する）。
#:
#: 鍵は (直前が前進か, 旋回の SCR が正か)。
TURN_DRIFT_MM = {
    (True, True): -25.9,
    (True, False): 4.6,
    (False, True): -29.7,
    (False, False): 1.1,
}

#: `TURN_DRIFT_MM` の補正を切る置き場（1行目に 1 か 0）。理由は `FOLLOW_PLAN_SWITCH` と同じ
TURN_DRIFT_SWITCH = "./sorot_spike/turn_drift.txt"

#: 補正を入れるか
USE_TURN_DRIFT = True
#: `USE_HEADING_HOLD` を走行体で切り替える置き場（1行目に 1 か 0）。
#: 理由は `FOLLOW_PLAN_SWITCH` と同じ
USE_HEADING_HOLD_SWITCH = "./sorot_spike/heading_hold.txt"

#: 走りながら経路を書き直すか（`route_follower.py`）。
#: **やめても経路はそのまま走る**（開ループに戻るだけ）
#:
#: **既定は True。** 2026-09-21 の実機で、終点の開きが
#: 開ループ 84/90/108mm に対し 23/63mm になった（`Docs/route_plan_follow.md` 0章）。
#: 重なりが無く、走行ごとのばらつきでは説明できない。
FOLLOW_PLAN = True

#: `FOLLOW_PLAN` を走行体で切り替える置き場（1行目に 1 か 0）。
#:
#: **走行体はこのリポジトリの checkout なので、ソースを直に書き換えても
#: `git pull` / merge で戻る**（2026-09-21 に戻り、効いていない設定で2走行した）。
#: この置き場は git の管理外なので戻らない。無ければ上の既定で走る
FOLLOW_PLAN_SWITCH = "./sorot_spike/follow_plan.txt"

#: `USE_HEADING_CAMERA` を走行体で切り替える置き場（1行目に 1 か 0）。
#: 理由は `FOLLOW_PLAN_SWITCH` と同じ
USE_HEADING_CAMERA_SWITCH = "./sorot_spike/heading_camera.txt"

#: `USE_HEADING_FIX` を走行体で切り替える置き場（1行目に 1 か 0）。
#: 理由は `FOLLOW_PLAN_SWITCH` と同じ
USE_HEADING_FIX_SWITCH = "./sorot_spike/heading_fix.txt"

#: 旋回の直後の直進で、カメラで向きを測るか
USE_HEADING_CAMERA = True

#: 向きを測るスクリプト（RoughSpot/Python からの相対パス。SCRIPT_LEN 40 まで）
HEADING_SCRIPT = "2026/recog/grid_heading.py"

#: 測った向きを**自己位置**へも打ち直す側に替えるか（Docs/heading_correction.md）
#:
#: **もう「どちらか」ではない。** SCRIPT は1行に1つしか書けないが、直す量が別でも
#: **測る量は同じ**（格子に対する向きのずれ）なので、打ち直す側（HEADING_FIX_SCRIPT）は
#: 1回の観測から両方へ配る。
#:
#:     床のコードで測れた  → 自己位置を打ち直す ＋ 走らせ方の目標も直す
#:     測れなかった        → 目印・黒線で測り直し、走らせ方の目標だけ直す（従来どおり）
#:
#: つまり True にしても、**向き保持のカメラ補正は止まらない**
#: （`heading_fix_task.PUBLISH_HEADING_DEV` / `FALLBACK_TO_MARKERS`）。
#: **既定はまだ False。** 実機で1本走らせて確かめてから既定を変えること
USE_HEADING_FIX = False

#: 自己位置の向きを打ち直すスクリプト
HEADING_FIX_SCRIPT = "2026/recog/heading_fix.py"

#: 何回の旋回につき 1回測るか。1 なら旋回のたび
HEADING_EVERY = 1

#: 後退の直後のその場旋回の前に、止まる行を挟むか（モジュールの説明を参照）
#:
#: **既定は False。挟むのは PC（PC-System_2026 の `CreateRouteV2`、2026-09-22）で、
#: こちらはその保険。** 両方効いても二重にはならない（`with_pause_after_backward` は
#: 後退の次の行が旋回のときだけ挟むので、PC が挟んでいれば当たらない）。
#:
#: **PC 側を配れないまま走るときは、走行体で
#: `sorot_spike/pause_after_backward.txt` に `1` を書くこと。**
#: 挟まっているかは、登録時のログ（`後退と旋回の間に停止 …`）と
#: `route_used.json` の行数で確かめられる
PAUSE_AFTER_BACKWARD = False

#: `PAUSE_AFTER_BACKWARD` を走行体で切り替える置き場（1行目に 1 か 0）。
#: 理由は `FOLLOW_PLAN_SWITCH` と同じ
PAUSE_AFTER_BACKWARD_SWITCH = "./sorot_spike/pause_after_backward.txt"

#: 挟む停止の長さ[ms]
#:
#: **200〜300ms を狙う。** 終端速度は 298〜374mm/s で、止まるまでの空走は
#: 実測 0〜5mm（`SubScenario/check_reverse_coast.json`）。250ms あれば足りる。
#: 本番の経路（71行）では12か所に挟まるので、**走行時間は約3秒延びる。**
#: 効きを見て詰めること
PAUSE_AFTER_BACKWARD_MS = 250

#: 走行方法の番号（C++ の RunMethod.h と同じ値）
FUNCNO_LINEAR = 5
FUNCNO_REVOLUTION = 3
FUNCNO_HEADING_LINEAR = 19

#: 切替条件の「角度」（C++ の SwitchiType.h の SW_DIR）
SCJFN_DIR = 0x0020

#: 切替条件の「走行距離」「時間経過」（C++ の SwitchiType.h の SW_DISTANCE / SW_TIMER）
SCJFN_DISTANCE = 0x0080
SCJFN_TIMER = 0x0001


class SwitchFile:
    """
    git の管理外の置き場で設定を上書きする。

    **走行体はこのリポジトリの checkout なので、ソースを直に書き換えても
    `git pull` / merge で戻る。** 置き場なら戻らない。
    """

    @classmethod
    def read(cls, path: str, default: bool) -> bool:
        """1行目が 1/true/on なら True、0/false/off なら False。無ければ既定"""
        try:
            with open(path, encoding="utf-8") as f:
                word = f.readline().strip().lower()
        except OSError:
            return default
        if word in ("1", "true", "on", "yes"):
            return True
        if word in ("0", "false", "off", "no"):
            return False
        logger.warning("%s の中身（%r）が読めません。既定 %s で走ります",
                       path, word, default)
        return default


def is_turn_row(row: Any) -> bool:
    """角度で止まるその場旋回の行か（C++ の classifyHeadingRow と同じ見分け方）"""
    run = row.get("RunInfo") if isinstance(row, dict) else None
    sw = row.get("SwitchInfo") if isinstance(row, dict) else None
    if not isinstance(run, dict) or not isinstance(sw, dict):
        return False
    return run.get("FUNCNO") == FUNCNO_REVOLUTION and (sw.get("SCJFN", 0) & SCJFN_DIR) != 0


def heading_script() -> str:
    """旋回の直後の直進に付ける SCRIPT（向き保持の目標を直す / 自己位置を打ち直す）"""
    fix = SwitchFile.read(USE_HEADING_FIX_SWITCH, USE_HEADING_FIX)
    logger.info("旋回の直後の直進: %s", "自己位置を打ち直す" if fix else "向き保持の目標を直す")
    return HEADING_FIX_SCRIPT if fix else HEADING_SCRIPT


def with_heading_camera(scenario: list, every: int = HEADING_EVERY) -> list:
    """
    旋回の直後の直進の行に、向きを測る SCRIPT を付けたコピーを返す。

    もともと SCRIPT が書かれている行（ヒントカードの読み取りなど）は触らない。
    受け取った辞書は書き換えない。
    """
    out = []
    turns = 0
    after_turn = False
    for row in scenario:
        run = row.get("RunInfo") if isinstance(row, dict) else None
        if isinstance(run, dict) and after_turn and run.get("FWD", 0) != 0 and not run.get("SCRIPT"):
            # 1回目の旋回の直後から every 回おき
            if (turns - 1) % max(1, every) == 0:
                row = {**row, "RunInfo": {**run, "SCRIPT": heading_script()}}
            after_turn = False
        if is_turn_row(row):
            turns += 1
            after_turn = True
        out.append(row)
    return out


def use_heading_hold(scenario: list) -> list:
    """
    直進（FUNCNO 5、FWD≠0、TRN 0）を向き保持直進（FUNCNO 19）に替えたコピーを返す。

    止まって待つ行（FWD 0）とカーブ（TRN≠0）は替えない。受け取った辞書は書き換えない。
    """
    out = []
    for row in scenario:
        run = row.get("RunInfo") if isinstance(row, dict) else None
        if (isinstance(run, dict) and run.get("FUNCNO") == FUNCNO_LINEAR
                and run.get("FWD", 0) != 0 and run.get("TRN", 0) == 0):
            row = {**row, "RunInfo": {**run, "FUNCNO": FUNCNO_HEADING_LINEAR}}
        out.append(row)
    return out


def compensate_turn_drift(scenario: list) -> list:
    """
    旋回の最中に動くぶんを、**直前の直進の距離から差し引いた**コピーを返す。

    旋回そのものには距離の指定が無いので、手前の直進で相殺する。

        いま  後退 -860 → 旋回で -52 → 合計 -912
        あと  後退 -808 → 旋回で -52 → 合計 -860

    **止まって待つ行（FWD 0）は読み飛ばして、その先の旋回を見る**
    （後退と旋回の間に停止を挟む仕組みと併用できるようにするため）。
    受け取った辞書は書き換えない。
    """
    out = [dict(row) if isinstance(row, dict) else row for row in scenario]
    for i, row in enumerate(out):
        run = row.get("RunInfo") if isinstance(row, dict) else None
        sw = row.get("SwitchInfo") if isinstance(row, dict) else None
        if not isinstance(run, dict) or not isinstance(sw, dict):
            continue
        if run.get("FUNCNO") not in (FUNCNO_LINEAR, FUNCNO_HEADING_LINEAR):
            continue
        scd = sw.get("SCD") or 0
        if not scd:
            continue
        turn = _next_turn_row(out, i)
        if turn is None:
            continue
        scr = turn.get("SwitchInfo", {}).get("SCR") or 0
        if not scr:
            continue
        drift = TURN_DRIFT_MM.get((scd > 0, scr > 0))
        if drift is None:
            continue
        out[i] = {**row, "SwitchInfo": {**sw, "SCD": int(round(scd - drift))}}
    return out


def _next_turn_row(rows: list, index: int):
    """`index` の次にある旋回の行。止まって待つ行は読み飛ばす。無ければ None"""
    for row in rows[index + 1:]:
        run = row.get("RunInfo") if isinstance(row, dict) else None
        if not isinstance(run, dict):
            return None
        if is_turn_row(row):
            return row
        if run.get("FWD", 0) == 0:
            continue                      # 止まって待つ行は跨ぐ
        return None                       # 走る行が挟まれば、その旋回は別の話
    return None

def no_heading_hold(scenario: list) -> list:
    """
    向き保持直進（FUNCNO 19）を直進（FUNCNO 5）に戻したコピーを返す。

    **PC はすでに FUNCNO 19 を送ってくる**（2026-09-22 の実測）。`use_heading_hold`
    は 5 を 19 に替える関数なので、止めても行は 19 のままだった。やめたいときは
    こちらで戻す。止まって待つ行とカーブは触らない（19 ではないので当たらない）。
    """
    out = []
    for row in scenario:
        run = row.get("RunInfo") if isinstance(row, dict) else None
        if isinstance(run, dict) and run.get("FUNCNO") == FUNCNO_HEADING_LINEAR:
            row = {**row, "RunInfo": {**run, "FUNCNO": FUNCNO_LINEAR}}
        out.append(row)
    return out


def is_backward_row(row: Any) -> bool:
    """距離で止まる後退の行か（`route_plan.RouteRow.is_straight` と同じ見分け方）"""
    run = row.get("RunInfo") if isinstance(row, dict) else None
    sw = row.get("SwitchInfo") if isinstance(row, dict) else None
    if not isinstance(run, dict) or not isinstance(sw, dict):
        return False
    if run.get("FUNCNO") not in (FUNCNO_LINEAR, FUNCNO_HEADING_LINEAR):
        return False
    if (sw.get("SCJFN", 0) & SCJFN_DISTANCE) == 0:
        return False
    return (sw.get("SCD") or 0) < 0


def pause_row(ms: int = PAUSE_AFTER_BACKWARD_MS, after: Any = None) -> dict:
    """止まって待つだけの行を作る（`SubScenario/check_reverse_coast.json` と同じ形）

    **`FUNCNO 5` で書く。** C++ はこの形（`FUNCNO 5`、`FWD 0`、`TRN 0`）を
    `HEADING_ROW_STOP` と見て、**向き保持の目標をそのまま残す**
    （`Utillity/HeadingHold.h`）。`FUNCNO 19` で書くと保持側が走り出してしまう。

    **`ARMANG` は前の行から写す。** 書かないと C++ は 0 として読み、
    **アームのモータが逆向き（+30）に回って下がり始める**
    （`Task/RunControl.cpp` の `controlArm`）。ETラリーの走行行はすべて
    `ARMANG -150` でアームを上げたまま走るので、挟んだ行だけ 0 にすると
    1回 250ms ×12回、アームを押し下げ続けることになる。

    :param after: 直前の行。`RunInfo.ARMANG` をここから引き継ぐ
    """
    run = {"FUNCNO": FUNCNO_LINEAR, "FWD": 0, "TRN": 0,
           "KP": 0, "KI": 0, "KD": 0, "NOBLNCE": 1, "WPER": 0}
    before = after.get("RunInfo") if isinstance(after, dict) else None
    if isinstance(before, dict) and before.get("ARMANG"):
        run["ARMANG"] = before["ARMANG"]
    return {
        "Comment": "後退の慣性を落とす停止 %dms（この次が旋回）" % ms,
        "RunInfo": run,
        "SwitchInfo": {"SCJFN": SCJFN_TIMER, "SCT": int(ms), "SCD": 0, "SCR": 0,
                       "SCC": 0, "SCV": 0, "JMPSNO": 0, "SCX": 0, "SCY": 0},
        "CoordCorrectInfo": {},
    }


def is_pause_row(row: Any) -> bool:
    """止まって待つだけの行か（C++ の `HEADING_ROW_STOP` と同じ見分け方）

    `route_plan.RouteRow.is_pause` と同じ規則。**PC が入れてきた停止も当たる**ので、
    すでに挟まっている経路へ二重に挟むことがない。
    """
    run = row.get("RunInfo") if isinstance(row, dict) else None
    if not isinstance(run, dict):
        return False
    return (run.get("FUNCNO") == FUNCNO_LINEAR
            and not run.get("FWD", 0) and not run.get("TRN", 0))


def with_pause_after_backward(scenario: list, ms: int = PAUSE_AFTER_BACKWARD_MS) -> list:
    """
    後退の直後がその場旋回なら、その間に止まる行を挟んだコピーを返す。

    後退が続くときに挟むのは**最後の1つのあと**だけ（旋回に接する所だけが問題で、
    後退どうしの間には慣性を落とす理由が無い）。受け取った辞書は書き換えない。

    **PC がすでに挟んでいれば何もしない。** 見るのは「後退の次の行が旋回か」なので、
    間に停止が入っていれば当たらない。**PC 側（`CreateRouteV2.convert_route_to_scenario`）
    と両方が効いても 500ms にはならない。**

    **行が1つ増える。** 経路の行番号は並びでそのまま付くので
    （`route_numbering.RouteNumbering`）、挟んだあとの並びを
    `route_follower` / `scenario_rows` へ渡すこと。渡すものが食い違うと、
    走行中の書き直しが**別の行を直す**（`extract` が1か所で作って配っている）。
    """
    out = []
    inserted = 0
    for index, row in enumerate(scenario):
        out.append(row)
        if not is_backward_row(row):
            continue
        following = scenario[index + 1] if index + 1 < len(scenario) else None
        if following is not None and is_turn_row(following):
            out.append(pause_row(ms, after=row))
            inserted += 1
    return renumber_rows(out) if inserted else out


def renumber_rows(scenario: list) -> list:
    """
    `SNO`/`CNO`/`PRVSNO`/`PRVCNO` を先頭から振り直したコピーを返す。

    **行を挟むと、後ろの番号が1つずつずれる。** 挟んだ行にだけ番号が無く、続く行が
    元の番号のままだと、経路を読む人にも道具にも「どの行がどれか」が分からなくなる
    （実機の `route_used.json` で 17 → 番号なし → 18 と並んだ）。PC が組むときは
    `ScenarioBuilder` が連番で振るので、**こちらで挟んだときも同じ形にそろえる。**

    **走行そのものには効かない。** C++ は経路の行の番号を捨て、挿し込み先に合わせて
    振り直す（`Task/ScenarioControl.cpp` の `applyInsertPlan`）。Python 側も経路の行の
    番号は読まず、**並び順から番号を出す**（`route_numbering.RouteNumbering`）。
    ここでそろえるのは、残った記録を読めるようにするためと、**番号の無い行が
    `SNO 0 / CNO 0`（シナリオの終わり）と見分けにくいのをやめるため。**

    元の番号が無い経路（テストの作り物など）は触らない。**終了コマンド
    （`SNO 0` かつ `CNO 0`）はそのまま**で、そこで振り直しを止める。
    """
    first = next((r for r in scenario if _numbered(r) and not is_end_row(r)), None)
    if first is None:
        return scenario         # 番号の無い経路（テストの作り物など）は触らない
    sno = int(first["SNO"])
    cno = int(first["CNO"])
    if sno <= 0 or cno <= 0:
        # 0 から振ると、先頭の行がシナリオの終わりに見える
        logger.warning("経路の先頭の番号が %s/%s なので、番号は振り直しません", sno, cno)
        return scenario

    out = []
    prv_sno, prv_cno = first.get("PRVSNO", sno), first.get("PRVCNO", cno - 1)
    for row in scenario:
        if not isinstance(row, dict) or is_end_row(row):
            out.append(row)         # 終了コマンドはそのまま
            continue
        out.append({**row, "SNO": sno, "CNO": cno, "PRVSNO": prv_sno, "PRVCNO": prv_cno})
        prv_sno, prv_cno = sno, cno
        cno += 1
    return out


def _numbered(row: Any) -> bool:
    return isinstance(row, dict) and "SNO" in row and "CNO" in row


def is_end_row(row: Any) -> bool:
    """終了コマンド（`SNO 0` かつ `CNO 0`）か

    **番号の無い行は終了ではない。** C++ もそう見ている（`ScenarioSender.cpp` の
    `isEndOfScenario` は `line.value("SNO", -1)` と、**既定を -1** にして読む）。
    ここで `or 0` と書くと 0 と欠落が同じになり、番号の無い行がシナリオの終わりに
    化ける。
    """
    if not _numbered(row):
        return False
    return int(row["SNO"] or 0) == 0 and int(row["CNO"] or 0) == 0


class RouteRegister:
    """届いたシナリオを、C++ の置き場へ1回だけ渡す。"""

    _instance: Optional["RouteRegister"] = None
    _instance_lock = threading.Lock()

    @classmethod
    def get_instance(cls) -> "RouteRegister":
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
        #: 経路を頼んだか。頼む前に届いたものは使わない（モジュールの説明を参照）
        self._expecting = False
        #: 頼んだ難所。応答に区分が入っていないときに補う
        self._expected_kbn = 0
        #: 置いた経路の中身。同じものが何度も届いても置き直さない
        self._placed: Optional[str] = None

    @staticmethod
    def _as_int(value: Any) -> Optional[int]:
        """数字にできれば int。できなければ None（True/False は数字とみなさない）"""
        if value is None or isinstance(value, bool):
            return None
        try:
            return int(value)
        except (TypeError, ValueError):
            return None


    # --- 要求 -----------------------------------------------------------------

    def expect(self, senario_kbn: int) -> None:
        """
        経路を頼んだことを知らせる（`handler.PcExchange.send` から呼ぶ）。

        :param senario_kbn: どの難所の経路か（`config.SENARIO_KBN_*`）
        """
        with self._lock:
            self._expecting = True
            self._expected_kbn = senario_kbn
        logger.info("経路を頼みました（難所=%s）。届いたら登録します", senario_kbn)

    def reset(self) -> None:
        """走行やり直し用。頼んだ印と置いた経路を捨てる"""
        with self._lock:
            self._expecting = False
            self._expected_kbn = 0
            self._placed = None

    # --- 受け取り -------------------------------------------------------------

    def handle_message(self, message: Any) -> bool:
        """
        PC から届いた JSON を受け取る。シナリオが入っていれば置き場へ置く。

        `PCClient.on_message` へ差して使う。要求の応答でも、頼んでいない便でも
        同じここを通る。

        :returns: 置いたら True
        """
        with self._lock:
            if not self._expecting:
                return False
            route = self.extract(message, self._expected_kbn)
            if route is None:
                return False

            flag_file = json.dumps(route, ensure_ascii=False, sort_keys=True)
            if flag_file == self._placed:
                # PC は作った経路を覚えていて、次の応答にも同じものを載せてくる
                return False

            if not ScenarioMailbox.place_route(route):
                return False
            self._placed = flag_file
            self._expecting = False
            # 打ち直しの書き込み先を決めるのに、経路の行の中身が要る
            # （番号は C++ が振り直すので、走行中に学ぶ。Common/scenario_rows.py）
            ScenarioRows.shared().set_route(route["scenario"], heading_script())

        logger.info("走行経路を登録しました: 難所=%s コマンド%d件",
                    route["senario_kbn"], len(route["scenario"]))

        # **読むだけ。走りは変えない。** PC が Comment の末尾に書いてくる場所を
        # 記録に残す（`Common/route_place.py`）。走行後に「どの行がコースのどこか」を
        # 突き合わせる相手が、いままで無かった
        try:
            logger.info("経路の場所: %s", RoutePlaces.read(route["scenario"]).summary())
        except Exception as e:      # noqa: BLE001 — 読めなくても走行は続く
            logger.error("経路の場所を読めませんでした: %s", e)

        # **走った経路は、書き直しをしなくても残す。** 走行後に突き合わせる相手が
        # これしかない（/dev/shm の経路は C++ が引き取ると消える）。開ループの
        # 基準を取るときにも要るので、FOLLOW_PLAN の外に置く
        try:
            route_follower.save_route(route["scenario"])
        except Exception as e:      # noqa: BLE001 — 残せなくても走行は続く
            logger.error("走った経路を残せませんでした: %s", e)

        # 経路が届いた＝これからその難所を走る。**その年の認識をここで起こす**
        # （ETラリーならゲートの見張り。`Docs/gate_check.md`）。
        # **シナリオの SCRIPT では起こせない。** 経路の行は PC が作るうえ、
        # 入り口の行は FUNCNO 15 と SCRIPT が同じ置き場を取り合う（同上）
        started = season_hook("on_route_registered")
        if started is not None:
            try:
                started(route["senario_kbn"], route["scenario"])
            except Exception as e:      # noqa: BLE001 — 起こせなくても経路は走る
                logger.error("経路が届いたときの処理に失敗しました: %s", e)

        # ここから先は自己位置を見ながら走る（`Docs/route_plan_follow.md`）。
        # **渡したのと同じ行を渡すこと。** 向き保持直進に替えたあとのものを
        # 積分しないと、居るべき場所が実際の走りとずれる
        # **どちらで走ったかを毎回残す。** あとから走行を見比べるとき、
        # 開ループだったのか書き直していたのかが分からなくなる（現地で起きた）
        follow = self.follow_plan()
        logger.info("経路の書き直し: %s", "する" if follow else "しない（開ループ）")
        if follow:
            try:
                route_follower.begin(route["scenario"], route["senario_kbn"])
            except Exception as e:      # noqa: BLE001 — 見張れなくても経路は走る
                logger.error("経路の見張りを起こせませんでした: %s", e)
        else:
            # 書き直しの記録にも区切りを入れておく（前の走行の節を
            # `Tools/compare_route.py --rewrites` に拾わせないため）
            route_follower.mark_open_loop()
        return True

    @classmethod
    def follow_plan(cls) -> bool:
        """
        走りながら経路を書き直すか。**置き場があればそちらが勝つ。**

        `FOLLOW_PLAN_SWITCH` の1行目が 1/true/on なら書き直す、0/false/off なら
        書き直さない。読めない中身なら既定（`FOLLOW_PLAN`）のまま。
        """
        return SwitchFile.read(FOLLOW_PLAN_SWITCH, FOLLOW_PLAN)

    @classmethod
    def extract(cls, message: Any, default_kbn: int = 0) -> Optional[dict]:
        """
        届いた JSON から走行経路を取り出す。使えなければ None。

        形は `SubScenario/*.json` と同じコマンドセットの配列で、C++ 側は
        読み込みと同じ変換で取り込む（`RoughSpot/Cpp/ResultRead.cpp`）。

            {"status": 1, "senario_kbn": 1, "scenario": [ {...}, {...} ]}

        `status` が 1 でない（未処理・失敗）ときや、コマンドが 0件のときは None を
        返す。走行中のシナリオに触らず、元の経路のまま走り続けさせるため。

        :param default_kbn: 応答に `senario_kbn` が無いときに使う難所の識別値
        """
        if not isinstance(message, dict):
            return None

        scenario = message.get("scenario")
        if not isinstance(scenario, list) or len(scenario) == 0:
            return None

        status = cls._as_int(message.get("status"))
        if status is not None and status != STATUS_OK:
            logger.warning("経路の応答が成功していません: status=%s", status)
            return None

        # 区分は旧い名前（scenarioType）でも読める。入っていなければ頼んだ難所で補う
        kbn = cls._as_int(message.get("senario_kbn"))
        if kbn is None:
            kbn = cls._as_int(message.get("scenarioType"))
        if kbn is None or kbn <= 0:
            kbn = default_kbn
        if kbn <= 0:
            logger.warning("経路のシナリオ区分が分かりません")
            return None

        if kbn == SENARIO_KBN_ET_RALLY:
            hold = SwitchFile.read(USE_HEADING_HOLD_SWITCH, USE_HEADING_HOLD)
            logger.info("直進の走り方: %s",
                        "向き保持（FUNCNO 19）" if hold else "エンコーダ補正（FUNCNO 5）")
            # **PC はすでに FUNCNO 19 を送ってくる。** 替えるほうではなく
            # 戻すほうが要る（use_heading_hold は 5 を 19 にする関数で空振りする）
            scenario = use_heading_hold(scenario) if hold else no_heading_hold(scenario)
            if SwitchFile.read(TURN_DRIFT_SWITCH, USE_TURN_DRIFT):
                before = scenario
                scenario = compensate_turn_drift(scenario)
                n = sum(1 for a, b in zip(before, scenario)
                        if a.get("SwitchInfo", {}).get("SCD")
                        != b.get("SwitchInfo", {}).get("SCD"))
                logger.info("旋回中の並進を %d 行で相殺しました", n)
            else:
                logger.info("旋回中の並進の相殺: しない")
            camera = SwitchFile.read(USE_HEADING_CAMERA_SWITCH, USE_HEADING_CAMERA)
            logger.info("カメラで向きを測る: %s", "する" if camera else "しない")
            if camera:
                scenario = with_heading_camera(scenario)
            # **いちばん最後に挟む。** 上の2つは「旋回の直後の直進」のような
            # 並びを見るので、先に行を増やすとその見え方が変わる
            pause = SwitchFile.read(PAUSE_AFTER_BACKWARD_SWITCH, PAUSE_AFTER_BACKWARD)
            if pause:
                before = len(scenario)
                scenario = with_pause_after_backward(scenario)
                logger.info("後退と旋回の間に停止 %dms を %d件 挟みました（%d行→%d行）",
                            PAUSE_AFTER_BACKWARD_MS, len(scenario) - before,
                            before, len(scenario))
            else:
                logger.info("後退と旋回の間の停止: 挟まない")
        return {"senario_kbn": kbn, "scenario": scenario}
