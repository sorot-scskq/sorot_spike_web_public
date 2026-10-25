"""
2026年度の設定。共通部分は Common/config.py にある。

ここに書くのは、その年に使う識別値の顔ぶれと、待ち時間だけ
（今年の難所: ボトルデリバリー・ET相撲・ET ラリー）。接続先・撮影のパス・ログ設定・識別値の採番は共通。

    from config import *                # 共通
    from config_2026 import USED_JUDG, TIMEOUT_CONFIG

【ファイル名に年が入っている理由】
年のフォルダは sys.path に足して読む（Common/config.py の add_season_to_path）。
同じ名前のファイルが年ごとにあると、先に見つかったほうが読まれてどちらか
分からなくなる。名前は全体で 1つにすること。

【シミュレータの置き場について】
シミュレータは Python を /python/<ファイル名> という**平らな置き場**へ配る
（vite.config.js の python-assets）。年ごとのフォルダに同じ名前のファイルが
あると、あとの年で上書きされて片方が消える。名前は全体で 1つにすること。
**2026/recog/ は配っていない**（Common/ と同じ綴りのファイルがあり、平らな
置き場では衝突するため）。
"""

import logging
import os

from config import (
    SENARIO_KBN_ET_RALLY,
    JUDG_BOTTLE, JUDG_ET_RALLY, JUDG_GATE, JUDG_QR1, JUDG_QR2,
)

#: ラベル色ごとの運び先シナリオ。色が決まったら、この JSON で走行中のシナリオを
#: 差し替える（Common/scenario_update.py）。基本のシナリオには既定のルートを
#: 書いておくので、決まらなければそのまま既定を走る
#: **どのシナリオを走らせるかで、向ける先を変える。**
#: 中の SNO/CNO が走行中のシナリオと一致していないと差し替えが効かない
#: （どこにも当たらない行は末尾に足されるだけで、実行されない）。
#:
#:   custom_bottle_route_*.json  custom.json 用（本番）。運び先の選択（20/7）の飛び先を 21/22/23 にする1行
#:   branch_route_*.json         三角コースでの確認用（SNO 99。差し替えと挿入を見る）
#:   bottle_route_*.json         sample_bottle_color_branch.json 用（SNO 2。運び先へ向かう6行）
BOTTLE_ROUTE_FILES = {
    "red": "./sorot_spike/SubScenario/custom_bottle_route_red.json",
    "blue": "./sorot_spike/SubScenario/custom_bottle_route_blue.json",
    "yellow": "./sorot_spike/SubScenario/custom_bottle_route_yellow.json",
}

#: ヒントカードの復号文字列ごとの差し替え先。使うようになったら足す
QR_ROUTE_FILES = {}

#: 床のホモグラフィ（画像[px] → 走行体から見た床[mm]）の置き場。
#:
#: **実機でメジャーを当てて求めるもので、求まるまでこのファイルは存在しない。**
#: 無ければ `GroundPlane.load()` が None を返し、床のコードによる補正は
#: 丸ごと止まる（走行は続く）。作り方は `Docs/ground_homography.md`。
#:
#: 走行ごとに変わるものではないので /dev/shm ではなくここに置き、git で持つ。
#: **カメラを動かしたら測り直す**（台の緩みも含む）。
#: 実行する場所に依らないよう、この設定ファイルの隣を絶対パスで指す。
GROUND_HOMOGRAPHY_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "ground_homography.json")

#: 走行中のシナリオ（走行体から見たパス）。
#:
#: **打ち直しを書き込む先の行の中身を知るために読む**（Common/scenario_rows.py）。
#: 差し替えは行を丸ごと置き換えるので、元の中身を写してから補正を足す必要がある。
#: C++ が読むのと同じファイルにすること（ScenarioSetFactory.cpp の "./sorot_spike/SubScenario/"）
SCENARIO_FILE = "./sorot_spike/SubScenario/custom.json"

#: 床の二次元コードで自己位置の向きを打ち直すか（Docs/heading_correction.md）
USE_HEADING_FIX = True

#: ヒントカードが読めた時点で、止まって読む区間を終わらせるか
#: （2026/hint_card_advance.py。シナリオ側は SCJFN 2049 と SCV）。
#:
#: **False にしてもシナリオは直さなくてよい。** 応答が来なくなるだけで、
#: 行は SCT（11秒）で抜ける。これまでと同じ走りに戻る。
#:
#: **制御関数の差し込み口は走行体に1つしかない**（remote_control.set_controller）。
#: 「経路を作りながら走らせる」（Docs/remote_control.md 4章）を入れるときは、
#: 1つの関数で両方さばくか、ここを False にすること
USE_HINT_CARD_ADVANCE = True
logger = logging.getLogger(__name__)

#: 今年つかう画像識別値
USED_JUDG = (
    JUDG_BOTTLE,
    JUDG_GATE,
    JUDG_QR1,
    JUDG_QR2,
    JUDG_ET_RALLY,
)

#: 要求区分(prefix)ごとのタイムアウト設定（秒）
TIMEOUT_CONFIG = {
    JUDG_BOTTLE: 3.0,  # ボトル判定
    JUDG_GATE: 3.0,  # ゲート判定
    JUDG_QR1: 2.0,  # QR1復号
    JUDG_QR2: 3.0,  # QR2復号
    JUDG_ET_RALLY: 5.0,  # ET ラリーの経路計算（PC 側で最短経路を組み立てる）
}


# --------------------------------------------------------------------------
# 差し込み口（ev3_python.py が getattr で探して呼ぶ。Common/config.py を参照）
# --------------------------------------------------------------------------


def set_pc_client(pc_client):
    """PC（無線通信デバイス）へ繋ぐ口を受け取る（ヒントカードの内容を送るのに使う）"""
    from hint_card_capture import HintCardCapture
    HintCardCapture.get_instance().set_pc_client(pc_client)


def start_recognizers():
    """走行体のカメラを使う認識処理を起こす"""
    from bottle_color_task import BottleColorTask
    BottleColorTask.get_instance().start()

    if USE_HINT_CARD_ADVANCE:
        # ヒントカードが読めたら、止まって読む区間を終わらせる（SCJFN 2049）。
        # 読めたかを知っているのは Python 側なので、区間の終わりもこちらが言う
        from hint_card_advance import HintCardAdvance
        from remote_control import set_controller
        set_controller(HintCardAdvance().control)

    if USE_HEADING_FIX:
        # 打ち直しを書き込む先の行を知るため、走り出す前にシナリオを読んでおく。
        # 読めなくても走行は続く（打ち直しをしないだけ）
        from scenario_rows import ScenarioRows
        ScenarioRows.shared().load_file(SCENARIO_FILE)
    # ET相撲の力士寄せは、シナリオの 40/2 から SCRIPT で差す
    # （2026/recog/et_sumo.py）。ここで差していたのをやめた。
    #
    # **戻すときはこの2行を生かすだけ。** SCRIPT の要求が届くまで最大 40ms あり、
    # その間は制御関数が無いので C++ は指令を受け取れず (0,0) を出す
    # （RunMethod/RemoteControl.cpp の末尾）。ここで差しておけばその空白が埋まる。
    # 二重に差しても後勝ちで無害。
    # from et_sumo import attach
    # attach()


def on_route_registered(senario_kbn, scenario):
    """
    PC から届いた経路を置いたとき（`Common/route_register.py` が呼ぶ）。

    **ふつうは何もしない。** ゲートの見張りは、PC がゲートの行に差した SCRIPT
    （`2026/recog/gate_check.py`）が起こす。そのほうがゲートの近くだけを見るので安い。

    **PC が古くて差してこないときだけ**、ここで走り終わりまでの見張りへ落とす。
    こちらは経路のあいだずっと CPU を使う（1枚 29.5ms）。`Docs/gate_check.md`
    """
    if senario_kbn != SENARIO_KBN_ET_RALLY:
        return
    from gate_check_task import GATE_CHECK_SCRIPT, GateCheckTask
    if any(str((row.get("RunInfo") or {}).get("SCRIPT", "")).endswith(GATE_CHECK_SCRIPT)
           for row in scenario or [] if isinstance(row, dict)):
        return
    logger.info("経路にゲートの確認が入っていません。走り終わりまで見張ります（PC が古い）")
    GateCheckTask.get_instance().watch(seconds=None)


def handle_request(identifier, request_id):
    """
    共通が知らない識別値を捌く。捌いたら True。

    走行体の中だけで完結する要求は、いまはシナリオの SCRIPT
    （2026/recog/*.py）から呼ぶので、ここで捌くものは無い。
    """
    return False


def fill_result(result):
    """判定結果の応答へ、今年ぶんの項目を足す"""
    from bottle_color_task import BottleColorTask
    BottleColorTask.get_instance().fill_result(result)


def classify_qr(decoded_text):
    """
    読んだ二次元コードが、どちらのヒントカードか。

    カードには番号が書かれていないので、中身で見分ける（2026/hint_card.py）。
    分からなければ None を返し、共通側は読んだ順（識別値）で決める。

    :returns: "qr1" | "qr2" | None
    """
    from hint_card import HintCard, HintCardText

    card = HintCardText.identify(decoded_text)
    if card == HintCard.CARD1:
        return "qr1"
    if card == HintCard.CARD2:
        return "qr2"
    return None
