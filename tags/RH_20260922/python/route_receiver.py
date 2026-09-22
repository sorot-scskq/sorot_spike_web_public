"""
走行経路の受け取り（2026: 無線通信デバイス／identifier=3）。

LAP ゲートを通過したあと、PC 側へ「この先の走行経路をください」と要求し、
返ってきたシナリオを走行中のシナリオへ足す。走行経路は走ってみるまで
決まらない（PC 側がコースの状況を見て決める）ので、走行体は自分で経路を
持たず、通過した時点で取りにいく。

    走行体 ──{"identifier": 3}──▶ 無線通信デバイス（PC）
           ◀─{"identifier": 3, "status": 1, "senario_kbn": n, "scenario": [...]}─

【共通部分は Common/wireless_device.py にある】
「合図が来たら 1回だけ要求し、使える応答を 1回だけ受け取る」という流れは
毎年同じなので、そちらへ出してある。ここに書くのは今年ぶんだけ。
  - 識別値の割り当て
  - シナリオ区分（今年の難所の名前）
  - 届いたものが走行に使えるかの判定

【識別値の番号について】
この番号は**無線通信デバイス専用**で、Common/config.py の JUDG_* とは
別のものである。同じ 3 でも意味が違うので混ぜないこと。

    こちら（WebSocket 8765）  3 = 走行経路送付
    config.py（TCP 12345）    3 = JUDG_GATE（キャリーゲート判定）

【いつ要求するかを、ここでは決めない】
「LAP ゲートを通過した」ことをどう知るかは環境で違う。実機はコースを走った
距離やゲートのセンサ、シミュレータはコース図の線分との交差で判定する。
どちらも外から notify_lap_gate_passed() を呼んでもらう形にしてあるので、
このモジュールは判定方法を知らない。
"""

from __future__ import annotations

import logging
from typing import Any, Optional

try:
    from wireless_device import RequestOnceReceiver, as_int, get_field, get_identifier, is_status_ok
except ImportError:  # パッケージとして読み込まれた場合
    try:
        from ..Common.wireless_device import (
            RequestOnceReceiver, as_int, get_field, get_identifier, is_status_ok,
        )
    except (ImportError, ValueError):
        from Common.wireless_device import (
            RequestOnceReceiver, as_int, get_field, get_identifier, is_status_ok,
        )

logger = logging.getLogger(__name__)

# --------------------------------------------------------------------------
# 無線通信デバイスの識別値（Common/config.py の JUDG_* とは別物）
# --------------------------------------------------------------------------

#: ヒントカード1
IDENTIFIER_HINT_CARD1 = 1
#: ヒントカード2
IDENTIFIER_HINT_CARD2 = 2
#: 走行経路送付
IDENTIFIER_ROUTE = 3

# --------------------------------------------------------------------------
# シナリオ区分（今年の難所）
# --------------------------------------------------------------------------

SENARIO_KBN_ET_RALLY = 1
SENARIO_KBN_ET_SUMO = 2
SENARIO_KBN_GARAGE = 3

SENARIO_KBN_LABEL = {
    SENARIO_KBN_ET_RALLY: "ETラリー",
    SENARIO_KBN_ET_SUMO: "ET相撲",
    SENARIO_KBN_GARAGE: "ガレージまで",
}


def senario_kbn_label(kbn: Any) -> str:
    """シナリオ区分の呼び名。知らない番号はそのまま返す"""
    return SENARIO_KBN_LABEL.get(kbn, str(kbn))


def get_senario_kbn(message: Any) -> Optional[int]:
    """シナリオ区分を取る。旧い名前（scenarioType）でも読める"""
    return as_int(get_field(message, "senario_kbn", "scenarioType"))


def is_applicable_route(message: Any) -> bool:
    """
    走行に使える経路が入っているか。

    識別値が 3 で、処理が成功していて、コマンドが 1件以上あること。
    status が 0（未処理）や -1（失敗）のときは、走行中のシナリオを
    触らずに元の経路のまま走り続ける。
    """
    if get_identifier(message) != IDENTIFIER_ROUTE:
        return False
    if not is_status_ok(message):
        return False
    scenario = message.get("scenario")
    return isinstance(scenario, list) and len(scenario) > 0


# --------------------------------------------------------------------------
# 受け取り本体
# --------------------------------------------------------------------------


class RouteReceiver(RequestOnceReceiver):
    """
    LAP ゲート通過後に走行経路を受け取り、走行中のシナリオへ足す。

    差し替え口は 3つ。
        set_communicator … 送り口（実機は TCP、シミュレータは WebSocket）
        set_applier      … 受け取ったコマンド列を走行体へ渡す口
        on_route         … 受け取ったときの通知（画面表示・ログなど）
    """

    IDENTIFIER = IDENTIFIER_ROUTE

    def __init__(self, communicator=None, senario_kbn: int = SENARIO_KBN_GARAGE):
        super().__init__(communicator=communicator)
        #: 既定で要求するシナリオ区分
        self.senario_kbn = senario_kbn

    # --- 年ごとの中身 -----------------------------------------------------

    def build_request(self, senario_kbn: Optional[int] = None) -> dict:
        """
        送る JSON。仕様上は識別値だけでよいが、区分も付けて送る。

        キーの名前は PC 側（PC-System_2026 の WebSocketCommunicator）に合わせて
        `id` / `scenarioType` にしてある。あちらはこの名前しか読まない。
        デバッグ用サーバ（tools/ws_debug_server）は `identifier` / `senario_kbn`
        も読むが、`id` / `scenarioType` も受けるので、どちらへ繋いでも通る。
        """
        return {
            "id": IDENTIFIER_ROUTE,
            "scenarioType": self.senario_kbn if senario_kbn is None else senario_kbn,
        }

    def accepts(self, message: Any) -> bool:
        return is_applicable_route(message)

    def to_result(self, message: Any) -> dict:
        kbn = get_senario_kbn(message)
        return {
            "senario_kbn": kbn,
            "label": senario_kbn_label(kbn),
            "scenario": message["scenario"],
        }

    def apply(self, result: dict) -> None:
        """
        受け取ったコマンド列を走行体へ渡す。

        走行中に受け取るので、シナリオを入れ替えるのではなく**足す**こと。
        入れ替えると走行が最初からやり直しになる。
        """
        if self._applier is not None:
            self._applier(result["scenario"], f"走行経路({result['label']})")
        logger.info("走行経路を受け取りました: %s コマンド%d件",
                    result["label"], len(result["scenario"]))

    # --- 呼びやすい名前 ---------------------------------------------------

    def notify_lap_gate_passed(self, senario_kbn: Optional[int] = None) -> bool:
        """
        LAP ゲートを通過したことを知らせる。要求はこの 1回だけ送る。

        通過の判定は環境ごとに違うので、ここではしない（モジュールの説明を参照）。
        :returns: 要求を送ったら True。すでに送り済み・送れなかったら False
        """
        return self.notify(senario_kbn=senario_kbn)

    def request_route(self, senario_kbn: Optional[int] = None) -> bool:
        """経路を要求する（応答は待たない）"""
        return self.request(senario_kbn=senario_kbn)

    def receive_route(self, senario_kbn: Optional[int] = None,
                      timeout: Optional[float] = None) -> Optional[dict]:
        """経路を要求し、応答が返るまで待つ（実機のように待てる環境向け）"""
        return self.receive(timeout=timeout, senario_kbn=senario_kbn)

    def on_route(self, fn):
        """受け取ったときの通知を登録する。戻り値を呼ぶと解除できる"""
        return self.on_result(fn)

    # --- 受け取り結果 -----------------------------------------------------

    @property
    def route(self) -> Optional[dict]:
        """受け取った経路。まだなら None"""
        return self.result

    def get_scenario(self) -> list:
        """受け取ったコマンド列。まだなら空"""
        return list(self.result["scenario"]) if self.result else []

    def get_senario_kbn(self) -> Optional[int]:
        """受け取った経路のシナリオ区分。まだなら None"""
        return self.result["senario_kbn"] if self.result else None
