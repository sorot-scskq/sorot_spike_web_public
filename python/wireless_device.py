"""
無線通信デバイス（PC）とのやりとりの土台。

走行体は、自分だけでは決められないことを PC 側へ聞く。ヒントカードの内容、
この先の走行経路、といったものが該当する。聞き方と受け取り方は毎年ほぼ同じで、
変わるのは「何を聞くか」「返ってきたものをどう使うか」だけなので、
変わらない部分をここに置く。

    毎年変わらない（ここ）
        - 届いた JSON から識別値を読む（旧い名前でも読めるようにする）
        - 合図が来たら 1回だけ要求する
        - 使える応答が届いたら 1回だけ受け取り、走行体へ渡す
        - 走行やり直しでまた要求できるようにする
    毎年変わる（年ごとのフォルダ）
        - 識別値の割り当て
        - 要求に載せる中身
        - 届いたものが使えるかの判定

【なぜ「1回だけ」なのか】
合図（LAP ゲート通過など）の判定は毎周期呼ばれるので、素直に書くと要求を
何通も送ってしまう。また PC 側が頼んでいないものを送ってくることもある
（tools/ws_debug_server は接続した時点で経路を 1通送る）。どちらの向きでも
二重にならないよう、要求も受け取りも 1回で止める。

【通信方式は知らない】
送り口は Common/communicator.py の BaseCommunicator を差し替えて選ぶ。
実機は TcpCommunicator、シミュレータはブラウザの WebSocket。応答を待てる
環境では receive()、待てない環境（ブラウザ）では request() で送っておき、
届いた JSON を handle_message() へ流す。
"""

from __future__ import annotations

import logging
from typing import Any, Callable, Optional

logger = logging.getLogger(__name__)

#: 応答の status。1 = 処理成功、0 = 未処理、-1 = 失敗
STATUS_OK = 1


def as_int(value: Any) -> Optional[int]:
    """数字にできれば int。できなければ None（True/False は数字とみなさない）"""
    if value is None or isinstance(value, bool):
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def get_field(message: Any, *names: str) -> Any:
    """
    いくつかの名前のうち、最初に入っているものを返す。

    送り側が名前を変えた時期があるので、旧い名前でも読めるようにしてある
    （identifier / id、senario_kbn / scenarioType）。
    """
    if not isinstance(message, dict):
        return None
    for name in names:
        if message.get(name) is not None:
            return message.get(name)
    return None


def get_identifier(message: Any) -> Optional[int]:
    """識別値を取る。旧い名前（id）でも読める"""
    return as_int(get_field(message, "identifier", "id"))


def is_status_ok(message: Any) -> bool:
    """処理が成功しているか（status が 1）"""
    return as_int(get_field(message, "status")) == STATUS_OK


class RequestOnceReceiver:
    """
    合図が来たら 1回だけ要求し、使える応答を 1回だけ受け取る土台。

    年ごとの処理は、次の 3つを実装して作る。
        IDENTIFIER     … この要求の識別値
        build_request  … 送る JSON
        accepts        … 届いたものが使えるか
        to_result      … 届いたものを、走行体へ渡す形に直す
    """

    #: この要求の識別値。派生側で決める
    IDENTIFIER: Optional[int] = None

    def __init__(self, communicator=None):
        self.communicator = communicator
        self._applier: Optional[Callable] = None
        self._handlers: list = []
        #: 受け取って走行体へ渡したもの。まだなら None
        self.result: Optional[dict] = None
        #: 要求を送ったか（合図が何度も来ても送り直さないため）
        self.requested = False

    # --- 差し替え口 -------------------------------------------------------

    def set_communicator(self, communicator) -> None:
        """送り口を差す（Common/communicator.py の BaseCommunicator）"""
        self.communicator = communicator

    def set_applier(self, fn) -> None:
        """受け取ったものを走行体へ渡す口を差す"""
        self._applier = fn if callable(fn) else None

    def on_result(self, fn):
        """受け取ったときの通知を登録する。戻り値を呼ぶと解除できる"""
        if not callable(fn):
            return lambda: None
        self._handlers.append(fn)

        def off() -> None:
            if fn in self._handlers:
                self._handlers.remove(fn)

        return off

    # --- 年ごとに実装するもの ---------------------------------------------

    def build_request(self, **kwargs) -> dict:
        """送る JSON。派生側で組み立てる"""
        raise NotImplementedError

    def accepts(self, message: Any) -> bool:
        """届いたものが使えるか。派生側で判定する"""
        raise NotImplementedError

    def to_result(self, message: Any) -> dict:
        """届いたものを、走行体へ渡す形に直す。派生側で決める"""
        raise NotImplementedError

    def apply(self, result: dict) -> None:
        """走行体へ渡す。渡し方が特殊なら派生側で置き換える"""
        if self._applier is not None:
            self._applier(result)

    # --- 要求 -------------------------------------------------------------

    def notify(self, **kwargs) -> bool:
        """
        合図が来たことを知らせる。要求はこの 1回だけ送る。

        合図をどう検出するかは環境で違うので、ここではしない。
        :returns: 要求を送ったら True。すでに送り済み・送れなかったら False
        """
        if self.requested:
            return False
        return self.request(**kwargs)

    def request(self, **kwargs) -> bool:
        """
        要求を送る（応答は待たない）。

        応答は handle_message() へ流してもらう。ブラウザのように送信と受信が
        別々に起きる環境ではこちらを使う。
        :returns: 送れたら True
        """
        if self.communicator is None:
            logger.warning("送り口が差さっていないため、要求できません")
            return False
        try:
            self.communicator.send(self.build_request(**kwargs))
        except Exception:  # noqa: BLE001 — 送れなくても走行は続ける
            logger.exception("要求の送信に失敗しました")
            return False
        self.requested = True
        return True

    def receive(self, timeout: Optional[float] = None, **kwargs) -> Optional[dict]:
        """
        要求を送り、応答が返るまで待つ（実機のように待てる環境向け）。

        :returns: 受け取ったもの。使えなければ None
        """
        if self.communicator is None:
            logger.warning("送り口が差さっていないため、要求できません")
            return None
        try:
            answer = self.communicator.request(self.build_request(**kwargs), timeout=timeout)
        except Exception:  # noqa: BLE001
            logger.exception("応答の受け取りに失敗しました")
            return None
        self.requested = True
        return self.handle_message(answer)

    # --- 受け取り ---------------------------------------------------------

    def handle_message(self, message: Any) -> Optional[dict]:
        """
        届いた JSON を受け取る。使えるものなら走行体へ渡す。

        自分あて以外（別の識別値）が混ざって届いても何もしない。同じ 1本の
        通信路に、いろいろな識別値が流れてくるため。
        使えるものが 2通以上届いても、走行体へ渡すのは最初の 1回だけ。

        :returns: 受け取ったもの。使えなければ None
        """
        if self.IDENTIFIER is not None and get_identifier(message) != self.IDENTIFIER:
            return None

        if self.result is not None:
            logger.info("すでに受け取り済みのため、今回は使いません")
            return None

        if not self.accepts(message):
            logger.warning("届いた内容を使えません: %s", message)
            return None

        result = self.to_result(message)
        self.result = result

        # 走行体へ渡す。渡し先で失敗しても、受け取った事実は残す
        try:
            self.apply(result)
        except Exception:  # noqa: BLE001
            logger.exception("受け取ったものを走行体へ渡せませんでした")

        for fn in list(self._handlers):
            try:
                fn(result)
            except Exception:  # noqa: BLE001
                logger.exception("受け取り側で失敗しました")

        return result

    # --- 受け取り結果 -----------------------------------------------------

    def has_received(self) -> bool:
        """受け取り済みか"""
        return self.result is not None

    def reset(self) -> None:
        """走行やり直し用。受け取ったものと要求済みの印を捨てる"""
        self.result = None
        self.requested = False
