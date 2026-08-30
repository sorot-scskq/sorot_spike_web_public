"""
ヒントカード（二次元コード）の読み取りとゲート位置解析。

ET ラリーのゲート配置は、コース上に立ててある 2枚のヒントカードを読まないと
分からない（規約 7.3.1、表 7-2）。カードは 5cm 四方の二次元コードで、
走行体の正面を向いて台に立っている。
    ヒント1 … 赤ゲートの位置              例 "25,35"
    ヒント2 … 青ゲートと黄ゲートの位置    例 "53,54/12,22"

【構成（Common モジュールの利用）】
    RobotCamera   … カメラ映像の取得
    QrCodeDecoder … 二次元コードの検出・デコード
    MessageSender … 上位・サーバーへの送信インターフェース
    HintCardReader … 上記を統合してカードの読み取りと通知を管理

【1枚目と2枚目の見分け方】
カードには番号が書かれていないので、中身で見分ける。位置の組が
1つならヒント1（赤だけ）、2つならヒント2（青と黄）。規約の表 7-2 どおり。
実際のカードが番号を持つなら "1:25,35" のように頭に付ければそちらを優先する。
"""

from __future__ import annotations

import json
import logging
import re
from typing import Any, Callable, Iterable, Optional

logger = logging.getLogger(__name__)

class HintCard:
    """カードの種類"""

    CARD1 = "card1"
    CARD2 = "card2"


# --------------------------------------------------------------------------
# カードの中身の解釈
# --------------------------------------------------------------------------

_POSITION_DASH = re.compile(r"^G?(\d)[-_](\d)$")
_POSITION_PLAIN = re.compile(r"^G?(\d)(\d)$")
_CARD_NUMBER = re.compile(r"^\s*([12])\s*:\s*(.*)$", re.S)
_CARD_NUMBER_HEAD = re.compile(r"^\s*([12])\s*:")


def _parse_position_key(text: str) -> Optional[dict]:
    """位置の呼び名（"25" や "G2-5"）を {'x': .., 'y': ..} にする"""
    if not isinstance(text, str):
        return None
    t = text.strip().upper()
    m = _POSITION_DASH.match(t) or _POSITION_PLAIN.match(t)
    if not m:
        return None
    return {"x": int(m.group(1)), "y": int(m.group(2))}


def strip_card_number(text: Optional[str]) -> str:
    """頭に付いた "1:" / "2:" を外した本文"""
    m = _CARD_NUMBER.match(text or "")
    return m.group(2) if m else (text or "")


def _card_number_prefix(text: Optional[str]) -> Optional[str]:
    """頭に付いた "1:" / "2:" から読み取れるカード番号"""
    m = _CARD_NUMBER_HEAD.match(text or "")
    if not m:
        return None
    return HintCard.CARD1 if m.group(1) == "1" else HintCard.CARD2


def parse_hint_card_text(text: Optional[str]) -> list:
    """
    ヒントカードの中身を、ゲートの位置の組に分解する。

    "25,35/53,54" のように、ゲート 1つぶんが "始点,終点"、ゲート同士は "/" 区切り。
    """
    if not isinstance(text, str):
        return []
    body = strip_card_number(text).strip()
    if body == "":
        return []

    pairs = []
    for chunk in re.split(r"[/\n]", body):
        parts = [s.strip() for s in chunk.split(",")]
        parts = [s for s in parts if s != ""]
        if len(parts) != 2:
            continue
        a = _parse_position_key(parts[0])
        b = _parse_position_key(parts[1])
        if a and b:
            pairs.append({"a": a, "b": b})
    return pairs


def identify_hint_card(text: Optional[str]) -> Optional[str]:
    """
    読んだ文字列が 1枚目か 2枚目かを見分ける。

    中身の位置の組の数で決める（表 7-2）。番号が頭に付いていればそちらを優先する。
    """
    prefixed = _card_number_prefix(text)
    if prefixed:
        return prefixed

    pairs = parse_hint_card_text(text)
    if len(pairs) == 1:
        return HintCard.CARD1
    if len(pairs) >= 2:
        return HintCard.CARD2
    return None


# --------------------------------------------------------------------------
# 共通カメラ・二次元コードデコーダ・送信機能の利用
# --------------------------------------------------------------------------

try:
    from camera import RobotCamera
    from qr_decoder import QrCodeDecoder, find_card_panels
    from sender import CallbackSender as BaseCallbackSender, MessageSender, WebSocketSender as BaseWebSocketSender
except (ImportError, ValueError):
    try:
        from ..Common.camera import RobotCamera
        from ..Common.qr_decoder import QrCodeDecoder, find_card_panels
        from ..Common.sender import CallbackSender as BaseCallbackSender, MessageSender, WebSocketSender as BaseWebSocketSender
    except (ImportError, ValueError):
        try:
            from Common.camera import RobotCamera
            from Common.qr_decoder import QrCodeDecoder, find_card_panels
            from Common.sender import CallbackSender as BaseCallbackSender, MessageSender, WebSocketSender as BaseWebSocketSender
        except (ImportError, ValueError):
            from RoughSpot.Python.Common.camera import RobotCamera
            from RoughSpot.Python.Common.qr_decoder import QrCodeDecoder, find_card_panels
            from RoughSpot.Python.Common.sender import CallbackSender as BaseCallbackSender, MessageSender, WebSocketSender as BaseWebSocketSender

# 後方互換性エイリアス
HintCardCamera = RobotCamera
HintCardSender = MessageSender


# --------------------------------------------------------------------------
# 送信（カード用アダプタ）
# --------------------------------------------------------------------------


def card_to_payload(card: dict) -> dict:
    """送信する形。JS 側・サーバ側と同じ並びにしておく"""
    return {
        "type": "hintCard",
        "card": card["card"],
        "text": card["text"],
        "gates": card["gates"],
    }


class WebSocketSender(BaseWebSocketSender):
    """WebSocket でカード情報を上位へ送るクラス。"""

    def send(self, card: dict) -> None:
        super().send(card_to_payload(card))


class CallbackSender(BaseCallbackSender):
    """関数を 1つ差すだけのカード用送信口。"""

    def send(self, card: dict) -> None:
        super().send(card_to_payload(card))


# --------------------------------------------------------------------------
# 読み取り本体
# --------------------------------------------------------------------------


class HintCardReader:
    """
    ヒントカードを読み、読めた内容を送る。
    """

    def __init__(
        self,
        camera: Optional[HintCardCamera] = None,
        decoder: Optional[QrCodeDecoder] = None,
        sender: Optional[HintCardSender] = None,
    ):
        self.camera = camera or HintCardCamera()
        self.decoder = decoder or QrCodeDecoder()
        self.sender = sender
        self._panel_resolver: Optional[Callable[[list], Optional[str]]] = None
        self._handlers = {HintCard.CARD1: [], HintCard.CARD2: []}
        self.cards = {HintCard.CARD1: None, HintCard.CARD2: None}

    # --- 差し替え口 -------------------------------------------------------

    def set_sender(self, sender) -> None:
        """送信口を設定する。関数を渡してもよい。"""
        if sender is None:
            self.sender = None
        elif isinstance(sender, HintCardSender):
            self.sender = sender
        elif callable(sender):
            self.sender = CallbackSender(sender)
        else:
            raise TypeError("sender は HintCardSender か関数")

    def set_panel_resolver(self, fn) -> None:
        """
        検出されたカード領域から文字列を解決するリゾルバを設定する。

        :param fn: (panels) -> str|None。panels は find_card_panels の戻り値
        """
        self._panel_resolver = fn if callable(fn) else None

    def on_card(self, card_key: str, fn: Callable[[dict], None]) -> Callable[[], None]:
        """
        カードごとの受け取り口を登録する。戻り値を呼ぶと解除できる。

        ヒント1（赤）とヒント2（青・黄）で処理が違うので、ここで呼び分ける。
        """
        handlers = self._handlers.get(card_key)
        if handlers is None or not callable(fn):
            return lambda: None
        handlers.append(fn)

        def off() -> None:
            if fn in handlers:
                handlers.remove(fn)

        return off

    # --- 読み取り ---------------------------------------------------------

    def read(self, frame=None) -> Optional[dict]:
        """
        映像を 1枚読む。

        読めた内容が前と同じなら何もしない。同じカードを見続けているあいだ
        何度も送ってしまわないようにするため。

        :param frame: 与えれば、その映像を読む（与えなければカメラから取る）
        :returns: 新しく読めたカード。無ければ None
        """
        if frame is None:
            frame = self.camera.grab()
        if frame is None:
            return None

        text = self.decoder.decode(frame)
        if text is None and self._panel_resolver is not None:
            panels = find_card_panels(frame)
            if panels:
                try:
                    text = self._panel_resolver(panels)
                except Exception:  # noqa: BLE001
                    logger.exception("カードの中身を補えませんでした")
                    text = None

        return self.accept(text)

    def accept(self, text: Optional[str]) -> Optional[dict]:
        """
        読めた文字列を受け取る。読み取り方に依らない部分。

        :returns: 新しく読めたカード。前と同じ・読めない場合は None
        """
        if not isinstance(text, str) or text.strip() == "":
            return None

        card_key = identify_hint_card(text)
        if card_key is None:
            return None
        known = self.cards.get(card_key)
        if known is not None and known["text"] == text:
            return None

        card = {"card": card_key, "text": text, "gates": parse_hint_card_text(text)}
        self.cards[card_key] = card

        # 送る。送り先で失敗しても、読めた事実は残す
        if self.sender is not None:
            try:
                self.sender.send(card)
            except Exception:  # noqa: BLE001
                logger.exception("読んだ内容の送信に失敗しました")
        # カードごとの受け取り口へ渡す（ヒント1／ヒント2 の呼び分け）
        for fn in list(self._handlers[card_key]):
            try:
                fn(card)
            except Exception:  # noqa: BLE001
                logger.exception("受け取り側で失敗しました")

        return card

    # --- 読み取り結果 -----------------------------------------------------

    def get_card(self, card_key: str) -> Optional[dict]:
        """そのカードの読み取り結果。まだ読んでいなければ None"""
        card = self.cards.get(card_key)
        if card is None:
            return None
        return {
            "card": card["card"],
            "text": card["text"],
            "gates": [dict(g) for g in card["gates"]],
        }

    def has_read(self, card_key: str) -> bool:
        return self.cards.get(card_key) is not None

    def has_read_all(self) -> bool:
        """2枚とも読み終えているか。ゲート配置がすべて分かった状態"""
        return self.has_read(HintCard.CARD1) and self.has_read(HintCard.CARD2)

    def get_placement_string(self) -> str:
        """
        読み取り済みの 2枚から、ゲート配置の文字列を組み立てる。

        表 7-2 の並び（赤 / 青 / 黄）でつなぐので、そのまま配置の指定に使える。
        まだ読めていない部分は含まない。
        """
        texts = []
        for key in (HintCard.CARD1, HintCard.CARD2):
            card = self.cards.get(key)
            body = strip_card_number(card["text"]).strip() if card else ""
            if body != "":
                texts.append(body)
        return "/".join(texts)

    def reset(self) -> None:
        """読み取り結果を捨てる。走行をやり直すときに呼ぶ"""
        self.cards = {HintCard.CARD1: None, HintCard.CARD2: None}
