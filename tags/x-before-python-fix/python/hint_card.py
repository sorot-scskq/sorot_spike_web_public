"""
ヒントカードの中身を見分ける（2026・ETラリー）。

ET ラリーのゲート配置は、コース上に立ててある 2枚のヒントカードを読まないと
分からない（規約 7.3.1、表 7-2）。カードは 5cm 四方の二次元コードで、
走行体の正面を向いて台に立っている。

    ヒント1 … 赤ゲートの位置              平文。例 "25,35"
    ヒント2 … 青ゲートと黄ゲートの位置    暗号文。例 "NKVD6nnS28waG2lbRQlD8A=="

ヒント2 は暗号化されて掲示される（難所組立図 6.2。パスフレーズは4文字）。
走行体は中身を解釈できないので、読めた文字列をそのまま PC 側へ渡し、
PC が復号してゲート配置を組み立てる。

【なぜ「どちらのカードか」を中身で見分けるのか】
**カードには番号が書かれていない。** シナリオは `hint_card1.py` → `hint_card2.py` の
順に読むが、それは段取りでしかなく、読み損ねや読み直しがあると入れ替わる。
ゲート配置が入れ替わると、通過順（赤→青→黄）がそのまま崩れる。

中身で見分ければ、どちらを先に読んでも正しい側へ入る。

    位置の組が1つ    → ヒント1（赤だけ）
    位置の組が2つ    → ヒント2（青と黄。暗号化しない相手と繋いだとき）
    Base64 の塊      → ヒント2（本番）
    頭に "1:" / "2:" → その番号（カードが番号を持つ場合はこちらを優先）

sorot_spike_web の 2026/hint_card_reader.py から、中身の解釈だけを移した。
カメラから読むところは `2026/hint_card_capture.py`、PC への送信は
`Common/handler.py`（`handle_decoded_qr`）にある。
"""

from __future__ import annotations

import re
from typing import Optional


class HintCard:
    """カードの種類"""

    CARD1 = "card1"
    CARD2 = "card2"


class HintCardText:
    """
    ヒントカードから読めた文字列の読み方。

    **状態を持たない。** 生成せずにクラスのまま呼ぶ（`HintCardText.identify(text)`）。
    文字列の形だけを見る処理をここへ集めてあるので、カメラも通信も要らず、
    母艦でそのまま試せる（`tests/test_hint_card.py`）。
    """

    _POSITION_DASH = re.compile(r"^G?(\d)[-_](\d)$")
    _POSITION_PLAIN = re.compile(r"^G?(\d)(\d)$")
    _CARD_NUMBER = re.compile(r"^\s*([12])\s*:\s*(.*)$", re.S)
    _CARD_NUMBER_HEAD = re.compile(r"^\s*([12])\s*:")
    #: 暗号化されたヒントカード2 の見た目（Base64）。1ブロックで 24文字になる
    _ENCRYPTED_LIKE = re.compile(r"^[A-Za-z0-9+/]{16,}={0,2}$")

    # --- 見分け -------------------------------------------------------------

    @classmethod
    def identify(cls, text: Optional[str]) -> Optional[str]:
        """
        読んだ文字列が 1枚目か 2枚目かを見分ける。

        ヒント1 は平文（位置の組が 1つ）、ヒント2 は暗号文で届く（規約 表 7-2）。
        番号が頭に付いていればそちらを優先する。

            "25,35"                     位置の組が 1つ  → ヒント1
            "NKVD6nnS28waG2lbRQlD8A=="  暗号文          → ヒント2
            "53,54/12,22"               位置の組が 2つ  → ヒント2（平文のとき）

        3行目は本番では起きない（ヒント2 は暗号化されている）が、暗号化しない
        相手（試験用のサーバなど）と繋いだときのために残してある。

        :returns: `HintCard.CARD1` / `HintCard.CARD2`。分からなければ None
        """
        prefixed = cls._card_number_prefix(text)
        if prefixed:
            return prefixed

        pairs = cls.parse(text)
        if len(pairs) == 1:
            return HintCard.CARD1
        if len(pairs) >= 2:
            return HintCard.CARD2
        if cls.is_encrypted(text):
            return HintCard.CARD2
        return None

    @classmethod
    def is_encrypted(cls, text: Optional[str]) -> bool:
        """
        暗号化されたヒントカード2 の見た目か（Base64 の塊）。

        本番のヒントカード2 は暗号化されて掲示される。走行体は中身を解釈できず、
        そのまま PC 側へ渡して復号してもらう。読み取りの失敗（短い断片や記号）を
        カードとして受け取ってしまわないよう、Base64 の形をしていることは見る。
        """
        return bool(text) and bool(cls._ENCRYPTED_LIKE.match(text.strip()))

    # --- 中身の取り出し -----------------------------------------------------

    @classmethod
    def parse(cls, text: Optional[str]) -> list:
        """
        ヒントカードの中身を、ゲートの位置の組に分解する。

        "25,35/53,54" のように、ゲート 1つぶんが "始点,終点"、ゲート同士は "/" 区切り。
        """
        if not isinstance(text, str):
            return []
        body = cls.strip_card_number(text).strip()
        if body == "":
            return []

        pairs = []
        for chunk in re.split(r"[/\n]", body):
            parts = [s.strip() for s in chunk.split(",")]
            parts = [s for s in parts if s != ""]
            if len(parts) != 2:
                continue
            a = cls._position(parts[0])
            b = cls._position(parts[1])
            if a and b:
                pairs.append({"a": a, "b": b})
        return pairs

    @classmethod
    def strip_card_number(cls, text: Optional[str]) -> str:
        """頭に付いた "1:" / "2:" を外した本文"""
        m = cls._CARD_NUMBER.match(text or "")
        return m.group(2) if m else (text or "")

    # --- 部品 ---------------------------------------------------------------

    @classmethod
    def _position(cls, text: str) -> Optional[dict]:
        """位置の呼び名（"25" や "G2-5"）を {'x': .., 'y': ..} にする"""
        if not isinstance(text, str):
            return None
        t = text.strip().upper()
        m = cls._POSITION_DASH.match(t) or cls._POSITION_PLAIN.match(t)
        if not m:
            return None
        return {"x": int(m.group(1)), "y": int(m.group(2))}

    @classmethod
    def _card_number_prefix(cls, text: Optional[str]) -> Optional[str]:
        """頭に付いた "1:" / "2:" から読み取れるカード番号"""
        m = cls._CARD_NUMBER_HEAD.match(text or "")
        if not m:
            return None
        return HintCard.CARD1 if m.group(1) == "1" else HintCard.CARD2
