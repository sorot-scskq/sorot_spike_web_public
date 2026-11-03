"""
ヒントカードが読めたら、その区間を終わらせる（2026・ETラリー）。

止まってカードを読む行は、読み取りが別スレッドで走るため、これまで時間
（`SCJFN 1` の `SCT`）でしか抜けられなかった。`SCT` は読めなかったときの粘り
（3秒 ＋ 1.5秒×5回）に合わせて 11秒あるので、**一発で読めても11秒止まっていた。**

読めたかを知っているのは Python 側なので、**区間の終わりも Python が言う。**
リモートコントロール走行の応答（`Common/remote_control.py`）をそのまま使う。
C++ には手を入れない。

    "SwitchInfo": {"SCJFN": 2049, "SCV": 1, "SCT": 11000}
                            │       │        └ 読めなかったときの上限（縮めないこと）
                            │       └ 読めている必要のある枚数
                            └ 2048（リモートの応答）＋ 1（時間）

`SCJFN 2048`（`SW_REMOTE_CONTROL`）は**走行方法が FUNCNO 18 でなくても使える**
（`SwitchMethod/RemoteControlASM.h`）。切替判定そのものが「リモートの区間にいる」
と C++ へ伝えるので、この行に入ったときだけ 10ms の往復が立ち上がる。指令の
`forward` / `turn` を読むのは `RunMethod/RemoteControl.cpp`（FUNCNO 18）だけなので、
**FUNCNO 5 / 2 の行では走りに影響しない。効くのは `Status` だけ。**

【枚数で数える理由（どちらのカードかで見ない）】
読めた文字列を `qr1` / `qr2` のどちらとして覚えるかは**中身で決まる**
（`handler.PcExchange.qr_key`）。シナリオの読む順とは限らないので、
「1枚目が読めたか」を `qr1` で見ると取りこぼす。1枚目の行では1枚、
2枚目の行では2枚と数えるほうが、入れ替わっても正しい。

【リモートで走らせる区間には手を出さない】
制御関数の差し込み口は走行体に1つしかない（`set_controller`）。あとから
「経路を作りながら走らせる」（`Docs/remote_control.md` 4章）を入れるときは、
FUNCNO 18 の区間をそちらが受け持つ。ここでは `None` を返して何も言わない。
"""

import logging

from config import DECODE_ERROR_VAL
from remote_control import Command, Status
from state import ResultStore

logger = logging.getLogger(__name__)


class HintCardAdvance:
    """ヒントカードが要る枚数だけ読めたら、区間を終わらせる制御関数。"""

    #: リモートコントロール走行の FUNCNO。この区間は受け持たない
    REMOTE_CONTROL_FUNCNO = 18

    #: **SCV を書いていない行は受け持たない。**
    #:
    #: 以前は書き忘れへの保険として 1 を既定にしていたが、それが穴になっていた。
    #: この制御関数は sno/cno を見ないので、2048 ビットが立った行なら何でも拾う。
    #: カードを2枚読み終えたあとに 2048 の行を足すと、**その区間が一瞬で終わる**。
    #:
    #: いまの custom.json では 20/2（SCV 1）と 20/6（SCV 2）だけが該当し、
    #: 40/2 は FUNCNO 18 なので除外される。PC が出す経路にも 2048 は無い。
    #: つまり現状は誤爆しないが、行を足せば壊れる。受け持つ行は SCV で明示させる。
    #:
    #: 書き忘れは黙って拾わず、ログに出して SCT で抜けさせる（以前の動き）。
    DEFAULT_REQUIRED = 0

    def __init__(self):
        #: 同じ区間で何度もログを出さないための目印（SNO, CNO）
        self._logged = None

    # --- 制御関数 -------------------------------------------------------------

    def control(self, obs):
        """
        1周期ぶんの応答を作る。`remote_control.set_controller()` へ差す。

        **`forward` / `turn` は返さない。** 走行方法は C++ 側のまま（止まっている）
        なので、伝えるのは「この区間を終えてよいか」だけ。

        **受け持つのは `SCV` で枚数を書いた行だけ。** この関数は sno/cno を見ないので、
        そうしないと 2048 が立った行を見境なく終わらせてしまう。

        :returns: Command、または受け持たない区間なら None
        """
        if obs.run_info.get("FUNCNO") == self.REMOTE_CONTROL_FUNCNO:
            return None

        required = obs.switch_info.get("SCV") or self.DEFAULT_REQUIRED
        if required <= 0:
            # 受け持つ行ではない。SCV を書き忘れているならここで気づけるように出す
            here = (obs.sno, obs.cno)
            if self._logged != here:
                self._logged = here
                logger.warning("%d/%d は SCJFN に 2048 が立っているのに SCV が無い。"
                               "受け持たない（書き忘れなら SCV に枚数を書く）",
                               obs.sno, obs.cno)
            return None
        read = self.read_count()
        if read < required:
            self._logged = None
            return Command(status=Status.NG)

        here = (obs.sno, obs.cno)
        if self._logged != here:
            self._logged = here
            logger.info("ヒントカードが%d枚読めたので %d/%d を終えます（要る枚数 %d）",
                        read, obs.sno, obs.cno, required)
        return Command(status=Status.OK)

    # --- 読めた枚数 -----------------------------------------------------------

    @staticmethod
    def read_count() -> int:
        """読めているヒントカードの枚数。空文字と DECODE_ERROR は数えない"""
        store = ResultStore.get_instance()
        return sum(1 for key in ("qr1", "qr2")
                   if HintCardAdvance.is_read(store.get(key, "")))

    @staticmethod
    def is_read(text) -> bool:
        """その1枚が読めているか（`ResultRead::isHintCardRead` と同じ見方）"""
        return bool(text) and text != DECODE_ERROR_VAL
