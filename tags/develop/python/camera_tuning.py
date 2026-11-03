"""
カメラの露出・ゲインを手で決める（`v4l2-ctl` 越し）。

【焦点は触れない。走行体のカメラは固定フォーカス】
**`focus_absolute` / `focus_automatic_continuous` は UVC の記述子に出ているが、
このカメラには焦点機構が無い。書いても光学的には何も起きない。**
実機で 0〜240 を 16刻みで振っても、ぼけ具合（Laplacian 分散）は 139 のまま
平らだった（2026-09-21）。

**「自動焦点が遠側に張り付いているからぼける」という見立ては誤り。**
至近で撮った1枚がぼけるのは、**カメラの最短撮影距離より近づいているから**で、
設定では直らない。直し方は「近づかない」＝遠くのコードを読むこと
（`Docs/position_correction_design.md` 5.1）。

`manual(focus=...)` は残してあるが、**このカメラでは効果が無い**。別のカメラに
替えたときのために口だけ開けてある。

【効くのは露出とゲイン】
初期状態は**自動露出**。実機で見た値（`v4l2-ctl --list-ctrls`）。

    auto_exposure              = 3       ← Aperture Priority（自動）
    exposure_time_absolute     = 332〜600（flags=inactive）。**33〜60ms**
    gain                       = 32      ← 下限 16 に近い

露出 33〜60ms は、走りながらでは確実にぶれる（`FWD 30` で 1フレームに
数モジュールぶん流れる）。**走行中に読みたいならここを短くする。**

**`exposure_time_absolute` は自動を切るまで書けない**（`flags=inactive`）。
順序を守ること。このクラスがその順序を持っている。

【OpenCV の CAP_PROP ではなく v4l2-ctl を使う理由】
`cv2.CAP_PROP_FOCUS` などは backend ごとに単位が違い、UVC の生の値と一致しない。
`v4l2-ctl` なら `--list-ctrls` に出る値がそのまま書ける＝**実機で見た数字を
そのまま渡せる**。設定は開きっぱなしのまま効くので、取り込みを止めなくてよい。

    v4l2-ctl -d /dev/video0 --list-ctrls-menus     # いまの値と範囲

【誰も呼ばなければ何も変わらない】
既定は「触らない」。`Tools/read_floor_code.py` で良い値を決めてから、
本番へ入れるかどうかを決めること。
"""

from __future__ import annotations

import logging
import subprocess
import time
from typing import Dict, Optional

logger = logging.getLogger(__name__)


#: 実機で確かめた各つまみの範囲（2026-09-20、Full HD webcam）。
#: 範囲の外を書くとドライバが黙って丸めるので、こちらで弾く。
LIMITS = {
    "focus_absolute": (0, 255),
    "exposure_time_absolute": (3, 2047),
    "gain": (16, 255),
    "brightness": (0, 255),
    "contrast": (0, 95),
    "sharpness": (0, 100),
}

#: `exposure_time_absolute` を書く回数。
#:
#: **これは保険であって、直しではない。** 「掃引では効くのに単発では効かない」の
#: 原因は回数ではなく順序だった。**撮影が始まる前に書いた設定は捨てられる**
#: （OpenCV は最初に読まれるまで `VIDIOC_STREAMON` しない）。呼ぶ側が
#: 1枚読んでから書くこと（`camera.py` の `open()`、`Tools/read_floor_code.py`）。
#:
#: 2回書くのは、書いた直後にカメラ側で取りこぼされる場合への備えとして残して
#: ある。現地で通した設定（2026-09-21、Lコース）がこの形だったので変えていない。
EXPOSURE_WRITES = 2

#: 2回目を書くまでの待ち[s]
EXPOSURE_WRITE_INTERVAL = 0.2

#: `exposure_time_absolute` の単位[ms]。UVC は 100us 刻み。
#: 600 = 60ms、50 = 5ms、10 = 1ms。
EXPOSURE_STEP_MS = 0.1


class CameraTuning:
    """
    1台のカメラのつまみを読み書きする。

    `v4l2-ctl` が無い母艦では、読み書きとも黙って何もせずに False を返す。
    """

    def __init__(self, device: str = "/dev/video0") -> None:
        self.device = device

    # --- 読む -----------------------------------------------------------------

    def read(self) -> Dict[str, int]:
        """いまの値を名前→数値で返す。読めなければ空"""
        return self._list("value=")

    def defaults(self) -> Dict[str, int]:
        """カメラが言う既定値を名前→数値で返す。`auto()` が戻す先"""
        return self._list("default=")

    def _list(self, key: str) -> Dict[str, int]:
        """`--list-ctrls` の各行から `key` の数値を拾う"""
        out = self._run(["--list-ctrls"])
        if out is None:
            return {}
        values = {}
        for line in out.splitlines():
            if key not in line or ":" not in line:
                continue
            name = line.split(":")[0].split()[0]
            for token in line.split():
                if token.startswith(key):
                    try:
                        values[name] = int(token[len(key):])
                    except ValueError:
                        pass
        return values

    # --- 書く -----------------------------------------------------------------

    def manual(self, focus: Optional[int] = None,
               exposure: Optional[int] = None,
               gain: Optional[int] = None) -> bool:
        """
        自動を切ってから、渡された値だけを書く。

        **自動を切る順序が要る。** 自動が入っているあいだ `focus_absolute` と
        `exposure_time_absolute` は `inactive` で、書いても効かない。

        **呼ぶ前に1枚読んでおくこと。** 撮影が始まる前に書いた設定は捨てられる
        （`EXPOSURE_WRITES` の説明）。ここでは順序を保証できない。

        :param focus: 0〜255。**走行体のカメラは固定フォーカスなので効果が無い**
        :param exposure: 100us 刻み（600 = 60ms）。走行中は短くする
        :param gain: 16〜255。露出を短くしたぶんここで持ち上げる
        :returns: 書けたら True
        """
        ok = True
        if focus is not None:
            ok &= self.set("focus_automatic_continuous", 0)
            ok &= self.set("focus_absolute", focus)
        if exposure is not None:
            ok &= self.set("auto_exposure", 1)          # 1 = Manual Mode
            for n in range(EXPOSURE_WRITES):
                if n:
                    time.sleep(EXPOSURE_WRITE_INTERVAL)
                ok &= self.set("exposure_time_absolute", exposure)
        if gain is not None:
            ok &= self.set("gain", gain)
        return ok

    #: `auto()` で既定値へ戻すつまみ。**自動に任せられないものはここで戻す**
    #:
    #: `gain` には自動が無いので、上げたままにすると自動露出へ戻しても明るすぎる。
    #: 実機で `--gain 200` のまま `--auto` を掛け、そのあと何を撮っても真っ白
    #: （ぼけ具合きっかり 0）になった（2026-09-21）。
    RESTORE = ("gain", "brightness", "contrast", "sharpness", "gamma",
               "backlight_compensation")

    def auto(self) -> bool:
        """
        触ったつまみを元へ戻す（点検のあと片付けるため）。

        自動焦点・自動露出を入れ直すだけでは足りない。**自動が無いつまみ
        （ゲイン等）はカメラが言う既定値へ書き戻す。**
        """
        ok = self.set("focus_automatic_continuous", 1)
        ok &= self.set("auto_exposure", 3)              # 3 = Aperture Priority
        defaults = self.defaults()
        for name in self.RESTORE:
            if name in defaults:
                ok &= self.set(name, defaults[name])
        return ok

    def set(self, name: str, value: int) -> bool:
        """つまみを1つ書く。範囲の外なら書かずに False"""
        low, high = LIMITS.get(name, (None, None))
        if low is not None and not (low <= value <= high):
            logger.warning("%s は %d〜%d の範囲外です: %d", name, low, high, value)
            return False
        return self._run(["--set-ctrl", "%s=%d" % (name, value)]) is not None

    # --- v4l2-ctl -------------------------------------------------------------

    def _run(self, args) -> Optional[str]:
        """`v4l2-ctl` を呼ぶ。無い・失敗なら None（走行は止めない）"""
        try:
            done = subprocess.run(["v4l2-ctl", "-d", self.device] + args,
                                  capture_output=True, text=True, timeout=5)
        except (OSError, subprocess.SubprocessError) as e:
            logger.warning("v4l2-ctl を呼べません（%s）: %s", self.device, e)
            return None
        if done.returncode != 0:
            logger.warning("v4l2-ctl が失敗しました: %s", done.stderr.strip())
            return None
        return done.stdout

    # --- 表示 -----------------------------------------------------------------

    @staticmethod
    def exposure_ms(value: int) -> float:
        """`exposure_time_absolute` をミリ秒にする（人が読むため）"""
        return value * EXPOSURE_STEP_MS
