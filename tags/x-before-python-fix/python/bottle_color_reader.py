"""
キャリーボトルのラベル色を、走りながら読む（走行体側）。

【なぜ「走りながら」なのか】
色はボトルを抱えた瞬間に決まってほしいが、抱えた時点ではもう読めない。
ボトルがカメラに近すぎて、ラベルが画角の下へ外れるため。シミュレータの投影で
測ると、読めるのはボトルの 449mm〜249mm 手前だけだった。

  ボトルまで 449mm 〜 249mm … 読める（区間 200mm、通過に約 0.5秒）
             199mm 以下     … ラベルが画角の外
              90mm          … 抱えている位置

つまり「近づきながら読んで覚えておき、抱えたときにそれを使う」しかない。
覚える側は BottleColorMonitor.hold() にある。ここはその手前、
「区間を通り抜けるあいだに何枚読めるか」を受け持つ。

【1枚撮りでは間に合わない】
かつては v4l2-ctl の 1枚撮りだった。プロセス起動とデバイス open が毎回かかり、
シャッターが実際に切れる時刻もばらつくので、0.5秒の区間を狙って撮れない。

いまはデバイスを開きっぱなしにして連続で取る。1枚あたりのコストが下がり、
区間内で複数枚読める。判定は数枚あれば決まる。
**撮影（PC へ送る1枚）も同じカメラを借りる。**カメラは1台しか無く、
別々に開くとデバイスが使用中で失敗するため（Common/camera.py の
RobotCamera.shared() / RobotCamera.capture_still）。

【溜め込みに注意】
VideoCapture.read() は「次の1枚」を返すので、取り込みが撮影より遅いと古い順に
溜まったものが返る。走行体は動いているため、これは「さっき居た場所の映像」に
なる。grab_latest() で捨ててから取る。
"""

import logging
import os
import time
from typing import Any, Optional

logger = logging.getLogger(__name__)

#: 判定した瞬間の映像を残す先。
#:
#: 走行体には画面が無いため、色が読めなかったときに「カメラが撮れていないのか」
#: 「撮れているが色と判定できないのか」を切り分ける手立てが無かった。
#: 確定（hold）のたびに1枚残しておけば、走行後に目で確かめられる。
SNAPSHOT_DIR = "./sorot_spike/RoughSpot/PhotoImage/bottle"

#: 残す枚数の上限。走行を重ねても溢れないよう、古いものから消す
SNAPSHOT_KEEP = 40

#: 0 にすると保存しない（本番用）
SNAPSHOT_ENABLE = 1

#: 【デバッグ】走行中も定期的に1枚残す間隔[s]。0 で止める。**本番は 0 にすること**
#:
#: hold() のときの1枚だけでは、確定に至らなかった走行で何も残らない。
#: 「カメラが撮れていないのか」「撮れているが色と判定できないのか」を
#: 切り分けられず、走行体には画面が無いので走行中には確かめようがない。
#: これを 1.0 にしておくと、毎秒1枚ずつ、そのとき見えていた色と占有率を
#: ファイル名に入れて残る。読み取りは 30Hz 回っているので、毎回残すと
#: 書き込みが追いつかない。**間引くこと。**
DEBUG_SNAPSHOT_SEC = 1.0

#: 定期保存を残す枚数の上限。hold() のぶん（SNAPSHOT_KEEP）とは別に数える
DEBUG_SNAPSHOT_KEEP = 60

try:
    from camera import RobotCamera
    from bottle_color_monitor import BottleColorMonitor
    from bottle_color import BottleColor
except (ImportError, ValueError):
    try:
        from ..Common.camera import RobotCamera
        from ..Common.bottle_color_monitor import BottleColorMonitor
        from ..Common.bottle_color import BottleColor
    except (ImportError, ValueError):
        try:
            from Common.camera import RobotCamera
            from Common.bottle_color_monitor import BottleColorMonitor
            from Common.bottle_color import BottleColor
        except (ImportError, ValueError):
            from RoughSpot.Python.Common.camera import RobotCamera
            from RoughSpot.Python.Common.bottle_color_monitor import BottleColorMonitor
            from RoughSpot.Python.Common.bottle_color import BottleColor


try:
    from opencv import OpenCv
except ImportError:  # パッケージとして読み込まれた場合
    try:
        from .opencv import OpenCv
    except (ImportError, ValueError):
        from Common.opencv import OpenCv


class BottleColorReader:
    """カメラを開きっぱなしにして、ラベル色を読み続ける。"""

    @staticmethod
    def _frame_size(frame: Any):
        """映像の縦横を取る。numpy 配列でも、入れ子のリストでも読めるようにする"""
        shape = getattr(frame, "shape", None)
        if shape is not None and len(shape) >= 2:
            return int(shape[0]), int(shape[1])
        try:
            return len(frame), len(frame[0])
        except (TypeError, IndexError):
            return 0, 0

    def __init__(self, camera: Optional[RobotCamera] = None,
                 monitor: Optional[BottleColorMonitor] = None,
                 width: int = 640, height: int = 480):
        # **カメラは走行体に1台しか無い。** 撮影（handler の capture_still）と
        # 同じ1台を借りる。別々に開くとデバイスが使用中で失敗する。
        # 解像度は config の CAMERA_WIDTH / CAMERA_HEIGHT（先に開いたほうが決める）
        self.camera = camera or RobotCamera.shared()
        self.monitor = monitor or BottleColorMonitor()
        # 確定したときに残すため、最後に読んだ1枚を持っておく
        self._last_frame: Any = None
        self._frames_read = 0
        self.snapshot_dir = SNAPSHOT_DIR
        # 【デバッグ】定期保存を最後に書いた時刻。DEBUG_SNAPSHOT_SEC の間引きに使う
        self._last_debug_at = 0.0

    def set_frame_source(self, fn) -> None:
        """映像の取り込み口を差す（シミュレータ・テスト用）"""
        self.camera.set_frame_source(fn)

    def read(self, frame: Any = None) -> str:
        """
        映像を 1枚読んで色を判定する。

        読めた色は BottleColorMonitor が「最後に見えた色」として覚える。
        抱えたときに hold() がそれを採る。

        :param frame: 与えればその映像を読む（与えなければカメラから取る）
        :returns: いま見えている色。見えなければ BottleColor.NONE
        """
        if frame is None:
            frame = self.camera.grab_latest()
        if frame is None:
            return BottleColor.NONE

        height, width = self._frame_size(frame)
        if width <= 0 or height <= 0:
            return BottleColor.NONE

        self._last_frame = frame
        self._frames_read += 1
        color = self.monitor.read(frame, width, height)
        # 【デバッグ】確定に至らなくても様子が残るように、間引いて1枚置く
        self.save_debug_snapshot()
        return color

    def hold(self) -> str:
        """抱えた色を覚える（初回のみ）。抱えた周期に呼ぶ

        あわせて、判定した瞬間の映像を1枚残す。色が読めなかったときに
        カメラの問題か判定の問題かを切り分けるため。
        """
        color = self.monitor.hold()
        self.save_snapshot(color)
        return color

    def save_snapshot(self, color: str) -> Optional[str]:
        """最後に読んだ1枚を、判定した色を名前に入れて残す。

        **失敗しても走行は止めない。** 例外は握りつぶして警告だけ出す。

        :returns: 書けたファイルのパス。書けなければ None
        """
        if not SNAPSHOT_ENABLE:
            return None

        if self._last_frame is None:
            # ここが出たら、色ではなくカメラを疑う
            logger.warning(
                "映像が1枚も取れていないため画像を残せません（読み取り回数 %d）。"
                "カメラが開けているかを確かめてください", self._frames_read)
            return None

        try:
            cv2 = OpenCv.cv2()
            if cv2 is None:
                logger.warning("OpenCV が無いため画像を残せません")
                return None

            os.makedirs(self.snapshot_dir, exist_ok=True)

            image = self._last_frame
            marked = self._draw_roi(cv2, image)

            stamp = time.strftime("%Y%m%d_%H%M%S")
            msec = int((time.time() % 1) * 1000)
            name = "bottle_%s_%03d_%s.jpg" % (stamp, msec, color or "none")
            path = os.path.join(self.snapshot_dir, name)

            if not cv2.imwrite(path, marked):
                logger.warning("画像を書けませんでした: %s", path)
                return None

            logger.info("判定時の画像を残しました: %s", path)
            self._prune()
            return path
        except Exception as e:  # noqa: BLE001 — 残せなくても走行は続ける
            logger.warning("画像を残せませんでした: %s", e)
            return None

    def _draw_roi(self, cv2, image: Any) -> Any:
        """判定に使った範囲（ROI）を枠で描く。どこを見ているかが分かる"""
        try:
            height, width = self._frame_size(image)
            roi = self.monitor.classifier.default_options['roi']
            x0 = int(width * roi['x'])
            y0 = int(height * roi['y'])
            x1 = int(width * (roi['x'] + roi['width']))
            y1 = int(height * (roi['y'] + roi['height']))
            marked = image.copy()
            cv2.rectangle(marked, (x0, y0), (x1, y1), (0, 255, 0), 2)
            return marked
        except Exception:  # noqa: BLE001 — 枠が描けなくても元の映像は残す
            return image

    def _prune(self, prefix: str = "bottle_", keep: int = SNAPSHOT_KEEP) -> None:
        """古い画像から消す。走行を重ねてもディスクを埋めない

        確定時（bottle_）と定期保存（debug_）は別々に数える。定期保存は
        枚数が多いので、混ぜて数えると確定時の1枚が押し出されてしまう。
        """
        try:
            files = sorted(
                f for f in os.listdir(self.snapshot_dir)
                if f.startswith(prefix) and f.endswith(".jpg"))
            for f in files[:-keep]:
                os.remove(os.path.join(self.snapshot_dir, f))
        except Exception:  # noqa: BLE001 — 消せなくても走行は続ける
            pass

    def save_debug_snapshot(self) -> Optional[str]:
        """【デバッグ】いま見えているものを1枚残す。DEBUG_SNAPSHOT_SEC ごと。

        **確定（hold）に至らなくても残る**のが要点。走行が途中で終わっても、
        カメラが撮れていたのか、撮れていて色にならなかったのかが後から分かる。

        ファイル名に判定した色と占有率を入れる。占有率はしきい値（既定 0.06）と
        比べるためのもので、0.05 のように惜しい値なら「見えてはいる」と分かる。

        **失敗しても走行は止めない。**
        """
        if not DEBUG_SNAPSHOT_SEC or self._last_frame is None:
            return None
        now = time.time()
        if (now - self._last_debug_at) < DEBUG_SNAPSHOT_SEC:
            return None
        self._last_debug_at = now

        try:
            cv2 = OpenCv.cv2()
            if cv2 is None:
                return None
            os.makedirs(self.snapshot_dir, exist_ok=True)
            info = self.monitor.get_color_info()
            marked = self._draw_roi(cv2, self._last_frame)
            name = "debug_%s_%03d_%s_cov%03d.jpg" % (
                time.strftime("%Y%m%d_%H%M%S"), int((now % 1) * 1000),
                info.get("color") or "none", int(info.get("coverage", 0.0) * 1000))
            path = os.path.join(self.snapshot_dir, name)
            if not cv2.imwrite(path, marked):
                return None
            self._prune("debug_", DEBUG_SNAPSHOT_KEEP)
            return path
        except Exception as e:  # noqa: BLE001 — 残せなくても走行は続ける
            logger.warning("デバッグ画像を残せませんでした: %s", e)
            return None

    def release(self) -> str:
        """離した。次に抱えるまで色を空にする"""
        return self.monitor.release()

    def get_held_color(self) -> str:
        return self.monitor.get_held_color()

    def get_color_info(self) -> dict:
        return self.monitor.get_color_info()

    def reset(self) -> None:
        self.monitor.reset()

    def close(self) -> None:
        self.camera.close()


