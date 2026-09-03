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
走行体の従来のカメラ利用は v4l2-ctl の 1枚撮り（PythonFile/camera.py の
execute_camera_command）で、プロセス起動とデバイス open が毎回かかる。
シャッターが実際に切れる時刻もばらつくので、0.5秒の区間を狙って撮れない。

ここではデバイスを開きっぱなしにして連続で取る。1枚あたりのコストが下がり、
区間内で複数枚読める。判定は数枚あれば決まる。

【溜め込みに注意】
VideoCapture.read() は「次の1枚」を返すので、取り込みが撮影より遅いと古い順に
溜まったものが返る。走行体は動いているため、これは「さっき居た場所の映像」に
なる。grab_latest() で捨ててから取る。
"""

import logging
from typing import Any, Optional

logger = logging.getLogger(__name__)

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


class BottleColorReader:
    """カメラを開きっぱなしにして、ラベル色を読み続ける。"""

    def __init__(self, camera: Optional[RobotCamera] = None,
                 monitor: Optional[BottleColorMonitor] = None,
                 width: int = 640, height: int = 480):
        # 640x480 で開く。色を見るだけなので高解像度は要らず、1枚あたりの
        # 取り込みとデコードを軽くしたい
        self.camera = camera or RobotCamera(width=width, height=height)
        self.monitor = monitor or BottleColorMonitor()

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

        height, width = _frame_size(frame)
        if width <= 0 or height <= 0:
            return BottleColor.NONE
        return self.monitor.read(frame, width, height)

    def hold(self) -> str:
        """抱えた色を覚える（初回のみ）。抱えた周期に呼ぶ"""
        return self.monitor.hold()

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


def _frame_size(frame: Any):
    """映像の縦横を取る。numpy 配列でも、入れ子のリストでも読めるようにする"""
    shape = getattr(frame, "shape", None)
    if shape is not None and len(shape) >= 2:
        return int(shape[0]), int(shape[1])
    try:
        return len(frame), len(frame[0])
    except (TypeError, IndexError):
        return 0, 0
