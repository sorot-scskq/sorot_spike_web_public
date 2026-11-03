"""
OpenCV と numpy の読み込みを1か所にまとめる。

【なぜ遅延して読むのか】
走行体に OpenCV が入っていない環境で、モジュールの先頭の import が失敗して
ev3_python 全体が起動できず、C++ 側が「接続失敗、再試行します」を出し続けた
ことがある。**映像を扱わない処理は動くようにする**ため、使うところで読む。

以前は camera / qr_decoder / bottle_color_monitor / bottle_color_reader が
それぞれ同じ `_cv2()` を持っていた。読めなかったときの振る舞い（None を返す）を
4か所で合わせ続けることになるので、ここ1つにしてある。
"""

import logging
from typing import Any, Optional

logger = logging.getLogger(__name__)


class OpenCv:
    """OpenCV（cv2）と numpy を、入っていなければ None で返す"""

    _cv2: Optional[Any] = None
    _numpy: Optional[Any] = None
    _warned = False

    @classmethod
    def cv2(cls) -> Optional[Any]:
        """OpenCV。無ければ None（映像を扱わない処理は続けられるようにする）"""
        if cls._cv2 is None:
            try:
                import cv2
                cls._cv2 = cv2
            except ImportError:
                cls._warn()
                return None
        return cls._cv2

    @classmethod
    def numpy(cls) -> Optional[Any]:
        """numpy。無ければ None"""
        if cls._numpy is None:
            try:
                import numpy
                cls._numpy = numpy
            except ImportError:
                cls._warn()
                return None
        return cls._numpy

    @classmethod
    def _warn(cls) -> None:
        """1回だけ知らせる。毎周期出すとログが埋まる"""
        if not cls._warned:
            cls._warned = True
            logger.warning("OpenCV / numpy が入っていません。映像を使う処理は動きません"
                           "（Common/setup_venv.sh）")
