"""
向きの補正を当てた前と後の写真を残す（効いたかを後から絵で確かめるため）。

  前   補正の値を測った絵（呼ぶ側が渡す）
  後   当ててから `AFTER_SECONDS` 秒後のカメラの絵（向き保持が追いついた頃）

`RoughSpot/PhotoImage/correction/` に、組の番号・出どころ・前後・その時の行・値を名前に入れて置く。
同じ所に `index.csv` を1行ずつ足す（自己位置付き。組を並べるときに使う）。

**書き込みは別のスレッドで行う。** 呼ぶ側（ゲートの確認の流れなど）を待たせない。
**失敗しても走行は止めない。** 止めるときは `correction_photo.txt` に 0。
"""

from __future__ import annotations

import logging
import os
import threading
import time
from typing import Any, Callable, Optional, Sequence

from route_register import SwitchFile

logger = logging.getLogger(__name__)

ENABLED = True
ENABLED_SWITCH = "./sorot_spike/correction_photo.txt"
PHOTO_DIR = "./sorot_spike/RoughSpot/PhotoImage/correction"

#: 後の写真を撮るまでの時間[秒]。C++ は 1回 3°まで当て、向き保持はおよそ 1秒で追いつく
AFTER_SECONDS = (1.0,)

#: 残す枚数の上限。古いものから消す（1走行でおよそ 25組＝50枚）
KEEP = 400


class CorrectionPhoto:
    """走行体で1つだけ使う"""

    _instance: Optional["CorrectionPhoto"] = None
    _instance_lock = threading.Lock()

    @classmethod
    def shared(cls) -> "CorrectionPhoto":
        with cls._instance_lock:
            if cls._instance is None:
                cls._instance = cls()
            return cls._instance

    @classmethod
    def reset_shared(cls) -> None:
        """共有している1つを捨てる（テスト用）"""
        with cls._instance_lock:
            cls._instance = None

    def __init__(self, photo_dir: str = PHOTO_DIR,
                 camera_frame: Optional[Callable[[], Any]] = None,
                 pose_source: Optional[Callable[[], Any]] = None,
                 writer: Optional[Callable[[str, Any], bool]] = None,
                 sleep: Callable[[float], None] = time.sleep):
        self.photo_dir = photo_dir
        self._camera_frame = camera_frame
        self._pose_source = pose_source
        self._writer = writer
        self._sleep = sleep
        self._lock = threading.Lock()
        self._pair = 0
        self._told_off = False

    def enabled(self) -> bool:
        on = SwitchFile.read(ENABLED_SWITCH, ENABLED)
        if not on and not self._told_off:
            self._told_off = True
            logger.info("補正の前後の写真は残しません（%s が 0）", ENABLED_SWITCH)
        return on

    def record(self, frame: Any, source: str, deviation_deg: float,
               after_seconds: Sequence[float] = AFTER_SECONDS, wait: bool = False) -> Optional[int]:
        """
        当てた補正の前の絵を残し、後の絵を撮る段取りをする。

        :param frame: 補正の値を測った絵
        :param source: 出どころ（qr_steer / qr_turn / markers など）
        :param wait: True なら書き終わるまで待つ（テスト用）
        :returns: 組の番号。残さないなら None
        """
        if frame is None or not self.enabled():
            return None
        with self._lock:
            self._pair += 1
            pair = self._pair
        before = frame.copy() if hasattr(frame, "copy") else frame
        stamp = time.strftime("%H%M%S") + "_%03d" % (int(time.time() * 1000) % 1000)
        job = threading.Thread(target=self._run, name="correction-photo",
                               args=(pair, stamp, before, source, float(deviation_deg),
                                     tuple(after_seconds)), daemon=True)
        job.start()
        if wait:
            job.join()
        return pair

    # -- まわり ---------------------------------------------------------------

    def _run(self, pair: int, stamp: str, before: Any, source: str, dev: float,
             after_seconds: Sequence[float]) -> None:
        try:
            self._save(pair, stamp, "before", source, dev, before)
            waited = 0.0
            for i, at in enumerate(after_seconds, start=1):
                self._sleep(max(0.0, at - waited))
                waited = at
                frame = self._grab()
                if frame is None:
                    logger.info("補正の後の写真を撮れませんでした（組 %d・映像が無い）", pair)
                    return
                self._save(pair, stamp, "after%d" % i if len(after_seconds) > 1 else "after",
                           source, dev, frame, after_s=at)
            self._prune()
        except Exception:  # noqa: BLE001 — 残せなくても走行は続ける
            logger.warning("補正の前後の写真を残せませんでした（組 %d）", pair, exc_info=True)

    def _save(self, pair: int, stamp: str, kind: str, source: str, dev: float, frame: Any,
              after_s: float = 0.0) -> None:
        pose = self._pose()
        sno = getattr(pose, "sno", -1) if pose is not None else -1
        cno = getattr(pose, "cno", -1) if pose is not None else -1
        name = "corr_%s_%04d_%s_%s_s%dc%d_dev%+.2f.jpg" % (stamp, pair, source, kind, sno, cno, dev)
        os.makedirs(self.photo_dir, exist_ok=True)
        path = os.path.join(self.photo_dir, name)
        if not self._write(path, frame):
            logger.warning("補正の写真を書けませんでした: %s", path)
            return
        with open(os.path.join(self.photo_dir, "index.csv"), "a", encoding="utf-8") as f:
            f.write("%s,%d,%s,%s,%.2f,%.1f,%s,%s,%s,%s,%s,%s\n" % (
                time.strftime("%H:%M:%S"), pair, kind, source, dev, after_s, sno, cno,
                _fmt(pose, "x"), _fmt(pose, "y"), _fmt(pose, "heading_deg"), name))
        logger.info("補正の%s写真を残しました（組 %d・%s %+.2f°・%d/%d）",
                    "前の" if kind == "before" else "後の", pair, source, dev, sno, cno)

    def _write(self, path: str, frame: Any) -> bool:
        if self._writer is not None:
            return self._writer(path, frame)
        from opencv import OpenCv
        cv2 = OpenCv.cv2()
        return bool(cv2 is not None and cv2.imwrite(path, frame))

    def _grab(self) -> Any:
        if self._camera_frame is not None:
            return self._camera_frame()
        from camera import RobotCamera
        frame = RobotCamera.shared().latest_frame()
        return frame.copy() if frame is not None else None

    def _pose(self) -> Any:
        if self._pose_source is not None:
            return self._pose_source()
        from pose import PoseReader
        return PoseReader.read_fresh()

    def _prune(self) -> None:
        try:
            files = sorted(f for f in os.listdir(self.photo_dir)
                           if f.startswith("corr_") and f.endswith(".jpg"))
            for f in files[:-KEEP]:
                os.remove(os.path.join(self.photo_dir, f))
        except Exception:  # noqa: BLE001 — 消せなくても走行は続ける
            pass


def _fmt(pose: Any, key: str) -> str:
    value = getattr(pose, key, None) if pose is not None else None
    return "" if value is None else "%.1f" % float(value)
