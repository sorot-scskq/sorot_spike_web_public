"""
前面カメラで、ET ラリーのグリッドに対する向きのずれを測る（2026・ETラリー）。

経路の直進の行に書いた SCRIPT（2026/recog/grid_heading.py）から呼ばれる。
測った値は判定結果に載せ、C++ が向き保持直進（FUNCNO 19）の目標の向きを直す
（`Docs/heading_hold.md`）。

    カメラ ──▶ HeadingReader ──▶ HeadingTask ──▶ CMD_GET_RESULT の heading_dev / heading_seq
                                                      │
                                          C++ ResultRead ──▶ RunControl ──▶ HeadingPlan

【走行を止めないこと】
カメラが無い・開けない・測れない、のいずれでも走行は続ける。測れなければ
heading_seq が増えないだけで、C++ は何もしない。ここで例外を投げると、
C++ が判定結果を取りにいくたびに詰まる。

【映像の取り方】
ヒントカードと同じ（`2026/hint_card_capture.py`）。色の読み取りが回っていれば
その最後の1枚を借り、回っていなければ自分で1枚取る。**カメラは1台しかないので、
2つのスレッドから read() しない。**

【カメラの定数は未実測】
`heading_reader.ROBOT_CAMERA` の画角と俯角は仮の値。**実機で測ってから使うこと。**
黒線で測るときは、俯角が 0.1° ずれると 1°近く外れる。
"""

import logging
import os
import threading
import time
from typing import Any, Callable, Dict, Optional

try:
    from heading_reader import ROBOT_CAMERA, HeadingReader
except ImportError:  # パッケージとして読み込まれた場合
    from .heading_reader import ROBOT_CAMERA, HeadingReader

from floor_code import FloorCodeDecoder
from grid_position import GridPositionFixer
from pose import PoseReader
from robot_pose import RobotPose
from state import ResultStore

try:
    from opencv import OpenCv
except ImportError:  # パッケージとして読み込まれた場合
    try:
        from ..Common.opencv import OpenCv
    except (ImportError, ValueError):
        from Common.opencv import OpenCv

logger = logging.getLogger(__name__)

#: 測った瞬間の映像を残す先。
#:
#: 走行体には画面が無いので、走行中は「撮れていないのか」「撮れているが目印を
#: 見つけられないのか」「見つけたが値が外れているのか」を切り分けられない。
#: 座標と向きを名前に入れておけば、あとから絵と軌跡（trajectory.csv）を突き合わせられる。
#: カメラの画角・俯角を実機で合わせるときの材料にもなる。
SNAPSHOT_DIR = "./sorot_spike/RoughSpot/PhotoImage/heading"

#: 残す枚数の上限。走行を重ねても溢れないよう、古いものから消す
SNAPSHOT_KEEP = 80

#: 0 にすると保存しない。**本番は 0 にすること**
SNAPSHOT_ENABLE = 1

#: 測れなかった回も残すか。1 なら残す（切り分けの手立てになるのはむしろこちら）
SNAPSHOT_ON_FAIL = 1

#: 床の二次元コードの中身（`A1`〜`D4`）も読むか
#:
#: **読めても位置を当てるのには使っていない**（`2026/grid_position.py` の説明）。
#: 「実機で 1枚に何個読めるか」を数えるために回している。数が足りると分かれば、
#: 最寄り判定をやめて ID で当てる作りに移れる（設計 5.5 の (a)）。
#: **本番は 0 にしてよい**（50ms ほど余分にかかる）。
READ_CODE_IDS = 1

#: コードを読むときの縦の引き伸ばし。実機の1枚では 3.0 がいちばん当たった
#: （`Common/floor_code.py`）。**1つに絞ってある**。全部試すと 200ms を超える
CODE_STRETCH_FACTORS = (3.0,)

#: 測れなかった理由を、ファイル名に入れる短い綴りへ直す表
#:
#: 理由の文は grid_heading.py が日本語で作る。ファイル名には英数字だけを入れる。
REASON_SLUGS = (
    ("映像が無い", "noframe"),
    ("食い違う", "conflict"),
    ("格子に乗る目印", "fewgrid"),    # 「目印が」より先に見る（どちらにも「目印が」が入る）
    ("目印が", "fewmark"),
    ("組が無い", "nopair"),
    ("当てはまりが悪い", "badfit"),
    ("間隔が合わない", "badscale"),
    ("回転が定まらない", "unstable"),
)


def reason_slug(reason: Optional[str]) -> str:
    """測れなかった理由を、ファイル名に入れる短い綴りにする"""
    text = reason or ""
    for keyword, slug in REASON_SLUGS:
        if keyword in text:
            return slug
    return "ng"


def build_snapshot_name(result: Optional[Dict[str, Any]], pose: Optional[Dict[str, Any]],
                        when: Optional[float] = None) -> str:
    """
    デバッグ画像のファイル名を組み立てる。

        heading_20260919_121530_123_s30c12_x+1234_y+0568_d135_dev-1.23_m7_markers.jpg
        heading_20260919_121530_123_s30c12_x+1234_y+0568_d135_ng-fewmark_m3.jpg
                 ↑日時_ミリ秒      ↑区間  ↑座標[mm]      ↑向き ↑測ったずれ/理由

    - 座標と向きは**自己位置推定の値**（走行体が思っている位置）。真値は分からないので、
      軌跡（trajectory.csv）と突き合わせて使う
    - 座標が届いていなければ `xNA_yNA_dNA`（座標を載せない古い C++ と混ぜたとき）
    - 向きは 0〜359 に丸める

    :param result: HeadingReader.read() の戻り。None なら測る前に落ちた扱い
    :param pose: RobotPose.snapshot()
    :param when: 時刻（time.time()）。省略すると今
    """
    now = time.time() if when is None else when
    stamp = "%s_%03d" % (time.strftime("%Y%m%d_%H%M%S", time.localtime(now)), int((now % 1) * 1000))

    pose = pose or {}
    if "sno" in pose and "cno" in pose:
        section = "s%dc%d" % (pose["sno"], pose["cno"])
    else:
        section = "sNAcNA"
    if "x" in pose and "y" in pose:
        place = "x%+05d_y%+05d" % (round(pose["x"]), round(pose["y"]))
    else:
        place = "xNA_yNA"
    if "heading" in pose:
        place += "_d%03d" % (round(pose["heading"]) % 360)
    else:
        place += "_dNA"

    result = result or {}
    markers = result.get("markers")
    tail = "m%d" % markers if isinstance(markers, int) else "mNA"
    if result.get("ok"):
        body = "dev%+.2f_%s" % (float(result.get("deviation_deg") or 0.0), tail)
        source = result.get("source")
        if source:
            body += "_%s" % source
    else:
        body = "ng-%s_%s" % (reason_slug(result.get("reason")), tail)

    return "heading_%s_%s_%s_%s.jpg" % (stamp, section, place, body)


def _make_floor_code_fallback(task: Any) -> Optional[Any]:
    """床の二次元コードで測る道具。**作れなければ None**（校正が無いなど）"""
    try:
        from floor_code_reader import FloorCodeReader
        return FloorCodeReader(frame_source=lambda: None)
    except Exception:  # noqa: BLE001 — 落とし先が無くても走行は続ける
        logger.warning("床のコードの読み取りを用意できませんでした", exc_info=True)
        return None


class HeadingTask:
    """SCRIPT で指示された区間で、向きのずれを 1回測る。"""

    _instance: Optional["HeadingTask"] = None
    _instance_lock = threading.Lock()

    @classmethod
    def get_instance(cls) -> "HeadingTask":
        """走行体で共有する1つを返す。無ければ作る"""
        with cls._instance_lock:
            if cls._instance is None:
                cls._instance = cls()
            return cls._instance

    @classmethod
    def reset_instance(cls) -> None:
        """共有している1つを捨てる（テスト用）"""
        with cls._instance_lock:
            cls._instance = None

    def __init__(self, frame_source: Optional[Callable[[], Any]] = None, camera=ROBOT_CAMERA,
                 reader: Optional[Any] = None, fallback: Optional[Any] = None):
        """
        :param reader: 測る道具。省くと `HeadingReader`（灰色の丸と黒線）。
                       **床の二次元コードで測るなら `FloorCodeReader` を渡す**
                       （`2026/recog/floor_code_heading.py`）。`read()` と
                       `last_frame` を持っていればよい
        """
        self._frame_source = frame_source
        self._reader = reader if reader is not None else HeadingReader(
            frame_source=self._frame, camera=camera)
        #: 目印で測れなかったときに、**同じ1枚**で試すもう一つの道具。
        #: 省くと床の二次元コード（`FloorCodeReader`）。`None` にすれば落とし先なし
        self._fallback = fallback if fallback is not None else _make_floor_code_fallback(self)
        self._lock = threading.Lock()
        #: 測れた回数。C++ はこれが増えたときだけ取り込む
        self._seq = 0
        #: 位置を打ち直した回数。C++ はこれが増えたときだけ取り込む（向きとは別に数える）
        self._coord_seq = 0
        #: 直近の結果（確かめる用）
        self.last: Optional[dict] = None
        #: デバッグ画像の置き場（テストで差し替える）
        self.snapshot_dir = SNAPSHOT_DIR
        #: 床のコードの中身を読む口。読めても当てるのには使わない（READ_CODE_IDS）
        self._decoder = FloorCodeDecoder(CODE_STRETCH_FACTORS)

    def measure(self, request_id: int = 0) -> Optional[dict]:
        """
        1枚撮って測り、測れたら判定結果に載せる。

        :returns: 測れた結果。測れなければ None
        """
        with self._lock:
            try:
                result = self._reader.read()
            except Exception:  # noqa: BLE001 — 測れなくても走行は続ける
                logger.exception("向きのずれの測定に失敗しました")
                return None

            self.last = result
            self.save_snapshot(result)
            if not result.get("ok"):
                alt = self._try_fallback(result)
                if alt is None:
                    logger.info("向きのずれは測れませんでした（%s）", result.get("reason"))
                    return None
                result = alt

            self.publish(result["deviation_deg"],
                         source=result.get("source"), markers=result.get("markers"),
                         elapsed_ms=result.get("elapsed_ms", 0))
            # 同じ1枚から位置も出す。**向きより後**（Docs/position_correction_design.md 5.5）
            self.fix_position(result)
            return result

    def _try_fallback(self, failed: dict) -> Optional[dict]:
        """目印で測れなかった1枚を、**同じ絵のまま**床の二次元コードで測り直す。

        **撮り直さない。** 走行中なので、撮り直すと別の場所の絵になる。

        目印は床の格子（灰色の丸とQR）が要るが、コースの3割は格子の外を通る
        （2026-09-23 の実測: 105枚中 37枚が目印 0〜3個）。床のコードはファインダ
        3個だけを見るので、そこでも測れることがある（同じ実測で 22枚を拾えた）。

        :returns: 測れた結果。落とし先が無い・測れないなら None
        """
        if self._fallback is None:
            return None
        frame = getattr(self._reader, "last_frame", None)
        if frame is None:
            return None
        try:
            self._fallback.frame_source = lambda: frame
            alt = self._fallback.read()
        except Exception:  # noqa: BLE001 — 落とし先が転んでも走行は続ける
            logger.exception("床のコードでの測り直しに失敗しました")
            return None
        if not alt.get("ok"):
            logger.debug("床のコードでも測れませんでした（%s）", alt.get("reason"))
            return None
        logger.info("目印では測れず（%s）、床のコードで測りました", failed.get("reason"))
        return alt

    def publish(self, deviation_deg: float, source: Any = None,
                markers: Any = None, elapsed_ms: float = 0.0) -> int:
        """
        測ったずれを判定結果に載せる（C++ が向き保持の目標を直す）。

        **測る道が2つあるので、載せる口はここ1つにしてある。** 目印・黒線で測る
        こちらと、床の二次元コードで測る `heading_fix_task` が、**同じ通し番号**を
        使う（別々に数えると、C++ 側の「増えた回だけ取り込む」が噛み合わない）。

        :returns: 載せた通し番号
        """
        self._seq += 1
        store = ResultStore.get_instance()
        store.set("heading_dev", round(float(deviation_deg), 2))
        store.set("heading_seq", self._seq)
        logger.info("向きのずれ %+.2f°（%s・目印%s個・%.0fms・%d回目）",
                    deviation_deg, source, markers, elapsed_ms, self._seq)
        return self._seq

    def fix_position(self, result: Dict[str, Any]) -> Optional[dict]:
        """測った1枚から、自己位置の位置の打ち直し量を出して判定結果に載せる。

        **測れなくても走行は続ける。** 採らなかった回は coord_seq が増えないだけで、
        C++ は何もしない。採否と量は必ずログに残す（後から「効いたのか、たまたま
        合ったのか」を見分けるため。`Docs/position_correction_design.md` 6）。

        :returns: 打ち直した中身。打ち直さなければ None
        """
        try:
            pose = self.pose_now()
            fix = GridPositionFixer.get_instance().fix(result, pose, self.read_code_ids())
        except Exception:  # noqa: BLE001 — 出せなくても走行は続ける
            logger.exception("位置の打ち直し量の計算に失敗しました")
            return None

        if fix.code_ids:
            # (a) へ移れるかの材料。**1枚で2個読めないと格子の向きが決まらない**
            logger.info("床のコードを %d個 読めました: %s",
                        len(fix.code_ids), " ".join(fix.code_ids))

        if not fix.ok:
            logger.info("位置は打ち直しませんでした（%s）", fix.reason)
            return None
        if fix.anchored:
            logger.info("格子の置き場を決めました（x %+.0f y %+.0f 向き %+.2f°）",
                        GridPositionFixer.get_instance().anchor.x,
                        GridPositionFixer.get_instance().anchor.y,
                        GridPositionFixer.get_instance().anchor.angle_deg)

        self._coord_seq += 1
        store = ResultStore.get_instance()
        store.set("coord_dx", round(fix.dx, 1))
        store.set("coord_dy", round(fix.dy, 1))
        store.set("coord_dr", round(fix.dheading_deg, 2))
        store.set("coord_seq", self._coord_seq)
        logger.info("位置を打ち直します dx %+.0f dy %+.0f mm（格子に沿って %+.0f, %+.0f）"
                    " 向き %+.2f°（%d回目）",
                    fix.dx, fix.dy, fix.along[0], fix.along[1], fix.dheading_deg, self._coord_seq)
        return fix.as_dict()

    def save_snapshot(self, result: Optional[Dict[str, Any]]) -> Optional[str]:
        """測った1枚を、座標・向き・測った値を名前に入れて残す。

        **失敗しても走行は止めない。** 例外は握りつぶして警告だけ出す。

        :returns: 書けたファイルのパス。書けなければ None
        """
        if not SNAPSHOT_ENABLE:
            return None
        if not (result or {}).get("ok") and not SNAPSHOT_ON_FAIL:
            return None

        frame = self._reader.last_frame
        if frame is None:
            # ここが出たら、測り方ではなくカメラを疑う
            logger.warning("映像が取れていないため、向きのデバッグ画像を残せません")
            return None

        try:
            cv2 = OpenCv.cv2()
            if cv2 is None:
                logger.warning("OpenCV が無いため、向きのデバッグ画像を残せません")
                return None

            os.makedirs(self.snapshot_dir, exist_ok=True)
            name = build_snapshot_name(result, RobotPose.get_instance().snapshot())
            path = os.path.join(self.snapshot_dir, name)
            if not cv2.imwrite(path, frame):
                logger.warning("向きのデバッグ画像を書けませんでした: %s", path)
                return None

            logger.info("向きのデバッグ画像を残しました: %s", path)
            self._prune()
            return path
        except Exception as e:  # noqa: BLE001 — 残せなくても走行は続ける
            logger.warning("向きのデバッグ画像を残せませんでした: %s", e)
            return None

    def read_code_ids(self) -> list:
        """測った1枚から、床のコードの中身（`A1`〜`D4`）を読む。

        **読めても位置を当てるのには使わない**（`2026/grid_position.py` の説明）。
        実機で「1枚に何個読めるか」を数えるために回している。
        読めなくても、失敗しても、走行は続ける。
        """
        if not READ_CODE_IDS:
            return []
        frame = self._reader.last_frame
        if frame is None:
            return []
        try:
            return self._decoder.read_ids(frame)
        except Exception:  # noqa: BLE001 — 読めなくても走行は続ける
            logger.exception("床のコードの読み取りに失敗しました")
            return []

    def pose_now(self) -> Dict[str, Any]:
        """撮った時点の自己位置。

        **C++ が 100ms ごとに置いているものを読む**（`Common/pose.py`）。要求に
        載ってきた値（`robot_pose.py`）より新しく、撮った瞬間に近い。

        古い値で位置を打ち直すと事故になるので `read_fresh()` を使う。C++ が
        止まってもファイルは残るため、`read()` では古さに気づけない。
        置かれていない／古いときは、要求に載ってきた値に落ちる。
        """
        pose = PoseReader.read_fresh()
        if pose is not None:
            return {"x": pose.x, "y": pose.y, "heading": pose.heading_deg,
                    "sno": pose.sno, "cno": pose.cno}
        return RobotPose.get_instance().snapshot()

    def _prune(self) -> None:
        """古い画像から消す。走行を重ねてもディスクを埋めない"""
        try:
            files = sorted(
                f for f in os.listdir(self.snapshot_dir)
                if f.startswith("heading_") and f.endswith(".jpg"))
            for f in files[:-SNAPSHOT_KEEP]:
                os.remove(os.path.join(self.snapshot_dir, f))
        except Exception:  # noqa: BLE001 — 消せなくても走行は続ける
            pass

    def _frame(self) -> Any:
        """いまの映像。色の読み取りが回っていれば、その最後の1枚を借りる"""
        if self._frame_source is not None:
            try:
                return self._frame_source()
            except Exception:  # noqa: BLE001
                logger.exception("映像の取り込みに失敗しました")
                return None

        from bottle_color_task import BottleColorTask
        from camera import RobotCamera

        camera = RobotCamera.shared()
        if BottleColorTask.get_instance().is_reading():
            # 同時に read() すると壊れる（camera.py の RobotCamera を参照）
            return camera.latest_frame()
        return camera.grab_latest()
