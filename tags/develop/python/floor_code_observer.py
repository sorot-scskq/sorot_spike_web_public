"""
床の二次元コードの「読めた四隅」から、向きの観測を作る（2026・ETラリー）。

**読むのは `Common/floor_code.py`（B の成果）。** そこが返す `FloorCodeSighting` は
中身（`A1`〜`D4`）と四隅を画素で持っているので、ここでは

    1. 四隅を床の座標（x=前・y=左[mm]）へ戻す
    2. コードの辺の向きから、**コース座標系での走行体の向き**を出す
    3. 中身（ID）をそのまま持たせる（`floor_code_gate` の照合に使う）

の3つだけをする。判断（採るか捨てるか）は `floor_code_gate.py` が持つ。

【床へ戻す道は2つある】
    GroundPlane（D の校正）があればそれを使う。メジャーで測った4点から作るので確か
    無ければ**カメラの幾何**（`heading_reader.ROBOT_CAMERA`）で戻す。
    いまの値は `heading_reader.ROBOT_CAMERA` を見ること（2026-09-23 時点で
    画角 53.9° / 高さ 205mm / 俯角 21.0°）。**ここに数字を書かない。**
    2026-09-23 に「高さ 220mm・俯角 41.3° は現地の実測」と書いてあったが、
    どちらもコードの値と食い違っていた（幾何は 2026-09 だけで4度変わっている）

【向きの出どころ】
コードは 50mm の正方形で、**床のコードは全部同じ向きに貼ってある**（2026-09-20 に確認）。
四隅のうち上辺（左上→右上）と下辺（左下→右下）、左辺（左上→左下）と右辺（右上→右下）を
それぞれ平均して2本の軸にし、**2本とも使って**コードの向きを決める。
1辺だけで決めると、奥行き方向に潰れた辺の誤差がそのまま乗る
（`Docs/ground_homography.md` 3章。奥行きは横の数倍に効く）。

    走行体の向き（コース座標系） = CODE_AXIS_COURSE_DEG − コードの上向きの方位

方位は**時計回りが正**（C++ の自己位置と同じ）。
"""

import logging
import math
from typing import Any, Dict, List, Optional, Sequence

try:
    from heading_reader import ROBOT_CAMERA
except ImportError:  # パッケージとして読み込まれた場合
    from .heading_reader import ROBOT_CAMERA

logger = logging.getLogger(__name__)

#: コードの1辺[mm]（規約 7.3.2）
CODE_SIZE_MM = 50.0

#: コードの「上」がコース座標系で向いている方位[度]
#:
#: **45 の倍数だけを入れること。** 打ち直しは「いちばん近い 45度の倍数からのずれ」を
#: 使うので、半端な値を入れると基準のずれが相殺されなくなる（`floor_code_gate`）。
#: 実コースで一度測れば決まる定数。0 のままなら「格子基準のずれ」として働く
CODE_AXIS_COURSE_DEG = 0.0

#: 辺の長さがこれだけ食い違ったら、床へ戻せていないとみなして捨てる
#: （遠すぎる・地平線の向こう・カメラの幾何が違う）
#:
#: **辺の狂いは倍率の狂い。** カメラの高さだけが違うと、床の座標は一様に伸び縮みし、
#: 辺は外れるが**向きは合ったまま**になる。向きに効くのは下の直角のほう
SIZE_TOLERANCE_MM = 25.0

#: 直角からの外れの上限[度]。平面として素直でなければ捨てる
#:
#: **こちらが向きに効く。** 俯角が違うと床の座標が縦横で別々に伸びるので、
#: 正方形が平行四辺形に見える。そのとき向きも同じだけ狂っている
RIGHT_ANGLE_TOLERANCE_DEG = 25.0


def bearing_deg(dx_mm: float, dy_mm: float) -> float:
    """床のベクトル（x=前・y=左）が向いている方位[度]。時計回りが正"""
    return math.degrees(math.atan2(-dy_mm, dx_mm))


class FloorCodeObservation:
    """床のコード1個ぶんの観測（`floor_code_gate` が受け取る形）"""

    def __init__(self, heading_deg: float, x_mm: float, y_mm: float,
                 sigma_deg: Optional[float] = None, code_id: str = "",
                 size_mm: float = 0.0, right_angle_deg: float = 90.0):
        #: コース座標系での走行体の向き[度]
        self.heading_deg = float(heading_deg)
        #: コードの中心（走行体から見て前[mm]）
        self.x_mm = float(x_mm)
        #: 同（左[mm]）
        self.y_mm = float(y_mm)
        #: 精度の目安[度]
        self.sigma_deg = None if sigma_deg is None else float(sigma_deg)
        #: 読めた中身（`A1`〜`D4`）。読めていなければ空
        self.code_id = code_id
        #: 床へ戻した辺の長さ[mm]（理想は 50mm）
        self.size_mm = float(size_mm)
        #: 床へ戻した左上の角[度]（理想は 90度）
        self.right_angle_deg = float(right_angle_deg)

    def __repr__(self) -> str:
        return ("FloorCode(%s %.2f°, 前%.0f 左%.0f, 辺%.0fmm)"
                % (self.code_id or "??", self.heading_deg, self.x_mm, self.y_mm,
                   self.size_mm))


class FloorCodeObserver:
    """
    読めたコードを、向きの観測に直す。

    **状態を持たない。** 1枚ごとに完結する。
    """

    def __init__(self, plane: Any = None, camera: Any = ROBOT_CAMERA,
                 code_axis_course_deg: float = CODE_AXIS_COURSE_DEG,
                 decoder: Any = None):
        #: 床平面（D の GroundPlane）。無ければカメラの幾何で戻す
        self.plane = plane
        self.camera = camera
        self.code_axis_course_deg = float(code_axis_course_deg)
        self._decoder = decoder
        self._told_size = False

    # --- 1枚から観測を作る ----------------------------------------------------

    def observe(self, frame: Any) -> List[FloorCodeObservation]:
        """映像1枚から、読めたコードぶんの観測を返す。読めなければ空"""
        size = image_size_of(frame)
        out = []
        for sighting in self.read(frame):
            observation = self.from_sighting(sighting, image_size=size)
            if observation is not None:
                out.append(observation)
        return out

    def read(self, frame: Any) -> Sequence[Any]:
        """映像1枚から読めたコード（`FloorCodeSighting`）。**直す前の段**

        **「読めていない」と「読めたが捨てた」を分けて見るために公開してある**
        （`Tools/check_heading_observation.py`）。
        """
        return self._read(frame)

    def _read(self, frame: Any) -> Sequence[Any]:
        decoder = self._decoder
        if decoder is None:
            from floor_code import FloorCodeDecoder
            decoder = self._decoder = FloorCodeDecoder()
        return decoder.read(frame)

    def from_sighting(self, sighting: Any,
                      image_size: Any = None) -> Optional[FloorCodeObservation]:
        """読めた1個を観測に直す。直せなければ None（理由が要るなら `inspect`）"""
        observation, _ = self.inspect(sighting, image_size)
        return observation

    def inspect(self, sighting: Any, image_size: Any = None):
        """
        読めた1個（`FloorCodeSighting`）を観測に直す。直せなければ None。

        四隅は隣どうしが辺になる順（`Common/floor_code.py`）。**どの隅が「上」かは
        使わない。** 読み取りによって回り方が違うが、打ち直しに使うのは
        「いちばん近い 45度の倍数からのずれ」なので、90度ずれても同じ値になる。

        :param image_size: その映像の (横, 縦)[px]。カメラの定数は校正したときの
                           画素に紐づくので、違う大きさの映像はここで合わせる
        :returns: (観測, 理由)。直せたら理由は空文字
        """
        corners = getattr(sighting, "corners", None)
        if not corners or len(corners) != 4:
            return None, "四隅が4つ無い"

        if image_size is None and not self._told_size:
            # **渡し忘れると、辺も角も検査を通ったまま向きが十度ずれる。**
            # 映像の大きさが定数と同じなら実害は無いので、止めずに知らせるだけ
            self._told_size = True
            logger.warning("映像の大きさが渡されていません。カメラの定数（%dx%d）の"
                           "ままとみなします", self.camera.width, self.camera.height)

        floor = [self.to_floor(u, v, image_size) for u, v in corners]
        if any(p is None for p in floor):
            # 地平線の向こう＝床ではない。遠いコードでは起きる
            return None, "地平線の向こう（床として戻せない）"

        shape = square_geometry(floor)
        if shape is None:
            return None, "床へ戻すと潰れてしまう"
        up_len, right_len, angle, axis = (
            shape["up_len"], shape["right_len"], shape["angle"], shape["axis"])

        # **床へ戻した形が正方形になっていなければ、戻し方が合っていない。**
        # カメラの幾何が違う・遠すぎる・隅を取り違えた、のどれか
        size = (up_len + right_len) / 2.0
        if (abs(up_len - CODE_SIZE_MM) > SIZE_TOLERANCE_MM
                or abs(right_len - CODE_SIZE_MM) > SIZE_TOLERANCE_MM):
            why = ("床へ戻した辺が %.0f/%.0fmm（50mm のはず。倍率＝高さ・画角を疑う）"
                   % (up_len, right_len))
            logger.debug("%s", why)
            return None, why

        if abs(angle - 90.0) > RIGHT_ANGLE_TOLERANCE_DEG:
            why = "床へ戻した角が %.0f度（90度のはず。俯角を疑う）" % angle
            logger.debug("%s", why)
            return None, why

        center_x, center_y = shape["center_x"], shape["center_y"]

        return FloorCodeObservation(
            heading_deg=self.code_axis_course_deg - axis,
            x_mm=center_x, y_mm=center_y,
            sigma_deg=self._sigma(sighting, size),
            code_id=str(getattr(sighting, "text", "") or ""),
            size_mm=size, right_angle_deg=angle), ""

    # --- 床へ戻す -------------------------------------------------------------

    def to_floor(self, u: float, v: float, image_size: Any = None):
        """画素 (u, v) が見ている床の点 (前[mm], 左[mm])。床でなければ None

        **映像の大きさが校正のときと違えば、先に合わせる。** 合わせずに渡すと
        床の座標が倍ずれる（1920 の映像を 1280 の定数で戻すなど）。
        """
        if self.plane is not None:
            points = self.plane.to_floor([(u, v)], image_size)
            point = points[0] if points else None
            return None if point is None else (float(point[0]), float(point[1]))
        return self._camera_for(image_size).image_to_ground(u, v)

    def _camera_for(self, image_size: Any):
        """その映像の大きさに合わせたカメラの定数（画角・俯角・高さは変わらない）"""
        if not image_size:
            return self.camera
        width, height = int(image_size[0]), int(image_size[1])
        if width == self.camera.width and height == self.camera.height:
            return self.camera
        key = (width, height)
        if getattr(self, "_scaled_key", None) != key:
            self._scaled_key = key
            self._scaled = self.camera.with_size(width, height)
        return self._scaled

    def _sigma(self, sighting: Any, size_mm: float) -> Optional[float]:
        """
        精度の目安[度]。**遠くて小さく写るほど悪くなる。**

        隅の位置決めを 1px とみなし、それが床で何度の傾きになるかで見る
        （`Docs/ground_homography.md` 3章）。
        """
        height_px = float(getattr(sighting, "height_px", 0.0) or 0.0)
        if height_px <= 0.0 or size_mm <= 0.0:
            return None
        mm_per_px = size_mm / height_px
        return math.degrees(math.atan2(mm_per_px, size_mm))


def square_geometry(floor: Sequence[Sequence[float]]) -> Optional[Dict[str, float]]:
    """
    床へ戻した四隅から、コードの形と向きを出す。潰れていれば None。

    **当てはめ（`floor_code_fit.py`）と共有する。** あちらは「50mm の正方形・直角に
    なるカメラの定数」を探すので、同じ計算でなければ意味が無い。

    :returns: up_len / right_len[mm]、angle[度]（理想は90）、axis（上向きの方位）、
              center_x / center_y[mm]
    """
    if len(floor) != 4:
        return None
    top_left, top_right, bottom_right, bottom_left = floor

    # 上向き（下辺 → 上辺）を、左右の2辺の平均で取る
    up_x = ((top_left[0] - bottom_left[0]) + (top_right[0] - bottom_right[0])) / 2.0
    up_y = ((top_left[1] - bottom_left[1]) + (top_right[1] - bottom_right[1])) / 2.0
    # 右向き（左辺 → 右辺）も同じく2辺の平均で取る
    right_x = ((top_right[0] - top_left[0]) + (bottom_right[0] - bottom_left[0])) / 2.0
    right_y = ((top_right[1] - top_left[1]) + (bottom_right[1] - bottom_left[1])) / 2.0

    up_len = math.hypot(up_x, up_y)
    right_len = math.hypot(right_x, right_y)
    if up_len < 1e-6 or right_len < 1e-6:
        return None

    angle = abs(bearing_deg(up_x, up_y) - bearing_deg(right_x, right_y))
    angle = min(angle % 360.0, 360.0 - angle % 360.0)

    # 上向きと、右向きを90度戻したものの平均（2本とも使う）
    axis = _average_deg(bearing_deg(up_x, up_y), bearing_deg(right_x, right_y) - 90.0)

    return {"up_len": up_len, "right_len": right_len, "angle": angle, "axis": axis,
            "center_x": sum(p[0] for p in floor) / 4.0,
            "center_y": sum(p[1] for p in floor) / 4.0}


def image_size_of(frame: Any) -> Optional[tuple]:
    """映像の (横, 縦)[px]。分からなければ None"""
    shape = getattr(frame, "shape", None)
    if not shape or len(shape) < 2:
        return None
    return (int(shape[1]), int(shape[0]))


def _average_deg(a: float, b: float) -> float:
    """2つの方位[度]の平均（0度をまたいでも正しい）"""
    x = math.cos(math.radians(a)) + math.cos(math.radians(b))
    y = math.sin(math.radians(a)) + math.sin(math.radians(b))
    return math.degrees(math.atan2(y, x))


def as_dicts(observations: Sequence[FloorCodeObservation]) -> List[Dict[str, Any]]:
    """確かめる用（ログや JSON に載せる）"""
    return [{"code_id": o.code_id, "heading_deg": round(o.heading_deg, 2),
             "x_mm": round(o.x_mm), "y_mm": round(o.y_mm),
             "size_mm": round(o.size_mm), "sigma_deg": None if o.sigma_deg is None
             else round(o.sigma_deg, 2)}
            for o in observations]
