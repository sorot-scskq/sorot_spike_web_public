"""
床の二次元コード（規約 7.3.2 の A1〜D4、50mm 四方）から走行体の向きを出す。

**復号しない。** ファインダパターン3個の並びだけを見る（`Docs/position_correction_design.md` 5.2〜5.4）。

【なぜ復号しないか】
実機で撮れた1枚では、`cv2.QRCodeDetector.detectAndDecode` も zxing-cpp も読めず、
**復号しない `detect` すら四隅を返さなかった**（CLAHE・鮮鋭化・適応二値・大津のどれを
噛ませても）。1モジュールは 46px あり、復号に要る 4px の10倍以上ある。読めないのは
小さいからではなく、ぼけているから（Laplacian 分散 6）。

一方、**輪郭の入れ子（3重に入れ子になった四角）を拾う道では、同じ画像でファインダ3個が
ちょうど、誤検出ゼロで取れた**。向きだけなら、これで足りる。

【向きだけなら、どのコードかが要らない】
**床のコードは全部同じ向きに貼られている（2026-09-20 に確認）。** ファインダは3隅にしか
無いので4回対称ではなく、向きが曖昧にならない。よって A1〜D4 のどれを見ていても、
そのコードの向き = コース座標系の向き。ID の対応付けが要るのは位置のほうだけ。

【QRコードの方向線】
**コードに描かれているものではない。こちらが引く線。** ファインダパターン3個の
**中心どうしを結んだ2本の線分**で、コードの上辺（左上→右上）と左辺（左上→左下）に
あたる。`direction_line` の 'top' と 'left' はこの2本のこと。

    ●━━━━━━━●   ← top（上辺の方向線）
    ┃ left
    ┃（左辺の方向線）
    ●

**向きはこの線分の向きそのもの。** 床のコードは全部同じ向きに貼ってあるので、
この線が床の上でどちらを向いているかが分かれば、走行体の向きが決まる。
線は2本あるので、**どちらで測るかを選ぶ**（`_sigma` を参照。長いほうではない）。

【床座標（この module の座標の決まり）】
床平面は `(x, y)` の2次元で扱う。**x は前、y は左**[mm]。角度は**右まわりが正**
（真正面が 0、右へ向けば +）。**C++ の自己位置と同じ決まり**にしてある。

    axis_deg     コードの上辺（左上→右上）が、走行体から見てどちらを向いているか
    heading_deg  コース座標系での走行体の向き = code_axis_course_deg - axis_deg
    x_mm, y_mm   コードの中心の位置（走行体から見て 前 / 左）

`code_axis_course_deg` は「コースの上辺が、コース座標系でどちらを向いているか」。
マットの上で一度測れば決まる定数。**既定は 0 なので、そのままだと heading_deg は
「コードの向きを 0 とした向き」になる**（= 格子に対するずれ）。

【床平面ホモグラフィは外から受け取る】
画像 → 床座標の変換は `FloorPlane` として外から渡す（校正は別作業）。**渡さなければ
`IdentityFloorPlane`（画像の画素をそのまま床座標とみなす）で動き、結果の
`calibrated` が False になる。** 真上から撮っていないかぎり値は当てにならないので、
使う側は `calibrated` を見ること。

`Common/ground_plane.py` の `GroundPlane`（校正する側が作るもの）は
`GroundPlaneAdapter` で包んで渡す。未校正で None を返すあいだは
`calibrated` が False になる。

**`GroundPlane.shared()` は、床のコードで校正したものだと None を返すことがある**
（走行体の置き方を渡すまで `frame` が 'grid' のまま。格子座標で位置を打ち直すと
全部ずれるための歯止め）。**向きだけなら 'grid' のままで使える** — 一定のずれが
乗るだけなので `code_axis_course_deg` で吸収できる。そのときは `shared()` ではなく
`GroundPlane.load()` で読んで渡すこと。

    plane = GroundPlaneAdapter(GroundPlane.load())   # 'grid' でも向きは出る

**ただし 'grid' のとき `x_mm` / `y_mm` は走行体基準ではない。** 取り違えないよう、
結果の `frame` に何基準かを載せてある。位置を使う側は必ず見ること。

【ゲートはここに入れていない】
自己位置との差で捨てる・2枚の間隔が 250mm 格子に合うか確かめる・打ち直しをログに残す
（設計書 6章）は、**使う側の仕事**。ここは観測を素のまま返す。ただし「三角形が直角でない」
「2本の方向線の長さが違いすぎる」のような、コードでないものを弾く検査だけは入れてある
（緩い既定値。`Options` で変えられる。判定の材料は結果にすべて載せてある）。

使い方

    from floor_code_heading import FloorCodeHeading, HomographyFloorPlane

    plane = GroundPlaneAdapter(GroundPlane.shared())      # 校正済みの床平面
    found = FloorCodeHeading(plane).measure(frame)
    for code in found.codes:
        print(code.heading_deg, code.x_mm, code.y_mm, code.sigma_deg)
"""

from __future__ import annotations

import json
import math
import time
from dataclasses import asdict, dataclass, field
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

try:
    from opencv import OpenCv
except ImportError:  # パッケージとして読み込まれた場合
    try:
        from .opencv import OpenCv
    except (ImportError, ValueError):
        from Common.opencv import OpenCv

#: 床のコードの一辺[mm]（規約 7.3.2）
CODE_SIZE_MM = 50.0

#: 床のコードのモジュール数（版1。実機の1枚で約21モジュール四方だった）
CODE_MODULES = 21

#: ファインダパターンの一辺[モジュール]。中心どうしは CODE_MODULES - 7 = 14 モジュール離れる
FINDER_MODULES = 7

#: ファインダが立たなくなる、1モジュールの画素数（**潰れているほうの向き**）。
#:
#: D の実測（2026-08-29 の1枚を縮めて）。**横ではなく縦で決まる**（床を浅く見ると縦が潰れる）。
#: 1920 なら奥のコードも取れるが、**1280 では奥が 1/3 しか立たない**（縦 1.1px/モジュール）。
#: 向きの観測に復号は要らないので、向きだけなら 1920 のほうが有利。
MODULE_PX_FLOOR = 1.5

#: ファインダ中心どうしの間隔[mm]。50 × 14/21 = 33.3mm。
#: **床座標の縮尺が合っているかは、これと `CodeObservation.line_mm` を比べれば分かる**
FINDER_SPACING_MM = CODE_SIZE_MM * (CODE_MODULES - FINDER_MODULES) / CODE_MODULES


# ---------------------------------------------------------------------------
# 床平面（画像 → 床座標）
# ---------------------------------------------------------------------------

class FloorPlane:
    """
    画像の画素を床座標 `(x, y)`[mm]（前 / 左）へ写すもの。

    差し替えられるようにしてある（校正は別作業）。`to_floor` だけ持っていればよい。
    """

    calibrated = False

    #: 床座標が何を基準にしているか。'robot'（走行体から見た前・左）か 'grid'（コースの格子）
    frame = "robot"

    def to_floor(self, points: Any, image_size: Optional[Tuple[int, int]] = None) -> Any:
        """
        :param points: (N, 2) の画素座標
        :param image_size: 映像の (幅, 高さ)。校正時と画素数が違うときに要る
        :returns: (N, 2) の床座標 `(x, y)`[mm]。写せなければ None
        """
        raise NotImplementedError


class IdentityFloorPlane(FloorPlane):
    """
    画素をそのまま床座標とみなす（校正前の仮）。

    真上から撮っていないかぎり値は当てにならない。`calibrated` が False なので、
    使う側はそれを見て捨てられる。
    """

    calibrated = False
    frame = "image"

    def to_floor(self, points: Any, image_size: Optional[Tuple[int, int]] = None) -> Any:
        np = OpenCv.numpy()
        return np.asarray(points, dtype=np.float64).reshape(-1, 2).copy()


class HomographyFloorPlane(FloorPlane):
    """
    床の既知4点で作ったホモグラフィで、画素を床座標へ戻す。

    カメラの内部パラメータが要らず、毎フレームの姿勢分解も要らない
    （`Docs/position_correction_design.md` 5.4）。行列は一度作れば固定。

    :param matrix: 画像 → 床座標 の 3x3
    :param y_is_left: 行列の2列目が「左」なら True（この module の決まり）。
                      「右」を返す行列なら False にすると内部で直す
    :param image_size: **校正したときの映像の (幅, 高さ)。** 行列はこれに紐づいている。
                       本番が違う画素数なら、渡しておけば内部で直す
                       （1920 で校正して 1280 で撮るのが実際の使い方）

    **`image_size` を渡さないまま違う画素数の映像を食わせると、黙ってずれる。**
    校正は 1920、本番は 1280 になる見込みなので、ここは必ず入れること。
    """

    calibrated = True

    def __init__(self, matrix: Any, y_is_left: bool = True, frame: str = "robot",
                 image_size: Optional[Tuple[int, int]] = None):
        np = OpenCv.numpy()
        if np is None:
            raise RuntimeError("numpy が入っていません")
        self.matrix = np.asarray(matrix, dtype=np.float64).reshape(3, 3)
        self.y_is_left = y_is_left
        self.frame = frame
        self.image_size = tuple(image_size) if image_size else None

    @classmethod
    def from_points(cls, image_points: Sequence[Sequence[float]],
                    floor_points: Sequence[Sequence[float]],
                    y_is_left: bool = True,
                    image_size: Optional[Tuple[int, int]] = None) -> "HomographyFloorPlane":
        """
        校正。画像で見た点と、メジャーで測った床座標 `(x, y)`（前 / 左）の対応から作る。

        4点以上。長方形でよい（設計書 5.4）。
        """
        cv2, np = OpenCv.cv2(), OpenCv.numpy()
        if cv2 is None or np is None:
            raise RuntimeError("OpenCV / numpy が入っていません")
        src = np.asarray(image_points, dtype=np.float64).reshape(-1, 1, 2)
        dst = np.asarray(floor_points, dtype=np.float64).reshape(-1, 1, 2)
        if len(src) < 4 or len(src) != len(dst):
            raise ValueError("画像と床の点は同じ数で、4点以上要る")
        matrix, _ = cv2.findHomography(src, dst, 0)
        if matrix is None:
            raise ValueError("ホモグラフィを作れなかった（4点が一直線に並んでいないか）")
        return cls(matrix, y_is_left=y_is_left, image_size=image_size)

    @classmethod
    def from_json(cls, obj: Dict[str, Any]) -> "HomographyFloorPlane":
        """
        `{"matrix": [[...]], "frame": "robot", "image_size": [1920, 1080]}`、
        または校正の点から作る。

        **`frame` を読み落とすと 'grid' の行列を走行体基準と取り違える**
        （位置の打ち直しが壊れる）。`Common/ground_plane.py` が書く欄に合わせてある。
        """
        y_is_left = bool(obj.get("y_is_left", True))
        frame = str(obj.get("frame", "robot") or "robot")
        size = obj.get("image_size")
        size = tuple(size) if size else None
        if "matrix" in obj:
            return cls(obj["matrix"], y_is_left=y_is_left, frame=frame, image_size=size)
        return cls.from_points(obj["image_points"], obj["floor_points"],
                               y_is_left=y_is_left, image_size=size)

    @classmethod
    def load(cls, path: str) -> "HomographyFloorPlane":
        with open(path, "r", encoding="utf-8") as handle:
            return cls.from_json(json.load(handle))

    def to_json(self) -> Dict[str, Any]:
        out = {"matrix": self.matrix.tolist(), "y_is_left": self.y_is_left,
               "frame": self.frame}
        if self.image_size:
            out["image_size"] = list(self.image_size)
        return out

    def _rescale(self, np: Any, points: Any, image_size: Any) -> Any:
        """
        校正と違う画素数で撮った点を、校正時の画素へ直す。

        **行列は校正した解像度に紐づいている。** 1920 で校正して 1280 の映像を
        渡すと、直さなければ床の座標が 1.5倍ずれる（`Common/ground_plane.py` と同じ直し方）。
        `image_size` を覚えていなければ直しようが無いので、そのまま通す。
        """
        if not self.image_size or image_size is None:
            return points
        scale_x = self.image_size[0] / float(image_size[0])
        scale_y = self.image_size[1] / float(image_size[1])
        if scale_x == 1.0 and scale_y == 1.0:
            return points
        return points * np.array([scale_x, scale_y])

    def to_floor(self, points: Any, image_size: Optional[Tuple[int, int]] = None) -> Any:
        cv2, np = OpenCv.cv2(), OpenCv.numpy()
        src = np.asarray(points, dtype=np.float64).reshape(-1, 2)
        src = self._rescale(np, src, image_size)
        src = src.reshape(-1, 1, 2)
        out = cv2.perspectiveTransform(src, self.matrix).reshape(-1, 2)
        if not self.y_is_left:
            out[:, 1] = -out[:, 1]
        return out


class GroundPlaneAdapter(FloorPlane):
    """
    校正する側が持っている床平面（`Common/ground_plane.py` の `GroundPlane`）を、
    この module の口に合わせる。

    あちらは `to_floor(points_px, image_size)` で、**未校正なら None を返す**。
    包んでおけば、校正が入るまでは `calibrated` が False のまま動く。

    :param ground_plane: `to_floor(points_px, image_size)` を持つもの。None でもよい
    :param y_is_left: あちらの2列目が「左」なら True
    """

    def __init__(self, ground_plane: Any, y_is_left: bool = True):
        self.ground_plane = ground_plane
        self.y_is_left = y_is_left

    @property
    def frame(self) -> str:
        """あちらが 'robot' か 'grid' を持っていればそれを使う"""
        return str(getattr(self.ground_plane, "frame", "robot") or "robot")

    @property
    def calibrated(self) -> bool:
        if self.ground_plane is None:
            return False
        flag = getattr(self.ground_plane, "calibrated", None)
        return True if flag is None else bool(flag)

    def to_floor(self, points: Any, image_size: Optional[Tuple[int, int]] = None) -> Any:
        np = OpenCv.numpy()
        if self.ground_plane is None:
            return None
        out = self.ground_plane.to_floor(points, image_size)
        if out is None:
            return None
        out = np.asarray(out, dtype=np.float64).reshape(-1, 2).copy()
        if not self.y_is_left:
            out[:, 1] = -out[:, 1]
        return out


# ---------------------------------------------------------------------------
# 設定
# ---------------------------------------------------------------------------

@dataclass
class Options:
    """測り方の設定。既定は実機の1枚（1280x720、コード約100px）に合わせてある"""

    #: 二値化のやり方。'clahe_otsu'（既定）/ 'otsu' / 'adaptive'
    #:
    #: 2026-09-22 の実機88枚（Lコース）で測った。**以前の既定（clip2.0/tile8）は
    #: 据え置きの近接画像で1枚も取れず、どの指標でも他に負けていた**ので替えた。
    #:
    #:     やり方                    据え置き8枚  走行80枚  1枚
    #:     clahe_otsu clip2.0/tile8      0/8     30/80   10.4ms  ← 以前の既定
    #:     **clahe_otsu clip1.5/tile24   5/8     40/80    5.6ms**  ← いまの既定
    #:     otsu                          5/8     29/80    2.1ms
    #:     adaptive block51 C7           5/8     44/80   39.4ms
    #:
    #: **`adaptive` は検出率で勝るが 39.4ms かかり、走行中には重い**
    #: （旋回の直後の直進は 1〜3秒しかなく、その間に何枚も撮りたい）。
    #: **角度が正しいかは別の話**で、実機の校正が要る（同じ画像でも答えが最大 8° 違う）。
    #: 決める道具は `Tools/check_floor_code_heading.py --compare`
    binarize_method: str = "clahe_otsu"

    #: CLAHE（明るさのむらを平らにする）。`binarize_method` が 'clahe_otsu' のときだけ効く。
    #: **強くかけると近接の画像で取れなくなる**（上の表）。窓は大きめにする
    clahe_clip: float = 1.5
    clahe_tiles: int = 24

    #: 適応二値のとき。窓の大きさ（奇数）と引く定数
    adaptive_block: int = 51
    adaptive_c: float = 7.0

    #: 入れ子の段数の下限。ファインダは 黒枠 → 白枠 → 黒の芯 で2段
    min_nesting: int = 2

    #: ファインダとして拾う面積[px^2]の下限と、画面に対する上限（紙全体を拾わないため）。
    #: **上限は 0.05 では狭い。** 実機の1枚（1モジュール46px）ではファインダ1個が
    #: 画面の 5% 近くを占め、3個のうち2個が落ちた。紛れ込みは入れ子の同心・面積比で弾く
    min_area_px: float = 36.0
    max_area_ratio: float = 0.30

    #: 四角とみなす近似の粗さ（周長に対する割合）と、辺の長さの比の上限
    quad_eps_ratio: float = 0.05
    max_side_ratio: float = 6.0

    #: 入れ子が同心かの許容（外枠の √面積 に対する割合）
    center_tol_ratio: float = 0.30
    #: 子・孫の面積が外枠に占める割合（理想は (5/7)^2=0.51 と (3/7)^2=0.18）
    child_area_range: Tuple[float, float] = (0.20, 0.85)
    grandchild_area_range: Tuple[float, float] = (0.03, 0.50)

    #: 同じコードのファインダとみなす距離（ファインダの床での一辺に対する割合）。
    #: 同じコードの中心どうしは 2.0（隣）〜2.83（対角）倍。隣のコードは 250mm = 15倍離れる
    group_radius_ratio: float = 3.4

    #: ファインダ候補の上限（多すぎる画像で総当たりしないため。面積の大きいものを残す）
    max_finders: int = 60

    #: コードでないものを弾く検査（緩め。使う側のゲートとは別）
    max_right_angle_error_deg: float = 20.0
    max_line_ratio: float = 1.6

    #: 1モジュールがこの画素数を切るコードを捨てる（`MODULE_PX_FLOOR` は 1.5）。
    #:
    #: **既定は 0 ＝ 捨てない。** ここは素の観測を返す約束なので、切るかどうかは使う側が決める。
    #: **切る前に、写りの悪いコードが本当に偏っているか確かめること。** 崖の近くは雑音が
    #: 大きいだけで偏ってはいないかもしれず、そのときは捨てるより `sigma_deg` の逆二乗で
    #: 重みを付けるほうが得（捨てると情報を捨てることになる）。
    #: 見分け方は `Tools/check_floor_code_heading.py --stability`
    #: （同じ絵の2枚の食い違いが毎回同じ向きなら偏り、ばらつくなら雑音）
    min_module_px: float = 0.0

    #: ファインダ中心の位置決め誤差[px]。精度の目安（sigma_deg）を出すのに使う。
    #:
    #: **実測は 0.48px**（D が 2026-08-29 の1枚で、床のコードを的にしたホモグラフィの
    #: 残差から。自由度で割ってある）。外周の四隅は 2.51px で、**中心のほうが 5.2倍よい**。
    #: ここはカメラのつまみを直す前の1枚なので、2倍の余裕を見て 1.0 にしてある。
    #: つまみを直した映像で測り直したら詰めること
    center_error_px: float = 1.0

    #: コードの上辺が、コース座標系でどちらを向いているか[deg]。マットで一度測る
    code_axis_course_deg: float = 0.0


# ---------------------------------------------------------------------------
# 検出の結果
# ---------------------------------------------------------------------------

@dataclass
class FinderPattern:
    """ファインダパターン1個（3重に入れ子になった四角）"""

    center: Tuple[float, float]
    corners: List[Tuple[float, float]]
    area_px: float
    side_px: float
    nesting: int


@dataclass
class CodeObservation:
    """床のコード1個から測った観測。**素の観測。ゲートは掛けていない**"""

    #: コース座標系での走行体の向き[deg]（= code_axis_course_deg - axis_deg）
    heading_deg: float
    #: コードの上辺が走行体から見てどちらを向いているか[deg]（右まわりが正）
    axis_deg: float
    #: コードの中心の床座標[mm]。x は前、y は左
    x_mm: float
    y_mm: float
    #: 精度の目安[deg]。**方向線に直交する向きの分解能**で決まる（`_sigma`）
    sigma_deg: float
    #: 使った方向線。'top'（左上→右上）か 'left'（左上→左下）。
    #: **直交する側が長く写っているほうを選ぶ**（その方向線自身が長いほうではない）
    direction_line: str
    #: 2本の方向線の画像での長さ[px]。(top, left)
    line_px: Tuple[float, float]
    #: 2本の方向線の床での長さ[mm]。(top, left)。理想は FINDER_SPACING_MM（33.3mm）
    line_mm: Tuple[float, float]
    #: 1モジュールが何画素で写っているか（**潰れているほうの向き**）。
    #: **1.5px を切るとファインダが立たなくなる**（D の実測。床を浅く見ると縦が潰れる）
    module_px: float
    #: 床座標で測った左上の角度[deg]。90 から離れていれば平面として怪しい
    right_angle_deg: float
    #: ファインダ3個の中心（画像 / 床）。左上・右上・左下の順
    centers_image: List[Tuple[float, float]]
    centers_floor: List[Tuple[float, float]]

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class FloorHeadingResult:
    """1枚から測った結果。**測れなかったことも値で返す（例外にしない）**"""

    ok: bool
    #: 測れなかった理由。測れていれば ''
    reason: str = ""
    #: 見えたコードごとの観測。近いほうが先（x_mm の小さい順）
    codes: List[CodeObservation] = field(default_factory=list)
    #: 拾えたファインダの数
    finders: int = 0
    #: ファインダ3個にまとまらなかった組の数
    skipped: int = 0
    #: 床平面が校正済みか。False なら値は当てにならない
    calibrated: bool = False
    #: 床座標の基準。'robot' なら x_mm/y_mm は走行体から見た 前/左。
    #: **'grid' のときは走行体基準ではない**（向きにも一定のずれが乗る）
    frame: str = "robot"
    elapsed_ms: float = 0.0
    width: int = 0
    height: int = 0

    @property
    def heading_deg(self) -> Optional[float]:
        """いちばん近いコードの向き[deg]。1個も無ければ None"""
        return self.codes[0].heading_deg if self.codes else None

    def to_dict(self) -> Dict[str, Any]:
        out = asdict(self)
        out["codes"] = [code.to_dict() for code in self.codes]
        return out


# ---------------------------------------------------------------------------
# ファインダパターンを探す
# ---------------------------------------------------------------------------

class FinderPatternFinder:
    """
    輪郭の入れ子から、ファインダパターン（3重に入れ子になった四角）を拾う。

    `cv2.QRCodeDetector.detect` の代わり。**実機の画質ではあちらは四隅を返さない**
    （`Docs/position_correction_design.md` 5.1）。

    1. CLAHE → 大津の二値化
    2. `findContours(RETR_TREE)` で入れ子を取る
    3. 子が2段以上ある四角で、入れ子が同心・面積の比がファインダらしいものを残す
    """

    def __init__(self, options: Optional[Options] = None):
        self.options = options or Options()

    def find(self, image: Any) -> List[FinderPattern]:
        """:returns: 見つけたファインダ（面積の大きい順）。取れなければ空"""
        cv2, np = OpenCv.cv2(), OpenCv.numpy()
        if cv2 is None or np is None or image is None:
            return []
        binary = self.binarize(image)
        if binary is None:
            return []
        contours, hierarchy = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if hierarchy is None or len(contours) == 0:
            return []
        tree = hierarchy[0]
        nesting = self._nesting(tree)
        opt = self.options
        max_area = binary.shape[0] * binary.shape[1] * opt.max_area_ratio
        found: List[FinderPattern] = []
        for index, contour in enumerate(contours):
            if nesting[index] < opt.min_nesting:
                continue
            quad = self._as_quad(cv2, contour)
            if quad is None:
                continue
            area = self._quad_area(quad)
            if area < opt.min_area_px or area > max_area:
                continue
            if not self._nested_like_finder(cv2, np, tree, contours, index, quad, area):
                continue
            center = self._contour_center(cv2, np, contour)
            if center is None:
                continue
            corners = [(float(p[0]), float(p[1])) for p in quad]
            found.append(FinderPattern(center=center, corners=corners,
                                       area_px=float(area), side_px=self._quad_side(corners),
                                       nesting=int(nesting[index])))
        found.sort(key=lambda f: f.area_px, reverse=True)
        return self._dedupe(found)[:opt.max_finders]

    def binarize(self, image: Any) -> Any:
        """
        白黒に分ける。灰色1枚を返す（デバッグでも見たいので分けてある）。

        やり方は `Options.binarize_method`。**実機の画像では既定が最良とは限らない**
        （`Options` のコメントに 2026-09-22 の実測がある）。
        """
        cv2, np = OpenCv.cv2(), OpenCv.numpy()
        if cv2 is None or np is None or image is None:
            return None
        array = np.asarray(image)
        gray = cv2.cvtColor(array, cv2.COLOR_BGR2GRAY) if array.ndim == 3 else array
        if gray.dtype != np.uint8:
            gray = np.clip(gray, 0, 255).astype(np.uint8)
        opt = self.options
        if opt.binarize_method == "adaptive":
            block = opt.adaptive_block | 1          # 窓は奇数でなければならない
            return cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                         cv2.THRESH_BINARY, block, opt.adaptive_c)
        if opt.binarize_method == "clahe_otsu":
            gray = cv2.createCLAHE(clipLimit=opt.clahe_clip,
                                   tileGridSize=(opt.clahe_tiles,) * 2).apply(gray)
        elif opt.binarize_method != "otsu":
            raise ValueError(f"知らない binarize_method: {opt.binarize_method}")
        _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        return binary

    @staticmethod
    def _nesting(tree: Any) -> List[int]:
        """輪郭ごとの「下に何段あるか」。[next, prev, first_child, parent] をたどる"""
        depth = [-1] * len(tree)

        def walk(index: int) -> int:
            if depth[index] >= 0:
                return depth[index]
            depth[index] = 0
            best = 0
            child = int(tree[index][2])
            while child != -1:
                best = max(best, 1 + walk(child))
                child = int(tree[child][0])
            depth[index] = best
            return best

        for index in range(len(tree)):
            walk(index)
        return depth

    def _as_quad(self, cv2: Any, contour: Any) -> Any:
        """四角に近似できれば4点を返す。できなければ None"""
        perimeter = cv2.arcLength(contour, True)
        if perimeter <= 0:
            return None
        approx = cv2.approxPolyDP(contour, self.options.quad_eps_ratio * perimeter, True)
        if len(approx) != 4 or not cv2.isContourConvex(approx):
            return None
        quad = approx.reshape(4, 2).astype(float)
        sides = [math.dist(quad[i], quad[(i + 1) % 4]) for i in range(4)]
        if min(sides) <= 1.0 or max(sides) / min(sides) > self.options.max_side_ratio:
            return None
        return quad

    def _nested_like_finder(self, cv2: Any, np: Any, tree: Any, contours: Any,
                            index: int, quad: Any, area: float) -> bool:
        """入れ子が同心で、面積の比がファインダらしいか"""
        opt = self.options
        center = self._contour_center(cv2, np, contours[index])
        if center is None:
            return False
        outer_center = np.asarray(center)
        tolerance = opt.center_tol_ratio * math.sqrt(area)
        current = index
        for low, high in (opt.child_area_range, opt.grandchild_area_range):
            current = self._largest_child(cv2, tree, contours, current)
            if current is None:
                return False
            inner_area = abs(cv2.contourArea(contours[current]))
            ratio = inner_area / area if area > 0 else 0.0
            if not low <= ratio <= high:
                return False
            moments = cv2.moments(contours[current])
            if moments["m00"] <= 0:
                return False
            inner_center = np.array([moments["m10"] / moments["m00"],
                                     moments["m01"] / moments["m00"]])
            if float(np.linalg.norm(inner_center - outer_center)) > tolerance:
                return False
        return True

    @staticmethod
    def _largest_child(cv2: Any, tree: Any, contours: Any, index: int) -> Optional[int]:
        """子のうち面積がいちばん大きいもの。子が無ければ None"""
        child = int(tree[index][2])
        best, best_area = None, -1.0
        while child != -1:
            area = abs(cv2.contourArea(contours[child]))
            if area > best_area:
                best, best_area = child, area
            child = int(tree[child][0])
        return best

    @staticmethod
    def _contour_center(cv2: Any, np: Any, contour: Any) -> Optional[Tuple[float, float]]:
        """
        ファインダの中心。**外枠の輪郭の重心を使う。**

        理屈では、斜めから見た正方形の中心は対角線の交点（射影で保たれる）で、重心は
        そこからずれる。**だが実測では重心のほうが 10倍正確だった**（偏り 0.231° 対
        0.023°、写りの大きさ 6〜46px・ぼけ 3〜7px の全条件で重心が良い）。

        対角線の交点は四隅に頼るが、**四隅は `approxPolyDP` が輪郭の頂点へ寄せた点**で、
        ぼけた斜めの四角では数px 単純に外れる。重心は輪郭の全画素をならすので、
        ぼけても位置がほとんど動かない。ファインダ1個の中では射影のゆがみが小さい
        （7モジュール＝約17mm）ので、重心のずれのほうが小さく収まる。

        **芯（内側の 3x3）の重心はさらに少し良いが、大きく写ると取れなくなる**ので採らない。
        """
        moments = cv2.moments(contour)
        if moments["m00"] <= 0:
            return None
        return (moments["m10"] / moments["m00"], moments["m01"] / moments["m00"])

    @staticmethod
    def _quad_area(corners: Sequence[Sequence[float]]) -> float:
        """四角の面積。`cv2.contourArea` は float64 を受け取らないので自分で出す"""
        total = 0.0
        for i in range(4):
            (x1, y1), (x2, y2) = corners[i], corners[(i + 1) % 4]
            total += x1 * y2 - x2 * y1
        return abs(total) / 2.0

    @staticmethod
    def _quad_side(corners: Sequence[Tuple[float, float]]) -> float:
        """四辺の長さの平均"""
        return sum(math.dist(corners[i], corners[(i + 1) % 4]) for i in range(4)) / 4.0

    @staticmethod
    def _dedupe(found: List[FinderPattern]) -> List[FinderPattern]:
        """同じファインダを二重に拾っていたら、大きいほうを残す"""
        kept: List[FinderPattern] = []
        for candidate in found:
            limit = 0.5 * candidate.side_px
            if any(math.dist(candidate.center, other.center) < limit for other in kept):
                continue
            kept.append(candidate)
        return kept


# ---------------------------------------------------------------------------
# 向きを出す
# ---------------------------------------------------------------------------

class FloorCodeHeading:
    """
    1枚の画像から、見えた床のコードごとに走行体の向きを出す。

    :param plane: 画像 → 床座標。省くと `IdentityFloorPlane`（結果の calibrated が False）
    :param options: 測り方の設定
    :param clock: 時間の出どころ（テストで差し替えられるように）
    """

    def __init__(self, plane: Optional[FloorPlane] = None, options: Optional[Options] = None,
                 clock: Callable[[], float] = time.monotonic):
        self.plane = plane or IdentityFloorPlane()
        self.options = options or Options()
        self.finder = FinderPatternFinder(self.options)
        self._clock = clock

    def measure(self, image: Any) -> FloorHeadingResult:
        """
        1枚測る。**測れなくても例外を投げず、ok=False と reason を返す。**

        :param image: BGR か灰色の1枚（numpy）
        """
        started = self._clock()
        result = FloorHeadingResult(ok=False, calibrated=self.plane.calibrated,
                                    frame=getattr(self.plane, "frame", "robot"))
        np = OpenCv.numpy()
        if np is None:
            result.reason = "numpy が入っていない"
            return result
        if image is None or getattr(image, "size", 0) == 0:
            result.reason = "映像が無い"
            return result
        result.height, result.width = int(image.shape[0]), int(image.shape[1])

        finders = self.finder.find(image)
        result.finders = len(finders)
        if len(finders) < 3:
            result.reason = f"ファインダが{len(finders)}個（3個で1コード）"
            result.elapsed_ms = self._elapsed(started)
            return result

        image_size = (result.width, result.height)
        centers_image = np.array([f.center for f in finders], dtype=np.float64)
        floor = self.plane.to_floor(centers_image, image_size)
        if floor is None:
            result.reason = "床平面が使えない（校正されていない）"
            result.elapsed_ms = self._elapsed(started)
            return result
        centers_floor = np.asarray(floor, dtype=np.float64)
        sides_floor = self._sides_on_floor(np, finders, image_size)

        # 1モジュールが何px か（潰れているほうの向き）。崖に近いかを知らせるため
        module_px = [min(math.dist(f.corners[i], f.corners[(i + 1) % 4]) for i in range(4))
                     / FINDER_MODULES for f in finders]

        codes: List[CodeObservation] = []
        for group in self._group(np, centers_floor, sides_floor):
            if len(group) != 3:
                result.skipped += 1
                continue
            code = self._observe(np, group, centers_image, centers_floor, module_px)
            if code is None:
                result.skipped += 1
                continue
            if self.options.min_module_px and code.module_px < self.options.min_module_px:
                result.skipped += 1
                continue
            codes.append(code)
        codes.sort(key=lambda c: c.x_mm)
        result.codes = codes
        result.ok = bool(codes)
        if not result.ok:
            # **校正前は、斜めから見た画素のままなので形の検査を通らない。**
            # 「コードが無い」と区別できるように理由を分ける
            result.reason = (f"床平面が校正されていない（ファインダは{len(finders)}個取れている）"
                             if not result.calibrated
                             else f"ファインダ{len(finders)}個がコードにまとまらなかった")
        result.elapsed_ms = self._elapsed(started)
        return result

    def _elapsed(self, started: float) -> float:
        return round((self._clock() - started) * 1000, 1)

    def _sides_on_floor(self, np: Any, finders: List[FinderPattern],
                        image_size: Tuple[int, int]) -> Any:
        """ファインダごとの、床での一辺の長さ[mm]。まとめ方の尺度に使う"""
        flat = np.asarray([corner for finder in finders for corner in finder.corners],
                          dtype=np.float64)
        floor = self.plane.to_floor(flat, image_size)
        if floor is None:
            return np.zeros(len(finders), dtype=np.float64)
        corners = np.asarray(floor, dtype=np.float64).reshape(len(finders), 4, 2)
        sides = [sum(float(np.linalg.norm(quad[i] - quad[(i + 1) % 4])) for i in range(4)) / 4.0
                 for quad in corners]
        return np.asarray(sides, dtype=np.float64)

    def _group(self, np: Any, centers: Any, sides: Any) -> List[List[int]]:
        """
        近いファインダをまとめる。同じコードの3個が1組になる。

        同じコードの中心どうしは一辺の 2.0〜2.83倍しか離れていない。隣のコードは
        250mm（一辺の約15倍）離れているので、間隔で切り分けられる。
        """
        count = len(centers)
        parent = list(range(count))

        def root(index: int) -> int:
            while parent[index] != index:
                parent[index] = parent[parent[index]]
                index = parent[index]
            return index

        for i in range(count):
            for j in range(i + 1, count):
                limit = self.options.group_radius_ratio * 0.5 * (sides[i] + sides[j])
                if limit <= 0:
                    continue
                if float(np.linalg.norm(centers[i] - centers[j])) <= limit:
                    parent[root(i)] = root(j)

        groups: Dict[int, List[int]] = {}
        for index in range(count):
            groups.setdefault(root(index), []).append(index)
        return list(groups.values())

    def _observe(self, np: Any, group: List[int], centers_image: Any,
                 centers_floor: Any, module_px: Any = None) -> Optional[CodeObservation]:
        """ファインダ3個から向きを出す。コードらしくなければ None"""
        opt = self.options
        floor = [centers_floor[i] for i in group]
        order = self._order_finders(np, floor)
        if order is None:
            return None
        top_left, top_right, bottom_left = (group[i] for i in order)

        v_top = centers_floor[top_right] - centers_floor[top_left]
        v_left = centers_floor[bottom_left] - centers_floor[top_left]
        length_top, length_left = float(np.linalg.norm(v_top)), float(np.linalg.norm(v_left))
        if length_top <= 0 or length_left <= 0:
            return None
        ratio = max(length_top, length_left) / min(length_top, length_left)
        if ratio > opt.max_line_ratio:
            return None
        right_angle = self._angle_between(np, v_top, v_left)
        if abs(right_angle - 90.0) > opt.max_right_angle_error_deg:
            return None

        # **直交する側が長く写っている方向線を使う**（その方向線自身が長いかではない）
        px_top = float(np.linalg.norm(centers_image[top_right] - centers_image[top_left]))
        px_left = float(np.linalg.norm(centers_image[bottom_left] - centers_image[top_left]))
        sigma_top = self._sigma(opt.center_error_px, px_left, length_top, length_left)
        sigma_left = self._sigma(opt.center_error_px, px_top, length_left, length_top)
        if sigma_top <= sigma_left:
            direction_line, sigma_deg, axis_deg = "top", sigma_top, self._bearing(v_top)
        else:
            direction_line, sigma_deg, axis_deg = "left", sigma_left, self._bearing(v_left) - 90.0
        axis_deg = self._wrap180(axis_deg)

        center = centers_floor[top_left] + 0.5 * (v_top + v_left)
        return CodeObservation(
            heading_deg=self._wrap180(opt.code_axis_course_deg - axis_deg),
            axis_deg=axis_deg,
            x_mm=float(center[0]), y_mm=float(center[1]),
            sigma_deg=sigma_deg,
            direction_line=direction_line,
            line_px=(round(px_top, 1), round(px_left, 1)),
            line_mm=(round(length_top, 2), round(length_left, 2)),
            module_px=round(min(module_px[i] for i in group), 2) if module_px else 0.0,
            right_angle_deg=round(right_angle, 2),
            centers_image=[tuple(float(v) for v in centers_image[i])
                           for i in (top_left, top_right, bottom_left)],
            centers_floor=[tuple(float(v) for v in centers_floor[i])
                           for i in (top_left, top_right, bottom_left)],
        )

    def _order_finders(self, np: Any, floor: List[Any]) -> Optional[Tuple[int, int, int]]:
        """
        3個を 左上・右上・左下 の順に並べる。

        **直角をなす頂点がコードの左上。** 残り2個は外積の符号で決まる（床を上から
        見るので手性が保たれる）。上辺から左辺へは**右まわりに 90度**回る。床座標は
        x が前・y が左なので、右まわりは外積が負。
        """
        angles = []
        for i in range(3):
            a, b = floor[(i + 1) % 3] - floor[i], floor[(i + 2) % 3] - floor[i]
            angles.append(self._angle_between(np, a, b))
        corner = min(range(3), key=lambda i: abs(angles[i] - 90.0))
        first, second = (corner + 1) % 3, (corner + 2) % 3
        v1, v2 = floor[first] - floor[corner], floor[second] - floor[corner]
        cross = float(v1[0] * v2[1] - v1[1] * v2[0])
        if abs(cross) < 1e-9:
            return None
        return (corner, first, second) if cross < 0 else (corner, second, first)

    @staticmethod
    def _angle_between(np: Any, a: Any, b: Any) -> float:
        """2本のベクトルの間の角[deg]（0〜180）"""
        norm = float(np.linalg.norm(a)) * float(np.linalg.norm(b))
        if norm <= 0:
            return 0.0
        cosine = max(-1.0, min(1.0, float(np.dot(a, b)) / norm))
        return math.degrees(math.acos(cosine))

    @staticmethod
    def _bearing(vector: Any) -> float:
        """床座標 (x, y)＝(前, 左) のベクトルの向き[deg]。真正面が 0、右まわりが正"""
        return math.degrees(math.atan2(-float(vector[1]), float(vector[0])))

    @staticmethod
    def _sigma(center_error_px: float, perpendicular_px: float,
               line_mm: float, perpendicular_mm: float) -> float:
        """
        その方向線で測ったときの、向きの不確かさの目安[deg]。

        **効くのは方向線の長さではなく、方向線に直交する向きの分解能。** 中心が ±1px ずれる
        と、方向線は直交方向へ「1px が何mmに当たるか」だけ振れ、その振れを方向線の長さで
        割ったものが角度の誤差になる。

            誤差[mm] = √2 × 誤差[px] × 直交する方向線の mm / 直交する方向線の px
            角度      = atan(誤差[mm] / 方向線の mm)

        **√2 は方向線の両端ぶん。** 中心は2個あって独立に振れるので、その差は √2 倍荒れる。
        入れる前は実測の RMS より一律に 1.4 倍小さく出ていた（±1px を入れた 400回×6条件）。

        斜めから見た床では、奥行き方向の 1px が横方向より何倍も大きな mm に化ける。
        そのため**横に長く写っている方向線ほど、直交する奥行きの粗さを受けて弱い**。
        設計書 5.4 の「画像で長く写っていたほうの腕を使う」（腕＝ここでいう方向線）は
        画像の中の角度の話で、
        床へ戻すと逆になる（合成画像に ±1px を入れて 400回×6条件、5条件で逆）。
        """
        if perpendicular_px <= 0 or line_mm <= 0:
            return 0.0
        spread_mm = math.sqrt(2.0) * center_error_px * perpendicular_mm / perpendicular_px
        return round(math.degrees(math.atan2(spread_mm, line_mm)), 3)

    @staticmethod
    def _wrap180(degrees: float) -> float:
        """-180 〜 180 に折り返す"""
        return round((degrees + 180.0) % 360.0 - 180.0, 4)
