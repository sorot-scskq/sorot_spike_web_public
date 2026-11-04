"""
前面カメラで ET ラリーのゲートを見つけ、いまの姿勢のまま進んで通れるかを測る。

    映像 ──▶ 色の塊を拾う ──▶ 柱らしいものだけ残す ──▶ 同じ色の2本を組にする
                                                            │
                          足元を床へ戻して（image_to_ground）中心・開き・ずれ

【足元しか使わない】
柱は立っているので、画素を床へ戻せるのは**床に接している足元だけ**。上のほうを
戻すと地平線の向こうになる。逆にこれが「立っているか」の見分けにもなる
（平らな床の印は、上端を戻しても足元のすぐ奥にしか行かない。`_stands_up`）。

【見える帯はカメラの幾何で決まる。数字を焼き込まない】
床が写るのは、画面の下端と上端が見ている床のあいだだけ。**カメラの幾何が変わるたびに
動く**ので、定数に書かず `visible_floor_band()` で毎回出す（幾何は 2026-09 に3度変わった）。

    高さ 205mm・俯角 25.8°・縦画角 31.9° なら **230〜1180mm**

通り抜ける直前は下端より近くなり、足元が画角から外れる。**測れるのは近づく途中だけ。**

【色は ボトルのラベルと同じ定義を使う】
ゲートも赤・青・黄なので `BottleColor.HUE_RANGES` をそのまま使う。しきい値だけ
別に持つ（ラベルは近くの面、柱は遠くの細い棒で、写り方が違うため）。
"""

from __future__ import annotations

import dataclasses
import math
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple

from bottle_color import BottleColor
from opencv import OpenCv

try:
    from grid_heading import CameraConstants
except ImportError:  # パッケージとして読み込まれた場合
    from .grid_heading import CameraConstants

#: 柱の中心どうしの間隔[mm]。**カメラが見るのは色のついた部分なので、
#: これは「色の高さでの芯々」**。`_best_pair` はこの値にいちばん近い組を選ぶ。
#:
#: **これと下の `FOOT_GAP_MM` は別の高さの話。混ぜてはいけない**（2026-09-23 夜に
#: 一度混ぜて 248mm と書き、矛盾で戻した）。裏付けは独立に2つ:
#:
#:   * 目印の2点解（俯角と色の始まりを同時に解く）が、258mm を既知として
#:     **色の始まり 45.2mm** を出した
#:   * 現地でメジャー: 色が始まる高さ = **45mm**（0.2mm 差で一致）
#:
#: 248mm（= 足元のすき間 230 + 管の外径 18）にすると、上の解が 51.4mm へ動いて
#: **メジャーと 6.4mm 食い違う**。足元には管ではなく**継手**があり、そこだけ太い
#: （両立に要る外径は 28.3mm。VP13 のソケットはおおむね 26〜28mm）。
#:
#: **未確定。** 色が始まる高さで芯々を直接測れば確定する（`Docs/gate_check.md`）
POST_SPAN_MM = 258.0

#: **足元のすき間[mm]。現地でメジャー**（2026-09-23）。
#:
#: **走行体が通るのはこの高さ。** カメラが見る色の高さ（芯々 258mm）とは別で、
#: 足元は継手があって内側へ張り出している。
#: **以前は芯々から `span/2 - 半径` で出しており、240mm と見積もって 10mm 甘かった。**
FOOT_GAP_MM = 230.0

#: 通れる開きの半分[mm]。**足元のすき間の半分**
HALF_OPENING_MM = FOOT_GAP_MM / 2.0

#: 管の半径[mm]。足元は柱の中心なので、通れる幅はここまで
POST_RADIUS_MM = 9.0

#: **柱の色のついた部分が床から始まる高さ[mm]。**
#:
#: 足元の継手は柱と別の色なので、色の塊の下端は床に接していない。そこを床だと思って
#: 逆投影すると、光線が奥まで伸びて**距離も横のずれも同じ倍率で膨らむ**。
#: 倍率は `高さ / (高さ - これ)` で、45mm なら 1.28倍。
#:
#: 2026-09-23 に実機で確定した。同じ青のゲートをメジャーで 500mm と 1000mm に置き、
#: **どちらも報告が 1.283倍**（642 / 1283mm）。距離によらない一定の倍率で、
#: 「床より上の物を床として延ばした」形そのものだった。
#:
#: **カメラの定数（高さ・俯角・画角）は動かしていない。** この 45mm はゲートのもので、
#: カメラのものではない。高さを 160mm にしても同じ数字は出るが、その定数は
#: 自己位置の打ち直しや床のホモグラフィと共有なので、巻き添えで狂う。
POST_COLOR_BASE_MM = 45.0

#: 走行体の半幅[mm]。**最大幅 135mm の半分**（`Docs/robot_dimensions.md`。2026-09-23 実測）。
#:
#: **`NXT_WHEEL_INTERVAL 112.0` と混同しないこと。** あれは車輪の中心間で、タイヤの幅も
#: 車体の張り出しも入っていない。当たり判定に使えるのは 135mm のほう。
#:
#: PC が経路を作るとき（ゲートの足を避ける判定）は 60.0 を使っている
#: （PC-System_2026 の `CreateRouteV2.ROBOT_HALF_WIDTH_MM`）。**あちらも直すこと。**
ROBOT_HALF_WIDTH_MM = 67.5

#: 車軸から後端まで[mm]。**傾いたときに開きを食うのはここ**（`Docs/robot_dimensions.md`）
ROBOT_REAR_MM = 220.0

#: 車軸から前端まで[mm]。後ろのほうが必ず大きいので余裕には効かないが、揃えて持つ
ROBOT_FRONT_MM = 80.0

#: レンズは車軸より何mm前か。`image_to_ground` が返すのはレンズからの距離なので、
#: 車軸からの距離にするにはこれを足す（`heading_reader.CAMERA_TO_AXLE_FORWARD_MM` と同値）
CAMERA_TO_AXLE_FORWARD_MM = 20.0


def foot_camera(camera: CameraConstants) -> CameraConstants:
    """
    柱の足元を測るためのカメラ。

    色の下端は床から `POST_COLOR_BASE_MM` 上にあるので、**その平面を「床」として
    逆投影する**。高さをその分だけ下げたカメラと同じことになる。
    """
    return dataclasses.replace(
        camera, eye_height_mm=camera.eye_height_mm - POST_COLOR_BASE_MM)


def gate_fits_from_mm(camera: CameraConstants) -> float:
    """
    **ゲート全体が画角の横に収まる最短の距離[mm]。**

    近づくほど柱は画面の端へ寄り、ある所で外へ出る。縦（床の見える帯）より
    こちらのほうが手前で効く。

    2026-09-23 の実機: 250mm では 449枚のうち 33枚（7%）しか組にならず、
    片方だけが見える枚が大半だった。1000mm では 449/449（100%）。
    いまの横画角 53.9° では **271mm**（芯々 258mm）。
    """
    return (POST_SPAN_MM / 2 + POST_RADIUS_MM) / math.tan(math.radians(camera.hfov_deg / 2))


def margin_mm(offset_mm: float, approach_deg: float,
              *, gap_mm: float = FOOT_GAP_MM) -> float:
    """
    開きの端までの余裕[mm]。負なら柱に当たる。**式はここ1か所**

    **測った間隔からは出さない。** 理由が2つある。

      * **高さが違う。** 走行体が通るのは足元で、カメラが測るのは色の高さ。
        足元には継手があって内側へ張り出している（230mm と 258mm の差）
      * **測りはぶれる。** 間隔の実測は1枚ごとに ±30mm ぶれるので、それで
        開きを決めると**余裕が甘いほうへ ±15mm の幻を持つ**（2026-09-23 の3走行）

    ゲートは動かない剛体なので、開きは既知の `FOOT_GAP_MM` でよい。
    """
    return (gap_mm / 2.0 - abs(offset_mm) - body_half_span_mm(approach_deg))


def visible_floor_band(camera: CameraConstants) -> Tuple[float, float]:
    """
    床が写る帯（レンズからの距離[mm]）。近いほうと遠いほう。

    画面の下端・上端が見ている床までの距離。**足元を測れるのはこの内側だけ**で、
    カメラの高さ・俯角・縦画角が変わると動く。

    :returns: (近いほう, 遠いほう)。上端が地平線より上なら遠いほうは inf
    """
    half = camera.vfov_deg / 2.0
    near_deg, far_deg = camera.tilt_deg + half, camera.tilt_deg - half
    near = camera.eye_height_mm / math.tan(math.radians(near_deg))
    if far_deg <= 0.01:
        return near, float('inf')        # 地平線が写っている
    return near, camera.eye_height_mm / math.tan(math.radians(far_deg))


def body_half_span_mm(approach_deg: float) -> float:
    """
    ゲートの線に沿って測った、車体の半分の張り出し[mm]。

    **まっすぐなら 67.5mm だが、傾くと後ろの角が食う。** 車軸が前寄り（前80／後220）
    なので、**1度あたり 前角 1.4mm / 後角 3.8mm** と後ろが 2.8倍 敏感
    （`Docs/robot_dimensions.md`）。

    通れる横ずれは「開きの半分 − これ」。0°で 52.5mm、6°で 29.9mm まで落ちる。
    """
    t = math.radians(abs(approach_deg))
    lengthwise = max(ROBOT_FRONT_MM, ROBOT_REAR_MM)
    return ROBOT_HALF_WIDTH_MM * math.cos(t) + lengthwise * math.sin(t)


@dataclass
class Post:
    """柱1本。足元が床のどこにあるか"""
    color: str
    #: 足元（レンズ直下から前へ[mm]、右へ[mm]）
    distance_mm: float
    lateral_mm: float
    #: 画素での足元と大きさ（デバッグ画像に描くため）
    foot_px: Tuple[float, float]
    bbox: Tuple[int, int, int, int]
    area_px: int


@dataclass
class GateSighting:
    """ゲート1つが見えたときの1回ぶんの測り"""
    color: str
    #: 2本の足元の中点まで（**車軸から**）の距離[mm]
    distance_mm: float
    #: ゲートの中心の横位置[mm]（走行体から見て右が正）。**寄せるのに使うのはこちら**
    #: （`offset_mm` はゲートの線に沿った量で、斜めだと 1/cos θ に育つ）
    lateral_mm: float
    #: 測った柱の間隔[mm]。POST_SPAN_MM から大きく外れていれば組が誤り
    span_mm: float
    #: まっすぐ進んだとき、ゲートの開きの中心からどれだけ横へ外れて通るか[mm]。
    #: 正なら中心より右寄り。ゲートと並んでしまい交わらなければ None
    offset_mm: Optional[float]
    #: 開きの端までの余裕[mm]。負なら**柱に当たる**
    margin_mm: Optional[float]
    #: 進む向きとゲートの法線のなす角[deg]。0 が真正面から直角に通る形
    approach_deg: Optional[float]
    posts: List[Post] = field(default_factory=list)

    @property
    def will_pass(self) -> Optional[bool]:
        """このまま進んで通れるか。測れなければ None"""
        return None if self.margin_mm is None else self.margin_mm >= 0.0

    def label(self) -> str:
        """絵に重ねる1行。**英数字だけ**（OpenCV は日本語を描けない）"""
        if self.offset_mm is None:
            return "%s d=%.0f span=%.0f (parallel)" % (self.color, self.distance_mm, self.span_mm)
        return "%s d=%.0f off=%+.0f margin=%+.0f approach=%+.1f span=%.0f" % (
            self.color, self.distance_mm, self.offset_mm, self.margin_mm,
            self.approach_deg or 0.0, self.span_mm)

    def summary(self) -> str:
        """ログ1行ぶん"""
        name = BottleColor.NAMES.get(self.color, self.color)
        if self.offset_mm is None:
            return "%s ゲート %.0fmm 先（ゲートと並んでいて通り抜けの位置が決まらない）" % (
                name, self.distance_mm)
        # **2つの「ずれ」を両方出す。** どちらか片方だと、読む側が取り違える
        # （寄せに使うのは横位置のほう。`gate_steer.py`）
        return ("%s ゲート %.0fmm 先 通る位置 中心から %+.0fmm・横位置 %+.0fmm"
                "・余裕 %+.0fmm・斜め %+.1f°（間隔 %.0fmm）") % (
            name, self.distance_mm, self.offset_mm, self.lateral_mm, self.margin_mm,
            self.approach_deg if self.approach_deg is not None else 0.0, self.span_mm)


@dataclass
class FindResult:
    """1枚ぶんの結果"""
    ok: bool
    reason: str = ""
    gates: List[GateSighting] = field(default_factory=list)
    #: 組にならなかった柱も返す（しきい値を現地で合わせるときに要る）
    posts: List[Post] = field(default_factory=list)
    elapsed_ms: float = 0.0
    width: int = 0
    height: int = 0

    @property
    def nearest(self) -> Optional[GateSighting]:
        """いちばん近いゲート"""
        return min(self.gates, key=lambda g: g.distance_mm) if self.gates else None


class GateFinder:
    """
    映像1枚から、写っているゲートを見つける。

    :param camera: カメラの定数（`heading_reader.ROBOT_CAMERA`）
    :param options: しきい値。既定は DEFAULT_OPTIONS
    """

    DEFAULT_OPTIONS: Dict[str, Any] = {
        #: 色として数えるのに要る彩度・明度（ラベル色の読み取りと同じ）
        'minSaturation': 0.35,
        'minValue': 0.18,
        #: 見る色
        'colors': [BottleColor.RED, BottleColor.BLUE, BottleColor.YELLOW],
        #: 柱1本とみなす塊の最小画素数（縮小後の画素で数える）。
        #: 1.1m の柱は 640x360 で およそ 7x30px = 200px。取りこぼさないよう低め
        'minPostPixels': 120,
        #: 縦横比の下限。柱は立っているので縦長に写る。床の印を弾く1つ目
        'minAspect': 1.5,
        #: 足元の上端を床へ戻したとき、足元より何mm奥へ行けば「立っている」とみなすか。
        #: 床の印は自分の大きさぶんしか奥へ行かない。**床の印を弾く2つ目**
        'minApparentDepthMm': 150.0,
        #: 足元を測れる距離[mm]の範囲。**None ならカメラの幾何から出す**
        #: （`visible_floor_band`）。数字を書くと幾何が変わったとき黙って外れる
        'minDistanceMm': None,
        'maxDistanceMm': None,
        #: 組にする柱の間隔[mm]の許容。258mm が正
        'minSpanMm': 200.0,
        'maxSpanMm': 330.0,
        #: 足元の左右位置を決めるのに使う、塊の下端から何行ぶんか
        'footRows': 4,
        #: 柱が1本しか見つからないとき、**相方が写るはずの所だけを見に行くか**。
        #: 258mm の拘束で場所は決まっているので、走行体を動かして探す必要は無い
        'partnerSearch': True,
        #: 相方を探すとき、斜めに入っている可能性を何度まで見込むか。
        #: 窓の大きさはここから決まる（遠いほど画素では小さくなる）
        'partnerApproachDeg': 20.0,
        #: 窓に足す余白[px]（縮小後）。足元の推定のばらつきぶん
        'partnerMarginPx': 8,
        #: 窓の中にその色が何画素あれば「居る」とみなすか
        'partnerMinPixels': 25,
        #: 処理する前に縮める倍率。1.0 なら縮めない。
        #: **速さのため。** 柱は 1.1m でも 640x360 で 7px 幅あるので見失わない
        'scale': 0.5,
    }

    def __init__(self, camera: CameraConstants, options: Optional[Dict[str, Any]] = None):
        self.camera = camera
        self.options = {**self.DEFAULT_OPTIONS, **(options or {})}
        near, far = visible_floor_band(foot_camera(camera))
        if self.options['minDistanceMm'] is None:
            # 横に収まるほうが手前で効く（近づくと柱が画面の外へ出る）
            self.options['minDistanceMm'] = max(near, gate_fits_from_mm(camera))
        if self.options['maxDistanceMm'] is None:
            self.options['maxDistanceMm'] = min(far, 2000.0)

    # -- 入り口 ---------------------------------------------------------------

    def find(self, frame: Any) -> FindResult:
        """
        1枚から、写っているゲートを全部返す。

        **例外を投げない。** 測れなければ ok=False で返す（走行を止めないため）。
        """
        started = time.perf_counter()
        cv2 = OpenCv.cv2()
        np = OpenCv.numpy()
        if cv2 is None or np is None:
            return FindResult(ok=False, reason="OpenCV が無い")
        if frame is None:
            return FindResult(ok=False, reason="映像が無い")

        try:
            image, camera = self._prepare(cv2, frame)
            hue, bright = self._hue_and_bright(cv2, np, image)
            posts = self._find_posts(cv2, np, hue, bright, camera)
            gates = self._pair_posts(cv2, np, hue, bright, camera, posts)
        except Exception as e:  # noqa: BLE001 — 測れなくても走行は続ける
            return FindResult(ok=False, reason="測れませんでした: %s" % e)

        elapsed = round((time.perf_counter() - started) * 1000, 1)
        result = FindResult(ok=bool(gates), gates=gates, posts=posts, elapsed_ms=elapsed,
                            width=camera.width, height=camera.height)
        if not gates:
            result.reason = "柱が2本そろわない（柱 %d本）" % len(posts)
        return result

    # -- 中身 -----------------------------------------------------------------

    def _prepare(self, cv2, frame) -> Tuple[Any, CameraConstants]:
        """縮めた画と、その大きさに合わせたカメラの定数"""
        height, width = frame.shape[:2]
        if frame.ndim == 3 and frame.shape[2] == 4:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        scale = float(self.options['scale'])
        if scale != 1.0:
            width, height = max(1, int(width * scale)), max(1, int(height * scale))
            frame = cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)
        return frame, foot_camera(self.camera).with_size(width, height)

    def _hue_and_bright(self, cv2, np, image):
        """色相[度]と「色として数えてよい明るさか」。

        **1枚につき1回だけ作る。** 色ごとに作り直すと、走行体では 1枚 57ms が
        3分の1の手間を3回払う形になる（実測 2026-09-22）。
        """
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        hue = hsv[:, :, 0].astype(np.int16) * 2          # OpenCV の H は 0〜179
        bright = ((hsv[:, :, 1] >= int(self.options['minSaturation'] * 255))
                  & (hsv[:, :, 2] >= int(self.options['minValue'] * 255)))
        return hue, bright

    @staticmethod
    def _color_mask(np, hue, bright, color: str):
        """その色の画素だけ立てた面"""
        inside = None
        for r in BottleColor.HUE_RANGES:
            if r['color'] != color:
                continue
            if r['from'] <= r['to']:
                inside = (hue >= r['from']) & (hue <= r['to'])
            else:
                inside = (hue >= r['from']) | (hue <= r['to'])   # 赤は 0度をまたぐ
        if inside is None:
            return None
        return (inside & bright).astype(np.uint8)

    def _find_posts(self, cv2, np, hue, bright, camera: CameraConstants) -> List[Post]:
        """色の塊のうち、柱らしいものだけを足元つきで返す"""
        posts: List[Post] = []
        for color in self.options['colors']:
            mask = self._color_mask(np, hue, bright, color)
            if mask is None:
                continue
            count, labels, stats, _centroids = cv2.connectedComponentsWithStats(mask, 8)
            for index in range(1, count):
                post = self._as_post(cv2, np, color, index, labels, stats, camera)
                if post is not None:
                    posts.append(post)
        return posts

    def _as_post(self, cv2, np, color: str, index: int, labels, stats,
                 camera: CameraConstants) -> Optional[Post]:
        """塊1つを柱として測る。柱でなければ None"""
        x = int(stats[index, cv2.CC_STAT_LEFT])
        y = int(stats[index, cv2.CC_STAT_TOP])
        w = int(stats[index, cv2.CC_STAT_WIDTH])
        h = int(stats[index, cv2.CC_STAT_HEIGHT])
        area = int(stats[index, cv2.CC_STAT_AREA])

        if area < self.options['minPostPixels']:
            return None
        if h < w * self.options['minAspect']:
            return None
        # 下端が画面の底に接していたら、足元は画角の下へ外れている（近すぎる）
        if y + h >= camera.height - 1:
            return None

        foot_u = self._foot_column(np, labels, index, x, y, w, h,
                                   int(self.options['footRows']))
        foot_v = float(y + h - 1)
        ground = camera.image_to_ground(foot_u, foot_v)
        if ground is None:
            return None
        distance, lateral = ground
        if not (self.options['minDistanceMm'] <= distance <= self.options['maxDistanceMm']):
            return None
        if not self._stands_up(camera, foot_u, float(y), distance):
            return None
        return Post(color=color, distance_mm=distance, lateral_mm=lateral,
                    foot_px=(foot_u, foot_v), bbox=(x, y, w, h), area_px=area)

    @staticmethod
    def _foot_column(np, labels, index: int, x: int, y: int, w: int, h: int,
                     foot_rows: int) -> float:
        """足元の左右位置[px]。塊の下のほうだけの重心を取る（枠の中心より当たる）"""
        rows = max(1, min(foot_rows, h))
        band = labels[y + h - rows:y + h, x:x + w] == index
        columns = np.nonzero(band.any(axis=0))[0]
        if columns.size == 0:
            return x + (w - 1) / 2.0
        return float(x + columns.mean())

    def _stands_up(self, camera: CameraConstants, foot_u: float, top_v: float,
                   foot_distance: float) -> bool:
        """
        立っているか。**上端を床へ戻して、足元よりずっと奥になれば立っている。**

        地平線の向こうなら（None）、それはもっと奥なので立っている。
        床に貼った印は、自分の大きさぶんしか奥へ行かない。
        """
        top = camera.image_to_ground(foot_u, top_v)
        if top is None:
            return True
        return (top[0] - foot_distance) >= self.options['minApparentDepthMm']

    def _pair_posts(self, cv2, np, hue, bright, camera: CameraConstants,
                    posts: List[Post]) -> List[GateSighting]:
        """同じ色の柱を2本ずつ組にする。間隔が 258mm に近い組を選ぶ"""
        gates: List[GateSighting] = []
        for color in self.options['colors']:
            same = [p for p in posts if p.color == color]
            pair = self._best_pair(same)
            if pair is None and len(same) == 1 and self.options['partnerSearch']:
                # **1本しか見えないときは、相方が写るはずの所を見に行く。**
                # どちら側かは推測しない。両側を見て、色があったほうを採る
                found = self._search_partner(cv2, np, hue, bright, camera, same[0])
                if found is not None:
                    pair = (same[0], found)
                    posts.append(found)
            if pair is not None:
                gates.append(self._measure(color, pair[0], pair[1]))
        return gates

    def _search_partner(self, cv2, np, hue, bright, camera: CameraConstants,
                        post: Post) -> Optional[Post]:
        """
        見えている柱から 258mm 離れた**両側**を見に行き、色があったほうを柱にする。

        **走行体を動かして探す必要は無い。** 相方の居場所は柱の間隔で決まっているので、
        画面のその1点だけを見ればよい（窓は `partnerWindowPx`）。

        なぜ要るか: 柱が1本だけでは、ゲートが左右どちらへ続くか分からない。
        一般の探索（`_find_posts`）で落ちるのは画素が足りないか形が崩れたときで、
        **場所が分かっているならその歯止めは要らない**。ここでは色があるかだけを見る。

        両側に色があれば決めない（ゲートでない何かが写っている）。
        """
        found = []
        for side in (-1.0, +1.0):
            box = self._partner_window(np, camera, post, side)
            if box is None:
                continue                      # 画面の外。ここでは決められない
            seen = self._foot_in_window(np, hue, bright, post.color, box, camera)
            if seen is not None:
                found.append(seen)
        if len(found) != 1:
            return None                       # 両側に有る／どちらにも無い

        count, foot_u, foot_v = found[0]
        # **位置は仮定せず、窓の中の画素から測る。** 仮定すると間隔が必ず 258mm に
        # なってしまい、`span` の検算（床へ戻す計算が正しいか）が効かなくなる
        ground = camera.image_to_ground(foot_u, foot_v)
        if ground is None:
            return None
        return Post(color=post.color, distance_mm=ground[0], lateral_mm=ground[1],
                    foot_px=(foot_u, foot_v),
                    bbox=(int(foot_u) - 1, int(foot_v) - 1, 2, 2), area_px=count)

    def _partner_window(self, np, camera: CameraConstants, post: Post,
                        side: float) -> Optional[Tuple[int, int, int, int]]:
        """
        相方が写りうる画素の範囲。

        **斜めに入っていると、相方は前後にずれる**（角度 θ で 258·sinθ）。その振れ幅を
        `partnerApproachDeg` まで見込んだ点をすべて投影し、その外接に余白を足す。
        距離が遠いほど画素では小さくなるので、窓も自動で縮む。
        """
        limit = math.radians(self.options['partnerApproachDeg'])
        spots = []
        for t in (-limit, 0.0, limit):
            d = post.distance_mm + side * POST_SPAN_MM * math.sin(t)
            l = post.lateral_mm + side * POST_SPAN_MM * math.cos(t)
            u, v, ok = camera.ground_to_image(np.array([d]), np.array([l]))
            if bool(ok[0]):
                spots.append((float(u[0]), float(v[0])))
        if not spots:
            return None
        pad = int(self.options['partnerMarginPx'])
        us = [p[0] for p in spots]
        vs = [p[1] for p in spots]
        return (max(0, int(min(us)) - pad), max(0, int(min(vs)) - pad),
                min(camera.width, int(max(us)) + pad + 1),
                min(camera.height, int(max(vs)) + pad + 1))

    def _foot_in_window(self, np, hue, bright, color: str,
                        box: Tuple[int, int, int, int],
                        camera: CameraConstants) -> Optional[Tuple[int, float, float]]:
        """
        窓の中から、その色の**足元**（いちばん下の行の重心）を返す。

        :returns: (画素数, 足元の u, 足元の v)。色が足りなければ None
        """
        x0, y0, x1, y1 = box
        if x0 >= x1 or y0 >= y1:
            return None
        mask = self._color_mask(np, hue[y0:y1, x0:x1], bright[y0:y1, x0:x1], color)
        if mask is None:
            return None
        count = int(np.count_nonzero(mask))
        if count < self.options['partnerMinPixels']:
            return None
        rows = np.nonzero(mask.any(axis=1))[0]
        bottom = int(rows[-1])
        columns = np.nonzero(mask[bottom])[0]
        return count, float(x0 + columns.mean()), float(y0 + bottom)

    def _best_pair(self, posts: Sequence[Post]) -> Optional[Tuple[Post, Post]]:
        """間隔が POST_SPAN_MM にいちばん近い組"""
        best, best_error = None, None
        for i in range(len(posts)):
            for j in range(i + 1, len(posts)):
                span = math.hypot(posts[i].distance_mm - posts[j].distance_mm,
                                  posts[i].lateral_mm - posts[j].lateral_mm)
                if not (self.options['minSpanMm'] <= span <= self.options['maxSpanMm']):
                    continue
                error = abs(span - POST_SPAN_MM)
                if best_error is None or error < best_error:
                    best, best_error = (posts[i], posts[j]), error
        return best

    @staticmethod
    def _measure(color: str, a: Post, b: Post) -> GateSighting:
        """
        2本の足元から、まっすぐ進んだときにどこを通るかを出す。

        走行体はレンズの真下から前へ伸びる線（左右 0）の上を進む。その線と
        柱を結ぶ線が交わる所が「通る点」で、開きの中心からのずれが offset_mm。
        """
        # 左にあるほうを起点にする（ずれの符号を「右が正」にそろえるため）
        left, right = (a, b) if a.lateral_mm <= b.lateral_mm else (b, a)
        ad, al = left.distance_mm, left.lateral_mm
        bd, bl = right.distance_mm, right.lateral_mm
        span = math.hypot(bd - ad, bl - al)
        center_d, center_l = (ad + bd) / 2.0, (al + bl) / 2.0

        # 距離は**車軸から**にする。測りはレンズからなので、前に出ているぶんを足す
        center_d += CAMERA_TO_AXLE_FORWARD_MM
        sighting = GateSighting(color=color, distance_mm=center_d, lateral_mm=center_l,
                                span_mm=span, offset_mm=None, margin_mm=None,
                                approach_deg=None, posts=[left, right])
        if span <= 0.0:
            return sighting

        # 柱を結ぶ線の単位ベクトル（左の柱 → 右の柱）
        ed, el = (bd - ad) / span, (bl - al) / span
        if abs(el) < 1e-6:
            return sighting          # ゲートと並んで走っている。交わらない

        # 左の柱から s だけ進んだ点の左右が 0 になる s
        s = -al / el
        if ad + s * ed <= 0.0:
            return sighting          # 交わるのは後ろ側

        sighting.offset_mm = s - span / 2.0
        # ゲートの法線と進む向き（前）のなす角。0 なら直角に通る
        sighting.approach_deg = math.degrees(math.atan2(ed, abs(el)))
        # **余裕は傾きで変わる。** まっすぐ 47.5mm が 6°で 24.9mm まで落ちる
        # **測った間隔ではなく、足元の既知のすき間で測る**（通るのはその高さ）
        sighting.margin_mm = margin_mm(sighting.offset_mm, sighting.approach_deg)
        return sighting
