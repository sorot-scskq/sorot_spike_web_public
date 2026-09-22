"""
前面カメラの 1枚から、ET ラリーのグリッドに対する走行体の向きのずれを測る。

走行体へ持っていけるよう、入力は「画像」と「カメラの定数」だけにしてある。
シミュレータにも走行体の他の処理にも依存しない（OpenCV と NumPy だけ）。

【どこで動くか】
  走行体          heading_reader.py から呼ぶ（カメラの 1枚を渡す）
  シミュレータ    sim/pyscript/grid_heading_bridge.py が PyScript で同じファイルを読む
  検証の道具      tools/camera-heading/heading_server.py（ブラウザを使わずに何千枚も回す）

検証の結果は docs/ETラリー_カメラで向きを補正する検証.md。

【何を見るか】
マットに印刷済みの 2種類の目印。どちらも 255mm 間隔の格子に並ぶ。
  - 灰色の丸（ゲートポジション、5x5、直径 約50mm）… 格子点
  - QR の四角（ゲート位置補助情報、4x4、50mm 四方）… 格子のマスの中心
  - 外周の黒線（東・西・南、幅 20mm）… グリッドの軸と平行。目印が足りないときの予備

【どう測るか】
  1. 画像を、地面を真上から見た図に引き直す（カメラの定数から 1画素ずつ写す）
  2. 白地より暗い、目印くらいの大きさの塊を拾い、重心を出す。
     色の付いたもの（ゲートの柱、色の丸）に触れている塊は捨てる
  3. 同じ種類の目印どうしで、255mm / 510mm 離れた組の向きから、おおよその回転を出す
  4. 全部の目印を「回転・拡大・平行移動した格子」に最小二乗で当てはめて詰める。
     格子から 20mm 以上外れる目印は外して当てはめ直す
  5. 真っ直ぐな黒線（長さ 150mm 以上。西の線は色の丸で 175〜210mm ごとに切れる）があれば、その傾きも出す
  6. 目印で測れればそれを使う。黒線でも測れて 1°以上食い違うときは「測れない」にする。
     目印で測れず黒線だけで測れれば、黒線の値を使う
  7. どちらでも測れないときは「測れない」を返す

【出す値の向き】
deviation_deg は「走行体の向き − いちばん近いグリッドの向き（45度刻み）」。
シミュレータの robot.angle と同じ回り方（ワールド座標の x→y、画面では時計回り）。
直すには次の旋回の角度に -deviation_deg を足す。

使い方
    python src/RoughSpot/Python/2026/grid_heading.py image.png --camera camera.json
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import asdict, dataclass, field
from pathlib import Path

import cv2
import numpy as np

# 目印の格子の間隔[mm]（競技規約。灰色の丸も QR の四角も同じ）
GRID_STEP_MM = 255.0
# 目印の大きさ[mm]
MARKER_SIZE_MM = 50.0


# ---------------------------------------------------------------------------
# カメラ
# ---------------------------------------------------------------------------

@dataclass
class CameraConstants:
    """
    カメラの定数。地面（高さ 0）の点と画面の画素を行き来するのに要るものだけ。

    model
      'sim'     … シミュレータの疑似投影。縦はピンホール、横は「距離に比例した幅」
                  （sim/camera-model.js）
      'pinhole' … 実機のカメラ。俯角だけ傾けたピンホール。レンズの歪みは
                  先に cv2.undistort で取っておくこと
    """
    width: int
    height: int
    hfov_deg: float
    vfov_deg: float
    eye_height_mm: float
    tilt_deg: float
    model: str = 'sim'
    # pinhole のときだけ。省くと画角と画面中心から決める
    fx: float | None = None
    fy: float | None = None
    cx: float | None = None
    cy: float | None = None

    @classmethod
    def from_json(cls, obj: dict) -> 'CameraConstants':
        keys = cls.__dataclass_fields__.keys()
        return cls(**{k: v for k, v in obj.items() if k in keys})

    def with_size(self, width: int, height: int) -> 'CameraConstants':
        """
        同じカメラを、別の画素数で撮ったときの定数。

        画角・俯角・高さは変わらない。画素で持つ fx fy cx cy だけ比で直す。
        縮めた画像を渡すときや、映像の大きさが定数と違うときに使う。
        """
        if width == self.width and height == self.height:
            return self
        sx, sy = width / self.width, height / self.height
        scale = lambda v, k: None if v is None else v * k  # noqa: E731
        return CameraConstants(
            width=width, height=height, hfov_deg=self.hfov_deg, vfov_deg=self.vfov_deg,
            eye_height_mm=self.eye_height_mm, tilt_deg=self.tilt_deg, model=self.model,
            fx=scale(self.fx, sx), fy=scale(self.fy, sy),
            # 画素の中心は (n-1)/2 なので、0.5 ずらしてから比を掛けて戻す
            cx=None if self.cx is None else (self.cx + 0.5) * sx - 0.5,
            cy=None if self.cy is None else (self.cy + 0.5) * sy - 0.5,
        )

    # 地面 → 画面 ------------------------------------------------------------
    def ground_to_image(self, d: np.ndarray, l: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        レンズ直下から前へ d[mm]、右へ l[mm] の地面の点が写る画素 (u, v)。
        3つ目は「画面に写るか」。
        """
        t = math.radians(self.tilt_deg)
        h = self.eye_height_mm
        if self.model == 'sim':
            f = (self.height / 2) / math.tan(math.radians(self.vfov_deg) / 2)
            tan_h = math.tan(math.radians(self.hfov_deg) / 2)
            with np.errstate(divide='ignore', invalid='ignore'):
                u = (l / (2 * d * tan_h) + 0.5) * self.width - 0.5
                v = self.height / 2 + f * np.tan(np.arctan(h / d) - t)
            ok = d > 1.0
        elif self.model == 'pinhole':
            fx = self.fx or (self.width / 2) / math.tan(math.radians(self.hfov_deg) / 2)
            fy = self.fy or (self.height / 2) / math.tan(math.radians(self.vfov_deg) / 2)
            cx = self.cx if self.cx is not None else (self.width - 1) / 2
            cy = self.cy if self.cy is not None else (self.height - 1) / 2
            zc = d * math.cos(t) + h * math.sin(t)
            yc = h * math.cos(t) - d * math.sin(t)
            with np.errstate(divide='ignore', invalid='ignore'):
                u = cx + fx * l / zc
                v = cy + fy * yc / zc
            ok = zc > 1.0
        else:
            raise ValueError(f'知らないカメラの model: {self.model}')
        ok = ok & (u >= 0) & (u <= self.width - 1) & (v >= 0) & (v <= self.height - 1)
        return u, v, ok

    def image_to_ground(self, u: float, v: float) -> tuple[float, float] | None:
        """画素 (u, v) が見ている地面の点 (d, l)[mm]。地平線より上なら None"""
        t = math.radians(self.tilt_deg)
        h = self.eye_height_mm
        if self.model == 'sim':
            f = (self.height / 2) / math.tan(math.radians(self.vfov_deg) / 2)
            down = t + math.atan((v - self.height / 2) / f)
            if down <= 1e-4:
                return None
            d = h / math.tan(down)
            tan_h = math.tan(math.radians(self.hfov_deg) / 2)
            return d, ((u + 0.5) / self.width - 0.5) * 2 * d * tan_h
        fx = self.fx or (self.width / 2) / math.tan(math.radians(self.hfov_deg) / 2)
        fy = self.fy or (self.height / 2) / math.tan(math.radians(self.vfov_deg) / 2)
        cx = self.cx if self.cx is not None else (self.width - 1) / 2
        cy = self.cy if self.cy is not None else (self.height - 1) / 2
        down = t + math.atan((v - cy) / fy)
        if down <= 1e-4:
            return None
        # 光線 (x, y, 1) を地面と交わるまで伸ばす
        xn, yn = (u - cx) / fx, (v - cy) / fy
        # カメラ座標の光線を、前・下の成分へ
        fwd = math.cos(t) - yn * math.sin(t)
        dwn = math.sin(t) + yn * math.cos(t)
        s = h / dwn
        return s * fwd, s * xn


# ---------------------------------------------------------------------------
# 結果
# ---------------------------------------------------------------------------

@dataclass
class HeadingResult:
    ok: bool
    """測れたか"""
    deviation_deg: float | None = None
    """走行体の向き − いちばん近いグリッドの向き[度]"""
    system: int | None = None
    """いちばん近いグリッドの向きが 0度系（0/90/180/270）か 45度系か"""
    sigma_deg: float | None = None
    """当てはめから見積もった deviation のばらつき[度]"""
    markers: int = 0
    """当てはめに使った目印の数"""
    rejected: int = 0
    """格子から外れて捨てた目印の数"""
    residual_mm: float | None = None
    """目印の格子からの外れの二乗平均[mm]"""
    reason: str = ''
    """測れなかった理由"""
    source: str = ''
    """何で測ったか。'markers'（目印）/ 'line'（黒線）"""
    line_deviation_deg: float | None = None
    """黒線だけで測った値（黒線が見えたとき）"""
    points: list = field(default_factory=list)
    """使った目印（d, l[mm], 種類）。確かめる用"""


@dataclass
class Options:
    # 真上から見た図の範囲と細かさ
    d_min_mm: float = 150.0
    d_max_mm: float = 1100.0
    lateral_mm: float = 800.0
    res_mm: float = 2.5
    supersample: int = 3
    # 当てはめの合格ライン
    min_markers: int = 3
    max_residual_mm: float = 8.0
    max_sigma_deg: float = 0.6
    outlier_mm: float = 20.0
    # 黒線
    use_lines: bool = True
    line_min_len_mm: float = 150.0
    line_max_width_mm: float = 32.0
    line_max_rms_mm: float = 0.1
    line_min_px: float = 3.0
    max_line_disagree_deg: float = 1.0


# ---------------------------------------------------------------------------
# 本体
# ---------------------------------------------------------------------------

_map_cache: dict = {}


def _ground_maps(cam: CameraConstants, opt: Options):
    """真上から見た図の各画素が、カメラ画面のどこを読むか。カメラと範囲ごとに 1回だけ作る"""
    key = (tuple(asdict(cam).items()), opt.d_min_mm, opt.d_max_mm, opt.lateral_mm, opt.res_mm, opt.supersample)
    if key in _map_cache:
        return _map_cache[key]
    rows = int(round((opt.d_max_mm - opt.d_min_mm) / opt.res_mm))
    cols = int(round(2 * opt.lateral_mm / opt.res_mm))
    ss = opt.supersample
    fine = opt.res_mm / ss
    # 行 0 が奥（d_max）、列 0 が左（-lateral）。細かく作ってから縮める
    d = opt.d_max_mm - (np.arange(rows * ss) + 0.5) * fine
    l = -opt.lateral_mm + (np.arange(cols * ss) + 0.5) * fine
    dd, ll = np.meshgrid(d, l, indexing='ij')
    u, v, ok = cam.ground_to_image(dd, ll)
    map_u = np.where(ok, u, -1).astype(np.float32)
    map_v = np.where(ok, v, -1).astype(np.float32)
    # 縮めた図の 1画素が、細かい画素すべて写っているときだけ「写っている」とする
    ok_small = cv2.resize(ok.astype(np.float32), (cols, rows), interpolation=cv2.INTER_AREA) > 0.999
    _map_cache[key] = (map_u, map_v, ok_small, (cols, rows))
    return _map_cache[key]


def birdseye(image_bgr: np.ndarray, cam: CameraConstants, opt: Options | None = None):
    """
    画像を、地面を真上から見た図にする。戻りは (図, 写っている所の印, None, None)。

    【細かく作ってから縮める理由】
    足元の地面はカメラの画素が 0.3mm ほどと細かく、図の 1画素（2.5mm）に 1点だけ拾うと
    線や目印の縁が 2.5mm 刻みに丸まる。長さ 200mm の線なら 0.7° の刻みになる。
    supersample 倍の細かさで拾って面積で平均し、縁の位置を濃淡として残す。
    """
    opt = opt or Options()
    map_u, map_v, ok, size = _ground_maps(cam, opt)
    fine = cv2.remap(image_bgr, map_u, map_v, cv2.INTER_LINEAR,
                     borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0))
    top = cv2.resize(fine, size, interpolation=cv2.INTER_AREA) if opt.supersample > 1 else fine
    return top, ok, None, None


def find_markers(top_bgr: np.ndarray, valid: np.ndarray, opt: Options):
    """
    真上から見た図から目印を拾う。

    戻りは [(行, 列, 種類)] の重心（図の画素）。種類は 'qr'（濃い灰）か 'dot'（薄い灰）。
    """
    res = opt.res_mm
    gray = cv2.cvtColor(top_bgr, cv2.COLOR_BGR2GRAY).astype(np.float32)
    hsv = cv2.cvtColor(top_bgr, cv2.COLOR_BGR2HSV)
    # 白地の明るさ。目印より大きい窓で膨らませてから縮める（目印を消した地の色）
    k = int(2.2 * MARKER_SIZE_MM / res) | 1
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
    background = cv2.morphologyEx(gray, cv2.MORPH_CLOSE, kernel)
    diff = background - gray

    # 地が白くないところ（緑のフィールド、コース外の塗り、写っていない所）は使わない
    bright_bg = (background > 170)
    colored = (hsv[:, :, 1] > 70) & (hsv[:, :, 2] > 40)
    bad = (~valid) | (~bright_bg) | colored
    bad = cv2.dilate(bad.astype(np.uint8), np.ones((5, 5), np.uint8)) > 0

    cand = ((diff > 14) & ~bad).astype(np.uint8)
    # QR の四角は細かいマスの集まりなので、つないで 1つの塊にする
    close = max(3, int(round(12 / res)) | 1)
    cand = cv2.morphologyEx(cand, cv2.MORPH_CLOSE, np.ones((close, close), np.uint8))

    n, labels, stats, _ = cv2.connectedComponentsWithStats(cand)
    area_mm2 = MARKER_SIZE_MM ** 2
    out = []
    H, W = gray.shape
    bad_near = cv2.dilate(bad.astype(np.uint8), np.ones((7, 7), np.uint8))
    for i in range(1, n):
        x, y, w, h, a = stats[i]
        a_mm2 = a * res * res
        if not (0.45 * area_mm2 <= a_mm2 <= 1.6 * area_mm2):
            continue
        if max(w, h) * res > 1.6 * MARKER_SIZE_MM or min(w, h) * res < 0.55 * MARKER_SIZE_MM:
            continue
        if x <= 1 or y <= 1 or x + w >= W - 1 or y + h >= H - 1:
            continue
        comp = labels[y:y + h, x:x + w] == i
        # 写っていない所や色の付いた物に接していたら、欠けているかもしれないので捨てる
        if (bad_near[y:y + h, x:x + w][comp]).any():
            continue
        # 形。四角でも丸でも凸なので、凸包に対してほぼ埋まっているはず
        cnts, _ = cv2.findContours(comp.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        hull = cv2.convexHull(cnts[0])
        if cv2.contourArea(hull) <= 0 or a / cv2.contourArea(hull) < 0.8:
            continue
        wgt = np.clip(diff[y:y + h, x:x + w], 0, None) * comp
        s = wgt.sum()
        if s <= 0:
            continue
        yy, xx = np.mgrid[y:y + h, x:x + w]
        mean_diff = wgt.sum() / comp.sum()
        # 灰色の丸は白地との差が小さい、QR は大きい
        kind = 'qr' if mean_diff > 45 else 'dot'
        out.append(((yy * wgt).sum() / s, (xx * wgt).sum() / s, kind))
    return out


def _refine_line(image_bgr, cam: CameraConstants, mean, direction, normal, s0, s1, width):
    """
    線の周りの地面を細かく取り直し、長さ方向の各所で太さ方向の中心を出して直線を当てる。
    戻りは (傾き k[太さ方向/長さ方向], 中心の外れの二乗平均[mm]) か None
    """
    if s1 - s0 < 60:
        return None
    step = 0.5
    ss = np.arange(s0, s1, step)
    tt = np.arange(-(width / 2 + 12), width / 2 + 12, step)
    S, T = np.meshgrid(ss, tt, indexing='ij')
    d = mean[0] + S * direction[0] + T * normal[0]
    l = mean[1] + S * direction[1] + T * normal[1]
    u, v, ok = cam.ground_to_image(d, l)
    # 帯の断面がまるごと画面に入っている所だけ使う（線が画面の縁で切れていることがある）
    row_ok = ok.all(axis=1)
    if row_ok.sum() * step < 60:
        return None
    ss, u, v = ss[row_ok], u[row_ok], v[row_ok]
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    patch = cv2.remap(gray, u.astype(np.float32), v.astype(np.float32), cv2.INTER_LINEAR).astype(np.float32)
    # 白地を 0、黒を 1 に。重みは覆われ具合に比例させる
    white = np.percentile(patch, 95)
    wgt = np.clip((white - patch) / max(white - patch.min(), 1), 0, None)
    # 8mm ごとにまとめる
    n = int(8 / step)
    rows = (len(ss) // n) * n
    if rows < 5 * n:
        return None
    W = wgt[:rows].reshape(-1, n, len(tt)).sum(axis=1)
    centers = (W * tt).sum(axis=1) / np.maximum(W.sum(axis=1), 1e-9)
    xs = ss[:rows].reshape(-1, n).mean(axis=1)
    # 途中が画面から外れて飛んでいる所をまとめると、8mm の中に離れた点が混ざる。広がりで捨てる
    spread = np.ptp(ss[:rows].reshape(-1, n), axis=1)
    keep = spread < 8 + 1e-6
    if keep.sum() < 5:
        return None
    xs, centers = xs[keep], centers[keep]
    k, b = np.polyfit(xs, centers, 1)
    rms = float(np.sqrt(np.mean((centers - (k * xs + b)) ** 2)))
    return float(k), rms


def find_lines(top_bgr: np.ndarray, valid: np.ndarray, opt: Options, image_bgr: np.ndarray, cam: CameraConstants):
    """
    真っ直ぐな黒線を拾い、その傾きを出す。

    戻りは [(角度[rad], 長さ[mm], 太さ方向の外れの二乗平均[mm])]。角度は走行体から見た
    (前 d, 右 l) の向き。黒線の縁で途切れた短い線や、曲がった線は捨てる。
    """
    res = opt.res_mm
    gray = cv2.cvtColor(top_bgr, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(top_bgr, cv2.COLOR_BGR2HSV)
    # 黒に近いと彩度が大きく出る（V が小さいと S の分母が小さい）ので、暗いものは色を問わない
    dark = (gray < 90) & ((hsv[:, :, 1] < 90) | (hsv[:, :, 2] < 60)) & valid
    white = (gray > 170) & (hsv[:, :, 1] < 60)
    # 写っている所の縁で黒く見える画素（remap の外側）を拾わない
    inner = cv2.erode(valid.astype(np.uint8), np.ones((5, 5), np.uint8)) > 0
    dark = (dark & inner).astype(np.uint8)
    n, labels, stats, _ = cv2.connectedComponentsWithStats(dark)
    lines = []
    for i in range(1, n):
        x, y, w, h, a = stats[i]
        if max(w, h) * res < opt.line_min_len_mm * 0.7:
            continue
        ys, xs = np.nonzero(labels[y:y + h, x:x + w] == i)
        # 図の画素 → (d, l)[mm]
        d = opt.d_max_mm - (ys + y + 0.5) * res
        l = -opt.lateral_mm + (xs + x + 0.5) * res
        pts = np.stack([d, l], axis=1)
        mean = pts.mean(axis=0)
        cov = np.cov((pts - mean).T)
        evals, evecs = np.linalg.eigh(cov)
        direction = evecs[:, 1]
        along = (pts - mean) @ direction
        across = (pts - mean) @ evecs[:, 0]
        length = float(along.max() - along.min())
        width = float(4 * math.sqrt(max(evals[0], 0)))  # 一様な帯なら幅 ≈ √12·σ ≈ 3.5σ
        if length < opt.line_min_len_mm or width > opt.line_max_width_mm:
            continue
        # 線の太さがカメラの画像で 3画素に満たない所は、太さ方向の中心が出せない（240x144 の
        # 映像で 1°を超えて外れた）。線の画素のうちいちばん遠い点で確かめる
        far_end = pts[np.argmax(pts[:, 0])]
        u0, v0, _ = cam.ground_to_image(np.array(far_end[0]), np.array(far_end[1]))
        px = 0.0
        for sgn in (-1.0, 1.0):
            u1, v1, _ = cam.ground_to_image(np.array(far_end[0] + sgn * evecs[0, 0] * 20.0),
                                            np.array(far_end[1] + sgn * evecs[1, 0] * 20.0))
            px = max(px, math.hypot(float(u1 - u0), float(v1 - v0)))
        if not px >= opt.line_min_px:
            continue
        # 傾きは、線の周りだけをカメラ画像から 0.5mm 刻みで取り直して出す。
        # 図（2.5mm/画素）の画素で出すと、長さ 200mm ほどの線では縁の位置が画素に
        # 丸まって 0.3〜0.5° ずれた（足元はカメラの画素が 0.3mm ほどと細かいのに、図が粗い）
        refined = _refine_line(image_bgr, cam, mean, direction, evecs[:, 0],
                               along.min() + max(width, 10.0), along.max() - max(width, 10.0), width)
        if refined is None:
            continue
        k, rms = refined
        if rms > opt.line_max_rms_mm:
            continue
        # 傾き k は「長さ方向に 1 進むと法線方向に k ずれる」。法線が長さ方向の左右どちらを
        # 向いているかは固有ベクトルの符号しだいなので、角度を足さずにベクトルで合わせる
        fitted = direction + k * evecs[:, 0]
        angle = math.atan2(fitted[1], fitted[0])
        # 白地に引かれた線か。両側 25mm の所が、長さのほとんどで白地であること。
        # 緑地の上の文字（「ETロボコン実行委員会」）が並んで線に見えるのを捨てる
        t = np.arange(along.min(), along.max(), 5.0)
        normal = evecs[:, 0]
        both_white = True
        for side in (-1, 1):
            q = mean + np.outer(t, direction) + side * (width / 2 + 25.0) * normal
            rows = np.round((opt.d_max_mm - q[:, 0]) / res - 0.5).astype(int)
            cols = np.round((q[:, 1] + opt.lateral_mm) / res - 0.5).astype(int)
            inside = (rows >= 0) & (rows < gray.shape[0]) & (cols >= 0) & (cols < gray.shape[1])
            inside[inside] &= valid[rows[inside], cols[inside]]
            if inside.sum() < 0.6 * len(t) or white[rows[inside], cols[inside]].mean() < 0.8:
                both_white = False
        if not both_white:
            continue
        lines.append((angle, length, rms))
    return lines


def _wrap(a: float, period: float) -> float:
    """a を (-period/2, period/2] へ"""
    return (a + period / 2) % period - period / 2


def _initial_rotation(pts: np.ndarray, kinds: list[str]):
    """同じ種類の目印どうしで、1〜2マス離れた組の向きから回転のおおよそを出す"""
    acc = 0j
    count = 0
    for kind in ('qr', 'dot'):
        idx = [i for i, k in enumerate(kinds) if k == kind]
        for a in range(len(idx)):
            for b in range(a + 1, len(idx)):
                v = pts[idx[b]] - pts[idx[a]]
                length = math.hypot(*v)
                for m in (1, 2):
                    if abs(length - m * GRID_STEP_MM) < 0.08 * m * GRID_STEP_MM:
                        # 4倍角で平均すると、90度違いの向きが同じ所へ重なる
                        acc += length ** 2 * complex(math.cos(4 * math.atan2(v[1], v[0])),
                                                     math.sin(4 * math.atan2(v[1], v[0])))
                        count += 1
    if count == 0:
        return None, 0
    return math.atan2(acc.imag, acc.real) / 4, count


def _fit_lattice(pts: np.ndarray, kinds: list[str], theta0: float, outlier_mm: float):
    """
    目印を格子に当てはめる。

    走行体から見た点 p を、グリッドの座標へ q = s·R(φ)·p + t と写したとき、
    丸は格子点 (i, j)·step、QR はマスの中心 (i+½, j+½)·step に乗るはず。
    φ, s, t を Gauss-Newton で詰める。φ が求める回転。
    """
    offs = np.array([0.5 if k == 'qr' else 0.0 for k in kinds])
    phi, s = theta0, 1.0
    # 平行移動の初期値。回転だけ戻した点の、格子に対する端数の円周平均
    rot = lambda a: np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
    q = pts @ rot(-phi).T
    t = np.zeros(2)
    for ax in range(2):
        frac = (q[:, ax] / GRID_STEP_MM - offs) * 2 * math.pi
        c = np.exp(1j * frac).mean()
        t[ax] = -math.atan2(c.imag, c.real) / (2 * math.pi) * GRID_STEP_MM
    use = np.ones(len(pts), bool)
    for _ in range(3):
        for _ in range(8):
            R = rot(-phi)
            q = s * pts @ R.T + t
            node = (np.round(q / GRID_STEP_MM - offs[:, None]) + offs[:, None]) * GRID_STEP_MM
            r = (q - node)[use]
            # ヤコビアン（φ, s, tx, ty）
            dR = np.array([[-math.sin(-phi), -math.cos(-phi)], [math.cos(-phi), -math.sin(-phi)]]) * -1
            p = pts[use]
            J = np.zeros((2 * len(p), 4))
            J[:, 0] = (s * p @ dR.T).reshape(-1)
            J[:, 1] = (p @ R.T).reshape(-1)
            J[0::2, 2] = 1
            J[1::2, 3] = 1
            step, *_ = np.linalg.lstsq(J, -r.reshape(-1), rcond=None)
            phi += step[0]
            s += step[1]
            t += step[2:]
            if abs(step[0]) < 1e-7:
                break
        q = s * pts @ rot(-phi).T + t
        node = (np.round(q / GRID_STEP_MM - offs[:, None]) + offs[:, None]) * GRID_STEP_MM
        err = np.hypot(*(q - node).T)
        new_use = err < outlier_mm
        if (new_use == use).all():
            break
        use = new_use
        if use.sum() < 2:
            break
    r = (q - node)[use]
    dof = max(1, 2 * use.sum() - 4)
    rms = float(math.sqrt((r ** 2).sum() / max(1, 2 * use.sum())))
    # φ のばらつき。残差の分散 × (JᵀJ)⁻¹
    sigma = None
    if use.sum() >= 3:
        p = pts[use]
        dR = np.array([[-math.sin(-phi), -math.cos(-phi)], [math.cos(-phi), -math.sin(-phi)]]) * -1
        J = np.zeros((2 * len(p), 4))
        J[:, 0] = (s * p @ dR.T).reshape(-1)
        J[:, 1] = (p @ rot(-phi).T).reshape(-1)
        J[0::2, 2] = 1
        J[1::2, 3] = 1
        var = (r ** 2).sum() / dof
        try:
            cov = np.linalg.inv(J.T @ J) * var
            sigma = math.degrees(math.sqrt(max(cov[0, 0], 0)))
        except np.linalg.LinAlgError:
            sigma = None
    return phi, s, use, rms, sigma


def _to_deviation(psi_deg: float) -> tuple[int, float]:
    """グリッドの軸に対する走行体の向き（90度周期）を、45度刻みからのずれにする"""
    psi = _wrap(psi_deg, 90.0)
    if abs(psi) <= 22.5:
        return 0, psi
    return 45, psi - math.copysign(45.0, psi)


def _estimate_by_lines(top, valid, opt: Options, image_bgr, cam):
    """黒線だけで測る。戻りは (ずれ[度], 系, σ[度], 本数) か None"""
    lines = find_lines(top, valid, opt, image_bgr, cam)
    if not lines:
        return None
    # 向きが揃う線の群のうち、長さの合計がいちばん大きいものを使う（90度違いの線も同じ向きとして扱う）
    best = None
    for ang0, _, _ in lines:
        group = [ln for ln in lines if abs(_wrap(math.degrees(ln[0] - ang0), 90.0)) < 0.5]
        size = sum(length for _, length, _ in group)
        if best is None or size > best[0]:
            best = (size, group)
    lines = best[1]
    # 4倍角で平均する。長いほど重く
    acc = sum(length ** 3 * complex(math.cos(4 * ang), math.sin(4 * ang)) for ang, length, _ in lines)
    phi = math.atan2(acc.imag, acc.real) / 4
    total = sum(length for _, length, _ in lines)
    # 端から端までの長さと、区切りの中心の外れから見積もる（下限 0.05°）
    rms = max(r for _, _, r in lines)
    sigma = max(0.05, math.degrees(math.sqrt(12) * max(rms, 0.3) / total / math.sqrt(8)))
    system, dev = _to_deviation(-math.degrees(phi))
    return dev, system, sigma, len(lines)


def estimate_heading(image_bgr: np.ndarray, cam: CameraConstants, opt: Options | None = None) -> HeadingResult:
    """
    1枚の画像から、グリッドに対する向きのずれを測る。
    """
    opt = opt or Options()
    if image_bgr.shape[1] != cam.width or image_bgr.shape[0] != cam.height:
        raise ValueError(f'画像の大きさ {image_bgr.shape[1]}x{image_bgr.shape[0]} が'
                         f'カメラの定数 {cam.width}x{cam.height} と違う')
    top, valid, _, _ = birdseye(image_bgr, cam, opt)
    by_markers = _estimate_by_markers(top, valid, opt)
    by_lines = _estimate_by_lines(top, valid, opt, image_bgr, cam) if opt.use_lines else None

    if by_lines is not None:
        by_markers.line_deviation_deg = round(by_lines[0], 4)
    if by_markers.ok:
        if by_lines is not None and abs(by_lines[0] - by_markers.deviation_deg) > opt.max_line_disagree_deg:
            by_markers.ok = False
            by_markers.reason = (f'目印（{by_markers.deviation_deg:+.2f}°）と黒線（{by_lines[0]:+.2f}°）が食い違う')
            by_markers.deviation_deg = None
        return by_markers
    if by_lines is not None:
        dev, system, sigma, count = by_lines
        return HeadingResult(ok=True, deviation_deg=round(dev, 4), system=system, sigma_deg=round(sigma, 4),
                             markers=by_markers.markers, source='line', line_deviation_deg=round(dev, 4),
                             reason=f'目印では測れず（{by_markers.reason}）、黒線 {count} 本で測った')
    return by_markers


def _estimate_by_markers(top, valid, opt: Options) -> HeadingResult:
    """目印（丸と QR の四角）で測る"""
    found = find_markers(top, valid, opt)
    if len(found) < opt.min_markers:
        return HeadingResult(ok=False, markers=len(found), reason=f'目印が {len(found)} 個しか見えない')

    # 図の画素 → 走行体から見た (前 d, 右 l)[mm]
    pts = np.array([(opt.d_max_mm - (r + 0.5) * opt.res_mm, -opt.lateral_mm + (c + 0.5) * opt.res_mm)
                    for r, c, _ in found])
    kinds = [k for _, _, k in found]

    theta0, _pairs = _initial_rotation(pts, kinds)
    if theta0 is None:
        return HeadingResult(ok=False, markers=len(found), reason='格子の間隔で並んだ目印の組が無い')

    phi, scale, use, rms, sigma = _fit_lattice(pts, kinds, theta0, opt.outlier_mm)
    n_use = int(use.sum())
    base = dict(markers=n_use, rejected=len(found) - n_use, residual_mm=round(rms, 2),
                points=[(round(float(p[0]), 1), round(float(p[1]), 1), k)
                        for p, k, u in zip(pts, kinds, use) if u])
    if n_use < opt.min_markers:
        return HeadingResult(ok=False, reason=f'格子に乗る目印が {n_use} 個しか無い', **base)
    if rms > opt.max_residual_mm:
        return HeadingResult(ok=False, reason=f'格子への当てはまりが悪い（{rms:.1f}mm）', **base)
    if abs(scale - 1) > 0.05:
        return HeadingResult(ok=False, reason=f'格子の間隔が合わない（{scale:.3f}倍）', **base)
    if sigma is None or sigma > opt.max_sigma_deg:
        return HeadingResult(ok=False, sigma_deg=sigma, reason=f'回転が定まらない（σ {sigma}）', **base)

    # φ はグリッドの軸が走行体から見てどちらを向いているか。走行体の向きはその逆
    system, dev = _to_deviation(-math.degrees(phi))
    return HeadingResult(ok=True, deviation_deg=round(dev, 4), system=system,
                         sigma_deg=round(sigma, 4), source='markers', **base)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    ap.add_argument('image')
    ap.add_argument('--camera', required=True, help='カメラの定数（JSON）')
    ap.add_argument('--debug', help='真上から見た図と拾った目印を書き出す先')
    args = ap.parse_args()
    cam = CameraConstants.from_json(json.loads(Path(args.camera).read_text()))
    img = cv2.imread(args.image, cv2.IMREAD_COLOR)
    res = estimate_heading(img, cam)
    if args.debug:
        opt = Options()
        top, valid, _, _ = birdseye(img, cam, opt)
        for r, c, k in find_markers(top, valid, opt):
            cv2.circle(top, (int(c), int(r)), 12, (0, 0, 255) if k == 'qr' else (255, 0, 0), 2)
        cv2.imwrite(args.debug, top)
    out = asdict(res)
    out.pop('points')
    print(json.dumps(out, ensure_ascii=False))


if __name__ == '__main__':
    main()
