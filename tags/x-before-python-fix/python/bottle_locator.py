"""
力士ボトル（黒ラベル 500mL）の位置特定（Python + OpenCV 版）。

前面カメラ映像からペットボトルを探し、走行体から見た位置を返す。
シミュレータの JS 実装（src/Sensor/BottleLocator.js）と同じ判定ロジックを
OpenCV で再現し、実機（Raspberry Pi）でも同じコードを使えるようにする。

【探し方は2通り。既定は黒ラベル】

    'label' … 黒ラベルの塊を探す（既定）。**実機はこちらでないと見つからない**
    'body'  … 胴の水色で塊を探し、中の黒ラベルで裏を取る（従来の探し方）

胴の水色（色相190〜225度・彩度0.03以上）はシミュレータの描き方に合わせた値で、
**実機の透明ボトルは白いコースの上でほぼ白に写り、彩度が0.03に届かない**。
塊が1画素も立たず、見えない扱いになる（`px0000`）。白いコースの上では
黒ラベルのほうがよほど目立つので、そちらを先に探す。

黒ラベルで探すときは、コースの黒線を形で落とす（`line_px` で薄い横帯を消し、
`min_aspect` 〜 `max_aspect` と `min_fill` でふるう）。詳細は Docs/et_sumo.md。

【返す値のうち Teams 指示の左右位置】
  lateral_score … 画面中央を 0 とし、左へ行くほど +100、右へ行くほど -100
  zone          … left / center / right（中央付近は center）
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, Optional, Tuple

try:
    import cv2
    import numpy as np
except ImportError:  # pragma: no cover - テスト環境で OpenCV が無い場合
    cv2 = None
    np = None

# ---------------------------------------------------------------------------
# しきい値（BottleLocator.js と揃える）
# ---------------------------------------------------------------------------

BODY_RANGE = {
    'hue_from': 190,
    'hue_to': 225,
    'min_saturation': 0.03,
    'max_saturation': 0.45,
    'min_value': 0.6,
}

#: 黒ラベルとみなす条件。
#:
#: **彩度（相対）で判定してはいけない。** HSV の彩度は (max-min)/max なので、
#: 暗い画素ほど分母が小さく、1〜2 の差で 1.0 まで跳ね上がる。実機のラベルは
#: BGR(0,3,1) のような真っ黒でも彩度 1.00 になり、`max_saturation 0.5` では
#: **本物の黒ラベルの 87% が捨てられていた**（2026-09-19、実機画像3枚で確認）。
#:
#: 代わりに**絶対的な色の差**（max-min、0〜255）で見る。真っ黒なら 2〜5 にしかならず、
#: 暗さに関係なく安定する。色のついたラベル（赤・青・黄）は明るいので `max_value` で落ちる。
BLACK_LABEL_RANGE = {
    'max_value': 0.35,
    'max_chroma': 40,
}

#: 力士ボトルの寸法[mm]。黒ラベルで探したときに、ラベルの下端から接地点までを
#: 画面の中で見積もるのに使う（ラベルの見かけの幅を直径とみなして縮尺を出す）。
#:
#: **胴が写っていればそちらの下端を使う**ので、この見積もりが効くのは実機の
#: 白いコースのように胴が写らないときだけ。ここがずれると止まる場所がそのぶんずれる。
#:
#: 直径は sorot_spike_web の public/config/course/2026/bottles.json（`sumo.bottle`）の値。
#: 同じ定義でボトルは高さ 205mm・重さ 105g、土俵は半径 187mm（直径 374mm）。
#:
#: **BASE_BELOW_LABEL_MM だけは未実測。** bottles.json はラベルの高さ（50mm）は持つが、
#: ボトルのどこに貼られているかを持たないため、下端から接地面までを出せない。
#: メジャーで測って直すこと（測り方は Docs/et_sumo.md）。
BOTTLE_DIAMETER_MM = 66.0
BASE_BELOW_LABEL_MM = 20.0

DEFAULT_LOCATOR = {
    'min_pixels': 40,
    'min_label_ratio': 0.05,
    'any_label': False,
    'column_gap': 2,
    'center_deadband': 10.0,
    #: 'label'（既定）… 黒ラベルの塊を探す / 'body' … 胴の水色で探す
    #: any_label を True にすると、ラベルの色を問わないので 'body' で探す
    'search': 'label',
    #: 黒ラベルの形のふるい。コースの黒線を落とす
    #: 線は「横に薄く長い」か「縦に細く長い」かのどちらかに写る。ラベルは
    #: ボトルの直径ぶんの幅と、その1〜2倍の高さを持つ塊なので、縦横比で分かれる
    'min_aspect': 0.25,     # 横幅 ÷ 高さ の下限。縦に細い線を落とす
    #: 横幅 ÷ 高さ の上限。**本物のラベルは縦長**（実機で 0.63〜0.75）。
    #: 2.5 では、横に広い黒線の塊が通る
    'max_aspect': 1.5,
    #: 外接する四角に占める割合の下限。**本物のラベルは 0.88〜0.95**（実機）。
    #: 0.5 では黒線や機材（0.58〜0.64）が通り、しかも大きいので本物に競り勝つ
    #: （2026-09-22、土俵で向きが ±24度に振れ続けた）
    'min_fill': 0.75,
    #: **足元が白（土俵）でない塊は採らない。**
    #:
    #: 力士ボトルは白い土俵の上に立つ。土俵の外は緑なので、**足元の色で
    #: 「土俵の上の物」と「場外の物」を分けられる**。距離にも大きさにも寄らないので、
    #: 近づいても成立しない＝寄っていって吸い込まれることが無い。
    #:
    #: 2026-09-27、土俵の北に置かれた**昔の走行体のタイヤ**を力士ボトルとして採り、
    #: 1.3〜1.7m 先へ寄っていった（実機 scsk3）。タイヤは縦長で詰まってもいるので、
    #: 縦横比（0.63〜0.75）と詰まり（0.88〜0.95）のふるいを通り抜けていた。
    #: **足元は緑だったので、これで分かれる。**
    #:
    #: **確かめられないときは通す。** 塊の下が画面の外なら白かどうか分からないが、
    #: そこで落とすと「近づいてラベルが画面下へ外れる」ふつうの場面まで弾いてしまう
    #: （押し切りの判断が変わる。`et_sumo.LOST_CLOSE_MM`）。
    'require_white_base': True,
    #: 足元とみなして見る高さ[px]（横320px 基準。実際の幅に合わせて伸縮する）
    'base_band_px': 6,
    #: その帯の何割が白ければ「土俵の上」とみなすか
    'base_white_ratio': 0.4,
    #: 白とみなす明るさの下限。土俵は白、場外の緑はこれを下回る
    'base_white_value': 110,
    #: 足元を見る幅を、塊の幅の何倍ぶん左右へ広げるか。
    #: **真下はボトル自身の影で暗いことがある。** 土俵は横に広いので脇まで見る
    'base_side_pad': 1.0,
    #: 画面の左右の端に接している塊は捨てる。**幅が切れていると距離が狂う**
    #: （距離はラベルの幅から出しているため）。土俵で拾った黒線も機材も端に接していた
    'drop_edge_touching': True,
    #: 横向きの線を消す太さ[px]（横320px のときの値）。これより薄い横帯は消える。
    #: **縦方向にだけ削る**ので、ラベルの幅は縮まない（幅は接地点の見積もりに使う）
    'line_px': 7.0,
}


def _require_cv2():
    if cv2 is None or np is None:
        raise ImportError('opencv-python と numpy が必要です')


def _opencv_hsv_bounds() -> Tuple[np.ndarray, np.ndarray]:
    """JS の HSV 範囲（0〜360 / 0〜1）を OpenCV 用（H:0〜180, S/V:0〜255）へ変換。"""
    _require_cv2()
    lower = np.array([
        BODY_RANGE['hue_from'] / 2.0,
        BODY_RANGE['min_saturation'] * 255,
        BODY_RANGE['min_value'] * 255,
    ], dtype=np.uint8)
    upper = np.array([
        BODY_RANGE['hue_to'] / 2.0,
        BODY_RANGE['max_saturation'] * 255,
        255,
    ], dtype=np.uint8)
    return lower, upper


def compute_lateral_score(u_center: float, width: int) -> float:
    """
    ボトル重心の横位置を -100〜+100 に正規化する。

    中央 = 0、左 = +100、右 = -100
    """
    if width <= 0:
        return 0.0
    norm = 0.5 - (u_center / width)
    return max(-100.0, min(100.0, norm * 200.0))


def lateral_zone(score: float, deadband: float = DEFAULT_LOCATOR['center_deadband']) -> str:
    """左右位置を left / center / right に分類する。"""
    if score > deadband:
        return 'left'
    if score < -deadband:
        return 'right'
    return 'center'


def _empty_hit() -> Dict[str, Any]:
    return {
        'found': False,
        'u_center': 0.0,
        'v_base': 0.0,
        'width_px': 0,
        'height_px': 0,
        'pixels': 0,
        'label_ratio': 0.0,
        'lateral_score': 0.0,
        'zone': 'center',
        'base_from': 'none',
    }


def _bgr_from_frame(frame: Any) -> Optional[np.ndarray]:
    """RGBA / RGB / BGR の numpy 配列または PyScript 由来データを BGR へ。"""
    _require_cv2()
    if frame is None:
        return None

    if isinstance(frame, dict):
        data = frame.get('data')
        width = int(frame.get('width') or 0)
        height = int(frame.get('height') or 0)
        if data is None or width <= 0 or height <= 0:
            return None
        if hasattr(data, 'to_py'):
            data = data.to_py()
        arr = np.frombuffer(bytearray(data), dtype=np.uint8)
        pixels = width * height
        if arr.size == pixels * 4:
            rgba = arr.reshape((height, width, 4))
            return cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGR)
        if arr.size == pixels * 3:
            return arr.reshape((height, width, 3))
        return None

    if np is not None and isinstance(frame, np.ndarray):
        if frame.ndim == 3 and frame.shape[2] == 4:
            return cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
        if frame.ndim == 3 and frame.shape[2] == 3:
            return frame.copy()
    return None


def _body_mask(bgr: np.ndarray) -> np.ndarray:
    """胴（透明プラの水色）らしい画素を立てる。"""
    _require_cv2()
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    lower, upper = _opencv_hsv_bounds()
    return cv2.inRange(hsv, lower, upper)


def _dark_mask(bgr: np.ndarray) -> np.ndarray:
    """黒ラベルらしい画素（暗く、色がついていない）を立てる。"""
    _require_cv2()
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    chroma = bgr.max(axis=2).astype(np.int16) - bgr.min(axis=2).astype(np.int16)
    dark = (hsv[:, :, 2] <= BLACK_LABEL_RANGE['max_value'] * 255) & (
        chroma <= BLACK_LABEL_RANGE['max_chroma']
    )
    return dark.astype(np.uint8) * 255


def estimate_base_v(label_bottom: int, label_width: int, body_mask: np.ndarray,
                    left: int, right: int, height: int) -> Tuple[float, str]:
    """
    ラベルの下端から、ボトルの接地点（画面の y）を出す。

    ラベルの下端は接地面より上にあるので、そのまま距離に使うと遠く出る。

    - 胴がラベルの真下に写っていれば、その下端を使う（正確。シミュレータと、
      実機でもボトルの足元に色が出ているとき）
    - 写っていなければ、**ラベルの見かけの幅をボトルの直径とみなして**縮尺を求め、
      `BASE_BELOW_LABEL_MM` ぶん下へ伸ばす（実機の白いコースはこちら）

    :returns: (接地点の y, 'body' / 'estimate' / 'clipped')

    `'clipped'` は足元が画面の外に出ていて、**距離が下限にしかならない**とき。
    近づいても値が変わらないので、これを見たら「これ以上近い」と扱うこと。
    """
    below = body_mask[label_bottom + 1:, left:right + 1]
    if below.size:
        rows = np.where(below.any(axis=1))[0]
        if rows.size:
            return float(label_bottom + 1 + int(rows.max())), 'body'
    if label_width <= 0 or BOTTLE_DIAMETER_MM <= 0:
        return float(label_bottom), 'estimate'
    grow = BASE_BELOW_LABEL_MM / BOTTLE_DIAMETER_MM * label_width
    v = label_bottom + grow
    if v >= height - 1:
        # 足元が画面の下へ外れている。**距離は「これより近い」ことしか言えない。**
        # そのまま使うと、どれだけ近づいても同じ距離が返り続ける
        return float(height - 1), 'clipped'
    return float(v), 'estimate'


def _stands_on_white(bgr, left: int, right: int, bottom: int,
                     opt: Dict[str, Any]) -> bool:
    """塊の足元が白（土俵）か。**確かめられないときは True**（下記）。

    力士ボトルは白い土俵の上に立ち、土俵の外は緑。足元の色で分けられる。

    **塊の下が画面の外なら True を返す。** そこで落とすと、近づいて足元が
    画面下へ外れるふつうの場面まで弾いてしまう。
    """
    if not opt.get('require_white_base', False):
        return True

    height, width = bgr.shape[:2]
    band = max(1, int(round(float(opt['base_band_px']) * width / 320.0)))
    y0 = bottom + 1
    y1 = min(height, y0 + band)
    if y0 >= height:
        return True                                 # 足元が画面の外。確かめられない

    # **左右に広げて見る。** ボトルの真下は自分の影で暗いことがある。
    # 土俵は横に広いので、脇まで見れば白が入る。場外の物は脇も緑のまま
    pad = int(round((right - left + 1) * float(opt['base_side_pad'])))
    x0 = max(0, left - pad)
    x1 = min(width, right + 1 + pad)
    if x1 <= x0:
        return True

    patch = bgr[y0:y1, x0:x1]
    if patch.size == 0:
        return True
    # 明るさは「いちばん明るいチャンネル」で見る（緑は G だけ高いので、
    # 平均だと白に近づく。最大値なら白と緑をはっきり分けられない場合があるため、
    # ここでは3channel の最小値を使う＝白は3つとも高い）
    value = patch.min(axis=2)
    white = int((value >= int(opt['base_white_value'])).sum())
    return white >= float(opt['base_white_ratio']) * value.size


def find_black_label_in_bgr(bgr: np.ndarray,
                            options: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """
    BGR 画像から**黒ラベルの塊**を探す（実機の既定の探し方）。

    白いコースの上では、透明な胴はほぼ白に写って拾えない（彩度が `min_saturation`
    0.03 に届かない）。黒ラベルのほうがよほど目立つので、そちらを直接探す。
    キャリーボトルのラベルは赤・青・黄なので、暗さだけで見分けられる。

    **コースの黒線を落とすのが肝。** 線はカメラから見て、横に薄く長い帯か、
    奥へ伸びる縦に細い帯として写る。

    - `line_px` … これより薄い横帯を消す。**縦にだけ削る**ので、ラベルの幅は
      縮まない（幅は接地点の見積もりに使う）。ラベルに線が繋がるのもここで切れる
    - `min_aspect` / `max_aspect` … 縦横比。縦に細い線と横に薄い線を落とす
    - `min_fill` … 外接する四角に対する詰まり具合。斜めの線や影は低く出る

    @returns u_center, v_base, lateral_score, zone など（`find_bottle_in_bgr` と同じ形）
    """
    _require_cv2()
    opt = {**DEFAULT_LOCATOR, **(options or {})}
    none = _empty_hit()

    if bgr is None or bgr.size == 0:
        return none

    height, width = bgr.shape[:2]
    mask = _dark_mask(bgr)

    # 横向きの線を消す。line_px は横320px を基準にした値なので、実際の幅に合わせる。
    # 縦長（幅1）の窓で削るため、消えるのは「縦に薄いもの」だけ。ラベルの幅は変わらない
    k = int(round(float(opt['line_px']) * width / 320.0))
    if k >= 2:
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (1, k))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    count, _labels, stats, _centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
    best = None
    for i in range(1, count):
        left, top, w, h, area = (int(v) for v in stats[i][:5])
        if area < opt['min_pixels'] or w <= 0 or h <= 0:
            continue
        if opt['drop_edge_touching'] and (left <= 0 or left + w >= width):
            continue                                # 左右の端で切れている＝幅が当てにならない
        aspect = w / float(h)
        if aspect > opt['max_aspect'] or aspect < opt['min_aspect']:
            continue                                # 横に薄い／縦に細い＝コースの黒線
        fill = area / float(w * h)
        if fill < opt['min_fill']:
            continue                                # 詰まっていない＝影・模様・斜めの線
        if not _stands_on_white(bgr, left, left + w - 1, top + h - 1, opt):
            continue                                # 足元が白くない＝土俵の外の物
        if best is None or area > best['pixels']:
            best = {'left': left, 'right': left + w - 1, 'top': top,
                    'bottom': top + h - 1, 'pixels': area, 'label_ratio': fill}

    if best is None:
        return none

    label_width = best['right'] - best['left'] + 1
    v_base, base_from = estimate_base_v(best['bottom'], label_width, _body_mask(bgr),
                                        best['left'], best['right'], height)
    u_center = (best['left'] + best['right']) / 2.0
    score = compute_lateral_score(u_center, width)
    return {
        'found': True,
        'u_center': u_center,
        'v_base': v_base,
        'width_px': label_width,
        'height_px': best['bottom'] - best['top'] + 1,
        'pixels': best['pixels'],
        'label_ratio': best['label_ratio'],
        'lateral_score': score,
        'zone': lateral_zone(score, opt['center_deadband']),
        'base_from': base_from,
    }


def find_bottle_in_bgr(bgr: np.ndarray, options: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """
    BGR 画像から力士ボトルを探す（OpenCV）。

    `search` が 'label'（既定）なら黒ラベルの塊で探す（`find_black_label_in_bgr`）。
    'body' なら胴の水色で探し、中の黒ラベルで裏を取る（従来）。
    `any_label` を True にするとラベルの色を問わないので、'body' で探す。

    @returns u_center, v_base, lateral_score, zone など
    """
    _require_cv2()
    opt = {**DEFAULT_LOCATOR, **(options or {})}
    none = _empty_hit()

    if opt['search'] == 'label' and not opt['any_label']:
        return find_black_label_in_bgr(bgr, options)

    if bgr is None or bgr.size == 0:
        return none

    height, width = bgr.shape[:2]
    body_mask = _body_mask(bgr)

    col_count = (body_mask > 0).sum(axis=0).astype(np.int32)
    col_bottom = np.full(width, -1, dtype=np.int32)
    col_top = np.full(width, height, dtype=np.int32)
    for x in range(width):
        ys_col = np.where(body_mask[:, x] > 0)[0]
        if ys_col.size == 0:
            continue
        col_bottom[x] = int(ys_col.max())
        col_top[x] = int(ys_col.min())

    def count_label_pixels(left: int, right: int, top: int, bottom: int) -> int:
        roi = bgr[top:bottom + 1, left:right + 1]
        if roi.size == 0:
            return 0
        roi_hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        v = roi_hsv[:, :, 2]
        chroma = roi.max(axis=2).astype(np.int16) - roi.min(axis=2).astype(np.int16)
        body_roi = body_mask[top:bottom + 1, left:right + 1] > 0
        dark = (~body_roi) & (v <= BLACK_LABEL_RANGE['max_value'] * 255) & (
            chroma <= BLACK_LABEL_RANGE['max_chroma']
        )
        return int(dark.sum())

    best = None
    x = 0
    while x < width:
        if col_count[x] == 0:
            x += 1
            continue

        left = x
        right = x
        gap = 0
        cur = x
        while cur < width:
            if col_count[cur] > 0:
                right = cur
                gap = 0
            else:
                gap += 1
                if gap > opt['column_gap']:
                    break
            cur += 1

        pixels = int(col_count[left:right + 1].sum())
        bottom = int(col_bottom[left:right + 1].max())
        top = int(col_top[left:right + 1].min())
        x = right + 1

        if pixels < opt['min_pixels']:
            continue

        area = (right - left + 1) * (bottom - top + 1)
        dark = 0 if opt['any_label'] else count_label_pixels(left, right, top, bottom)
        ratio = dark / area if area > 0 else 0.0
        if not opt['any_label'] and ratio < opt['min_label_ratio']:
            continue

        if best is None or pixels > best['pixels']:
            best = {
                'left': left,
                'right': right,
                'pixels': pixels,
                'bottom': bottom,
                'top': top,
                'label_ratio': ratio,
            }

    if best is None:
        return none

    u_center = (best['left'] + best['right']) / 2.0
    score = compute_lateral_score(u_center, width)
    return {
        'found': True,
        'u_center': u_center,
        'v_base': float(best['bottom']),
        'width_px': best['right'] - best['left'] + 1,
        'height_px': best['bottom'] - best['top'] + 1,
        'pixels': best['pixels'],
        'label_ratio': best['label_ratio'],
        'lateral_score': score,
        'zone': lateral_zone(score, opt['center_deadband']),
        'base_from': 'body',
    }


def find_bottle_in_frame(frame: Any, options: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """RGBA dict または BGR numpy からボトルを探す。"""
    bgr = _bgr_from_frame(frame) if not (np is not None and isinstance(frame, np.ndarray)) else frame
    if bgr is None and np is not None and isinstance(frame, np.ndarray):
        bgr = _bgr_from_frame({'data': frame.tobytes(), 'width': frame.shape[1], 'height': frame.shape[0]})
    if bgr is None:
        return _empty_hit()
    return find_bottle_in_bgr(bgr, options)


@dataclass
class BottlePose:
    """走行体から見たボトル位置。"""

    found: bool = False
    distance_mm: float = 0.0
    lateral_mm: float = 0.0
    bearing_rad: float = 0.0
    lateral_score: float = 0.0
    zone: str = 'center'
    image: Optional[Dict[str, Any]] = None


class BottleLocator:
    """
    前面カメラでボトルを探す（OpenCV 版）。

    JS の BottleLocator と同じ差し替え口を持つ。
    """

    def __init__(self, options: Optional[Dict[str, Any]] = None):
        self.options = options
        #: 最後に読んだ映像と、そのときの探索の結果（デバッグ画像に使う。読み取りには使わない）
        self.last_frame: Any = None
        self.last_hit: Optional[Dict[str, Any]] = None
        self.frame_source: Optional[Callable[[], Any]] = None
        self.ground_projection: Optional[
            Callable[[float, float, int, int], Optional[Dict[str, float]]]
        ] = None
        #: ラベルの見かけの幅から距離を出す関数（u_center, 幅[px], 画像の幅）。
        #: **こちらが入っていればこちらを使う。** 接地点から出す方法はカメラの
        #: 俯角がそのまま誤差になり、布の上のわずかな傾きで大きく振れる
        #: （実機で同じ位置が 225〜304mm、実測は 450mm。2026-09-22）
        self.size_projection: Optional[
            Callable[[float, int, int], Optional[Dict[str, float]]]
        ] = None
        self.pose = BottlePose()

    def set_frame_source(self, fn: Optional[Callable[[], Any]]):
        self.frame_source = fn

    def set_size_projection(
        self,
        fn: Optional[Callable[[float, int, int], Optional[Dict[str, float]]]],
    ):
        self.size_projection = fn

    def set_ground_projection(
        self,
        fn: Optional[Callable[[float, float, int, int], Optional[Dict[str, float]]]],
    ):
        self.ground_projection = fn

    def set_options(self, options: Dict[str, Any]):
        merged = dict(self.options or {})
        merged.update(options or {})
        self.options = merged

    def read(self) -> Dict[str, Any]:
        if not self.frame_source or not (self.ground_projection or self.size_projection):
            return self.get_pose()

        try:
            frame = self.frame_source()
        except Exception:
            frame = None
        self.last_frame = frame
        self.last_hit = None
        if frame is None:
            return self.clear()

        hit = find_bottle_in_frame(frame, self.options)
        self.last_hit = hit
        if not hit['found']:
            return self.clear()

        width = int(frame.get('width') if isinstance(frame, dict) else frame.shape[1])
        height = int(frame.get('height') if isinstance(frame, dict) else frame.shape[0])
        if self.size_projection is not None:
            ground = self.size_projection(hit['u_center'], int(hit['width_px']), width)
        else:
            ground = self.ground_projection(hit['u_center'], hit['v_base'], width, height)
        if not ground:
            return self.clear()

        self.pose = BottlePose(
            found=True,
            distance_mm=float(ground['distance_mm']),
            lateral_mm=float(ground['lateral_mm']),
            bearing_rad=math.atan2(float(ground['lateral_mm']), float(ground['distance_mm'])),
            lateral_score=float(hit['lateral_score']),
            zone=str(hit['zone']),
            image=hit,
        )
        return self.get_pose()

    def clear(self) -> Dict[str, Any]:
        self.pose = BottlePose()
        return self.get_pose()

    def get_pose(self) -> Dict[str, Any]:
        p = self.pose
        return {
            'found': p.found,
            'distance_mm': p.distance_mm,
            'lateral_mm': p.lateral_mm,
            'bearing_rad': p.bearing_rad,
            'lateral_score': p.lateral_score,
            'zone': p.zone,
            'image': p.image,
        }
