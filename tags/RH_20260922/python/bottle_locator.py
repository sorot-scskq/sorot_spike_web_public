"""
力士ボトル（黒ラベル 500mL）の位置特定（Python + OpenCV 版）。

前面カメラ映像からペットボトルを探し、走行体から見た位置を返す。
シミュレータの JS 実装（src/Sensor/BottleLocator.js）と同じ判定ロジックを
OpenCV で再現し、実機（Raspberry Pi）でも同じコードを使えるようにする。

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

BLACK_LABEL_RANGE = {
    'max_value': 0.35,
    'max_saturation': 0.5,
}

DEFAULT_LOCATOR = {
    'min_pixels': 40,
    'min_label_ratio': 0.05,
    'any_label': False,
    'column_gap': 2,
    'center_deadband': 10.0,
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


def find_bottle_in_bgr(bgr: np.ndarray, options: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """
    BGR 画像から力士ボトルを探す（OpenCV）。

    @returns u_center, v_base, lateral_score, zone など
    """
    _require_cv2()
    opt = {**DEFAULT_LOCATOR, **(options or {})}
    none = _empty_hit()

    if bgr is None or bgr.size == 0:
        return none

    height, width = bgr.shape[:2]
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    lower, upper = _opencv_hsv_bounds()
    body_mask = cv2.inRange(hsv, lower, upper)

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
        s = roi_hsv[:, :, 1]
        body_roi = body_mask[top:bottom + 1, left:right + 1] > 0
        dark = (~body_roi) & (v <= BLACK_LABEL_RANGE['max_value'] * 255) & (
            s <= BLACK_LABEL_RANGE['max_saturation'] * 255
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
        self.frame_source: Optional[Callable[[], Any]] = None
        self.ground_projection: Optional[
            Callable[[float, float, int, int], Optional[Dict[str, float]]]
        ] = None
        self.pose = BottlePose()

    def set_frame_source(self, fn: Optional[Callable[[], Any]]):
        self.frame_source = fn

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
        if not self.frame_source or not self.ground_projection:
            return self.get_pose()

        try:
            frame = self.frame_source()
        except Exception:
            frame = None
        if frame is None:
            return self.clear()

        hit = find_bottle_in_frame(frame, self.options)
        if not hit['found']:
            return self.clear()

        width = int(frame.get('width') if isinstance(frame, dict) else frame.shape[1])
        height = int(frame.get('height') if isinstance(frame, dict) else frame.shape[0])
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
