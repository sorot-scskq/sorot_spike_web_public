"""
前面カメラで、ET ラリーのグリッドに対する向きのずれを測る（走行体の Python）。

測る処理そのものは grid_heading.py。ここは「映像を 1枚もらって測り、結果を覚えておく」口。
いまは測って知らせるだけで、走りは変えない（ずれを把握する段階）。

【映像の入り口は差し替える】
  走行体        RobotCamera.shared().latest_frame()（自分で read() しないこと）
  シミュレータ  sim/pyscript/grid_heading_bridge.py が前面カメラの読み取り用の画を渡す

【カメラの定数】
  SIM_CAMERA   シミュレータ（sim/camera-model.js と同じ値）。シミュレータでは JS から渡し直す
  ROBOT_CAMERA 実機。**まだ測っていない。仮の値**。俯角は 0.1° の桁で合わせないと、黒線で
               測ったときに 1°近く外れる（docs/ETラリー_カメラで向きを補正する検証.md §4.4）

使い方

    reader = HeadingReader(frame_source=camera.latest_frame, camera=ROBOT_CAMERA)
    result = reader.read()
    if result['ok']:
        print(result['deviation_deg'])   # 走行体の向き − いちばん近いグリッドの向き
"""

from __future__ import annotations

import json
import logging
import math
import os
import socket
import time
from dataclasses import asdict, replace
from typing import Callable, Optional

try:
    from grid_heading import CameraConstants, Options, estimate_heading
except ImportError:  # パッケージとして読み込まれた場合
    from .grid_heading import CameraConstants, Options, estimate_heading

#: シミュレータの前面カメラ（読み取り用の画 960x576）
SIM_CAMERA = CameraConstants(model='sim', width=960, height=576, hfov_deg=70.0, vfov_deg=36.0,
                             eye_height_mm=170.0, tilt_deg=20.0)

#: 実機の前面カメラ。1280x720。
#:
#: **2026-09-22 に、床のホモグラフィから3つとも解き直した値。**
#:
#:   eye_height_mm  199.5   **光学的な高さ。** メジャーで測った床からレンズまでは
#:                          205mm だが、当てはめの倍率が 2.8% 足りなかった。
#:                          差 5.7mm はレンズの入射瞳の位置と思われる
#:   tilt_deg        26.6
#:   hfov/vfov       56.1 / 33.4
#:
#: 解き方: D が床の二次元コードから当てはめたホモグラフィ
#:（`2026/ground_homography.json`、残差 rms 0.42mm）を K[r1 r2 t] に分解し、
#: r1⊥r2 と |r1|=|r2| から焦点距離を出した。**高さが独立の検算になっている** ——
#: 分解は高さを一切使わないのに 204.1mm を返し、メジャーの 205mm と 0.9mm で一致した。
#: f=1202px（高さがちょうど 205mm になる解）。画角 56.1° は、G のチェスボード校正が
#: 出していた 53.9° に近い。
#:
#: **それまでの 220 / 20.0 / 49.0 は3つとも誤り**（2026-09-21 に入れた値）。
#: この幾何で組んだ行列は実写と合わず、ファインダの脚が 33.3mm のはずが 45.1mm
#:（+35%）、コードの間隔が +142mm 出ていた。高さの誤り（7%）だけでは説明できず、
#: 残りは俯角と画角だった。
#:
#:   220mm  メジャーで測り直したら 205mm。**裏取りの記録が無い値だった**
#:   20.0°  「地平線が写るから俯角は縦画角の半分より小さい」という消去法だけで
#:          41.3° を棄却して戻した値。**その議論は縦画角 42° を前提にしていた**
#:   49.0°  チェスボード校正が 42.6 → 49.1 → 50.4 → 53.9° と上がり続けていた途中で、
#:          小さい側を採った。53.9° のほうが今回の解に近い
#:
#: **未確認: 俯角 26.6° なら縦画角の半分（16.7°）より大きいので、地平線は写らないはず。**
#: 実機の絵には地平線が写っていたという観察と食い違う。撮り直して確かめること。
#:
#: **高さ 199.5 は F が実写88枚で決めた**（2026-09-22）。高さだけを振ると当てはめの
#: 倍率が単調に動き、残差は 2.9mm のまま変わらない（高さは一様な縮尺しか変えない）。
#: 倍率が 1.00 を横切るのが 199.3mm で、目印で測れた枚数もそこで最多（193→30枚 /
#: 199.3→37枚 / 205→34枚）。
#:
#: **2026-09-23 に 56.1° / 199.5mm / 26.6° から 53.9° / 205mm / 25.8° へ変えた。**
#: 以前の根拠だった「成立数の崖（画角 53.0°→3枚 / 56.1°→36枚 / 57.5°→4枚）」は
#: **再現しなかった。** 9/22 の完走した走行の実画像27枚で数え直すと平らだった。
#:
#:   画角  高さ  俯角   成立   ずれの平均  標準偏差   （D の残差の当てはめ）
#:   49.0   215  24.8   4/27    -0.09      1.59      2.18倍
#:   52.0   208  25.4   7/27    -0.22      1.79      1.21倍
#:   53.0   206  25.6   7/27    -0.20      1.74      1.04倍
#:   53.6   206  25.8   6/27    -0.14      1.91      最小 1.00倍
#:   53.9   205  25.8   6/27    -0.11      1.84      1.02倍 ← 採った値
#:   55.0   204  26.2   6/27    -0.09      1.91      1.18倍
#:   56.1   202  26.4   6/27    -0.14      1.92      1.45倍
#:   56.1   200  26.6   7/27    -0.94      2.71      ← 以前の値
#:   57.5   200  26.8   8/27    -0.49      1.85      1.90倍
#:
#: **成立数は画角を切り分けられない指標だった**（49〜57.5° で 4〜8 枚）。以前の崖は
#: D の暫定行列を経由する解き方で出たもので、行列を経由しない数え方では出ない。
#:
#: **効くのは2つ。** 一つはずれの偏り。**以前の値だけ平均が -0.94° と片側に寄り、
#: 標準偏差も 2.71° と大きい**（他はどれも -0.1〜-0.2°、1.6〜1.9°）。実際 9/22 の
#: 完走した走行で測れた9回の平均は -0.76° で、**定数が入れていた偏りだった。**
#:
#: もう一つは D の当てはめ。**暫定行列を経由せず、実写23枚のファインダ中心に
#: カメラの模型を直接当てた**（採点は脚 33.3mm とコード間隔 245.75mm だけ）。
#: 53.6° に本物の残差の最小があり、**G のチェスボード校正 53.9° がその 1.02倍に乗る。**
#: 原理の違う2つが 53.6〜53.9° で一致するので 53.9° を採った。
#:
#: **高さの実測では f を切れない**（53.6°→206mm、56.1°→202mm。どちらも実測に近い）。
#: 以前ここに書いていた「高さが実測と合う解を採った」という判断基準は効かなかった。
#:
#: **2026-09-23 15時台に 21.5° → 21.0° へ詰めた。** 21.5° を入れたあとの走行（14:38）を
#: 含む3走行の絵を、目印の帯にあるものだけ 25枚まとめて振り直した。
#:
#:   俯角      測れた/25   黒線との差（中央値）
#:   20.9°      13         0.33°
#:   **21.0°      16         0.32°**  ← 採った
#:   21.2°      15         0.25°
#:   21.5°      12         0.38°   ← それまでの値
#:   21.8°      11         0.80°
#:
#: **21.0〜21.2 は横並び**（16 と 15 は 25枚では誤差）で、**確からしさは ±0.2° 程度**。
#: どちらもそれまでの 21.5° より明確に良い。高さは 198〜212mm を振って 205mm が最良だった。
#:
#: 物差しを2つ使った。**測れた枚数**（多いほどよい）と、**目印の答えと黒線の答えの差**
#: （黒線は幾何への依存が弱いので、一致するほど幾何が正しい）。両方が 21.0〜21.2 を指した。
#:
#: **2026-09-23 13時台に俯角を 25.8° → 21.5° にした。カメラの取り付けが動いたため。**
#:
#: 実画像で俯角だけを振って測った（画角 53.9°・高さ 205mm は固定。目印の帯にある絵だけ）。
#:
#:   俯角      13:05 の走行      12:16 の走行
#:   21.0°     5/11             0/7
#:   **21.5°     7/11             3/7**  ← 最良
#:   22.5°     3/11             2/7
#:   24.0°     1/11             0/7
#:   **25.8°     0/11             0/7**  ← それまでの定数
#:
#: **それまでの定数では目印が1個も立たない。** 11:05 の走行（17/27 成功）と 11:58 の走行
#: （0/27）で `RoughSpot/Python` は1行も変わっておらず、ソフトではなく機体が変わった。
#: 窓は 11:40〜11:58 で、`asp` の再ビルドと複製（11:50）のために走行体を持ち上げている。
#:
#: **2026-09-23 に 21.0° で確定した。取り付けは戻さない。**
#: ゲートの柱（既知 258mm）を 500 / 1000mm の2点で解くと「俯角 21.0° / 色は床から 45.2mm」になり、
#: **現地のメジャー実測 45mm と一致した**
#:
#: **この解は芯々 258mm を拘束にして出したもの。**（2026-09-25 に芯々は
#: **254mm** に確定した＝足元のすき間 230 + 継手の外径 24。254 で解き直すと
#: 色の始まりは 44.5mm 前後になり、**実測 45mm にはより近い**）。
#: **いま使っているのは 18.2° なので、この行は履歴**（`Docs/run_investigation_20260923.md` §5.0）。
#: それでも取り付けを戻すなら、この値も 25.8° へ戻すこと。**両方やると二重に効く。**
#:
#: 副産物として、床が写る範囲は遠くへ伸びる（230〜1181mm → 268〜2110mm）。
#: ゲートが手前から見えるので、そちらは有利になる。
#: **実機で確かめた（2026-09-23）。** 直したあとの走行で測れた回数が戻っている。
#: 11:58 は 0/27、14:38（21.5°）は 8/15、14:59（21.0°）は **16/24**。
#: カメラのレンズ中心から、車軸の中心までの距離[mm]。**取り付けで決まる固定値。**
#:
#: 2026-09-22 に現地でメジャー実測した。
#:
#:     前後   レンズは車軸より 20mm 前（車軸は 20mm 後ろ）
#:     左右   ずれ無し（中心線上）。測った 55〜60mm はトレッド 112mm の半分
#:
#: **床の二次元コードから走行体の絶対位置を出すのに要る。** コードの ID が読めれば
#: そのコードの絶対位置が分かり（B の復号）、ホモグラフィでカメラから見た相対位置が
#: 出る（D）。そこからこの値で車軸へ移す。
#:
#: この3つが揃えば、据え置きでの校正（走行体が格子のどこに居たかの実測）は要らない。
CAMERA_TO_AXLE_FORWARD_MM = -20.0
CAMERA_TO_AXLE_LATERAL_MM = 0.0

#: 校正の置き場の親。**機体ごとの部屋に分けてある。**
#:
#:     camera_calib/scsk2/calibration.json
#:     camera_calib/scsk3/calibration.json
#:
#: **同じパスを2台で奪い合わせない。** 走行体ごとにブランチを分けているので、
#: 1つのパスに置くと、ブランチを互いにマージしたとき**同じファイルに別の値**が
#: ぶつかる。2026-09-25 に実際に起きかけた（J の枝が `develop_scsk2` の先端まで
#: 進み、scsk2 の校正を抱えたまま `develop_scsk3` へ入りかけた）。
#: そのときは add/add の衝突で git が止めたが、**片方だけが変わっていれば黙って通る。**
#:
#: 部屋を分ければ、そもそも同じファイルにならない。**両方のブランチに両機体ぶんを
#: 置いてあるので、合流しても取り違えようがない。**
CALIB_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'camera_calib')

#: 旧い置き場（部屋に分ける前）。**移行のため、部屋が無ければここを見る**
CALIB_JSON = os.path.join(CALIB_DIR, 'calibration.json')


def calib_path_for_host(base_dir: str = CALIB_DIR, host: str = None) -> str:
    """このホストの校正の置き場。**部屋の名前がホスト名に含まれていれば、その部屋**

    `robocon-scsk2` なら `camera_calib/scsk2/calibration.json`。
    見つからなければ、部屋に分ける前の `camera_calib/calibration.json` を返す
    （そこにあるものは `robot` の照合で弾かれるので、間違ったまま走ることはない）。
    """
    host = (host or socket.gethostname()).lower()
    try:
        rooms = sorted(n for n in os.listdir(base_dir)
                       if os.path.isdir(os.path.join(base_dir, n)))
    except OSError:
        return os.path.join(base_dir, 'calibration.json')
    # **長い名前から先に見る。** `scsk2` と `scsk23` があるとき、短いほうに
    # 先に当たらないようにする
    for name in sorted(rooms, key=len, reverse=True):
        if name.lower() in host:
            return os.path.join(base_dir, name, 'calibration.json')
    return os.path.join(base_dir, 'calibration.json')

#: 校正がまだ無い機体のための値。**測る前の当て推量**なので、校正が置かれたら負ける
FALLBACK_CAMERA = CameraConstants(model='pinhole', width=1280, height=720,
                                  hfov_deg=53.9, vfov_deg=31.9,
                                  eye_height_mm=205.0, tilt_deg=21.0)


def _uncalibrated(base: CameraConstants, why: str, path: str) -> CameraConstants:
    """校正を使わずに手入力の値へ戻る。**戻ったことを必ず残す**

    **黙って戻ると、走行のまとめを読むときに気づけない。** 戻り先は「測る前の
    当て推量」なので、校正との差は 2026-09-24 の実測で **fx +4.7% / 俯角 2.8°**。
    これは 9/23〜25 に俯角の食い違いを追いかけたときの幅そのもので、
    **黙って戻られると、その走行の距離も間隔も丸ごとずれたまま読むことになる。**
    """
    fx = (base.width / 2) / math.tan(math.radians(base.hfov_deg) / 2)
    _LOG('**カメラの校正を使いません（%s）: %s** '
         '手入力の値に戻ります: 画角 %.2f°（fx 相当 %.1f）/ 俯角 %.2f° / 高さ %.1fmm'
         % (why, path, base.hfov_deg, fx, base.tilt_deg, base.eye_height_mm))
    return base


def load_calibrated_camera(base: CameraConstants = FALLBACK_CAMERA,
                           path: str = None) -> CameraConstants:
    """校正の JSON があれば、レンズ由来の値だけ差し替えて返す

    **差し替えるのはレンズが決める値**（`fx fy cx cy` と歪み、そこから出る画角）。

    **`eye_height_mm` と `tilt_deg` は、JSON に書いてあるときだけ上書きする。**
    チェスボードの校正では出ない（カメラが床に対してどう向いているかは写真に
    写らない）ので、道具は書き出さない。**人が測って足す値。**
    書いていなければ `base` のまま。機体ごとに取り付けが違うので、
    **2台で同じ俯角を使わないこと。**

    **画角も上書きする。** `grid_heading` は `fx` があればそちらを使うが、
    `gate_finder` は `hfov_deg` を直に読んでいる（`gate_finder.py` の視野の計算）。
    片方だけ直すと、同じカメラに2つの画角がある状態になる。

    大きさが合わない校正は使わない。**`fx cx cy` は画素なので、別の解像度の値を
    そのまま当てると桁で外れる**（倍率で移せる場合もあるが、カメラが解像度で
    切り出し方を変えていると移せない）。
    """
    path = path or calib_path_for_host()
    obj = _read_calibration(path)
    if obj is None:
        return _uncalibrated(base, '置き場が無いか読めません', path)
    try:
        matrix = obj['camera_matrix']
        fx, fy = float(matrix[0][0]), float(matrix[1][1])
        cx, cy = float(matrix[0][2]), float(matrix[1][2])
        width, height = int(obj['width']), int(obj['height'])
    except (ValueError, KeyError, IndexError, TypeError) as e:
        return _uncalibrated(base, '中身が読めません（%s）' % e.__class__.__name__, path)

    # **どの機体の校正かを確かめる。** 走行体ごとにブランチを分けているので、
    # 2つのブランチを互いにマージすると、**同じパスに別の値が入る**。
    # ファイルの中身が違うだけなのでコンフリクトにならず、片方が黙って採られる。
    # 2026-09-24 の実測で fx は 3.6%、俯角は 3.8° 違う。取り違えると床の距離が
    # 丸ごとずれる。**間違った校正で走るくらいなら、手入力の値のほうがまだよい。**
    owner = str(obj.get('robot') or '').strip()
    if owner and owner not in socket.gethostname():
        return _uncalibrated(
            base, 'これは %s の校正で、いまの機体は %s です'
                  % (owner, socket.gethostname()), path)

    if (width, height) != (base.width, base.height):
        return _uncalibrated(
            base, '校正は %dx%d のもので、いまの %dx%d と合いません'
                  % (width, height, base.width, base.height), path)
    if not (fx > 0 and fy > 0):
        return _uncalibrated(base, '焦点距離が 0 以下です（fx %r / fy %r）' % (fx, fy), path)

    dist = obj.get('dist_coeffs')
    camera = replace(
        base,
        fx=fx, fy=fy, cx=cx, cy=cy,
        hfov_deg=math.degrees(2 * math.atan(width / 2 / fx)),
        vfov_deg=math.degrees(2 * math.atan(height / 2 / fy)),
        dist_coeffs=tuple(float(v) for v in dist) if dist else None)

    # **取り付けの値は、書いてあるときだけ上書きする。**
    # チェスボードの校正では出ないので、道具は書き出さない。**人が測って足す値。**
    # 機体ごとに違う（`tilt_deg` は取り付けの角度、`eye_height_mm` はレンズの高さ）。
    for key in ('tilt_deg', 'eye_height_mm'):
        if key in obj:
            try:
                camera = replace(camera, **{key: float(obj[key])})
                _LOG('取り付けの %s を校正の置き場から読みました: %.2f'
                     % (key, float(obj[key])))
            except (TypeError, ValueError):
                pass
    _LOG('カメラの校正を読みました（%s）: fx %.1f / fy %.1f / cx %.1f / cy %.1f / '
         '画角 %.2f° x %.2f°（手入力は %.2f° x %.2f° でした）'
         % (os.path.basename(os.path.dirname(path)), fx, fy, cx, cy,
            camera.hfov_deg, camera.vfov_deg, base.hfov_deg, base.vfov_deg))
    return camera


def _read_calibration(path: str):
    """校正を読む。`calibration.json` が無ければ `K.csv` / `d.csv` から組み立てる

    **道具は3つとも書き出すが、置き忘れることがある**（2026-09-24、2台目は
    CSV だけが置かれていた）。CSV は OpenCV がそのまま吐く形なので、
    そちらからでも読めるようにしておく。

    **CSV には解像度が書かれていない。** そのため大きさの照合ができず、
    走行と同じ 1280x720 で撮ったものと信じるしかない。**JSON のほうが安全。**
    """
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        pass

    folder = os.path.dirname(path)
    try:
        rows = _read_csv(os.path.join(folder, 'K.csv'))
        if len(rows) != 3 or len(rows[0]) != 3:
            return None
        dist = _read_csv(os.path.join(folder, 'd.csv'))
    except (OSError, ValueError):
        return None

    _LOG('%s が無いので K.csv / d.csv から読みます。'
         '**解像度が書かれていないので、%dx%d で撮ったものとして扱います。**'
         % (os.path.basename(path), 1280, 720))
    return {'camera_matrix': rows, 'width': 1280, 'height': 720,
            'dist_coeffs': dist[0] if dist else None}


def _read_csv(path: str) -> list:
    with open(path) as f:
        return [[float(v) for v in line.split(',')]
                for line in f.read().splitlines() if line.strip()]


def _LOG(message: str) -> None:
    """**読み込みは走行の前に1回しか起きない**ので、素直に出す"""
    try:
        logging.getLogger(__name__).info('%s', message)
    except Exception:  # noqa: BLE001 — 記録できなくても走行は続く
        pass


#: 実機の前面カメラ。**このホストの部屋の校正があればそれ、無ければ手入力の値。**
ROBOT_CAMERA = load_calibrated_camera()


class HeadingReader:
    """
    映像を 1枚もらって、グリッドに対する向きのずれを測る。

    :param frame_source: BGR の画像（numpy）を返す関数。撮れなければ None
    :param camera: カメラの定数。映像の大きさが違えば、画角はそのままで合わせて使う
    :param options: 測り方の細かい設定（grid_heading.Options）
    """

    def __init__(self, frame_source: Callable[[], object], camera: CameraConstants = SIM_CAMERA,
                 options: Optional[Options] = None, clock: Callable[[], float] = time.monotonic):
        self.frame_source = frame_source
        self.camera = camera
        self.options = options or Options()
        self._clock = clock
        self.last: Optional[dict] = None
        #: 最後に測った1枚。デバッグ画像として残すために覚えておく（heading_task.py）
        self.last_frame = None

    def read(self) -> dict:
        """
        1枚撮って測る。

        :returns: grid_heading.HeadingResult を辞書にしたもの（points は除く）に、
                  elapsed_ms（測るのにかかった時間）と width / height（映像の大きさ）を足したもの
        """
        frame = self.frame_source()
        self.last_frame = frame
        if frame is None:
            result = {'ok': False, 'reason': '映像が無い', 'deviation_deg': None, 'source': ''}
            self.last = result
            return result
        height, width = frame.shape[:2]
        camera = self.camera.with_size(width, height)
        started = self._clock()
        found = asdict(estimate_heading(frame, camera, self.options))
        found.pop('points', None)
        found['elapsed_ms'] = round((self._clock() - started) * 1000, 1)
        found['width'] = width
        found['height'] = height
        self.last = found
        return found
