"""
ET相撲（2026）の力士寄せ。走行体上の Python で、リモートコントロール走行の区間を走らせる。

土俵の力士ボトルへ、前面カメラで見ながら寄る。押し出せる近さと向きに来たら
Status.OK を返し、押し出しはシナリオの次のコマンドに任せる。

    撮る   … 止まって少し待ってから、力士ボトルがどこに見えるかを読む
    寄る   … 残りの距離の半分だけ寄る（1回 MAX_STEP_MM まで。APPROACH_STOP_AT_MM で止まる）
    首振り … 見えなければ 左 → 中央 → 右 → 中央 と振って撮り直す。一巡しても見えなければ少し前へ出る
             ただし**近くで見えていた直後に見失ったときは振らない**。押している最中なので
             前進を続けたまま OK を返す（LOST_CLOSE_MM）
    向き直り … 近く（APPROACH_DONE_MM 以内）でずれていれば、前へ出ずに向きだけ直す
    OK     … 近くで正面（ALIGN_DEG 以内）に見えた

**掴んだかどうかは見ない。** アームが触れたかを確かめる手段は実機にも無い。

【シナリオの書き方】
    { "SNO": 40, "CNO": 2, "RunInfo": { "FUNCNO": 18, "FWD": 40, "TRN": 40, "NOBLNCE": 1 },
      "SwitchInfo": { "SCJFN": 2049, "SCT": 20000 } }
FWD は寄るときの速さ、**TRN は寄りながらの操舵の強さ**（左右のPWM差の上限）。
**首振りと向き直りの強さはシナリオからは変えられない**（SCAN_TURN / ALIGN_TURN。
単位が各輪のPWMで TRN とは違うため、2026-09-23 に切り離した）。
この区間の OK で次（押し出し）へ進む。
2049 = 2048（リモート側のOK）+ 1（時間）で、OK を出せなくなっても SCT で先へ進む保険。
**TRN は 0 にしないこと。** FWD 0 の周期（撮影・首振り）で C++ が出力しなくなる
（RunControl は TRN=0 の区間を「停止」として扱う）。
本番は SubScenario/custom.json の 40/2、単体で試すのは SubScenario/check_et_sumo.json。

【差し方】
ev3_python の起動時に attach() が呼ばれる（config_2026.py の start_recognizers）。
リモート側の制御関数は走行体に1つだけ差せる（remote_control.py）。ETラリーでも
リモートコントロールを使うなら、sno を見て振り分ける親の制御関数を作ること。
この制御関数は、担当の SNO 以外の区間では指令を出さない（None を返す）。

【カメラ】
1280x720 を連続で取り込む（camera.py の RobotCamera.shared）。そのまま探すと重いので、
横 PROCESS_WIDTH に縮めてから探す。**映像はラベル色の読み取りが回している1台を借りる**
（latest_frame。走行体のカメラは1台しか開けない）。

地面の写し方（CameraGeometry）はカメラの取り付けで決まる。**高さ・傾き・車軸からの
前後はまだ測っていない**（Docs/et_sumo.md）。ここがずれると距離が丸ごとずれるので、
実機で確かめてから走らせること（Tools/check_sumo_locator.py）。

sorot_spike_web（シミュレータ）から移したもので、あちらは sim/pyscript/et_sumo_bridge.py が
同じファイルを読み込み、映像の入り口と「後で呼ぶ」口だけを差し替えて attach() する。
**直すときは両方**（判定のしきい値は両方で同じものを使う）。
"""

from __future__ import annotations

import logging
import math
import os
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Optional

from bottle_locator import BOTTLE_DIAMETER_MM, BottleLocator
from remote_control import Command, Observation, Status, set_controller

logger = logging.getLogger(__name__)

# --- 段取りのしきい値 --------------------------------------------------------
#: 担当する SNO（シナリオ番号の設計で ET相撲 は 40）
SUMO_SNO = 40
#: これより近くで正面に見えたら寄せ終わり。近すぎると足元が画面の下へ外れて写らない
#: （シミュレータの前面カメラは画面の一番下でも 219mm 先）。写る範囲の内側に置く
APPROACH_DONE_MM = 280.0
#: 寄るときはここで止まるように詰める。半分ずつ寄ると写らない近さまで入ってしまう
APPROACH_STOP_AT_MM = 250.0
#: 寄せ終わりとみなす向きのずれ[度]（車軸から見て）。250mm 先で 5度なら横に約 22mm
ALIGN_DEG = 5.0
#: 回り出すのに要る各輪のPWM。**これを下回ると布の上では動かない。**
#:
#: FWD 0 の指令は、そのまま各輪のPWMになる（Task/RunControl.cpp:383）。
#: 布の上で静止から回り出す下限は各輪PWM 40（RunMethod/RevolutionRunning.cpp の
#: MIN_TURN_POWER。測り直すなら SubScenario/check_turn_breakaway_pwm.json）。
#:
#: 2026-09-22、これを下回っていたために向き直りが永久に終わらなかった。
#: 242mm/+15.8度で向き直りに入り、PWM 20 を出して回らず、3分半そのままだった。
MIN_TURN_PWM = 45.0
#: 近くで向きだけ直すときの旋回量[各輪PWM]。
#:
#: **この値の出どころ**（2026-09-25〜26。混ぜないこと）
#:
#:     45   実機で回らなかった              MIN_TURN_PWM そのもの＝回り出せる下限
#:     50   実機でまだ弱い
#:     55   実機でまだ弱い（2026-09-26）。**ただしこのとき向き直りは 50 で回っていた**
#:          （`TURN_APPROACH_DEG` 20 が十数度の向き直りを丸ごと弱い段に入れていた）
#:     80   **強い（ユーザーの経験からの見立て。実機で走らせてはいない）**
#:     55   **いまの値。経験から置いたもので、実測ではない**
#:
#: **CLAUDE.md の「TRN 80 で 90度 0.83秒・±1.1度」は当てにしないこと。**
#: あれは `FUNCNO 3`（`RevolutionRunning`）の実測で、**全力 → 空走 → 近づく**の
#: 減速制御つきの数字。ET相撲は `FUNCNO 18` でそこを通らないので、同じ PWM でも
#: 中身が別物。2026-09-25 に「実測に合わせる」として 80 を入れたのは当てはめ違い。
#:
#: **ET相撲自身の実力はまだ測れていない。** 次の走行で「旋回の先読み停止（… deg/s）」
#: の行を見れば、初めて数字で出る。それまでは上の4つとも実測ではない。
#:
#: 行き過ぎは `TURN_LEAD_TIME` の先読み停止で抑える（速いほど手前で切る）。
#: **単位はシナリオの `TRN` と同じ「各輪のPWM」**（`FWD 0` の指令はそのまま各輪へ出る。
#: `Task/RunControl.cpp` の `power = runParameter.turn`）。シナリオの `TRN` とは
#: 別物なので、あちらを上げてもここは変わらない。
ALIGN_TURN = 80.0
#: 回りきったとみなす許容[度]。**ちょうど目標角に達したときに取りこぼさないため。**
#: `_wrap_pi` を通すと丸めで目標をわずかに下回ることがあり、`>=` が偽になる。
#: 旋回の実力は ±0.5度（CLAUDE.md「精度の下限」）なので、それより細かく詰めても意味がない
TURN_DONE_TOL_DEG = 0.5
#: 近づく段へ移る残り角[度]と、その段の強さ[各輪PWM]。
#:
#: **`FUNCNO 3` のその場旋回がやっている「全力 → 近づく」を、こちらでも作る。**
#: あちらは `RevolutionRunning` が3段階で詰めるが、**ET相撲は `FUNCNO 18` なので
#: そこを通らない**（`RemoteControl.cpp` は Python の指令をそのまま渡すだけで、
#: シナリオの `KP` / `KI` / `KD` を1つも読まない）。だから同じことをここに置く。
#:
#: 回り出しは `ALIGN_TURN` / `SCAN_TURN`（強い）。残りが `TURN_APPROACH_DEG` を
#: 切ったら `TURN_APPROACH_PWM`（弱い）へ落とす。
#:
#: **向き直りはたいてい十数度なので、最初から弱い側だけで回ることになる。**
#: 首振り（50度）は前半だけ強い。**「大きく振るのは速く、詰めは穏やかに」**
#:
#: `TURN_APPROACH_PWM` は `MIN_TURN_PWM` を下回らせない（回らなくなる）。
TURN_APPROACH_DEG = 10.0
TURN_APPROACH_PWM = 65.0
#: 先読み停止の時定数[s]。**旋回はここで見込んだぶん手前で力を切る。**
#:
#: この旋回は目標角に達したら出力を切るだけで、空走のぶんがそのまま行き過ぎになる。
#: 速くするほど空走が長いので、**速さを上げると小さい角度ほど悪くなる**
#: （2026-09-25、60 にして向き直りの直後に見失う回が続いた）。
#:
#: 値は直進の `STOP_LEAD_TIME`（0.073、CLAUDE.md の校正定数）と同じにしてある。
#: **旋回は少し短いはず**（`Utillity/TurnAngle.h` の注記）なので、行き過ぎるなら下げる。
#: 0 にすれば先読みしない（以前の動き）。
#:
#: 切ったあとは止まるのを待たずに撮影へ進むが、**撮影は SETTLE_SEC（0.2秒）待ってから
#: 1枚読む**ので、その間に空走が収まる。
TURN_LEAD_TIME = 0.073
#: 旋回に許す時間[s]。**超えたら撮り直しへ戻る。**
#: 回らないまま固まると、区間の SCT（20秒）まで何もできない
TURN_TIMEOUT_SEC = 3.0
#: 1回に寄る上限[mm]
#:
#: **歩数がそのまま所要時間になる。** 1歩ごとに止まって撮るので、1歩 = 停止 +
#: SETTLE_SEC + 撮影 + 前進。実機で 1.15秒/歩、1200mm から 6歩で約7秒かかっていた
#: （2026-09-23 16:42）。300 にすると歩数が半分になる。
#: 「残りの距離の半分まで」の制約が別にあるので、遠い読みで大股になりすぎることはない
MAX_STEP_MM = 300.0
#: 一巡しても見つからないときに前へ出る距離[mm]。
#: 土俵は直径 374mm なので、80mm ずつなら数回で全体を見渡せる
SEEK_MM = 80.0
#: **近くで見失ったら、押している最中とみなして、そのまま押し切る**。
#:
#: 実機では、近づきすぎるとボトルが写らなくなる（2026-09-19 に現場で確認）。
#: そのとき走行体はもうボトルに当たって押している。足元が画面の外へ出るのが
#: APPROACH_DONE_MM より手前で起きるため、距離も下限に張り付き、「これ以上近い」
#: ことを機体が知る手段が無い。
#:
#: **ここで止めてはいけないし、首を振ってもいけない。** 止めれば押し切れず、
#: 首を振ればボトルを横へ払う。前進をそのまま続けたまま OK を返し、
#: 押し出しはシナリオの次のコマンドへ渡す（SCJFN 2048）
LOST_CLOSE_MM = 280.0
#: 止まってから撮るまで待つ時間[s]。止まりきる前に撮るとぶれた映像になる
#:
#: **0.1 でも 30Hz なら3枚入れ替わる。** 借りているのは連続取り込みの最新の1枚
#: （`latest_frame`）なので、待っているあいだに映像は更新され続けている。
#: 1回 0.1秒の短縮でも、1回の寄せで10回前後撮るので約1秒効く
SETTLE_SEC = 0.1
#: 見つからないときの首振り[度]。左 → 右端 → 中央 の順。
#:
#: **水平画角の半分（28度）より広く振る。** 画角の外に居るから見えないので、
#: 同じだけ振っても届かない。±50 なら正面から左右 78度までを見たことになる。
SCAN_SIDE_DEG = 50.0
#: **一度に振るのは片側ぶんまで。** 以前は 2番目が -100度で、右端から左端まで
#: 一気に振っていた。その間は撮らないので、**正面を通り過ぎるときに見ていない**。
#: 正面に居るボトルを1枚目で拾い損ねると、そのまま左端まで行ってしまう
#: （2026-09-23、実機で「見つけきれていない」）。50度ずつに割って、
#: 正面でもう一度撮る。撮る向きは +50 → 0 → -50 → 0 の4か所になる
SCAN_SEQUENCE_DEG = (SCAN_SIDE_DEG, -SCAN_SIDE_DEG, -SCAN_SIDE_DEG, SCAN_SIDE_DEG)
#: 首振りの強さ[各輪PWM]。**向き直りと同じ値にそろえてある**（上の実測より 80）。
#:
#: 2026-09-23 に「探すときの旋回が早すぎて見つけきれない」と言われて 65 → 50 に
#: 落としたが、その後
#:
#:   * **回りながらも探すようになった**（`_watch`。見えたら止まって測り直す）
#:   * **先読み停止が入った**（`TURN_LEAD_TIME`。行き過ぎない）
#:
#: ので、落とした理由が両方とも別の手で解けている。
#:
#: **それでも 80 は「探しているときが早すぎる」（2026-09-26 実機）。** 首振りは
#: 1回 50度と大きいので、速さが体感にそのまま出る。60 にする。
#:
#: **向き直り（ALIGN_TURN）とは要求が逆向きなので、同じ値にしないこと。**
#:
#:     首振り    50度。**止まる位置の精度は要らない**（見えたらその場で測り直す）。
#:               回りながら見ているので、速いとぶれる・通り過ぎる
#:     向き直り  8〜22度。**狙った角で止まることが要る**（ずれが押し出しの向きになる）。
#:               角が小さいので速くしても時間は縮まらず、行き過ぎだけが増える
SCAN_TURN = 60.0

# --- カメラ ------------------------------------------------------------------
#: 探す前に縮める横幅[px]。実機 1280x720 → 320x180
PROCESS_WIDTH = 320
#: 塊とみなす画素数（240x144 の映像での値。下で面積の比に直す）。
#:
#: **黒ラベルの塊の大きさ**。実機では 300mm で約 7000px、1m で約 400px、1.5m で
#: 約 200px（横320px に縮めたとき）。土俵の中で使う範囲は 1.5m もあれば足りるので、
#: 320px 換算で 200px 相当にしてある。これを下回る暗い塊は、影やコース脇の
#: 暗がりであることのほうが多い（実機の赤ボトルの画像で 98〜121px の誤検出を確認）。
BASE_MIN_PIXELS = 120
BASE_AREA = 240 * 144


# --- デバッグの絵 ------------------------------------------------------------
#: 撮った1枚を残す先（走行体から見た相対パス）。
#:
#: 走行体には画面が無いので、走っている最中は「写っていないのか」「写っているが
#: 黒ラベルを拾えないのか」「拾えたが距離が合っていないのか」を切り分けられない。
#: 撮るたびに1枚残し、そのとき読んだ値をファイル名に入れておく（Docs/et_sumo.md）。
SNAPSHOT_DIR = "./sorot_spike/RoughSpot/PhotoImage/sumo"

#: 0 にすると残さない。**本番は 0 にすること**
SNAPSHOT_ENABLE = 1

#: 残す枚数の上限。古いものから消す。1回の寄せで 10〜30枚ほど撮る
SNAPSHOT_KEEP = 90


def build_snapshot_name(pose: Optional[dict], hit: Optional[dict], place: Optional[dict],
                        shot: int = 0, pivot: Optional[float] = None,
                        when: Optional[float] = None) -> str:
    """
    デバッグの絵のファイル名を組み立てる。

        sumo_20260919_121530_123_s40c2_n03_x+1234_y+0568_d0268_b+1.2_p+0.8_px1840_lab0.21.jpg
        sumo_20260919_121531_456_s40c2_n04_x+1240_y+0570_nf_px0000_lab0.00.jpg
                 ↑日時_ミリ秒      ↑区間 ↑何枚目 ↑自己位置[mm]  ↑見えなかった

    - `d` はカメラから見た距離[mm]、`b` はカメラから見た向き[度]、`p` は車軸から見た向き[度]
      （寄せが回るのは `p`。OK は `p` が ±ALIGN_DEG のとき）
    - `px` は黒ラベルの塊の画素数、`lab` はその塊の詰まり具合（min_fill 未満は捨てる）
    - 自己位置は走行体が思っている値。真値ではないので trajectory.csv と突き合わせて使う
    """
    now = time.time() if when is None else when
    stamp = "%s_%03d" % (time.strftime("%Y%m%d_%H%M%S", time.localtime(now)), int((now % 1) * 1000))

    place = place or {}
    section = "s%dc%d" % (int(place.get("sno", 0)), int(place.get("cno", 0)))
    if ("x" in place) and ("y" in place):
        where = "x%+05d_y%+05d" % (round(place["x"]), round(place["y"]))
    else:
        where = "xNA_yNA"

    hit = hit or {}
    tail = "px%04d_lab%.2f" % (int(hit.get("pixels") or 0), float(hit.get("label_ratio") or 0.0))
    if pose and pose.get("found"):
        body = "d%04d_b%+.1f" % (round(float(pose["distance_mm"])),
                                 math.degrees(float(pose["bearing_rad"])))
        if pivot is not None:
            body += "_p%+.1f" % pivot
    else:
        body = "nf"

    return "sumo_%s_%s_n%02d_%s_%s_%s.jpg" % (stamp, section, shot, where, body, tail)


@dataclass(frozen=True)
class CameraGeometry:
    """前面カメラの取り付け。画面の位置を、走行体から見た距離・左右のずれに直すのに使う"""

    #: 横・縦の画角[度]
    hfov_deg: float
    vfov_deg: float
    #: レンズの高さ[mm]
    height_mm: float
    #: 下向きの傾き[度]
    tilt_deg: float
    #: レンズが車軸（その場旋回の中心）より前にある距離[mm]
    forward_mm: float
    #: これより遠い地面は見えないものとして扱う[mm]
    far_mm: float = 5000.0


#: シミュレータの前面カメラ（sorot_spike_web の sim/camera-model.js。水平 70度・垂直 36度、
#: 高さ 34px・前後 18px を 0.2px/mm で直した値）。テストはこれを使う
SIM_CAMERA = CameraGeometry(hfov_deg=70.0, vfov_deg=36.0, height_mm=170.0, tilt_deg=20.0,
                            forward_mm=90.0)
#: 実機の前面カメラ。走行体のカメラは1台しかないので、**向きの測定と同じ値を使う**
#: （2026-09-22 に確定。`2026/heading_reader.py` の ROBOT_CAMERA と揃えること）。
#:
#: D のホモグラフィ（残差 rms 0.42mm）を K[r1 r2 t] に分解して求めた値で、
#: 高さは分解が独立に返した実効値。以前の 62/37/170/20 は仮置きで、
#: 400mm 付近の距離が 15% ずれていた（Docs/handoff_20260922.md §1）。
#:
#: **forward_mm（車軸からレンズまでの前後）だけは未実測。** 近い物ほど効くので、
#: 寄せ終わりの向き（pivot_deg）がずれるなら、まずここを疑うこと。
#: **2026-09-23: 53.9 / 31.9 / 205.0 / 21.0 に直した。** 取り付けが動き、画角も
#: 採り直したため（`heading_reader.ROBOT_CAMERA` と同じ値。そちらの注記に根拠がある）。
#:
#: 俯角 21.0° は3つの独立した証拠で裏が取れている。
#:   目印の当てはめが成立した枚数（相対）        25.8° で 0/11、21° 近傍で最良
#:   ゲートの既知 258mm と 500/1000mm の実測（絶対）  21.0° / 色 45.2mm、残差 0.14%
#:   柱の色が始まる高さの現地実測                **45mm**（2点解の 45.2mm と 0.2mm 差）
#:
#: **ここが自前の複製になっているのが事故の元。** `heading_reader` が直っても、
#: こちらは黙って古いまま残る（実際 2026-09-23 に半日そうなっていた）。
#: `CameraGeometry` と `CameraConstants` の型が違うので機械的には共有できないが、
#: **片方を直したらもう片方も直すこと。**
ROBOT_CAMERA = CameraGeometry(hfov_deg=53.9, vfov_deg=31.9, height_mm=205.0, tilt_deg=21.0,
                              forward_mm=90.0)


def ground_projection(geometry: CameraGeometry) -> Callable[[float, float, int, int], Optional[dict]]:
    """画面の (u, v) を、カメラから見た距離・左右のずれ[mm]に直す関数を作る"""
    tan_half_h = math.tan(math.radians(geometry.hfov_deg) / 2.0)
    tan_half_v = math.tan(math.radians(geometry.vfov_deg) / 2.0)

    def project(u: float, v: float, width: int, height: int) -> Optional[dict]:
        if width <= 0 or height <= 0:
            return None
        focal = (height / 2.0) / tan_half_v
        down = math.radians(geometry.tilt_deg) + math.atan((v - height / 2.0) / focal)
        distance = geometry.far_mm if down <= 1e-4 else min(geometry.far_mm, geometry.height_mm / math.tan(down))
        half_lateral = distance * tan_half_h
        lateral = ((u + 0.5) / width - 0.5) * 2.0 * half_lateral
        return {"distance_mm": distance, "lateral_mm": lateral}

    return project


def size_projection(geometry: CameraGeometry) -> Callable[[float, int, int], Optional[dict]]:
    """
    ラベルの**見かけの幅**から、カメラから見た距離・左右のずれ[mm]を出す関数を作る。

    ボトルは円筒なので、写る幅はどこから見ても直径になる。距離は

        距離 = 直径 × 焦点[px] ÷ 見かけの幅[px]

    **カメラの俯角にも高さにも依存しない。** 要るのは水平画角と直径だけで、
    どちらも実測済み（画角は 2026-09-22、直径は bottles.json の 66mm）。

    接地点から出す方法をやめた理由（2026-09-22、実機で測った）:

        実測 450mm に対し  接地点から 225〜304mm（σ 32mm） / 幅から 422〜431mm（σ 4.6mm）

    接地点の方式は「足元が画面のどこに写ったか」なので、**カメラの俯角の誤差が
    そのまま距離の誤差になる。** 布の上では機体がわずかに傾くだけで塊が上下に
    平行移動し（実機で 22px＝俯角 4度ぶん）、大きさは変わらないのに距離だけが振れた。
    """
    tan_half_h = math.tan(math.radians(geometry.hfov_deg) / 2.0)

    def project(u: float, label_width_px: int, image_width: int) -> Optional[dict]:
        if image_width <= 0 or label_width_px <= 0 or BOTTLE_DIAMETER_MM <= 0:
            return None
        focal = (image_width / 2.0) / tan_half_h
        distance = min(geometry.far_mm, BOTTLE_DIAMETER_MM * focal / label_width_px)
        half_lateral = distance * tan_half_h
        lateral = ((u + 0.5) / image_width - 0.5) * 2.0 * half_lateral
        return {"distance_mm": distance, "lateral_mm": lateral}

    return project


def pivot_deg(distance_mm: float, bearing_deg: float, forward_mm: float) -> float:
    """
    カメラから見た向きを、車軸から見た向きに直す。

    カメラは車軸より前にあるので、近い物ほどカメラから見た角度が大きく出る。
    その角度のまま回ると回りすぎて、左右に振れ続ける。
    """
    b = math.radians(bearing_deg)
    return math.degrees(math.atan2(distance_mm * math.sin(b), distance_mm * math.cos(b) + forward_mm))


def _wrap_pi(rad: float) -> float:
    return (rad + math.pi) % (2.0 * math.pi) - math.pi


def shrink_for_processing(frame: Any) -> Any:
    """映像を横 PROCESS_WIDTH に縮める（縦横比は保つ）。縮める必要が無ければそのまま"""
    if frame is None:
        return None
    import cv2  # 使うときだけ読む（RoughSpot/README.md の決まり）
    height, width = frame.shape[:2]
    if width <= PROCESS_WIDTH:
        return frame
    size = (PROCESS_WIDTH, max(1, round(height * PROCESS_WIDTH / width)))
    return cv2.resize(frame, size, interpolation=cv2.INTER_AREA)


def make_locator(frame_source: Callable[[], Any], geometry: CameraGeometry) -> BottleLocator:
    """力士ボトルの探索器。**黒ラベルの塊**を探す（キャリーボトルのラベルは赤・青・黄）。

    以前は胴の水色で探していたが、実機の透明ボトルは白いコースの上でほぼ白に写り、
    彩度が下限に届かず1画素も立たない。詳しくは Docs/et_sumo.md。
    """
    ratio = (PROCESS_WIDTH * PROCESS_WIDTH * 9 / 16) / BASE_AREA
    locator = BottleLocator({"min_pixels": max(1, round(BASE_MIN_PIXELS * ratio))})
    locator.set_frame_source(lambda: shrink_for_processing(frame_source()))
    locator.set_ground_projection(ground_projection(geometry))
    locator.set_size_projection(size_projection(geometry))
    return locator


def _robot_frame_source() -> Callable[[], Any]:
    """前面カメラ（連続取り込みの最新の1枚）"""
    from camera import RobotCamera

    def latest() -> Any:
        camera = RobotCamera.shared()
        frame = camera.latest_frame()
        if frame is None:
            # 連続取り込み（ラベル色の読み取り）が回っていない。誰も読んでいないので自分で取る
            # （camera.py の capture_still と同じ。VideoCapture を触る側は常に1つにする）
            frame = camera.grab_latest()
        return frame

    return latest


def _run_later_with_thread(fn: Callable[[], None], delay_sec: float) -> None:
    """実機: 別スレッドで待ってから呼ぶ。制御関数（5ms ごと）を待たせない"""
    timer = threading.Timer(delay_sec, fn)
    timer.daemon = True
    timer.start()


class SumoApproach:
    """
    力士寄せの制御関数（remote_control の set_controller に差す）。

    撮影と位置の読み取りは重いので、制御関数の中ではやらない。run_later で後から
    呼んでもらい、制御関数は「もう読めたか」を見るだけにする。
    """

    def __init__(self, locator: BottleLocator, geometry: CameraGeometry,
                 run_later: Callable[[Callable[[], None], float], None] = _run_later_with_thread,
                 sno: int = SUMO_SNO):
        self.locator = locator
        self.geometry = geometry
        self.run_later = run_later
        self.sno = sno
        self._lock = threading.Lock()
        self._seen: Optional[dict] = None
        self._shooting = False
        self._shot_done = False
        #: 探索で例外が出たことを知らせたか（0.2秒ごとに出すとログが埋まる）
        self._read_warned = False
        self._state = "shoot"
        self._cmd_seq: Optional[int] = None
        self._tick = 0
        self._start_distance = 0.0
        self._step_mm = 0.0
        self._bearing_deg = 0.0
        self._start_direction = 0.0
        self._target_rad = 0.0
        self._turn = 0.0
        self._turn_started_at = 0.0
        self._scan_index = 0
        #: いまの動き（旋回・前進）の最中も探すか。首振りと「前へ出る」だけ True
        self._watching = False
        #: 役目を終えた（"done"）あとに出し続ける前進の指令。押し切りの最中は止めない
        self._done_forward = 0.0
        #: 最後に見えたときの距離[mm]。近くで見失ったかの判断に使う
        self._last_seen_mm: Optional[float] = None
        #: デバッグの絵に入れる、いまの区間と自己位置（毎周期の観測から写す）
        self._place: dict = {}
        #: 撮った回数。絵の名前に入れる
        self._shots = 0
        #: 絵を残せないことを知らせたか（撮るたびに出すとログが埋まる）。
        #: **「映像が無い」と「書けない」で分ける。** 1つにすると、起動直後に
        #: カメラがまだ開いていないだけで出る警告が、そのあとの本当の書き込み
        #: 失敗を黙らせてしまう（2026-09-22 の実機ログで判明）
        self._snapshot_warned = False
        self._no_frame_warned = False

    # --- 撮影（run_later から呼ばれる）---------------------------------------

    def _shoot(self) -> None:
        try:
            pose = self.locator.read()
        except Exception as e:  # noqa: BLE001 — 見えなくても走行は続ける（OpenCV が無い等）
            if not self._read_warned:
                self._read_warned = True
                logger.warning("力士ボトルを探せませんでした（以降は出しません）: %s", e)
            pose = {"found": False}
        seen = None
        if pose.get("found"):
            seen = {"distance_mm": float(pose["distance_mm"]),
                    "bearing_deg": math.degrees(float(pose["bearing_rad"]))}
        with self._lock:
            self._seen = seen
            self._shot_done = True
            self._shooting = False
        self.save_snapshot(pose)

    # --- デバッグの絵 ----------------------------------------------------------

    def save_snapshot(self, pose: Optional[dict]) -> Optional[str]:
        """撮った1枚を、読んだ値を名前に入れて残す。

        **残せなくても寄せは続ける。** OpenCV が無い・書けない・映像が無い、の
        いずれも警告を1度出すだけ（撮るたびに出すとログが埋まる）。

        :returns: 書けたファイルのパス。書けなければ None
        """
        if not SNAPSHOT_ENABLE:
            return None

        frame = getattr(self.locator, "last_frame", None)
        if isinstance(frame, dict):
            # dict はシミュレータの RGBA。実機の絵（BGR の numpy）だけを残す
            return None
        if frame is None:
            # カメラから1枚も来ていない。**ここで黙ってはいけない。**
            # 絵が無い相談のとき、「区間に入っていない」のか「映像が来ていない」のかを
            # 分ける唯一の手がかりがこの行になる
            if not self._no_frame_warned:
                self._no_frame_warned = True
                logger.warning("力士寄せの映像が取れないので絵を残せません（以降は出しません）。"
                               "起動直後ならカメラが開く前なので実害は無い。"
                               "走行中に出るなら連続取り込みが止まっている")
            return None

        self._shots += 1
        try:
            from opencv import OpenCv
            cv2 = OpenCv.cv2()
            if cv2 is None:
                raise RuntimeError("OpenCV が無い")

            pivot = None
            if pose and pose.get("found"):
                pivot = pivot_deg(float(pose["distance_mm"]),
                                  math.degrees(float(pose["bearing_rad"])),
                                  self.geometry.forward_mm)
            name = build_snapshot_name(pose, getattr(self.locator, "last_hit", None),
                                       self._place, self._shots, pivot)
            os.makedirs(SNAPSHOT_DIR, exist_ok=True)
            path = os.path.join(SNAPSHOT_DIR, name)
            if not cv2.imwrite(path, frame):
                raise RuntimeError("書けなかった: %s" % path)
            _prune(SNAPSHOT_DIR, SNAPSHOT_KEEP)
            return path
        except Exception as e:  # noqa: BLE001 — 残せなくても寄せは続ける
            if not self._snapshot_warned:
                self._snapshot_warned = True
                logger.warning("力士寄せのデバッグ画像を残せません（以降は出しません）: %s", e)
            return None

    # --- 状態の移り変わり ------------------------------------------------------

    def _start_turn(self, obs: Observation, target_deg: float, turn: float,
                    watching: bool = False) -> Command:
        """
        その場旋回を始める。

        :param watching: 回りながらも探すか。**首振りだけ True。**
                     向き直りで True にすると、目の前のボトルを見つけて
                     回るのをやめてしまう（向きが直らない）
        """
        self._state = "turn"
        self._watching = watching
        self._start_direction = float(obs.coordinate.get("direction", 0.0))
        self._target_rad = math.radians(target_deg)
        #: 弱すぎると回らない。FWD 0 の turn はそのまま各輪のPWMになる
        self._turn = max(abs(turn), MIN_TURN_PWM)
        self._turn_started_at = time.time()
        return Command(0, 0, Status.NG)

    def _watch(self) -> bool:
        """
        動きながら探す。**見えたら True**（呼ぶ側が動くのをやめる）

        **ここで読んだ値は使わない。** 動いている最中の1枚なので、距離も向きも
        当てにならない。使うのは「いま画角に入っている」ことだけで、止まってから
        撮り直して測る（呼ぶ側が `shoot` へ戻る）。

        撮影は `run_later` で別スレッドへ出す。待たないので、制御関数（10ms）は
        そのまま回り続ける。頼んでいる最中は二重に頼まない。
        """
        with self._lock:
            done = self._shot_done
            ask = not done and not self._shooting
            if ask:
                self._shooting = True
            found = False
            if done:
                # 撮り直すので、読んだ値はここで捨てる
                self._shot_done = False
                found = self._seen is not None
                self._seen = None
        if found:
            return True
        if ask:
            # **待たない。** 止まっていないので、落ち着くのを待つ意味がない
            self.run_later(self._shoot, 0.0)
        return False

    def _back_to_shoot(self) -> Command:
        """止まって撮り直す状態へ戻す（`_watch` が見つけたあと）"""
        self._state = "shoot"
        self._watching = False
        with self._lock:
            self._seen = None
            self._shooting = False
            self._shot_done = False
        return Command(0, 0, Status.NG)

    def _finish(self, fwd: float) -> Command:
        """
        役目を終える。**以後この区間では探し直さない。**

        OK を返したあとも、C++ が切替を済ませるまでは 10ms ごとに呼ばれ続ける。
        そのまま撮影へ戻ると、

        * 押し切りの最中（`LOST_CLOSE_MM`）に「見えない」で止まる指令を出す
          （OK の次の周期で forward が 0 に上書きされる）
        * 終わった区間で撮影を頼み、デバッグ画像を1枚よけいに残す
        * 首を振ってボトルを横へ払う

        いずれも起きうる。決めたことを持ったまま OK を出し続け、区間に入り直す
        （SCRIPT が attach() をやり直す／周期番号が戻る）まで何もしない。

        :param fwd: 出し続ける前進の指令。押し切りの最中は止めない
        """
        self._state = "done"
        self._done_forward = fwd
        return Command(fwd, 0, Status.OK)

    def _start_scan_or_seek(self, obs: Observation) -> Command:
        if self._last_seen_mm is not None and self._last_seen_mm <= LOST_CLOSE_MM:
            # 直前まで近くで見えていた → もう当たって押している。**止めない・振らない**
            fwd = float(obs.run_info.get("FWD") or 40.0)
            logger.info("近く（%.0fmm）で見失った → 押している最中とみなしてそのまま押し切る",
                        self._last_seen_mm)
            return self._finish(fwd)
        if self._scan_index < len(SCAN_SEQUENCE_DEG):
            deg = SCAN_SEQUENCE_DEG[self._scan_index]
            self._scan_index += 1
            logger.info("力士ボトルが見えない → 首振り %+g度 (%d/%d)", deg, self._scan_index, len(SCAN_SEQUENCE_DEG))
            return self._start_turn(obs, deg, SCAN_TURN, watching=True)
        self._scan_index = 0
        self._step_mm = SEEK_MM
        self._start_distance = float(obs.coordinate.get("distance", 0.0))
        self._bearing_deg = 0.0
        self._state = "move"
        #: **前へ出る間も探す。** 見えていないから出るのであって、出ている
        #: あいだ目をつぶる理由が無い（寄るときの move は既に見えているので見ない）
        self._watching = True
        logger.info("首振りでも見えない → %gmm 前へ出る", SEEK_MM)
        return Command(0, 0, Status.NG)

    def __call__(self, obs: Observation) -> Optional[Command]:
        if obs.sno != self.sno:
            return None     # 担当の区間ではない

        # 区間に入り直したら、撮影から始める。通し番号が同じでも、区間の中の周期番号（tick）が
        # 戻っていれば入り直している（走行体を起動し直すと、通し番号はまた同じ値から振られる）
        self._place = {"sno": obs.sno, "cno": obs.cno,
                       "x": float(obs.coordinate.get("x", 0.0)),
                       "y": float(obs.coordinate.get("y", 0.0))}
        tick = int(obs.tick or 0)
        restarted = tick < self._tick
        self._tick = tick

        # **役目を終えていたら、もう働かない。** 通し番号が変わっただけでは戻らない
        # （区間に入り直せば SCRIPT が attach() をやり直して新しい制御関数が差さる）。
        # 走行体を起動し直したときだけ（周期番号が戻る）やり直す
        if self._state == "done" and not restarted:
            return Command(self._done_forward, 0, Status.OK)

        if obs.cmd_seq != self._cmd_seq or restarted:
            self._cmd_seq = obs.cmd_seq
            self._state = "shoot"
            self._scan_index = 0
            self._watching = False
            self._last_seen_mm = None
            with self._lock:
                self._seen = None
                self._shooting = False
                self._shot_done = False

        fwd = float(obs.run_info.get("FWD") or 40.0)
        turn_gain = abs(float(obs.run_info.get("TRN") or 40.0))
        distance_now = float(obs.coordinate.get("distance", 0.0))

        if self._state == "shoot":
            with self._lock:
                done = self._shot_done
                ask = not done and not self._shooting
                if ask:
                    self._shooting = True
                if done:
                    self._shot_done = False
                    seen, self._seen = self._seen, None
            if not done:
                # 頼むのはロックの外で。すぐ呼ばれる run_later だと、_shoot が同じロックで止まる
                if ask:
                    self.run_later(self._shoot, SETTLE_SEC)
                return Command(0, 0, Status.NG)     # 止まったまま待つ

            if seen is None:
                return self._start_scan_or_seek(obs)

            self._scan_index = 0
            distance = seen["distance_mm"]
            bearing = seen["bearing_deg"]
            self._last_seen_mm = distance
            # **`clipped` で近道をしない。** あれは接地点（v_base）が画面の外へ
            # 出たという印で、距離を接地点から出していた頃の名残。いまの距離は
            # ラベルの幅から出していて v_base に依存しないので、両者は無関係。
            # 使うと寄せ終わりが早まる（実機で 310mm で OK が出た。2026-09-22）
            if distance <= APPROACH_DONE_MM:
                pivot = pivot_deg(distance, bearing, self.geometry.forward_mm)
                if abs(pivot) <= ALIGN_DEG:
                    logger.info("力士ボトルが近くで正面に見えた（%.0fmm, %+.1f度）→ OK", distance, pivot)
                    return self._finish(0.0)
                logger.info("近い（%.0fmm）→ 向きだけ直す %+.1f度", distance, pivot)
                return self._start_turn(obs, pivot, ALIGN_TURN)

            self._step_mm = min(distance / 2.0, MAX_STEP_MM, distance - APPROACH_STOP_AT_MM)
            self._start_distance = distance_now
            self._watching = False      # もう見えている。歩いている間は探さない
            self._bearing_deg = bearing
            self._state = "move"
            return Command(0, 0, Status.NG)

        if self._state == "turn":
            # **回りながらも探す（首振りのときだけ）。** 見えたら回るのをやめて、
            # 止まってから測り直す。旋回の 1.2 秒が丸ごと「見ていない時間」だった
            if self._watching and self._watch():
                logger.info("首振りの最中に見えた → 止まって測り直す")
                return self._back_to_shoot()

            progressed = _wrap_pi(float(obs.coordinate.get("direction", 0.0)) - self._start_direction)
            tol = math.radians(TURN_DONE_TOL_DEG)
            done = (progressed >= self._target_rad - tol if self._target_rad >= 0.0
                    else progressed <= self._target_rad + tol)
            if done:
                self._state = "shoot"
                self._watching = False
                return Command(0, 0, Status.NG)
            elapsed = time.time() - self._turn_started_at
            if elapsed > TURN_TIMEOUT_SEC:
                # 回りきらない（滑っている・弱い・ぶつかっている）。**ここで諦めて撮り直す。**
                # 固まると区間の SCT まで何もできない
                logger.warning("旋回が %.1f 秒で終わらない（目標 %+.1f度、進み %+.1f度）→ 撮り直す",
                               TURN_TIMEOUT_SEC, math.degrees(self._target_rad),
                               math.degrees(progressed))
                self._state = "shoot"
                self._watching = False
                return Command(0, 0, Status.NG)

            # **先読み停止。** 空走のぶん手前で力を切る。
            #
            # 速さは旋回の始めからの平均で見る（一定のPWMで回しているのでほぼ一定。
            # 1周期の差分は 10ms きざみで荒く、止め際に暴れる）。始めは進みが 0 なので
            # 速さも 0 になり、**早切りしない**。
            remain = abs(self._target_rad) - abs(progressed)

            if TURN_LEAD_TIME > 0.0 and elapsed > 0.0:
                rate = abs(progressed) / elapsed            # [rad/s]
                if remain <= rate * TURN_LEAD_TIME:
                    logger.info("旋回の先読み停止（残り %+.1f度、%.0fdeg/s）",
                                math.degrees(remain), math.degrees(rate))
                    self._state = "shoot"
                    self._watching = False
                    return Command(0, 0, Status.NG)

            # **近づいたら弱める。** 回り出しは強く、詰めは穏やかに
            power = self._turn
            if remain <= math.radians(TURN_APPROACH_DEG):
                power = min(power, max(TURN_APPROACH_PWM, MIN_TURN_PWM))

            return Command(0.0, power if self._target_rad >= 0.0 else -power, Status.NG)

        # move: 寄る／探して前へ出る
        if self._watching and self._watch():
            logger.info("前へ出る最中に見えた → 止まって測り直す")
            return self._back_to_shoot()

        if distance_now - self._start_distance >= self._step_mm:
            self._state = "shoot"
            self._watching = False
            return Command(0, 0, Status.NG)
        turn = max(-turn_gain, min(turn_gain, self._bearing_deg * 2.0))
        return Command(fwd, turn, Status.NG)


def _prune(directory: str, keep: int) -> None:
    """古い絵から消す。走行を重ねてもディスクを埋めない"""
    try:
        files = sorted(f for f in os.listdir(directory)
                       if f.startswith("sumo_") and f.endswith(".jpg"))
        for name in files[:-keep]:
            os.remove(os.path.join(directory, name))
    except OSError:
        pass


def attach(frame_source: Optional[Callable[[], Any]] = None,
           geometry: CameraGeometry = ROBOT_CAMERA,
           run_later: Callable[[Callable[[], None], float], None] = _run_later_with_thread,
           sno: int = SUMO_SNO) -> SumoApproach:
    """
    力士寄せを remote_control に差す。差した制御関数を返す。

    :param frame_source: 映像（BGR の numpy 配列）を返す関数。省略すると実機の前面カメラ
    :param geometry: カメラの取り付け
    :param run_later: fn を delay_sec 秒後に呼ぶ関数。省略すると別スレッド
    :param sno: 担当する SNO
    """
    source = frame_source or _robot_frame_source()
    control = SumoApproach(make_locator(source, geometry), geometry, run_later, sno)
    set_controller(control)
    logger.info("ET相撲の力士寄せを差しました（SNO %d）", sno)
    return control
