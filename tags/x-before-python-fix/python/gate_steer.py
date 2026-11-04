"""
見えたゲートの中心へ寄せる（2026・ETラリー）。

`gate_finder` は「このまま進んだら開きの中心からどれだけ外れて通るか」（`offset_mm`）と
「柱までの余裕」（`margin_mm`）を測る。**測るだけでは通れない。** ここがその観測を
走りへ返す側で、**向き保持直進（FUNCNO 19）の目標を少し振って、中心へ寄せる。**

【2段階でないと効かない】
横へ平行移動する走り方が無いので、向きを振って寄せる。**だが振りっぱなしで着くと、
斜めに入るぶん通れる幅が減る**（1度あたり 3.8mm。10°で 15.3mm しか残らない）。
**振って、戻す。**

  遠い側（`SQUARE_UP_MM` より先）  横のずれを消す傾きを狙う。**`MAX_TILT_DEG` で頭打ち**
  近い側（`SQUARE_UP_MM` 以内）    狙う傾きを 0 にする＝正面へ戻す

**どちらも「いまの傾きから狙う傾きへ動かす」**という同じ式。横だけを見て命じると、
すでに傾いているぶんが積み上がる（2026-09-23 19:33 に 22° まで育って失敗した）。

戻すので到着時の傾きが残らず、**そのぶん大きく振れる**。以前の「2回（6°）まで」は
戻さない作りの歯止めだった。

**どちらの段も、測った値だけで決める（閉ループ）。** 振った量は数えない
（C++ は 3°ずつしか当てず、間に旋回が入ると拒否するので、数えると実際とずれる）。

【出す先は既存の1本】
`HeadingTask.publish()` に載せる。**向きを直す口はそこ1つ**にしてあるので
（`feature-heading-one-script-E`）、通し番号もそこで揃う。C++ 側は
`CMD_HEADING_CORRECT` として受け、`HEADING_CORRECT_STEP_DEG`（3°）ずつ当てる。

【使うのは lateral_mm。offset_mm ではない】
`lateral_mm` は「走行体から見たゲート中心の横位置」（右が正）で、狙いを中心へ向けるのに
いちばん素直な量。**直角に入るときは `offset_mm = -lateral_mm` で同じ値**になるが、
`offset_mm` は**ゲートの線に沿って**測るので、斜めに入ると `1/cos θ` に育って
**過補正になる**（35° で 1.22倍。J の指摘）。

【符号】
ゲート中心が右（`lateral_mm` が正）なら右へ寄せたい＝向きを増やしたい。
C++ は `plan.shift(-dev)` なので、**負の `dev` を出すと目標は右へ動く。**
したがって `dev = atan2(-lateral_mm, distance_mm)`。

`distance_mm` は**車軸基準**（`gate_finder` が `CAMERA_TO_AXLE_FORWARD_MM` で戻している）。
atan2 の分母としてはこちらが正しい原点。

【既定は False】
**実機で一度も動かしていない。** `gate_steer.txt` に 1 を置くと効く。
カメラの幾何（俯角）が狂っていると `gate_finder` 自体が黙るので、まず
そちらを直してから試すこと（2026-09-23 に俯角が 25.8°→21.5° に動いた）。
"""

import collections
import dataclasses
import logging
import math
import statistics
import time
from typing import Any, Optional

try:
    from gate_finder import POST_SPAN_MM
except ImportError:                         # パッケージとして読み込まれた場合
    from .gate_finder import POST_SPAN_MM

logger = logging.getLogger(__name__)

#: 【2026-09-23 15時: 測りの 1.28倍の膨らみが直った】
#:
#: `gate_finder` が返す距離と横のずれは、**柱の色が床から 45mm 上で始まる**ぶん
#: `205/(205-45) = 1.281` 倍に膨らんでいた（J が据え置きのメジャーで確認。
#: 500mm→642mm、1000mm→1283mm。**距離によらない一定の倍率**）。`ed63f86` で直っている。
#:
#: **振る角度そのものは影響を受けない。** `atan2(lateral, distance)` は分子も分母も
#: 同じ倍率で膨らむので比が変わらない。**もともと正しかった。**
#:
#: **効くのは、ここの3つのしきい値のほう。** どれも実寸[mm]で決めてあるので、
#: 測りが実寸に直った**いま初めて意図どおりに効く**（それまでは 1.28倍の値と
#: 比べていた＝実寸では MIN 195mm / MAX 703mm 相当の門になっていた）。
#:
#: `margin_mm` も絶対値なので、**それまで過大に「当たる」と出ていた。**

#: 中心へ寄せるか。**やめるときはここか置き場を False / 0 にする**
ENABLE = False

#: 走行体で切り替える置き場（1行目に 1 か 0）。git の管理外なので pull で戻らない
ENABLE_SWITCH = "./sorot_spike/gate_steer.txt"

#: これより余裕があるなら触らない[mm]。**通れている接近を崩さないための歯止め**
#:
#: 車体の幅 135mm、**足元のすき間 230mm**（`gate_finder.FOOT_GAP_MM`）で、
#: まっすぐなら片側 47.5mm。**芯々 258mm から出すと 52.5mm になって 5mm 甘い**
#: （`Docs/robot_dimensions.md`）。`margin_mm` は `gate_finder` が**車体の半幅 67.5mm と
#: 傾きぶん（1度あたり 3.8mm）まで込みで**出す余裕なので、そこに 15mm 残っていれば触らない。
#:
#: **2026-09-23 に margin の定義が変わった**（半幅 60.0→67.5、傾きを見込む。J の 17394db）。
#: 同じ接近でも margin は 7.5mm 以上小さく出るので、**以前より寄せが多く発動する。**
#: それが正しい挙動
SAFE_MARGIN_MM = 15.0

#: 1回に出す向きの上限[度]。**C++ は1回に 3° しか当てない**ので、それ以上出しても切られる
#: （`Utillity/HeadingHold.h` の `plan.shift(-step)`）。切られると Python 側の見込みと
#: ずれるので、こちらでも 3° に丸めてから出す
STEP_DEG = 3.0

#: これ未満なら出さない[度]。もう合っている
MIN_STEP_DEG = 0.4

#: **ここから先は横へ寄せるのをやめ、正面へ戻すことだけをする**[mm]。
#:
#: これが「寄せ」の要。**振りっぱなしで着くと、斜めに入るぶん通れる幅が減る**
#: （1度あたり 3.8mm。10°で 15.3mm しか残らない）。横のずれを消したあと、
#: 残りのこの距離で姿勢を正せば、**到着時の傾きはほぼ 0 になり、余裕を食わない。**
#:
#: 350mm は、寄せをやめてから 3°×数回 で正面へ戻れる長さ。短くすると戻りきらず、
#: 長くすると横を詰める距離が減る
SQUARE_UP_MM = 350.0

#: **到着してよい傾きの上限[度]。これが寄せ全体の枷。**
#:
#: 車体は幅135mm・車軸が前寄り（前80/後220）なので、斜めに入ると後角が張り出す。
#: **通れる横ずれは傾きだけで決まる**（`gate_finder.body_half_span_mm`）。
#:
#: **現地の実測（すき間 230mm ＝ 芯々 248mm）で引き直した**（2026-09-23 夜）:
#:
#:   傾き   0°    4°    8°   10°   12°   13°   15°   22°
#:   横ずれ 47.5  32.3  17.5  10.3   3.2  -0.3  -7.1 -31.0   ← 12.93°以上はどこを通っても入らない
#:
#: **組立図から起こした 258mm 前提では 14.37° だった。1.4° 詰まった。**
#:
#: **2026-09-23 19:33 の走行で、1周目の黄で 22° に達して失敗した。**
#: 横は -174mm → +29mm と消せたのに、そのとき斜めが -22.0° まで育っていた。
#: 横を詰めることだけを命じ、**いまの傾きを見ていなかった**のが原因。
#:
#: 8° なら横 17.5mm まで許される。ここを超えて振るのは、**横が消えても入れない**ので損。
#: **限界 12.93° までの余白は 4.9° しかない**（258mm 前提のときは 6.4° あった）。
MAX_TILT_DEG = 8.0

#: 横を詰める残りの距離の下限[mm]。0 で割らないためと、近すぎる所で角度が跳ねないため
MIN_REACH_MM = 120.0

#: これより近いと振っても間に合わない[mm]。振るだけ無駄なのでやめる
#:
#: **損得の分かれ目は 218mm で、振る角度に依らない**（J の計算）。向きを θ 振って
#: 距離 d を走れば横は `d·sinθ` 縮むが、斜めに入るぶん余裕が `3.8·θ[deg]` 減る。
#: 釣り合うのは `d = 3.8 × 180/π ≒ 218mm`。**そこより手前で振っても損。**
MIN_DISTANCE_MM = 250.0

#: これより遠い測りでは振らない[mm]
#:
#: **理由は測りの質ではなく経路の形。** ゲートを通る行はいちばん長いもので 1106mm、
#: **ゲート線の 856mm 手前から始まる。** 900mm より遠い測りは「ゲートの行より前」で、
#: **あいだに旋回が入る**ので、そこで振った向きは次の旋回で置き換わる。
MAX_DISTANCE_MM = 900.0

#: 正面からこれ以上ずれて見ているときは信じない[度]
#:
#: 2026-09-23 の走行（50件）で、間隔（正 258mm）が 0〜10° では中央値 254mm、
#: 25°以上では 204〜307mm とばらついた。**そのとき横位置も信じられない。**
MAX_APPROACH_DEG = 20.0

#: 同じゲートへ何回まで出すか（暴走よけ）。
#:
#: **以前の 2回は「振ったぶんを戻さない」作りの歯止めだった。** いまは閉ループで、
#: 横が詰まれば出す角度が自然に小さくなり、`SQUARE_UP_MM` で正面へ戻る。
#: ここは「測り間違いで延々と振り続ける」ことだけを止める安全弁
MAX_STEPS_PER_GATE = 12


#: 同じ向きへ上限で押し続けてよい回数。
#:
#: **傾きの持ち分（`MAX_TILT_DEG`）が効いていれば、同符号の上限張り付きは
#: 3回で終わる**（0° から 8° まで 3°刻みで3回）。それを超えて同じ向きへ
#: 押し続けるのは、測りのほうが偏っている（カメラの取り付けのヨー、
#: `approach_deg` の系統誤差）ときだけ。
#:
#: **見つけても「出すのをやめる」のは間違い。** 寄せを止めても傾きは戻らず、
#: 傾いたまま着いて余裕を食う（1度 3.8mm）。**横を詰めるのをあきらめ、
#: 正面へ戻す段だけ残す。** 斜めをやめるのは、いつやっても余裕が増える
MAX_SATURATED_STEPS = 4



#: 間隔がこれ以上ずれていたら、その測りでは寄せない。
#:
#: **ゲートの間隔は、観測の良し悪しをその場で測れる唯一の物差し。** 258mm は
#: コースに置かれた既知の長さで、走行体が何をしていても変わらない。報告が
#: そこから外れていれば、同じ測りから出た `lateral_mm` も信じられない。
#:
#: **`gate_check` の判定には斜め 15° の門があったが、寄せには門が1つも無かった。**
#:
#: 2026-09-23 の3走行 142件の実測は中央値 269mm で、**真値 258mm に対して +4.3%**。
#: この倍率誤差はまだ直っていない（`POST_COLOR_BASE_MM` か俯角の零点）。
#:
#: **そのあいだ、門を真値ぴったりに狭く張ってはいけない。** 測りが一様に上へ
#: 寄っているので、狭く張ると「遠いゲートほど落ちる」偏った門になる
#: （±30mm で 60%、±40mm で 74%、±50mm で 86%）。**倍率を直すまでの暫定で ±50mm。**
#: 直したら ±30mm まで詰めること。19:34:01 の 322mm（斜め -60°）はどれでも落ちる
MAX_SPAN_ERROR_MM = 50.0

#: 何枚の中央値で寄せるか。
#:
#: **1枚で寄せてはいけない。** 2026-09-23 の3走行で、連続するコマの横位置は
#: まっすぐな動きから ±33mm（最大 83mm）外れていた。**通れる横ずれが 47.5mm**
#: しかないので、1枚の雑音のほうが守ろうとしている幅より大きい。
#: 3枚の中央値で ±18mm 相当まで落ちる。0.15秒間隔なので 0.3秒＝`FWD 60` で 18mm 進む
MEDIAN_SAMPLES = 3


#: 同じゲートへ出してよい総量[度]。**これが本当の枷。**
#:
#: `MAX_TILT_DEG` は「狙う角度」を抑えるだけで、**出した総量は抑えていなかった**
#: （2026-09-23 20:51 の走行で判明）。C++ は1通ごとに目標を `STEP_DEG` 動かすので、
#: **出した数だけ目標が動く。**
#:
#:   20:50:14  +3.00°（斜め -4.5°）
#:   20:50:14  +3.00°（斜め -4.5°）  ← 測りが動いていないのに押し続ける
#:   20:50:14  +3.00°（斜め -3.9°）
#:
#: 測りは 0.15秒ごと、走行体が 3° 回るのはそれより遅い。**測りが追いつく前に
#: 次を出すので、目標だけが先へ行く。** 1秒で7回出れば目標は 21° 動く。
#: 到着時の傾きが -24.2° まで育ったのはこれ（限界 12.93°）。
#:
#: **総量を持ち分と同じ 8° で止める。**
#:
#: **ただし「正面へ戻す」段はこの枷に縛られない。** 止めると、寄せが触る前から
#: 傾いて着いたときに斜めを消せない（2026-09-23 22:16 の走行で、黄に
#: **-17.7° 傾いたまま着いた**。寄せは1回も出していない）。
#: 戻す段は狙いが 0° で動かないので、測りが追いつけば自然に止まる。
#: **追う段だけが、狙いが逃げていく＝暴走する。**
MAX_TOTAL_DEG = MAX_TILT_DEG

#: 同じゲートへ続けて出すまでの間隔[秒]。
#:
#: **測りが1枚更新されるより速く出しても、同じ測りを見て二度押すだけ。**
#: 3°の指令が当たって、それがカメラに写るまでを待つ
MIN_INTERVAL_S = 0.5


class GateSteer:
    """見えたゲートの中心へ寄せる。走行体で1つだけ使う"""

    _instance: Optional["GateSteer"] = None

    @classmethod
    def get_instance(cls) -> "GateSteer":
        if cls._instance is None:
            cls._instance = cls()
        return cls._instance

    @classmethod
    def reset_instance(cls) -> None:
        cls._instance = None

    def __init__(self, publish=None):
        #: 向きのずれを載せる先。**既定で HeadingTask へ繋ぐ**（口は1つにしてある）。
        #:
        #: **ここを None のままにしてはいけない。** 2026-09-23 17:56 の走行で、
        #: `get_instance()` が引数無しで作るため None のままになり、**27回すべてが
        #: 出ていなかった**（C++ 側の log.txt に 1件も届いていない）。
        #: それでもログには「寄せます」と出るので、**ログだけでは気づけない。**
        self._publish = publish if publish is not None else default_publish
        #: 色ごとに、何回振ったか
        self._nudges = {}
        #: 色ごとに、出した総量[度]（符号つき）
        self._applied = {}
        #: 色ごとに、最後に出した時刻
        self._last_at = {}
        #: 色ごとに、直近の測り（中央値を採るため）
        self._recent = {}
        #: 色ごとに [同符号で上限に張り付いた向き, その回数]
        self._saturated = {}
        #: 横を詰めるのをあきらめた色（正面へ戻す段だけ残す）
        self._give_up = set()

    def enabled(self) -> bool:
        """置き場を見る。無ければ ENABLE"""
        try:
            with open(ENABLE_SWITCH, encoding="utf-8") as f:
                return f.readline().strip() == "1"
        except OSError:
            return ENABLE

    def forget(self) -> None:
        """周が変わるなどで数え直すとき"""
        self._nudges = {}
        self._applied = {}
        self._last_at = {}
        self._recent = {}
        self._saturated = {}
        self._give_up = set()

    def consider(self, sighting: Any) -> Optional[float]:
        """
        1回ぶんの測りを見て、必要なら向きを振る。

        :param sighting: `gate_finder.GateSighting`
        :returns: 振った角度[度]。振らなければ None
        """
        if not self.enabled():
            return None
        why = self._skip_reason(sighting)
        if why is not None:
            logger.debug("ゲートへ寄せませんでした（%s）", why)
            return None

        sighting = self._smoothed(sighting)
        if sighting is None:
            logger.debug("ゲートへ寄せませんでした（まだ %d枚そろっていない）", MEDIAN_SAMPLES)
            return None

        want_deg, phase = self._want(sighting, sighting.color in self._give_up)
        if self._note_saturation(sighting.color, want_deg):
            # あきらめたばかり。横を詰めずに正面へ戻す向きへ出し直す
            want_deg, phase = self._want(sighting, True)
        dev_deg = max(-STEP_DEG, min(STEP_DEG, want_deg))
        # **戻す段は、測った傾きのぶんまで出してよい**（狙いは 0° で動かない）
        budget = (max(MAX_TOTAL_DEG, abs(sighting.approach_deg or 0.0))
                  if "戻します" in phase else MAX_TOTAL_DEG)
        dev_deg = self._within_total(sighting.color, dev_deg, budget)
        if abs(dev_deg) < MIN_STEP_DEG:
            logger.debug("ゲートへ寄せませんでした（もう合っている: %s %+.2f°）", phase, want_deg)
            return None

        now = time.time()
        since = now - self._last_at.get(sighting.color, 0.0)
        if since < MIN_INTERVAL_S:
            # **測りが追いつく前に二度押さない。** 同じ測りを見て押すだけになる
            logger.debug("ゲートへ寄せませんでした（前回から %.2f秒。%.1f秒待つ）",
                         since, MIN_INTERVAL_S)
            return None
        self._last_at[sighting.color] = now
        self._applied[sighting.color] = self._applied.get(sighting.color, 0.0) + dev_deg

        n = self._nudges.get(sighting.color, 0) + 1
        self._nudges[sighting.color] = n
        if self._publish is None:
            # **出せないのに「寄せます」と書かない。** 黙って効かない状態が続く
            logger.error("ゲートへ寄せる先が繋がっていません（publish が None）")
            return None
        self._publish(dev_deg, source="gate", markers=0, elapsed_ms=0.0)
        logger.info("ゲートへ%s %+.2f°（残り %+.2f°・%s・中心は横 %+.0fmm・斜め %+.1f°・"
                    "余裕 %+.0fmm・%.0fmm 先・%d回目）",
                    phase, dev_deg, want_deg - dev_deg, sighting.color,
                    sighting.lateral_mm, sighting.approach_deg or 0.0,
                    sighting.margin_mm, sighting.distance_mm, n)
        return dev_deg

    def _within_total(self, color: str, dev_deg: float, budget: float) -> float:
        """
        出した総量が持ち分を越えないように削る。

        **C++ は1通ごとに目標を動かす。** 測りが追いつかないので「いま何度
        傾いているか」だけでは抑えられない。**出した数を数えるしかない。**
        戻す向き（総量が減る向き）は削らない。
        """
        total = self._applied.get(color, 0.0)
        if abs(total + dev_deg) <= abs(total):
            return dev_deg                      # 戻す向き。いつでも通す
        room = budget - abs(total)
        if room <= 0.0:
            logger.warning("ゲート（%s）へ出した総量が %+.1f°（持ち分 %.1f°）で"
                           "いっぱいです。**これ以上は目標だけが先へ行く**",
                           color, total, budget)
            return 0.0
        return math.copysign(min(abs(dev_deg), room), dev_deg)

    @staticmethod
    def _want(s: Any, square_up_only: bool = False) -> tuple:
        """
        いま出したい向き[度]と、どちらの段か。

        **2段階にするのが要。**

          遠い側（`SQUARE_UP_MM` より先）  横のずれを消す。`SQUARE_UP_MM` の地点で
                                          横が 0 になる角度を出す
          近い側（`SQUARE_UP_MM` 以内）    横は触らず、**正面へ戻す**。測った
                                          `approach_deg` を打ち消せば、到着時の傾きが
                                          残らず、通れる幅を食わない

        **どちらも測った値だけで決める（閉ループ）。** 振った量を数えないのは、
        C++ が 3°ずつしか当てず、しかも間に旋回が入ると拒否するため
        （`Utillity/HeadingHold.h`）。数えると実際とずれる。
        """
        傾き = s.approach_deg or 0.0
        if s.distance_mm > SQUARE_UP_MM and not square_up_only:
            # 横を詰めるのに要る傾き。**到着してよい上限で頭打ちにする**
            reach = max(s.distance_mm - SQUARE_UP_MM, MIN_REACH_MM)
            狙う傾き = math.degrees(math.atan2(s.lateral_mm, reach))
            狙う傾き = max(-MAX_TILT_DEG, min(MAX_TILT_DEG, 狙う傾き))
            phase = "中心へ寄せます"
        else:
            狙う傾き = 0.0
            phase = "正面へ戻します（横は詰めません）" if square_up_only else "正面へ戻します"
        # **いまの傾きから狙う傾きへ動かす。** 横だけを見て命じると、すでに傾いている
        # ぶんが積み上がって 22° まで育つ（2026-09-23 19:33 の失敗）
        return 傾き - 狙う傾き, phase

    def _smoothed(self, s: Any) -> Optional[Any]:
        """
        直近 `MEDIAN_SAMPLES` 枚の中央値にした測りを返す。まだそろっていなければ None。

        **平均ではなく中央値。** 外れ値は「少しずれる」のではなく「柱でないものを
        柱として組にした」形で来るので、1枚で大きく飛ぶ。平均だと引きずられる。
        """
        q = self._recent.setdefault(s.color, collections.deque(maxlen=MEDIAN_SAMPLES))
        q.append((s.distance_mm, s.lateral_mm, s.approach_deg or 0.0))
        if len(q) < MEDIAN_SAMPLES:
            return None
        d, l, a = (statistics.median(v[i] for v in q) for i in range(3))
        return dataclasses.replace(s, distance_mm=d, lateral_mm=l, approach_deg=a) \
            if dataclasses.is_dataclass(s) else _Smoothed(s, d, l, a)

    def _note_saturation(self, color: str, want_deg: float) -> bool:
        """
        同じ向きへ上限で押し続けていないか数える。

        :returns: この1回であきらめに入ったら True
        """
        if color in self._give_up:
            return False
        if abs(want_deg) <= STEP_DEG:
            self._saturated.pop(color, None)
            return False
        sign = 1 if want_deg > 0 else -1
        last_sign, count = self._saturated.get(color, (sign, 0))
        count = count + 1 if last_sign == sign else 1
        self._saturated[color] = (sign, count)
        if count < MAX_SATURATED_STEPS:
            return False
        self._give_up.add(color)
        logger.warning("ゲート（%s）へ同じ向きへ %d回 上限で押しました。"
                       "**測りが偏っている疑い**（横位置か斜め）。"
                       "横を詰めるのをやめ、正面へ戻すだけにします", color, count)
        return True

    def _skip_reason(self, s: Any) -> Optional[str]:
        if s is None:
            return "測れていない"
        if s.offset_mm is None or s.margin_mm is None:
            return "中心からのずれが出ていない（ゲートと並んでいる）"
        # **余裕の門は「横を詰める」段だけ。** 正面へ戻す段は、余裕があっても
        # 斜めのまま着かないようにしたい（斜めはそれ自体が余裕を食う）
        if s.distance_mm > SQUARE_UP_MM and s.margin_mm >= SAFE_MARGIN_MM:
            return "余裕が %+.0fmm ある（%.0fmm 以上は触らない）" % (s.margin_mm, SAFE_MARGIN_MM)
        # **「近すぎる」の門は横を詰める段だけ。** 正面へ戻すのは近くても損しない
        # （斜めをやめるのは、いつやっても余裕が増える。損得の分かれ目が無い）
        if s.distance_mm > SQUARE_UP_MM and s.distance_mm < MIN_DISTANCE_MM:
            return "%.0fmm まで近く、振っても間に合わない" % s.distance_mm
        if s.distance_mm > MAX_DISTANCE_MM:
            return "%.0fmm 先で、ゲートの行より前（あいだに旋回が入る）" % s.distance_mm
        span = getattr(s, "span_mm", None)
        if span is not None and abs(span - POST_SPAN_MM) > MAX_SPAN_ERROR_MM:
            # **ゲートは剛体。** 間隔が合わない測りは、横位置も信じられない
            return "間隔が %.0fmm（正 %.0fmm）で、測りが信じられない" % (span, POST_SPAN_MM)
        if s.approach_deg is not None and abs(s.approach_deg) > MAX_APPROACH_DEG:
            return "斜めから見ている（%.1f°）" % s.approach_deg
        if self._nudges.get(s.color, 0) >= MAX_STEPS_PER_GATE:
            return "同じゲートへ %d回 出した（安全弁）" % MAX_STEPS_PER_GATE
        return None


class _Smoothed:
    """dataclass でない測りを渡されたときだけ使う、読み出しの上書き"""

    def __init__(self, base: Any, distance_mm: float, lateral_mm: float, approach_deg: float):
        self._base = base
        self.distance_mm, self.lateral_mm, self.approach_deg = distance_mm, lateral_mm, approach_deg

    def __getattr__(self, name):
        return getattr(self._base, name)


def default_publish(deviation_deg: float, **kw) -> None:
    """向きを直す口（`HeadingTask.publish`）へ載せる。**口は1つにしてある**"""
    from heading_task import HeadingTask
    HeadingTask.get_instance().publish(deviation_deg, **kw)
