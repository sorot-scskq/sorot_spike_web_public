"""
リモートコントロール走行の見本（2026・ETラリー）。

**そのまま競技に使うものではない。** リモート側の書き方と、ETラリーで
「経路が出来るまで待たせる」やり方を示すためのもの。

【なぜETラリーでリモートコントロールを使うのか】
ETラリーのゲート配置は当日まで分からない（規約6.3.1）。経路をPCに作って
もらう作り（Docs/et_rally_route.md）は「頼んで、届くまで待って、挿し込む」で、
待っているあいだ走行体は止まっている。

リモートコントロール走行なら、**経路を作りながら走らせておける**。
区間の終わりに Status.WAIT_SCENARIO を返すと、走行体は次のコマンドへ進みつつ
「シナリオ待ち」を立て、毎周期の観測でそれを知らせ続ける。出来た時点で
insert_scenario() を呼べば、その場で走行中のシナリオへ入る
（Docs/scenario_update_pipeline.md 5.2）。

    obs.awaiting_scenario == True のあいだ、走行体は経路を待っている
    　→ 出来たら insert_scenario(コマンド列)

【使い方】
ev3_python の起動後に、このモジュールを読み込んで差す。走行体で試すなら
Common/ev3_python.py の起動直後に import する行を足すか、
config_2026.start_recognizers() から呼ぶ。

    import remote_rally_sample
    remote_rally_sample.attach()
"""

from __future__ import annotations

import logging

from remote_control import Command, Status, insert_scenario, set_controller

logger = logging.getLogger(__name__)


class RallyController:
    """
    ETラリーの区間を、経路が出来るまで待たせながら走らせる見本。

    経路を作るのは PC 側なので、ここでは「出来たことにする」代わりに
    set_route() で外から渡してもらう形にしてある。実際には PC からの応答
    （Docs/et_rally_route.md）や、自分で組み立てた結果を入れる。
    """

    def __init__(self, rally_sno: int = 30):
        #: この SNO の区間に入ったら、経路の待ち合わせを始める
        self.rally_sno = rally_sno
        #: 走らせたい経路。まだ出来ていなければ None
        self.route = None
        #: 待ち合わせを始めたか（1区間につき1回だけ WAIT_SCENARIO を返す）
        self._asked = False

    def set_route(self, commands) -> None:
        """出来上がった経路を渡す。次の観測で走行体へ届く"""
        self.route = commands

    def __call__(self, obs) -> Command:
        """毎周期呼ばれる。1周期ぶんの指令を返す"""
        # 走行体が待っていて、経路が出来ていれば渡す。
        # 渡すのは1回だけ（insert_scenario 後に None へ戻す）
        if obs.awaiting_scenario and self.route is not None:
            insert_scenario(self.route)
            self.route = None
            logger.info("経路を渡しました（SNO %d の待ちを解きます）", obs.sno)

        # ETラリーの区間に入った。経路がまだ無ければ、待たせたまま先へ進める
        if obs.sno == self.rally_sno and not self._asked and self.route is None:
            self._asked = True
            logger.info("経路がまだ出来ていません。シナリオ待ちにします")
            # forward / turn を省くと「Status だけを伝える指令」になり、
            # いまの走行指令をそのまま保つ（0 で上書きして止めない）
            return Command(status=Status.WAIT_SCENARIO)

        return self.edge_trace(obs)

    # --- 走り方 ---------------------------------------------------------------

    def edge_trace(self, obs) -> Command:
        """
        左エッジ追従（P制御だけ）。

        校正値の真ん中を狙う。ゲインと基本速度はシナリオの RunInfo をそのまま使う
        （どう使うかを決めるのはリモート側なので、既存の走行方法と同じ意味にしてある）。
        """
        calib = obs.calibration
        if not calib:
            # 校正がまだ。走らせない（指令を出さなければ走行体は止まる）
            return Command(forward=0, turn=0, status=Status.NG)

        black = calib.get("black", 0)
        white = calib.get("white", 0)
        width = white - black
        if width == 0:
            return Command(forward=0, turn=0, status=Status.NG)

        target = (black + white) / 2.0
        deviation = target - obs.color.get("value", target)

        kp = obs.run_info.get("KP", 0) or 0
        forward = obs.run_info.get("FWD", 0) or 0

        # 偏差を輝度の幅で正規化してからゲインを掛ける。校正値が変わっても
        # 同じゲインが効くようにするため
        turn = (deviation / width) * kp * 100.0

        return Command(forward=forward, turn=turn, status=self.end_status(obs))

    def end_status(self, obs) -> str:
        """
        区間を終える条件。シナリオの SCD（距離）と SCT（時間）を見る。

        どちらも 0 なら終わらない（リモート側が別の判断で OK を出すまで走り続ける）。
        """
        scd = obs.switch_info.get("SCD", 0) or 0
        sct = obs.switch_info.get("SCT", 0) or 0

        if scd != 0 and abs(obs.coordinate.get("distance", 0)) >= abs(scd):
            return Status.OK
        if sct != 0 and obs.elapsed_ms >= sct:
            return Status.OK
        return Status.NG


def attach(rally_sno: int = 30) -> RallyController:
    """見本の制御を差す。戻り値の set_route() で経路を渡せる"""
    control = RallyController(rally_sno)
    set_controller(control)
    return control
