"""
リモートコントロール走行のリモート側（走行体上の Python）。

走らせ方を C++ の中で計算せず、**毎周期の FWD/TRN をここで決める**。
C++ 側の窓口は RoughSpot/Cpp/RemoteControlBridge で、やりとりは専用タスク
（app.cpp の remote_control_task）とファイルで行う（ソケットは使わない）。

【sorot_spike_web での扱い】
実機（sorot_spike の RoughSpot/Python/Common/remote_control.py）をそのまま持ってきたもの。
**実機と違うのはアームの直接操作（set_arm_pwm / release_arm）を足したところだけ。**
実機の C++ にはまだ同じ口が無いので、実機へ戻すときは C++ 側も足すこと
（docs/リモートコントロール走行(RemoteControl)手順.md の「アームの直接操作」）。
シミュレータではファイルのやりとりを使わず、sim/pyscript/et_sumo_bridge.py が
handle_observation() を毎周期直接呼ぶ。

    C++ ── /dev/shm/sorot_remote_obs.json（最新の観測だけ）──────▶ handle_observation()
        ◀─ /dev/shm/sorot_remote_cmd.<通し番号>.json（応答を1通ずつ）─

【往復は10ms周期。ここで待たせないこと】
C++ 側の制御ループは 10ms。やりとりタスクは制御ループとは別に回っているので
ここで多少待っても制御周期は崩れないが、待たせたぶんだけ指令が古くなる。
**制御関数の中でファイルを読んだり通信したりしないこと。**

【指令が途切れたら止まる】
C++ 側は 200ms（RemoteControlBridge::HOLD_MS）指令が来なければ停止する。
制御関数が例外を投げた周期は指令を出さないので、投げ続ければ止まる。
「動かない」側に倒してある。

【区間の終わり方（Status）】
    Status.NG             まだ終わっていない。いまのコマンドを続ける（既定）
    Status.OK             この区間は終わり。シナリオ上の次のコマンドへ移る
    Status.WAIT_SCENARIO  次のコマンドへ移るが、走らせたいシナリオはまだ出来ていない

WAIT_SCENARIO を返すと、C++ は「シナリオ待ち」を立て、**毎周期の観測に
awaiting_scenario=True を載せ続ける**。観測そのものがポーリングになっているので、
シナリオが出来たかを問い合わせる経路を別に作る必要はない。出来たら
insert_scenario() を呼ぶと、次の応答に載って走行体へ届く
（Docs/scenario_update_pipeline.md 5.2）。

【使い方】

    from remote_control import Command, Status, controller, insert_scenario

    @controller
    def control(obs):
        # 校正値の真ん中を狙う左エッジ追従
        target = (obs.calibration["black"] + obs.calibration["white"]) / 2
        deviation = target - obs.color["value"]
        return Command(
            forward=obs.run_info["FWD"],
            turn=deviation * obs.run_info["KP"],
            status=Status.OK if obs.coordinate["distance"] >= obs.switch_info["SCD"]
                   else Status.NG,
        )

状態（差してある制御関数・溜めたメッセージ・応答の通し番号）は RemoteControl が
持つ。走行体では1つだけ使うので、上の書き方（モジュールの関数）はその1つへの
差し込み口になっている。
"""

from __future__ import annotations

import json
import logging
import os
import threading
import time
from typing import Any, Callable, Optional

try:
    import config
except ImportError:  # パッケージとして読み込まれた場合
    from . import config

logger = logging.getLogger(__name__)


class Status:
    """区間の終わり方。C++ 側の RemoteStatus と同じ文字列にすること"""

    NG = "ng"
    OK = "ok"
    WAIT_SCENARIO = "waitScenario"


class Command:
    """
    1周期ぶんの指令。

    forward / turn を省くと「Status だけを伝える指令」になり、走行体は
    いまの走行指令をそのまま保つ。0 で上書きすると、「まだ終わっていない」と
    伝えたつもりが走行を止めてしまう。

    :param forward: FWD 相当（±100）
    :param turn:    TRN 相当（±100）。その場旋回では各輪の PWM になる
    :param status:  Status のいずれか
    """

    __slots__ = ("forward", "turn", "status")

    def __init__(self, forward: Optional[float] = None, turn: Optional[float] = None,
                 status: str = Status.NG):
        self.forward = forward
        self.turn = turn
        self.status = status

    def to_message(self, cmd_seq: Optional[int] = None) -> dict:
        msg: dict = {"type": "cmd", "status": self.status}
        if self.forward is not None:
            msg["forward"] = float(self.forward)
        if self.turn is not None:
            msg["turn"] = float(self.turn)
        if cmd_seq is not None:
            msg["cmdSeq"] = cmd_seq
        return msg


class Observation:
    """
    走行体の現在値。C++ の RemoteControlBridge::buildObservation が作る。

    辞書のままでも読めるが、名前で引けるようにしておく（`obs.run_info["FWD"]`）。
    知らないキーが増えても壊れないよう、元の辞書は `raw` に残す。
    """

    __slots__ = ("raw", "cmd_seq", "sno", "cno", "tick", "elapsed_ms",
                 "awaiting_scenario", "run_info", "switch_info",
                 "calibration", "color", "coordinate", "body")

    def __init__(self, raw: dict):
        self.raw = raw
        self.cmd_seq = raw.get("cmdSeq", 0)
        self.sno = raw.get("sno", 0)
        self.cno = raw.get("cno", 0)
        self.tick = raw.get("tick", 0)
        self.elapsed_ms = raw.get("elapsedMs", 0)
        #: シナリオ待ちか。True のあいだ、走らせたいシナリオを送ると走り出す
        self.awaiting_scenario = bool(raw.get("awaitingScenario", False))
        self.run_info = raw.get("runInfo") or {}
        self.switch_info = raw.get("switchInfo") or {}
        #: 校正値（黒・灰・白・ジャイロオフセット）。まだ取れていなければ None
        self.calibration = raw.get("calibration")
        self.color = raw.get("color") or {}
        self.coordinate = raw.get("coordinate") or {}
        self.body = raw.get("body") or {}


class RemoteControl:
    """
    リモート側の窓口。走行体で1つだけ使う（`get_instance()`）。

    差してある制御関数、次の応答に載せるメッセージ、応答の通し番号、
    直近の観測を持つ。
    """

    _instance: Optional["RemoteControl"] = None
    _instance_lock = threading.Lock()

    @classmethod
    def get_instance(cls) -> "RemoteControl":
        """走行体で共有する1つを返す。無ければ作る"""
        with cls._instance_lock:
            if cls._instance is None:
                cls._instance = cls()
            return cls._instance

    @classmethod
    def reset_instance(cls) -> None:
        """
        共有している1つを捨てる（テスト用）。

        **見張るスレッドも止める。** 残すと、捨てたほうと新しいほうが同じ置き場へ
        応答を書き、通し番号が衝突して応答が消える。
        """
        with cls._instance_lock:
            old, cls._instance = cls._instance, None
        if old is not None:
            old.stop_file_loop()

    def __init__(self):
        self._lock = threading.Lock()
        self._controller: Optional[Callable[[Observation], Any]] = None
        #: 次の応答に載せるメッセージ（シナリオ・PWM・解放）
        self._pending: list = []
        #: 直近の観測。制御関数の外から見たいとき用
        self.last_observation: Optional[Observation] = None
        #: 応答の通し番号。ファイル名に入れ、C++ はこの順に取り込む
        self._response_seq = 0
        #: 応答した観測の中身。同じものへ二度応答しないために覚える
        self._last_seen: Optional[str] = None
        #: 観測を見張るスレッド
        self._loop_thread: Optional[threading.Thread] = None
        self._stop = threading.Event()

    # --- 制御関数の登録 -------------------------------------------------------

    def set_controller(self, fn: Optional[Callable[[Observation], Any]]) -> None:
        """制御関数を差す。None で外す"""
        with self._lock:
            self._controller = fn if callable(fn) else None
        logger.info("リモート制御を%sしました", "登録" if fn else "解除")

    def has_controller(self) -> bool:
        return self._controller is not None

    # --- 走行体そのものを操作する ---------------------------------------------

    def insert_scenario(self, commands) -> None:
        """
        走行中のシナリオへコマンドを足す。

        **入れ替えではなく「足す」。** 入れ替えると走行が最初からやり直しになる。
        入るのは「いま走っているコマンドの直後」で、番号は受け取る側（C++）が振る。

        次の応答に載せて渡すので、実際に入るのは次の観測への応答が取り込まれたとき
        （20msほど後）。
        :param commands: コマンドセットの配列（SubScenario/*.json と同じ形）
        """
        if not isinstance(commands, list) or len(commands) == 0:
            raise ValueError("シナリオのコマンドが空です")
        with self._lock:
            self._pending.append({"type": "scenario", "commands": commands})
        logger.info("シナリオを%d件、次の応答で渡します", len(commands))

    def set_motor_pwm(self, left: float, right: float) -> None:
        """
        モータの PWM を直接書く。走らせ方の計算を通さない。

        **効くのは FUNCNO 18 の区間を走っているあいだだけ**（出しているのが
        走行方法 RemoteControl の中なので）。また、シナリオの NOBLNCE が 1 で
        あること、左右を逆向きにした「その場旋回」はその区間の TRN が 0 で
        ないことが要る（RunControl が TRN=0 の区間を停止の指示として扱うため）。

        **200ms 更新が無ければ 0 に落ちる。** 走らせ続けたい側が周期的に呼び直すこと。
        握りが外れるのは release_motor() を呼んだときだけ。
        """
        with self._lock:
            self._pending.append({"type": "pwm", "left": float(left), "right": float(right)})

    def release_motor(self) -> None:
        """直接操作をやめ、走行方法にモータを返す"""
        with self._lock:
            self._pending.append({"type": "release"})

    def set_arm_pwm(self, pwm: float) -> None:
        """
        アームの PWM を直接書く。シナリオの ARMANG より優先する。

        車輪の直接操作と同じく、**効くのは FUNCNO 18 の区間を走っているあいだだけ**で、
        **200ms 更新が無ければ 0 に落ちる。** 動かし続けたい側が周期的に呼び直すこと。
        いまの角度は観測の body["arm"]（エンコーダ値[度]）で見る。
        握りが外れるのは release_arm() を呼んだときだけ。

        **実機の C++ には未実装**（sorot_spike_web で先に足した口）。
        """
        with self._lock:
            self._pending.append({"type": "arm", "pwm": float(pwm)})

    def release_arm(self) -> None:
        """アームの直接操作をやめ、シナリオの ARMANG にアームを返す"""
        with self._lock:
            self._pending.append({"type": "armRelease"})

    # --- やりとり本体 ---------------------------------------------------------

    def handle_observation(self, request: dict) -> dict:
        """
        観測を1つ受け取り、応答を作る。

        制御関数が例外を投げても応答は返す（走行体を待たせない）。指令が無い
        周期が続けば、走行体は保持時間切れで停止側に倒れる。

        :param request: C++ から届いた観測のJSON
        :returns: {"messages": [...]} の形の応答
        """
        obs = Observation(request if isinstance(request, dict) else {})
        self.last_observation = obs

        with self._lock:
            fn = self._controller

        # 制御関数を先に呼ぶ。中から insert_scenario() や set_motor_pwm() を呼んだぶんも
        # この応答に載せるため。あとで拾うと、同じ周期で決めたことが1周期遅れる
        # （ロックの外で呼ぶこと。中から自分のロックを取るので、握ったままだと止まる）
        command = None
        if fn is not None:
            try:
                command = fn(obs)
            except Exception as e:  # noqa: BLE001 — 制御の例外で走行体を待たせない
                logger.exception("リモート制御で例外が出ました: %s", e)
                command = None

        messages = []
        with self._lock:
            if self._pending:
                messages.extend(self._pending)
                self._pending.clear()

        if isinstance(command, Command):
            messages.append(command.to_message(obs.cmd_seq))
        elif isinstance(command, dict):
            msg = dict(command)
            msg.setdefault("type", "cmd")
            msg.setdefault("cmdSeq", obs.cmd_seq)
            messages.append(msg)

        if not messages:
            # 何も無くても応答は返す（観測1つに応答1つ、と数を合わせておく）
            messages.append({"type": "nop"})

        return {"messages": messages}

    def reset(self) -> None:
        """走行やり直し用。溜めてあるメッセージを捨てる（制御関数は残す）"""
        with self._lock:
            self._pending.clear()
            self._last_seen = None
        self.last_observation = None

    # --- ファイルでのやりとり -------------------------------------------------

    def write_response(self, response: dict, cmd_dir: Optional[str] = None) -> str:
        """
        応答を1通、C++ が取り込む置き場へ置く。

        **応答は上書きしない。** 通し番号入りの名前で1通ずつ置き、C++ が順に取り込んで
        消す。上書きすると、C++ が取り込む前に次の応答で消え、シナリオの挿し込みや
        直接PWMの指示が失われる。

        :returns: 置いたファイルのパス
        """
        cmd_dir = cmd_dir or config.COMM_DIR
        with self._lock:
            self._response_seq += 1
            seq = self._response_seq
        tmp = os.path.join(cmd_dir, config.COMM_REMOTE_CMD_PREFIX + "tmp")
        path = os.path.join(cmd_dir, "%s%010d.json" % (config.COMM_REMOTE_CMD_PREFIX, seq))
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(response, f)
        os.replace(tmp, path)
        return path

    def clear_responses(self, cmd_dir: Optional[str] = None) -> None:
        """取り込まれずに残った応答を消す（前回の走行のもの）"""
        cmd_dir = cmd_dir or config.COMM_DIR
        try:
            names = os.listdir(cmd_dir)
        except OSError:
            return
        for name in names:
            if name.startswith(config.COMM_REMOTE_CMD_PREFIX):
                try:
                    os.remove(os.path.join(cmd_dir, name))
                except OSError:
                    pass
        with self._lock:
            self._response_seq = 0

    def poll_once(self, obs_file: Optional[str] = None,
                  cmd_dir: Optional[str] = None) -> bool:
        """
        観測が置き直されていたら、1回ぶん応答する。

        **置き直されたかは中身で見分ける。** 以前はファイルの inode と更新時刻で
        見ていたが、C++ は 10ms ごとに一時ファイル＋rename で置くので、
        inode が使い回され、更新時刻の粒度（tmpfs は粗い）に収まると「変わって
        いない」と見えて応答を取りこぼす。中身は毎周期変わる（経過時間が進む）ので、
        そのまま鍵にできる。

        :returns: 応答したら True
        """
        obs_file = obs_file or config.COMM_REMOTE_OBS_FILE
        try:
            with open(obs_file, encoding="utf-8") as f:
                body = f.read()
        except OSError:
            return False        # まだ置かれていない（リモートの区間に入っていない）

        with self._lock:
            if body == self._last_seen:
                return False

        try:
            request = json.loads(body)
        except ValueError as e:
            # C++ は rename で置くので書きかけは読まない。壊れていたら次の周期で読み直す
            logger.warning("リモートの観測を読めません: %s", e)
            return False

        with self._lock:
            self._last_seen = body
        self.write_response(self.handle_observation(request), cmd_dir)
        return True

    def start_file_loop(self, interval: Optional[float] = None) -> bool:
        """
        観測を見張る別スレッドを起こす。起こしたら True（動いていれば何もしない）。

        **Python 部のメインループとは別に回す。** メインループは撮影
        （perform_capture）で待つことがあり、そこに巻き込まれると指令が
        保持時間（200ms）を過ぎて走行体が止まる。
        """
        if self._loop_thread is not None and self._loop_thread.is_alive():
            return False
        self.clear_responses()
        self._stop.clear()
        wait = config.REMOTE_POLL_INTERVAL if interval is None else interval

        def _run():
            while not self._stop.is_set():
                try:
                    self.poll_once()
                except Exception as e:  # noqa: BLE001 — やりとりが落ちても Python 部は止めない
                    logger.error("リモートのやりとりに失敗しました: %s", e)
                self._stop.wait(wait)

        self._loop_thread = threading.Thread(target=_run, name="remote-control", daemon=True)
        self._loop_thread.start()
        logger.info("リモートコントロールの観測を見張ります（%s）", config.COMM_REMOTE_OBS_FILE)
        return True

    def stop_file_loop(self) -> None:
        """見張るスレッドを止める（走行の終わりとテスト用）"""
        self._stop.set()
        thread = self._loop_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)
        self._loop_thread = None


# --------------------------------------------------------------------------
# リモート側の制御を書く人への差し込み口
#
# 走行体で使う RemoteControl は1つなので、よく使うものだけ関数にしてある
# （Docs/remote_control.md がこの名前で説明している）。
# --------------------------------------------------------------------------


def set_controller(fn: Optional[Callable[[Observation], Any]]) -> None:
    """制御関数を差す。None で外す"""
    RemoteControl.get_instance().set_controller(fn)


def controller(fn: Callable[[Observation], Any]):
    """デコレータ版。`@controller` を付けるだけで差さる"""
    set_controller(fn)
    return fn


def has_controller() -> bool:
    return RemoteControl.get_instance().has_controller()


def insert_scenario(commands) -> None:
    """走行中のシナリオへコマンドを足す（`RemoteControl.insert_scenario`）"""
    RemoteControl.get_instance().insert_scenario(commands)


def set_motor_pwm(left: float, right: float) -> None:
    """モータの PWM を直接書く（`RemoteControl.set_motor_pwm`）"""
    RemoteControl.get_instance().set_motor_pwm(left, right)


def release_motor() -> None:
    """直接操作をやめ、走行方法にモータを返す"""
    RemoteControl.get_instance().release_motor()


def set_arm_pwm(pwm: float) -> None:
    """アームの PWM を直接書く（`RemoteControl.set_arm_pwm`）"""
    RemoteControl.get_instance().set_arm_pwm(pwm)


def release_arm() -> None:
    """アームの直接操作をやめ、シナリオの ARMANG にアームを返す"""
    RemoteControl.get_instance().release_arm()


def start_file_loop(interval: Optional[float] = None) -> bool:
    """観測を見張る別スレッドを起こす（`ev3_python` の起動時に呼ぶ）"""
    return RemoteControl.get_instance().start_file_loop(interval)
