"""
走行体共通 Python モジュール。

socket / subprocess / OpenCV は使うところで import する。走行体に OpenCV が
無い環境でモジュール先頭の import が失敗すると、ev3_python 全体が起動できず、
C++側が「接続失敗、再試行します」を出し続ける事象が実機で起きたため
（読み込みは Common/opencv.py の OpenCv にまとめてある）。

    認識  camera / qr_decoder / bottle_color / color_converter / bottle_color_monitor
    通信  pc_client（PCとWebSocket）/ communicator（枠付けと差し替え口）
    走行  remote_control（毎周期の指令をここで決める）
    設定  config / state（判定結果は ResultStore が持つ）

年ごとの処理（今年の識別値・難所ごとの判定）は年のフォルダにある。
共通側から年のフォルダを参照しないこと。参照は「共通 ← 年」の一方通行にする
（唯一の例外が config.SEASON_CONFIG_MODULE で、つなぐのはその 1行だけ）。

親（ev3_python）とその周り（pc_client / handler / route_register）は、ここからは
公開していない。スクリプトとして起動するものなので、直接 import すること。
"""

from .bottle_color import BottleColor
from .bottle_color_monitor import BottleColorClassifier, BottleColorMonitor
from .camera import RobotCamera
from .color_converter import ColorConverter
from .communicator import (
    BaseCommunicator,
    CallbackCommunicator,
    TcpCommunicator,
    WebSocketCommunicator,
    PacketCodec,
)
from .opencv import OpenCv
from .qr_decoder import QrCodeDecoder
from .remote_control import Command, Observation, RemoteControl, Status, controller, set_controller
from .scenario_update import ScenarioMailbox
from .state import ResultStore

__all__ = [
    "BaseCommunicator",
    "BottleColor",
    "BottleColorClassifier",
    "BottleColorMonitor",
    "CallbackCommunicator",
    "ColorConverter",
    "Command",
    "Observation",
    "OpenCv",
    "PacketCodec",
    "QrCodeDecoder",
    "RemoteControl",
    "ResultStore",
    "RobotCamera",
    "ScenarioMailbox",
    "Status",
    "TcpCommunicator",
    "WebSocketCommunicator",
    "controller",
    "set_controller",
]
