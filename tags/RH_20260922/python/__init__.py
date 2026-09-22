"""
走行体共通 Python モジュール。

実機（Raspberry Pi 上の Python）とシミュレータ（PyScript）で同じものを使う。
どちらでも動くよう、socket / subprocess / OpenCV は使う関数の中で import する
（ブラウザにはこれらが無く、モジュールの先頭で import すると読み込んだ時点で
落ちるため）。

    認識  camera / qr_decoder / bottle_color / color_converter / bottle_color_monitor
    通信  communicator（TCP=実機、WebSocket=シミュレータ、Callback=PyScript・テスト）
          wireless_device（PC へ 1回だけ要求して 1回だけ受け取る土台）
    設定  config / state

年ごとの処理（今年の識別値・難所ごとの判定）は年のフォルダにある。
共通側から年のフォルダを参照しないこと。参照は「共通 ← 年」の一方通行にする
（唯一の例外が config.SEASON_CONFIG_MODULE で、つなぐのはその 1行だけ）。

実機だけで動くもの（ev3_python / ev3_server / pc_client / handler）は、
ここからは公開していない。ブラウザで import しても意味が無いので、
実機側から直接 import すること。
"""

from .bottle_color import BottleColor
from .bottle_color_monitor import BottleColorClassifier, BottleColorMonitor
from .camera import RobotCamera, encode_image_to_base64, execute_camera_command
from .color_converter import ColorConverter
from .communicator import (
    BaseCommunicator,
    CallbackCommunicator,
    CallbackSender,
    MessageSender,
    TcpCommunicator,
    WebSocketCommunicator,
    WebSocketSender,
    recv_packet,
    send_packet,
)
from .qr_decoder import QrCodeDecoder
from .wireless_device import (
    STATUS_OK,
    RequestOnceReceiver,
    as_int,
    get_field,
    get_identifier,
    is_status_ok,
)

__all__ = [
    "STATUS_OK",
    "BaseCommunicator",
    "BottleColor",
    "BottleColorClassifier",
    "BottleColorMonitor",
    "CallbackCommunicator",
    "CallbackSender",
    "ColorConverter",
    "MessageSender",
    "QrCodeDecoder",
    "RequestOnceReceiver",
    "RobotCamera",
    "TcpCommunicator",
    "WebSocketCommunicator",
    "WebSocketSender",
    "as_int",
    "encode_image_to_base64",
    "execute_camera_command",
    "get_field",
    "get_identifier",
    "is_status_ok",
    "recv_packet",
    "send_packet",
]
