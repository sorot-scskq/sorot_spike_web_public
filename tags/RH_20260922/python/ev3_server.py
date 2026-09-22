"""
C++ 制御部からの接続を受ける TCP サーバ（実機）。

【遅延 import にしている理由】
このファイルは Common/ に置いてあり、シミュレータ側（PyScript / Pyodide）から
同じフォルダの別モジュールを読むときに一緒に読み込まれうる。ブラウザには
socket も subprocess も無いので、先頭で import すると読み込んだ時点で落ちる。
実機でしか通らない関数の中で import する。
"""

import traceback
import time
import logging

try:
    from communicator import recv_packet as _recv_packet, send_packet as _send_packet
except ImportError:  # パッケージとして読み込まれた場合
    from .communicator import recv_packet as _recv_packet, send_packet as _send_packet

logger = logging.getLogger(__name__)

class EV3Server:
    """
    EV3(SPIKE Prime)のC++プログラムからの接続を受け付けるTCPソケットサーバー。
    C++部からの要求を受信するための通信口を提供する。
    """
    def __init__(self, host, port):
        import socket

        logger.info("Pythonサーバ起動")

        self.host = host
        self.port = port
        # IPv4/TCPソケットを作成
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        # TIME_WAIT状態のポートを再利用できるように設定（再起動時の「Address already in use」エラー防止）
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        # ポートが他のプロセスによって既に使用されているか事前チェック (Linux環境専用)
        if self.check_port_in_use():
            logger.error(f"ポート {self.port} は使用中です。サーバー起動を中止します。")
            raise SystemExit(1)

        try:
            # IPアドレスとポートをソケットに紐付けし、接続要求の待機を開始する
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(1)  # 同時接続可能数は1つに制限
        except Exception as e:
            logger.error("サーバー起動失敗: %s", e)
            logger.error(traceback.format_exc())
            raise SystemExit(1)

        logger.info("サーバー起動成功")

    def check_port_in_use(self):
        """
        対象のポートが使用中かどうかを調べる関数。
        ※ Linuxの lsof コマンドを利用するため、Windows環境では常にFalse相当の挙動となります。
        """
        import subprocess

        result = subprocess.run(
            f"lsof -i :{self.port}",
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        return bool(result.stdout)

    def accept_connection(self):
        """
        C++部（クライアント）からのソケット接続を待ち受け、確立する関数。
        接続が確立するまでブロッキング（待機）し、接続成功後に通信用ソケット(conn)を返す。
        """
        logger.info("C++部との接続を待機中...")

        while True:
            try:
                conn, addr = self.server_socket.accept()
                logger.info("C++部と接続しました: %s", addr)
                return conn
            except Exception as e:
                logger.error("C++部との接続待機中にエラー: %s", e)
                logger.error(traceback.format_exc())
                time.sleep(1)

    def close(self):
        """
        サーバーのソケットを閉じるクリーンアップ処理。
        """
        try:
            self.server_socket.close()
            logger.info("サーバーソケットを閉じました")
        except Exception as e:
            logger.error("サーバーソケット閉鎖中にエラー: %s", e)
            logger.error(traceback.format_exc())


def free_port(port):
    """
    指定されたポート番号を使用している古いプロセスを強制終了するユーティリティ関数。
    ※ Linuxの lsof および kill コマンドを利用するため、Windows環境ではエラー例外としてキャッチされます。
    """
    try:
        import subprocess

        # ポートを占有しているプロセスの一覧を取得
        result = subprocess.run(
            f"lsof -i :{port}",
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        lines = result.stdout.decode().splitlines()
        for line in lines[1:]:  # 最初の行（ヘッダー）をスキップ
            parts = line.split()
            pid = parts[1]      # PID（プロセスID）の取得
            # プロセスを強制終了
            subprocess.run(f"kill -9 {pid}", shell=True)
            print(f"ポート {port} を使用していたプロセス {pid} を強制終了しました")
    except Exception as e:
        print("ポート解放中にエラー:", e)


def recv_packet(conn):
    """
    4バイトのデータ長ヘッダ（ビッグエンディアン）を読み、その長さぶんの
    JSON を受け取る。形式は C++ 側（PythonCommServer.cpp）と同じ。

    実体は Common/communicator.py にある。ここと pc_client.py で別々に
    持つと、片方だけ直したときに気づけないため。
    """
    return _recv_packet(conn)


def send_packet(conn, payload_str):
    """JSON 文字列に 4バイトの長ヘッダを付けて送る"""
    _send_packet(conn, payload_str)
