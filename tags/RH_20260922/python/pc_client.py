"""
PC 上の推論サーバへつなぐ TCP クライアント（実機）。

【遅延 import にしている理由】
このファイルは Common/ に置いてあり、シミュレータ側（PyScript / Pyodide）から
同じフォルダの別モジュールを読むときに一緒に読み込まれうる。ブラウザには
socket が無いので、先頭で import すると読み込んだ時点で落ちる。
実機でしか通らない関数の中で import する。
"""

import json
import time
import traceback
import logging
import threading

try:
    from communicator import recv_packet, send_packet
except ImportError:  # パッケージとして読み込まれた場合
    from .communicator import recv_packet, send_packet

logger = logging.getLogger(__name__)

class PCClient:
    """
    外部PC上のAI推論サーバーと通信を行うソケットクライアント。
    EV3(Raspberry Pi等)上で撮影した画像データを外部PCへ送信し、判定結果を受け取る。
    """
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.conn = None
        self.lock = threading.Lock()
        logger.info("PC側Pythonと接続中・・・")

    def connect(self):
        """
        外部PCのAI推論サーバーにソケット接続する関数。
        サーバー側がまだ起動していない可能性を考慮し、接続が確立するまで1秒おきにリトライを行う。
        """
        import socket

        while True:
            try:
                self.conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                logger.debug("connの型: %s", type(self.conn))
                self.conn.connect((self.host, self.port))
                logger.info("PC側Pythonと接続しました")
                break
            except Exception as e:
                logger.warning("PC側Pythonが未起動。再試行します... %s", e)
                time.sleep(1)

    def send_and_receive(self, payload, timeout=None):
        """
        引数 payload (dict形式の画像＋識別子) を JSON 形式にシリアライズして送信し、
        PC側からの推論結果 JSON データを受信して辞書オブジェクトとして返す関数。
        任意のタイムアウト秒を指定でき、超過した場合は TimeoutError を発生させる。
        """
        import socket  # socket.timeout を except で使うため、ここでも取る

        self.lock.acquire()
        try:
            if self.conn is None:
                logger.info("PCとの接続がありません。再接続します。")
                self.connect()

            logger.info(f"send_and_receive start (timeout={timeout})")
            old_timeout = self.conn.gettimeout()
            try:
                if timeout is not None:
                    self.conn.settimeout(timeout)

                # 送信データをJSONシリアライズしてデータ長付きで送信。
                # 4バイト長ヘッダの並びは Common/communicator.py の 1組に寄せてある
                logger.info("送信中")
                start_time = time.time()  # 送信開始時間を記録
                send_packet(self.conn, payload)
                logger.info("送信完了")

                raw = recv_packet(self.conn)
                if raw == "":
                    raise ConnectionError("PC接続切断")

                obj = json.loads(raw)
                elapsed_time = time.time() - start_time  # 経過時間を計算
                msg = f"JSON受信成功 (所要時間: {elapsed_time:.3f}秒)"
                logger.info(msg)
                print(msg)  # コンソールへも直接出力
                return obj

            except (socket.timeout, TimeoutError) as te:
                logger.error(f"通信タイムアウトが発生しました（制限時間: {timeout}秒）")
                self.close()  # タイムアウトしたため接続をクローズして次回再接続を促す
                raise TimeoutError(f"PCからの応答がタイムアウト時間（{timeout}秒）内に受信できませんでした。") from te
            except ConnectionResetError as cre:
                logger.error("接続がリセットされました")
                self.close()
                raise cre
            except Exception as e:
                logger.exception(f"予期しない通信エラー: {e}")
                self.close()
                raise e
            finally:
                # タイムアウト設定を元に戻す
                try:
                    if self.conn:
                        self.conn.settimeout(old_timeout)
                except Exception:
                    pass
        finally:
            self.lock.release()

    def close(self):
        """
        外部PC側とのソケット接続を閉じるクリーンアップ処理。
        """
        try:
            if self.conn:
                self.conn.close()
            logger.info("PC側との接続を閉じました")
        except Exception as e:
            logger.error("PC側との接続閉鎖中にエラー: %s", e)
            logger.error(traceback.format_exc())
        finally:
            self.conn = None
