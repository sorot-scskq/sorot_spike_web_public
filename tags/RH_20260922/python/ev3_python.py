import json
import state
import logging
from config import (
    load_season, setup_logging, CXX_HOST, CXX_PORT, PC_HOST, PC_PORT,
    JUDG_MINIFIGURE, JUDG_PLARAIL, JUDG_BOTTLE, JUDG_GATE,
    JUDG_QR1, JUDG_QR2, JUDG_ET_RALLY, CMD_GET_RESULT
)
from ev3_server import EV3Server, free_port
from pc_client import PCClient
from handler import perform_capture

# ログ初期設定の実行
setup_logging()
# その年の設定（使う識別値と待ち時間）を重ねる
load_season()
logger = logging.getLogger(__name__)

def main():
    """
    EV3(SPIKE Prime)と外部PCとの連携を制御するメインオーケストレータ。
    1. C++制御部からのコマンド待ち受信用サーバー(EV3Server)を起動。
    2. 外部PCのAI推論サーバへ接続するクライアント(PCClient)を起動。
    3. C++側からの要求を受信し、識別値に応じて処理を分岐・実行する。
    """
    logger.info("Python起動")

    try:
        # CXXポートがすでに使用されている場合は強制解放して、ソケットサーバーを起動
        free_port(CXX_PORT)
        server = EV3Server(CXX_HOST, CXX_PORT)
    except Exception as e:
        logger.error("EV3Serverの初期化に失敗: %s", e)
        return

    # C++制御部（クライアント）からの接続を確立するまで待機（ブロッキング）
    cxx_conn = server.accept_connection()

    # 外部PC側のAI推論サーバに接続を確立するまで再試行（ブロッキング）
    pc_client = PCClient(PC_HOST, PC_PORT)
    pc_client.connect()

    # C++からのリクエスト処理用の無限ループ
    from ev3_server import recv_packet, send_packet
    while True:
        try:
            # C++側からのデータを受信（4バイトヘッダー付きJSON）
            raw_payload = recv_packet(cxx_conn)
            if not raw_payload:
                logger.info("C++部との接続が切断されました")
                break

            try:
                request_json = json.loads(raw_payload)
            except Exception as e:
                logger.error(f"JSONパースエラー: {e} (生データ: {raw_payload})")
                continue

            identifier = request_json.get("identifier")
            request_id = request_json.get("request_id", 0)

            logger.info("C++部から受信: 識別値=%s, request_id=%s", identifier, request_id)

            # C++部からの推論結果取得（99）の問い合わせ
            if identifier == CMD_GET_RESULT:
                # スレッドセーフに最新の判定結果をキャッシュから取り出す
                with state.result_lock:
                    result_dict = state.latest_result or state.DEFAULT_RESULT
                
                # 応答JSONの生成
                response_dict = {
                    "completed_req_id": result_dict.get("completed_req_id", 0),
                    "minifigure": result_dict.get("minifigure", 100),
                    "plarail": result_dict.get("plarail", 100),
                    "bottle": result_dict.get("bottle", 100),
                    "gate": result_dict.get("gate", 100),
                    "qr1": result_dict.get("qr1", ""),
                    "qr2": result_dict.get("qr2", "")
                }
                response = json.dumps(response_dict)
                send_packet(cxx_conn, response)
                logger.info(f"C++部からの判定結果要求に応答: {response}")
                continue

            # 各識別値ごとの処理の分岐 (Switch-Like Branching)
            if identifier == JUDG_MINIFIGURE:
                # ミニフィグ判定（撮影回数：1回）
                perform_capture(identifier, pc_client, cxx_conn, request_id=request_id, repeat=1)
            elif identifier == JUDG_PLARAIL:
                # プラレール判定（撮影回数：5回連続）
                perform_capture(identifier, pc_client, cxx_conn, request_id=request_id, repeat=5)
            elif identifier == JUDG_BOTTLE:
                # ボトル判定（撮影回数：1回）
                perform_capture(identifier, pc_client, cxx_conn, request_id=request_id, repeat=1)
            elif identifier == JUDG_GATE:
                # キャリーゲート判定（撮影回数：1回）
                perform_capture(identifier, pc_client, cxx_conn, request_id=request_id, repeat=1)
            elif identifier == JUDG_QR1:
                # QRコード判定（撮影回数：1回）
                perform_capture(identifier, pc_client, cxx_conn, request_id=request_id, repeat=1)
            elif identifier == JUDG_QR2:
                # QRコード判定（撮影回数：1回）
                perform_capture(identifier, pc_client, cxx_conn, request_id=request_id, repeat=1)
            elif identifier == JUDG_ET_RALLY:
                # ET ラリー（撮影回数：1回）。PC 側が経路を計算して返す
                perform_capture(identifier, pc_client, cxx_conn, request_id=request_id, repeat=1)
            else:
                logger.warning(f"未知の識別値を受信しました: {identifier}")

        except Exception as e:
            logger.error("通信エラー: %s", e)

    # 終了時のソケットクローズ処理
    cxx_conn.close()
    pc_client.close()
    server.close()


if __name__ == "__main__":
    main()
