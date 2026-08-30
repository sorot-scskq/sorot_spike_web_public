import threading
import time
import logging
import state
from config import (
    IMAGE_PATH, CAMERA_COMMAND,
    JUDG_MINIFIGURE, JUDG_PLARAIL, JUDG_BOTTLE, JUDG_GATE,
    JUDG_QR1, JUDG_QR2, JUDG_ET_RALLY, PC_TIMEOUT, TIMEOUT_CONFIG, DECODE_ERROR_VAL
)
from camera import execute_camera_command, encode_image_to_base64, decode_qr_code

logger = logging.getLogger(__name__)

def safe_int(val, default_val=100):
    if val is None:
        return default_val
    try:
        return int(val)
    except (ValueError, TypeError):
        return default_val

def handle_command(identifier, pc_client, cxx_conn, request_id, use_mock=False):
    """
    バックグラウンドスレッドで実行される通信処理。
    判定要求の時は画像をBase64形式にエンコード、QRコード要求の時はQRコードをデコードしたテキストを、
    それぞれ外部PCのAI推論サーバへ送信し、判定結果を受信して
    グローバル変数 `state.latest_result` にスレッドセーフに格納する。
    """
    logger.info("[%s] 処理開始 (模擬画像モード=%s, request_id=%s)", identifier, use_mock, request_id)
    
    # 画像ファイルパスの決定（模擬画像モード時はテスト用画像を使用する）
    if use_mock:
        mock_map = {
            JUDG_MINIFIGURE: "./pc_com_test/image/minifigure.jpg",
            JUDG_PLARAIL:    "./pc_com_test/image/minifigure.jpg",
            JUDG_BOTTLE:     "./pc_com_test/image/bottle.jpg",
            JUDG_GATE:       "./pc_com_test/image/minifigure.jpg",
            JUDG_QR1:        "./pc_com_test/image/qr1.png",
            JUDG_QR2:        "./pc_com_test/image/qr2.png",
            JUDG_ET_RALLY:   "./pc_com_test/image/minifigure.jpg",
        }
        image_path = mock_map.get(identifier, IMAGE_PATH)
    else:
        image_path = IMAGE_PATH

    # 識別子に応じてPCへ送るペイロードを構築
    if identifier in (JUDG_QR1, JUDG_QR2):
        # 走行体側でQRコードのデコードを試行
        decoded_text = decode_qr_code(image_path)
        if not decoded_text:
            logger.warning("[%s] QRコードのデコードに失敗しました。PC送信を行わずにエラーをキャッシュします。", identifier)
            # デコード失敗時はPCに送信せず、即座に走行体側のステートにエラー応答を設定して終了
            with state.result_lock:
                key = "qr1" if identifier == JUDG_QR1 else "qr2"
                if not isinstance(state.latest_result, dict):
                    state.latest_result = dict(state.DEFAULT_RESULT)
                state.latest_result[key] = DECODE_ERROR_VAL
                state.latest_result["completed_req_id"] = request_id
            return
        
        # デコード成功時はテキストデータをPCへ送信
        payload = {
            "prefix": identifier,
            "request_id": request_id,
            "qr_data": decoded_text,
        }
    elif identifier in (JUDG_MINIFIGURE, JUDG_PLARAIL, JUDG_BOTTLE, JUDG_GATE, JUDG_ET_RALLY):
        encoded_image = encode_image_to_base64(image_path)
        payload = {
            "prefix": identifier,
            "request_id": request_id,
            "image": encoded_image,
        }
    else:
        logger.warning("[%s] 未サポートの識別子のため、ペイロードの送信をスキップします", identifier)
        return
    try:
        # 識別子に応じた個別タイムアウト時間を取得し、なければデフォルト値を使用
        timeout = TIMEOUT_CONFIG.get(identifier, PC_TIMEOUT)
        
        # PCにデータを送信し、解析結果を待つ（ここで通信待ちブロックが発生する。タイムアウト付き）
        result_json = pc_client.send_and_receive(payload, timeout=timeout)
        logger.info(f"[{identifier}] PC側から判定結果受信: {result_json}")
        
        # 受信結果から辞書を生成（JSONでの送信になるためカンマのエスケープ処理等は不要です）
        latest_dict = {
            "completed_req_id": request_id,
            "minifigure": safe_int(result_json.get("minifigure")),
            "plarail": safe_int(result_json.get("plarail")),
            "bottle": safe_int(result_json.get("bottle")),
            "gate": safe_int(result_json.get("gate")),
            "qr1": result_json.get("qr1", ""),
            "qr2": result_json.get("qr2", "")
        }

        # 排他制御を行い最新結果をキャッシュ（C++部からの get_result 要求に対する応答用）
        with state.result_lock:
            state.latest_result = latest_dict

        return
    except Exception as e:
        logger.error("[%s] 通信エラー: %s", identifier, e)
        # エラー発生時は、現在のキャッシュ結果から該当する識別値のみをエラーに書き換える
        with state.result_lock:
            if not isinstance(state.latest_result, dict):
                state.latest_result = dict(state.DEFAULT_RESULT)

            # completed_req_idを更新してC++側へ完了(エラー)を通知する
            state.latest_result["completed_req_id"] = request_id

            # 識別子（要求区分）に応じたキーをエラー値に書き換え
            key_map = {
                JUDG_MINIFIGURE: "minifigure",
                JUDG_PLARAIL: "plarail",
                JUDG_BOTTLE: "bottle",
                JUDG_GATE: "gate",
                JUDG_QR1: "qr1",
                JUDG_QR2: "qr2"
            }
            if identifier in key_map:
                key = key_map[identifier]
                if identifier in (JUDG_MINIFIGURE, JUDG_PLARAIL, JUDG_BOTTLE, JUDG_GATE):
                    state.latest_result[key] = -1
                else:
                    state.latest_result[key] = "ERROR"

            logger.info(f"通信エラーによりキャッシュ結果を部分更新しました: {state.latest_result}")


def perform_capture(identifier, pc_client, cxx_conn, request_id, repeat=1):
    """
    カメラ撮影を制御する関数。
    指定された回数(repeat)だけ撮影処理を行い、成功時に `handle_command` をスレッド起動する。
    ローカル環境などの理由でカメラコマンド実行に失敗した場合は、模擬画像を用いたデバッグモードに切り替えて継続する。
    """
    for i in range(repeat):
        logger.info(f"撮影処理 {i + 1}/{repeat} 開始")
        # カメラコマンドを実行して撮影をおこなう
        success = execute_camera_command(CAMERA_COMMAND)
        
        if success:
            logger.info("撮影情報をPCへ送信")
            use_mock = False
        else:
            logger.warning("カメラ撮影コマンドの実行に失敗したため、ローカル用の模擬画像を使用してPCと通信します。")
            use_mock = True

        # 通信処理がメインループをブロックしないよう、別スレッドを生成してバックグラウンド実行
        threading.Thread(
            target=handle_command,
            args=(identifier, pc_client, cxx_conn, request_id, use_mock),
            daemon=True,
        ).start()

        # 連続撮影に備えて1秒間スリープ
        time.sleep(1)
