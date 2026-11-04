"""
PC（無線通信デバイス）とのやりとりの手順。

撮る → 送る → 判定結果を覚える、までをここにまとめてある。送り先は
`pc_client.PCClient`（WebSocket）で、覚えるのは `state.ResultStore`。

    C++ の要求 ──▶ ev3_python ──▶ PcExchange.capture_and_send()
                                      撮る（camera）
                                      → 送る（PCClient）
                                      → 判定結果を覚える（ResultStore）

**走行経路（シナリオ）はここでは扱わない。** 受け取ったシナリオの登録は受信側に
まとめてある（`Common/route_register.py`）。ここでするのは「経路を頼んだ」と
伝えることだけ。
"""
import threading
import time
import logging
import config
import route_register
from state import ResultStore
from config import (
    IMAGE_PATH,
    JUDG_MINIFIGURE, JUDG_PLARAIL, JUDG_BOTTLE, JUDG_GATE,
    JUDG_QR1, JUDG_QR2, JUDG_ET_RALLY, PC_TIMEOUT, TIMEOUT_CONFIG, DECODE_ERROR_VAL,
    SENARIO_KBN_ET_RALLY
)
from camera import RobotCamera

logger = logging.getLogger(__name__)


class PcExchange:
    """
    PC とのやりとり。

    **状態を持たない。** 生成せずにクラスのまま呼ぶ（`PcExchange.capture_and_send(...)`）。
    覚えておくものは判定結果（`ResultStore`）と、経路を頼んだかどうか
    （`RouteRegister`）がそれぞれ持つ。
    """

    #: PC の応答で判定結果を辞書ごと差し替えるとき、空で来たぶんだけ前の値を引き継ぐ項目。
    #:
    #: ヒントカードは2枚そろってから経路を頼む（ReqToPython）。PC の応答が片方しか
    #: 持っていないと、もう片方が空になって「そろっていない」に戻る
    KEEP_ON_REPLACE = ("qr1", "qr2")

    #: カメラが使えないときに送る模擬画像（母艦での試験用）
    MOCK_IMAGES = {
        JUDG_MINIFIGURE: "./pc_com_test/image/minifigure.jpg",
        JUDG_PLARAIL:    "./pc_com_test/image/minifigure.jpg",
        JUDG_BOTTLE:     "./pc_com_test/image/bottle.jpg",
        JUDG_GATE:       "./pc_com_test/image/minifigure.jpg",
        JUDG_QR1:        "./pc_com_test/image/qr1.png",
        JUDG_QR2:        "./pc_com_test/image/qr2.png",
        JUDG_ET_RALLY:   "./pc_com_test/image/minifigure.jpg",
    }

    #: 画像を送る要求（二次元コードは走行体で読んでから文字列で送る）
    IMAGE_JUDGES = (JUDG_MINIFIGURE, JUDG_PLARAIL, JUDG_BOTTLE, JUDG_GATE, JUDG_ET_RALLY)

    #: 失敗したときに数で返すもの（文字列で返すものは "ERROR"）
    NUMBER_JUDGES = (JUDG_MINIFIGURE, JUDG_PLARAIL, JUDG_BOTTLE, JUDG_GATE)

    #: 識別値 → 判定結果のキー
    RESULT_KEYS = {
        JUDG_MINIFIGURE: "minifigure",
        JUDG_PLARAIL: "plarail",
        JUDG_BOTTLE: "bottle",
        JUDG_GATE: "gate",
        JUDG_QR1: "qr1",
        JUDG_QR2: "qr2",
    }

    # --- 撮る -----------------------------------------------------------------

    @classmethod
    def capture_and_send(cls, identifier, pc_client, request_id, repeat=1):
        """
        撮ってから送る。

        指定された回数(repeat)だけ撮影し、そのたびに送信を別スレッドで起こす。
        カメラが使えないときは模擬画像に切り替えて続ける（走行は止めない）。
        """
        for i in range(repeat):
            logger.info("撮影処理 %d/%d 開始", i + 1, repeat)
            # OpenCV で1枚撮る。色の読み取りと同じカメラを借りる
            success = RobotCamera.capture_still(IMAGE_PATH)

            if success:
                logger.info("撮影情報をPCへ送信")
                use_mock = False
            else:
                logger.warning("カメラ撮影コマンドの実行に失敗したため、"
                               "ローカル用の模擬画像を使用してPCと通信します。")
                use_mock = True

            # 通信がメインループをブロックしないよう、別スレッドで送る
            threading.Thread(
                target=cls.handle_command,
                args=(identifier, pc_client, request_id, use_mock),
                daemon=True,
            ).start()

            # 連続撮影に備えて1秒待つ。
            #
            # **最後の1枚のあとには待たない。** ここはメインループ（ev3_python）の
            # 中なので、待っているあいだ他の要求を捌けない
            if i + 1 < repeat:
                time.sleep(1)

    # --- 送る -----------------------------------------------------------------

    @classmethod
    def handle_command(cls, identifier, pc_client, request_id,
                       use_mock=False, image_path=None):
        """
        撮った画像から要求を組んで送る（別スレッドで呼ばれる）。

        画像判定はBase64にして送り、二次元コードは走行体で読んでから文字列で送る。
        """
        logger.info("[%s] 処理開始 (模擬画像モード=%s, request_id=%s)",
                    identifier, use_mock, request_id)

        # 画像ファイルパスの決定（模擬画像モード時はテスト用画像を使用する）
        if use_mock:
            image_path = cls.MOCK_IMAGES.get(identifier, IMAGE_PATH)
        elif image_path is None:
            image_path = IMAGE_PATH
        # image_path が渡されていれば、呼び出し側が撮った画像を使う
        # （色の読み取りが借りているカメラの1枚など）

        if identifier in (JUDG_QR1, JUDG_QR2):
            # 走行体側で二次元コードのデコードを試行
            decoded_text = RobotCamera.decode_qr(image_path)
            if not decoded_text:
                logger.warning("[%s] QRコードのデコードに失敗しました。"
                               "PC送信を行わずにエラーを残します。", identifier)
                cls.record_decode_error(identifier, request_id)
                return
            cls.send_decoded_qr(identifier, decoded_text, pc_client, request_id)
            return

        if identifier not in cls.IMAGE_JUDGES:
            logger.warning("[%s] 未サポートの識別子のため、ペイロードの送信をスキップします",
                           identifier)
            return

        payload = {
            # 振り分けに使う識別値。**PC は identifier しか読まない**
            # （PC-System_2026 の pc_system.run_main_loop）。prefix は以前の名前で、
            # 古い PC と繋いだときのために残してある
            "prefix": identifier,
            "identifier": identifier,
            "request_id": request_id,
            "image": RobotCamera.encode_base64(image_path),
        }
        cls.send(identifier, payload, pc_client, request_id)

    @classmethod
    def send_decoded_qr(cls, identifier, decoded_text, pc_client, request_id):
        """
        読めたヒントカードの文字列を、自分で覚えてから PC へ送る。

        **読めたあとの道をここ1本にまとめてある。** 撮った1枚を読む
        （`handle_command`）のと、映像から粘って読む
        （`2026/hint_card_capture.py`）の、どちらからも同じここを通す。

        読めた文字列は自分で覚えておく。C++ 側は2枚そろってから経路を頼む
        （`ReqToPython`）。PC の応答を待ってから覚えると、そのぶん経路の要求が
        遅れるうえ、PC が qr1/qr2 を返さない実装だと永久にそろわない。
        読めたのは走行体なので、走行体が覚えていればよい。
        """
        key = cls.qr_key(identifier, decoded_text)
        ResultStore.get_instance().set(key, decoded_text)

        # **送る識別値は中身で決める。** 撮った順（identifier）のまま送ると、
        # 入れ替わったときに PC が平文を復号しようとして失敗する（qr_key を参照）
        send_identifier = JUDG_QR1 if key == "qr1" else JUDG_QR2
        payload = {
            "prefix": send_identifier,
            "identifier": send_identifier,
            "request_id": request_id,
            "qr_data": decoded_text,
            # PC が入れ違いを直せるよう、走行体の見立ても添えて送る
            "qr_key": key,
        }
        cls.send(send_identifier, payload, pc_client, request_id)

    @classmethod
    def send(cls, identifier, payload, pc_client, request_id):
        """
        PC へ送り、応答を判定結果へ取り込む。

        **走行経路（シナリオ）はここでは触らない。** 受け取ったシナリオの登録は
        受信側で一手に引き受ける（`Common/route_register.py`）。要求の応答でも、
        頼んでいない便でも同じ口を通るので、置き場へ置く処理が2か所に散らない。
        """
        if identifier in (JUDG_QR2, JUDG_ET_RALLY):
            # この走行で経路を頼んだ、と受信側へ伝える。置くのは受信側。
            #
            # **ヒントカード2でも構える。** PC は QR2 を復号した直後に経路を
            # 計算し終えている（PC-System_2026 の TaskCoordinator.execute_task）。
            # あとは応答に載せてもらうだけなので、構えておけばその場で登録できる。
            # 構えていないと、届いても handle_message が捨てる。
            #
            # **前の走行の経路が混ざる心配はない。** PC は QR1 を受けた時点で
            # 覚えている経路を捨てるので、QR2 の応答に載るのはこの走行のぶんだけ。
            route_register.RouteRegister.get_instance().expect(SENARIO_KBN_ET_RALLY)

        if pc_client is None:
            logger.warning("[%s] PC へ繋ぐ口がありません。送信を省きます", identifier)
            ResultStore.get_instance().complete(request_id)
            return

        try:
            # 識別子に応じた個別タイムアウト時間を取得し、なければデフォルト値を使用
            timeout = TIMEOUT_CONFIG.get(identifier, PC_TIMEOUT)

            # PCへ送り、応答を待つ（ここで通信待ちが発生する。タイムアウト付き）
            result_json = pc_client.send_and_receive(payload, timeout=timeout)
            logger.info("[%s] PC側から判定結果を受け取りました", identifier)

            latest = {
                "completed_req_id": request_id,
                "minifigure": cls._safe_int(result_json.get("minifigure")),
                "plarail": cls._safe_int(result_json.get("plarail")),
                "bottle": cls._safe_int(result_json.get("bottle")),
                "gate": cls._safe_int(result_json.get("gate")),
                "qr1": result_json.get("qr1", ""),
                "qr2": result_json.get("qr2", ""),
            }

            # 最新結果を差し替える（C++部が読む判定結果ファイルの元）
            ResultStore.get_instance().replace(latest, keep=cls.KEEP_ON_REPLACE)
        except Exception as e:  # noqa: BLE001 — 送れなくても走行は続ける
            logger.error("[%s] 通信エラー: %s", identifier, e)
            cls._record_error(identifier, request_id)

    # --- 覚える ---------------------------------------------------------------

    @classmethod
    def record_decode_error(cls, identifier, request_id):
        """
        二次元コードを読めなかったことを判定結果へ残す（PC へは送らない）。

        C++ 側はこの `DECODE_ERROR` を見て「読めなかった」と分かる（`ResultRead`）。
        空のままにすると「まだ結果が無い」と区別できず、経路の要求が永久に待つ。
        """
        store = ResultStore.get_instance()
        store.set(cls.qr_key(identifier, None), DECODE_ERROR_VAL)
        store.complete(request_id)

    @classmethod
    def _record_error(cls, identifier, request_id):
        """通信に失敗したことを判定結果へ残す。該当する識別値だけを書き換える"""
        store = ResultStore.get_instance()

        # completed_req_id を更新して C++ 側へ完了(エラー)を通知する。
        # 知らせないと、C++ は結果を待ち続ける
        store.complete(request_id)

        key = cls.RESULT_KEYS.get(identifier)
        if key is not None:
            store.set(key, -1 if identifier in cls.NUMBER_JUDGES else "ERROR")

        logger.info("通信エラーにより判定結果を部分更新しました: %s", store.snapshot())

    # --- どちらのヒントカードか -----------------------------------------------

    @classmethod
    def qr_key(cls, identifier, decoded_text):
        """
        読んだ二次元コードを、どちらのヒントカードとして覚えるか。

        **カードには番号が書かれていない。** シナリオは hint_card1 → hint_card2 の順に
        読むが、それは段取りでしかなく、読み損ねや読み直しがあると入れ替わる。
        ゲート配置が入れ替わると通過順（赤→青→黄）がそのまま崩れるので、中身で見分ける。

        見分け方は年ごとに違う（今年のカードの書式）ので、年の設定モジュールの
        classify_qr() に聞く。無ければ、これまでどおり読んだ順（識別値）で決める。

        :param identifier: JUDG_QR1 / JUDG_QR2
        :param decoded_text: 読めた文字列。読めなかったときは None
        :returns: "qr1" または "qr2"
        """
        default_key = "qr1" if identifier == JUDG_QR1 else "qr2"
        if not decoded_text:
            return default_key
        season = config.load_season()
        classify = getattr(season, "classify_qr", None)
        if not callable(classify):
            return default_key
        try:
            key = classify(decoded_text)
        except Exception as e:  # noqa: BLE001 — 見分けに失敗しても走行は続ける
            logger.warning("ヒントカードの見分けに失敗しました: %s", e)
            return default_key
        if key not in ("qr1", "qr2"):
            return default_key
        if key != default_key:
            logger.info("読んだ内容から %s と判断しました（読んだ順では %s）", key, default_key)
        return key

    # --- 部品 -----------------------------------------------------------------

    @staticmethod
    def _safe_int(val, default_val=100):
        """数にできなければ番兵（100 = まだ結果が無い）にする"""
        if val is None:
            return default_val
        try:
            return int(val)
        except (ValueError, TypeError):
            return default_val
