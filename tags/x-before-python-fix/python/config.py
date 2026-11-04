"""
走行体 Python の共通設定（接続先・パス・共通の識別値・ログ）。

毎年変わらないものだけを置く。年ごとに変わるぶんは年のフォルダにある
（2025/config_2025.py、2026/config_2026.py）。

    共通（ここ）  接続先、撮影のパスとコマンド、識別値の採番、ログ設定
    年ごと        その年に使う識別値の顔ぶれと、要求区分ごとのタイムアウト

【識別値の番号について】
JUDG_* が正（0始まり）。C++ 側に PhotoObject という 1始まりの enum もあるが、
C++ の実装（PythonCommServer.cpp の sendJsonCommand と ResultRead.cpp の
getLatestRequestId(0..3)）はどちらも 0始まりを使っており、PhotoObject は
どこからも参照されていない。使うのはこちら。

なお、難所そのものの識別値（RoughSpot/Cpp/RoughSpotId.h）とは**別物**である。
同じ番号でも意味が違うので混ぜないこと。

【パスについて】
IMAGE_PATH / CAMERA_COMMAND は走行体（Raspberry Pi）で ./ から見たパス。
"""

import logging
import logging.handlers
import os
import sys
import time

# --------------------------------------------------------------------------
# 画像識別値（PC 側と同値にすること）
#
# 番号の割り当ては毎年同じ。年で変わるのは「どれを使うか」と「どれだけ待つか」
# なので、それは年のフォルダ（2025/config_2025.py、2026/config_2026.py）で決める。
# 使わない年があっても番号は空けておくこと。詰め直すと PC 側とずれる。
#
# ここは PC 側（PC-System_2026/config/image_config.py）と同じ値にすること。
# ずれると、走行体が頼んだのと違う判定が走る。
# --------------------------------------------------------------------------
JUDG_MINIFIGURE = 0  # ミニフィグ判定（2025 で使用）
JUDG_PLARAIL = 1  # プラレール判定（2025 で使用）
JUDG_BOTTLE = 2  # ボトル判定
JUDG_GATE = 3  # キャリーゲート判定
JUDG_QR1 = 4  # QRコード1判定（2026 で使用）
JUDG_QR2 = 5  # QRコード2判定（2026 で使用）
JUDG_ET_RALLY = 6  # ET ラリー（経路の計算・生成）
CMD_GET_RESULT = 99  # 推論結果取得要求（定期的な問い合わせ用）

# --------------------------------------------------------------------------
# 走行体の中だけで使う識別値（PC へは送らない）
#
# C++ から Python へ「いま○○せよ」と伝えるためのもので、PC の推論サーバとは
# 関係が無い。番号は上の JUDG_* と同じ空間なので、重ならないように採ること
# （PythonCommServer.cpp が latest_request_ids[identifier] を引くため 0〜9）。
#
# 受けるのは年ごとの handle_request()。共通側は素通しする。
# C++ 側は RunMethod/RunMethodFactory.cpp に同じ番号を書くこと。
# --------------------------------------------------------------------------

DECODE_ERROR_VAL = "DECODE_ERROR"  # QRデコード失敗時のエラー応答値

# --------------------------------------------------------------------------
# 接続先
# --------------------------------------------------------------------------
#: C++ 部との受け渡しに使うファイル。**ソケットは使わない。**
#:
#: 同じ機械の中の話なので、ストリームである必要が無い。運ぶのは
#:   C++ → Python  識別値ひとつ
#:   Python → C++  判定結果のスナップショット
#: の2つだけ。接続状態を持たないぶん、どちらが先に起動しても、途中で
#: 落ちても、生きている側は動き続ける。中身は JSON なので cat して読める。
#:
#: C++ 側は RoughSpot/Cpp/PythonCommServer.cpp に同じパスを書くこと。
#: 置き場は /dev/shm（tmpfs＝RAM上のファイルシステム）。**ディスクには書かない。**
#: 実体はメモリなので SD カードに触らず、読み書きでストレージを待つこともない。
#: それでいてファイルなので、一時ファイル＋rename の不可分性がそのまま使え、
#: 走行中でも cat で中身を見られる。再起動で消えるが、消えてよいもの。
COMM_DIR = "/dev/shm"

COMM_REQ_FILE = COMM_DIR + "/sorot_comm_req.json"   # C++ が書き、Python が読む
COMM_RES_FILE = COMM_DIR + "/sorot_comm_res.json"   # Python が書き、C++ が読む
COMM_RES_TMP = COMM_DIR + "/sorot_comm_res.tmp"

#: Python が書き、C++ が読んで走行中のシナリオへ上書きする。
#: 認識で分かったこと（ボトルの色など）から運び先を決め、その区間のコマンドを差し替える。
#: 中身は custom.json と同じ形のコマンドセットの配列
COMM_SCN_FILE = COMM_DIR + "/sorot_scenario_update.json"
COMM_SCN_TMP = COMM_DIR + "/sorot_scenario_update.tmp"

#: ETラリーの経路。Python が置き、C++ が1回だけ引き取る（引き取ると消える）
COMM_ROUTE_FILE = COMM_DIR + "/sorot_route.json"
COMM_ROUTE_TMP = COMM_DIR + "/sorot_route.tmp"

#: リモートコントロール走行。C++ が最新の観測を置き、Python が応答を1通ずつ置く
#: （sorot_remote_cmd.<通し番号10桁>.json。C++ が順に引き取って消す）
COMM_REMOTE_OBS_FILE = COMM_DIR + "/sorot_remote_obs.json"
COMM_REMOTE_CMD_PREFIX = "sorot_remote_cmd."

#: 観測が置き直されたかを見にいく間隔[s]。C++ は10msごとに置くので、それより短くする
REMOTE_POLL_INTERVAL = 0.005

#: 走行中の自己位置。C++ が最新だけを置き、Python が読む。
#:
#: **リモートの観測（COMM_REMOTE_OBS_FILE）とは別物。** あちらはリモート
#: コントロール走行（FUNCNO 18）の区間でしか置かれないが、こちらは走行中
#: いつでも置かれる。置くのは app.cpp の result_reader_task（100ms周期）。
#: C++ 側は RoughSpot/Cpp/PythonCommServer.cpp に同じパスを書くこと
COMM_POSE_FILE = COMM_DIR + "/sorot_pose.json"

#: 自己位置がこれより古ければ使わない[s]。C++ は100msごとに置き直すので、
#: 3周期ぶん待っても来なければ C++ 側が止まっているとみなす
POSE_STALE_SEC = 0.3

#: 要求ファイルを見にいく間隔[s]。撮影要求は数秒に1回なので細かくしなくてよい
COMM_POLL_INTERVAL = 0.05

#: 結果ファイルを書き直す間隔[s]。**新しさを決める値ではない。**
#:
#: 結果が変わったときは、この間隔を待たずに即座に置き直す（COMM_POLL_INTERVAL
#: ＝50ms 以内）。ここで決めているのは「中身が変わらないときに、生存を示すため
#: どれくらいの間隔で py_time を更新するか」だけ。
#:
#: 変わらないのに 10Hz で書き直しても、C++ に届く速さは変わらず書き込みが増える
#: だけなので、生存確認としてちょうどよい 1秒にしてある。
#: 届く速さを決めているのは C++ 側の読み周期（app.cpp の result_reader_task）。
COMM_RESULT_REFRESH = 1.0


# --------------------------------------------------------------------------
# 無線通信デバイス（PC）
# --------------------------------------------------------------------------

# PC_HOST = "133.17.164.44"
# PC_HOST = "192.168.43.43"
PC_HOST = "172.20.1.18"

#: PC が待ち受ける WebSocket のポート（PC-System_2026 の socket_config.WS_PORT）
PC_WS_PORT = 8765

#: 繋ぎ先。**ソケット（TCP 49661）から WebSocket に変えた。**
#: 運ぶものは同じで、4バイトの長さヘッダ＋JSON をバイナリフレームで送る
#: （pc_client.py。PC 側は COMM_MODE = 'WS_TCP'）
PC_WS_URL = "ws://%s:%d" % (PC_HOST, PC_WS_PORT)

#: 繋ぐのを諦めるまで[s]。**短くすること。** PC が居ないときに、走行体の中だけで
#: 完結する判定（ボトルの色）まで待たせないため
PC_WS_OPEN_TIMEOUT = 2.0

# --------------------------------------------------------------------------
# 難所の識別値（シナリオ区分）
# --------------------------------------------------------------------------
#: C++ 側 RoughSpot/Cpp/RoughSpotId.h と同値にすること。片方だけ変えると、
#: 経路の入り口（SW_ROUTE の SCV）と噛み合わなくなる。
#: **PC へ送るときの識別値（上の JUDG_*）とは別物。** 同じ番号でも意味が違う
SENARIO_KBN_ET_RALLY = 1
SENARIO_KBN_ET_SUMO = 2
SENARIO_KBN_GARAGE = 3

# 要求区分(prefix)ごとのタイムアウト設定（秒）。
# どの年でも使う判定ぶんだけを既定として持ち、その年ぶんは load_season() で重ねる
TIMEOUT_CONFIG = {
    JUDG_BOTTLE: 3.0,  # ボトル判定
    JUDG_GATE:   3.0,  # ゲート判定
}
PC_TIMEOUT = 5.0  # デフォルトのタイムアウト時間（秒）

# --------------------------------------------------------------------------
# その年の設定
# --------------------------------------------------------------------------

#: その年のフォルダ名。★毎年ここだけ変える
SEASON = "2026"

# その年の設定モジュール（config_<年>.py）が、任意で持てる差し込み口。
# 共通側から年のフォルダを名指しで import しないためのもので、
# ev3_python.py は getattr で探して、あれば呼ぶ。無ければ何もしない。
#
#   USED_JUDG                          その年に使う画像識別値
#   TIMEOUT_CONFIG                     要求区分ごとの待ち時間[s]
#   start_recognizers()                走り出しに起こす認識処理（カメラ等）
#   handle_request(identifier, req_id) 共通が知らない識別値を捌く。捌いたら True
#   fill_result(result)                CMD_GET_RESULT の応答へその年の項目を足す
#   classify_qr(decoded_text)          読んだ二次元コードがどちらのヒントカードか
#                                      （"qr1" / "qr2" / None）
#   on_route_registered(kbn, rows)     PC から届いた経路を置いたとき（難所の識別値と行）

#: その年の設定モジュール
SEASON_CONFIG_MODULE = "config_" + SEASON

#: load_season() が読んだ年の設定。season_hook() が使い回す
_SEASON = None
_SEASON_LOADED = False


#: 走行体 Python の根（RoughSpot/Python）。シナリオの SCRIPT はここからの相対パス
PYTHON_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def season_dir():
    """その年のフォルダの絶対パス（RoughSpot/Python/<年>）"""
    return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), SEASON)


def add_season_to_path():
    """
    その年のフォルダを import 対象に加える。

    走行体は Common/ev3_python.py をスクリプトとして起動するため、sys.path には
    Common/ しか入らない。年のフォルダは隣にあるので、ここで足さないと
    config_2026 も年ごとのモジュールも見つからない。
    """
    path = season_dir()
    if os.path.isdir(path) and path not in sys.path:
        sys.path.insert(0, path)
    return path


def load_season():
    """
    その年の設定（使う識別値と待ち時間）を重ねる。走行体の起動時に 1回呼ぶ。

    年ごとの値をここから直接持たないのは、共通部分が特定の年を知ってしまうと
    翌年に共通側まで書き換えることになるため。参照は「共通 ← 年」の一方通行に
    しておき、つなぐのは SEASON_CONFIG_MODULE の 1行だけにする。

    :returns: 読み込んだ年の設定モジュール。無ければ None（共通の既定で動く）
    """
    global _SEASON, _SEASON_LOADED
    add_season_to_path()
    _SEASON_LOADED = True
    try:
        season = __import__(SEASON_CONFIG_MODULE)
    except ImportError:
        logging.getLogger(__name__).warning(
            "その年の設定 %s が見つかりません。共通の既定で動きます",
            SEASON_CONFIG_MODULE,
        )
        _SEASON = None
        return None
    TIMEOUT_CONFIG.update(getattr(season, "TIMEOUT_CONFIG", {}))
    _SEASON = season
    return season


def season_hook(name):
    """
    その年の差し込み口を引く。無ければ None。

    `ev3_python.SorotApp.season_hook` と同じものを、アプリの実体を持たない側
    （PC からの受信で動く `route_register` など）からも引けるようにしたもの。
    **共通側から年のフォルダを名指しで import しないための唯一の口。**
    """
    season = _SEASON if _SEASON_LOADED else load_season()
    fn = getattr(season, name, None)
    return fn if callable(fn) else None

# --------------------------------------------------------------------------
# 撮影
# --------------------------------------------------------------------------
IMAGE_PATH = "./sorot_spike/RoughSpot/PhotoImage/single/image.jpg"

#: 撮影の解像度。
#:
#: **走行体のカメラは1台で、色の読み取りと撮影が同じ1台を共有する**
#: （camera.py の get_shared_camera）。開けるのは1回だけなので、解像度も
#: ひとつしか選べない。
#:
#: 上げると PC 送りの画像とQRの読み取りに有利だが、色の読み取り（30Hz）の
#: 1枚あたりの取り込みとデコードが重くなる。色はラベルが読める区間
#: （200mm・約0.5秒）を通り抜けるあいだに数枚拾えればよいので、重くしすぎない。
#:
#: 以前の v4l2-ctl は 1920x1080 で1枚撮りしていた（撮影専用だったため）。
#:
#: **いちど 1920x1080 へ上げたが、1280x720 に戻した（2026-09-21、Lコース実機）。**
#:
#: 決め手は読み取り率ではなく、**1枚あたりの復号時間**だった。同じ場所・同じ
#: つまみ（露出10・ゲイン208）で 10秒ずつ測ると:
#:
#:   1280x720   47枚/10.1s = 4.65Hz  1枚 215ms   B2 45/47, C2 47/47
#:   1920x1080  19枚/10.3s = 1.84Hz  1枚 542ms   B2 19/19, C2 17/19, B1 2/19, D1 2/19
#:
#: **どちらも読めた率は 100%。**取り込みはどちらも 30Hz で、差がつくのは復号。
#: 走行中にコードが読める帯は 0.2〜0.3秒しかないので、1回の通過で試せるのは
#: 1280 が 0.9〜1.4回、**1920 は 0.4〜0.6回**。1920 では半分以上の通過で
#: 1枚も撮れずに終わる。走行中に読めない原因は、ぶれより先にこれが効く。
#:
#: **1920 が優るのは遠いコード。** B1・D1（潰れ 4.6:1）は 1920 でしか読めない。
#: ただしその 1920 でも 19枚中2枚（10%）しか拾えておらず、走行中はさらに落ちる。
#: 近いコード（B2・C2）は 1280 で 100% 読める。自己位置の打ち直しに
#: 「いちばん近い1個が読めれば足りる」なら 1280 で足りる。**遠いコードから早く
#: 補正したい設計にするなら、ここは 1920 に戻すこと。**
#:
#: CPU は 1280 が 41%、1920 が 88%（取り込みのみ、100枚の実測）。ヒントカードは
#: 1280 が 97%・218ms/枚、1920 が 96%・389ms/枚で、読み取り率は変わらない。
#:
#: **「1280 では原理的に届かない（縦 3.8px/モジュール < 要 4.0）」は、近いコード
#: には当てはまらなかった。** 1280 で読めた B2 は 171x49px ＝ 縦 2.33px/モジュール
#: で、3.8 どころか 2.5 も下回るのに 47/47 読めている。復号器が縦に 2.5〜4倍
#: 引き伸ばしてから渡すので、縦の px/モジュールはそのままでは効かない
#: （`Common/floor_code.py`）。**遠いコードで届かないのは本当**（1280 換算で
#: 108x24px ＝ 縦 1.0px/モジュール）。
#:
#: **ボトルの色は割合で見ているので、解像度では壊れない。** ROI は
#: `{'x':0.3,'width':0.4,...}` と比率で、占有率のしきい値も割合。16:9 のまま
#: なので画角も変わらない（`Common/bottle_color_monitor.py`）。
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720
#: カメラのつまみ（シャッター・ゲイン）。**None は「触らない」**。
#:
#: 既定はすべて None ＝ 自動露出のまま。色の読み取りとヒントカードはそれで足りている。
#:
#: **走行中に床の二次元コードを読むなら、露出を短くすることになる。**
#: 初期状態は自動露出で 33〜60ms。走りながらでは 1フレームに数モジュールぶん
#: 流れる（`Docs/position_correction_design.md` 5.1）。
#:
#:   CAMERA_EXPOSURE  100us 刻み（600=60ms、50=5ms）。短いほどぶれない
#:   CAMERA_GAIN      16〜255。露出を短くしたぶんを持ち上げる
#:   CAMERA_FOCUS     **走行体のカメラは固定フォーカス。書いても効かない**
#:                    （別のカメラに替えたときのために口だけ開けてある）
#:
#: **Lコースの実機で決めた値（2026-09-21）。** 1920x1080 で、
#: 止まって 48/48、手で動かしながら 26/40＝65%、見える9種すべてを読めた。
#: そのとき明るさ 131〜157・白飛び 0%。
#:
#: **ゲインを上げてあるのは露出を 1.0ms まで短くしたから。** 走行中のぶれを
#: 抑えるにはここまで短くする必要がある（速度 4.59×FWD mm/s、1モジュール
#: 2.38mm。1.0ms なら FWD 50 でも 0.1モジュールしか流れない）。
#:
#: **照明が変われば当たりも変わる。会場が変わったら掃引し直すこと**
#: （`Tools/read_floor_code.py --sweep exposure`）。仕組みは `Common/camera_tuning.py`。
#:
#: **ボトルの色と共用している。確かめた（2026-09-21、Lコース実機）。落ちない。**
#: むしろ良くなる。同じ配置で `Tools/check_bottle_color.py`:
#:
#:   黄  yellow 15/15・占有率 0.082（しきい値 0.020 の 4.1倍）
#:   赤  red    15/15・占有率 0.071（同 3.6倍）
#:   青  blue   15/15・占有率 0.069（しきい値 0.030 の 2.3倍）
#:
#: **自動露出のほうが危ない。** 同じ青ボトルを自動露出で見ると占有率 0.024 で、
#: 黄のしきい値 0.020 は超えるのに青の 0.030 に届かず、`colorPriority` で先に
#: 見る黄が成立して **yellow 15/15 と誤判定した**。ゲイン 208 が彩度と明度を
#: 持ち上げ、青の塊が育って正しく通る。
#:
#: **`HUE_RANGES` は直さなくてよい。** 点検ツールが「範囲の外に 51%」と警告して
#: いたが、あれはツール側が範囲を書き写していて青の下限が 195 のまま（本番は 180）
#: だったため。本番の範囲で数え直すと 56% → 27% になる。ツールは
#: `BottleColor.HUE_RANGES` から読むよう直した（`Tools/check_bottle_color.py`）。
#:
#: **ヒントカードもこの設定で落ちない。** カード1が 27/27（142px＝5.7px/モジュール、
#: 329ms/枚）、カード2が 40/40（162px＝6.5px/モジュール、207ms/枚）。どちらも
#: 算法の下限 4px/モジュールを上回る。**カード1は余裕が 1.4倍しかない。**
CAMERA_FOCUS = None      # このカメラは固定フォーカス。書いても効かない
CAMERA_EXPOSURE = 10     # 1.0ms
CAMERA_GAIN = 208

#: 【上の定義が本番の値。ここで二重に定義しないこと】
#: マージで同じ3つが2か所に現れ、**後の None が勝って今日の調整が無効になった**
#: （2026-09-21、現地で発見）。値の定義は上の1か所だけにする。
#:
#:   CAMERA_FOCUS     **このカメラは固定フォーカス。書いても光学的に何も変わらない**
#:                    （2026-09-21 に実機で確認。焦点を 0〜240 まで振っても成績が同じ）
#:   CAMERA_EXPOSURE  100us 刻み（600=60ms、10=1.0ms）。短いほどぶれない
#:   CAMERA_GAIN      16〜255。露出を短くしたぶんを持ち上げる
#:
#: **None は「触らない」＝自動。自動露出は現地で 60ms まで伸び、白飛びで
#: 1枚も読めなかった。** 床の二次元コードを使うなら None にしてはいけない。
#: 値は実機で決めること（`Tools/read_floor_code.py --sweep exposure`）。
#: 仕組みは `Common/camera_tuning.py`。

#: v4l2-ctl で開き直すとデバイスが使用中で失敗する。
#: 参考として残している。
CAMERA_COMMAND = (
    "v4l2-ctl --device=/dev/video0 "
    "--set-fmt-video=width=1920,height=1080,pixelformat=MJPG "
    "--stream-mmap=3 --stream-count=1 "
    "--stream-to=./sorot_spike/RoughSpot/PhotoImage/single/image.jpg"
)


#: Python 部のログの出力先。C++ 側の log.txt / stop_latency.log と同じ場所に並ぶ。
#:
#: 走行体では asp と同じディレクトリ（workspace）から起動されるため、
#: ここからの相対パスは C++ 側と同じ場所を指す。
LOG_PATH = "./sorot_spike/ev3_python.log"

#: 1ファイルの上限[byte]と、残す世代数。
#:
#: 走行を重ねると際限なく伸びるため区切る。2MB×3世代=6MBまで。
#: 1走行あたり数十KBなので、直近の数十走行ぶんが残る。
LOG_MAX_BYTES = 2 * 1024 * 1024
LOG_BACKUP_COUNT = 3

#: 通信のログを、相手ごとに別ファイルへも出す。
#:
#: Python 部は2つの相手とやり取りする。片方が黙っているとき、どちらの経路が
#: 止まっているのかが1つのファイルでは追いにくい。
#:
#:   comm_pc.log   走行体Python ⇔ PC（WebSocket 8765。pc_client / communicator）
#:
#: **C++ 部とのやりとりはここに出ない。** ファイル（/dev/shm）で受け渡すので
#: 通信のログという形にならない。様子は本体の ev3_python.log に出る。
#:
#: communicator の警告はすべてPC側の通信クラス由来（WebSocket/Callback/Tcp）で、
#: 共有している枠付け（communicator.PacketCodec）はログを出さない。
#: そのため名前で振り分けられる。
#:
#: **本体の ev3_python.log にも従来どおり出る。** 時系列でまとめて追いたいため、
#: 分けたぶんを本体から抜くことはしない。
LOG_LINKS = {
    "comm_pc.log":  ("pc_client", "communicator"),
}


# ログ設定
def setup_logging():
    """Python 部のログをテキストファイルへ出す。

    **ログが出せなくても走行は止めない。** 書けない場合は画面出力だけにして続ける。
    """
    root_logger = logging.getLogger()
    root_logger.setLevel(logging.DEBUG)

    # 二重に呼ばれてもハンドラを増やさない（重複行が出るため）
    if getattr(root_logger, "_sorot_configured", False):
        return
    root_logger._sorot_configured = True

    # 共通のフォーマッター (%(name)s を含めることでモジュール名を出力可能に)
    formatter = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")

    # ロギング用フォルダの存在確認（存在しない場合はカレントフォルダへ出力するフォールバック）
    log_path = LOG_PATH if os.path.exists(os.path.dirname(LOG_PATH)) else "./ev3_python.log"

    # ファイルハンドラー (ファイル書き出し用)
    #
    # 走行を重ねても溢れないよう、大きさで区切って世代を残す。
    try:
        file_handler = logging.handlers.RotatingFileHandler(
            log_path, maxBytes=LOG_MAX_BYTES, backupCount=LOG_BACKUP_COUNT,
            encoding="utf-8")
        file_handler.setLevel(logging.DEBUG)
        file_handler.setFormatter(formatter)
        root_logger.addHandler(file_handler)
    except OSError as e:
        # 書けなくても走行は続ける。画面には出るので、そちらで追う
        print(f"ログファイルを開けません({log_path}): {e}", file=sys.stderr)
        log_path = None

    # ストリームハンドラー (コマンドプロンプト画面出力用)
    stream_handler = logging.StreamHandler(sys.stdout)
    stream_handler.setLevel(logging.DEBUG)
    stream_handler.setFormatter(formatter)
    root_logger.addHandler(stream_handler)

    # 相手ごとの通信ログ。どちらの経路が止まっているかを別ファイルで追える
    if log_path is not None:
        _setup_link_logs(os.path.dirname(log_path) or ".", formatter)

    # 走行の区切りを入れる。1つのファイルに何度も追記されるため、
    # どこからが今回の走行かを見つけられるようにする（C++ 側の
    # StopLatencyMonitor が "===== 計測開始 =====" を入れているのと同じ理由）
    if log_path is not None:
        logging.getLogger(__name__).info(
            "===== Python部 起動 %s ===== (%s)",
            time.strftime("%Y-%m-%d %H:%M:%S"), os.path.abspath(log_path))


def _setup_link_logs(log_dir: str, formatter: logging.Formatter) -> None:
    """通信のログを相手ごとの別ファイルへも出す。

    本体のログにも出る（propagate はそのまま）。分けたぶんを本体から抜くと、
    時系列でまとめて追えなくなるため。

    **開けなくても走行は止めない。** その相手ぶんが本体のログだけになる。
    """
    for filename, logger_names in LOG_LINKS.items():
        path = os.path.join(log_dir, filename)
        try:
            handler = logging.handlers.RotatingFileHandler(
                path, maxBytes=LOG_MAX_BYTES, backupCount=LOG_BACKUP_COUNT,
                encoding="utf-8")
        except OSError as e:
            print(f"通信ログを開けません({path}): {e}", file=sys.stderr)
            continue
        handler.setLevel(logging.DEBUG)
        handler.setFormatter(formatter)
        for name in logger_names:
            logging.getLogger(name).addHandler(handler)
