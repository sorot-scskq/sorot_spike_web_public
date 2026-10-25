"""
走行中のシナリオを差し替える／受け取った経路を渡す置き場。

認識で分かったこと（ボトルのラベル色など）から運び先を決め、**その区間の
コマンドを書き換える**ためのもの。置いたファイルは C++ 部（ResultRead）が
100ms 周期で拾い、同じ SNO/CNO の行を上書きする。

    Python                                   C++
    ScenarioMailbox.place("…/bottle_route_red.json")
      → /dev/shm/sorot_scenario_update.json
                                             ResultRead が読む
                                             → CMD_INSERT_SCENARIO を ScenarioControl へ
                                             → Scenario::insertCommandSet が同じ行を上書き

**シナリオ側に分岐を並べなくてよくなる。** 基本のシナリオには既定のルートを
書いておき、認識できたときだけ差し替える。認識できなければ何も置かれず、
既定のルートをそのまま走る（これが逃げ道になる）。

**いま走っている行より後ろだけを差し替えること。** 前の行を入れ替えると
実行位置がずれる。C++ 側でも弾いているが、置くほうで気をつける
（Docs/scenario_update_pipeline.md）。
"""
import json
import logging
import os
from typing import Optional

try:
    import config
except ImportError:  # パッケージとして読み込まれた場合
    from . import config

logger = logging.getLogger(__name__)


class ScenarioMailbox:
    """
    C++ へシナリオを渡す置き場。

    **状態を持たない。** 生成せずにクラスのまま呼ぶ（`ScenarioMailbox.place(...)`）。
    置き場のパスはクラスの属性にしてあるので、テストは一時ディレクトリへ向けられる。
    """

    #: 走行中のシナリオ差し替えの置き場
    SCN_FILE = config.COMM_SCN_FILE
    SCN_TMP = config.COMM_SCN_TMP
    #: 受け取った経路（ETラリー）の置き場。C++ は引き取ると消す＝1回だけ渡る
    ROUTE_FILE = config.COMM_ROUTE_FILE
    ROUTE_TMP = config.COMM_ROUTE_TMP

    @classmethod
    def place(cls, path: str) -> bool:
        """
        シナリオ JSON を C++ へ渡す置き場へ複製する。

        :param path: 走行体から見たシナリオ JSON のパス
        :returns: 置けたら True
        """
        try:
            with open(path, encoding="utf-8") as f:
                body = f.read()
            json.loads(body)            # 壊れたものを渡さない
        except FileNotFoundError:
            logger.error("差し替えるシナリオが見つかりません: %s", path)
            return False
        except (OSError, ValueError) as e:
            logger.error("差し替えるシナリオを読めません（%s）: %s", path, e)
            return False

        if not cls._write(body, cls.SCN_TMP, cls.SCN_FILE, "シナリオ更新"):
            return False

        logger.info("シナリオ更新を置きました: %s", path)
        return True

    @classmethod
    def pending(cls) -> bool:
        """置いたシナリオ更新が、まだ C++ に引き取られずに残っているか

        C++ は読むときに置き場を `.taken` へ rename する（`PythonCommServer.cpp`）。
        **残っているうちに置き直すと、前に置いたぶんが消える。** 走行中に何度も
        書き直す使い方（`route_follower.py`）では、引き取られるのを待つこと。
        """
        return os.path.exists(cls.SCN_FILE)

    @classmethod
    def place_rows(cls, rows: list) -> bool:
        """
        コマンドの配列をそのまま置く（ファイルに無いものを渡すとき）。

        中身は `SubScenario/*.json` と同じ形。**同じ SNO/CNO の行を丸ごと
        上書きする**ので、変えたいキーだけでなく行を全部書くこと。

        認識の結果から**その場で作った行**を渡すためのもの。走行中に自己位置を
        打ち直す行（CoordCorrectInfo）は、測った値が入るので用意しておけない
        （`Common/coord_correct.py` / `Common/route_follower.py`）。

        :param rows: custom.json と同じ形のコマンドセットの配列
        :returns: 置けたら True
        """
        if not rows:
            return False
        if cls.pending():
            logger.warning("前のシナリオ更新がまだ引き取られていません。置き直しません")
            return False
        if not cls._write(json.dumps(rows, ensure_ascii=False),
                          cls.SCN_TMP, cls.SCN_FILE, "シナリオ更新"):
            return False

        logger.info("シナリオ更新を置きました: %s",
                    " ".join("%s/%s" % (r.get("SNO"), r.get("CNO")) for r in rows))
        return True

    @classmethod
    def place_route(cls, route: dict) -> bool:
        """
        ETラリーの経路を C++ へ渡す置き場へ置く。

        C++（ResultRead）は 100ms 周期で引き取り、引き取ると消す。**1回だけ渡る。**
        まだ引き取られていない経路があれば、新しい経路で置き換える（新しいほうが正しい）。

        :param route: {"senario_kbn": n, "scenario": [...]}
        :returns: 置けたら True
        """
        body = json.dumps(route)
        if not cls._write(body, cls.ROUTE_TMP, cls.ROUTE_FILE, "経路"):
            return False

        logger.info("走行経路を C++ へ渡しました: 区分=%s コマンド%d件",
                    route.get("senario_kbn"), len(route.get("scenario", [])))
        return True

    @classmethod
    def _write(cls, body: str, tmp: str, dest: str, label: str) -> bool:
        """
        一時ファイルへ書いてから rename する。

        rename は同じファイルシステム内では不可分なので、**C++ が書きかけを読む
        ことがない**（ロックが要らないのはこのため）。置けなくても走行は止めない。
        """
        try:
            with open(tmp, "w", encoding="utf-8") as f:
                f.write(body)
            os.replace(tmp, dest)
        except OSError as e:
            logger.error("%sを置けません（%s）: %s", label, dest, e)
            return False
        return True
