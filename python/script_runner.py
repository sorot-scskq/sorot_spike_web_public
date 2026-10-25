"""
シナリオが指定したスクリプトを実行する。

シナリオのコマンド（RunInfo.SCRIPT）に **Python スクリプトのパス**を書くと、
その区間に入ったときに C++ 部がパスを送ってきて、ここが読んで走らせる。

    {"SCRIPT": "2026/recog/bottle_color.py", "FUNCNO": 5, "FWD": 30, ...}

**認識を足すときに、共通側も C++ も触らずに済ませるための仕掛け。**
以前は認識ごとに走行方法（FUNCNO 12/13/16/17）を作っていたため、その行では
走行を指定できず、識別値も C++ と Python の両方に書く必要があった。

スクリプトの入口は `run(request_id: int) -> None`。長い処理でも呼び出し側が
別スレッドで呼ぶので要求ループは止まらないが、さらに待つものがあるなら
自分でスレッドを切ること。

**落ちても走行は止めない。** 見つからない・読めない・例外、どれもログだけ出す。
認識が動かなければ結果が来ないだけで、シナリオは既定のルートを走る。
"""
import importlib.util
import logging
import os
import threading
from typing import Optional

from config import PYTHON_ROOT

logger = logging.getLogger(__name__)


class ScriptRunner:
    """
    シナリオに書かれたパスのスクリプトを読んで走らせる。

    読み込み済みのものを覚えているので、走行体では1つを使い回す
    （`get_instance()`）。根（`root`）を差し替えて作れるのはテスト用。
    """

    _instance: Optional["ScriptRunner"] = None
    _instance_lock = threading.Lock()

    @classmethod
    def get_instance(cls) -> "ScriptRunner":
        """走行体で共有する1つを返す。無ければ作る"""
        with cls._instance_lock:
            if cls._instance is None:
                cls._instance = cls()
            return cls._instance

    @classmethod
    def reset_instance(cls) -> None:
        """共有している1つを捨てる（テスト用。読み込み済みも忘れる）"""
        with cls._instance_lock:
            cls._instance = None

    def __init__(self, root: Optional[str] = None):
        #: この下のスクリプトしか読まない
        self.root = root or PYTHON_ROOT
        #: 読み込み済みのスクリプト（パス → モジュール）。毎回読み直さない
        self._loaded = {}
        self._lock = threading.Lock()

    # --- 実行 -----------------------------------------------------------------

    def run(self, script: str, request_id: int) -> bool:
        """
        スクリプトの run(request_id) を別スレッドで呼ぶ。

        :param script: 根（`root`）からの相対パス
        :param request_id: C++ が採番した要求ID
        :returns: 呼び出しを起こせたら True
        """
        path = self._resolve(script)
        if path is None:
            return False

        try:
            module = self._load(path)
        except Exception as e:  # noqa: BLE001 — 読めなくても走行は続ける
            logger.error("SCRIPT の読み込みに失敗しました（%s）: %s", script, e)
            return False

        if module is None:
            return False

        entry = getattr(module, "run", None)
        if not callable(entry):
            logger.error("SCRIPT に run(request_id) がありません: %s", script)
            return False

        def _call():
            try:
                entry(request_id)
            except Exception as e:  # noqa: BLE001 — 認識が落ちても走行は続ける
                logger.error("SCRIPT の実行に失敗しました（%s）: %s", script, e)

        threading.Thread(target=_call, name="script:" + os.path.basename(path),
                         daemon=True).start()
        logger.info("SCRIPT を実行します: %s (request_id=%s)", script, request_id)
        return True

    # --- 読み込み -------------------------------------------------------------

    def _resolve(self, script: str) -> Optional[str]:
        """
        根からの相対パスを絶対パスにする。

        **根の外は読ませない。** シナリオは実行時に差し替えられるファイルなので、
        そこに書かれたパスをそのまま開くと、置き場の外の何でも実行できてしまう。

        :returns: 絶対パス。読めない指定なら None
        """
        if not script or os.path.isabs(script):
            logger.error("SCRIPT は RoughSpot/Python からの相対パスで書いてください: %s", script)
            return None

        path = os.path.normpath(os.path.join(self.root, script))
        if not path.startswith(self.root + os.sep):
            logger.error("SCRIPT が RoughSpot/Python の外を指しています: %s", script)
            return None
        if not os.path.isfile(path):
            logger.error("SCRIPT が見つかりません: %s", path)
            return None
        return path

    def _load(self, path: str):
        """スクリプトを読む（一度読んだら使い回す）"""
        with self._lock:
            module = self._loaded.get(path)
            if module is not None:
                return module

            name = "sorot_script_" + os.path.splitext(os.path.basename(path))[0]
            spec = importlib.util.spec_from_file_location(name, path)
            if spec is None or spec.loader is None:
                logger.error("SCRIPT を読み込めません: %s", path)
                return None
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            self._loaded[path] = module
            return module

    def loaded(self) -> dict:
        """読み込み済みのスクリプト（写し）。読み直していないことを確かめるのに使う"""
        with self._lock:
            return dict(self._loaded)
