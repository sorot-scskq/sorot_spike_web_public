"""
受け取った経路が、走行中のシナリオで**どの番号になるか**を知る。

走行中の差し替え（`/dev/shm/sorot_scenario_update.json`）は `SNO`/`CNO` で行を指す。
ところが**経路の番号を振るのは C++ 側**で、PC が書いてきた番号は捨てられる
（`Task/ScenarioControl.cpp` の `applyInsertPlan`）。Python は自分が置いた経路が
何番になったかを知らないままでは、1行も書き直せない。

知る道は2つある。**両方使う。**

| | どうやって | いつ分かる | 確かさ |
| --- | --- | --- | --- |
| 予測 | `RouteInserter.cpp` と同じ決め方を、手元のシナリオ JSON でやり直す | 置いた時点 | シナリオが走行中に変わっていれば外れうる |
| 観測 | 印（`SW_ROUTE`）の行の**次に走り出した行**が経路の先頭 | 経路に入った時点 | **確実** |

予測は「入る前から構えておく」ため、観測は「取り違えない」ため。
観測が取れたらそちらで上書きする。

【C++ 側の決め方（`RouteInserter.cpp` の `planAfter`）】

    印の SNO + 1 が空いていれば、そのシナリオ番号を起こして CNO 1 から並べる
    空いていなければ、印と同じ SNO の続き番号にする
"""

import json
import logging

logger = logging.getLogger(__name__)

#: 切替条件のビット（SwitchMethod/SwitchiType.h）。経路の入り口の印
SW_ROUTE = 4096

#: 走行体で走っているシナリオ。**実行時は asp と同じ場所から見た相対パス**
SCENARIO_PATH = "./sorot_spike/SubScenario/custom.json"


class RouteNumbering:
    """経路が入った場所の番号"""

    def __init__(self, sno, first_cno, marker_sno=0, marker_cno=0, observed=False):
        self.sno = sno
        self.first_cno = first_cno
        self.marker_sno = marker_sno
        self.marker_cno = marker_cno
        #: 観測で確かめたか（予測のままなら False）
        self.observed = observed

    @property
    def marker_key(self):
        return (self.marker_sno, self.marker_cno)

    def row_index(self, key, count):
        """走っている行（`(sno, cno)`）が経路の何行目か。経路の外なら None"""
        sno, cno = key
        if sno != self.sno:
            return None
        index = cno - self.first_cno
        return index if 0 <= index < count else None

    def numbers_for(self, index):
        """経路の `index` 行目の `(SNO, CNO, PRVSNO, PRVCNO)`

        先頭行の前は印そのもの（C++ が `plan.prvSno`/`prvCno` に印を入れる）。
        """
        cno = self.first_cno + index
        if index == 0:
            return (self.sno, cno, self.marker_sno, self.marker_cno)
        return (self.sno, cno, self.sno, cno - 1)

    def __repr__(self):
        return "RouteNumbering(SNO=%d CNO=%d〜 印=%d/%d %s)" % (
            self.sno, self.first_cno, self.marker_sno, self.marker_cno,
            "観測" if self.observed else "予測")

    # --- 予測 -----------------------------------------------------------------

    @classmethod
    def predict(cls, rows, rough_spot_id):
        """`RouteInserter.cpp` と同じ決め方で番号を読む。印が無ければ None

        :param rows: 走行中のシナリオ（`custom.json` と同じコマンドセットの配列）
        :param rough_spot_id: 難所の識別値（`SW_ROUTE` の `SCV`）
        """
        marker = None
        for row in rows:
            switch = row.get("SwitchInfo") or {}
            if (int(switch.get("SCJFN") or 0) & SW_ROUTE) != 0 \
                    and int(switch.get("SCV") or 0) == rough_spot_id:
                marker = row                # 同じ印が2つあったら先頭側（C++ と同じ）
                break
        if marker is None:
            return None

        marker_sno = int(marker.get("SNO") or 0)
        marker_cno = int(marker.get("CNO") or 0)
        candidate = (marker_sno + 1) & 0xFF
        used = any(int(r.get("SNO") or 0) == candidate for r in rows)
        sno = marker_sno if used else candidate
        first_cno = max([int(r.get("CNO") or 0) for r in rows
                         if int(r.get("SNO") or 0) == sno] or [0]) + 1
        return cls(sno, first_cno, marker_sno, marker_cno)

    @classmethod
    def predict_from_file(cls, path, rough_spot_id):
        """走行中のシナリオ JSON から読む。読めなければ None（走行は止めない）"""
        try:
            with open(path, encoding="utf-8") as f:
                data = json.load(f)
        except (OSError, ValueError) as e:
            logger.warning("シナリオを読めないので経路の番号を予測できません（%s）: %s", path, e)
            return None
        rows = data.get("scenario") if isinstance(data, dict) else data
        return cls.predict(rows or [], rough_spot_id)
