"""
キャリーボトルの色認識を、シミュレータのブラウザ内で動かす（PyScript 配線）。

走行体（Raspberry Pi）で動く色判定ロジック本体は
  src/RoughSpot/Python/Common/bottle_color_monitor.py
にある。ここはシミュレータ固有の配線（ブラウザ DOM、Canvas 画素取得、ボタン連携）だけを持つ。
"""

import asyncio
from js import alert, console, document, window
from pyodide.http import pyfetch


class PyScriptColorBridge:
    """
    PyScript / ブラウザ DOM と BottleColorMonitor を接続するシミュレータ専用ブリッジ。
    """

    def __init__(self, monitor=None):
        self.monitor = monitor
        self.classifier = getattr(monitor, "classifier", None)

    def get_canvas_frame(self, canvas_id="cameraView"):
        """指定 canvas から画素データを取得する。"""
        canvas = document.getElementById(canvas_id)
        if not canvas:
            return None

        ctx = canvas.getContext("2d", {"willReadFrequently": True})
        w = int(canvas.width)
        h = int(canvas.height)
        if w <= 0 or h <= 0:
            return None

        image_data = ctx.getImageData(0, 0, w, h)
        return {"data": image_data.data, "width": w, "height": h}

    def detect_and_alert(self, event=None):
        """前面カメラ中央の色を検知し、アラートダイアログで表示する。"""
        if not self.classifier:
            alert("色判定モジュールをロード中です。しばらくお待ちください。")
            return
        try:
            frame = self.get_canvas_frame("cameraView")
            if not frame:
                alert("前面カメラ (cameraView) の映像を取得できませんでした。")
                return

            result = self.classifier.classify(frame["data"], frame["width"], frame["height"])
            color_name = BottleColor.NAMES.get(result["color"], result["color"])
            coverage_pct = round(result["coverage"] * 100, 1)
            counts = result["counts"]

            msg = (
                f"【PyScript 前面カメラ色検知結果】\n\n"
                f"■ 判定結果: {color_name}\n"
                f"■ 占有率: {coverage_pct}%\n\n"
                f"■ 画素カウント (ROI内):\n"
                f"  ・赤 (RED):    {counts['red']} px\n"
                f"  ・青 (BLUE):   {counts['blue']} px\n"
                f"  ・黄 (YELLOW): {counts['yellow']} px"
            )
            alert(msg)
            console.log(f"PyScript BottleColor Detection: {result['color']} ({coverage_pct}%)")
        except Exception as e:
            console.error(f"PyScript detect_and_alert error: {e}")
            alert(f"色検知中にエラーが発生しました:\n{e}")

    def setup(self, button_id="pyDetectBottleColorButton"):
        """DOMイベントと window グローバルへのエクスポートを行う。"""
        btn = document.getElementById(button_id)
        if btn:
            btn.onclick = self.detect_and_alert

        # JavaScript グローバルへ関数・クラスをエクスポート
        window.pyDetectBottleColor = self.detect_and_alert
        if self.classifier:
            window.pyClassifyBottleColor = self.classifier.classify
        if "BottleColor" in globals():
            window.BottleColor = BottleColor
        if "BottleColorMonitor" in globals():
            window.BottleColorMonitor = BottleColorMonitor

        console.log("PyScript: BottleColorMonitor (sim/pyscript/bottle_color_bridge.py) のバインドが完了しました。")


bridge = PyScriptColorBridge()


async def _init():
    global BottleColor, BottleColorClassifier, BottleColorMonitor, ColorConverter
    try:
        for _mod_name in ("bottle_color.py", "color_converter.py", "bottle_color_monitor.py"):
            _resp = await pyfetch(f"python/{_mod_name}")
            with open(_mod_name, "w") as _f:
                _f.write(await _resp.string())

        from bottle_color_monitor import (
            BottleColor,
            BottleColorClassifier,
            BottleColorMonitor,
            ColorConverter,
        )

        bridge.monitor = BottleColorMonitor()
        bridge.classifier = bridge.monitor.classifier
        bridge.setup()
    except Exception as e:
        console.error(f"PyScript bottle_color_bridge init error: {e}")


# 非同期タスクとして実行（トップレベル await の構文エラーを防止）
asyncio.ensure_future(_init())
