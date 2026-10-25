"""
JS から受け取った値を、Python の None にそろえる（シミュレータの読み込み口で共通）。

Pyodide 0.28 から、JavaScript の `null` は Python の `None` ではなく `pyodide.ffi.jsnull`
という別のものになった（PyScript 2025.8.1 以降。`undefined` はこれまでどおり `None`）。
読み込み口（sim/pyscript/*_bridge.py）は「JS 側が無ければ None」と見て分かれていたので、
そのままでは `null` が入っている所で判定をすり抜け、その先で落ちる。

**JS から値を受け取る入口でこれを通す。** 判定のほうを1つずつ直すと漏れるため。

    from sim_ffi import js_value
    manager = js_value(getattr(window, 'canvasManager', None))
    if manager is None: ...

古い Pyodide（jsnull が無い版）でもそのまま動く。
"""

try:
    from pyodide.ffi import jsnull as _JSNULL
except ImportError:  # Pyodide 0.27 以前。null は None になっていた
    _JSNULL = None


def is_none(value):
    """None か、JS の null か"""
    return value is None or (_JSNULL is not None and value is _JSNULL)


def js_value(value):
    """JS の null を None にする。それ以外はそのまま"""
    return None if is_none(value) else value
