"""Bot host process: loads one bot script and calls its drive(state) every control tick.

Protocol (JSON lines over stdin/stdout):
  in : {"type":"init", "track":..., "params":..., ...}   once
  in : {"type":"tick", ...state...}                       every tick
  out: {"tick":n, "controls":{...}, "log":"...", "ms":1.2} or {"tick":n, "error":"traceback"}
"""
import io
import json
import sys
import time
import traceback

proto = sys.stdout
path = sys.argv[1]


def out(obj):
    proto.write(json.dumps(obj) + "\n")
    proto.flush()


def main():
    line = sys.stdin.readline()
    if not line:
        return
    init = json.loads(line)
    track = init["track"]
    params = init["params"]
    tyres = init.get("tyres")

    buf = io.StringIO()
    sys.stdout = buf
    g = {"__name__": "bot", "__file__": path}
    try:
        with open(path) as f:
            code = compile(f.read(), path, "exec")
        exec(code, g)
        if "drive" not in g:
            raise RuntimeError("your script must define a function drive(state) -> dict")
        if callable(g.get("init")):
            g["init"](track, params)
    except Exception:
        sys.stdout = proto
        out({"type": "fatal", "error": traceback.format_exc(limit=6)})
        # keep draining so the server doesn't block; report the error each tick
        for line in sys.stdin:
            try:
                tick = json.loads(line).get("tick")
            except Exception:
                tick = None
            out({"tick": tick, "error": "script failed to load:\n" + traceback.format_exc(limit=0)})
        return
    sys.stdout = proto
    out({"type": "ready", "log": buf.getvalue()})

    drive = g["drive"]
    from botview import TrackView, build_state
    view = TrackView(track)
    my_id = init["car_id"]
    for line in sys.stdin:
        try:
            state = build_state(view, json.loads(line), my_id)   # world snapshot -> this car's view of it
        except Exception:
            out({"tick": None, "error": "state build failed:\n" + traceback.format_exc(limit=3)})
            continue
        if state is None:
            out({"tick": None, "controls": {}, "log": "", "ms": 0})
            continue
        state["track"] = track
        state["params"] = params
        state["tyres"] = tyres
        buf = io.StringIO()
        sys.stdout = buf
        t0 = time.perf_counter()
        try:
            res = drive(state)
            if res is None:
                res = {}
            if not isinstance(res, dict):
                raise TypeError("drive() must return a dict, got %r" % type(res).__name__)
            ms = (time.perf_counter() - t0) * 1000
            sys.stdout = proto
            out({"tick": state.get("tick"), "controls": res, "log": buf.getvalue()[-2000:], "ms": ms})
        except Exception:
            sys.stdout = proto
            out({"tick": state.get("tick"), "error": traceback.format_exc(limit=4), "log": buf.getvalue()[-2000:]})


if __name__ == "__main__":
    main()
