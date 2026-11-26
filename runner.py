"""Bot host process: loads one bot script and calls its drive(state) every control tick.

Protocol (JSON lines over stdin/stdout):
  in : {"type":"init", "track":..., "params":..., ...}   once
  in : {"type":"tick", ...state...}                       every tick
  out: {"tick":n, "controls":{...}, "log":"...", "ms":1.2} or {"tick":n, "error":"traceback"}
"""
import io
import json
import os
import sys
import time
import traceback

proto = sys.stdout
path = sys.argv[1]


def out(obj):
    proto.write(json.dumps(obj) + "\n")
    proto.flush()


_BLOCK = ("socket.", "subprocess.", "os.system", "os.exec", "os.spawn", "os.posix_spawn", "os.fork",
          "os.forkpty", "pty.", "ctypes.", "os.kill", "os.killpg", "signal.", "sys.addaudithook",
          "os.chmod", "os.chown", "os.remove", "os.rename", "os.rmdir", "os.mkdir", "os.symlink", "os.link",
          "os.truncate", "shutil.", "urllib.", "http.", "ftplib.", "smtplib.", "webbrowser.", "winreg.")
_WRITE = os.O_WRONLY | os.O_RDWR | os.O_CREAT | os.O_APPEND | os.O_TRUNC


def install_guard():
    """Linux sandbox layer 2 (layer 1: an unprivileged user that can't read app data, no file writes).
    Python audit hooks can't be removed once installed; this blocks network, processes and ctypes."""
    def hook(event, args):
        if event.startswith(_BLOCK):
            raise PermissionError("PITWALL sandbox: %s is not allowed in bot scripts" % event)
        if event == "open" and len(args) >= 3:
            mode, flags = args[1], args[2]
            if (isinstance(mode, str) and any(ch in mode for ch in "wax+")) or (isinstance(flags, int) and flags & _WRITE):
                if args[0] != "/dev/null":
                    raise PermissionError("PITWALL sandbox: writing files is not allowed")
    sys.addaudithook(hook)


def main():
    line = sys.stdin.readline()
    if not line:
        return
    init = json.loads(line)
    track = init["track"]
    params = init["params"]
    tyres = init.get("tyres")

    if os.environ.get("PITWALL_GUARD") == "1":
        import botview  # noqa: F401  (numpy etc. load before the guard goes up)
        install_guard()

    buf = io.StringIO()
    sys.stdout = buf
    g = {"__name__": "bot", "__file__": path}
    try:
        if "code" in init:
            src = init["code"]
        else:
            with open(path) as f:
                src = f.read()
        code = compile(src, path, "exec")
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
