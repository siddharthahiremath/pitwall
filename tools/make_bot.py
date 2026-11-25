import json, os, sys
D = os.path.dirname(os.path.abspath(__file__))
DEF = dict(GRIP_USE=0.99, BRAKE_USE=1.0, LAT=0.075, LAM0=6.0, LAM1=0.25, PREVIEW=0.05, KV=4.0,
           DRS_LAT=0.6, DRS_CLOSE=0.45, TYRE="soft", PIT_WEAR=0.6, OUTLAP_GRIP=0.0, JIT_MARGIN=0.01, PASS_GAP=3.3, GUARD_A=20.0, GUARD_DO=12.0, GUARD_MIN=8.0, ALONG_GAP=0.0, ALONG_LAT=4.5, ALONG_REL=3.0, LANE_RATE_STR=0.30, LANE_RATE_CRN=0.10, SHIFT_GRIP=0.93, FOLLOW_ZONE=0.0, QUALI_CARS=[], Q_PIT_WEAR=0.40, Q_GRIP=1.005, Q_OUTLAP=0.7)


def b36(x):
    x = int(round((x + 10.0) * 1000))
    d = "0123456789abcdefghijklmnopqrstuvwxyz"
    s = ""
    while True:
        s = d[x % 36] + s
        x //= 36
        if x == 0:
            return s


def make(line_file, out, **kw):
    p = dict(DEF); p.update(kw)
    off = json.load(open(line_file))
    src = open(os.path.join(D, "bot_src.py")).read()
    src = src.replace("__LINE_N__", str(len(off))).replace("__LINE__", ",".join(b36(o) for o in off))
    for k, v in p.items():
        src = src.replace("__%s__" % k, str(v))
    open(out, "w").write(src)
    return out


if __name__ == "__main__":
    kw = {}
    for a in sys.argv[3:]:
        k, v = a.split("=")
        kw[k] = v
    make(sys.argv[1], sys.argv[2], **kw)
