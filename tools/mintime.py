"""Min-lap-time racing line: periodic cubic B-spline offsets, L-BFGS-B with parallel finite-diff gradients
on a quasi-steady lap model that mirrors the bot's own speed profile."""
import json, math, os, sys, time
import numpy as np
from multiprocessing import Pool
from scipy.optimize import minimize
D = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(D, ".."))
from track import Track

T = Track()
C = np.array(T.pts); H = np.array(T.hd)
N = np.stack([-np.sin(H), np.cos(H)], 1)
n = len(C)
GU = float(os.environ.get("GU", "1.02"))
mu0, g, kdn, kdr, pw, roll = 1.75 * GU, 9.81, 0.00265, 0.00098, 760e3 / 800, 0.15
GF = float(os.environ.get("GF", "1.03"))   # soft tyre at ~0.2 wear


def basis(K):
    # periodic cubic B-spline basis evaluated at n points
    B = np.zeros((n, K))
    t = np.arange(n) * K / n
    for i in range(n):
        j0 = int(math.floor(t[i])); u = t[i] - j0
        w = [(1 - u) ** 3 / 6, (3 * u ** 3 - 6 * u ** 2 + 4) / 6, (-3 * u ** 3 + 3 * u ** 2 + 3 * u + 1) / 6, u ** 3 / 6]
        for m in range(4):
            B[i, (j0 - 1 + m) % K] += w[m]
    return B


def curv(a):
    x = C[:, 0] + N[:, 0] * a; y = C[:, 1] + N[:, 1] * a
    xp, yp, xn, yn = np.roll(x, 1), np.roll(y, 1), np.roll(x, -1), np.roll(y, -1)
    ux, uy, vx, vy = x - xp, y - yp, xn - x, yn - y
    k = 2 * (ux * vy - uy * vx) / (np.hypot(ux, uy) * np.hypot(vx, vy) * np.hypot(xn - xp, yn - yp))
    for _ in range(2):
        k = (np.roll(k, 1) + 2 * k + np.roll(k, -1)) / 4
    return np.abs(k), np.hypot(vx, vy)


BOT = None
TR = None


def lap(a):
    global BOT, TR
    if BOT is None:
        import server as S
        from make_bot import make
        path = os.path.join(D, "out", "_opt_%d.py" % os.getpid())
        make(os.path.join(D, "line_w6.5_l0.json"), path, GRIP_USE=GU, LAT=0.05)
        BOT = {}
        exec(open(path).read(), BOT)
        TR = json.loads(json.dumps(S.GEOM))
        BOT["P"].update(json.loads(json.dumps(S.P)))
    BOT["build"](TR, [float(x) for x in a])
    BOT["profile"](GF)
    G = BOT["G"]
    v, ds = G["v"], G["ds"]
    return sum(2 * ds[i] / (v[i] + v[(i + 1) % n]) for i in range(n))


def lap_old(a):
    k, ds = curv(a)
    k = k.tolist(); ds = ds.tolist()
    mu = mu0 * GF
    v = [0.0] * n
    for i in range(n):
        den = k[i] - mu * kdn
        v[i] = 140.0 if den <= 1e-9 else min(140.0, math.sqrt(mu * g / den))
    for _ in range(2):
        for it in range(2 * n, 0, -1):
            j = it % n; jp = j - 1
            vv = v[j]; grip = mu * (g + kdn * vv * vv); lat = vv * vv * k[j]
            avail = math.sqrt(max(grip * grip - lat * lat, 0.0))
            ab = min(0.93 * grip, 55.0, avail) + kdr * vv * vv + roll
            lim = math.sqrt(vv * vv + 2 * ab * ds[jp])
            if lim < v[jp]: v[jp] = lim
        for it in range(2 * n):
            j = it % n; jn = (j + 1) % n
            vv = v[j]; grip = mu * (g + kdn * vv * vv); lat = vv * vv * k[j]
            d = 1.0 if lat < 0.35 * grip else 0.0
            grip_d = mu * (g + kdn * (1 - 0.45 * d) * vv * vv)
            avail = math.sqrt(max(grip_d * grip_d - lat * lat, 0.0))
            acc = min(pw / max(vv, 1.0), 13.0, 0.97 * grip_d, avail) - kdr * (1 - 0.2 * d) * vv * vv - roll
            lim = math.sqrt(max(vv * vv + 2 * acc * ds[j], 1.0))
            if lim < v[jn]: v[jn] = lim
    t = 0.0
    for i in range(n):
        t += 2 * ds[i] / (v[i] + v[(i + 1) % n])
    return t


B = None; W = None


def _init(K, w):
    global B, W
    B = basis(K); W = w


def f(c):
    a = np.clip(B @ c, -W, W)
    return lap(a)


def fpair(args):
    c, j, h = args
    cp = c.copy(); cp[j] += h
    cm = c.copy(); cm[j] -= h
    return (f(cp) - f(cm)) / (2 * h)


def main():
    global B, W
    src = sys.argv[1]; W = float(sys.argv[2]); K = int(sys.argv[3]); out = sys.argv[4]
    B = basis(K)
    a0 = np.array(json.load(open(src)))
    c0 = np.linalg.lstsq(B, np.clip(a0, -W, W), rcond=None)[0]
    pool = Pool(15, initializer=_init, initargs=(K, W))
    hist = [time.time()]

    def fg(c):
        val = f(c)
        gr = np.array(pool.map(fpair, [(c, j, 0.03) for j in range(K)], chunksize=4))
        return val, gr
    print("start %.3f" % f(c0), flush=True)
    best = [1e9, None]

    def cb(c):
        v = f(c)
        if v < best[0]:
            best[0] = v; best[1] = c.copy()
            a = np.clip(B @ c, -W, W)
            json.dump([round(float(x), 3) for x in a], open(out, "w"))
        print("it %.3f  (%.0fs)" % (v, time.time() - hist[0]), flush=True)
    r = minimize(fg, c0, jac=True, method="L-BFGS-B", bounds=[(-W, W)] * K, callback=cb,
                 options=dict(maxiter=int(os.environ.get("IT", "150")), maxcor=30))
    cb(r.x)
    print("done", r.fun, r.message)


if __name__ == "__main__":
    main()
