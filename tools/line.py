"""Offline racing-line optimiser. Writes tools/line_<name>.json with lateral offsets per centerline index."""
import json, math, os, sys
import numpy as np
from scipy.optimize import lsq_linear, minimize
from scipy import sparse
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from track import Track

T = Track()
C = np.array(T.pts)
H = np.array(T.hd)
N = np.stack([-np.sin(H), np.cos(H)], 1)
n = len(C)

P = dict(mass=800.0, power=760e3, g=9.81, mu=1.75, k_down=0.00265, k_drag=0.00098, rolling=0.15,
         drs_drag_cut=0.2, drs_down_cut=0.45, max_drive=13.0, traction=0.97, brake=0.93)


def d2_matrix(n):
    rows, cols, vals = [], [], []
    for i in range(n):
        for j, v in ((i - 1, 1.0), (i, -2.0), (i + 1, 1.0)):
            rows.append(i); cols.append(j % n); vals.append(v)
    return sparse.csr_matrix((vals, (rows, cols)), shape=(n, n))


def mincurv(w, lam_len=0.0):
    D2 = d2_matrix(n)
    # residual = D2 (C + N a): x and y parts
    Ax = D2 @ sparse.diags(N[:, 0]); Ay = D2 @ sparse.diags(N[:, 1])
    bx = -(D2 @ C[:, 0]); by = -(D2 @ C[:, 1])
    A = sparse.vstack([Ax, Ay]); b = np.concatenate([bx, by])
    if lam_len > 0:
        D1 = sparse.csr_matrix((np.r_[-np.ones(n), np.ones(n)], (np.r_[np.arange(n), np.arange(n)], np.r_[np.arange(n), (np.arange(n) + 1) % n])), shape=(n, n))
        A = sparse.vstack([A, math.sqrt(lam_len) * (D1 @ sparse.diags(N[:, 0])), math.sqrt(lam_len) * (D1 @ sparse.diags(N[:, 1]))])
        b = np.concatenate([b, -math.sqrt(lam_len) * (D1 @ C[:, 0]), -math.sqrt(lam_len) * (D1 @ C[:, 1])])
    r = lsq_linear(A, b, bounds=(-w * np.ones(n), w * np.ones(n)), method="bvls" if n < 0 else "trf", lsmr_tol="auto", max_iter=5000)
    return r.x


def geom(a):
    Pt = C + N * a[:, None]
    nxt = np.roll(Pt, -1, 0); prv = np.roll(Pt, 1, 0)
    u = Pt - prv; v = nxt - Pt
    cr = u[:, 0] * v[:, 1] - u[:, 1] * v[:, 0]
    k = 2 * cr / (np.linalg.norm(u, axis=1) * np.linalg.norm(v, axis=1) * np.linalg.norm(nxt - prv, axis=1))
    ds = np.linalg.norm(v, axis=1)
    return Pt, k, ds


def laptime(a, gf=1.0, use=1.0, drs=True, ret=False):
    Pt, k, ds = geom(a)
    k = np.abs(k)
    mu = P["mu"] * gf * use
    den = k - mu * P["k_down"]
    vc = np.where(den > 1e-9, np.sqrt(mu * P["g"] / np.maximum(den, 1e-9)), 150.0)
    vc = np.minimum(vc, 150.0)
    v = vc.copy()
    # iterate forward/backward around the loop twice for periodicity
    for _ in range(2):
        for i in range(2 * n):
            j = i % n; jn = (j + 1) % n
            vv = v[j]
            grip = mu * (P["g"] + P["k_down"] * vv * vv)
            lat = vv * vv * k[j]
            avail = math.sqrt(max(grip * grip - lat * lat, 0.0))
            a_dr = min(P["power"] / P["mass"] / max(vv, 1.0), P["max_drive"], P["traction"] * grip, avail)
            dd = drs and lat < 0.4 * grip
            kd = P["k_drag"] * (0.8 if dd else 1.0)
            acc = a_dr - kd * vv * vv - P["rolling"]
            v[jn] = min(v[jn], math.sqrt(max(vv * vv + 2 * acc * ds[j], 1.0)))
        for i in range(2 * n, 0, -1):
            j = i % n; jp = (j - 1) % n
            vv = v[j]
            grip = mu * (P["g"] + P["k_down"] * vv * vv)
            lat = vv * vv * k[j]
            avail = math.sqrt(max(grip * grip - lat * lat, 0.0))
            a_b = min(P["brake"] * grip, 55.0, avail) + P["k_drag"] * vv * vv + P["rolling"]
            v[jp] = min(v[jp], math.sqrt(vv * vv + 2 * a_b * ds[jp]))
    t = float(np.sum(2 * ds / (v + np.roll(v, -1))))
    return (t, v, k, ds) if ret else t


if __name__ == "__main__":
    w = float(sys.argv[1]) if len(sys.argv) > 1 else 7.0
    a0 = np.zeros(n)
    print("centerline model time %.3f" % laptime(a0))
    for lam in (0.0, 0.002, 0.01, 0.05):
        a = mincurv(w, lam)
        print("w=%.2f lam=%.3f model time %.3f  maxoff %.2f" % (w, lam, laptime(a), np.abs(a).max()))
        json.dump([round(float(x), 3) for x in a], open(os.path.join(os.path.dirname(__file__), "line_w%.1f_l%g.json" % (w, lam)), "w"))
