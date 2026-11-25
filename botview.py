"""Builds each bot's `state` from the shared world snapshot, inside the bot's own process.

The server sends one snapshot of every car per control tick to all bots. Each bot process computes its own
lidar scan, track lookahead and relative positions here, in parallel, so the race can run much faster than
real time without the server doing per-bot work.
"""
import math

import numpy as np

LIDAR_RAYS = 36
LIDAR_RANGE = 150.0
RAY_ANGLES = np.array([-math.pi + i * 2 * math.pi / LIDAR_RAYS for i in range(LIDAR_RAYS)])
RAY_ANGLE_LIST = RAY_ANGLES.tolist()
_KIND_NAMES = ("edge", "wall", "car")
_LIDAR_W = 40                       # +-40 centreline points (+-160 m) covers the 150 m range
_WIN = np.arange(-_LIDAR_W, _LIDAR_W)
_AHEAD_D = np.arange(0, 401, 10, dtype=float)


class TrackView:
    def __init__(self, geom):
        self.length = float(geom["length"])
        cl = geom["centerline"]
        self.n = n = len(cl)
        self.sa = np.concatenate([np.array(geom[k], dtype=float) for k in ("left", "right", "wall_left", "wall_right")])
        self.sb = np.concatenate([np.roll(np.array(geom[k], dtype=float), -1, axis=0)
                                  for k in ("left", "right", "wall_left", "wall_right")])
        self.poly_off = (np.arange(4) * n)[:, None]
        self.static_kinds = np.repeat(np.array([0, 0, 1, 1]), len(_WIN))
        self.ts = np.array(list(geom["s"]) + [self.length])
        self.tp = np.array(list(cl) + [cl[0]], dtype=float)
        self.th = np.unwrap(np.array(list(geom["heading"]) + [geom["heading"][0]]))
        self.tk = np.array(list(geom["curvature"]) + [geom["curvature"][0]])

    def lidar(self, x, y, h, idx, others):
        """36 rays around the car; hits the track edges, walls, or other cars' hitboxes (boxed cars are invisible)."""
        sel = (self.poly_off + (idx + _WIN) % self.n).ravel()
        A, B, kinds = self.sa[sel], self.sb[sel], self.static_kinds
        owners = None
        extra_a, extra_b, ids = [], [], []
        r2 = (LIDAR_RANGE + 5) ** 2
        for o in others:
            if o["status"] == "boxed" or (o["x"] - x) ** 2 + (o["y"] - y) ** 2 > r2:
                continue
            cs = [tuple(p) for p in o["hitbox"]["corners"]]
            extra_a += cs
            extra_b += cs[1:] + cs[:1]
            ids += [o["id"]] * 4
        if extra_a:
            A = np.concatenate([A, extra_a])
            B = np.concatenate([B, extra_b])
            kinds = np.concatenate([kinds, np.full(len(extra_a), 2)])
            owners = [None] * len(sel) + ids
        ang = RAY_ANGLES + h
        dx, dy = np.cos(ang)[:, None], np.sin(ang)[:, None]
        E = B - A
        Q = A - (x, y)
        ex, ey, qx, qy = E[:, 0], E[:, 1], Q[:, 0], Q[:, 1]
        den = dx * ey - dy * ex
        den[np.abs(den) < 1e-9] = np.nan
        inv = 1.0 / den
        t = (qx * ey - qy * ex) * inv
        u = (qx * dy - qy * dx) * inv
        with np.errstate(invalid="ignore"):
            t[~((t > 0) & (t <= LIDAR_RANGE) & (u >= 0) & (u <= 1))] = np.inf
        j = t.argmin(axis=1)
        best = t[np.arange(LIDAR_RAYS), j]
        hm = np.isfinite(best)
        dist = np.where(hm, best, LIDAR_RANGE).tolist()
        kj = np.where(hm, kinds[j], -1).tolist()
        hit = [_KIND_NAMES[k] if k >= 0 else "none" for k in kj]
        if owners is None:
            cid = [None] * LIDAR_RAYS
        else:
            jl = j.tolist()
            cid = [owners[jl[r]] if kj[r] == 2 else None for r in range(LIDAR_RAYS)]
        return dist, hit, cid

    def ahead(self, s0):
        """Centreline samples every 10 m for the next 400 m."""
        s = (s0 + _AHEAD_D) % self.length
        i = np.clip(np.searchsorted(self.ts, s, side="right") - 1, 0, len(self.ts) - 2)
        u = (s - self.ts[i]) / (self.ts[i + 1] - self.ts[i])
        x = self.tp[i, 0] + (self.tp[i + 1, 0] - self.tp[i, 0]) * u
        y = self.tp[i, 1] + (self.tp[i + 1, 1] - self.tp[i, 1]) * u
        h = (self.th[i] + (self.th[i + 1] - self.th[i]) * u + math.pi) % (2 * math.pi) - math.pi
        return [{"d": int(d), "x": a, "y": b, "heading": c, "curvature": e}
                for d, a, b, c, e in zip(_AHEAD_D.tolist(), x.tolist(), y.tolist(), h.tolist(), self.tk[i].tolist())]


def build_state(view, world, my_id):
    """Turn the shared world snapshot into the documented per-bot state dict."""
    me = None
    others = []
    for c in world["cars"]:
        if c["id"] == my_id:
            me = dict(c)
        else:
            others.append(dict(c))
    if me is None:
        return None
    idx = me.pop("_idx")
    x, y, h = me["x"], me["y"], me["heading"]
    fx, fy = math.cos(h), math.sin(h)
    L = view.length
    for o in others:
        o.pop("_idx", None)
        dx, dy = o["x"] - x, o["y"] - y
        o["rel_x"] = dx * fx + dy * fy       # metres ahead (+) / behind (-)
        o["rel_y"] = -dx * fy + dy * fx      # metres to the left (+) / right (-)
        o["distance"] = math.hypot(dx, dy)
        gap = (o["s"] - me["s"]) % L
        o["track_gap"] = gap if gap < L / 2 else gap - L
        o["teammate"] = o["owner"] == me["owner"]
    dist, hit, cid = view.lidar(x, y, h, idx, others)
    return {
        "type": "tick", "tick": world["tick"], "t": world["t"], "dt": world["dt"],
        "me": me, "cars": others,
        "lidar": [{"angle": RAY_ANGLE_LIST[i], "dist": dist[i], "hit": hit[i], "car_id": cid[i]} for i in range(LIDAR_RAYS)],
        "ahead": view.ahead(me["s"]),
    }
