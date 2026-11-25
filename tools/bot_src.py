"""PITWALL bot "rocket" — precomputed racing line + physics-model speed profile.

How it drives:
  * LINE below is a lap-time-optimised racing line (lateral offset from the centerline per track point,
    computed offline). If the track ever changes, init() rebuilds a min-curvature line on the fly.
  * init() builds a speed profile along that line from the sim's own physics (grip incl. downforce,
    power/drag, friction circle, DRS drag cut), forward+backward passes around the closed lap.
  * drive() predicts the car ~1 control tick ahead (controls land one tick late), steers with
    curvature feed-forward + lateral/heading feedback, and inverts the longitudinal model to get
    exact throttle/brake for the profile acceleration.
"""
import math

# ---- tuning knobs ----
GRIP_USE = __GRIP_USE__     # fraction of the modelled grip the speed profile uses
BRAKE_USE = __BRAKE_USE__   # fraction of braking grip used in the backward pass
LAT = __LAT__               # control latency compensation (s)
LAM0, LAM1 = __LAM0__, __LAM1__   # steering feedback length = LAM0 + LAM1 * v  (m)
PREVIEW = __PREVIEW__       # curvature feed-forward preview (s)
KV = __KV__                 # speed-error gain (1/s)
DRS_LAT = __DRS_LAT__       # open DRS where lateral demand < this fraction of (DRS-open) grip
DRS_CLOSE = __DRS_CLOSE__   # seconds of look-ahead to have the flap shut before braking/cornering
TYRE = "__TYRE__"
OUTLAP_GRIP = __OUTLAP_GRIP__   # grip use on the out-lap after a stop (saves the tyres for the flying lap; 0 = push)
JIT_MARGIN = __JIT_MARGIN__     # grip use dropped by this much when our replies arrive late often
PASS_GAP = __PASS_GAP__         # lateral centre-to-centre clearance when passing (m)
PIT_WEAR = __PIT_WEAR__     # pit when tyre wear passes this (0..1)

LINE_N = __LINE_N__
LINE = "__LINE__"

P = {}
G = {"ok": False}


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def _fallback_offsets(C, H, w):
    """Pure-python min-curvature-ish line (elastic band) in case the track differs from LINE."""
    n = len(C)
    nx = [-math.sin(h) for h in H]
    ny = [math.cos(h) for h in H]
    o = [0.0] * n
    for it in range(400):
        px = [C[i][0] + nx[i] * o[i] for i in range(n)]
        py = [C[i][1] + ny[i] * o[i] for i in range(n)]
        new = o[:]
        for i in range(n):
            a, b = i - 1, (i + 1) % n
            mx = 0.5 * (px[a] + px[b]) - px[i]
            my = 0.5 * (py[a] + py[b]) - py[i]
            new[i] = max(-w, min(w, o[i] + 0.9 * (mx * nx[i] + my * ny[i])))
        o = new
    return o


def build(track, off=None):
    C = track["centerline"]
    H = track["heading"]
    n = len(C)
    if off is not None:
        pass
    elif n == LINE_N:
        off = [int(t, 36) / 1000.0 - 10.0 for t in LINE.split(",")]
    else:
        off = _fallback_offsets(C, H, track["half_width"] - 0.8)
    xs = [C[i][0] - math.sin(H[i]) * off[i] for i in range(n)]
    ys = [C[i][1] + math.cos(H[i]) * off[i] for i in range(n)]
    ds = [math.hypot(xs[(i + 1) % n] - xs[i], ys[(i + 1) % n] - ys[i]) for i in range(n)]
    s = [0.0] * n
    for i in range(1, n):
        s[i] = s[i - 1] + ds[i - 1]
    k = [0.0] * n
    hd = [0.0] * n
    for i in range(n):
        a, b = i - 1, (i + 1) % n
        ux, uy = xs[i] - xs[a], ys[i] - ys[a]
        vx, vy = xs[b] - xs[i], ys[b] - ys[i]
        cr = ux * vy - uy * vx
        k[i] = 2 * cr / (math.hypot(ux, uy) * math.hypot(vx, vy) * math.hypot(xs[b] - xs[a], ys[b] - ys[a]) + 1e-12)
        hd[i] = math.atan2(ys[b] - ys[a], xs[b] - xs[a])
    for _ in range(2):
        k = [(k[i - 1] + 2 * k[i] + k[(i + 1) % n]) / 4 for i in range(n)]
    G["L_c"] = track["length"]
    G.update(n=n, x=xs, y=ys, ds=ds, s=s, k=k, hd=hd, off=off, L=sum(ds), idx=None, gf=None, ok=True,
             cx=[c[0] for c in C], cy=[c[1] for c in C], cs=track["s"], chd=track["heading"], hw=track["half_width"],
             bias=0.0, last=None, late=0.0, outlap=False, laps0=None, gu=None)


def profile(gf, dmg=0.0, gu=None):
    gu = GRIP_USE if gu is None else gu
    n, k, ds = G["n"], G["k"], G["ds"]
    mu = P["mu"] * gf * gu
    g, kdn, kdr = P["g"], P["k_down"] * (1 - 0.35 * dmg), P["k_drag"] * (1 + 0.25 * dmg)
    pw = P["power"] * (1 - 0.25 * dmg) / P["mass"]
    amax, tl, bl, roll = P["max_drive_accel"], P["traction_limit"], P["brake_limit"] * BRAKE_USE, P["rolling"]
    dcut, ddown = P["drs_drag_cut"], P["drs_down_cut"]
    v = [0.0] * n
    for i in range(n):
        ak = abs(k[i])
        den = ak - mu * kdn
        v[i] = 140.0 if den <= 1e-9 else min(140.0, math.sqrt(mu * g / den))
    drs = [0.0] * n
    brk = [False] * n
    for rep in range(3):
        # backward (braking) pass, DRS shut
        for it in range(2 * n, 0, -1):
            j = it % n
            jp = j - 1 if j > 0 else n - 1
            vv = v[j]
            grip = mu * (g + kdn * vv * vv)
            lat = vv * vv * abs(k[j])
            avail = math.sqrt(max(grip * grip - lat * lat, 0.0))
            ab = min(bl * grip, 55.0, avail) + kdr * vv * vv + roll
            lim = math.sqrt(vv * vv + 2 * ab * ds[jp])
            if lim < v[jp]:
                v[jp] = lim
                brk[jp] = True
        # forward (acceleration) pass with DRS where it is open
        for it in range(2 * n):
            j = it % n
            jn = j + 1 if j < n - 1 else 0
            vv = v[j]
            d = drs[j]
            grip = mu * (g + kdn * (1 - ddown * d) * vv * vv)
            lat = vv * vv * abs(k[j])
            avail = math.sqrt(max(grip * grip - lat * lat, 0.0))
            adr = min(pw / max(vv, 1.0), amax, tl * grip, avail)
            acc = adr - kdr * (1 - dcut * d) * vv * vv - roll
            lim = math.sqrt(max(vv * vv + 2 * acc * ds[j], 1.0))
            if lim < v[jn]:
                v[jn] = lim
                brk[jn] = False
        # DRS zones: lateral demand small even with the flap open, not braking, and shut DRS_CLOSE before trouble
        ok = [False] * n
        for i in range(n):
            vv = v[i]
            grip_open = mu * (g + kdn * (1 - ddown) * vv * vv)
            ok[i] = (not brk[i]) and vv * vv * abs(k[i]) < DRS_LAT * grip_open
        # look-ahead: shut if any not-ok point within v*DRS_CLOSE metres
        nxt_bad = [0.0] * n   # distance to next not-ok point
        dist = 1e9
        for it in range(2 * n - 1, -1, -1):
            j = it % n
            if not ok[j]:
                dist = 0.0
            else:
                dist += ds[j]
            nxt_bad[j] = dist
        for i in range(n):
            drs[i] = 1.0 if nxt_bad[i] > v[i] * DRS_CLOSE + 5.0 else 0.0
        if rep < 2:
            # reset speeds for the next refinement with updated DRS
            for i in range(n):
                ak = abs(k[i])
                den = ak - mu * kdn
                v[i] = 140.0 if den <= 1e-9 else min(140.0, math.sqrt(mu * g / den))
            brk = [False] * n
    G["v"], G["drs"], G["gf"], G["dmg"], G["gu"] = v, drs, gf, dmg, gu


def init(track, params):
    P.update(params)
    build(track)
    profile(1.0)


def locate(px, py):
    n, xs, ys = G["n"], G["x"], G["y"]
    i0 = G["idx"]
    if i0 is None:
        rng = range(n)
    else:
        rng = [(i0 + d) % n for d in range(-8, 40)]
    best, bi = 1e18, 0
    for i in rng:
        d = (px - xs[i]) ** 2 + (py - ys[i]) ** 2
        if d < best:
            best, bi = d, i
    if i0 is not None and best > 400:   # lost (pit exit, spin...): global search
        for i in range(n):
            d = (px - xs[i]) ** 2 + (py - ys[i]) ** 2
            if d < best:
                best, bi = d, i
    # pick segment
    a = bi
    j = (a + 1) % n
    ex, ey = xs[j] - xs[a], ys[j] - ys[a]
    u = ((px - xs[a]) * ex + (py - ys[a]) * ey) / (ex * ex + ey * ey + 1e-12)
    if u < 0:
        a = (bi - 1) % n
        j = bi
        ex, ey = xs[j] - xs[a], ys[j] - ys[a]
        u = ((px - xs[a]) * ex + (py - ys[a]) * ey) / (ex * ex + ey * ey + 1e-12)
    u = max(0.0, min(1.0, u))
    seg = math.hypot(ex, ey) + 1e-12
    e = (ex * (py - ys[a]) - ey * (px - xs[a])) / seg
    G["idx"] = a
    return a, u, e


def at(arr, i, u):
    n = G["n"]
    return arr[i] + (arr[(i + 1) % n] - arr[i]) * u


def ahead_idx(i, u, dist):
    """index/fraction dist metres further along the line."""
    n, ds = G["n"], G["ds"]
    d = dist + u * ds[i]
    while d > ds[i]:
        d -= ds[i]
        i = (i + 1) % n
    return i, d / ds[i]


def cidx(s_track):
    cs, n = G["cs"], G["n"]
    L = G["L_c"]
    s_track %= L
    i = int(s_track / L * n) % n
    while i > 0 and cs[i] > s_track:
        i -= 1
    while i < n - 1 and cs[i + 1] <= s_track:
        i += 1
    return i


DECEL = {}
_PREV = {}


def track_others(state):
    t = state["t"]
    for o in state["cars"]:
        p = _PREV.get(o["id"])
        if p and t - p[0] > 1e-3:
            a = (p[1] - o["speed"]) / (t - p[0])
            DECEL[o["id"]] = DECEL.get(o["id"], 0.0) * 0.4 + max(a, 0.0) * 0.6
        _PREV[o["id"]] = (t, o["speed"])


def traffic(state, me, v, i_line):
    """Returns (bias in metres to add to the racing line here, speed cap)."""
    W = G["hw"] - 0.7
    off_here = G["off"][i_line]
    my_lat = me["lateral_offset"]
    best = None
    vcap = 1e9
    need = PASS_GAP
    for o in state["cars"]:
        if o.get("status") != "track":
            continue
        gap = o["track_gap"]
        if gap < -8 or gap > 160:
            continue
        vo = o["speed"]
        rel = v - vo
        if gap > 6:
            if rel <= 0.3 or gap / rel > 4.0:
                continue
            tt = gap / rel
        else:
            tt = 0.0
        ci = cidx(o["s"])
        olat = o["lateral_offset"]
        olat_rate = vo * math.sin(_wrap(o["heading"] - G["chd"][ci]))
        olat_p = max(-7.5, min(7.5, olat + olat_rate * min(tt, 1.0)))
        lo, hi = min(olat, olat_p), max(olat, olat_p)
        line_there = G["off"][cidx(o["s"] + 0.0)]
        conflict = (lo - need < line_there < hi + need) or (gap < 15 and lo - need < my_lat < hi + need)
        if not conflict:
            continue
        cands = [t for t in (hi + need, lo - need) if -W <= t <= W]
        if gap < 25 + 1.5 * max(rel, 0.0):
            # close: never cut across them - only the side we're already on
            side = 1 if my_lat >= 0.5 * (lo + hi) else -1
            cands = [t for t in cands if (t - 0.5 * (lo + hi)) * side > 0]
        if cands:
            tgt = min(cands, key=lambda t: abs(t - my_lat) + 0.5 * abs(t - line_there))
            b = tgt - off_here
            if best is None or gap < best[0]:
                best = (gap, b)
        # sharing their lane: keep a gap we can stop in even if they keep braking as hard as they are now
        if gap > 0 and lo - 2.7 < my_lat < hi + 2.7:
            d_o = max(DECEL.get(o["id"], 0.0) * 1.3, 12.0)
            A = 20.0
            R = gap - 8.0 + vo * vo / (2 * d_o)
            vc = A * (-0.15 + math.sqrt(0.0225 + 2 * max(R, 0.0) / A))
            vcap = min(vcap, max(vc, vo - 3.0) if gap > 8.0 else vo - 2.0)
    return (best[1] if best else 0.0), vcap


def drive(state):
    me = state["me"]
    if not G["ok"]:
        init(state["track"], state["params"])
    v = me["speed"]
    gf = me["tyre_grip_factor"]
    dmg = me["damage"]
    # late-reply detection: did the controls we sent last tick actually land?
    last = G["last"]
    if last is not None:
        late = (state["tick"] - last[0] > 1) or abs(me["steer"] - last[1]) > 1e-6
        G["late"] += ((1.0 if late else 0.0) - G["late"]) * 0.02
    # out-lap after a stop: take it easy to keep the tyres fresh for the next flying lap
    if me["status"] != "track":
        G["outlap"] = OUTLAP_GRIP > 0
        G["laps0"] = None
    elif G["outlap"]:
        if G["laps0"] is None:
            G["laps0"] = me["laps"]
        elif me["laps"] > G["laps0"]:
            G["outlap"] = False
    gu = OUTLAP_GRIP if G["outlap"] else GRIP_USE - (JIT_MARGIN if G["late"] > 0.04 else 0.0)
    if G["gf"] is None or abs(gf - G["gf"]) > 0.004 or abs(dmg - G["dmg"]) > 0.02 or gu != G["gu"]:
        profile(gf, dmg, gu)

    pit = me["tyre_wear"] > PIT_WEAR or dmg > 0.35
    out = {"throttle": 0.0, "brake": 0.0, "steer": 0.0, "drs": 0, "pit": pit, "tyre": TYRE}
    if me["status"] != "track":
        G["idx"] = None
        G["last"] = None
        G["bias"] = 0.0
        return out

    # ---- predict where the car is when these controls land ----
    px = me["x"] + me["vx"] * LAT
    py = me["y"] + me["vy"] * LAT
    ph = me["heading"] + me["yaw_rate"] * LAT
    i, u, e = locate(px, py)
    track_others(state)
    btgt, vcap = traffic(state, me, v, i)
    kc = max(abs(G["k"][i]), abs(G["k"][(i + 3) % G["n"]]), abs(G["k"][(i + 8) % G["n"]]))
    rate = 0.30 if kc < 1 / 500.0 else 0.10              # change lanes on straights, gently in corners
    shifting = abs(btgt - G["bias"]) > 0.3
    G["bias"] += max(-rate, min(rate, btgt - G["bias"]))
    W = G["hw"] - 0.7
    off_here = G["off"][i]
    Wl = max(W, abs(off_here))
    G["bias"] = max(-Wl, min(Wl, off_here + G["bias"])) - off_here   # never aim off the asphalt
    bias = G["bias"]
    e -= bias
    psi = _wrap(ph - (G["hd"][i] + _wrap(G["hd"][(i + 1) % G["n"]] - G["hd"][i]) * u))

    # ---- steering: curvature feed-forward + 2nd-order lateral feedback ----
    j, uj = ahead_idx(i, u, v * PREVIEW)
    kff = at(G["k"], j, uj)
    lam = LAM0 + LAM1 * v
    kcmd = kff - e / (lam * lam) - 2.0 * psi / lam
    steer = math.atan(kcmd * P["wheelbase"]) / P["max_steer"]

    # ---- speed: hit the profile a short way ahead ----
    j2, u2 = ahead_idx(i, u, max(v * 0.05, 0.5))
    vt = at(G["v"], j2, u2)
    j3, u3 = ahead_idx(j2, u2, max(v * 0.10, 1.0))
    vt2 = at(G["v"], j3, u3)
    if abs(bias) > 0.3 or shifting:
        # off the racing line: corner speed for the offset path (k' = k / (1 - k*d)), small margin
        kb = max(abs(at(G["k"], j2, u2)), abs(at(G["k"], j3, u3)))
        kk = kff
        kb2 = abs(kk / max(0.2, 1 - kk * bias))
        kb = max(kb, kb2)
        mu = P["mu"] * gf * (0.88 if shifting else 0.95)
        den = kb - mu * P["k_down"]
        if den > 1e-9:
            vb = math.sqrt(mu * P["g"] / den)
            vt, vt2 = min(vt, vb), min(vt2, vb)
    if vcap < vt:
        vt = vt2 = max(vcap, 0.0)
    a_ff = (vt2 * vt2 - vt * vt) / (2 * max(v * 0.10, 1.0))
    a_des = a_ff + KV * (vt - v)

    drs_now = me["drs"] / 100.0
    grip = P["mu"] * me["tyre_grip_factor"] * P["surface"].get(me["surface"], 1.0) * \
        (P["g"] + P["k_down"] * (1 - P["drs_down_cut"] * drs_now) * (1 - 0.35 * dmg) * v * v)
    if me["sliding"]:
        grip *= P["kinetic_grip"]
    lat_need = v * v * abs(kcmd)
    lat_room = math.sqrt(max(grip * grip - lat_need * lat_need, 0.0))
    kd = P["k_drag"] * (1 - P["drs_drag_cut"] * drs_now) * (1 + 0.25 * dmg)
    need = a_des + kd * v * v + P["rolling"]
    throttle = brake = 0.0
    if need >= 0:
        adr = min(P["power"] * (1 - 0.25 * dmg) / P["mass"] / max(v, 1.0), P["max_drive_accel"])
        cap = min(P["traction_limit"] * grip, lat_room * 0.98)
        throttle = min(1.0, need, cap) / adr if adr > 0 else 1.0
        throttle = max(0.0, min(1.0, min(need, cap) / adr))
    else:
        bmax = min(P["brake_limit"] * grip, P["max_brake_accel"])
        brake = min(1.0, -need / bmax, lat_room * 0.98 / bmax)
        brake = max(0.0, brake)

    # recovery: facing the wrong way or off on the grass
    if abs(me["heading_error"]) > 1.2:
        steer = -1.0 if _wrap(me["heading"] - me["track_heading"]) > 0 else 1.0
        throttle, brake = (0.4, 0.0) if v < 8 else (0.0, 0.6)

    jd, _ = ahead_idx(i, u, 0.0)
    drs = 100 if G["drs"][jd] > 0.5 and brake == 0.0 else 0

    steer = max(-1.0, min(1.0, steer))
    out.update(throttle=throttle, brake=brake, steer=steer, drs=drs)
    G["last"] = (state["tick"], steer)
    return out
