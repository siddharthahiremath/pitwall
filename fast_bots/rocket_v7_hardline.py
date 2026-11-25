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
GRIP_USE = 0.99     # fraction of the modelled grip the speed profile uses
BRAKE_USE = 1.0   # fraction of braking grip used in the backward pass
LAT = 0.075               # control latency compensation (s)
LAM0, LAM1 = 6.0, 0.25   # steering feedback length = LAM0 + LAM1 * v  (m)
PREVIEW = 0.05       # curvature feed-forward preview (s)
KV = 4.0                 # speed-error gain (1/s)
DRS_LAT = 0.6       # open DRS where lateral demand < this fraction of (DRS-open) grip
DRS_CLOSE = 0.45   # seconds of look-ahead to have the flap shut before braking/cornering
TYRE = "hard"
OUTLAP_GRIP = 0.0   # grip use on the out-lap after a stop (saves the tyres for the flying lap; 0 = push)
JIT_MARGIN = 0.01     # grip use dropped by this much when our replies arrive late often
PASS_GAP = 3.3
FOLLOW_ZONE = 0.0
LANE_RATE_STR, LANE_RATE_CRN, SHIFT_GRIP = 0.3, 0.1, 0.93
ALONG_GAP, ALONG_LAT, ALONG_REL = 0.0, 4.5, 3.0
GUARD_A, GUARD_DO, GUARD_MIN = 20.0, 12.0, 8.0   # following guard (lower = more aggressive)         # lateral centre-to-centre clearance when passing (m)
PIT_WEAR = 0.7     # pit when tyre wear passes this (0..1)

# per-car roles (all cars run this same file): QUALI cars hunt the fastest lap on softs, the rest race on hards
QUALI_CARS = []
Q_TYRE, Q_PIT_WEAR, Q_GRIP, Q_OUTLAP = "soft", 0.4, 1.005, 0.7
_ROLE = [None]

LINE_N = 1183
LINE = "47j,47c,476,470,46v,46q,46m,46i,46f,46d,46b,46a,469,468,468,468,467,467,467,466,466,465,464,463,461,460,45y,45w,45t,45q,45m,45i,45d,457,451,44u,44m,44d,443,43s,43g,433,42p,42a,41t,41c,40t,408,3zm,3yz,3ya,3xj,3wr,3vx,3v1,3u3,3t3,3s1,3qw,3pp,3og,3n4,3lq,3k9,3iq,3h3,3fe,3dl,3bp,39o,37j,35a,32v,30f,2y2,2vv,2ty,2se,2ra,2qq,2qq,2rd,2sl,2ug,2wx,2zy,33k,37o,3c9,3hb,3ms,3sm,3yu,45c,4c2,4j0,4q1,4x2,53y,5am,5gw,5mo,5rt,5w8,5zs,62g,644,64r,64m,64b,64c,659,66y,68r,6a0,6a4,693,67i,65u,64m,646,64p,66g,69l,6e1,6jo,6q9,6xo,75w,7ey,7ow,7zp,8b9,8nf,8zy,9cp,9pg,a1y,ady,ap7,aza,b7p,be0,bhu,bj9,big,bfi,baq,b4m,axp,aqj,ajn,ad8,a7j,a2s,9z5,9wq,9vl,9vt,9xg,a0e,a4m,aa2,agk,anx,avz,b4j,bd6,blb,bsc,bxm,c0y,c2l,c2q,c1o,bzo,bwy,btq,bq9,bmp,bj3,bfi,bc3,b8t,b5q,b2v,b08,axw,avw,au9,at1,arp,apm,alz,ag5,a7q,9we,9hz,90a,8fz,7u4,77o,6ll,608,5fw,4ws,4f4,3z6,3l8,39k,30c,2tc,2o6,2kj,2i5,2h0,2h5,2il,2lf,2pt,2vx,33y,3dz,3pv,43f,4ih,4yp,5fi,5w8,6c7,6qx,744,7fj,7ow,7w4,81b,84l,863,85w,842,80m,7vl,7p4,7hd,78m,6z0,6or,6e2,635,5s6,5hm,580,4zz,4u1,4q7,4o2,4n6,4n5,4nn,4ob,4ou,4oz,4oj,4nh,4lq,4j9,4g2,4c5,47k,42b,3wi,3qb,3ju,3d8,36s,30u,2vp,2rs,2pn,2px,2t8,2zz,39u,3m7,40i,4g8,4ww,5e3,5vb,6c8,6sl,784,7mm,7zv,8bt,8mc,8vc,92t,98o,9cy,9fn,9gp,9fz,9dc,98o,927,8ul,8mk,8ev,881,82f,7yb,7w1,7vk,7wk,7yp,81n,858,89j,8em,8kj,8r3,8xx,94m,9au,9ga,9kp,9nt,9pd,9p5,9mx,9ig,9bl,92j,8rs,8fs,82z,7pa,7ad,6u1,6c3,5td,5az,4u3,4fn,43r,3u9,3n0,3hx,3ew,3dx,3ez,3i0,3mz,3tr,42b,4ci,4oa,51j,5g6,5w3,6d5,6v6,7e0,7xi,8hg,91m,9ls,a5o,ap0,b7f,bom,c49,chx,cta,d1z,d7x,dbm,ddj,de3,ddo,dcf,dak,d87,d5j,d2p,czv,cx4,cul,csd,cqh,cp0,co0,cnf,cnb,cnn,coe,cpk,cr4,ct1,cv7,cxi,czs,d1x,d3n,d4m,d4i,d32,d09,cw8,cr4,cl2,ce8,c6s,byz,bqy,bit,bao,b2m,aup,an5,ag0,a9i,a3q,9yo,9ub,9qm,9nl,9lc,9jy,9jk,9k9,9m4,9p7,9tj,9z5,a5w,adp,amd,avh,b3w,baf,bdw,bdv,baw,b5n,ayq,aqm,ahd,a75,9w2,9k7,97p,8ul,8h1,835,7p7,7bf,6y1,6l4,68o,5wq,5l8,5a8,4zp,4pn,4g3,473,3yu,3rf,3l0,3fi,3as,36p,336,304,2xk,2vh,2tw,2st,2s7,2s3,2sf,2t6,2u7,2vg,2wt,2y7,2zk,30t,31w,32t,33l,348,34s,35a,35q,367,36p,377,37q,389,38s,399,39p,3a4,3ag,3ar,3b0,3b9,3bg,3bn,3bu,3c1,3c7,3ce,3ck,3cq,3cw,3d1,3d5,3d8,3da,3db,3db,3db,3da,3d9,3d7,3d5,3d2,3cy,3cu,3cp,3cj,3cc,3c5,3bw,3bm,3bb,3az,3al,3a7,39r,39b,38u,38c,37t,379,36n,360,35b,34k,33s,32y,322,315,306,2z6,2y5,2x3,2w0,2uy,2tw,2sw,2ry,2r4,2qe,2pt,2pd,2p3,2ow,2ot,2os,2os,2ou,2ow,2oz,2p1,2p3,2p5,2p7,2p9,2pc,2pf,2pj,2pn,2ps,2px,2q1,2q6,2qb,2qg,2ql,2qp,2qt,2qv,2qv,2qt,2qm,2q9,2pt,2pb,2os,2ob,2nv,2nb,2ml,2lm,2km,2jy,2jz,2l1,2mz,2pk,2sh,2vh,2yg,31e,34b,376,3a5,3dd,3h1,3la,3q1,3v7,40q,46f,4c5,4ho,4mr,4ra,4vb,4z0,52h,55t,593,5cb,5fg,5ij,5li,5ob,5qx,5t8,5v7,5wr,5xv,5yn,5zm,61b,64b,68t,6ep,6lq,6tp,72g,7bx,7m0,7wl,87j,8ip,8tw,94x,9fk,9ph,9yf,a62,aca,ah2,akf,amf,an8,an7,amq,am4,alm,alb,ald,alv,amx,aoj,aqq,atl,ax3,b18,b60,bbf,bhf,bnx,bus,c1z,c99,cgh,cne,ctt,cz7,d32,d4s,d41,d10,cw1,cph,chn,c8w,bzm,bq5,bgr,b7n,ayw,aqn,aj0,ac1,a5u,a0i,9w0,9sa,9p6,9mk,9ke,9iq,9hl,9h2,9gs,9fv,9df,98k,90w,8qq,8ed,805,7kj,744,6ng,675,5rl,5d1,4zo,4nr,4df,44v,3y6,3th,3qu,3q8,3rn,3v0,3zs,458,4aj,4ez,4ik,4ln,4ok,4ro,4vo,51c,59f,5km,5yt,6fo,6yv,7k0,86a,8su,9eu,9zg,ai5,ayg,bbw,bm7,btk,byb,c0u,c1c,bzq,bvw,bpo,bh1,b63,at0,adz,9x7,9ez,8vm,8be,7qm,75f,6jy,5yd,5d7,4tl,4co,3zm,3r1,3mr,3mm,3qf,3xi,46t,4h7,4rj,514,59m,5gp,5m1,5ph,5r1,5qm,5oa,5k4,5ef,57e,4zc,4qp,4i6,4ad,43x,3zu,3z7,437,4cp,4ra,59u,5v7,6ib,76g,7v2,8jl,97h,9ud,afw,azp,bhg,bx0,ca9,cl0,ct9,cyz,d2c,d3d,d2b,czt,cwk,ct9,cqf,cnt,cl7,ci9,cet,cav,c6g,c1n,bwj,brc,bm9,bhh,bd8,b9p,b71,b5e,b4w,b5c,b6n,b8n,bb8,be5,bh9,bke,bnf,bqc,bt6,bvw,byi,c0y,c37,c59,c75,c8v,cak,cc8,cdx,cfq,chm,cjn,clr,cnv,cpv,crn,ct2,cu0,cud,ctz,csv,cr3,cop,clt,cin,cfi,ccp,caj,c91,c87,c80,c8g,c9f,cas,ccf,ce6,cfw,chg,cim,cjb,cjc,cim,ch0,cej,cbm,c8s,c6i,c53,c4i,c4l,c58,c6c,c7y,ca2,ccp,cfu,cj8,cmq,cq2,ct4,cvq,cxr,cz0,czh,cz4,cxz,cw2,cta,cpd,ck3,cd9,c4v,bv7,bkj,b93,ax6,al0,a8v,9x0,9lm,9av,90z,8s3,8k9,8dg,87m,82r,7yu,7vt,7tm,7s9,7rm,7rl,7s2,7sx,7tx,7ur,7v5,7uv,7u3,7t5,7sc,7rx,7s1,7ss,7u9,7wi,7zm,83j,88b,8dx,8ke,8rq,8zv,98t,9if,9sm,a3a,ae9,apd,b0g,bbc,blt,bvk,c4c,cbu,chp,cle,cmf,ck9,ceo,c5s,btt,bew,axe,adu,9sq,96n,8jz,7x4,7ac,6o0,62d,5hp,4y9,4g9,3zv,3l5,387,2x5,2o3,2h6,2cj,2a9,29z,2b1,2ct,2en,2ge,2hz,2jd,2kj,2lj,2mh,2ng,2ok,2px,2rn,2tv,2wp,2zy,33i,376,3as,3e6,3h9,3jw,3m1,3nn,3ot,3pk,3pz,3q4,3q3,3py,3pr,3pj,3p8,3ot,3ob,3nn,3mw,3m0,3l0,3jw,3iq,3hg,3g5,3es,3df,3c2,3aq,39e,383,36s,35i,348,32z,31q,30i,2za,2y4,2wz,2vx,2uw,2ty,2t3,2sa,2rl,2qy,2qf,2pz,2pm,2pc,2p5,2p2,2p2,2p7,2pi,2pz,2qm,2rg,2sh,2to,2v0,2wg,2y0,2zm,319,32v,34f,35y,37f,38v,3a9,3bm,3cz,3ea,3fk,3gt,3i1,3j8,3ke,3lj,3mm,3np,3or,3pr,3qr,3rp,3sm,3tj,3ue,3v8,3w2,3wu,3xm,3yc,3z2,3zr,40e,411,41n,427,42r,43a,43s,449,44p,455,45k,45y,46b,46n,46y,478,47i,47q,47y,485,48b,48g,48l,48p,48s,48v,48x,48y,48y,48y,48x,48v,48s,48p,48k,48f,489,483,47w,47p"

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
SIDE = {}
SIDE_T = {}
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
            if gap < FOLLOW_ZONE and rel > -2.0:
                tt = gap / max(rel, 1.0)          # sitting in their wake: pull out now, pass under braking
            elif rel <= 0.3 or gap / rel > 4.0:
                continue
            else:
                tt = gap / rel
        else:
            tt = 0.0
        ci = cidx(o["s"])
        olat = o["lateral_offset"]
        olat_rate = vo * math.sin(_wrap(o["heading"] - G["chd"][ci]))
        olat_p = max(-7.5, min(7.5, olat + olat_rate * min(tt, 1.0)))
        lo, hi = min(olat, olat_p), max(olat, olat_p)
        line_there = G["off"][cidx(o["s"] + 0.0)]
        # half-alongside a car just ahead without a real speed edge: don't hang there, drop back and re-attack
        if 0 < gap < ALONG_GAP and abs(my_lat - olat) < ALONG_LAT and rel < ALONG_REL:
            vcap = min(vcap, vo - 2.5)
        conflict = (lo - need < line_there < hi + need) or (gap < 15 and lo - need < my_lat < hi + need)
        if not conflict:
            continue
        cands = [t for t in (hi + need, lo - need) if -W <= t <= W]
        if gap < 25 + 1.5 * max(rel, 0.0):
            # close: never cut across them - only the side we're already on
            side = 1 if my_lat >= 0.5 * (lo + hi) else -1
            cands = [t for t in cands if (t - 0.5 * (lo + hi)) * side > 0]
        if cands:
            prev = SIDE.get(o["id"])
            same = [t for t in cands if prev is not None and (t - 0.5 * (lo + hi)) * prev > 0]
            if same:
                tgt = same[0]          # committed: keep attacking on the side we picked
            else:
                tgt = min(cands, key=lambda t: abs(t - my_lat) + 0.5 * abs(t - line_there))
            SIDE[o["id"]] = 1 if tgt > 0.5 * (lo + hi) else -1
            SIDE_T[o["id"]] = state["t"]
            b = tgt - off_here
            if best is None or gap < best[0]:
                best = (gap, b)
        # sharing their lane: keep a gap we can stop in even if they keep braking as hard as they are now
        if gap > 0 and lo - 2.7 < my_lat < hi + 2.7:
            d_o = max(DECEL.get(o["id"], 0.0) * 1.3, GUARD_DO)
            A = GUARD_A
            R = gap - GUARD_MIN + vo * vo / (2 * d_o)
            vc = A * (-0.15 + math.sqrt(0.0225 + 2 * max(R, 0.0) / A))
            vcap = min(vcap, max(vc, vo - 3.0) if gap > GUARD_MIN else vo - 2.0)
    for k in [k for k, t in SIDE_T.items() if state["t"] - t > 2.0]:
        SIDE.pop(k, None)
        SIDE_T.pop(k, None)
    return (best[1] if best else 0.0), vcap


def drive(state):
    me = state["me"]
    if _ROLE[0] is None:
        global TYRE, PIT_WEAR, GRIP_USE, OUTLAP_GRIP
        _ROLE[0] = "quali" if me.get("car_number") in QUALI_CARS else "race"
        if _ROLE[0] == "quali":
            TYRE, PIT_WEAR, GRIP_USE, OUTLAP_GRIP = Q_TYRE, Q_PIT_WEAR, Q_GRIP, Q_OUTLAP
            G["gu"] = None
        print("car %s role %s tyre %s" % (me.get("car_number"), _ROLE[0], TYRE))
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
    rate = LANE_RATE_STR if kc < 1 / 500.0 else LANE_RATE_CRN              # change lanes on straights, gently in corners
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
        mu = P["mu"] * gf * (SHIFT_GRIP if shifting else 0.95)
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
