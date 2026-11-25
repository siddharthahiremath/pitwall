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
GRIP_USE = 1.02     # fraction of the modelled grip the speed profile uses
BRAKE_USE = 1.0   # fraction of braking grip used in the backward pass
LAT = 0.05               # control latency compensation (s)
LAM0, LAM1 = 6.0, 0.25   # steering feedback length = LAM0 + LAM1 * v  (m)
PREVIEW = 0.05       # curvature feed-forward preview (s)
KV = 4.0                 # speed-error gain (1/s)
DRS_LAT = 0.6       # open DRS where lateral demand < this fraction of (DRS-open) grip
DRS_CLOSE = 0.45   # seconds of look-ahead to have the flap shut before braking/cornering
TYRE = "soft"
PIT_WEAR = 0.6     # pit when tyre wear passes this (0..1)

LINE_N = 1183
LINE = "47j,47c,476,471,46v,46q,46m,46i,46f,46d,46b,46a,469,468,468,468,467,467,467,466,466,465,464,463,461,460,45y,45w,45t,45q,45m,45i,45d,457,451,44u,44m,44d,443,43s,43g,433,42p,42a,41u,41c,40t,408,3zm,3yz,3ya,3xj,3wr,3vx,3v1,3u3,3t3,3s0,3qw,3pp,3og,3n4,3lq,3ka,3iq,3h4,3fe,3dl,3bo,39n,37i,359,32v,30g,2y3,2vw,2ty,2sd,2r8,2qn,2qo,2rc,2sl,2ug,2ww,2zv,33f,37i,3c3,3h6,3mo,3sj,3yr,45a,4c2,4j1,4q5,4xa,54c,5b5,5hm,5nm,5t2,5xs,61r,650,67k,69f,6an,6b8,6b8,6ao,69m,687,66m,64x,63c,61x,60v,60b,60c,612,62n,655,68q,6dg,6jh,6qv,6zk,79g,7kd,7w7,88t,8m2,8zt,9dx,9rz,a5j,ai6,atg,b2x,ba5,beo,bg8,bf5,bbu,b6t,b0g,at9,aln,ae3,a6x,a0f,9ur,9q2,9ml,9kh,9jy,9l5,9o5,9sp,9yn,a5n,adj,am2,auy,b3z,bcx,blg,bt9,c03,c5m,c9k,cbn,cbk,c9d,c5f,c04,btt,bmy,bfu,b8x,b2k,aww,arv,anj,ajs,ago,ae4,ac5,aan,a9a,a7l,a4y,a0x,9uy,9mi,9b2,8w9,8ei,7uk,799,6nb,61k,5gs,4xp,4h1,42p,3qj,3gb,37u,30x,2vc,2qw,2ng,2l4,2k3,2ki,2mk,2qe,2w7,344,3e8,3q4,43h,4hw,4wz,5cd,5ro,66i,6kk,6xm,79j,7k3,7t4,80e,85s,892,8a8,89d,86n,827,7w7,7os,7g3,769,6vl,6kh,69a,5yh,5of,5fi,586,52r,4z7,4x7,4we,4wd,4wu,4xe,4xp,4xf,4wf,4uo,4s6,4ou,4kp,4fq,49v,434,3vs,3o5,3gl,39e,32x,2xi,2ti,2r7,2qq,2s8,2vu,31m,39q,3k9,3xb,4cw,4ui,5dj,5xf,6hm,71h,7kg,81y,8hj,8v5,96o,9g3,9nc,9sc,9v3,9vh,9tl,9ps,9kc,9dk,95t,8xf,8oo,8fx,87h,7zm,7so,7mw,7im,7g5,7fq,7hp,7ly,7s4,7zs,88i,8hx,8rl,915,9a5,9i9,9p5,9ui,9xz,9za,9y3,9u1,9mw,9cu,909,8lk,856,7nh,74w,6lt,62n,5jq,51f,4k2,43y,3ph,3cy,32p,2v0,2pv,2n9,2n4,2pd,2ty,30t,39w,3l5,3yh,4dr,4uv,5dr,5ya,6kd,77w,7wl,8lw,9b4,9zn,amt,b80,bqk,c5v,chm,cq5,cvs,cyw,czu,cyz,cwn,ct8,coz,ck2,ceq,c93,c3c,bxp,bsb,bne,bj0,bf6,bby,b9d,b7f,b65,b5l,b5q,b6k,b7z,b9w,bc8,bew,bht,bkv,bnz,bqy,btj,bvi,bwm,bwm,bv9,bsb,bnl,bhb,b9s,b1a,as6,air,a9c,a09,9rr,9jy,9cy,96t,91l,8xd,8u8,8s8,8rh,8ry,8tr,8wv,91d,979,9em,9nf,9xl,a8g,ajg,atx,b39,bat,bfz,bi5,bh2,bd4,b6r,ayd,aof,adb,a1h,9pc,9d4,90u,8og,8bz,7zf,7ms,7a1,6x7,6kc,67l,5v3,5iy,57c,4we,4m7,4cx,44l,3x3,3qe,3kh,3f8,3al,36k,331,300,2xg,2vf,2tv,2ss,2s6,2s2,2se,2t4,2u4,2vd,2wr,2y6,2zk,30t,31w,32s,33j,346,34q,358,35p,366,36o,377,37p,388,38r,398,39o,3a3,3af,3aq,3b0,3b8,3bg,3bn,3bt,3c0,3c6,3cd,3cj,3cp,3cv,3d0,3d4,3d7,3d9,3db,3db,3db,3da,3d9,3d7,3d5,3d2,3cy,3cu,3cp,3cj,3cc,3c5,3bw,3bm,3ba,3ay,3al,3a7,39s,39b,38u,38c,37t,379,36n,360,35b,34l,33s,32y,322,315,306,2z6,2y5,2x3,2w1,2uy,2tw,2sw,2rz,2r4,2qe,2pt,2pe,2p3,2ow,2ot,2os,2ot,2ov,2ox,2oz,2p1,2p3,2p5,2p7,2p9,2pc,2pf,2pj,2pn,2pr,2pw,2q1,2q6,2qb,2qf,2qk,2qo,2qr,2qt,2qv,2qv,2qu,2qs,2qo,2qj,2qc,2q3,2pt,2ph,2p4,2oo,2oa,2nz,2nu,2nz,2oh,2pe,2qu,2sw,2vh,2ym,326,365,3af,3ez,3jr,3op,3tq,3ys,43v,48u,4dq,4if,4mw,4r3,4v1,4yr,52b,55q,590,5c8,5fe,5ij,5lm,5ol,5rg,5u3,5wj,5yq,60m,629,63v,65n,67u,6an,6ea,6j0,6oz,6wa,74p,7e1,7o1,7yj,89a,8k3,8ur,952,9eu,9nx,9w4,a39,a96,adp,agn,ai5,aii,ai0,agy,afo,aeg,adl,ade,ady,afa,ahh,akh,aoe,at7,ayy,b5m,bd0,bkv,bsx,c0v,c8h,cff,clh,cqe,cu1,cwd,cx9,cwp,cuk,cqt,clc,ce7,c5p,bw6,bm0,bbk,b17,arb,ai8,aa7,a3a,9xh,9sq,9p1,9me,9kt,9ka,9kl,9l9,9lu,9ls,9kn,9hw,9d1,95m,8vk,8j9,859,7px,79t,6tb,6cy,5x5,5ia,54l,4sd,4hv,49c,432,3zb,3y9,3zo,42y,47g,4ck,4ho,4m7,4pi,4r3,4ra,4qt,4qh,4qz,4t4,4xl,557,5gj,5v8,6co,6w6,7h2,82p,8of,99k,9tj,ac2,asv,b7s,bkk,buz,c2s,c7t,c9u,c8x,c56,byl,bpb,bdd,ayv,ahu,9yi,9dd,8qy,83t,7gg,6tf,679,5mg,53i,4mp,48f,3wz,3oo,3ju,3it,3lv,3st,42n,4ei,4re,54e,5gj,5qv,5yh,634,64y,649,61a,5w9,5pg,5h3,57h,4x7,4n5,4e7,477,431,42j,46l,4fx,4u5,5cd,5xo,6l6,79z,7z5,8nu,9b7,9wy,agz,az5,bfc,btg,c5d,cex,cm4,cr5,cua,cvu,cw3,cvb,cts,crt,cpl,cn5,ckf,che,ce2,cac,c69,c1q,bwu,brs,bmr,bhz,bdn,ba0,b78,b5j,b50,b5i,b6t,b8u,bbd,be9,bhb,bkf,bnh,bqf,bta,bw0,byk,c0z,c37,c58,c72,c8s,caf,cc3,cds,cfl,chk,cjr,cm3,coe,cqj,csc,cto,cud,cu9,ct9,crg,cp1,cm9,cjb,cge,cdr,cbk,ca1,c94,c8r,c8w,c9g,cac,cbi,ccw,cef,cfx,chb,cif,cj5,cjd,ciw,cho,cfm,ccz,ca2,c75,c4j,c2h,c1a,c19,c2h,c4r,c7u,cbi,cfg,cji,cnc,cqs,ctn,cvx,cxj,cyj,cyw,cyj,cxi,cvq,ct7,cpu,clj,cg7,c9s,c27,btc,bj5,b7t,avl,aiu,a5w,9t2,9gp,952,8uj,8l4,8cv,85o,7zj,7uf,7qa,7n3,7kr,7j7,7i7,7hl,7h5,7gr,7g6,7f8,7ds,7c0,7a2,785,76h,758,74l,74t,760,788,7bg,7fo,7kw,7r3,7ya,86g,8fk,8ph,903,9b9,9mu,9yo,aao,amp,ayg,b9j,bjf,brn,bxq,c16,c1j,byb,bre,bh8,b45,aon,a74,9o0,93p,8im,7x6,7bo,6qg,65u,5m5,53p,4mu,47w,3uw,3jr,3ab,32h,2w3,2r3,2na,2km,2iw,2hz,2hq,2hz,2il,2je,2k9,2l1,2lr,2mi,2ne,2oh,2pv,2ro,2tx,2wp,2zx,33f,371,3an,3e3,3h8,3jw,3m0,3nl,3op,3pg,3pv,3q1,3q1,3pw,3pq,3ph,3p5,3oq,3o7,3nl,3mu,3lz,3kz,3jv,3io,3hf,3g3,3er,3de,3c1,3ao,39d,381,36r,35h,347,32y,31p,30h,2z9,2y3,2wz,2vw,2uw,2ty,2t2,2sa,2rl,2qz,2qg,2pz,2pm,2pc,2p5,2p2,2p3,2p8,2pj,2pz,2qm,2rf,2sg,2to,2v0,2wh,2y1,2zn,319,32v,34f,35y,37f,38v,3a9,3bn,3cz,3ea,3fk,3gt,3i1,3j8,3ke,3lj,3mn,3np,3or,3pr,3qr,3rp,3sn,3tj,3ue,3v9,3w2,3wu,3xm,3yd,3z2,3zr,40e,411,41n,428,42r,43a,43s,449,44q,455,45k,45y,46b,46n,46y,478,47i,47q,47y,485,48b,48g,48l,48p,48s,48v,48x,48y,48z,48y,48x,48v,48t,48p,48k,48f,489,483,47w,47p"

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
    G.update(n=n, x=xs, y=ys, ds=ds, s=s, k=k, hd=hd, off=off, L=sum(ds), idx=None, gf=None, ok=True,
             cx=[c[0] for c in C], cy=[c[1] for c in C])


def profile(gf, dmg=0.0):
    n, k, ds = G["n"], G["k"], G["ds"]
    mu = P["mu"] * gf * GRIP_USE
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
    G["v"], G["drs"], G["gf"], G["dmg"] = v, drs, gf, dmg


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


def drive(state):
    me = state["me"]
    if not G["ok"]:
        init(state["track"], state["params"])
    v = me["speed"]
    gf = me["tyre_grip_factor"]
    dmg = me["damage"]
    if G["gf"] is None or abs(gf - G["gf"]) > 0.004 or abs(dmg - G["dmg"]) > 0.02:
        profile(gf, dmg)

    pit = me["tyre_wear"] > PIT_WEAR or dmg > 0.35
    out = {"throttle": 0.0, "brake": 0.0, "steer": 0.0, "drs": 0, "pit": pit, "tyre": TYRE}
    if me["status"] != "track":
        G["idx"] = None
        return out

    # ---- predict where the car is when these controls land ----
    px = me["x"] + me["vx"] * LAT
    py = me["y"] + me["vy"] * LAT
    ph = me["heading"] + me["yaw_rate"] * LAT
    i, u, e = locate(px, py)
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

    out.update(throttle=throttle, brake=brake, steer=max(-1.0, min(1.0, steer)), drs=drs)
    return out
