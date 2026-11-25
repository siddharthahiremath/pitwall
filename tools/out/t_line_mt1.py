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
LINE = "47q,47j,47d,477,472,46w,46s,46o,46l,46i,46g,46f,46e,46e,46d,46d,46d,46c,46c,46b,46b,46a,469,468,466,464,462,460,45x,45u,45q,45m,45h,45b,454,44x,44p,44g,446,43v,43j,436,42s,42d,41w,41e,40v,40b,3zp,3z1,3yc,3xl,3wt,3vy,3v2,3u4,3t4,3s1,3qx,3pq,3oh,3n5,3lr,3ka,3iq,3h3,3fd,3dj,3bm,39k,37d,352,32m,304,2xn,2vc,2t9,2ri,2q6,2pd,2p4,2ph,2qe,2rw,2ty,2wk,2zq,33e,37m,3cb,3hg,3n2,3t3,3zh,468,4db,4kq,4sf,50f,58p,5ha,5q5,5za,68o,6i9,6rw,71f,7aq,7jm,7s0,7zp,86l,8cn,8i0,8mp,8qu,8uk,8xw,910,93x,96s,99o,9cq,9g0,9jm,9no,9sa,9xi,a3d,a9q,agk,anr,ava,b31,baz,biy,bqu,byf,c5i,cbw,chc,clp,cor,cqd,cqo,cpt,cnz,cld,ci3,cee,cae,c6a,c28,byf,buz,bs4,bpy,bop,boi,bpf,bra,btx,bx5,c0s,c4n,c8l,cce,cfx,cj5,cly,co9,cq0,cr5,crk,cr8,cq5,cok,cml,ckf,ci9,cga,cen,cdj,cd0,ccz,cdg,cec,cfm,ch7,cj2,cl5,cn6,cot,cpp,cph,cnt,ckc,ceo,c6l,bw8,bjz,b68,arc,abp,9vo,9fo,900,8kp,85s,7r8,7cz,6z3,6li,687,5v8,5iu,57b,4wx,4ny,4gp,4bf,48e,47r,493,4bz,4g1,4ks,4pt,4uo,4yy,52e,54x,56m,57g,57f,56k,54v,52c,4z2,4v1,4qe,4l5,4fe,497,42m,3vq,3op,3hq,3b1,34v,2zg,2uy,2ro,2pr,2p7,2ps,2ra,2ti,2w7,2z7,328,355,37r,3a0,3bt,3d3,3dq,3dm,3cq,3az,38i,35i,329,2yy,2vs,2t0,2qu,2ph,2oy,2pb,2qj,2so,2vq,2zq,34p,3al,3h8,3og,3w0,43p,4bc,4iq,4pn,4vx,51g,566,59z,5cr,5eg,5ex,5e4,5c3,594,55l,51r,4y0,4un,4rz,4qe,4q1,4qx,4t4,4wn,51h,57o,5fa,5o9,5yj,69v,6m1,6yt,7c0,7pc,82l,8fk,8s4,945,9fl,9qb,a0a,a9e,ahk,aop,auq,azi,b2y,b4z,b5h,b4d,b1i,aww,aqt,ajq,ac2,a4b,9wu,9q5,9ko,9gr,9eb,9d6,9d7,9e7,9fz,9ie,9la,9oi,9s4,9w6,a0q,a5w,abp,ai8,apk,axn,b68,bf2,bnu,bwc,c48,cb9,ch5,clr,cp4,crc,csf,csi,crl,cps,cn5,cjr,cfw,cbo,c7b,c31,byz,bve,bsf,bq6,bol,bno,bne,bnp,bol,bq0,bry,bud,bx4,c05,c3e,c6q,ca3,cde,cgj,cjf,clz,co4,cpp,cqo,cqx,cqc,cov,cmm,cjo,cg7,cca,c84,c3t,bzh,bv8,br4,bn4,bj9,bfi,bbw,b8g,b54,b1x,ayx,aw2,ate,aqy,aoq,amq,akz,ajf,ahy,agd,aej,aca,a9g,a5v,a1d,9vx,9pm,9ij,9ar,92e,8tk,8kc,8at,814,7r8,7h9,778,6x6,6n7,6dc,63m,5u3,5ks,5bq,52x,4uf,4m9,4ef,46z,3zw,3t9,3n3,3hd,3c5,37g,33b,2zr,2wr,2ub,2sf,2r0,2q4,2pq,2ps,2qa,2r5,2sa,2tn,2v4,2wm,2y2,2zd,30i,31g,328,32w,33h,340,34i,350,35j,362,36m,376,37p,388,38p,394,39i,39t,3a4,3ad,3al,3at,3b1,3b8,3bg,3bn,3bu,3c1,3c8,3cd,3ci,3cn,3cq,3cs,3ct,3cu,3cu,3cu,3ct,3cr,3cp,3cm,3cj,3cf,3ca,3c4,3bx,3bp,3bg,3b6,3au,3ai,3a5,39q,39b,38v,38e,37v,37c,36r,365,35h,34r,340,336,32b,31f,30h,2zi,2yi,2xg,2wf,2vd,2uc,2tc,2sf,2rm,2qw,2qb,2pw,2pl,2pe,2pb,2p9,2p9,2pa,2pb,2pb,2pa,2pa,2p9,2p8,2p7,2p7,2p6,2p6,2p7,2p7,2p8,2p9,2p9,2pa,2pa,2p9,2p9,2p8,2p8,2p7,2p7,2p7,2p7,2p7,2p8,2p9,2pb,2pd,2pf,2pi,2pl,2pp,2pu,2q0,2q9,2ql,2qz,2rh,2s3,2st,2tl,2uf,2va,2w5,2wz,2xs,2yi,2z6,2zt,30f,310,31l,326,32s,33h,34c,35k,378,39j,3cm,3gl,3lm,3rs,3yz,476,4g7,4q0,50i,5bl,5n4,5z3,6bg,6o5,716,7eh,7s2,85w,8jx,8y3,9c8,9q7,a3v,ah2,atm,b5f,bgb,bq6,byy,c6l,cd1,ci8,cm4,com,cpq,cpi,co8,cm3,cjb,cg5,cct,c9i,c6h,c3u,c1m,bzv,byn,bxz,bxx,byk,bzv,c1t,c47,c6x,c9v,ccu,cfr,cig,ckt,cms,cob,cpc,cpu,cpr,cp1,cno,clm,cj0,cg1,ccu,c9l,c6g,c3k,c14,bz7,bxv,bx4,bx1,bxl,byu,c0s,c3g,c6r,cae,ce3,chh,cka,cm6,cmv,cm1,cjl,cfk,ca4,c3b,bvb,bm7,bc2,b10,ap8,acy,a0b,9nl,9ay,8ym,8mu,8bq,81f,7rv,7j0,7au,73a,6wc,6pz,6k5,6f1,6av,67v,66a,66e,68d,6ch,6iw,6r8,712,7bt,7n0,7y5,88q,8i9,8qb,8wt,91u,95g,97p,98l,987,96l,93s,8zr,8uj,8o3,8gf,87j,7xe,7m0,79f,6vx,6hs,63a,5op,5aa,4wc,4j4,46u,3vn,3lo,3d0,35s,305,2w5,2tz,2ti,2ue,2wb,2yw,31t,34p,379,392,3a3,3ac,39x,38x,37d,35d,32z,30a,2xl,2v8,2tm,2t3,2u2,2ww,31y,39m,3jp,3vx,49x,4pf,562,5nj,65j,6np,75x,7o1,85w,8nf,94f,9kt,a0f,af5,asv,b5g,bgt,bqt,bzd,c6b,cbl,cf3,ch0,chi,cgq,cew,cc4,c8m,c4k,c04,bvh,bqu,bmf,bif,bez,bcd,baq,ba5,bah,bbl,bda,bfg,bhx,bkk,bn7,bps,bsb,bus,bx7,bzm,c1z,c4c,c6q,c93,cbf,cdp,cfx,ci2,ck3,cm1,cns,cpd,cql,crf,crr,crg,cqf,col,clv,cib,ce1,c98,c3z,byf,bsp,bmx,bh9,bbt,b6m,b1u,axj,att,aqr,aoh,an1,ame,amg,an4,aob,aq0,as2,auf,ax1,azv,b2y,b69,b9u,bdn,bhp,bm1,bqk,bv7,bzu,c4e,c8p,ccp,cg9,cj9,cln,cng,con,cpa,cpc,cov,cnu,cma,cka,chy,cfd,cco,c9z,c7e,c52,c32,c1f,c06,bz9,byn,byc,byb,byk,bz1,bzr,c0q,c1z,c3j,c5e,c7l,ca5,cd1,cg4,cj9,cm7,cos,cqu,cs5,csi,crt,cq7,cnu,cky,chq,cee,cb5,c87,c5p,c3n,c21,c0t,bzy,bzf,bz8,bza,bzm,c09,c15,c2e,c3y,c5v,c85,cat,cdt,cgu,cjn,clx,cnf,cnu,cmw,ckc,cg2,ca4,c2o,btt,bjo,b8d,aw1,air,a4o,9q0,9aw,8vi,8g2,80o,7lk,76v,6so,6f0,61w,5pb,5da,51u,4qz,4gq,474,3y6,3py,3ih,3bu,361,313,2x3,2u0,2rs,2qe,2pu,2q2,2r0,2so,2v1,2xw,314,34k,381,3be,3ei,3h5,3j9,3ku,3lz,3mr,3n7,3nf,3ng,3ne,3n9,3n2,3mt,3mg,3m0,3lg,3ks,3k0,3j2,3i1,3gw,3fp,3eg,3d6,3bv,3al,39b,382,36t,35l,34e,337,320,30u,2zo,2yj,2xg,2we,2vd,2uf,2tk,2sr,2s1,2re,2qu,2qd,2pz,2po,2pf,2pa,2p8,2pa,2ph,2ps,2q9,2qx,2rr,2ss,2u0,2vd,2wu,2ye,300,31m,338,34s,36b,37s,398,3am,3bz,3db,3em,3fw,3h5,3id,3jk,3kq,3lv,3my,3o1,3p2,3q3,3r2,3s0,3sy,3tu,3up,3vk,3wd,3x5,3xx,3yn,3zd,401,40p,41b,41x,42i,431,43k,442,44j,44z,45f,45t,467,46k,46w,477,47h,47r,47z,487,48e,48k,48p,48t,48x,490,493,495,496,497,496,495,493,490,48w,48s,48m,48g,48a,483,47x"

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


def build(track):
    C = track["centerline"]
    H = track["heading"]
    n = len(C)
    if n == LINE_N:
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
