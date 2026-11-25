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
GRIP_USE = 0.98     # fraction of the modelled grip the speed profile uses
BRAKE_USE = 1.0   # fraction of braking grip used in the backward pass
LAT = 0.075               # control latency compensation (s)
LAM0, LAM1 = 6.0, 0.25   # steering feedback length = LAM0 + LAM1 * v  (m)
PREVIEW = 0.05       # curvature feed-forward preview (s)
KV = 4.0                 # speed-error gain (1/s)
DRS_LAT = 0.6       # open DRS where lateral demand < this fraction of (DRS-open) grip
DRS_CLOSE = 0.45   # seconds of look-ahead to have the flap shut before braking/cornering
TYRE = "soft"
PIT_WEAR = 0.6     # pit when tyre wear passes this (0..1)

LINE_N = 1183
LINE = "3rq,3rf,3r6,3qx,3qp,3qh,3qb,3q5,3q0,3pv,3pq,3pm,3pj,3pf,3pc,3pa,3p7,3p4,3p2,3oz,3ox,3ov,3os,3op,3om,3oj,3og,3oc,3o8,3o4,3nz,3nt,3nn,3nh,3na,3n2,3mu,3mk,3mb,3m0,3lo,3lb,3ky,3kj,3k4,3jn,3j5,3il,3i1,3hf,3gr,3g2,3fc,3ek,3dq,3cu,3bx,3ax,39w,38s,37m,36e,353,33q,32a,30s,2z6,2xi,2vq,2tu,2rv,2pt,2nn,2lc,2ix,2gf,2e8,2co,2bq,2bc,2bk,2cb,2dl,2fe,2ht,2kq,2o5,2s5,2wn,31o,376,3d6,3jm,3qj,3xv,45n,4dr,4m8,4v1,543,5df,5mx,5wl,66c,6g5,6py,6zm,797,7ih,7ri,803,887,8fs,8mk,8sq,8xu,91z,955,973,999,9by,9f7,9iz,9na,9s4,9xf,a37,a9f,ag0,amx,au4,b1j,b94,bgu,boi,bw1,c3f,cak,ch4,cn7,csl,cx3,d0p,d32,d48,d3u,d26,czi,cwb,csm,cov,cl7,chn,ceg,cbp,c9i,c7v,c6t,c6g,c6r,c7r,c9f,cbm,cee,chn,clb,cp2,csz,cwp,czy,d2g,d3u,d48,d3z,d3c,d2e,d18,d02,cys,cxp,cwp,cvy,cvd,cv4,cv1,cvc,cvs,cwk,cxg,cyl,czr,d12,d29,d3f,d48,d2u,cyl,cry,cj7,c8n,bwl,bj9,b4z,apx,aac,9ue,9ea,8y4,8i0,824,7mk,77h,6sx,6f0,61v,5pj,5e5,53u,4uq,4mx,4gj,4bq,48o,47p,48x,4co,4ix,4p0,4tx,4xp,50g,526,52y,52s,51q,4zu,4x6,4tr,4po,4ky,4fo,49w,43p,3x6,3qf,3jk,3cp,35x,2zf,2tc,2nt,2j0,2f6,2cl,2bc,2bp,2di,2fk,2hz,2ke,2mr,2p0,2qx,2sj,2ts,2uk,2ux,2uu,2u9,2t7,2ru,2q1,2ny,2lp,2j9,2gz,2er,2d1,2bs,2bc,2c3,2eb,2i8,2n3,2t2,2zm,36k,3dx,3la,3sj,3zk,469,4ck,4ic,4nj,4s1,4vu,4yw,516,52o,53e,53a,52e,50p,4y9,4v3,4r7,4mp,4id,4gk,4h8,4k1,4ou,4vc,538,5cc,5mg,5xe,693,6lb,6xv,7an,7nl,80i,8d9,8pt,920,9dq,9oy,9zg,a97,ai3,apz,awp,b27,b6e,b92,b9y,b91,b65,b13,ath,akx,adb,a6q,a15,9wg,9sp,9px,9o0,9my,9mt,9nh,9oy,9r9,9ua,9y1,a2j,a7m,ada,ajk,aq9,axd,b4t,bck,bkc,bs5,bzx,c7i,ceq,clc,cr9,cwb,d0b,d31,d48,d3z,d2r,d0s,cya,cve,csa,cp3,clw,ciu,cfz,cdf,cb7,c9d,c7v,c6u,c6a,c67,c6m,c7j,c8u,cak,ccp,cf7,ci0,cl2,coa,crj,cur,cxr,d0f,d2k,d3v,d48,d3r,d2j,d0p,cya,cvd,cs2,cof,ckj,cge,cc5,c7t,c3f,bz1,bup,bqg,bma,bia,beg,baq,b78,b3y,b0v,ay2,avg,at3,aqw,aoy,an9,alr,ak7,ahh,ady,a98,a3v,9xg,9qj,9ir,9ak,91r,8sl,8j1,894,7z3,7or,7ed,73x,6te,6iz,68n,5ye,5oc,5ei,54w,4vl,4mm,4dy,45o,3xs,3qc,3jb,3cr,36o,313,2w1,2ri,2nh,2k1,2h5,2eu,2d3,2bx,2bc,2bc,2c0,2d6,2ei,2fq,2gu,2hw,2iv,2jr,2km,2lf,2m5,2mt,2ng,2o3,2on,2p6,2po,2q5,2qm,2r1,2rf,2rt,2s6,2sj,2su,2t6,2th,2tr,2u0,2ua,2uj,2ur,2uz,2v7,2ve,2vk,2vq,2vw,2w2,2w6,2wb,2wf,2wi,2wk,2wn,2wo,2wp,2wp,2wp,2wo,2wm,2wk,2wh,2wc,2w7,2w2,2vv,2vn,2ve,2v5,2uu,2ui,2u5,2tr,2tb,2sv,2sd,2rt,2r9,2qn,2q0,2pb,2ok,2nt,2mz,2m4,2l8,2k9,2j9,2i8,2h4,2g0,2ez,2e5,2dg,2cv,2cf,2c2,2bs,2bl,2bh,2be,2bd,2bc,2bc,2bc,2bc,2bc,2bc,2bc,2bc,2bc,2bc,2bc,2bc,2bc,2bc,2bc,2bc,2bd,2bc,2bc,2bc,2bc,2bd,2bd,2bc,2bc,2bc,2bd,2be,2bg,2bi,2bm,2bs,2c0,2ca,2cn,2d0,2dh,2e0,2em,2fb,2g3,2gz,2hv,2iu,2jv,2ky,2m3,2na,2oi,2pr,2r1,2s9,2th,2um,2vo,2wm,2xf,2y1,2yk,2zn,31r,34v,38y,3dy,3jv,3qo,3yb,46s,4g0,4pz,50m,5bx,5nt,608,6d6,6qi,749,7i7,7wf,8as,8p5,93k,9hr,9vr,a9h,amp,azb,bbb,bmk,bwv,c62,ce3,cku,cq7,cu1,cw8,cww,cwn,cvn,cu3,cs5,cpz,cnp,clf,cj9,ch9,cfk,ce6,cd6,ccm,cch,ccu,cdo,cey,cgp,civ,cle,co7,cr8,cu9,cx7,czx,d2a,d3o,d48,d3j,d29,d08,cxy,cvg,cst,cqc,co0,clv,ck2,cin,cho,ch3,cgx,ch8,chz,cj6,ckr,cmn,cou,cr9,ctu,cwi,cz1,d1f,d3l,d48,d28,cy1,crz,ckb,cbb,c11,bpy,be9,b22,aph,acu,a04,9nj,9b7,8z4,8nh,8ca,81p,7rp,7ie,79u,722,6v6,6p7,6k7,6g9,6de,6bt,6bf,6cj,6f3,6jb,6pe,6xc,77o,7k4,7w6,86j,8f8,8mg,8s7,8wi,8zf,90z,918,906,8xu,8u7,8pe,8jd,8c4,83u,7ue,7jy,78l,6w9,6j9,65k,5rb,5cn,4xw,4j6,44r,3qw,3dx,327,2s4,2jz,2ef,2bs,2bc,2bz,2di,2fk,2hq,2k2,2m3,2nx,2pb,2q6,2ql,2qd,2pp,2oh,2mt,2kt,2ik,2ga,2e5,2cc,2bc,2bc,2cu,2gw,2o7,2y8,3ak,3ou,44t,4m2,546,5mu,65v,6ox,77s,7qa,887,8pi,962,9lp,a0d,ae1,aqj,b1x,bc3,bl0,bso,byz,c3y,c7j,c9p,cag,c9o,c7d,c3i,bxy,brk,bmc,bi5,bf2,bcw,bbg,bav,bat,bbb,bce,bdu,bfm,bhs,bk6,bms,bpm,bsl,bvo,byu,c22,c5c,c8l,cbt,cey,ci2,cl2,cnw,cql,ct3,cvf,cxg,cz8,d0q,d1w,d2l,d2y,d2w,d26,d0z,cz8,cwo,ctf,cpg,ckf,ceh,c7j,c0c,btx,bo7,bj7,beu,bb6,b84,b5q,b3w,b2o,b21,b1w,b2b,b36,b4k,b6b,b8k,bb3,be3,bhb,bkw,bom,bso,bwr,c13,c5c,c9s,ce0,ci9,cm9,cq3,ctn,cwr,czk,d1o,d39,d48,d48,d3f,d1s,czn,cxb,cuw,csh,cq0,cnl,cld,cjc,chf,cft,cej,cdf,ccp,ccd,cc8,cch,cd5,ce0,cf8,cgt,cil,ckl,cmt,cp8,crn,cu3,cwi,cyt,d0v,d2l,d3q,d48,d3x,d2l,d0o,cyc,cvu,ctb,cqt,cog,cmb,ckg,ciw,cho,cgu,cge,cgd,cgr,chk,ciq,ck9,cm3,co6,cqf,css,cv7,cxk,czr,d1p,d37,d43,d48,d3h,d1d,cxc,crb,cjf,c9p,byn,bmf,b8z,auo,afv,a0a,9kj,94g,8oe,88a,7sg,7cr,6xj,6ik,646,5qb,5d0,50f,4oe,4d5,42n,3su,3ju,3bn,348,2xo,2rx,2n0,2iy,2fq,2de,2bx,2bc,2bm,2cw,2f5,2if,2m3,2pc,2s7,2up,2wu,2yp,308,31i,32j,33c,33x,34b,34j,34l,34h,348,33v,33e,32t,325,31e,30k,2zp,2yr,2xs,2ws,2vr,2up,2tm,2si,2rf,2qb,2p7,2o4,2n2,2m1,2l0,2k0,2j2,2i5,2h9,2gf,2fn,2ex,2e9,2dn,2d3,2cl,2c7,2bv,2bm,2bf,2bc,2bc,2bf,2bm,2bx,2cd,2cy,2do,2ej,2fl,2gt,2i8,2ju,2lk,2n9,2ox,2qj,2s4,2tn,2v5,2wm,2y2,2zg,30t,325,33g,34p,35x,374,38a,39e,3ah,3bk,3cl,3dk,3ej,3fh,3gd,3h8,3i2,3iw,3jo,3ke,3l4,3lt,3mh,3n4,3np,3oa,3ou,3pc,3pu,3qb,3qr,3r5,3rj,3rw,3s8,3sj,3st,3t3,3tb,3tj,3tp,3tv,3u0,3u4,3u7,3ua,3uc,3ud,3ud,3uc,3ub,3u9,3u6,3u2,3ty,3tt,3tn,3th,3ta,3t2,3st,3sk,3sb,3s1"

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
