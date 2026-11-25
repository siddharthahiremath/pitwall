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
GRIP_USE = 1.0     # fraction of the modelled grip the speed profile uses
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
LINE = "47q,47i,47b,475,470,46v,46s,46o,46m,46j,46i,46g,46f,46e,46d,46d,46c,46c,46b,46b,46a,46a,469,468,466,465,463,460,45x,45u,45q,45m,45g,45b,454,44x,44p,44g,446,43v,43j,436,42s,42d,41w,41e,40v,40b,3zp,3z1,3yc,3xl,3wt,3vy,3v2,3u4,3t4,3s1,3qx,3pq,3oh,3n5,3lq,3k9,3ip,3h2,3fc,3di,3bl,39k,37g,357,32v,30e,2xt,2v4,2sq,2qz,2pt,2p8,2p8,2pq,2qq,2s8,2ua,2wu,2zw,33h,37k,3c4,3h5,3mo,3sm,3z0,45t,4d1,4kl,4si,50q,597,5hy,5qu,5zx,692,6i9,6rg,70j,79i,7i7,7qm,7yn,866,8d6,8je,8oz,8tj,8x4,8zr,914,92r,94y,97o,9ay,9es,9j5,9o1,9td,9z6,a5e,aby,ait,apx,ax8,b4o,bc3,bjf,bqm,bxk,c3z,c9x,cf6,cjk,cn2,cpa,cqc,cpu,co0,cl7,chu,ce0,ca5,c6d,c2q,bzg,bwo,bug,bst,brs,brf,brs,bsv,bum,bwx,bzs,c35,c6y,cau,ceu,cio,cm1,cok,cpz,cqc,cq1,cpb,coa,cmz,clp,ckb,cj4,ci0,ch6,cgi,cg7,cg3,cgd,cgs,chm,cik,cjs,cl3,cmi,cnw,cpa,cqc,cp7,cl6,cep,c65,bvs,bjy,b6v,asu,ae2,9yt,9j7,93g,8np,881,7sl,7dk,6yz,6ky,67l,5v0,5ja,58i,4yt,4qb,4j4,4dd,497,46t,46h,48d,4cq,4jn,4qd,4vx,50d,53r,564,57i,57y,57j,568,545,51a,4xq,4tj,4or,4jg,4dq,47n,41b,3uu,3ob,3hw,3bq,35x,30o,2w5,2sk,2q8,2p8,2pv,2rx,2u8,2wv,2zh,321,34f,36g,387,39i,3ac,3ap,3am,39z,38w,37f,35h,339,30u,2y8,2vr,2ta,2rd,2pv,2p8,2ps,2rv,2vn,30e,36a,3cq,3jk,3qs,3y0,453,4bx,4if,4oi,4u0,4yx,534,56m,59c,5ba,5cf,5cr,5ca,5az,58x,562,52h,4y8,4tb,4ol,4mf,4mq,4p8,4tq,4zw,57i,5gb,5q5,60s,6c7,6o4,70e,7cw,7pj,825,8em,8qv,92r,9e7,9p4,9zb,a8q,ahb,aox,avb,b0i,b4d,b6p,b79,b60,b2r,axc,apc,age,a8e,a1f,9ve,9qb,9m6,9iz,9go,9f8,9ep,9ez,9g3,9i1,9kp,9o4,9sa,9x1,a2f,a8f,aeu,alo,asw,b0f,b7z,bfm,bn8,bun,c1q,c86,cdz,ciw,cmq,cpb,cqc,cpw,coh,cmb,cjm,cgj,cd9,c9v,c6j,c3c,c0d,bxo,bvd,bth,bry,bqv,bqa,bq7,bqn,brm,bsy,bur,bx0,bzm,c2j,c5q,c93,ccj,cfx,cj4,clz,coa,cpt,cqc,cq2,cp1,cnd,cl5,cig,cfc,cbx,c89,c4e,c0e,bwb,bs7,bo4,bk3,bg5,bcc,b8o,b57,b1u,ayq,avu,at6,aqr,aol,amn,akw,aje,ai5,ah4,ag1,adr,aaq,a6g,a1k,9vn,9p7,9hx,9a7,91w,8t7,8k6,8ar,818,7rd,7hh,77j,6xj,6nm,6dr,640,5ug,5l3,5by,534,4um,4me,4ek,474,403,3th,3nb,3hm,3cf,37p,33i,2zu,2wo,2u2,2s1,2qj,2pm,2p8,2pf,2q9,2rl,2t3,2uh,2vq,2wx,2y0,2z2,301,30y,31r,32j,33a,33z,34n,358,35s,36c,36v,37c,37r,387,38m,38z,39c,39o,3a0,3aa,3ak,3au,3b3,3bb,3bi,3bq,3bx,3c2,3c8,3cd,3ch,3cl,3co,3cr,3cs,3cu,3cu,3cu,3ct,3cs,3cq,3cn,3cj,3cf,3ca,3c3,3bw,3bp,3bf,3b5,3av,3aj,3a5,39r,39c,38w,38e,37v,37b,36q,363,35f,34q,33y,336,32c,31h,30k,2zm,2ym,2xk,2wg,2vb,2u6,2t4,2s8,2ri,2qw,2qf,2q1,2pq,2pj,2pe,2pb,2p9,2p8,2p8,2p8,2p8,2p8,2p8,2p8,2p8,2p8,2p8,2p8,2p8,2p8,2p8,2p8,2p8,2p9,2p8,2p8,2p8,2p8,2p9,2p9,2p8,2p8,2p8,2p8,2p9,2p8,2p8,2p9,2pa,2pd,2pi,2po,2pv,2q3,2qe,2qr,2r6,2ro,2s9,2su,2th,2u6,2uw,2vp,2wi,2xd,2y8,2z3,2zx,30q,31g,323,32m,330,336,33a,33x,35m,38a,3bx,3gi,3m0,3sd,3zl,47m,4gg,4pz,508,5b4,5mm,5yn,6b7,6o5,71i,7f3,7sz,86z,8l0,8z3,9cz,9qn,a42,agz,atb,b51,bg0,bq1,byy,c6p,cd7,cia,clu,cnq,co3,cnj,cm8,ckd,ci4,cfn,cd2,cai,c82,c5t,c3v,c28,c10,c08,bzx,c03,c0r,c1x,c3j,c5l,c80,car,cdq,cgp,cjl,cm9,cok,cpw,cqc,cpj,co2,clw,cjg,cgr,cdz,cbc,c8v,c6l,c4p,c37,c25,c1i,c1b,c1m,c2e,c3m,c5a,c79,c9k,cc4,ceu,chp,ckg,cn2,cph,cqc,coj,ckj,cem,c74,byb,bo7,bdb,b1u,apv,adl,a18,9ou,9cl,90n,8oy,8dr,830,7sw,7je,7al,72k,6vd,6p1,6jn,6f8,6bw,69n,68p,68x,6ao,6dv,6iq,6ph,6y2,792,7m6,7yv,89v,8j9,8r4,8xh,92f,95z,985,98z,98h,96o,93l,8z9,8tp,8mx,8f1,85z,7vw,7ku,78s,6w0,6ii,64f,5pv,5b7,4wk,4i5,44a,3ra,3fk,35g,2xc,2rw,2pf,2p8,2q5,2rw,2u6,2wj,2z1,317,335,34k,35g,35v,35k,34u,33i,31p,2zk,2x5,2up,2sd,2qe,2pa,2p8,2qo,2uq,324,3cb,3ou,43c,4jj,50z,5j9,621,6l5,749,7n4,85j,8nd,94k,9kx,a0b,aeq,as2,b49,bf9,bp0,bxh,c4n,cag,cew,chx,cjj,cjo,cib,cfe,caw,c4q,bxo,bru,bmz,bj9,bgg,bed,bd5,bcg,bcb,bcr,bdj,bep,bg8,bi0,bk0,bm8,bom,br3,bto,bwc,bz1,c1q,c4f,c72,c9o,cc6,cek,cgs,civ,ckt,cmg,cnu,cp0,cpu,cq9,cqc,cq1,cp4,cnp,cls,cj2,cfn,cbj,c6d,c0b,bt9,bly,bff,b9m,b4j,b03,awe,ata,aqu,aoz,anr,an2,amx,anc,ao7,apm,are,ato,aw8,aza,b2j,b66,b9y,be1,bi7,bml,bqw,bve,bzo,c40,c82,cby,cfk,ciq,clk,cnq,cpc,cqc,cqc,cpj,cnw,clr,cjf,ch0,cek,cc3,c9o,c7g,c5f,c3i,c1x,c0n,bzk,byu,byj,bye,byp,bzd,c09,c1i,c33,c4w,c6w,c95,cbl,ce0,cgg,civ,cl5,cn6,cou,cpx,cqc,cpx,cog,cmd,cjv,ch7,ceh,cbt,c9a,c6z,c4y,c37,c1v,c0w,c0d,c09,c0l,c1d,c2j,c42,c5y,c84,cah,ccz,cfk,ci5,ckl,cmt,com,cpu,cqc,cpz,coa,cko,cf1,c7j,by9,bnm,bbv,ayx,al6,a6x,9ry,9cu,8xg,8i4,82t,7nq,78v,6uh,6ge,62x,5py,5dk,51w,4qu,4gi,46y,3y2,3pz,3ip,3c6,36g,31k,2xg,2u6,2rp,2q1,2p8,2p8,2q2,2rv,2ul,2yc,32f,362,399,3c3,3ej,3gn,3if,3jx,3l5,3m4,3mu,3nd,3no,3nt,3nr,3nk,3n7,3mq,3m4,3lf,3km,3jr,3is,3hs,3gp,3fk,3ef,3d7,3bz,3aq,39g,386,36x,35n,34e,336,31y,30r,2zm,2yh,2xe,2wd,2ve,2uh,2tm,2st,2s2,2rf,2qu,2qc,2px,2pm,2pe,2p9,2p8,2pb,2pi,2pv,2qc,2qz,2rr,2sq,2tw,2v8,2ws,2yg,303,31o,339,34s,36a,37r,397,3al,3bz,3db,3em,3fw,3h5,3id,3jk,3kq,3lv,3my,3o1,3p2,3q3,3r2,3s0,3sy,3tu,3up,3vk,3wd,3x5,3xx,3yn,3zd,401,40p,41b,41x,42i,431,43k,442,44j,450,45f,45u,467,46k,46w,477,47h,47q,47z,487,48e,48k,48p,48u,48y,491,493,495,496,496,495,494,492,48z,48w,48s,48n,48i,48c,485,47y"

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
