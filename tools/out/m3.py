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
LINE = "47j,47c,476,470,46v,46q,46m,46i,46f,46d,46b,46a,469,468,468,468,467,467,467,466,466,465,464,463,461,460,45y,45w,45t,45q,45m,45i,45d,457,451,44u,44m,44d,443,43s,43g,433,42p,42a,41t,41c,40t,408,3zm,3yz,3ya,3xj,3wr,3vx,3v1,3u3,3t3,3s0,3qw,3pp,3og,3n5,3lq,3k9,3iq,3h3,3fe,3dl,3bp,39o,37j,35a,32v,30f,2y1,2vu,2tx,2se,2rb,2qq,2qp,2rb,2sk,2uf,2ww,2zx,33i,37m,3c7,3h9,3mq,3sk,3ys,45a,4c2,4j1,4q5,4x9,547,5at,5h0,5mo,5rs,5wc,609,63l,66a,68d,69u,6aq,6b1,6as,6a0,68r,671,64z,62t,60t,5z6,5y7,5y1,5yv,60w,64b,696,6fg,6mw,6ve,74r,7et,7pg,80i,8bv,8ne,8z0,9am,9m6,9xk,a8q,aja,ass,b0q,b6n,ba7,bbi,bat,b8d,b4e,az8,at9,amx,agq,ab1,a64,a24,9z4,9x7,9wj,9x2,9ys,a1p,a5r,aax,ah0,anu,av9,b31,bav,bia,bot,btw,bx4,byh,bya,bwv,bui,bri,bo3,bkf,bgp,bd2,b9o,b6k,b3s,b1d,azd,axs,awl,avu,avg,avf,av3,atk,apx,aj8,a8u,9v2,9eh,8vo,8b8,7pp,73q,6hu,5wm,5cm,4u7,4dl,3yz,3mi,3cd,34b,2y0,2t4,2pa,2m9,2k6,2jc,2jx,2m7,2qc,2wc,348,3e1,3pr,434,4hp,4x1,5cn,5s5,677,6lj,6yy,7b5,7lx,7v3,82f,87t,8b2,8c4,8b5,88b,83r,7xn,7q5,7hf,77n,6x0,6lo,6a2,5yk,5nn,5dq,555,4y2,4sk,4os,4mt,4mf,4n6,4of,4pp,4qh,4qk,4py,4ol,4mg,4jl,4fy,4bn,46p,414,3v1,3ol,3hz,3be,350,2za,2uw,2sl,2t1,2wt,33v,3dq,3py,446,4jw,50p,5i6,5zs,6h5,6xw,7du,7ss,86j,8ix,8tt,935,9at,9gs,9kz,9na,9nm,9lx,9i4,9ca,94y,8wn,8o0,8fk,87t,811,7vj,7rm,7pk,7pc,7qr,7tp,7xx,83a,89k,8gi,8nx,8vk,938,9al,9hd,9na,9s1,9v9,9wk,9vl,9rw,9l6,9bk,8zj,8li,85w,7p4,77h,6p9,66s,5oa,566,4ox,492,3v5,3jm,3ap,34c,30g,2yz,2zv,332,38g,3fz,3pl,415,4ef,4t5,590,5pr,677,6pa,77z,7r6,8au,8us,9ep,9yb,ahb,azf,bg8,bva,c88,cim,cq9,cvf,cyj,czx,d00,cz3,cxd,cv2,csf,cpm,cms,ck3,chl,cfe,cdm,cc6,cb0,ca1,c97,c8g,c7w,c7l,c7o,c87,c97,cah,cbx,cdg,ceu,cfv,cg7,cfh,cde,c9o,c4k,bya,br3,bj8,baz,b2g,ats,al7,acs,a4q,9x3,9q0,9jl,9dy,993,954,920,8zu,8ym,8yh,8zg,91n,955,9a0,9g9,9ny,9x3,a7n,aj1,au8,b47,bbz,bgq,bih,bhn,bel,b9o,b36,av7,als,ab0,9yx,9lr,97v,8tk,8fa,81b,7nq,7ah,6xf,6kj,67r,5v7,5j1,57f,4wi,4mf,4d6,44r,3x7,3qg,3ki,3fa,3ao,36o,334,302,2xi,2vf,2tv,2st,2s8,2s3,2se,2t3,2u5,2vf,2wt,2y9,2zl,30s,31u,32s,33k,348,34s,359,35p,366,36o,377,37q,389,38s,399,39p,3a3,3ag,3ar,3b0,3b9,3bg,3bn,3bu,3c0,3c7,3cd,3cj,3cp,3cv,3d0,3d4,3d8,3da,3db,3db,3db,3da,3d9,3d7,3d5,3d2,3cy,3cu,3cp,3cj,3cc,3c5,3bw,3bm,3bb,3ay,3al,3a7,39r,39b,38u,38c,37t,379,36n,360,35b,34k,33s,32y,322,315,306,2z6,2y5,2x3,2w0,2uy,2tw,2sw,2ry,2r4,2qe,2pt,2pd,2p3,2ow,2ot,2os,2ot,2ou,2ox,2oz,2p1,2p3,2p5,2p7,2p9,2pc,2pf,2pj,2pn,2ps,2pw,2q1,2q6,2qb,2qg,2qk,2qp,2qs,2qu,2qw,2qw,2qu,2qs,2qp,2qk,2qe,2q6,2pw,2pj,2p4,2op,2oc,2o2,2nz,2o4,2om,2pj,2qy,2t0,2vm,2yr,32c,36a,3ak,3f4,3jv,3os,3tu,3yx,440,490,4du,4ii,4mx,4r4,4v3,4yu,52e,55s,591,5c8,5fe,5ik,5lo,5oo,5rj,5u6,5wl,5yr,60q,62h,641,65o,67o,6ad,6e5,6j8,6pk,6x1,75i,7ev,7ox,7zk,8ak,8ls,8x1,982,9if,9rs,9zp,a5u,aab,adb,af5,ag2,agb,ag4,afq,afd,afb,afo,agi,ahv,ajr,am8,apa,asz,axc,b2d,b83,bef,blb,bsl,c08,c80,cfl,cmj,csf,cwv,czl,d0g,czd,cw8,cqy,cjs,cb9,c1u,bs2,bic,b8v,azs,ar5,aj3,abp,a52,9z6,9u5,9q1,9mt,9ki,9j2,9if,9il,9j6,9jo,9jl,9ie,9fm,9ax,93z,8ui,8i5,832,7m0,73w,6lm,641,5nn,58l,4v0,4j1,48s,40k,3un,3re,3r2,3tq,3yk,44i,4al,4fs,4jl,4ma,4o7,4po,4r2,4sx,4w0,512,58t,5jw,5y3,6eu,6xn,7hz,83a,8oz,9ae,9uy,ae1,av5,ba1,bme,bvz,c2j,c62,c6n,c4g,bzk,bs6,bie,b6h,asm,ad0,9vu,9da,8tj,88r,7n5,713,6f3,5tp,59g,4qx,4as,3xs,3oo,3k6,3kq,3ph,3xb,475,4ht,4sg,52k,5br,5jj,5pj,5ti,5vg,5vc,5t7,5p1,5j2,5bk,52u,4t6,4j2,49o,428,3y0,3y7,43f,4dc,4rj,59m,5v4,6jb,795,7zl,8pm,9eb,a19,am8,b4z,blb,bz2,caa,cj3,cpm,cty,cwb,cx3,cwo,cvg,cts,cru,cpl,cn2,ckb,cha,cdy,caa,c68,c1o,bwq,brl,bmi,bhq,bdi,ba0,b7c,b5l,b4w,b5a,b6n,b8r,bbe,beb,bhb,bkd,bne,bqc,bt8,bvz,byk,c0z,c38,c58,c72,c8s,cag,cc5,cdw,cfr,chq,cju,cm3,cof,cqo,csm,cu0,cum,cue,cte,crq,cph,cms,cjt,cgt,ce2,cbs,ca7,c9a,c8y,c93,c9n,cai,cbn,ccy,cef,cfx,che,cim,cjf,cjl,ciy,chk,cfk,cd4,cae,c7k,c4x,c2t,c1k,c1j,c2s,c55,c8b,cbz,cfw,cju,cnl,cr2,cu0,cwc,cxz,cyx,cz7,cyt,cxr,cvz,ctf,cpz,clm,cg9,c9n,c1n,bs3,bgs,b3z,aq7,abz,9xw,9ke,97q,8w1,8lg,8c4,844,7xf,7rw,7nj,7k7,7hu,7ge,7fs,7fw,7go,7hw,7j9,7ki,7lc,7lj,7l9,7kr,7kb,7k5,7ki,7lh,7n3,7pg,7sl,7wj,81b,86w,8d9,8ke,8s9,90r,99u,9jd,9t9,a3e,adk,anm,axd,b6m,bf3,bmi,bsm,bx4,bzn,bzt,bx8,brf,bi5,b5m,aqd,a8y,9pv,95m,8kl,7z5,7dp,6sn,688,5oq,56d,4pc,49w,3w4,3k3,39t,31d,2uq,2pr,2m7,2ju,2ig,2ht,2hr,2i4,2iq,2jg,2k6,2kx,2lq,2mk,2nh,2ok,2pw,2rm,2tv,2wo,2zy,33i,376,3ar,3e4,3h6,3ju,3m1,3np,3ot,3pi,3pv,3q0,3q1,3py,3ps,3pj,3p6,3or,3o8,3nm,3mv,3m0,3l0,3jw,3ip,3hf,3g4,3es,3df,3c2,3ap,39d,382,36s,35i,348,32y,31p,30h,2za,2y4,2wz,2vw,2uw,2ty,2t3,2sa,2rl,2qz,2qf,2pz,2pm,2pc,2p5,2p2,2p3,2p8,2pi,2py,2qm,2rg,2sh,2to,2v0,2wg,2y0,2zm,319,32v,34f,35y,37f,38v,3a9,3bm,3cz,3ea,3fk,3gt,3i1,3j8,3ke,3lj,3mm,3np,3or,3pr,3qr,3rp,3sm,3tj,3ue,3v9,3w2,3wu,3xm,3yc,3z2,3zr,40e,411,41n,427,42r,43a,43s,449,44p,455,45k,45y,46b,46n,46y,478,47i,47q,47y,485,48b,48g,48l,48p,48s,48v,48x,48y,48z,48y,48x,48v,48s,48p,48k,48f,489,483,47w,47p"

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
