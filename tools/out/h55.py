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
ALONG_GAP, ALONG_LAT, ALONG_REL = 0.0, 4.5, 3.0
GUARD_A, GUARD_DO, GUARD_MIN = 20.0, 12.0, 8.0   # following guard (lower = more aggressive)         # lateral centre-to-centre clearance when passing (m)
PIT_WEAR = 0.55     # pit when tyre wear passes this (0..1)

LINE_N = 1183
LINE = "47j,47c,476,470,46v,46q,46m,46i,46f,46d,46b,46a,469,468,468,468,467,467,467,466,466,465,464,463,461,460,45y,45w,45t,45q,45m,45i,45d,457,451,44u,44m,44d,443,43s,43g,433,42p,42a,41t,41c,40t,408,3zm,3yz,3ya,3xj,3wr,3vx,3v1,3u3,3t3,3s1,3qw,3pp,3og,3n4,3lq,3k9,3iq,3h3,3fe,3dl,3bp,39o,37j,35a,32v,30f,2y2,2vv,2ty,2se,2ra,2qq,2qq,2rd,2sl,2ug,2ww,2zy,33j,37n,3c9,3ha,3mr,3sm,3yt,45b,4c2,4iz,4q1,4x2,53y,5an,5gy,5ms,5s2,5wn,60i,63m,661,67r,68u,69f,69l,69f,690,68b,679,65x,64d,62v,61o,60z,610,61x,63y,678,6bq,6hc,6nx,6vf,73r,7d2,7nb,7yj,8ai,8n0,8zp,9cd,9ot,a0w,acj,anj,axg,b5t,bc5,bg3,bhn,bgz,be7,b9k,b3k,aws,apr,aj0,acq,a75,a2h,9yx,9wk,9vg,9vo,9xa,a07,a4d,a9p,ag3,anc,av8,b3k,bc0,bjx,bqr,bvw,bz5,c0o,c0r,bzn,bxk,bur,brg,bnw,bk8,bgl,bd3,b9t,b6u,b46,b1v,azx,ayc,ax5,awb,avs,auz,at2,ap5,aig,a8r,9w2,9ge,8xw,8d6,7r4,74i,6i7,5wn,5c7,4t8,4c4,3x0,3k5,39p,31q,2vt,2rg,2o4,2lg,2jm,2iw,2jl,2lx,2q1,2w1,341,3e0,3ps,432,4hn,4x7,5d9,5tf,696,6o5,71x,7e6,7oi,7wq,82u,86w,88y,892,87c,83z,7z2,7ss,7l8,7ck,72x,6si,6hj,668,5ut,5js,59p,51a,4v1,4r0,4op,4no,4nh,4nr,4o7,4ok,4oi,4ny,4mw,4l9,4j1,4g8,4cs,48n,43v,3yh,3sm,3mf,3g0,39o,33s,2yn,2uo,2se,2sf,2vc,31k,3ar,3mg,405,4fb,4vk,5cg,5ti,6ae,6qu,76m,7li,7zb,8bv,8n2,8wr,94u,9b9,9g0,9j0,9k8,9jj,9gv,9c4,95g,8xj,8p0,8gj,88p,81x,7wp,7tg,7s9,7su,7uw,7y4,82a,87a,8cz,8j8,8px,8wt,93p,9ad,9gh,9ll,9p8,9r0,9qq,9oc,9jr,9cy,942,8te,8h6,83m,7oy,79c,6sy,6c0,5uw,5e4,4y5,4jf,46a,3v1,3lz,3fb,3b1,392,39b,3bo,3g4,3mk,3ux,454,4h0,4uf,597,5p7,66e,6oo,780,7s8,8d3,8y8,9jb,a40,anz,b6w,boe,c45,chr,csy,d1b,d6v,da3,dbg,dbh,dai,d8q,d6d,d3m,d0n,cxk,cuh,crk,cou,cmf,ckb,cik,ch8,cga,cfq,cfk,cfs,cgf,chh,ciw,ckn,cmk,cog,cq7,cri,cs2,crj,cpn,cmf,ci0,cck,c67,bz5,brj,bjl,bbg,b39,av5,an8,afn,a8g,a1u,9vv,9qo,9ma,9is,9g6,9ei,9ds,9e2,9fc,9ho,9l6,9pz,9w8,a3u,ace,ale,aub,b2k,b9h,bea,bgb,bfb,bbu,b6i,azw,as9,aji,a9e,9xs,9ku,971,8su,8es,810,7nj,7a9,6x4,6k5,67f,5v0,5iz,57h,4wl,4mg,4d6,44t,3xa,3qk,3kl,3fc,3aq,36p,335,304,2xj,2vh,2tw,2st,2s7,2s3,2sf,2t6,2u7,2vg,2wt,2y7,2zk,30t,31w,32t,33l,348,34s,359,35q,367,36o,377,37q,389,38s,399,39p,3a4,3ag,3ar,3b0,3b9,3bg,3bn,3bu,3c1,3c7,3ce,3ck,3cq,3cv,3d0,3d5,3d8,3da,3db,3db,3db,3da,3d9,3d7,3d5,3d2,3cy,3cu,3cp,3cj,3cc,3c5,3bw,3bm,3bb,3az,3al,3a7,39r,39b,38u,38c,37t,379,36n,360,35b,34k,33s,32y,322,315,306,2z6,2y5,2x3,2w0,2uy,2tw,2sw,2ry,2r4,2qe,2pt,2pd,2p3,2ow,2ot,2os,2os,2ou,2ow,2oz,2p1,2p3,2p5,2p7,2p9,2pc,2pf,2pj,2pn,2ps,2px,2q1,2q6,2qb,2qg,2ql,2qp,2qt,2qv,2qw,2qw,2qv,2qs,2qp,2ql,2qf,2q6,2pw,2pj,2p6,2or,2oe,2o4,2o1,2o6,2oo,2pl,2r1,2t3,2vp,2yu,32f,36d,3an,3f6,3jy,3ow,3tx,3z0,442,492,4dw,4ik,4n0,4r6,4v3,4yt,52d,55s,593,5cb,5fg,5il,5lm,5ok,5rd,5u0,5wc,5yb,5zv,611,627,63r,664,69k,6e3,6jq,6qj,6yf,779,7gv,7r4,81u,8cs,8nt,8yo,995,9iy,9rs,9ze,a5m,aai,ae2,agb,ahg,ahu,ahs,aho,ahm,aht,ai9,aj4,akf,ama,aop,arr,avg,azt,b4s,bae,bgj,bn5,bu2,c18,c8g,cfj,cma,csg,cxm,d19,d2x,d28,czd,cuk,co4,cga,c7h,by4,bok,bf6,b62,axb,ap1,ahb,aa8,a3w,9yd,9tp,9pu,9mq,9kb,9ij,9hb,9gm,9ge,9ga,9fg,9d4,98f,911,8r4,8ex,80n,7kq,73t,6mj,65k,5pb,5a4,4wc,4k9,4a4,422,3wa,3sx,3ru,3ss,3vf,3zh,44e,49i,4e8,4hx,4kp,4n1,4pb,4s0,4vs,51c,59e,5kk,5yp,6fg,6yd,7j3,84x,8r3,9cv,9xj,agn,axo,bc9,bo0,bwu,c2s,c5r,c5u,c37,by1,bqj,bgw,b59,arq,ach,9vo,9dk,8ue,8ag,7py,750,6jo,5y5,5cv,4t1,4bw,3yn,3px,3ln,3ln,3pr,3xe,47b,4ic,4t9,53b,5c6,5jg,5ou,5s6,5td,5se,5p7,5k0,5d5,550,4w0,4mq,4du,462,405,3wy,3xd,42c,4cn,4rv,5az,5wz,6kx,79z,7zf,8oj,9cl,9z9,aka,b3g,bkg,bz4,cb9,cko,cra,cve,cxh,cxy,cx9,cvp,ctn,crg,cpa,cn2,ckm,chr,cef,cal,c69,c1h,bwd,br3,bly,bh6,bcz,b9j,b6y,b5e,b4x,b5d,b6o,b8o,bb8,be6,bha,bke,bnf,bqd,bt6,bvw,byi,c0y,c38,c59,c74,c8v,caj,cc7,cdx,cfr,chq,cjy,cma,com,cqt,csq,cu8,cv5,cvc,cup,cta,cr7,cok,cll,cig,cfh,ccz,cb5,c9z,c9g,c9e,c9s,cak,cbn,ccz,cej,cg4,chk,cip,cjd,cji,ciz,chq,cfs,cda,cai,c7p,c53,c31,c1t,c1s,c32,c5e,c8j,cc7,cg4,ck3,cnw,crb,cu7,cwh,cy5,cz6,czh,cz2,cxy,cw4,cth,cpx,cld,cfn,c8m,c02,bps,bdm,azw,alh,a76,9tq,9hh,96e,8wg,8nl,8fu,894,83d,7ym,7us,7ru,7pp,7oc,7no,7nm,7o3,7oz,7q1,7qy,7rf,7r8,7ql,7pr,7p2,7or,7p1,7px,7rk,7u1,7xe,81m,86q,8cr,8jm,8ra,8zq,98u,9ij,9sp,a38,ae0,aot,azg,b9r,bjh,bsf,c0b,c6w,cbu,cem,ceq,cbn,c55,bve,bij,b2s,aki,a0c,9et,8sj,860,7ji,6xc,6bt,5r8,57s,4pr,49c,3up,3hz,379,2ym,2rz,2n5,2jv,2hy,2h3,2h1,2hf,2i0,2ip,2jf,2k6,2kx,2lp,2mj,2nh,2ok,2px,2rn,2tv,2wo,2zy,33i,376,3ar,3e5,3h9,3jw,3m0,3nn,3os,3pk,3pz,3q4,3q2,3px,3pr,3pj,3p8,3ot,3oa,3nn,3mw,3m0,3l0,3jw,3ip,3hg,3g5,3es,3df,3c2,3ap,39e,382,36s,35i,348,32z,31q,30h,2za,2y4,2wz,2vx,2uw,2ty,2t3,2sa,2rl,2qy,2qf,2pz,2pm,2pc,2p5,2p2,2p2,2p8,2pi,2pz,2qm,2rg,2sh,2to,2v0,2wg,2y0,2zm,319,32v,34f,35y,37f,38v,3a9,3bm,3cz,3ea,3fk,3gt,3i1,3j8,3ke,3lj,3mm,3np,3or,3pr,3qr,3rp,3sm,3tj,3ue,3v8,3w2,3wu,3xm,3yc,3z2,3zr,40e,411,41n,427,42r,43a,43s,449,44p,455,45k,45y,46b,46n,46y,478,47i,47q,47y,485,48b,48g,48l,48p,48s,48v,48x,48y,48y,48y,48x,48v,48s,48p,48k,48f,489,483,47w,47p"

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
            tgt = min(cands, key=lambda t: abs(t - my_lat) + 0.5 * abs(t - line_there))
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
