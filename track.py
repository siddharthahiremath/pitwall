"""Track geometry: a closed centripetal Catmull-Rom spline resampled at uniform spacing.

World coordinates are metres, x east, y north (y-up). Headings are radians CCW from +x.
"""
import math

HALF_WIDTH = 7.5        # asphalt half width (15 m wide track)
KERB = 1.4              # kerb width outside the white line
WALL = 22.0             # wall distance from centerline
PIT_OFFSET = -36.0      # pit fast-lane offset from centerline (negative = right side)
BOX_OFFSET = -42.0      # pit box offset
PIT_SPEED = 22.2        # pit limiter, m/s (80 km/h)
SPACING = 4.0

# Control points, driven counter-clockwise. The main straight runs east along y=0.
CONTROL = [
    (-560, 0), (-300, 0), (0, 0), (300, 0), (470, 6),
    (560, 60), (575, 160), (515, 230),
    (390, 250), (285, 300), (250, 390),
    (310, 480), (430, 540), (520, 640), (500, 760),
    (400, 830), (220, 845),
    (-100, 845), (-380, 835),
    (-520, 790), (-570, 690), (-520, 600),
    (-380, 560), (-260, 500), (-260, 410), (-380, 360),
    (-560, 330), (-700, 260), (-760, 140), (-720, 30),
]


def _catmull(points, samples=60, alpha=0.5):
    n = len(points)
    out = []
    for i in range(n):
        p0, p1, p2, p3 = points[i - 1], points[i], points[(i + 1) % n], points[(i + 2) % n]

        def tj(ti, a, b):
            return ti + max(1e-6, math.hypot(b[0] - a[0], b[1] - a[1])) ** alpha
        t0 = 0.0
        t1 = tj(t0, p0, p1)
        t2 = tj(t1, p1, p2)
        t3 = tj(t2, p2, p3)
        for k in range(samples):
            t = t1 + (t2 - t1) * k / samples

            def lerp(a, b, ta, tb):
                wa = (tb - t) / (tb - ta)
                wb = (t - ta) / (tb - ta)
                return (a[0] * wa + b[0] * wb, a[1] * wa + b[1] * wb)
            a1 = lerp(p0, p1, t0, t1)
            a2 = lerp(p1, p2, t1, t2)
            a3 = lerp(p2, p3, t2, t3)
            b1 = lerp(a1, a2, t0, t2)
            b2 = lerp(a2, a3, t1, t3)
            out.append(lerp(b1, b2, t1, t2))
    return out


def _resample(poly, spacing):
    # cumulative length of closed poly
    pts = poly + [poly[0]]
    cum = [0.0]
    for i in range(1, len(pts)):
        cum.append(cum[-1] + math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]))
    total = cum[-1]
    n = max(8, int(round(total / spacing)))
    step = total / n
    out = []
    j = 0
    for k in range(n):
        d = k * step
        while cum[j + 1] < d:
            j += 1
        seg = cum[j + 1] - cum[j]
        u = 0 if seg == 0 else (d - cum[j]) / seg
        out.append((pts[j][0] + (pts[j + 1][0] - pts[j][0]) * u,
                    pts[j][1] + (pts[j + 1][1] - pts[j][1]) * u))
    return out, total


def _smoothstep(u):
    u = max(0.0, min(1.0, u))
    return u * u * (3 - 2 * u)


class Track:
    def __init__(self):
        raw = _catmull(CONTROL)
        pts, _ = _resample(raw, SPACING)
        # rotate so index 0 (start/finish line) sits at x≈0 on the main straight
        i0 = min(range(len(pts)), key=lambda i: math.hypot(pts[i][0], pts[i][1]))
        pts = pts[i0:] + pts[:i0]
        self.pts = pts
        n = self.n = len(pts)
        self.ds = []
        self.s = [0.0]
        for i in range(n):
            a, b = pts[i], pts[(i + 1) % n]
            d = math.hypot(b[0] - a[0], b[1] - a[1])
            self.ds.append(d)
            if i < n - 1:
                self.s.append(self.s[-1] + d)
        self.length = sum(self.ds)
        # headings and normals (central differences)
        self.hd = []
        self.nx = []
        self.ny = []
        for i in range(n):
            a, b = pts[i - 1], pts[(i + 1) % n]
            h = math.atan2(b[1] - a[1], b[0] - a[0])
            self.hd.append(h)
            self.nx.append(-math.sin(h))   # left normal
            self.ny.append(math.cos(h))
        # curvature (signed, + = left turn), smoothed
        raw_k = []
        for i in range(n):
            dh = (self.hd[(i + 1) % n] - self.hd[i - 1] + math.pi) % (2 * math.pi) - math.pi
            raw_k.append(dh / (2 * SPACING))
        k = raw_k
        for _ in range(3):
            k = [(k[i - 2] + 2 * k[i - 1] + 3 * k[i] + 2 * k[(i + 1) % n] + k[(i + 2) % n]) / 9 for i in range(n)]
        self.k = k

        def off(o):
            return [(pts[i][0] + self.nx[i] * o, pts[i][1] + self.ny[i] * o) for i in range(n)]
        self.left = off(HALF_WIDTH)
        self.right = off(-HALF_WIDTH)
        self.wall_left = off(WALL)
        self.wall_right = off(-WALL)

        # pit lane: offset function of s, entry before the line, exit after it
        L = self.length
        self.pit_entry_s = L - 400.0
        self.pit_exit_s = 250.0
        self.pit_ramp = 95.0
        self.pit_len = (L - self.pit_entry_s) + self.pit_exit_s
        self.box_spacing = 14.0
        self.box_start_u = 120.0   # pit-path distance of first box
        self.max_boxes = 20

    # ---- helpers ----
    def wrap(self, s):
        return s % self.length

    def idx_at(self, s):
        s = self.wrap(s)
        # uniform-ish spacing
        i = int(s / self.length * self.n) % self.n
        while self.s[i] > s and i > 0:
            i -= 1
        while i < self.n - 1 and self.s[i + 1] <= s:
            i += 1
        return i

    def point_at(self, s, offset=0.0):
        """Point on centerline at arc length s, offset to the left. Returns x, y, heading."""
        s = self.wrap(s)
        i = self.idx_at(s)
        j = (i + 1) % self.n
        u = (s - self.s[i]) / self.ds[i] if self.ds[i] else 0
        x = self.pts[i][0] + (self.pts[j][0] - self.pts[i][0]) * u
        y = self.pts[i][1] + (self.pts[j][1] - self.pts[i][1]) * u
        nx = self.nx[i] + (self.nx[j] - self.nx[i]) * u
        ny = self.ny[i] + (self.ny[j] - self.ny[i]) * u
        nl = math.hypot(nx, ny) or 1
        h = math.atan2(-nx, ny)
        return x + nx / nl * offset, y + ny / nl * offset, h

    def curvature_at(self, s):
        return self.k[self.idx_at(s)]

    def project(self, x, y, hint, window=20):
        """Project point onto centerline near index hint. Returns (idx, s, lateral offset(+left))."""
        n = self.n
        best = None
        bi = hint
        for d in range(-window, window + 1):
            i = (hint + d) % n
            px, py = self.pts[i]
            dd = (x - px) ** 2 + (y - py) ** 2
            if best is None or dd < best:
                best = dd
                bi = i
        # refine on segment bi->bi+1 or bi-1->bi
        best_res = None
        for i in (bi - 1, bi):
            i %= n
            j = (i + 1) % n
            ax, ay = self.pts[i]
            bx, by = self.pts[j]
            ex, ey = bx - ax, by - ay
            L2 = ex * ex + ey * ey or 1
            u = max(0.0, min(1.0, ((x - ax) * ex + (y - ay) * ey) / L2))
            qx, qy = ax + ex * u, ay + ey * u
            dd = (x - qx) ** 2 + (y - qy) ** 2
            if best_res is None or dd < best_res[0]:
                seglen = math.sqrt(L2)
                lat = (ex * (y - ay) - ey * (x - ax)) / seglen
                best_res = (dd, i, self.s[i] + u * self.ds[i], lat)
        _, i, s, lat = best_res
        return i, self.wrap(s), lat

    def global_project(self, x, y):
        best = min(range(self.n), key=lambda i: (x - self.pts[i][0]) ** 2 + (y - self.pts[i][1]) ** 2)
        return self.project(x, y, best, 3)

    # ---- pit lane (parameterised by pit-path distance u from the entry) ----
    def pit_s(self, u):
        return self.wrap(self.pit_entry_s + u)

    def pit_offset(self, u, box_u=None):
        r = self.pit_ramp
        o = PIT_OFFSET * _smoothstep(u / r) * _smoothstep((self.pit_len - u) / r)
        if box_u is not None:
            # pull into the box lane around the box location
            w = _smoothstep(1 - abs(u - box_u) / 22.0) if abs(u - box_u) < 22 else 0.0
            o += (BOX_OFFSET - PIT_OFFSET) * w
        return o

    def box_u(self, slot):
        return self.box_start_u + slot * self.box_spacing

    def pit_pose(self, u, box_u=None, blend_from=None):
        o = self.pit_offset(u, box_u)
        if blend_from is not None:
            o += blend_from * (1 - _smoothstep(u / 70.0))
        return self.point_at(self.pit_s(u), o)

    def geometry(self):
        """Static data sent to the browser and to bots."""
        r = lambda p: [round(p[0], 2), round(p[1], 2)]
        pit_path = []
        u = 0.0
        while u <= self.pit_len:
            x, y, _ = self.pit_pose(u)
            pit_path.append([round(x, 2), round(y, 2)])
            u += 4.0
        boxes = []
        for slot in range(self.max_boxes):
            bu = self.box_u(slot)
            x, y, h = self.point_at(self.pit_s(bu), BOX_OFFSET)
            boxes.append({"slot": slot, "x": round(x, 2), "y": round(y, 2), "heading": round(h, 4),
                          "u": bu, "s": round(self.pit_s(bu), 2)})
        return {
            "length": round(self.length, 2),
            "spacing": SPACING,
            "half_width": HALF_WIDTH,
            "kerb": KERB,
            "wall": WALL,
            "centerline": [r(p) for p in self.pts],
            "s": [round(v, 2) for v in self.s],
            "heading": [round(v, 5) for v in self.hd],
            "curvature": [round(v, 6) for v in self.k],
            "left": [r(p) for p in self.left],
            "right": [r(p) for p in self.right],
            "wall_left": [r(p) for p in self.wall_left],
            "wall_right": [r(p) for p in self.wall_right],
            "pit": {
                "entry_s": round(self.pit_entry_s, 2),
                "exit_s": round(self.pit_exit_s, 2),
                "length": round(self.pit_len, 2),
                "lane_offset": PIT_OFFSET,
                "box_offset": BOX_OFFSET,
                "speed_limit": PIT_SPEED,
                "path": pit_path,
                "boxes": boxes,
            },
            "sectors": [0.0, round(self.length / 3, 2), round(2 * self.length / 3, 2)],
        }


if __name__ == "__main__":
    t = Track()
    print("length", round(t.length), "points", t.n)
    kmax = max(abs(k) for k in t.k)
    print("min radius", round(1 / kmax, 1))
    # clearance between non-adjacent parts of the track
    worst = (1e9, None)
    for i in range(0, t.n, 2):
        for j in range(0, t.n, 2):
            sep = abs(t.s[i] - t.s[j])
            sep = min(sep, t.length - sep)
            if sep < 250:
                continue
            d = math.hypot(t.pts[i][0] - t.pts[j][0], t.pts[i][1] - t.pts[j][1])
            if d < worst[0]:
                worst = (d, (i, j, t.pts[i], t.pts[j]))
    print("min clearance between distinct parts", round(worst[0], 1), worst[1])
    # radius report per corner
    tight = [(round(t.s[i]), round(1 / abs(t.k[i]), 1)) for i in range(t.n) if abs(t.k[i]) > 1 / 60]
    print("tight spots", tight[::5])
    # pit lane clearance from other track parts
    worst = 1e9
    for u in range(0, int(t.pit_len), 4):
        x, y, _ = t.pit_pose(u)
        for j in range(t.n):
            sep = abs(t.s[j] - t.pit_s(u))
            sep = min(sep, t.length - sep)
            if sep < 200:
                continue
            worst = min(worst, math.hypot(x - t.pts[j][0], y - t.pts[j][1]))
    print("pit clearance", round(worst, 1))
