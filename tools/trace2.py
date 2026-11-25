import sys, os, math
sys.path.insert(0,'tools'); sys.path.insert(0,'.')
import harness as Hn, server as S
orig = S.Sim.step
st = {"c": 0, "slide": False, "grass": False}
L = S.L
def step(self, dt):
    orig(self, dt)
    c = self.cars.get("test#1")
    if not c or c.status != "track": return
    G = c.worker.g["G"]
    if c.collisions != st["c"] or (c.sliding and not st["slide"]) or (c.surface == "grass" and not st["grass"]):
        kind = "COLL" if c.collisions != st["c"] else ("SLIDE" if c.sliding and not st["slide"] else "GRASS")
        st["c"] = c.collisions
        others = sorted(((math.hypot(o.x-c.x, o.y-c.y), o) for o in self.cars.values() if o is not c and o.status == "track"), key=lambda t: t[0])
        if others:
            dd, o = others[0]
            gap = (o.s - c.s) % L; gap = gap if gap < L/2 else gap - L
            print("%s t=%.1f s=%.0f v=%.1f lat=%.2f line=%.2f bias=%.2f | %s gap=%.1f v=%.1f lat=%.2f h_err=%.2f" % (kind, self.t, c.s, c.speed, c.lat, G['off'][G['idx']], G["bias"], o.name, gap, o.speed, o.lat, S.wrap_angle(o.h - S.track.hd[o.idx])))
    st["slide"] = c.sliding; st["grass"] = c.surface == "grass"
S.Sim.step = step
D='tools'
tr = [(os.path.join(D, "out", "tr_fast.py"), 1.0), ("examples/nova.py", 6.0), ("examples/blaze.py", 11.0), ("examples/comet.py", 16.0)]
Hn.run(sys.argv[1], 5, quiet=False, traffic=tr, delay=float(sys.argv[2]))
