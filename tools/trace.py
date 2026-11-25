import sys, os, math
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__))); sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import harness as Hn, server as S
orig = S.Sim.step
state = {"slide": False, "lastcoll": 0}
def step(self, dt):
    orig(self, dt)
    c = self.cars.get("test#1")
    if not c: return
    G = c.worker.g["G"]
    if c.collisions != state["lastcoll"]:
        state["lastcoll"] = c.collisions
        near = sorted(((math.hypot(o.x-c.x, o.y-c.y), o) for o in self.cars.values() if o is not c), key=lambda t: t[0])[0]
        o = near[1]
        print("COLL t=%.1f s=%.0f v=%.1f lat=%.2f bias=%.2f | other %s d=%.1f v=%.1f lat=%.2f ghost=%s/%s st=%s late=%.3f" % (self.t, c.s, c.speed, c.lat, G["bias"], o.name, near[0], o.speed, o.lat, c.ghost, o.ghost, o.status, G["late"]))
    if c.sliding and not state["slide"]:
        print("SLIDE t=%.1f s=%.0f v=%.1f lat=%.2f bias=%.2f late=%.3f" % (self.t, c.s, c.speed, c.lat, G["bias"], G["late"]))
    state["slide"] = c.sliding
S.Sim.step = step
tr = [(x.split(":")[0], float(x.split(":")[1])) for x in sys.argv[3:]]
Hn.run(sys.argv[1], int(sys.argv[2]), traffic=tr, delay=float(os.environ.get("DELAY", "1")))
