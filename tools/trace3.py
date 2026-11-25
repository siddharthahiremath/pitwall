import sys, os, math, random
sys.path.insert(0,'tools'); sys.path.insert(0,'.')
import harness as Hn, server as S
orig = S.Sim.step
n = [0]; L = S.L; st = {"c": 0}
def step(self, dt):
    orig(self, dt)
    n[0] += 1
    c = self.cars.get("test#1")
    if not c or c.status != "track": return
    G = c.worker.g["G"]
    near = None
    for o in self.cars.values():
        if o is c or o.status != "track": continue
        gap = (o.s - c.s) % L; gap = gap if gap < L / 2 else gap - L
        if -10 < gap < 70 and (near is None or gap < near[0]): near = (gap, o)
    coll = c.collisions != st["c"]; st["c"] = c.collisions
    if near and (n[0] % 15 == 0 or coll):
        gap, o = near
        print("%s t=%.2f s=%.0f gap=%5.1f v=%5.1f vo=%5.1f lat=%5.2f olat=%5.2f line=%5.2f bias=%5.2f thr=%.2f brk=%.2f k=%.4f" % ("COLL" if coll else "    ", self.t, c.s, gap, c.speed, o.speed, c.lat, o.lat, G["off"][G["idx"]], G["bias"], c.throttle, c.brake, G["k"][G["idx"]]))
S.Sim.step = step
seed = int(sys.argv[2]); rng = random.Random(seed); opp = "tools/out/opp62.py"
tr = [(opp, rng.uniform(1, 3), "opp"), (opp, rng.uniform(3, 5), "opp"), (opp, rng.uniform(5, 7), "opp")]
Hn.run(sys.argv[1], 5, quiet=False, max_t=500, traffic=tr, delay=rng.uniform(8, 12))
