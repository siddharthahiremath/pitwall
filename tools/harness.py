"""Headless lap timer: runs one bot through the real server physics (same tick/latency model).
usage: python3 tools/harness.py bot.py [laps] [--quiet]
"""
import json, math, os, sys, threading, time
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import server as S


class FakeWorker:
    def __init__(self, path):
        self.lock = threading.Lock()
        self.latest = None
        self.error = None
        self.alive = True
        self.ms = 0.0
        self.log = []
        g = {"__name__": "bot", "__file__": path}
        exec(compile(open(path).read(), path, "exec"), g)
        self.g = g
        self.init = json.loads(json.dumps({"track": S.GEOM, "params": S.P,
                                           "tyres": {k: {"grip": v[0], "wear_rate": v[1]} for k, v in S.TYRES.items()}}))
        t0 = time.perf_counter()
        if callable(g.get("init")):
            g["init"](self.init["track"], self.init["params"])
        self.init_ms = (time.perf_counter() - t0) * 1000
        self.max_ms = 0.0
        self.pending = None
        import random
        self.rng = random.Random(int(os.environ.get('SEED', '1')))
        self.jit = float(os.environ.get('JITTER', '0'))

    def ready(self):
        if self.pending is not None:
            self.latest = self.pending
            self.pending = None
            return False
        return True

    def check_hang(self):
        pass

    def send(self, state):
        st = json.loads(json.dumps(state, separators=(",", ":")))
        st["track"] = self.init["track"]; st["params"] = self.init["params"]; st["tyres"] = self.init["tyres"]
        t0 = time.perf_counter()
        res = self.g["drive"](st) or {}
        if self.jit and self.rng.random() < self.jit:
            self.pending = res
        else:
            self.latest = res
        self.max_ms = max(self.max_ms, (time.perf_counter() - t0) * 1000)

    def stop(self):
        pass


def run(path, laps=3, quiet=False, max_t=None, traffic=(), delay=1.0):
    S.SIM.__init__()
    S.EVENTS.clear()
    car = S.Car(1, "test", 1, path, 0, "#fff")
    car.worker = FakeWorker(path)
    car.box_timer = delay
    S.SIM.cars = {car.name: car}
    nums = {"test": 1}
    for k, t in enumerate(traffic):
        tp, td = t[0], t[1]
        owner = t[2] if len(t) > 2 else "traffic%d" % k
        nums[owner] = nums.get(owner, 0) + 1
        o = S.Car(2 + k, owner, nums[owner], tp, 1 + k, "#888")
        o.worker = FakeWorker(tp)
        o.box_timer = td
        S.SIM.cars[o.name] = o
    dt = 1.0 / S.PHYS_HZ
    res = []
    last_seen = None
    grass = slide = 0.0
    maxlat = 0.0
    wall = 0
    max_t = max_t or 90 * (laps + 1)
    while S.SIM.t < max_t:
        S.SIM.step(dt)
        if car.status == "track":
            if car.surface == "grass": grass += dt
            if car.sliding: slide += dt
            maxlat = max(maxlat, abs(car.lat))
        if car.last_lap is not None and car.last_lap != last_seen:
            last_seen = car.last_lap
            res.append((car.last_lap, car.wear, car.compound, grass, slide, maxlat, car.collisions, car.damage))
            if not quiet:
                print("lap %.3f  wear %.3f %s  grass %.2fs slide %.2fs maxlat %.2f coll %d dmg %.2f" % res[-1], flush=True)
            grass = slide = 0.0; maxlat = 0.0
            if len(res) >= laps:
                break
    if not quiet:
        print("init %.0f ms, max drive %.1f ms" % (car.worker.init_ms, car.worker.max_ms))
        if not res:
            print("no lap; s=%.0f status=%s laps=%d" % (car.s, car.status, car.laps))
    return res


if __name__ == "__main__":
    p = sys.argv[1]
    n = int(sys.argv[2]) if len(sys.argv) > 2 else 3
    tr = [(x.split(":")[0], float(x.split(":")[1])) for x in sys.argv[3:]]
    run(p, n, traffic=tr, delay=float(os.environ.get("DELAY", "1.0")))
