import os, sys, random
from multiprocessing import Pool
D = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, D)
def job(a):
    path, opp, seed = a
    import harness, server as S
    rng = random.Random(seed)
    dl = [rng.uniform(1, 25) for _ in range(6)]
    tr = [(path, dl[0], "test"), (path, dl[1], "test"), (opp, dl[2], "opp"), (opp, dl[3], "opp"), (opp, dl[4], "opp")]
    harness.run(path, 999, quiet=True, max_t=float(os.environ.get("T", "400")), traffic=tr, delay=dl[5])
    cs = list(S.SIM.cars.values())
    ours = [c for c in cs if c.owner == "test"]
    return sum(c.dist for c in ours), sum(c.collisions for c in ours), min((c.best_lap or 99) for c in ours)
if __name__ == "__main__":
    opp = sys.argv[1]
    for path in sys.argv[2:]:
        with Pool(12) as p:
            res = p.map(job, [(path, opp, s) for s in range(12)])
        print("%-28s team dist %.0f  coll %d  best %.2f  per-seed %s" % (os.path.basename(path), sum(r[0] for r in res) / len(res), sum(r[1] for r in res), min(r[2] for r in res), [round(r[0] / 1000, 1) for r in res]), flush=True)
