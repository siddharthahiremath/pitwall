"""Our car vs N slower opponents; reports time lost to traffic = (race time for K laps) - solo time."""
import os, sys, random
from multiprocessing import Pool
D = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, D)
def job(a):
    path, opp, seed = a
    import harness, server as S
    rng = random.Random(seed)
    tr = [(opp, rng.uniform(1, 3), "opp"), (opp, rng.uniform(3, 5), "opp"), (opp, rng.uniform(5, 7), "opp")]
    r = harness.run(path, 5, quiet=True, max_t=500, traffic=tr, delay=rng.uniform(8, 12))
    me = S.SIM.cars["test#1"]
    return sum(x[0] for x in r[1:5]), me.collisions, [round(x[0], 1) for x in r]
if __name__ == "__main__":
    opp = sys.argv[1]
    for path in sys.argv[2:]:
        import harness
        solo = harness.run(path, 5, quiet=True)
        st = sum(x[0] for x in solo[1:5])
        with Pool(12) as p:
            res = p.map(job, [(path, opp, s) for s in range(12)])
        lost = [r[0] - st for r in res]
        print("%-22s lost/4laps avg %.1fs  coll %d  %s" % (os.path.basename(path), sum(lost) / len(lost), sum(r[1] for r in res), [round(x, 1) for x in lost]), flush=True)
