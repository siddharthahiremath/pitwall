import os, sys
from multiprocessing import Pool
D = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, D)
def job(a):
    path, opp, delays = a
    import harness, server as S
    tr = [(path, delays[0], "test"), (path, delays[1], "test"), (opp, delays[2], "opp"), (opp, delays[3], "opp"), (opp, delays[4], "opp")]
    r = harness.run(path, 6, quiet=True, max_t=520, traffic=tr, delay=delays[5])
    cs = list(S.SIM.cars.values())
    allc = sum(c.collisions for c in cs)
    ours = [c for c in cs if c.owner == "test"]
    return delays, [round(x[0], 2) for x in r], allc, [(c.name, c.laps, round(c.dist)) for c in ours]
if __name__ == "__main__":
    path = sys.argv[1]; opp = sys.argv[2]
    sets = [(1, 3, 5, 7, 9, 11), (1, 9, 3, 13, 6, 20), (4, 8, 1, 2, 3, 12), (2, 30, 1, 15, 45, 60), (10, 20, 1, 5, 30, 40), (1, 2, 3, 4, 5, 6)]
    with Pool(6) as p:
        res = p.map(job, [(path, opp, s) for s in sets])
    tot = 0
    for d, laps, allc, ours in res:
        tot += allc
        print("delays %s allcoll %2d test laps %s ours %s" % (d, allc, laps, ours))
    print("TOTAL", tot)
