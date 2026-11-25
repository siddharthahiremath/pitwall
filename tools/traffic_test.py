import os, sys, json
from multiprocessing import Pool
D = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, D)
R = os.path.join(D, "..")
def job(a):
    path, delay, seed = a
    os.environ["SEED"] = str(seed)
    import harness
    tr = [(os.path.join(D, "out", "tr_fast.py"), 1.0), (os.path.join(R, "examples/nova.py"), 6.0), (os.path.join(R, "examples/blaze.py"), 11.0), (os.path.join(R, "examples/comet.py"), 16.0)]
    r = harness.run(path, 6, quiet=True, max_t=520, traffic=tr, delay=delay)
    return delay, [round(x[0], 2) for x in r], (r[-1][6] if r else -1), sum(x[3] for x in r)
if __name__ == "__main__":
    from make_bot import make
    make(os.path.join(D, "line_w745.json"), os.path.join(D, "out", "tr_fast.py"), GRIP_USE=0.9)
    path = sys.argv[1]
    with Pool(8) as p:
        res = p.map(job, [(path, d, 1) for d in (3, 9, 15, 21, 27, 33, 40, 47)])
    tot = 0
    for d, laps, coll, grass in res:
        tot += max(coll, 0)
        print("delay %2d coll %2d grass %5.1fs laps %s" % (d, coll, grass, laps))
    print("TOTAL collisions", tot, " best", min(min(r[1][1:] or [99]) for r in res))
