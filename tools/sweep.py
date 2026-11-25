import itertools, json, os, sys, tempfile
from multiprocessing import Pool
D = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, D)
from make_bot import make


def job(args):
    line, kw = args
    import harness
    fd, path = tempfile.mkstemp(suffix=".py", dir=os.path.join(D, "out")); os.close(fd)
    make(line, path, **kw)
    try:
        r = harness.run(path, 3, quiet=True, max_t=260)
    finally:
        os.unlink(path)
    laps = [x[0] for x in r]
    bad = sum(x[3] + x[4] for x in r[1:])  # grass + slide seconds
    return kw, line, laps, bad, max([x[5] for x in r] or [0])


if __name__ == "__main__":
    spec = json.loads(sys.argv[1])          # {"line": [...], "PARAM": [values...]}
    lines = spec.pop("line")
    keys = list(spec)
    jobs = [(l, dict(zip(keys, vals))) for l in lines for vals in itertools.product(*[spec[k] for k in keys])]
    with Pool(min(10, os.cpu_count())) as p:
        res = p.map(job, jobs)
    res.sort(key=lambda r: r[2][1] if len(r[2]) > 1 else 999)
    for kw, line, laps, bad, ml in res:
        print("%-28s %-60s laps %s bad %.2f maxlat %.2f" % (os.path.basename(line), kw, ["%.3f" % x for x in laps], bad, ml))
