import sys, json, os
sys.argv = sys.argv[:1]
import line as LN
from multiprocessing import Pool
def f(w):
    a = LN.mincurv(w, 0.0)
    json.dump([round(float(x), 3) for x in a], open(os.path.join(os.path.dirname(LN.__file__), "line_w%.1f_l0.json" % w), "w"))
    return w, LN.laptime(a)
if __name__ == "__main__":
    with Pool(6) as p:
        for r in p.map(f, [6.5, 7.2, 7.4, 7.5, 7.8, 8.2]):
            print(r, flush=True)
