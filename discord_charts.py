#!/usr/bin/env python3
"""PITWALL -> Discord: every run pulls the latest race recording from the server, draws one chart
(rotating through KINDS) and posts it to the #pitwall channel as an embed.

  python3 discord_charts.py            # post the next chart in the rotation
  python3 discord_charts.py best_lap   # post a specific chart
  python3 discord_charts.py --dry best_lap   # just render to data/charts/
  python3 discord_charts.py --loop 1800      # post every 30 min

Secrets (gitignored): data/DISCORD_TOKEN.local.txt, data/EXPORT_TOKEN.local.txt."""
import io, json, os, re, statistics, sys, tarfile, time, urllib.request, uuid
from datetime import datetime
from zoneinfo import ZoneInfo

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter

ROOT = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(ROOT, "data")
OUT = os.path.join(DATA, "charts")
STATE = os.path.join(DATA, "discord_state.json")
CHANNEL = os.environ.get("PITWALL_DISCORD_CHANNEL", "1575025174103724092")   # #pitwall
SITE = "https://pitwall-three-gamma.vercel.app"
TZ = ZoneInfo("America/Los_Angeles")
UA = "pitwall-charts/1.0"

BG, PANEL, GRID, TXT, SUB = "#0b0e13", "#11151c", "#222935", "#e8ecf2", "#8a93a6"


def read(p):
    with open(p) as f:
        return f.read().strip()


def api_base():
    return re.search(r'PITWALL_API\s*=\s*"([^"]+)"', read(os.path.join(ROOT, "static", "config.js"))).group(1)


# ---------------------------------------------------------------- data
def fetch():
    """events + car identities, straight from the race server (falls back to the local mirror)."""
    files = {}
    try:
        req = urllib.request.Request(api_base() + "/api/admin/export",
                                     headers={"Authorization": "Bearer " + read(os.path.join(DATA, "EXPORT_TOKEN.local.txt")), "User-Agent": UA})
        with urllib.request.urlopen(req, timeout=120) as r:
            with tarfile.open(fileobj=r, mode="r|") as tar:
                for m in tar:
                    if m.name in ("replay/events.jsonl", "replay/cars.jsonl"):
                        files[m.name] = tar.extractfile(m).read().decode()
    except Exception as e:
        print("fetch failed, using local mirror:", e)
    for name in ("replay/events.jsonl", "replay/cars.jsonl"):
        if name not in files:
            files[name] = open(os.path.join(DATA, "export", "live", name)).read()
    events = [json.loads(l) for l in files["replay/events.jsonl"].splitlines() if l.strip()]
    cars = {}
    for l in files["replay/cars.jsonl"].splitlines():
        if l.strip():
            for c in json.loads(l)["cars"].values():
                cars[c["name"]] = c
    return events, cars


LAP_RE = re.compile(r"^(\S+) lap (\d+) — (\d+):(\d+\.\d+)")


def race(events, cars):
    """Everything since the most recent lights-out."""
    starts = [e for e in events if e["kind"] == "start"]
    t0 = starts[-1]["w"] if starts else events[0]["w"]
    laps, reloads, pits, crashes = [], [], [], []
    for e in events:
        if e["w"] < t0:
            continue
        if e["kind"] == "lap":
            m = LAP_RE.match(e["text"])
            if m:
                name = m.group(1)
                laps.append({"w": e["w"], "car": name, "owner": name.split("#")[0], "lap": int(m.group(2)),
                             "t": int(m.group(3)) * 60 + float(m.group(4))})
        elif e["kind"] == "reload":
            reloads.append({"w": e["w"], "owner": e["text"].split(":")[0]})
        elif e["kind"] == "pit" and "pit stop" in e["text"]:
            pits.append({"w": e["w"], "owner": e["text"].split("#")[0]})
        elif e["kind"] == "crash":
            crashes.append({"w": e["w"], "owner": e["text"].split("#")[0]})
    color = {}
    for c in cars.values():
        color.setdefault(c["owner"], c["color"])
    quali = {}
    for e in events:
        if e["kind"] == "quali" and e["w"] < t0:
            m = re.match(r"QUALI (\S+): (\d+):(\d+\.\d+)", e["text"])
            if m:
                quali[m.group(1).split("#")[0]] = int(m.group(2)) * 60 + float(m.group(3))
    return {"t0": t0, "laps": laps, "reloads": reloads, "pits": pits, "crashes": crashes, "color": color,
            "owners": sorted({l["owner"] for l in laps}), "quali": quali, "now": time.time()}


def clean(laps):
    """Drop pit / out / crash laps: anything > 12% slower than that player's median."""
    by = {}
    for l in laps:
        by.setdefault(l["owner"], []).append(l["t"])
    med = {o: statistics.median(v) for o, v in by.items()}
    return [l for l in laps if l["t"] <= med[l["owner"]] * 1.12]


def fmt(t):
    return "%d:%06.3f" % (t // 60, t % 60)


def clock(w):
    return datetime.fromtimestamp(w, TZ).strftime("%-I:%M %p").lower()


# ---------------------------------------------------------------- styling
def figure(title, subtitle):
    plt.rcParams.update({"font.family": ["Helvetica Neue", "Arial", "DejaVu Sans"], "font.size": 11})
    fig, ax = plt.subplots(figsize=(12, 6.6), dpi=150)
    fig.patch.set_facecolor(BG)
    ax.set_facecolor(PANEL)
    for s in ax.spines.values():
        s.set_visible(False)
    ax.grid(True, color=GRID, lw=0.8)
    ax.set_axisbelow(True)
    ax.tick_params(colors=SUB, length=0, pad=6)
    fig.text(0.055, 0.945, title, color=TXT, fontsize=19, fontweight="bold", ha="left", va="top")
    fig.text(0.055, 0.893, subtitle, color=SUB, fontsize=11.5, ha="left", va="top")
    fig.text(0.055, 0.03, "PITWALL · bot racing", color=SUB, fontsize=9.5, ha="left", fontweight="bold")
    fig.text(0.945, 0.03, "updated " + clock(time.time()), color=SUB, fontsize=9.5, ha="right")
    fig.subplots_adjust(left=0.08, right=0.845, top=0.82, bottom=0.12)
    return fig, ax


def end_labels(ax, items, fmtv):
    """Label each line at its right end; nudge apart so labels never overlap."""
    if not items:
        return
    lo, hi = ax.get_ylim()
    gap = (hi - lo) * 0.075
    items = sorted(items, key=lambda i: i[1], reverse=ax.yaxis_inverted())
    ys, placed = [i[1] for i in items], []
    for y in ys:
        if placed and abs(y - placed[-1]) < gap:
            y = placed[-1] + (gap if not ax.yaxis_inverted() else -gap) * (1 if y >= placed[-1] or ax.yaxis_inverted() else 1)
        placed.append(y)
    x1 = ax.get_xlim()[1]
    for (owner, y, col, extra), yl in zip(items, placed):
        ax.annotate("%s  %s" % (owner, fmtv(y if extra is None else extra)), xy=(x1, y), xytext=(x1 + (x1 - ax.get_xlim()[0]) * 0.012, yl),
                    color=col, fontsize=11, fontweight="bold", va="center", annotation_clip=False,
                    arrowprops=dict(arrowstyle="-", color=col, lw=0.8, alpha=0.5) if abs(yl - y) > gap * 0.3 else None)


def minutes_axis(ax, R):
    ax.xaxis.set_major_formatter(FuncFormatter(lambda m, _: clock(R["t0"] + m * 60)))
    ax.set_xlabel("time (PT)  ·  lights out " + clock(R["t0"]), color=SUB, labelpad=8)


# ---------------------------------------------------------------- charts
def best_lap(R):
    laps = clean(R["laps"])
    fig, ax = figure("Fastest lap so far, by player",
                     "every dot is a lap  ·  line = each player's best (any of their 3 cars)  ·  ▲ = script update")
    lines, notes = [], {}
    span = (R["now"] - R["t0"]) / 60
    for o in R["owners"]:
        mine = sorted((l for l in laps if l["owner"] == o), key=lambda l: l["w"])
        if not mine:
            continue
        col = R["color"].get(o, "#888")
        xs = [(l["w"] - R["t0"]) / 60 for l in mine]
        ax.scatter(xs, [l["t"] for l in mine], s=9, color=col, alpha=0.32, lw=0)
        bx, by, best = [], [], 1e9
        for l, x in zip(mine, xs):
            if l["t"] < best:
                if bx:
                    bx.append(x); by.append(best)
                best = l["t"]
                bx.append(x); by.append(best)
        bx.append(span); by.append(best)
        ax.plot(bx, by, color=col, lw=2.6, solid_capstyle="round", zorder=3)
        ax.scatter([bx[-2]], [best], s=46, color=col, zorder=4, edgecolor=BG, lw=1.5)
        lines.append((o, best, col, None))
        early = [l["t"] for l in mine if l["w"] - R["t0"] < 600] or [mine[0]["t"]]   # best in the first 10 min
        notes[o] = {"best": best, "at": mine[[l["t"] for l in mine].index(best)]["w"], "gain": min(early) - best,
                    "car": mine[[l["t"] for l in mine].index(best)]["car"]}
    allb = [b for _, b, _, _ in lines]
    lo = min(allb) - 0.5
    hi = max(allb) + 2.6
    ax.set_ylim(lo, hi)
    ax.set_xlim(0, max(span, 1))
    for r in R["reloads"]:
        ax.scatter([(r["w"] - R["t0"]) / 60], [lo + (hi - lo) * 0.025], marker="^", s=70,
                   color=R["color"].get(r["owner"], "#888"), zorder=5, edgecolor=BG, lw=1)
    ax.yaxis.set_major_formatter(FuncFormatter(lambda v, _: "%d:%04.1f" % (v // 60, v % 60)))
    minutes_axis(ax, R)
    end_labels(ax, lines, fmt)
    rank = sorted(notes.items(), key=lambda kv: kv[1]["best"])
    lead, nl = rank[0]
    desc = ["**%s** holds fastest lap: **%s** (%s, %s)" % (lead, fmt(nl["best"]), nl["car"], clock(nl["at"]))]
    for o, n in rank[1:]:
        desc.append("%s  %s  (+%.3f)" % (o, fmt(n["best"]), n["best"] - nl["best"]))
    mover = max(notes.items(), key=lambda kv: kv[1]["gain"])
    if mover[1]["gain"] > 0.05:
        desc.append("\nBiggest improvement: **%s**, %.3fs quicker than their best from the first 10 min" % (mover[0], mover[1]["gain"]))
    q = R["quali"]
    beat = [o for o, n in notes.items() if o in q and n["best"] < q[o]]
    if beat:
        desc.append("Faster than their own quali lap: " + ", ".join("%s (%s → %s)" % (o, fmt(q[o]), fmt(notes[o]["best"])) for o in beat))
    return fig, "Fastest lap over time", "\n".join(desc)


def pace(R):
    laps = clean(R["laps"])
    fig, ax = figure("Race pace: rolling median lap time", "median of each player's last 25 clean laps (all 3 cars)  ·  lower is faster")
    span = (R["now"] - R["t0"]) / 60
    items, now_p = [], {}
    for o in R["owners"]:
        mine = sorted((l for l in laps if l["owner"] == o), key=lambda l: l["w"])
        if len(mine) < 10:
            continue
        col = R["color"].get(o, "#888")
        xs, ys = [], []
        for i in range(9, len(mine)):
            xs.append((mine[i]["w"] - R["t0"]) / 60)
            ys.append(statistics.median(l["t"] for l in mine[max(0, i - 24):i + 1]))
        ax.plot(xs, ys, color=col, lw=2.4)
        items.append((o, ys[-1], col, None))
        now_p[o] = ys[-1]
    ax.set_xlim(0, max(span, 1))
    ax.yaxis.set_major_formatter(FuncFormatter(lambda v, _: fmt(v)))
    minutes_axis(ax, R)
    end_labels(ax, items, fmt)
    rank = sorted(now_p.items(), key=lambda kv: kv[1])
    desc = "Current race pace:\n" + "\n".join("%s **%s** %s" % (["🥇", "🥈", "🥉"][i] if i < 3 else "▫️", o, fmt(t)) for i, (o, t) in enumerate(rank))
    return fig, "Race pace", desc


def laps_done(R):
    fig, ax = figure("Laps completed", "total laps by each player's 3 cars combined")
    span = (R["now"] - R["t0"]) / 60
    items, tot = [], {}
    for o in R["owners"]:
        mine = sorted((l for l in R["laps"] if l["owner"] == o), key=lambda l: l["w"])
        col = R["color"].get(o, "#888")
        xs = [0] + [(l["w"] - R["t0"]) / 60 for l in mine] + [span]
        ys = [0] + list(range(1, len(mine) + 1)) + [len(mine)]
        ax.step(xs, ys, where="post", color=col, lw=2.4)
        items.append((o, len(mine), col, None))
        tot[o] = len(mine)
    ax.set_xlim(0, max(span, 1))
    minutes_axis(ax, R)
    end_labels(ax, items, lambda v: "%d" % v)
    rank = sorted(tot.items(), key=lambda kv: -kv[1])
    desc = "\n".join("**%s** %d laps%s" % (o, n, "" if i == 0 else "  (−%d)" % (rank[0][1] - n)) for i, (o, n) in enumerate(rank))
    return fig, "Laps completed", desc


def incidents(R):
    fig, ax = figure("Crashes & pit stops", "per player, since lights out")
    owners = R["owners"]
    cr = [sum(1 for c in R["crashes"] if c["owner"] == o) for o in owners]
    pt = [sum(1 for p in R["pits"] if p["owner"] == o) for o in owners]
    import numpy as np
    x = np.arange(len(owners))
    cols = [R["color"].get(o, "#888") for o in owners]
    b1 = ax.bar(x - 0.2, pt, 0.38, color=cols, alpha=0.45, label="pit stops")
    b2 = ax.bar(x + 0.2, cr, 0.38, color=cols, label="wall hits")
    for b, v in list(zip(b1, pt)) + list(zip(b2, cr)):
        ax.text(b.get_x() + b.get_width() / 2, v, str(v), ha="center", va="bottom", color=TXT, fontsize=11, fontweight="bold")
    ax.set_xticks(x, owners, color=TXT, fontsize=12, fontweight="bold")
    ax.grid(axis="x", visible=False)
    ax.legend(frameon=False, labelcolor=SUB, loc="upper right")
    clean_o = min(zip(cr, owners))[1]
    desc = "Cleanest driver: **%s** (%d wall hits)\n" % (clean_o, min(cr)) + "\n".join("%s: %d stops, %d hits" % (o, p, c) for o, p, c in zip(owners, pt, cr))
    return fig, "Crashes & pit stops", desc


KINDS = {"best_lap": best_lap, "pace": pace, "laps": laps_done, "incidents": incidents}
ORDER = ["best_lap", "pace", "laps", "incidents"]


# ---------------------------------------------------------------- discord
def post(png, title, desc):
    token = read(os.path.join(DATA, "DISCORD_TOKEN.local.txt"))
    embed = {"title": "🏎️  " + title, "description": desc[:4000], "color": 0xE10600,
             "image": {"url": "attachment://chart.png"}, "url": SITE,
             "footer": {"text": "PITWALL · live at " + SITE.replace("https://", "")},
             "timestamp": datetime.utcnow().isoformat() + "Z"}
    boundary = uuid.uuid4().hex
    body = io.BytesIO()
    def part(headers, data):
        body.write(("--%s\r\n%s\r\n\r\n" % (boundary, "\r\n".join(headers))).encode())
        body.write(data)
        body.write(b"\r\n")
    part(['Content-Disposition: form-data; name="payload_json"', "Content-Type: application/json"],
         json.dumps({"embeds": [embed], "attachments": [{"id": 0, "filename": "chart.png"}]}).encode())
    part(['Content-Disposition: form-data; name="files[0]"; filename="chart.png"', "Content-Type: image/png"], png)
    body.write(("--%s--\r\n" % boundary).encode())
    req = urllib.request.Request("https://discord.com/api/v10/channels/%s/messages" % CHANNEL, data=body.getvalue(), method="POST",
                                 headers={"Authorization": "Bot " + token, "User-Agent": UA,
                                          "Content-Type": "multipart/form-data; boundary=" + boundary})
    with urllib.request.urlopen(req, timeout=60) as r:
        return json.loads(r.read())["id"]


def run(kind=None, dry=False):
    st = json.load(open(STATE)) if os.path.exists(STATE) else {"i": 0}
    if kind is None:
        kind = ORDER[st["i"] % len(ORDER)]
    events, cars = fetch()
    R = race(events, cars)
    fig, title, desc = KINDS[kind](R)
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, "%s-%s.png" % (time.strftime("%Y%m%d-%H%M"), kind))
    buf = io.BytesIO()
    fig.savefig(buf, format="png", facecolor=BG)
    plt.close(fig)
    open(path, "wb").write(buf.getvalue())
    print(path)
    print(desc)
    if not dry:
        mid = post(buf.getvalue(), title, desc)
        st["i"] = ORDER.index(kind) + 1
        st["last"] = {"kind": kind, "at": time.time(), "message": mid}
        json.dump(st, open(STATE, "w"))
        print("posted", mid)


if __name__ == "__main__":
    args = sys.argv[1:]
    dry = "--dry" in args
    if "--loop" in args:
        every = int(args[args.index("--loop") + 1])
        while True:
            time.sleep(every - time.time() % every)   # on the clock: :00 and :30
            try:
                st = json.load(open(STATE)) if os.path.exists(STATE) else {}
                if time.time() - st.get("last", {}).get("at", 0) < every * 0.6:
                    continue   # posted by hand recently; wait for the next slot
                run()
            except Exception as e:
                print(time.strftime("%H:%M:%S"), "post failed:", e, flush=True)
    else:
        rest = [a for a in args if not a.startswith("--")]
        run(rest[0] if rest else None, dry=dry)
