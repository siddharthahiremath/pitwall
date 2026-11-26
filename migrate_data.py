"""Copy this machine's PITWALL data (accounts, sessions, scripts, laps, live positions) to a remote
server that has PITWALL_IMPORT_TOKEN set.   usage: PITWALL_IMPORT_TOKEN=... python3 migrate_data.py https://host"""
import base64, json, os, sys, urllib.request

ROOT = os.path.dirname(os.path.abspath(__file__))
DATA = os.environ.get("PITWALL_DATA") or os.path.join(ROOT, "data")
files = {}
for name in ("accounts.json", "sessions.json", "stats.json", "live.json"):
    p = os.path.join(DATA, name)
    if os.path.exists(p):
        files[name] = base64.b64encode(open(p, "rb").read()).decode()
for fn in sorted(os.listdir(os.path.join(DATA, "bots"))):
    if fn.endswith(".py"):
        files["bots/" + fn] = base64.b64encode(open(os.path.join(DATA, "bots", fn), "rb").read()).decode()
req = urllib.request.Request(sys.argv[1].rstrip("/") + "/api/admin/import", data=json.dumps({"files": files}).encode(),
                             headers={"X-Import-Token": os.environ["PITWALL_IMPORT_TOKEN"], "Content-Type": "application/json"})
print(urllib.request.urlopen(req, timeout=60).read().decode())
