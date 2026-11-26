#!/usr/bin/env python3
"""PITWALL — a bot racing sim. Each account gets one car, driven by the Python script they upload.

Run:  python3 manage.py init      (creates accounts, once)
      python3 server.py           then open http://localhost:8765
"""
import base64
import hashlib
import hmac
import json
import math
import secrets
import os
import random
import select
import signal
import subprocess
import sys
import threading
import time
import traceback
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, unquote, parse_qs

import numpy as np

from track import Track, HALF_WIDTH, KERB, WALL, PIT_SPEED

ROOT = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.environ.get("PITWALL_DATA") or os.path.join(ROOT, "data")   # never committed: accounts, sessions, scripts, stats
BOT_DIR = os.path.join(DATA_DIR, "bots")
ACCOUNTS_FILE = os.path.join(DATA_DIR, "accounts.json")
SESSIONS_FILE = os.path.join(DATA_DIR, "sessions.json")
STATS_FILE = os.path.join(DATA_DIR, "stats.json")
TEMPLATE_FILE = os.path.join(ROOT, "examples", "template.py")
SANDBOX = os.path.join(ROOT, "sandbox.sb")
STATIC = os.path.join(ROOT, "static")
MAX_CODE = 200_000
CARS_PER_PLAYER = 3
MIN_SPEED, MAX_SPEED = 0.1, 50.0   # admin sim-speed range
SLOTS_FILE = os.path.join(DATA_DIR, "slots.json")   # each car's permanent (random) pit box
LIVE_FILE = os.path.join(DATA_DIR, "live.json")   # car positions, so a restart resumes mid-race
PORT = int(os.environ.get("PORT", "8765"))

PHYS_HZ = 120
CTRL_HZ = 20
NET_HZ = 30

# ---------------- car physics parameters (also handed to bots) ----------------
P = {
    "mass": 800.0,
    "power": 760e3,            # W
    "g": 9.81,
    "mu": 1.75,                # tyre friction coefficient (new medium, on asphalt)
    "k_down": 0.00265,         # downforce accel per (m/s)^2  -> extra grip = mu*k_down*v^2
    "k_drag": 0.00098,         # drag decel per (m/s)^2
    "rolling": 0.15,           # m/s^2
    "drs_drag_cut": 0.20,      # at 100% DRS drag is reduced 20%
    "drs_down_cut": 0.45,      # ...and downforce is reduced 45% (corners get slower)
    "max_drive_accel": 13.0,   # traction-limited launch
    "max_brake_accel": 55.0,
    "wheelbase": 3.6,
    "max_steer": 0.32,         # rad at full steer input
    "length": 5.6,
    "width": 2.0,
    "traction_limit": 0.97,    # throttle can use at most 97% of grip (no wheelspin unless cornering too)
    "brake_limit": 0.93,       # full brake uses 93% of grip -> ~37% left for steering
    "kinetic_grip": 0.80,      # grip multiplier while sliding
    "steer_overreach": 1.15,   # how far past the grip limit steering can push yaw (=> slides)
    "reverse_speed": 8.0,
    "surface": {"asphalt": 1.0, "kerb": 0.92, "grass": 0.50},
    "grass_drag": 2.5,         # extra decel on grass, m/s^2 (+ proportional to speed)
    "pit_speed_limit": PIT_SPEED,
    "pit_stop_time": 2.4,      # seconds stationary for a tyre change
    "repair_time_per_damage": 6.0,
}
TYRES = {
    #          grip   wear rate
    "soft":   (1.06, 1.75),
    "medium": (1.00, 1.00),
    "hard":   (0.955, 0.62),
}
WEAR_BASE = 4.0e-5   # per metre at moderate load (medium)
INERTIA = 1400.0
LIDAR_RAYS = 36
LIDAR_RANGE = 150.0

COLORS = ["#ff2d55", "#00c2ff", "#ffb800", "#2bd96b", "#b76dff", "#ff7a1a", "#f5f5f5",
          "#00e0c6", "#ff5fd2", "#9adf2a", "#4f7dff", "#ffd84d", "#c0c0c0", "#ff4040",
          "#33ffaa", "#a8865f", "#7fd1ff", "#e0a3ff", "#88ff66", "#ff9e9e"]

track = Track()
GEOM = track.geometry()
L = track.length


def clamp(v, a, b):
    return a if v < a else b if v > b else v


def num(v, default=0.0):
    try:
        f = float(v)
        if math.isnan(f) or math.isinf(f):
            return default
        return f
    except Exception:
        return default


def wrap_angle(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def tyre_grip(comp, wear):
    base = TYRES.get(comp, TYRES["medium"])[0]
    if wear <= 0.75:
        f = 1 - 0.16 * wear
    else:
        f = 1 - 0.12 - (wear - 0.75) * 1.7
    return base * max(0.42, f)


# ---------------- event log ----------------
EVENTS = []
EV_LOCK = threading.Lock()
EV_ID = [0]


def event(kind, text, **extra):
    with EV_LOCK:
        EV_ID[0] += 1
        EVENTS.append({"id": EV_ID[0], "kind": kind, "text": text, "t": round(SIM.t, 2) if "SIM" in globals() else 0, **extra})
        del EVENTS[:-200]


# ---------------- bot worker (one subprocess per car) ----------------
class Worker:
    def __init__(self, car):
        self.car = car
        self.proc = None
        self.lock = threading.Lock()
        self.latest = None      # latest controls dict
        self.busy_since = None
        self.pending_tick = None
        self.error = None
        self.log = []
        self.ms = 0.0
        self.generation = 0
        self.alive = False

    def start(self):
        self.stop()
        self.generation += 1
        gen = self.generation
        self.error = None
        self.log = []
        self.busy_since = None
        try:
            self.proc = subprocess.Popen(
                bot_command(self.car.path),
                stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                cwd=ROOT, text=True, bufsize=1,
                env={"PATH": "/usr/bin:/bin", "PYTHONDONTWRITEBYTECODE": "1", "PYTHONUNBUFFERED": "1",
                     "HOME": "/var/empty", "OPENBLAS_NUM_THREADS": "1", "OMP_NUM_THREADS": "1",
                     "PITWALL_GUARD": "1" if _linux_bot_user() else "0"},
                **bot_popen_extra())
        except Exception as e:
            self.error = "failed to start: %s" % e
            return
        init = {"type": "init", "track": GEOM, "params": P, "tyres": {k: {"grip": v[0], "wear_rate": v[1]} for k, v in TYRES.items()},
                "car_id": self.car.id, "name": self.car.name}
        if _linux_bot_user():   # the bot user can't read the data volume, so send the code itself
            try:
                with open(self.car.path) as f:
                    init["code"] = f.read()
            except Exception as e:
                init["code"] = "raise RuntimeError(%r)" % ("could not read script: %s" % e)
        try:
            self.proc.stdin.write(json.dumps(init) + "\n")
            self.proc.stdin.flush()
        except Exception as e:
            self.error = "init failed: %s" % e
        self.alive = True
        threading.Thread(target=self._reader, args=(self.proc, gen), daemon=True).start()
        threading.Thread(target=self._stderr, args=(self.proc, gen), daemon=True).start()

    def _reader(self, proc, gen):
        for line in proc.stdout:
            if gen != self.generation:
                return
            try:
                msg = json.loads(line)
            except Exception:
                continue
            with self.lock:
                if msg.get("type") == "fatal":
                    self.error = msg.get("error", "fatal error")
                    self.latest = None
                    continue
                if msg.get("type") == "ready":
                    continue
                self.busy_since = None
                self.ms = msg.get("ms", 0.0)
                if msg.get("log"):
                    self.log = (self.log + msg["log"].rstrip("\n").split("\n"))[-12:]
                if "error" in msg and msg["error"]:
                    self.error = msg["error"]
                    self.latest = None
                else:
                    self.error = None
                    self.latest = msg.get("controls") or {}
        if gen == self.generation:
            self.alive = False
            if not self.error:
                self.error = "bot process exited"

    def _stderr(self, proc, gen):
        for line in proc.stderr:
            if gen != self.generation:
                return
            with self.lock:
                self.log = (self.log + ["[stderr] " + line.rstrip()])[-12:]

    def ready(self):
        return self.alive and self.busy_since is None

    def send(self, line):
        if not self.alive:
            return
        try:
            self.busy_since = time.time()
            self.proc.stdin.write(line)
            self.proc.stdin.flush()
        except Exception as e:
            self.alive = False
            self.error = "pipe broken: %s" % e

    def check_hang(self):
        if self.busy_since and time.time() - self.busy_since > 3.0:
            self.error = "drive() took > 3 s — restarted the bot"
            event("error", "%s: bot timed out, restarting" % self.car.name, car=self.car.id)
            err = self.error
            self.start()
            self.error = err

    def stop(self):
        self.generation += 1
        self.alive = False
        if self.proc:
            try:
                self.proc.kill()
            except Exception:
                pass
            self.proc = None


def bot_command(path):
    """macOS: bots run under sandbox-exec (no network, no file writes, can't read your home folder
    or other people's scripts). Linux (Railway): see bot_popen_extra()."""
    cmd = [sys.executable, "-u", os.path.join(ROOT, "runner.py"), path]
    if sys.platform == "darwin" and os.path.exists("/usr/bin/sandbox-exec") and os.environ.get("PITWALL_NO_SANDBOX") != "1":
        home = os.path.expanduser("~")
        return ["/usr/bin/sandbox-exec", "-f", SANDBOX, "-D", "HOME=" + home, "-D", "PYHOME=" + sys.base_prefix,
                "-D", "PYHOME2=" + sys.prefix, "-D", "APPDIR=" + ROOT, "-D", "DATADIR=" + DATA_DIR,
                "-D", "BOTFILE=" + path] + cmd
    return cmd


def _linux_bot_user():
    if sys.platform != "linux" or os.environ.get("PITWALL_NO_SANDBOX") == "1" or os.geteuid() != 0:
        return None
    try:
        import pwd
        return pwd.getpwnam("bot")
    except (ImportError, KeyError):
        return None


def _bot_limits():
    import resource
    resource.setrlimit(resource.RLIMIT_FSIZE, (0, 0))              # can't write files
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
    resource.setrlimit(resource.RLIMIT_AS, (1536 << 20, 1536 << 20))  # 1.5 GB of address space
    os.setsid()


def bot_popen_extra():
    """Linux: bots run as an unprivileged 'bot' user that can't read the data volume (accounts,
    other people's scripts), with no file writes, and runner.py's audit hook blocks sockets,
    subprocesses and ctypes. The script itself is handed over in the init message."""
    u = _linux_bot_user()
    if not u:
        return {}
    return {"user": u.pw_uid, "group": u.pw_gid, "extra_groups": [], "preexec_fn": _bot_limits}


# ---------------- car ----------------
class Car:
    def __init__(self, cid, owner, num, path, slot, color):
        name = "%s#%d" % (owner, num)
        self.id = cid
        self.owner = owner
        self.num = num
        self.name = name
        self.path = path
        self.slot = slot
        self.color = color
        self.mtime = os.path.getmtime(path)
        self.x = self.y = self.h = 0.0
        self.vx = self.vy = 0.0
        self.omega = 0.0        # yaw rate (actual)
        self.omega_ext = 0.0    # yaw from impacts, decays
        self.throttle = self.brake = self.steer = 0.0
        self.drs = 0.0
        self.drs_cmd = 0.0
        self.pit_req = False
        self.next_tyre = "medium"
        self.compound = "medium"
        self.wear = 0.0
        self.damage = 0.0
        self.sliding = False
        self.slip = 0.0
        self.surface = "asphalt"
        self.idx = 0
        self.s = 0.0
        self.lat = 0.0
        self.dist = 0.0
        self.laps = 0
        self.lap_start = None
        self.last_lap = None
        self.best_lap = None
        self.cp = [False, False, False]
        self.status = "boxed"   # track | pit_in | boxed | pit_out
        self.pit_u = 0.0
        self.pit_v = 0.0
        self.pit_blend = 0.0
        self.box_timer = 0.0
        self.ghost = True
        self.stuck = 0.0
        self.a_long = 0.0
        self.a_lat = 0.0
        self.collisions = 0
        self.pit_stops = 0
        self.lidar = [LIDAR_RANGE] * LIDAR_RAYS
        self.lidar_hit = ["none"] * LIDAR_RAYS
        self.lidar_car = [None] * LIDAR_RAYS
        st = STATS.get(name, {})
        self.laps = st.get("laps", 0)
        self.best_lap = st.get("best")
        self.last_lap = st.get("last")
        self.pit_stops = st.get("stops", 0)
        self.collisions = st.get("collisions", 0)
        self.dist = st.get("dist", 0.0)
        self.worker = Worker(self)
        self.put_in_box(first=True)
        restore_live(self)

    @property
    def speed(self):
        return math.hypot(self.vx, self.vy)

    def put_in_box(self, first=False, wait=1.5):
        self.status = "boxed"
        self.pit_u = track.box_u(self.slot)
        self.pit_v = 0.0
        self.pit_blend = 0.0
        self.box_timer = wait
        self.vx = self.vy = 0.0
        self.omega = self.omega_ext = 0.0
        self.ghost = True
        self.sliding = False
        self._pose_from_pit()
        if first:
            self.wear = 0.0
            self.compound = "medium"

    def _pose_from_pit(self):
        bu = track.box_u(self.slot)
        x, y, _ = track.pit_pose(self.pit_u, bu, self.pit_blend)
        x2, y2, _ = track.pit_pose(self.pit_u + 0.5, bu, self.pit_blend)
        self.x, self.y = x, y
        self.h = math.atan2(y2 - y, x2 - x)
        self.s = track.pit_s(self.pit_u)
        self.idx = track.idx_at(self.s)
        _, _, self.lat = track.project(x, y, self.idx, 3)

    def corners(self):
        c, s = math.cos(self.h), math.sin(self.h)
        hl, hw = P["length"] / 2, P["width"] / 2
        out = []
        for lx, ly in ((hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw)):
            out.append((self.x + c * lx - s * ly, self.y + s * lx + c * ly))
        return out

    def grip(self):
        surf = P["surface"][self.surface]
        down = P["k_down"] * (1 - P["drs_down_cut"] * self.drs) * (1 - 0.35 * self.damage)
        v = self.speed
        g = P["mu"] * surf * tyre_grip(self.compound, self.wear) * (P["g"] + down * v * v)
        if self.sliding:
            g *= P["kinetic_grip"]
        return g

    # ---- physics on track ----
    def step_track(self, dt):
        h = self.h
        fx, fy = math.cos(h), math.sin(h)
        rx, ry = -fy, fx
        vl = self.vx * fx + self.vy * fy
        vt = self.vx * rx + self.vy * ry
        v = math.hypot(vl, vt)

        # DRS flap moves at most 250%/s
        self.drs += clamp(self.drs_cmd - self.drs, -2.5 * dt, 2.5 * dt)

        grip = self.grip()
        # steering -> yaw (kinematic), but limited by what the front tyres can hold
        delta = self.steer * P["max_steer"]
        w_cmd = vl * math.tan(delta) / P["wheelbase"]
        w_cap = grip * P["steer_overreach"] / max(abs(vl), 3.0)
        w = clamp(w_cmd, -w_cap, w_cap)
        self.omega = w + self.omega_ext
        self.h = wrap_angle(self.h + self.omega * dt)
        self.omega_ext *= math.exp(-2.5 * dt)

        # longitudinal tyre force demand
        thr = self.throttle
        a_tyre = 0.0
        if thr >= 0:
            power = P["power"] * (1 - 0.25 * self.damage)
            a_drive = thr * min(power / P["mass"] / max(abs(vl), 1.0), P["max_drive_accel"])
            if vl < -0.5:  # throttle while rolling backwards acts as a brake
                a_drive = thr * P["max_drive_accel"]
            a_tyre += min(a_drive, P["traction_limit"] * grip)   # built-in traction control
        else:
            a_tyre += thr * 6.0 if vl > -P["reverse_speed"] else 0.0
        if self.brake > 0 and abs(vl) > 0.05:
            a_b = self.brake * min(P["brake_limit"] * grip, P["max_brake_accel"])   # ABS-ish: keeps a little steering
            a_tyre -= math.copysign(a_b, vl)
        # friction circle: longitudinal first (wheelspin / lockup = slide)
        long_slip = abs(a_tyre) > grip
        a_tyre = clamp(a_tyre, -grip, grip)
        lat_avail = math.sqrt(max(grip * grip - a_tyre * a_tyre, 0.0))
        a_lat_need = -vt / dt
        a_lat = clamp(a_lat_need, -lat_avail, lat_avail)

        # resistances
        kd = P["k_drag"] * (1 - P["drs_drag_cut"] * self.drs) * (1 + 0.25 * self.damage)
        a_res = kd * vl * abs(vl) + math.copysign(P["rolling"], vl) * (abs(vl) > 0.1)
        if self.surface == "grass":
            a_res += math.copysign(P["grass_drag"] + 0.04 * abs(vl), vl) * (abs(vl) > 0.1)
        if self.brake > 0 and abs(vl) <= 0.3 and thr <= 0.05:
            vl_new = 0.0
        else:
            vl_new = vl + (a_tyre - a_res) * dt
            if (vl > 0 and vl_new < 0 and thr >= 0) or (vl < 0 and vl_new > 0 and thr <= 0 and self.brake > 0):
                vl_new = 0.0
        vt_new = vt + a_lat * dt
        # sliding scrubs speed a bit
        if abs(vt_new) > 0.3:
            vl_new -= math.copysign(min(abs(vl_new), 0.15 * grip * dt * min(1.0, abs(vt_new) / 4)), vl_new)

        # write back in the pre-rotation frame: next step the heading/velocity mismatch is the
        # centripetal demand the tyres must supply (that's what makes grip limits & slides real)
        self.vx = fx * vl_new + rx * vt_new
        self.vy = fy * vl_new + ry * vt_new
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.a_long = (vl_new - vl) / dt
        self.a_lat = a_lat

        # slide state with hysteresis
        speed = math.hypot(vl_new, vt_new)
        self.slip = math.degrees(math.atan2(abs(vt_new), max(abs(vl_new), 1.0)))
        if not self.sliding and ((self.slip > 5.0 and abs(vt_new) > 1.5) or abs(vt_new) > 4.0 or (long_slip and speed > 5)):
            self.sliding = True
        elif self.sliding and self.slip < 2.5 and abs(vt_new) < 1.5 and not long_slip:
            self.sliding = False

        # tyre wear
        usage = min(1.5, math.hypot(a_tyre, a_lat) / max(grip, 1e-3))
        rate = TYRES.get(self.compound, TYRES["medium"])[1]
        self.wear += rate * WEAR_BASE * speed * dt * (0.25 + 0.75 * usage * usage)
        if self.sliding:
            self.wear += rate * 0.0010 * abs(vt_new) * dt + (0.0004 * speed * dt if long_slip else 0)
        if self.surface == "grass":
            self.wear += 0.00002 * speed * dt
        self.wear = min(self.wear, 1.0)

    # ---- pit-lane autopilot ----
    def step_pit(self, dt):
        bu = track.box_u(self.slot)
        lim = PIT_SPEED
        if self.status == "pit_in":
            # stop at the box
            dist = bu - self.pit_u
            v_stop = math.sqrt(max(0.0, 2 * 9.0 * max(dist, 0)))
            target = min(lim, v_stop)
            if self.pit_v > target:
                self.pit_v = max(target, self.pit_v - 30.0 * dt)
            else:
                self.pit_v = min(target, self.pit_v + 10.0 * dt)
            if dist <= 0.05:
                self.pit_u = bu
                self.pit_v = 0.0
                self.status = "boxed"
                t = P["pit_stop_time"] + P["repair_time_per_damage"] * self.damage
                self.box_timer = t
                self.pit_stops += 1
                old = self.compound
                event("pit", "%s pit stop: %s → %s (%.1fs)" % (self.name, old, self.next_tyre, t), car=self.id)
        elif self.status == "boxed":
            self.pit_v = 0.0
            held = time.time() < SIM.start_at   # grid start: everyone waits in their box for lights out
            if not held:
                self.box_timer -= dt
            if self.box_timer <= 0 and not held:
                self.compound = self.next_tyre if self.next_tyre in TYRES else "medium"
                self.wear = 0.0
                self.damage = 0.0
                self.status = "pit_out"
        elif self.status == "pit_out":
            self.pit_v = min(lim, self.pit_v + 9.0 * dt)
            if self.pit_u >= track.pit_len - 0.5:
                # release onto the track
                self.status = "track"
                x, y, hd = track.point_at(track.pit_s(track.pit_len), 0.0)
                self.x, self.y, self.h = x, y, hd
                self.vx, self.vy = math.cos(hd) * self.pit_v, math.sin(hd) * self.pit_v
                self.idx, self.s, self.lat = track.project(x, y, track.idx_at(track.pit_s(track.pit_len)), 4)
                self.ghost = True
                return
        prev_s = self.s
        self.pit_u = min(self.pit_u + self.pit_v * dt, track.pit_len)
        old = (self.x, self.y, self.h)
        self._pose_from_pit()
        if self.pit_v < 0.1:
            self.h = old[2]
        self.vx = (self.x - old[0]) / dt
        self.vy = (self.y - old[1]) / dt
        self.omega = wrap_angle(self.h - old[2]) / dt
        self.drs = 0.0
        self.sliding = False
        self.surface = "asphalt"
        self.after_move(prev_s)

    def after_move(self, prev_s):
        ds = self.s - prev_s
        if ds < -L / 2:
            ds += L
        elif ds > L / 2:
            ds -= L
        self.dist += ds
        # checkpoints and lap counting
        for i, frac in enumerate((0.25, 0.5, 0.75)):
            if abs(self.s - frac * L) < 40:
                self.cp[i] = True
        if prev_s > 0.8 * L and self.s < 0.2 * L:
            now = SIM.t
            if all(self.cp) and self.lap_start is not None:
                self.laps += 1
                lt = now - self.lap_start
                self.last_lap = lt
                best = self.best_lap is None or lt < self.best_lap
                if best:
                    self.best_lap = lt
                event("lap", "%s lap %d — %s%s" % (self.name, self.laps, fmt_time(lt), " (PB)" if best else ""), car=self.id)
            elif all(self.cp):
                self.laps += 1
                event("lap", "%s lap %d" % (self.name, self.laps), car=self.id)
            self.lap_start = now
            self.cp = [False, False, False]
        elif prev_s < 0.2 * L and self.s > 0.8 * L:
            self.cp = [False, False, False]  # went backwards over the line

    def update_track_info(self):
        # cars move < 1 m per step and points are 4 m apart, so a small search window is plenty;
        # fall back to a wide search if the car somehow ended up far from its last index
        i, s, lat = track.project(self.x, self.y, self.idx, 4)
        px, py = track.pts[i]
        if (self.x - px) ** 2 + (self.y - py) ** 2 > 40.0 ** 2:
            i, s, lat = track.project(self.x, self.y, self.idx, 25)
        self.idx, self.s, self.lat = i, s, lat
        a = abs(self.lat)
        self.surface = "asphalt" if a <= HALF_WIDTH else "kerb" if a <= HALF_WIDTH + KERB else "grass"

    def public(self):
        return {
            "id": self.id, "name": self.name, "owner": self.owner, "car_number": self.num, "x": self.x, "y": self.y, "heading": self.h,
            "vx": self.vx, "vy": self.vy, "speed": self.speed, "yaw_rate": self.omega,
            "s": self.s, "lateral_offset": self.lat, "laps": self.laps, "dist": self.dist,
            "status": self.status, "in_pit": self.status != "track", "ghost": self.ghost,
            "sliding": self.sliding, "tyre": self.compound, "tyre_wear": self.wear, "damage": self.damage,
            "drs": self.drs * 100, "hitbox": {"length": P["length"], "width": P["width"],
                                                "corners": [list(p) for p in self.corners()]},
        }


def car_full(car, sim):
    d = car.public()
    fx, fy = math.cos(car.h), math.sin(car.h)
    hd = track.hd[car.idx]
    d.update({
        "v_long": car.vx * fx + car.vy * fy, "v_lat": -car.vx * fy + car.vy * fx,
        "accel_long": car.a_long, "accel_lat": car.a_lat,
        "throttle": car.throttle, "brake": car.brake, "steer": car.steer,
        "slip_angle_deg": car.slip, "surface": car.surface, "on_track": car.surface != "grass",
        "track_heading": hd, "heading_error": wrap_angle(car.h - hd),
        "lap_progress": car.s / L, "lap_time": (sim.t - car.lap_start) if car.lap_start is not None else None,
        "last_lap": car.last_lap, "best_lap": car.best_lap, "grip_now": car.grip(),
        "tyre_grip_factor": tyre_grip(car.compound, car.wear), "next_tyre": car.next_tyre,
        "pit_stops": car.pit_stops, "collisions": car.collisions, "pit_requested": car.pit_req,
        "distance_to_pit_entry": (track.pit_entry_s - car.s) % L,
        "position": getattr(car, "position", None),
        "_idx": car.idx,
    })
    return d


def fmt_time(t):
    if t is None:
        return "—"
    m = int(t // 60)
    return "%d:%06.3f" % (m, t - 60 * m)


# ---------------- collisions ----------------
def obb_axes(car):
    c, s = math.cos(car.h), math.sin(car.h)
    return [(c, s), (-s, c)]


def sat(a, b):
    ca, cb = a.corners(), b.corners()
    best = None
    for ax in obb_axes(a) + obb_axes(b):
        pa = [p[0] * ax[0] + p[1] * ax[1] for p in ca]
        pb = [p[0] * ax[0] + p[1] * ax[1] for p in cb]
        o = min(max(pa), max(pb)) - max(min(pa), min(pb))
        if o <= 0:
            return None
        if best is None or o < best[0]:
            best = (o, ax)
    depth, n = best
    dx, dy = b.x - a.x, b.y - a.y
    if dx * n[0] + dy * n[1] < 0:
        n = (-n[0], -n[1])
    # contact point: corners of one inside the other
    pts = [p for p in cb if inside(a, p)] + [p for p in ca if inside(b, p)]
    if pts:
        cx = sum(p[0] for p in pts) / len(pts)
        cy = sum(p[1] for p in pts) / len(pts)
    else:
        cx, cy = (a.x + b.x) / 2, (a.y + b.y) / 2
    return depth, n, (cx, cy)


def inside(car, p):
    c, s = math.cos(car.h), math.sin(car.h)
    dx, dy = p[0] - car.x, p[1] - car.y
    lx = dx * c + dy * s
    ly = -dx * s + dy * c
    return abs(lx) <= P["length"] / 2 and abs(ly) <= P["width"] / 2


def cross(ax, ay, bx, by):
    return ax * by - ay * bx


def resolve_pair(a, b, depth, n, cp):
    m = P["mass"]
    a.x -= n[0] * depth / 2
    a.y -= n[1] * depth / 2
    b.x += n[0] * depth / 2
    b.y += n[1] * depth / 2
    rax, ray = cp[0] - a.x, cp[1] - a.y
    rbx, rby = cp[0] - b.x, cp[1] - b.y
    va = (a.vx - a.omega * ray, a.vy + a.omega * rax)
    vb = (b.vx - b.omega * rby, b.vy + b.omega * rbx)
    rvx, rvy = vb[0] - va[0], vb[1] - va[1]
    vn = rvx * n[0] + rvy * n[1]
    if vn >= 0:
        return 0.0
    ran = cross(rax, ray, n[0], n[1])
    rbn = cross(rbx, rby, n[0], n[1])
    e = 0.25
    j = -(1 + e) * vn / (2 / m + ran * ran / INERTIA + rbn * rbn / INERTIA)
    # friction
    tx, ty = -n[1], n[0]
    vt = rvx * tx + rvy * ty
    rat = cross(rax, ray, tx, ty)
    rbt = cross(rbx, rby, tx, ty)
    jt = -vt / (2 / m + rat * rat / INERTIA + rbt * rbt / INERTIA)
    jt = clamp(jt, -0.3 * j, 0.3 * j)
    ix, iy = n[0] * j + tx * jt, n[1] * j + ty * jt
    a.vx -= ix / m
    a.vy -= iy / m
    b.vx += ix / m
    b.vy += iy / m
    a.omega_ext -= cross(rax, ray, ix, iy) / INERTIA * 0.6
    b.omega_ext += cross(rbx, rby, ix, iy) / INERTIA * 0.6
    dv = j / m
    a.damage = min(1.0, a.damage + max(0, dv - 2.0) / 70)
    b.damage = min(1.0, b.damage + max(0, dv - 2.0) / 70)
    return dv


def wall_collide(car):
    if abs(car.lat) + 3.2 < WALL:   # no corner can reach a wall (half-diagonal of the car is ~3 m)
        return 0.0, None
    m = P["mass"]
    hit = 0.0
    pos = None
    for _ in range(2):
        worst = None
        for (px, py) in car.corners():
            i, s, lat = track.project(px, py, car.idx, 6)
            pen = abs(lat) - WALL
            if pen > 0 and (worst is None or pen > worst[0]):
                sign = 1 if lat > 0 else -1
                n = (-track.nx[i] * sign, -track.ny[i] * sign)  # points back toward the track
                worst = (pen, n, (px, py))
        if worst is None:
            break
        pen, n, (px, py) = worst
        car.x += n[0] * pen
        car.y += n[1] * pen
        rx, ry = px - car.x, py - car.y
        vx = car.vx - car.omega * ry
        vy = car.vy + car.omega * rx
        vn = vx * n[0] + vy * n[1]
        if vn < 0:
            rn = cross(rx, ry, n[0], n[1])
            j = -(1 + 0.3) * vn / (1 / m + rn * rn / INERTIA)
            tx, ty = -n[1], n[0]
            vt = vx * tx + vy * ty
            rt = cross(rx, ry, tx, ty)
            jt = clamp(-vt / (1 / m + rt * rt / INERTIA), -0.35 * j, 0.35 * j)
            ix, iy = n[0] * j + tx * jt, n[1] * j + ty * jt
            car.vx += ix / m
            car.vy += iy / m
            car.omega_ext += cross(rx, ry, ix, iy) / INERTIA * 0.6
            hit = max(hit, j / m)
            pos = (px, py)
    if hit > 2.0:
        car.damage = min(1.0, car.damage + (hit - 2.0) / 60)
    return hit, pos


# ---------------- lidar (numpy raycast) ----------------
# static boundary segments: [left edge, right edge, left wall, right wall], flattened once
_N = track.n
_SA = np.concatenate([np.array(GEOM[k], dtype=float) for k in ("left", "right", "wall_left", "wall_right")])
_SB = np.concatenate([np.roll(np.array(GEOM[k], dtype=float), -1, axis=0) for k in ("left", "right", "wall_left", "wall_right")])
_KIND_NAMES = ("edge", "wall", "car")
_HIT_CHARS = "ewc"
RAY_ANGLES = np.array([-math.pi + i * 2 * math.pi / LIDAR_RAYS for i in range(LIDAR_RAYS)])
_LIDAR_W = 40                                     # +-40 points (+-160 m) covers the 150 m range
_WIN = np.arange(-_LIDAR_W, _LIDAR_W)
_POLY_OFF = (np.arange(4) * _N)[:, None]
_STATIC_KINDS = np.repeat(np.array([0, 0, 1, 1]), len(_WIN))


def lidar_scan(car, cars):
    sel = (_POLY_OFF + (car.idx + _WIN) % _N).ravel()
    A, B, kinds = _SA[sel], _SB[sel], _STATIC_KINDS
    owners = None
    extra_a, extra_b, ids = [], [], []
    r2 = (LIDAR_RANGE + 5) ** 2
    for o in cars:
        if o is car or o.status == "boxed" or (o.x - car.x) ** 2 + (o.y - car.y) ** 2 > r2:
            continue
        cs = o.corners()
        extra_a += cs
        extra_b += cs[1:] + cs[:1]
        ids += [o.id] * 4
    if extra_a:
        A = np.concatenate([A, extra_a])
        B = np.concatenate([B, extra_b])
        kinds = np.concatenate([kinds, np.full(len(extra_a), 2)])
        owners = [None] * len(sel) + ids
    ang = RAY_ANGLES + car.h
    dx, dy = np.cos(ang)[:, None], np.sin(ang)[:, None]
    E = B - A
    Q = A - (car.x, car.y)
    ex, ey, qx, qy = E[:, 0], E[:, 1], Q[:, 0], Q[:, 1]
    den = dx * ey - dy * ex
    den[np.abs(den) < 1e-9] = np.nan
    inv = 1.0 / den
    t = (qx * ey - qy * ex) * inv
    u = (qx * dy - qy * dx) * inv
    t[~((t > 0) & (t <= LIDAR_RANGE) & (u >= 0) & (u <= 1))] = np.inf
    j = t.argmin(axis=1)
    best = t[np.arange(LIDAR_RAYS), j]
    hm = np.isfinite(best)
    car.lidar = np.where(hm, best, LIDAR_RANGE).tolist()
    kj = np.where(hm, kinds[j], -1).tolist()
    car.lidar_hit = [_KIND_NAMES[k] if k >= 0 else "none" for k in kj]
    if owners is None:
        car.lidar_car = [None] * LIDAR_RAYS
    else:
        jl = j.tolist()
        car.lidar_car = [owners[jl[r]] if kj[r] == 2 else None for r in range(LIDAR_RAYS)]


# ---------------- simulation ----------------
class Sim:
    def __init__(self):
        self.t = 0.0
        self.cars = {}
        self.lock = threading.RLock()
        self.paused = False
        self.speed = 1.0
        self.next_id = 1
        self.ctrl_acc = 0.0
        self.tick = 0
        self.sparks = []
        self.rate = 1.0
        self.start_at = 0.0     # wall-clock time of the next grid start (cars stay boxed until then)

    def free_slot(self, name=None):
        """A car's permanent pit box: the one in SLOTS if it's free, otherwise a random free box
        (not one reserved for another car), which then becomes its permanent box."""
        used = {c.slot for c in self.cars.values()}
        if name and SLOTS.get(name) is not None and SLOTS[name] not in used and SLOTS[name] < track.max_boxes:
            return SLOTS[name]
        reserved = {v for k, v in SLOTS.items() if k != name}
        free = [i for i in range(track.max_boxes) if i not in used and i not in reserved] or \
               [i for i in range(track.max_boxes) if i not in used]
        if not free:
            return None
        slot = random.choice(free)
        if name:
            SLOTS[name] = slot
            save_json(SLOTS_FILE, SLOTS)
        return slot

    def grid_start(self, at):
        """Reset every lap count, shuffle all cars into random permanent pit boxes, and hold them
        there until wall-clock time `at`, when they all leave at once."""
        with self.lock:
            cars = list(self.cars.values())
            boxes = random.sample(range(track.max_boxes), len(cars))
            SLOTS.clear()
            for c, b in zip(cars, boxes):
                c.slot = b
                SLOTS[c.name] = b
            save_json(SLOTS_FILE, SLOTS)
            STATS.clear()
            for c in cars:
                c.laps = 0
                c.last_lap = c.best_lap = None
                c.lap_start = None
                c.pit_stops = 0
                c.collisions = 0
                c.dist = 0.0
                c.cp = [False, False, False]
                c.put_in_box(first=True, wait=0.0)
                c.next_tyre = "medium"
            self.start_at = at
        save_stats()
        save_live()
        event("reset", "Race control: laps reset, cars to random pit boxes, lights out at %s" %
              time.strftime("%H:%M:%S UTC", time.gmtime(at)))

    def team(self, owner):
        return sorted((c for c in self.cars.values() if c.owner == owner), key=lambda c: c.num)

    def add_team(self, owner, path):
        """Give a player their cars (all running the same script). Only adds the ones they don't have yet,
        each in a free pit box, so existing cars keep racing untouched."""
        added = []
        with self.lock:
            have = {c.num for c in self.team(owner)}
            color = ACCOUNTS.get(owner, {}).get("color")
            if not color:
                used = {c.color for c in self.cars.values() if c.owner != owner}
                color = next((c for c in COLORS if c not in used), "#%06x" % random.randint(0, 0xffffff))
            for num in range(1, CARS_PER_PLAYER + 1):
                if num in have:
                    continue
                name = "%s#%d" % (owner, num)
                live = LIVE.get(name)
                slot = SLOTS.get(name, live.get("slot") if live else None)
                if slot is None or slot in {c.slot for c in self.cars.values()}:
                    slot = self.free_slot(name)
                elif name not in SLOTS:
                    SLOTS[name] = slot
                    save_json(SLOTS_FILE, SLOTS)
                if slot is None:
                    break
                car = Car(self.next_id, owner, num, path, slot, color)
                self.next_id += 1
                if car.status == "boxed" and name not in LIVE:
                    car.box_timer = 1.0 + 1.5 * len(added)
                self.cars[name] = car
                added.append(car)
        for car in added:
            car.worker.start()
            event("join", "%s joined (pit box %d)" % (car.name, car.slot + 1), car=car.id)
        return added

    def reload_team(self, owner):
        for car in self.team(owner):
            car.mtime = os.path.getmtime(car.path)
            car.worker.start()
        event("reload", "%s: new script deployed to all %d cars" % (owner, len(self.team(owner))))

    # --- control: build state for each bot that is ready ---
    def world_line(self, cars):
        """One JSON line describing every car, shared by all bots this tick (botview.py builds each bot's state)."""
        world = {"type": "tick", "tick": self.tick, "t": self.t, "dt": 1.0 / CTRL_HZ,
                 "cars": [car_full(c, self) for c in cars]}
        return json.dumps(world, separators=(",", ":")) + "\n"

    def apply_controls(self, car, c):
        if c is None:
            # bot is erroring: coast to a stop
            car.throttle, car.brake, car.steer = 0.0, 0.4, 0.0
            return
        car.throttle = clamp(num(c.get("throttle", 0)), -1, 1)
        car.brake = clamp(num(c.get("brake", 0)), 0, 1)
        car.steer = clamp(num(c.get("steer", 0)), -1, 1)
        car.drs_cmd = clamp(num(c.get("drs", 0)), 0, 100) / 100.0
        car.pit_req = bool(c.get("pit", False))
        tyre = c.get("tyre", c.get("tire"))
        if isinstance(tyre, str) and tyre.lower() in TYRES:
            car.next_tyre = tyre.lower()

    def step(self, dt):
        with self.lock:
            cars = list(self.cars.values())
            self.ctrl_acc += dt
            if self.ctrl_acc >= 1.0 / CTRL_HZ:
                self.ctrl_acc -= 1.0 / CTRL_HZ
                self.tick += 1
                ready = []
                for car in cars:
                    w = car.worker
                    with w.lock:
                        c = w.latest
                        err = w.error
                    if c is not None or err:
                        self.apply_controls(car, c if not err else None)
                    w.check_hang()
                    if w.ready():
                        ready.append(w)
                if ready:
                    line = self.world_line(cars)
                    for w in ready:
                        w.send(line)

            for car in cars:
                if car.status == "track":
                    prev_s = car.s
                    car.step_track(dt)
                    car.update_track_info()
                    car.after_move(prev_s)
                    # pit capture: request + near the entry
                    if car.pit_req and car.status == "track":
                        u = (car.s - track.pit_entry_s) % L
                        if u < 25 and car.lat < 4.0 and abs(wrap_angle(car.h - track.hd[car.idx])) < 1.0:
                            car.status = "pit_in"
                            car.pit_u = u
                            car.pit_blend = car.lat
                            car.pit_v = max(car.speed, 5.0)
                            car.ghost = True
                            event("pit", "%s enters the pit lane" % car.name, car=car.id)
                    # stuck detection -> tow to the box
                    if car.speed < 1.0 and car.status == "track":
                        car.stuck += dt
                        if car.stuck > 10:
                            car.stuck = 0
                            event("tow", "%s stuck — towed to the pits" % car.name, car=car.id)
                            car.put_in_box(wait=8.0)
                    else:
                        car.stuck = 0
                else:
                    if car.status == "pit_in" and car.pit_v > PIT_SPEED:
                        car.pit_v = max(PIT_SPEED, car.pit_v - 28 * dt)  # brake for limiter
                    car.step_pit(dt)

            # car-car collisions
            solid = [c for c in cars if c.status == "track"]
            for i in range(len(solid)):
                a = solid[i]
                for j in range(i + 1, len(solid)):
                    b = solid[j]
                    if (a.x - b.x) ** 2 + (a.y - b.y) ** 2 > 36:
                        continue
                    r = sat(a, b)
                    if r is None:
                        continue
                    if a.ghost or b.ghost:
                        continue
                    dv = resolve_pair(a, b, *r)
                    if dv > 3.0:
                        a.collisions += 1
                        b.collisions += 1
                        self.sparks.append({"x": r[2][0], "y": r[2][1], "p": min(1, dv / 20)})
                        if dv > 6:
                            event("crash", "Contact: %s × %s (%.0f km/h)" % (a.name, b.name, dv * 3.6), a=a.id, b=b.id)
            # ghosts become solid once clear of everyone
            for c in solid:
                if c.ghost:
                    clear = all(((o is c) or (o.x - c.x) ** 2 + (o.y - c.y) ** 2 > 64 or sat(c, o) is None)
                                for o in solid)
                    if clear:
                        c.ghost = False
            for c in solid:
                hit, pos = wall_collide(c)
                if hit > 3.0:
                    c.collisions += 1
                    self.sparks.append({"x": pos[0], "y": pos[1], "p": min(1, hit / 20)})
                    if hit > 10:
                        event("crash", "%s hit the wall (%.0f km/h)" % (c.name, hit * 3.6), car=c.id)
            self.t += dt

    def rank(self):
        cars = sorted(self.cars.values(), key=lambda c: (-c.laps, -c.dist))
        for i, c in enumerate(cars):
            c.position = i + 1
        return cars

    def snapshot(self):
        now = time.perf_counter()
        cached = getattr(self, "_snap", None)
        if cached and now - cached[0] < 1.0 / 45:      # every viewer shares one snapshot per frame
            return dict(cached[1])
        snap = self._snapshot(now)
        self._snap = (now, snap)
        return dict(snap)

    def _snapshot(self, now):
        with self.lock:
            cars = self.rank()
            for c in cars:                              # lidar rays for the UI only
                if now - getattr(c, "lidar_at", 0) > 0.1:
                    c.lidar_at = now
                    lidar_scan(c, cars)
            out = []
            for c in cars:
                w = c.worker
                with w.lock:
                    err, log, ms, alive = w.error, list(w.log[-6:]), w.ms, w.alive
                out.append({
                    "id": c.id, "name": c.name, "owner": c.owner, "num": c.num, "color": c.color, "slot": c.slot, "pos": c.position,
                    "x": round(c.x, 3), "y": round(c.y, 3), "h": round(c.h, 4), "v": round(c.speed, 2),
                    "thr": round(c.throttle, 2), "brk": round(c.brake, 2), "str": round(c.steer, 3),
                    "drs": round(c.drs * 100), "laps": c.laps, "s": round(c.s, 1), "dist": round(c.dist, 1),
                    "lt": round(self.t - c.lap_start, 2) if c.lap_start is not None else None,
                    "last": c.last_lap, "best": c.best_lap, "tyre": c.compound, "next": c.next_tyre,
                    "wear": round(c.wear, 4), "grip": round(tyre_grip(c.compound, c.wear), 3),
                    "dmg": round(c.damage, 3), "st": c.status, "slide": c.sliding, "slip": round(c.slip, 1),
                    "ghost": c.ghost, "surf": c.surface, "pitreq": c.pit_req, "stops": c.pit_stops,
                    "glat": round(c.a_lat / 9.81, 2), "glong": round(c.a_long / 9.81, 2),
                    "err": bool(err), "ms": round(ms, 2), "alive": alive,
                    "lidar": [round(d, 1) for d in c.lidar],
                    "lhit": "".join({"edge": "e", "wall": "w", "car": "c"}.get(k, "-") for k in c.lidar_hit),
                })
            sparks, self.sparks = self.sparks, []
            return {"t": round(self.t, 3), "paused": self.paused, "speed": self.speed, "rate": round(self.rate, 2), "start_at": self.start_at, "now": time.time(), "cars": out, "sparks": sparks}

    def run(self):
        """Fixed-step physics at PHYS_HZ, run self.speed times faster than real time when the CPU allows.
        self.rate is the speed actually achieved (shown in the UI when it falls short)."""
        dt = 1.0 / PHYS_HZ
        last = time.perf_counter()
        acc = 0.0
        win_real, win_sim = 0.0, 0.0
        while True:
            now = time.perf_counter()
            frame = min(now - last, 0.1)
            last = now
            t_before = self.t
            if not self.paused:
                acc += frame * self.speed
                budget_end = now + 0.025               # never hog the loop for more than 25 ms
                while acc >= dt and time.perf_counter() < budget_end:
                    try:
                        self.step(dt)
                    except Exception:
                        traceback.print_exc()
                    acc -= dt
                acc = min(acc, 0.25 * self.speed)     # can't keep up: drop the backlog instead of spiralling
            win_real += frame
            win_sim += self.t - t_before
            if win_real >= 1.0:
                self.rate = 0.0 if self.paused else win_sim / win_real
                win_real = win_sim = 0.0
            time.sleep(0.001)


STATS = {}
SIM = Sim()


# ---------------- persistence ----------------
def load_json(path, default):
    try:
        with open(path) as f:
            return json.load(f)
    except Exception:
        return default


def save_json(path, obj):
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(obj, f, indent=1)
    os.replace(tmp, path)


LIVE_FIELDS = ("x", "y", "h", "vx", "vy", "status", "pit_u", "pit_v", "pit_blend", "box_timer", "s", "idx", "lat",
               "wear", "compound", "next_tyre", "damage", "ghost", "slot", "cp", "drs", "dist")


FROZEN = threading.Event()   # set while an import replaces the data files; nothing may overwrite them


def save_live():
    if FROZEN.is_set():
        return
    with SIM.lock:
        cars = {}
        for c in SIM.cars.values():
            d = {k: getattr(c, k) for k in LIVE_FIELDS}
            d["lap_age"] = (SIM.t - c.lap_start) if c.lap_start is not None else None
            cars[c.name] = d
    save_json(LIVE_FILE, {"saved_at": time.time(), "cars": cars, "speed": SIM.speed, "paused": SIM.paused,
                          "start_at": SIM.start_at, "grid_id": GRID_ID[0]})


def restore_live(car):
    """Put a car back exactly where it was before the last restart (if that was recent)."""
    d = LIVE.pop(car.name, None)   # use once
    if not d:
        return False
    for k in LIVE_FIELDS:
        if k in d and k != "slot":
            setattr(car, k, d[k])
    car.lap_start = (SIM.t - d["lap_age"]) if d.get("lap_age") is not None else None
    if car.status == "track":
        car.idx, car.s, car.lat = track.global_project(car.x, car.y)   # re-find where on the lap it is
        car.update_track_info()
    if "cp" not in d:   # checkpoints already passed this lap
        car.cp = [car.s > f * L + 40 for f in (0.25, 0.5, 0.75)]
    car.ghost = True   # becomes solid again as soon as it's clear of other cars
    if car.status != "track":
        car.status = "boxed" if car.status == "boxed" else car.status
    return True


def save_stats():
    if FROZEN.is_set():
        return
    with SIM.lock:
        for c in SIM.cars.values():
            STATS[c.name] = {"laps": c.laps, "best": c.best_lap, "last": c.last_lap, "stops": c.pit_stops,
                             "collisions": c.collisions, "dist": round(c.dist, 1)}
    save_json(STATS_FILE, STATS)


def stats_saver():
    while True:
        time.sleep(5)
        try:
            save_stats()
            save_live()
        except Exception:
            traceback.print_exc()


# ---------------- accounts ----------------
def hash_pw(pw, salt):
    return hashlib.pbkdf2_hmac("sha256", pw.encode(), bytes.fromhex(salt), 200_000).hex()


def check_pw(user, pw):
    acc = ACCOUNTS.get(user)
    if not acc:
        hash_pw(pw, "00" * 16)  # same work either way
        return False
    return hmac.compare_digest(hash_pw(pw, acc["salt"]), acc["hash"])


SESS_LOCK = threading.Lock()
FAILS = {}


def session_user(handler):
    auth = handler.headers.get("Authorization", "")
    tok = auth[7:].strip() if auth.startswith("Bearer ") else ""
    if not tok:
        return None
    with SESS_LOCK:
        return SESSIONS.get(hashlib.sha256(tok.encode()).hexdigest())


def bot_path(user):
    return os.path.join(BOT_DIR, user + ".py")


def deploy_script(user, code):
    """Write the user's script and swap it into their car immediately (first upload creates the car)."""
    fp = bot_path(user)
    with open(fp, "w") as f:
        f.write(code)
    if SIM.team(user):
        SIM.reload_team(user)
    SIM.add_team(user, fp)   # tops the team up to CARS_PER_PLAYER (no-op once they have all of them)


# ---------------- bot folder watcher (also picks up local edits to data/bots/<user>.py) ----------------
def scan_bots():
    os.makedirs(BOT_DIR, exist_ok=True)
    while True:
        try:
            for name in ACCOUNTS:
                path = bot_path(name)
                if not os.path.exists(path):
                    continue
                team = SIM.team(name)
                if len(team) < CARS_PER_PLAYER:
                    SIM.add_team(name, path)
                elif os.path.getmtime(path) != team[0].mtime:
                    SIM.reload_team(name)
        except Exception:
            traceback.print_exc()
        time.sleep(0.5)


# ---------------- HTTP ----------------
class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):
        pass

    def _cors(self):
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Headers", "Authorization, Content-Type")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")

    def _send(self, code, body, ctype="application/json"):
        if isinstance(body, (dict, list)):
            body = json.dumps(body).encode()
        elif isinstance(body, str):
            body = body.encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self._cors()
        self.end_headers()
        self.wfile.write(body)

    def _json(self):
        n = int(self.headers.get("Content-Length", 0) or 0)
        if n > MAX_CODE + 10_000:
            raise ValueError("request too large")
        raw = self.rfile.read(n).decode("utf-8", "replace") if n else ""
        return json.loads(raw or "{}")

    def do_OPTIONS(self):
        self.send_response(204)
        self._cors()
        self.send_header("Content-Length", "0")
        self.end_headers()

    def do_GET(self):
        path = urlparse(self.path).path
        if path == "/":
            path = "/index.html"
        if path == "/api/track":
            return self._send(200, GEOM)
        if path == "/api/params":
            return self._send(200, {"params": P, "tyres": TYRES})
        if path == "/api/stream":
            return self.stream()
        if path == "/api/ws":
            return self.websocket()
        if path == "/api/snapshot":   # polling fallback: ?since=<last event id>
            q = parse_qs(urlparse(self.path).query)
            since = int(num(q.get("since", ["0"])[0]))
            snap, _ = live_payload(since if since > 0 else max(0, EV_ID[0] - 30))
            return self._send(200, snap)
        if path == "/api/template":
            with open(TEMPLATE_FILE) as f:
                return self._send(200, {"code": f.read()})
        if path == "/api/me":
            user = session_user(self)
            return self._send(200, {"user": user, "admin": bool(user and ACCOUNTS.get(user, {}).get("admin")),
                                    "has_bot": bool(user and os.path.exists(bot_path(user)))})
        if path in ("/api/my/bot", "/api/my/status"):
            user = session_user(self)
            if not user:
                return self._send(401, {"error": "log in first"})
            if path == "/api/my/bot":
                fp = bot_path(user)
                code = open(fp).read() if os.path.exists(fp) else None
                return self._send(200, {"code": code})
            out = {}
            for car in SIM.team(user):
                w = car.worker
                with w.lock:
                    out[car.name] = {"err": w.error, "log": list(w.log), "ms": w.ms, "alive": w.alive}
            return self._send(200, {"running": bool(out), "cars": out})
        if any(part.startswith(".") for part in path.split("/")):
            return self._send(404, {"error": "not found"})
        if path == "/config.js" and not os.path.exists(os.path.join(STATIC, "config.js")):
            return self._send(200, 'window.PITWALL_API = "";\n', "application/javascript")
        fp = os.path.normpath(os.path.join(STATIC, path.lstrip("/")))
        if fp.startswith(STATIC) and os.path.isfile(fp):
            ctype = {"html": "text/html", "js": "application/javascript", "css": "text/css",
                     "svg": "image/svg+xml", "png": "image/png"}.get(fp.rsplit(".", 1)[-1], "application/octet-stream")
            with open(fp, "rb") as f:
                return self._send(200, f.read(), ctype + ("; charset=utf-8" if ctype.startswith(("text", "application/j")) else ""))
        self._send(404, {"error": "not found"})

    def _import(self):
        """One-time move of a whole PITWALL data folder onto this server (e.g. Mac -> Railway).
        Only works when PITWALL_IMPORT_TOKEN is set in this server's environment."""
        want = os.environ.get("PITWALL_IMPORT_TOKEN", "")
        got = self.headers.get("X-Import-Token", "")
        if len(want) < 24 or not hmac.compare_digest(want, got):
            return self._send(403, {"error": "import disabled"})
        n = int(self.headers.get("Content-Length", 0) or 0)
        if n > 20_000_000:
            return self._send(413, {"error": "too large"})
        try:
            files = json.loads(self.rfile.read(n).decode())["files"]
            ok = {}
            for name, b64 in files.items():
                if not (name in ("accounts.json", "sessions.json", "stats.json", "live.json")
                        or (name.startswith("bots/") and name.count("/") == 1 and name.endswith(".py")
                            and all(ch.isalnum() or ch in "_-." for ch in name[5:]))):
                    return self._send(400, {"error": "unexpected file %s" % name})
                ok[name] = base64.b64decode(b64)
        except Exception as e:
            return self._send(400, {"error": "bad import: %s" % e})
        FROZEN.set()
        time.sleep(0.3)   # let any in-flight save finish
        for name, raw in ok.items():
            fp = os.path.join(DATA_DIR, name)
            os.makedirs(os.path.dirname(fp), exist_ok=True)
            with open(fp + ".tmp", "wb") as f:
                f.write(raw)
            os.replace(fp + ".tmp", fp)
        self._send(200, {"ok": True, "files": sorted(ok), "restarting": True})
        print("import: %d files written, restarting" % len(ok), flush=True)
        threading.Timer(0.5, lambda: os._exit(3)).start()   # the host restarts us and we load the new data

    def do_POST(self):
        path = urlparse(self.path).path
        if path == "/api/admin/import":
            return self._import()
        try:
            data = self._json()
        except Exception as e:
            return self._send(400, {"error": "bad request: %s" % e})
        if path == "/api/login":
            user = str(data.get("user", "")).strip().lower()
            now = time.time()
            fails = [t for t in FAILS.get(user, []) if now - t < 60]
            if len(fails) >= 8:
                return self._send(429, {"error": "too many attempts, wait a minute"})
            if not check_pw(user, str(data.get("password", ""))):
                FAILS[user] = fails + [now]
                return self._send(401, {"error": "wrong username or password"})
            tok = secrets.token_urlsafe(32)
            with SESS_LOCK:
                SESSIONS[hashlib.sha256(tok.encode()).hexdigest()] = user
                save_json(SESSIONS_FILE, SESSIONS)
            return self._send(200, {"token": tok, "user": user, "admin": bool(ACCOUNTS[user].get("admin"))})
        if path == "/api/logout":
            auth = self.headers.get("Authorization", "")
            if auth.startswith("Bearer "):
                with SESS_LOCK:
                    SESSIONS.pop(hashlib.sha256(auth[7:].strip().encode()).hexdigest(), None)
                    save_json(SESSIONS_FILE, SESSIONS)
            return self._send(200, {"ok": True})

        user = session_user(self)
        if not user:
            return self._send(401, {"error": "log in first"})
        if path == "/api/my/bot":
            code = data.get("code", "")
            if not isinstance(code, str) or not code.strip():
                return self._send(400, {"error": "empty script"})
            if len(code) > MAX_CODE:
                return self._send(400, {"error": "script too large (200 KB max)"})
            try:
                compile(code, user + ".py", "exec")
            except SyntaxError as e:
                return self._send(400, {"error": "SyntaxError line %s: %s" % (e.lineno, e.msg)})
            deploy_script(user, code)
            return self._send(200, {"ok": True})
        if path == "/api/my/box":
            for car in SIM.team(user):
                if data.get("id") in (None, car.id):
                    with SIM.lock:
                        car.put_in_box(wait=2.0)
                    event("tow", "%s reset to pit box" % car.name, car=car.id)
            return self._send(200, {"ok": True})
        if path == "/api/control":
            if not ACCOUNTS.get(user, {}).get("admin"):
                return self._send(403, {"error": "admin only"})
            act = data.get("action")
            with SIM.lock:
                if act == "pause":
                    SIM.paused = True
                elif act == "resume":
                    SIM.paused = False
                elif act == "speed":
                    SIM.speed = clamp(num(data.get("value", 1), 1), MIN_SPEED, MAX_SPEED)
                elif act == "box":
                    car = next((c for c in SIM.cars.values() if c.id == data.get("id")), None)
                    if car:
                        car.put_in_box(wait=2.0)
                        event("tow", "%s reset to pit box" % car.name, car=car.id)
                elif act == "grid_start":
                    at = parse_when(data.get("at", time.time() + 60))
                    threading.Thread(target=SIM.grid_start, args=(at,), daemon=True).start()
                elif act == "rebox_all":
                    for c in SIM.cars.values():
                        c.put_in_box(wait=1.0 + 0.8 * c.slot)
                    event("reset", "Race control: all cars back to the pits")
            return self._send(200, {"ok": True})
        self._send(404, {"error": "not found"})

    def websocket(self):
        """Minimal send-only WebSocket (RFC 6455). Works through Cloudflare quick tunnels, unlike SSE."""
        key = self.headers.get("Sec-WebSocket-Key")
        if not key or "websocket" not in self.headers.get("Upgrade", "").lower():
            return self._send(400, {"error": "websocket upgrade required"})
        accept = base64.b64encode(hashlib.sha1((key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode()).digest()).decode()
        self.send_response(101, "Switching Protocols")
        self.send_header("Upgrade", "websocket")
        self.send_header("Connection", "Upgrade")
        self.send_header("Sec-WebSocket-Accept", accept)
        self.end_headers()
        self.close_connection = True
        sock = self.connection
        sock.settimeout(None)
        closed = threading.Event()

        def reader():   # drain client frames; answer pings, stop on close
            try:
                while not closed.is_set():
                    hdr = self.rfile.read(2)
                    if len(hdr) < 2:
                        break
                    op, ln = hdr[0] & 0x0F, hdr[1] & 0x7F
                    if ln == 126:
                        ln = int.from_bytes(self.rfile.read(2), "big")
                    elif ln == 127:
                        ln = int.from_bytes(self.rfile.read(8), "big")
                    mask = self.rfile.read(4) if hdr[1] & 0x80 else b"\0\0\0\0"
                    data = bytes(b ^ mask[i % 4] for i, b in enumerate(self.rfile.read(ln)))
                    if op == 0x8:
                        break
                    if op == 0x9:
                        send(data, 0xA)
            except Exception:
                pass
            closed.set()

        lock = threading.Lock()

        def send(payload, op=0x1):
            n = len(payload)
            head = bytes([0x80 | op]) + (bytes([n]) if n < 126 else bytes([126]) + n.to_bytes(2, "big") if n < 65536
                                          else bytes([127]) + n.to_bytes(8, "big"))
            with lock:
                sock.sendall(head + payload)

        threading.Thread(target=reader, daemon=True).start()
        last_ev = max(0, EV_ID[0] - 30)
        try:
            while not closed.is_set():
                snap, last_ev = live_payload(last_ev)
                send(json.dumps(snap, separators=(",", ":")).encode())
                time.sleep(1.0 / NET_HZ)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        closed.set()

    def stream(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache, no-transform")   # no-transform: proxies must not gzip/buffer
        self.send_header("X-Accel-Buffering", "no")
        self.send_header("Transfer-Encoding", "chunked")
        self._cors()
        self.end_headers()
        with EV_LOCK:
            last_ev = max(0, EV_ID[0] - 30)
        try:
            n = 0
            while True:
                snap = SIM.snapshot()
                with EV_LOCK:
                    evs = [e for e in EVENTS if e["id"] > last_ev]
                    if evs:
                        last_ev = evs[-1]["id"]
                snap["events"] = evs
                chunk = ("data: " + json.dumps(snap, separators=(",", ":")) + "\n\n").encode()
                self.wfile.write(b"%x\r\n%s\r\n" % (len(chunk), chunk))
                self.wfile.flush()
                n += 1
                time.sleep(1.0 / NET_HZ)
        except (BrokenPipeError, ConnectionResetError, OSError):
            return


def live_payload(last_ev):
    snap = SIM.snapshot()
    with EV_LOCK:
        evs = [e for e in EVENTS if e["id"] > last_ev]
        if evs:
            last_ev = evs[-1]["id"]
    snap["events"] = evs
    snap["ev"] = last_ev
    return snap, last_ev


ACCOUNTS = {}
SESSIONS = {}
LIVE = {}
SLOTS = {}
GRID_ID = [""]   # PITWALL_GRID_START value already applied (so a redeploy doesn't reset laps again)


def parse_when(v):
    v = str(v).strip()
    try:
        return float(v)
    except ValueError:
        import datetime
        return datetime.datetime.fromisoformat(v.replace("Z", "+00:00")).timestamp()


def pending_grid_start(at):
    """Apply a grid start once the cars have been loaded."""
    time.sleep(4)
    SIM.grid_start(at)


def main():
    os.makedirs(BOT_DIR, exist_ok=True)
    if _linux_bot_user():
        os.chmod(DATA_DIR, 0o700)   # bots run as another user and must not read this
    ACCOUNTS.update(load_json(ACCOUNTS_FILE, {}))
    STATS.update(load_json(STATS_FILE, {}))
    for k in [k for k in STATS if "#" not in k]:          # stats from the one-car-per-player days -> car #1
        STATS.setdefault(k + "#1", STATS[k])
        del STATS[k]
    SLOTS.update(load_json(SLOTS_FILE, {}))
    live = load_json(LIVE_FILE, {})
    SIM.start_at = float(live.get("start_at", 0) or 0)
    GRID_ID[0] = live.get("grid_id", "")
    if time.time() - live.get("saved_at", 0) < 600:        # only resume positions after a quick restart
        LIVE.update(live.get("cars", {}))
        SIM.speed = clamp(num(live.get("speed", 1.0), 1.0), MIN_SPEED, MAX_SPEED)
        SIM.paused = bool(live.get("paused", False))
    signal.signal(signal.SIGTERM, lambda *a: (_ for _ in ()).throw(KeyboardInterrupt()))
    if not ACCOUNTS:
        print("No accounts yet. Run:  python3 manage.py init", flush=True)
    SESSIONS.update({k: v for k, v in load_json(SESSIONS_FILE, {}).items() if v in ACCOUNTS})
    threading.Thread(target=SIM.run, daemon=True).start()
    threading.Thread(target=scan_bots, daemon=True).start()
    threading.Thread(target=stats_saver, daemon=True).start()
    want = os.environ.get("PITWALL_GRID_START", "").strip()
    if want and want != GRID_ID[0]:
        GRID_ID[0] = want
        SIM.start_at = parse_when(want)    # hold everyone right away; laps reset once cars are loaded
        threading.Thread(target=pending_grid_start, args=(SIM.start_at,), daemon=True).start()
    host = os.environ.get("HOST", "127.0.0.1")
    srv = ThreadingHTTPServer((host, PORT), Handler)
    srv.daemon_threads = True
    print("PITWALL running on http://localhost:%d" % PORT, flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        save_stats()
        save_live()
        for c in list(SIM.cars.values()):
            c.worker.stop()


if __name__ == "__main__":
    main()
