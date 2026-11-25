# PITWALL: bot racing with Python

An F1-style racing sim where **every car is driven by a Python script**. Each player has an account and
a team of **3 cars that all run the same script**. Your first upload puts all three on track, each from its
own pit box. Every upload after that takes over all three instantly, mid-lap. The race never stops, and
lap counts are all-time totals, per car and per team. Restarting the server resumes every car where it was.

![cars, lidar, pit lane, standings](https://img.shields.io/badge/physics-grip%20%C2%B7%20slides%20%C2%B7%20DRS%20%C2%B7%20tyres%20%C2%B7%20collisions-ff2d3f)

## Playing

1. Open the race site and click **Log in** (the host gives you your username and password).
2. Click **My script**. It opens a working starter bot ([`examples/template.py`](examples/template.py)).
   Tweak it, then hit **Deploy** (or ⌘S). Or use **Upload .py** to send a file from your computer.
3. Your 3 cars leave the pits on your first deploy. Every deploy after that hot-swaps the code on all of
   them; laps, positions and tyres carry over. `state["me"]["car_number"]` (1–3) lets one script give each
   car a different job, and `teammate` marks your other cars in `state["cars"]`.
4. Click your car to see its telemetry, onboard camera, lidar and your `print()` output.
   Only you can see your own logs and code.

## Writing a bot

Your file defines `drive(state) -> dict`. It's called about 20× per second:

```python
def drive(state):
    me = state["me"]
    return {
        "throttle": 1.0,   # -1..1  (negative = slow reverse)
        "brake": 0.0,      # 0..1
        "steer": 0.0,      # -1..1  (+ = left)
        "drs": 0,          # 0..100 %: less drag (faster straights), less downforce (slower corners)
        "pit": False,      # True = take the pit lane next time past the entry; the crew autopilot does the rest
        "tyre": "medium",  # soft | medium | hard, fitted at your next stop
    }
```

`state` has everything that's happening right now:
- **`me`**: position, heading, speed, acceleration, current inputs, lap and lap times, tyre wear and grip,
  damage, whether you're sliding, and your hitbox.
- **`cars`**: every other car, including where it is relative to you and its hitbox.
- **`lidar`**: a 36-ray scan around your car that says whether each ray hit the track edge, a wall or a car.
- **`ahead`**: the next 400 m of track, with curvature.
- **`track`**: the full map, including the pit lane and boxes.
- **`params`**: the physics constants.

The full list is at the top of [`examples/template.py`](examples/template.py).
[`examples/`](examples) also has four tuned sample bots.

**Physics, briefly:**
- **Grip:** grip = μ × surface × tyre × (g + downforce). Downforce grows with speed².
- **Friction circle:** braking or throttle uses up some of the grip you have for turning.
- **Sliding:** ask for more grip than you have and the car slides. Grip drops, tyres wear fast, and you can spin.
- **DRS** trades corner grip for straight-line speed.
- **Tyres:** softs grip most but wear fastest. All compounds fall off a cliff past ~75% wear, so you need to pit.
- **Grass** gives half grip.
- **Collisions:** car-to-car and wall hits are real impacts. They push you around and cause damage.
- **Pit stops:** return `pit: True` and the pit crew handles everything.
- **Stuck cars** get towed to their box after 10 s.

`print()` works. Module-level variables persist between calls until you redeploy.
`numpy` and the standard library are available. Bots run sandboxed: no network, no file access.

## Hosting your own race (use this repo as a template)

Needs Python 3.9+ with `numpy`. Bot sandboxing uses macOS `sandbox-exec`. On other OSes bots run
unsandboxed, so only host people you trust.

```bash
pip install numpy
# edit the PLAYERS list in manage.py, then:
python3 manage.py init      # creates accounts; passwords land in data/ACCOUNTS.local.txt
python3 server.py           # http://localhost:8765
```

- **Making it public from your own machine:** `./go_live.sh` keeps your Mac awake, runs the server, and opens a
  free [Cloudflare quick tunnel](https://developers.cloudflare.com/cloudflare-one/connections/connect-networks/do-more-with-tunnels/trycloudflare/).
  It then publishes the tunnel address to a secret gist that the static site (in `static/`) reads.
  Run `./go_live.sh --deploy` once from a terminal to put the site on Vercel. Re-run `./go_live.sh`
  after a reboot.
- **Accounts:** `python3 manage.py add NAME` adds a player and `python3 manage.py passwd NAME` resets a
  password. Restart the server after either.
- **Saved data:** everything lives in `data/` (git-ignored): accounts as salted hashes, sessions, uploaded
  scripts, all-time stats, and car positions (saved every 5 s and on shutdown, so a quick restart resumes the
  race mid-lap). `CARS_PER_PLAYER` in `server.py` sets the team size.
- **Admin:** an admin account also gets pause and 1×/2×/4× sim speed.

Files: `server.py` (physics, collisions, lidar, pit autopilot, HTTP/SSE API), `runner.py` (per-bot
process), `track.py` (circuit geometry), `static/` (the web UI), `sandbox.sb` (bot sandbox profile).
