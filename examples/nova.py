"""PITWALL bot "nova". One file = one car. Save the file (or hit Deploy in the UI) and it hot-reloads.

drive(state) is called ~20x per second and must return a dict of controls:
    throttle  -1..1   (negative = reverse, slow)
    brake      0..1
    steer     -1..1   (+ = left, - = right)
    drs        0..100 (% rear-wing open: less drag => more top speed, but less downforce => less corner grip)
    pit        bool   (True = take the pit lane next time you pass the entry; the pit crew autopilot does the rest)
    tyre       "soft" | "medium" | "hard"   (compound fitted at your next stop)

state (all metres / seconds / radians, world y-up, headings CCW from +x):
    state["me"]     your car: x, y, heading, speed, vx, vy, v_long, v_lat, yaw_rate, accel_long, accel_lat,
                    throttle, brake, steer, drs, s (metres along the lap), lap_progress (0..1),
                    lateral_offset (+ left of centerline), track_heading, heading_error,
                    surface ("asphalt"|"kerb"|"grass"), on_track, sliding, slip_angle_deg,
                    laps, lap_time, last_lap, best_lap, position,
                    tyre, tyre_wear (0 new .. 1 dead), tyre_grip_factor, grip_now (m/s^2 available),
                    damage (0..1), status ("track"|"pit_in"|"boxed"|"pit_out"), in_pit, ghost,
                    distance_to_pit_entry, pit_requested, pit_stops, collisions,
                    hitbox {length, width, corners [[x,y]*4]}
    state["cars"]   every other car with the same public fields + rel_x (ahead +), rel_y (left +),
                    distance, track_gap (along-track metres, + = they are ahead)
    state["lidar"]  36 rays, 10 deg apart, angle relative to your heading (0 = straight ahead, + = left):
                    {"angle", "dist" (<=150), "hit": "edge"|"wall"|"car"|"none", "car_id"}
    state["ahead"]  centerline samples every 10 m for the next 400 m: {"d", "x", "y", "heading", "curvature"}
    state["track"]  full static track: centerline, left/right edges, walls, curvature, pit lane, boxes, length...
    state["params"] car physics constants (mass, power, mu, k_down, k_drag, wheelbase, max_steer, drs cuts, ...)
    state["t"], state["dt"], state["tick"]

Grip you have right now (m/s^2) is roughly:
    mu * surface * tyre_grip_factor * (g + k_down * (1 - drs_down_cut*drs) * (1 - 0.35*damage) * v^2)
Ask the tyres for more than that and the car slides (and wears tyres fast).

print() works — output shows in the UI log for your car.
Module-level variables persist between calls (until you redeploy).
"""
import math

# ---- tuning knobs ----
GRIP_USE = 0.90       # fraction of the grip limit to drive at
BRAKE_USE = 0.85      # fraction of grip to plan braking with
LINE_GAIN = 450.0     # how hard to cut toward the apex (0 = drive the centerline)
PIT_WEAR = 0.78       # pit when tyre wear passes this
STINT_TYRE = "hard"

P = {}


def init(track, params):
    P.update(params)


def corner_speed(k, grip_factor):
    mu = P["mu"] * grip_factor * GRIP_USE
    den = abs(k) - mu * P["k_down"]
    if den <= 1e-6:
        return 120.0
    return min(120.0, math.sqrt(mu * P["g"] / den))


def drive(state):
    me = state["me"]
    ahead = state["ahead"]
    v = me["speed"]
    surf = P["surface"].get(me["surface"], 1.0)
    gf = me["tyre_grip_factor"] * (1 - 0.3 * me["damage"]) * (surf if surf < 0.9 else 1.0)

    # ---------- target speed: corner limits, then a backward braking pass ----------
    # (braking capacity shrinks when the tyres are also busy turning - friction circle)
    mu = P["mu"] * gf
    vmax = [corner_speed(p["curvature"], gf) for p in ahead]
    for i in range(len(ahead) - 2, -1, -1):
        vn = vmax[i + 1]
        g_av = mu * BRAKE_USE * (P["g"] + P["k_down"] * vn * vn)
        lat = vn * vn * abs(ahead[i + 1]["curvature"])
        a_b = math.sqrt(max(g_av * g_av - lat * lat, 0.15 * g_av * g_av)) + P["k_drag"] * vn * vn
        vmax[i] = min(vmax[i], math.sqrt(vn * vn + 2 * a_b * (ahead[i + 1]["d"] - ahead[i]["d"])))
    lead = min(1.0, v * 0.12 / 10.0)        # look ~0.12 s ahead to cover control latency
    target = vmax[0] + (vmax[1] - vmax[0]) * lead

    # ---------- racing line + traffic ----------
    look = max(12.0, min(48.0, 9.0 + 0.36 * v))
    i = min(len(ahead) - 2, int(look // 10))
    u = (look - ahead[i]["d"]) / 10.0
    pa, pb = ahead[i], ahead[i + 1]
    tx = pa["x"] + (pb["x"] - pa["x"]) * u
    ty = pa["y"] + (pb["y"] - pa["y"]) * u
    th = pa["heading"]
    k_here = max((p["curvature"] for p in ahead[i:i + 4]), key=abs)
    offset = max(-5.0, min(5.0, k_here * LINE_GAIN))

    for o in state["cars"]:
        if o["in_pit"] or o["ghost"]:
            continue
        if 0 < o["rel_x"] < 40 and abs(o["rel_y"]) < 4.0:
            closing = v - o["speed"]
            if closing > -1:
                side = 1 if o["lateral_offset"] < 0 else -1      # pass on the side with more room
                offset = max(-5.5, min(5.5, o["lateral_offset"] + side * 4.2))
            if o["rel_x"] < 14 and abs(o["rel_y"]) < 2.4:
                target = min(target, o["speed"] - 1.0)          # don't rear-end them

    tx += -math.sin(th) * offset
    ty += math.cos(th) * offset

    # ---------- pure pursuit steering ----------
    c, s = math.cos(me["heading"]), math.sin(me["heading"])
    dx, dy = tx - me["x"], ty - me["y"]
    fx = dx * c + dy * s
    fy = -dx * s + dy * c
    curv = 2 * fy / max(1.0, fx * fx + fy * fy)
    steer = math.atan(curv * P["wheelbase"]) / P["max_steer"]

    if abs(me["heading_error"]) > 1.3:      # facing the wrong way: turn around slowly
        target = min(target, 10.0)

    # ---------- pedals ----------
    if v < target - 0.8:
        throttle, brake = 1.0, 0.0
    elif v > target + 0.3:
        throttle, brake = 0.0, min(1.0, (v - target) / 2.0)
    else:
        throttle, brake = 0.5, 0.0
    # trail-brake: never ask the tyres for more than the friction circle allows
    grip = me["grip_now"]
    lat_need = v * v * abs(ahead[1]["curvature"])
    brake_cap = math.sqrt(max(0.0, grip * grip - lat_need * lat_need)) / (P["brake_limit"] * grip + 1e-6)
    brake = min(brake, max(0.15, brake_cap * 0.95))
    if me["sliding"]:                       # catch the slide: countersteer, off the pedals
        steer += 0.08 * me["v_lat"]
        throttle, brake = min(throttle, 0.2), min(brake, 0.3)

    # ---------- DRS: open on straights only ----------
    horizon = max(60.0, v * 1.2)
    straight = all(abs(p["curvature"]) < 1 / 600 for p in ahead if p["d"] <= horizon)
    drs = 100 if straight and brake == 0 else 0

    # ---------- strategy ----------
    pit = me["tyre_wear"] > PIT_WEAR or me["damage"] > 0.45

    return {"throttle": throttle, "brake": brake, "steer": max(-1, min(1, steer)),
            "drs": drs, "pit": pit, "tyre": STINT_TYRE}
