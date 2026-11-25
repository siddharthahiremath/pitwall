"use strict";
// ============================================================ PITWALL client
const $ = (s) => document.querySelector(s);
let API = "";   // race server base URL ("" = same origin). config.js can set PITWALL_API or PITWALL_API_SOURCE
async function resolveApi() {
  if (window.PITWALL_API) return String(window.PITWALL_API).replace(/\/$/, "");
  if (window.PITWALL_API_SOURCE) {   // a tiny JSON file ({"api": "https://..."}) the host updates when the tunnel changes
    const r = await fetch(window.PITWALL_API_SOURCE + (window.PITWALL_API_SOURCE.includes("?") ? "&" : "?") + "t=" + Date.now(), { cache: "no-store" });
    return String((await r.json()).api || "").replace(/\/$/, "");
  }
  return "";
}
const store = { get(k) { try { return localStorage.getItem(k); } catch { return null; } },
                set(k, v) { try { v == null ? localStorage.removeItem(k) : localStorage.setItem(k, v); } catch {} } };
let auth = { token: store.get("pw_token"), user: null, admin: false };
const TYRE_COL = { soft: "#ff3b30", medium: "#ffd60a", hard: "#f2f2f2" };
const HIT_COL = { e: "#3ee6ff", w: "#ffa53e", c: "#ff4fd8", "-": "rgba(255,255,255,0.10)" };
const CAR_L = 5.6, CAR_W = 2.0;

let T = null;                 // track geometry
let paths = null;             // Path2D cache (world coords)
const snaps = [];             // snapshot buffer
let latest = null;
let selected = null;          // car id
const opts = { follow: false, lidar: true, hitbox: false, labels: true };
const view = { cx: 0, cy: 0, z: 1, fitted: false };
let clock = { offset: 0, rate: 1, init: false };
let skids = [];               // {x1,y1,x2,y2,t}
const lastRear = new Map();
let sparks = [];
let lastBoard = 0;
const knownEvents = new Set();

const world = $("#world"), wctx = world.getContext("2d");
const cam = $("#cam"), cctx = cam.getContext("2d");

// ------------------------------------------------------------ utils
const clamp = (v, a, b) => (v < a ? a : v > b ? b : v);
const lerp = (a, b, u) => a + (b - a) * u;
function lerpAngle(a, b, u) { let d = ((b - a + Math.PI) % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI) - Math.PI; return a + d * u; }
function fmt(t) { if (t == null) return "—"; const m = Math.floor(t / 60); const s = t - m * 60; return m + ":" + s.toFixed(3).padStart(6, "0"); }
function fmtClock(t) { const m = Math.floor(t / 60); return m + ":" + (t - 60 * m).toFixed(1).padStart(4, "0"); }
function esc(s) { return String(s).replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[c])); }

function fitCanvas(c) {
  const dpr = window.devicePixelRatio || 1;
  const r = c.getBoundingClientRect();
  const w = Math.max(1, Math.round(r.width * dpr)), h = Math.max(1, Math.round(r.height * dpr));
  if (c.width !== w || c.height !== h) { c.width = w; c.height = h; }
  return { w: r.width, h: r.height, dpr };
}

// ------------------------------------------------------------ track paths
function polyPath(pts, closed = true) {
  const p = new Path2D();
  p.moveTo(pts[0][0], pts[0][1]);
  for (let i = 1; i < pts.length; i++) p.lineTo(pts[i][0], pts[i][1]);
  if (closed) p.closePath();
  return p;
}
function offsetPts(i0, i1, off) {
  const out = [], n = T.centerline.length;
  for (let k = i0; k <= i1; k++) {
    const i = ((k % n) + n) % n, h = T.heading[i];
    out.push([T.centerline[i][0] - Math.sin(h) * off, T.centerline[i][1] + Math.cos(h) * off]);
  }
  return out;
}
function buildPaths() {
  const n = T.centerline.length;
  paths = {};
  paths.center = polyPath(T.centerline);
  paths.left = polyPath(T.left);
  paths.right = polyPath(T.right);
  // kerbs: runs where curvature is high, on both edges
  paths.kerbs = new Path2D();
  const thr = 1 / 260;
  let i = 0;
  const hot = T.curvature.map((k) => Math.abs(k) > thr);
  // start scanning from a non-hot index so runs don't wrap
  let start = hot.findIndex((h) => !h); if (start < 0) start = 0;
  for (let c = 0; c < n; c++) {
    const idx = (start + c) % n;
    if (hot[idx] && !hot[(idx - 1 + n) % n]) {
      let len = 0; while (len < n && hot[(idx + len) % n]) len++;
      const a = idx - 3, b = idx + len + 3;
      for (const side of [1, -1]) {
        const pts = offsetPts(a, b, side * (T.half_width + T.kerb / 2));
        paths.kerbs.addPath(polyPath(pts, false));
      }
    }
  }
  // pit lane
  paths.pit = polyPath(T.pit.path, false);
  const L = T.length, P = T.pit;
  const idxAt = (s) => { s = ((s % L) + L) % L; let lo = 0, hi = n - 1; while (lo < hi) { const m = (lo + hi + 1) >> 1; if (T.s[m] <= s) lo = m; else hi = m - 1; } return lo; };
  const b0 = idxAt(P.entry_s + P.boxes[0].u - 12), b1 = idxAt(P.entry_s + P.boxes[P.boxes.length - 1].u + 12);
  const span = ((b1 - b0) % n + n) % n;
  paths.boxLane = polyPath(offsetPts(b0, b0 + span, P.box_offset), false);
  paths.garage = polyPath(offsetPts(b0, b0 + span, P.box_offset - 8.5), false);
  const w0 = idxAt(P.entry_s + 100), w1 = idxAt(P.entry_s + P.length - 100);
  const wspan = ((w1 - w0) % n + n) % n;
  paths.pitWall = polyPath(offsetPts(w0, w0 + wspan, P.lane_offset + 6.2), false);
  paths.pitLine = polyPath(offsetPts(w0, w0 + wspan, P.lane_offset - 3.2), false);
  // grass pattern
  const pc = document.createElement("canvas"); pc.width = pc.height = 64;
  const g = pc.getContext("2d");
  g.fillStyle = "#1f4a26"; g.fillRect(0, 0, 64, 64);
  g.fillStyle = "#235229"; g.fillRect(0, 0, 32, 64);
  paths.grass = g;
  paths.grassCanvas = pc;
  // bounds
  let minx = 1e9, miny = 1e9, maxx = -1e9, maxy = -1e9;
  for (const p of T.wall_left.concat(T.wall_right, T.pit.path)) { minx = Math.min(minx, p[0]); maxx = Math.max(maxx, p[0]); miny = Math.min(miny, p[1]); maxy = Math.max(maxy, p[1]); }
  paths.bounds = { minx, miny, maxx, maxy };
}

function fitView() {
  const r = world.getBoundingClientRect(), b = paths.bounds;
  const pad = 40;
  view.z = Math.min((r.width - pad * 2) / (b.maxx - b.minx), (r.height - pad * 2) / (b.maxy - b.miny));
  view.cx = (b.minx + b.maxx) / 2; view.cy = (b.miny + b.maxy) / 2;
  view.fitted = true;
}

// ------------------------------------------------------------ drawing
function drawTrack(ctx, z) {
  const px = 1 / z; // one screen pixel in metres
  // grass inside the barriers
  if (!paths.patterns) paths.patterns = new Map();
  let pattern = paths.patterns.get(ctx);
  if (!pattern) {
    pattern = ctx.createPattern(paths.grassCanvas, "repeat");
    pattern.setTransform(new DOMMatrix().rotate(35).scale(0.55));
    paths.patterns.set(ctx, pattern);
  }
  ctx.lineJoin = "round"; ctx.lineCap = "round";
  // pit lane apron (under everything so the barrier gap looks clean)
  ctx.strokeStyle = "#2a2e35"; ctx.lineWidth = 12; ctx.stroke(paths.pit);
  ctx.strokeStyle = "#2c3038"; ctx.lineWidth = 9; ctx.stroke(paths.boxLane);
  ctx.strokeStyle = "#1c2027"; ctx.lineWidth = 8; ctx.stroke(paths.garage);
  // barrier then grass
  ctx.strokeStyle = "#cfd5de"; ctx.lineWidth = 2 * T.wall + Math.max(1.4, 2 * px); ctx.stroke(paths.center);
  ctx.strokeStyle = "#7c2a2a"; ctx.lineWidth = 2 * T.wall + Math.max(0.5, 0.8 * px); ctx.stroke(paths.center);
  ctx.strokeStyle = pattern; ctx.lineWidth = 2 * T.wall; ctx.stroke(paths.center);
  // runoff tarmac strip next to the kerbs
  ctx.strokeStyle = "rgba(120,128,110,0.22)"; ctx.lineWidth = 2 * (T.half_width + T.kerb + 3); ctx.stroke(paths.center);
  // pit lane surface on top of grass
  ctx.strokeStyle = "#30343c"; ctx.lineWidth = 8; ctx.stroke(paths.pit);
  ctx.strokeStyle = "#30343c"; ctx.lineWidth = 7; ctx.stroke(paths.boxLane);
  ctx.strokeStyle = "rgba(255,255,255,0.55)"; ctx.lineWidth = Math.max(0.25, px); ctx.setLineDash([3, 3]); ctx.stroke(paths.pitLine); ctx.setLineDash([]);
  ctx.strokeStyle = "#d8dde5"; ctx.lineWidth = Math.max(0.8, 1.5 * px); ctx.stroke(paths.pitWall);
  // kerbs
  ctx.lineCap = "butt";
  ctx.strokeStyle = "#e0262f"; ctx.lineWidth = T.kerb; ctx.stroke(paths.kerbs);
  ctx.strokeStyle = "#f4f4f4"; ctx.setLineDash([2.2, 2.2]); ctx.stroke(paths.kerbs); ctx.setLineDash([]);
  ctx.lineCap = "round";
  // asphalt
  ctx.strokeStyle = "#353941"; ctx.lineWidth = 2 * T.half_width; ctx.stroke(paths.center);
  ctx.strokeStyle = "rgba(0,0,0,0.12)"; ctx.lineWidth = 5.5; ctx.stroke(paths.center); // rubbered-in line
  // white edges
  ctx.strokeStyle = "#f3f5f8"; ctx.lineWidth = Math.max(0.3, 0.9 * px);
  ctx.stroke(paths.left); ctx.stroke(paths.right);
  // pit boxes
  for (const b of T.pit.boxes) {
    ctx.save(); ctx.translate(b.x, b.y); ctx.rotate(b.heading);
    const car = latest && latest.cars.find((c) => c.slot === b.slot);
    ctx.fillStyle = car ? car.color + "55" : "rgba(255,255,255,0.05)";
    ctx.fillRect(-4, -2.2, 8, 4.4);
    ctx.strokeStyle = car ? car.color : "rgba(255,255,255,0.35)"; ctx.lineWidth = Math.max(0.18, px);
    ctx.strokeRect(-4, -2.2, 8, 4.4);
    ctx.restore();
  }
  // start / finish chequers
  const c0 = T.centerline[0], h0 = T.heading[0];
  ctx.save(); ctx.translate(c0[0], c0[1]); ctx.rotate(h0);
  const sq = 1.0, rows = 2, cols = Math.round((2 * T.half_width) / sq);
  for (let r = 0; r < rows; r++) for (let k = 0; k < cols; k++) {
    ctx.fillStyle = (r + k) % 2 ? "#111" : "#f5f5f5";
    ctx.fillRect(-r * sq - sq, -T.half_width + k * sq, sq, sq);
  }
  ctx.restore();
  // sector lines
  for (let s = 1; s < 3; s++) {
    const i = T.s.findIndex((v) => v >= T.sectors[s]);
    const c = T.centerline[i], h = T.heading[i];
    ctx.save(); ctx.translate(c[0], c[1]); ctx.rotate(h);
    ctx.fillStyle = "rgba(255,210,60,0.55)"; ctx.fillRect(-0.3, -T.half_width, 0.6, 2 * T.half_width);
    ctx.restore();
  }
}

function drawSkids(ctx, z, now) {
  ctx.lineCap = "round";
  ctx.lineWidth = 0.45;
  for (const s of skids) {
    const age = (now - s.t) / 1000;
    const a = clamp(0.55 * (1 - age / 40), 0, 0.55);
    if (a <= 0) continue;
    ctx.strokeStyle = `rgba(12,12,14,${a})`;
    ctx.beginPath(); ctx.moveTo(s.x1, s.y1); ctx.lineTo(s.x2, s.y2); ctx.stroke();
  }
}

function carPath(ctx) {
  // body silhouette, car frame: +x forward, +y left, metres
  ctx.beginPath();
  ctx.moveTo(2.45, 0.13);
  ctx.lineTo(1.0, 0.3); ctx.lineTo(0.35, 0.42); ctx.lineTo(0.1, 0.8); ctx.lineTo(-1.15, 0.78);
  ctx.lineTo(-1.75, 0.42); ctx.lineTo(-2.35, 0.28); ctx.lineTo(-2.35, -0.28); ctx.lineTo(-1.75, -0.42);
  ctx.lineTo(-1.15, -0.78); ctx.lineTo(0.1, -0.8); ctx.lineTo(0.35, -0.42); ctx.lineTo(1.0, -0.3); ctx.lineTo(2.45, -0.13);
  ctx.closePath();
}

function drawCar(ctx, c, k, isSel) {
  ctx.save();
  ctx.translate(c.x, c.y); ctx.rotate(c.h); ctx.scale(k, k);
  if (c.ghost) ctx.globalAlpha = 0.5;
  // shadow
  ctx.fillStyle = "rgba(0,0,0,0.35)";
  ctx.fillRect(-2.7, -1.05, 5.5, 2.15);
  // selection halo
  if (isSel) {
    ctx.strokeStyle = "rgba(255,255,255,0.9)"; ctx.lineWidth = 0.22;
    ctx.beginPath(); ctx.ellipse(0, 0, 3.9, 2.0, 0, 0, Math.PI * 2); ctx.stroke();
  }
  // tyres
  const tc = TYRE_COL[c.tyre] || "#999";
  ctx.fillStyle = "#121214";
  ctx.fillRect(1.35, 0.6, 0.7, 0.38); ctx.fillRect(1.35, -0.98, 0.7, 0.38);
  ctx.fillRect(-2.05, 0.56, 0.8, 0.46); ctx.fillRect(-2.05, -1.02, 0.8, 0.46);
  ctx.fillStyle = tc;
  ctx.fillRect(1.62, 0.6, 0.16, 0.38); ctx.fillRect(1.62, -0.98, 0.16, 0.38);
  ctx.fillRect(-1.73, 0.56, 0.16, 0.46); ctx.fillRect(-1.73, -1.02, 0.16, 0.46);
  // suspension arms
  ctx.strokeStyle = "#2a2a2e"; ctx.lineWidth = 0.07;
  ctx.beginPath(); ctx.moveTo(1.7, 0.6); ctx.lineTo(1.0, 0.25); ctx.moveTo(1.7, -0.6); ctx.lineTo(1.0, -0.25);
  ctx.moveTo(-1.65, 0.56); ctx.lineTo(-1.2, 0.4); ctx.moveTo(-1.65, -0.56); ctx.lineTo(-1.2, -0.4); ctx.stroke();
  // body
  carPath(ctx);
  ctx.fillStyle = c.color; ctx.fill();
  ctx.strokeStyle = "rgba(0,0,0,0.55)"; ctx.lineWidth = 0.06; ctx.stroke();
  // engine cover spine
  ctx.fillStyle = "rgba(0,0,0,0.22)"; ctx.fillRect(-2.2, -0.1, 2.0, 0.2);
  // front wing
  ctx.fillStyle = "#1b1d22"; ctx.fillRect(2.3, -0.98, 0.42, 1.96);
  ctx.fillStyle = c.color; ctx.fillRect(2.5, -0.98, 0.18, 1.96);
  // rear wing (lighter when DRS open)
  const drsOpen = c.drs > 40;
  ctx.fillStyle = "#1b1d22"; ctx.fillRect(-2.78, -0.56, 0.42, 1.12);
  ctx.fillStyle = drsOpen ? "#6fe3ff" : c.color; ctx.fillRect(-2.74, -0.52, drsOpen ? 0.12 : 0.24, 1.04);
  // cockpit + halo + helmet
  ctx.fillStyle = "#0d0e11"; ctx.beginPath(); ctx.ellipse(0.15, 0, 0.5, 0.24, 0, 0, Math.PI * 2); ctx.fill();
  ctx.fillStyle = "#f4f4f4"; ctx.beginPath(); ctx.arc(0.02, 0, 0.15, 0, Math.PI * 2); ctx.fill();
  ctx.strokeStyle = "#2b2d33"; ctx.lineWidth = 0.09;
  ctx.beginPath(); ctx.moveTo(0.6, 0); ctx.lineTo(0.15, 0.24); ctx.lineTo(-0.3, 0.2); ctx.moveTo(0.15, -0.24); ctx.lineTo(-0.3, -0.2); ctx.moveTo(0.15, 0.24); ctx.lineTo(0.15, -0.24); ctx.stroke();
  // brake light
  if (c.brk > 0.08) { ctx.fillStyle = "#ff2020"; ctx.fillRect(-2.86, -0.1, 0.1, 0.2); }
  ctx.restore();
}

function drawLidar(ctx, c, z) {
  if (!c.lidar) return;
  const n = c.lidar.length;
  ctx.lineWidth = Math.max(0.15, 1 / z);
  for (let i = 0; i < n; i++) {
    const a = c.h - Math.PI + (i * 2 * Math.PI) / n;
    const d = c.lidar[i], kind = c.lhit[i] || "-";
    const ex = c.x + Math.cos(a) * d, ey = c.y + Math.sin(a) * d;
    ctx.strokeStyle = kind === "-" ? HIT_COL["-"] : HIT_COL[kind] + "88";
    ctx.beginPath(); ctx.moveTo(c.x, c.y); ctx.lineTo(ex, ey); ctx.stroke();
    if (kind !== "-") {
      ctx.fillStyle = HIT_COL[kind];
      const r = Math.max(0.35, 2.2 / z);
      ctx.beginPath(); ctx.arc(ex, ey, r, 0, Math.PI * 2); ctx.fill();
    }
  }
}

function drawHitbox(ctx, c, z) {
  ctx.save(); ctx.translate(c.x, c.y); ctx.rotate(c.h);
  ctx.strokeStyle = c.ghost ? "rgba(182,168,255,0.9)" : "rgba(80,255,180,0.95)";
  ctx.lineWidth = Math.max(0.08, 1.2 / z);
  ctx.setLineDash(c.ghost ? [0.5, 0.4] : []);
  ctx.strokeRect(-CAR_L / 2, -CAR_W / 2, CAR_L, CAR_W);
  ctx.setLineDash([]);
  ctx.restore();
}

function drawSparks(ctx, dt) {
  ctx.save(); ctx.globalCompositeOperation = "lighter";
  for (const p of sparks) {
    p.life -= dt; p.x += p.vx * dt; p.y += p.vy * dt; p.vx *= 0.94; p.vy *= 0.94;
    if (p.life <= 0) continue;
    const a = p.life / p.max;
    ctx.strokeStyle = `rgba(255,${180 + 60 * a | 0},80,${a})`; ctx.lineWidth = 0.18;
    ctx.beginPath(); ctx.moveTo(p.x, p.y); ctx.lineTo(p.x - p.vx * 0.03, p.y - p.vy * 0.03); ctx.stroke();
  }
  ctx.restore();
  sparks = sparks.filter((p) => p.life > 0);
}

function setWorldTransform(ctx, dpr, w, h, cx, cy, z, rot = 0, oy = 0.5) {
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.translate(w / 2, h * oy);
  ctx.scale(z, z);
  if (rot) ctx.rotate(rot);
  ctx.scale(1, -1);
  ctx.translate(-cx, -cy);
}

function renderScene(ctx, z, cars, now, dt, focus, real) {
  drawTrack(ctx, z);
  drawSkids(ctx, z, now);
  const k = real ? 1 : Math.max(1, 15 / (CAR_L * z));
  const sel = cars.find((c) => c.id === selected);
  if (opts.lidar && sel) drawLidar(ctx, sel, z);
  // draw boxed/pit cars first, then track cars, selected on top
  const order = cars.slice().sort((a, b) => (a.id === selected) - (b.id === selected) || (a.st === "track") - (b.st === "track"));
  for (const c of order) drawCar(ctx, c, k, c.id === selected);
  drawSparks(ctx, dt);
  if (opts.hitbox) for (const c of cars) drawHitbox(ctx, c, z);
}

// ------------------------------------------------------------ interpolation
function interpCars(rt) {
  if (!snaps.length) return [];
  let a = snaps[0], b = snaps[snaps.length - 1];
  if (rt <= a.t) return a.cars;
  if (rt >= b.t) return b.cars;
  for (let i = snaps.length - 1; i > 0; i--) { if (snaps[i - 1].t <= rt) { a = snaps[i - 1]; b = snaps[i]; break; } }
  const u = b.t > a.t ? (rt - a.t) / (b.t - a.t) : 1;
  const prev = new Map(a.cars.map((c) => [c.id, c]));
  return b.cars.map((cb) => {
    const ca = prev.get(cb.id);
    if (!ca || Math.hypot(cb.x - ca.x, cb.y - ca.y) > 40) return cb;
    return Object.assign({}, cb, { x: lerp(ca.x, cb.x, u), y: lerp(ca.y, cb.y, u), h: lerpAngle(ca.h, cb.h, u), v: lerp(ca.v, cb.v, u) });
  });
}

// ------------------------------------------------------------ main loop
let lastFrame = performance.now();
function frame(now) {
  const dt = Math.min(0.1, (now - lastFrame) / 1000); lastFrame = now;
  requestAnimationFrame(frame);
  if (!T || !paths) return;
  const simNow = now / 1000 * clock.rate + clock.offset;
  const rt = simNow - 0.09 * Math.max(clock.rate, 0.0001);
  const cars = interpCars(rt);
  const sel = cars.find((c) => c.id === selected);

  // skid marks from sliding cars (rear axle)
  for (const c of cars) {
    const rx = c.x - Math.cos(c.h) * 1.65, ry = c.y - Math.sin(c.h) * 1.65;
    const prev = lastRear.get(c.id);
    if (c.slide && prev && Math.hypot(rx - prev.x, ry - prev.y) < 8 && Math.hypot(rx - prev.x, ry - prev.y) > 0.3) {
      const nx = -Math.sin(c.h) * 0.75, ny = Math.cos(c.h) * 0.75;
      skids.push({ x1: prev.x + nx, y1: prev.y + ny, x2: rx + nx, y2: ry + ny, t: now });
      skids.push({ x1: prev.x - nx, y1: prev.y - ny, x2: rx - nx, y2: ry - ny, t: now });
    }
    if (!prev || Math.hypot(rx - prev.x, ry - prev.y) > 0.3) lastRear.set(c.id, { x: rx, y: ry });
  }
  if (skids.length > 6000) skids = skids.slice(-5000);
  if (skids.length && now - skids[0].t > 40000) skids = skids.filter((s) => now - s.t < 40000);

  // ---- main view
  const W = fitCanvas(world);
  if (!view.fitted) fitView();
  if (opts.follow && sel) { view.cx = sel.x; view.cy = sel.y; }
  wctx.setTransform(W.dpr, 0, 0, W.dpr, 0, 0);
  wctx.fillStyle = "#0c120e"; wctx.fillRect(0, 0, W.w, W.h);
  setWorldTransform(wctx, W.dpr, W.w, W.h, view.cx, view.cy, view.z);
  renderScene(wctx, view.z, cars, now, dt, sel, false);
  // labels in screen space
  wctx.setTransform(W.dpr, 0, 0, W.dpr, 0, 0);
  if (opts.labels) {
    wctx.font = "600 11px -apple-system, BlinkMacSystemFont, Inter, sans-serif";
    wctx.textBaseline = "middle";
    const placed = [];
    const order = cars.slice().sort((a, b) => a.pos - b.pos);
    for (const c of order) {
      const sx = (c.x - view.cx) * view.z + W.w / 2, sy = -(c.y - view.cy) * view.z + W.h / 2;
      if (sx < -50 || sy < -30 || sx > W.w + 50 || sy > W.h + 30) continue;
      const text = c.pos + "  " + c.name;
      const tw = wctx.measureText(text).width + 14;
      const carPx = Math.max(15, CAR_L * view.z) / 2;
      let lx = sx + carPx + 6, ly = sy - carPx - 8;
      // nudge overlapping labels apart
      for (let tries = 0; tries < 6; tries++) {
        const hit = placed.find((p) => lx < p.x + p.w && lx + tw > p.x && Math.abs(ly - p.y) < 17);
        if (!hit) break; ly = hit.y - 18;
      }
      lx = clamp(lx, 4, W.w - tw - 4); ly = clamp(ly, 10, W.h - 10);
      placed.push({ x: lx, y: ly, w: tw });
      wctx.strokeStyle = "rgba(255,255,255,0.35)"; wctx.lineWidth = 1;
      wctx.beginPath(); wctx.moveTo(sx, sy); wctx.lineTo(lx, ly + 6); wctx.stroke();
      wctx.fillStyle = c.id === selected ? "rgba(255,255,255,0.95)" : "rgba(10,12,16,0.82)";
      roundRect(wctx, lx, ly - 9, tw, 18, 5); wctx.fill();
      wctx.fillStyle = c.color; wctx.fillRect(lx, ly - 9, 3, 18);
      wctx.fillStyle = c.id === selected ? "#0a0c10" : "#eef2f8";
      wctx.fillText(text, lx + 8, ly + 0.5);
    }
  }

  // ---- onboard camera
  const C = fitCanvas(cam);
  cctx.setTransform(C.dpr, 0, 0, C.dpr, 0, 0);
  cctx.fillStyle = "#0c120e"; cctx.fillRect(0, 0, C.w, C.h);
  if (sel) {
    const zc = 3.4;
    setWorldTransform(cctx, C.dpr, C.w, C.h, sel.x, sel.y, zc, sel.h - Math.PI / 2, 0.66);
    renderScene(cctx, zc, cars, now, 0, sel, true);
    if (!opts.lidar) { /* nothing */ }
    if (!opts.hitbox) drawHitbox(cctx, sel, zc);
  }
}
function roundRect(ctx, x, y, w, h, r) {
  ctx.beginPath(); ctx.moveTo(x + r, y); ctx.arcTo(x + w, y, x + w, y + h, r); ctx.arcTo(x + w, y + h, x, y + h, r);
  ctx.arcTo(x, y + h, x, y, r); ctx.arcTo(x, y, x + w, y, r); ctx.closePath();
}

// ------------------------------------------------------------ UI panels
function tyreHTML(c) {
  const col = TYRE_COL[c.tyre] || "#999";
  return `<span class="tyre"><i style="border-color:${col};color:${col}">${c.tyre[0].toUpperCase()}</i>${Math.round(c.wear * 100)}%</span>`;
}
function statusChips(c) {
  const out = [];
  if (c.err) out.push('<span class="chip err">ERR</span>');
  if (c.st !== "track") out.push(`<span class="chip pit">${c.st === "boxed" ? "BOX" : "PIT"}</span>`);
  else if (c.pitreq) out.push('<span class="chip pit">BOX→</span>');
  if (c.slide) out.push('<span class="chip slide">SLIDE</span>');
  if (c.drs > 40 && c.st === "track") out.push(`<span class="chip drs">DRS ${c.drs}</span>`);
  if (c.ghost && c.st === "track") out.push('<span class="chip ghost">GHOST</span>');
  if (c.surf === "grass" && c.st === "track") out.push('<span class="chip grass">OFF</span>');
  return out.join("");
}
function updateBoard(s) {
  const tb = $("#board tbody");
  $("#board-empty").style.display = s.cars.length ? "none" : "block";
  const bestOverall = Math.min(...s.cars.map((c) => c.best ?? 1e9));
  tb.innerHTML = s.cars.map((c) => {
    const chips = c.err ? '<span class="chip sm err">ERR</span>' : c.st !== "track" ? '<span class="chip sm pit">PIT</span>' : c.slide ? '<span class="chip sm slide">SLD</span>' : "";
    return `<tr data-id="${c.id}" class="${c.id === selected ? "sel" : ""}">
      <td>${c.pos}</td><td class="l name"><span class="sw" style="background:${c.color}"></span>${esc(c.name)}</td>
      <td>${c.laps}</td><td class="c-last">${fmt(c.last)}</td><td class="best ${c.best != null && c.best === bestOverall ? "pb" : ""}">${fmt(c.best)}</td>
      <td>${tyreHTML(c)}</td><td>${chips}</td></tr>`;
  }).join("");
  const leader = s.cars[0];
  $("#lapinfo").textContent = leader ? `leader on lap ${leader.laps + 1}` : "";
}
function setBar(id, frac, label) { $(id).style.width = clamp(frac, 0, 1) * 100 + "%"; }
function updateSel(s) {
  const c = s.cars.find((x) => x.id === selected);
  const mine = !!(c && auth.user && c.name === auth.user);
  $("#edit").hidden = !mine; $("#tobox").hidden = !(mine || (c && auth.admin));
  if (!c) {
    $("#sel-name").textContent = s.cars.length ? "Select a car" : "No car selected";
    $("#sel-sw").style.background = "#444"; $("#sel-chips").innerHTML = ""; $("#botlog").textContent = "";
    $("#cam-name").textContent = "—"; $("#cam-v").textContent = "0"; $("#cam-flags").innerHTML = "";
    return;
  }
  $("#sel-name").textContent = `P${c.pos} · ${c.name}`;
  $("#sel-sw").style.background = c.color; $("#cam-sw").style.background = c.color;
  $("#sel-chips").innerHTML = statusChips(c);
  $("#t-v").textContent = Math.round(c.v * 3.6) + " km/h";
  $("#t-lt").textContent = c.lt != null ? fmt(c.lt) : "out lap";
  $("#t-g").textContent = `${c.glat.toFixed(1)} / ${c.glong.toFixed(1)}`;
  $("#t-slip").textContent = c.slip.toFixed(1) + "°";
  setBar("#b-thr", Math.max(0, c.thr)); setBar("#b-brk", c.brk); setBar("#b-drs", c.drs / 100);
  setBar("#b-wear", c.wear); setBar("#b-dmg", c.dmg);
  $("#t-drs").textContent = c.drs + "%"; $("#t-wear").textContent = Math.round(c.wear * 100) + "% · grip " + Math.round(c.grip * 100) + "%";
  $("#t-dmg").textContent = Math.round(c.dmg * 100) + "%";
  const st = $("#b-str"); const sv = clamp(c.str, -1, 1);
  // + steer = left: draw from centre toward the left
  st.style.left = (sv >= 0 ? 50 - sv * 50 : 50) + "%"; st.style.width = Math.abs(sv) * 50 + "%";
  $("#t-ms").textContent = c.alive ? `drive() ${c.ms.toFixed(2)} ms` : "bot not running";
  const log = $("#botlog");
  if (mine && myStatus) {
    if (myStatus.err) { log.className = "log err"; log.textContent = myStatus.err; }
    else { log.className = "log"; log.textContent = myStatus.log && myStatus.log.length ? myStatus.log.join("\n") : "(print() output from your bot shows up here)"; }
  } else if (!mine) {
    log.className = c.err ? "log err" : "log";
    log.textContent = c.err ? `${c.name}'s bot is throwing errors (details are private to them).` : `Bot output is private to ${c.name}.`;
  }
  $("#cam-name").textContent = `P${c.pos} ${c.name}`;
  $("#cam-v").textContent = Math.round(c.v * 3.6);
  $("#cam-thr").style.height = Math.max(0, c.thr) * 100 + "%"; $("#cam-brk").style.height = c.brk * 100 + "%";
  $("#cam-flags").innerHTML = `<span class="chip" style="color:${TYRE_COL[c.tyre]}">${c.tyre.toUpperCase()} ${Math.round(c.wear * 100)}%</span>` + statusChips(c);
}
function addEvents(evs) {
  const ul = $("#events");
  for (const e of evs) {
    if (knownEvents.has(e.id)) continue;
    knownEvents.add(e.id);
    const li = document.createElement("li"); li.className = e.kind;
    li.innerHTML = `<span class="et">${fmtClock(e.t)}</span><span class="tx">${esc(e.text)}</span>`;
    ul.prepend(li);
  }
  while (ul.children.length > 150) ul.lastChild.remove();
}

// ------------------------------------------------------------ networking
function onSnap(s) {
  latest = s;
  const now = performance.now();
  const rate = s.paused ? 0 : s.speed;
  const target = s.t - now / 1000 * rate;
  if (!clock.init || rate !== clock.rate || Math.abs(target - clock.offset) > 0.4) { clock.offset = target; clock.rate = rate; clock.init = true; }
  else clock.offset += (target - clock.offset) * 0.08;
  snaps.push(s);
  while (snaps.length > 2 && snaps[1].t < s.t - 2) snaps.shift();
  for (const sp of s.sparks || []) {
    for (let i = 0; i < 10 * sp.p + 4; i++) {
      const a = Math.random() * Math.PI * 2, v = 8 + Math.random() * 25;
      const life = 0.25 + Math.random() * 0.45;
      sparks.push({ x: sp.x, y: sp.y, vx: Math.cos(a) * v, vy: Math.sin(a) * v, life, max: life });
    }
  }
  if (s.events && s.events.length) addEvents(s.events);
  if (selected == null && s.cars.length) selected = s.cars[0].id;
  if (selected != null && !s.cars.find((c) => c.id === selected)) selected = s.cars.length ? s.cars[0].id : null;
  if (now - lastBoard > 200) {
    lastBoard = now;
    updateBoard(s); updateSel(s);
    $("#clock").textContent = fmtClock(s.t);
    $("#carcount").textContent = `${s.cars.length} car${s.cars.length === 1 ? "" : "s"}${s.paused ? " · paused" : s.speed !== 1 ? " · " + s.speed + "×" : ""}`;
    $("#pausebtn").textContent = s.paused ? "▶" : "❚❚";
    document.querySelectorAll("[data-speed]").forEach((b) => b.classList.toggle("on", +b.dataset.speed === s.speed && !s.paused));
  }
}
async function api(path, body, method = "POST") {
  const headers = { "Content-Type": "application/json" };
  if (auth.token) headers.Authorization = "Bearer " + auth.token;
  const r = await fetch(API + path, { method, headers, body: body ? JSON.stringify(body) : undefined });
  const j = await r.json().catch(() => ({}));
  if (!r.ok) throw new Error(j.error || r.statusText);
  return j;
}
// live feed: WebSocket (works through Cloudflare tunnels), falling back to polling
let lastEv = 0, liveOk = false;
function feed(s) { liveOk = true; if (s.ev) lastEv = s.ev; onSnap(s); }
function connectLive() {
  const base = API || location.origin;
  let ws;
  try { ws = new WebSocket(base.replace(/^http/, "ws") + "/api/ws"); } catch { return startPolling(); }
  let opened = false;
  const giveUp = setTimeout(() => { if (!opened) { try { ws.close(); } catch {} startPolling(); } }, 6000);
  ws.onopen = () => { opened = true; clearTimeout(giveUp); };
  ws.onmessage = (m) => feed(JSON.parse(m.data));
  ws.onclose = () => {
    clearTimeout(giveUp);
    if (!opened) return startPolling();
    setTimeout(async () => {   // dropped: the server may have moved to a new tunnel
      try { if ((await resolveApi()) !== API) return location.reload(); } catch {}
      connectLive();
    }, 2000);
  };
}
let polling = false;
async function startPolling() {
  if (polling) return; polling = true;
  let fails = 0;
  for (;;) {
    try {
      const r = await fetch(API + "/api/snapshot?since=" + lastEv, { cache: "no-store" });
      feed(await r.json()); fails = 0;
    } catch {
      if (++fails % 10 === 0) { try { if ((await resolveApi()) !== API) return location.reload(); } catch {} }
    }
    await new Promise((res) => setTimeout(res, 100));
  }
}
async function boot() {
  try {
    API = await resolveApi();
    T = await (await fetch(API + "/api/track")).json();
  } catch (e) {
    $("#board-empty").textContent = "Race server is offline right now — try again in a bit.";
    setTimeout(boot, 5000);
    return;
  }
  buildPaths();
  connectLive();
  requestAnimationFrame(frame);
  await refreshMe();
  setInterval(pollMyStatus, 1000);
}

// ------------------------------------------------------------ input
let drag = null;
world.addEventListener("mousedown", (e) => { drag = { x: e.clientX, y: e.clientY, cx: view.cx, cy: view.cy, moved: false }; world.classList.add("drag"); });
window.addEventListener("mousemove", (e) => {
  if (!drag) return;
  const dx = e.clientX - drag.x, dy = e.clientY - drag.y;
  if (Math.abs(dx) + Math.abs(dy) > 3) { drag.moved = true; if (opts.follow) setToggle("follow", false); }
  view.cx = drag.cx - dx / view.z; view.cy = drag.cy + dy / view.z;
});
window.addEventListener("mouseup", (e) => {
  if (drag && !drag.moved && latest) {
    const r = world.getBoundingClientRect();
    const mx = e.clientX - r.left, my = e.clientY - r.top;
    let best = null, bd = 22;
    for (const c of interpCars(performance.now() / 1000 * clock.rate + clock.offset - 0.09 * clock.rate)) {
      const sx = (c.x - view.cx) * view.z + r.width / 2, sy = -(c.y - view.cy) * view.z + r.height / 2;
      const d = Math.hypot(sx - mx, sy - my);
      if (d < bd) { bd = d; best = c; }
    }
    if (best) { selected = best.id; updateBoard(latest); updateSel(latest); }
  }
  drag = null; world.classList.remove("drag");
});
world.addEventListener("wheel", (e) => {
  e.preventDefault();
  const r = world.getBoundingClientRect();
  const mx = e.clientX - r.left - r.width / 2, my = e.clientY - r.top - r.height / 2;
  const wx = view.cx + mx / view.z, wy = view.cy - my / view.z;
  const f = Math.exp(-e.deltaY * 0.0015);
  view.z = clamp(view.z * f, 0.08, 30);
  if (!opts.follow) { view.cx = wx - mx / view.z; view.cy = wy + my / view.z; }
}, { passive: false });
world.addEventListener("dblclick", () => { setToggle("follow", false); fitView(); });
window.addEventListener("resize", () => { if (paths && !opts.follow) fitView(); });

$("#board tbody").addEventListener("click", (e) => {
  const tr = e.target.closest("tr"); if (!tr) return;
  selected = +tr.dataset.id; if (latest) { updateBoard(latest); updateSel(latest); }
});
function setToggle(name, val) {
  opts[name] = val;
  document.querySelector(`[data-toggle="${name}"]`).classList.toggle("on", val);
  if (name === "follow" && val && view.z < 1.2) view.z = 1.6;
}
document.querySelectorAll("[data-toggle]").forEach((b) => b.addEventListener("click", () => setToggle(b.dataset.toggle, !opts[b.dataset.toggle])));
document.querySelectorAll("[data-speed]").forEach((b) => b.addEventListener("click", async () => {
  await api("/api/control", { action: "speed", value: +b.dataset.speed });
  await api("/api/control", { action: "resume" });
}));
$("#pausebtn").addEventListener("click", () => api("/api/control", { action: latest && latest.paused ? "resume" : "pause" }).catch(toastErr));
$("#tobox").addEventListener("click", () => {
  const c = latest && latest.cars.find((x) => x.id === selected); if (!c) return;
  (c.name === auth.user ? api("/api/my/box", {}) : api("/api/control", { action: "box", id: selected })).catch(toastErr);
});

// ------------------------------------------------------------ accounts
let myStatus = null;
function toast(msg, bad) {
  const t = $("#toast"); t.textContent = msg; t.className = bad ? "bad" : ""; t.hidden = false;
  clearTimeout(toast.h); toast.h = setTimeout(() => { t.hidden = true; }, 3500);
}
function toastErr(e) { toast(e.message || String(e), true); }
function renderAuth() {
  const on = !!auth.user;
  $(".authed").hidden = !on; $("#loginbtn").hidden = on;
  $("#speedseg").hidden = !auth.admin;
  $("#who").textContent = auth.user || "";
  const acc = latest && latest.cars.find((c) => c.name === auth.user);
  $("#who-sw").style.background = acc ? acc.color : "#555";
  if (latest) updateSel(latest);
}
async function refreshMe() {
  if (!auth.token) { auth.user = null; auth.admin = false; return renderAuth(); }
  try {
    const me = await api("/api/me", null, "GET");
    if (!me.user) { auth.token = null; store.set("pw_token", null); }
    auth.user = me.user; auth.admin = me.admin;
  } catch { /* server offline: keep token */ }
  renderAuth();
}
async function pollMyStatus() {
  if (!auth.user) { myStatus = null; return; }
  try { myStatus = await api("/api/my/status", null, "GET"); } catch { myStatus = null; }
}
$("#loginbtn").addEventListener("click", () => { $("#login").hidden = false; $("#lg-err").hidden = true; $("#lg-user").focus(); });
$("#lg-cancel").addEventListener("click", () => { $("#login").hidden = true; });
$("#login").addEventListener("mousedown", (e) => { if (e.target.id === "login") $("#login").hidden = true; });
$("#loginform").addEventListener("submit", async (e) => {
  e.preventDefault();
  try {
    const j = await api("/api/login", { user: $("#lg-user").value, password: $("#lg-pass").value });
    auth = { token: j.token, user: j.user, admin: j.admin };
    store.set("pw_token", j.token);
    $("#login").hidden = true; $("#lg-pass").value = "";
    renderAuth();
    const c = latest && latest.cars.find((x) => x.name === j.user);
    if (c) selected = c.id;
    toast(c ? `Welcome back, ${j.user}` : `Logged in as ${j.user} — open "My script" and deploy to start your car`);
  } catch (err) { $("#lg-err").hidden = false; $("#lg-err").textContent = err.message; }
});
$("#logout").addEventListener("click", async () => {
  await api("/api/logout", {}).catch(() => {});
  auth = { token: null, user: null, admin: false }; store.set("pw_token", null); myStatus = null; renderAuth();
});

window.addEventListener("keydown", (e) => {
  if (!$("#modal").hidden || !$("#login").hidden || e.target.tagName === "INPUT" || e.target.tagName === "TEXTAREA") return;
  if (e.code === "Space" && auth.admin) { e.preventDefault(); $("#pausebtn").click(); }
  if (e.key === "f") setToggle("follow", !opts.follow);
  if (e.key === "l") setToggle("lidar", !opts.lidar);
  if (e.key === "h") setToggle("hitbox", !opts.hitbox);
});

// ------------------------------------------------------------ editor
const ed = $("#ed-code"), gut = $("#ed-gutter");
let edOriginalName = null;
function syncGutter() {
  const n = ed.value.split("\n").length;
  let s = ""; for (let i = 1; i <= n; i++) s += i + "\n";
  gut.textContent = s; gut.scrollTop = ed.scrollTop;
}
ed.addEventListener("input", syncGutter);
ed.addEventListener("scroll", () => { gut.scrollTop = ed.scrollTop; });
ed.addEventListener("keydown", (e) => {
  if (e.key === "Tab") {
    e.preventDefault();
    const s = ed.selectionStart, en = ed.selectionEnd;
    ed.setRangeText("    ", s, en, "end"); syncGutter();
  } else if ((e.metaKey || e.ctrlKey) && e.key === "s") { e.preventDefault(); deploy(); }
  else if (e.key === "Escape") closeEditor();
});
function openEditor(code) {
  $("#ed-name").value = auth.user; $("#ed-name").disabled = true;
  ed.value = code; syncGutter();
  $("#ed-err").hidden = true; $("#ed-status").textContent = "";
  $("#modal").hidden = false; ed.focus(); ed.setSelectionRange(0, 0); ed.scrollTop = 0; gut.scrollTop = 0;
}
function closeEditor() { $("#modal").hidden = true; }
async function pushScript(code) {
  await api("/api/my/bot", { code });
  setTimeout(() => { const c = latest && latest.cars.find((x) => x.name === auth.user); if (c) selected = c.id; renderAuth(); }, 700);
}
async function deploy() {
  try {
    $("#ed-status").textContent = "deploying…";
    await pushScript(ed.value);
    $("#ed-status").textContent = "deployed ✓ " + new Date().toLocaleTimeString() + " — your car is running it now";
    $("#ed-err").hidden = true;
  } catch (err) {
    $("#ed-status").textContent = "";
    $("#ed-err").hidden = false; $("#ed-err").textContent = err.message;
  }
}
$("#ed-deploy").addEventListener("click", deploy);
$("#ed-cancel").addEventListener("click", closeEditor);
$("#modal").addEventListener("mousedown", (e) => { if (e.target.id === "modal") closeEditor(); });
async function openMyScript() {
  try {
    let { code } = await api("/api/my/bot", null, "GET");
    if (!code) code = (await api("/api/template", null, "GET")).code;
    openEditor(code);
  } catch (err) { toastErr(err); }
}
$("#myscript").addEventListener("click", openMyScript);
$("#edit").addEventListener("click", openMyScript);
$("#upload").addEventListener("change", async (e) => {
  const f = e.target.files[0]; e.target.value = ""; if (!f) return;
  try { await pushScript(await f.text()); toast(`${f.name} deployed — it's driving your car now`); }
  catch (err) { toastErr(err); }
});

boot();
