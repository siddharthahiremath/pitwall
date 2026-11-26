"use strict";
// ============================================================ PITWALL 3D view (three.js)
// Uses app.js globals: T (track), interpCars, clock, selected, latest, opts.
// Sim (x, y) -> three (x, -y) with y up; sim heading h -> rotation.y = h.
(() => {
  const CAMS = [
    ["chase", "Chase"], ["cockpit", "Cockpit"], ["tcam", "T-cam"], ["wheel", "Wheel"],
    ["tv", "TV"], ["heli", "Heli"], ["orbit", "Orbit"],
  ];
  const v3 = { on: false, cam: "chase", built: false, cars: new Map(), orbit: { yaw: 0.8, pitch: 0.55, dist: 60, free: false, cx: 0, cz: 0 } };
  window.v3 = v3;
  let renderer, scene, camera, canvas, hud, tvCams = [], smooth = null, lastT = performance.now();

  // ---------------------------------------------------------- UI
  const stage = document.getElementById("stage");
  canvas = document.createElement("canvas");
  canvas.id = "world3d"; canvas.hidden = true;
  stage.appendChild(canvas);
  hud = document.createElement("div");
  hud.id = "hud3d"; hud.hidden = true;
  stage.appendChild(hud);
  const bar = document.createElement("div");
  bar.id = "cams3d"; bar.hidden = true;
  bar.innerHTML = CAMS.map(([k, n]) => `<button data-cam="${k}">${n}</button>`).join("") + '<span class="k">C = next cam · 3 = 2D/3D</span>';
  stage.appendChild(bar);
  bar.addEventListener("click", (e) => { const b = e.target.closest("[data-cam]"); if (b) setCam(b.dataset.cam); });
  const toggles = document.querySelector(".seg.toggles");
  const btn = document.createElement("button");
  btn.textContent = "3D"; btn.title = "3D view with chase / cockpit / TV cameras (3)";
  toggles.prepend(btn);
  btn.addEventListener("click", () => set3d(!v3.on));
  window.addEventListener("keydown", (e) => {
    if (e.target.tagName === "INPUT" || e.target.tagName === "TEXTAREA" || !document.getElementById("modal").hidden) return;
    if (e.key === "3") set3d(!v3.on);
    if (v3.on && e.key === "c") { const i = CAMS.findIndex(([k]) => k === v3.cam); setCam(CAMS[(i + 1) % CAMS.length][0]); }
  });

  function set3d(on) {
    if (on && !window.THREE) return;
    v3.on = on; btn.classList.toggle("on", on);
    canvas.hidden = bar.hidden = hud.hidden = !on;
    document.getElementById("world").style.visibility = on ? "hidden" : "";
    for (const id of ["legend"]) { const el = document.getElementById(id); if (el) el.style.display = on ? "none" : ""; }
    const hint = stage.querySelector(".hint"); if (hint) hint.style.display = on ? "none" : "";
    if (on && !v3.built && T) build();
    try { localStorage.setItem("pw_3d", on ? v3.cam : ""); } catch {}
  }
  function setCam(k) {
    v3.cam = k; smooth = null;
    bar.querySelectorAll("[data-cam]").forEach((b) => b.classList.toggle("on", b.dataset.cam === k));
    try { if (v3.on) localStorage.setItem("pw_3d", k); } catch {}
  }

  // orbit: drag rotates, wheel zooms (works in every camera mode as a look-around for orbit)
  let drag = null;
  canvas.addEventListener("pointerdown", (e) => { drag = { x: e.clientX, y: e.clientY }; canvas.setPointerCapture(e.pointerId); });
  canvas.addEventListener("pointermove", (e) => {
    if (!drag) return;
    const dx = e.clientX - drag.x, dy = e.clientY - drag.y; drag = { x: e.clientX, y: e.clientY };
    if (v3.cam !== "orbit") setCam("orbit");
    v3.orbit.yaw -= dx * 0.006; v3.orbit.pitch = Math.max(0.05, Math.min(1.5, v3.orbit.pitch + dy * 0.005));
  });
  canvas.addEventListener("pointerup", () => { drag = null; });
  canvas.addEventListener("wheel", (e) => {
    e.preventDefault();
    if (v3.cam !== "orbit") setCam("orbit");
    v3.orbit.dist = Math.max(8, Math.min(3000, v3.orbit.dist * Math.exp(e.deltaY * 0.0012)));
  }, { passive: false });

  // ---------------------------------------------------------- scene
  const P = (x, y, h = 0) => new THREE.Vector3(x, h, -y);

  function ribbon(left, right, closed, h, mat) {
    const n = left.length, pos = [], idx = [];
    for (let i = 0; i < n; i++) { pos.push(left[i][0], h, -left[i][1], right[i][0], h, -right[i][1]); }
    const m = closed ? n : n - 1;
    for (let i = 0; i < m; i++) {
      const a = 2 * i, b = 2 * ((i + 1) % n);
      idx.push(a, a + 1, b, a + 1, b + 1, b);
    }
    const g = new THREE.BufferGeometry();
    g.setAttribute("position", new THREE.Float32BufferAttribute(pos, 3));
    g.setIndex(idx); g.computeVertexNormals();
    return new THREE.Mesh(g, mat);
  }
  function offset(pts, i, d) {   // point i of a polyline pushed sideways by d (left = +)
    const n = pts.length, a = pts[(i - 1 + n) % n], b = pts[(i + 1) % n];
    const dx = b[0] - a[0], dy = b[1] - a[1], L = Math.hypot(dx, dy) || 1;
    return [pts[i][0] - dy / L * d, pts[i][1] + dx / L * d];
  }
  function wall(pts, height, mat) {
    const top = pts.map((p) => p), pos = [], idx = [], n = pts.length;
    for (let i = 0; i < n; i++) pos.push(pts[i][0], 0, -pts[i][1], top[i][0], height, -top[i][1]);
    for (let i = 0; i < n; i++) { const a = 2 * i, b = 2 * ((i + 1) % n); idx.push(a, b, a + 1, a + 1, b, b + 1); }
    const g = new THREE.BufferGeometry();
    g.setAttribute("position", new THREE.Float32BufferAttribute(pos, 3));
    g.setIndex(idx); g.computeVertexNormals();
    return new THREE.Mesh(g, mat);
  }

  function build() {
    v3.built = true;
    renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
    renderer.setPixelRatio(Math.min(2, window.devicePixelRatio || 1));
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x8fb8e0);
    scene.fog = new THREE.Fog(0x8fb8e0, 400, 2600);
    camera = new THREE.PerspectiveCamera(60, 1, 0.1, 6000);
    scene.add(new THREE.HemisphereLight(0xdfeeff, 0x3c5a2a, 0.9));
    const sun = new THREE.DirectionalLight(0xfff2dd, 1.6);
    sun.position.set(300, 500, 200);
    sun.castShadow = true;
    sun.shadow.mapSize.set(2048, 2048);
    Object.assign(sun.shadow.camera, { left: -60, right: 60, top: 60, bottom: -60, near: 10, far: 1500 });
    scene.add(sun, sun.target);
    v3.sun = sun;

    const cl = T.centerline;
    let minx = 1e9, maxx = -1e9, miny = 1e9, maxy = -1e9;
    for (const [x, y] of cl) { minx = Math.min(minx, x); maxx = Math.max(maxx, x); miny = Math.min(miny, y); maxy = Math.max(maxy, y); }
    const cx = (minx + maxx) / 2, cy = (miny + maxy) / 2, span = Math.max(maxx - minx, maxy - miny);
    v3.center = { cx, cy, span };
    v3.orbit.cx = cx; v3.orbit.cz = -cy;

    // grass with a little noise texture
    const gc = document.createElement("canvas"); gc.width = gc.height = 256;
    const gx = gc.getContext("2d"); gx.fillStyle = "#3f7a34"; gx.fillRect(0, 0, 256, 256);
    for (let i = 0; i < 5000; i++) { gx.fillStyle = `rgba(${20 + Math.random() * 60},${80 + Math.random() * 70},${20 + Math.random() * 40},0.35)`; gx.fillRect(Math.random() * 256, Math.random() * 256, 2, 2); }
    const gt = new THREE.CanvasTexture(gc); gt.wrapS = gt.wrapT = THREE.RepeatWrapping; gt.repeat.set(span / 20, span / 20);
    const ground = new THREE.Mesh(new THREE.PlaneGeometry(span * 3, span * 3), new THREE.MeshLambertMaterial({ map: gt }));
    ground.rotation.x = -Math.PI / 2; ground.position.set(cx, -0.02, -cy); ground.receiveShadow = true;
    scene.add(ground);

    // run-off (gravel/tarmac band between edge and wall)
    const runoff = new THREE.MeshLambertMaterial({ color: 0x9a9384 });
    scene.add(ribbon(cl.map((_, i) => offset(cl, i, T.wall - 1)), T.left, true, 0.005, runoff));
    scene.add(ribbon(T.right, cl.map((_, i) => offset(cl, i, -(T.wall - 1))), true, 0.005, runoff));
    // asphalt
    const ac = document.createElement("canvas"); ac.width = ac.height = 128;
    const ax = ac.getContext("2d"); ax.fillStyle = "#3a3d42"; ax.fillRect(0, 0, 128, 128);
    for (let i = 0; i < 3000; i++) { const v = 40 + Math.random() * 40; ax.fillStyle = `rgba(${v},${v},${v + 4},0.5)`; ax.fillRect(Math.random() * 128, Math.random() * 128, 1, 1); }
    const at = new THREE.CanvasTexture(ac); at.wrapS = at.wrapT = THREE.RepeatWrapping; at.repeat.set(1, 1);
    const asphalt = ribbon(T.left, T.right, true, 0.02, new THREE.MeshLambertMaterial({ color: 0xffffff, map: at }));
    asphalt.receiveShadow = true;
    { const uv = []; for (let i = 0; i < cl.length; i++) { const s = T.s[i] / 6; uv.push(0, s, 2.5, s); } asphalt.geometry.setAttribute("uv", new THREE.Float32BufferAttribute(uv, 2)); }
    scene.add(asphalt);
    // white edge lines
    const white = new THREE.MeshLambertMaterial({ color: 0xf2f2f2 });
    scene.add(ribbon(T.left, cl.map((_, i) => offset(cl, i, T.half_width - 0.25)), true, 0.03, white));
    scene.add(ribbon(cl.map((_, i) => offset(cl, i, -(T.half_width - 0.25))), T.right, true, 0.03, white));
    // kerbs on corners (red/white) where curvature is high
    const kr = new THREE.MeshLambertMaterial({ color: 0xd8262d }), kw = new THREE.MeshLambertMaterial({ color: 0xf5f5f5 });
    for (const side of [1, -1]) {
      for (let i = 0; i < cl.length; i++) {
        if (Math.abs(T.curvature[i]) < 0.006) continue;
        const j = (i + 1) % cl.length;
        const a0 = offset(cl, i, side * T.half_width), a1 = offset(cl, i, side * (T.half_width + T.kerb));
        const b0 = offset(cl, j, side * T.half_width), b1 = offset(cl, j, side * (T.half_width + T.kerb));
        const k = ribbon(side > 0 ? [a1, b1] : [a0, b0], side > 0 ? [a0, b0] : [a1, b1], false, 0.06, (i >> 0) % 2 ? kr : kw);
        scene.add(k);
      }
    }
    // barriers: tyre-wall look (low, dark) with a red/white top band
    const wl = T.wall_left, wr = T.wall_right;
    const barrier = new THREE.MeshLambertMaterial({ color: 0x2b2f36, side: THREE.DoubleSide });
    for (const w of [wl, wr]) { const m = wall(w, 1.1, barrier); m.castShadow = true; scene.add(m); }
    // pit lane + boxes
    if (T.pit && T.pit.path) {
      const pp = T.pit.path;
      const pl = pp.map((_, i) => offset(pp, i, 5)), pr = pp.map((_, i) => offset(pp, i, -9));
      scene.add(ribbon(pl, pr, false, 0.015, new THREE.MeshLambertMaterial({ color: 0x4a4d52 })));
      const garage = new THREE.MeshLambertMaterial({ color: 0xcfd3da });
      const roof = new THREE.Mesh(new THREE.BoxGeometry(T.pit.boxes.length * 14 + 20, 7, 12), garage);
      const bx = T.pit.boxes, mid = bx[Math.floor(bx.length / 2)];
      roof.position.set((bx[0].x + bx[bx.length - 1].x) / 2, 3.5, -(mid.y - 12));
      roof.castShadow = true; roof.receiveShadow = true;
      scene.add(roof);
      for (const b of bx) {
        const mark = new THREE.Mesh(new THREE.PlaneGeometry(6, 3), new THREE.MeshLambertMaterial({ color: 0x6a6f78 }));
        mark.rotation.x = -Math.PI / 2; mark.position.set(b.x, 0.03, -b.y); scene.add(mark);
      }
    }
    // start/finish line + gantry
    {
      const p0 = cl[0], h0 = T.heading[0];
      const line = new THREE.Mesh(new THREE.PlaneGeometry(1.2, T.half_width * 2), new THREE.MeshLambertMaterial({ color: 0xffffff }));
      line.rotation.x = -Math.PI / 2; line.rotation.z = h0; line.position.set(p0[0], 0.04, -p0[1]); scene.add(line);
      const g = new THREE.Group();
      const post = new THREE.MeshLambertMaterial({ color: 0x22252b });
      for (const s of [1, -1]) { const p = new THREE.Mesh(new THREE.BoxGeometry(0.6, 8, 0.6), post); p.position.set(0, 4, s * (T.half_width + 3)); g.add(p); }
      const beam = new THREE.Mesh(new THREE.BoxGeometry(1, 1.4, T.half_width * 2 + 6.6), post); beam.position.y = 8; g.add(beam);
      g.position.set(p0[0], 0, -p0[1]); g.rotation.y = h0; scene.add(g);
    }
    // grandstands + trees for depth
    const stand = new THREE.MeshLambertMaterial({ color: 0x5d6b80 });
    const crowd = new THREE.MeshLambertMaterial({ color: 0xb05a4a });
    const tree = new THREE.MeshLambertMaterial({ color: 0x2d5a27 }), trunk = new THREE.MeshLambertMaterial({ color: 0x5b4330 });
    const coneG = new THREE.ConeGeometry(3.2, 9, 7), trunkG = new THREE.CylinderGeometry(0.4, 0.5, 2.5, 6);
    let seed = 7; const rnd = () => (seed = (seed * 16807) % 2147483647) / 2147483647;
    for (let i = 0; i < cl.length; i += 3) {
      for (const side of [1, -1]) {
        if (rnd() < 0.55) continue;
        const d = side * (T.wall + 12 + rnd() * 70), p = offset(cl, i, d);
        // keep away from any part of the track
        let ok = true; for (let j = 0; j < cl.length; j += 4) { if (Math.hypot(cl[j][0] - p[0], cl[j][1] - p[1]) < T.wall + 8) { ok = false; break; } }
        if (!ok) continue;
        const t = new THREE.Mesh(coneG, tree); t.position.set(p[0], 6.5, -p[1]); t.castShadow = true; scene.add(t);
        const tr = new THREE.Mesh(trunkG, trunk); tr.position.set(p[0], 1.25, -p[1]); scene.add(tr);
      }
    }
    for (let i = 20; i < cl.length; i += Math.floor(cl.length / 7)) {
      const p = offset(cl, i, T.wall + 10), h = T.heading[i];
      const g = new THREE.Group();
      for (let r = 0; r < 6; r++) {
        const step = new THREE.Mesh(new THREE.BoxGeometry(60, 1.2, 2.2), r % 2 ? stand : crowd);
        step.position.set(0, 0.6 + r * 1.2, -r * 2.2 - 2); g.add(step);
      }
      const roofS = new THREE.Mesh(new THREE.BoxGeometry(62, 0.4, 16), stand); roofS.position.set(0, 10, -8); g.add(roofS);
      g.position.set(p[0], 0, -p[1]); g.rotation.y = h; scene.add(g);
    }
    // TV camera spots: every ~250 m, outside the wall, on alternating sides
    for (let i = 0; i < cl.length; i += 60) {
      const side = T.curvature[i] > 0 ? -1 : 1;
      const p = offset(cl, i, side * (T.wall + 6));
      tvCams.push(new THREE.Vector3(p[0], 7, -p[1]));
    }
    onResize();
  }

  // ---------------------------------------------------------- cars
  function carModel(color) {
    const g = new THREE.Group();
    const body = new THREE.MeshStandardMaterial({ color, metalness: 0.3, roughness: 0.45 });
    const dark = new THREE.MeshStandardMaterial({ color: 0x15171b, roughness: 0.8 });
    const tyreM = new THREE.MeshStandardMaterial({ color: 0x111111, roughness: 0.9 });
    const add = (geo, mat, x, y, z, shadow = true) => { const m = new THREE.Mesh(geo, mat); m.position.set(x, y, z); m.castShadow = shadow; g.add(m); return m; };
    // (car points along +x; width along z)
    add(new THREE.BoxGeometry(3.4, 0.45, 0.9), body, 0.1, 0.42, 0);            // monocoque
    add(new THREE.BoxGeometry(1.6, 0.25, 0.42), body, 2.25, 0.32, 0);          // nose
    add(new THREE.BoxGeometry(1.8, 0.4, 1.7), body, -0.4, 0.38, 0);            // sidepods
    add(new THREE.BoxGeometry(1.3, 0.55, 0.6), body, -1.1, 0.75, 0);           // engine cover
    add(new THREE.BoxGeometry(0.45, 0.05, 1.95), dark, 2.85, 0.14, 0);         // front wing
    add(new THREE.BoxGeometry(0.35, 0.05, 1.4), body, 2.8, 0.2, 0);
    add(new THREE.BoxGeometry(0.45, 0.06, 1.2), dark, -2.5, 0.95, 0);          // rear wing
    add(new THREE.BoxGeometry(0.5, 0.55, 0.06), dark, -2.5, 0.7, 0.6);
    add(new THREE.BoxGeometry(0.5, 0.55, 0.06), dark, -2.5, 0.7, -0.6);
    const helmet = add(new THREE.SphereGeometry(0.2, 12, 10), new THREE.MeshStandardMaterial({ color: 0xf5d33a, roughness: 0.3 }), 0.15, 0.85, 0);
    add(new THREE.TorusGeometry(0.33, 0.035, 6, 16, Math.PI), dark, 0.35, 0.82, 0).rotation.set(0, Math.PI / 2, 0); // halo
    const wheels = [];
    const tyreG = new THREE.CylinderGeometry(0.36, 0.36, 0.4, 16); tyreG.rotateX(Math.PI / 2);
    const stripeG = new THREE.CylinderGeometry(0.3, 0.3, 0.41, 16); stripeG.rotateX(Math.PI / 2);
    const stripe = new THREE.MeshBasicMaterial({ color: 0xffffff });
    for (const [x, z, front] of [[1.75, 0.82, 1], [1.75, -0.82, 1], [-1.65, 0.82, 0], [-1.65, -0.82, 0]]) {
      const w = new THREE.Group(); w.position.set(x, 0.36, z);
      const t = new THREE.Mesh(tyreG, tyreM); t.castShadow = true; w.add(t);
      const s = new THREE.Mesh(stripeG, stripe); w.add(s);
      const hub = new THREE.Mesh(new THREE.CylinderGeometry(0.22, 0.22, 0.42, 10).rotateX(Math.PI / 2), dark); w.add(hub);
      g.add(w); wheels.push({ w, front });
    }
    // name tag
    const lc = document.createElement("canvas"); lc.width = 256; lc.height = 64;
    const label = new THREE.Sprite(new THREE.SpriteMaterial({ map: new THREE.CanvasTexture(lc), depthTest: false, transparent: true }));
    label.scale.set(6, 1.5, 1); label.position.set(0, 3.2, 0); label.renderOrder = 10;
    g.add(label);
    // brake light
    const brake = add(new THREE.BoxGeometry(0.05, 0.12, 0.22), new THREE.MeshBasicMaterial({ color: 0x400000 }), -2.6, 0.55, 0, false);
    return { g, wheels, stripe, label, lc, brake, helmet, spin: 0, tag: "" };
  }
  const TYRE_HEX = { soft: 0xff3b30, medium: 0xffd60a, hard: 0xf2f2f2 };

  function syncCars(cars, dt) {
    const seen = new Set();
    for (const c of cars) {
      seen.add(c.id);
      let m = v3.cars.get(c.id);
      if (!m) { m = carModel(c.color || "#888"); v3.cars.set(c.id, m); scene.add(m.g); }
      m.g.position.set(c.x, 0, -c.y);
      m.g.rotation.y = c.h;
      m.spin += (c.v || 0) / 0.36 * dt;
      for (const { w, front } of m.wheels) { w.children[0].rotation.z = -m.spin; w.rotation.y = front ? (c.str || 0) * 0.6 : 0; }
      m.stripe.color.setHex(TYRE_HEX[c.tyre] || 0xffffff);
      m.brake.material.color.setHex((c.brk || 0) > 0.1 ? 0xff2020 : 0x400000);
      const tag = `${c.pos ?? ""}|${c.name}|${c.id === selected}`;
      if (tag !== m.tag) {
        m.tag = tag;
        const x = m.lc.getContext("2d"); x.clearRect(0, 0, 256, 64);
        x.fillStyle = c.id === selected ? "rgba(255,255,255,0.92)" : "rgba(10,12,16,0.72)";
        x.beginPath(); x.roundRect(4, 8, 248, 48, 10); x.fill();
        x.fillStyle = c.color || "#888"; x.fillRect(14, 18, 8, 28);
        x.font = "600 26px -apple-system, Inter, sans-serif"; x.textBaseline = "middle";
        x.fillStyle = c.id === selected ? "#111" : "#fff";
        x.fillText(`${c.pos ?? ""}  ${c.name}`, 32, 33);
        m.label.material.map.needsUpdate = true;
      }
      m.label.visible = opts.labels && !(c.id === selected && ["cockpit", "tcam", "wheel"].includes(v3.cam));
      m.helmet.visible = !(c.id === selected && v3.cam === "cockpit");
      m.g.visible = !(c.ghost && false);
    }
    for (const [id, m] of v3.cars) if (!seen.has(id)) { scene.remove(m.g); v3.cars.delete(id); }
  }

  // ---------------------------------------------------------- cameras
  const tmp = new THREE.Vector3(), look = new THREE.Vector3();
  function place(c, dt) {
    const fwd = c ? new THREE.Vector3(Math.cos(c.h), 0, -Math.sin(c.h)) : null, side = c ? new THREE.Vector3(Math.sin(c.h), 0, Math.cos(c.h)) : null;
    const pos = c ? P(c.x, c.y) : null;
    let fov = 60, want, target, lerpK = 1;
    switch (c ? v3.cam : "orbit") {
      case "chase": want = pos.clone().addScaledVector(fwd, -9).add(tmp.set(0, 3.2, 0)); target = pos.clone().addScaledVector(fwd, 6).add(tmp.set(0, 1, 0)); fov = 62; lerpK = 1 - Math.exp(-dt * 8); break;
      case "cockpit": want = pos.clone().addScaledVector(fwd, 0.25).add(tmp.set(0, 1.08, 0)); target = want.clone().addScaledVector(fwd, 20).add(tmp.set(0, -0.35, 0)); fov = 80; break;
      case "tcam": want = pos.clone().addScaledVector(fwd, -0.6).add(tmp.set(0, 1.3, 0)); target = want.clone().addScaledVector(fwd, 20).add(tmp.set(0, -0.8, 0)); fov = 70; break;
      case "wheel": want = pos.clone().addScaledVector(fwd, -1.2).addScaledVector(side, 1.05).add(tmp.set(0, 0.55, 0)); target = want.clone().addScaledVector(fwd, 20); fov = 72; break;
      case "heli": want = pos.clone().addScaledVector(fwd, -40).add(tmp.set(0, 55, 0)); target = pos.clone().addScaledVector(fwd, 10); fov = 50; lerpK = 1 - Math.exp(-dt * 2.5); break;
      case "tv": {
        let best = tvCams[0], bd = 1e9;
        for (const p of tvCams) { const d = p.distanceTo(pos); if (d < bd) { bd = d; best = p; } }
        want = best.clone(); target = pos.clone().add(tmp.set(0, 0.6, 0));
        fov = Math.max(6, Math.min(55, 2 * Math.atan(9 / Math.max(1, bd)) * 180 / Math.PI));
        break;
      }
      default: {   // orbit around the selected car (or the whole track)
        const o = v3.orbit, cx = pos ? pos.x : o.cx, cz = pos ? pos.z : o.cz, d = pos ? o.dist : Math.max(o.dist, v3.center.span * 0.8);
        want = new THREE.Vector3(cx + d * Math.cos(o.pitch) * Math.cos(o.yaw), d * Math.sin(o.pitch), cz + d * Math.cos(o.pitch) * Math.sin(o.yaw));
        target = new THREE.Vector3(cx, 0.5, cz); fov = 55;
      }
    }
    if (!smooth || lerpK >= 1) smooth = { p: want.clone(), t: target.clone() };
    else { smooth.p.lerp(want, lerpK); smooth.t.lerp(target, Math.min(1, lerpK * 1.6)); }
    camera.position.copy(smooth.p); camera.lookAt(smooth.t);
    const dFar = camera.position.distanceTo(smooth.t);
    scene.fog.near = 400 + dFar * 1.2; scene.fog.far = 2600 + dFar * 2.5;
    if (Math.abs(camera.fov - fov) > 0.01) { camera.fov = v3.cam === "tv" ? camera.fov + (fov - camera.fov) * Math.min(1, dt * 4) : fov; camera.updateProjectionMatrix(); }
    // keep the shadow box around what we're looking at
    if (v3.sun) { const f = smooth.t; v3.sun.target.position.copy(f); v3.sun.position.set(f.x + 150, 300, f.z + 100); }
  }

  function updateHud(c) {
    if (!c) { hud.innerHTML = ""; return; }
    const kmh = Math.round((c.v || 0) * 3.6);
    hud.innerHTML = `<div class="h-name"><i style="background:${c.color}"></i>P${c.pos ?? "–"} ${esc(c.name)}</div>
      <div class="h-speed">${kmh}<small>km/h</small></div>
      <div class="h-bars"><span>THR<b style="width:${Math.round((c.thr || 0) * 100)}%"></b></span><span class="brk">BRK<b style="width:${Math.round((c.brk || 0) * 100)}%"></b></span></div>
      <div class="h-row">LAP ${c.laps ?? 0} · ${c.lt != null ? fmt(c.lt) : "—"} · <span style="color:${TYRE_COL[c.tyre] || "#fff"}">${(c.tyre || "").toUpperCase()}</span> ${Math.round((1 - (c.wear || 0)) * 100)}%${c.drs ? " · DRS" : ""}${c.st && c.st !== "track" ? " · " + c.st.toUpperCase() : ""}</div>`;
  }

  function onResize() {
    if (!renderer) return;
    const r = stage.getBoundingClientRect();
    renderer.setSize(r.width, r.height, false);
    camera.aspect = r.width / Math.max(1, r.height); camera.updateProjectionMatrix();
  }
  window.addEventListener("resize", onResize);

  let hudAt = 0;
  function loop(now) {
    requestAnimationFrame(loop);
    const dt = Math.min(0.1, (now - lastT) / 1000); lastT = now;
    if (!v3.on) return;
    if (!v3.built) { if (T) build(); else return; }
    const r = stage.getBoundingClientRect();
    if (canvas.width !== Math.round(r.width * renderer.getPixelRatio())) onResize();
    const simNow = now / 1000 * clock.rate + clock.offset;
    const cars = interpCars(simNow - 0.09 * Math.max(clock.rate, 0.0001));
    syncCars(cars, dt);
    const sel = cars.find((c) => c.id === selected);
    place(sel, dt);
    if (now - hudAt > 80) { hudAt = now; updateHud(sel); }
    renderer.render(scene, camera);
  }
  requestAnimationFrame(loop);

  setCam("chase");
  let saved = null; try { saved = localStorage.getItem("pw_3d"); } catch {}
  if (saved) { setCam(saved); const go = () => (T ? set3d(true) : setTimeout(go, 200)); go(); }
})();
