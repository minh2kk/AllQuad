// AllQuad configurator - hardware in the loop in the browser (same protocol as tools/hitl.py,
// src/comms/hitl.h): the connected FC flies a quad simulated here. Over USB (Web Serial / WebUSB) or
// the in-browser demo drone; not over WiFi (the FC takes the sensor frames on its USB console only).
// While it runs the page's own requests stop (the console belongs to HITL); at the end the FC
// restarts by itself (a real board reconnects, the demo is started again).
"use strict";
const HITL = { on: false, timer: 0, rx: new Uint8Array(0), seq: 0, auto: false, t0: 0, tPhys: 0, gpsNext: 0,
  motors: [0, 0, 0, 0], armed: false, mode: 0, fs: false, est: [0, 0, 0, 0], loopHz: 0, got: 0, sent: 0, lat: 0, latN: 0,
  sentAt: new Map(), trail: [], m: null, rc: [1500, 1500, 1000, 1500, 1000, 1500, 1000, 1000], res: null, wind: 0, refused: false };
const HITL_CMD = 0x4160;
const HITL_MODES = ["ACRO", "ANGLE", "HORIZON", "ALTHOLD", "POSHOLD", "RTH", "MISSION", "LAND", "FAILSAFE", "TURTLE",
  "CRUISE", "AUTOTUNE", "GUIDED", "SMART_RTL"];

function hitlCrc8(b, from, to) {
  let c = 0;
  for (let i = from; i < to; i++) {
    c ^= b[i];
    for (let k = 0; k < 8; k++) c = c & 0x80 ? ((c << 1) ^ 0xD5) & 0xFF : (c << 1) & 0xFF;
  }
  return c;
}
function hitlMsp(payload) {
  const n = payload.length, f = new Uint8Array(9 + n);
  f.set([0x24, 0x58, 0x3C, 0, HITL_CMD & 255, HITL_CMD >> 8, n & 255, n >> 8]);
  f.set(payload, 8);
  f[8 + n] = hitlCrc8(f, 3, 8 + n);
  return f;
}
// MSP v2 answers out of the console stream
function hitlFeed(chunk) {
  let b = HITL.rx;
  if (b.length) { const r = new Uint8Array(b.length + chunk.length); r.set(b); r.set(chunk, b.length); b = r; } else b = chunk;
  let i = 0;
  while (i + 9 <= b.length) {
    if (b[i] !== 0x24 || b[i + 1] !== 0x58) { i++; continue; }
    const len = b[i + 6] | (b[i + 7] << 8);
    if (len > 512) { i++; continue; }
    if (i + 9 + len > b.length) break;
    if (hitlCrc8(b, i + 3, i + 8 + len) !== b[i + 8 + len]) { i++; continue; }
    const dir = b[i + 2], cmd = b[i + 4] | (b[i + 5] << 8), p = b.slice(i + 8, i + 8 + len);
    i += 9 + len;
    if (cmd !== HITL_CMD) continue;
    if (dir === 0x21) { HITL.refused = true; continue; }  // '!' : armed / not on the USB console
    if (dir !== 0x3E || p.length < 36) continue;
    const dv = new DataView(p.buffer, p.byteOffset, p.byteLength);
    HITL.got++;
    const s = dv.getUint32(0, true), t = HITL.sentAt.get(s);
    if (t !== undefined) { HITL.lat += performance.now() - t; HITL.latN++; HITL.sentAt.delete(s); }
    for (let k = 0; k < 4; k++) HITL.motors[k] = dv.getFloat32(4 + k * 4, true);
    HITL.armed = !!p[20]; HITL.mode = p[21]; HITL.fs = !!(p[23] & 1);
    HITL.est = [dv.getInt16(24, true) / 10, dv.getInt16(26, true) / 10, dv.getInt16(28, true) / 10, dv.getInt32(30, true) / 100];
    HITL.loopHz = dv.getUint16(34, true);
  }
  HITL.rx = b.slice(i);
  if (HITL.rx.length > 16384) HITL.rx = HITL.rx.slice(-1024);
}

// ---- the craft: same model as src/sim and tools/hitl.py (mixer order: rear-right, front-right, rear-left, front-left)
function hitlQuad(mass, tmax) {
  const k = mass / 0.55, h = 30 * Math.PI / 360;
  return { mass, tmax, arm: 0.08, kyaw: 0.016, I: [0.0026 * k, 0.0028 * k, 0.0048 * k], tau: 0.03, drag: 0.25 * k,
    pos: [0, 0, 0], vel: [0, 0, 0], w: [0, 0, 0], acc: [0, 0, 0], q: [Math.cos(h), 0, 0, Math.sin(h)], m: [0, 0, 0, 0], ground: true, wind: [0, 0] };
}
const HQ = {
  mul: (a, b) => [a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3], a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
    a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1], a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]],
  rot(q, v) {  // body -> earth
    const [w, x, y, z] = q, tx = 2 * (y * v[2] - z * v[1]), ty = 2 * (z * v[0] - x * v[2]), tz = 2 * (x * v[1] - y * v[0]);
    return [v[0] + w * tx + y * tz - z * ty, v[1] + w * ty + z * tx - x * tz, v[2] + w * tz + x * ty - y * tx];
  },
  inv: (q) => [q[0], -q[1], -q[2], -q[3]],
  euler(q) {
    const [w, x, y, z] = q;
    return [Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)), Math.asin(Math.max(-1, Math.min(1, 2 * (w * y - z * x)))),
      Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))].map((v) => v * 180 / Math.PI);
  },
};
function hitlStep(Q, cmd, armed, dt) {
  const X = [-1, 1, -1, 1], Y = [1, 1, -1, -1], D = [-1, 1, 1, -1], G = 9.80665;
  let thrust = 0, tx = 0, ty = 0, tz = 0;
  for (let i = 0; i < 4; i++) {
    const c = armed ? Math.max(0, Math.min(1, cmd[i])) : 0;
    Q.m[i] += (c - Q.m[i]) * (dt / (Q.tau + dt));
    const t = Q.tmax * (Q.m[i] * Q.m[i] * 0.6 + Q.m[i] * 0.4);
    thrust += t; tx += -Y[i] * Q.arm * t; ty += X[i] * Q.arm * t; tz += D[i] * Q.kyaw * t;
  }
  const w = Q.w, I = Q.I, Iw = [I[0] * w[0], I[1] * w[1], I[2] * w[2]];
  const cr = [w[1] * Iw[2] - w[2] * Iw[1], w[2] * Iw[0] - w[0] * Iw[2], w[0] * Iw[1] - w[1] * Iw[0]];
  [tx, ty, tz].forEach((tq, k) => { w[k] += (tq - cr[k]) / I[k] * dt; });
  const qd = HQ.mul(Q.q, [0, w[0], w[1], w[2]]);
  let q = Q.q.map((v, k) => v + 0.5 * qd[k] * dt);
  const n = Math.hypot(...q);
  Q.q = q.map((v) => v / n);
  const f = HQ.rot(Q.q, [0, 0, -thrust]), wd = Q.ground ? [0, 0] : Q.wind;
  const F = [f[0] + (wd[0] - Q.vel[0]) * Q.drag, f[1] + (wd[1] - Q.vel[1]) * Q.drag, f[2] + Q.mass * G - Q.vel[2] * Q.drag];
  Q.acc = F.map((v) => v / Q.mass);
  for (let k = 0; k < 3; k++) { Q.vel[k] += Q.acc[k] * dt; Q.pos[k] += Q.vel[k] * dt; }
  Q.ground = Q.pos[2] >= 0;
  if (Q.ground) {
    Q.pos[2] = 0;
    if (Q.vel[2] > 0) Q.vel = [0, 0, 0];
    Q.vel[0] *= 0.9; Q.vel[1] *= 0.9;
    if (Q.acc[2] > 0) Q.acc = [0, 0, 0];
    Q.w = Q.w.map((v) => v * 0.9);
  }
}
const hitlGauss = (s) => { let u = 0, v = 0; while (!u) u = Math.random(); while (!v) v = Math.random(); return s * Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v); };

// one sensor frame (layout: src/comms/hitl.cpp)
function hitlFrame(Q, now, gps) {
  const b = new ArrayBuffer(108), dv = new DataView(b), f = (o, v) => dv.setFloat32(o, v, true);
  HITL.seq = (HITL.seq + 1) >>> 0;
  dv.setUint32(0, HITL.seq, true);
  for (let k = 0; k < 3; k++) f(4 + k * 4, Q.w[k] + hitlGauss(0.004));
  const sf = HQ.rot(HQ.inv(Q.q), [Q.acc[0], Q.acc[1], Q.acc[2] - 9.80665]);
  for (let k = 0; k < 3; k++) f(16 + k * 4, sf[k] + hitlGauss(0.3));
  const alt = -Q.pos[2];
  f(28, 101325 * Math.pow(1 - (alt + hitlGauss(0.08)) / 44330, 5.255));
  const mb = HQ.rot(HQ.inv(Q.q), [0.22, 0, 0.42]);
  for (let k = 0; k < 3; k++) f(32 + k * 4, mb[k] + hitlGauss(0.003));
  let flags = 8 | (HITL.auto ? 4 : 0);  // battery, RC from the page in the check flight
  if (gps) {
    flags |= 1;
    const lat0 = 21.0285, lon0 = 105.8542, n = Q.pos[0] + hitlGauss(0.3), e = Q.pos[1] + hitlGauss(0.3);
    dv.setInt32(48, Math.round((lat0 + n / 6378137 * 180 / Math.PI) * 1e7), true);
    dv.setInt32(52, Math.round((lon0 + e / (6378137 * Math.cos(lat0 * Math.PI / 180)) * 180 / Math.PI) * 1e7), true);
    f(56, 10 + alt); f(60, Q.vel[0] + hitlGauss(0.1)); f(64, Q.vel[1] + hitlGauss(0.1)); f(68, Q.vel[2]); f(72, 1.2);
  }
  dv.setUint8(44, flags);
  dv.setUint8(45, 14);
  for (let k = 0; k < 8; k++) dv.setUint16(80 + k * 2, HITL.rc[k], true);
  f(96, 16.4 - 0.004 * now);
  f(100, 0.5 + 18 * (Q.m[0] + Q.m[1] + Q.m[2] + Q.m[3]) / 4);
  HITL.sentAt.set(HITL.seq, performance.now());
  if (HITL.sentAt.size > 1000) HITL.sentAt.clear();
  return hitlMsp(new Uint8Array(b));
}

// the check flight (same script and limits as tools/hitl.py --auto)
function hitlPilot(t) {
  const rc = HITL.rc;
  rc[4] = t >= 4 && t < 44 ? 2000 : 1000;
  rc[2] = t < 4.5 ? 1000 : t < 7 ? 1580 : 1500;
  rc[5] = t >= 7 ? 2000 : 1500;
  rc[6] = t >= 10 ? 1500 : 1000;
  rc[7] = t >= 28 ? 2000 : 1000;
  rc[1] = t >= 18 && t < 19.5 ? 1650 : 1500;
}
function hitlMeasure(t, Q) {
  const r = HITL.res, alt = -Q.pos[2], [tr, tp] = HQ.euler(Q.q);
  if (HITL.armed) r.armed = true;
  if (HITL.armed && t > 8 && HITL.got) r.attErr = Math.max(r.attErr, Math.abs(HITL.est[0] - tr), Math.abs(HITL.est[1] - tp));
  if (t >= 12 && t < 18) r.alt.push(alt);
  if (!r.hold && t >= 12) r.hold = [Q.pos[0], Q.pos[1]];
  if (r.hold && t >= 12 && t < 28) r.drift = Math.max(r.drift, Math.hypot(Q.pos[0] - r.hold[0], Q.pos[1] - r.hold[1]));
  if (t > 28 && r.landed === null && !HITL.armed && r.armed) r.landed = t;
}
function hitlVerdict(dur) {
  const r = HITL.res, rate = HITL.got / dur, n = r.alt.length, mean = n ? r.alt.reduce((a, b) => a + b, 0) / n : 0;
  const sd = n ? Math.sqrt(r.alt.reduce((a, b) => a + (b - mean) ** 2, 0) / n) : 99, lat = HITL.latN ? HITL.lat / HITL.latN : 999;
  return [
    ["FC trả lời khung HITL", rate > 150, `${Math.round(rate)} khung/s, trễ ${lat.toFixed(1)} ms`],
    ["Arm và bay lên", r.armed && n > 0 && Math.max(...r.alt) > 1, `treo ở ${mean.toFixed(1)} m`],
    ["ALTHOLD giữ độ cao", n > 0 && sd < 0.5, `dao động ${sd.toFixed(2)} m`],
    ["POSHOLD giữ chỗ (có đẩy cần)", r.drift < 4, `lệch tối đa ${r.drift.toFixed(1)} m`],
    ["Tư thế ước lượng theo kịp", r.attErr < 5, `lệch tối đa ${r.attErr.toFixed(1)}°`],
    ["LAND: hạ cánh và disarm", r.landed !== null, r.landed !== null ? `lúc ${r.landed.toFixed(1)} s` : "chưa disarm"],
  ];
}

function hitlTick() {
  const now = (performance.now() - HITL.t0) / 1000, Q = HITL.m;
  if (!LINK.open || !LINK.send) { hitlStop("Mất kết nối với FC."); return; }
  if (HITL.refused) { hitlStop("FC từ chối HITL: disarm trước, và chỉ qua cáp USB."); return; }
  if (HITL.auto && now > 50) { hitlStop(null); return; }
  while (HITL.tPhys < now) { hitlStep(Q, HITL.motors, HITL.armed, 0.001); HITL.tPhys += 0.001; }
  if (HITL.auto) { hitlPilot(now); hitlMeasure(now, Q); }
  const gps = now >= HITL.gpsNext;
  if (gps) HITL.gpsNext = now + 0.1;
  LINK.send(hitlFrame(Q, now, gps)).catch(() => {});
  HITL.sent++;
  if (HITL.sent % 25 === 0) {
    HITL.trail.push([Q.pos[0], Q.pos[1]]);
    if (HITL.trail.length > 400) HITL.trail.shift();
    hitlDraw(now);
  }
}
function hitlStart(auto) {
  if (!LINK.open || !LINK.send) { toast("Cần nối FC qua cáp USB (hoặc chạy drone ảo) trước."); return; }
  if (HITL.on) return;
  Object.assign(HITL, { on: true, auto, rx: new Uint8Array(0), seq: 0, t0: performance.now(), tPhys: 0, gpsNext: 0, motors: [0, 0, 0, 0],
    armed: false, mode: 0, fs: false, got: 0, sent: 0, lat: 0, latN: 0, trail: [], refused: false,
    rc: [1500, 1500, 1000, 1500, 1000, 1500, 1000, 1000], res: { armed: false, alt: [], hold: null, drift: 0, attErr: 0, landed: null } });
  HITL.sentAt.clear();
  const wind = parseFloat($("hitlWind").value) || 0;
  HITL.m = hitlQuad(parseFloat($("hitlMass").value) || 0.55, parseFloat($("hitlTmax").value) || 3.6);
  HITL.m.wind = [0, wind];
  LINK.tap = hitlFeed;   // the console stream goes to HITL, the page's requests stop
  LINK.hitl = true;
  $("hitlRes").innerHTML = "";
  $("hitlStop").disabled = false;
  $("hitlAuto").disabled = $("hitlManual").disabled = true;
  HITL.timer = setInterval(hitlTick, 4);  // 250 frames/s (browser timers do not go faster)
  setTimeout(() => { if (HITL.on) $("sSub").textContent = "HITL đang chạy: FC bay drone mô phỏng, motor đứng yên."; }, 1500);
}
function hitlStop(err) {
  if (!HITL.on) return;
  clearInterval(HITL.timer);
  HITL.on = false;
  const dur = (performance.now() - HITL.t0) / 1000;
  // leave HITL: the FC restarts (disarmed) and is back on its own sensors
  const exit = new Uint8Array(108);
  exit[44] = 0x80;
  if (LINK.send) LINK.send(hitlMsp(exit)).catch(() => {});
  LINK.tap = null;
  LINK.hitl = false;
  $("hitlStop").disabled = true;
  $("hitlAuto").disabled = $("hitlManual").disabled = false;
  const box = $("hitlRes");
  if (err) box.innerHTML = `<p style="color:var(--danger)">${err}</p>`;
  else if (HITL.auto) {
    const v = hitlVerdict(dur), ok = v.filter((x) => x[1]).length;
    box.innerHTML = v.map(([n, g, d]) => `<div class="hitl-r" style="color:var(${g ? "--ok" : "--danger"})">${g ? "✓" : "✗"} ${n}: <span class="num">${d}</span></div>`).join("") +
      `<p><b>${ok}/${v.length} mục đạt</b></p>`;
    window.AQ_HITL_RESULT = { pass: ok, total: v.length, items: v };
  }
  setTimeout(() => linkLost(err || "HITL đã dừng: FC khởi động lại về cảm biến thật. Kết nối lại khi nó hiện lên."), 600);
}
function hitlDraw(now) {
  const Q = HITL.m, cv = $("hitlCv");
  if (!cv || !cv.offsetParent) return;
  const { g, w, h } = setupCanvas(cv);
  g.fillStyle = css("--sheet") || "#111"; g.fillRect(0, 0, w, h);
  // track from above (north up), 2 m grid
  const ext = Math.max(10, ...HITL.trail.map(([n, e]) => Math.max(Math.abs(n), Math.abs(e)))) * 1.2;
  const cx = h / 2, sc = (h / 2 - 8) / ext, P = (n, e) => [cx + e * sc, h / 2 - n * sc];
  g.strokeStyle = css("--grid") || "#333"; g.lineWidth = 1;
  for (let k = -Math.ceil(ext / 2) * 2; k <= ext; k += 2) {
    const [x] = P(0, k), [, y] = P(k, 0);
    g.beginPath(); g.moveTo(x, 4); g.lineTo(x, h - 4); g.stroke();
    g.beginPath(); g.moveTo(cx - h / 2 + 4, y); g.lineTo(cx + h / 2 - 4, y); g.stroke();
  }
  g.strokeStyle = css("--yaw") || "#48f"; g.lineWidth = 2; g.beginPath();
  HITL.trail.forEach(([n, e], i) => { const [x, y] = P(n, e); i ? g.lineTo(x, y) : g.moveTo(x, y); });
  g.stroke();
  const [x, y] = P(Q.pos[0], Q.pos[1]), yaw = HQ.euler(Q.q)[2] * Math.PI / 180;
  g.fillStyle = css("--amber") || "#fa0";
  g.beginPath(); g.moveTo(x + 9 * Math.sin(yaw), y - 9 * Math.cos(yaw)); g.lineTo(x - 5 * Math.cos(yaw), y - 5 * Math.sin(yaw)); g.lineTo(x + 5 * Math.cos(yaw), y + 5 * Math.sin(yaw)); g.fill();
  // height bar: true (filled) and the FC's estimate (line)
  const bx = h + 18, bw = 16, top = 8, bh = h - 16, amax = Math.max(10, -Q.pos[2] * 1.3);
  g.strokeStyle = css("--rule") || "#555"; g.strokeRect(bx, top, bw, bh);
  g.fillStyle = css("--ok") || "#3c6";
  const a = Math.max(0, -Q.pos[2]);
  g.fillRect(bx + 1, top + bh - a / amax * bh, bw - 2, a / amax * bh);
  g.strokeStyle = css("--ink") || "#fff"; g.beginPath();
  const ey = top + bh - Math.max(0, HITL.est[3]) / amax * bh; g.moveTo(bx - 4, ey); g.lineTo(bx + bw + 4, ey); g.stroke();
  // text
  const [tr, tp] = HQ.euler(Q.q), rate = HITL.got / Math.max(now, 0.1), lat = HITL.latN ? HITL.lat / HITL.latN : 0;
  g.fillStyle = css("--ink") || "#fff"; g.font = "12px " + (css("--mono") || "monospace");
  const lines = [
    `${now.toFixed(1)} s   ${HITL_MODES[HITL.mode] || HITL.mode}   ${HITL.armed ? "ARMED" : "disarmed"}${HITL.fs ? "   FAILSAFE" : ""}`,
    `cao ${a.toFixed(1)} m (FC ${HITL.est[3].toFixed(1)})`,
    `N ${Q.pos[0].toFixed(1)}  E ${Q.pos[1].toFixed(1)} m`,
    `roll ${tr.toFixed(1)}° (FC ${HITL.est[0].toFixed(1)})`,
    `pitch ${tp.toFixed(1)}° (FC ${HITL.est[1].toFixed(1)})`,
    `liên kết ${Math.round(rate)}/s, trễ ${lat.toFixed(1)} ms`,
    `vòng bay FC ${HITL.loopHz} Hz`,
  ];
  lines.forEach((t, i) => g.fillText(t, bx + bw + 16, 22 + i * 18));
}
function hitlInit() {
  const host = document.querySelector("#tab-system .cards");
  if (!host || $("hitlPanel")) return;
  const p = document.createElement("div");
  p.className = "panel"; p.id = "hitlPanel";
  p.innerHTML = `<div class="card-h"><span class="dot"></span><h3>Bay thử trên bàn (HITL)</h3></div>
    <p class="muted" style="margin-top:0">FC thật bay một chiếc drone mô phỏng trong trang này: trang gửi gyro, gia tốc, baro, la bàn, GPS qua USB, FC trả lệnh motor. <b>Motor không quay.</b> Chỉ qua cáp USB (hoặc drone ảo), FC phải đang disarm. Giữ tab này mở trong lúc chạy. Hết lượt FC tự khởi động lại về cảm biến thật. Bản dòng lệnh: <span class="num">tools/hitl.py</span>.</p>
    <div class="row">
      <button class="btn primary" id="hitlAuto">Bay kiểm tra tự động (50 s)</button>
      <button class="btn" id="hitlManual">Bay bằng tay điều khiển</button>
      <button class="btn danger" id="hitlStop" disabled>Dừng</button>
    </div>
    <div class="row" style="margin-top:8px">
      <label>Gió <input id="hitlWind" type="number" step="0.5" min="0" max="10" value="0" style="width:4.5em"> m/s</label>
      <label>Khối lượng <input id="hitlMass" type="number" step="0.05" min="0.2" max="3" value="0.55" style="width:4.5em"> kg</label>
      <label>Lực đẩy max <input id="hitlTmax" type="number" step="0.1" min="1" max="20" value="3.6" style="width:4.5em"> N/motor</label>
    </div>
    <canvas id="hitlCv" style="width:100%;height:190px;margin-top:10px"></canvas>
    <div id="hitlRes"></div>
    <p class="muted" style="margin-bottom:0">Kiểm tra tự động dùng công tắc mặc định (AUX1 arm, AUX2 ALTHOLD, AUX3 POSHOLD, AUX4 LAND). Bay bằng tay: receiver thật trên FC vẫn chạy; tắt tay điều khiển giữa chừng để thử failsafe.</p>`;
  host.prepend(p);
  $("hitlAuto").onclick = () => hitlStart(true);
  $("hitlManual").onclick = () => hitlStart(false);
  $("hitlStop").onclick = () => hitlStop(null);
  const show = () => { p.hidden = !LINK.open && !HITL.on; };  // a USB / demo / simulator link (not WiFi)
  show();
  setInterval(show, 1000);
}
hitlInit();
