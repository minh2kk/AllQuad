// ============================================================================================
// AllQuad configurator - laptop tools: sensors + 3D model, motor test, ports / VTX / LED, OSD
// layout, CLI, mission editor, presets, firmware flasher (ESP32 ROM loader over Web Serial,
// STM32 DfuSe over WebUSB). Bundled into index.html by tools/embed_web.py.
// Uses the page helpers: $, S, api, LINK, setEdit, val, fmt, plot, setupCanvas, css, toast...
// ============================================================================================
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
const esc = (t) => String(t).replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" })[c]);
const has = (n) => !!S.params[n];

function onParamsLoaded() {
  buildPorts();
  buildVtx();
  buildLed();
  buildOsd();
  buildPresets();
  buildMotors();
  if (!$("tab-blackbox").hidden) sdRefresh();
}
function onParamEdit(n) {
  if (/^serial\d_fn|^udp_fn|^rx_protocol/.test(n)) buildPorts();
  if (n === "vtxt_powers" || /^vtxt_mw/.test(n)) buildVtx();
  else if (/^vtx_|^vtxt_|^serial\d_fn/.test(n)) updateVtx();
  if (/^led_/.test(n)) updateLed();
  if (/^osd_/.test(n)) drawOsd();
  if (n === "yaw_reversed") drawMotorMap();
  if (n === "sd_mode") sdPinRows();
  else if (/^sd_/.test(n)) sdWiring();
}
let lastTab = "overview";
function onTabShown(t) {
  if (lastTab === "motors" && t !== "motors") motorsOff("tab");
  lastTab = t;
  if (t === "sensors") sensStart();
  if (t === "motors") drawMotorMap();
  if (t === "osd") drawOsd();
  if (t === "ports") updateLed();
  if (t === "cli") setTimeout(() => $("cliIn").focus(), 50);
  if (t === "blackbox") sdRefresh();
}

// ---------------------------------------------------------------- sensors + 3D model
const SENS = { rate: 20, timer: null, n: 200, gyro: [[], [], []], acc: [[], [], []], mag: [[], [], []], baro: [], range: [], yaw0: null, last: null };
function sensStart() {
  if (SENS.timer || !shown("sensors")) return;
  sensPoll();
}
async function sensPoll() {
  SENS.timer = null;
  if (!shown("sensors") || document.hidden || S.needLogin) return;
  const t0 = performance.now();
  try { sensAdd(await api("/api/sensors", { timeout: 1500 })); } catch (e) {}
  SENS.timer = setTimeout(sensPoll, Math.max(10, 1000 / SENS.rate - (performance.now() - t0)));
}
function sensAdd(j) {
  SENS.last = j;
  const push = (arr, v) => { arr.push(v); if (arr.length > SENS.n) arr.shift(); };
  for (let a = 0; a < 3; a++) { push(SENS.gyro[a], j.gyro[a]); push(SENS.acc[a], j.acc[a]); push(SENS.mag[a], j.magok ? j.mag[a] : NaN); }
  push(SENS.baro, j.baro.ok ? j.baro.alt : NaN);
  push(SENS.range, j.range >= 0 ? j.range : NaN);
  const C = ["--roll", "--pitch", "--yaw"];
  const tri = (b) => b.map((y, a) => ({ y, color: C[a], width: 1.6 }));
  plot($("chGyro"), tri(SENS.gyro), { xmin: 0, xmax: SENS.n - 1 });
  plot($("chAcc"), tri(SENS.acc), { xmin: 0, xmax: SENS.n - 1 });
  plot($("chMag"), tri(SENS.mag), { xmin: 0, xmax: SENS.n - 1 });
  plot($("chBaro"), [{ y: SENS.baro, color: "--ink", width: 1.8 }, { y: SENS.range, color: "--pitch", width: 1.6 }], { xmin: 0, xmax: SENS.n - 1 });
  const g = Math.hypot(...j.acc);
  const rows = [
    ["Gyro", j.gyro.map((v) => v.toFixed(1)).join(" / ") + " °/s"],
    ["Gia tốc", `${j.acc.map((v) => v.toFixed(2)).join(" / ")} (|a| ${g.toFixed(2)})`],
    ["Compass", j.magok ? j.mag.map((v) => v.toFixed(3)).join(" / ") : "không có"],
    ["Baro", j.baro.ok ? `${j.baro.alt.toFixed(2)} m, ${(j.baro.pa / 100).toFixed(1)} hPa, ${j.baro.temp.toFixed(1)} °C` : "không có"],
    ["Khoảng cách", j.range >= 0 ? j.range.toFixed(2) + " m" : "không có"],
    ["Optical flow", j.flowq >= 0 ? `${j.flow[0].toFixed(2)} / ${j.flow[1].toFixed(2)} rad/s, chất lượng ${j.flowq}` : "không có"],
  ];
  setHtml($("sensKv"), rows.map(([k, v]) => `<dt>${k}</dt><dd>${v}</dd>`).join(""));
  const escOn = j.rpm.some((r) => r >= 0);
  $("escPanel").hidden = !escOn;
  if (escOn) {
    const v = j.escv || [], a = j.esca || [], dash = "—";
    setHtml($("escTable"), "<tr><th>ESC</th><th>RPM</th><th>Nhiệt độ</th><th>Điện áp</th><th>Dòng</th></tr>" +
      j.rpm.map((r, i) => `<tr><td>M${i + 1}</td><td class="num">${r >= 0 ? r : dash}</td>` +
        `<td class="num">${j.esct[i] >= 0 ? j.esct[i] + " °C" : dash}</td>` +
        `<td class="num">${v[i] >= 0 ? v[i].toFixed(2) + " V" : dash}</td>` +
        `<td class="num">${a[i] >= 0 ? a[i] + " A" : dash}</td></tr>`).join(""));
  }
  draw3d(j.q);
}
segment("sensRate", "sensRateSel", () => { SENS.rate = S.sensRateSel || 20; });
$("modelZero").onclick = () => { SENS.yaw0 = null; if (SENS.last) draw3d(SENS.last.q); };

// quaternion (body FRD -> NED) to rotation matrix
function qMat(q) {
  const [w, x, y, z] = q;
  return [
    [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
    [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
    [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
  ];
}
const mv = (m, v) => [m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2], m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2], m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2]];
// a quad X model in body coordinates (x forward, y right, z down), faces with colours
const MODEL = (() => {
  const F = [];
  const box = (cx, cy, cz, sx, sy, sz, col, rot = 0) => {
    const c = Math.cos(rot), s = Math.sin(rot);
    const P = (dx, dy, dz) => [cx + dx * c - dy * s, cy + dx * s + dy * c, cz + dz];
    const v = [];
    for (const dz of [-sz, sz]) for (const dy of [-sy, sy]) for (const dx of [-sx, sx]) v.push(P(dx, dy, dz));
    // v index: bit0 x, bit1 y, bit2 z
    [[0, 1, 3, 2], [4, 6, 7, 5], [0, 4, 5, 1], [2, 3, 7, 6], [0, 2, 6, 4], [1, 5, 7, 3]].forEach((f) => F.push({ p: f.map((i) => v[i]), col }));
  };
  const disc = (cx, cy, cz, r, col, alpha = 1) => {
    const p = [];
    for (let i = 0; i < 18; i++) { const a = i / 18 * Math.PI * 2; p.push([cx + Math.cos(a) * r, cy + Math.sin(a) * r, cz]); }
    F.push({ p, col, alpha, two: true });
  };
  box(0, 0, 0, 0.26, 0.15, 0.06, "#3B4A5C");                       // body
  box(0.27, 0, -0.07, 0.07, 0.07, 0.02, "#E8742A");                // nose marker (front)
  const arms = [[0.62, -0.62, "FL"], [0.62, 0.62, "FR"], [-0.62, -0.62, "RL"], [-0.62, 0.62, "RR"]];
  arms.forEach(([x, y]) => box(x / 2, y / 2, 0, 0.47, 0.035, 0.025, x > 0 ? "#E8742A" : "#5E6E80", Math.atan2(y, x)));
  arms.forEach(([x, y]) => { box(x, y, -0.06, 0.07, 0.07, 0.05, "#1E2833"); disc(x, y, -0.13, 0.3, x > 0 ? "#F2A15E" : "#9FB0BF", 0.45); });
  return F;
})();
function draw3d(q) {
  const cv = $("model3d");
  if (!cv || !shown("sensors")) return;
  const r = draw3dOn(cv, q, SENS);
  // heading: the corrected one (compass / GPS / EKF) as on the navigation tab, not the raw AHRS yaw
  const hdg = S.status && S.status.att ? S.status.att[2] : r.yaw;
  $("modelAtt").textContent = `roll ${r.roll.toFixed(1)}°  pitch ${r.pitch.toFixed(1)}°  hướng ${((hdg % 360) + 360) % 360 | 0}°`;
}
// quaternion (body FRD -> NED) from roll / pitch / yaw in degrees (the status stream sends angles)
function qEuler(rd, pd, yd) {
  const k = Math.PI / 360, cr = Math.cos(rd * k), sr = Math.sin(rd * k), cp = Math.cos(pd * k), sp = Math.sin(pd * k), cy = Math.cos(yd * k), sy = Math.sin(yd * k);
  return [cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy];
}
// draws the quad model on any canvas; view.yaw0 = heading the camera looks along (null: take the current one)
// opt.grid = false leaves out the ground grid (small canvases); returns the angles in degrees
function draw3dOn(cv, q, view, opt) {
  opt = opt || {};
  const { g, w, h } = setupCanvas(cv);
  let R = qMat(q);
  const yaw = Math.atan2(R[1][0], R[0][0]);
  if (view.yaw0 === null || view.yaw0 === undefined) view.yaw0 = yaw;
  const c = Math.cos(-view.yaw0), s = Math.sin(-view.yaw0);
  const el = 0.45, ce = Math.cos(el), se = Math.sin(el), dist = 3.6, f = Math.min(w, h) * (opt.zoom || 1.7);
  const tf = (pb) => {
    const p = mv(R, pb);                                  // NED
    const n = c * p[0] - s * p[1], e = s * p[0] + c * p[1], d = p[2];  // viewer heading removed
    const x = e, depth = n * ce + d * se + dist, up = n * se - d * ce;   // camera behind and above
    return [w / 2 + f * x / depth, h / 2 - f * up / depth + 10, depth];
  };
  const light = [0.4, -0.3, -0.85];
  const faces = MODEL.map((fc) => {
    const pts = fc.p.map(tf);
    const z = pts.reduce((a, p) => a + p[2], 0) / pts.length;
    // face normal in NED for shading / back-face culling
    const a = mv(R, fc.p[0]), b = mv(R, fc.p[1]), cc = mv(R, fc.p[2]);
    const u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]], v = [cc[0] - a[0], cc[1] - a[1], cc[2] - a[2]];
    let nrm = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
    const L = Math.hypot(...nrm) || 1;
    nrm = nrm.map((x) => x / L);
    const shade = 0.55 + 0.45 * Math.abs(nrm[0] * light[0] + nrm[1] * light[1] + nrm[2] * light[2]);
    return { pts, z, col: fc.col, alpha: fc.alpha || 1, shade };
  }).sort((p, q2) => q2.z - p.z);
  g.clearRect(0, 0, w, h);
  // ground grid for reference
  g.strokeStyle = css("--grid"); g.lineWidth = 1;
  for (let i = -3; i <= 3 && opt.grid !== false; i++) {
    const A = (n, e) => { const depth = n * ce + 0.9 * se + dist, up = n * se - 0.9 * ce; return [w / 2 + f * e / depth, h / 2 - f * up / depth + 10]; };
    let p1 = A(-3, i * 0.6), p2 = A(3, i * 0.6);
    g.beginPath(); g.moveTo(...p1); g.lineTo(...p2); g.stroke();
    p1 = A(i, -1.8); p2 = A(i, 1.8);
    g.beginPath(); g.moveTo(...p1); g.lineTo(...p2); g.stroke();
  }
  for (const fc of faces) {
    const hex = fc.col, r = parseInt(hex.slice(1, 3), 16) * fc.shade, gg = parseInt(hex.slice(3, 5), 16) * fc.shade, b = parseInt(hex.slice(5, 7), 16) * fc.shade;
    g.fillStyle = `rgba(${r | 0},${gg | 0},${b | 0},${fc.alpha})`;
    g.beginPath();
    fc.pts.forEach((p, i) => (i ? g.lineTo(p[0], p[1]) : g.moveTo(p[0], p[1])));
    g.closePath();
    g.fill();
  }
  const roll = Math.atan2(R[2][1], R[2][2]) * 180 / Math.PI, pitch = -Math.asin(Math.max(-1, Math.min(1, R[2][0]))) * 180 / Math.PI;
  return { roll, pitch, yaw: yaw * 180 / Math.PI };
}

// ---------------------------------------------------------------- motor test
const MT = { v: [0, 0, 0, 0], master: 0, timer: null, busy: false };
// Betaflight quad X numbering (props in): M4 FL CW, M2 FR CCW, M3 RL CCW, M1 RR CW
const MOTOR_XY = [[1, -1], [1, 1], [-1, -1], [-1, 1]];  // [right, front] for M1..M4
const MOTOR_CW = [true, false, false, true];
function buildMotors() {
  const host = $("mtSliders");
  if (!host || host.childElementCount) { drawMotorMap(); return; }
  const mk = (label, i) => {
    const d = document.createElement("div");
    d.className = "slider" + (i < 0 ? " master" : "");
    d.innerHTML = `<span>${label}</span><input type="range" min="0" max="100" step="1" value="0" aria-label="${label}"><span class="num">0%</span>`;
    const r = d.querySelector("input"), out = d.children[2];
    r.oninput = () => {
      const v = +r.value / 100;
      out.textContent = r.value + "%";
      if (i < 0) {
        MT.master = v;
        for (let k = 0; k < 4; k++) { MT.v[k] = v; const s = host.children[k + 1]; s.querySelector("input").value = r.value; s.children[2].textContent = r.value + "%"; }
      } else MT.v[i] = v;
      drawMotorMap();
      motorSend();
    };
    host.appendChild(d);
  };
  mk("Tất cả", -1);
  for (let i = 0; i < 4; i++) mk("Motor " + (i + 1), i);
  drawMotorMap();
}
async function motorSend() {
  const any = MT.v.some((x) => x > 0);
  clearTimeout(MT.timer);
  if (MT.busy) { MT.again = true; return; }
  MT.busy = true;
  try {
    await api(`/api/motor?m1=${MT.v[0]}&m2=${MT.v[1]}&m3=${MT.v[2]}&m4=${MT.v[3]}`, { method: "POST", timeout: 900 });
  } catch (e) {
    if (e.code === 423) { toast("Không thử motor khi đang arm."); motorsOff("armed"); }
  }
  MT.busy = false;
  if (MT.again) { MT.again = false; return motorSend(); }
  if (any) MT.timer = setTimeout(motorSend, 300);  // keep alive: the FC stops by itself after 1 s
}
function motorsOff(why) {
  const was = MT.v.some((x) => x > 0);
  MT.v = [0, 0, 0, 0];
  document.querySelectorAll("#mtSliders input").forEach((r) => { r.value = 0; r.parentElement.children[2].textContent = "0%"; });
  clearTimeout(MT.timer);
  if (was || why === "button") api("/api/motor?m1=0&m2=0&m3=0&m4=0", { method: "POST", timeout: 900 }).catch(() => {});
  drawMotorMap();
}
$("mtArm").onchange = (e) => { $("mtLock").disabled = !e.target.checked; if (!e.target.checked) motorsOff("lock"); };
$("mtStop").onclick = () => motorsOff("button");
document.addEventListener("visibilitychange", () => { if (document.hidden) motorsOff("hidden"); });
window.addEventListener("pagehide", () => motorsOff("leave"));
function drawMotorMap() {
  const cv = $("motorMap");
  if (!cv || !shown("motors")) return;
  const { g, w, h } = setupCanvas(cv);
  const cx = w / 2, cy = h / 2 + 6, sp = Math.min(w, h) * 0.3, R = Math.min(w, h) * 0.15;
  const rev = val("yaw_reversed") ? true : false;
  g.clearRect(0, 0, w, h);
  g.strokeStyle = css("--ink-2"); g.lineWidth = 6; g.lineCap = "round";
  MOTOR_XY.forEach(([x, y]) => { g.beginPath(); g.moveTo(cx, cy); g.lineTo(cx + x * sp, cy - y * sp); g.stroke(); });
  g.fillStyle = css("--ink"); g.fillRect(cx - 18, cy - 24, 36, 48);
  g.fillStyle = css("--amber"); g.beginPath(); g.moveTo(cx, cy - 40); g.lineTo(cx - 9, cy - 28); g.lineTo(cx + 9, cy - 28); g.fill();
  g.font = `600 13px ${css("--num")}`; g.textAlign = "center"; g.textBaseline = "middle";
  MOTOR_XY.forEach(([x, y], i) => {
    const mx = cx + x * sp, my = cy - y * sp, on = MT.v[i];
    g.fillStyle = on > 0 ? `rgba(224,155,18,${0.25 + on * 0.75})` : css("--sheet");
    g.strokeStyle = css("--ink"); g.lineWidth = 2;
    g.beginPath(); g.arc(mx, my, R, 0, Math.PI * 2); g.fill(); g.stroke();
    // rotation arrow
    const cw = MOTOR_CW[i] !== rev;
    const a0 = -2.3, a1 = 0.4, rr = R * 0.72;
    g.strokeStyle = css("--yaw"); g.lineWidth = 2.5;
    g.beginPath(); g.arc(mx, my, rr, a0, a1); g.stroke();
    const ae = cw ? a1 : a0, dir = cw ? 1 : -1;
    const tx = mx + Math.cos(ae) * rr, ty = my + Math.sin(ae) * rr;
    const tang = ae + dir * Math.PI / 2;
    g.fillStyle = css("--yaw");
    g.beginPath(); g.moveTo(tx + Math.cos(tang) * 8, ty + Math.sin(tang) * 8);
    g.lineTo(tx + Math.cos(ae) * 6, ty + Math.sin(ae) * 6); g.lineTo(tx - Math.cos(ae) * 6, ty - Math.sin(ae) * 6); g.fill();
    g.fillStyle = css("--ink"); g.fillText("M" + (i + 1), mx, my);
  });
  g.fillStyle = css("--ink-2"); g.font = `12px ${css("--body")}`; g.fillText("phía trước", cx, 12);
  $("motorDirNote").textContent = rev ? "Chiều quay: cánh ra ngoài (yaw_reversed 1). M1 và M4 quay ngược chiều kim đồng hồ."
    : "Chiều quay: cánh vào trong (mặc định Betaflight). M1 và M4 quay theo chiều kim đồng hồ, M2 và M3 ngược lại. Sai chiều thì đảo trong ESC Configurator.";
}
$("escRelease").onclick = async () => {
  if (!LINK.open || LINK.kind !== "usb") return toast("Trang không giữ cổng USB nào.");
  motorsOff("release");
  await closeLink();
  linkState(false);
  showConnect("Đã nhả cổng USB. Mở ESC Configurator và chọn cổng này. Xong thì quay lại đây kết nối lại.");
};

// ---------------------------------------------------------------- ports
const PORT_FN = [[0, "Không dùng"], [1, "Receiver (CRSF / SBUS / iBUS)"], [2, "GPS"], [3, "MAVLink (trạm mặt đất)"], [4, "MSP (cảm biến flow / khoảng cách)"],
  [5, "OSD kính số (MSP DisplayPort)"], [6, "Telemetry ESC"], [7, "Cảm biến khoảng cách TFmini"], [8, "Blackbox (OpenLog)"], [9, "VTX SmartAudio"], [10, "VTX Tramp"],
  [11, "Configurator (cầu WiFi ESP32-C3 / Bluetooth)"]];
const BAUDS = [0, 9600, 19200, 38400, 57600, 115200, 230400, 250000, 400000, 460800, 921600, 1000000, 2000000];
// [param prefix, label, hal::Port index] - a row is shown when the board has that port
const PORTS = [["serial1", "SERIAL1", 0], ["serial2", "SERIAL2", 1], ["serial3", "SERIAL3", 2], ["serial4", "SERIAL4", 5],
  ["serial5", "SERIAL5", 6], ["serial6", "SERIAL6", 7], ["serial7", "SERIAL7", 8], ["udp", "WiFi UDP (ESP32)", 3]];
const portShown = ([p, , idx]) => has(p + "_fn") && (S.portMask === undefined || ((S.portMask >> idx) & 1) || idx < 3);
function selectHtml(n, opts) {
  return `<select data-p="${n}" aria-label="${n}">${opts.map(([v, t]) => `<option value="${v}">${esc(t)}</option>`).join("")}</select>`;
}
function bindSelects(host) {
  host.querySelectorAll("select[data-p]").forEach((el) => {
    const n = el.dataset.p;
    el.value = String(fmt(n, val(n)));
    el.onchange = () => setEdit(n, +el.value);
  });
  host.querySelectorAll("input[data-p]").forEach((el) => {
    const n = el.dataset.p;
    if (el.type === "checkbox") { el.checked = !!val(n); el.onchange = () => setEdit(n, el.checked ? 1 : 0); }
    else if (el.type === "range") { el.value = val(n); el.oninput = () => setEdit(n, +el.value); }
    else if (el.type === "color") { el.value = "#" + (val(n) | 0).toString(16).padStart(6, "0"); el.oninput = () => setEdit(n, parseInt(el.value.slice(1), 16)); }
    else { el.value = fmt(n, val(n)); el.onchange = () => setEdit(n, el.value); }
  });
}
function buildPorts() {
  const host = $("portTable");
  if (!host || !has("serial1_fn")) return;
  const rows = PORTS.filter(portShown).map(([p, label, idx]) => {
    const fn = val(p + "_fn"), fixed = [0, 1, 2, 9, 10].includes(fn) || p === "udp";
    const opts = PORT_FN.filter(([v]) => p !== "udp" || [0, 3, 4].includes(v) || v === fn);
    const baud = has(p + "_baud") && !fixed ? selectHtml(p + "_baud", BAUDS.map((b) => [b, b ? String(b) : "mặc định"])) : `<span class="muted">${fn === 1 ? "theo giao thức" : fn === 2 ? "tự cấu hình" : fn >= 9 ? (fn === 9 ? "4800" : "9600") : "—"}</span>`;
    const extra = fn === 1 && has("rx_protocol") ? selectHtml("rx_protocol", [[0, "CRSF / ELRS"], [1, "SBUS"], [2, "iBUS"]]) : "";
    const pins = S.portPins && S.portPins[idx];  // e.g. "USART1 RX PA10 TX PA9": what to wire
    return `<tr><td><b>${label}</b>${pins ? `<br><small class="muted num">${esc(pins)}</small>` : ""}</td><td>${selectHtml(p + "_fn", opts)}</td><td>${extra}</td><td>${baud}</td></tr>`;
  });
  host.innerHTML = "<tr><th>Cổng</th><th>Chức năng</th><th></th><th>Tốc độ</th></tr>" + rows.join("");
  bindSelects(host);
  const fns = PORTS.filter(portShown).map(([p]) => val(p + "_fn")).filter((f) => f && f !== 3 && f !== 4);
  const dup = fns.filter((f, i) => fns.indexOf(f) !== i);
  $("portNote").textContent = (dup.length ? `Cảnh báo: "${PORT_FN[dup[0]][1]}" đang gán cho hai cổng. ` : "") +
    "Đổi chức năng cổng cần Lưu rồi khởi động lại FC. Cổng USB luôn dành cho configurator, CLI, MSP và MAVLink.";
}

// ---------------------------------------------------------------- VTX
const VTX_BANDS = ["A", "B", "E", "F (Fatshark)", "R (Raceband)", "L (Lowband)", "U (bảng VTX riêng)"];
const VTX_FREQ = [[5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725], [5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866],
  [5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945], [5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880],
  [5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917], [5362, 5399, 5436, 5473, 5510, 5547, 5584, 5621]];
const VTX_MW = [25, 100, 200, 400, 600];
const vtxtOn = () => has("vtxt_powers") && val("vtxt_powers") > 0;
function vtxPowers() {  // [level, label] from the custom table or the built-in one
  if (vtxtOn()) return Array.from({ length: val("vtxt_powers") }, (_, i) => [i + 1, val("vtxt_mw" + (i + 1)) + " mW"]);
  return VTX_MW.map((m, i) => [i + 1, m + " mW"]);
}
function vtxFreq() {
  const f = val("vtx_freq");
  if (f >= 5000) return f;
  if (val("vtx_band") === 7 && has("vtxt_band1")) { const u = val("vtxt_band" + (val("vtx_channel") || 1)); if (u >= 5000) return u; }
  return (VTX_FREQ[val("vtx_band") - 1] || VTX_FREQ[4])[(val("vtx_channel") || 1) - 1];
}
function buildVtx() {
  const host = $("vtxPanel");
  if (!host || !has("vtx_band")) return;
  host.innerHTML = `<div class="row" style="justify-content:space-between;align-items:flex-end">
      <div><div class="freq" id="vtxFreq">—</div><div class="muted" id="vtxSrc"></div></div>
      <label class="check"><input type="checkbox" data-p="vtx_pit"> Pit mode (công suất tối thiểu, khi disarm)</label></div>
    <div class="fields" style="margin-top:12px">
      <div class="field"><label>Băng tần</label>${selectHtml("vtx_band", VTX_BANDS.map((b, i) => [i + 1, b]))}</div>
      <div class="field"><label>Kênh</label>${selectHtml("vtx_channel", [1, 2, 3, 4, 5, 6, 7, 8].map((c) => [c, String(c)]))}</div>
      <div class="field"><label>Công suất</label>${selectHtml("vtx_power", vtxPowers())}</div>
      <div class="field"><label>Tần số riêng (MHz, 0 = theo kênh)</label><input data-p="vtx_freq" inputmode="numeric"></div>
    </div>
    <label class="check"><input type="checkbox" data-p="vtx_low_disarm"> Công suất thấp nhất khi chưa arm</label>
    ${has("vtxt_powers") ? `<details class="vtxt" id="vtxTable"${vtxtOn() ? " open" : ""}><summary>Bảng VTX riêng (công suất, băng U)</summary>
      <p class="muted">Như vtxtable của Betaflight: cho VTX có mức công suất khác 25/100/200/400/600 mW, hoặc tần số riêng. SmartAudio 2.1 nhận dBm; VTX SmartAudio cũ nhận số thứ tự mức (bật ô dưới).</p>
      <div class="fields"><div class="field"><label>Số mức công suất (0 = bảng có sẵn)</label>${selectHtml("vtxt_powers", [0, 1, 2, 3, 4, 5, 6, 7, 8].map((n) => [n, n ? String(n) : "bảng có sẵn"]))}</div></div>
      <label class="check"><input type="checkbox" data-p="vtxt_sa_index"> SmartAudio cũ: gửi số thứ tự mức thay vì dBm</label>
      <table class="ports vtxt-t"><tr><th>Mức</th><th>mW</th><th>Giá trị gửi (dBm / chỉ số)</th></tr>
      ${[1, 2, 3, 4, 5, 6, 7, 8].map((i) => `<tr data-lv="${i}"><td>${i}</td><td><input data-p="vtxt_mw${i}" inputmode="numeric"></td><td><input data-p="vtxt_val${i}" inputmode="numeric"></td></tr>`).join("")}</table>
      <p class="muted" style="margin:10px 0 4px">Băng U: tần số từng kênh (MHz)</p>
      <div class="fields">${[1, 2, 3, 4, 5, 6, 7, 8].map((i) => `<div class="field"><label>U${i}</label><input data-p="vtxt_band${i}" inputmode="numeric"></div>`).join("")}</div>
    </details>` : ""}`;
  bindSelects(host);
  updateVtx();
}
function updateVtx() {
  if (!$("vtxFreq") || !has("vtx_band")) return;
  document.querySelectorAll("#vtxTable tr[data-lv]").forEach((r) => { r.hidden = +r.dataset.lv > val("vtxt_powers"); });
  $("vtxFreq").textContent = vtxFreq() + " MHz";
  const fns = PORTS.map(([p]) => val(p + "_fn"));
  const sa = fns.includes(9), tr = fns.includes(10), dp = fns.includes(5);
  $("vtxSrc").textContent = sa ? "Điều khiển qua SmartAudio" : tr ? "Điều khiển qua Tramp" : dp ? "Kính số: đọc/đổi qua MSP (cổng OSD kính số)"
    : "Chưa có cổng điều khiển VTX: gán VTX SmartAudio hoặc Tramp cho một cổng ở trên.";
}

// ---------------------------------------------------------------- LED strip
function ledColorsJs(st, n) {
  const now = Date.now(), fast = Math.floor(now / 100) % 2, slow = Math.floor(now / 400) % 2;
  const mode = val("led_mode"), bri = (val("led_brightness") || 40) / 100;
  const modeCol = { ACRO: 0xFFFFFF, ANGLE: 0x00FF00, HORIZON: 0x00FFFF, ALTHOLD: 0x0040FF, POSHOLD: 0x8000FF, CRUISE: 0x8000FF,
    RTH: 0xFFC000, MISSION: 0xFFC000, LAND: 0xFFC000, TURTLE: 0xFF6000, AUTOTUNE: 0xFF00FF };
  let whole = 0, over = true, level = 1;
  const t = (now / 20) % 100, br = 0.25 + 0.75 * (t < 50 ? t : 100 - t) / 50;
  if (st && st.failsafe) whole = fast ? 0xFF0000 : 0;
  else over = false;
  if (!over) {
    if (mode === 2) whole = val("led_color");
    else if (mode === 1) whole = -1;
    else if (st && st.armed) whole = modeCol[st.mode] ?? 0xFF0000;
    else { whole = st && st.blockers && st.blockers.length ? 0xFFA000 : 0x00FF00; level = br; }
  }
  const out = [];
  for (let i = 0; i < n; i++) {
    const c = whole === -1 ? (i < (n + 1) / 2 ? 0xFFFFFF : 0xFF0000) : whole;
    const k = level * bri, r = ((c >> 16) & 255) * k, g = ((c >> 8) & 255) * k, b = (c & 255) * k;
    out.push(`rgb(${r | 0},${g | 0},${b | 0})`);
  }
  return out;
}
function buildLed() {
  const host = $("ledPanel");
  if (!host || !has("led_pin")) return;
  host.innerHTML = `<div class="leds" id="ledPreview" aria-label="Xem trước dải LED"></div>
    <div class="fields">
      <div class="field"><label>Chân GPIO (0 = tắt)</label><input data-p="led_pin" inputmode="numeric"></div>
      <div class="field"><label>Số LED</label><input data-p="led_count" inputmode="numeric"></div>
      <div class="field"><label>Hiển thị</label>${selectHtml("led_mode", [[0, "Trạng thái"], [1, "Hướng (trước trắng, sau đỏ)"], [2, "Một màu"]])}</div>
      <div class="field"><label>Màu (chế độ một màu)</label><input type="color" data-p="led_color" style="min-height:40px;width:100%"></div>
      <div class="field"><label>Độ sáng</label><input type="range" min="1" max="100" data-p="led_brightness" style="width:100%"></div>
    </div>
    <p class="muted" style="margin-bottom:0">WS2812 / SK6812. ESP32: chân GPIO bất kỳ (ESP32-S3 DevKit có sẵn một LED RGB ở GPIO 48 hoặc 38); ESP32-S3 dùng DShot, hoặc bidirectional DShot trên mọi ESP32, thì hết kênh RMT cho LED. STM32: dải LED luôn ở chân <span class="num">PB8</span> (TIM4 + DMA, không tốn CPU), đặt chân bất kỳ khác 0 để bật. Cần khởi động lại sau khi đổi.</p>`;
  bindSelects(host);
  updateLed();
}
function updateLed() {
  const p = $("ledPreview");
  if (!p || !has("led_pin")) return;
  const n = Math.min(32, val("led_count") || 8);
  p.innerHTML = ledColorsJs(S.status, n).map((c) => `<i style="background:${c}"></i>`).join("");
  clearTimeout(updateLed.t);
  if (shown("ports")) updateLed.t = setTimeout(updateLed, 120);
}

// ---------------------------------------------------------------- OSD layout
const OSD_EL = [["osd_vbat_pos", "Điện áp pin", "16.4V"], ["osd_cur_pos", "Dòng", "12.3A"], ["osd_mah_pos", "mAh đã dùng", " 850MAH"],
  ["osd_mode_pos", "Chế độ bay", "ANGLE"], ["osd_time_pos", "Thời gian bay", "02:15"], ["osd_alt_pos", "Độ cao", "A 12.3"],
  ["osd_vario_pos", "Tốc độ leo", "V+1.2"], ["osd_hdist_pos", "Khoảng cách về nhà", "H 120M"], ["osd_hdir_pos", "Mũi tên về nhà", "^"],
  ["osd_sats_pos", "Số vệ tinh", "S12"], ["osd_speed_pos", "Tốc độ", " 35KMH"], ["osd_link_pos", "Chất lượng sóng", "LQ100"],
  ["osd_cross_pos", "Tâm ngắm", "-+-"], ["osd_horizon_pos", "Đường chân trời", "---------- ----------"],
  ["osd_warn_pos", "Cảnh báo", "LOW BATTERY"], ["osd_coord_pos", "Toạ độ", "21.02850 105.85420"],
  ["osd_esct_pos", "Nhiệt độ ESC", "T 45C"], ["osd_dist_pos", "Tổng quãng đường", "D1.25KM"], ["osd_eff_pos", "mAh/km", "120MAH/KM"],
  ["osd_tleft_pos", "Thời gian bay còn lại", "L05:30"], ["osd_batbar_pos", "Thanh pin", "[######--]"],
  ["osd_cell_pos", "Điện áp mỗi cell", "3.85V/C"], ["osd_rssi_pos", "RSSI (dBm)", " -72DBM"]];
const OSD = { drag: null, hd: false };
const osdRows = () => (val("osd_pal") ? 16 : 13);
// canvas the FC will draw on (same rule as osd.cpp): [cols, rows]
function osdCanvasSize() {
  const c = has("osd_canvas") ? val("osd_canvas") : 0, sd = [30, osdRows()];
  if (c === 1) return sd;
  if (c === 2) return [50, 18];
  if (c === 3) return [53, 20];
  if (c === 4) return [60, 22];
  return val("osd_backend") === 2 ? [53, 20] : sd;
}
// layout cell -> canvas cell, keeping left / centre / right and top / bottom anchors (osd.cpp Renderer::put)
function osdMap(row, col, len, cols, rows) {
  const sdRows = osdRows();
  if (cols !== 30) {
    const right = 30 - (col + len);
    if (col <= right && col < 10) { /* left */ } else if (right < col && right < 10) col = cols - len - right;
    else col = Math.floor(((col + len * 0.5) * cols) / 30 - len * 0.5 + 0.5);
  }
  if (rows !== sdRows) {
    const bottom = sdRows - 1 - row;
    if (row > bottom) row = rows - 1 - bottom;
    else if (row > 2) row = Math.floor((row * (rows - 1) + Math.floor((sdRows - 1) / 2)) / (sdRows - 1));
  }
  if (col + len > cols) col = cols - len;
  return [row, Math.max(0, col)];
}
function buildOsd() {
  if (!$("osdList") || !has("osd_vbat_pos")) return;
  $("osdTop").innerHTML = `<div class="field"><label>OSD</label>${selectHtml("osd_backend", [[0, "Tắt"], [1, "Analog (chip MAX7456)"], [2, "Kính số (MSP DisplayPort)"]])}</div>
    <div class="field"><label>Hệ màu (analog)</label>${selectHtml("osd_pal", [[1, "PAL (16 dòng)"], [0, "NTSC (13 dòng)"]])}</div>
    ${has("osd_canvas") ? `<div class="field"><label>Lưới chữ (canvas)</label>${selectHtml("osd_canvas", [[0, "Tự động"], [1, "SD 30×16"], [2, "HD 50×18 (HDZero)"], [3, "HD 53×20 (DJI O3 / O4, Walksnail)"], [4, "HD 60×22"]])}</div>
    <div class="field"><label>Ký hiệu</label>${selectHtml("osd_font", [[0, "Icon Betaflight"], [1, "Chỉ chữ"]])}</div>` : ""}
    ${has("osd_stats") ? `<div class="field"><label>Thống kê sau khi disarm (giây, 0 = tắt)</label><input data-p="osd_stats" inputmode="numeric"></div>` : ""}
    ${has("osd_blink") ? `<div class="field"><label>Nhấp nháy</label>${selectHtml("osd_blink", [[1, "Cảnh báo + pin yếu nhấp nháy"], [0, "Không nhấp nháy"]])}</div>` : ""}`;
  bindSelects($("osdTop"));
  const view = $("osdView");
  if (view) {
    view.hidden = !has("osd_canvas");
    view.querySelectorAll("button").forEach((b) => (b.onclick = () => {
      OSD.hd = b.dataset.v === "hd";
      view.querySelectorAll("button").forEach((x) => x.setAttribute("aria-pressed", String(x === b)));
      drawOsd();
    }));
  }
  $("osdList").innerHTML = OSD_EL.filter(([n]) => has(n)).map(([n, label]) => `<label><input type="checkbox" data-osd="${n}"> ${label}</label>`).join("");
  $("osdList").querySelectorAll("input").forEach((el) => {
    const n = el.dataset.osd;
    el.checked = !!(val(n) & 0x8000);
    el.onchange = () => setEdit(n, (val(n) & 0x7FFF) | (el.checked ? 0x8000 : 0));
  });
  drawOsd();
}
function osdGeom() {
  const cv = $("osdCanvas");
  const [cols, rows] = OSD.hd ? osdCanvasSize() : [30, osdRows()];
  const w = cv.getBoundingClientRect().width || 600, cw = w / cols, chh = OSD.hd ? (w * 9) / 16 / rows : cw * 1.45;
  return { cv, w, cw, chh, rows, cols };
}
function drawOsd() {
  if (!$("osdCanvas") || !shown("osd") || !has("osd_vbat_pos")) return;
  const { cv, cw, chh, rows, cols } = osdGeom();
  const lbl = $("osdViewInfo");
  if (lbl) lbl.textContent = OSD.hd ? `Xem trước trên lưới ${cols}×${rows} của kính (chỉ xem, kéo ở chế độ Bố cục)` : "Bố cục 30×" + rows + ": kéo để đổi chỗ";
  cv.style.height = rows * chh + "px";
  const { g, w, h } = setupCanvas(cv);
  const sky = g.createLinearGradient(0, 0, 0, h);
  sky.addColorStop(0, "#4A6F93"); sky.addColorStop(0.55, "#8DA9B8"); sky.addColorStop(0.56, "#5C6B45"); sky.addColorStop(1, "#3A4530");
  g.fillStyle = sky; g.fillRect(0, 0, w, h);
  g.strokeStyle = "rgba(255,255,255,.07)"; g.lineWidth = 1;
  for (let c = 1; c < cols; c++) { g.beginPath(); g.moveTo(c * cw, 0); g.lineTo(c * cw, h); g.stroke(); }
  for (let r = 1; r < rows; r++) { g.beginPath(); g.moveTo(0, r * chh); g.lineTo(w, r * chh); g.stroke(); }
  g.font = `700 ${Math.round(chh * 0.78)}px ui-monospace, Menlo, Consolas, monospace`;
  g.textBaseline = "middle";
  for (const [n, , sample] of OSD_EL) {
    const v = val(n);
    if (!(v & 0x8000)) continue;
    let row = Math.floor((v & 0x7FFF) / 32), col = (v & 0x7FFF) % 32;
    if (row >= osdRows()) continue;
    if (OSD.hd) [row, col] = osdMap(row, col, sample.length, cols, rows);
    const text = sample.slice(0, Math.max(0, cols - col));
    const sel = OSD.drag && OSD.drag.n === n;
    if (sel) { g.fillStyle = "rgba(224,155,18,.35)"; g.fillRect(col * cw, row * chh, text.length * cw, chh); }
    for (let i = 0; i < text.length; i++) {
      const x = (col + i + 0.5) * cw, y = (row + 0.5) * chh;
      g.textAlign = "center";
      g.lineWidth = 3; g.strokeStyle = "#000"; g.strokeText(text[i], x, y);
      g.fillStyle = "#fff"; g.fillText(text[i], x, y);
    }
  }
}
function osdHit(px, py) {
  const { cw, chh, rows } = osdGeom();
  const col = Math.floor(px / cw), row = Math.floor(py / chh);
  for (const [n, , sample] of OSD_EL.slice().reverse()) {
    const v = val(n);
    if (!(v & 0x8000)) continue;
    const r = Math.floor((v & 0x7FFF) / 32), c = (v & 0x7FFF) % 32;
    if (r === row && col >= c && col < c + Math.max(1, Math.min(sample.length, 30 - c)) && r < rows) return { n, dc: col - c };
  }
  return null;
}
$("osdCanvas").addEventListener("pointerdown", (ev) => {
  if (OSD.hd) return;  // preview only
  const r = ev.currentTarget.getBoundingClientRect();
  const hit = osdHit(ev.clientX - r.left, ev.clientY - r.top);
  if (!hit) return;
  OSD.drag = hit;
  ev.currentTarget.setPointerCapture(ev.pointerId);
  drawOsd();
});
$("osdCanvas").addEventListener("pointermove", (ev) => {
  if (!OSD.drag) return;
  const r = ev.currentTarget.getBoundingClientRect(), { cw, chh, rows } = osdGeom();
  const col = Math.max(0, Math.min(29, Math.floor((ev.clientX - r.left) / cw) - OSD.drag.dc));
  const row = Math.max(0, Math.min(rows - 1, Math.floor((ev.clientY - r.top) / chh)));
  const nv = 0x8000 | (row * 32 + col);
  if (nv !== val(OSD.drag.n)) setEdit(OSD.drag.n, nv);
});
const osdUp = () => { if (OSD.drag) { OSD.drag = null; drawOsd(); } };
$("osdCanvas").addEventListener("pointerup", osdUp);
$("osdCanvas").addEventListener("pointercancel", osdUp);
window.addEventListener("resize", () => { drawOsd(); drawMotorMap(); });

// ---------------------------------------------------------------- CLI
const CLI = { hist: [], i: -1, busy: false };
function cliPrint(t) {
  const o = $("cliOut");
  o.textContent += t;
  if (o.textContent.length > 200000) o.textContent = o.textContent.slice(-150000);
  o.scrollTop = o.scrollHeight;
}
async function cliRun(line) {
  line = line.trim();
  if (!line || CLI.busy) return;
  if (CLI.hist[CLI.hist.length - 1] !== line) CLI.hist.push(line);
  CLI.i = CLI.hist.length;
  cliPrint("# " + line + "\n");
  CLI.busy = true;
  try {
    const out = await api("/api/cli", { method: "POST", body: line, text: true, timeout: 8000 });
    cliPrint(out.replace(/\r/g, "") || "(không có kết quả)\n");
    if (/^(set|save|defaults|mission|rally)\b/.test(line)) loadParams();
  } catch (e) {
    cliPrint("lỗi: " + e.message + "\n");
  }
  CLI.busy = false;
}
$("cliForm").onsubmit = (ev) => { ev.preventDefault(); const v = $("cliIn").value; $("cliIn").value = ""; cliRun(v); };
$("cliIn").onkeydown = (ev) => {
  // Tab: complete a parameter name after set / get
  if (ev.key === "Tab") {
    const v = $("cliIn").value, m = v.match(/^(set|get)\s+(\S*)$/);
    if (m && S.order) {
      ev.preventDefault();
      const opts = S.order.filter((n) => n.startsWith(m[2]));
      if (opts.length === 1) $("cliIn").value = `${m[1]} ${opts[0]}${m[1] === "set" ? " " : ""}`;
      else if (opts.length > 1) {
        let pre = opts[0];
        for (const o of opts) while (!o.startsWith(pre)) pre = pre.slice(0, -1);
        $("cliIn").value = `${m[1]} ${pre}`;
        cliPrint(opts.slice(0, 40).join("  ") + (opts.length > 40 ? ` … (${opts.length})` : "") + "\n");
      }
      return;
    }
  }
  if (ev.key === "ArrowUp" && CLI.hist.length) { CLI.i = Math.max(0, CLI.i - 1); $("cliIn").value = CLI.hist[CLI.i]; ev.preventDefault(); }
  if (ev.key === "ArrowDown" && CLI.hist.length) { CLI.i = Math.min(CLI.hist.length, CLI.i + 1); $("cliIn").value = CLI.hist[CLI.i] || ""; ev.preventDefault(); }
};
$("cliQuick").querySelectorAll("[data-c]").forEach((b) => (b.onclick = () => cliRun(b.dataset.c)));
$("cliClear").onclick = () => { $("cliOut").textContent = ""; };

// ---------------------------------------------------------------- mission editor
const WP_TYPES = [[16, "Bay tới điểm"], [19, "Bay tới và chờ (giây)"], [18, "Bay vòng quanh"], [17, "Bay tới và giữ mãi"], [21, "Hạ cánh"], [20, "Về nhà"], [22, "Cất cánh"]];
const WP_HAS_POS = (cmd) => [16, 17, 18, 19, 21].includes(cmd);
const EARTH_R = 6371000;
function wpRef() {
  const m = S.mission;
  if (m && m.home) return { lat: m.lat, lon: m.lon, src: "home" };
  const st = S.status;
  if (st && st.gps && st.gps.fix) return { lat: st.gps.lat, lon: st.gps.lon, src: "gps" };
  return null;
}
function wpNE(it, ref) {
  const lat = it.la * 1e-7, lon = it.lo * 1e-7;
  return [(lat - ref.lat) * Math.PI / 180 * EARTH_R, (lon - ref.lon) * Math.PI / 180 * EARTH_R * Math.cos(ref.lat * Math.PI / 180)];
}
function wpLL(n, e, ref) {
  return [Math.round((ref.lat + n / EARTH_R * 180 / Math.PI) * 1e7), Math.round((ref.lon + e / (EARTH_R * Math.cos(ref.lat * Math.PI / 180)) * 180 / Math.PI) * 1e7)];
}
function wpSync() {  // edit model -> what the map draws
  const ref = S.wp.ref;
  S.wp.items.forEach((it) => {
    it.pos = WP_HAS_POS(it.cmd) && (it.la || it.lo);
    if (it.pos) [it.n, it.e] = wpNE(it, ref);
  });
  S.mission = { ...(S.mission || {}), items: S.wp.items, rally: S.wp.rally, fence: S.wp.fence };
  drawNav();
  wpList();
  wpLayerUi();
}
// ---- editor layers: waypoints, survey area (-> waypoints), polygon geofence
const FENCE_MAX = 16, MISSION_MAX = 120;
function wpLayerUi() {
  if (!S.wp) return;
  const L = S.wp.layer;
  $("wpLayer").querySelectorAll("button").forEach((b) => b.setAttribute("aria-pressed", String(b.dataset.l === L)));
  $("wpList").hidden = L !== "wp";
  $("svPanel").hidden = L !== "survey";
  $("fcPanel").hidden = L !== "fence";
  $("wpHint").textContent = L === "wp" ? "Chạm bản đồ để thêm điểm, kéo một điểm để di chuyển, chạm điểm để sửa bên dưới."
    : L === "survey" ? "Chạm để vẽ các góc của vùng cần quét (kéo để chỉnh), rồi chọn khoảng cách đường và bấm Tạo đường quét."
    : `Chạm để vẽ các đỉnh hàng rào (tối đa ${FENCE_MAX}, kéo để chỉnh). Điểm nhà phải nằm trong vùng.`;
  if (L === "survey") svPreview();
  if (L === "fence") {
    const n = S.wp.poly.length, inside = n >= 3 && pointInPoly([0, 0], S.wp.poly.map((p) => wpNE(p, S.wp.ref)));
    $("fcInfo").textContent = `${n} / ${FENCE_MAX} đỉnh.` + (n && n < 3 ? " Cần ít nhất 3 đỉnh." : "") + (n >= 3 && !inside ? " Cảnh báo: điểm nhà nằm ngoài vùng!" : "");
    $("fcOn").checked = S.wp.polyOn;
  }
}
function pointInPoly([n, e], P) {
  let inside = false;
  for (let i = 0, j = P.length - 1; i < P.length; j = i++) {
    const [ni, ei] = P[i], [nj, ej] = P[j];
    if ((ni > n) !== (nj > n) && e < ((ej - ei) * (n - ni)) / (nj - ni) + ei) inside = !inside;
  }
  return inside;
}
// lawnmower lanes across a polygon: lanes run at angleDeg from north, spaced `space` metres apart
function surveyPath(P, space, angleDeg) {
  if (P.length < 3 || !(space > 0.5)) return [];
  const a = (angleDeg * Math.PI) / 180, s = Math.sin(a), c = Math.cos(a);
  const UV = P.map(([n, e]) => [e * s + n * c, e * c - n * s]);  // u along the lane, v across
  const vs = UV.map((p) => p[1]), v0 = Math.min(...vs), v1 = Math.max(...vs);
  const out = [];
  let dir = 0;
  for (let v = v0 + space / 2; v < v1; v += space) {
    const us = [];
    for (let i = 0, j = UV.length - 1; i < UV.length; j = i++) {
      const [ui, vi] = UV[i], [uj, vj] = UV[j];
      if ((vi <= v) !== (vj <= v)) us.push(ui + ((v - vi) * (uj - ui)) / (vj - vi));
    }
    if (us.length < 2) continue;
    let lo = Math.min(...us), hi = Math.max(...us);
    if (dir++ % 2) [lo, hi] = [hi, lo];
    for (const u of [lo, hi]) out.push([u * c - v * s, u * s + v * c]);  // back to (n, e)
    if (out.length > 400) break;
  }
  return out;
}
function svPreview() {
  const P = S.wp.area.map((p) => wpNE(p, S.wp.ref));
  const path = surveyPath(P, +$("svSpace").value, +$("svAngle").value);
  S.svPath = path;
  let len = 0;
  for (let i = 1; i < path.length; i++) len += Math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1]);
  const sp = has("wp_speed") ? val("wp_speed") : 5, room = MISSION_MAX - (+$("svRtl").value ? 1 : 0);
  $("svInfo").textContent = P.length < 3 ? `Vùng: ${P.length} góc (cần ít nhất 3).`
    : `${path.length / 2} đường, ${path.length} waypoint, ${(len / 1000).toFixed(2)} km, khoảng ${Math.ceil(len / Math.max(sp, 1) / 60)} phút ở ${sp} m/s.` +
      (path.length > room ? ` Quá ${room} điểm: tăng khoảng cách.` : "");
  drawNav();
}
["svSpace", "svAngle", "svAlt", "svRtl"].forEach((id) => ($(id).onchange = $(id).oninput = () => S.wp && svPreview()));
$("svMake").onclick = () => {
  const path = S.svPath || [], rtl = +$("svRtl").value, alt = +$("svAlt").value || 30;
  if (path.length < 2) return toast("Vẽ vùng quét (ít nhất 3 góc) trước.");
  if (path.length + rtl > MISSION_MAX) return toast(`Quá ${MISSION_MAX} waypoint: tăng khoảng cách giữa các đường.`);
  if (S.wp.items.length && !confirm(`Thay ${S.wp.items.length} điểm hiện có bằng ${path.length} điểm của đường quét?`)) return;
  S.wp.items = path.map(([n, e]) => { const [la, lo] = wpLL(n, e, S.wp.ref); return { cmd: 16, la, lo, alt, p1: 0, p2: 0, p3: 0, p4: 0 }; });
  if (rtl) S.wp.items.push({ cmd: 20, la: 0, lo: 0, alt: 0, p1: 0, p2: 0, p3: 0, p4: 0 });
  S.wp.layer = "wp";
  wpSync();
  toast(`Đã tạo ${path.length} waypoint. Kiểm tra rồi bấm Gửi lên drone.`);
};
$("svUndo").onclick = () => { S.wp.area.pop(); wpSync(); };
$("svClear").onclick = () => { S.wp.area = []; wpSync(); };
$("fcUndo").onclick = () => { S.wp.poly.pop(); S.wp.polyDirty = true; wpSync(); };
$("fcClear").onclick = () => { S.wp.poly = []; S.wp.polyDirty = true; wpSync(); };
$("fcOn").onchange = () => { S.wp.polyOn = $("fcOn").checked; S.wp.polyDirty = true; };
$("wpLayer").querySelectorAll("button").forEach((b) => (b.onclick = () => { S.wp.layer = b.dataset.l; S.wpSel = -1; wpSync(); }));
async function wpStart() {
  if (S.armed) return toast("Disarm trước khi soạn nhiệm vụ.");
  if (!S.mission) await loadMission();
  const ref = wpRef();
  if (!ref) return toast("Cần GPS có fix (hoặc điểm nhà) để đặt điểm trên bản đồ.");
  const m = S.mission || { items: [], rally: [], fence: 0 };
  S.wp = { ref, items: m.items.map((i) => ({ ...i, p2: i.p2 || 0, p4: i.p4 || 0 })), rally: m.rally || [], fence: m.fence || 0, orig: m,
    layer: "wp", area: [], poly: (m.poly || []).map((p) => ({ la: p[2], lo: p[3] })), polyOn: !!m.polyOn, polyDirty: false };
  S.gotoMode = false;
  $("gotoBtn").setAttribute("aria-pressed", "false");
  $("navMap").classList.remove("goto");
  S.wpSel = -1;
  const T = S.navT || { cn: 0, ce: 0, span: 60 };
  S.navFreeze = { cn: T.cn, ce: T.ce, span: Math.max(T.span || 60, 60) };
  $("wpPanel").hidden = false;
  $("wpLayer").hidden = false;
  $("wpEditBtn").hidden = true;
  $("wpHint").hidden = false;
  $("wpHint").textContent = "Chạm bản đồ để thêm điểm, kéo một điểm để di chuyển, chạm điểm để sửa bên dưới." +
    (ref.src === "gps" ? " Chưa có điểm nhà: đang lấy vị trí GPS hiện tại làm gốc." : "");
  wpSync();
}
function wpStop(reload) {
  S.wp = null; S.wpSel = -1; S.navFreeze = null;
  $("wpPanel").hidden = true; $("wpEditBtn").hidden = false; $("wpHint").hidden = true; $("wpLayer").hidden = true;
  if (reload) loadMission(); else drawNav();
}
function wpList() {
  if (!S.wp) return;
  $("wpCount").textContent = `${S.wp.items.length} mục`;
  const host = $("wpList");
  host.innerHTML = "";
  S.wp.items.forEach((it, i) => {
    const d = document.createElement("div");
    d.className = "wp" + (i === S.wpSel ? " sel" : "");
    const known = WP_TYPES.some(([c]) => c === it.cmd);
    const typeSel = known ? `<select data-k="cmd">${WP_TYPES.map(([c, t]) => `<option value="${c}"${c === it.cmd ? " selected" : ""}>${t}</option>`).join("")}</select>`
      : `<span class="muted">Lệnh ${it.cmd} (giữ nguyên)</span>`;
    const p1 = it.cmd === 19 ? ["p1", "Chờ (s)"] : it.cmd === 18 ? ["p1", "Số vòng"] : it.cmd === 16 ? ["p1", "Dừng (s)"] : null;
    d.innerHTML = `<div class="n">${i + 1}</div><div>
      <div class="fields">
        <div class="field"><label>Loại</label>${typeSel}</div>
        ${it.cmd !== 20 ? `<div class="field"><label>Độ cao (m)</label><input data-k="alt" inputmode="decimal" value="${it.alt}"></div>` : ""}
        ${p1 ? `<div class="field"><label>${p1[1]}</label><input data-k="p1" inputmode="decimal" value="${it.p1}"></div>` : ""}
        ${it.cmd === 18 ? `<div class="field"><label>Bán kính (m, âm = ngược chiều)</label><input data-k="p3" inputmode="decimal" value="${it.p3}"></div>` : ""}
      </div>
      <div class="row" style="margin-top:6px">${it.pos ? `<span class="muted num">N ${it.n.toFixed(1)} m, E ${it.e.toFixed(1)} m</span>` : ""}
        <button class="btn ghost" data-a="up" aria-label="Lên">↑</button><button class="btn ghost" data-a="down" aria-label="Xuống">↓</button><button class="btn ghost" data-a="del">Xoá</button></div></div>`;
    d.querySelectorAll("[data-k]").forEach((el) => (el.onchange = () => {
      const k = el.dataset.k;
      it[k] = +el.value;
      if (k === "cmd" && WP_HAS_POS(it.cmd) && !(it.la || it.lo)) [it.la, it.lo] = wpLL(0, 0, S.wp.ref);
      wpSync();
    }));
    d.querySelectorAll("[data-a]").forEach((b) => (b.onclick = () => {
      const a = b.dataset.a, L = S.wp.items;
      if (a === "del") L.splice(i, 1);
      if (a === "up" && i > 0) [L[i - 1], L[i]] = [L[i], L[i - 1]];
      if (a === "down" && i < L.length - 1) [L[i + 1], L[i]] = [L[i], L[i + 1]];
      S.wpSel = -1;
      wpSync();
    }));
    d.onclick = (ev) => { if (ev.target.closest("input,select,button")) return; S.wpSel = i; drawNav(); wpList(); };
    host.appendChild(d);
  });
  if (!S.wp.items.length) host.innerHTML = '<p class="muted">Chưa có điểm nào: chạm lên bản đồ để thêm.</p>';
}
function mapToNE(ev) {
  const T = S.navT, r = $("navMap").getBoundingClientRect();
  const x = ev.clientX - r.left, y = ev.clientY - r.top;
  return [T.cn - (y - T.h / 2) / T.k, T.ce + (x - T.w / 2) / T.k, x, y];
}
$("navMap").addEventListener("pointerdown", (ev) => {
  if (S.gotoMode && !S.wp && S.navT) return gotoTap(ev);
  if (!S.wp || !S.navT) return;
  const [n, e, x, y] = mapToNE(ev), T = S.navT;
  if (S.wp.layer !== "wp") {  // polygon layers: drag a vertex or add one
    const L = S.wp.layer === "fence" ? S.wp.poly : S.wp.area;
    const vi = L.findIndex((p) => { const [pn, pe] = wpNE(p, S.wp.ref); return Math.hypot(T.w / 2 + (pe - T.ce) * T.k - x, T.h / 2 - (pn - T.cn) * T.k - y) < 16; });
    if (vi >= 0) { S.vDrag = vi; ev.currentTarget.setPointerCapture(ev.pointerId); return; }
    if (S.wp.layer === "fence" && L.length >= FENCE_MAX) return toast(`Tối đa ${FENCE_MAX} đỉnh.`);
    const [la, lo] = wpLL(n, e, S.wp.ref);
    L.push({ la, lo });
    if (S.wp.layer === "fence") S.wp.polyDirty = true;
    return wpSync();
  }
  const hit = S.wp.items.findIndex((it) => it.pos && Math.hypot(T.w / 2 + (it.e - T.ce) * T.k - x, T.h / 2 - (it.n - T.cn) * T.k - y) < 16);
  if (hit >= 0) { S.wpSel = hit; S.wpDrag = hit; ev.currentTarget.setPointerCapture(ev.pointerId); drawNav(); wpList(); return; }
  const last = S.wp.items.filter((i) => i.pos).slice(-1)[0];
  const [la, lo] = wpLL(n, e, S.wp.ref);
  S.wp.items.push({ cmd: 16, la, lo, alt: last ? last.alt : 20, p1: 0, p2: 0, p3: 0, p4: 0 });
  S.wpSel = S.wp.items.length - 1;
  wpSync();
});
$("navMap").addEventListener("pointermove", (ev) => {
  if (S.wp && S.vDrag != null && S.vDrag >= 0) {
    const L = S.wp.layer === "fence" ? S.wp.poly : S.wp.area, [n, e] = mapToNE(ev);
    [L[S.vDrag].la, L[S.vDrag].lo] = wpLL(n, e, S.wp.ref);
    if (S.wp.layer === "fence") S.wp.polyDirty = true;
    if (S.wp.layer === "survey") svPreview(); else drawNav();
    return;
  }
  if (!S.wp || S.wpDrag == null || S.wpDrag < 0) return;
  const [n, e] = mapToNE(ev), it = S.wp.items[S.wpDrag];
  [it.la, it.lo] = wpLL(n, e, S.wp.ref);
  it.n = n; it.e = e;
  drawNav();
});
const wpUp = () => {
  if (S.vDrag != null && S.vDrag >= 0) { S.vDrag = -1; wpSync(); }
  if (S.wpDrag != null && S.wpDrag >= 0) { S.wpDrag = -1; wpSync(); }
};
$("navMap").addEventListener("pointerup", wpUp);
$("navMap").addEventListener("pointercancel", wpUp);
$("wpEditBtn").onclick = wpStart;
$("wpCancel").onclick = () => wpStop(true);
$("wpClear").onclick = () => { if (confirm("Xoá mọi điểm của nhiệm vụ?")) { S.wp.items = []; wpSync(); } };
$("wpRtl").onclick = () => { S.wp.items.push({ cmd: 20, la: 0, lo: 0, alt: 0, p1: 0, p2: 0, p3: 0, p4: 0 }); wpSync(); };
$("wpUpload").onclick = async () => {
  const L = S.wp.items, W = S.wp;
  const lines = ["mission clear"].concat(L.map((it) => `mission add ${it.cmd} ${it.frame || 3} ${it.pos ? it.la : 0} ${it.pos ? it.lo : 0} ${+it.alt || 0} ${+it.p1 || 0} ${+it.p2 || 0} ${+it.p3 || 0} ${+it.p4 || 0}`));
  if (W.polyDirty) {
    if (W.poly.length && W.poly.length < 3) return toast("Geofence đa giác cần ít nhất 3 đỉnh (hoặc xoá hết).");
    if (W.polyOn && W.poly.length >= 3 && !pointInPoly([0, 0], W.poly.map((p) => wpNE(p, W.ref))) &&
        !confirm("Điểm nhà nằm NGOÀI hàng rào: drone sẽ báo vượt rào ngay khi cất cánh. Vẫn gửi?")) return;
    lines.push("fence clear", ...W.poly.map((p) => `fence add ${p.la} ${p.lo}`));
    const on = W.polyOn && W.poly.length >= 3;
    lines.push(`set fence_poly ${on ? 1 : 0}`);
    if (on) lines.push("set fence_enable 1");
  }
  try {
    const j = await api("/api/restore", { method: "POST", upload: lines.join("\n") + "\n", timeout: 10000 });
    toast(`Đã gửi ${L.length} điểm${W.polyDirty ? ` và hàng rào ${W.poly.length} đỉnh` : ""}, đã lưu vào drone.`);
    if (W.polyDirty) loadParams();
    wpStop(true);
  } catch (e) {
    toast(e.code === 423 ? "Hãy disarm trước." : "Không gửi được: " + e.message);
  }
};

// ---- tap-to-fly (GUIDED): armed, with a position fix; every tap asks first
function gotoUi() {
  const st = S.status, ok = !!(st && st.armed && st.est && st.est.pos && S.mission && S.mission.home && !S.wp);
  $("gotoBtn").hidden = !ok;
  if (!ok && S.gotoMode) { S.gotoMode = false; $("navMap").classList.remove("goto"); $("gotoBtn").setAttribute("aria-pressed", "false"); }
  if (st && !st.armed) S.gotoT = null;
}
$("gotoBtn").onclick = () => {
  S.gotoMode = !S.gotoMode;
  $("gotoBtn").setAttribute("aria-pressed", String(S.gotoMode));
  $("navMap").classList.toggle("goto", S.gotoMode);
  if (S.gotoMode) toast("Chạm lên bản đồ chỗ muốn drone bay tới.");
};
async function gotoTap(ev) {
  const [n, e] = mapToNE(ev), m = S.mission, est = S.status.est;
  const dist = Math.hypot(n - est.n, e - est.e), alt = Math.max(5, Math.round(est.alt || 10));
  const ans = prompt(`Bay tới điểm cách drone ${dist.toFixed(0)} m (N ${n.toFixed(0)}, E ${e.toFixed(0)} so với nhà).\nDrone chuyển sang GUIDED. Gạt công tắc chế độ để lấy lại điều khiển.\n\nĐộ cao so với nhà (m):`, String(alt));
  if (ans === null) return;
  const a = +ans;
  if (!(a >= 2 && a <= 500)) return toast("Độ cao không hợp lệ.");
  const [la, lo] = wpLL(n, e, { lat: m.lat, lon: m.lon });
  try {
    const out = await api("/api/cli", { method: "POST", body: `goto ${la} ${lo} ${a}`, text: true, timeout: 5000 });
    if (!/ok/.test(out)) throw new Error(out.trim());
    S.gotoT = [n, e];
    toast("Đang bay tới điểm đã chọn (GUIDED).");
    drawNav();
  } catch (err) { toast("Không gửi được: " + err.message); }
}

// ---------------------------------------------------------------- presets
const PRESET_LIST = [
  ["Khung 5 inch freestyle", "PID mặc định cho khung 5 inch, motor 2207, pin 4–6S.", { roll_p: 45, roll_i: 80, roll_d: 30, roll_f: 60, pitch_p: 47, pitch_i: 84, pitch_d: 34, pitch_f: 60, yaw_p: 45, yaw_i: 80, yaw_d: 0, yaw_f: 60, tpa_rate: 0.4, gyro_lpf_hz: 150, dterm_lpf_hz: 90 }],
  ["Khung 3 inch / toothpick", "Khung nhỏ, motor quay nhanh: P và feedforward cao hơn, lọc mở rộng.", { roll_p: 52, roll_i: 85, roll_d: 28, roll_f: 70, pitch_p: 55, pitch_i: 90, pitch_d: 31, pitch_f: 70, yaw_p: 50, yaw_i: 85, yaw_d: 0, yaw_f: 70, gyro_lpf_hz: 200, dterm_lpf_hz: 110 }],
  ["Cinewhoop (có ống bảo vệ)", "Nặng đầu, ống cánh: D cao, feedforward thấp cho hình êm.", { roll_p: 60, roll_i: 90, roll_d: 38, roll_f: 50, pitch_p: 64, pitch_i: 95, pitch_d: 42, pitch_f: 50, yaw_p: 60, yaw_i: 90, yaw_d: 0, yaw_f: 40, tpa_rate: 0.5, dterm_lpf_hz: 80, anti_gravity: 5 }],
  ["Khung 7 inch bay xa", "Cánh lớn quay chậm: P thấp hơn, lọc và notch động thấp hơn.", { roll_p: 38, roll_i: 75, roll_d: 32, roll_f: 45, pitch_p: 40, pitch_i: 78, pitch_d: 35, pitch_f: 45, yaw_p: 40, yaw_i: 75, yaw_d: 0, yaw_f: 45, gyro_lpf_hz: 100, gyro_lpf2_hz: 200, dterm_lpf_hz: 70, dyn_notch_min: 70, dyn_notch_max: 350 }],
  ["Bộ lọc: khung sạch", "Motor mới, khung cứng, ít rung: lọc nhẹ, phản hồi nhanh hơn.", { gyro_lpf_hz: 200, gyro_lpf2_hz: 300, dterm_lpf_hz: 110, dyn_notch_count: 1 }],
  ["Bộ lọc: nhiều rung", "Khung mềm, cánh cong, motor cũ: lọc mạnh và hai notch động.", { gyro_lpf_hz: 100, gyro_lpf2_hz: 180, dterm_lpf_hz: 70, dyn_notch_count: 2, dyn_notch_q: 3 }],
  ["Rates: quay phim", "Êm ở giữa cần, xoay chậm.", { roll_rc_rate: 50, roll_max_rate: 400, roll_expo: 0.3, pitch_rc_rate: 50, pitch_max_rate: 400, pitch_expo: 0.3, yaw_rc_rate: 40, yaw_max_rate: 300, yaw_expo: 0.2 }],
  ["Rates: freestyle", "Mặc định: 670°/s tối đa.", { roll_rc_rate: 70, roll_max_rate: 670, roll_expo: 0, pitch_rc_rate: 70, pitch_max_rate: 670, pitch_expo: 0, yaw_rc_rate: 70, yaw_max_rate: 670, yaw_expo: 0 }],
  ["Rates: đua", "Nhạy ở giữa cần, 850°/s tối đa.", { roll_rc_rate: 90, roll_max_rate: 850, roll_expo: 0.1, pitch_rc_rate: 90, pitch_max_rate: 850, pitch_expo: 0.1, yaw_rc_rate: 80, yaw_max_rate: 650, yaw_expo: 0 }],
  ["An toàn GPS", "Mất sóng thì bay về; hàng rào 300 m, cao tối đa 120 m; cần GPS mới arm được.", { fs_action: 2, fence_enable: 1, fence_radius: 300, fence_alt_max: 120, fence_action: 2, rth_alt: 30, arm_require_gps: 1 }],
];
function buildPresets() {
  const host = $("presetList");
  if (!host || host.childElementCount) return;
  PRESET_LIST.forEach(([name, desc, set]) => {
    const d = document.createElement("div");
    d.className = "preset";
    d.innerHTML = `<h4>${name}</h4><p class="muted">${desc}</p><button class="btn">Dùng bộ này</button>`;
    d.querySelector("button").onclick = () => {
      const ch = Object.entries(set).filter(([n, v]) => has(n) && Math.abs(val(n) - v) > 1e-6);
      if (!ch.length) return toast("Đang dùng đúng bộ này rồi.");
      const list = ch.slice(0, 12).map(([n, v]) => `${n}: ${fmt(n, val(n))} → ${v}`).join("\n");
      if (!confirm(`${name}: ${ch.length} thay đổi\n\n${list}${ch.length > 12 ? "\n…" : ""}\n\nĐưa vào hàng chờ?`)) return;
      ch.forEach(([n, v]) => setEdit(n, v));
      toast(`${ch.length} thay đổi đang chờ: bấm Áp dụng, rồi Lưu vào flash.`);
    };
    host.appendChild(d);
  });
}

// ================================================================ firmware flasher
const FL = { target: "esp", file: null, data: null, busy: false, dfuDev: null };
const flLog = (t) => { const l = $("flLog"); l.hidden = false; l.textContent += t + "\n"; l.scrollTop = l.scrollHeight; };
const flPct = (p) => { $("flProg").hidden = false; $("flBar").value = p; $("flPct").textContent = Math.round(p) + "%"; };
function flHow() {
  $("flHow").textContent = FL.target === "esp"
    ? "File firmware.bin của PlatformIO (.pio/build/esp32dev, esp32s3, esp32c6 hoặc esp32p4) hoặc một file gộp đủ bootloader. Cắm USB; trang tự đưa ESP32 vào chế độ nạp. Nếu không được: giữ nút BOOT, bấm RESET, thả BOOT rồi thử lại."
    : "File firmware.bin của PlatformIO (.pio/build/blackpill_f411ce, stm32f405, stm32f722, stm32h743_weact hoặc stm32h743_devebox). Trang đưa STM32 vào DFU khi đang kết nối; board trống thì giữ BOOT0, bấm NRST. Windows cần driver WinUSB cho \"STM32 BOOTLOADER\" (Zadig) một lần.";
}
function flCheck() {
  const d = FL.data;
  $("flGo").disabled = true;
  if (!d) { $("flInfo").textContent = ""; return; }
  let info = `${FL.file.name}, ${(d.length / 1024).toFixed(0)} kB. `, ok = false;
  if (FL.target === "esp") {
    const merged = (d[0] === 0xE9 && d.length > 0x8002 && d[0x8000] === 0xAA && d[0x8001] === 0x50) || (d.length > 0x1001 && d[0] === 0xFF && d[0x1000] === 0xE9);
    if (merged) { info += "File gộp (bootloader + bảng phân vùng + firmware), ghi từ địa chỉ 0."; ok = true; }
    else if (d[0] === 0xE9) {
      const chip = d[12] | (d[13] << 8);
      info += `Firmware ứng dụng cho ${ESP_CHIPS[chip] || "chip " + chip}, ghi vào ô ota_0 (0x10000).`;
      ok = chip in ESP_CHIPS;
    } else info += "Không phải firmware ESP32 (byte đầu phải là 0xE9).";
  } else {
    const dv = new DataView(d.buffer, d.byteOffset, Math.min(8, d.length));
    const sp = d.length >= 8 ? dv.getUint32(0, true) : 0, rv = d.length >= 8 ? dv.getUint32(4, true) : 0;
    ok = ((sp & 0xFFF00000) === 0x20000000 || (sp & 0xFFF00000) === 0x24000000) && (rv & 0xFFF00000) === 0x08000000 && (rv & 1) === 1 && d.length <= 0x60000;
    info += ok ? "Firmware STM32 (bắt đầu ở 0x08000000)." : d.length > 0x60000 ? "File quá lớn (sẽ đè lên vùng lưu cấu hình)." : "Không phải firmware STM32 (.bin bắt đầu ở 0x08000000).";
  }
  $("flInfo").textContent = info;
  $("flGo").disabled = !ok;
}
// firmware published next to the page (GitHub Pages: tools/make_pages.py writes firmware/manifest.json and
// the .bin files): pick one, it is downloaded from the same site and checked (size + SHA-256)
const REL = { list: null };
async function relLoad() {
  if (REL.list !== null || !/^https?:$/.test(location.protocol) || !window.crypto || !crypto.subtle) return;
  REL.list = [];
  try {
    const r = await fetch(new URL("firmware/manifest.json", location.href), { cache: "no-store" });
    if (!r.ok) return;
    const m = await r.json();
    REL.version = m.version || "";
    REL.list = (m.files || []).filter((f) => f && f.file && (f.target === "esp" || f.target === "stm") && /^[\w.\-]+$/.test(f.file));
  } catch (e) { REL.list = []; }
  relRender();
}
function relRender() {
  const box = $("flRel");
  if (!box || !REL.list || !REL.list.length) { if (box) box.hidden = true; return; }
  const items = REL.list.filter((f) => f.target === FL.target);
  box.hidden = !items.length;
  const board = (S.lastHello && S.lastHello.board) || "";
  $("flRelSel").innerHTML = items.map((f, i) => `<option value="${REL.list.indexOf(f)}"${f.match && board && new RegExp(f.match, "i").test(board) ? " selected" : ""}>${esc(f.name)}${REL.version ? " · " + esc(REL.version) : ""}</option>`).join("");
}
async function relGet() {
  const f = REL.list[+$("flRelSel").value];
  if (!f || FL.busy) return;
  $("flInfo").textContent = `Đang tải ${f.file}…`;
  try {
    const r = await fetch(new URL("firmware/" + f.file, location.href), { cache: "no-store" });
    if (!r.ok) throw new Error("HTTP " + r.status);
    const d = new Uint8Array(await r.arrayBuffer());
    if (f.size && d.length !== f.size) throw new Error(`kích thước ${d.length} khác ${f.size}`);
    if (f.sha256) {
      const h = Array.from(new Uint8Array(await crypto.subtle.digest("SHA-256", d)), (b) => b.toString(16).padStart(2, "0")).join("");
      if (h !== f.sha256.toLowerCase()) throw new Error("SHA-256 không khớp (file hỏng hoặc bị đổi)");
    }
    FL.file = { name: f.file };
    FL.data = d;
    flCheck();
    if (f.note) $("flInfo").textContent += " " + f.note;
  } catch (e) {
    FL.data = null;
    flCheck();
    $("flInfo").textContent = `Không tải được ${f.file}: ${e.message}`;
  }
}
if ($("flRelGet")) $("flRelGet").onclick = relGet;
document.querySelectorAll(".flOpen").forEach((b) => (b.onclick = () => {
  $("flasher").hidden = false;
  if (S.lastHello && /STM32/i.test(S.lastHello.board)) FL.target = "stm";
  $("flTarget").querySelectorAll("button").forEach((x) => x.setAttribute("aria-pressed", String(x.dataset.t === FL.target)));
  flHow();
  flCheck();
  if (REL.list) relRender(); else relLoad();
}));
$("flTarget").querySelectorAll("button").forEach((b) => (b.onclick = () => {
  if (FL.busy) return;
  FL.target = b.dataset.t;
  $("flTarget").querySelectorAll("button").forEach((x) => x.setAttribute("aria-pressed", String(x === b)));
  flHow();
  flCheck();
  relRender();
}));
$("flFile").onchange = async (ev) => {
  const f = ev.target.files[0];
  ev.target.value = "";
  if (!f) return;
  FL.file = f;
  FL.data = new Uint8Array(await f.arrayBuffer());
  flCheck();
};
$("flClose").onclick = () => { if (!FL.busy) $("flasher").hidden = true; };
$("flGo").onclick = async () => {
  if (FL.busy || !FL.data) return;
  if (S.armed) return toast("Hãy disarm trước.");
  $("flLog").textContent = "";
  // keep the current settings before the flash (a new firmware may reset them)
  if ($("flBackup") && $("flBackup").checked && S.params && Object.keys(S.params).length) {
    try { await $("backupBtn").onclick(); flLog("Đã tải file sao lưu cấu hình."); } catch (e) { flLog("Không tải được file sao lưu (bỏ qua)."); }
  }
  FL.busy = true;
  $("flGo").disabled = true;
  try {
    if (FL.target === "esp") await espFlashFlow();
    else await dfuFlashFlow(false);
  } catch (e) {
    flLog("Lỗi: " + (e && e.message ? e.message : e));
    toast("Nạp firmware thất bại: " + (e && e.message ? e.message : e));
  }
  FL.busy = false;
  flCheck();
};
$("flDfu").onclick = async () => {
  if (FL.busy) return;
  FL.busy = true;
  try { await dfuFlashFlow(true); } catch (e) { flLog("Lỗi: " + e.message); }
  FL.busy = false;
};

// ---------------- ESP32 ROM serial loader (SLIP, esptool protocol) ----------------
// image header chip id -> name; ROM magic (register 0x40001000) or GET_SECURITY_INFO chip id
const ESP_CHIPS = { 0: "ESP32", 9: "ESP32-S3", 13: "ESP32-C6", 18: "ESP32-P4" };
const ESP_MAGIC = { 0x00F01D83: 0, 0x9: 9, 0x2CE0806F: 13, 0x0ADDBAD0: 18 };
const ESP_FLASH_MB = { 0: 4, 9: 8, 13: 8, 18: 16 };
const ESP = { SECURITY_INFO: 0x14, SYNC: 0x08, READ_REG: 0x0A, SPI_SET_PARAMS: 0x0B, SPI_ATTACH: 0x0D, DEFL_BEGIN: 0x10, DEFL_DATA: 0x11, DEFL_END: 0x12, MD5: 0x13 };
class EspRom {
  constructor(port) { this.port = port; this.rx = []; this.frames = []; this.esc = false; this.inFrame = false; this.cur = []; this.waiters = []; }
  async open(baud) {
    await this.port.open({ baudRate: baud, bufferSize: 65536 });
    this.writer = this.port.writable.getWriter();
    this.reader = this.port.readable.getReader();
    this.alive = true;
    (async () => {
      try { for (;;) { const { value, done } = await this.reader.read(); if (done) break; if (value) this.feed(value); } } catch (e) {}
      this.alive = false;
    })();
  }
  async close() {
    try { await this.reader.cancel(); } catch (e) {}
    try { this.reader.releaseLock(); } catch (e) {}
    try { this.writer.releaseLock(); } catch (e) {}
    try { await this.port.close(); } catch (e) {}
  }
  feed(bytes) {
    for (const b of bytes) {
      if (b === 0xC0) {
        if (this.inFrame && this.cur.length) { this.frames.push(new Uint8Array(this.cur)); this.wake(); }
        this.inFrame = true; this.cur = []; this.esc = false;
      } else if (this.inFrame) {
        if (this.esc) { this.cur.push(b === 0xDC ? 0xC0 : b === 0xDD ? 0xDB : b); this.esc = false; }
        else if (b === 0xDB) this.esc = true;
        else this.cur.push(b);
      }
    }
  }
  wake() { const w = this.waiters; this.waiters = []; w.forEach((f) => f()); }
  async signals(dtr, rts) { await this.port.setSignals({ dataTerminalReady: dtr, requestToSend: rts }); }
  async resetToBootloader(usbJtag) {
    if (usbJtag) {  // esptool "USB-JTAG/Serial" sequence
      await this.signals(false, false); await sleep(100);
      await this.signals(true, false); await sleep(100);
      await this.signals(false, true); await sleep(100);
      await this.signals(false, false);
    } else {  // classic auto-reset circuit: EN low, then IO0 low while EN rises
      await this.signals(false, true); await sleep(100);
      await this.signals(true, false); await sleep(50);
      await this.signals(false, false);
    }
  }
  async hardReset() { await this.signals(false, true); await sleep(120); await this.signals(false, false); }
  async send(op, data, chk = 0) {
    const pkt = new Uint8Array(8 + data.length);
    const dv = new DataView(pkt.buffer);
    pkt[0] = 0; pkt[1] = op; dv.setUint16(2, data.length, true); dv.setUint32(4, chk, true);
    pkt.set(data, 8);
    const out = [0xC0];
    for (const b of pkt) { if (b === 0xC0) out.push(0xDB, 0xDC); else if (b === 0xDB) out.push(0xDB, 0xDD); else out.push(b); }
    out.push(0xC0);
    await this.writer.write(new Uint8Array(out));
  }
  async response(op, ms) {
    const end = Date.now() + ms;
    for (;;) {
      while (this.frames.length) {
        const f = this.frames.shift();
        if (f.length >= 8 && f[0] === 1 && f[1] === op) {
          const dv = new DataView(f.buffer, f.byteOffset);
          return { val: dv.getUint32(4, true), data: f.slice(8) };
        }
      }
      const left = end - Date.now();
      if (left <= 0) throw new Error("ROM không trả lời (lệnh 0x" + op.toString(16) + ")");
      await new Promise((r) => { const t = setTimeout(r, left); this.waiters.push(() => { clearTimeout(t); r(); }); });
    }
  }
  async cmd(op, data, ms = 3000, chk = 0) {
    await this.send(op, data, chk);
    const r = await this.response(op, ms);
    const st = op === ESP.MD5 ? r.data[32] : r.data[0];
    if (st !== 0) throw new Error(`ROM báo lỗi 0x${(r.data[op === ESP.MD5 ? 33 : 1] || 0).toString(16)} ở lệnh 0x${op.toString(16)}`);
    return r;
  }
  async sync() {
    const d = new Uint8Array(36);
    d.set([0x07, 0x07, 0x12, 0x20]);
    d.fill(0x55, 4);
    for (let i = 0; i < 12; i++) {
      this.frames = [];
      try { await this.send(ESP.SYNC, d); await this.response(ESP.SYNC, 150); await sleep(60); this.frames = []; return true; } catch (e) {}
    }
    return false;
  }
}
const u32le = (...v) => { const b = new Uint8Array(v.length * 4), dv = new DataView(b.buffer); v.forEach((x, i) => dv.setUint32(i * 4, x >>> 0, true)); return b; };
async function deflate(data) {
  const s = new Blob([data]).stream().pipeThrough(new CompressionStream("deflate"));
  return new Uint8Array(await new Response(s).arrayBuffer());
}
async function waitPort(match, ms) {
  const end = Date.now() + ms;
  while (Date.now() < end) {
    const ports = await navigator.serial.getPorts();
    const p = ports.find(match);
    if (p) return p;
    await sleep(200);
  }
  return null;
}
async function espFlashFlow() {
  if (!("serial" in navigator)) throw new Error("trình duyệt không có Web Serial (dùng Chrome / Edge)");
  let port = null;
  const wasLink = LINK.open && LINK.kind === "usb";
  if (wasLink) {
    port = LINK.port;
    const s3 = S.lastHello && /S3/.test(S.lastHello.board);
    if (s3) {
      flLog("Yêu cầu FC khởi động vào chế độ nạp…");
      try { await api("/api/cmd?c=bootloader", { method: "POST", timeout: 1500 }); } catch (e) {}
    }
    await closeLink();
    linkState(false);
    if (s3) {  // the USB device restarts as the ROM loader
      const info = port.getInfo();
      await sleep(1500);
      port = (await waitPort((p) => p.getInfo().usbVendorId === info.usbVendorId, 6000)) || port;
    }
  } else {
    port = await navigator.serial.requestPort();
  }
  const usbJtag = port.getInfo().usbVendorId === 0x303A;
  const rom = new EspRom(port);
  await rom.open(115200);
  try {
    flLog("Đưa chip vào chế độ nạp…");
    await rom.resetToBootloader(usbJtag);
    await sleep(100);
    if (!(await rom.sync())) throw new Error("không thấy ROM bootloader: giữ BOOT, bấm RESET rồi thử lại");
    // newer ROMs (S3, C6, P4) answer GET_SECURITY_INFO with their chip id; the ESP32 ROM does not
    let chipId = null;
    try { const r = await rom.cmd(ESP.SECURITY_INFO, new Uint8Array(0), 500); if (r.data.length >= 16) chipId = new DataView(r.data.buffer, r.data.byteOffset).getUint32(12, true); } catch (e) {}
    if (!(chipId in ESP_CHIPS)) {
      const magic = (await rom.cmd(ESP.READ_REG, u32le(0x40001000))).val >>> 0;
      chipId = magic in ESP_MAGIC ? ESP_MAGIC[magic] : null;
      if (chipId === null) throw new Error("chip không hỗ trợ (magic 0x" + magic.toString(16) + ")");
    }
    const chip = ESP_CHIPS[chipId];
    flLog("Chip: " + chip);
    const d = FL.data;
    const appOnly = d[0] === 0xE9 && !(d.length > 0x8002 && d[0x8000] === 0xAA && d[0x8001] === 0x50);
    if (appOnly) {
      const want = d[12] | (d[13] << 8);
      if (want !== chipId) throw new Error(`file dành cho ${ESP_CHIPS[want] || "chip " + want}, board là ${chip}`);
    }
    const s3 = chipId !== 0;  // every ROM after the ESP32 takes 5 words in FLASH_BEGIN
    await rom.cmd(ESP.SPI_ATTACH, new Uint8Array(8));
    await rom.cmd(ESP.SPI_SET_PARAMS, u32le(0, ESP_FLASH_MB[chipId] * 1048576, 65536, 4096, 256, 0xFFFF));
    const parts = appOnly ? [[0xE000, new Uint8Array(0x2000).fill(0xFF), "otadata (khởi động ô ota_0)"], [0x10000, d, "firmware"]] : [[0, d, "toàn bộ"]];
    const total = parts.reduce((a, p) => a + p[1].length, 0);
    let done = 0;
    for (const [addr, img, label] of parts) {
      const comp = await deflate(img);
      const BS = 0x400, blocks = Math.ceil(comp.length / BS), erase = Math.ceil(img.length / BS) * BS;
      flLog(`Ghi ${label}: ${(img.length / 1024).toFixed(0)} kB (nén ${(comp.length / 1024).toFixed(0)} kB) tại 0x${addr.toString(16)}`);
      const beginArgs = s3 ? u32le(erase, blocks, BS, addr, 0) : u32le(erase, blocks, BS, addr);
      await rom.cmd(ESP.DEFL_BEGIN, beginArgs, 3000 + Math.ceil(erase / 1048576 * 30000));
      for (let i = 0; i < blocks; i++) {
        const chunk = comp.subarray(i * BS, (i + 1) * BS);
        let chk = 0xEF;
        for (const b of chunk) chk ^= b;
        const pkt = new Uint8Array(16 + chunk.length);
        pkt.set(u32le(chunk.length, i, 0, 0));
        pkt.set(chunk, 16);
        await rom.cmd(ESP.DEFL_DATA, pkt, 8000, chk);
        flPct(((done + img.length * (i + 1) / blocks) / total) * 100);
      }
      done += img.length;
      const md5 = new TextDecoder().decode((await rom.cmd(ESP.MD5, u32le(addr, img.length, 0, 0), 8000 + img.length / 100)).data.slice(0, 32));
      if (md5 !== md5hex(img)) throw new Error("kiểm tra MD5 sai: ghi không đúng");
      flLog("  MD5 khớp.");
    }
    await rom.cmd(ESP.DEFL_END, u32le(1));  // stay in the loader; the reset below starts the firmware
    flLog("Khởi động lại vào firmware mới…");
    await rom.hardReset();
    flPct(100);
  } finally {
    await rom.close();
  }
  toast("Đã nạp firmware.");
  flLog("Xong. Đang kết nối lại…");
  await sleep(usbJtag ? 2500 : 1200);
  const again = usbJtag ? (await waitPort((p) => p.getInfo().usbVendorId === 0x303A, 6000)) || port : port;
  $("flasher").hidden = true;
  if (!LINK.open) connectPort(again);
}

// MD5 (RFC 1321) for the ROM's flash checksum
function md5hex(bytes) {
  const K = new Uint32Array(64), sh = [7, 12, 17, 22, 5, 9, 14, 20, 4, 11, 16, 23, 6, 10, 15, 21];
  for (let i = 0; i < 64; i++) K[i] = Math.floor(Math.abs(Math.sin(i + 1)) * 4294967296) >>> 0;
  const n = bytes.length, total = (((n + 8) >> 6) + 1) * 64, m = new Uint8Array(total);
  m.set(bytes); m[n] = 0x80;
  const dv = new DataView(m.buffer);
  dv.setUint32(total - 8, (n * 8) >>> 0, true);
  dv.setUint32(total - 4, Math.floor(n / 0x20000000), true);
  let a0 = 0x67452301, b0 = 0xefcdab89, c0 = 0x98badcfe, d0 = 0x10325476;
  for (let o = 0; o < total; o += 64) {
    let A = a0, B = b0, C = c0, D = d0;
    for (let i = 0; i < 64; i++) {
      let F, g;
      if (i < 16) { F = (B & C) | (~B & D); g = i; }
      else if (i < 32) { F = (D & B) | (~D & C); g = (5 * i + 1) % 16; }
      else if (i < 48) { F = B ^ C ^ D; g = (3 * i + 5) % 16; }
      else { F = C ^ (B | ~D); g = (7 * i) % 16; }
      F = (F + A + K[i] + dv.getUint32(o + g * 4, true)) >>> 0;
      A = D; D = C; C = B;
      const s = sh[(i >> 4) * 4 + (i % 4)];
      B = (B + ((F << s) | (F >>> (32 - s)))) >>> 0;
    }
    a0 = (a0 + A) >>> 0; b0 = (b0 + B) >>> 0; c0 = (c0 + C) >>> 0; d0 = (d0 + D) >>> 0;
  }
  return [a0, b0, c0, d0].map((x) => [0, 8, 16, 24].map((s) => ((x >>> s) & 255).toString(16).padStart(2, "0")).join("")).join("");
}

// ---------------- STM32 DfuSe over WebUSB ----------------
const DFU_FILTER = [{ vendorId: 0x0483, productId: 0xDF11 }];
async function dfuFlashFlow(picked) {
  if (!("usb" in navigator)) throw new Error("trình duyệt không có WebUSB (dùng Chrome / Edge)");
  let dev = null;
  if (!picked) {
    if (LINK.open && LINK.kind === "usb") {
      flLog("Yêu cầu FC khởi động vào DFU…");
      try { await api("/api/cmd?c=bootloader", { method: "POST", timeout: 1500 }); } catch (e) {}
      await closeLink();
      linkState(false);
      await sleep(1500);
    }
    const end = Date.now() + 4000;
    while (!dev && Date.now() < end) { dev = (await navigator.usb.getDevices()).find((d) => d.vendorId === 0x0483 && d.productId === 0xDF11); if (!dev) await sleep(250); }
    if (!dev) {
      try { dev = await navigator.usb.requestDevice({ filters: DFU_FILTER }); }
      catch (e) {
        flLog("Bấm \"Chọn thiết bị DFU\" và chọn STM32 BOOTLOADER (lần đầu trình duyệt cần bạn cho phép).");
        $("flDfu").hidden = false;
        return;
      }
    }
  } else {
    dev = await navigator.usb.requestDevice({ filters: DFU_FILTER });
  }
  $("flDfu").hidden = true;
  await dfuWrite(dev, FL.data, 0x08000000);
  toast("Đã nạp firmware.");
  flLog("Xong. FC khởi động lại; cổng COM sẽ tự kết nối lại.");
  $("flasher").hidden = true;
  showConnect("Đã nạp firmware. Chờ FC khởi động lại; trang tự kết nối khi cổng xuất hiện.");
}
async function dfuWrite(dev, img, base) {
  const ctl = (request, value) => ({ requestType: "class", recipient: "interface", request, value, index: 0 });
  const status = async () => {
    const r = await dev.controlTransferIn(ctl(3, 0), 6);
    const d = new Uint8Array(r.data.buffer);
    return { status: d[0], poll: d[1] | (d[2] << 8) | (d[3] << 16), state: d[4] };
  };
  const idle = async () => {
    for (let i = 0; i < 2000; i++) {
      const s = await status();
      if (s.status !== 0) throw new Error("DFU lỗi trạng thái " + s.status);
      if (s.state === 5 || s.state === 2) return;
      if (s.state === 10) throw new Error("DFU báo lỗi");
      await sleep(Math.max(2, s.poll));
    }
    throw new Error("DFU quá thời gian");
  };
  const dnload = async (block, data) => { await dev.controlTransferOut(ctl(1, block), data); };
  const special = async (cmd, addr) => { await dnload(0, new Uint8Array([cmd, addr & 255, (addr >> 8) & 255, (addr >> 16) & 255, (addr >>> 24) & 255])); await idle(); };
  await dev.open();
  if (dev.configuration === null) await dev.selectConfiguration(1);
  await dev.claimInterface(0);
  await dev.selectAlternateInterface(0, 0);
  try {
    let s = await status();
    if (s.state === 10) { await dev.controlTransferOut(ctl(4, 0)); s = await status(); }
    if (s.state !== 2) { await dev.controlTransferOut(ctl(6, 0)); await status(); }
    // sectors from the interface name, e.g. "@Internal Flash  /0x08000000/04*016Kg,01*064Kg,03*128Kg"
    const name = dev.configuration.interfaces[0].alternates[0].interfaceName || "";
    const m = /\/0x([0-9a-f]+)\/([^@]*)/i.exec(name);
    const sectors = [];
    let a = m ? parseInt(m[1], 16) : base;
    for (const part of (m ? m[2] : "04*016Kg,01*064Kg,03*128Kg").split(",")) {
      const p = /(\d+)\*(\d+)([KM ])/i.exec(part);
      if (!p) continue;
      const size = +p[2] * (p[3].toUpperCase() === "M" ? 1048576 : p[3].toUpperCase() === "K" ? 1024 : 1);
      for (let i = 0; i < +p[1]; i++) { sectors.push([a, size]); a += size; }
    }
    const need = sectors.filter(([sa, sz]) => sa < base + img.length && sa + sz > base);
    flLog(`STM32 DFU: xoá ${need.length} sector (${(img.length / 1024).toFixed(0)} kB)…`);
    let k = 0;
    for (const [sa] of need) { await special(0x41, sa); flPct(++k / need.length * 20); }
    await special(0x21, base);
    const XFER = 2048, n = Math.ceil(img.length / XFER);
    flLog("Ghi…");
    for (let i = 0; i < n; i++) {
      await dnload(2 + i, img.subarray(i * XFER, (i + 1) * XFER));
      await idle();
      flPct(20 + (i + 1) / n * 60);
    }
    flLog("Đọc lại để kiểm tra…");
    await special(0x21, base);
    await dev.controlTransferOut(ctl(6, 0));
    await status();
    for (let i = 0; i < n; i++) {
      const len = Math.min(XFER, img.length - i * XFER);
      const r = await dev.controlTransferIn(ctl(2, 2 + i), XFER);
      const got = new Uint8Array(r.data.buffer);
      for (let j = 0; j < len; j++) if (got[j] !== img[i * XFER + j]) throw new Error(`đọc lại sai tại 0x${(base + i * XFER + j).toString(16)}`);
      flPct(80 + (i + 1) / n * 20);
    }
    flLog("Khớp. Khởi động vào firmware…");
    await dev.controlTransferOut(ctl(6, 0));
    await status();
    await special(0x21, base);
    await dnload(2, new Uint8Array(0));
    try { await status(); } catch (e) {}  // the device leaves DFU and disconnects
  } finally {
    try { await dev.close(); } catch (e) {}
  }
}

// ---------------------------------------------------------------- SD card: wiring (System tab), logs (Blackbox tab)
// Read only when the Blackbox tab opens or on request: nothing is polled, the card is not touched in flight.
const SD_PINS = ["sd_clk", "sd_cmd", "sd_d0", "sd_d1", "sd_d2", "sd_d3"];
const SD_NEED = [0, 0x27, 0x07, 0x3F];  // pins of each mode (bit 0 CLK ... bit 5 D3)
const SD_STATE = { off: "Tắt", unsupported: "Không dùng được", "no card": "Không thấy thẻ", ready: "Sẵn sàng",
  logging: "Đang ghi", full: "Thẻ đầy", error: "Lỗi ghi" };
function buildSdParams() {
  const host = $("sdParams");
  if (!host) return;
  host.innerHTML = "";
  const sd = S.sd || { modes: 0 };
  if (!has("sd_mode")) return;
  if (!sd.modes) { host.innerHTML = '<p class="muted">Board này không nối được thẻ SD.</p>'; return; }
  host.appendChild(prow("sd_mode"));
  host.querySelectorAll('select[data-p="sd_mode"] option').forEach((o) => { if (+o.value && !(sd.modes & (1 << +o.value))) o.remove(); });
  const opt = document.createElement("div");
  opt.id = "sdOpt";
  host.appendChild(opt);
  sdPinRows();
}
function sdPinRows() {
  const box = $("sdOpt");
  if (!box) return;
  box.innerHTML = "";
  const m = val("sd_mode") | 0, sd = S.sd || {};
  if (!m) return;
  // sd_dma: STM32H7 SDMMC only (the boards with fixed card pins)
  ["sd_mhz", "sd_keep", "sd_dma"].forEach((n) => has(n) && !(sd.fixed && n === "sd_mhz") && (n !== "sd_dma" || sd.fixed) && box.appendChild(prow(n)));
  if (sd.fixed) {
    box.insertAdjacentHTML("beforeend", '<p class="muted">Khe thẻ có sẵn trên board (SDMMC1, 4-bit): không cần chọn chân.</p>');
    return;
  }
  const def = sd.def || [];
  SD_PINS.forEach((n, i) => {
    if (!(SD_NEED[m] & (1 << i)) || !has(n)) return;
    const row = prow(n), h = row.querySelector(".hint");
    if (h) h.insertAdjacentHTML("beforeend", def[i] !== undefined && def[i] !== 255 ? ` · <b>255 = GPIO ${def[i]}</b> (mặc định board)` : " · <b>board không có chân mặc định: phải chọn</b>");
    box.appendChild(row);
  });
  box.insertAdjacentHTML("beforeend", '<p class="muted" id="sdWire"></p>');
  sdWiring();
}
function sdWiring() {
  const w = $("sdWire");
  if (!w) return;
  const m = val("sd_mode") | 0, def = (S.sd && S.sd.def) || [];
  const names = m === 1 ? ["SCK", "MOSI", "MISO", "", "", "CS"] : ["CLK", "CMD", "D0", "D1", "D2", "D3"];
  let missing = false;
  const parts = SD_PINS.map((n, i) => {
    if (!(SD_NEED[m] & (1 << i))) return "";
    const v = val(n) !== 255 ? val(n) : def[i] !== undefined && def[i] !== 255 ? def[i] : -1;
    if (v < 0) missing = true;
    return `${names[i]} → ${v < 0 ? "<b>chưa chọn</b>" : "GPIO " + v}`;
  }).filter(Boolean);
  w.innerHTML = `Đấu dây: ${parts.join(" · ")}.${missing ? " Chọn chân còn thiếu rồi lưu và khởi động lại." : ""}` +
    (m === 1 ? " SPI dùng bus SPI thứ hai của chip (chỉ khi không lắp chip OSD MAX7456)." : "") +
    " Module thẻ: nguồn 3.3 V (module có ổn áp thì 5 V). Dùng thẻ FAT32, loại A1/A2 hoặc U3.";
}

async function sdRefresh() {
  const pnl = $("sdPanel");
  if (!pnl) return;
  if (!S.sd || !S.sd.modes || !has("sd_mode")) { pnl.hidden = true; return; }
  pnl.hidden = false;
  try { sdRender(await api("/api/sd", { timeout: 5000 })); }
  catch (e) { $("sdInfo").textContent = "Không đọc được trạng thái thẻ: " + e.message; }
}
function sdRender(j) {
  const gb = (b) => (b < 1e9 ? (b / 1e6).toFixed(0) + " MB" : (b / 1e9).toFixed(b < 1e10 ? 2 : 1) + " GB");
  const kb = (b) => (b >= 1048576 ? (b / 1048576).toFixed(1) + " MB" : Math.max(1, Math.round(b / 1024)) + " kB");
  const good = j.state === "ready" || j.state === "logging";
  $("sdDot").style.background = good ? "var(--ok)" : j.state === "off" ? "var(--ink-2)" : "var(--danger)";
  let t = SD_STATE[j.state] || j.state;
  if (j.state === "off") t += ": chọn kiểu nối thẻ trong tab Hệ thống → Blackbox";
  if (j.type) t += " · " + j.type;
  if (j.total) t += ` · đã dùng ${gb(j.used)} / ${gb(j.total)}`;
  if (j.err) t += " · " + j.err;
  if (good && val("bb_backend") !== 3) t += " · Blackbox đang ghi nơi khác: đặt Nơi ghi = Thẻ SD.";
  if (j.wms) t += ` · ghi chậm nhất ${j.wms} ms`;
  if (j.drop) t += ` · đã mất ${kb(j.drop)} (thẻ chậm / đầy hàng đợi)`;
  $("sdInfo").textContent = t;
  $("sdBar").hidden = !j.total;
  if (j.total) $("sdBar").firstChild.style.width = Math.min(100, j.used / j.total * 100).toFixed(1) + "%";
  const box = $("sdFiles");
  if (!j.listed) { box.innerHTML = j.state === "off" || j.state === "no card" || j.state === "unsupported" ? "" : '<p class="muted">Danh sách log hiện khi drone đã disarm.</p>'; return; }
  const files = (j.files || []).slice().sort((a, b) => (a.n < b.n ? 1 : -1));
  if (!files.length) { box.innerHTML = '<p class="muted">Chưa có log nào trên thẻ.</p>'; return; }
  const rate = val("bb_rate_hz") || 500;
  box.innerHTML = files.map((f) => `<div class="sdrow"><div><span class="num">${esc(f.n)}</span> <span class="muted">${kb(f.s)} · ≈ ${fmtDur(f.s / (rate * 24))}</span></div>` +
    `<div class="row"><button class="btn primary" data-sda="${esc(f.n)}">Phân tích</button><button class="btn" data-sds="${esc(f.n)}">Lưu</button>` +
    `<button class="btn danger" data-sdd="${esc(f.n)}">Xoá</button></div></div>`).join("") +
    '<p class="muted">Mỗi lần arm là một file. Có thể rút thẻ ra và mở file .BBL bằng nút "Mở file log" ở trên (nhanh hơn nhiều khi log lớn).</p>';
  box.querySelectorAll("button[data-sda]").forEach((b) => b.onclick = async () => { const d = await sdGet(b.dataset.sda, b, files); if (d) loadLog(d); });
  box.querySelectorAll("button[data-sds]").forEach((b) => b.onclick = async () => {
    const d = await sdGet(b.dataset.sds, b, files);
    if (!d) return;
    const a = document.createElement("a");
    a.href = URL.createObjectURL(new Blob([d], { type: "application/octet-stream" }));
    a.download = b.dataset.sds;
    a.click();
    setTimeout(() => URL.revokeObjectURL(a.href), 10000);
  });
  box.querySelectorAll("button[data-sdd]").forEach((b) => b.onclick = async () => {
    if (!confirm(`Xoá ${b.dataset.sdd} khỏi thẻ SD? Không khôi phục được.`)) return;
    try { await api("/api/cmd?c=sd_rm_" + b.dataset.sdd, { method: "POST" }); toast("Đã xoá " + b.dataset.sdd); }
    catch (e) { toast("Không xoá được: " + e.message); }
    sdRefresh();
  });
}
function fmtDur(s) { return s < 60 ? Math.max(1, Math.round(s)) + " s" : Math.floor(s / 60) + " ph " + String(Math.round(s % 60)).padStart(2, "0") + " s"; }
async function sdGet(name, btn, files) {
  const f = files.find((x) => x.n === name), size = f ? f.s : 0;
  if (LINK.open && size > 8 * 1048576 && !confirm(`${name} lớn (${(size / 1048576).toFixed(0)} MB): qua USB sẽ mất vài phút. Rút thẻ ra đọc bằng máy tính sẽ nhanh hơn. Vẫn tải?`)) return null;
  const label = btn.textContent;
  btn.disabled = true;
  btn.textContent = "Đang tải…";
  try {
    return await api("/api/sdfile?n=" + encodeURIComponent(name), { binary: true, timeout: Math.max(60000, size / 40),
      progress: (d, n) => { btn.textContent = Math.round(d / n * 100) + "%"; } });
  } catch (e) {
    toast(S.armed ? "Hãy disarm trước khi tải log." : "Không tải được: " + e.message);
    return null;
  } finally { btn.disabled = false; btn.textContent = label; }
}
$("sdRefresh").onclick = sdRefresh;
$("sdMount").onclick = async () => {
  try { await api("/api/cmd?c=sd_mount", { method: "POST" }); } catch (e) { toast(e.message); return; }
  $("sdInfo").textContent = "Đang tìm thẻ…";
  setTimeout(sdRefresh, 1500);
};


// ---------------------------------------------------------------- installable app (GitHub Pages copy)
// tools/make_pages.py adds the manifest and <meta name="aq-build"> to the published page only. There the
// page registers sw.js (offline copy, network first) and, whenever there is internet, compares its build
// with version.json on the site: a newer build is loaded at once when no FC is connected, otherwise
// right after disconnecting (or now, from the bar).
const PWA = {
  build: (document.querySelector('meta[name="aq-build"]') || {}).content || "",
  version: (document.querySelector('meta[name="aq-version"]') || {}).content || "",
  next: null, prompt: null, timer: null,
};
function pwaIdle() { const c = $("connect"); return !!c && !c.hidden; }  // connect dialog shown = no FC
function pwaInfo() {
  const el = $("pwaInfo");
  if (!el) return;
  el.hidden = false;
  const inst = matchMedia("(display-mode: standalone)").matches;
  el.textContent = `Ứng dụng web v${PWA.version} · bản ${PWA.build.slice(0, 7)}${inst ? " · đã cài" : ""} · ` +
    (navigator.onLine ? "tự cập nhật từ GitHub khi có mạng" : "đang offline, dùng bản đã lưu");
}
async function pwaCheck() {
  if (!PWA.build || !navigator.onLine) return pwaInfo();
  let v;
  try {
    // the query defeats the CDN cache (GitHub Pages caches files ~10 min)
    const r = await fetch("version.json?t=" + Date.now(), { cache: "no-store" });
    if (!r.ok) return;
    v = await r.json();
  } catch (e) { return; }
  pwaInfo();
  if (!v.build || v.build === PWA.build) return;
  // came back from a reload that still served the old page (CDN not refreshed yet): try again later
  let tried = null;
  try { tried = sessionStorage.getItem("aq.pwaTried"); } catch (e) {}
  if (tried === v.build) return pwaBar(v, true);
  PWA.next = v;
  if (pwaIdle()) pwaApply(); else pwaBar(v, false);
}
function pwaApply() {
  const v = PWA.next;
  if (!v) return;
  try { sessionStorage.setItem("aq.pwaTried", v.build); } catch (e) {}
  toast(`Đang cập nhật lên v${v.version}…`);
  // a new URL: neither the browser nor the CDN can answer with the old copy
  setTimeout(() => location.replace(location.pathname + "?v=" + v.build), 600);
}
function pwaBar(v, waiting) {
  let bar = $("pwaBar");
  if (!bar) {
    bar = document.createElement("div");
    bar.id = "pwaBar";
    bar.className = "pwa-bar";
    document.body.appendChild(bar);
  }
  bar.innerHTML = waiting
    ? `<span>Bản v${esc(v.version)} đã đăng trên GitHub, máy chủ đang làm mới (vài phút).</span><button class="btn" id="pwaLater">Ẩn</button>`
    : `<span>Có bản mới v${esc(v.version)} (${esc(v.date || "")}). Tự cập nhật khi ngắt kết nối FC.</span><button class="btn primary" id="pwaNow">Cập nhật ngay</button><button class="btn" id="pwaLater">Để sau</button>`;
  bar.hidden = false;
  if ($("pwaNow")) $("pwaNow").onclick = () => { if (pwaIdle() || confirm("Cập nhật sẽ tải lại trang và ngắt kết nối với FC. Tiếp tục?")) pwaApply(); };
  $("pwaLater").onclick = () => { bar.hidden = true; };
}
function pwaInit() {
  if (!PWA.build) return;  // page served by the FC or opened as a file: no app features
  pwaInfo();
  const okHost = location.protocol === "https:" || /^(localhost|127\.0\.0\.1)$/.test(location.hostname);
  if ("serviceWorker" in navigator && okHost) navigator.serviceWorker.register("sw.js").catch(() => {});
  try {
    const t = sessionStorage.getItem("aq.pwaTried");
    if (t && t === PWA.build) { sessionStorage.removeItem("aq.pwaTried"); setTimeout(() => toast(`Đã cập nhật lên v${PWA.version}.`), 800); }
  } catch (e) {}
  // offer "install" where the browser supports it
  addEventListener("beforeinstallprompt", (e) => {
    e.preventDefault();
    PWA.prompt = e;
    const b = $("pwaInstall");
    if (b) b.hidden = false;
  });
  const b = $("pwaInstall");
  if (b) b.onclick = async () => { if (!PWA.prompt) return; PWA.prompt.prompt(); try { await PWA.prompt.userChoice; } catch (e) {} PWA.prompt = null; b.hidden = true; };
  addEventListener("online", () => { pwaInfo(); pwaCheck(); });
  addEventListener("offline", pwaInfo);
  document.addEventListener("visibilitychange", () => { if (!document.hidden) pwaCheck(); });
  setTimeout(pwaCheck, 2000);
  PWA.timer = setInterval(() => {
    if (PWA.next && pwaIdle()) return pwaApply();  // the FC was disconnected: take the waiting update
    if (Date.now() - (PWA.last || 0) > 15 * 60000) { PWA.last = Date.now(); pwaCheck(); }
  }, 5000);
  PWA.last = Date.now();
}
pwaInit();
