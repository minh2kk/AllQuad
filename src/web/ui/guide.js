// AllQuad configurator - guidance layer (loaded after ux.js)
//  1. first-flight guide (tab Trước bay): wiring, sensors, board orientation (live tilt test), receiver,
//     modes, battery, motors, failsafe (live test: radio off -> the FC must notice), arm check
//  2. RPM filter state in words (PID & lọc, Motor & ESC, pre-flight list)
//  3. optical flow / rangefinder check (tab Cảm biến): roll the drone by hand, the page works out
//     flow_align / flow_flip from the gyro and proposes them
//  4. flight summary after a blackbox log is opened (battery, current, height, vibration, notes)
//  5. glossary: "?" next to headings with a known term, search entries, a dialog with every term
//  6. "tăng / giảm thì sao" lines under the parameters pilots tune most
//  7. tab bar: in the simple view the expert tabs (Tham số, CLI) fold under the "Công cụ" label
"use strict";
const GD = { live: null, flow: null };
const gdLs = (k, d) => { try { const v = localStorage.getItem(k); return v === null ? d : JSON.parse(v); } catch (e) { return d; } };
const gdLsSet = (k, v) => { try { localStorage.setItem(k, JSON.stringify(v)); } catch (e) {} };

// ---------------------------------------------------------------- 1. first-flight guide
const FF_KEY = "aq-ff";
function ffMarks() { return gdLs(FF_KEY, {}); }
function ffMark(id, on) { const m = ffMarks(); if (on) m[id] = Date.now(); else delete m[id]; gdLsSet(FF_KEY, m); ffRender(S.status); }
function ffBoxes() {
  const b = new Set();
  for (let i = 1; i <= 20; i++) if (S.params[`mr${i}_mode`]) { const v = val(`mr${i}_mode`); if (v) b.add(v); }
  return b;
}
// every step: state ok / warn / bad / todo / off, a line of detail, buttons [label, spec | fn]
const FF_STEPS = [
  { id: "wire", t: "Đấu dây theo sơ đồ chân", why: "Mỗi thiết bị một UART riêng, RX của module nối TX của FC và ngược lại; IMU, baro, compass trên đúng bus.",
    check: (st, m) => m.wire ? ["ok", "đã xác nhận"] : ["todo", "mở sơ đồ chân, so với dây thật rồi bấm Đã xong"],
    act: [["Sơ đồ chân", () => pinShow()]], manual: true },
  { id: "sens", t: "Cảm biến & hiệu chuẩn", why: "Accel phẳng thì ANGLE mới đứng yên; compass chỉ cần khi bay GPS.",
    check: (st) => {
      const sn = st.sensors || {};
      if (!sn.imu) return ["bad", "không thấy IMU: kiểm tra dây SPI"];
      if (S.params.acc_calibrated && !val("acc_calibrated")) return ["bad", "accel chưa hiệu chuẩn"];
      if (sn.mag && !["x", "y", "z"].some((k) => S.params["mag_bias_" + k] && val("mag_bias_" + k))) return ["warn", `${sn.imu} · compass chưa hiệu chuẩn`];
      return ["ok", [sn.imu, sn.baro, sn.mag].filter(Boolean).join(" · ")];
    }, act: [["Cảm biến", "sensors"]] },
  { id: "orient", t: "Hướng lắp FC", why: "Lắp lệch 90° hay ngược mà không chỉnh imu_align: drone lật ngay khi cất cánh.",
    check: (st, m) => m.orient ? ["ok", "đã thử nghiêng: đúng chiều"] : ["todo", "bấm Thử nghiêng, cầm drone làm theo hướng dẫn"],
    act: [["Thử nghiêng", () => ffLive("orient")]] },
  { id: "rx", t: "Tay điều khiển", why: "Cần ga, roll, pitch, yaw phải đi đúng kênh, đúng chiều và về giữa khi thả tay.",
    check: (st, m) => st.rxok === false ? ["bad", "chưa nhận receiver: bind, cổng, giao thức"] :
      m.rx ? ["ok", "đang nhận" + (st.lq ? ` · LQ ${st.lq}%` : "") + " · đã kiểm tra cần"] : ["todo", "đang nhận: chạy Dò cần trong tab Tay điều khiển rồi bấm Đã xong"],
    act: [["Tay điều khiển", "rx"]], manual: true },
  { id: "modes", t: "Công tắc ARM & chế độ bay", why: "ARM trên một công tắc riêng; lần đầu nên bay ANGLE (tự cân bằng).",
    check: () => {
      const b = ffBoxes();
      if (!b.has(1)) return ["bad", "chưa gán công tắc ARM"];
      if (!b.has(2) && !(S.params.beginner && val("beginner"))) return ["warn", "chưa có ANGLE: drone sẽ bay ACRO (không tự cân bằng)"];
      return ["ok", [...b].map((x) => BOXES[x]).filter(Boolean).join(", ")];
    }, act: [["Chế độ", "modes"]] },
  { id: "bat", t: "Đo pin", why: "Sai điện áp thì cảnh báo pin yếu và failsafe pin sẽ báo sai lúc bay.",
    check: (st) => {
      const b = st.bat || {};
      if (!b.v || b.v < 1) return ["warn", "chưa đo được điện áp: cắm pin, hoặc chân đo chưa nối"];
      const c = b.cells || 0, pc = c ? b.v / c : 0;
      if (!c || pc < 3.0 || pc > 4.4) return ["bad", `${b.v.toFixed(2)} V nhưng không ra số cell hợp lý: kiểm tra vbat_scale`];
      return ["ok", `${b.v.toFixed(2)} V · ${c}S · ${pc.toFixed(2)} V/cell`];
    }, act: [["Pin & dòng", "modes#batHelp"]] },
  { id: "motor", t: "Thứ tự & chiều quay motor", why: "Motor sai vị trí hoặc quay ngược chiều: drone lật hoặc xoay tít khi cất cánh.",
    check: (st, m) => m.motor ? ["ok", "đã thử (tháo cánh)"] : ["todo", "THÁO CÁNH, thử từng motor ở tab Motor rồi bấm Đã xong"],
    act: [["Thử motor", "motors"]], manual: true },
  { id: "fs", t: "Thử failsafe", why: "Mất sóng mà FC không biết thì drone bay mất. Thử trên bàn: tắt tay điều khiển, FC phải báo mất sóng.",
    check: (st, m) => m.fs ? ["ok", "FC đã nhận ra khi tắt tay điều khiển" + (S.params.fs_action ? ` · khi bay sẽ: ${proValText("fs_action", val("fs_action"))}` : "")] :
      ["todo", "bấm Thử failsafe (cần tháo cánh)"],
    act: [["Thử failsafe", () => ffLive("fs")]] },
  { id: "arm", t: "Điều kiện arm", why: "FC tự kiểm tra trước khi cho arm; còn mục nào thì bấm để sửa.",
    check: (st) => {
      const bl = (st.blockers || []).filter((x) => x !== "CLI");
      return st.armed ? ["warn", "đang ARM"] : bl.length ? ["bad", bl.map((x) => (BLOCK_HELP[x] || [x])[0]).join("; ")] : ["ok", "đạt (trang cấu hình đang nối thì vẫn chặn, tự hết khi ngắt)"];
    }, act: [["Tổng quan", "overview"]] },
];
function ffHost() {
  if ($("ffPanel")) return $("ffPanel");
  const sec = $("tab-preflight");
  if (!sec) return null;
  const h = document.createElement("h2");
  h.textContent = "Hướng dẫn bay lần đầu"; h.dataset.j = "Bay lần đầu";
  const p = document.createElement("div");
  p.className = "panel"; p.id = "ffPanel";
  sec.appendChild(h); sec.appendChild(p);
  p.addEventListener("click", (e) => {
    const b = e.target.closest("button");
    if (!b) return;
    if (b.dataset.ffGo) gotoTab(b.dataset.ffGo);
    else if (b.dataset.ffFn) { const st = FF_STEPS.find((x) => x.id === b.dataset.ffFn); const a = st && st.act[+b.dataset.k]; if (a) a[1](); }
    else if (b.dataset.ffDone) ffMark(b.dataset.ffDone, true);
    else if (b.dataset.ffUndo) ffMark(b.dataset.ffUndo, false);
    else if (b.id === "ffBeginner") { setEdit("beginner", 1); toast("beginner 1 vào hàng chờ: bấm Lưu."); }
    else if (b.id === "ffBb") { setEdit("bb_rate_hz", 500); toast("Blackbox 500 Hz vào hàng chờ: bấm Lưu."); }
    else if (b.id === "ffReset") { gdLsSet(FF_KEY, {}); ffRender(S.status); }
    else if (b.id === "ffLiveStop") ffLiveStop();
  });
  return p;
}
function ffRender(st) {
  const p = ffHost();
  if (!p || !S.params || !st) return;
  const m = ffMarks();
  const rows = FF_STEPS.map((s) => ({ s, r: s.check(st, m) }));
  const done = rows.filter((x) => x.r[0] === "ok").length;
  const first = rows.find((x) => x.r[0] !== "ok");
  const lv = GD.live ? ffLiveHtml() : "";
  const fin = !first ? `<div class="ff-fin"><b>Sẵn sàng cho chuyến bay đầu.</b> Bay ở bãi trống, cao 1–2 m, ANGLE, vài phút rồi hạ cánh và mở log ở tab Blackbox để xem tóm tắt.
      <div class="row">${S.params.beginner && !val("beginner") ? '<button type="button" class="btn" id="ffBeginner">Bật chế độ bay lần đầu</button>' : ""}${S.params.bb_rate_hz && !val("bb_rate_hz") ? '<button type="button" class="btn" id="ffBb">Bật blackbox</button>' : ""}</div></div>` : "";
  setHtml(p, `<div class="card-h"><span class="dot"></span><h3>Từng bước, theo thứ tự</h3><span class="card-x num">${done}/${FF_STEPS.length}</span></div>
    <div class="ff-bar"><i style="width:${Math.round(100 * done / FF_STEPS.length)}%"></i></div>${lv}
    <ol class="ff">${rows.map(({ s, r }, i) => `<li class="pf-row ${r[0] === "todo" ? "warn" : r[0]}${first && first.s === s ? " ff-now" : ""}"><i></i>
      <div><b>${i + 1}. ${esc(s.t)}</b><small>${esc(r[1])}</small>${first && first.s === s ? `<small class="ff-why">${esc(s.why)}</small>` : ""}</div>
      <span class="row ff-act">${s.act.map(([l, a], k) => typeof a === "string" ? `<button type="button" class="btn ghost" data-ff-go="${a}">${esc(l)}</button>` :
        `<button type="button" class="btn ghost" data-ff-fn="${s.id}" data-k="${k}">${esc(l)}</button>`).join("")}${s.manual && r[0] !== "ok" && r[0] !== "bad" ? `<button type="button" class="btn" data-ff-done="${s.id}">Đã xong</button>` : ""}${m[s.id] ? `<button type="button" class="btn ghost" data-ff-undo="${s.id}" title="Làm lại bước này">↺</button>` : ""}</span></li>`).join("")}</ol>
    ${fin}<p class="muted" style="font-size:.8rem;margin:8px 0 0">Tiến độ lưu trong trình duyệt này. <button type="button" class="linkbtn" id="ffReset">Làm lại từ đầu</button></p>`);
}
// live tests: orientation (tilt right, nose up) and failsafe (radio off, then on)
function ffLive(kind) {
  if (kind === "fs" && S.status && S.status.armed) return toast("Disarm trước.");
  GD.live = { kind, phase: 0, t0: Date.now(), hold: 0, msg: "" };
  ffRender(S.status);
  const p = $("ffPanel"); if (p) p.scrollIntoView({ block: "start" });
}
function ffLiveStop() { GD.live = null; ffRender(S.status); }
function ffLiveHtml() {
  const L = GD.live;
  const T = {
    orient: ["Đặt drone nằm phẳng, đứng yên.", "Nghiêng sang PHẢI khoảng 30° (cánh tay phải hạ xuống).", "Đặt phẳng lại, rồi NGÓC MŨI LÊN khoảng 30°."],
    fs: ["THÁO CÁNH. Tay điều khiển đang bật, FC đang nhận. Giờ TẮT tay điều khiển.", "Tốt: FC đã báo mất sóng. Giờ BẬT lại tay điều khiển."],
  }[L.kind];
  const step = T[Math.min(L.phase, T.length - 1)];
  return `<div class="ff-live${L.fail ? " bad" : ""}"><b>${L.fail ? "Chưa đúng" : `Bước ${L.phase + 1}/${T.length}`}</b><span>${esc(L.fail || step)}</span>${L.msg ? `<small>${esc(L.msg)}</small>` : ""}
    <button type="button" class="btn ghost" id="ffLiveStop">${L.fail ? "Đóng" : "Huỷ"}</button></div>`;
}
function ffLiveTick(st) {
  const L = GD.live;
  if (!L || L.fail) return;
  const now = Date.now();
  if (L.kind === "orient") {
    const [r, p] = st.att || [0, 0];
    const held = (ok) => { if (ok) { L.hold = L.hold || now; return now - L.hold > 600; } L.hold = 0; return false; };
    if (L.phase === 0) { L.msg = `roll ${r.toFixed(0)}° · pitch ${p.toFixed(0)}°`; if (held(Math.abs(r) < 6 && Math.abs(p) < 6)) { L.phase = 1; L.hold = 0; } }
    else if (L.phase === 1) {
      L.msg = `roll ${r.toFixed(0)}° (cần > +20) · pitch ${p.toFixed(0)}°`;
      if (Math.abs(p) > 20 && Math.abs(r) < 10) L.fail = "Nghiêng phải mà FC thấy pitch thay đổi: FC lắp lệch 90°. Chỉnh imu_align (thêm 90° hoặc 270°) ở tab Cảm biến rồi thử lại.";
      else if (r < -20) L.fail = "Nghiêng phải mà FC thấy nghiêng trái: FC lắp ngược (180° hoặc lật). Chỉnh imu_align ở tab Cảm biến rồi thử lại.";
      else if (held(r > 20)) { L.phase = 2; L.hold = 0; }
    } else if (L.phase === 2) {
      L.msg = `pitch ${p.toFixed(0)}° (cần > +20) · roll ${r.toFixed(0)}°`;
      if (Math.abs(r) > 20 && Math.abs(p) < 10) L.fail = "Ngóc mũi mà FC thấy roll: FC lắp lệch 90°. Chỉnh imu_align ở tab Cảm biến.";
      else if (p < -20) L.fail = "Ngóc mũi lên mà FC thấy chúc xuống: FC lắp ngược trước / sau. Chỉnh imu_align (180°) ở tab Cảm biến.";
      else if (held(p > 20)) { GD.live = null; toast("Hướng lắp FC đúng."); ffMark("orient", true); return; }
    }
  } else if (L.kind === "fs") {
    const lost = st.rxok === false || st.failsafe || (st.blockers || []).some((b) => b === "NO_RX" || b === "FAILSAFE");
    if (L.phase === 0) {
      const s = Math.round((now - L.t0) / 1000);
      L.msg = lost ? "" : `đang chờ FC báo mất sóng… ${s} s`;
      if (lost) { L.phase = 1; L.lostMs = now - L.t0; }
      else if (now - L.t0 > 20000) L.fail = "Đã 20 s mà receiver vẫn gửi dữ liệu: receiver đang ở chế độ failsafe \"giữ kênh\". Đặt failsafe của receiver là \"No pulses\" (ELRS: mặc định đã đúng; SBUS / PPM: xem hướng dẫn receiver), rồi thử lại.";
    } else {
      L.msg = "đang chờ tín hiệu trở lại…";
      if (!lost && st.rxok !== false) { GD.live = null; toast("Failsafe hoạt động: FC nhận ra mất sóng."); ffMark("fs", true); return; }
    }
  }
  ffRender(st);
}
// overview to-do card: one more suggestion until the guide is finished
function ffDoneAll() { const m = ffMarks(); return ["wire", "orient", "rx", "motor", "fs"].every((k) => m[k]); }

// ---------------------------------------------------------------- 2. RPM filter in words
function rpmState(st) {
  if (!S.params || !S.params.rpm_harmonics) return null;
  const h = val("rpm_harmonics"), pr = val("motor_protocol"), bd = val("dshot_bidir"), tel = val("esc_telemetry");
  const er = st && st.erpm, n = er ? er.filter((x) => x >= 0).length : 0;
  if (!h) return ["off", "RPM filter tắt", "rpm_harmonics = 0: chỉ có notch động và lọc thông thấp. Có ESC BLHeli_32 / AM32 / Bluejay thì bật DShot hai chiều và rpm_harmonics 3: lọc nhiễu motor chính xác hơn, ít trễ hơn."];
  if (!bd && !tel) return ["warn", "RPM filter chưa có nguồn RPM", pr >= 2 ? "Bật dshot_bidir (ESC hỗ trợ) hoặc nối dây telemetry ESC (esc_telemetry): không có RPM thì RPM filter không làm gì." : "Cần giao thức DShot + dshot_bidir, hoặc dây telemetry ESC."];
  if (n === 4) return ["ok", "RPM filter đang chạy", `${h} hài × 4 motor, RPM từ ${bd ? "DShot hai chiều" : "dây telemetry ESC"}.`];
  if (n) return ["warn", `RPM filter thiếu ${4 - n} motor`, `Chỉ ${n}/4 ESC trả RPM: kiểm tra ESC còn lại (firmware, dây tín hiệu).`];
  return ["info", "RPM filter: chờ ESC trả RPM", `Đã bật (${h} hài) nhưng chưa thấy RPM. Bình thường khi motor chưa quay; khi thử motor / arm mà vẫn không có: ESC không hỗ trợ ${bd ? "DShot hai chiều" : "telemetry"}, notch động sẽ thay thế.`];
}
function rpmRender(st) {
  const r = rpmState(st);
  for (const [host, before] of [["tab-pid", "filterParams"], ["tab-motors", "motorParams"]]) {
    const anchor = $(before);
    if (!anchor) continue;
    let el = $(before + "Rpm");
    if (!el) { el = document.createElement("p"); el.id = before + "Rpm"; anchor.before(el); }
    el.hidden = !r;
    if (!r) continue;
    el.className = "note rpm-st " + r[0];
    setHtml(el, `<b>${esc(r[1])}.</b> ${esc(r[2])}`);
  }
}
if (typeof pfItems === "function") {
  const _pf = pfItems;
  pfItems = function (st) {
    const out = _pf(st);
    const r = st && rpmState(st);
    if (r && r[0] !== "info") out.splice(out.length - 1, 0, { state: r[0] === "ok" ? "ok" : r[0] === "off" ? "off" : "warn", title: "Lọc nhiễu RPM", detail: r[1], go: "pid" });
    return out;
  };
}

// ---------------------------------------------------------------- 3. optical flow check
// Over a textured floor, roll / pitch the drone by hand without moving it: the image moves only because the
// sensor turns, so after flow_align / flow_flip it must follow the gyro as the estimator assumes:
// flow x = -pitch rate, flow y = +roll rate (src/estimation/nav_estimator.cpp, fuseFlow).
// 2x2 matrix of an alignment (180° = both axes flipped: several settings give the same result)
function flowXf(align, flip) {
  const v = [[1, 0], [0, 1]].map(([x, y]) => {
    for (let i = 0; i < align; i++) { const t = x; x = -y; y = t; }
    return [flip & 1 ? -x : x, flip & 2 ? -y : y];
  });
  return v.join(",");
}
function flowFit(samples, cur) {
  let best = null;
  const order = [];
  if (cur) order.push([cur[0], cur[1]]);  // the current setting wins a tie
  for (let align = 0; align < 4; align++) for (let flip = 0; flip < 4; flip++) order.push([align, flip]);
  for (const [align, flip] of order) {
    let sxx = 0, sx = 0, sy = 0, syy = 0;
    for (const [fx, fy, p, q] of samples) {
      let ix = fx, iy = fy;
      for (let i = 0; i < align; i++) { const t = ix; ix = -iy; iy = t; }
      if (flip & 1) ix = -ix;
      if (flip & 2) iy = -iy;
      sx += ix * -q; sy += iy * p; sxx += q * q; syy += p * p;
    }
    const score = (sxx ? sx / sxx : 0) + (syy ? sy / syy : 0);
    if (!best || score > best.score + 1e-6) best = { align, flip, score, kx: sxx ? sx / sxx : 0, ky: syy ? sy / syy : 0 };
  }
  return best;
}
function flowHost() {
  if ($("flowPanel") || !$("escPanel")) return $("flowPanel");
  const p = document.createElement("div");
  p.className = "panel"; p.id = "flowPanel"; p.hidden = true;
  p.innerHTML = `<div class="card-h"><span class="dot"></span><h3>Kiểm tra optical flow & khoảng cách</h3><span class="card-x num" id="flowNow"></span></div>
    <p class="card-d">Cầm drone cao 0.3–1 m trên nền có hoa văn (thảm, gạch, cỏ), không dịch chuyển, rồi nghiêng qua lại quanh trục roll và trục pitch vài lần. Trang so chiều flow với gyro và đề xuất <span class="num">flow_align</span> / <span class="num">flow_flip</span>.</p>
    <canvas id="flowChart" class="chart" style="height:150px"></canvas>
    <div class="legend"><span><i style="background:var(--roll)"></i>roll rate (rad/s)</span><span><i style="background:var(--pitch)"></i>flow y sau chỉnh</span><span><i style="background:var(--yaw)"></i>chất lượng / 255</span></div>
    <div class="row"><button type="button" class="btn" id="flowGo">Bắt đầu đo</button><span class="muted" id="flowRes"></span></div>`;
  $("escPanel").before(p);
  $("flowGo").onclick = () => {
    if (GD.flow && GD.flow.on) return flowDone();
    GD.flow = { on: true, s: [], plot: [] };
    $("flowGo").textContent = "Dừng & tính";
    $("flowRes").textContent = "Đang đo: nghiêng drone qua lại…";
  };
  return p;
}
function flowAdd(j) {
  const p = flowHost();
  if (!p) return;
  const has = j.flowq >= 0 || j.range >= 0;
  p.hidden = !has;
  if (!has) return;
  $("flowNow").textContent = `${j.range >= 0 ? j.range.toFixed(2) + " m" : "không có khoảng cách"}${j.flowq >= 0 ? ` · chất lượng ${j.flowq}` : " · không có flow"}`;
  const F = GD.flow || (GD.flow = { on: false, s: [], plot: [] });
  if (j.flowq < 0) { $("flowGo").disabled = true; $("flowRes").textContent = "Module chỉ đo khoảng cách: không cần kiểm tra flow (giữ vị trí cần GPS)."; return; }
  $("flowGo").disabled = false;
  const D = Math.PI / 180, p_ = j.gyro[0] * D, q_ = j.gyro[1] * D;
  // current alignment, for the live plot
  let ix = j.flow[0], iy = j.flow[1];
  for (let i = 0; i < (val("flow_align") || 0); i++) { const t = ix; ix = -iy; iy = t; }
  if (val("flow_flip") & 1) ix = -ix;
  if (val("flow_flip") & 2) iy = -iy;
  F.plot.push([p_, iy, j.flowq / 255]);
  if (F.plot.length > 200) F.plot.shift();
  if (F.on && j.flowq >= 30 && (Math.abs(p_) > 0.3 || Math.abs(q_) > 0.3)) F.s.push([j.flow[0], j.flow[1], p_, q_]);
  if (F.on) $("flowRes").textContent = `Đang đo: ${F.s.length} mẫu có chuyển động (cần ≥ 40)…`;
  if (typeof plot === "function") plot($("flowChart"), [0, 1, 2].map((k) => ({ y: F.plot.map((x) => x[k]), color: ["--roll", "--pitch", "--yaw"][k], width: 1.5 })), { xmin: 0, xmax: 199 });
}
function flowDone() {
  const F = GD.flow;
  F.on = false;
  $("flowGo").textContent = "Đo lại";
  if (F.s.length < 40) { $("flowRes").textContent = `Chỉ có ${F.s.length} mẫu: nghiêng mạnh hơn (> 20°/s) và lâu hơn, chất lượng ≥ 30.`; return; }
  const curA = val("flow_align") || 0, curF = val("flow_flip") || 0;
  const b = flowFit(F.s, [curA, curF]);
  const same = flowXf(b.align, b.flip) === flowXf(curA, curF);
  const k = (b.kx + b.ky) / 2;
  let msg = same ? "Hướng đang đúng." : `Đề xuất flow_align ${ENUMS.flow_align[b.align]}, flow_flip ${b.flip ? ["", "lật X", "lật Y", "lật X+Y"][b.flip] : "không lật"}.`;
  if (k < 0.3) msg += ` Biên độ flow chỉ ${(k * 100).toFixed(0)}% so với gyro: chưa tin được (nền trơn, quá cao, hoặc hệ số sai).`;
  else if (Math.abs(k - 1) > 0.35) msg += ` Biên độ flow ${(k * 100).toFixed(0)}% so với gyro: ${val("flow_scale") ? "chỉnh flow_scale theo (module MSP)" : "kiểm tra hệ số module"}.`;
  setHtml($("flowRes"), esc(msg) + (!same ? ' <button type="button" class="btn" id="flowApply">Áp dụng</button>' : ""));
  const ap = $("flowApply");
  if (ap) ap.onclick = () => {
    setEdit("flow_align", b.align); setEdit("flow_flip", b.flip);
    if (S.params.flow_scale && k > 0.3 && Math.abs(k - 1) > 0.35 && val("flow_scale")) setEdit("flow_scale", +(val("flow_scale") * k).toFixed(2));
    toast("Đã đưa vào hàng chờ: bấm Lưu, rồi đo lại để xác nhận.");
  };
}
if (typeof sensAdd === "function") { const _sa = sensAdd; sensAdd = function (j) { _sa(j); try { flowAdd(j); } catch (e) {} }; }

// ---------------------------------------------------------------- 4. flight summary
function bbReportRender() {
  const fl = S.flight, res = S.analysis;
  if (!fl || !res || !$("bbResults")) return;
  let p = $("bbReport");
  if (!p) {
    p = document.createElement("div");
    p.className = "panel"; p.id = "bbReport";
    $("bbResults").prepend(p);
  }
  const r = AQ.report(fl, res);
  const f1 = (v, d = 1) => v == null ? "—" : (Math.round(v * Math.pow(10, d)) / Math.pow(10, d)).toFixed(d);
  const mm = (s) => `${Math.floor(s / 60)}:${String(Math.round(s % 60)).padStart(2, "0")}`;
  const tiles = [
    ["Thời gian bay", mm(r.armedSec), r.arms > 1 ? `${r.arms} lần arm` : ""],
    ["Pin", r.vRest ? `${f1(r.vRest, 2)} → ${f1(r.vEnd, 2)} V` : "—", r.cells ? `${r.cells}S · thấp nhất ${f1(r.vMin / r.cells, 2)} V/cell` : ""],
    ["Dòng", r.iMax != null ? `${f1(r.iMax)} A max` : "—", r.mah != null ? `TB ${f1(r.iMean)} A · ${Math.round(r.mah)} mAh` : "không đo"],
    ["Ga", r.hover != null ? `lơ lửng ~${Math.round(r.hover * 100)}%` : "—", `tối đa ${Math.round(r.thrMax * 100)}%`],
    ["Độ cao / xa", r.altMax != null ? `${f1(r.altMax)} m` : "—", r.far != null ? `xa nhất ${f1(r.far, 0)} m` : ""],
    ["Rung", r.vib != null ? `${f1(r.vib)} °/s` : "—", r.vibFilt != null ? `sau lọc ${f1(r.vibFilt, 2)} °/s` : ""],
  ];
  if (r.tempMax != null) tiles.push(["ESC", `${r.tempMax} °C`, "nhiệt độ cao nhất"]);
  const modes = r.modes.map((m) => (typeof MODE_NAMES !== "undefined" && MODE_NAMES[m]) || m).join(", ");
  setHtml(p, `<div class="card-h"><span class="dot"></span><h3>Tóm tắt chuyến bay</h3><span class="card-x muted">${esc(modes)}</span></div>
    <div class="rep-tiles">${tiles.map(([k, v, s]) => `<div><small>${k}</small><b class="num">${esc(v)}</b><small>${esc(s)}</small></div>`).join("")}</div>
    <div class="rep-notes">${r.notes.map((n) => `<p class="rep-n ${n.level}"><i></i>${esc(n.text)}</p>`).join("")}</div>`);
}
if (typeof analyzeFlight === "function") { const _af = analyzeFlight; analyzeFlight = function (i) { _af(i); try { bbReportRender(); } catch (e) { console.warn(e); } }; }

// ---------------------------------------------------------------- 4b. high-rate gyro capture (bb_gyro_s)
function hfHost() {
  let p = $("hfPanel");
  if (p || !$("specChart")) return p;
  p = document.createElement("div");
  p.className = "panel"; p.id = "hfPanel";
  $("specChart").closest(".panel").after(p);
  return p;
}
function hfRender() {
  const fl = S.flight, p = hfHost();
  if (!p || !fl) return;
  const caps = fl.hf || [];
  if (!caps.length) {
    const sd = S.params.bb_backend && val("bb_backend") === 3;
    const can = S.params.bb_gyro_s && sd;
    setHtml(p, `<div class="card-h"><span class="dot"></span><h3>Gyro tần số cao</h3></div>
      <p class="card-d">Log thường ghi ${fl.rateHz} Hz nên chỉ thấy nhiễu dưới ${Math.round(fl.rateHz / 2)} Hz. Nhiễu motor ở ga cao (300–800 Hz) cần <span class="num">bb_gyro_s</span>: vài giây mọi mẫu gyro (tới 4 kHz) sau khi cất cánh, ghi vào thẻ SD.</p>
      ${can ? `<div class="row"><button type="button" class="btn" id="hfOn">Bật: 4 s mỗi chuyến</button><span class="muted">Hoặc khi đang bay: CLI <span class="num">gyrocap 4</span>.</span></div>` :
        `<p class="muted" style="margin:0">${S.params.bb_gyro_s ? "Cần blackbox trên thẻ SD (bb_backend 3)." : "Firmware này chưa có bb_gyro_s."}</p>`}`);
    const b = $("hfOn");
    if (b) b.onclick = () => { setEdit("bb_gyro_s", 4); toast("bb_gyro_s 4 vào hàng chờ: bấm Lưu."); };
    return;
  }
  const ci = Math.min(GD.hfCap || 0, caps.length - 1), c = caps[ci], a = S.specAxis || 0;
  const key = ci + ":" + a;
  if (!c._sp || c._key !== key) {
    const nfft = c.n >= 4096 ? 1024 : 512;
    c._sp = [AQ.spectrum(c.raw[a], c.rate, nfft), AQ.spectrum(c.filt[a], c.rate, nfft)];
    c._key = key;
  }
  const [r, f] = c._sp;
  const pk = AQ.bandPeak(r, 80, c.rate / 2 - 10);
  const k = r.f.indexOf(pk.hz);
  const att = k >= 0 && r.amp[k] > 0 && f.amp[k] > 0 ? 20 * Math.log10(f.amp[k] / r.amp[k]) : null;
  setHtml(p, `<div class="card-h"><span class="dot"></span><h3>Gyro tần số cao</h3><span class="card-x muted num">${caps.length > 1 ? `<select id="hfSel" class="btn">${caps.map((x, i) => `<option value="${i}"${i === ci ? " selected" : ""}>đợt ${i + 1}</option>`).join("")}</select> ` : ""}${c.duration.toFixed(1)} s · ${c.rate} Hz</span></div>
    <p class="card-d">Mọi mẫu gyro trong đợt ghi nhanh, trục theo nút Roll / Pitch / Yaw ở Phổ nhiễu. Thấy được tới ${c.rate / 2} Hz (log thường: ${Math.round(fl.rateHz / 2)} Hz).</p>
    <canvas id="hfChart" class="chart"></canvas>
    <div class="legend"><span><i style="background:var(--ink-2)"></i>Gyro thô</span><span><i style="background:var(--ink)"></i>Gyro sau lọc</span>${S.params.gyro_lpf_hz ? `<span>┆ gyro_lpf_hz ${val("gyro_lpf_hz")} Hz</span>` : ""}</div>
    <p class="muted" style="margin:6px 0 0">${pk.hz ? `Đỉnh nhiễu thô lớn nhất ở <b>${Math.round(pk.hz)} Hz</b> (gấp ${pk.ratio.toFixed(1)} lần mức nền)${att != null ? `, bộ lọc giảm ${Math.abs(att).toFixed(0)} dB ở đó` : ""}.${att != null && att > -10 && pk.ratio > 4 ? " Lọc chưa đủ ở tần số này: bật RPM filter hoặc thêm notch." : ""}` : "Không có đỉnh nhiễu rõ."}</p>`);
  const sel = $("hfSel");
  if (sel) sel.onchange = () => { GD.hfCap = +sel.value; hfRender(); };
  const lines = S.params.gyro_lpf_hz && val("gyro_lpf_hz") ? [{ x: [val("gyro_lpf_hz"), val("gyro_lpf_hz")], y: [0, Math.max(...r.amp)], color: "--yaw", width: 1 }] : [];
  plot($("hfChart"), [{ x: r.f, y: r.amp, color: "--ink-2", width: 1.4 }, { x: f.f, y: f.amp, color: "--ink", width: 2 }].concat(lines), { xmin: 0, ymin: 0, xunit: " Hz" });
}
if (typeof drawBlackbox === "function") { const _db = drawBlackbox; drawBlackbox = function () { _db(); try { hfRender(); } catch (e) { console.warn(e); } }; }

// ---------------------------------------------------------------- 5. glossary
const GLOSS = {
  "PID": "Bộ điều khiển giữ tốc độ quay đúng lệnh. P: phản ứng theo sai lệch hiện tại (cứng, nhanh). I: bù sai lệch kéo dài (gió, trọng tâm lệch). D: hãm, chống vọt lố và rung lắc.",
  "Feedforward": "Đẩy motor ngay theo tốc độ di cần, không chờ sai lệch xuất hiện: cảm giác nhạy, ít trễ. Quá cao: giật, vọt lố khi lắc cần nhanh.",
  "Rates": "Quy đổi độ lệch cần sang tốc độ quay (°/s). Rate cao: xoay nhanh; expo: mềm quanh tâm cần, nhanh ở cuối hành trình.",
  "TPA": "Throttle PID Attenuation: giảm PID khi ga cao, vì lúc đó motor phản ứng mạnh hơn và dễ rung.",
  "Anti-gravity": "Tăng I tạm thời khi ga thay đổi nhanh để mũi drone không bị chúi / ngóc.",
  "Airmode": "Giữ điều khiển tư thế cả khi ga về 0 (lộn, rơi tự do). Tắt thì ga 0 là mất điều khiển.",
  "Bộ lọc": "Gyro đo cả chuyển động thật lẫn rung của motor / cánh. Lọc bớt rung thì motor êm, mát; lọc quá nhiều thì thêm trễ, bay kém nhạy.",
  "Lọc thông thấp": "Cắt mọi tín hiệu nhanh hơn tần số đặt (Hz). Tần số thấp: êm nhưng trễ; cao: nhạy nhưng nhiễu lọt qua.",
  "Notch": "Bộ lọc khoét một dải tần hẹp (ví dụ cộng hưởng khung) mà không làm trễ phần còn lại.",
  "Notch động": "Notch tự tìm đỉnh nhiễu trong phổ gyro và bám theo khi ga đổi.",
  "RPM filter": "Notch đặt đúng tốc độ quay từng motor (lấy từ ESC qua DShot hai chiều): lọc nhiễu motor chính xác nhất, ít trễ nhất.",
  "DShot": "Giao thức số giữa FC và ESC: không cần hiệu chỉnh ga; bản hai chiều (bidirectional) còn gửi RPM về FC.",
  "ESC": "Bộ điều tốc: biến lệnh của FC thành dòng điện cho motor.",
  "Failsafe": "Việc FC làm khi mất sóng tay điều khiển: tắt motor, hạ cánh tại chỗ hoặc bay về nhà (RTH, cần GPS).",
  "RTH": "Return To Home: tự bay về điểm arm rồi hạ cánh. Cần GPS fix tốt và compass đã hiệu chuẩn.",
  "ANGLE": "Chế độ tự cân bằng: thả cần là drone nằm phẳng. Phù hợp bay lần đầu.",
  "HORIZON": "Như ANGLE khi cần gần giữa, nhưng cho lộn vòng khi đẩy cần hết cỡ.",
  "ACRO": "Không tự cân bằng: cần điều khiển tốc độ quay, thả cần drone giữ nguyên góc. Dành cho bay FPV thành thạo.",
  "ALTHOLD": "Giữ độ cao bằng baro / cảm biến khoảng cách; cần ga giữa = đứng độ cao.",
  "POSHOLD": "Giữ vị trí bằng GPS hoặc optical flow; thả cần là drone đứng yên.",
  "ARM": "Cho phép motor quay. FC chỉ cho arm khi mọi điều kiện an toàn đạt.",
  "Blackbox": "Ghi log từng mẫu gyro, lệnh, motor… trong lúc bay để phân tích nhiễu và chỉnh PID sau đó.",
  "Accel": "Gia tốc kế: cho biết hướng trọng lực, dùng để tự cân bằng. Phải hiệu chuẩn khi đặt drone phẳng.",
  "Gyro": "Con quay hồi chuyển: đo tốc độ quay ba trục, cảm biến quan trọng nhất của FC.",
  "Compass": "La bàn từ: cho hướng mũi drone so với Bắc, cần cho GPS / RTH. Đặt xa dây nguồn và motor.",
  "Baro": "Cảm biến áp suất: đo độ cao tương đối.",
  "EKF": "Bộ ước lượng kết hợp gyro, accel, baro, GPS, compass để ra vị trí / vận tốc / tư thế tin cậy nhất.",
  "Optical flow": "Camera nhỏ nhìn xuống đất, đo trôi ngang; cùng cảm biến khoảng cách thì giữ vị trí được trong nhà, không GPS.",
  "LQ": "Link Quality: tỉ lệ gói tin tay điều khiển tới nơi (%). Dưới 70% nên quay về.",
  "RSSI": "Cường độ sóng thu được (dBm). Gần −100 dBm (tuỳ chế độ ELRS) là sắp mất sóng.",
  "ELRS": "ExpressLRS: hệ tay điều khiển tầm xa, độ trễ thấp, nối FC qua giao thức CRSF trên UART.",
  "CRSF": "Giao thức của TBS / ELRS giữa receiver và FC: kênh điều khiển và telemetry hai chiều.",
  "UART": "Cổng nối tiếp (RX / TX). RX của thiết bị nối TX của FC và ngược lại.",
  "OSD": "Chữ và số hiện chồng lên hình camera FPV (điện áp, thời gian bay…).",
  "VTX": "Bộ phát hình FPV.",
  "Motor idle": "Tốc độ motor tối thiểu khi đã arm: đủ cao để motor không khựng khi ga về thấp.",
  "Vọt lố": "Overshoot: tốc độ quay vượt quá lệnh rồi mới về. Nhiều quá: drone giật, dội lại sau cú lắc cần.",
  "Prop wash": "Rung lắc khi drone rơi qua luồng gió của chính cánh (hạ ga đột ngột, lộn vòng).",
  "Đáp ứng bước": "Step response: gyro bám một cú đổi lệnh đột ngột như thế nào. Lý tưởng: lên nhanh tới 1.0, vọt lố dưới 10–15%.",
  "Phổ nhiễu": "Biểu đồ nhiễu theo tần số: đỉnh cao là nguồn rung (motor, khung) cần lọc.",
  "Hiệu chuẩn": "Calibration: dạy FC giá trị chuẩn của cảm biến (accel phẳng, compass xoay mọi hướng, dòng / áp pin).",
};
const GLOSS_KEYS = Object.keys(GLOSS).sort((a, b) => b.length - a.length);
function noAccent(s) { return String(s).normalize("NFD").replace(/[̀-ͯ]/g, "").replace(/đ/g, "d").replace(/Đ/g, "D").toLowerCase(); }
function glossFind(text) {
  const t = noAccent(text);
  for (const k of GLOSS_KEYS) {
    const kk = noAccent(k);
    const i = t.indexOf(kk);
    if (i >= 0 && !/[a-z0-9]/.test(t[i - 1] || " ") && !/[a-z0-9]/.test(t[i + kk.length] || " ")) return k;
  }
  return null;
}
function glossMark(root) {
  (root || document).querySelectorAll("main h2, main h3").forEach((h) => {
    if (h.dataset.gl !== undefined) return;
    const k = glossFind(h.textContent);
    h.dataset.gl = k || "";
    if (!k) return;
    const b = document.createElement("button");
    b.type = "button"; b.className = "gl-q"; b.dataset.term = k;
    b.setAttribute("aria-label", "Giải thích: " + k); b.title = k + ": " + GLOSS[k];
    h.appendChild(b);
  });
}
function glossShow(term) {
  const m = proModal("glossDlg", "Thuật ngữ", `<input type="search" id="glIn" placeholder="tìm: notch, failsafe, rates…" autocomplete="off"><dl class="gl-list" id="glList"></dl>`);
  m.hidden = false;
  const draw = () => {
    const q = noAccent($("glIn").value.trim());
    const keys = Object.keys(GLOSS).filter((k) => !q || noAccent(k + " " + GLOSS[k]).includes(q));
    setHtml($("glList"), keys.map((k) => `<dt id="gl-${noAccent(k).replace(/\W+/g, "-")}">${esc(k)}</dt><dd>${esc(GLOSS[k])}</dd>`).join("") || '<p class="muted">Không có.</p>');
  };
  $("glIn").value = "";
  $("glIn").oninput = draw;
  draw();
  if (term) {
    const el = $("gl-" + noAccent(term).replace(/\W+/g, "-"));
    if (el) { el.scrollIntoView({ block: "center" }); el.classList.add("on"); }
  } else setTimeout(() => $("glIn").focus(), 30);
}
document.addEventListener("click", (e) => {
  const b = e.target.closest && e.target.closest(".gl-q");
  if (!b) return;
  e.preventDefault(); e.stopPropagation();
  glossShow(b.dataset.term);
}, true);
if (typeof SEARCH_ACTIONS !== "undefined") {
  SEARCH_ACTIONS.push(["Thuật ngữ (giải thích PID, notch, failsafe…)", "fn:glossShow", "thuat ngu giai thich la gi glossary tu dien " + Object.keys(GLOSS).join(" ")]);
  SEARCH_ACTIONS.push(["Hướng dẫn bay lần đầu", "fn:ffOpen", "bay lan dau huong dan first flight bat dau moi"]);
  SEARCH_ACTIONS.push(["Kiểm tra optical flow", "fn:flowOpen", "optical flow mtf mt-01p huong lap flow_align kiem tra"]);
}
function ffOpen() { gotoTab("preflight#ffPanel"); }
function flowOpen() { gotoTab("sensors#flowPanel"); }

// ---------------------------------------------------------------- 6. what raising / lowering does
const EFFECT = {
  roll_p: ["bám lệnh chặt, nhạy hơn; quá cao: rung nhanh, motor nóng", "mềm, trôi, chậm bám lệnh"],
  pitch_p: ["bám lệnh chặt, nhạy hơn; quá cao: rung nhanh, motor nóng", "mềm, trôi, chậm bám lệnh"],
  yaw_p: ["xoay đuôi dứt khoát hơn", "đuôi lờ đờ, trôi khi đổi ga"],
  roll_i: ["giữ góc chắc trước gió / trọng tâm lệch; quá cao: dao động chậm", "drone trôi dần khỏi góc khi giữ cần"],
  pitch_i: ["giữ góc chắc trước gió / trọng tâm lệch; quá cao: dao động chậm", "drone trôi dần khỏi góc khi giữ cần"],
  roll_d: ["hãm tốt, ít vọt lố và prop wash; quá cao: motor nóng, nhiễu khuếch đại", "vọt lố, dội lại sau cú lắc cần"],
  pitch_d: ["hãm tốt, ít vọt lố và prop wash; quá cao: motor nóng, nhiễu khuếch đại", "vọt lố, dội lại sau cú lắc cần"],
  roll_f: ["phản ứng tức thì theo cần; quá cao: giật, vọt lố", "trễ nhẹ, mềm"],
  pitch_f: ["phản ứng tức thì theo cần; quá cao: giật, vọt lố", "trễ nhẹ, mềm"],
  gyro_lpf_hz: ["ít trễ, nhạy hơn; nhiễu lọt qua nhiều hơn (motor nóng)", "êm, mát motor; thêm trễ, bay kém sắc"],
  dterm_lpf_hz: ["D phản ứng nhanh hơn; D ồn hơn, motor nóng", "D êm; hãm chậm, dễ prop wash"],
  motor_idle: ["motor không khựng khi ga thấp, chống prop wash", "drone rơi nhanh hơn khi về ga, motor dễ mất đồng bộ"],
  angle_limit: ["ANGLE cho nghiêng nhiều, bay nhanh hơn", "an toàn, chậm, dễ điều khiển"],
  rc_smooth_hz: ["cần nhạy, sát tay; tín hiệu RX xấu thì giật", "mượt hơn, thêm trễ cảm nhận được"],
  rc_deadband: ["cần không trôi khi gimbal kém", "phản ứng cả với chạm cần rất nhẹ"],
  fs_delay_ms: ["chịu được mất sóng chớp nhoáng", "phản ứng failsafe sớm hơn"],
  rth_alt: ["bay về cao, tránh cây / nhà", "bay về thấp, tốn ít pin hơn"],
  vbat_warn_cell: ["cảnh báo sớm, pin bền", "bay lâu hơn, pin nhanh chai"],
  dyn_notch_count: ["lọc được nhiều đỉnh nhiễu", "ít trễ hơn (đủ khi đã có RPM filter)"],
  rpm_harmonics: ["lọc cả hài bậc cao của motor", "ít trễ hơn; 3 là mặc định tốt"],
  thr_limit_pct: ["đủ lực đẩy", "ga dễ kiểm soát (drone dư lực, bay trong nhà)"],
};
if (typeof prow === "function") {
  const _prow = prow;
  prow = function (n) {
    const d = _prow(n);
    const e = EFFECT[n];
    if (e && d && d.querySelector) {
      const h = d.querySelector(".hint");
      if (h) { const s = document.createElement("div"); s.className = "fx"; s.innerHTML = `<span>↑ ${esc(e[0])}</span><span>↓ ${esc(e[1])}</span>`; h.after(s); }
    }
    return d;
  };
}

// ---------------------------------------------------------------- 7. tab bar: simple view folds the expert tabs
// (full parameter list, CLI) under the "Công cụ" group label; search and links still open them
const TAB_EXPERT = ["params", "cli"];
function tabGroups() {
  const nav = document.querySelector("nav.tabs");
  if (!nav || nav.dataset.gx) return;
  nav.dataset.gx = 1;
  for (const t of TAB_EXPERT) { const b = nav.querySelector(`button[data-tab="${t}"]`); if (b) b.classList.add("tab-x"); }
  const g = [...nav.querySelectorAll(".tgroup")].find((x) => /Công cụ/.test(x.textContent));
  if (!g) return;
  g.classList.add("tg-x");
  g.title = "Chế độ đơn giản: bấm để hiện / ẩn tab Tham số và CLI";
  g.onclick = () => document.body.classList.toggle("tabs-open");
}
// ---------------------------------------------------------------- hooks
function guideStatus(st) {
  try {
    ffLiveTick(st);
    if (shown("preflight")) ffRender(st);
    if (shown("pid") || shown("motors")) rpmRender(st);
  } catch (e) { console.warn(e); }
}
{
  const _ts = typeof toolsStatus === "function" ? toolsStatus : null;
  toolsStatus = function (st) { if (_ts) _ts(st); guideStatus(st); };
}
tabGroups();
setTimeout(() => glossMark(), 0);
// headings added later (panels built after the parameters arrive): marked at most twice a second
new MutationObserver(() => { if (!GD.glT) GD.glT = setTimeout(() => { GD.glT = 0; glossMark(); }, 500); }).observe(document.querySelector("main"), { childList: true, subtree: true });
