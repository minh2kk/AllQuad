// ============================================================================================
// AllQuad configurator - extras: hardware setup wizard, in-browser demo (firmware compiled to
// WebAssembly), OpenStreetMap tiles under the navigation map, blackbox compare + CSV export,
// parameter history with undo, EKF panel and the warnings of the newer firmware features.
// Bundled into index.html after tools.js by tools/embed_web.py. Hooks called by index.html:
//   extrasParamsLoaded() extrasParamEdit(n) extrasStatus(st) extrasTabShown(t)
//   extrasApplied(changes) mapTiles(g, w, h, k, cn, ce) bbRefSeries()
// ============================================================================================
const lsGet = (k, d) => { try { const v = localStorage.getItem(k); return v === null ? d : JSON.parse(v); } catch (e) { return d; } };
const lsSet = (k, v) => { try { localStorage.setItem(k, JSON.stringify(v)); } catch (e) {} };
function saveText(name, text, type = "text/csv") {
  const a = document.createElement("a");
  a.href = URL.createObjectURL(new Blob([text], { type }));
  a.download = name;
  document.body.appendChild(a);
  a.click();
  a.remove();
  setTimeout(() => URL.revokeObjectURL(a.href), 5000);
}
const stamp = () => { const d = new Date(), p = (x) => String(x).padStart(2, "0"); return `${d.getFullYear()}${p(d.getMonth() + 1)}${p(d.getDate())}-${p(d.getHours())}${p(d.getMinutes())}`; };

// ---------------------------------------------------------------- hooks
function extrasParamsLoaded() {
  histAfterLoad();
  histRender();
  ekfPanelInit();
  wzRender();
  calBuild();
  lgLoad();
  escBuild();
  moBuild();
}
function extrasParamEdit(n) {
  if (n === "imu_align" && WZ.step === 1) wzRender();
  if (/^(imu_align|mag_align)$/.test(n)) calSync();
  if (n === "motor_protocol") escBuild();
  if (/^motor\d_out$|^yaw_reversed$/.test(n)) moSync();
}
function extrasStatus(st) {
  const w = [];
  if (st.warn & 512) w.push(`<span class="chip bad">Rung mạnh: ${st.vibe != null ? st.vibe.toFixed(0) + " m/s²" : ""} (kiểm tra cánh, ốc motor, đệm FC)</span>`);
  if (st.warn & 1024) w.push('<span class="chip bad">EKF: cảm biến bị loại, xem tab Điều hướng</span>');
  if (st.warn & 2048) w.push('<span class="chip">Launch control: đẩy ga để phóng</span>');
  if (st.warn & 4096) w.push('<span class="chip bad">GPS Rescue: bay về không tiến triển, đang hạ cánh</span>');
  const host = $("xWarn");
  if (host) { setHtml(host, w.join("")); host.hidden = !w.length; }
  if (shown("nav")) ekfRender(st);
  if (shown("motors")) rxPreview(st);
  if (shown("setup")) wzLive(st);
  if (shown("sensors")) calLive(st);
  if (shown("ports")) lgLive(st);
  perfCard(st);
  voiceStatus(st);
}
function extrasTabShown(t) {
  if (t !== "setup") wzMotorStop();
  if (t !== "motors") { escCalStop(); moStop(); }
  if (t === "setup") wzRender();
  if (t === "params") { histRender(); cmpHost(); }
  if (t === "pid") sliderHost();
  if (t === "motors") vibHost();
  if (t === "system") voiceHost();
}

// ---------------------------------------------------------------- setup wizard
// 0 intro, 1 orientation, 2 accel, 3 compass, 4 baro, 5 GPS, 6 receiver, 7 motors, 8 summary
const WZ = { step: 0, caps: {}, done: {}, baro0: null, baroMax: 0, rx: {}, motor: -1, motorTimer: null, sensTimer: null, last: null };
const WZ_STEPS = ["Bắt đầu", "Hướng FC", "Accel", "Compass", "Baro", "GPS", "Tay điều khiển", "Motor", "Xong"];
// replaceable (tests stub it): one /api/sensors reading
let wzSensors = () => api("/api/sensors", { timeout: 1500 });

// the firmware's drv::applyAlign: k = 0..3 rotates 0/90/180/270° clockwise, +4 flips (upside down)
function applyAlign(v, a) {
  let [x, y, z] = v;
  if (a >= 4) { y = -y; z = -z; a -= 4; }
  return a === 1 ? [-y, x, z] : a === 2 ? [-x, -y, z] : a === 3 ? [y, -x, z] : [x, y, z];
}
// raw sensor vector from an aligned reading: the alignments are signed permutations (orthogonal),
// so the inverse is the transpose of the matrix built from the basis vectors
function unAlign(v, a) {
  const cols = [[1, 0, 0], [0, 1, 0], [0, 0, 1]].map((e) => applyAlign(e, a));
  return [0, 1, 2].map((i) => cols[i][0] * v[0] + cols[i][1] * v[1] + cols[i][2] * v[2]);
}
// level: specific force (0, 0, -g) in FRD; nose up 90°: (+g, 0, 0)
function solveAlign(level, noseUp, cur) {
  const rl = unAlign(level, cur), rn = unAlign(noseUp, cur);
  const nl = Math.hypot(...rl) || 1, nn = Math.hypot(...rn) || 1;
  let best = 0, score = -Infinity;
  for (let a = 0; a < 8; a++) {
    const l = applyAlign(rl, a), n = applyAlign(rn, a);
    const s = -l[2] / nl + n[0] / nn;
    if (s > score) { score = s; best = a; }
  }
  return { align: best, quality: score / 2 };  // 1 = both captures exactly on their axis
}
const ALIGN_NAME = (a) => `${[0, 90, 180, 270][a & 3]}°${a >= 4 ? ", lật ngửa" : ""}`;

function wzSection() {
  if ($("tab-setup")) return;
  const tabs = document.querySelector("nav.tabs");
  const b = document.createElement("button");
  b.setAttribute("role", "tab");
  b.dataset.tab = "setup";
  b.setAttribute("aria-selected", "false");
  b.innerHTML = '<svg viewBox="0 0 24 24"><path d="M4 20l6-6M14 4l6 6-8 8-6-6z"/><path d="M13 7l4 4"/></svg>Thiết lập';
  tabs.insertBefore(b, tabs.querySelector('[data-tab="overview"]').nextSibling);
  const sec = document.createElement("section");
  sec.id = "tab-setup";
  sec.hidden = true;
  sec.innerHTML = `<h2>Thiết lập phần cứng</h2>
    <p class="muted">Đi lần lượt từng bước cho một drone mới lắp. Mỗi bước kiểm tra bằng số liệu thật từ FC; thay đổi nằm ở thanh "chưa áp dụng" cho đến khi bấm Áp dụng hoặc Lưu ở bước cuối.</p>
    <ol class="wzsteps" id="wzSteps"></ol>
    <div class="panel" id="wzBody"></div>
    <div class="row" style="justify-content:space-between"><button class="btn" id="wzPrev">Quay lại</button><button class="btn primary" id="wzNext">Tiếp</button></div>`;
  document.querySelector("main").insertBefore(sec, $("tab-pid"));
  // the page binds the tab buttons once at load: this one joins in the same way
  const all = () => document.querySelectorAll("nav.tabs button");
  b.onclick = () => {
    all().forEach((x) => x.setAttribute("aria-selected", String(x === b)));
    document.querySelectorAll("main > section").forEach((s) => { s.hidden = s.id !== "tab-setup"; });
    window.scrollTo(0, 0);
    if (typeof onTabShown === "function") onTabShown("setup");
    extrasTabShown("setup");
  };
  $("wzPrev").onclick = () => wzGo(WZ.step - 1);
  $("wzNext").onclick = () => wzGo(WZ.step + 1);
}
function wzGo(i) {
  wzMotorStop();
  WZ.step = Math.max(0, Math.min(WZ_STEPS.length - 1, i));
  if (WZ.step === 4) { WZ.baro0 = null; WZ.baroMax = 0; }
  if (WZ.step === 6) WZ.rx = {};
  wzRender();
}
function wzMark(k, ok) { WZ.done[k] = ok; const s = $("wzSteps"); if (s) wzStepList(); }
function wzStepList() {
  setHtml($("wzSteps"), WZ_STEPS.map((n, i) => `<li class="${i === WZ.step ? "cur" : ""}${WZ.done[i] ? " ok" : ""}"><button type="button" data-i="${i}">${i + 1}. ${n}</button></li>`).join(""));
  $("wzSteps").querySelectorAll("button").forEach((b) => b.onclick = () => wzGo(+b.dataset.i));
}
function wzRender() {
  if (!$("tab-setup") || !S.params.imu_align) return;
  wzStepList();
  const st = S.status || {}, sn = st.sensors || {};
  const body = $("wzBody");
  $("wzPrev").disabled = WZ.step === 0;
  $("wzNext").hidden = WZ.step === WZ_STEPS.length - 1;
  const ok = (t) => `<p class="note good">✓ ${t}</p>`, bad = (t) => `<p class="note bad">${t}</p>`;
  let h = "";
  switch (WZ.step) {
    case 0:
      h = `<h3 style="margin-top:0">Trước khi bắt đầu</h3>
        <ul class="steps"><li><b>Tháo hết cánh quạt.</b> Bước motor sẽ quay motor thật.</li><li>Cắm pin nếu muốn thử ESC/motor và tay điều khiển (USB thường chỉ nuôi FC).</li><li>Đặt drone trên mặt bàn phẳng.</li></ul>
        <dl class="kv"><dt>Board</dt><dd>${esc((S.lastHello && S.lastHello.board) || "—")}</dd><dt>IMU</dt><dd>${esc(sn.imu || "không thấy")}</dd>
        <dt>Baro</dt><dd>${esc(sn.baro || "không có")}</dd><dt>Compass</dt><dd>${esc(sn.mag || "không có")}</dd><dt>GPS</dt><dd>${sn.gps ? "có" : "không có"}</dd></dl>`;
      break;
    case 1: {
      const r = WZ.caps.level && WZ.caps.nose ? solveAlign(WZ.caps.level, WZ.caps.nose, val("imu_align")) : null;
      h = `<h3 style="margin-top:0">Hướng lắp FC (imu_align)</h3>
        <p class="muted">Ghi hai tư thế: đặt drone phẳng, rồi dựng drone chúc mũi thẳng lên trời (đuôi chạm bàn). Trang tự tìm hướng lắp khớp với cả hai.</p>
        <div class="row"><button class="btn" id="wzCapL">${WZ.caps.level ? "✓ " : ""}Ghi: đặt phẳng</button><button class="btn" id="wzCapN">${WZ.caps.nose ? "✓ " : ""}Ghi: mũi chúc lên</button></div>
        <p class="num muted" id="wzAcc"></p>
        ${r ? (r.quality < 0.8 ? bad(`Hai lần ghi không rõ ràng (độ khớp ${(r.quality * 100).toFixed(0)}%). Giữ drone yên và đúng tư thế rồi ghi lại.`)
          : r.align === val("imu_align") ? ok(`Hướng lắp hiện tại đúng: ${ALIGN_NAME(r.align)}.`)
          : `<p class="note">Hướng lắp tìm được: <b>${ALIGN_NAME(r.align)}</b> (đang đặt ${ALIGN_NAME(val("imu_align"))}).</p><button class="btn primary" id="wzUseAlign">Dùng hướng này</button>`) : ""}`;
      if (r && r.quality >= 0.8 && r.align === val("imu_align")) WZ.done[1] = true;
      break;
    }
    case 2:
      h = `<h3 style="margin-top:0">Hiệu chuẩn accelerometer</h3>
        <p class="muted">Đặt drone phẳng trên bàn, không chạm vào trong 2 giây.</p>
        <button class="btn primary" id="wzAccCal">Hiệu chuẩn accel</button><p class="num muted" id="wzAccLive"></p>${WZ.done[2] ? ok("Đã hiệu chuẩn.") : ""}`;
      break;
    case 3:
      h = sn.mag ? `<h3 style="margin-top:0">Compass</h3>
        <p class="muted">Bấm hiệu chuẩn rồi xoay drone theo mọi hướng (lật, nghiêng, quay) trong 30 giây. Sau đó quay mũi về phía Bắc và so với hướng hiển thị.</p>
        <button class="btn primary" id="wzMagCal">Hiệu chuẩn compass</button><p class="num" id="wzHead"></p>
        <label class="check"><input type="checkbox" id="wzMagOk"${WZ.done[3] ? " checked" : ""}> Hướng hiển thị đúng khi xoay drone</label>`
        : `<h3 style="margin-top:0">Compass</h3><p class="muted">Không có compass. Hướng sẽ được học từ GPS khi bay vài đường thẳng (gps_heading = 1)${has("gps_heading") ? `, hiện ${val("gps_heading") ? "đang bật" : "đang tắt"}` : ""}.</p>`;
      break;
    case 4:
      h = sn.baro ? `<h3 style="margin-top:0">Barometer</h3><p class="muted">Nhấc drone lên cao khoảng 1 m rồi đặt xuống. Độ cao baro phải tăng theo.</p>
        <p class="num" id="wzBaro">—</p>${WZ.done[4] ? ok("Baro phản ứng đúng.") : ""}`
        : `<h3 style="margin-top:0">Barometer</h3><p class="muted">Không có baro: ALTHOLD dùng GPS (kém chính xác hơn), hoặc cảm biến khoảng cách nếu có.</p>`;
      break;
    case 5:
      h = `<h3 style="margin-top:0">GPS</h3>${sn.gps ? '<p class="muted">Mang drone ra chỗ thoáng. Lần đầu có thể mất vài phút để fix.</p><p class="num" id="wzGps">—</p>' : '<p class="muted">Không thấy GPS. Gán chức năng GPS cho một cổng serial ở tab Cổng & VTX nếu đã lắp.</p>'}`;
      break;
    case 6: {
      const hasArm = Array.from({ length: 12 }, (_, i) => val(`mr${i + 1}_mode`)).includes(1);
      h = `<h3 style="margin-top:0">Tay điều khiển</h3><p class="muted">Bật tay điều khiển và làm lần lượt (mode 2):</p>
        <ul class="wzchk" id="wzRx"></ul>
        ${hasArm ? ok("Đã gán công tắc ARM.") : bad('Chưa gán công tắc ARM: thêm ở tab Chế độ.')}`;
      break;
    }
    case 7:
      h = `<h3 style="margin-top:0">Motor: vị trí và chiều quay</h3>
        <label class="check"><input type="checkbox" id="wzProps"> Đã tháo cánh quạt, pin đã cắm</label>
        <p class="muted">Bấm từng motor: nó quay chậm 2 giây. Kiểm tra đúng vị trí và chiều quay (nhìn từ trên xuống, ${val("yaw_reversed") ? "cánh quạt ra ngoài" : "cánh quạt vào trong"}).</p>
        <div class="wzmot">${[0, 1, 2, 3].map((i) => {
          const [x, y] = MOTOR_XY[i], cw = MOTOR_CW[i] !== !!val("yaw_reversed");
          return `<div><button class="btn" data-m="${i}" disabled>Motor ${i + 1}</button><span class="muted">${y > 0 ? "trước" : "sau"} ${x > 0 ? "phải" : "trái"}, ${cw ? "thuận chiều kim đồng hồ" : "ngược chiều kim đồng hồ"}</span>
            <label class="check"><input type="checkbox" data-mok="${i}"${WZ.caps["m" + i] ? " checked" : ""}> đúng</label></div>`;
        }).join("")}</div>
        <p class="muted">Sai vị trí: đổi dây/cổng motor hoặc remap trong CLI. Sai chiều: đảo 2 dây motor, hoặc dùng ESC configurator (BLHeli / Bluejay) ở tab Motor.</p>`;
      break;
    default: {
      const items = [[1, "Hướng FC"], [2, "Accel"], [3, "Compass"], [4, "Baro"], [5, "GPS"], [6, "Tay điều khiển"], [7, "Motor"]];
      const k = Object.keys(S.edits).length;
      h = `<h3 style="margin-top:0">Tóm tắt</h3><ul class="wzchk">${items.map(([i, n]) => `<li class="${WZ.done[i] ? "ok" : ""}">${WZ.done[i] ? "✓" : "○"} ${n}</li>`).join("")}</ul>
        <p class="muted">${k ? `${k} thay đổi chưa áp dụng.` : "Không có thay đổi chờ."} Lưu vào flash để giữ sau khi tắt; hướng lắp FC cần khởi động lại.</p>
        <button class="btn primary" id="wzSave">Áp dụng và lưu vào flash</button>`;
    }
  }
  body.innerHTML = h;
  wzBind();
  wzSensStart();
  if (S.status) wzLive(S.status);
}
async function wzCapture(key) {
  const n = 10, sum = [0, 0, 0];
  for (let i = 0; i < n; i++) {
    const j = await wzSensors();
    for (let a = 0; a < 3; a++) sum[a] += j.acc[a] / n;
    await sleep(40);
  }
  WZ.caps[key] = sum;
  wzRender();
}
function wzBind() {
  const on = (id, f) => { const e = $(id); if (e) e.onclick = f; };
  on("wzCapL", () => wzCapture("level").catch(() => toast("Không đọc được cảm biến.")));
  on("wzCapN", () => wzCapture("nose").catch(() => toast("Không đọc được cảm biến.")));
  on("wzUseAlign", () => {
    const r = solveAlign(WZ.caps.level, WZ.caps.nose, val("imu_align"));
    setEdit("imu_align", r.align);
    WZ.done[1] = true;
    WZ.caps = {};
    toast("Đã đưa imu_align vào thay đổi chờ. Áp dụng, lưu rồi khởi động lại; sau đó hiệu chuẩn accel.");
    wzRender();
  });
  const cmd = async (c, k, msg) => {
    try { await api("/api/cmd?c=" + c, { method: "POST" }); toast(msg); wzMark(k, true); wzRender(); }
    catch (e) { toast(e.code === 423 ? "Hãy disarm trước." : "Lỗi: " + e.message); }
  };
  on("wzAccCal", () => cmd("calib_acc", 2, "Đang hiệu chuẩn accel, giữ drone phẳng."));
  on("wzMagCal", () => cmd("calib_mag", -1, "Xoay drone theo mọi hướng trong 30 giây."));
  const mo = $("wzMagOk");
  if (mo) mo.onchange = () => { WZ.done[3] = mo.checked; wzStepList(); };
  const pr = $("wzProps");
  if (pr) pr.onchange = () => { document.querySelectorAll("#wzBody [data-m]").forEach((b) => { b.disabled = !pr.checked; }); if (!pr.checked) wzMotorStop(); };
  document.querySelectorAll("#wzBody [data-m]").forEach((b) => b.onclick = () => wzMotorSpin(+b.dataset.m));
  document.querySelectorAll("#wzBody [data-mok]").forEach((c) => c.onchange = () => {
    WZ.caps["m" + c.dataset.mok] = c.checked;
    WZ.done[7] = [0, 1, 2, 3].every((i) => WZ.caps["m" + i]);
    wzStepList();
  });
  on("wzSave", async () => {
    if (Object.keys(S.edits).length) await applyEdits();
    try { await api("/api/save", { method: "POST" }); toast("Đã lưu vào flash."); wzMark(8, true); }
    catch (e) { toast(e.code === 423 ? "Hãy disarm trước khi lưu." : "Không lưu được: " + e.message); }
  });
}
async function wzMotorSpin(i) {
  wzMotorStop();
  WZ.motor = i;
  const v = [0, 0, 0, 0];
  v[i] = 0.06;
  const t0 = Date.now();
  const send = async () => {
    if (WZ.motor !== i) return;
    if (Date.now() - t0 > 2000) return wzMotorStop();
    try { await api(`/api/motor?m1=${v[0]}&m2=${v[1]}&m3=${v[2]}&m4=${v[3]}`, { method: "POST", timeout: 900 }); }
    catch (e) { if (e.code === 423) { toast("Không thử motor khi đang arm."); return wzMotorStop(); } }
    WZ.motorTimer = setTimeout(send, 300);
  };
  send();
}
function wzMotorStop() {
  clearTimeout(WZ.motorTimer);
  if (WZ.motor >= 0) { WZ.motor = -1; api("/api/motor?m1=0&m2=0&m3=0&m4=0", { method: "POST", timeout: 900 }).catch(() => {}); }
}
// steps that read the sensors directly (orientation, accel, baro): 5 Hz while the step is open
function wzSensStart() {
  clearTimeout(WZ.sensTimer);
  if (![1, 2, 4].includes(WZ.step)) return;
  const tick = async () => {
    if (!shown("setup") || ![1, 2, 4].includes(WZ.step)) return;
    try {
      const j = await wzSensors();
      WZ.last = j;
      const a = j.acc.map((x) => x.toFixed(2)).join(", ");
      if ($("wzAcc")) $("wzAcc").textContent = `accel ${a} m/s²`;
      if ($("wzAccLive")) $("wzAccLive").textContent = `accel ${a} m/s² (phẳng: gần 0, 0, −9.8)`;
      if (WZ.step === 4 && j.baro && j.baro.ok) {
        if (WZ.baro0 === null) WZ.baro0 = j.baro.alt;
        const d = j.baro.alt - WZ.baro0;
        WZ.baroMax = Math.max(WZ.baroMax, d);
        if ($("wzBaro")) $("wzBaro").textContent = `độ cao baro ${d >= 0 ? "+" : ""}${d.toFixed(2)} m, cao nhất +${WZ.baroMax.toFixed(2)} m`;
        if (WZ.baroMax > 0.5 && !WZ.done[4]) { wzMark(4, true); wzRender(); return; }
      }
    } catch (e) {}
    WZ.sensTimer = setTimeout(tick, 200);
  };
  tick();
}
// steps that follow the status stream (compass heading, GPS, receiver)
function wzLive(st) {
  if (WZ.step === 3 && $("wzHead")) {
    const hs = st.est ? ["chưa có hướng", "compass", "GPS", "EKF"][st.est.hsrc] : "";
    $("wzHead").textContent = `Hướng mũi: ${((st.att[2] % 360) + 360) % 360 | 0}° (${compassName(st.att[2])})${hs ? " · nguồn: " + hs : ""}`;
  }
  if (WZ.step === 5 && $("wzGps")) {
    const ok = st.gps && st.gps.fix && st.gps.sats >= (val("gps_min_sats") || 6);
    $("wzGps").textContent = st.gps ? `${st.gps.fix ? "3D fix" : "chưa fix"}, ${st.gps.sats} vệ tinh, sai số ${st.gps.hacc} m` : "—";
    if (ok && !WZ.done[5]) wzMark(5, true);
  }
  if (WZ.step === 6 && $("wzRx")) {
    const rc = st.rc || [];
    const checks = [
      ["rxok", "Tay điều khiển đã kết nối", st.rxok],
      ["thr", "Đẩy cần ga lên cao nhất: kênh 3 lên gần 2000", rc[2] > 1850],
      ["roll", "Gạt cần roll sang phải: kênh 1 tăng", rc[0] > 1850],
      ["pitch", "Đẩy cần pitch lên (về phía trước): kênh 2 tăng", rc[1] > 1850],
      ["yaw", "Gạt cần yaw sang phải: kênh 4 tăng", rc[3] > 1850],
      ["low", "Ga về thấp nhất: dưới 1050", WZ.rx.thr && rc[2] < 1050],
    ];
    checks.forEach(([k, , v]) => { if (v) WZ.rx[k] = true; });
    setHtml($("wzRx"), checks.map(([k, t]) => `<li class="${WZ.rx[k] ? "ok" : ""}">${WZ.rx[k] ? "✓" : "○"} ${t}</li>`).join("") +
      `<li class="muted num">CH1–4: ${[0, 1, 2, 3].map((i) => rc[i] || "—").join(" / ")}</li>`);
    const all = checks.every(([k]) => WZ.rx[k]);
    if (all && !WZ.done[6]) wzMark(6, true);
  }
}

// ---------------------------------------------------------------- demo: firmware in WebAssembly
// The simulator build (tools/build_wasm.sh) runs in a Web Worker; the page talks to its USB console
// with the same binary link protocol as to a real board, so every tab works on the virtual drone.
const DEMO_WORKER = `
let ex = null, mem = null, pending = [], t0 = 0;
const u8 = () => new Uint8Array(mem.buffer), dv = () => new DataView(mem.buffer);
const wasi = {
  fd_write(fd, iovs, n, out) {
    let total = 0, s = "";
    for (let i = 0; i < n; i++) {
      const p = dv().getUint32(iovs + i * 8, true), l = dv().getUint32(iovs + i * 8 + 4, true);
      s += new TextDecoder().decode(u8().slice(p, p + l)); total += l;
    }
    if (s.trim()) console.log("[demo]", s.trimEnd());
    dv().setUint32(out, total, true);
    return 0;
  },
  clock_time_get(id, prec, out) { dv().setBigUint64(out, BigInt(Math.round((performance.now() - t0) * 1e6)), true); return 0; },
  environ_get() { return 0 }, environ_sizes_get(c, s) { dv().setUint32(c, 0, true); dv().setUint32(s, 0, true); return 0; },
  args_get() { return 0 }, args_sizes_get(c, s) { dv().setUint32(c, 0, true); dv().setUint32(s, 0, true); return 0; },
  fd_close() { return 0 }, fd_seek() { return 70 }, fd_read() { return 8 }, fd_fdstat_get() { return 8 },
  fd_prestat_get() { return 8 }, fd_prestat_dir_name() { return 8 },
  random_get(p, l) { crypto.getRandomValues(u8().subarray(p, p + l)); return 0; },
  sched_yield() { return 0 }, poll_oneoff() { return 52 },
  proc_exit(c) { throw new Error("exit " + c); },
};
const imports = new Proxy({ wasi_snapshot_preview1: new Proxy(wasi, { get: (o, k) => o[k] || (() => 52) }) }, { get: (o, k) => o[k] || {} });
onmessage = async (e) => {
  if (e.data.wasm) {
    try {
      const { instance } = await WebAssembly.instantiate(e.data.wasm, imports);
      ex = instance.exports; mem = ex.memory;
      if (ex._initialize) ex._initialize();
      t0 = performance.now();
      ex.aq_init();
      postMessage({ ready: true });
      setInterval(tick, 4);
    } catch (err) { postMessage({ error: String(err) }); }
    return;
  }
  if (e.data.input) pending.push(e.data.input);
};
function tick() {
  try {
    while (pending.length) {
      let b = pending.shift();
      if (b.length > 8192) { pending.unshift(b.subarray(8192)); b = b.subarray(0, 8192); }
      u8().set(b, ex.aq_in_buf());
      ex.aq_input(b.length);
    }
    const n = ex.aq_step((performance.now() - t0) * 1000);
    if (n > 0) { const p = ex.aq_out_buf(); postMessage({ out: u8().slice(p, p + n) }); }
  } catch (err) { postMessage({ error: String(err) }); }
}`;
function demoBytes() {
  const s = atob(window.AQ_DEMO_WASM);
  const b = new Uint8Array(s.length);
  for (let i = 0; i < s.length; i++) b[i] = s.charCodeAt(i);
  return b.buffer;
}
async function demoStart() {
  connBusy("Đang khởi động drone ảo…");
  let worker;
  try { worker = new Worker(URL.createObjectURL(new Blob([DEMO_WORKER], { type: "text/javascript" }))); }
  catch (e) { connBusy(""); return showConnect("Trình duyệt chặn Web Worker: không chạy được demo.", true); }
  LINK.worker = worker;
  worker.onmessage = async (ev) => {
    const m = ev.data;
    if (m.out) { linkFeed(m.out); return; }
    if (m.error) { if (LINK.worker === worker) { worker.terminate(); LINK.worker = null; connBusy(""); linkLost("Drone ảo dừng: " + m.error); } return; }
    if (m.ready) {
      LINK.kind = "demo"; LINK.baud = 2000000; LINK.rx = new Uint8Array(0); LINK.closing = false; LINK.open = true;
      LINK.send = async (f) => worker.postMessage({ input: f });
      try { const h = await hello(8); connBusy(""); await connected(h, "Demo"); toast("Drone ảo đang chạy trong trình duyệt: mọi tab đều dùng được, không có phần cứng nào."); }
      catch (e) { connBusy(""); linkLost("Drone ảo không trả lời."); }
    }
  };
  worker.postMessage({ wasm: demoBytes() });
}
function demoInit() {
  const b = $("demoBtn");
  if (!b) return;
  b.hidden = !window.AQ_DEMO_WASM || typeof WebAssembly !== "object";
  b.onclick = demoStart;
}

// ---------------------------------------------------------------- OpenStreetMap tiles
// Web Mercator around the home position; drawn under the grid / mission / trail of the nav map.
const TILES = { on: lsGet("aq-tiles", false), cache: new Map(), redraw: 0 };
function tileOrigin() {
  const m = S.mission;
  if (m && m.home && (m.lat || m.lon)) return [m.lat, m.lon];
  if (S.wp && S.wp.ref) return [S.wp.ref[0], S.wp.ref[1]];
  return null;
}
function mapTiles(g, w, h, k, cn, ce) {
  if (!TILES.on) return;
  const o = tileOrigin();
  if (!o) { g.fillStyle = css("--ink-2"); g.font = `12px ${css("--num")}`; g.textAlign = "right"; g.fillText("Bản đồ nền: chưa có vị trí nhà", w - 8, 16); g.textAlign = "left"; return; }
  const [lat, lon] = o, c = Math.cos(lat * Math.PI / 180);
  const z = Math.max(1, Math.min(19, Math.round(Math.log2(156543.034 * c * k))));
  const mpp = 156543.034 * c / 2 ** z;               // metres per tile pixel at this zoom
  const n2 = 2 ** z * 256;
  const gx0 = (lon + 180) / 360 * n2;                // home in global tile pixels
  const sl = Math.sin(lat * Math.PI / 180);
  const gy0 = (0.5 - Math.log((1 + sl) / (1 - sl)) / (4 * Math.PI)) * n2;
  const cx = gx0 + ce / mpp, cy = gy0 - cn / mpp;    // canvas centre in global tile pixels
  const s = mpp * k;                                  // canvas px per tile px
  const tx0 = Math.floor((cx - w / 2 / s) / 256), tx1 = Math.floor((cx + w / 2 / s) / 256);
  const ty0 = Math.floor((cy - h / 2 / s) / 256), ty1 = Math.floor((cy + h / 2 / s) / 256);
  if ((tx1 - tx0 + 1) * (ty1 - ty0 + 1) > 64) return;
  g.save();
  g.globalAlpha = document.documentElement.dataset.theme === "dark" ? 0.55 : 0.8;
  for (let ty = ty0; ty <= ty1; ty++) for (let tx = tx0; tx <= tx1; tx++) {
    if (ty < 0 || ty >= 2 ** z) continue;
    const xx = ((tx % 2 ** z) + 2 ** z) % 2 ** z;
    const url = `https://tile.openstreetmap.org/${z}/${xx}/${ty}.png`;
    let img = TILES.cache.get(url);
    if (!img) {
      img = new Image();
      img.crossOrigin = "anonymous";
      img.onload = () => { cancelAnimationFrame(TILES.redraw); TILES.redraw = requestAnimationFrame(drawNav); };
      img.src = url;
      TILES.cache.set(url, img);
      if (TILES.cache.size > 300) TILES.cache.delete(TILES.cache.keys().next().value);
    }
    if (img.complete && img.naturalWidth)
      g.drawImage(img, w / 2 + (tx * 256 - cx) * s, h / 2 + (ty * 256 - cy) * s, 256 * s + 0.5, 256 * s + 0.5);
  }
  g.restore();
  g.font = `11px ${css("--num")}`;
  const t = "© OpenStreetMap contributors";
  const tw = g.measureText(t).width;
  g.fillStyle = css("--sheet"); g.fillRect(w - tw - 10, h - 18, tw + 10, 18);
  g.fillStyle = css("--ink-2"); g.textAlign = "left"; g.fillText(t, w - tw - 5, h - 5);
}
function tilesInit() {
  const row = $("mapReload") && $("mapReload").parentElement;
  if (!row || $("mapTiles")) return;
  const b = document.createElement("button");
  b.className = "btn ghost";
  b.id = "mapTiles";
  b.setAttribute("aria-pressed", String(TILES.on));
  b.textContent = "Bản đồ nền";
  b.title = "Ảnh bản đồ OpenStreetMap dưới bản đồ điều hướng (cần Internet)";
  b.onclick = () => { TILES.on = !TILES.on; lsSet("aq-tiles", TILES.on); b.setAttribute("aria-pressed", String(TILES.on)); drawNav(); };
  row.insertBefore(b, row.firstChild);
}

// ---------------------------------------------------------------- blackbox: compare + CSV
const BB = { ref: null };
function bbMetrics(res) {
  return res.axes.map((ax) => ({
    ov: ax.step ? ax.step.overshoot * 100 : null, rise: ax.step ? ax.step.riseMs : null,
    noise: ax.noiseFilt, raw: ax.noiseRaw, dn: ax.dNoise, trk: ax.trackRms,
  }));
}
function bbRefSeries() {
  if (!BB.ref) return [];
  return BB.ref.steps.map((s, k) => (s ? { x: s.t, y: s.y, color: AXCOL[k], width: 1.4, dash: [5, 4] } : null)).filter(Boolean);
}
function bbCompareRender() {
  const host = $("bbCmp");
  if (!host || !S.analysis) return;
  const cur = bbMetrics(S.analysis), ref = BB.ref;
  $("bbUnpin").hidden = !ref;
  if (!ref) { host.innerHTML = '<p class="muted">Ghim chuyến này làm mốc, đổi PID/bộ lọc, bay lại rồi mở log mới: đường đứt nét trên biểu đồ đáp ứng bước là chuyến mốc.</p>'; return; }
  const f = (v, d = 0) => (v === null || v === undefined || !isFinite(v) ? "—" : v.toFixed(d));
  const delta = (a, b, d, lowerBetter = true) => {
    if (a === null || b === null || !isFinite(a) || !isFinite(b)) return "";
    const x = a - b;
    if (Math.abs(x) < 10 ** -d / 2) return "";
    return ` <span class="${(x < 0) === lowerBetter ? "good" : "bad"}">${x > 0 ? "+" : ""}${x.toFixed(d)}</span>`;
  };
  const rows = [["Vọt lố (%)", "ov", 0], ["Lên 90% (ms)", "rise", 0], ["Nhiễu gyro sau lọc (°/s)", "noise", 2], ["Nhiễu gyro thô (°/s)", "raw", 1], ["Nhiễu D-term", "dn", 1], ["Sai lệch bám (°/s RMS)", "trk", 1]];
  host.innerHTML = `<p class="muted">Mốc: ${esc(ref.label)}. Số màu xanh là tốt hơn mốc.</p><table class="ports"><tr><th></th>${AXES.map((a) => `<th>${a}: mốc</th><th>${a}: chuyến này</th>`).join("")}</tr>` +
    rows.map(([t, k, d]) => `<tr><td>${t}</td>${[0, 1, 2].map((a) => `<td class="num">${f(ref.m[a][k], d)}</td><td class="num">${f(cur[a][k], d)}${delta(cur[a][k], ref.m[a][k], d)}</td>`).join("")}</tr>`).join("") + "</table>";
}
function bbPin() {
  if (!S.analysis || !S.flight) return;
  const fl = S.flight;
  BB.ref = {
    label: `${$("bbFlight").selectedOptions[0] ? $("bbFlight").selectedOptions[0].textContent : "chuyến bay"}, PID ${AXES.map((a) => (fl.pid[a] ? `${fl.pid[a].p}/${fl.pid[a].i}/${fl.pid[a].d}` : "?")).join(" · ")}`,
    steps: S.analysis.axes.map((ax) => (ax.step ? { t: Array.from(ax.step.t), y: Array.from(ax.step.y) } : null)),
    m: bbMetrics(S.analysis),
  };
  toast("Đã ghim làm mốc. Mở log khác để so sánh.");
  drawBlackbox();
}
function bbCsv() {
  const fl = S.flight;
  if (!fl) return;
  const cols = [["t_s", (k) => (fl.t[k] - fl.t[0]).toFixed(4)], ["throttle", (k) => fl.thr[k]]];
  AXES.forEach((a, i) => {
    cols.push([a + "_setpoint", (k) => fl.sp[i][k]], [a + "_gyro", (k) => fl.gyro[i][k]]);
    if (fl.gyroRaw) cols.push([a + "_gyro_raw", (k) => fl.gyroRaw[i][k]]);
    cols.push([a + "_P", (k) => fl.P[i][k]], [a + "_I", (k) => fl.I[i][k]], [a + "_D", (k) => fl.D[i][k]]);
  });
  for (let m = 0; m < 4; m++) cols.push([`motor${m + 1}`, (k) => fl.motor[m][k]]);
  const num = (v) => (typeof v === "number" ? +v.toFixed(4) : v);
  const lines = [cols.map((c) => c[0]).join(",")];
  for (let k = 0; k < fl.n; k++) lines.push(cols.map((c) => num(c[1](k))).join(","));
  saveText(`allquad-blackbox-${stamp()}.csv`, lines.join("\n") + "\n");
}
function bbInit() {
  if ($("bbCmp") || !$("stepLegend")) return;
  const p = document.createElement("div");
  p.className = "panel";
  p.innerHTML = `<div class="row" style="justify-content:space-between"><h3 style="margin:0">So sánh & xuất</h3>
    <span class="row"><button class="btn" id="bbPin">Ghim làm mốc</button><button class="btn" id="bbUnpin" hidden>Bỏ mốc</button><button class="btn" id="bbCsv">Xuất CSV</button></span></div>
    <div id="bbCmp" style="margin-top:10px"></div>`;
  const stepPanel = $("stepLegend").closest(".panel");
  stepPanel.parentElement.insertBefore(p, stepPanel.nextSibling);
  $("bbPin").onclick = bbPin;
  $("bbUnpin").onclick = () => { BB.ref = null; drawBlackbox(); };
  $("bbCsv").onclick = bbCsv;
}

// ---------------------------------------------------------------- parameter history
// every applied change (old -> new) in this browser, newest first; restore / defaults are recorded
// by comparing the parameters before and after the reload
const HIST_MAX = 200;
function histList() { return lsGet("aq-history", []); }
function histAdd(tag, changes) {
  if (!changes.length) return;
  const l = histList();
  l.unshift({ t: Date.now(), tag, board: (S.lastHello && S.lastHello.board) || "", c: changes });
  lsSet("aq-history", l.slice(0, HIST_MAX));
  histRender();
}
function extrasApplied(changes) { histAdd(S.histTag || "Áp dụng", changes); S.histTag = ""; }
function histBeforeLoad(tag) {
  S.histPrev = {};
  for (const n of S.order) S.histPrev[n] = S.params[n].v;
  S.histTag = tag;
}
function histAfterLoad() {
  if (!S.histPrev) return;
  const ch = [];
  for (const n of S.order) if (n in S.histPrev && Math.abs(S.histPrev[n] - S.params[n].v) > 1e-9) ch.push([n, S.histPrev[n], S.params[n].v]);
  const tag = S.histTag;
  S.histPrev = null; S.histTag = "";
  histAdd(tag, ch);
}
function histRender() {
  const host = $("histList");
  if (!host) return;
  const l = histList();
  if (!l.length) { host.innerHTML = '<p class="muted">Chưa có thay đổi nào được ghi trong trình duyệt này.</p>'; return; }
  host.innerHTML = l.slice(0, 40).map((h, i) => {
    const d = new Date(h.t);
    const shown = h.c.slice(0, 6).map(([n, a, b]) => `${esc(n)} ${fmt(n, a)} ▸ ${fmt(n, b)}`).join(", ") + (h.c.length > 6 ? `, … (+${h.c.length - 6})` : "");
    return `<div class="sug"><div class="bar all"></div><div><h4>${esc(h.tag)} <span class="muted num">${d.toLocaleString()}</span></h4><div class="change">${shown}</div></div><button class="btn" data-h="${i}">Hoàn tác</button></div>`;
  }).join("");
  host.querySelectorAll("[data-h]").forEach((b) => b.onclick = () => {
    const h = histList()[+b.dataset.h];
    let n = 0;
    for (const [k, old] of h.c) if (S.params[k]) { setEdit(k, old); n++; }
    S.histTag = "Hoàn tác: " + h.tag;
    toast(n ? `Đã đưa ${n} giá trị cũ vào thay đổi chờ. Bấm Áp dụng.` : "Không còn tham số nào khớp.");
  });
}
function histCsv() {
  const lines = ["time,tag,board,param,old,new"];
  for (const h of histList()) for (const [n, a, b] of h.c) lines.push([new Date(h.t).toISOString(), `"${h.tag.replace(/"/g, "'")}"`, `"${h.board}"`, n, a, b].join(","));
  saveText(`allquad-param-history-${stamp()}.csv`, lines.join("\n") + "\n");
}
function histInit() {
  if ($("histList") || !$("paramGroups")) return;
  const p = document.createElement("div");
  p.className = "panel";
  p.innerHTML = `<div class="row" style="justify-content:space-between"><h3 style="margin:0">Lịch sử thay đổi</h3>
    <span class="row"><button class="btn" id="histCsv">Xuất CSV</button><button class="btn ghost" id="histClear">Xoá lịch sử</button></span></div>
    <p class="muted">Lưu trong trình duyệt này: mỗi lần Áp dụng, Khôi phục file hay Nạp mặc định. Hoàn tác đưa giá trị cũ vào thay đổi chờ.</p><div id="histList"></div>`;
  $("paramGroups").parentElement.insertBefore(p, $("paramGroups"));
  $("histCsv").onclick = histCsv;
  $("histClear").onclick = () => { if (confirm("Xoá toàn bộ lịch sử thay đổi trong trình duyệt này?")) { lsSet("aq-history", []); histRender(); } };
}

// ---------------------------------------------------------------- EKF panel (navigation tab)
const EKF_FAULTS = ["GPS nhiễu (vị trí bị loại)", "Tốc độ GPS bị loại", "Baro lỗi: độ cao từ GPS", "Compass sai: hướng từ GPS", "Không có GPS/flow: ước lượng quán tính", "Đã reset toàn bộ", "Optical flow bị loại", "Hướng bị loại"];
function ekfPanelInit() {
  if ($("ekfPanel") || !$("navParams")) return;
  const p = document.createElement("div");
  p.className = "panel";
  p.id = "ekfPanel";
  p.innerHTML = `<h3 style="margin-top:0">EKF (bộ lọc Kalman mở rộng 15 trạng thái)</h3><div id="ekfBody" class="muted">Chưa có dữ liệu.</div>`;
  $("navParams").parentElement.insertBefore(p, $("navParams"));
}
function ekfRender(st) {
  const host = $("ekfBody");
  if (!host) return;
  const k = st.ekf;
  if (!k || !k.on) { setHtml(host, `<p class="muted">EKF chưa chạy${has("est_type") && !val("est_type") ? " (est_type = 0: dùng bộ ước lượng cũ)" : " (chờ IMU ổn định và tư thế hợp lệ)"}.</p>`); return; }
  const names = ["Vị trí GPS", "Tốc độ GPS", "Độ cao", "Hướng", "Compass", "Optical flow"];
  const bars = k.tr.map((v, i) => {
    const pct = Math.min(100, v * 50);
    return `<div class="rc"><span style="min-width:96px">${names[i]}</span><div class="track"><span style="left:0;width:${pct}%;margin-left:0;border-radius:4px;background:var(${v > 1 ? "--danger" : v > 0.5 ? "--amber" : "--ok"})"></span></div><span class="num">${v.toFixed(2)}</span></div>`;
  }).join("");
  const faults = EKF_FAULTS.filter((_, i) => k.f & (1 << i)).map((t) => `<span class="chip${/Không có|reset/.test(t) ? "" : " bad"}">${t}</span>`).join("");
  setHtml(host, `<p class="muted" style="margin-top:0">Tỉ số kiểm định: dưới 1 là phép đo được dùng, trên 1 là bị loại (vạch ở giữa = 1).</p>${bars}
    <div class="chips" style="margin:8px 0">${faults || '<span class="chip good">Mọi cảm biến nhất quán</span>'}</div>
    <dl class="kv"><dt>Độ bất định</dt><dd class="num">vị trí ±${k.sd[0].toFixed(2)} m, tốc độ ±${k.sd[1].toFixed(2)} m/s, độ cao ±${k.sd[2].toFixed(2)} m, hướng ±${k.sd[3].toFixed(1)}°</dd>
    <dt>Nguồn độ cao</dt><dd>${["baro", "GPS", "không có"][k.hs] || "?"}</dd>
    <dt>Reset</dt><dd class="num">vị trí ${k.rs[0]}, tốc độ ${k.rs[1]}, toàn bộ ${k.rs[2]}</dd>
    <dt>Bias gyro</dt><dd class="num">${k.gb.map((x) => (x * 57.2958).toFixed(2)).join(", ")} °/s</dd>
    <dt>Bias accel</dt><dd class="num">${k.ab.map((x) => x.toFixed(3)).join(", ")} m/s²</dd></dl>`);
}

// ---------------------------------------------------------------- sensors: calibration + mounting
const ALIGNS = ["0° (mặc định)", "90° theo chiều kim đồng hồ", "180°", "270° theo chiều kim đồng hồ",
  "0°, lật ngửa", "90°, lật ngửa", "180°, lật ngửa", "270°, lật ngửa"];
const CAL = { magUntil: 0, magTimer: null };
function calBuild() {
  const host = $("calPanel");
  if (!host || !S.params.imu_align) return;
  const sel = (n) => `<select data-p="${n}" class="btn" style="width:100%">${ALIGNS.map((t, i) => `<option value="${i}">${i}: ${t}</option>`).join("")}</select>`;
  host.innerHTML = `<h3 style="margin-top:0">Hiệu chuẩn & hướng lắp</h3>
    <div class="grid2" style="gap:16px">
      <div>
        <h4 style="margin:0 0 6px">Hiệu chuẩn</h4>
        <div class="calrow"><button class="btn" data-cal="calib_gyro">Gyro</button><span class="muted">để drone đứng yên 2 giây (tự làm mỗi lần cấp nguồn)</span></div>
        <div class="calrow"><button class="btn" data-cal="calib_acc">Accel</button><span class="muted" id="calAccState"></span></div>
        <div class="calrow"><button class="btn" data-cal="calib_acc6">Accel 6 mặt</button><span class="muted" id="calAcc6State">chính xác hơn: đặt lần lượt 6 mặt, mỗi mặt giữ yên ~1 s</span></div>
        <div class="calrow"><button class="btn" data-cal="calib_mag">Compass</button><span class="muted" id="calMagState"></span></div>
        <div class="calrow"><button class="btn danger" data-cal="compassmot">Compassmot</button><span class="muted" id="calCmotState">nhiễu compass theo dòng: LẮP CÁNH, giữ chặt drone, motor lên 60 % trong 20 s</span></div>
        <p class="note" id="calMsg" hidden></p>
        <h4 style="margin:14px 0 6px">Cân bằng (ANGLE / HORIZON)</h4>
        <p class="muted" style="margin:0 0 6px">Drone bay trôi về một phía khi buông cần: chỉnh từng 0,1–0,5°.</p>
        <div class="row" id="calTrim"></div>
      </div>
      <div>
        <h4 style="margin:0 0 6px">Hướng lắp FC (IMU)</h4>
        ${sel("imu_align")}
        <p class="muted" style="margin:6px 0">Mũi tên / chấm chân 1 trên chip, nhìn từ trên xuống, so với mũi drone. Không chắc thì dùng <a href="#" id="calWizard">trình thiết lập</a>: nó tự tìm từ hai tư thế.</p>
        <h4 style="margin:10px 0 6px">Chỉnh tinh (FC lắp nghiêng, lệch)</h4>
        <div class="row" id="calBoard"></div>
        <h4 style="margin:14px 0 6px">Hướng lắp compass</h4>
        ${sel("mag_align")}
        <p class="muted" style="margin:6px 0 0">Compass trên module GPS: mũi tên trên module hướng về mũi drone là 0°. Sau khi đổi: Áp dụng, rồi quay drone 360° và xem hướng trên tab Điều hướng có đổi đúng chiều không.</p>
      </div>
    </div>
    <p class="muted" style="margin-bottom:0">Đổi hướng lắp có tác dụng ngay sau khi bấm Áp dụng: nghiêng drone và xem mô hình 3D. Nhớ Lưu vào flash, rồi hiệu chuẩn lại accel.</p>`;
  host.querySelectorAll("select[data-p]").forEach((el) => { el.value = String(val(el.dataset.p)); el.onchange = () => setEdit(el.dataset.p, el.value); });
  [["level_trim_roll", 0.1, "Roll °"], ["level_trim_pitch", 0.1, "Pitch °"]].forEach(([n, st, l]) => has(n) && $("calTrim").appendChild(stepper(n, st, l)));
  [["board_roll", 1, "Roll °"], ["board_pitch", 1, "Pitch °"], ["board_yaw", 1, "Yaw °"]].forEach(([n, st, l]) => has(n) && $("calBoard").appendChild(stepper(n, st, l)));
  host.querySelectorAll("[data-cal]").forEach((b) => b.onclick = () => calRun(b.dataset.cal));
  $("calWizard").onclick = (e) => {
    e.preventDefault();
    const t = document.querySelector("nav.tabs button[data-tab=setup]");
    if (t) { t.click(); wzGo(1); }
  };
  calSync();
}
function calSync() {
  const host = $("calPanel");
  if (!host) return;
  host.querySelectorAll("select[data-p]").forEach((el) => { if (document.activeElement !== el) el.value = String(val(el.dataset.p)); });
  if ($("calAccState")) $("calAccState").textContent = has("acc_calibrated") && val("acc_calibrated") ? "đã hiệu chuẩn · đặt phẳng, đứng yên khi làm lại" : "CHƯA hiệu chuẩn: đặt drone phẳng, đứng yên rồi bấm";
  const magCal = ["mag_bias_x", "mag_bias_y", "mag_bias_z"].some((n) => has(n) && val(n) !== 0);
  if ($("calMagState") && !CAL.magTimer) $("calMagState").textContent = magCal ? "đã hiệu chuẩn · xoay mọi hướng 30 s khi làm lại" : "CHƯA hiệu chuẩn: bấm rồi xoay drone theo mọi hướng trong 30 giây";
}
const ACC6_SIDES = ["phẳng", "lật ngửa", "chúi mũi xuống", "ngóc mũi lên", "nghiêng trái xuống", "nghiêng phải xuống"];
function calLive(st) {
  if (!$("calPanel")) return;
  const s6 = $("calAcc6State");
  if (s6 && st.a6 !== undefined) {
    if (st.a6 >= 0) {
      CAL.a6On = true;
      const left = ACC6_SIDES.filter((_, i) => !((st.a6 >> i) & 1));
      s6.textContent = `đã xong ${6 - left.length}/6 · còn: ${left.join(", ")}`;
    } else if (CAL.a6On) {
      CAL.a6On = false;
      s6.textContent = "xong 6 mặt · bấm Lưu vào flash";
      loadParams();
    }
  }
  const cm = $("calCmotState");
  if (cm && st.cm !== undefined) {
    if (st.cm >= 0) { CAL.cmOn = true; cm.textContent = `đang chạy… ${st.cm} % (rút pin để huỷ)`; }
    else if (CAL.cmOn) { CAL.cmOn = false; cm.textContent = "xong · xem thông báo, bấm Lưu vào flash"; loadParams(); }
  }
}
async function calRun(c) {
  const m = $("calMsg");
  if (c === "compassmot" && !confirm("Compassmot cho motor chạy tới 60 % trong 20 giây.\n\nPHẢI lắp cánh, giữ / buộc chặt drone, đứng xa cánh quạt. Cần cảm biến dòng hoặc ESC telemetry (AM32 EDT).\n\nBắt đầu?")) return;
  try {
    await api("/api/cmd?c=" + c, { method: "POST" });
    m.hidden = false;
    m.className = "note";
    m.textContent = c === "calib_acc6" ? "Đặt drone lần lượt lên 6 mặt, giữ yên mỗi mặt đến khi danh sách cập nhật." : c === "compassmot" ? "Compassmot đang chạy: tránh xa cánh quạt." : c === "calib_mag" ? "Xoay drone theo mọi hướng: lật ngửa, nghiêng trái/phải, chúi mũi, quay vòng…" : c === "calib_acc" ? "Đang hiệu chuẩn accel: giữ yên, phẳng." : "Đang hiệu chuẩn gyro: giữ yên.";
    if (c === "calib_mag") {
      CAL.magUntil = Date.now() + 31000;
      clearInterval(CAL.magTimer);
      CAL.magTimer = setInterval(() => {
        const left = Math.ceil((CAL.magUntil - Date.now()) / 1000);
        if ($("calMagState")) $("calMagState").textContent = left > 0 ? `đang hiệu chuẩn… còn ${left} s` : "xong";
        if (left <= 0) { clearInterval(CAL.magTimer); CAL.magTimer = null; m.textContent = "Compass đã hiệu chuẩn. Bấm Lưu vào flash để giữ."; setTimeout(loadParams, 1500); }
      }, 500);
    } else if (c === "calib_acc6" || c === "compassmot") {
      // progress comes from the status stream (calLive)
    } else setTimeout(() => { m.textContent = "Xong. Bấm Lưu vào flash để giữ."; loadParams(); }, 2500);
  } catch (e) {
    m.hidden = false; m.className = "note bad";
    m.textContent = e.code === 423 ? "Hãy disarm trước." : "Lỗi: " + e.message;
  }
}
function calInit() {
  const sec = $("tab-sensors");
  if (!sec || $("calPanel")) return;
  const p = document.createElement("div");
  p.className = "panel";
  p.id = "calPanel";
  const first = sec.querySelector(".grid2");
  sec.insertBefore(p, first ? first.nextSibling : null);
}

// ---------------------------------------------------------------- motors: order (remap outputs) + props direction
// mixer motors (Betaflight numbering): 1 rear-right, 2 front-right, 3 rear-left, 4 front-left
const MO_POS = [[4, "Trước trái"], [2, "Trước phải"], [3, "Sau trái"], [1, "Sau phải"]];
const MO = { step: -1, map: [0, 0, 0, 0], timer: null, until: 0 };
function moCurrent() { return [1, 2, 3, 4].map((i) => (has(`motor${i}_out`) ? S.params[`motor${i}_out`].v : i)); }
function moBuild() {
  const host = $("motOrderPanel");
  if (!host || !has("motor1_out")) return;
  if (MO.step >= 0 && $("moProps")) return moSync();  // parameters reloaded mid-search: keep the wizard
  host.innerHTML = `<h3 style="margin-top:0">Vị trí & chiều quay motor</h3>
    <div class="grid2" style="gap:16px">
      <div>
        <h4 style="margin:0 0 6px">Chiều cánh quạt</h4>
        <div class="seg" role="group" aria-label="Chiều cánh quạt" id="moProps"><button data-v="0">Cánh vào trong (Betaflight)</button><button data-v="1">Cánh ra ngoài (props-out)</button></div>
        <p class="muted">Đổi cả bộ: cả 4 motor quay ngược lại so với mặc định. Motor nào quay sai riêng lẻ thì đảo chiều ESC ở phần dưới (DShot) hoặc đổi 2 dây motor.</p>
        <h4 style="margin:12px 0 6px">Đầu ra cho từng vị trí</h4>
        <table class="ports motab"><tr><th>Vị trí</th><th>Motor (firmware)</th><th>Đầu ra</th></tr>
        ${MO_POS.map(([m, t]) => `<tr><td>${t}</td><td>M${m}</td><td><select data-p="motor${m}_out" class="btn">${[1, 2, 3, 4].map((o) => `<option value="${o}">Đầu ra ${o}</option>`).join("")}</select></td></tr>`).join("")}</table>
        <p class="muted" id="moWarn"></p>
      </div>
      <div>
        <h4 style="margin:0 0 6px">Tìm thứ tự tự động</h4>
        <p class="muted" style="margin-top:0">Trang quay lần lượt từng đầu ra (chậm, 2 giây). Bạn bấm vào vị trí của motor đang quay. Cần tích "Tôi đã tháo cánh quạt" ở trên và cắm pin.</p>
        <button class="btn primary" id="moStart">Bắt đầu tìm thứ tự</button>
        <div id="moStepBox" hidden style="margin-top:10px">
          <p class="note" id="moAsk"></p>
          <div class="mogrid">${MO_POS.map(([m, t]) => `<button class="btn" data-pos="${m}">${t}</button>`).join("")}</div>
          <div class="row" style="margin-top:8px"><button class="btn" id="moAgain">Quay lại đầu ra này</button><button class="btn danger" id="moCancel">Huỷ</button></div>
        </div>
      </div>
    </div>`;
  host.querySelectorAll("select[data-p]").forEach((el) => el.onchange = () => setEdit(el.dataset.p, el.value));
  $("moProps").querySelectorAll("button").forEach((b) => b.onclick = () => setEdit("yaw_reversed", +b.dataset.v));
  $("moStart").onclick = moStart;
  $("moAgain").onclick = () => moSpin();
  $("moCancel").onclick = () => { moStop(); $("moStepBox").hidden = true; };
  host.querySelectorAll("[data-pos]").forEach((b) => b.onclick = () => moPick(+b.dataset.pos));
  moSync();
}
function moSync() {
  const host = $("motOrderPanel");
  if (!host || !$("moProps")) return;
  host.querySelectorAll("select[data-p]").forEach((el) => { if (document.activeElement !== el) el.value = String(val(el.dataset.p)); });
  $("moProps").querySelectorAll("button").forEach((b) => b.setAttribute("aria-pressed", String(+b.dataset.v === (val("yaw_reversed") ? 1 : 0))));
  const outs = [1, 2, 3, 4].map((i) => val(`motor${i}_out`));
  const dup = new Set(outs).size !== 4;
  $("moWarn").textContent = dup ? "Hai vị trí đang dùng chung một đầu ra: firmware sẽ bỏ qua bảng này (dùng thứ tự mặc định) cho tới khi sửa." : "";
  $("moWarn").className = dup ? "note bad" : "muted";
}
function moStart() {
  const a = $("mtArm");
  if (!a || !a.checked) return toast("Tích \"Tôi đã tháo cánh quạt\" ở phần thử motor trước.");
  if (Object.keys(S.edits).some((k) => /^motor\d_out$/.test(k))) return toast("Áp dụng hoặc bỏ thay đổi thứ tự motor đang chờ trước.");
  MO.step = 0;
  MO.map = [0, 0, 0, 0];
  $("moStepBox").hidden = false;
  moSpin();
}
function moSpin() {
  moStop(true);
  const k = MO.step + 1;  // output to spin
  const cur = moCurrent();
  const m = cur.indexOf(k);  // mixer motor currently wired to output k
  if (m < 0) return;
  $("moAsk").textContent = `Đầu ra ${k}/4 đang quay. Motor nào quay? Bấm vào vị trí của nó.`;
  document.querySelectorAll("#moStepBox [data-pos]").forEach((b) => { b.disabled = MO.map[+b.dataset.pos - 1] > 0; });
  const v = [0, 0, 0, 0];
  v[m] = 0.06;
  MO.until = Date.now() + 2000;
  const send = async () => {
    if (MO.step < 0 || Date.now() > MO.until) return moStop(true);
    try { await api(`/api/motor?m1=${v[0]}&m2=${v[1]}&m3=${v[2]}&m4=${v[3]}`, { method: "POST", timeout: 900 }); }
    catch (e) { if (e.code === 423) { toast("Không thử motor khi đang arm."); return moStop(); } }
    MO.timer = setTimeout(send, 300);
  };
  send();
}
function moPick(pos) {
  if (MO.step < 0) return;
  MO.map[pos - 1] = MO.step + 1;  // mixer motor at this position <- output
  MO.step++;
  if (MO.step < 4) return moSpin();
  moStop();
  $("moStepBox").hidden = true;
  MO.map.forEach((o, i) => setEdit(`motor${i + 1}_out`, o));
  toast("Đã tìm xong thứ tự: bấm Áp dụng (và Lưu), rồi thử lại từng motor.");
}
function moStop(keepStep) {
  clearTimeout(MO.timer);
  MO.timer = null;
  if (!keepStep) MO.step = -1;
  api("/api/motor?m1=0&m2=0&m3=0&m4=0", { method: "POST", timeout: 900 }).catch(() => {});
}
function moInit() {
  const sec = $("tab-motors");
  if (!sec || $("motOrderPanel")) return;
  const p = document.createElement("div");
  p.className = "panel";
  p.id = "motOrderPanel";
  const g = sec.querySelector(".grid2");
  sec.insertBefore(p, g ? g.nextSibling : null);
}

// ---------------------------------------------------------------- motors: expected response (no arming needed)
// Same sign conventions as the firmware mixer (control/mixer.h) and ANGLE controller: shows which
// motors the flight controller WOULD speed up for the current tilt and sticks, without spinning anything.
const MIX_R = [-1, -1, 1, 1], MIX_P = [-1, 1, -1, 1], MIX_Y = [-1, 1, 1, -1];
function rxPreviewInit() {
  const sec = $("tab-motors");
  if (!sec || $("rxPrevPanel")) return;
  const p = document.createElement("div");
  p.className = "panel";
  p.id = "rxPrevPanel";
  p.innerHTML = `<h3 style="margin-top:0">Kiểm tra phản ứng (không cần arm, motor không quay)</h3>
    <p class="muted" style="margin-top:0">Nghiêng drone hoặc gạt cần: thanh cho biết FC <b>sẽ</b> tăng (cam) hay giảm (xám) motor nào, như ở chế độ ANGLE. Đúng là: nghiêng phải → hai motor <b>bên phải</b> tăng; chúc mũi xuống → hai motor <b>phía trước</b> tăng; cần roll phải → motor <b>bên trái</b> tăng; cần pitch tới → motor <b>phía sau</b> tăng.</p>
    <div class="grid2" style="gap:16px"><canvas id="rxPrevCv" style="width:100%;height:240px"></canvas><dl class="kv" id="rxPrevKv"></dl></div>`;
  const anchor = $("motOrderPanel") || sec.querySelector(".grid2");
  sec.insertBefore(p, anchor);
}
function rxPreview(st) {
  const cv = $("rxPrevCv");
  if (!cv || !st || !st.att || !st.rc) return;
  const lim = has("angle_limit") ? val("angle_limit") : 45;
  const stk = (us) => Math.max(-1, Math.min(1, ((us || 1500) - 1500) / 500));
  const rs = stk(st.rc[0]), ps = stk(st.rc[1]), ys = stk(st.rc[3]);
  const roll = Math.max(-1, Math.min(1, (rs * lim - st.att[0]) / 30));
  const pitch = Math.max(-1, Math.min(1, (-ps * lim - st.att[1]) / 30));
  const yaw = ys * (val("yaw_reversed") ? -1 : 1);
  const m = [0, 1, 2, 3].map((i) => MIX_R[i] * roll + MIX_P[i] * pitch + MIX_Y[i] * yaw * 0.5);
  rxPreview.last = m;
  const { g, w, h } = setupCanvas(cv);
  g.clearRect(0, 0, w, h);
  const cx = w / 2, cy = h / 2 + 4, d = Math.min(w, h) * 0.27;
  const pos = [[1, 1], [1, -1], [-1, 1], [-1, -1]];  // M1 rear-right, M2 front-right, M3 rear-left, M4 front-left ([x right, y down])
  g.strokeStyle = css("--ink-2"); g.lineWidth = 6;
  g.beginPath(); g.moveTo(cx - d, cy - d); g.lineTo(cx + d, cy + d); g.moveTo(cx + d, cy - d); g.lineTo(cx - d, cy + d); g.stroke();
  g.fillStyle = css("--amber"); g.beginPath(); g.moveTo(cx, cy - 18); g.lineTo(cx - 8, cy - 4); g.lineTo(cx + 8, cy - 4); g.fill();
  g.font = `12px ${css("--num")}`; g.textAlign = "center";
  const outs = [1, 2, 3, 4].map((i) => (has(`motor${i}_out`) ? val(`motor${i}_out`) : i));
  pos.forEach(([x, y], i) => {
    const px = cx + x * d, py = cy + y * d, v = m[i];
    g.fillStyle = css("--sheet"); g.strokeStyle = css("--ink"); g.lineWidth = 2;
    g.beginPath(); g.arc(px, py, 32, 0, Math.PI * 2); g.fill(); g.stroke();
    const bh = Math.min(1, Math.abs(v)) * 40;
    g.fillStyle = v > 0.05 ? css("--amber") : css("--ink-2");
    if (Math.abs(v) > 0.05) g.fillRect(px + 38, v > 0 ? py - bh : py, 10, bh);
    g.fillStyle = css("--ink");
    g.font = `700 13px ${css("--num")}`;
    g.fillText(`M${i + 1}`, px, py - 1);
    g.fillStyle = css("--ink-2");
    g.font = `10px ${css("--num")}`;
    g.fillText(`đầu ra ${outs[i]}`, px, py + 13);
    g.font = `12px ${css("--num")}`;
    g.fillText(v > 0.05 ? "tăng" : v < -0.05 ? "giảm" : "", px, py + 46);
  });
  g.fillStyle = css("--ink-2"); g.fillText("phía trước", cx, 12);
  const f = (x) => (x >= 0 ? "+" : "") + x.toFixed(0);
  setHtml($("rxPrevKv"), `<dt>Tư thế</dt><dd class="num">roll ${f(st.att[0])}° pitch ${f(st.att[1])}°</dd>
    <dt>Cần</dt><dd class="num">roll ${st.rc[0] || "—"} · pitch ${st.rc[1] || "—"} · ga ${st.rc[2] || "—"} · yaw ${st.rc[3] || "—"}</dd>
    <dt>Chế độ</dt><dd>${esc(st.mode || "")}${st.armed ? " (armed)" : ""}</dd>
    <dt>Lưu ý</dt><dd class="muted">Thử khi đã arm mà <b>không có cánh</b>: drone không bay lên được nên I-term tích luỹ, motor dần lệch hẳn về một phía, dù hướng ban đầu đúng. Đó là bình thường trên bàn.</dd>`);
}

// ---------------------------------------------------------------- motors: ESC calibration + DShot commands
const PROTO = ["PWM (50–490 Hz)", "OneShot125", "DShot300", "DShot600"];
const ESCC = { step: 0, timer: null, state: 0 };
function escBuild() {
  const host = $("escCalPanel");
  if (!host || !has("motor_protocol")) return;
  const proto = val("motor_protocol"), dshot = proto >= 2;
  escCalStop();
  ESCC.render = null;
  host.innerHTML = `<h3 style="margin-top:0">Hiệu chuẩn ESC & lệnh ESC</h3>
    <div class="fields"><div class="field"><label>Giao thức motor (cần lưu + khởi động lại)</label>
      <select data-p="motor_protocol" class="btn">${PROTO.map((t, i) => `<option value="${i}">${t}</option>`).join("")}</select></div></div>
    ${dshot ? `<p class="muted">DShot là tín hiệu số: <b>không cần hiệu chuẩn dải ga</b>. Các lệnh dưới đây gửi thẳng tới ESC (cắm pin, tháo cánh, tích "đã tháo cánh quạt" ở trên).</p>
      <h4 style="margin:10px 0 6px">Tìm motor (ESC kêu bíp)</h4>
      <div class="row">${[1, 2, 3, 4].map((m) => `<button class="btn" data-beep="${1 << (m - 1)}">Motor ${m}</button>`).join("")}<button class="btn" data-beep="15">Tất cả</button></div>
      <h4 style="margin:14px 0 6px">Chiều quay (lưu trong ESC)</h4>
      <p class="muted" style="margin:0 0 6px">Motor quay sai chiều trên sơ đồ: đảo chiều ở đây thay vì hàn lại dây. Thử lại bằng thanh trượt ở trên.</p>
      <div class="escdir">${[1, 2, 3, 4].map((m) => `<div><span>Motor ${m}</span><button class="btn" data-dir="${m}_0">Bình thường</button><button class="btn" data-dir="${m}_1">Đảo chiều</button></div>`).join("")}</div>`
    : `<p class="muted">ESC dùng ${PROTO[proto]} cần <b>hiệu chuẩn dải ga</b> một lần, để mọi ESC hiểu cùng một mức ga thấp nhất / cao nhất.</p>
      <ol class="escsteps" id="escSteps">
        <li>Tháo cánh quạt, <b>rút pin</b> (ESC tắt), cắm USB.</li>
        <li>Bấm <b>Bắt đầu</b>: FC xuất mức ga cao nhất.</li>
        <li>Cắm pin. ESC kêu một chuỗi bíp xác nhận mức cao (thường 2 tiếng ngắn).</li>
        <li>Bấm <b>Hạ ga</b> ngay sau tiếng bíp: ESC kêu xác nhận mức thấp rồi tiếng báo xong.</li>
        <li>Bấm <b>Xong</b>, rút pin. Thử lại từng motor bằng thanh trượt ở trên.</li>
      </ol>
      <label class="check"><input type="checkbox" id="escCalOk"> Đã tháo cánh quạt và rút pin</label>
      <div class="row" style="margin-top:8px"><button class="btn primary" id="escCalStart" disabled>Bắt đầu (ga cao nhất)</button><button class="btn" id="escCalLow" disabled>Hạ ga</button><button class="btn" id="escCalDone" disabled>Xong</button><button class="btn danger" id="escCalAbort" disabled>Huỷ</button></div>
      <p class="note" id="escCalMsg" hidden></p>`}`;
  const ps = host.querySelector("select[data-p=motor_protocol]");
  ps.value = String(proto);
  ps.onchange = () => setEdit("motor_protocol", ps.value);
  const propsOff = () => { const a = $("mtArm"); if (a && a.checked) return true; toast("Tích \"Tôi đã tháo cánh quạt\" ở phần thử motor trước."); return false; };
  host.querySelectorAll("[data-beep]").forEach((b) => b.onclick = async () => {
    if (!propsOff()) return;
    try { await api("/api/cmd?c=escbeep_" + b.dataset.beep, { method: "POST" }); toast("Đã gửi lệnh bíp."); }
    catch (e) { toast(e.code === 423 ? "Hãy disarm trước." : "Lỗi: " + e.message); }
  });
  host.querySelectorAll("[data-dir]").forEach((b) => b.onclick = async () => {
    if (!propsOff()) return;
    const [m, r] = b.dataset.dir.split("_");
    if (!confirm(`Đặt chiều quay motor ${m} thành "${r === "1" ? "đảo chiều" : "bình thường"}" và lưu trong ESC?`)) return;
    try { await api("/api/cmd?c=escdir_" + b.dataset.dir, { method: "POST" }); toast(`Motor ${m}: đã lưu chiều quay trong ESC.`); }
    catch (e) { toast(e.code === 423 ? "Hãy disarm trước." : "Lỗi: " + e.message); }
  });
  if (!dshot) {
    const ok = $("escCalOk"), set = (st) => {
      if (!$("escCalStart")) return;
      $("escCalStart").disabled = !ok.checked || st !== 0;
      $("escCalLow").disabled = st !== 1;
      $("escCalDone").disabled = st !== 2;
      $("escCalAbort").disabled = st === 0;
      document.querySelectorAll("#escSteps li").forEach((li, i) => li.classList.toggle("cur", i === [1, 2, 3, 4][st] || (st === 0 && i === 0)));
    };
    ESCC.render = set;
    ok.onchange = () => set(ESCC.state);
    $("escCalStart").onclick = () => escCalGo(1, "Mức ga CAO đang xuất ra. Cắm pin ngay, chờ ESC kêu bíp xác nhận, rồi bấm Hạ ga.");
    $("escCalLow").onclick = () => escCalGo(2, "Mức ga THẤP. Chờ ESC kêu tiếng báo xong (vài giây), rồi bấm Xong và rút pin.");
    $("escCalDone").onclick = () => { escCalStop(); escMsg("Hiệu chuẩn xong. Thử từng motor bằng thanh trượt ở trên (motor phải bắt đầu quay cùng một mức ga).", false); };
    $("escCalAbort").onclick = () => { escCalStop(); escMsg("Đã huỷ. FC xuất mức ga thấp; rút pin trước khi làm lại.", true); };
    set(0);
  }
}
function escMsg(t, bad) { const m = $("escCalMsg"); if (!m) return; m.hidden = false; m.className = "note" + (bad ? " bad" : ""); m.textContent = t; }
async function escCalSend() {
  if (!ESCC.state) return;
  try { await api("/api/cmd?c=esccal_" + (ESCC.state === 1 ? "hi" : "lo"), { method: "POST", timeout: 900 }); }
  catch (e) { escCalStop(); escMsg(e.code === 409 ? "Giao thức đang là DShot: không cần hiệu chuẩn." : e.code === 423 ? "Hãy disarm trước." : "Mất liên lạc với FC: đã dừng (FC tự về ga thấp sau 1,5 s).", true); return; }
  ESCC.timer = setTimeout(escCalSend, 300);  // keep-alive: the FC drops back to zero throttle by itself after 1.5 s
}
function escCalGo(st, msg) {
  clearTimeout(ESCC.timer);
  ESCC.state = st;
  escMsg(msg, st === 1);
  if (ESCC.render) ESCC.render(st);
  escCalSend();
}
function escCalStop() {
  const was = ESCC.state;
  clearTimeout(ESCC.timer);
  ESCC.state = 0;
  if (ESCC.render) ESCC.render(0);
  if (was) api("/api/cmd?c=esccal_off", { method: "POST", timeout: 900 }).catch(() => {});
}
function escInit() {
  const sec = $("tab-motors");
  if (!sec || $("escCalPanel")) return;
  const p = document.createElement("div");
  p.className = "panel";
  p.id = "escCalPanel";
  const g = $("motOrderPanel") || sec.querySelector(".grid2");
  sec.insertBefore(p, g ? g.nextSibling : null);
  window.addEventListener("pagehide", escCalStop);
  document.addEventListener("visibilitychange", () => { if (document.hidden) escCalStop(); });
}

// ---------------------------------------------------------------- init
(function extrasInit() {
  const style = document.createElement("style");
  style.textContent = `.wzsteps{display:flex;flex-wrap:wrap;gap:6px;list-style:none;padding:0;margin:0 0 12px}
.wzsteps button{border:1px solid var(--rule);background:var(--sheet);color:var(--ink-2);border-radius:999px;padding:5px 11px;font:inherit;font-size:.85rem;cursor:pointer}
.wzsteps li.ok button{color:var(--ink);border-color:var(--ok)}.wzsteps li.cur button{background:var(--amber);color:var(--amber-ink);border-color:var(--amber)}
.wzchk{list-style:none;padding:0}.wzchk li{padding:4px 0;color:var(--ink-2)}.wzchk li.ok{color:var(--ink)}
.wzmot>div{display:flex;flex-wrap:wrap;align-items:center;gap:10px;padding:6px 0;border-bottom:1px solid var(--rule)}
.note.good{border-left-color:var(--ok);background:color-mix(in srgb,var(--ok) 10%,var(--sheet))} span.good{color:var(--ok)} span.bad{color:var(--danger)}
#mapTiles[aria-pressed=true]{background:var(--amber);color:var(--amber-ink)}
#calPanel .cl{display:block;font-size:.8rem;color:var(--ink-2);margin-bottom:2px}#calPanel .row .cell{flex:1;min-width:110px}
.calrow{display:flex;align-items:center;gap:10px;padding:5px 0}.calrow .btn{min-width:92px}
.escdir>div{display:flex;align-items:center;gap:8px;padding:4px 0}.escdir span{min-width:70px}
.mogrid{display:grid;grid-template-columns:1fr 1fr;gap:8px;max-width:340px}.motab select{min-width:120px}
.escsteps li{padding:3px 0;color:var(--ink-2)}.escsteps li.cur{color:var(--ink);font-weight:600}`;
  document.head.appendChild(style);
  wzSection();
  demoInit();
  tilesInit();
  bbInit();
  histInit();
  calInit();
  moInit();
  rxPreviewInit();
  escInit();
  ekfPanelInit();
  const om = $("oMsg");
  if (om && !$("xWarn")) { const d = document.createElement("div"); d.className = "chips"; d.id = "xWarn"; d.hidden = true; d.style.marginTop = "8px"; om.parentElement.insertBefore(d, om); }
})();

// ---------------------------------------------------------------- logic conditions (iNav style)
// The rules live in the config struct and are edited with the CLI 'logic' command (one path for the
// USB console, the web and backups); this panel only formats those commands.
const LG = { rules: [], loaded: false, dirty: {}, extra: 0 };
const LG_VALUES = ["Đã arm", "Độ cao (dm)", "Tốc độ (cm/s)", "Cách home (m)", "Pin (cV)", "Pin %", "Dòng (dA)", "Thời gian arm (s)",
  "Chế độ bay", "Link %", "Roll °", "Pitch °", "Số vệ tinh", "mAh đã dùng"];
const LG_OPS = ["luôn đúng", "=", ">", "<", "VÀ", "HOẶC", "KHÔNG (A)", "≥", "≤", "≠"];
const LG_ACT = ["chỉ kết quả", "Còi", "Bật mode box", "Ghi đè kênh RC", "Công suất VTX", "Hiện thông báo", "Chặn arm"];
const LG_TYPES = ["Hằng số", "Kênh RC", "Giá trị bay", "Kết quả rule"];
function lgParse(txt) {
  const rules = [];
  txt.split("\n").forEach((l) => {
    const m = l.match(/^logic (\d+) (-?\d+) (-?\d+) (-?\d+) (-?\d+) (-?\d+) (-?\d+) (-?\d+) (-?\d+) (-?\d+)/);
    if (m) { const v = m.slice(2).map(Number); rules[+m[1] - 1] = { en: v[0], at: v[1], av: v[2], op: v[3], bt: v[4], bv: v[5], ac: v[6], arg: v[7], val: v[8] }; }
  });
  return rules;
}
async function lgLoad() {
  const host = lgHost();
  if (!host) return;
  try {
    const out = await api("/api/cli", { method: "POST", body: "logic", text: true, timeout: 8000 });
    LG.rules = lgParse(out);
    LG.loaded = LG.rules.length > 0;
  } catch (e) { LG.loaded = false; }
  LG.dirty = {};
  lgRender();
}
function lgHost() {
  let p = $("lgPanel");
  const sec = $("tab-ports");
  if (!p && sec) {
    const h = document.createElement("h2"); h.textContent = "Logic (điều kiện tự động)"; h.dataset.j = "Logic";
    p = document.createElement("div"); p.className = "panel"; p.id = "lgPanel";
    sec.appendChild(h); sec.appendChild(p);
  }
  return p;
}
function lgOpnd(i, k, type, v) {
  const opts = LG_TYPES.map((t, j) => `<option value="${j}"${j === type ? " selected" : ""}>${t}</option>`).join("");
  let vin;
  if (type === 2) vin = `<select data-lg="${i}" data-k="${k}v">${LG_VALUES.map((t, j) => `<option value="${j}"${j === v ? " selected" : ""}>${t}</option>`).join("")}</select>`;
  else if (type === 3) vin = `<select data-lg="${i}" data-k="${k}v">${[0, 1, 2, 3, 4, 5, 6, 7].map((j) => `<option value="${j}"${j === v ? " selected" : ""}>Rule ${j + 1}</option>`).join("")}</select>`;
  else vin = `<input type="number" data-lg="${i}" data-k="${k}v" value="${v}" style="width:6em" ${type === 1 ? 'min="1" max="16" title="kênh 1-16"' : ""}>`;
  return `<select data-lg="${i}" data-k="${k}t">${opts}</select> ${vin}`;
}
function lgRender() {
  const host = lgHost();
  if (!host) return;
  if (!LG.loaded) { host.innerHTML = '<p class="muted" style="margin:0">Firmware này chưa có logic, hoặc chưa kết nối.</p>'; return; }
  // only rules in use plus one empty slot; "+ Thêm rule" opens more
  let blanks = 1 + LG.extra;
  const rows = LG.rules.map((r, i) => {
    if (!r) return "";
    if (!r.en && !LG.dirty[i] && !(r.op || r.ac) && blanks-- <= 0) return "";
    const unary = r.op === 0 || r.op === 6;
    const argIn = r.ac === 2 ? `<select data-lg="${i}" data-k="arg">${(typeof BOXES !== "undefined" ? BOXES : []).map((t, j) => `<option value="${j}"${j === r.arg ? " selected" : ""}>${t}</option>`).join("")}</select>`
      : r.ac === 3 ? `kênh <input type="number" data-lg="${i}" data-k="arg" value="${r.arg}" min="1" max="16" style="width:4em"> = <input type="number" data-lg="${i}" data-k="val" value="${r.val}" min="800" max="2200" style="width:5em"> µs`
      : r.ac === 4 ? `mức <input type="number" data-lg="${i}" data-k="val" value="${r.val}" min="1" max="5" style="width:4em">` : "";
    return `<tr data-row="${i}"><td><label><input type="checkbox" data-lg="${i}" data-k="en"${r.en ? " checked" : ""}> ${i + 1}</label> <span class="chip" id="lgRes${i}" hidden>ĐÚNG</span></td>
      <td>${r.op === 0 ? "" : lgOpnd(i, "a", r.at, r.av)}</td>
      <td><select data-lg="${i}" data-k="op">${LG_OPS.map((t, j) => `<option value="${j}"${j === r.op ? " selected" : ""}>${t}</option>`).join("")}</select></td>
      <td>${unary ? "" : lgOpnd(i, "b", r.bt, r.bv)}</td>
      <td><select data-lg="${i}" data-k="ac">${LG_ACT.map((t, j) => `<option value="${j}"${j === r.ac ? " selected" : ""}>${t}</option>`).join("")}</select> ${argIn}</td></tr>`;
  }).join("");
  host.innerHTML = `<p class="muted" style="margin-top:0">Mỗi rule: nếu điều kiện đúng thì làm hành động. Ví dụ: Giá trị bay "Cách home (m)" &gt; Hằng số 500 → Còi. "Kết quả rule" dùng rule khác làm điều kiện (VÀ / HOẶC).</p>
    <div style="overflow-x:auto"><table class="ports lgtab"><tr><th>Rule</th><th>A</th><th>Phép so</th><th>B</th><th>Hành động</th></tr>${rows}</table></div>
    <div class="row" style="margin-top:8px"><button class="btn primary" id="lgApply" disabled>Ghi rule đã sửa</button><button class="btn" id="lgMore">+ Thêm rule</button><button class="btn" id="lgReload">Đọc lại</button><span class="muted" id="lgMsg"></span></div>`;
  host.querySelectorAll("[data-lg]").forEach((el) => (el.onchange = () => {
    const i = +el.dataset.lg, r = LG.rules[i], k = el.dataset.k;
    const v = el.type === "checkbox" ? (el.checked ? 1 : 0) : Math.round(+el.value || 0);
    const map = { en: "en", at: "at", av: "av", op: "op", bt: "bt", bv: "bv", ac: "ac", arg: "arg", val: "val" };
    r[map[k]] = v;
    if (k === "at" || k === "bt") r[k === "at" ? "av" : "bv"] = k === "at" ? (v === 1 ? 1 : 0) : (v === 1 ? 1 : 0);
    if (k === "ac") { r.arg = r.ac === 3 ? 1 : 0; r.val = r.ac === 3 ? 1500 : r.ac === 4 ? 1 : 0; }
    LG.dirty[i] = true;
    const keep = { scroll: host.scrollTop };
    lgRender();
    host.scrollTop = keep.scroll;
  }));
  $("lgApply").disabled = !Object.keys(LG.dirty).length;
  $("lgApply").onclick = lgApply;
  $("lgReload").onclick = lgLoad;
  $("lgMore").onclick = () => { LG.extra++; lgRender(); };
  $("lgMore").hidden = host.querySelectorAll("tr[data-row]").length >= LG.rules.filter(Boolean).length;
}
async function lgApply() {
  const msg = $("lgMsg");
  try {
    for (const i of Object.keys(LG.dirty)) {
      const r = LG.rules[i];
      const out = await api("/api/cli", { method: "POST", body: `logic ${+i + 1} ${r.en} ${r.at} ${r.av} ${r.op} ${r.bt} ${r.bv} ${r.ac} ${r.arg} ${r.val}`, text: true, timeout: 8000 });
      if (!/ok/.test(out)) throw new Error(`rule ${+i + 1}: ${out.trim()}`);
    }
    LG.dirty = {};
    lgRender();
    $("lgMsg").textContent = "Đã ghi. Bấm Lưu vào flash để giữ.";
  } catch (e) { msg.textContent = "Lỗi: " + e.message; }
}
function lgLive(st) {
  if (st.lg === undefined) return;
  for (let i = 0; i < 8; i++) { const c = $("lgRes" + i); if (c) c.hidden = !((st.lg >> i) & 1); }
}

// ---------------------------------------------------------------- overview: dashboard tiles
// Built from the status the page already receives: no extra request to the FC.
const PC = { amps: null, armT: 0 };
const TILES_DEF = [["bat", "Pin"], ["link", "Tín hiệu"], ["gps", "GPS"], ["fly", "Bay"], ["pos", "Độ cao"], ["left", "Còn bay"], ["loop", "Tải CPU"]];
function tileSet(id, cls, v, sub, pct) {
  const t = $("tl-" + id);
  if (!t) return;
  t.className = "tile " + cls;
  t.querySelector(".v").textContent = v;
  t.querySelector(".s").textContent = sub;
  const r = t.querySelector(".ring");
  r.hidden = pct == null;
  if (pct != null) r.style.setProperty("--p", Math.max(0, Math.min(100, pct)));
}
function perfCard(st) {
  let p = $("pcPanel");
  if (!p) {
    const sec = $("tab-overview");
    if (!sec) return;
    p = document.createElement("div");
    p.className = "dash"; p.id = "pcPanel";
    p.innerHTML = TILES_DEF.map(([id, k]) => `<div class="tile idle" id="tl-${id}"><div class="k">${k}</div><div class="v">—</div><div class="s"></div><div class="ring" hidden></div></div>`).join("");
    const anchor = $("blockers");
    sec.insertBefore(p, anchor ? anchor.nextSibling : sec.firstChild);
    // keep the ids other code and tests use
    $("tl-loop").querySelector(".s").id = "pcLoadT";
    $("tl-left").querySelector(".v").id = "pcLeft";
    $("tl-left").querySelector(".s").id = "pcLeftT";
  }
  const b = st.bat || {}, rc = st.rc || {}, g = st.gps || {}, e = st.est || {};
  // battery
  if (typeof b.v === "number") {
    const pct = typeof b.pct === "number" ? b.pct : null;
    tileSet("bat", b.v < 1 ? "idle" : pct != null && pct < 20 ? "bad" : pct != null && pct < 40 ? "warn" : "ok",
      `${b.v.toFixed(1)} V`, `${b.cells ? b.cells + "S · " : ""}${pct != null ? pct + " % · " : ""}${(b.mah || 0).toFixed(0)} mAh`, pct);
  }
  // link
  const lq = typeof rc.lq === "number" ? rc.lq : st.lq;
  const rxok = rc.rxok !== undefined ? rc.rxok : st.rxok;
  if (lq != null || rxok != null) tileSet("link", rxok === false ? "bad" : lq < 50 ? "warn" : "ok", rxok === false ? "MẤT SÓNG" : `LQ ${lq} %`, rxok === false ? "không nhận được tay điều khiển" : "đang nhận", rxok === false ? 0 : lq);
  // gps
  const fx = g.fix === true ? 3 : g.fix === false ? 0 : +g.fix || 0;
  if (g.fix !== undefined) tileSet("gps", fx >= 3 && g.sats >= 8 ? "ok" : fx >= 2 ? "warn" : "bad",
    `${g.sats || 0} vệ tinh`, fx >= 2 ? `${fx === 3 ? "3D" : "2D"} fix · sai số ${(g.hacc || 0).toFixed(1)} m` : "chưa fix", Math.min(100, (g.sats || 0) * 100 / 16));
  // flight
  if (st.armed && !PC.armT) PC.armT = Date.now();
  if (!st.armed) PC.armT = 0;
  const ft = PC.armT ? Math.floor((Date.now() - PC.armT) / 1000) : 0;
  tileSet("fly", st.failsafe ? "bad" : st.armed ? "warn" : "ok", st.mode || "—",
    st.failsafe ? "FAILSAFE" : st.armed ? `ARMED · ${Math.floor(ft / 60)}:${String(ft % 60).padStart(2, "0")}` : `Disarmed${st.nav && st.nav !== "IDLE" ? " · " + st.nav : ""}`);
  // altitude / home
  if (typeof e.alt === "number") tileSet("pos", "ok", `${e.alt.toFixed(1)} m`,
    `${typeof e.vz === "number" ? (e.vz >= 0 ? "+" : "") + e.vz.toFixed(1) + " m/s" : ""}${e.home && typeof e.dist === "number" ? ` · cách nhà ${e.dist.toFixed(0)} m` : ""}`);
  // loop load (perf comes with the full status, about once a second)
  if (st.perf) {
    const [, , , , , total, worst, period, load] = st.perf;
    tileSet("loop", load > 70 ? "bad" : load > 50 ? "warn" : "ok", `${load.toFixed(0)} %`, `${load.toFixed(0)} % · TB ${total.toFixed(0)} µs, chậm nhất ${worst.toFixed(0)} / ${period.toFixed(0)} µs`, load);
  }
  // time left: (capacity - reserve - used) / ~10 s average current
  const cap = has("bat_capacity") ? val("bat_capacity") : 0;
  if (typeof b.a === "number") PC.amps = PC.amps == null ? b.a : PC.amps + (b.a - PC.amps) * 0.02;
  if (!cap || typeof b.mah !== "number") { tileSet("left", "idle", "—", cap ? "" : "đặt bat_capacity (mAh) để tính"); return; }
  const usable = cap * (100 - (has("bat_reserve_pct") ? val("bat_reserve_pct") : 0)) / 100, left = usable - b.mah;
  if (PC.amps > 0.5 && left > 0) {
    const s = Math.round(left / (PC.amps * 1000 / 3600));
    tileSet("left", s < 120 ? "bad" : s < 300 ? "warn" : "ok", `${Math.floor(s / 60)}:${String(s % 60).padStart(2, "0")}`,
      `còn ${left.toFixed(0)} mAh trước mức dự trữ · ${PC.amps.toFixed(1)} A`, (100 * left) / usable);
  } else {
    tileSet("left", left <= 0 ? "bad" : "idle", left <= 0 ? "0:00" : "—", left <= 0 ? "đã chạm mức dự trữ" : "chờ có dòng điện để ước tính", left <= 0 ? 0 : (100 * left) / usable);
  }
}

// ---------------------------------------------------------------- PID: simple sliders (Betaflight style)
const SL = { base: null };
const SL_AX = ["roll", "pitch", "yaw"];
const SL_DEF = [
  ["master", "Độ mạnh chung", "Nhân P, I, D, F cùng lúc: tăng = bám cần chặt hơn, quá cao thì rung.", 0.5, 1.5],
  ["pd", "Giảm chấn (D so với P)", "Tăng = ít vọt lố, êm hơn khi dừng; cao quá thì motor nóng.", 0.5, 1.5],
  ["ff", "Phản hồi cần (feedforward)", "Tăng = phản ứng ngay khi gạt cần; quá cao thì vọt khi gạt nhanh.", 0, 2],
  ["i", "Giữ hướng (I)", "Tăng = giữ thẳng tốt hơn trước gió và lệch trọng tâm; quá cao thì lắc chậm.", 0.5, 1.5],
];
function slBase() {
  SL.base = {};
  for (const a of SL_AX) for (const k of ["p", "i", "d", "f"]) if (has(`${a}_${k}`)) SL.base[`${a}_${k}`] = val(`${a}_${k}`);
}
function sliderHost() {
  const sec = $("tab-pid");
  if (!sec || !has("roll_p")) return;
  if (!SL.base) slBase();
  let p = $("slPanel");
  if (p) return;
  p = document.createElement("div");
  p.className = "panel"; p.id = "slPanel";
  p.innerHTML = `<div class="card-h"><span class="dot"></span><h3>Chỉnh nhanh bằng thanh trượt</h3></div>
    <p class="card-d">Các thanh nhân so với mốc (giá trị lúc mở trang hoặc lúc bấm "Lấy mốc mới"). Chỉnh từng chút, bay thử, xem blackbox.</p>
    <div class="slgrid">${SL_DEF.map(([k, t, d, lo, hi]) => `<div class="prow"><div><div class="name">${t}</div><div class="hint">${d}</div></div>
      <div class="ctl"><input type="range" min="${lo}" max="${hi}" step="0.05" value="1" data-sl="${k}" style="width:100%;min-height:0;border:0;padding:0"><div class="num muted" style="text-align:right" id="slv-${k}">×1.00</div></div></div>`).join("")}</div>
    <div class="row" style="margin-top:8px"><button class="btn" id="slReset">Về mốc</button><button class="btn" id="slRebase">Lấy mốc mới</button></div>`;
  const matrix = $("pidMatrix");
  const after = matrix ? matrix.closest(".panel") : null;
  sec.insertBefore(p, after ? after.nextSibling : null);
  p.querySelectorAll("[data-sl]").forEach((el) => (el.oninput = slApply));
  $("slReset").onclick = () => { p.querySelectorAll("[data-sl]").forEach((el) => (el.value = 1)); slApply(); };
  $("slRebase").onclick = () => { slBase(); p.querySelectorAll("[data-sl]").forEach((el) => (el.value = 1)); slApply(true); toast("Đã lấy giá trị hiện tại làm mốc."); };
}
function slApply(quiet) {
  const v = {};
  $("slPanel").querySelectorAll("[data-sl]").forEach((el) => { v[el.dataset.sl] = +el.value; $("slv-" + el.dataset.sl).textContent = "×" + (+el.value).toFixed(2); });
  if (quiet === true) return;
  for (const a of SL_AX) {
    const m = v.master;
    const target = { p: m, i: m * v.i, d: a === "yaw" ? m : m * v.pd, f: m * v.ff };
    for (const k of ["p", "i", "d", "f"]) {
      const n = `${a}_${k}`;
      if (!(n in SL.base)) continue;
      const nv = Math.round(SL.base[n] * target[k]);
      const pr = S.params[n];
      const clamped = pr ? Math.min(pr.max, Math.max(pr.min, nv)) : nv;
      if (clamped !== val(n)) setEdit(n, clamped);
    }
  }
}

// ---------------------------------------------------------------- motors: vibration of each motor (props off)
const VB = { run: false };
function vibHost() {
  const sec = $("tab-motors");
  if (!sec || $("vbPanel") || !has("motor1_out")) return;
  const p = document.createElement("div");
  p.className = "panel"; p.id = "vbPanel";
  p.innerHTML = `<div class="card-h"><span class="dot"></span><h3>Đo rung từng motor</h3></div>
    <p class="card-d">Tháo cánh quạt. Mỗi motor quay riêng 3 giây ở mức ga đã chọn, trang đo rung bằng gyro và gia tốc. Motor rung hơn hẳn các motor khác thường do trục cong, bạc đạn hỏng hoặc chuông motor lệch.</p>
    <div class="row"><label>Ga <input type="number" id="vbThr" value="30" min="10" max="50" style="width:5em"> %</label><button class="btn primary" id="vbGo">Bắt đầu đo</button><span class="muted" id="vbMsg"></span></div>
    <div id="vbOut" style="margin-top:10px"></div>`;
  const mt = $("mtArm") ? $("mtArm").closest(".grid2") : null;
  if (mt) mt.after(p); else sec.appendChild(p);
  $("vbGo").onclick = vibRun;
}
async function vibSample(ms) {
  const g = [], a = [];
  const end = performance.now() + ms;
  while (performance.now() < end && VB.run) {
    try {
      const s = await api("/api/sensors", { timeout: 800 });
      g.push(Math.hypot(...s.gyro));
      a.push(Math.hypot(...s.acc));
    } catch (e) {}
    await new Promise((r) => setTimeout(r, 40));
  }
  const rms = (x) => { if (x.length < 3) return 0; const m = x.reduce((p, q) => p + q, 0) / x.length; return Math.sqrt(x.reduce((p, q) => p + (q - m) ** 2, 0) / x.length); };
  const mean = (x) => (x.length ? x.reduce((p, q) => p + q, 0) / x.length : 0);
  return { gyro: mean(g), acc: rms(a), n: g.length };
}
async function vibRun() {
  if (VB.run) { VB.run = false; return; }
  if (S.armed) return toast("Hãy disarm trước.");
  if (!$("mtArm") || !$("mtArm").checked) return toast('Tích "Tôi đã tháo cánh quạt" ở phần Thử motor trước.');
  const thr = Math.min(50, Math.max(10, +$("vbThr").value || 30)) / 100;
  VB.run = true;
  $("vbGo").textContent = "Dừng";
  const res = [];
  try {
    $("vbMsg").textContent = "Đo nền (motor đứng yên)…";
    const base = await vibSample(1500);
    for (let m = 0; m < 4 && VB.run; m++) {
      $("vbMsg").textContent = `Motor ${m + 1}…`;
      const v = [0, 0, 0, 0];
      v[m] = thr;
      const keep = setInterval(() => api(`/api/motor?m1=${v[0]}&m2=${v[1]}&m3=${v[2]}&m4=${v[3]}`, { method: "POST", timeout: 900 }).catch(() => {}), 300);
      try {
        await api(`/api/motor?m1=${v[0]}&m2=${v[1]}&m3=${v[2]}&m4=${v[3]}`, { method: "POST", timeout: 900 });
        await new Promise((r) => setTimeout(r, 700));  // spin up
        res.push(await vibSample(2500));
      } finally {
        clearInterval(keep);
        await api("/api/motor?m1=0&m2=0&m3=0&m4=0", { method: "POST", timeout: 900 }).catch(() => {});
      }
      await new Promise((r) => setTimeout(r, 800));
    }
    if (res.length === 4) {
      const score = res.map((r) => Math.max(0, r.gyro - base.gyro) + 20 * Math.max(0, r.acc - base.acc));
      const med = score.slice().sort((x, y) => x - y)[1] * 0.5 + score.slice().sort((x, y) => x - y)[2] * 0.5;
      const max = Math.max(...score, 1e-6);
      $("vbOut").innerHTML = score.map((s, i) => {
        const bad = med > 0 && s > med * 1.6;
        return `<div class="row" style="gap:10px;margin:4px 0"><b style="width:5em">Motor ${i + 1}</b><div class="mbar" style="flex:1"><span style="width:${(100 * s / max).toFixed(0)}%;background:${bad ? "var(--danger)" : "var(--pitch)"}"></span></div>
          <span class="num" style="width:13em">${res[i].gyro.toFixed(1)} °/s · ${res[i].acc.toFixed(2)} m/s²</span>${bad ? '<span class="chip bad">rung bất thường</span>' : ""}</div>`;
      }).join("");
      $("vbMsg").textContent = score.some((s) => med > 0 && s > med * 1.6) ? "Có motor rung hơn hẳn: kiểm tra trục, bạc đạn, ốc bắt motor." : "Các motor rung tương đương nhau.";
    }
  } finally {
    VB.run = false;
    $("vbGo").textContent = "Bắt đầu đo";
  }
}

// ---------------------------------------------------------------- params: compare with a backup file
function cmpHost() {
  const sec = $("tab-params");
  if (!sec || $("cmpPanel")) return;
  const p = document.createElement("div");
  p.className = "panel"; p.id = "cmpPanel";
  p.innerHTML = `<div class="card-h"><span class="dot"></span><h3>So sánh với file sao lưu</h3></div>
    <p class="card-d">Chọn một file sao lưu (.txt): trang liệt kê các tham số khác với drone hiện tại.</p>
    <div class="row"><label class="btn" for="cmpFile">Chọn file…</label><input type="file" id="cmpFile" accept=".txt,text/plain" hidden><span class="muted" id="cmpMsg"></span></div>
    <div id="cmpOut"></div>`;
  const grid = $("paramGroups");
  sec.insertBefore(p, grid);
  $("cmpFile").onchange = async (ev) => {
    const f = ev.target.files[0];
    if (!f) return;
    const text = await f.text();
    ev.target.value = "";
    const diffs = [];
    let n = 0;
    for (const line of text.split(/\r?\n/)) {
      const m = line.match(/^set\s+(\S+)\s*=?\s*(\S+)/);
      if (!m || !S.params[m[1]]) continue;
      n++;
      const fv = parseFloat(m[2]), cv = val(m[1]);
      if (Math.abs(fv - cv) > Math.max(1e-4, Math.abs(cv) * 1e-4)) diffs.push([m[1], cv, fv]);
    }
    $("cmpMsg").textContent = `${f.name}: ${n} tham số, ${diffs.length} khác.`;
    $("cmpOut").innerHTML = diffs.length ? `<div style="overflow-x:auto;margin-top:8px"><table class="ports"><tr><th>Tham số</th><th>Drone</th><th>File</th></tr>
      ${diffs.map(([k, c, v]) => `<tr><td><code>${k}</code></td><td class="num">${fmt(k, c)}</td><td class="num">${fmt(k, v)}</td></tr>`).join("")}</table></div>
      <div class="row" style="margin-top:8px"><button class="btn primary" id="cmpTake">Đưa giá trị trong file vào hàng chờ</button></div>` : "";
    if (diffs.length) $("cmpTake").onclick = () => { diffs.forEach(([k, , v]) => setEdit(k, v)); toast(`${diffs.length} thay đổi đang chờ: kiểm tra rồi bấm Áp dụng.`); };
  };
}

// ---------------------------------------------------------------- voice alerts (phone in hand while flying)
const VO = { on: lsGet("aq-voice", false), last: 0, armed: null, said: {} };
function voiceHost() {
  const sec = $("tab-system");
  if (!sec || $("voPanel") || !("speechSynthesis" in window)) return;
  const p = document.createElement("div");
  p.className = "panel"; p.id = "voPanel";
  p.innerHTML = `<div class="card-h"><span class="dot"></span><h3>Cảnh báo bằng giọng nói</h3></div>
    <p class="card-d">Điện thoại đọc to khi arm / disarm, pin yếu, pin cạn, mất sóng, vượt hàng rào. Chỉ hoạt động khi trang đang mở.</p>
    <label class="row" style="gap:8px"><input type="checkbox" id="voOn"> Bật giọng nói</label> <button class="btn" id="voTest" style="margin-top:8px">Thử</button>`;
  const conn = $("sysParams");
  if (conn) conn.appendChild(p); else sec.appendChild(p);
  $("voOn").checked = VO.on;
  $("voOn").onchange = () => { VO.on = $("voOn").checked; lsSet("aq-voice", VO.on); if (VO.on) say("Đã bật giọng nói"); };
  $("voTest").onclick = () => say("Pin yếu");
}
function say(text) {
  try {
    const u = new SpeechSynthesisUtterance(text);
    u.lang = "vi-VN";
    const v = speechSynthesis.getVoices().find((x) => /^vi/i.test(x.lang));
    if (v) u.voice = v;
    speechSynthesis.speak(u);
  } catch (e) {}
}
function voiceStatus(st) {
  if (!VO.on || !("speechSynthesis" in window)) return;
  if (VO.armed !== null && st.armed !== VO.armed) say(st.armed ? "Đã arm" : "Đã disarm");
  VO.armed = st.armed;
  const now = Date.now(), w = st.warn || 0;
  const items = [[4, "Mất sóng tay điều khiển"], [2, "Pin cạn, hạ cánh ngay"], [1, "Pin yếu"], [16, "Vượt hàng rào"], [8192, "Mất liên lạc trạm mặt đất"]];
  for (const [bit, text] of items) {
    const on = !!(w & bit);
    // say it when it starts, then again every 20 s while it lasts
    if (on && (!VO.said[bit] || now - VO.said[bit] > 20000)) { say(text); VO.said[bit] = now; break; }
    if (!on) VO.said[bit] = 0;
  }
}

// ---------------------------------------------------------------- mission file import / export (QGC WPL 110)
function wplExport() {
  if (!S.wp) return;
  const r = S.wp.ref, L = [`QGC WPL 110`, `0\t1\t0\t16\t0\t0\t0\t0\t${r.lat.toFixed(7)}\t${r.lon.toFixed(7)}\t0\t1`];
  S.wp.items.forEach((it, i) => L.push([i + 1, 0, it.frame || 3, it.cmd, it.p1 || 0, it.p2 || 0, it.p3 || 0, it.p4 || 0,
    it.pos || it.la || it.lo ? (it.la * 1e-7).toFixed(7) : 0, it.pos || it.la || it.lo ? (it.lo * 1e-7).toFixed(7) : 0, it.alt || 0, 1].join("\t")));
  const a = document.createElement("a");
  a.href = URL.createObjectURL(new Blob([L.join("\n") + "\n"], { type: "text/plain" }));
  a.download = "allquad-mission.waypoints";
  document.body.appendChild(a); a.click(); a.remove();
  setTimeout(() => URL.revokeObjectURL(a.href), 5000);
}
function wplParse(text) {
  const lines = text.split(/\r?\n/).map((l) => l.trim()).filter(Boolean);
  if (!/^QGC WPL 1\d\d/.test(lines[0] || "")) throw new Error("không phải file QGC WPL");
  const items = [];
  for (const l of lines.slice(1)) {
    const f = l.split(/\s+/);
    if (f.length < 12) continue;
    const seq = +f[0], frame = +f[2], cmd = +f[3];
    if (seq === 0 && frame === 0) continue;  // home line
    const lat = +f[8], lon = +f[9];
    items.push({ cmd, frame: frame === 0 ? 0 : 3, p1: +f[4], p2: +f[5], p3: +f[6], p4: +f[7], la: Math.round(lat * 1e7), lo: Math.round(lon * 1e7), alt: +f[10] });
  }
  return items;
}
function wplHost() {
  const host = $("wpPanel");
  if (!host || $("wplExp")) return;
  const row = host.querySelector(".row span.row");
  if (!row) return;
  row.insertAdjacentHTML("afterbegin", `<button class="btn" id="wplExp">Xuất file</button><label class="btn" for="wplFile">Nhập file</label><input type="file" id="wplFile" accept=".waypoints,.txt,.plan" hidden>`);
  $("wplExp").onclick = wplExport;
  $("wplFile").onchange = async (ev) => {
    const f = ev.target.files[0];
    ev.target.value = "";
    if (!f || !S.wp) return;
    try {
      const items = wplParse(await f.text());
      if (items.length > 120) throw new Error(`${items.length} điểm, tối đa 120`);
      if (S.wp.items.length && !confirm(`Thay ${S.wp.items.length} điểm hiện có bằng ${items.length} điểm trong file?`)) return;
      S.wp.items = items;
      S.wp.layer = "wp";
      wpSync();
      toast(`Đã nhập ${items.length} điểm. Kiểm tra rồi bấm Gửi lên drone.`);
    } catch (e) { toast("Không đọc được file: " + e.message); }
  };
}
if ($("wpPanel")) wplHost();
