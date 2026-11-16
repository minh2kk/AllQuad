// ================================================================ AllQuad configurator: v3 tools
// Everything here runs in the browser from data the page already receives (status stream, parameter
// list, blackbox files): no extra request and no extra work for the flight controller.
//   pre-flight checklist · review / undo of pending changes · applied-but-not-saved reminder
//   drone profiles (browser storage) · RC link history · blackbox noise heatmap + event list
//   mission distance / time / battery · global search (Ctrl+K) · simple / advanced view
//   theme switch incl. high-contrast outdoor theme · compact sidebar · status-rate pill
//   USB link through WebUSB (phones / browsers without Web Serial)

const PRO = { rate: [], hist: [], lastOk: null, drops: 0, lqMin: 101, lossStart: 0, lossMax: 0, unsaved: false, heat: {}, events: [] };

// ---------------------------------------------------------------- small helpers
function proTitle(n) {
  const h = HINT[n];
  if (!h) return n;
  return h.split(" · ")[0].split(/:\s|\s\(/)[0].trim();
}
function proValText(n, v) {
  const en = ENUMS[n];
  if (en && en[v] !== undefined) return en[v];
  const p = S.params[n];
  if (p && !p.f && p.min === 0 && p.max === 1 && !en) return v ? "bật" : "tắt";
  return String(fmt(n, v));
}
function proModal(id, title, body) {
  let m = $(id);
  if (!m) {
    m = document.createElement("div");
    m.className = "login pro-modal";
    m.id = id;
    m.hidden = true;
    m.innerHTML = `<div class="panel pro-dlg" role="dialog" aria-modal="true"><div class="pro-dlg-h"><h3></h3><button type="button" class="btn ghost pro-x" aria-label="Đóng">✕</button></div><div class="pro-dlg-b"></div></div>`;
    document.body.appendChild(m);
    m.querySelector(".pro-x").onclick = () => (m.hidden = true);
    m.addEventListener("click", (e) => { if (e.target === m) m.hidden = true; });
  }
  m.querySelector("h3").textContent = title;
  if (body !== undefined) m.querySelector(".pro-dlg-b").innerHTML = body;
  return m;
}
document.addEventListener("keydown", (e) => {
  if (e.key === "Escape") document.querySelectorAll(".pro-modal:not([hidden])").forEach((m) => (m.hidden = true));
});

// ---------------------------------------------------------------- status hook
function proStatus(st) {
  const now = Date.now();
  // update rate actually reaching the page
  PRO.rate.push(now);
  while (PRO.rate.length && now - PRO.rate[0] > 2000) PRO.rate.shift();
  const hz = PRO.rate.length / 2, pill = $("linkRate");
  if (pill) {
    pill.hidden = false;
    pill.textContent = `${LINK.open ? (LINK.kind === "usb" ? "USB" : LINK.kind.toUpperCase()) : "WiFi"} · ${hz.toFixed(hz < 10 ? 1 : 0)} Hz`;
    pill.className = "rate-pill num" + (hz < 1 ? " bad" : "");
  }
  // RC link history (2 minutes)
  const ok = st.rxok !== false, lq = ok ? (typeof st.lq === "number" && st.lq > 0 ? st.lq : 100) : 0;
  PRO.hist.push([now, lq, ok ? 1 : 0]);
  while (PRO.hist.length && now - PRO.hist[0][0] > 120000) PRO.hist.shift();
  if (PRO.lastOk === true && !ok) { PRO.drops++; PRO.lossStart = now; }
  if (!ok && PRO.lossStart) PRO.lossMax = Math.max(PRO.lossMax, now - PRO.lossStart);
  if (ok) PRO.lossStart = 0;
  PRO.lastOk = ok;
  if (ok) PRO.lqMin = Math.min(PRO.lqMin, lq);
  if (shown("preflight")) pfRender(st);
  if (shown("modes")) rxLinkRender();
}
function proTabShown(t) {
  if (t === "preflight") pfRender(S.status);
  if (t === "modes") rxLinkHost();
  if (t === "params") profHost();
  if (t === "nav" && S.wp) wpStats();
  markAdv();
}

// ---------------------------------------------------------------- pre-flight checklist
const PF_MANUAL = [
  ["props", "Cánh quạt siết chặt, đúng chiều, không nứt mẻ"],
  ["motors", "Đã thử thứ tự và chiều quay motor (tab Motor, tháo cánh)"],
  ["frame", "Ốc khung, ốc motor chắc; dây không chạm cánh"],
  ["battery", "Pin sạc đầy, buộc chắc; giắc pin không lỏng"],
  ["antenna", "Ăng-ten RX / VTX đúng vị trí, không bị cánh chạm"],
  ["failsafe", "Đã thử failsafe: tắt tay điều khiển (tháo cánh) thì motor dừng / RTH"],
  ["area", "Khu vực bay trống, không người, đúng quy định"],
];
const NAV_BOXES = [5, 6, 7, 13, 15];  // POSHOLD, RTH, MISSION, GPS RESCUE, SMART RTL
function pfBoxes() {
  const b = new Set();
  for (let i = 1; i <= 20; i++) if (S.params[`mr${i}_mode`]) { const v = val(`mr${i}_mode`); if (v) b.add(v); }
  return b;
}
function pfItems(st) {
  const out = [];
  const add = (state, title, detail, go) => out.push({ state, title, detail, go });
  if (!st) { add("bad", "Kết nối FC", "chưa nhận dữ liệu", null); return out; }
  add(S.linkOk ? "ok" : "bad", "Kết nối FC", S.linkOk ? "đang nhận dữ liệu" : "mất kết nối", null);
  const sn = st.sensors || {};
  add(sn.imu ? "ok" : "bad", "IMU (gyro + accel)", sn.imu || "không thấy chip", "sensors");
  if (S.params.acc_calibrated) add(val("acc_calibrated") ? "ok" : "bad", "Hiệu chuẩn accel", val("acc_calibrated") ? "đã hiệu chuẩn" : "chưa: drone sẽ trôi ở ANGLE", "sensors");
  if (sn.mag) {
    const cal = ["x", "y", "z"].some((k) => S.params["mag_bias_" + k] && val("mag_bias_" + k) !== 0);
    add(cal ? "ok" : "warn", "Compass", cal ? sn.mag + " · đã hiệu chuẩn" : sn.mag + " · chưa hiệu chuẩn", "sensors");
  }
  add(sn.baro ? "ok" : "off", "Baro", sn.baro || "không có (không giữ được độ cao)", "sensors");
  const ok = st.rxok !== false, lq = st.lq || 0;
  add(!ok ? "bad" : lq && lq < 50 ? "bad" : lq && lq < 80 ? "warn" : "ok", "Tay điều khiển",
    !ok ? "mất sóng" : lq ? `LQ ${lq}%${PRO.drops ? ` · mất sóng ${PRO.drops} lần từ lúc mở trang` : ""}` : "đang nhận", "modes");
  const boxes = pfBoxes();
  add(boxes.has(1) ? "ok" : "bad", "Công tắc ARM", boxes.has(1) ? "đã gán" : "chưa gán kênh ARM", "modes");
  if (S.params.fs_action) add("ok", "Failsafe mất sóng", proValText("fs_action", val("fs_action")), "modes");
  const b = st.bat || {};
  if (!b.v || b.v < 1) add("warn", "Pin", "không đo được điện áp (chưa cắm pin hoặc chưa nối chân đo)", "modes");
  else add(b.pct < 20 ? "bad" : b.pct < 60 ? "warn" : "ok", "Pin", `${b.v.toFixed(2)} V${b.cells ? ` · ${b.cells}S · ${(b.v / b.cells).toFixed(2)} V/cell` : ""}${b.pct != null ? ` · ${b.pct}%` : ""}`, "modes");
  const needGps = NAV_BOXES.some((x) => boxes.has(x)) || (S.params.fs_action && val("fs_action") >= 2);
  const g = st.gps || {};
  if (needGps) {
    add(!sn.gps ? "bad" : g.fix && g.sats >= 8 ? "ok" : "warn", "GPS (cần cho RTH / giữ vị trí)",
      !sn.gps ? "không có GPS nhưng đang dùng chế độ cần GPS" : g.fix ? `${g.sats} vệ tinh, sai số ${(+g.hacc || 0).toFixed(1)} m${g.sats < 8 ? " · chờ thêm vệ tinh" : ""}` : "chưa fix: chờ ngoài trời", "nav");
  } else if (sn.gps) add(g.fix ? "ok" : "off", "GPS", g.fix ? `${g.sats} vệ tinh` : "chưa fix (không bắt buộc)", "nav");
  const bl = (st.blockers || []).filter((x) => x !== "CLI");
  add(st.armed ? "warn" : bl.length ? "bad" : "ok", "Điều kiện arm",
    st.armed ? "đang ARM" : bl.length ? "còn chặn: " + bl.map((x) => (BLOCK_HELP[x] || [x])[0]).join("; ") : "đạt (CLI chỉ chặn khi đang nối cấu hình)", "overview");
  const pend = Object.keys(S.edits).length;
  add(pend ? "bad" : PRO.unsaved ? "warn" : "ok", "Cấu hình đã lưu", pend ? `${pend} thay đổi chưa áp dụng` : PRO.unsaved ? "đã áp dụng nhưng chưa Lưu vào flash" : "không còn thay đổi treo", "system");
  if (st.log && st.log.cap) {
    const free = 1 - st.log.size / st.log.cap;
    add(st.log.rate ? (free > 0.15 ? "ok" : "warn") : "off", "Blackbox", st.log.rate ? `${st.log.rate} Hz · còn trống ${(free * 100).toFixed(0)}%` : "đang tắt", "system");
  }
  if (typeof st.warn === "number" && st.warn & 512) add("warn", "Rung", "rung mạnh: kiểm tra cánh, ốc motor, đệm FC", "motors");
  return out;
}
function pfRender(st) {
  const host = $("pfAuto");
  if (!host) return;
  const items = pfItems(st);
  setHtml(host, items.map((it) => `<div class="pf-row ${it.state}"><i></i><div><b>${esc(it.title)}</b><small>${esc(it.detail)}</small></div>${it.go ? `<button type="button" class="btn ghost" data-go="${it.go}">Sửa</button>` : "<span></span>"}</div>`).join(""));
  const man = lsGet("aq-pf", {});
  const mh = $("pfManual");
  if (!mh.dataset.built) {
    mh.dataset.built = 1;
    mh.innerHTML = PF_MANUAL.map(([k, t]) => `<label class="pf-chk"><input type="checkbox" data-pf="${k}"><span>${t}</span></label>`).join("");
    mh.querySelectorAll("[data-pf]").forEach((c) => (c.onchange = () => { const m = lsGet("aq-pf", {}); m[c.dataset.pf] = c.checked ? Date.now() : 0; lsSet("aq-pf", m); pfRender(S.status); }));
    $("pfReset").onclick = () => { lsSet("aq-pf", {}); mh.querySelectorAll("[data-pf]").forEach((c) => (c.checked = false)); pfRender(S.status); };
  }
  // ticks older than 12 h do not count for a new flying day
  const fresh = (k) => man[k] && Date.now() - man[k] < 12 * 3600e3;
  mh.querySelectorAll("[data-pf]").forEach((c) => { if (document.activeElement !== c) c.checked = !!fresh(c.dataset.pf); });
  const bad = items.filter((i) => i.state === "bad").length, warn = items.filter((i) => i.state === "warn").length;
  const manOk = PF_MANUAL.filter(([k]) => fresh(k)).length;
  const total = items.filter((i) => i.state !== "off").length + PF_MANUAL.length;
  const good = items.filter((i) => i.state === "ok").length + manOk;
  const cls = bad ? "bad" : warn || manOk < PF_MANUAL.length ? "warn" : "ok";
  setHtml($("pfSum"), `<div class="pf-score ${cls}"><div class="pf-ring" style="--p:${Math.round(100 * good / Math.max(1, total))}"><b class="num">${good}/${total}</b></div>
    <div><h3>${bad ? `Chưa nên bay: ${bad} mục lỗi` : warn ? `Bay được, nhưng xem lại ${warn} mục cảnh báo` : manOk < PF_MANUAL.length ? "Tự kiểm tra đạt · còn mục kiểm tra bằng mắt" : "Sẵn sàng bay"}</h3>
    <p class="muted">${manOk}/${PF_MANUAL.length} mục kiểm tra bằng mắt đã tích (tự bỏ tích sau 12 giờ).</p></div></div>`);
}
document.addEventListener("click", (e) => {
  const b = e.target.closest && e.target.closest("#tab-preflight [data-go]");
  if (b) gotoTab(b.dataset.go);
});

// ---------------------------------------------------------------- review / undo pending changes, save reminder
const _proRefresh = refreshApplyBar;
refreshApplyBar = function () {
  _proRefresh();
  const k = Object.keys(S.edits).length, bar = $("applyBar");
  if (!$("reviewBtn")) {
    const r = document.createElement("button");
    r.className = "btn"; r.id = "reviewBtn"; r.type = "button"; r.textContent = "Xem lại";
    r.onclick = proReview;
    $("discardBtn").before(r);
    const sv = document.createElement("button");
    sv.className = "btn primary"; sv.id = "saveNowBtn"; sv.type = "button"; sv.textContent = "Lưu vào flash";
    sv.onclick = proSave;
    $("applyBtn").after(sv);
  }
  const unsavedOnly = !k && PRO.unsaved && !S.armed;
  bar.classList.toggle("show", k > 0 || unsavedOnly);
  bar.classList.toggle("saved-only", unsavedOnly);
  if (unsavedOnly) $("applyText").textContent = "Đã áp dụng, chưa lưu vào flash (mất khi tắt nguồn)";
  $("reviewBtn").hidden = !k; $("discardBtn").hidden = !k; $("applyBtn").hidden = !k;
  $("saveNowBtn").hidden = !unsavedOnly;
  const m = $("reviewDlg");
  if (m && !m.hidden) proReviewBody();
};
const _proApplied = typeof extrasApplied === "function" ? extrasApplied : null;
extrasApplied = function (changes) {
  PRO.unsaved = true;
  if (_proApplied) _proApplied(changes);
  setTimeout(refreshApplyBar, 0);
};
async function proSave() {
  try {
    await api("/api/save", { method: "POST" });
    PRO.unsaved = false;
    toast("Đã lưu vào flash.");
  } catch (e) { toast("Không lưu được: " + e.message); }
  refreshApplyBar();
}
{
  const o = $("saveBtn").onclick;
  $("saveBtn").onclick = async (e) => { await o(e); PRO.unsaved = false; refreshApplyBar(); };
}
function proReviewBody() {
  const m = $("reviewDlg"), rows = Object.entries(S.edits);
  const body = m.querySelector(".pro-dlg-b");
  if (!rows.length) { body.innerHTML = '<p class="muted">Không còn thay đổi nào.</p>'; return; }
  const reboot = rows.some(([n]) => S.params[n] && S.params[n].r);
  body.innerHTML = `<div class="rv-list">${rows.map(([n, v]) => {
    const old = S.params[n].v, d = v - old;
    return `<div class="rv-row"><div><b>${esc(proTitle(n))}</b><code>${n}</code></div><div class="rv-val num"><s>${esc(proValText(n, old))}</s><span>→</span><b>${esc(proValText(n, v))}</b>${!ENUMS[n] && Math.abs(old) > 1e-9 && !(S.params[n].max === 1 && S.params[n].min === 0) ? `<em class="${d > 0 ? "up" : "dn"}">${d > 0 ? "+" : ""}${(100 * d / Math.abs(old)).toFixed(0)}%</em>` : ""}</div><button type="button" class="btn ghost" data-undo="${n}">Hoàn tác</button></div>`;
  }).join("")}</div>
  ${reboot ? '<p class="note">Có tham số cần khởi động lại FC sau khi Lưu.</p>' : ""}
  <div class="row rv-foot"><button type="button" class="btn danger" id="rvDrop">Bỏ tất cả</button><span style="flex:1"></span><button type="button" class="btn" id="rvApply">Áp dụng</button><button type="button" class="btn primary" id="rvApplySave">Áp dụng & lưu flash</button></div>`;
  body.querySelectorAll("[data-undo]").forEach((b) => (b.onclick = () => { const n = b.dataset.undo; setEdit(n, S.params[n].v); }));
  $("rvDrop").onclick = () => { $("discardBtn").click(); m.hidden = true; };
  $("rvApply").onclick = async () => { await applyEdits(); m.hidden = true; };
  $("rvApplySave").onclick = async () => { await applyEdits(); if (!Object.keys(S.edits).length) await proSave(); m.hidden = true; };
}
function proReview() {
  const m = proModal("reviewDlg", `Xem lại ${Object.keys(S.edits).length} thay đổi`);
  proReviewBody();
  m.hidden = false;
}
window.addEventListener("beforeunload", (e) => {
  if (Object.keys(S.edits).length || PRO.unsaved) { e.preventDefault(); e.returnValue = ""; }
});

// ---------------------------------------------------------------- drone profiles (browser storage)
// calibration and identity values belong to one airframe: never copied between drones
const PROF_SKIP = /^(acc_|mag_bias|mag_scale|mag_decl|level_|board_|imu_align|mag_align|wifi|web_|mav_sysid|serial\d_)|_calibrated$/;
function profList() { return lsGet("aq-profiles", {}); }
function profHost() {
  if ($("profPanel") || !$("paramGroups")) return;
  const p = document.createElement("div");
  p.className = "panel"; p.id = "profPanel";
  p.innerHTML = `<div class="card-h"><span class="dot"></span><h3>Hồ sơ drone</h3><span class="card-x muted">lưu trong trình duyệt này</span></div>
    <p class="card-d">Mỗi hồ sơ giữ PID, rates, bộ lọc, chế độ, failsafe… (không gồm hiệu chuẩn cảm biến, WiFi, cổng). Nạp hồ sơ = đưa vào thay đổi chờ, xem lại rồi Áp dụng.</p>
    <div class="row"><input type="text" id="profName" placeholder="Tên, ví dụ: 5 inch freestyle" maxlength="40" style="flex:1;min-width:180px"><button type="button" class="btn primary" id="profSave">Lưu cấu hình hiện tại</button>
    <label class="btn" for="profImp" style="display:inline-flex;align-items:center">Nhập file</label><input type="file" id="profImp" accept=".json,application/json" hidden></div>
    <div id="profRows" style="margin-top:8px"></div>`;
  const anchor = $("pSearch");
  anchor.parentElement.insertBefore(p, anchor);
  $("profSave").onclick = () => {
    const name = $("profName").value.trim() || `Hồ sơ ${new Date().toLocaleString("vi-VN")}`;
    const vals = {};
    for (const n of S.order) if (!PROF_SKIP.test(n)) vals[n] = val(n);
    const L = profList();
    L[name] = { date: Date.now(), board: (S.lastHello && S.lastHello.board) || "", v: vals };
    lsSet("aq-profiles", L);
    $("profName").value = "";
    profRender();
    toast(`Đã lưu hồ sơ "${name}" (${Object.keys(vals).length} tham số).`);
  };
  $("profImp").onchange = async (e) => {
    const f = e.target.files[0];
    if (!f) return;
    try {
      const j = JSON.parse(await f.text());
      if (!j || typeof j.v !== "object") throw new Error("không phải file hồ sơ AllQuad");
      const L = profList();
      L[j.name || f.name.replace(/\.json$/, "")] = { date: j.date || Date.now(), board: j.board || "", v: j.v };
      lsSet("aq-profiles", L);
      profRender();
      toast("Đã nhập hồ sơ.");
    } catch (err) { toast("Không đọc được file: " + err.message); }
    e.target.value = "";
  };
  profRender();
}
function profRender() {
  const L = profList(), names = Object.keys(L).sort((a, b) => L[b].date - L[a].date);
  const host = $("profRows");
  if (!host) return;
  host.innerHTML = names.length ? names.map((n, i) => {
    const pr = L[n], diff = Object.entries(pr.v).filter(([k, v]) => S.params[k] && Math.abs(val(k) - v) > 1e-9).length;
    return `<div class="sdrow"><div><b>${esc(n)}</b> <span class="muted">${new Date(pr.date).toLocaleDateString("vi-VN")}${pr.board ? " · " + esc(pr.board) : ""} · ${diff ? diff + " khác hiện tại" : "trùng hiện tại"}</span></div>
      <span class="row"><button type="button" class="btn primary" data-pl="${i}"${diff ? "" : " disabled"}>Nạp</button><button type="button" class="btn" data-px="${i}">Xuất file</button><button type="button" class="btn danger" data-pd="${i}">Xoá</button></span></div>`;
  }).join("") : '<p class="muted">Chưa có hồ sơ nào.</p>';
  host.querySelectorAll("[data-pl]").forEach((b) => (b.onclick = () => {
    if (S.armed) return toast("Disarm trước khi nạp hồ sơ.");
    const pr = L[names[+b.dataset.pl]];
    let k = 0;
    for (const [n, v] of Object.entries(pr.v)) if (S.params[n] && !PROF_SKIP.test(n) && Math.abs(val(n) - v) > 1e-9) { setEdit(n, v); k++; }
    toast(`${k} thay đổi đã vào hàng chờ: bấm Xem lại.`);
    profRender();
  }));
  host.querySelectorAll("[data-px]").forEach((b) => (b.onclick = () => {
    const n = names[+b.dataset.px];
    const blob = new Blob([JSON.stringify({ name: n, ...L[n] }, null, 1)], { type: "application/json" });
    const a = document.createElement("a");
    a.href = URL.createObjectURL(blob); a.download = `allquad-${n.replace(/[^\w-]+/g, "_")}.json`; a.click();
    setTimeout(() => URL.revokeObjectURL(a.href), 2000);
  }));
  host.querySelectorAll("[data-pd]").forEach((b) => (b.onclick = () => {
    const n = names[+b.dataset.pd], M = profList();
    delete M[n]; lsSet("aq-profiles", M); profRender();
  }));
}

// ---------------------------------------------------------------- RC link history (tab Chế độ)
function rxLinkHost() {
  if ($("rxLinkPanel")) return rxLinkRender();
  const sec = $("tab-modes");
  if (!sec) return;
  const p = document.createElement("div");
  p.className = "panel"; p.id = "rxLinkPanel";
  p.innerHTML = `<div class="card-h"><span class="dot"></span><h3>Tín hiệu tay điều khiển</h3><span class="card-x num" id="rxNow"></span></div>
    <div class="rx-stats" id="rxStats"></div>
    <canvas class="chart" id="rxChart" style="height:150px"></canvas>
    <div class="legend"><span><i style="background:var(--pitch)"></i>LQ (%) 2 phút gần nhất</span><span><i style="background:var(--danger)"></i>mất sóng</span><button type="button" class="btn ghost" id="rxReset" style="margin-left:auto;min-height:28px;padding:2px 10px">Đặt lại thống kê</button></div>`;
  const h2 = sec.querySelector("h2");
  const after = h2 && h2.nextElementSibling && h2.nextElementSibling.matches("p") ? h2.nextElementSibling : h2;
  after.after(p);
  $("rxReset").onclick = () => { PRO.drops = 0; PRO.lqMin = 101; PRO.lossMax = 0; PRO.hist = []; rxLinkRender(); };
  rxLinkRender();
}
function rxLinkRender() {
  if (!$("rxChart")) return;
  const H = PRO.hist, now = Date.now();
  const last = H.length ? H[H.length - 1] : null;
  $("rxNow").textContent = !last ? "" : last[2] ? `LQ ${last[1]}%` : "MẤT SÓNG";
  $("rxNow").className = "card-x num" + (last && !last[2] ? " bad" : "");
  let sum = 0; for (const h of H) sum += h[1];
  const avg = H.length ? sum / H.length : 0;
  setHtml($("rxStats"), [
    ["Trung bình", H.length ? avg.toFixed(0) + "%" : "—"],
    ["Thấp nhất", PRO.lqMin <= 100 ? PRO.lqMin + "%" : "—"],
    ["Số lần mất sóng", String(PRO.drops)],
    ["Lần mất lâu nhất", PRO.lossMax ? (PRO.lossMax / 1000).toFixed(1) + " s" : "—"],
  ].map(([k, v]) => `<div><span>${k}</span><b class="num">${v}</b></div>`).join(""));
  const x = H.map((h) => (h[0] - now) / 1000), y = H.map((h) => h[1]), lost = H.map((h) => (h[2] ? NaN : 0));
  plot($("rxChart"), [{ x, y, color: "--pitch", width: 2 }, { x, y: lost, color: "--danger", width: 5 }], { xmin: -120, xmax: 0, ymin: 0, ymax: 100 });
}

// ---------------------------------------------------------------- blackbox: noise heatmap + events
const MODE_NAMES = ["ACRO", "ANGLE", "HORIZON", "ALTHOLD", "POSHOLD", "RTH", "MISSION", "LAND", "FAILSAFE", "TURTLE", "CRUISE", "AUTOTUNE", "GUIDED", "SMARTRTL"];
function bbProHost() {
  if ($("bbHeatPanel") || !$("specChart")) return;
  const spec = $("specChart").closest(".panel");
  const p = document.createElement("div");
  p.className = "panel"; p.id = "bbHeatPanel";
  p.innerHTML = `<div class="card-h"><span class="dot"></span><h3>Nhiễu theo mức ga</h3><span class="card-x muted">trục theo nút Roll / Pitch / Yaw ở Phổ nhiễu</span></div>
    <p class="card-d">Gyro thô: hàng = mức ga, cột = tần số, màu sáng = nhiễu mạnh. Vệt chéo đi lên theo ga là nhiễu motor (RPM filter xử lý); vệt đứng yên là cộng hưởng khung (notch tĩnh / động).</p>
    <canvas id="heatChart" class="chart" style="height:240px"></canvas>`;
  spec.after(p);
  const e = document.createElement("div");
  e.className = "panel"; e.id = "bbEventPanel";
  e.innerHTML = `<div class="card-h"><span class="dot"></span><h3>Sự kiện trong chuyến bay</h3><span class="card-x muted">bấm để xem đoạn đó</span></div><div id="bbEvents"></div>`;
  const ts = $("tsChart").closest(".panel");
  ts.before(e);
}
function bbHeat(fl, a) {
  const key = a;
  if (PRO.heat[key]) return PRO.heat[key];
  const rows = 10, n = fl.n, nfft = 256;
  const out = { rows: [], f: null, max: 0 };
  for (let r = 0; r < rows; r++) {
    const lo = r / rows, hi = (r + 1) / rows;
    const mask = new Uint8Array(n);
    let cnt = 0;
    for (let k = 0; k < n; k++) if (fl.armed[k] && fl.thr[k] >= lo && fl.thr[k] < hi) { mask[k] = 1; cnt++; }
    if (cnt < nfft * 2) { out.rows.push(null); continue; }
    const sp = AQ.spectrum(fl.gyroRaw[a], fl.rateHz, nfft, mask);
    if (!sp.segments) { out.rows.push(null); continue; }
    out.f = sp.f;
    out.rows.push(sp.amp);
    for (let k = 2; k < sp.amp.length; k++) out.max = Math.max(out.max, sp.amp[k]);
  }
  return (PRO.heat[key] = out);
}
function heatColor(t) {  // dark blue -> violet -> orange -> yellow
  const stops = [[13, 23, 34], [46, 54, 140], [160, 50, 140], [235, 110, 40], [250, 225, 90]];
  const x = Math.max(0, Math.min(1, t)) * (stops.length - 1), i = Math.min(stops.length - 2, Math.floor(x)), u = x - i;
  const c = stops[i].map((v, k) => Math.round(v + (stops[i + 1][k] - v) * u));
  return `rgb(${c[0]},${c[1]},${c[2]})`;
}
function drawHeat() {
  const cv = $("heatChart"), fl = S.flight;
  if (!cv || !fl || $("tab-blackbox").hidden) return;
  const H = bbHeat(fl, S.specAxis || 0);
  const { g, w, h } = setupCanvas(cv);
  const L = 44, R = 10, T = 8, B = 24;
  g.clearRect(0, 0, w, h);
  g.fillStyle = "#AFC0D2"; g.font = "11px system-ui";
  if (!H.f) { g.fillText("Log quá ngắn hoặc không có đoạn đang arm.", L, h / 2); return; }
  const fmax = Math.min(H.f[H.f.length - 1], 1000);
  const kmax = H.f.findIndex((f) => f > fmax); const K = kmax > 0 ? kmax : H.f.length;
  const cw = (w - L - R) / K, rh = (h - T - B) / H.rows.length;
  const lmax = Math.log10(H.max + 1e-6), lmin = lmax - 2.5;
  H.rows.forEach((row, r) => {
    const y = T + (H.rows.length - 1 - r) * rh;
    if (!row) { g.fillStyle = "rgba(255,255,255,.05)"; g.fillRect(L, y, w - L - R, rh - 1); return; }
    for (let k = 1; k < K; k++) {
      g.fillStyle = heatColor((Math.log10(row[k] + 1e-6) - lmin) / (lmax - lmin));
      g.fillRect(L + k * cw, y, Math.ceil(cw) + 0.5, rh - 1);
    }
  });
  g.fillStyle = "#AFC0D2";
  g.textAlign = "right"; g.textBaseline = "middle";
  for (let r = 0; r <= H.rows.length; r += 2) g.fillText(`${r * 10}%`, L - 6, T + (H.rows.length - r) * rh);
  g.textAlign = "center"; g.textBaseline = "top";
  const step = niceStep(fmax, 6);
  for (let f = 0; f <= fmax; f += step) g.fillText(`${f} Hz`, L + (f / H.f[1]) * cw, h - B + 6);
  // filter cut-offs for reference
  const lines = [["gyro_lpf_hz", "--yaw"], ["dyn_notch_min", "--amber"], ["dyn_notch_max", "--amber"]];
  for (const [n, c] of lines) if (S.params[n] && val(n) > 0 && val(n) < fmax) {
    const x = L + (val(n) / H.f[1]) * cw;
    g.strokeStyle = css(c); g.setLineDash([4, 4]); g.lineWidth = 1.5;
    g.beginPath(); g.moveTo(x, T); g.lineTo(x, h - B); g.stroke(); g.setLineDash([]);
  }
}
function bbEvents(fl) {
  const ev = [], t0 = fl.t[0];
  let lastArm = fl.armed[0], lastMode = fl.mode[0], satStart = -1, vmin = Infinity, vminT = 0, gyroPeak = 0, gyroT = 0;
  for (let k = 1; k < fl.n; k++) {
    const t = fl.t[k] - t0;
    if (fl.armed[k] !== lastArm) { ev.push([t, fl.armed[k] ? "arm" : "disarm", fl.armed[k] ? "ARM" : "DISARM"]); lastArm = fl.armed[k]; }
    if (fl.mode[k] !== lastMode) { ev.push([t, "mode", `Chế độ ${MODE_NAMES[fl.mode[k]] || fl.mode[k]}`]); lastMode = fl.mode[k]; }
    const sat = fl.armed[k] && (fl.motor[0][k] > 0.98 || fl.motor[1][k] > 0.98 || fl.motor[2][k] > 0.98 || fl.motor[3][k] > 0.98);
    if (sat && satStart < 0) satStart = t;
    if (!sat && satStart >= 0) { if (t - satStart > 0.05) ev.push([satStart, "warn", `Motor chạm 100% trong ${((t - satStart) * 1000).toFixed(0)} ms`]); satStart = -1; }
    if (fl.armed[k] && fl.vbat[k] > 1 && fl.vbat[k] < vmin) { vmin = fl.vbat[k]; vminT = t; }
    for (let a = 0; a < 3; a++) { const g = Math.abs(fl.gyro[a][k]); if (g > gyroPeak) { gyroPeak = g; gyroT = t; } }
  }
  if (vmin < Infinity) ev.push([vminT, "info", `Điện áp thấp nhất ${vmin.toFixed(2)} V`]);
  if (gyroPeak > 0) ev.push([gyroT, gyroPeak > 1500 ? "warn" : "info", `Tốc độ quay lớn nhất ${gyroPeak.toFixed(0)} °/s`]);
  ev.sort((x, y) => x[0] - y[0]);
  // a busy log: keep the first 60
  return ev.slice(0, 60);
}
function bbEventsRender() {
  const host = $("bbEvents"), fl = S.flight;
  if (!host || !fl) return;
  const ev = PRO.events;
  host.innerHTML = ev.length ? `<div class="ev-bar">${ev.map(([t, k], i) => `<i class="${k}" style="left:${(100 * t / Math.max(0.001, fl.duration)).toFixed(2)}%" data-ev="${i}"></i>`).join("")}</div>
    <div class="ev-list">${ev.map(([t, k, s], i) => `<button type="button" class="ev ${k}" data-ev="${i}"><span class="num">${t.toFixed(1)} s</span>${esc(s)}</button>`).join("")}</div>` : '<p class="muted">Không có sự kiện đáng chú ý.</p>';
  host.querySelectorAll("[data-ev]").forEach((b) => (b.onclick = () => {
    const t = ev[+b.dataset.ev][0], span = fl.duration * (+$("tsZoom").value / 100);
    $("tsPos").value = fl.duration > span ? Math.round(Math.max(0, Math.min(1, (t - span / 3) / (fl.duration - span))) * 1000) : 0;
    drawTimeSeries();
    $("tsChart").scrollIntoView({ behavior: "smooth", block: "center" });
  }));
}
{
  const _af = analyzeFlight;
  analyzeFlight = function (i) {
    _af(i);
    PRO.heat = {};
    bbProHost();
    PRO.events = S.flight ? bbEvents(S.flight) : [];
    bbEventsRender();
    drawHeat();
  };
  const _db = drawBlackbox;
  drawBlackbox = function () { _db(); drawHeat(); };
}

// ---------------------------------------------------------------- mission: distance / time / battery
function wpStats() {
  if (!S.wp) return;
  let host = $("wpStats");
  if (!host) {
    host = document.createElement("div");
    host.id = "wpStats"; host.className = "wp-stats";
    $("wpList").before(host);
  }
  const spd = S.params.wp_speed ? val("wp_speed") : 5, climb = S.params.climb_rate_max ? val("climb_rate_max") : 2;
  let n = 0, e = 0, alt = 0, dist = 0, time = 0, maxD = 0, maxAlt = 0, legs = 0;
  for (const it of S.wp.items) {
    if (it.cmd === 22) { time += Math.abs(it.alt - alt) / climb; alt = it.alt; }
    if (it.pos) {
      const d = Math.hypot(it.n - n, it.e - e), dz = Math.abs((it.alt || alt) - alt);
      dist += d; time += d / spd + dz / climb; legs++;
      n = it.n; e = it.e; alt = it.alt || alt;
    }
    if (it.cmd === 19 || (it.cmd === 16 && it.p1)) time += +it.p1 || 0;
    if (it.cmd === 18) time += Math.max(1, +it.p1 || 1) * 2 * Math.PI * Math.abs(+it.p3 || 10) / spd;
    if (it.cmd === 20) { const d = Math.hypot(n, e); dist += d; time += d / spd + alt / 1; n = e = 0; }
    if (it.cmd === 21) time += alt / 1;
    maxD = Math.max(maxD, Math.hypot(n, e)); maxAlt = Math.max(maxAlt, alt);
  }
  const amps = PC && PC.amps > 1 ? PC.amps : null;
  const mah = amps ? amps * time / 3.6 : null;
  const cap = S.params.bat_capacity ? val("bat_capacity") : 0;
  setHtml(host, [
    ["Quãng đường", dist >= 1000 ? (dist / 1000).toFixed(2) + " km" : dist.toFixed(0) + " m"],
    ["Thời gian ước tính", `${Math.floor(time / 60)}:${String(Math.round(time % 60)).padStart(2, "0")}`],
    ["Xa nhà nhất", maxD.toFixed(0) + " m"],
    ["Cao nhất", maxAlt.toFixed(0) + " m"],
    ["Pin cần", mah ? `${mah.toFixed(0)} mAh${cap ? ` (${(100 * mah / cap).toFixed(0)}%)` : ""}` : "— (cần đo dòng khi bay)"],
  ].map(([k, v]) => `<div><span>${k}</span><b class="num">${v}</b></div>`).join("") +
    (cap && mah && mah > cap * 0.7 ? '<p class="note bad" style="grid-column:1/-1;margin:4px 0 0">Nhiệm vụ tốn hơn 70% pin: rút ngắn hoặc bay chậm hơn.</p>' : ""));
}
{
  const _wl = wpList;
  wpList = function () { _wl(); wpStats(); };
}

// ---------------------------------------------------------------- global search (Ctrl+K)
const SEARCH_ACTIONS = [
  ["Hiệu chuẩn accel / compass / gyro", "sensors", "calib accel compass hieu chuan"],
  ["Thử motor (tháo cánh)", "motors", "motor test thu"],
  ["Thứ tự & chiều quay motor", "motors", "motor order direction chieu quay"],
  ["ESC passthrough / BLHeli", "motors", "esc blheli am32 bluejay"],
  ["Gán chế độ cho công tắc", "modes", "mode switch aux cong tac"],
  ["Failsafe & pin", "modes", "failsafe pin battery"],
  ["Bản đồ & nhiệm vụ waypoint", "nav", "mission waypoint map ban do"],
  ["Cổng serial, VTX, LED", "ports", "uart serial vtx led"],
  ["Bố cục OSD", "osd", "osd"],
  ["Phân tích blackbox", "blackbox", "blackbox log pid tune"],
  ["Lưu vào flash, sao lưu, nạp firmware", "system", "save backup restore firmware flash ota"],
  ["Kiểm tra trước khi bay", "preflight", "preflight checklist truoc bay"],
  ["Thiết lập phần cứng (từng bước)", "setup", "setup wizard thiet lap"],
  ["CLI", "cli", "cli terminal"],
];
const noAcc = (s) => s.normalize("NFD").replace(/[̀-ͯ]/g, "").replace(/đ/g, "d").replace(/Đ/g, "D").toLowerCase();
function searchOpen() {
  const m = proModal("searchDlg", "Tìm tham số hoặc chức năng",
    `<input type="search" id="gsIn" placeholder="ví dụ: failsafe, notch, rth_alt, motor…" autocomplete="off"><div id="gsRes" class="gs-res"></div><p class="muted gs-tip">↑ ↓ để chọn · Enter để mở · Esc để đóng</p>`);
  m.hidden = false;
  const inp = $("gsIn");
  inp.value = "";
  inp.oninput = searchRun;
  inp.onkeydown = (e) => {
    const items = [...$("gsRes").querySelectorAll(".gs-it")];
    let i = items.findIndex((x) => x.classList.contains("on"));
    if (e.key === "ArrowDown" || e.key === "ArrowUp") {
      e.preventDefault();
      i = e.key === "ArrowDown" ? Math.min(items.length - 1, i + 1) : Math.max(0, i - 1);
      items.forEach((x, k) => x.classList.toggle("on", k === i));
      if (items[i]) items[i].scrollIntoView({ block: "nearest" });
    }
    if (e.key === "Enter" && items[Math.max(0, i)]) items[Math.max(0, i)].click();
  };
  searchRun();
  setTimeout(() => inp.focus(), 30);
}
function searchRun() {
  const q = noAcc($("gsIn").value.trim());
  const res = [];
  for (const [t, tab, kw] of SEARCH_ACTIONS) if (!q || noAcc(t + " " + kw).includes(q)) res.push({ kind: "tab", t, tab });
  if (q) for (const n of S.order) {
    const title = proTitle(n);
    if (noAcc(n).includes(q) || noAcc(title).includes(q)) res.push({ kind: "p", n, t: title });
    if (res.length > 40) break;
  }
  const host = $("gsRes");
  host.innerHTML = res.length ? res.slice(0, 40).map((r, i) => r.kind === "tab"
    ? `<button type="button" class="gs-it${i ? "" : " on"}" data-tab="${r.tab}"><i>→</i><span>${esc(r.t)}</span></button>`
    : `<button type="button" class="gs-it${i ? "" : " on"}" data-n="${r.n}"><i>#</i><span>${esc(r.t)}</span><code>${r.n}</code><b class="num">${esc(proValText(r.n, val(r.n)))}</b></button>`).join("")
    : '<p class="muted">Không tìm thấy.</p>';
  host.querySelectorAll(".gs-it").forEach((b) => (b.onclick = () => {
    $("searchDlg").hidden = true;
    if (b.dataset.tab) return gotoTab(b.dataset.tab);
    searchGo(b.dataset.n);
  }));
}
function searchGo(n) {
  // the row on a regular tab first, the full parameter list otherwise
  let row = null;
  for (const r of document.querySelectorAll(`main > section:not(#tab-params) .prow[data-name="${n}"]`)) { row = r; break; }
  if (row && document.body.classList.contains("simple") && row.classList.contains("adv")) setSimple(false);
  if (row) {
    gotoTab(row.closest("section").id.replace("tab-", ""));
    const d = row.closest("details"); if (d) d.open = true;
  } else {
    gotoTab("params");
    $("pSearch").value = n;
    $("pSearch").dispatchEvent(new Event("input"));
    row = document.querySelector(`#paramGroups .prow[data-name="${n}"]`);
  }
  if (!row) return;
  setTimeout(() => {
    row.scrollIntoView({ behavior: "smooth", block: "center" });
    row.classList.remove("flash"); void row.offsetWidth; row.classList.add("flash");
    const c = row.querySelector("input, select"); if (c) c.focus({ preventScroll: true });
  }, 60);
}
$("searchBtn").onclick = searchOpen;
document.addEventListener("keydown", (e) => {
  const typing = /^(INPUT|TEXTAREA|SELECT)$/.test(document.activeElement && document.activeElement.tagName);
  if ((e.key === "k" && (e.ctrlKey || e.metaKey)) || (e.key === "/" && !typing)) { e.preventDefault(); searchOpen(); }
});

// ---------------------------------------------------------------- simple / advanced view
// simple: the settings most pilots touch; everything stays reachable in Tham số and with the search
const BASIC = new Set(("fs_action fs_delay_ms bat_fs_action cell_count vbat_warn_cell vbat_crit_cell bat_capacity bat_reserve_pct vbat_scale cur_scale " +
  "arm_throttle_max arm_max_angle arm_require_gps angle_limit airmode rth_alt rth_speed rth_land land_speed fence_enable fence_radius fence_alt_max " +
  "motor_protocol motor_idle dshot_bidir yaw_reversed gyro_lpf_hz dterm_lpf_hz dyn_notch_count rpm_harmonics loop_hz bb_rate_hz bb_backend " +
  "wifi_enable web_enable crash_detect rc_smooth_hz pid_profile hover_learn wp_speed wp_radius osd_backend").split(" "));
function markAdv() {
  document.querySelectorAll("main > section:not(#tab-params) .prow[data-name]").forEach((r) => r.classList.toggle("adv", !BASIC.has(r.dataset.name)));
  document.querySelectorAll("main > section:not(#tab-params) .panel").forEach((p) => {
    const rows = p.querySelectorAll(".prow[data-name]");
    if (!rows.length) return;
    const onlyRows = !p.querySelector("canvas, .stepper, table, .mrow, button.btn:not(.ghost)");
    p.classList.toggle("adv-only", onlyRows && [...rows].every((r) => r.classList.contains("adv")));
  });
}
function setSimple(on) {
  document.body.classList.toggle("simple", on);
  $("viewBtn").setAttribute("aria-pressed", String(on));
  $("viewTxt").textContent = on ? "Đơn giản" : "Nâng cao";
  lsSet("aq-simple", on);
  markAdv();
}
$("viewBtn").onclick = () => { setSimple(!document.body.classList.contains("simple")); toast(document.body.classList.contains("simple") ? "Chế độ đơn giản: chỉ hiện các thông số chính. Tab Tham số vẫn đủ." : "Chế độ nâng cao: hiện mọi thông số."); };
{
  const _bp = buildPid;
  buildPid = function () { _bp(); markAdv(); if (shown("params")) profHost(); };
}

// ---------------------------------------------------------------- theme: auto / light / dark / outdoor
const THEMES = [["", "Tự động"], ["light", "Sáng"], ["dark", "Tối"], ["sun", "Ngoài trời"]];
function themeShow() {
  const t = document.documentElement.dataset.theme || "";
  $("themeTxt").textContent = (THEMES.find(([k]) => k === t) || THEMES[0])[1];
}
$("themeQuick").onclick = () => {
  const r = document.documentElement, cur = r.dataset.theme || "";
  const i = (THEMES.findIndex(([k]) => k === cur) + 1) % THEMES.length;
  if (THEMES[i][0]) r.dataset.theme = THEMES[i][0]; else delete r.dataset.theme;
  try { localStorage.setItem("aq-theme", THEMES[i][0]); } catch (e) {}
  themeShow();
  drawRates(); drawFilters(); drawBlackbox();
  if (S.status) renderStatus(S.status);
};
themeShow();

// ---------------------------------------------------------------- compact sidebar (laptops)
{
  const nav = document.querySelector("nav.tabs");
  const b = document.createElement("button");
  b.type = "button"; b.className = "navmini-btn"; b.id = "navMiniBtn";
  b.innerHTML = '<svg viewBox="0 0 24 24"><path d="M15 6l-6 6 6 6"/></svg><span>Thu gọn</span>';
  b.title = "Thu gọn / mở rộng thanh bên";
  b.onclick = () => { const on = !document.body.classList.contains("navmini"); document.body.classList.toggle("navmini", on); lsSet("aq-navmini", on); setTimeout(() => { drawRates(); drawFilters(); drawBlackbox(); }, 50); };
  nav.appendChild(b);
  if (lsGet("aq-navmini", false)) document.body.classList.add("navmini");
  if (lsGet("aq-simple", false)) setSimple(true);
}

// ---------------------------------------------------------------- USB through WebUSB (phones)
// Chrome on Android before Web Serial, and other browsers with WebUSB: talk to the USB CDC port of the
// STM32 (0483:5740) or ESP32-S3 (303A) directly. CP210x / CH340 bridge chips are not CDC: not supported.
const USB_FILTERS = [{ vendorId: 0x0483 }, { vendorId: 0x303A }, { classCode: 2 }];
async function usbConnect(dev) {
  connBusy("Đang mở USB…");
  try {
    await dev.open();
    if (!dev.configuration) await dev.selectConfiguration(1);
    let dataIf = null, commIf = null, epIn = 0, epOut = 0;
    for (const itf of dev.configuration.interfaces) {
      const alt = itf.alternate || itf.alternates[0];
      if (alt.interfaceClass === 10 && dataIf === null) {
        dataIf = itf.interfaceNumber;
        for (const ep of alt.endpoints) if (ep.type === "bulk") { if (ep.direction === "in") epIn = ep.endpointNumber; else epOut = ep.endpointNumber; }
      }
      if (alt.interfaceClass === 2 && commIf === null) commIf = itf.interfaceNumber;
    }
    if (dataIf === null || !epIn || !epOut) throw new Error("thiết bị không phải cổng COM USB (CDC)");
    if (commIf !== null) { try { await dev.claimInterface(commIf); } catch (e) {} }
    await dev.claimInterface(dataIf);
    if (commIf !== null) {
      // 921600 8N1, then DTR + RTS: the port counts as opened
      try { await dev.controlTransferOut({ requestType: "class", recipient: "interface", request: 0x20, value: 0, index: commIf }, new Uint8Array([0x00, 0x10, 0x0E, 0x00, 0, 0, 8])); } catch (e) {}
      try { await dev.controlTransferOut({ requestType: "class", recipient: "interface", request: 0x22, value: 3, index: commIf }); } catch (e) {}
    }
    LINK.usbdev = dev; LINK.port = null; LINK.kind = "usb"; LINK.baud = 921600; LINK.rx = new Uint8Array(0); LINK.closing = false;
    LINK.send = async (f) => { for (let o = 0; o < f.length; o += 4096) await dev.transferOut(epOut, f.subarray(o, o + 4096)); };
    LINK.open = true;
    (async () => {
      while (LINK.usbdev === dev && !LINK.closing) {
        try {
          const r = await dev.transferIn(epIn, 512);
          if (r.data && r.data.byteLength) linkFeed(new Uint8Array(r.data.buffer.slice(r.data.byteOffset, r.data.byteOffset + r.data.byteLength)));
        } catch (e) { break; }
      }
      if (LINK.usbdev === dev && !LINK.closing) linkLost("Mất kết nối USB. Cắm lại cáp rồi bấm Kết nối USB.");
    })();
    const h = await hello(6);
    connBusy("");
    return connected(h, "USB");
  } catch (e) {
    await closeLink();
    connBusy("");
    showConnect(/Access denied|claim/i.test(e.message || "") ? "Cổng USB đang bị ứng dụng khác giữ (hoặc Android chưa cho phép): rút cáp, cắm lại rồi thử." : "Không mở được USB: " + (e.message || e), true);
  }
}
{
  const _sc = showConnect;
  showConnect = function (msg, bad) {
    _sc(msg, bad);
    const webusb = "usb" in navigator, serial = "serial" in navigator;
    $("usbPick").hidden = !webusb || serial;
    if (!serial && webusb) {
      $("comPick").hidden = true;
      const err = $("comErr");
      if (!msg) err.hidden = true;
    }
    if (!serial && !webusb && !msg) $("comErr").textContent = "Trình duyệt này không kết nối USB được. Dùng Chrome trên Android (cáp OTG) hoặc Chrome / Edge trên máy tính.";
  };
  $("usbPick").onclick = async () => {
    let dev;
    try { dev = await navigator.usb.requestDevice({ filters: USB_FILTERS }); } catch (e) { return; }
    usbConnect(dev);
  };
  if ("usb" in navigator && !("serial" in navigator)) {
    navigator.usb.addEventListener("disconnect", (ev) => { if (LINK.usbdev === ev.device) linkLost("Đã rút cáp USB."); });
    navigator.usb.addEventListener("connect", (ev) => { if (!LINK.open && !$("connect").hidden) usbConnect(ev.device); });
  }
}
