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
    pill.textContent = `${LINK.open ? (LINK.kind === "usb" ? "USB" : LINK.kind === "bridge" ? "Cầu WiFi" : LINK.kind.toUpperCase()) : "WiFi"} · ${hz.toFixed(hz < 10 ? 1 : 0)} Hz`;
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
  if (st.ports) portStatsRender(st);
  crashRender(st);
}
function proTabShown(t) {
  if (t === "preflight") pfRender(S.status);
  if (t === "modes") rxLinkHost();
  if (t === "ports") portStatsRender(S.status);
  if (t === "system") crashRender(S.status);
  if (t === "params") profHost();
  if (t === "nav" && S.wp) wpStats();
  if (t === "pid") { tuneHost(); tuneRender(); }
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
  if (st.crash && st.crash.ago === 0) add("bad", "Lần khởi động trước", `FC tự reset vì lỗi: ${CRASH_VI[st.crash.kind] || st.crash.kind} (${st.crash.where || "?"}) - xem tab Hệ thống`, "system");
  else if (st.rr === 9) add("warn", "Lần khởi động trước", "sụt áp (brownout): nguồn 5 V / 3.3 V yếu hoặc dây USB kém", "system");
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
  if ($("profHost")) $("profHost").appendChild(p);
  else $("pSearch").parentElement.insertBefore(p, $("pSearch"));
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

// ---------------------------------------------------------------- reset reason, crash record, watchdog (System tab)
const RR_VI = { 0: "không rõ", 1: "cấp nguồn", 2: "nút reset", 3: "phần mềm (lệnh reboot / nạp firmware)", 4: "LỖI FIRMWARE (crash)",
  5: "watchdog ngắt", 6: "watchdog task", 7: "WATCHDOG (firmware bị treo)", 8: "thức dậy từ ngủ sâu", 9: "SỤT ÁP (brownout)", 10: "SDIO" };
const CRASH_VI = { "hard fault": "hard fault", "memory fault": "truy cập bộ nhớ sai", "bus fault": "lỗi bus", "usage fault": "lệnh không hợp lệ",
  "stack overflow": "tràn stack", "out of memory": "hết RAM", assert: "kiểm tra nội bộ thất bại (assert)", watchdog: "treo (watchdog)" };
function crashHost() {
  if ($("relPanel")) return true;
  const sec = $("tab-system");
  if (!sec || !sec.querySelector(".panel")) return false;
  const p = document.createElement("div");
  p.className = "panel"; p.id = "relPanel";
  p.innerHTML = `<div class="card-h"><span class="dot"></span><h3>Khởi động &amp; sự cố</h3><span class="card-x num" id="relWdt"></span></div>
    <div class="rx-stats" id="relStats"></div><div id="relCrash"></div>`;
  sec.querySelector(".panel").after(p);
  return true;
}
// WiFi bridge (page served by the ESP32-C3 bridge): its access point name / password
async function bridgeHost() {
  if (LINK.kind !== "bridge" || $("bridgePanel") || !$("relPanel")) return;
  const p = document.createElement("div");
  p.className = "panel"; p.id = "bridgePanel";
  p.innerHTML = `<div class="card-h"><span class="dot"></span><h3>Cầu WiFi</h3><span class="card-x muted" id="brInfo"></span></div>
    <p class="card-d" id="brWarn"></p>
    <div class="fields"><div class="field"><label>Tên WiFi của cầu</label><input id="brSsid" maxlength="32" autocomplete="off"></div>
    <div class="field"><label>Mật khẩu mới (8-63 ký tự)</label><input id="brPass" type="password" maxlength="63" autocomplete="new-password"></div></div>
    <div class="row" style="margin-top:8px"><button type="button" class="btn" id="brSave">Lưu vào cầu (cầu khởi động lại)</button></div>`;
  $("relPanel").after(p);
  try {
    const j = await (await fetch("/bridge/info", { cache: "no-store" })).json();
    $("brSsid").value = j.ssid || "";
    $("brInfo").textContent = `firmware cầu ${j.fw} · UART ${j.baud} baud`;
    $("brWarn").textContent = j.defpw ? "Cầu đang dùng mật khẩu mặc định allquad123: ai ở gần cũng vào được trang cấu hình. Đổi ngay." : "Điện thoại / máy tính nối WiFi này rồi mở http://192.168.4.1";
    $("brWarn").className = j.defpw ? "note bad" : "card-d";
  } catch (e) { $("brInfo").textContent = "không đọc được thông tin cầu"; }
  $("brSave").onclick = async () => {
    const s = $("brSsid").value.trim(), pw = $("brPass").value;
    if (!s || s.length > 32 || pw.length < 8 || pw.length > 63) { toast("Tên 1-32 ký tự, mật khẩu 8-63 ký tự"); return; }
    try {
      const r = await fetch("/bridge/wifi", { method: "POST", body: s + "\n" + pw });
      toast(r.ok ? "Đã lưu: nối lại vào WiFi \"" + s + "\" với mật khẩu mới" : await r.text());
    } catch (e) { toast("Cầu đang khởi động lại: nối lại WiFi \"" + s + "\""); }
  };
}
function crashRender(st) {
  if (!st || !shown("system") || !crashHost()) return;
  bridgeHost();
  const c = st.crash;
  $("relWdt").textContent = st.wdt === true ? "watchdog: đang chạy" : st.wdt === false && st.rr !== undefined ? "watchdog: chưa chạy / ESP32 dùng watchdog riêng" : "";
  setHtml($("relStats"), [
    ["Lần reset gần nhất", st.rr !== undefined ? RR_VI[st.rr] || String(st.rr) : "—"],
    ["Bản ghi crash", c ? `${c.n} lần` : "không có"],
  ].map(([k, v]) => `<div><span>${k}</span><b class="${(st.rr === 4 || st.rr === 7 || st.rr === 9) && k[0] === "L" ? "bad" : ""}">${esc(v)}</b></div>`).join(""));
  setHtml($("relCrash"), !c ? `<p class="muted" style="margin-bottom:0">Không có lỗi nào được ghi từ lúc cấp nguồn. Bản ghi nằm trong RAM: mất khi rút nguồn.</p>` :
    `<div class="note${c.ago === 0 ? " bad" : ""}" style="margin-top:8px"><b>${esc(CRASH_VI[c.kind] || c.kind)}</b> ${c.ago === 0 ? "đã làm FC khởi động lại lần này" : `ở một lần chạy trước (${c.ago} lần khởi động trước)`},
      sau ${(c.t / 1000).toFixed(1)} s, trong <b>${esc(c.where || "?")}</b>.
      ${c.pc && c.pc !== "0x00000000" ? `<br><span class="num muted" style="font-size:12px">pc ${esc(c.pc)} · lr ${esc(c.lr)} · cfsr ${esc(c.cfsr)} - gửi kèm khi báo lỗi (arm-none-eabi-addr2line -e firmware.elf ${esc(c.pc)})</span>` : ""}
      <div class="row" style="margin-top:8px"><button type="button" class="btn ghost" id="relCopy">Chép thông tin</button><button type="button" class="btn ghost" id="relClear">Xoá bản ghi</button></div></div>`);
  if ($("relClear")) $("relClear").onclick = async () => { try { await api("/api/cli", { method: "POST", body: "crash clear", text: true, timeout: 4000 }); toast("Đã xoá bản ghi crash"); } catch (e) { toast("Lỗi: " + e.message, true); } };
  if ($("relCopy")) $("relCopy").onclick = () => {
    const t = `AllQuad ${st.fw && st.fw.version || ""} crash: ${c.kind} in ${c.where} after ${c.t} ms, pc ${c.pc} lr ${c.lr} cfsr ${c.cfsr}, ${c.n}x, boots ago ${c.ago}`;
    try { navigator.clipboard.writeText(t); toast("Đã chép"); } catch (e) { toast(t); }
  };
}

// ---------------------------------------------------------------- per-UART traffic and errors (Ports tab)
// status.ports: [[hal port, rx bytes, tx bytes, overruns, framing, noise, dropped], ...]; status.rxc: receiver
// [bytes, good frames, checksum errors]. Counters run from boot; the panel shows rates and what changed
// since it was opened (or "Đặt lại").
const PS = { last: null, base: null, rate: {} };
function portStatsHost() {
  if ($("portStatsPanel")) return true;
  const t = $("portTable");
  if (!t) return false;
  const p = document.createElement("div");
  p.className = "panel"; p.id = "portStatsPanel";
  p.innerHTML = `<div class="card-h"><span class="dot"></span><h3>Lưu lượng &amp; lỗi đường truyền</h3><button type="button" class="btn ghost" id="psReset" style="margin-left:auto;min-height:28px;padding:2px 10px">Đặt lại</button></div>
    <table class="ports" id="psTable"></table><p class="muted" id="psNote" style="margin-bottom:0"></p>`;
  t.closest(".panel").after(p);
  $("psReset").onclick = () => { PS.base = PS.last; psDraw(); };
  return true;
}
function portStatsRender(st) {
  if (!st || !Array.isArray(st.ports)) return;
  // WebSocket frames between the once-a-second full ones repeat the same array: nothing new to count
  if (st.ports !== PS.ref) { PS.ref = st.ports; psCount(st); }
  if (shown("ports") && portStatsHost()) psDraw();
}
function psCount(st) {
  const now = Date.now(), cur = { t: now, p: {}, rxc: Array.isArray(st.rxc) ? st.rxc : null, gpsl: Array.isArray(st.gpsl) ? st.gpsl : null,
    snsl: Array.isArray(st.snsl) ? st.snsl : null };
  for (const r of st.ports) if (Array.isArray(r) && r.length >= 7) cur.p[r[0]] = r;
  const prev = PS.last;
  if (prev && now - prev.t >= 300) {
    const dt = (now - prev.t) / 1000;
    for (const k in cur.p) {
      const a = cur.p[k], b = prev.p[k];
      if (b && a[1] >= b[1] && a[2] >= b[2]) PS.rate[k] = [(a[1] - b[1]) / dt, (a[2] - b[2]) / dt];
    }
  }
  if (!prev || now - prev.t >= 300) PS.last = cur;
  // counters restart when the FC reboots: start over
  if (PS.base) for (const k in cur.p) { const b = PS.base.p[k]; if (b && cur.p[k][1] < b[1]) PS.base = null; }
  if (PS.base && PS.base.rxc && cur.rxc && cur.rxc[0] < PS.base.rxc[0]) PS.base = null;
  if (!PS.base) PS.base = cur;
  if (prev && prev.snsl && cur.snsl && now - prev.t >= 300 && cur.snsl[0] === prev.snsl[0])
    PS.snHz = (cur.snsl[2] + cur.snsl[3] - prev.snsl[2] - prev.snsl[3]) / ((now - prev.t) / 1000);
  if (prev && prev.rxc && cur.rxc && now - prev.t >= 300 && cur.rxc[1] >= prev.rxc[1]) PS.rxHz = (cur.rxc[1] - prev.rxc[1]) / ((now - prev.t) / 1000);
}
function psDraw() {
  const cur = PS.last;
  if (!cur) return;
  const name = (idx) => { const e = typeof PORTS !== "undefined" && PORTS.find((x) => x[2] === idx); return e ? e[1] : "Port " + idx; };
  const fnName = (idx) => {
    const e = typeof PORTS !== "undefined" && PORTS.find((x) => x[2] === idx);
    const f = e && has(e[0] + "_fn") ? S.params[e[0] + "_fn"].v : 0;
    const t = typeof PORT_FN !== "undefined" && PORT_FN.find((x) => x[0] === f);
    const g = cur.gpsl && cur.gpsl[0] === idx ? cur.gpsl : null;  // GPS: protocol + baud found
    const sn = cur.snsl && cur.snsl[0] === idx ? cur.snsl : null;  // flow / range module: protocol seen
    const snTxt = sn ? (sn[2] || sn[3] ? ` · ${sn[3] > sn[2] ? "Micolink" : "MSP"} ${(PS.snHz || 0).toFixed(0)} khung/s` : sn[4] ? " · đang nhận MAVLink" : " · chưa có khung") : "";
    return (t ? t[1] : "") + snTxt + (g ? (g[2] ? ` · ${g[3] === 1 ? "UBX" : g[3] === 2 ? "NMEA" : ""} ${g[1]} baud` : ` · đang dò baud (${g[1]})`) : "");
  };
  const fmtB = (v) => (v >= 10240 ? (v / 1024).toFixed(0) + " KB/s" : v >= 1024 ? (v / 1024).toFixed(1) + " KB/s" : v.toFixed(0) + " B/s");
  let warn = [];
  if (cur.gpsl && !cur.gpsl[2]) {
    const r = cur.p[cur.gpsl[0]];
    warn.push(!r || !r[1] ? `GPS (${name(cur.gpsl[0])}): chưa nhận được byte nào - nối TX của GPS vào chân RX của cổng này (đấu chéo), GPS có nguồn 5 V, chung GND`
      : `GPS (${name(cur.gpsl[0])}): có dữ liệu nhưng chưa có khung GPS hợp lệ (đang thử ${cur.gpsl[1]} baud) - sai baud hoặc nhiễu`);
  }
  if (cur.snsl && !cur.snsl[2] && !cur.snsl[3]) {
    const sn = cur.snsl, r = cur.p[sn[0]];
    warn.push(sn[4] ? `${name(sn[0])}: module cảm biến đang gửi MAVLink - dùng MicoAssistant (cáp USB-TTL) chuyển module sang MSP (iNav) hoặc Micolink, 115200 baud`
      : `${name(sn[0])}: có dữ liệu nhưng không có khung MSP / Micolink - sai baud (module thường 115200)${r && r[4] ? ", có lỗi khung" : ""} hoặc sai giao thức`);
  }
  const rows = Object.keys(cur.p).map(Number).sort((a, b) => a - b).map((k) => {
    const a = cur.p[k], b = (PS.base.p[k]) || a, r = PS.rate[k] || [0, 0];
    const d = [3, 4, 5, 6].map((i) => Math.max(0, a[i] - b[i]));
    const errs = d[0] + d[1] + d[2] + d[3];
    if (d[0] || d[3]) warn.push(`${name(k)}: mất byte (tràn bộ đệm) - FC đọc không kịp hoặc baud quá cao`);
    if (d[1] > 2) warn.push(`${name(k)}: lỗi khung - sai baud, sai đảo tín hiệu (SBUS) hoặc nhiễu / dây lỏng`);
    const cls = errs ? "bad" : "";
    return `<tr><td><b>${esc(name(k))}</b><div class="muted" style="font-size:12px">${esc(fnName(k))}</div></td>
      <td class="num">↓ ${fmtB(r[0])}<br>↑ ${fmtB(r[1])}</td>
      <td class="num ${cls}" title="tràn phần cứng / khung / nhiễu / đầy bộ đệm">${errs ? `<b>${errs}</b> lỗi<br><span class="muted" style="font-size:12px">tràn ${d[0]} · khung ${d[1]} · nhiễu ${d[2]} · đầy ${d[3]}</span>` : "0 lỗi"}</td></tr>`;
  });
  let rxRow = "";
  if (cur.rxc) {
    const b = PS.base.rxc || cur.rxc, fr = Math.max(0, cur.rxc[1] - b[1]), ce = Math.max(0, cur.rxc[2] - b[2]);
    const pct = fr + ce ? (100 * ce) / (fr + ce) : 0;
    if (pct > 2) warn.push(`Receiver: ${pct.toFixed(1)}% khung sai checksum - kiểm tra dây, mass chung, baud / giao thức`);
    if (cur.rxc[0] > 0 && cur.rxc[1] === 0) warn.push("Receiver: có dữ liệu nhưng không có khung hợp lệ - sai giao thức (CRSF / SBUS / iBUS) hoặc sai đảo tín hiệu");
    rxRow = `<tr><td><b>Receiver</b><div class="muted" style="font-size:12px">khung kênh</div></td><td class="num">${(PS.rxHz || 0).toFixed(0)} khung/s<br><span class="muted" style="font-size:12px">${fr} khung</span></td>
      <td class="num ${pct > 2 ? "bad" : ""}">${ce} sai checksum<br><span class="muted" style="font-size:12px">${pct.toFixed(1)}%</span></td></tr>`;
  }
  setHtml($("psTable"), "<tr><th>Cổng</th><th>Lưu lượng</th><th>Lỗi (từ lúc đặt lại)</th></tr>" + rows.join("") + rxRow);
  $("psNote").textContent = warn.length ? warn.join(". ") + "." : rows.length ? "Không có lỗi đường truyền." : "Chưa có cổng UART nào được mở.";
  $("psNote").className = warn.length ? "note" : "muted";
}

// ---------------------------------------------------------------- blackbox: noise heatmap + events
const MODE_NAMES = ["ACRO", "ANGLE", "HORIZON", "ALTHOLD", "POSHOLD", "RTH", "MISSION", "LAND", "FAILSAFE", "TURTLE", "CRUISE", "AUTOTUNE", "GUIDED", "SMARTRTL", "BRAKE", "CIRCLE", "FLIP", "FOLLOW"];
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
// ---------------------------------------------------------------- blackbox: flight path (top view)
// position from home (m) logged with every record; colour = height; the dot follows the time view
function bbPathHost() {
  if ($("bbPathPanel") || !$("bbHeatPanel")) return;
  const p = document.createElement("div");
  p.className = "panel"; p.id = "bbPathPanel";
  p.innerHTML = `<div class="card-h"><span class="dot"></span><h3>Đường bay (nhìn từ trên)</h3><span class="card-x muted num" id="bbPathInfo"></span></div>
    <canvas id="bbPathChart" class="chart" style="height:320px"></canvas>
    <div class="legend"><span><i style="background:#2F8F5B"></i>thấp</span><span><i style="background:#E0A21B"></i>cao</span><span>⌂ nhà · ● vị trí ở thời điểm đang xem (thanh thời gian bên dưới)</span></div>`;
  $("bbHeatPanel").after(p);
}
function bbPathPoints(fl) {
  if (fl._path !== undefined) return fl._path;
  const step = Math.max(1, Math.floor(fl.n / 4000)), pts = [];
  for (let k = 0; k < fl.n; k += step)
    if (fl.armed[k] && (fl.posN[k] || fl.posE[k])) pts.push([fl.posN[k], fl.posE[k], fl.alt[k], fl.t[k] - fl.t[0]]);
  fl._path = pts.length >= 10 ? pts : null;
  return fl._path;
}
function drawPath() {
  const cv = $("bbPathChart"), fl = S.flight;
  if (!cv || !fl || $("tab-blackbox").hidden) return;
  const pts = fl.posN ? bbPathPoints(fl) : null;
  const panel = $("bbPathPanel");
  panel.hidden = !pts;
  if (!pts) return;
  const { g, w, h } = setupCanvas(cv);
  g.clearRect(0, 0, w, h);
  let nMin = 0, nMax = 0, eMin = 0, eMax = 0, aMin = Infinity, aMax = -Infinity, dist = 0, far = 0;
  pts.forEach(([n, e, a], i) => {
    nMin = Math.min(nMin, n); nMax = Math.max(nMax, n); eMin = Math.min(eMin, e); eMax = Math.max(eMax, e);
    aMin = Math.min(aMin, a); aMax = Math.max(aMax, a); far = Math.max(far, Math.hypot(n, e));
    if (i) dist += Math.hypot(n - pts[i - 1][0], e - pts[i - 1][1]);
  });
  const pad = 24, span = Math.max(nMax - nMin, eMax - eMin, 10);
  const k = Math.min((w - 2 * pad) / span, (h - 2 * pad) / span);
  const cx = w / 2 - ((eMin + eMax) / 2) * k, cy = h / 2 + ((nMin + nMax) / 2) * k;
  const X = (e) => cx + e * k, Y = (n) => cy - n * k;
  // grid with a round step
  const stepM = [1, 2, 5, 10, 20, 50, 100, 200, 500, 1000].find((s) => s * k >= 50) || 2000;
  g.strokeStyle = css("--rule"); g.lineWidth = 1;
  for (let e = Math.ceil((-cx / k) / stepM) * stepM; X(e) < w; e += stepM) { g.beginPath(); g.moveTo(X(e), 0); g.lineTo(X(e), h); g.stroke(); }
  for (let n = Math.ceil(((cy - h) / k) / stepM) * stepM; Y(n) > 0; n += stepM) { g.beginPath(); g.moveTo(0, Y(n)); g.lineTo(w, Y(n)); g.stroke(); }
  const col = (a) => {
    const t = aMax > aMin ? (a - aMin) / (aMax - aMin) : 0;
    return `rgb(${Math.round(47 + t * (224 - 47))},${Math.round(143 + t * (162 - 143))},${Math.round(91 + t * (27 - 91))})`;
  };
  g.lineWidth = 2.4; g.lineCap = "round";
  for (let i = 1; i < pts.length; i++) {
    const [n0, e0] = pts[i - 1], [n1, e1, a1] = pts[i];
    if (Math.hypot(n1 - n0, e1 - e0) > 50) continue;  // a position reset: no line across
    g.strokeStyle = col(a1); g.beginPath(); g.moveTo(X(e0), Y(n0)); g.lineTo(X(e1), Y(n1)); g.stroke();
  }
  g.fillStyle = css("--ink"); g.font = `bold 15px ${css("--num")}`; g.textAlign = "center"; g.textBaseline = "middle";
  g.fillText("⌂", X(0), Y(0));
  // north arrow + scale
  g.font = `11px ${css("--num")}`; g.fillStyle = css("--ink-2"); g.textAlign = "left";
  g.fillText("N ↑", 8, 14);
  g.fillRect(8, h - 12, stepM * k, 2); g.fillText(`${stepM} m`, 12 + stepM * k, h - 10);
  // where the time view is
  const span2 = fl.duration * (+$("tsZoom").value / 100);
  const tv = (fl.duration - span2) * (+$("tsPos").value / 1000) + span2 / 2;
  let best = pts[0];
  for (const p of pts) if (Math.abs(p[3] - tv) < Math.abs(best[3] - tv)) best = p;
  g.fillStyle = css("--danger"); g.beginPath(); g.arc(X(best[1]), Y(best[0]), 5, 0, 2 * Math.PI); g.fill();
  $("bbPathInfo").textContent = `${dist.toFixed(0)} m bay · xa nhất ${far.toFixed(0)} m · cao ${aMin.toFixed(0)}–${aMax.toFixed(0)} m`;
}
{
  const _af = analyzeFlight;
  analyzeFlight = function (i) {
    _af(i);
    PRO.heat = {};
    bbProHost();
    bbPathHost();
    PRO.events = S.flight ? bbEvents(S.flight) : [];
    bbEventsRender();
    drawHeat();
    drawPath();
    tuneAfterAnalysis();
  };
  const _db = drawBlackbox;
  drawBlackbox = function () { _db(); drawHeat(); drawPath(); };
  const _ts = drawTimeSeries;
  drawTimeSeries = function () { _ts(); drawPath(); };
  $("tsPos").oninput = drawTimeSeries;
  $("tsZoom").oninput = drawTimeSeries;
}

// ---------------------------------------------------------------- PID tuning assistant (PID tab)
// 1 frame -> base values; 2 test flight; 3 blackbox analysis -> plan; repeat until nothing is left.
// Progress (rounds, metrics per round) stays in this browser.
const TUNE_FRAMES = [
  ["5", "5 inch freestyle", "Khung 5 inch freestyle"], ["3", "3 inch / toothpick", "Khung 3 inch / toothpick"],
  ["cw", "Cinewhoop", "Cinewhoop (có ống bảo vệ)"], ["7", "7 inch bay xa", "Khung 7 inch bay xa"], ["keep", "Giữ PID hiện tại", null]];
const TUNE_FLIGHT = ["Pin đầy, cánh mới, bay ở chế độ ACRO hoặc ANGLE nơi trống.", "Treo máy 10-15 giây ở ga hover.",
  "5 lần giật cần roll hết cỡ mỗi bên (thả về giữa ngay), rồi 5 lần pitch.", "3 lần đẩy ga hết rồi thả về 0 (punch-out / prop wash).",
  "Vài vòng xoay yaw, sau đó hạ cánh. 60-90 giây là đủ."];
function tuneState() { return lsGet("aq-tune", { step: 0, frame: "", rounds: [] }); }
function tuneSave(t) { lsSet("aq-tune", t); tuneRender(); }
function tuneHost() {
  if ($("tunePanel") || !$("pidMatrix")) return;
  const p = document.createElement("div");
  p.className = "panel"; p.id = "tunePanel";
  $("pidMatrix").closest(".panel").before(p);
  tuneRender();
}
function tuneChecks() {
  const out = [];
  const bb = has("bb_rate_hz") ? val("bb_rate_hz") : 0;
  out.push([bb >= 500, bb >= 500 ? `blackbox ${bb} Hz` : `blackbox ${bb ? bb + " Hz (thấp)" : "đang tắt"}: cần ≥ 500 Hz`, bb >= 500 ? null : { bb_rate_hz: 500 }]);
  if (has("dshot_bidir")) out.push([!!val("dshot_bidir") || !!val("esc_telemetry"), val("dshot_bidir") ? "RPM filter (bidirectional DShot) bật" : "không có RPM: lọc nhiễu motor kém hơn (bật dshot_bidir nếu ESC hỗ trợ)", null]);
  if (has("acc_calibrated")) out.push([!!val("acc_calibrated"), val("acc_calibrated") ? "accel đã hiệu chuẩn" : "accel chưa hiệu chuẩn (tab Cảm biến)", null]);
  return out;
}
function tuneRender() {
  const host = $("tunePanel");
  if (!host) return;
  const t = tuneState(), steps = ["Khung", "Bay thử", "Phân tích", "Xong"];
  const head = `<div class="card-h"><span class="dot"></span><h3>Trợ lý chỉnh PID</h3><span class="card-x muted">${t.rounds.length ? `vòng ${t.rounds.length}` : "từng bước, dựa trên blackbox"}</span></div>
    <div class="tw-steps">${steps.map((s, i) => `<span class="${i < t.step ? "done" : i === t.step ? "cur" : ""}">${i + 1}. ${s}</span>`).join("")}</div>`;
  let body = "";
  if (t.step === 0) {
    body = `<p class="card-d">Chọn loại khung để bắt đầu từ bộ PID / bộ lọc hợp lý, hoặc giữ giá trị đang có.</p>
      <div class="row" style="flex-wrap:wrap;gap:6px">${TUNE_FRAMES.map(([k, l]) => `<button type="button" class="btn ghost" data-tf="${k}">${l}</button>`).join("")}</div>`;
  } else if (t.step === 1) {
    const ck = tuneChecks();
    body = `<div class="pf-list">${ck.map(([ok, txt, fix]) => `<div class="pf-row ${ok ? "ok" : "warn"}"><i></i><div><b>${esc(txt)}</b></div>${fix ? `<button type="button" class="btn ghost" data-tfix='${JSON.stringify(fix)}'>Sửa</button>` : "<span></span>"}</div>`).join("")}</div>
      <p class="card-d" style="margin-top:10px">Nhớ bấm <b>Áp dụng</b> rồi <b>Lưu</b> trước khi bay. Bài bay thử:</p><ol class="card-d">${TUNE_FLIGHT.map((x) => `<li>${esc(x)}</li>`).join("")}</ol>
      <div class="row"><button type="button" class="btn primary" id="tuneFlown">Đã bay xong: tải log và phân tích</button><button type="button" class="btn ghost" id="tuneBack">Chọn lại khung</button></div>`;
  } else if (t.step === 2) {
    const r = t.rounds[t.rounds.length - 1];
    const n = (S.plan || []).length;
    body = `<p class="card-d">Vòng ${t.rounds.length}: ${r ? tuneSummary(r) : ""}</p>
      ${tuneTable(t)}
      <p class="card-d">${n ? `Kế hoạch có <b>${n}</b> thay đổi (xem tab Blackbox). Đưa vào hàng chờ, Áp dụng, Lưu rồi bay lại.` : "Log này không cần đổi gì thêm."}</p>
      <div class="row">${n ? `<button type="button" class="btn primary" id="tuneTake">Đưa ${n} thay đổi vào hàng chờ</button>` : ""}<button type="button" class="btn ${n ? "ghost" : "primary"}" id="tuneNext">${n ? "Bay vòng tiếp" : "Kết thúc"}</button><button type="button" class="btn ghost" id="tuneGoBb">Xem chi tiết</button></div>`;
  } else {
    body = `<p class="card-d">Đã xong ${t.rounds.length} vòng. Bay thử cảm nhận; muốn tinh chỉnh tiếp thì bắt đầu lại từ bước 2.</p>${tuneTable(t)}
      <div class="row"><button type="button" class="btn" id="tuneMore">Bay thêm một vòng</button><button type="button" class="btn ghost" id="tuneReset">Bắt đầu lại từ đầu</button></div>`;
  }
  host.innerHTML = head + body;
  host.querySelectorAll("[data-tf]").forEach((b) => (b.onclick = () => {
    const f = TUNE_FRAMES.find((x) => x[0] === b.dataset.tf);
    const pr = f[2] && typeof PRESET_LIST !== "undefined" ? PRESET_LIST.find((x) => x[0] === f[2]) : null;
    let n = 0;
    if (pr) for (const [k, v] of Object.entries(pr[2])) if (has(k) && Math.abs(val(k) - v) > 1e-6) { setEdit(k, v); n++; }
    toast(n ? `${n} giá trị cho ${f[1]} đang chờ: bấm Áp dụng rồi Lưu.` : "Giữ giá trị hiện tại.");
    tuneSave({ ...tuneState(), step: 1, frame: f[0] });
  }));
  host.querySelectorAll("[data-tfix]").forEach((b) => (b.onclick = () => {
    for (const [k, v] of Object.entries(JSON.parse(b.dataset.tfix))) if (has(k)) setEdit(k, v);
    toast("Đã đưa vào hàng chờ: Áp dụng rồi Lưu."); tuneRender();
  }));
  const on = (id, f) => { if ($(id)) $(id).onclick = f; };
  on("tuneBack", () => tuneSave({ ...tuneState(), step: 0 }));
  on("tuneFlown", () => { const s = tuneState(); s.waiting = true; lsSet("aq-tune", s); gotoTab("blackbox"); toast("Tải log (hoặc mở file log): phân tích xong trợ lý tự quay lại."); });
  on("tuneTake", () => { (S.plan || []).forEach((p) => setEdit(p.name, p.to)); toast(`${(S.plan || []).length} thay đổi đang chờ: Áp dụng, Lưu, rồi bay lại.`); });
  on("tuneNext", () => tuneSave({ ...tuneState(), step: (S.plan || []).length ? 1 : 3 }));
  on("tuneGoBb", () => gotoTab("blackbox"));
  on("tuneMore", () => tuneSave({ ...tuneState(), step: 1 }));
  on("tuneReset", () => { if (confirm("Xoá tiến trình của trợ lý (không đổi tham số nào)?")) tuneSave({ step: 0, frame: "", rounds: [] }); });
}
function tuneSummary(r) {
  return r.ax.map((a, i) => `${AXES[i]} vọt lố ${a.ov == null ? "—" : a.ov.toFixed(0) + "%"}, lên ${a.rise == null ? "—" : a.rise + " ms"}`).join(" · ");
}
function tuneTable(t) {
  if (!t.rounds.length) return "";
  const f = (v, d) => (v == null || !isFinite(v) ? "—" : v.toFixed(d));
  return `<div style="overflow-x:auto"><table class="ports"><tr><th>Vòng</th>${AXES.map((a) => `<th>${a}: vọt lố</th><th>${a}: nhiễu</th>`).join("")}<th>Thay đổi</th></tr>
    ${t.rounds.map((r, i) => `<tr><td>${i + 1}</td>${r.ax.map((a) => `<td class="num">${a.ov == null ? "—" : f(a.ov, 0) + "%"}</td><td class="num">${f(a.noise, 2)}</td>`).join("")}<td class="num">${r.plan}</td></tr>`).join("")}</table></div>`;
}
// a new analysis while the assistant waits for the test flight: one more round
function tuneAfterAnalysis() {
  const t = tuneState();
  if (!t.waiting || !S.analysis || !S.flight) return;
  const sig = `${S.flight.n}:${S.flight.duration.toFixed(2)}`;
  if (t.rounds.some((r) => r.sig === sig)) return;  // the same log opened again
  t.rounds.push({ sig, ax: S.analysis.axes.map((ax) => ({ ov: ax.step ? ax.step.overshoot * 100 : null, rise: ax.step ? ax.step.riseMs : null, noise: ax.noiseFilt })), plan: (S.plan || []).length });
  t.step = 2;
  t.waiting = false;
  lsSet("aq-tune", t);
  tuneRender();
  toast("Trợ lý chỉnh PID: đã có kết quả vòng " + t.rounds.length + " (tab PID).");
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
  ["Sơ đồ chân GPIO (đấu dây)", "pins", "gpio pin chan dau day wiring pinout so do"],
  ["Tay điều khiển: kênh, thứ tự kênh, hành trình cần", "rx", "receiver tay dieu khien kenh channel aetr taer endpoint hanh trinh deadband"],
  ["Gói chẩn đoán (gửi khi nhờ hỗ trợ)", "system", "chan doan diagnostic support log ho tro bao loi"],
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
    if (b.dataset.tab === "pins" && typeof pinShow === "function") return pinShow();
    if (b.dataset.tab && b.dataset.tab.startsWith("fn:")) { const f = window[b.dataset.tab.slice(3)]; return typeof f === "function" && f(); }
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
  buildPid = function () { _bp(); markAdv(); tuneHost(); if (shown("params")) profHost(); };
}

// ---------------------------------------------------------------- theme: cockpit (dark, default) / light / follow the system / outdoor
const THEMES = [["", "Buồng lái"], ["light", "Sáng"], ["auto", "Theo máy"], ["sun", "Ngoài trời"]];
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
    // phones: Web Serial on Android does not list every USB CDC device, so WebUSB is offered there too
    const phone = /Android|Mobi/i.test(navigator.userAgent);
    $("usbPick").hidden = !webusb || (serial && !phone);
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
  if ("usb" in navigator && (!("serial" in navigator) || /Android|Mobi/i.test(navigator.userAgent))) {
    navigator.usb.addEventListener("disconnect", (ev) => { if (LINK.usbdev === ev.device) linkLost("Đã rút cáp USB."); });
    navigator.usb.addEventListener("connect", (ev) => { if (!LINK.open && !$("connect").hidden) usbConnect(ev.device); });
  }
}
