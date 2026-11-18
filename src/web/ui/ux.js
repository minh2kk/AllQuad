// AllQuad configurator - usability layer (loaded last, after pro.js)
//  1. long tabs are split into pages: the pill bar under the header shows one part at a time ("Tất cả" = all)
//  2. phones: the bottom bar keeps the main tabs, the rest sit in a "Thêm" sheet
//  3. saving: one "Lưu" button (apply + flash), Ctrl+S, a reboot reminder for settings that need one
//  4. connect screen: names the device it can reconnect to
//  5. firmware flasher: file dropped onto the dialog, ESP32 / STM32 recognised from the file, and a warning
//     when the file was built for another board than the FC that is connected
const UX = { applied: new Set(), reboot: [], obs: new Map() };
const uxLs = (k, d) => { try { const v = localStorage.getItem(k); return v === null ? d : v; } catch (e) { return d; } };
const uxLsSet = (k, v) => { try { localStorage.setItem(k, v); } catch (e) {} };

// ---------------------------------------------------------------- 1. pages
// automated browsers (the UI tests) see every part at once unless a test turns paging on (aq.pages = 1)
const UX_PAGING = uxLs("aq.pages", navigator.webdriver ? "0" : "1") === "1";
function uxHeads(sec) { return [...sec.querySelectorAll(":scope > h2")]; }
function uxShow(sec, i, scroll) {
  sec.dataset.page = String(i);
  let pg = -1;
  for (const el of sec.children) {
    if (el.classList.contains("jump")) continue;
    if (el.tagName === "H2") pg++;
    // what sits before the first heading belongs to every page
    el.toggleAttribute("data-pghide", i >= 0 && pg >= 0 && pg !== i);
  }
  const nav = sec.querySelector(":scope > .jump");
  if (nav) [...nav.children].forEach((a) => {
    const on = +a.dataset.pg === i;
    a.classList.toggle("on", on);
    if (on) a.setAttribute("aria-current", "page"); else a.removeAttribute("aria-current");
  });
  if (scroll) window.scrollTo(0, 0);
}
buildJump = function (sec) {
  if (!sec) return;
  sec.querySelector(":scope > .jump")?.remove();
  const hs = uxHeads(sec);
  if (hs.length < 2) { uxShow(sec, -1); return; }
  const nav = document.createElement("nav");
  nav.className = "jump";
  nav.setAttribute("aria-label", "Phần của trang");
  const pill = (t, i) => {
    const a = document.createElement("a");
    a.href = "#";
    a.textContent = t;
    a.dataset.pg = String(i);
    a.onclick = (e) => {
      e.preventDefault();
      if (UX_PAGING) { uxLsSet("aq.page." + sec.id, String(i)); uxShow(sec, i, true); }
      else if (i >= 0) hs[i].scrollIntoView({ behavior: "smooth" });
    };
    return a;
  };
  hs.forEach((h, i) => nav.appendChild(pill(h.dataset.j || h.textContent, i)));
  if (UX_PAGING) nav.appendChild(pill("Tất cả", -1));
  sec.insertBefore(nav, sec.firstChild);
  let cur = UX_PAGING ? +uxLs("aq.page." + sec.id, "0") : -1;
  if (!(cur >= -1 && cur < hs.length)) cur = 0;
  uxShow(sec, cur);
  // panels added later (extras, a new heading): put them on their page
  if (!UX.obs.has(sec)) {
    const o = new MutationObserver(() => {
      if (sec.hidden) return;
      const n = uxHeads(sec).length, have = sec.querySelectorAll(":scope > .jump > a").length - (UX_PAGING ? 1 : 0);
      if (n !== have) buildJump(sec); else uxShow(sec, +(sec.dataset.page || 0));
    });
    o.observe(sec, { childList: true });
    UX.obs.set(sec, o);
  }
};
// search results, check-list links ... scroll to an element: open its page first
{
  const siv = Element.prototype.scrollIntoView;
  Element.prototype.scrollIntoView = function (...a) {
    const sec = this.closest && this.closest("main > section");
    if (sec && sec.dataset.page && sec.dataset.page !== "-1") {
      let top = this;
      while (top.parentElement && top.parentElement !== sec) top = top.parentElement;
      if (top.parentElement === sec && top.hasAttribute("data-pghide")) {
        let pg = -1;
        for (const el of sec.children) { if (el.tagName === "H2") pg++; if (el === top) break; }
        if (pg >= 0) { uxLsSet("aq.page." + sec.id, String(pg)); uxShow(sec, pg); }
      }
    }
    return siv.apply(this, a);
  };
}
// the tab that is open at load
buildJump(document.querySelector("main > section:not([hidden])"));

// ---------------------------------------------------------------- 2. phone navigation
{
  const nav = document.querySelector("nav.tabs");
  const more = document.createElement("button");
  more.type = "button";
  more.id = "navMore";
  more.setAttribute("aria-expanded", "false");
  more.innerHTML = '<svg viewBox="0 0 24 24"><circle cx="5" cy="12" r="1.6"/><circle cx="12" cy="12" r="1.6"/><circle cx="19" cy="12" r="1.6"/></svg>Thêm';
  nav.appendChild(more);
  const icon = more.innerHTML;
  const set = (open) => {
    document.body.classList.toggle("navsheet", open);
    more.setAttribute("aria-expanded", String(open));
    more.innerHTML = open ? '<svg viewBox="0 0 24 24"><path d="M6 6l12 12M18 6L6 18"/></svg>Đóng' : icon;
  };
  const close = () => set(false);
  more.onclick = () => set(!document.body.classList.contains("navsheet"));
  nav.addEventListener("click", (e) => { if (e.target.closest("button[data-tab]")) close(); });
  // (the button's own label is swapped on open: its old icon is no longer in the page, so skip detached targets)
  document.addEventListener("click", (e) => { if (document.body.classList.contains("navsheet") && e.target.isConnected && !e.target.closest("nav.tabs")) close(); });
  document.addEventListener("keydown", (e) => { if (e.key === "Escape") close(); });
}

// ---------------------------------------------------------------- 3. saving
// settings that need a reboot: remembered when applied, turned into a reminder once saved
{
  const apply0 = applyEdits;
  applyEdits = async function () {
    const r = Object.keys(S.edits).filter((n) => S.params[n] && S.params[n].r);
    await apply0();
    if (!Object.keys(S.edits).length) r.forEach((n) => UX.applied.add(n));
  };
  $("applyBtn").onclick = () => applyEdits();
  const api0 = api;
  api = async function (path, opts) {
    const r = await api0(path, opts);
    if (/^\/api\/save(\?|$)/.test(path) && UX.applied.size) {
      UX.reboot = [...new Set([...UX.reboot, ...UX.applied])];
      UX.applied.clear();
      setTimeout(refreshApplyBar, 0);
    }
    if (/^\/api\/cmd\?c=reboot/.test(path)) { UX.reboot = []; setTimeout(refreshApplyBar, 0); }
    return r;
  };
}
async function uxApplySave() {
  if (S.armed) return toast("Hãy disarm trước khi lưu.");
  const n = Object.keys(S.edits).length;
  if (n) { await applyEdits(); if (Object.keys(S.edits).length) return; }  // refused: the toast says why
  try {
    await api("/api/save", { method: "POST" });
    if (typeof PRO !== "undefined") PRO.unsaved = false;
    toast(UX.reboot.length ? "Đã lưu. Khởi động lại FC để dùng thiết lập mới." : n ? `Đã lưu ${n} thay đổi vào flash.` : "Đã lưu vào flash.");
  } catch (e) { toast(e.code === 423 ? "Hãy disarm trước khi lưu." : "Không lưu được: " + e.message); }
  refreshApplyBar();
}
$("applySaveBtn").onclick = uxApplySave;
{
  const bar = $("applyBar");
  const rb = document.createElement("button");
  rb.type = "button"; rb.className = "btn primary"; rb.id = "rebootNowBtn"; rb.textContent = "Khởi động lại FC"; rb.hidden = true;
  rb.onclick = async () => {
    try { await api("/api/cmd?c=reboot", { method: "POST" }); toast("FC đang khởi động lại: trang tự kết nối lại khi FC lên."); }
    catch (e) { toast(e.code === 423 ? "Hãy disarm trước." : "Lỗi: " + e.message); }
  };
  const later = document.createElement("button");
  later.type = "button"; later.className = "btn"; later.id = "rebootLaterBtn"; later.textContent = "Để sau"; later.hidden = true;
  later.onclick = () => { UX.reboot = []; refreshApplyBar(); };
  bar.querySelector(".row").append(later, rb);
  const refresh0 = refreshApplyBar;
  refreshApplyBar = function () {
    refresh0();
    const k = Object.keys(S.edits).length;
    const unsaved = typeof PRO !== "undefined" && PRO.unsaved;
    const reboot = !k && !unsaved && UX.reboot.length > 0 && !S.armed;
    $("applySaveBtn").hidden = !k;
    rb.hidden = later.hidden = !reboot;
    if (reboot) {
      bar.classList.add("show", "saved-only");
      $("applyText").textContent = `Đã lưu. Cần khởi động lại FC để dùng: ${UX.reboot.map((n) => (typeof proTitle === "function" ? proTitle(n) : n)).join(", ")}`;
    }
    document.body.classList.toggle("barshow", bar.classList.contains("show"));
  };
  refreshApplyBar();
}
document.addEventListener("keydown", (e) => {
  if ((e.ctrlKey || e.metaKey) && !e.altKey && e.key.toLowerCase() === "s") {
    e.preventDefault();
    if (Object.keys(S.edits).length || (typeof PRO !== "undefined" && PRO.unsaved)) uxApplySave();
    else toast("Không có thay đổi nào cần lưu.");
  }
});

// ---------------------------------------------------------------- 4. connect screen
const USB_NAMES = { 0x0483: "STM32", 0x10C4: "ESP32 (CP210x)", 0x1A86: "ESP32 (CH340)", 0x303A: "ESP32-S3 / C6 / P4", 0x0403: "FTDI" };
async function uxComLabel() {
  if (!("serial" in navigator) || $("connect").hidden) return;
  try {
    const ps = await navigator.serial.getPorts();
    if (!ps.length) return;
    const i = ps[0].getInfo ? ps[0].getInfo() : {};
    const name = USB_NAMES[i.usbVendorId] || (i.usbVendorId ? `USB ${i.usbVendorId.toString(16).padStart(4, "0")}:${(i.usbProductId || 0).toString(16).padStart(4, "0")}` : "");
    $("comAgain").textContent = name ? `Kết nối lại ${name}` : "Kết nối lại cổng vừa dùng";
  } catch (e) {}
}
new MutationObserver(uxComLabel).observe($("connect"), { attributes: true, attributeFilter: ["hidden"] });
uxComLabel();

// ---------------------------------------------------------------- 5. firmware flasher
const UX_BOARDS = ["STM32H743-DevEBox", "STM32H743-WeAct", "STM32F411-BlackPill", "STM32F722", "STM32F405",
  "ESP32-S3-N16R8", "ESP32-S3-DevKitC", "ESP32-C6-DevKitC", "ESP32-P4", "ESP32-DevKit", "AllQuad-Bridge"];
// the board a firmware image was built for: its AQ_BOARD_NAME string is in the image
function uxBoardOf(d) {
  if (!d || d.length < 1024) return "";
  const txt = new TextDecoder("latin1").decode(d);
  for (const b of UX_BOARDS) {  // longest names first: "ESP32-S3-DevKitC" before "ESP32-DevKit"
    let i = txt.indexOf(b);
    while (i >= 0) {
      const next = txt.charCodeAt(i + b.length);
      if (!(next >= 45 && next <= 122 && next !== 47)) return b;  // the whole name, not the start of a longer one
      i = txt.indexOf(b, i + 1);
    }
  }
  return "";
}
{
  // short lead line; the long how-to goes into a fold
  const how = $("flHow");
  const det = document.createElement("details");
  det.className = "fl-help";
  det.innerHTML = "<summary>Hướng dẫn và khi nạp không được</summary>";
  how.before(det);
  det.appendChild(how);
  const lead = document.createElement("p");
  lead.className = "muted"; lead.id = "flLead";
  lead.textContent = "Chọn file .bin hoặc kéo thả vào khung này, rồi bấm Nạp. Loại board nhận ra từ file.";
  det.before(lead);

  const check0 = flCheck;
  flCheck = function () {
    const d = FL.data;
    FL.board = ""; FL.mismatch = "";
    if (d && d.length > 16 && !FL.busy) {
      const dv = new DataView(d.buffer, d.byteOffset, 8);
      const stm = (dv.getUint32(4, true) & 0xFFF00001) === 0x08000001 && ((dv.getUint32(0, true) & 0xFFF00000) === 0x20000000 || (dv.getUint32(0, true) & 0xFFF00000) === 0x24000000);
      const esp = d[0] === 0xE9 || (d[0] === 0xFF && d.length > 0x1001 && d[0x1000] === 0xE9);
      const t = stm ? "stm" : esp ? "esp" : FL.target;
      if (t !== FL.target) {  // the file says which loader to use
        FL.target = t;
        $("flTarget").querySelectorAll("button").forEach((x) => x.setAttribute("aria-pressed", String(x.dataset.t === t)));
        flHow();
        if (typeof relRender === "function") relRender();
      }
    }
    check0();
    if (!d) return;
    FL.board = uxBoardOf(d);
    const fc = (S.lastHello && S.lastHello.board) || "";
    if (FL.board) $("flInfo").textContent += ` Board: ${FL.board}.`;
    if (FL.board && fc && LINK.open && !fc.startsWith(FL.board)) {
      FL.mismatch = fc.split(" (")[0];
      $("flInfo").textContent += ` Cảnh báo: FC đang nối là ${FL.mismatch}.`;
      $("flInfo").classList.add("bad");
    } else $("flInfo").classList.remove("bad");
  };
  const go0 = $("flGo").onclick;
  $("flGo").onclick = async (e) => {
    if (FL.mismatch && !confirm(`File này dành cho ${FL.board}, còn FC đang nối là ${FL.mismatch}. Nạp nhầm board có thể làm sai chân motor / cảm biến. Vẫn nạp?`)) return;
    return go0(e);
  };
  // drag and drop a .bin onto the dialog
  const box = $("flasher").querySelector(".panel");
  box.addEventListener("dragover", (e) => { e.preventDefault(); box.classList.add("drop"); });
  box.addEventListener("dragleave", () => box.classList.remove("drop"));
  box.addEventListener("drop", async (e) => {
    e.preventDefault();
    box.classList.remove("drop");
    const f = e.dataTransfer && e.dataTransfer.files[0];
    if (!f || FL.busy) return;
    FL.file = f;
    FL.data = new Uint8Array(await f.arrayBuffer());
    flCheck();
  });
}
