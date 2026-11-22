// AllQuad configurator - cockpit design layer (loaded after guide.js): small structural touches the CSS
// cannot do alone. Nothing here changes what a control does.
//  - overview: the "missing configuration" card joins the status hero
//  - long explanations fold to one line (click to read), so a page shows controls first
//  - card titles get a small icon (known titles only); the decorative dot is gone
//  - serial ports: an icon per function, unused ports dimmed
//  - phone tab bar: short labels
"use strict";
const DZ = { t: 0 };
const DZ_SVG = (d) => `<svg viewBox="0 0 24 24" aria-hidden="true">${d}</svg>`;
const DZ_IC = {
  att: '<circle cx="12" cy="12" r="8"/><path d="M4 12h16M8 16l8-8"/>',
  rc: '<rect x="3" y="7" width="18" height="11" rx="3"/><circle cx="8" cy="12.5" r="1.6"/><circle cx="16" cy="12.5" r="1.6"/><path d="M8 7V4M16 7V4"/>',
  motor: '<circle cx="12" cy="12" r="3"/><path d="M12 9V3M12 21v-6M9 12H3M21 12h-6"/>',
  sens: '<path d="M3 12h4l2-6 4 12 2-6h6"/>',
  bat: '<rect x="3" y="7" width="16" height="10" rx="2"/><path d="M21 10v4M7 10v4M11 10v4"/>',
  gps: '<path d="M12 21s-6-5.6-6-11a6 6 0 0 1 12 0c0 5.4-6 11-6 11z"/><circle cx="12" cy="10" r="2"/>',
  link: '<path d="M4 16a12 12 0 0 1 16 0M7 19a7 7 0 0 1 10 0M12 21h.01"/>',
  filter: '<path d="M3 5h18l-7 8v6l-4 2v-8z"/>',
  shield: '<path d="M12 3l8 3v6c0 5-3.5 8-8 9-4.5-1-8-4-8-9V6z"/>',
  wrench: '<path d="M14 6a4 4 0 0 0 5 5l-9 9-3-3 9-9a4 4 0 0 1-2-2z"/>',
  chart: '<path d="M4 20V4M4 20h16M8 16l4-5 3 3 4-6"/>',
  list: '<path d="M8 6h12M8 12h12M8 18h12M4 6h.01M4 12h.01M4 18h.01"/>',
  map: '<path d="M9 4L3 6v14l6-2 6 2 6-2V4l-6 2z"/><path d="M9 4v14M15 6v14"/>',
  cpu: '<rect x="7" y="7" width="10" height="10" rx="2"/><path d="M10 3v4M14 3v4M10 17v4M14 17v4M3 10h4M3 14h4M17 10h4M17 14h4"/>',
  check: '<path d="M5 12l4 4 10-10"/>',
  eye: '<path d="M2 12s4-7 10-7 10 7 10 7-4 7-10 7S2 12 2 12z"/><circle cx="12" cy="12" r="3"/>',
};
// card title -> icon (first match wins; titles in Vietnamese, as written in the page)
const DZ_TITLES = [
  [/tư thế|chân trời|hướng lắp/i, "att"], [/tay điều khiển|receiver|công tắc|kênh/i, "rc"], [/motor|esc/i, "motor"],
  [/pin|điện áp|dòng/i, "bat"], [/gps|điều hướng|nhiệm vụ|bản đồ|hàng rào/i, "gps"], [/tín hiệu|sóng|vtx|wifi|kết nối/i, "link"],
  [/lọc|notch|nhiễu|phổ/i, "filter"], [/failsafe|an toàn|bảo vệ|arm/i, "shield"], [/hiệu chuẩn|thiết lập|cài/i, "wrench"],
  [/blackbox|log|đáp ứng|tóm tắt|gyro/i, "chart"], [/cảm biến|baro|compass|optical/i, "sens"], [/hiệu năng|cpu|hệ thống|task/i, "cpu"],
  [/kiểm tra|tự kiểm|sẵn sàng/i, "check"], [/bằng mắt|xem/i, "eye"], [/còn thiếu|lịch sử|danh sách|từng bước/i, "list"],
];
const DZ_PORT = {
  0: '<path d="M6 12h12"/>', 1: DZ_IC.rc, 2: DZ_IC.gps, 3: '<path d="M4 18L12 4l8 14z"/><path d="M12 10v3M12 16h.01"/>',
  4: DZ_IC.eye, 5: '<rect x="3" y="6" width="18" height="12" rx="3"/><path d="M8 12h.01M16 12h.01"/>', 6: DZ_IC.motor,
  7: '<path d="M12 4v16M8 16l4 4 4-4"/>', 8: DZ_IC.chart, 9: DZ_IC.link, 10: DZ_IC.link, 11: '<path d="M7 7l10 10-5 5V2l5 5L7 17"/>', 12: DZ_IC.link,
};
function portIcon(fn) { return DZ_SVG(DZ_PORT[fn] || DZ_PORT[0]); }

function dzIcons(root) {
  root.querySelectorAll(".card-h > .dot:not(.ic):not([data-dz])").forEach((d) => {
    d.dataset.dz = "1";
    const h = d.parentElement.querySelector("h3");
    if (!h) return;
    const m = DZ_TITLES.find(([re]) => re.test(h.textContent));
    if (!m) return;
    d.classList.add("ic");
    d.innerHTML = DZ_SVG(DZ_IC[m[1]]);
  });
}
// long explanations: one line (intro under a page title) or two (inside a card); click to read all
function dzClamp(root) {
  root.querySelectorAll("main > section > h2 + p.muted, main .panel p.muted, main .card-d").forEach((p) => {
    if (p.dataset.dz || p.closest(".ff, #ffPanel, .rep-notes, #bbReport, .ready, #todoPanel, .term, dialog")) return;
    const len = p.textContent.trim().length;
    if (len < 150) return;
    p.dataset.dz = "1";
    p.classList.add("clamp");
    if (p.closest(".panel")) p.classList.add("two");
    p.title = "Bấm để đọc hết";
    p.addEventListener("click", (e) => { if (!e.target.closest("a, button")) p.classList.toggle("open"); });
  });
}
// the to-do card belongs to the status hero
function dzHero() {
  const t = $("todoPanel"), b = $("blockers");
  if (t && b && t.parentElement !== b) b.appendChild(t);
}
function dzRun() {
  DZ.t = 0;
  const m = document.querySelector("main");
  if (!m) return;
  dzHero();
  dzIcons(m);
  dzClamp(m);
}
// phone tab bar: short labels (CSS shows data-short below 900 px; the button text itself stays)
const DZ_SHORT = { overview: "Tổng quan", setup: "Thiết lập", preflight: "Trước bay", pid: "PID", modes: "Chế độ", nav: "Điều hướng",
  rx: "Tay ĐK", sensors: "Cảm biến", motors: "Motor", ports: "Cổng", osd: "OSD", blackbox: "Log", params: "Tham số", cli: "CLI", system: "Hệ thống" };
document.querySelectorAll("nav.tabs button[data-tab]").forEach((b) => { if (DZ_SHORT[b.dataset.tab]) b.dataset.short = DZ_SHORT[b.dataset.tab]; });
// the port icon follows the function picked
document.addEventListener("change", (e) => { if (e.target.closest && e.target.closest("#portTable")) setTimeout(() => typeof buildPorts === "function" && buildPorts(), 0); });
dzRun();
new MutationObserver(() => { if (!DZ.t) DZ.t = setTimeout(dzRun, 300); }).observe(document.querySelector("main"), { childList: true, subtree: true });
