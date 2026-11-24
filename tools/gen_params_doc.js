#!/usr/bin/env node
// Writes docs/PARAMS.md: every parameter with its group, range, default, unit, choices and the
// Vietnamese description shown in the configurator (HINT / ENUMS / UNITS / GROUPS of src/web/ui/index.html).
// Needs build/tests (tools/build_host.sh).   node tools/gen_params_doc.js
"use strict";
const fs = require("fs"), path = require("path"), { execFileSync } = require("child_process");
const root = path.join(__dirname, "..");
const html = fs.readFileSync(path.join(root, "src", "web", "ui", "index.html"), "utf8");

// the text of `const NAME = {...}` / `[...]` with bracket matching (strings and regexes skipped)
function literal(name) {
  const at = html.indexOf(`const ${name} = `);
  if (at < 0) throw new Error(name + " not found");
  let i = html.indexOf("=", at) + 1;
  while (html[i] === " ") i++;
  const open = html[i], close = open === "{" ? "}" : "]";
  let depth = 0, j = i, q = null;
  for (; j < html.length; j++) {
    const c = html[j];
    if (q) { if (c === "\\") j++; else if (c === q) q = null; continue; }
    if (c === '"' || c === "'" || c === "`") { q = c; continue; }
    if (c === "/" && /[,[(:]\s*$/.test(html.slice(Math.max(i, j - 4), j))) {  // regex literal
      j++;
      for (; j < html.length && html[j] !== "/"; j++) if (html[j] === "\\") j++;
      continue;
    }
    if (c === open) depth++;
    else if (c === close && --depth === 0) break;
  }
  return Function(`"use strict"; return (${html.slice(i, j + 1)});`)();
}
const HINT = literal("HINT"), ENUMS = literal("ENUMS"), UNITS = literal("UNITS"), GROUPS = literal("GROUPS");
const dump = JSON.parse(execFileSync(path.join(root, "build", "tests"), ["web-dump", "params"]).toString());
const unit = (n) => { for (const [re, u] of UNITS) if (re.test(n)) return u; return ""; };
const fmt = (v) => (Math.abs(v) >= 100 || Number.isInteger(v) ? String(Math.round(v * 1000) / 1000) : String(+v.toFixed(4)));

const byGroup = new Map(GROUPS.map(([g]) => [g, []]));
byGroup.set("Khác", []);
for (const p of dump.params) {
  const g = GROUPS.find(([, re]) => re.test(p.n));
  byGroup.get(g ? g[0] : "Khác").push(p);
}
const esc = (s) => String(s).replace(/\|/g, "\\|");
// families of parameters the page edits with dedicated editors (no HINT entry)
const AXV = { roll: "roll", pitch: "pitch", yaw: "yaw" };
const TERM = { p: "P (bám theo sai lệch)", i: "I (triệt sai lệch kéo dài)", d: "D (giảm chấn)", f: "F / feedforward (bám cần nhanh)" };
const FN = "0 tắt, 1 receiver, 2 GPS, 3 MAVLink, 4 MSP, 5 DisplayPort (OSD kính số), 6 telemetry ESC, 7 TFmini, 8 blackbox, 9 VTX SmartAudio, 10 VTX Tramp";
function family(n) {
  let m;
  if ((m = /^(?:p([23])_)?(roll|pitch|yaw)_([pidf])$/.exec(n))) return `${TERM[m[3]]} trục ${AXV[m[2]]}${m[1] ? `, profile ${m[1]}` : ""}`;
  if ((m = /^(?:p([23])_)?(roll|pitch|yaw)_(rc_rate|rc|max_rate|max|expo)$/.exec(n))) {
    const what = /^rc/.test(m[3]) ? "Rate tâm cần (°/s ở giữa cần, rates Actual)" : /^max/.test(m[3]) ? "Tốc độ quay tối đa khi gạt hết cần (°/s)" : "Expo (độ cong quanh tâm cần)";
    return `${what}, trục ${AXV[m[2]]}${m[1] ? `, profile ${m[1]}` : ""}`;
  }
  if ((m = /^mr(\d+)_(mode|ch|lo|hi)$/.exec(n)))
    return { mode: `Dòng chế độ ${m[1]}: chế độ được bật (mã như tab Chế độ: 1 ARM, 2 ANGLE, 3 HORIZON, 4 ALTHOLD, 5 POSHOLD, 6 RTH, 7 MISSION, 8 LAND, 9 TURTLE, 10 BEEPER, 11 CRUISE, 12 AUTOTUNE, 13 GPS RESCUE, 14 LAUNCH, 15 SMART RTL, 16 HEADFREE, 17 BRAKE, 18 CIRCLE, 19 FLIP, 20 FOLLOW; 0 = không dùng)`,
      ch: `Dòng chế độ ${m[1]}: kênh RC (0-based, 4 = AUX1 / CH5)`, lo: `Dòng chế độ ${m[1]}: đầu dưới của khoảng (µs)`, hi: `Dòng chế độ ${m[1]}: đầu trên của khoảng (µs)` }[m[2]];
  if ((m = /^(serial(\d)|udp)_(fn|baud)$/.exec(n)))
    return m[3] === "fn" ? `Chức năng của ${m[2] ? "SERIAL" + m[2] : "cổng UDP (WiFi)"}: ${FN}` : `Tốc độ baud của SERIAL${m[2]}`;
  if ((m = /^vtxt_(mw|val|band)(\d)$/.exec(n)))
    return { mw: `Bảng VTX: công suất mức ${m[2]} (mW, 0 = không dùng)`, val: `Bảng VTX: giá trị gửi cho VTX ở mức ${m[2]}`, band: `Băng U tự chọn: tần số kênh ${m[2]} (MHz)` }[m[1]];
  if ((m = /^osd_(\w+)_pos$/.exec(n))) return `Vị trí phần tử OSD "${m[1]}" (cột + 32 × hàng trên lưới 30×16; bit 15 = hiện). Kéo thả trong tab OSD`;
  if ((m = /^(acc|mag)_(bias|scale)_([xyz])$/.exec(n))) return `Kết quả hiệu chuẩn ${m[1] === "acc" ? "gia tốc" : "la bàn"}: ${m[2] === "bias" ? "độ lệch" : "hệ số"} trục ${m[3]} (tự ghi khi hiệu chuẩn)`;
  if ((m = /^mag_odiag_(\w\w)$/.exec(n))) return `Hiệu chuẩn la bàn ellipsoid: hệ số chéo ${m[1]} (soft iron, tự ghi)`;
  if ((m = /^mag_cmot_([xyz])$/.exec(n))) return `compassmot: nhiễu la bàn theo dòng điện, trục ${m[1]} (G/A, tự ghi)`;
  const vtx = { vtx_band: "VTX: băng tần (1 A, 2 B, 3 E, 4 F, 5 R, 6 L, 7 U)", vtx_channel: "VTX: kênh 1–8", vtx_freq: "VTX: tần số tự chọn (MHz, 0 = theo băng/kênh)",
    vtx_power: "VTX: mức công suất (theo bảng VTX)", vtx_pit: "VTX: pit mode (phát rất yếu khi ở bàn)", vtx_low_disarm: "VTX: công suất thấp nhất khi disarm",
    led_pin: "Dải LED WS2812: chân GPIO (ESP32; STM32 cố định PB8), 0 = tắt", led_count: "Số LED", led_brightness: "Độ sáng LED (%)",
    led_mode: "Kiểu LED: 0 màu theo trạng thái, 1 hướng (trước trắng, sau đỏ), 2 màu cố định", led_color: "Màu LED cố định",
    vtxt_powers: "Bảng VTX: số mức công suất tự đặt (0 = mặc định 25/100/200/400/600 mW)",
    vtxt_sa_index: "SmartAudio gửi số thứ tự mức thay vì dBm (VTX firmware cũ)" };
  return vtx[n] || "";
}
let md = `# Bảng tham số AllQuad (cấu hình v25)\n\n` +
  `Tạo tự động bằng \`node tools/gen_params_doc.js\` từ firmware (giới hạn, mặc định) và trang cấu hình (mô tả). ` +
  `${dump.params.length} tham số. Đổi bằng trang cấu hình (tab Tham số, hoặc thẻ của từng tab), CLI \`set <tên> <giá trị>\` rồi \`save\`, ` +
  `MAVLink (QGroundControl / Mission Planner) hoặc MSP. Cột "KĐL" = cần khởi động lại sau khi lưu.\n\n`;
md += "Nhóm: " + [...byGroup.keys()].filter((g) => byGroup.get(g).length).map((g) => `[${g}](#${g.toLowerCase().replace(/[^a-z0-9à-ỹđ ]/gi, "").trim().replace(/ +/g, "-")})`).join(" · ") + "\n";
for (const [g, list] of byGroup) {
  if (!list.length) continue;
  md += `\n## ${g}\n\n| Tham số | Mặc định | Giới hạn | Mô tả |\n|---|---|---|---|\n`;
  for (const p of list) {
    const u = unit(p.n), en = ENUMS[p.n];
    let d = HINT[p.n] || family(p.n);
    if (en && !/(^|[ :])0 [a-zà-ỹ]/i.test(d)) d += (d ? ". " : "") + en.map((t, k) => (t === undefined ? null : `${k} = ${t}`)).filter(Boolean).join(", ");
    md += `| \`${p.n}\`${p.r ? " (KĐL)" : ""} | ${fmt(p.v)}${u ? " " + u : ""} | ${fmt(p.min)} … ${fmt(p.max)} | ${esc(d)} |\n`;
  }
}
fs.writeFileSync(path.join(root, "docs", "PARAMS.md"), md);
console.log(`docs/PARAMS.md: ${dump.params.length} parameters in ${[...byGroup.values()].filter((l) => l.length).length} groups`);
