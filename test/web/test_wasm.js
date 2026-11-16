// The firmware compiled to WebAssembly (tools/build_wasm.sh) answers the configurator's link
// protocol: /api/hello, /api/params, /api/status, and a CLI command, exactly like a board.
//   node test/web/test_wasm.js
const fs = require("fs"), path = require("path");
const ROOT = path.join(__dirname, "..", "..");
const wasmPath = path.join(ROOT, "build", "web", "allquad.wasm");
let fail = 0, total = 0;
const check = (ok, msg) => { total++; if (!ok) fail++; console.log(`[${ok ? "PASS" : "FAIL"}] ${msg}`); };

function crc16(b, from, to, crc = 0xFFFF) {
  for (let i = from; i < to; i++) {
    crc ^= b[i] << 8;
    for (let k = 0; k < 8; k++) crc = crc & 0x8000 ? ((crc << 1) ^ 0x1021) & 0xFFFF : (crc << 1) & 0xFFFF;
  }
  return crc;
}
function frame(seq, p) {
  const e = new TextEncoder().encode(p), len = e.length + 1, f = new Uint8Array(len + 7);
  f[0] = 0xA7; f[1] = 0x51; f[2] = seq; f[3] = len & 255; f[4] = len >> 8; f.set(e, 5);
  const c = crc16(f, 2, 5 + len); f[5 + len] = c & 255; f[6 + len] = c >> 8;
  return f;
}

(async () => {
  if (!fs.existsSync(wasmPath)) { console.log("(build/web/allquad.wasm missing: run tools/build_wasm.sh)"); process.exit(0); }
  let mem;
  const dv = () => new DataView(mem.buffer);
  const t0 = Date.now();
  const wasi = new Proxy({
    fd_write(fd, iovs, n, out) { let t = 0; for (let i = 0; i < n; i++) t += dv().getUint32(iovs + i * 8 + 4, true); dv().setUint32(out, t, true); return 0; },
    clock_time_get(id, p, out) { dv().setBigUint64(out, BigInt((Date.now() - t0) * 1e6), true); return 0; },
    environ_sizes_get(c, s) { dv().setUint32(c, 0, true); dv().setUint32(s, 0, true); return 0; },
    environ_get() { return 0; }, fd_close() { return 0; }, fd_seek() { return 70; },
    proc_exit(c) { throw new Error("exit " + c); },
  }, { get: (o, k) => o[k] || (() => 52) });
  const bytes = fs.readFileSync(wasmPath);
  const { instance } = await WebAssembly.instantiate(bytes, { wasi_snapshot_preview1: wasi });
  const ex = instance.exports;
  mem = ex.memory;
  const imports = WebAssembly.Module.imports(new WebAssembly.Module(bytes)).map((i) => i.name);
  check(bytes.length < 600000, `module ${(bytes.length / 1024).toFixed(0)} kB, imports: ${imports.join(" ")}`);
  if (ex._initialize) ex._initialize();
  ex.aq_init();
  let simUs = 0, rx = Buffer.alloc(0), seq = 0;
  const step = (ms) => {
    for (let k = 0; k < ms / 4; k++) {
      simUs += 4000;
      const n = ex.aq_step(simUs);
      if (n > 0) rx = Buffer.concat([rx, Buffer.from(new Uint8Array(mem.buffer, ex.aq_out_buf(), n))]);
    }
  };
  // one request -> the response document (all pages)
  const request = (p) => {
    let out = Buffer.alloc(0), total = -1;
    for (let tries = 0; tries < 200; tries++) {
      const q = out.length ? p + (p.includes("?") ? "&" : "?") + "o=" + out.length : p;
      const f = frame(seq = (seq + 1) & 255, q);
      new Uint8Array(mem.buffer, ex.aq_in_buf(), f.length).set(f);
      ex.aq_input(f.length);
      let got = null;
      for (let w = 0; w < 100 && !got; w++) {
        step(8);
        for (let i = 0; i + 17 <= rx.length; i++) {
          if (rx[i] !== 0xA7 || rx[i + 1] !== 0x52 || rx[i + 2] !== seq) continue;
          const len = rx[i + 13] | (rx[i + 14] << 8);
          if (i + 17 + len > rx.length) break;
          got = { status: rx.readUInt16LE(i + 3), total: rx.readUInt32LE(i + 5), data: rx.subarray(i + 15, i + 15 + len) };
          rx = rx.subarray(i + 17 + len);
          break;
        }
      }
      if (!got) return null;
      out = Buffer.concat([out, got.data]);
      total = got.total;
      if (got.status !== 200 || out.length >= total) return { status: got.status, text: out.toString("utf8") };
    }
    return null;
  };
  step(1500);  // boot, gyro calibration
  const h = request("/api/hello");
  check(h && h.status === 200 && /"board"/.test(h.text), "hello: " + (h ? h.text.slice(0, 80) : "no answer"));
  const p = request("/api/params");
  let params = null;
  try { params = JSON.parse(p.text).params; } catch (e) {}
  check(params && params.length > 200 && params.some((x) => x.n === "est_type") && params.some((x) => x.n === "vtxt_mw1"),
    `params: ${params ? params.length : 0} parameters, new v17 fields present`);
  step(3000);
  const s = request("/api/status");
  let st = null;
  try { st = JSON.parse(s.text); } catch (e) {}
  check(st && st.ekf && st.ekf.on && typeof st.vibe === "number", `status: EKF ${st && st.ekf && st.ekf.on ? "running" : "off"}, vibration ${st ? st.vibe : "?"}`);
  const set = request("/api/set?roll_p=51");
  const p2 = JSON.parse(request("/api/params").text).params.find((x) => x.n === "roll_p");
  check(set && set.status === 200 && p2.v === 51, "set roll_p=51 applied in the virtual board");
  console.log(`\n${total - fail}/${total} wasm tests passed`);
  process.exit(fail ? 1 : 0);
})().catch((e) => { console.log("[FAIL]", e); process.exit(1); });
