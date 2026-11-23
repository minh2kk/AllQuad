// Blackbox analyzer tests (Node >= 18):  node test/web/test_analysis.js
// 1. synthetic second-order system: step-response estimator measures the known overshoot
// 2. simulator flights with deliberately bad PIDs: suggestions point the right way
const { execFileSync } = require("child_process");
const fs = require("fs");
const path = require("path");
const AQ = require("../../src/web/ui/analysis.js");

const root = path.join(__dirname, "..", "..");
let fail = 0, total = 0;
const check = (ok, msg) => { total++; console.log(`[${ok ? "PASS" : "FAIL"}] ${msg}`); if (!ok) fail++; };

// ---- 1. synthetic: zeta 0.5 second-order system, wn = 2*pi*12 Hz, random setpoint steps at 500 Hz
{
  const rate = 500, n = rate * 40, zeta = 0.5, wn = 2 * Math.PI * 12, dt = 1 / rate;
  const sp = new Float32Array(n), y = new Float32Array(n), armed = new Uint8Array(n).fill(1);
  let x = 0, v = 0, target = 0, seed = 7;
  const rnd = () => ((seed = (seed * 1103515245 + 12345) % 2147483648) / 2147483648);
  for (let k = 0; k < n; k++) {
    if (k % 150 === 0) target = (rnd() - 0.5) * 600;
    sp[k] = target;
    for (let s = 0; s < 10; s++) { const a = wn * wn * (target - x) - 2 * zeta * wn * v; v += a * dt / 10; x += v * dt / 10; }
    y[k] = x + (rnd() - 0.5) * 2;
  }
  const st = AQ.stepResponse(sp, y, rate, armed);
  const truth = Math.exp(-Math.PI * zeta / Math.sqrt(1 - zeta * zeta));
  check(st && Math.abs(st.overshoot - truth) < 0.04, `step response overshoot ${(st.overshoot * 100).toFixed(1)}% vs true ${(truth * 100).toFixed(1)}%`);
}

// ---- 2. simulator flights
const sim = path.join(root, "build", "sim");
if (!fs.existsSync(sim)) { console.log("(build/sim missing: run tools/build_host.sh; simulator checks skipped)"); }
else {
  const out = path.join(root, "build", "web");
  const out_dir = () => out;
  fs.mkdirSync(out, { recursive: true });
  const fly = (name, env) => {
    const file = path.join(out, name + ".bin");
    execFileSync(sim, ["tune"], { env: { ...process.env, ...env, SIM_BLACKBOX: file }, stdio: "pipe" });
    const flights = AQ.decode(fs.readFileSync(file).buffer);
    const fl = flights[flights.length - 1];
    const res = AQ.analyze(fl);
    const params = { dterm_lpf_hz: 110, gyro_lpf_hz: 200, esc_telemetry: 1 };
    for (const a of AQ.AXES) Object.assign(params, { [a + "_p"]: fl.pid[a].p, [a + "_i"]: fl.pid[a].i, [a + "_d"]: fl.pid[a].d, [a + "_f"]: fl.pid[a].f });
    return { fl, res, sug: AQ.suggest(res, params), params };
  };
  const dir = (r, key) => {  // +1 suggests raising key, -1 lowering, 0 untouched
    const s = r.sug.find((x) => key in x.changes);
    return s ? Math.sign(s.changes[key] - r.params[key]) : 0;
  };
  const normal = fly("normal", {});
  {
    const weak = fly("weak", { SIM_WEAK_MOTOR: "3" });
    const w = weak.sug.find((x) => /Motor 3 quay chậm hơn/.test(x.title));
    check(!!w, `weak motor 3 (-15 %) found from ESC RPM: ${w ? w.title : weak.sug.map((x) => x.title).join(" | ")}`);
  }
  check(normal.fl.rateHz >= 450 && normal.fl.duration > 20, `sim log decoded: ${normal.fl.rateHz} Hz, ${normal.fl.duration.toFixed(0)} s`);
  {
    const size = fs.statSync(path.join(out, "normal.bin")).size, full = normal.fl.n * 94;
    check(normal.fl.header.encoding === "delta1" && full / size > 2.5,
          `blackbox delta frames: ${normal.fl.n} records in ${(size / 1024).toFixed(0)} kB (${(full / size).toFixed(1)}x smaller than full records)`);
    {
    const f2 = path.join(root, "build", "web", "ekf_yaw.bin");
    execFileSync(sim, ["ekf_yaw"], { env: { ...process.env, SIM_BLACKBOX: f2 }, stdio: "pipe" });
    const fy = AQ.decode(fs.readFileSync(f2).buffer).pop();
    const ry = AQ.report(fy, null);
    check(ry.nav && ry.nav.resets.heading >= 1 && ry.notes.some((x) => /Hướng EKF lệch/.test(x.text)),
      `EKF heading reset by the GPS referee logged and reported (${ry.nav ? ry.nav.resets.heading : "?"})`);
  }
  const py = execFileSync("python3", [path.join(root, "tools", "blackbox_decode.py"), path.join(out, "normal.bin"), path.join(out, "normal.csv")]).toString();
    const m = py.match(/(\d+) records/);
    check(m && +m[1] >= normal.fl.n && +m[1] <= normal.fl.n + 20, `Python decoder agrees (${m ? m[1] : "?"} records)`);
    const pe = py.match(/(\d+) ESC frames/);
    const e = normal.fl.esc;
    const rpmOk = e && [0, 1, 2, 3].every((k) => normal.res.esc && normal.res.esc.rpmMax[k] > 5000);
    check(e && Math.abs(e.n - normal.fl.duration * 50) < normal.fl.duration * 8 && rpmOk && pe && +pe[1] === e.n,
          `ESC frames: ${e ? e.n : 0} at ~50 Hz, RPM of 4 motors (max ${normal.res.esc ? normal.res.esc.rpmMax.map(Math.round).join("/") : "-"}), Python ${pe ? pe[1] : "?"}`);
    check(!normal.sug.some((x) => /quay chậm hơn/.test(x.title)), "no weak-motor warning on a healthy quad");
    // a damaged byte costs at most the frames up to the next full record
    const bad = new Uint8Array(fs.readFileSync(path.join(out, "normal.bin")));
    let kb = Math.floor(bad.length / 2);
    while (bad[kb] !== 0xA6) kb++;  // inside a delta frame (an ESC frame there would only lose that frame)
    bad[kb + 3] ^= 0x55;
    const fl2 = AQ.decode(bad.buffer).pop();
    check(fl2.n < normal.fl.n && fl2.n > normal.fl.n - 64, `one damaged byte drops ${normal.fl.n - fl2.n} records, decoding resumes at the next full record`);
    // a damaged full record: the delta frames built on it are dropped, never decoded against the wrong base
    const bad2 = new Uint8Array(fs.readFileSync(path.join(out, "normal.bin")));
    let k = Math.floor(bad2.length / 2);
    while (!(bad2[k] === 0xA5 && bad2[k + 1] === 94)) k++;
    bad2[k + 20] ^= 0x55;
    const fl3 = AQ.decode(bad2.buffer).pop();
    const ref = new Map();
    for (let r = 0; r < normal.fl.n; r++) ref.set(normal.fl.t[r], normal.fl.gyro[0][r] + "," + normal.fl.motor[0][r] + "," + normal.fl.alt[r]);
    let wrong = 0;
    for (let r = 0; r < fl3.n; r++) if (ref.get(fl3.t[r]) !== fl3.gyro[0][r] + "," + fl3.motor[0][r] + "," + fl3.alt[r]) wrong++;
    check(wrong === 0 && normal.fl.n - fl3.n >= 31 && normal.fl.n - fl3.n <= 64,
          `damaged full record: ${normal.fl.n - fl3.n} records dropped, ${wrong} decoded wrongly`);
  }
  const ov = normal.res.axes[0].step ? normal.res.axes[0].step.overshoot : 1;
  const flagged = normal.sug.some((s) => s.axis === "roll" && s.level === "warn");
  check(ov < 0.4 && (ov > 0.15) === flagged, `default tune: roll overshoot ${(ov * 100).toFixed(0)}%, flagged=${flagged}`);
  const high = fly("high_p", { SIM_TUNE_P: "2.2" });
  check(dir(high, "roll_p") < 0 || dir(high, "roll_d") < 0, "P x2.2: suggests lowering roll P or D");
  const low = fly("low_p", { SIM_TUNE_P: "0.5" });
  check(dir(low, "roll_p") > 0, "P x0.5: suggests raising roll P");
  check(low.sug.every((s) => Object.entries(s.changes).every(([k, v]) => isFinite(v))), "all suggested values are finite numbers");

  // ---- tuning assistant: merged plan, bounded steps, and a closed loop in the simulator
  {
    const pl = AQ.plan(low.sug, low.params, { roll_p: { min: 0, max: 200 } });
    const rp = pl.find((x) => x.name === "roll_p");
    check(rp && rp.to > rp.from && rp.to <= rp.from * 1.25 + 0.5, `plan: P x0.5 -> roll P ${rp ? rp.from + " -> " + rp.to : "?"} (one step, at most +25%)`);
    const ext = normal.res;
    check(ext.motorMean && ext.motorMean.length === 4 && normal.res.axes[0].lagMs != null,
          `flight facts: motor balance ${ext.motorMean ? ext.motorMean.map((m) => (m * 100).toFixed(0)).join("/") : "?"} %, roll lag ${normal.res.axes[0].lagMs} ms`);
    // flight summary of the simulator flight: battery, hover throttle, vibration, notes
    const rep = AQ.report(normal.fl, normal.res);
    check(rep.armedSec > 10 && rep.arms >= 1 && rep.cells >= 3 && rep.vMin > 0 && rep.vMin <= rep.vRest && rep.hover > 0.1 && rep.hover < 0.8 &&
          rep.notes.length && rep.notes.every((x) => ["ok", "info", "warn", "bad"].includes(x.level)),
          `flight summary: ${rep.armedSec.toFixed(0)} s armed, ${rep.cells}S ${rep.vRest.toFixed(2)} -> ${rep.vMin.toFixed(2)} V, hover ${(rep.hover * 100).toFixed(0)}%, ` +
          `${rep.mah != null ? rep.mah.toFixed(0) + " mAh" : "no current"}, vibration ${rep.vib.toFixed(1)}, ${rep.notes.length} notes`);
    {  // a weak pack: every voltage scaled down -> a battery warning comes first
      const w = Object.assign({}, normal.fl, { vbat: normal.fl.vbat.map((v) => v * 0.83) });
      const rw = AQ.report(w, normal.res);
      check(rw.notes[0].level === "bad" || rw.notes[0].level === "warn" && /V\/cell|Sụt áp/.test(rw.notes[0].text), `weak pack: "${rw.notes[0].text.slice(0, 60)}…"`);
    }
    // apply the plan round by round, as a pilot would: fly, analyse, take the plan, fly again
    const scale = { P: 0.5, I: 1, D: 1, F: 1 }, base = { P: 45, I: 80, D: 30, F: 60 };
    const ov0 = low.res.axes[0].step.overshoot;
    let r = low, ovN = ov0;
    for (let round = 1; round <= 3; round++) {
      const plan = AQ.plan(r.sug, r.params, null);
      for (const k of ["p", "i", "d", "f"]) {
        const e = plan.find((x) => x.name === "roll_" + k);
        if (e) scale[k.toUpperCase()] = e.to / base[k.toUpperCase()];
      }
      r = fly(`assist_${round}`, { SIM_TUNE_P: String(scale.P), SIM_TUNE_I: String(scale.I), SIM_TUNE_D: String(scale.D), SIM_TUNE_F: String(scale.F) });
      if (!r.res.axes[0].step) break;
      ovN = r.res.axes[0].step.overshoot;
    }
    check(ovN < ov0 - 0.1, `assistant closed loop from P x0.5: roll overshoot ${(ov0 * 100).toFixed(0)}% -> ${(ovN * 100).toFixed(0)}% in 3 rounds ` +
          `(P ${(scale.P * 45).toFixed(0)}, I ${(scale.I * 80).toFixed(0)}, D ${(scale.D * 30).toFixed(0)}, F ${(scale.F * 60).toFixed(0)})`);
  }

  // in-flight autotune cross-checked by this analyser: fly the tuned roll gains, measure the step
  for (const p of [0.5, 2.2]) {
    const out = path.join(out_dir(), `autotune_${p}.txt`);
    execFileSync(sim, ["autotune"], { env: { ...process.env, SIM_AT_P: String(p), SIM_AT_OUT: out }, stdio: "pipe" });
    const [rp, rd] = fs.readFileSync(out, "utf8").trim().split(/\s+/).map(Number);
    const before = fly(`at_before_${p}`, { SIM_TUNE_P: String(p) }).res.axes[0].step.overshoot;
    const after = fly(`at_after_${p}`, { SIM_TUNE_P: String(rp / 45), SIM_TUNE_D: String(rd / 30) }).res.axes[0].step.overshoot;
    check(after < before - 0.05, `autotune from P x${p}: roll ${rp.toFixed(0)}/${rd.toFixed(0)}, analyser overshoot ${(before * 100).toFixed(0)}% -> ${(after * 100).toFixed(0)}%`);
  }
}
// ---- 2b. high-rate gyro frames (0xA8, bb_gyro_s): 2 s of a 610 Hz motor line at 4 kHz inside a sim log
{
  const out = path.join(root, "build", "web");
  const base = fs.readFileSync(path.join(out, "normal.bin"));
  const end = base.indexOf("H end\n") + 6;
  const rate = 4000, N = 16, frames = [];
  for (let f = 0; f < (2 * rate) / N; f++) {
    const b = Buffer.alloc(9 + N * 12);
    b[0] = 0xA8; b[1] = N; b.writeUInt32LE(5000000 + Math.round(f * N * 1e6 / rate), 2); b.writeUInt16LE(rate, 6);
    for (let k = 0; k < N; k++) {
      const t = (f * N + k) / rate, raw = 40 * Math.sin(2 * Math.PI * 610 * t) + 5 * Math.sin(2 * Math.PI * 30 * t);
      b.writeInt16LE(Math.round(raw * 10), 8 + k * 12);
      b.writeInt16LE(Math.round(5 * Math.sin(2 * Math.PI * 30 * t) * 10), 8 + k * 12 + 6);
    }
    let x = 0;
    for (let i = 0; i < b.length - 1; i++) x ^= b[i];
    b[b.length - 1] = x;
    frames.push(b);
  }
  const fl = AQ.decode(Buffer.concat([base.subarray(0, end), ...frames, base.subarray(end)]).buffer)[0];
  const hf = fl.hf && fl.hf[0];
  const sp = hf && AQ.spectrum(hf.raw[0], hf.rate, 1024);
  const pk = sp && AQ.bandPeak(sp, 100, hf.rate / 2 - 10);
  const plain = AQ.decode(base.buffer.slice(base.byteOffset, base.byteOffset + base.length))[0];
  check(hf && hf.rate === 4000 && Math.abs(hf.duration - 2) < 0.01 && Math.abs(pk.hz - 610) < 8 && fl.n === plain.n,
    `high-rate gyro: ${hf ? hf.duration.toFixed(2) + " s at " + hf.rate + " Hz" : "none"}, peak ${pk ? pk.hz.toFixed(0) : "?"} Hz (true 610, above the 250 Hz of the regular log), records untouched`);
}

// ---- 2b. navigation frames (0xA9): the simulator's EKF fault flight (baro drift, compass interference)
if (fs.existsSync(sim)) {
  const file = path.join(root, "build", "web", "ekf_faults.bin");
  execFileSync(sim, ["ekf_faults"], { env: { ...process.env, SIM_BLACKBOX: file }, stdio: "pipe" });
  const fl = AQ.decode(fs.readFileSync(file).buffer).pop();
  const rep = AQ.report(fl, AQ.analyze(fl));
  const nv = rep.nav;
  const has = (re) => rep.notes.some((x) => re.test(x.text));
  check(fl.nav && fl.nav.n > 700 && nv && nv.ekf && nv.faultSec.baro > 5 && nv.faultSec.mag > 1 && has(/Baro bị loại/) && has(/La bàn lệch/),
    `navigation frames: ${fl.nav ? fl.nav.n : 0}, baro rejected ${nv ? nv.faultSec.baro.toFixed(0) : "?"} s, compass ${nv ? nv.faultSec.mag.toFixed(0) : "?"} s -> notes`);
  check(nv && nv.tiltErrMax < 10 && !nv.resets.tilt && !nv.resets.filter, `EKF tilt follows the AHRS (max difference ${nv ? nv.tiltErrMax.toFixed(1) : "?"} deg), no resets`);
  fl.nav.resets[3][fl.nav.n - 1] += 2;  // as if the EKF tilt had been reset twice
  check(AQ.report(fl, null).notes.some((x) => x.level === "warn" && /tự đặt lại 2 lần/.test(x.text)), "tilt resets reported");
  const py = execFileSync("python3", [path.join(root, "tools", "blackbox_decode.py"), file], { encoding: "utf8" });
  const nav = fs.readFileSync(file.replace(/\.bin$/, "_nav.csv"), "utf8").trim().split("\n");
  const hdr = nav[0].split(","), last = nav[nav.length - 1].split(",");
  check(/navigation frames/.test(py) && nav.length - 1 === fl.nav.n && +last[hdr.indexOf("est")] === 1 && hdr.includes("resets_tilt"),
    `blackbox_decode.py: ${nav.length - 1} navigation rows -> _nav.csv`);
}

// ---- 3. exports: .bbl read back with a minimal Betaflight-format decoder (I frames, variable-byte)
{
  const n = 300, t0 = 1000.5;
  const mk = (k) => Array.from({ length: k }, () => new Float32Array(n));
  const fl = { n, t: new Float64Array(n), gyro: mk(3), gyroRaw: mk(3), sp: mk(3), P: mk(3), I: mk(3), D: mk(3), F: mk(3), motor: mk(4),
    thr: new Float32Array(n), rc: mk(4), att: mk(3), alt: new Float32Array(n), vbat: new Float32Array(n), cur: new Float32Array(n),
    mode: new Uint8Array(n), armed: new Uint8Array(n), rateHz: 500, header: { gyro_lpf: "pt1 90" } };
  for (let r = 0; r < n; r++) {
    fl.t[r] = t0 + r * 0.002;
    for (let a = 0; a < 3; a++) { fl.gyro[a][r] = 300 * Math.sin(r * 0.05 + a); fl.sp[a][r] = -200 + r; fl.P[a][r] = (r % 50) - 25; }
    for (let m = 0; m < 4; m++) fl.motor[m][r] = r < 10 ? 0 : 0.2 + m * 0.1;
    fl.rc[0][r] = 1500 + r; fl.rc[1][r] = 1400; fl.rc[2][r] = 1200; fl.rc[3][r] = 1600; fl.vbat[r] = 16.2; fl.cur[r] = -1.5;
  }
  const b = AQ.toBbl(fl, { version: "t", board: "B" });
  const txt = Buffer.from(b).toString("latin1");
  const hdr = {};
  let i = 0;
  while (b[i] === 72) { const e = txt.indexOf("\n", i); const ln = txt.slice(i + 2, e); const c = ln.indexOf(":"); hdr[ln.slice(0, c)] = ln.slice(c + 1); i = e + 1; }
  const names = hdr["Field I name"].split(","), enc = hdr["Field I encoding"].split(",").map(Number);
  const uvb = () => { let v = 0, s = 0, x; do { x = b[i++]; v += (x & 0x7F) * 2 ** s; s += 7; } while (x & 0x80); return v; };
  const rows = [];
  while (b[i] === 73) { i++; rows.push(names.map((_, k) => { const u = uvb(); return enc[k] === 0 ? (u % 2 ? -(u + 1) / 2 : u / 2) : u; })); }
  const col = (nm) => rows.map((r) => r[names.indexOf(nm)]);
  const g = col("gyroADC[1]"), mo = col("motor[2]"), tm = col("time"), rcR = col("rcCommand[0]"), rcT = col("rcCommand[3]"), am = col("amperageLatest");
  check(rows.length === n && /Betaflight 4\.4\.0/.test(hdr["Firmware revision"]) && hdr["I interval"] === "1" && hdr.aq_gyro_lpf === "pt1 90",
    `.bbl export: ${rows.length} frames, header ${hdr["Firmware revision"]}`);
  check(g.every((v, r) => v === Math.round(fl.gyro[1][r])) && tm[1] === 2000 && mo[0] === 0 && mo[50] === Math.round(48 + 0.4 * 1999)
    && rcR[5] === 5 && rcT[5] === 1200 && am[0] === -150 && b.slice(i).toString() === [69, 255, ...Buffer.from("End of log"), 0].toString(),
    ".bbl export: gyro, time, motor (DShot range), rcCommand (AETR -> roll/pitch/yaw/throttle), current, end-of-log event");
  const csv = AQ.toCsv(fl).trim().split("\n");
  check(csv.length === n + 1 && csv[0].split(",").length === names.length + 6 && csv[1].split(",")[names.indexOf("vbatLatest")] === "16.20",
    `CSV export: ${csv.length - 1} rows, ${csv[0].split(",").length} columns, voltage in volts`);
}
console.log(`\n${total - fail}/${total} analyzer tests passed`);
process.exit(fail ? 1 : 0);
