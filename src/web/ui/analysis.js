// AllQuad blackbox analysis (browser + Node). Pure functions, no dependencies.
//   decode(ArrayBuffer)            -> flights[]
//   spectrum(x, rateHz)            -> {f, amp}  (Welch, Hann, 50% overlap)
//   stepResponse(sp, gyro, rateHz) -> {t, y, overshoot, riseMs, ss, windows}
//   analyze(flight)                -> per-axis metrics
//   suggest(analysis, params)      -> [{axis, level, title, detail, changes}]
//   report(flight, analysis)       -> flight summary (time, battery, current, height, vibration) + notes
"use strict";
(function (root) {
  const AXES = ["roll", "pitch", "yaw"];

  // ------------------------------------------------------------------ decoding
  // Record layout: src/logging/blackbox.h (packed, little-endian)
  const REC = 94;
  const ESC = 23;  // ESC frame (0xA7): time, RPM/10 x4, temperature x4, voltage, current
  const NAV = 41;  // navigation frame (0xA9): estimator health ~10 Hz (src/logging/blackbox.h NavRecord)

  // delta frames (0xA6, "H encoding delta1"): field table of src/logging/blackbox.cpp
  const FIELDS = (() => {
    const t = [];
    let off = 2;
    const add = (c, size, sign) => { for (let k = 0; k < c; k++) { t.push([off, size, sign]); off += size; } };
    add(1, 4, 0); add(21, 2, 1); add(4, 2, 0); add(1, 2, 0); add(4, 2, 0); add(3, 2, 1); add(1, 4, 1); add(1, 2, 1);
    add(2, 4, 1); add(1, 2, 0); add(1, 2, 1); add(3, 1, 0);
    return t;
  })();
  const MASK = (FIELDS.length + 7) >> 3;
  function getF(dv, [o, size, sign]) {
    if (size === 4) return sign ? dv.getInt32(o, true) : dv.getUint32(o, true);
    if (size === 2) return sign ? dv.getInt16(o, true) : dv.getUint16(o, true);
    return sign ? dv.getInt8(o) : dv.getUint8(o);
  }
  function setF(dv, [o, size, sign], v) {
    if (size === 4) sign ? dv.setInt32(o, v, true) : dv.setUint32(o, ((v % 4294967296) + 4294967296) % 4294967296, true);
    else if (size === 2) sign ? dv.setInt16(o, v, true) : dv.setUint16(o, v & 0xFFFF, true);
    else sign ? dv.setInt8(o, v) : dv.setUint8(o, v & 0xFF);
  }
  // expands one delta frame at i on top of prev; returns [record, length] or null
  function expandDelta(u8, i, prev) {
    if (u8[i + 1] !== prev[REC - 1]) return null;  // built on a record we do not have
    let j = i + 2 + MASK;
    if (j > u8.length) return null;
    const rec = prev.slice();
    const dv = new DataView(rec.buffer);
    for (let f = 0; f < FIELDS.length; f++) {
      if (!(u8[i + 2 + (f >> 3)] & (1 << (f & 7)))) continue;
      let z = 0, mul = 1, b;
      do {
        if (j >= u8.length || mul > 2 ** 63) return null;
        b = u8[j++];
        z += (b & 0x7F) * mul;
        mul *= 128;
      } while (b & 0x80);
      const d = z % 2 === 0 ? z / 2 : -(z + 1) / 2;
      setF(dv, FIELDS[f], getF(dv, FIELDS[f]) + d);
    }
    if (j >= u8.length) return null;
    let x = 0;
    for (let k = i; k < j; k++) x ^= u8[k];
    if (x !== u8[j]) return null;
    // the record's own checksum, as if it had been written in full
    let c = 0;
    for (let k = 0; k < REC - 1; k++) c ^= rec[k];
    rec[REC - 1] = c;
    return [rec, j + 1 - i];
  }

  function decode(buf) {
    const u8 = new Uint8Array(buf);
    const flights = [];
    let cur = null, prev = null;
    let i = 0;
    const text = (a, b) => String.fromCharCode.apply(null, u8.subarray(a, b));
    while (i < u8.length) {
      if (u8[i] === 0x48 && u8[i + 1] === 0x20) {  // "H "
        let e = i;
        while (e < u8.length && u8[e] !== 10) e++;
        const line = text(i + 2, e);
        if (line.startsWith("AllQuad blackbox")) {
          cur = { header: {}, recs: [], esc: [], gyro: [], nav: [] };
          prev = null;
          flights.push(cur);
        }
        if (cur) {
          const sp = line.indexOf(" ");
          if (sp > 0) cur.header[line.slice(0, sp)] = line.slice(sp + 1);
        }
        i = e + 1;
        continue;
      }
      if (u8[i] === 0xA5 && u8[i + 1] === REC && i + REC <= u8.length && cur) {
        let x = 0;
        for (let k = 0; k < REC - 1; k++) x ^= u8[i + k];
        if (x === u8[i + REC - 1]) {
          prev = u8.slice(i, i + REC);
          cur.recs.push(prev);
          i += REC;
          continue;
        }
      }
      if (u8[i] === 0xA7 && u8[i + 1] === ESC && i + ESC <= u8.length && cur) {
        let x = 0;
        for (let k = 0; k < ESC - 1; k++) x ^= u8[i + k];
        if (x === u8[i + ESC - 1]) {
          cur.esc.push(u8.slice(i, i + ESC));
          i += ESC;
          continue;
        }
      }
      if (u8[i] === 0xA9 && u8[i + 1] === NAV && i + NAV <= u8.length && cur) {
        let x = 0;
        for (let k = 0; k < NAV - 1; k++) x ^= u8[i + k];
        if (x === u8[i + NAV - 1]) {
          cur.nav.push(u8.slice(i, i + NAV));
          i += NAV;
          continue;
        }
      }
      if (u8[i] === 0xA8 && cur && i + 9 + u8[i + 1] * 12 <= u8.length && u8[i + 1]) {  // high-rate gyro (bb_gyro_s)
        const n = 9 + u8[i + 1] * 12;
        let x = 0;
        for (let k = 0; k < n - 1; k++) x ^= u8[i + k];
        if (x === u8[i + n - 1]) {
          cur.gyro.push(u8.slice(i, i + n));
          i += n;
          continue;
        }
      }
      if (u8[i] === 0xA6 && cur && prev) {
        const r = expandDelta(u8, i, prev);
        if (r) {
          prev = r[0];
          cur.recs.push(prev);
          i += r[1];
          continue;
        }
        prev = null;  // damaged: wait for the next full record
      }
      i++;
    }
    return flights.filter(f => f.recs.length > 10).map((f) => {
      const all = new Uint8Array(f.recs.length * REC);
      f.recs.forEach((r, k) => all.set(r, k * REC));
      f.rows = f.recs.map((_, k) => k * REC);
      f.bytes = all;
      delete f.recs;
      const c = columns(new DataView(all.buffer), f);
      c.esc = escColumns(f.esc);
      c.hf = gyroColumns(f.gyro);
      c.nav = navColumns(f.nav);
      return c;
    });
  }

  // ESC frames -> { n, t, rpm[4], temp[4], volt, cur } (NaN = that ESC sent nothing), null if none
  function escColumns(frames) {
    if (!frames || !frames.length) return null;
    const n = frames.length;
    const mk = () => Array.from({ length: 4 }, () => new Float32Array(n));
    const e = { n, t: new Float64Array(n), rpm: mk(), temp: mk(), volt: new Float32Array(n), cur: new Float32Array(n) };
    frames.forEach((b, k) => {
      const dv = new DataView(b.buffer, b.byteOffset, b.byteLength);
      e.t[k] = dv.getUint32(2, true) / 1e6;
      for (let m = 0; m < 4; m++) {
        const r = dv.getUint16(6 + m * 2, true), tc = dv.getUint8(14 + m);
        e.rpm[m][k] = r === 0xFFFF ? NaN : r * 10;
        e.temp[m][k] = tc === 255 ? NaN : tc;
      }
      const v = dv.getUint16(18, true), a = dv.getInt16(20, true);
      e.volt[k] = v ? v / 100 : NaN;
      e.cur[k] = a === -32768 ? NaN : a / 10;
    });
    return e;
  }

  // navigation frames -> columns (test ratios as ratio to the gate, angles in deg), null if none
  function navColumns(frames) {
    if (!frames || !frames.length) return null;
    const n = frames.length, f32 = () => new Float32Array(n), u8 = () => new Uint8Array(n);
    const v = { n, t: new Float64Array(n), est: u8(), heightSrc: u8(), headingSrc: u8(), sats: u8(), faults: new Uint16Array(n),
      tr: [0, 1, 2, 3, 4, 5].map(f32), ekfAtt: [0, 1, 2].map(f32), ahrsYaw: f32(), posStd: f32(), hgtStd: f32(), yawStd: f32(),
      hAcc: f32(), resets: [0, 1, 2, 3, 4, 5].map(u8) };
    frames.forEach((b, k) => {
      const dv = new DataView(b.buffer, b.byteOffset, b.byteLength);
      v.t[k] = dv.getUint32(2, true) / 1e6;
      v.est[k] = b[6]; v.heightSrc[k] = b[7]; v.headingSrc[k] = b[8]; v.sats[k] = b[9];
      v.faults[k] = dv.getUint16(10, true);
      for (let i = 0; i < 6; i++) v.tr[i][k] = b[12 + i] / 50;
      for (let i = 0; i < 3; i++) v.ekfAtt[i][k] = dv.getInt16(18 + i * 2, true) / 10;
      v.ahrsYaw[k] = dv.getInt16(24, true) / 10;
      v.posStd[k] = dv.getUint16(26, true) / 100;
      v.hgtStd[k] = dv.getUint16(28, true) / 100;
      v.yawStd[k] = dv.getUint16(30, true) / 10;
      const h = dv.getUint16(32, true);
      v.hAcc[k] = h === 0xFFFF ? NaN : h / 100;
      for (let i = 0; i < 6; i++) v.resets[i][k] = b[34 + i];
    });
    return v;
  }
  // EKF_* fault bits (src/estimation/ekf.h)
  const NAV_FAULT = { gpsGlitch: 1, gpsVel: 2, baro: 4, mag: 8, noAiding: 16, reset: 32, flow: 64, heading: 128 };
  // estimator health of one flight: resets during the flight, seconds each fault was set, worst
  // EKF-vs-AHRS tilt difference, GPS satellites / accuracy
  function navStats(fl) {
    const v = fl.nav;
    if (!v || v.n < 2) return null;
    const out = { ekf: false, duration: v.t[v.n - 1] - v.t[0], resets: {}, faultSec: {}, tiltErrMax: 0, satsMin: null, hAccMax: null, trMax: [0, 0, 0, 0, 0, 0] };
    const names = ["pos", "vel", "filter", "tilt", "heading", "baro"];
    names.forEach((nm, i) => { out.resets[nm] = (v.resets[i][v.n - 1] - v.resets[i][0] + 256) % 256; });
    for (const nm of Object.keys(NAV_FAULT)) out.faultSec[nm] = 0;
    let r = 0;
    for (let k = 0; k < v.n; k++) {
      const dt = k ? Math.min(0.5, Math.max(0, v.t[k] - v.t[k - 1])) : 0;
      if (v.est[k]) out.ekf = true;
      for (const [nm, bit] of Object.entries(NAV_FAULT)) if (v.faults[k] & bit) out.faultSec[nm] += dt;
      for (let i = 0; i < 6; i++) out.trMax[i] = Math.max(out.trMax[i], v.tr[i][k]);
      if (v.sats[k]) {
        out.satsMin = out.satsMin === null ? v.sats[k] : Math.min(out.satsMin, v.sats[k]);
        if (isFinite(v.hAcc[k])) out.hAccMax = Math.max(out.hAccMax || 0, v.hAcc[k]);
      }
      if (v.est[k] && fl.n) {
        while (r + 1 < fl.n && fl.t[r + 1] <= v.t[k]) r++;
        const e = Math.max(Math.abs(v.ekfAtt[0][k] - fl.att[0][r]), Math.abs(v.ekfAtt[1][k] - fl.att[1][r]));
        if (Math.abs(fl.att[1][r]) < 70) out.tiltErrMax = Math.max(out.tiltErrMax, e);  // Euler angles near 90 deg pitch mean little
      }
    }
    return out;
  }

  // high-rate gyro frames -> one series per capture: {rate, t0, raw[3], filt[3], n}; a gap of more than two
  // frames starts a new capture
  function gyroColumns(frames) {
    if (!frames || !frames.length) return null;
    const caps = [];
    let cur = null, lastT = 0;
    for (const b of frames) {
      const dv = new DataView(b.buffer, b.byteOffset, b.byteLength);
      const n = b[1], t = dv.getUint32(2, true) / 1e6, rate = dv.getUint16(6, true);
      if (!rate) continue;
      if (!cur || cur.rate !== rate || t - lastT > 3 * n / rate || t < lastT) { cur = { rate, t0: t, s: [] }; caps.push(cur); }
      lastT = t;
      for (let k = 0; k < n; k++) {
        const v = [];
        for (let a = 0; a < 6; a++) v.push(dv.getInt16(8 + (k * 6 + a) * 2, true) / 10);
        cur.s.push(v);
      }
    }
    return caps.filter((c) => c.s.length >= 256).map((c) => {
      const n = c.s.length, mk = () => [0, 1, 2].map(() => new Float32Array(n));
      const raw = mk(), filt = mk();
      c.s.forEach((v, k) => { for (let a = 0; a < 3; a++) { raw[a][k] = v[a]; filt[a][k] = v[a + 3]; } });
      return { rate: c.rate, t0: c.t0, n, duration: n / c.rate, raw, filt };
    });
  }

  // per motor: RPM per unit of command in steady flight (a weak motor / damaged prop needs more command
  // for the same RPM), peak RPM, peak temperature
  function escStats(fl, steady) {
    const e = fl.esc;
    if (!e) return null;
    const out = { rpmPerCmd: [null, null, null, null], rpmMax: [0, 0, 0, 0], tempMax: [null, null, null, null], haveRpm: false, haveTemp: false };
    const sumR = [0, 0, 0, 0], sumC = [0, 0, 0, 0], cnt = [0, 0, 0, 0];
    let r = 0;
    for (let k = 0; k < e.n; k++) {
      while (r + 1 < fl.n && fl.t[r + 1] <= e.t[k]) r++;  // record at the same time
      for (let m = 0; m < 4; m++) {
        const rpm = e.rpm[m][k], tc = e.temp[m][k];
        if (!isNaN(rpm)) {
          out.haveRpm = true;
          if (rpm > out.rpmMax[m]) out.rpmMax[m] = rpm;
          if (steady[r] && fl.motor[m][r] > 0.1) { sumR[m] += rpm; sumC[m] += fl.motor[m][r]; cnt[m]++; }
        }
        if (!isNaN(tc)) { out.haveTemp = true; out.tempMax[m] = Math.max(out.tempMax[m] || 0, tc); }
      }
    }
    for (let m = 0; m < 4; m++) if (cnt[m] > 20 && sumC[m] > 0) out.rpmPerCmd[m] = sumR[m] / sumC[m];
    return out;
  }

  function columns(dv, f) {
    const n = f.rows.length;
    const mk = (k) => Array.from({ length: k }, () => new Float32Array(n));
    const c = {
      header: f.header, n,
      t: new Float64Array(n), gyro: mk(3), gyroRaw: mk(3), sp: mk(3), P: mk(3), I: mk(3), D: mk(3), F: mk(3),
      motor: mk(4), thr: new Float32Array(n), rc: mk(4), att: mk(3), alt: new Float32Array(n),
      vz: new Float32Array(n), vbat: new Float32Array(n), cur: new Float32Array(n),
      mode: new Uint8Array(n), armed: new Uint8Array(n), posN: new Float32Array(n), posE: new Float32Array(n),
    };
    for (let r = 0; r < n; r++) {
      let o = f.rows[r] + 2;
      c.t[r] = dv.getUint32(o, true) / 1e6; o += 4;
      const i16 = () => { const v = dv.getInt16(o, true); o += 2; return v; };
      const u16 = () => { const v = dv.getUint16(o, true); o += 2; return v; };
      const i32 = () => { const v = dv.getInt32(o, true); o += 4; return v; };
      for (const arr of [c.gyro, c.gyroRaw, c.sp, c.P, c.I, c.D, c.F])
        for (let a = 0; a < 3; a++) arr[a][r] = i16() / 10;
      for (let m = 0; m < 4; m++) c.motor[m][r] = u16() / 1000;
      c.thr[r] = u16() / 1000;
      for (let k = 0; k < 4; k++) c.rc[k][r] = u16();
      for (let a = 0; a < 3; a++) c.att[a][r] = i16() / 10;
      c.alt[r] = i32() / 100;
      c.vz[r] = i16() / 100;
      c.posN[r] = i32() / 100;  // m from home (0, 0 = no position)
      c.posE[r] = i32() / 100;
      c.vbat[r] = u16() / 100;
      c.cur[r] = i16() / 100;
      c.mode[r] = dv.getUint8(o); o += 2;  // mode, navPhase
      c.armed[r] = dv.getUint8(o) & 1;
    }
    // sample rate from the median time step
    const d = [];
    for (let r = 1; r < Math.min(n, 2000); r++) d.push(c.t[r] - c.t[r - 1]);
    d.sort((a, b) => a - b);
    c.rateHz = d.length ? Math.round(1 / d[d.length >> 1]) : 0;
    c.duration = n > 1 ? c.t[n - 1] - c.t[0] : 0;
    c.pid = {};
    for (const ax of AXES) {
      const h = c.header["pid_" + ax];
      if (h) { const v = h.split(" ").map(Number); c.pid[ax] = { p: v[0], i: v[1], d: v[2], f: v[3] }; }
    }
    return c;
  }

  // ------------------------------------------------------------------ FFT
  function fft(re, im, inverse) {
    const n = re.length;
    for (let i = 1, j = 0; i < n; i++) {
      let bit = n >> 1;
      for (; j & bit; bit >>= 1) j ^= bit;
      j ^= bit;
      if (i < j) { [re[i], re[j]] = [re[j], re[i]]; [im[i], im[j]] = [im[j], im[i]]; }
    }
    for (let len = 2; len <= n; len <<= 1) {
      const ang = (inverse ? 2 : -2) * Math.PI / len, wr = Math.cos(ang), wi = Math.sin(ang);
      for (let i = 0; i < n; i += len) {
        let cr = 1, ci = 0;
        for (let k = 0; k < len / 2; k++) {
          const u = i + k, v = u + len / 2;
          const tr = re[v] * cr - im[v] * ci, ti = re[v] * ci + im[v] * cr;
          re[v] = re[u] - tr; im[v] = im[u] - ti; re[u] += tr; im[u] += ti;
          const nc = cr * wr - ci * wi; ci = cr * wi + ci * wr; cr = nc;
        }
      }
    }
    if (inverse) for (let i = 0; i < n; i++) { re[i] /= n; im[i] /= n; }
  }

  const hann = (n) => Float64Array.from({ length: n }, (_, i) => 0.5 - 0.5 * Math.cos(2 * Math.PI * i / (n - 1)));

  // Welch amplitude spectrum (deg/s RMS-ish per bin) of segments where mask (optional) is true
  function spectrum(x, rateHz, nfft = 256, mask = null) {
    const w = hann(nfft);
    let wsum = 0;
    for (const v of w) wsum += v;
    const acc = new Float64Array(nfft / 2);
    let segs = 0;
    for (let s = 0; s + nfft <= x.length; s += nfft >> 1) {
      if (mask) { let ok = true; for (let k = s; k < s + nfft; k += 8) if (!mask[k]) { ok = false; break; } if (!ok) continue; }
      let mean = 0;
      for (let k = 0; k < nfft; k++) mean += x[s + k];
      mean /= nfft;
      const re = new Float64Array(nfft), im = new Float64Array(nfft);
      for (let k = 0; k < nfft; k++) re[k] = (x[s + k] - mean) * w[k];
      fft(re, im);
      for (let k = 0; k < nfft / 2; k++) acc[k] += Math.hypot(re[k], im[k]) * 2 / wsum;
      segs++;
    }
    const f = Float64Array.from({ length: nfft / 2 }, (_, k) => k * rateHz / nfft);
    if (segs) for (let k = 0; k < acc.length; k++) acc[k] /= segs;
    return { f, amp: acc, segments: segs };
  }

  function bandPeak(sp, lo, hi) {
    let best = -1, sum = 0, cnt = 0;
    for (let k = 1; k < sp.f.length; k++) {
      if (sp.f[k] < lo || sp.f[k] > hi) continue;
      sum += sp.amp[k]; cnt++;
      if (best < 0 || sp.amp[k] > sp.amp[best]) best = k;
    }
    if (best < 0) return { hz: 0, amp: 0, ratio: 0 };
    const mean = cnt ? sum / cnt : 0;
    return { hz: sp.f[best], amp: sp.amp[best], ratio: mean > 0 ? sp.amp[best] / mean : 0 };
  }

  // ------------------------------------------------------------------ step response
  // Wiener deconvolution of setpoint -> gyro over overlapping windows (PID-Analyzer method)
  function stepResponse(sp, gyro, rateHz, armed) {
    const respLen = Math.round(0.5 * rateHz);
    let n = 1;
    while (n < 1.5 * rateHz) n <<= 1;
    const w = hann(n);
    const sum = new Float64Array(respLen);
    let used = 0;
    for (let s = 0; s + n <= sp.length; s += n >> 2) {
      let maxIn = 0, ok = true;
      for (let k = s; k < s + n; k++) {
        maxIn = Math.max(maxIn, Math.abs(sp[k]));
        if (armed && !armed[k]) { ok = false; break; }
      }
      if (!ok || maxIn < 20) continue;  // not enough stick input in this window
      const ir = new Float64Array(n), ii = new Float64Array(n), or = new Float64Array(n), oi = new Float64Array(n);
      for (let k = 0; k < n; k++) { ir[k] = sp[s + k] * w[k]; or[k] = gyro[s + k] * w[k]; }
      fft(ir, ii);
      fft(or, oi);
      let maxP = 0;
      for (let k = 0; k < n; k++) maxP = Math.max(maxP, ir[k] * ir[k] + ii[k] * ii[k]);
      const lambda = 1e-4 * maxP;
      const hr = new Float64Array(n), hi = new Float64Array(n);
      for (let k = 0; k < n; k++) {
        const den = ir[k] * ir[k] + ii[k] * ii[k] + lambda;
        // H = O * conj(I) / (|I|^2 + lambda)
        hr[k] = (or[k] * ir[k] + oi[k] * ii[k]) / den;
        hi[k] = (oi[k] * ir[k] - or[k] * ii[k]) / den;
      }
      fft(hr, hi, true);
      let acc = 0;
      const step = new Float64Array(respLen);
      for (let k = 0; k < respLen; k++) { acc += hr[k]; step[k] = acc; }
      let tail = 0;
      for (let k = Math.round(respLen * 0.4); k < respLen; k++) tail += step[k];
      tail /= respLen - Math.round(respLen * 0.4);
      if (tail < 0.5 || tail > 1.5) continue;  // window could not be identified
      for (let k = 0; k < respLen; k++) sum[k] += step[k];
      used++;
    }
    if (!used) return null;
    const y = Array.from(sum, v => v / used);
    const t = y.map((_, k) => k / rateHz * 1000);
    let ss = 0;
    const from = Math.round(respLen * 0.4);
    for (let k = from; k < respLen; k++) ss += y[k];
    ss /= respLen - from;
    let peak = -1e9, riseMs = null;
    for (let k = 0; k < respLen; k++) {
      peak = Math.max(peak, y[k]);
      if (riseMs === null && y[k] >= 0.9 * ss) riseMs = t[k];
    }
    return { t, y, ss, peak, overshoot: peak / ss - 1, riseMs, windows: used };
  }

  // ------------------------------------------------------------------ analysis
  function rmsHighPass(x, mask, alpha = 0.7) {
    // simple first-difference based high-pass energy
    let s = 0, c = 0, lp = x[0] || 0;
    for (let k = 1; k < x.length; k++) {
      lp += alpha * (x[k] - lp);
      if (mask && !mask[k]) continue;
      const hp = x[k] - lp;
      s += hp * hp; c++;
    }
    return c ? Math.sqrt(s / c) : 0;
  }

  function analyze(fl) {
    const rate = fl.rateHz;
    const armed = fl.armed;
    const nfft = rate >= 400 ? 256 : 128;
    // "steady" samples: armed, little stick input (noise measurements)
    const steady = new Uint8Array(fl.n);
    for (let k = 0; k < fl.n; k++)
      steady[k] = armed[k] && fl.thr[k] > 0.1 && Math.abs(fl.sp[0][k]) < 15 && Math.abs(fl.sp[1][k]) < 15 &&
        Math.abs(fl.sp[2][k]) < 15 ? 1 : 0;
    let sat = 0, armedN = 0;
    for (let k = 0; k < fl.n; k++) {
      if (!armed[k]) continue;
      armedN++;
      if (Math.max(fl.motor[0][k], fl.motor[1][k], fl.motor[2][k], fl.motor[3][k]) > 0.98) sat++;
    }
    const axes = AXES.map((name, a) => {
      const err = Float32Array.from(fl.gyro[a], (g, k) => g - fl.sp[a][k]);
      const step = stepResponse(fl.sp[a], fl.gyro[a], rate, armed);
      const specRaw = spectrum(fl.gyroRaw[a], rate, nfft, armed);
      const specFilt = spectrum(fl.gyro[a], rate, nfft, armed);
      const specD = spectrum(fl.D[a], rate, nfft, armed);
      const steadySegments = spectrum(err, rate, nfft, steady).segments;
      const specErr = spectrum(err, rate, nfft, steady);
      const osc = bandPeak(specErr, 6, Math.min(120, rate / 2 - 5));
      const rawPeak = bandPeak(specRaw, 40, rate / 2 - 2);
      let trk = 0, tc = 0;
      for (let k = 0; k < fl.n; k++) if (armed[k]) { trk += err[k] * err[k]; tc++; }
      return {
        name, step, steadySegments, specRaw, specFilt, specD, specErr, osc, rawPeak,
        noiseFilt: rmsHighPass(fl.gyro[a], steady),
        noiseRaw: rmsHighPass(fl.gyroRaw[a], steady),
        dNoise: rmsHighPass(fl.D[a], steady),
        trackRms: tc ? Math.sqrt(trk / tc) : 0,
      };
    });
    // ---- flight-wide measurements for the tuning assistant
    const ext = extraAnalysis(fl, steady, axes);
    return { rate, duration: fl.duration, armedSec: armedN / (rate || 1), saturation: armedN ? sat / armedN : 0, axes, ...ext };
  }

  // stick -> gyro delay: lag (ms) of the best correlation of setpoint and gyro while the sticks move
  function trackingLag(sp, gyro, armed, rate) {
    const maxLag = Math.round(0.06 * rate), n = sp.length;
    const stepK = Math.max(1, Math.floor(n / 30000));  // bounded work on long logs
    let best = -1, bestLag = 0;
    for (let L = 0; L <= maxLag; L++) {
      let c = 0, a = 0, b = 0;
      for (let k = 0; k + L < n; k += stepK) {
        if (!armed[k] || Math.abs(sp[k]) < 30) continue;
        c += sp[k] * gyro[k + L]; a += sp[k] * sp[k]; b += gyro[k + L] * gyro[k + L];
      }
      const r = a && b ? c / Math.sqrt(a * b) : 0;
      if (r > best) { best = r; bestLag = L; }
    }
    return best > 0.5 ? (bestLag / rate) * 1000 : null;
  }
  function maskedRms(x, mask, alpha) {
    let s = 0, c = 0, lp = x[0] || 0;
    for (let k = 1; k < x.length; k++) {
      lp += alpha * (x[k] - lp);
      if (!mask[k]) continue;
      const hp = x[k] - lp;
      s += hp * hp; c++;
    }
    return c > 50 ? Math.sqrt(s / c) : null;
  }
  function extraAnalysis(fl, steady, axes) {
    const n = fl.n, rate = fl.rateHz, armed = fl.armed;
    // prop wash: 0.4 s after the throttle drops by > 20 % within 0.2 s
    const pw = new Uint8Array(n), hiThr = new Uint8Array(n), midThr = new Uint8Array(n);
    const back = Math.round(0.2 * rate), hold = Math.round(0.4 * rate);
    for (let k = back; k < n; k++) {
      if (armed[k] && fl.thr[k - back] - fl.thr[k] > 0.2) for (let j = k; j < Math.min(n, k + hold); j++) pw[j] = 1;
    }
    for (let k = 0; k < n; k++) {
      const quiet = armed[k] && Math.abs(fl.sp[0][k]) < 30 && Math.abs(fl.sp[1][k]) < 30;
      hiThr[k] = quiet && fl.thr[k] > 0.65 ? 1 : 0;
      midThr[k] = quiet && fl.thr[k] > 0.25 && fl.thr[k] < 0.55 ? 1 : 0;
    }
    axes.forEach((ax, a) => {
      const err = Float32Array.from(fl.gyro[a], (g, k) => g - fl.sp[a][k]);
      const base = maskedRms(err, steady, 0.5);
      const pwR = maskedRms(err, pw, 0.5), hi = maskedRms(err, hiThr, 0.5), mid = maskedRms(err, midThr, 0.5);
      ax.propwash = base && pwR ? pwR / base : null;
      ax.tpaRatio = hi && mid ? hi / mid : null;
      ax.lagMs = trackingLag(fl.sp[a], fl.gyro[a], armed, rate);
      // slow drift: low-frequency tracking error while the sticks are centred (I too weak)
      let lp = 0, s2 = 0, c = 0;
      for (let k = 0; k < n; k++) {
        lp += 0.02 * (err[k] - lp);
        if (steady[k]) { s2 += lp * lp; c++; }
      }
      ax.drift = c > rate ? Math.sqrt(s2 / c) : null;
    });
    // motor balance in steady flight: one motor working harder = CG off-centre, bent prop or weak motor
    const mm = [0, 0, 0, 0];
    let mc = 0;
    for (let k = 0; k < n; k++) if (steady[k]) { for (let m = 0; m < 4; m++) mm[m] += fl.motor[m][k]; mc++; }
    const motorMean = mc > rate ? mm.map((v) => v / mc) : null;
    // battery: voltage vs current regression -> pack + wiring resistance
    let sx = 0, sy = 0, sxx = 0, sxy = 0, nc = 0, imax = 0, imin = 1e9;
    for (let k = 0; k < n; k += 5) {
      if (!armed[k] || fl.vbat[k] < 3 || fl.cur[k] <= 0) continue;
      sx += fl.cur[k]; sy += fl.vbat[k]; sxx += fl.cur[k] * fl.cur[k]; sxy += fl.cur[k] * fl.vbat[k]; nc++;
      imax = Math.max(imax, fl.cur[k]); imin = Math.min(imin, fl.cur[k]);
    }
    let resistance = null, sagV = null;
    if (nc > 50 && imax - imin > 5) {
      const slope = (nc * sxy - sx * sy) / (nc * sxx - sx * sx);
      if (slope < 0) { resistance = -slope; sagV = resistance * imax; }
    }
    // frame resonance: a raw gyro peak at the same frequency at low and high throttle (not motor driven)
    const lo = new Uint8Array(n), hi = new Uint8Array(n);
    for (let k = 0; k < n; k++) { lo[k] = armed[k] && fl.thr[k] < 0.4 ? 1 : 0; hi[k] = armed[k] && fl.thr[k] > 0.55 ? 1 : 0; }
    let resonance = null;
    const nfft = rate >= 400 ? 256 : 128;
    const pl = bandPeak(spectrum(fl.gyroRaw[0], rate, nfft, lo), 60, rate / 2 - 5);
    const ph = bandPeak(spectrum(fl.gyroRaw[0], rate, nfft, hi), 60, rate / 2 - 5);
    if (pl.hz && ph.hz && Math.abs(pl.hz - ph.hz) < 10 && pl.ratio > 6 && ph.ratio > 6) resonance = (pl.hz + ph.hz) / 2;
    return { motorMean, resistance, sagV, resonance, esc: escStats(fl, steady), rpmFilter: +((/rpm (\d+)/.exec(fl.header.gyro_lpf || "") || [])[1] || 0) };
  }

  // ------------------------------------------------------------------ tuning suggestions
  // params: { name: value } (PIDs as flown, from the log header). Each item names its evidence and
  // proposes changes; plan() merges them into one bounded set of changes the user applies explicitly.
  function suggest(res, params) {
    const out = [];
    const r = (v, d = 0) => Math.round(v * Math.pow(10, d)) / Math.pow(10, d);
    const add = (axis, level, title, detail, changes) => out.push({ axis, level, title, detail, changes: changes || {} });
    if (res.rate && res.rate < 250)
      add("all", "info", "Tần số log thấp", `Log ${res.rate} Hz: nên đặt bb_rate_hz 500 để phân tích nhiễu và đáp ứng chính xác hơn.`, { bb_rate_hz: 500 });
    if (res.armedSec < 15)
      add("all", "info", "Chuyến bay ngắn", `Chỉ ${r(res.armedSec)} s bay: kết quả kém tin cậy. Nên bay 60 s trở lên, có lắc cần nhanh roll / pitch và vài lần hạ ga đột ngột.`);
    for (const ax of res.axes) {
      const pre = ax.name;
      const P = params[pre + "_p"], I = params[pre + "_i"], D = params[pre + "_d"], F = params[pre + "_f"];
      const st = ax.step;
      if (!st) {
        if (pre !== "yaw")
          add(pre, "info", "Chưa đủ dữ liệu đáp ứng bước",
            "Hãy bay các cú lắc cần nhanh (roll/pitch qua lại) để đo đáp ứng.");
      } else {
        const ov = st.overshoot * 100;
        const rise = st.riseMs === null ? 999 : st.riseMs;
        const conf = st.windows >= 8 ? "" : ` (ít dữ liệu: ${st.windows} đoạn)`;
        if (ov > 15) {
          const ch = {};
          if (pre === "yaw") {
            ch[pre + "_p"] = r(P * 0.9);
          } else if (rise > 70) {
            // slow AND overshooting: I/feedforward dominate a too-weak P
            ch[pre + "_p"] = r(P * 1.12);
            if (D > 0) ch[pre + "_d"] = r(D * 1.1);
            ch[pre + "_i"] = r(I * 0.9);
          } else if (rise < 50) {
            // fast AND overshooting: P too strong for the damping
            ch[pre + "_p"] = r(P * 0.9);
            if (D > 0) ch[pre + "_d"] = r(D * 1.1);
          } else {
            // rise time fine: overshoot comes from I wind-up during snaps and low damping
            ch[pre + "_d"] = r(Math.max(D, 10) * 1.12);
            ch[pre + "_i"] = r(I * 0.85);
          }
          const why = rise > 70 ? "P yếu so với I/feedforward: tăng P và D, giảm nhẹ I." :
            (rise < 50 ? "P quá mạnh so với giảm chấn: giảm P, tăng D." :
              "I tích luỹ trong cú lắc và thiếu giảm chấn: giảm I, tăng D.");
          add(pre, "warn", `Vọt lố ${r(ov)}%`, `Đáp ứng bước vọt quá setpoint (lên 90% sau ${r(rise)} ms)${conf}. ${why}`, ch);
        } else if (ov < 3 && rise > 60) {
          const ch = {};
          ch[pre + "_p"] = r(P * 1.1);
          if (F !== undefined && pre !== "yaw") ch[pre + "_f"] = r(Math.max(F, 30) * 1.15);
          add(pre, "tip", `Đáp ứng chậm (${r(rise)} ms tới 90%)`, `Có thể tăng P và feedforward để bám cần nhanh hơn${conf}.`, ch);
        }
        if (st.ss < 0.85 && ov < 10) {
          const ch = {};
          ch[pre + "_i"] = r(I * 1.2);
          add(pre, "tip", `Không bám đủ setpoint (${r(st.ss * 100)}%)`, "Tăng I để triệt sai lệch kéo dài.", ch);
        }
      }
      // stick -> gyro delay: feedforward makes the quad move with the stick instead of after the error builds
      if (pre !== "yaw" && ax.lagMs != null && ax.lagMs > 25 && F !== undefined && !(st && st.overshoot > 0.12)) {
        add(pre, "tip", `Trễ so với cần ${r(ax.lagMs)} ms`, "Drone đi sau cần đáng kể: tăng feedforward để phản ứng ngay khi gạt cần.",
          { [pre + "_f"]: r(Math.max(F, 30) * 1.2) });
      }
      if (ax.steadySegments >= 4 && ax.osc.ratio > 6 && ax.osc.amp > 1.0) {
        const ch = {};
        if (ax.osc.hz > 40 && D > 0) {
          ch[pre + "_d"] = r(D * 0.85);
          if (params.dterm_lpf_hz) ch.dterm_lpf_hz = r(params.dterm_lpf_hz * 0.85);
          add(pre, "warn", `Dao động ${r(ax.osc.hz)} Hz`, "Dao động tần số cao: D quá lớn hoặc nhiễu lọt qua lọc D-term.", ch);
        } else {
          ch[pre + "_p"] = r(P * 0.88);
          add(pre, "warn", `Dao động ${r(ax.osc.hz)} Hz`, "Dao động tần số thấp: giảm P (hoặc I nếu chậm).", ch);
        }
      }
      // prop wash: shaking right after chopping the throttle
      if (pre !== "yaw" && ax.propwash != null && ax.propwash > 2.2) {
        const ch = {};
        if (params.d_min_pct !== undefined && params.d_min_pct > 0) ch.d_min_gain = r(Math.min(100, (params.d_min_gain || 30) * 1.25));
        else if (D > 0) ch[pre + "_d"] = r(D * 1.08);
        if (params.iterm_relax !== undefined) ch.iterm_relax = r(Math.max(10, params.iterm_relax * 0.85));
        add(pre, "warn", `Rung khi hạ ga (prop wash) ×${r(ax.propwash, 1)}`,
          "Lỗi bám tăng mạnh ngay sau khi hạ ga đột ngột: tăng giảm chấn (D / D-min) và để I-relax giữ I bớt tích luỹ.", ch);
      }
      // TPA: oscillation only at high throttle
      if (pre !== "yaw" && ax.tpaRatio != null && ax.tpaRatio > 1.8 && params.tpa_rate !== undefined) {
        add(pre, "tip", `Rung ở ga cao (×${r(ax.tpaRatio, 1)} so với ga giữa)`, "PID mạnh quá ở ga cao: tăng TPA (giảm PID khi ga lớn).",
          { tpa_rate: r(Math.min(100, Math.max(params.tpa_rate, 20) + 15)), tpa_breakpoint: params.tpa_breakpoint ? r(Math.max(1200, params.tpa_breakpoint - 50)) : undefined });
      }
      // slow drift with the sticks centred: I too weak (wind, off-centre CG)
      if (ax.drift != null && ax.drift > 6 && I !== undefined && !(st && st.overshoot > 0.15)) {
        add(pre, "tip", `Trôi chậm khi buông cần (${r(ax.drift, 1)} °/s)`, "Drone không giữ được hướng khi để cần giữa: tăng I.", { [pre + "_i"]: r(I * 1.15) });
      }
      if (pre !== "yaw" && ax.dNoise > 12) {
        const ch = {};
        if (params.dterm_lpf_hz) ch.dterm_lpf_hz = r(Math.max(60, params.dterm_lpf_hz * 0.8));
        add(pre, "warn", "D-term nhiễu", `Nhiễu D-term ${r(ax.dNoise, 1)} (motor dễ nóng): hạ dterm_lpf_hz hoặc giảm D.`, ch);
      }
      if (ax.noiseFilt > 2.0) {
        const ch = {};
        if (params.gyro_lpf_hz) ch.gyro_lpf_hz = r(Math.max(80, params.gyro_lpf_hz * 0.8));
        if (!params.esc_telemetry && !params.dshot_bidir) ch.dshot_bidir = 1;
        else if (!params.dyn_notch_count) ch.dyn_notch_count = 1;
        add(pre, "warn", "Gyro sau lọc còn nhiễu", `RMS ${r(ax.noiseFilt, 2)} °/s khi bay êm. Đỉnh rung thô ở ~${r(ax.rawPeak.hz)} Hz.`, ch);
      }
    }
    // over-filtering: very clean gyro and D-term on every axis -> filters can be raised (less delay)
    const rp = res.axes.filter((a) => a.name !== "yaw");
    if (rp.length && rp.every((a) => a.noiseFilt < 0.6 && a.dNoise < 4 && a.steadySegments >= 4)) {
      const ch = {};
      if (params.gyro_lpf_hz && params.gyro_lpf_hz < 250) ch.gyro_lpf_hz = r(Math.min(250, params.gyro_lpf_hz * 1.2));
      if (params.dterm_lpf_hz && params.dterm_lpf_hz < 150) ch.dterm_lpf_hz = r(Math.min(150, params.dterm_lpf_hz * 1.15));
      if (Object.keys(ch).length)
        add("all", "tip", "Lọc đang dư", "Gyro và D-term rất sạch: nâng tần số lọc để bớt trễ, drone bám cần chặt hơn. Sau đó kiểm tra motor có ấm lên không.", ch);
    }
    // frame resonance: fixed frequency at low and high throttle
    if (res.resonance && params.notch1_hz !== undefined && !params.notch1_hz) {
      const hz = r(res.resonance);
      add("all", "warn", `Cộng hưởng khung ~${hz} Hz`, "Đỉnh rung đứng yên khi đổi ga (không do motor): kiểm tra ốc khung, tay, FC; hoặc đặt notch tĩnh tại tần số này.",
        { notch1_hz: hz, notch1_cut: r(hz * 0.7) });
    }
    // motor balance
    if (res.motorMean) {
      const avg = res.motorMean.reduce((x, y) => x + y, 0) / 4;
      const dev = res.motorMean.map((m) => (avg ? (m - avg) / avg : 0));
      const worst = dev.reduce((bi, v, i) => (Math.abs(v) > Math.abs(dev[bi]) ? i : bi), 0);
      if (avg > 0.15 && Math.abs(dev[worst]) > 0.08)
        add("all", "tip", `Motor ${worst + 1} làm việc ${dev[worst] > 0 ? "nhiều" : "ít"} hơn ${r(Math.abs(dev[worst]) * 100)}%`,
          `Khi bay êm, các motor: ${res.motorMean.map((m) => r(m * 100) + "%").join(" / ")}. ` +
          "Thường do trọng tâm lệch (dời pin), cánh cong / lắp sai, hoặc motor yếu. Chênh dưới 5% là bình thường.");
    }
    // ESC telemetry (frames 0xA7): weak motor / prop, hot motors, RPM filter without RPM
    if (res.esc) {
      const k = res.esc.rpmPerCmd;
      if (k.every((v) => v)) {
        const avg = k.reduce((x, y) => x + y, 0) / 4;
        const dev = k.map((v) => (v - avg) / avg);
        const w = dev.reduce((bi, v, i) => (v < dev[bi] ? i : bi), 0);
        if (dev[w] < -0.08)
          add("all", "warn", `Motor ${w + 1} quay chậm hơn ${r(-dev[w] * 100)}% so với lệnh`,
            `RPM trên mỗi % ga khi bay êm: ${k.map((v) => r(v / 100)).join(" / ")}. Motor ${w + 1} cần nhiều ga hơn để đạt cùng vòng quay: ` +
            "kiểm tra cánh (mẻ, cong, lắp ngược), bạc đạn, ốc bắt motor và mối hàn dây motor.");
      }
      const t = res.esc.tempMax;
      const hot = t.map((v, i) => [v, i]).filter(([v]) => v !== null && v >= 80);
      if (hot.length)
        add("all", hot.some(([v]) => v >= 100) ? "warn" : "tip", `ESC nóng ${hot.map(([v, i]) => `M${i + 1} ${r(v)}°C`).join(", ")}`,
          "Nhiệt độ ESC cao nhất trong chuyến bay. Trên 100 °C là nguy hiểm: giảm D / bộ lọc D bớt nhiễu, cánh nhỏ hơn, hoặc tăng làm mát.");
    } else if (res.rpmFilter > 0) {
      add("all", "info", "Log không có RPM của motor",
        "Bộ lọc RPM đang bật nhưng log không có khung ESC: firmware cũ, hoặc ESC chưa bật DShot hai chiều (dshot_bidir). Kiểm tra trong tab Motor & ESC.");
    }
    // battery sag
    if (res.resistance != null) {
      const mohm = res.resistance * 1000;
      const ch = {};
      if (res.sagV > 1.0 && params.vbat_sag_comp !== undefined && params.vbat_sag_comp < 50) ch.vbat_sag_comp = 70;
      add("all", res.sagV > 2 ? "warn" : "info", `Pin sụt ${r(res.sagV, 1)} V ở dòng lớn nhất`,
        `Điện trở pin + dây ≈ ${r(mohm)} mΩ. ${res.sagV > 2 ? "Pin yếu hoặc chai: cảm giác ga thay đổi theo mức pin. " : ""}` +
        (ch.vbat_sag_comp ? "Bù sụt áp (vbat_sag_comp) giữ cảm giác ga đều suốt chuyến bay." : ""), ch);
    }
    if (res.saturation > 0.05)
      add("all", "warn", `Motor bão hoà ${r(res.saturation * 100)}% thời gian`,
        "Giảm rates hoặc PID, hoặc kiểm tra lực đẩy / pin.",
        { roll_max_rate: r((params.roll_max_rate || 670) * 0.9), pitch_max_rate: r((params.pitch_max_rate || 670) * 0.9) });
    for (const s of out) for (const k of Object.keys(s.changes)) if (s.changes[k] === undefined || !isFinite(s.changes[k])) delete s.changes[k];
    if (!out.some(s => s.level === "warn"))
      add("all", "ok", "Không phát hiện vấn đề lớn", "Các chỉ số nằm trong ngưỡng hợp lý cho log này.");
    return out;
  }

  // One set of changes from all suggestions: several proposals for the same parameter are combined
  // (product of their ratios), each change is limited to +-25 % per round (tune in small steps, fly
  // again), and to the parameter's own range. limits: { name: {min, max} }.
  function plan(sugs, params, limits) {
    const by = {};
    for (const s of sugs) for (const [k, v] of Object.entries(s.changes)) {
      const from = params[k];
      if (from === undefined || !isFinite(v)) continue;
      const e = by[k] || (by[k] = { name: k, from, ratio: 1, abs: null, reasons: [] });
      if (from) e.ratio *= v / from; else e.abs = v;  // from 0 (switches, notch off): take the value
      e.reasons.push((s.axis === "all" ? "" : s.axis + ": ") + s.title);
    }
    const out = [];
    for (const e of Object.values(by)) {
      let to = e.abs !== null ? e.abs : e.from * Math.min(1.25, Math.max(0.75, e.ratio));
      const lim = limits && limits[e.name];
      if (lim) to = Math.min(lim.max, Math.max(lim.min, to));
      to = Math.abs(to) >= 10 ? Math.round(to) : Math.round(to * 100) / 100;
      if (Math.abs(to - e.from) < 1e-6) continue;
      out.push({ name: e.name, from: e.from, to, reasons: e.reasons });
    }
    return out.sort((x, y) => x.name.localeCompare(y.name));
  }

  // ------------------------------------------------------------------ export
  // Columns in Betaflight's units (what Blackbox Explorer / PIDtoolbox / blackbox_decode expect):
  // gyro / setpoint deg/s, PID terms in PID-sum units, rcCommand roll/pitch/yaw -500..500 and throttle
  // 1000..2000, setpoint[3] throttle 0..1000, motor as DShot value 48..2047 (0 = stopped),
  // vbatLatest 0.01 V, amperageLatest 0.01 A. Our RC channels are AETR (ch3 throttle, ch4 yaw).
  const EXP_FIELDS = (() => {
    const f = [["loopIteration", 0], ["time", 0]];
    for (const k of ["axisP", "axisI", "axisD", "axisF"]) for (let a = 0; a < 3; a++) f.push([`${k}[${a}]`, 1]);
    for (let a = 0; a < 4; a++) f.push([`rcCommand[${a}]`, 1]);
    for (let a = 0; a < 4; a++) f.push([`setpoint[${a}]`, 1]);
    f.push(["vbatLatest", 0], ["amperageLatest", 1]);
    for (let a = 0; a < 3; a++) f.push([`gyroADC[${a}]`, 1]);
    for (let a = 0; a < 3; a++) f.push([`gyroUnfilt[${a}]`, 1]);
    for (let m = 0; m < 4; m++) f.push([`motor[${m}]`, 0]);
    return f;
  })();
  function expRow(fl, r, t0) {
    const R = Math.round, v = [r, R((fl.t[r] - t0) * 1e6)];
    for (const k of [fl.P, fl.I, fl.D, fl.F]) for (let a = 0; a < 3; a++) v.push(R(k[a][r]));
    const rc = (k) => fl.rc[k][r] || 1500;
    v.push(R(rc(0) - 1500), R(rc(1) - 1500), R(rc(3) - 1500), R(Math.max(1000, Math.min(2000, rc(2)))));
    for (let a = 0; a < 3; a++) v.push(R(fl.sp[a][r]));
    v.push(R(fl.thr[r] * 1000));
    v.push(Math.max(0, R(fl.vbat[r] * 100)), R(fl.cur[r] * 100));
    for (let a = 0; a < 3; a++) v.push(R(fl.gyro[a][r]));
    for (let a = 0; a < 3; a++) v.push(R(fl.gyroRaw[a][r]));
    for (let m = 0; m < 4; m++) v.push(fl.motor[m][r] > 0 ? R(48 + fl.motor[m][r] * 1999) : 0);
    return v;
  }
  // CSV (blackbox_decode column names) plus our extra columns: attitude, altitude, flight mode, armed
  function toCsv(fl) {
    const t0 = fl.n ? fl.t[0] : 0;
    const head = EXP_FIELDS.map(([n]) => (n === "time" ? "time (us)" : n === "vbatLatest" ? "vbatLatest (V)" : n === "amperageLatest" ? "amperageLatest (A)" : n));
    head.push("roll (deg)", "pitch (deg)", "heading (deg)", "altitude (m)", "flightModeFlags", "armed");
    const iv = EXP_FIELDS.findIndex(([n]) => n === "vbatLatest");
    const out = [head.map((h) => `"${h}"`).join(",")];
    for (let r = 0; r < fl.n; r++) {
      const v = expRow(fl, r, t0);
      v[iv] = (v[iv] / 100).toFixed(2);
      v[iv + 1] = (v[iv + 1] / 100).toFixed(2);
      v.push(fl.att[0][r].toFixed(1), fl.att[1][r].toFixed(1), fl.att[2][r].toFixed(1), fl.alt[r].toFixed(2), fl.mode[r], fl.armed[r]);
      out.push(v.join(","));
    }
    return out.join("\n") + "\n";
  }
  // Betaflight blackbox binary (.bbl) that Blackbox Explorer and PIDtoolbox open: every frame is an
  // intra frame ("I interval:1", raw values, variable-byte encoding), ended by a log-end event.
  function toBbl(fl, info) {
    info = info || {};
    const bytes = [];
    const str = (t) => { for (let i = 0; i < t.length; i++) bytes.push(t.charCodeAt(i) & 0x7F); };
    const uvb = (x) => { x = x >>> 0; while (x >= 0x80) { bytes.push((x & 0x7F) | 0x80); x >>>= 7; } bytes.push(x); };
    const svb = (x) => uvb((x << 1) ^ (x >> 31));
    const n = EXP_FIELDS.length, list = (f) => EXP_FIELDS.map(f).join(",");
    const looptime = Math.round(1e6 / (fl.rateHz || 500));
    const H = [
      "Product:Blackbox flight data recorder by Nicholas Sherlock",
      "Data version:2",
      "I interval:1",
      "P interval:1/1",
      "P ratio:1",
      "Field I name:" + list(([nm]) => nm),
      "Field I signed:" + list(([, sg]) => sg),
      "Field I predictor:" + Array(n).fill(0).join(","),
      "Field I encoding:" + list(([, sg]) => (sg ? 0 : 1)),
      "Field P predictor:" + Array(n).fill(0).join(","),
      "Field P encoding:" + list(([, sg]) => (sg ? 0 : 1)),
      "Firmware type:Cleanflight",
      // the version decides how the viewer reads units: AllQuad logs in Betaflight 4.4 units
      `Firmware revision:Betaflight 4.4.0 (AllQuad ${info.version || ""}) ${info.board || ""}`.trim(),
      "Firmware date:" + (info.date || ""),
      "Board information:AllQuad " + (info.board || ""),
      "Craft name:" + (info.craft || "AllQuad"),
      "looptime:" + looptime,
      "pid_process_denom:1",
      "gyro_scale:0x3f800000",
      "acc_1G:2048",
      "motorOutput:48,2047",
      "minthrottle:1000",
      "maxthrottle:2000",
      "vbat_scale:110",
      "vbatref:" + Math.round(Math.max(...Array.from(fl.vbat.subarray(0, Math.min(fl.n, 200)), (x) => x || 0), 0) * 100),
      "currentSensor:0,400",
      "features:0",
    ];
    for (const [k, v] of Object.entries(fl.header || {})) if (/^[\w .-]+$/.test(k) && !/[\n\r]/.test(String(v))) H.push(`aq_${k}:${v}`);
    for (const h of H) { str("H " + h + "\n"); }
    const t0 = fl.n ? fl.t[0] : 0;
    let last = -1;
    for (let r = 0; r < fl.n; r++) {
      const v = expRow(fl, r, t0);
      if (v[1] <= last) continue;  // the viewer needs a rising time; skip duplicate stamps
      last = v[1];
      bytes.push(73);  // 'I'
      for (let i = 0; i < n; i++) (EXP_FIELDS[i][1] ? svb : uvb)(v[i]);
    }
    bytes.push(69, 255);  // 'E', log end
    str("End of log");
    bytes.push(0);
    return Uint8Array.from(bytes);
  }


  // ------------------------------------------------------------------ flight summary
  // What a pilot asks after landing: how long, how much battery, how hard the pack was pushed, how high /
  // far, how much vibration. notes: [{level: ok|info|warn|bad, text}] in plain words, worst first.
  function report(fl, res) {
    const n = fl.n, armed = fl.armed, t = fl.t;
    let armedSec = 0, arms = 0, thrSum = 0, thrN = 0, thrMax = 0, vMax = 0, vMin = Infinity, vRest = 0, vEnd = 0;
    let iMax = 0, iSum = 0, iN = 0, mah = 0, altMax = -Infinity, alt0 = null, far = 0, posOk = false;
    const rateMax = [0, 0, 0], angMax = [0, 0];
    const modes = [];
    const thrs = [];
    for (let k = 0; k < n; k++) {
      const dt = k ? Math.min(0.1, Math.max(0, t[k] - t[k - 1])) : 0;
      if (armed[k] && (!k || !armed[k - 1])) {
        arms++;
        if (!vRest && fl.vbat[k] > 1) vRest = fl.vbat[Math.max(0, k - 1)] > 1 ? fl.vbat[Math.max(0, k - 1)] : fl.vbat[k];
      }
      if (fl.vbat[k] > 1) { vMax = Math.max(vMax, fl.vbat[k]); vEnd = fl.vbat[k]; }
      if (!armed[k]) continue;
      armedSec += dt;
      thrSum += fl.thr[k]; thrN++; thrMax = Math.max(thrMax, fl.thr[k]);
      if (!(k % 5)) thrs.push(fl.thr[k]);
      if (fl.vbat[k] > 1) vMin = Math.min(vMin, fl.vbat[k]);
      if (fl.cur[k] > 0) { iMax = Math.max(iMax, fl.cur[k]); iSum += fl.cur[k]; iN++; mah += fl.cur[k] * dt / 3.6; }
      if (alt0 === null) alt0 = fl.alt[k];
      altMax = Math.max(altMax, fl.alt[k] - alt0);
      if (fl.posN[k] || fl.posE[k]) { posOk = true; far = Math.max(far, Math.hypot(fl.posN[k], fl.posE[k])); }
      for (let a = 0; a < 3; a++) rateMax[a] = Math.max(rateMax[a], Math.abs(fl.gyro[a][k]));
      for (let a = 0; a < 2; a++) angMax[a] = Math.max(angMax[a], Math.abs(fl.att[a][k]));
      if (!modes.includes(fl.mode[k])) modes.push(fl.mode[k]);
    }
    thrs.sort((a, b) => a - b);
    const hover = thrs.length > 20 ? thrs[thrs.length >> 1] : null;
    if (!vRest) vRest = vMax;
    // cells: a charged LiPo cell sits at 3.7-4.35 V
    let cells = 0;
    for (let c = 1; c <= 8 && !cells; c++) if (vRest / c <= 4.4) cells = c;
    if (vRest < 2.5) cells = 0;
    const out = {
      armedSec, arms, thrMean: thrN ? thrSum / thrN : 0, thrMax, hover,
      vRest: vRest || null, vMin: vMin < Infinity ? vMin : null, vEnd: vEnd || null, cells,
      sag: vRest && vMin < Infinity ? vRest - vMin : null,
      iMax: iN ? iMax : null, iMean: iN ? iSum / iN : null, mah: iN ? mah : null,
      altMax: altMax > -Infinity ? altMax : null, far: posOk ? far : null, rateMax, angMax, modes,
      vib: res ? res.axes.slice(0, 2).reduce((m, a) => Math.max(m, a.noiseRaw || 0), 0) : null,
      vibFilt: res ? res.axes.slice(0, 2).reduce((m, a) => Math.max(m, a.noiseFilt || 0), 0) : null,
      tempMax: res && res.esc && res.esc.haveTemp && Math.max(...res.esc.tempMax.filter((x) => x != null)) > 0 ? Math.max(...res.esc.tempMax.filter((x) => x != null)) : null,
      saturation: res ? res.saturation : null, nav: navStats(fl), notes: [],
    };
    const note = (level, text) => out.notes.push({ level, text });
    const f1 = (v) => (Math.round(v * 10) / 10).toFixed(1);
    if (!thrN) note("info", "Log không có đoạn nào đang arm: chỉ có dữ liệu khi drone đứng yên.");
    if (cells && out.vMin) {
      const pc = out.vMin / cells, ec = (out.vEnd || out.vMin) / cells;
      if (pc < 3.3) note("bad", `Điện áp sụt xuống ${f1(out.vMin)} V (${pc.toFixed(2)} V/cell): pin bị ép quá sức, hạ cánh sớm hơn hoặc dùng pin xả mạnh hơn.`);
      else if (pc < 3.5) note("warn", `Điện áp thấp nhất ${pc.toFixed(2)} V/cell khi bay: gần ngưỡng hại pin (3.3–3.5 V/cell).`);
      if (ec < 3.6) note("warn", `Kết thúc ở ${ec.toFixed(2)} V/cell: lần sau hạ cánh sớm hơn để pin bền (khoảng 3.7 V/cell khi nghỉ).`);
    }
    if (out.sag != null && cells && out.sag / cells > 0.5) note("warn", `Sụt áp ${f1(out.sag)} V khi tăng ga: pin yếu / già, nội trở cao, hoặc dây và giắc nguồn kém.`);
    if (hover != null) {
      if (hover > 0.6) note("warn", `Ga lơ lửng khoảng ${Math.round(hover * 100)}%: drone nặng so với lực đẩy, còn ít dư để cơ động và chống gió.`);
      else if (hover < 0.18 && thrMax > 0.3) note("info", `Ga lơ lửng thấp (~${Math.round(hover * 100)}%): rất dư lực, có thể giảm thr_limit_pct cho dễ điều khiển.`);
    }
    if (out.saturation != null && out.saturation > 0.02) note("warn", `Motor chạm 100% trong ${(out.saturation * 100).toFixed(1)}% thời gian bay: mất kiểm soát tư thế lúc đó (thiếu lực đẩy hoặc PID quá mạnh).`);
    if (out.vib != null) {
      if (out.vib > 25) note("bad", `Rung rất mạnh (gyro thô ${f1(out.vib)} °/s RMS): kiểm tra cánh mẻ / cong, ốc motor, ổ bi, đệm cao su của FC trước khi bay tiếp.`);
      else if (out.vib > 10) note("warn", `Rung khá mạnh (gyro thô ${f1(out.vib)} °/s RMS): cân lại cánh, siết ốc; bộ lọc đang phải làm việc nhiều (thêm trễ).`);
      else note("ok", `Rung thấp (gyro thô ${f1(out.vib)} °/s RMS).`);
    }
    if (out.tempMax != null && out.tempMax > 85) note("warn", `ESC nóng tới ${out.tempMax} °C: giảm ga tối đa, thông gió hoặc dùng ESC lớn hơn.`);
    if (out.iMax != null && out.iMax < 0.5) note("info", "Không đo được dòng: không có mAh. Xem Chế độ & an toàn → Pin để nối và hiệu chỉnh cảm biến dòng.");
    const nv = out.nav;
    if (nv) {
      const s1 = (v) => `${f1(v)} s`;
      if (nv.resets.filter) note("bad", `EKF phải khởi động lại toàn bộ ${nv.resets.filter} lần (lỗi số): gửi log này để kiểm tra.`);
      if (nv.resets.tilt)
        note("warn", `Tư thế EKF lệch AHRS và đã tự đặt lại ${nv.resets.tilt} lần (lệch tối đa ${Math.round(nv.tiltErrMax)}°): thường do rung mạnh làm gia tốc kế bão hoà ` +
          "hoặc lộn nhào rất nhanh. Kiểm tra đệm chống rung của FC; nếu bay ACRO mạnh thì đây là bình thường.");
      if (nv.resets.heading)
        note("warn", `Hướng EKF lệch cả la bàn lẫn hướng tính từ GPS, đã đặt lại theo la bàn ${nv.resets.heading} lần: gửi log này để kiểm tra (thường sau va chạm hoặc lộn nhào).`);
      if (nv.resets.pos || nv.resets.vel)
        note("warn", `GPS nhảy, EKF đặt lại vị trí ${nv.resets.pos} lần, vận tốc ${nv.resets.vel} lần: ăng-ten GPS bị che, gần VTX / dây nguồn, hoặc ít vệ tinh.`);
      if (nv.faultSec.baro > 1)
        note("warn", `Baro bị loại ${s1(nv.faultSec.baro)}${nv.ekf ? " (EKF lấy độ cao từ GPS)" : ""}: che baro bằng mút xốp, tránh gió cánh thổi thẳng vào, kiểm tra nhiệt.`);
      if (nv.resets.baro)
        note("warn", `Độ cao bị đặt lại theo baro ${nv.resets.baro} lần (baro lệch xa ước lượng hơn 2 s): kiểm tra baro, mút che, hoặc va chạm.`);
      if (nv.faultSec.mag > 1) note("warn", `La bàn lệch hướng GPS ${s1(nv.faultSec.mag)}: hiệu chuẩn lại, đặt compass xa dây pin / ESC, kiểm tra hướng lắp (mag_align).`);
      else if (nv.faultSec.heading > 3) note("info", `EKF loại hướng la bàn ${s1(nv.faultSec.heading)}: nhiễu từ ngắn hạn (dòng lớn khi tăng ga).`);
      if (nv.faultSec.gpsGlitch > 2) note("info", `Vị trí GPS bị loại ${s1(nv.faultSec.gpsGlitch)} (lệch khỏi dự đoán).`);
      if (nv.satsMin !== null && nv.satsMin < 6) note("info", `GPS xuống còn ${nv.satsMin} vệ tinh trong chuyến bay: giữ chế độ GPS cẩn thận, chờ đủ vệ tinh trước khi cất cánh.`);
      if (nv.hAccMax !== null && nv.hAccMax > 5) note("info", `Sai số GPS lên tới ${f1(nv.hAccMax)} m.`);
      if (nv.ekf && !nv.resets.filter && !nv.resets.tilt && !nv.resets.heading && !nv.resets.pos && !nv.resets.vel && nv.faultSec.baro <= 1 && nv.faultSec.mag <= 1)
        note("ok", "EKF ổn định suốt chuyến bay: không phải đặt lại, không loại cảm biến lâu.");
    }
    if (res && res.rpmFilter === 0 && res.esc && res.esc.haveRpm) note("info", "ESC có báo RPM nhưng RPM filter đang tắt: bật rpm_harmonics để lọc nhiễu motor tốt hơn.");
    if (!out.notes.some((x) => x.level === "bad" || x.level === "warn") && thrN) note("ok", "Không thấy vấn đề lớn về pin, lực đẩy và rung trong chuyến này.");
    const rank = { bad: 0, warn: 1, info: 2, ok: 3 };
    out.notes.sort((a, b) => rank[a.level] - rank[b.level]);
    return out;
  }

  const api = { decode, fft, spectrum, stepResponse, analyze, suggest, plan, report, bandPeak, toCsv, toBbl, navStats, REC, ESC, NAV, AXES };
  if (typeof module !== "undefined" && module.exports) module.exports = api;
  else root.AQ = api;
})(typeof window !== "undefined" ? window : globalThis);
