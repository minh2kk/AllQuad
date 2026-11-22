#include "../common/size_opt.h"  // -Os on ESP32 (not flight critical)
#include "blackbox.h"

#include <stdio.h>
#include <string.h>

#include "../bus/bus.h"
#include "gyro_tap.h"
#include "sdlog.h"

namespace logging {

using namespace fc;

static int16_t s16(float v) { return (int16_t)constrainf(v, -32767, 32767); }

void Blackbox::begin(uint8_t backend, hal::Port port, uint16_t rateHz) {
  backend_ = rateHz ? backend : 0;
  port_ = port;
  if (backend_ == 2 && !hal::logOpen()) backend_ = 0;
  if (backend_ == 3 && !sdlog::enabled()) backend_ = 0;  // sd_mode off / not possible on this board
  escEvery_ = (uint16_t)(rateHz > 50 ? rateHz / 50 : 1);
}

bool Blackbox::write(const uint8_t* d, size_t n) {
  size_t done = n;
  if (backend_ == 1) done = hal::portWrite(port_, d, n);
  else if (backend_ == 2) done = hal::logWrite(d, n);
  else if (backend_ == 3) done = sdlog::push(d, n) ? n : 0;
  bytes_ += done;
  return done == n;
}

void Blackbox::writeHeader() {
  const Config& c = bus::config;
  char h[440];
  int n = snprintf(h, sizeof(h),
                   "H AllQuad blackbox v1\nH record_size %u\nH encoding delta1\nH loop_hz %u\n"
                   "H pid_roll %.0f %.0f %.0f %.0f\nH pid_pitch %.0f %.0f %.0f %.0f\nH pid_yaw %.0f %.0f %.0f %.0f\n"
                   "H gyro_lpf %.0f %.0f dterm_lpf %.0f dyn_notch %u rpm %u\nH esc_frame %u\nH gyro_frame %u %u\nH end\n",
                   (unsigned)sizeof(Record), c.loop_hz, c.pid[0].p, c.pid[0].i, c.pid[0].d, c.pid[0].f, c.pid[1].p,
                   c.pid[1].i, c.pid[1].d, c.pid[1].f, c.pid[2].p, c.pid[2].i, c.pid[2].d, c.pid[2].f, c.gyro_lpf_hz,
                   c.gyro_lpf2_hz, c.dterm_lpf_hz, c.dyn_notch_count, c.rpm_harmonics, (unsigned)sizeof(EscRecord),
                   (unsigned)sizeof(GyroFrame), (unsigned)gyroTapRate());
  write((const uint8_t*)h, n > 0 ? (size_t)n : 0);
}

// field layout of Record after sync / len: offset, size, signed (same order as the struct)
struct FieldDesc { uint8_t off, size, sign; };
static const FieldDesc* fieldTable() {
  static FieldDesc t[Blackbox::FIELDS];
  static bool init = false;
  if (!init) {
    int n = 0, off = 2;
    auto add = [&](int count, int size, bool sign) {
      for (int k = 0; k < count; k++) { t[n++] = {(uint8_t)off, (uint8_t)size, (uint8_t)sign}; off += size; }
    };
    add(1, 4, false);  // timeUs
    add(21, 2, true);  // gyro, gyroRaw, setpoint, P, I, D, F
    add(4, 2, false);  // motor
    add(1, 2, false);  // throttle
    add(4, 2, false);  // rc
    add(3, 2, true);   // att
    add(1, 4, true);   // altCm
    add(1, 2, true);   // vzCms
    add(2, 4, true);   // posN, posE
    add(1, 2, false);  // vbatCv
    add(1, 2, true);   // currentCa
    add(3, 1, false);  // mode, navPhase, flags
    init = true;
  }
  return t;
}

static int64_t fieldValue(const Record& r, const FieldDesc& f) {
  const uint8_t* p = (const uint8_t*)&r + f.off;
  uint32_t u = 0;
  memcpy(&u, p, f.size);  // little-endian targets
  if (!f.sign) return (int64_t)u;
  if (f.size == 2) return (int16_t)u;
  if (f.size == 4) return (int32_t)u;
  return (int8_t)u;
}

size_t Blackbox::encodeDelta(const Record& prev, const Record& cur, uint8_t* out) {
  static_assert(sizeof(Record) == 94, "update FIELDS / fieldTable and the decoders with the record");
  const FieldDesc* F = fieldTable();
  uint8_t* o = out;
  *o++ = 0xA6;
  *o++ = prev.crc;  // chain: the reader checks it is building on this very record
  uint8_t* mask = o;
  memset(mask, 0, (FIELDS + 7) / 8);
  o += (FIELDS + 7) / 8;
  for (int i = 0; i < FIELDS; i++) {
    int64_t d = fieldValue(cur, F[i]) - fieldValue(prev, F[i]);
    if (!d) continue;
    mask[i >> 3] |= (uint8_t)(1 << (i & 7));
    uint64_t z = ((uint64_t)d << 1) ^ (0 - ((uint64_t)d >> 63));  // zig-zag
    while (z >= 0x80) { *o++ = (uint8_t)(z | 0x80); z >>= 7; }
    *o++ = (uint8_t)z;
  }
  uint8_t x = 0;
  for (uint8_t* q = out; q < o; q++) x ^= *q;
  *o++ = x;
  return (size_t)(o - out);
}

void Blackbox::fill(Record& r, uint32_t nowUs) {
  memset(&r, 0, sizeof(r));
  AttitudeState a = bus::attitude.get();
  PidDebug p = bus::pid.get();
  ControlSetpoint sp = bus::setpoint.get();
  MotorOutputs m = bus::motors.get();
  RcInput rc = bus::rc.get();
  NavEstimate e = bus::nav.get();
  BatteryState b = bus::battery.get();
  VehicleStatus st = bus::status.get();
  r.sync = 0xA5;
  r.len = sizeof(Record);
  r.timeUs = nowUs;
  for (int i = 0; i < 3; i++) {
    r.gyro[i] = s16(a.rateDps[i] * 10);
    r.gyroRaw[i] = s16(p.gyroRaw[i] * 10);
    r.setpoint[i] = s16(sp.rateDps[i] * 10);
    r.pidP[i] = s16(p.p[i] * 10);
    r.pidI[i] = s16(p.i[i] * 10);
    r.pidD[i] = s16(p.d[i] * 10);
    r.pidF[i] = s16(p.f[i] * 10);
  }
  for (int i = 0; i < 4; i++) {
    r.motor[i] = (uint16_t)(m.armed && m.out[i] > 0 ? m.out[i] * 1000 : 0);
    r.rc[i] = rc.ch[i];
  }
  r.throttle = (uint16_t)(sp.throttle * 1000);
  r.att[0] = s16(a.roll * RAD2DEG * 10);
  r.att[1] = s16(a.pitch * RAD2DEG * 10);
  r.att[2] = s16(e.yaw * RAD2DEG * 10);  // true heading
  r.altCm = (int32_t)(e.alt * 100);
  r.vzCms = s16(e.vz * 100);
  r.posN = (int32_t)(e.n * 100);
  r.posE = (int32_t)(e.e * 100);
  r.vbatCv = (uint16_t)(b.voltage * 100);
  r.currentCa = s16(b.current * 100);
  r.mode = (uint8_t)st.mode;
  r.navPhase = (uint8_t)st.navPhase;
  r.flags = (st.armed ? 1 : 0) | (st.failsafe ? 2 : 0) | (st.landed ? 4 : 0) | ((st.warnings & WARN_ACC_CLIP) ? 8 : 0);
  uint8_t x = 0;
  const uint8_t* q = (const uint8_t*)&r;
  for (size_t i = 0; i < sizeof(r) - 1; i++) x ^= q[i];
  r.crc = x;
}

bool Blackbox::fillEsc(EscRecord& r, uint32_t nowUs) {
  static_assert(sizeof(EscRecord) == 23, "update the decoders with the ESC frame");
  memset(&r, 0, sizeof(r));
  r.sync = 0xA7;
  r.len = sizeof(EscRecord);
  r.timeUs = nowUs;
  float erpm[4];
  hal::motorsErpm(erpm);  // bidirectional DShot
  float pairs = bus::config.motor_poles >= 2 ? bus::config.motor_poles * 0.5f : 7.0f;
  EscTelemetry e = bus::esc.get();
  uint32_t nowMs = nowUs / 1000;
  bool any = false, fresh = false;
  for (int i = 0; i < 4; i++) {
    bool f = bus::esc.seq() && ageMs(nowMs, e.lastUpdateMs[i]) < 1000;
    fresh |= f;
    float rpm = erpm[i] >= 0 ? erpm[i] / pairs : (f && e.rpm[i] >= 0 ? e.rpm[i] : -1.0f);
    r.rpm10[i] = rpm >= 0 ? (uint16_t)constrainf(rpm * 0.1f + 0.5f, 0, 65534) : 0xFFFF;
    r.tempC[i] = f && e.tempC[i] >= 0 ? (uint8_t)constrainf(e.tempC[i] + 0.5f, 0, 254) : 255;
    any |= rpm >= 0 || r.tempC[i] != 255;
  }
  r.voltCv = fresh && e.voltage > 0 ? (uint16_t)(e.voltage * 100) : 0;
  r.currentDa = fresh && e.current >= 0 ? (int16_t)constrainf(e.current * 10, 0, 32767) : (int16_t)-32768;
  uint8_t x = 0;
  const uint8_t* q = (const uint8_t*)&r;
  for (size_t i = 0; i < sizeof(r) - 1; i++) x ^= q[i];
  r.crc = x;
  return any;
}

void Blackbox::update(uint32_t nowUs) {
  if (!backend_) return;
  bool armed = bus::status.get().armed;
  if (armed && !wasArmed_) {
    if (backend_ == 3) sdlog::startFile();  // one file per flight
    writeHeader();
    active_ = true;
    count_ = 0;
    burstDone_ = false;
  }
  if (!armed && wasArmed_) {
    active_ = false;
    gyroTapStop();
    if (backend_ == 2) hal::logFlush();
    if (backend_ == 3) sdlog::endFile();
  }
  wasArmed_ = armed;
  if (!active_) return;
  Record r;
  fill(r, nowUs);
  bool ok;
  if (count_++ % KEY_EVERY == 0) {
    ok = write((const uint8_t*)&r, sizeof(r));
  } else {
    uint8_t buf[MAX_DELTA];
    ok = write(buf, encodeDelta(prev_, r, buf));
  }
  prev_ = r;
  if (ok && ++escN_ >= escEvery_) {
    escN_ = 0;  // ESC frame between two records (does not touch the delta chain)
    EscRecord er;
    if (fillEsc(er, nowUs)) write((const uint8_t*)&er, sizeof(er));
  }
  // high-rate gyro capture (SD card only): bb_gyro_s once per flight when the throttle first passes 25 %
  // (in the air, not on the ground), or on request (CLI gyrocap / configurator)
  if (backend_ == 3 && gyroTapAvailable()) {
    const Config& c = bus::config;
    uint8_t req = bus::gyroCapReq;
    if (req) { bus::gyroCapReq = 0; gyroTapStart(nowUs, req * 1000u); }
    else if (!burstDone_ && c.bb_gyro_s && r.throttle > 250) { burstDone_ = true; gyroTapStart(nowUs, c.bb_gyro_s * 1000u); }
    GyroFrame gf;
    for (int k = 0; k < 8 && gyroTapFrame(gf); k++) write((const uint8_t*)&gf, sizeof(gf));  // <= 128 samples per call
  }
  // a frame was lost or cut: the deltas after it cannot be decoded -> send a full record next
  if (!ok) count_ = 0;
}

}  // namespace logging
