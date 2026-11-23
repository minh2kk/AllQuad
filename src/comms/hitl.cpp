#include "../common/size_opt.h"  // -Os on ESP32 (bench tool, not flight critical)
#include "hitl.h"

#include <math.h>
#include <string.h>

#include "../app/app.h"
#include "../bus/bus.h"
#include "../os/os.h"
#include "msp.h"

namespace hitl {

using fc::Vec3;

namespace {

struct Sample {
  Vec3 gyro, acc, mag;
  float pa = 0;
  uint32_t imuSeq = 0, baroSeq = 0, magSeq = 0, ms = 0;
  float vbat = -1, amps = -1;
};
volatile bool on = false;
bool rcOwned = false;
Sample s;
uint32_t frames = 0, bad = 0, rateN = 0, rateMs = 0;
float rateHz = 0;
float motorOut[4] = {0, 0, 0, 0};
bool motorArmed = false;
comms::MspParser parser;

float rdf(const uint8_t* p) { float f; memcpy(&f, p, 4); return f; }
int32_t rdi(const uint8_t* p) { int32_t v; memcpy(&v, p, 4); return v; }
uint16_t rd16(const uint8_t* p) { return (uint16_t)(p[0] | (p[1] << 8)); }
void wrf(uint8_t* p, float f) { memcpy(p, &f, 4); }
void wr16(uint8_t* p, int v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
// every float field of a frame finite (a damaged frame is dropped, the last sample stays)
bool finite(const uint8_t* p) {
  static const uint8_t at[] = {4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 56, 60, 64, 68, 72, 76, 96, 100};
  for (uint8_t o : at)
    if (!isfinite(rdf(p + o))) return false;
  return true;
}

class Imu : public drv::ImuDriver {
 public:
  bool init(uint16_t) override { return true; }
  bool read(Vec3& g, Vec3& a) override {
    static uint32_t lastSeq = 0, same = 0;
    Sample c;
    {
      os::CriticalGuard lock;
      c.gyro = s.gyro; c.acc = s.acc; c.imuSeq = s.imuSeq; c.ms = s.ms;
    }
    if (!c.imuSeq || os::timeMs() - c.ms > 200) return false;  // the PC stopped: like a dead IMU
    same = c.imuSeq == lastSeq ? same + 1 : 0;
    lastSeq = c.imuSeq;
    g = c.gyro;
    // the fast loop removes the accelerometer calibration of the real chip: put it in, so the
    // estimators see exactly the simulated specific force
    const fc::Config& cf = bus::config;
    for (int i = 0; i < 3; i++) a[i] = c.acc[i] / (cf.acc_scale[i] != 0 ? cf.acc_scale[i] : 1.0f) + cf.acc_bias[i];
    return true;
  }
  const char* name() const override { return "HITL-IMU"; }
};
class Baro : public drv::BaroDriver {
 public:
  bool init() override { return true; }
  bool update(float& pa, float& t) override {
    os::CriticalGuard lock;
    if (s.baroSeq == last_ || s.pa <= 0) return false;
    last_ = s.baroSeq;
    pa = s.pa;
    t = 25.0f;
    return true;
  }
  const char* name() const override { return "HITL-BARO"; }

 private:
  uint32_t last_ = 0;
};
class Mag : public drv::MagDriver {
 public:
  bool init() override { return true; }
  bool read(Vec3& f) override {
    os::CriticalGuard lock;
    if (!s.magSeq || os::timeMs() - s.ms > 200) return false;
    f = s.mag;
    return true;
  }
  const char* name() const override { return "HITL-MAG"; }
};
Imu imuDrv;
Baro baroDrv;
Mag magDrv;

// one PC frame: sensors in, the answer out (payload of the same MSP command)
void apply(const uint8_t* p, size_t n, uint8_t* r, size_t& rn) {
  uint32_t now = os::timeMs();
  if (n >= FRAME_LEN && finite(p)) {
    uint8_t flags = p[44];
    {
      os::CriticalGuard lock;
      s.gyro = Vec3(rdf(p + 4), rdf(p + 8), rdf(p + 12));
      s.acc = Vec3(rdf(p + 16), rdf(p + 20), rdf(p + 24));
      s.pa = rdf(p + 28);
      s.mag = Vec3(rdf(p + 32), rdf(p + 36), rdf(p + 40));
      s.imuSeq++;
      s.baroSeq++;
      s.magSeq++;
      s.ms = now;
      s.vbat = (flags & F_BATTERY) ? rdf(p + 96) : -1.0f;
      s.amps = (flags & F_BATTERY) ? rdf(p + 100) : -1.0f;
    }
    int32_t la = rdi(p + 48), lo = rdi(p + 52);
    if ((flags & F_GPS) && la >= -900000000 && la <= 900000000 && lo >= -1800000000 && lo <= 1800000000) {
      fc::GpsData g{};
      g.sats = p[45];
      g.fix3d = g.sats >= 4;
      g.lat = rdi(p + 48) * 1e-7;
      g.lon = rdi(p + 52) * 1e-7;
      g.altMsl = rdf(p + 56);
      g.velN = rdf(p + 60); g.velE = rdf(p + 64); g.velD = rdf(p + 68);
      g.hAcc = rdf(p + 72);
      g.vAcc = g.hAcc * 1.5f;
      g.groundSpeed = sqrtf(g.velN * g.velN + g.velE * g.velE);
      g.courseDeg = atan2f(g.velE, g.velN) * fc::RAD2DEG;
      if (g.courseDeg < 0) g.courseDeg += 360.0f;
      g.timeMs = now;
      bus::gps.publish(g);
    }
    if (flags & F_RANGE) {
      float d = rdf(p + 76);
      fc::RangeSample rs{d, d > 0.02f, 200};
      bus::range.publish(rs);
    }
    rcOwned = flags & F_RC;
    if (rcOwned) {
      fc::RcInput rc = bus::rc.get();
      for (int i = 0; i < 8; i++) rc.ch[i] = rd16(p + 80 + i * 2);
      for (int i = 8; i < fc::RC_MAX_CH; i++) rc.ch[i] = 1500;
      rc.count = 8;
      rc.failsafeFlag = false;
      rc.lastFrameMs = now;
      rc.linkQuality = 100;
      rc.everReceived = true;
      bus::rc.publish(rc);
    }
    frames++;
    rateN++;
    if (now - rateMs >= 1000) { rateHz = rateN * 1000.0f / (float)(now - rateMs); rateN = 0; rateMs = now; }
  } else {
    bad++;
  }
  // answer
  memset(r, 0, REPLY_LEN);
  memcpy(r, p, n >= 4 ? 4 : 0);  // sequence number echoed (latency on the PC)
  float m[4];
  bool armed;
  {
    os::CriticalGuard lock;
    memcpy(m, motorOut, sizeof(m));
    armed = motorArmed;
  }
  for (int i = 0; i < 4; i++) wrf(r + 4 + i * 4, armed ? m[i] : 0.0f);
  fc::VehicleStatus st = bus::status.get();
  fc::AttitudeState a = bus::attitude.get();
  fc::NavEstimate e = bus::nav.get();
  r[20] = st.armed;
  r[21] = (uint8_t)st.mode;
  r[22] = (uint8_t)st.navPhase;
  r[23] = (uint8_t)((st.failsafe ? 1 : 0) | 2);
  wr16(r + 24, (int)(a.roll * fc::RAD2DEG * 10));
  wr16(r + 26, (int)(a.pitch * fc::RAD2DEG * 10));
  wr16(r + 28, (int)(e.yaw * fc::RAD2DEG * 10));
  int32_t altCm = (int32_t)(e.alt * 100);
  memcpy(r + 30, &altCm, 4);
  wr16(r + 34, (int)(bus::loopPerf.periodUs > 0 ? 1000000.0f / bus::loopPerf.periodUs : 0));
  rn = REPLY_LEN;
}

}  // namespace

bool active() { return on; }
bool ownsRc() { return on && rcOwned; }
drv::ImuDriver* imu() { return &imuDrv; }
drv::BaroDriver* baro() { return &baroDrv; }
drv::MagDriver* mag() { return &magDrv; }

bool begin(const uint8_t* p, size_t n, uint8_t* r, size_t& rn) {
  if (!on) {
    if (bus::status.get().armed || n < FRAME_LEN) return false;
    rateMs = os::timeMs();
    apply(p, n, r, rn);   // a first sample before the drivers change
    app::hitlSwap();
    on = true;
    bus::gyroCalRequest = true;  // the real chip's gyro bias is not the simulated one
    bus::postMessage("HITL: sensors from the PC, motors stopped");
    return true;
  }
  apply(p, n, r, rn);
  return true;
}

void poll(uint32_t now) {
  if (!on) return;
  int ch;
  comms::MspFrame f;
  for (int budget = 4096; budget-- > 0 && (ch = hal::portRead(hal::Port::CONSOLE)) >= 0;) {
    if (!parser.feed((uint8_t)ch, f) || f.cmd != MSP_CMD) continue;
    uint8_t r[REPLY_LEN];
    size_t rn = 0;
    apply(f.payload, f.len, r, rn);
    uint8_t out[REPLY_LEN + 16];
    size_t on2 = comms::mspPack(out, true, '>', MSP_CMD, r, (uint16_t)rn);
    hal::portWrite(hal::Port::CONSOLE, out, on2);
    if (f.len > 44 && (f.payload[44] & F_EXIT) && !bus::status.get().armed) hal::reboot();
  }
  // the PC went away: back to the real sensors through a restart (disarmed only)
  uint32_t last;
  {
    os::CriticalGuard lock;
    last = s.ms;
  }
  if (now - last > 1000 && !bus::status.get().armed) hal::reboot();
}

void motorsOut(const float v[4], bool armed) {
  os::CriticalGuard lock;
  for (int i = 0; i < 4; i++) motorOut[i] = v[i];
  motorArmed = armed;
}

bool battery(float& volts, float& amps) {
  if (!on) return false;
  os::CriticalGuard lock;
  if (s.vbat < 0) return false;
  volts = s.vbat;
  amps = s.amps;
  return true;
}

Stats stats() { return {frames, bad, s.ms, rateHz}; }

}  // namespace hitl
