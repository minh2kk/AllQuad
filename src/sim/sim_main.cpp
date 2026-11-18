// AllQuad - software-in-the-loop simulator.
// Runs the complete firmware (all RTOS tasks, real scheduling priorities) on a
// deterministic host scheduler against a rigid-body quad model with sensor noise,
// frame vibration at the motor rotation frequency, wind, and scripted RC input.
//
//   tools/run_sim.sh              all scenarios
//   build/sim <scenario>          one of: basic mission fence battery flow vibration turtle tune wind glitch
//                                 hover nomag terrain cruise mission2 autotune autotune_abort motortest
//                                 bidir edt ekf_faults rescue rescue_sanity launch vibe
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <fcntl.h>
#include <time.h>
#include <unistd.h>

#include <functional>
#include <random>

#include "../../test/host/hal_host.h"
#include "../app/app.h"
#include "../bus/bus.h"
#include "../estimation/ekf.h"
#include "../estimation/nav_estimator.h"
#include "../navigation/mission.h"
#include "../os/os.h"

using namespace fc;

namespace {

std::mt19937 rng(42);
float gauss(float s) { std::normal_distribution<float> d(0.0f, s); return d(rng); }

// ------------------------------------------------------------------ physics
struct QuadModel {
  float mass = 0.55f, tMax = 3.6f, arm = 0.08f, kYaw = 0.016f;
  Vec3 inertia{0.0026f, 0.0028f, 0.0048f};
  float motorTau = 0.03f, drag = 0.25f;
  Vec3 pos, vel, w, accel, wind;
  Quat q;
  float m[4] = {0, 0, 0, 0};
  float eff[4] = {1, 1, 1, 1};  // < 1: weak motor / damaged prop (spins slower for the same command)
  bool onGround = true;

  void step(const float cmd[4], bool armed, float dt) {
    static const float X[4] = {-1, +1, -1, +1}, Y[4] = {+1, +1, -1, -1}, D[4] = {-1, +1, +1, -1};
    float thrust = 0;
    Vec3 tau;
    for (int i = 0; i < 4; i++) {
      float c = armed ? cmd[i] : 0.0f;
      m[i] += (c - m[i]) * (dt / (motorTau + dt));
      float sp = eff[i] * m[i];
      float t = tMax * (sp * sp * 0.6f + sp * 0.4f);
      thrust += t;
      tau.x += -Y[i] * arm * t;
      tau.y += X[i] * arm * t;
      tau.z += D[i] * kYaw * t;
    }
    Vec3 Iw(inertia.x * w.x, inertia.y * w.y, inertia.z * w.z);
    Vec3 wd = tau - w.cross(Iw);
    w += Vec3(wd.x / inertia.x, wd.y / inertia.y, wd.z / inertia.z) * dt;
    Quat dq(0, w.x, w.y, w.z);
    Quat qd = q * dq;
    q.w += 0.5f * qd.w * dt; q.x += 0.5f * qd.x * dt; q.y += 0.5f * qd.y * dt; q.z += 0.5f * qd.z * dt;
    q.normalize();
    Vec3 f = q.rotate(Vec3(0, 0, -thrust)) + Vec3(0, 0, mass * GRAVITY) - vel * drag + (onGround ? Vec3() : wind);
    accel = f * (1.0f / mass);
    vel += accel * dt;
    pos += vel * dt;
    onGround = pos.z >= 0;
    if (onGround) {
      pos.z = 0;
      if (vel.z > 0) vel = Vec3();
      vel.x *= 0.9f; vel.y *= 0.9f;
      if (accel.z > 0) accel = Vec3();
      w = w * 0.9f;
    }
  }
  Vec3 specificForceBody() const { return q.rotateInv(accel - Vec3(0, 0, GRAVITY)); }
  float motorHz(int i) const { return 27000.0f * eff[i] * m[i] / 60.0f; }
};

struct Options {
  bool gps = true, flow = false, vibration = false, mag = true, range = false, baro = true;
  float baroOffset = 0;                 // injected barometer error (m)
  float magRot = 0;                     // compass interference: field rotated about the vertical (rad)
  float accVib = 0;                     // accelerometer vibration amplitude (m/s^2)
  float currentA = 8.0f;
  bool adc = true;                      // battery divider / current sensor wired to the FC
  bool held = false;                    // craft strapped down (compassmot)
  bool imuDead = false;                 // first IMU stops answering (failover test)
  float magCmot = 0;                    // compass interference (G per A, body x)
  bool edtEsc = false;                  // ESCs answer DShot command 13 with extended telemetry (AM32)
  int bidirDead = -1;                   // bidirectional DShot: this output stops answering (-1 none)
  int bidirGlitchEvery = 0;             // every Nth answer of output 0 decodes to 4x its eRPM (0 = never)
  Vec3 gpsOffset;                       // injected GPS position error (glitch), m
  float (*terrain)(float n, float e) = nullptr;  // ground height under the craft (rangefinder only)
};

QuadModel quad;
Options opt;
RcInput rcIn;
bool rcAlive = true;
const Vec3 gyroBias(0.012f, -0.02f, 0.006f);
const Vec3 magEarth(0.22f, 0.0f, 0.42f);
const double LAT0 = 21.0285, LON0 = 105.8542;
float vibPhase[4] = {0, 0, 0, 0};
Vec3 lastVib;

// battery current from the motor commands (armed flight or a motor test)
float simCurrent() {
  float m = 0;
  for (int i = 0; i < 4; i++) m += quad.m[i];
  return opt.currentA * (m / 4.0f) / 0.45f;
}

// ------------------------------------------------------------------ simulated drivers
class SimImu : public drv::ImuDriver {
 public:
  bool init(uint16_t) override { return true; }
  bool read(Vec3& g, Vec3& a) override {
    Vec3 vib;
    if (opt.vibration) {
      for (int i = 0; i < 4; i++) {
        vibPhase[i] += 2.0f * PI_F * quad.motorHz(i) / 1000.0f;
        float s = sinf(vibPhase[i]) * (quad.m[i] > 0.05f ? 1.0f : 0.0f);
        vib += Vec3(35.0f * s, 30.0f * s * (i % 2 ? 1 : -1), 12.0f * s) * DEG2RAD;
      }
    }
    lastVib = vib;
    if (opt.imuDead) return false;
    g = quad.w + gyroBias + vib + Vec3(gauss(0.004f), gauss(0.004f), gauss(0.004f));
    for (int i = 0; i < 3; i++) g[i] = constrainf(g[i], -2000 * DEG2RAD, 2000 * DEG2RAD);  // gyro range
    a = quad.specificForceBody() + Vec3(gauss(0.4f), gauss(0.4f), gauss(0.6f));
    if (opt.accVib > 0) {
      static float ph = 0;
      ph += 2.0f * PI_F * 180.0f / 1000.0f;
      a += Vec3(opt.accVib * sinf(ph), opt.accVib * cosf(ph * 1.1f), opt.accVib * sinf(ph * 0.9f));
    }
    for (int i = 0; i < 3; i++) a[i] = constrainf(a[i], -16.0f * GRAVITY, 16.0f * GRAVITY);  // accel range
    return true;
  }
  const char* name() const override { return "SIM-IMU"; }
};

// second IMU (same physics, its own noise): takes over when the first one dies
class SimImu2 : public drv::ImuDriver {
 public:
  bool init(uint16_t) override { return true; }
  bool read(Vec3& g, Vec3& a) override {
    g = quad.w + gyroBias * 0.5f + Vec3(gauss(0.005f), gauss(0.005f), gauss(0.005f));
    a = quad.specificForceBody() + Vec3(gauss(0.5f), gauss(0.5f), gauss(0.7f));
    return true;
  }
  const char* name() const override { return "SIM-IMU2"; }
};
SimImu2 simImu2;
bool useImu2 = false;

class SimBaro : public drv::BaroDriver {
 public:
  bool init() override { return true; }
  bool update(float& pa, float& t) override {
    float alt = -quad.pos.z + gauss(0.08f) + opt.baroOffset;
    pa = 101325.0f * powf(1.0f - alt / 44330.0f, 5.255f);
    t = 25.0f;
    return true;
  }
  const char* name() const override { return "SIM-BARO"; }
};

class SimMag : public drv::MagDriver {
 public:
  bool init() override { return true; }
  bool read(Vec3& f) override {
    Vec3 m = magEarth;
    if (opt.magRot != 0) {  // interference: the measured field points elsewhere
      float c = cosf(opt.magRot), s = sinf(opt.magRot);
      m = Vec3(magEarth.x * c - magEarth.y * s, magEarth.x * s + magEarth.y * c, magEarth.z);
    }
    f = quad.q.rotateInv(m) + Vec3(gauss(0.003f), gauss(0.003f), gauss(0.003f));
    f.x += opt.magCmot * simCurrent();
    return true;
  }
  const char* name() const override { return "SIM-MAG"; }
};

SimImu simImu;
SimBaro simBaro;
SimMag simMag;

// ------------------------------------------------------------------ sim tasks
bool edtOn = false;
void physicsTask(void*) {
  quad.step(hal::host::motors, hal::host::motorsArmed, 1.0f / bus::config.loop_hz);
  if (opt.held) { quad.pos = Vec3(); quad.vel = Vec3(); quad.w = Vec3(); quad.q = Quat::fromEuler(0, 0, 30 * DEG2RAD); }
  // AM32-like ESCs: after DShot command 13 they slip temperature / voltage / current frames in
  if (opt.edtEsc && hal::host::bidir && hal::host::lastMotorCommand == 13) edtOn = true;
  if (edtOn) {
    static uint32_t n = 0;
    if (++n % 40 == 0)
      for (int i = 0; i < 4; i++) {
        dshot::Edt& e = hal::host::edt[i];
        float thr = hal::host::motorsArmed ? quad.m[i] : 0.0f;
        e.tempC = 30.0f + 40.0f * thr;
        e.volts = 16.0f - 0.01f * os::timeMs() / 1000.0f;
        e.amps = roundf(opt.currentA * 0.25f * (thr / 0.45f));
        e.frames++;
      }
  }
  if (hal::host::bidir) {  // bidirectional DShot: each ESC answers with its eRPM (+1 % noise)
    static uint32_t answers = 0;
    answers++;
    for (int i = 0; i < 4; i++) {
      float pairs = bus::config.motor_poles * 0.5f;
      hal::host::erpm[i] = quad.m[i] > 0.02f ? quad.motorHz(i) * 60.0f * pairs * (1.0f + gauss(0.01f)) : 0.0f;
      if (i == opt.bidirDead) hal::host::erpm[i] = -1;  // no valid answer
      if (i == 0 && opt.bidirGlitchEvery && answers % opt.bidirGlitchEvery == 0) hal::host::erpm[i] *= 4.0f;
    }
  }
}

void simIoTask(void*) {
  static uint32_t tick = 0;
  tick++;
  uint32_t now = os::timeMs();
  if (rcAlive) { rcIn.lastFrameMs = now; bus::rc.publish(rcIn); }
  if (opt.gps && tick % 10 == 0 && now > 1000) {
    GpsData g{};
    g.fix3d = true; g.sats = 14; g.hAcc = 1.2f; g.vAcc = 2.0f;
    double lat, lon;
    NavEstimator::nedToLla(quad.pos.x + opt.gpsOffset.x + gauss(0.3f), quad.pos.y + opt.gpsOffset.y + gauss(0.3f), LAT0,
                           LON0, lat, lon);
    g.lat = lat; g.lon = lon;
    g.altMsl = 10.0f - quad.pos.z;
    g.velN = quad.vel.x + gauss(0.1f); g.velE = quad.vel.y + gauss(0.1f); g.velD = quad.vel.z;
    g.groundSpeed = sqrtf(g.velN * g.velN + g.velE * g.velE);
    g.timeMs = now;
    bus::gps.publish(g);
  }
  if ((opt.flow || opt.range) && tick % 2 == 0) {
    float h = -quad.pos.z - (opt.terrain ? opt.terrain(quad.pos.x, quad.pos.y) : 0.0f);
    float ct = quad.q.cosTilt();
    RangeSample r{h / fmaxf(ct, 0.5f) + gauss(0.01f), h > 0.02f, 200};
    bus::range.publish(r);
    if (opt.flow && h > 0.05f) {
      float yaw = quad.q.yaw(), cy = cosf(yaw), sy = sinf(yaw);
      float vx = quad.vel.x * cy + quad.vel.y * sy, vy = -quad.vel.x * sy + quad.vel.y * cy;
      FlowSample f{-vx / h - quad.w.y + gauss(0.02f), -vy / h + quad.w.x + gauss(0.02f), 200, 0.02f};
      bus::flow.publish(f);
    }
  }
  if (opt.vibration) {
    EscTelemetry e{};
    for (int i = 0; i < 4; i++) { e.rpm[i] = quad.motorHz(i) * 60.0f * (1.0f + gauss(0.01f)); e.lastUpdateMs[i] = now; }
    bus::esc.publish(e);
  }
  hal::host::currentPin = opt.adc ? fmaxf(simCurrent(), 0.2f) / 10.0f : -1.0f;
  hal::host::vbatPin = opt.adc ? (16.4f - 0.01f * now / 1000.0f) / 11.0f : 0.0f;
}

// ------------------------------------------------------------------ harness
struct Result { const char* name; bool ok; char detail[100]; };
Result results[64];
int nResults = 0;
void check(const char* name, bool ok, const char* fmt, double v) {
  Result& r = results[nResults++];
  r.name = name; r.ok = ok;
  snprintf(r.detail, sizeof(r.detail), fmt, v);
}

void setup(const std::function<void(Config&)>& tweak, Quat attitude = Quat::fromEuler(0, 0, 30 * DEG2RAD)) {
  hal::host::reset();
  os::hostReset();
  quad = QuadModel();
  quad.q = attitude;
  rcAlive = true;
  memset(&rcIn, 0, sizeof(rcIn));
  for (int i = 0; i < RC_MAX_CH; i++) rcIn.ch[i] = 1500;
  rcIn.ch[RC_THROTTLE] = 1000;
  for (int i = 4; i < RC_MAX_CH; i++) rcIn.ch[i] = 1000;
  rcIn.ch[5] = 1500;  // ANGLE
  rcIn.count = 16;
  rcIn.everReceived = true;
  rcIn.linkQuality = 100;
  configDefaults(bus::config);
  Config& c = bus::config;
  for (int i = 0; i < NUM_PORTS; i++) c.port_fn[i] = (uint8_t)PortFn::NONE;
  for (int i = 0; i < NUM_PORTS_EXT; i++) c.port_fn_ext[i] = (uint8_t)PortFn::NONE;
  for (int i = 0; i < NUM_PORTS_EXT2; i++) c.port_fn_ext2[i] = (uint8_t)PortFn::NONE;
  c.acc_calibrated = 1;
  c.mag_bias[0] = 1e-4f;
  c.cur_scale = 10.0f;
  if (getenv("SIM_EST")) c.est_type = (uint8_t)atoi(getenv("SIM_EST"));  // compare the estimators
  tweak(c);
  bus::configChanged();
  nav::clear();
  app::Drivers d{&simImu, opt.baro ? &simBaro : nullptr, opt.mag ? &simMag : nullptr, useImu2 ? &simImu2 : nullptr};
  app::start(d, false);
  os::createTask({"physics", physicsTask, nullptr, 1000000u / c.loop_hz, 7, 0, -1, false});
  os::createTask({"sim-io", simIoTask, nullptr, 10000, 5, 0, -1, false});
}

void run(float seconds, const std::function<void(float)>& script, const std::function<void(float)>& eval) {
  const float dt = 0.001f;
  for (int k = 0; k < (int)(seconds / dt); k++) {
    float t = k * dt;
    script(t);
    os::hostRunUntil((uint64_t)((t + dt) * 1e6));
    static uint32_t msgSeen = 0;
    if (getenv("SIM_MSG") && bus::messageSeq() != msgSeen) {  // debug: every OSD / GCS message
      msgSeen = bus::messageSeq();
      char m[64];
      bus::readMessage(m, sizeof(m));
      printf("  [%6.2f] %s\n", t, m);
    }
    eval(t);
  }
}

float distHome() { return hypotf(quad.pos.x, quad.pos.y); }
float alt() { return -quad.pos.z; }
VehicleStatus status() { return bus::status.get(); }
void trace(float t) {
  VehicleStatus s = status();
  printf("  t=%5.1f %-8s %-12s armed=%d alt=%6.2f N=%7.2f E=%7.2f roll=%6.1f\n", t, flightModeName(s.mode),
         navPhaseName(s.navPhase), s.armed, alt(), quad.pos.x, quad.pos.y, quad.q.roll() * RAD2DEG);
}

// ================================================================== scenarios
void scenarioBasic() {
  printf("\n== basic: arm, ANGLE, ALTHOLD, angle step, ACRO, POSHOLD+wind, RC loss -> RTH -> land ==\n");
  setup([](Config& c) { c.bb_rate_hz = 100; c.bb_backend = 2; });  // blackbox to onboard log
  float altAt10 = 0, maxAltErr = 0, maxTilt = 0, rollErr = 0, rateErr = 0, posDev = 0, holdN = 0, holdE = 0;
  int rateN = 0;
  bool armed = false;
  float disarmT = -1, maxAltRth = 0;
  run(110, [&](float t) {
    if (t >= 3.0f) rcIn.ch[4] = 2000;
    if (t >= 3.5f && t < 6.0f) rcIn.ch[RC_THROTTLE] = (uint16_t)(1000 + 560 * fminf(1.0f, (t - 3.5f)));
    if (t >= 6.0f) { rcIn.ch[5] = 2000; rcIn.ch[RC_THROTTLE] = 1500; }
    rcIn.ch[RC_ROLL] = (t >= 14 && t < 16) ? 1750 : 1500;
    if (t >= 17.0f && t < 17.4f) { rcIn.ch[5] = 1000; rcIn.ch[RC_ROLL] = 1650; rcIn.ch[RC_THROTTLE] = 1520; }
    if (t >= 17.4f && t < 17.8f) { rcIn.ch[5] = 1000; rcIn.ch[RC_ROLL] = 1350; rcIn.ch[RC_THROTTLE] = 1520; }
    if (t >= 17.8f) { rcIn.ch[5] = 2000; rcIn.ch[RC_THROTTLE] = 1500; }
    if (t >= 21.0f) rcIn.ch[6] = 1500;  // POSHOLD
    quad.wind = (t >= 23 && t < 29) ? Vec3(0.3f, 0.6f, 0) : Vec3();
    if (t >= 32.0f) rcAlive = false;
  }, [&](float t) {
    VehicleStatus s = status();
    if (t > 3.2f && s.armed) armed = true;
    if (fabsf(t - 10.0f) < 0.0005f) altAt10 = alt();
    if (t > 10 && t < 14) maxAltErr = fmaxf(maxAltErr, fabsf(alt() - altAt10));
    if (t > 7 && t < 14) maxTilt = fmaxf(maxTilt, fabsf(quad.q.roll() * RAD2DEG));
    if (t > 15 && t < 16) rollErr = fmaxf(rollErr, fabsf(quad.q.roll() * RAD2DEG - 22.5f));
    if ((t > 17.15f && t < 17.4f) || (t > 17.55f && t < 17.8f)) {
      float stick = (rcIn.ch[RC_ROLL] - 1500) / 500.0f;
      float sp = stick * 70 + 600 * fabsf(stick) * stick;
      rateErr += fabsf(quad.w.x * RAD2DEG - sp);
      rateN++;
    }
    if (fabsf(t - 22.0f) < 0.0005f) { holdN = quad.pos.x; holdE = quad.pos.y; }
    if (t > 22 && t < 32) posDev = fmaxf(posDev, hypotf(quad.pos.x - holdN, quad.pos.y - holdE));
    if (t > 32) maxAltRth = fmaxf(maxAltRth, alt());
    if (t > 32 && !s.armed && disarmT < 0) disarmT = t;
    if (fmodf(t, 10.0f) < 0.0005f) trace(t);
  });
  check("Arming via switch", armed, "armed=%.0f", armed);
  check("ALTHOLD altitude error", maxAltErr < 0.5f, "max %.2f m", maxAltErr);
  check("ALTHOLD level", maxTilt < 3.0f, "max roll %.2f deg", maxTilt);
  check("ANGLE 22.5 deg step", rollErr < 3.0f, "max error %.2f deg", rollErr);
  check("ACRO rate tracking", rateN && rateErr / rateN < 12.0f, "mean error %.1f deg/s", rateN ? rateErr / rateN : 999.0);
  check("POSHOLD with wind gust", posDev < 2.5f, "max drift %.2f m", posDev);
  check("RC loss -> RTH climbs to rth_alt", maxAltRth > 18.0f, "max alt %.1f m", maxAltRth);
  check("RTH lands at home & disarms", disarmT > 0 && distHome() < 3.0f, "distance %.2f m", distHome());
  int fastRuns = 0;
  for (int i = 0; i < os::taskCount(); i++)
    if (!strcmp(os::taskStats(i).name, "fast")) fastRuns = (int)os::taskStats(i).runs;
  check("Fast task ran at 1 kHz", fastRuns > 109000 && fastRuns < 111000, "runs %.0f", fastRuns);
  {
    const volatile bus::LoopPerf& p = bus::loopPerf;
    printf("  fast loop on this PC: imu %.2f filters %.2f attitude %.2f pid %.2f motors %.2f = %.2f us, worst %.1f us\n",
           (double)p.imuUs, (double)p.filterUs, (double)p.ahrsUs, (double)p.pidUs, (double)p.motorUs, (double)p.totalUs,
           (double)p.maxTotalUs);
  }
  check("Blackbox recorded the flight", hal::host::log.size() > 50000, "%.0f bytes", (double)hal::host::log.size());
  if (const char* path = getenv("SIM_BLACKBOX")) {
    FILE* f = fopen(path, "wb");
    if (f) { fwrite(hal::host::log.data(), 1, hal::host::log.size(), f); fclose(f); }
  }
}

void scenarioMission() {
  printf("\n== mission: auto takeoff, 3 waypoints, speed change, RTL + land ==\n");
  setup([](Config& c) { c.rth_alt = 10; });
  struct P { float n, e, alt, hold; } wps[3] = {{30, 0, 10, 0}, {30, 30, 15, 2}, {0, 30, 10, 0}};
  int idx = 0;
  nav::MissionItem it{};
  it.cmd = nav::CMD_TAKEOFF; it.frame = 3; it.alt = 10;
  nav::set(idx++, it);
  for (int i = 0; i < 3; i++) {
    double lat, lon;
    NavEstimator::nedToLla(wps[i].n, wps[i].e, LAT0, LON0, lat, lon);
    it = nav::MissionItem{};
    it.cmd = nav::CMD_WAYPOINT; it.frame = 3; it.lat = (int32_t)(lat * 1e7); it.lon = (int32_t)(lon * 1e7);
    it.alt = wps[i].alt; it.param1 = wps[i].hold;
    nav::set(idx++, it);
    if (i == 1) { it = nav::MissionItem{}; it.cmd = nav::CMD_CHANGE_SPEED; it.param2 = 3; nav::set(idx++, it); }
  }
  it = nav::MissionItem{};
  it.cmd = nav::CMD_RTL;
  nav::set(idx++, it);
  nav::setCount(idx);
  float minD[3] = {1e9f, 1e9f, 1e9f};
  float disarmT = -1;
  bool sawMission = false;
  run(120, [&](float t) {
    if (t >= 2.5f) rcIn.ch[7] = 1500;  // MISSION
    if (t >= 3.0f) rcIn.ch[4] = 2000;  // arm
  }, [&](float t) {
    VehicleStatus s = status();
    if (s.mode == FlightMode::MISSION && s.armed) sawMission = true;
    for (int i = 0; i < 3; i++) minD[i] = fminf(minD[i], hypotf(quad.pos.x - wps[i].n, quad.pos.y - wps[i].e));
    if (t > 5 && !s.armed && disarmT < 0) disarmT = t;
    if (fmodf(t, 10.0f) < 0.0005f) trace(t);
  });
  check("Armed into MISSION (auto takeoff)", sawMission, "%.0f", sawMission);
  for (int i = 0; i < 3; i++) check("Waypoint reached", minD[i] < 2.5f, "closest %.2f m", minD[i]);
  check("Mission RTL landed & disarmed", disarmT > 0 && distHome() < 3.0f, "distance %.2f m", distHome());
}

void scenarioFence() {
  printf("\n== fence: fly away in POSHOLD, 25 m geofence -> RTH -> land ==\n");
  setup([](Config& c) { c.fence_enable = 1; c.fence_radius = 25; c.rth_alt = 8; });
  float maxD = 0, disarmT = -1;
  bool breach = false;
  run(90, [&](float t) {
    if (t >= 3.0f) rcIn.ch[4] = 2000;
    if (t >= 3.5f && t < 6.0f) rcIn.ch[RC_THROTTLE] = 1560;
    if (t >= 6.0f) { rcIn.ch[5] = 2000; rcIn.ch[RC_THROTTLE] = 1500; rcIn.ch[6] = 1500; }
    if (t >= 9.0f) rcIn.ch[RC_PITCH] = 2000;  // full forward, pilot does not let go
  }, [&](float t) {
    VehicleStatus s = status();
    maxD = fmaxf(maxD, distHome());
    if (s.mode == FlightMode::RTH) breach = true;
    if (t > 5 && !s.armed && disarmT < 0) disarmT = t;
    if (fmodf(t, 10.0f) < 0.0005f) trace(t);
  });
  check("Geofence breach triggers RTH", breach, "%.0f", breach);
  check("Overshoot past fence limited", maxD < 40.0f, "max distance %.1f m", maxD);
  check("Returned, landed & disarmed", disarmT > 0 && distHome() < 3.0f, "distance %.2f m", distHome());
}

void scenarioBattery() {
  printf("\n== battery: capacity reserve reached -> LAND ==\n");
  setup([](Config& c) { c.bat_capacity = 300; c.bat_reserve_pct = 20; c.bat_fs_action = 1; });
  opt.currentA = 30.0f;
  float landT = -1, disarmT = -1;
  run(70, [&](float t) {
    if (t >= 3.0f) rcIn.ch[4] = 2000;
    if (t >= 3.5f && t < 6.0f) rcIn.ch[RC_THROTTLE] = 1580;
    if (t >= 6.0f) { rcIn.ch[5] = 2000; rcIn.ch[RC_THROTTLE] = 1500; }
  }, [&](float t) {
    VehicleStatus s = status();
    if (s.mode == FlightMode::LAND && landT < 0) landT = t;
    if (t > 5 && !s.armed && disarmT < 0) disarmT = t;
    if (fmodf(t, 10.0f) < 0.0005f) trace(t);
  });
  opt.currentA = 8.0f;
  check("Battery failsafe triggers LAND", landT > 20 && landT < 40, "at t=%.1f s", landT);
  check("Landed & disarmed", disarmT > landT && alt() < 0.2f, "at t=%.1f s", disarmT);
}

void scenarioFlow() {
  printf("\n== flow: no GPS, optical flow + rangefinder POSHOLD with wind ==\n");
  opt.gps = false;
  opt.flow = true;
  setup([](Config&) {});
  float dev = 0, hn = 0, he = 0;
  bool held = true;
  run(40, [&](float t) {
    if (t >= 3.0f) rcIn.ch[4] = 2000;
    if (t >= 3.5f && t < 5.5f) rcIn.ch[RC_THROTTLE] = 1560;
    if (t >= 5.5f) { rcIn.ch[5] = 2000; rcIn.ch[RC_THROTTLE] = 1500; rcIn.ch[6] = 1500; }
    quad.wind = (t >= 15 && t < 25) ? Vec3(0.3f, -0.4f, 0) : Vec3();
  }, [&](float t) {
    VehicleStatus s = status();
    if (fabsf(t - 12.0f) < 0.0005f) { hn = quad.pos.x; he = quad.pos.y; }
    if (t > 12) {
      dev = fmaxf(dev, hypotf(quad.pos.x - hn, quad.pos.y - he));
      if (s.mode != FlightMode::POSHOLD) held = false;
    }
    if (fmodf(t, 10.0f) < 0.0005f) trace(t);
  });
  opt.gps = true;
  opt.flow = false;
  check("POSHOLD available without GPS", held, "%.0f", held);
  check("Optical-flow hold drift", dev < 1.5f, "max drift %.2f m", dev);
}

void scenarioVibration() {
  printf("\n== vibration: motor-frequency frame vibration, RPM filter + dynamic notch ==\n");
  opt.vibration = true;
  bool noFilters = getenv("SIM_NO_NOTCH") != nullptr;  // comparison run: low-pass filters only
  setup([&](Config& c) {
    c.esc_telemetry = noFilters ? 0 : 1;
    if (noFilters) c.dyn_notch_count = 0;
  });
  double rawSq = 0, filtSq = 0;
  int n = 0;
  float altAt12 = 0, maxAltErr = 0, maxTilt = 0;
  run(25, [&](float t) {
    if (t >= 3.0f) rcIn.ch[4] = 2000;
    if (t >= 3.5f && t < 6.0f) rcIn.ch[RC_THROTTLE] = 1560;
    if (t >= 6.0f) { rcIn.ch[5] = 2000; rcIn.ch[RC_THROTTLE] = 1500; }
  }, [&](float t) {
    if (t > 12 && fmodf(t, 0.004f) < 0.0005f) {
      AttitudeState a = bus::attitude.get();
      PidDebug p = bus::pid.get();
      Vec3 truth = quad.w * RAD2DEG;
      Vec3 er = p.gyroRaw - truth, ef = a.rateDps - truth;
      rawSq += er.dot(er);
      filtSq += ef.dot(ef);
      n++;
    }
    if (fabsf(t - 12.0f) < 0.0005f) altAt12 = alt();
    if (t > 12) { maxAltErr = fmaxf(maxAltErr, fabsf(alt() - altAt12)); maxTilt = fmaxf(maxTilt, fabsf(quad.q.roll() * RAD2DEG)); }
    if (fmodf(t, 10.0f) < 0.0005f) trace(t);
  });
  opt.vibration = false;
  float rawRms = (float)sqrt(rawSq / n), filtRms = (float)sqrt(filtSq / n);
  printf("  gyro noise RMS: raw %.1f deg/s -> filtered %.2f deg/s\n", rawRms, filtRms);
  check("Vibration attenuated > 10x", filtRms < rawRms / 10.0f, "filtered/raw = %.3f", filtRms / rawRms);
  check("Stable hover with vibration", maxAltErr < 0.6f && maxTilt < 4.0f, "alt err %.2f m", maxAltErr);
}

void scenarioTurtle() {
  printf("\n== turtle: upside down on the ground, crash-flip mode ==\n");
  setup([](Config&) {}, Quat::fromEuler(PI_F, 0, 0));
  bool turtleMode = false, cmdReversed = false, leftOnly = false, cmdNormal = false;
  run(8, [&](float t) {
    if (t >= 2.5f) rcIn.ch[9] = 2000;  // turtle switch
    if (t >= 3.0f && t < 6.0f) rcIn.ch[4] = 2000;
    if (t >= 6.0f) rcIn.ch[4] = 1000;
    rcIn.ch[RC_ROLL] = (t >= 4.0f && t < 5.0f) ? 2000 : 1500;
  }, [&](float t) {
    VehicleStatus s = status();
    if (s.mode == FlightMode::TURTLE && s.armed) turtleMode = true;
    if (t > 3.1f && t < 6 && hal::host::lastMotorCommand == 21) cmdReversed = true;
    if (t > 4.5f && t < 4.6f)
      leftOnly = hal::host::motors[2] > 0.3f && hal::host::motors[3] > 0.3f && hal::host::motors[0] == 0 &&
                 hal::host::motors[1] == 0;
    if (t > 6.1f && hal::host::lastMotorCommand == 20) cmdNormal = true;
  });
  check("Arms into TURTLE while inverted", turtleMode, "%.0f", turtleMode);
  check("DShot reverse command sent", cmdReversed, "%.0f", cmdReversed);
  check("Roll stick spins left motors only", leftOnly, "%.0f", leftOnly);
  check("Normal direction restored on disarm", cmdNormal, "%.0f", cmdNormal);
}

void scenarioTune() {
  // Blackbox capture for the web analyser. SIM_TUNE_P scales roll/pitch P, SIM_BLACKBOX = output file.
  float pScale = getenv("SIM_TUNE_P") ? (float)atof(getenv("SIM_TUNE_P")) : 1.0f;
  float dScale = getenv("SIM_TUNE_D") ? (float)atof(getenv("SIM_TUNE_D")) : 1.0f;
  printf("\n== tune: ACRO snap doublets, blackbox 500 Hz (P x%.2f, D x%.2f) ==\n", pScale, dScale);
  opt.vibration = true;
  setup([&](Config& c) {
    c.esc_telemetry = 1;
    c.bb_rate_hz = 500;
    c.bb_backend = 2;
    float fScale = getenv("SIM_TUNE_F") ? (float)atof(getenv("SIM_TUNE_F")) : 1.0f;
    float iScale = getenv("SIM_TUNE_I") ? (float)atof(getenv("SIM_TUNE_I")) : 1.0f;
    for (int a = 0; a < 2; a++) { c.pid[a].p *= pScale; c.pid[a].d *= dScale; c.pid[a].f *= fScale; c.pid[a].i *= iScale; }
  });
  if (const char* w = getenv("SIM_WEAK_MOTOR")) {  // 1..4: that motor 15 % weaker (analyser test)
    int k = atoi(w);
    if (k >= 1 && k <= 4) quad.eff[k - 1] = 0.85f;
  }
  bool crashed = false;
  run(32, [&](float t) {
    if (t >= 3.0f) rcIn.ch[4] = 2000;
    if (t >= 3.5f && t < 6.0f) rcIn.ch[RC_THROTTLE] = 1580;
    if (t >= 6.0f && t < 8.0f) { rcIn.ch[5] = 2000; rcIn.ch[RC_THROTTLE] = 1500; }
    // ACRO from t = 8 s: the scripted "pilot" holds altitude with the throttle stick
    if (t >= 8.0f) {
      rcIn.ch[5] = 1000;
      float thr = 0.50f + 0.06f * (3.0f - alt()) - 0.08f * (-quad.vel.z);
      rcIn.ch[RC_THROTTLE] = (uint16_t)(1000 + 1000 * constrainf(thr, 0.35f, 0.7f));
    }
    // snap doublets: +, -, pause, pause (0.3 s each), roll then pitch
    bool doublets = t >= 9 && t < 29;
    int phase = (int)((t - 9) / 0.3f);
    int k = phase % 8;
    rcIn.ch[RC_ROLL] = doublets && k == 0 ? 1650 : (doublets && k == 1 ? 1350 : 1500);
    rcIn.ch[RC_PITCH] = doublets && k == 4 ? 1650 : (doublets && k == 5 ? 1350 : 1500);
    if (t >= 30) rcIn.ch[4] = 1000;  // disarm -> log closed
  }, [&](float t) {
    if (t > 8 && t < 29 && alt() < 0.2f) crashed = true;
    if (fmodf(t, 10.0f) < 0.0005f) trace(t);
  });
  opt.vibration = false;
  for (int i = 0; i < 4; i++) quad.eff[i] = 1.0f;
  check("Tune flight stayed airborne", !crashed, "%.0f", !crashed);
  // 30 s x 500 Hz = 15000 records: ~1.4 MB as full records, ~0.3 MB with delta frames
  check("500 Hz blackbox recorded (delta frames)", hal::host::log.size() > 150000 && hal::host::log.size() < 700000, "%.0f bytes",
        (double)hal::host::log.size());
  if (const char* path = getenv("SIM_BLACKBOX")) {
    FILE* f = fopen(path, "wb");
    if (f) { fwrite(hal::host::log.data(), 1, hal::host::log.size(), f); fclose(f); }
  }
}

void scenarioWind() {
  printf("\n== wind: POSHOLD in a 3 m/s wind with gusts, wind estimate + feed-forward ==\n");
  bool comp = !getenv("SIM_WIND_COMP") || atoi(getenv("SIM_WIND_COMP"));
  setup([&](Config& c) { c.wind_comp = comp; });
  float hn = 0, he = 0, maxDev = 0, sumDev = 0, wErr = 0;
  int n = 0;
  run(50, [&](float t) {
    if (t >= 3.0f) rcIn.ch[4] = 2000;
    if (t >= 3.5f && t < 6.0f) rcIn.ch[RC_THROTTLE] = 1560;
    if (t >= 6.0f) { rcIn.ch[5] = 2000; rcIn.ch[RC_THROTTLE] = 1500; rcIn.ch[6] = 1500; }
    // steady 3 m/s from the south-west (force = k * m * wind) plus 4 s gusts of +50 %
    float k = 0.45f * quad.mass, ramp = constrainf((t - 14.0f) / 2.0f, 0.0f, 1.0f);
    float gust = (fmodf(t, 10.0f) < 4.0f && t > 24) ? 1.5f : 1.0f;
    quad.wind = Vec3(2.12f, 2.12f, 0) * (k * ramp * gust);
  }, [&](float t) {
    if (fabsf(t - 12.0f) < 0.0005f) { hn = quad.pos.x; he = quad.pos.y; }
    if (t > 12) maxDev = fmaxf(maxDev, hypotf(quad.pos.x - hn, quad.pos.y - he));
    if (t > 20) { sumDev += hypotf(quad.pos.x - hn, quad.pos.y - he); n++; }
    if (t > 20 && t < 24) {
      NavEstimate e = bus::nav.get();
      wErr = fmaxf(wErr, hypotf(e.windN - 2.12f, e.windE - 2.12f));
    }
    if (fmodf(t, 10.0f) < 0.0005f) trace(t);
  });
  printf("  wind compensation %s: max drift %.2f m, mean %.2f m\n", comp ? "on" : "off", maxDev, sumDev / n);
  check("Wind estimate (3 m/s SW)", wErr < 0.6f, "max error %.2f m/s", wErr);
  check("POSHOLD drift in wind + gusts", maxDev < 1.5f, "max drift %.2f m", maxDev);
}

void scenarioGlitch() {
  printf("\n== glitch: POSHOLD, 30 m GPS jump for 6 s (rejected), then a permanent 15 m shift (accepted after 10 s) ==\n");
  setup([](Config&) {});
  float hn = 0, he = 0, devGlitch = 0, offAfter = 0;
  bool flagged = false;
  run(60, [&](float t) {
    if (t >= 3.0f) rcIn.ch[4] = 2000;
    if (t >= 3.5f && t < 6.0f) rcIn.ch[RC_THROTTLE] = 1560;
    if (t >= 6.0f) { rcIn.ch[5] = 2000; rcIn.ch[RC_THROTTLE] = 1500; rcIn.ch[6] = 1500; }
    opt.gpsOffset = (t >= 20 && t < 26) ? Vec3(30, 0, 0) : (t >= 32 ? Vec3(15, 0, 0) : Vec3());
  }, [&](float t) {
    if (fabsf(t - 15.0f) < 0.0005f) { hn = quad.pos.x; he = quad.pos.y; }
    if (t > 20 && t < 31) devGlitch = fmaxf(devGlitch, hypotf(quad.pos.x - hn, quad.pos.y - he));
    if (t > 20 && t < 26 && (status().warnings & WARN_GPS_GLITCH)) flagged = true;
    if (t > 55) offAfter = bus::nav.get().n - quad.pos.x;
    if (fmodf(t, 10.0f) < 0.0005f) trace(t);
  });
  opt.gpsOffset = Vec3();
  check("GPS glitch detected & reported", flagged, "%.0f", flagged);
  check("Hold kept through a 30 m glitch", devGlitch < 2.0f, "max drift %.2f m", devGlitch);
  check("Persistent shift accepted after 10 s", fabsf(offAfter - 15.0f) < 2.0f, "estimate offset %.1f m", offAfter);
}

void scenarioHover() {
  printf("\n== hover: hover_throttle 0.30 (true ~0.50) learned in ALTHOLD and saved on disarm ==\n");
  setup([](Config& c) { c.hover_throttle = 0.30f; c.hover_learn = 2; });
  Config stored = bus::config;
  configSeal(stored);
  hal::storageSave(hal::Slot::CONFIG, &stored, sizeof(stored));
  double thrSum = 0;
  int thrN = 0;
  float disarmT = -1, dipFirst = 0, dipSecond = 0, a0 = 0;
  run(60, [&](float t) {
    if (t >= 3.0f) rcIn.ch[4] = 2000;
    if (t >= 3.5f && t < 6.0f) rcIn.ch[RC_THROTTLE] = 1600;
    // ANGLE -> ALTHOLD at 6 s and again at 20 s (after learning)
    if (t >= 6.0f) { rcIn.ch[RC_THROTTLE] = 1500; rcIn.ch[5] = (t >= 17 && t < 20) ? 1500 : 2000; }
    if (t >= 17.0f && t < 20.0f) rcIn.ch[RC_THROTTLE] = 1500;  // ANGLE, mid stick (does not hover)
    if (t >= 30.0f) rcIn.ch[RC_THROTTLE] = 1000;               // descend, land, auto-disarm
  }, [&](float t) {
    if (t > 12 && t < 17) {
      ControlSetpoint sp = bus::setpoint.get();
      thrSum += sp.throttle * quad.q.cosTilt();
      thrN++;
    }
    if (fabsf(t - 6.0f) < 0.0005f || fabsf(t - 20.0f) < 0.0005f) a0 = alt();
    if (t > 6 && t < 9) dipFirst = fmaxf(dipFirst, fabsf(alt() - a0));
    if (t > 20 && t < 23) dipSecond = fmaxf(dipSecond, fabsf(alt() - a0));
    if (t > 30 && !status().armed && disarmT < 0) disarmT = t;
    if (fmodf(t, 10.0f) < 0.0005f) trace(t);
  });
  float hover = (float)(thrSum / thrN);
  Config saved{};
  hal::storageLoad(hal::Slot::CONFIG, &saved, sizeof(saved));
  printf("  measured hover %.3f, learned %.3f, stored %.3f; ALTHOLD entry error %.2f m -> %.2f m\n", hover,
         bus::config.hover_throttle, saved.hover_throttle, dipFirst, dipSecond);
  check("Hover throttle learned", fabsf(bus::config.hover_throttle - hover) < 0.03f, "learned %.3f",
        bus::config.hover_throttle);
  check("Learned value saved on disarm", disarmT > 0 && configValid(saved) && fabsf(saved.hover_throttle - hover) < 0.03f,
        "stored %.3f", saved.hover_throttle);
  check("Smoother ALTHOLD entry after learning", dipSecond < dipFirst, "%.2f m", dipSecond);
}

void scenarioNoMag() {
  printf("\n== nomag: no compass, yaw 150 deg off; RC loss -> RTH learns heading from GPS (RTH_ALIGN) ==\n");
  opt.mag = false;
  setup([](Config& c) { c.gps_heading = 0; c.rth_alt = 10; }, Quat::fromEuler(0, 0, 150 * DEG2RAD));
  bool aligned = false, sawAlign = false;
  float disarmT = -1, maxD = 0, yawErr = 180;
  run(120, [&](float t) {
    if (t >= 3.0f) rcIn.ch[4] = 2000;
    if (t >= 3.5f && t < 6.0f) rcIn.ch[RC_THROTTLE] = 1580;
    if (t >= 6.0f) { rcIn.ch[5] = 2000; rcIn.ch[RC_THROTTLE] = 1500; }
    rcIn.ch[RC_PITCH] = (t >= 8 && t < 14) ? 1700 : 1500;  // fly away ~20 m in ALTHOLD
    if (t >= 18.0f && rcAlive) {                       // GPS heading enabled only now: forces RTH_ALIGN
      rcAlive = false;
      bus::config.gps_heading = 1;
      bus::configChanged();
    }
  }, [&](float t) {
    VehicleStatus s = status();
    NavEstimate e = bus::nav.get();
    maxD = fmaxf(maxD, distHome());
    if (s.navPhase == NavPhase::RTH_ALIGN) sawAlign = true;
    if (t > 18 && e.headingOk && !aligned) {
      aligned = true;
      yawErr = fabsf(wrapPi(e.yaw - quad.q.yaw())) * RAD2DEG;
      printf("  heading learned at t=%.1f s, error %.1f deg\n", t, yawErr);
    }
    if (t > 18 && !s.armed && disarmT < 0) disarmT = t;
    if (getenv("SIM_TRACE_YAW") && t > 18 && fmodf(t, 1.0f) < 0.0005f)
      { EkfReport k = bus::ekf.get();
      printf("    yaw t=%.0f err=%.1f ok=%d phase=%s d=%.1f v=%.1f,%.1f ekf yaw err %.1f roll err %.1f pitch err %.1f std %.1f tr %.2f ab %.2f %.2f %.2f\n", t, wrapPi(e.yaw - quad.q.yaw()) * RAD2DEG, e.headingOk,
             navPhaseName(s.navPhase), distHome(), quad.vel.x, quad.vel.y, wrapPi(k.yawDeg * DEG2RAD - quad.q.yaw()) * RAD2DEG,
             k.rollDeg - quad.q.roll() * RAD2DEG, k.pitchDeg - quad.q.pitch() * RAD2DEG, k.yawStdDeg, k.trYaw, k.accBias[0], k.accBias[1], k.accBias[2]); }
    if (fmodf(t, 10.0f) < 0.0005f) trace(t);
  });
  opt.mag = true;
  check("RTH started heading alignment", sawAlign, "%.0f", sawAlign);
  check("GPS heading learned", aligned && yawErr < 15.0f, "error %.1f deg", yawErr);
  check("Returned home without compass", disarmT > 0 && distHome() < 3.0f, "distance %.2f m", distHome());
  check("Flew away before failsafe", maxD > 12.0f, "max %.1f m", maxD);
}

float rampTerrain(float n, float) { return constrainf((n - 8.0f) / 20.0f, 0.0f, 1.0f) * 3.0f; }

void scenarioTerrain() {
  printf("\n== terrain: rangefinder surface tracking in POSHOLD over a 3 m slope ==\n");
  opt.range = true;
  opt.terrain = rampTerrain;
  setup([](Config& c) { c.alt_surface = 1; });
  float agl0 = 0, aglErr = 0, maxN = 0, altGain = 0;
  run(45, [&](float t) {
    if (t >= 3.0f) rcIn.ch[4] = 2000;
    if (t >= 3.5f && t < 5.5f) rcIn.ch[RC_THROTTLE] = 1580;
    if (t >= 5.5f) { rcIn.ch[5] = 2000; rcIn.ch[RC_THROTTLE] = 1500; rcIn.ch[6] = 1500; }
    rcIn.ch[RC_PITCH] = (t >= 12 && t < 35) ? 1600 : 1500;  // creep north over the slope
  }, [&](float t) {
    float agl = alt() - rampTerrain(quad.pos.x, quad.pos.y);
    if (fabsf(t - 11.0f) < 0.0005f) agl0 = agl;
    if (t > 12) aglErr = fmaxf(aglErr, fabsf(agl - agl0));
    maxN = fmaxf(maxN, quad.pos.x);
    altGain = alt() - agl0;
    if (fmodf(t, 10.0f) < 0.0005f) trace(t);
  });
  opt.range = false;
  opt.terrain = nullptr;
  printf("  flew %.1f m north, climbed %.2f m with the ground\n", maxN, altGain);
  check("Crossed the slope", maxN > 30.0f && altGain > 2.5f, "climbed %.2f m", altGain);
  check("Height above ground held", aglErr < 0.6f, "max error %.2f m", aglErr);
}

void scenarioCruise() {
  printf("\n== cruise: course & speed hold with cross wind, yaw stick turns ==\n");
  setup([](Config& c) {
    c.mode_range[9] = {(uint8_t)ModeBox::CRUISE, 10, 1700, 2100};  // AUX7 high
    c.pos_max_speed = 8;
  }, Quat::fromEuler(0, 0, 0));
  float xtrack = 0, minSpeed = 99, course1 = 0, course2 = 0;
  float startE = 0;
  bool cruising = false;
  run(50, [&](float t) {
    if (t >= 3.0f) rcIn.ch[4] = 2000;
    if (t >= 3.5f && t < 6.0f) rcIn.ch[RC_THROTTLE] = 1580;
    if (t >= 6.0f) { rcIn.ch[5] = 2000; rcIn.ch[RC_THROTTLE] = 1500; }
    if (t >= 9.0f) rcIn.ch[10] = 2000;                              // CRUISE
    rcIn.ch[RC_PITCH] = (t >= 10 && t < 12) ? 2000 : 1500;          // accelerate, then let go
    rcIn.ch[RC_YAW] = (t >= 30 && t < 31) ? 2000 : 1500;            // 90 deg/s for 1 s
    quad.wind = (t > 14 && t < 28) ? Vec3(0, 0.4f, 0) : Vec3();     // cross wind from the west
  }, [&](float t) {
    if (status().mode == FlightMode::CRUISE) cruising = true;
    if (fabsf(t - 16.0f) < 0.0005f) startE = quad.pos.y;
    float sp = hypotf(quad.vel.x, quad.vel.y);
    if (t > 16 && t < 29) {
      xtrack = fmaxf(xtrack, fabsf(quad.pos.y - startE));
      minSpeed = fminf(minSpeed, sp);
    }
    if (fabsf(t - 28.0f) < 0.0005f) course1 = atan2f(quad.vel.y, quad.vel.x) * RAD2DEG;
    if (fabsf(t - 40.0f) < 0.0005f) course2 = atan2f(quad.vel.y, quad.vel.x) * RAD2DEG;
    if (fmodf(t, 10.0f) < 0.0005f) trace(t);
  });
  float turn = wrapPi((course2 - course1) * DEG2RAD) * RAD2DEG;
  printf("  speed >= %.1f m/s, cross-track %.2f m, turn %.0f deg\n", minSpeed, xtrack, turn);
  check("CRUISE engaged", cruising, "%.0f", cruising);
  check("Speed held after releasing the stick", minSpeed > 3.0f, "min %.1f m/s", minSpeed);
  check("Track held in cross wind", xtrack < 1.5f, "max %.2f m", xtrack);
  check("Yaw stick turns the course", fabsf(turn - 90.0f) < 20.0f, "%.0f deg", turn);
}

void scenarioMission2() {
  printf("\n== mission2: ROI, orbit (loiter turns), DO_JUMP, condition yaw, RTL to the nearest rally point ==\n");
  setup([](Config& c) { c.rth_alt = 10; c.rally_max_m = 200; if (getenv("SIM_WIND_COMP")) c.wind_comp = atoi(getenv("SIM_WIND_COMP")); });
  auto item = [](uint16_t cmd, float n, float e, float alt, float p1 = 0, float p2 = 0, float p3 = 0, float p4 = 0) {
    nav::MissionItem it{};
    it.cmd = cmd; it.frame = 3; it.alt = alt;
    it.param1 = p1; it.param2 = p2; it.param3 = p3; it.param4 = p4;
    if (n != 0 || e != 0) {
      double lat, lon;
      NavEstimator::nedToLla(n, e, LAT0, LON0, lat, lon);
      it.lat = (int32_t)lrint(lat * 1e7); it.lon = (int32_t)lrint(lon * 1e7);
    }
    return it;
  };
  int i = 0;
  nav::set(i++, item(nav::CMD_TAKEOFF, 0, 0, 10));                         // 0
  nav::set(i++, item(nav::CMD_DO_SET_ROI_LOC, 0, 40, 0));                  // 1 look at (0, 40)
  nav::set(i++, item(nav::CMD_WAYPOINT, 30, 0, 10));                       // 2
  nav::set(i++, item(nav::CMD_DO_SET_ROI_NONE, 0, 0, 0));                  // 3
  nav::set(i++, item(nav::CMD_LOITER_TURNS, 30, 30, 10, 1, 0, 10));        // 4 one turn, r 10 m, clockwise
  nav::set(i++, item(nav::CMD_DO_JUMP, 0, 0, 0, 2, 1));                    // 5 back to item 2 once
  nav::set(i++, item(nav::CMD_CONDITION_YAW, 0, 0, 0, 90));                // 6 face east
  nav::set(i++, item(nav::CMD_WAYPOINT, 50, 50, 12));                      // 7
  nav::set(i++, item(nav::CMD_RTL, 0, 0, 0));                              // 8
  nav::setCount(i);
  nav::rallySet(0, item(nav::CMD_RALLY_POINT, 60, 40, 8));
  nav::rallySetCount(1);
  int wp2Visits = 0;
  bool near2 = false, sawOrbit = false;
  float orbitStart = 0;
  bool inOrbit = false;
  float roiErr = 0, orbitErr = 0, faceCenterErr = 0, yawEastErr = 180, disarmT = -1;
  run(240, [&](float t) {
    if (t >= 2.5f) rcIn.ch[7] = 1500;  // MISSION
    if (t >= 3.0f) rcIn.ch[4] = 2000;
  }, [&](float t) {
    VehicleStatus s = status();
    float dn = quad.pos.x - 30, de = quad.pos.y;
    bool at2 = hypotf(dn, de) < 2.5f;
    if (at2 && !near2) wp2Visits++;
    near2 = at2;
    float yaw = quad.q.yaw();
    // first leg (to item 2): the nose points at the ROI (0, 40)
    if (s.missionIndex == 2 && wp2Visits == 0 && distHome() > 8)
      roiErr = fmaxf(roiErr, fabsf(wrapPi(atan2f(40 - quad.pos.y, -quad.pos.x) - yaw)) * RAD2DEG);
    if (s.navPhase == NavPhase::WP_ORBIT) {
      if (!inOrbit) orbitStart = t;  // each orbit entry
      sawOrbit = inOrbit = true;
    } else {
      inOrbit = false;
    }
    if (s.navPhase == NavPhase::WP_ORBIT && t > orbitStart + 2.0f) {  // after the entry transient
      float r = hypotf(quad.pos.x - 30, quad.pos.y - 30);
      if (getenv("SIM_TRACE_ORBIT") && fmodf(t, 0.5f) < 0.0005f) {
        NavEstimate ne = bus::nav.get();
        printf("    orbit t=%.1f r=%.2f tgt r=%.2f est r=%.2f v=%.1f wind=%.2f,%.2f roll=%.1f pitch=%.1f\n", t, r, hypotf(ne.tgtN - 30, ne.tgtE - 30), hypotf(ne.n - 30, ne.e - 30), hypotf(quad.vel.x, quad.vel.y), ne.windN, ne.windE, quad.q.roll()*RAD2DEG, quad.q.pitch()*RAD2DEG);
      }
      orbitErr = fmaxf(orbitErr, fabsf(r - 10));
      faceCenterErr = fmaxf(faceCenterErr, fabsf(wrapPi(atan2f(30 - quad.pos.y, 30 - quad.pos.x) - yaw)) * RAD2DEG);
    }
    if (s.missionIndex == 7 && hypotf(quad.pos.x - 50, quad.pos.y - 50) < 3)
      yawEastErr = fabsf(wrapPi(PI_F / 2 - yaw)) * RAD2DEG;
    if (t > 5 && !s.armed && disarmT < 0) disarmT = t;
    if (fmodf(t, 20.0f) < 0.0005f) trace(t);
  });
  float dRally = hypotf(quad.pos.x - 60, quad.pos.y - 40);
  check("ROI: nose on the region of interest", roiErr < 20.0f, "max error %.0f deg", roiErr);
  check("Orbit radius held (r = 10 m)", sawOrbit && orbitErr < 1.0f, "max error %.2f m", orbitErr);
  check("Orbit faces the centre", sawOrbit && faceCenterErr < 25.0f, "max error %.0f deg", faceCenterErr);
  check("DO_JUMP repeated the leg once", wp2Visits == 2, "visits %.0f", wp2Visits);
  check("CONDITION_YAW 90 deg", yawEastErr < 15.0f, "error %.0f deg", yawEastErr);
  check("RTL landed at the nearest rally point", disarmT > 0 && dRally < 3.0f, "distance %.2f m", dRally);
}

void scenarioAutotune() {
  float pScale = getenv("SIM_AT_P") ? (float)atof(getenv("SIM_AT_P")) : 1.0f;
  float dScale = getenv("SIM_AT_D") ? (float)atof(getenv("SIM_AT_D")) : 1.0f;
  printf("\n== autotune: in-flight twitch autotune of roll + pitch (start P x%.2f, D x%.2f), save on disarm ==\n",
         pScale, dScale);
  opt.vibration = true;
  AxisPid start[3];
  setup([&](Config& c) {
    c.esc_telemetry = 1;
    c.mode_range[9] = {(uint8_t)ModeBox::AUTOTUNE, 10, 1700, 2100};  // AUX7 high
    for (int a = 0; a < 2; a++) { c.pid[a].p *= pScale; c.pid[a].d *= dScale; }
    for (int a = 0; a < 3; a++) start[a] = c.pid[a];
    if (getenv("SIM_AT_RATE")) c.at_rate = (float)atof(getenv("SIM_AT_RATE"));
  });
  Config stored = bus::config;
  configSeal(stored);
  hal::storageSave(hal::Slot::CONFIG, &stored, sizeof(stored));
  float doneT = -1, a0 = 0, altDev = 0, disarmT = -1, minAlt = 99, finalOv = -1;
  uint8_t lastAxis = 0, lastRound = 255;
  run(320, [&](float t) {
    if (t >= 3.0f) rcIn.ch[4] = 2000;
    if (t >= 3.5f && t < 6.0f) rcIn.ch[RC_THROTTLE] = 1600;
    if (t >= 6.0f) { rcIn.ch[5] = 2000; rcIn.ch[RC_THROTTLE] = 1500; }
    if (t >= 10.0f) rcIn.ch[10] = 2000;                                     // AUTOTUNE on
    if (doneT > 0 && t > doneT + 3) rcIn.ch[RC_THROTTLE] = 1000;            // land (switch stays on)
  }, [&](float t) {
    VehicleStatus s = status();
    if (fabsf(t - 10.0f) < 0.0005f) a0 = alt();
    if (t > 10 && doneT < 0) { altDev = fmaxf(altDev, fabsf(alt() - a0)); minAlt = fminf(minAlt, alt()); }
    if (s.atState == 2 && doneT < 0) doneT = t;
    if (s.atState >= 1 && s.atRound) finalOv = s.atOvershoot;
    if (s.atState == 1 && (s.atAxis != lastAxis || s.atRound != lastRound)) {
      if (s.atRound && s.atStep == 0)
        printf("    t=%5.1f axis %s P/D round %2u: overshoot %4.1f%% rise %3.0f ms -> P %.1f D %.1f\n", t,
               s.atAxis ? "pitch" : "roll ", s.atRound, s.atOvershoot * 100, s.atRiseMs, bus::config.pid[s.atAxis].p,
               bus::config.pid[s.atAxis].d);
      else if (s.atRound)
        printf("    t=%5.1f axis %s %s round %2u: lag %4.1f ms steady error %5.1f%% -> F %.0f I %.0f\n", t,
               s.atAxis ? "pitch" : "roll ", s.atStep == 1 ? "FF" : "I ", s.atRound, s.atLagMs, s.atSteadyErr * 100,
               bus::config.pid[s.atAxis].f, bus::config.pid[s.atAxis].i);
      lastAxis = s.atAxis;
      lastRound = s.atRound;
    }
    if (doneT > 0 && !s.armed && disarmT < 0) disarmT = t;
    if (fmodf(t, 20.0f) < 0.0005f) trace(t);
  });
  opt.vibration = false;
  Config saved{};
  hal::storageLoad(hal::Slot::CONFIG, &saved, sizeof(saved));
  { char m[64]; bus::readMessage(m, sizeof(m)); printf("  last message: %s\n", m); }
  printf("  feed-forward roll %.0f -> %.0f, pitch %.0f -> %.0f; I roll %.0f -> %.0f, pitch %.0f -> %.0f; gyro lpf %.0f dterm %.0f notch %.0f-%.0f Hz\n",
         start[0].f, saved.pid[0].f, start[1].f, saved.pid[1].f, start[0].i, saved.pid[0].i, start[1].i, saved.pid[1].i,
         saved.gyro_lpf_hz, saved.dterm_lpf_hz, saved.dyn_notch_min, saved.dyn_notch_max);
  printf("  start roll P/D %.1f/%.1f pitch %.1f/%.1f -> tuned roll %.1f/%.1f pitch %.1f/%.1f (saved roll P %.1f)\n",
         start[0].p, start[0].d, start[1].p, start[1].d, bus::config.pid[0].p, bus::config.pid[0].d,
         bus::config.pid[1].p, bus::config.pid[1].d, saved.pid[0].p);
  bool changed = fabsf(bus::config.pid[0].p - start[0].p) > 0.5f || fabsf(bus::config.pid[0].d - start[0].d) > 0.5f;
  check("Autotune finished both axes", doneT > 0 && doneT < 260, "after %.0f s", doneT - 10);
  check("Final response near the 4-12 % band", finalOv >= 0.03f && finalOv <= 0.15f, "%.3f", finalOv);
  check("Altitude held while twitching", altDev < 1.5f && minAlt > 0.8f, "max deviation %.2f m", altDev);
  check("Gains saved on disarm (switch on)", disarmT > 0 && changed && fabsf(saved.pid[0].p - bus::config.pid[0].p) < 1e-3f &&
        fabsf(saved.pid[1].d - bus::config.pid[1].d) < 1e-3f, "roll P %.1f", saved.pid[0].p);
  if (const char* path = getenv("SIM_AT_OUT")) {  // tuned gains for the follow-up check
    FILE* f = fopen(path, "w");
    if (f) { fprintf(f, "%f %f %f %f\n", bus::config.pid[0].p, bus::config.pid[0].d, bus::config.pid[1].p, bus::config.pid[1].d); fclose(f); }
  }
}

// motor test from the configurator: one motor spins, arming is refused meanwhile, the test stops by
// itself one second after the last command
void scenarioMotorTest() {
  setup([](Config&) {});
  float m2 = -1, others = 1, after = 1;
  bool armedDuring = false, outArmed = false, afterArmed = true;
  run(5.0f, [](float t) {
    static float lastCmd = -1;
    if (t >= 2.0f && t < 3.5f && t - lastCmd >= 0.5f) {
      lastCmd = t;
      float v[4] = {0, 0.3f, 0, 0};
      bus::setMotorTest(v, os::timeMs());
    }
    if (t >= 2.5f) rcIn.ch[4] = 2000;  // pilot flips the arm switch during the test
  }, [&](float t) {
    if (t > 3.0f && t < 3.01f) {
      m2 = hal::host::motors[1];
      others = fmaxf(fmaxf(hal::host::motors[0], hal::host::motors[2]), hal::host::motors[3]);
      outArmed = hal::host::motorsArmed;
    }
    if (status().armed && t < 3.6f) armedDuring = true;
    if (t > 4.9f && t < 4.91f) {
      after = fmaxf(fmaxf(hal::host::motors[0], hal::host::motors[1]), fmaxf(hal::host::motors[2], hal::host::motors[3]));
      afterArmed = hal::host::motorsArmed;
    }
  });
  check("Tested motor spins, others stopped", fabsf(m2 - 0.3f) < 1e-3f && others <= 0 && outArmed, "M2 %.2f", m2);
  check("Arming refused during the test", !armedDuring && !status().armed, "%.0f", armedDuring ? 1.0 : 0.0);
  check("Test stops by itself (1 s timeout)", after <= 0 && !afterArmed, "%.2f", after);
}

void scenarioMotorMap() {
  printf("\n== motormap: motor1..4_out reorder the outputs (motor 2 -> output 1) ==\n");
  setup([](Config& c) { c.motor_out[0] = 3; c.motor_out[1] = 1; c.motor_out[2] = 4; c.motor_out[3] = 2; });
  float out1 = -1, rest = 1;
  run(3.0f, [](float t) {
    static float lastCmd = -1;
    if (t >= 1.5f && t - lastCmd >= 0.4f) { lastCmd = t; float v[4] = {0, 0.3f, 0, 0}; bus::setMotorTest(v, os::timeMs()); }
  }, [&](float t) {
    if (t > 2.5f && t < 2.51f) {
      out1 = hal::host::motors[0];
      rest = fmaxf(fmaxf(hal::host::motors[1], hal::host::motors[2]), hal::host::motors[3]);
    }
  });
  check("Motor 2 drives output 1 after reordering", fabsf(out1 - 0.3f) < 1e-3f && rest <= 0, "out1 %.2f", out1);
}

void scenarioAutotuneAbort() {
  printf("\n== autotune_abort: switch off mid-tune and stick input -> old gains back, nothing saved ==\n");
  AxisPid start[3];
  setup([&](Config& c) {
    c.mode_range[9] = {(uint8_t)ModeBox::AUTOTUNE, 10, 1700, 2100};
    c.pid[0].d *= 0.4f;  // so that the tune changes something early
    for (int a = 0; a < 3; a++) start[a] = c.pid[a];
  });
  Config stored = bus::config;
  configSeal(stored);
  hal::storageSave(hal::Slot::CONFIG, &stored, sizeof(stored));
  bool changedMid = false, pausedOk = true;
  float maxRollDuringStick = 0;
  run(60, [&](float t) {
    if (t >= 3.0f) rcIn.ch[4] = 2000;
    if (t >= 3.5f && t < 6.0f) rcIn.ch[RC_THROTTLE] = 1600;
    if (t >= 6.0f) { rcIn.ch[5] = 2000; rcIn.ch[RC_THROTTLE] = 1500; }
    rcIn.ch[10] = (t >= 10.0f && t < 35.0f) ? 2000 : 1000;
    rcIn.ch[RC_PITCH] = (t >= 28 && t < 31) ? 1600 : 1500;  // pilot input: twitching pauses
    if (t >= 45.0f) rcIn.ch[RC_THROTTLE] = 1000;
  }, [&](float t) {
    if (t > 20 && t < 35 && fabsf(bus::config.pid[0].d - start[0].d) > 0.5f) changedMid = true;
    if (t > 28.6f && t < 31) maxRollDuringStick = fmaxf(maxRollDuringStick, fabsf(quad.w.x) * RAD2DEG);
  });
  if (maxRollDuringStick > 60) pausedOk = false;
  Config saved{};
  hal::storageLoad(hal::Slot::CONFIG, &saved, sizeof(saved));
  check("Gains changed while tuning", changedMid, "%.0f", changedMid);
  check("Stick input pauses the twitches", pausedOk, "max roll rate %.0f deg/s", maxRollDuringStick);
  check("Switch off mid-tune restores the old gains", fabsf(bus::config.pid[0].d - start[0].d) < 1e-4f &&
        fabsf(bus::config.pid[0].p - start[0].p) < 1e-4f, "roll D %.1f", bus::config.pid[0].d);
  check("Nothing saved", fabsf(saved.pid[0].d - start[0].d) < 1e-4f, "stored roll D %.1f", saved.pid[0].d);
}


void scenarioBidir() {
  printf("\n== bidir: bidirectional DShot RPM (no telemetry wire) feeds the RPM filter ==\n");
  opt.vibration = true;
  setup([](Config& c) { c.esc_telemetry = 0; c.dshot_bidir = 1; });
  double rawSq = 0, filtSq = 0;
  int n = 0;
  float rpmErr = 0;
  run(20, [&](float t) {
    if (t >= 3.0f) rcIn.ch[4] = 2000;
    if (t >= 3.5f && t < 6.0f) rcIn.ch[RC_THROTTLE] = 1560;
    if (t >= 6.0f) { rcIn.ch[5] = 2000; rcIn.ch[RC_THROTTLE] = 1500; }
  }, [&](float t) {
    if (t > 10 && fmodf(t, 0.004f) < 0.0005f) {
      AttitudeState a = bus::attitude.get();
      PidDebug p = bus::pid.get();
      Vec3 truth = quad.w * RAD2DEG;
      Vec3 er = p.gyroRaw - truth, ef = a.rateDps - truth;
      rawSq += er.dot(er);
      filtSq += ef.dot(ef);
      n++;
      rpmErr = fmaxf(rpmErr, fabsf(bus::bidirRpm[0] - quad.motorHz(0) * 60.0f) / (quad.motorHz(0) * 60.0f));
    }
  });
  opt.vibration = false;
  float rawRms = (float)sqrt(rawSq / n), filtRms = (float)sqrt(filtSq / n);
  printf("  gyro noise RMS: raw %.1f deg/s -> filtered %.2f deg/s\n", rawRms, filtRms);
  check("Motor RPM from the ESC answers", rpmErr < 0.08f, "max error %.3f", rpmErr);
  check("RPM filter with bidirectional DShot > 10x", filtRms < rawRms / 10.0f, "filtered/raw = %.3f", filtRms / rawRms);
}

void scenarioRpmLoss() {
  printf("\n== rpmloss: RPM decode glitches, then one ESC stops answering in flight ==\n");
  opt.vibration = true;
  opt.bidirGlitchEvery = 37;
  setup([](Config& c) { c.esc_telemetry = 0; c.dshot_bidir = 1; });
  double rawSq = 0, filtSq = 0;
  int n = 0;
  bool warnEarly = false, warnLost = false, warnAfter = false;
  uint8_t mask = 0;
  float maxTilt = 0;
  run(26, [&](float t) {
    if (t >= 3.0f) rcIn.ch[4] = 2000;
    if (t >= 3.5f && t < 6.0f) rcIn.ch[RC_THROTTLE] = 1560;
    if (t >= 6.0f && t < 22.0f) { rcIn.ch[5] = 2000; rcIn.ch[RC_THROTTLE] = 1500; }
    opt.bidirDead = t >= 14.0f ? 2 : -1;
    if (t >= 22.0f) { rcIn.ch[RC_THROTTLE] = 1000; rcIn.ch[4] = 1000; }
  }, [&](float t) {
    VehicleStatus s = status();
    if (t > 8 && t < 13.9f && (s.warnings & WARN_RPM_LOST)) warnEarly = true;
    if (t > 14.5f && t < 21 && (s.warnings & WARN_RPM_LOST)) { warnLost = true; mask = bus::rpmLostMask; }
    if (t > 25 && (s.warnings & WARN_RPM_LOST)) warnAfter = true;
    if (t > 14.5f && t < 21) maxTilt = fmaxf(maxTilt, fmaxf(fabsf(quad.q.roll()), fabsf(quad.q.pitch())) * RAD2DEG);
    if (t > 8 && t < 13.5f && fmodf(t, 0.004f) < 0.0005f) {
      AttitudeState a = bus::attitude.get();
      PidDebug p = bus::pid.get();
      Vec3 truth = quad.w * RAD2DEG;
      Vec3 er = p.gyroRaw - truth, ef = a.rateDps - truth;
      rawSq += er.dot(er);
      filtSq += ef.dot(ef);
      n++;
    }
  });
  opt.vibration = false;
  opt.bidirGlitchEvery = 0;
  opt.bidirDead = -1;
  float rawRms = (float)sqrt(rawSq / n), filtRms = (float)sqrt(filtSq / n);
  check("RPM filter > 10x with decode glitches", filtRms < rawRms / 10.0f, "filtered/raw = %.3f", filtRms / rawRms);
  check("No RPM warning while all ESCs answer", !warnEarly, "%.0f", warnEarly);
  check("RPM TELEMETRY LOST for motor 3 only", warnLost && mask == 4, "mask %.0f", mask);
  check("Still flies level without motor 3's notches", maxTilt < 10.0f, "max tilt %.1f deg", maxTilt);
  check("Warning gone after disarm", !warnAfter, "%.0f", warnAfter);
}

void scenarioConsole() {
  printf("\n== console: configurator UART (WiFi bridge) and USB share the console ==\n");
  setup([](Config& c) { c.port_fn[0] = (uint8_t)PortFn::CONFIGURATOR; });
  auto send = [](hal::Port p, const char* s) { for (const char* q = s; *q; q++) hal::host::in(p).push_back((uint8_t)*q); };
  std::string& uart = hal::host::out(hal::Port::SERIAL1);
  std::string& usb = hal::host::out(hal::Port::CONSOLE);
  bool bridgeAnswer = false, usbIgnored = false, usbTakesOver = false, bridgeIgnored = false;
  run(6, [&](float t) {
    if (fabsf(t - 1.0f) < 0.0005f) { uart.clear(); usb.clear(); send(hal::Port::SERIAL1, "version\n"); }
    if (fabsf(t - 1.5f) < 0.0005f) {
      bridgeAnswer = uart.find("AllQuad") != std::string::npos && usb.find("AllQuad") == std::string::npos;
      usb.clear();
      send(hal::Port::CONSOLE, "version\n");  // 0.5 s after the bridge spoke: not served
    }
    if (fabsf(t - 2.0f) < 0.0005f) usbIgnored = usb.find("AllQuad") == std::string::npos;
    if (fabsf(t - 3.5f) < 0.0005f) { uart.clear(); send(hal::Port::CONSOLE, "version\n"); }
    if (fabsf(t - 4.0f) < 0.0005f) {
      usbTakesOver = usb.find("AllQuad") != std::string::npos && uart.find("AllQuad") == std::string::npos;
      uart.clear();
      send(hal::Port::SERIAL1, "version\n");  // USB just spoke: the bridge waits
    }
    if (fabsf(t - 4.5f) < 0.0005f) bridgeIgnored = uart.find("AllQuad") == std::string::npos;
  }, [](float) {});
  check("Bridge UART answered on the bridge only", bridgeAnswer, "%.0f", bridgeAnswer);
  check("USB not served while the bridge is in use", usbIgnored, "%.0f", usbIgnored);
  check("USB takes over after 1.5 s of bridge silence", usbTakesOver, "%.0f", usbTakesOver);
  check("Bridge waits while USB is in use", bridgeIgnored, "%.0f", bridgeIgnored);
}

void scenarioEdt() {
  printf("\n== edt: AM32 extended DShot telemetry, no battery divider / current sensor on the FC ==\n");
  opt.adc = false;
  opt.edtEsc = true;
  edtOn = false;
  setup([](Config& c) { c.esc_telemetry = 0; c.dshot_bidir = 1; c.motor_protocol = (uint8_t)MotorProtocol::DSHOT600; c.cur_scale = 0; });
  float vHover = 0, aHover = 0, mah = 0, temp = 0;
  bool cmd13 = false;
  run(25, [&](float t) {
    if (t >= 8.0f) rcIn.ch[4] = 2000;  // arm after the ESCs were asked (3 s, then every 2 s)
    if (t >= 8.5f && t < 11.0f) rcIn.ch[RC_THROTTLE] = 1560;
    if (t >= 11.0f) { rcIn.ch[5] = 2000; rcIn.ch[RC_THROTTLE] = 1500; }
  }, [&](float t) {
    if (hal::host::lastMotorCommand == 13) cmd13 = true;
    if (t > 20 && t < 20.01f) {
      BatteryState b = bus::battery.get();
      EscTelemetry e = bus::esc.get();
      vHover = b.voltage; aHover = b.current; mah = b.mahDrawn; temp = e.tempC[0];
    }
  });
  opt.adc = true;
  opt.edtEsc = false;
  edtOn = false;
  printf("  hover: %.2f V, %.1f A, %.1f mAh, ESC1 %.0f C\n", vHover, aHover, mah, temp);
  check("EDT requested from the ESCs (DShot command 13)", cmd13, "%.0f", cmd13 ? 1.0 : 0.0);
  check("Battery voltage from the ESCs (no divider)", vHover > 15.5f && vHover < 16.1f, "%.2f V", vHover);
  check("Battery current + mAh from the ESCs", aHover > 3 && mah > 3, "%.1f A", aHover);
  check("ESC temperature published", temp > 35 && temp < 70, "%.0f C", temp);
}


// fly up in ANGLE -> ALTHOLD (common start of the navigation scenarios)
static void takeoffScript(float t) {
  if (t >= 3.0f) rcIn.ch[4] = 2000;
  if (t >= 3.5f && t < 6.0f) rcIn.ch[RC_THROTTLE] = 1580;
  if (t >= 6.0f) { rcIn.ch[5] = 2000; rcIn.ch[RC_THROTTLE] = 1500; }
}

void scenarioGuided() {
  printf("\n== guided: QGC 'go to location' (DO_REPOSITION) and a local target, then RTH ==\n");
  setup([](Config&) {});
  float err1 = 99, err2 = 99;
  bool guided = false;
  run(60, [&](float t) {
    takeoffScript(t);
    if (t >= 10.0f) rcIn.ch[6] = 1500;  // POSHOLD
    static int sent = 0;
    if (t >= 12.0f && sent == 0) {
      sent = 1;
      bus::GuidedTarget g{};
      NavEstimate e = bus::nav.get();
      double lat, lon;
      NavEstimator::nedToLla(40, 25, e.homeLat, e.homeLon, lat, lon);
      g.lat = (int32_t)(lat * 1e7); g.lon = (int32_t)(lon * 1e7); g.alt = 12;
      bus::guided.publish(g);
    }
    if (t >= 30.0f && sent == 1) {
      sent = 2;
      bus::GuidedTarget g{};
      g.local = true; g.n = -10; g.e = 5; g.alt = 8;
      bus::guided.publish(g);
    }
  }, [&](float t) {
    VehicleStatus s = status();
    if (s.mode == FlightMode::GUIDED) guided = true;
    if (t > 28 && t < 29) err1 = hypotf(quad.pos.x - 40, quad.pos.y - 25) + fabsf(alt() - 12);
    if (t > 50 && t < 51) err2 = hypotf(quad.pos.x + 10, quad.pos.y - 5) + fabsf(alt() - 8);
    if (fmodf(t, 10.0f) < 0.0005f) trace(t);
  });
  check("GUIDED entered from a GCS target", guided, "%.0f", guided ? 1.0 : 0.0);
  check("Reaches the lat/lon target (12 m)", err1 < 2.0f, "error %.2f m", err1);
  check("Reaches the local NED target (8 m)", err2 < 2.0f, "error %.2f m", err2);
}

void scenarioSmartRtl() {
  printf("\n== smartrtl: fly a dog-leg in POSHOLD, then SmartRTL back along it and land ==\n");
  setup([](Config& c) {
    c.mode_range[8] = ModeRange{(uint8_t)ModeBox::SMART_RTL, 8, 1700, 2100};  // AUX5 high (replaces BEEPER)
  });
  float maxDev = 0, landDist = 99, disarmT = -1, farthest = 0;
  bool srtl = false;
  static std::vector<Vec3> track;
  track.clear();
  run(130, [&](float t) {
    takeoffScript(t);
    if (t >= 9.0f) rcIn.ch[6] = 1500;  // POSHOLD
    rcIn.ch[RC_PITCH] = (t >= 11 && t < 21) ? 1800 : 1500;  // forward ~50 m
    rcIn.ch[RC_ROLL] = (t >= 23 && t < 31) ? 1800 : 1500;   // then right ~40 m
    if (t >= 35.0f) rcIn.ch[8] = 2000;                      // SmartRTL
  }, [&](float t) {
    VehicleStatus s = status();
    if (s.mode == FlightMode::SMART_RTL) srtl = true;
    if (t > 9 && t < 35 && fmodf(t, 0.2f) < 0.0005f) track.push_back(quad.pos);
    if (t < 35) farthest = fmaxf(farthest, distHome());
    if (s.navPhase == NavPhase::SRTL_RETURN && fmodf(t, 0.1f) < 0.0005f) {
      float best = 1e9f;
      for (const Vec3& p : track) best = fminf(best, hypotf(p.x - quad.pos.x, p.y - quad.pos.y));
      maxDev = fmaxf(maxDev, best);
    }
    if (!s.armed && t > 40 && disarmT < 0) { disarmT = t; landDist = distHome(); }
    if (fmodf(t, 10.0f) < 0.0005f) trace(t);
  });
  check("SmartRTL engaged", srtl, "%.0f", srtl ? 1.0 : 0.0);
  check("Return follows the flown path (no shortcut)", farthest > 40 && maxDev < 4.0f, "max %.1f m off the track", maxDev);
  check("Lands at home and disarms", disarmT > 0 && landDist < 3.0f, "%.1f m from home", landDist);
}

void scenarioCrash() {
  printf("\n== crash: craft hits a branch, rolls to ~100 deg and stays stuck in ANGLE mode -> disarm ==\n");
  setup([](Config&) {});
  float disarmT = -1;
  bool crashMsg = false;
  static Quat stuckQ;
  run(25, [&](float t) {
    takeoffScript(t);
    if (t >= 10.0f) rcIn.ch[5] = 1500;  // ANGLE
    if (t >= 12.0f && t < 12.5f) quad.w = Vec3(3.5f, 0, 0);               // knocked over: 200 deg/s roll
    if (t >= 12.5f) {                                                       // stuck in the branches
      if (t < 12.502f) stuckQ = quad.q;
      quad.q = stuckQ; quad.w = Vec3(); quad.vel = Vec3();
    }
  }, [&](float t) {
    VehicleStatus s = status();
    if (t > 12 && !s.armed && disarmT < 0) disarmT = t;
    if (getenv("SIM_DBG") && fmodf(t, 0.5f) < 0.0005f && t > 11) {
      AttitudeState a = bus::attitude.get();
      printf("  t=%.1f armed %d mode %s est roll %.0f true roll %.0f rates %.0f acc %.1f\n", t, s.armed, flightModeName(s.mode),
             a.roll * RAD2DEG, quad.q.roll() * RAD2DEG, a.rateDps.norm(), a.accBody.norm());
    }
    char m[64];
    bus::readMessage(m, sizeof(m));
    if (strstr(m, "crash")) crashMsg = true;
  });
  check("Crash detected and disarmed within 3 s", disarmT > 12 && disarmT < 15.5f && crashMsg, "after %.1f s", disarmT - 12);
}

void scenarioGcsFailsafe() {
  printf("\n== gcsfs: GUIDED flight, ground station heartbeats stop -> RTH ==\n");
  setup([](Config& c) { c.gcs_fs = 1; c.gcs_fs_s = 3; });
  float rthT = -1;
  run(45, [&](float t) {
    takeoffScript(t);
    if (t >= 9.0f) rcIn.ch[6] = 1500;
    if (t < 20.0f) bus::gcsHeartbeatMs = os::timeMs() + 1;  // GCS alive until t = 20 s
    static bool sent = false;
    if (t >= 11.0f && !sent) {
      sent = true;
      bus::GuidedTarget g{};
      g.local = true; g.n = 30; g.e = 0; g.alt = 10;
      bus::guided.publish(g);
    }
  }, [&](float t) {
    VehicleStatus s = status();
    if (s.mode == FlightMode::RTH && rthT < 0) rthT = t;
  });
  check("GCS lost in GUIDED -> RTH after gcs_fs_s", rthT > 22.5f && rthT < 24.5f, "at t=%.1f s", rthT);
}

void scenarioSpin() {
  printf("\n== spin: yaw spin beyond the gyro range (2600 deg/s) -> yaw spin recovery ==\n");
  setup([](Config&) {});
  float rateAfter = 999;
  bool seen = false;
  run(20, [&](float t) {
    takeoffScript(t);
    if (t >= 10.0f) rcIn.ch[5] = 1500;
    static bool kicked = false;
    if (t >= 12.0f && !kicked) { kicked = true; quad.w.z = 2600 * DEG2RAD; }
  }, [&](float t) {
    if (t > 12 && t < 13 && fabsf(quad.w.z) * RAD2DEG > 1950) seen = true;
    if (t > 15.0f && t < 15.01f) rateAfter = fabsf(quad.w.z) * RAD2DEG;
  });
  VehicleStatus s = status();
  check("Spin stopped within 3 s, still flying", seen && rateAfter < 30.0f && s.armed, "%.0f deg/s after 3 s", rateAfter);
}

void scenarioLogic() {
  printf("\n== logic: rule 1 'altitude > 6 m -> beep', rule 2 'rule 1 -> switch to POSHOLD box' ==\n");
  setup([](Config& c) {
    c.logic[0] = LogicRule{1, LOP_FLIGHT, LO_GT, LOP_CONST, LV_ALT_DM, 60, LA_BEEP, 0, 0};
    c.logic[1] = LogicRule{1, LOP_RULE, LO_TRUE, LOP_CONST, 0, 0, LA_MODE_BOX, (uint8_t)ModeBox::POSHOLD, 0};
    c.logic[1].op = LO_AND; c.logic[1].bType = LOP_RULE; c.logic[1].bVal = 0;
  });
  bool beepLow = false, beepHigh = false, posHold = false;
  run(20, [&](float t) {
    if (t >= 3.0f) rcIn.ch[4] = 2000;
    if (t >= 3.5f && t < 9.0f) rcIn.ch[RC_THROTTLE] = 1600;
    if (t >= 9.0f) { rcIn.ch[5] = 2000; rcIn.ch[RC_THROTTLE] = 1500; }
  }, [&](float t) {
    if (alt() < 4 && bus::logicBeep) beepLow = true;
    if (alt() > 7 && bus::logicBeep) beepHigh = true;
    if (t > 8 && status().mode == FlightMode::POSHOLD) posHold = true;
    if (getenv("SIM_DBG") && fmodf(t, 1.0f) < 0.0005f)
      printf("  t=%.0f alt %.1f mode %s logic 0x%x beep %d\n", t, alt(), flightModeName(status().mode), bus::logicResults, bus::logicBeep);
  });
  check("Rule true only above 6 m", !beepLow && beepHigh, "%.0f", beepHigh ? 1.0 : 0.0);
  check("Chained rule activates the POSHOLD box", posHold, "%.0f", posHold ? 1.0 : 0.0);
}

void scenarioProfiles() {
  printf("\n== profiles: AUX6 selects PID profile 3 (soft gains) in flight ==\n");
  setup([](Config& c) {
    c.prof_ch = 10;  // AUX6 (channel 10)
    for (int a = 0; a < 3; a++) { c.prof_pid[1][a].p *= 0.5f; c.prof_rates[1][a].max = 300; }
  });
  int profAt8 = -1, profAt16 = -1;
  run(20, [&](float t) {
    takeoffScript(t);
    rcIn.ch[9] = t >= 12 ? 2000 : 1000;
  }, [&](float t) {
    if (t > 8 && t < 8.01f) profAt8 = bus::pidProfile;
    if (t > 16 && t < 16.01f) profAt16 = bus::pidProfile;
  });
  check("Profile switch low -> high = 1 -> 3", profAt8 == 0 && profAt16 == 2 && status().armed, "profile %.0f", profAt16 + 1.0);
}

void scenarioPolyFence() {
  printf("\n== polyfence: polygon 40 m x 40 m uploaded, pilot flies out -> fence breach -> RTH ==\n");
  setup([](Config& c) { c.fence_enable = 1; c.fence_radius = 500; c.fence_poly = 1; });
  nav::fenceSetCount(0);
  double mlat = 1.0 / 111319.49, mlon = mlat / cos(LAT0 * 0.0174532925);
  double v[4][2] = {{-20, -20}, {20, -20}, {20, 20}, {-20, 20}};
  for (int i = 0; i < 4; i++) {
    nav::MissionItem it{};
    it.cmd = nav::CMD_FENCE_POLYGON_INCLUSION;
    it.lat = (int32_t)((LAT0 + v[i][0] * mlat) * 1e7);
    it.lon = (int32_t)((LON0 + v[i][1] * mlon) * 1e7);
    nav::fenceSet(i, it);
  }
  nav::fenceSetCount(4);
  float breachDist = -1, maxDist = 0;
  run(60, [&](float t) {
    takeoffScript(t);
    if (t >= 9.0f) rcIn.ch[6] = 1500;
    rcIn.ch[RC_PITCH] = t >= 11 ? 1800 : 1500;  // keeps flying north
  }, [&](float t) {
    VehicleStatus s = status();
    if (s.mode == FlightMode::RTH && breachDist < 0) breachDist = quad.pos.x;
    if (t > 11) maxDist = fmaxf(maxDist, quad.pos.x);
  });
  nav::fenceSetCount(0);
  check("Breach at the polygon edge (20 m north) -> RTH", breachDist > 17 && breachDist < 24 && maxDist < 32,
        "RTH at %.1f m", breachDist);
}

void scenarioImuFail() {
  printf("\n== imufail: the first IMU stops answering in flight -> second IMU takes over ==\n");
  useImu2 = true;
  setup([](Config&) {});
  float altAfter = 0;
  bool switched = false;
  run(25, [&](float t) {
    takeoffScript(t);
    opt.imuDead = t >= 12.0f;
  }, [&](float t) {
    if (bus::imuActive) switched = true;
    if (t > 20 && t < 20.01f) altAfter = alt();
  });
  opt.imuDead = false;
  useImu2 = false;
  check("Switched to IMU 2, still holding altitude", switched && status().armed && altAfter > 3.0f, "alt %.1f m", altAfter);
}

void scenarioAcc6() {
  printf("\n== acc6: 6-point accelerometer calibration recovers a 4 %% scale error and a bias ==\n");
  setup([](Config& c) { c.acc_bias[0] = 0; c.acc_bias[1] = 0; c.acc_bias[2] = 0; });
  // the simulated IMU reads x 4 % high and z with a 0.3 m/s^2 offset: emulate by a pre-set wrong cal
  bus::config.acc_scale[0] = 1.0f / 1.04f;  // what the craft "believes" before the calibration
  bus::config.acc_bias[2] = -0.3f;
  bus::configChanged();
  static const float sides[6][3] = {{0, -90, 0}, {0, 90, 0}, {90, 0, 0}, {-90, 0, 0}, {180, 0, 0}, {0, 0, 0}};
  run(25, [&](float t) {
    int k = (int)(t / 3.5f);
    if (t > 1 && k < 6) {
      float r = sides[k][0] * DEG2RAD, pch = sides[k][1] * DEG2RAD;
      quad.q = Quat::fromEuler(r, pch, 0);
      quad.w = Vec3();
      quad.pos = Vec3(); quad.vel = Vec3();
    }
    static bool posted = false;
    if (t > 0.5f && !posted) { posted = true; bus::postRequest(bus::Request::ACC_CAL, 6); }
  }, [&](float) {});
  const Config& c = bus::config;
  check("6 sides captured", bus::acc6Mask == 0x3F, "mask %.0f (63 = all)", (double)bus::acc6Mask);
  check("Scale and bias back to the true values", fabsf(c.acc_scale[0] - 1.0f) < 0.01f && fabsf(c.acc_bias[2]) < 0.08f,
        "scale x %.3f", c.acc_scale[0]);
}

void scenarioCompassMot() {
  printf("\n== compassmot: compass reads 6 mG per A on x -> measured and compensated ==\n");
  opt.magCmot = 0.006f;
  opt.held = true;
  opt.currentA = 20.0f;
  setup([](Config&) {});
  run(30, [&](float t) {
    static bool posted = false;
    if (t > 3 && !posted) { posted = true; bus::postRequest(bus::Request::COMPASSMOT); }
  }, [&](float) {});
  opt.magCmot = 0;
  opt.held = false;
  opt.currentA = 8.0f;
  const Config& c = bus::config;
  check("Interference per amp measured", fabsf(c.mag_cmot[0] - 0.006f) < 0.0015f && fabsf(c.mag_cmot[1]) < 0.0015f,
        "x %.4f G/A", c.mag_cmot[0]);
}


void scenarioFeel() {
  printf("\n== feel: dynamic idle + throttle boost + thrust linearisation + sag compensation + dyn LPF + D_min ==\n");
  opt.vibration = true;
  setup([](Config& c) {
    c.esc_telemetry = 0; c.dshot_bidir = 1;
    c.dyn_idle_rpm = 35; c.thr_boost = 5; c.thrust_expo = 0.6f; c.vbat_sag_comp = 100;
    c.dyn_lpf_min = 120; c.dyn_lpf_max = 300; c.d_min_pct = 60;
  });
  float minRpmDive = 1e9f, altErr = 0, maxRate = 0, hoverAlt = 0;
  run(30, [&](float t) {
    takeoffScript(t);
    if (t >= 14.0f && t < 15.0f) { rcIn.ch[5] = 1000; rcIn.ch[RC_THROTTLE] = 1000; }  // ACRO, throttle cut (dive)
    if (t >= 15.0f && t < 16.0f) { rcIn.ch[5] = 1000; rcIn.ch[RC_THROTTLE] = 1700; }  // punch out
    if (t >= 16.0f) { rcIn.ch[5] = 2000; rcIn.ch[RC_THROTTLE] = 1500; }
  }, [&](float t) {
    if (t > 14.2f && t < 15.0f) for (int i = 0; i < 4; i++) minRpmDive = fminf(minRpmDive, quad.motorHz(i) * 60.0f);
    if (fabsf(t - 10.0f) < 0.0005f) hoverAlt = alt();
    if (t > 10 && t < 14) altErr = fmaxf(altErr, fabsf(alt() - hoverAlt));
    if (t > 8 && t < 14) maxRate = fmaxf(maxRate, quad.w.norm() * RAD2DEG);
  });
  opt.vibration = false;
  check("Dynamic idle keeps the props turning in a dive (target 3500 rpm)", minRpmDive > 3150, "min %.0f rpm", minRpmDive);
  check("Hover stays steady with all features on", altErr < 0.6f && maxRate < 30.0f, "alt err %.2f m", altErr);
  check("Recovers after dive + punch-out", status().armed && alt() > 1.0f, "alt %.1f m", alt());
}

void scenarioEkfFaults() {
  printf("\n== ekf_faults: baro drifts 12 m, then compass interference 70 deg -> EKF rejects both ==\n");
  setup([](Config& c) { c.rth_alt = 15; });
  bool baroFault = false, magFault = false;
  float altErr = 0, disarmT = -1, holdErr = 0;
  uint32_t warnSeen = 0;
  run(150, [&](float t) {
    if (t >= 3.0f) rcIn.ch[4] = 2000;
    if (t >= 3.5f && t < 6.0f) rcIn.ch[RC_THROTTLE] = 1580;
    if (t >= 6.0f) { rcIn.ch[5] = 2000; rcIn.ch[RC_THROTTLE] = 1500; }
    opt.baroOffset = t > 15 ? fminf((t - 15) * 4.0f, 12.0f) : 0.0f;   // e.g. sun on the sensor / prop wash
    if (t >= 10) rcIn.ch[6] = 1500;                                     // POSHOLD
    opt.magRot = t > 40 ? 70.0f * DEG2RAD : 0.0f;                      // power wires next to the compass
    // the pilot flies a few legs (accelerations make the heading observable through GPS)
    rcIn.ch[RC_PITCH] = (t > 45 && t < 75) ? (fmodf(t, 6.0f) < 3.0f ? 1800 : 1200) : 1500;
    rcIn.ch[RC_ROLL] = (t > 45 && t < 75) ? (fmodf(t, 8.0f) < 4.0f ? 1700 : 1300) : 1500;
    if (t >= 90.0f) rcAlive = false;                                    // then RC lost: RTH
  }, [&](float t) {
    VehicleStatus s = status();
    EkfReport k = bus::ekf.get();
    if (k.faults & EKF_BARO_FAULT) baroFault = true;
    if (k.faults & EKF_MAG_FAULT) magFault = true;
    warnSeen |= s.warnings;
    if (t > 25 && t < 40) altErr = fmaxf(altErr, fabsf(alt() - 7.0f));
    if (t > 25 && t < 40) holdErr = fmaxf(holdErr, fabsf(bus::nav.get().alt - alt()));
    if (t > 90 && !s.armed && disarmT < 0) disarmT = t;
    if (fmodf(t, 10.0f) < 0.0005f) trace(t);
  });
  opt.baroOffset = 0;
  opt.magRot = 0;
  check("Baro drift detected (EKF baro fault)", baroFault, "%.0f", baroFault);
  check("Height kept on GPS during the baro drift", holdErr < 2.5f, "estimate error %.2f m", holdErr);
  check("Compass interference detected", magFault && (warnSeen & WARN_EKF), "%.0f", magFault);
  check("RTH home and landed after both faults", disarmT > 0 && distHome() < 3.0f, "distance %.2f m", distHome());
}

void scenarioRescue() {
  printf("\n== rescue: no barometer, no compass; GPS RESCUE switch -> home on GPS height + GPS heading ==\n");
  opt.baro = false;
  opt.mag = false;
  setup([](Config& c) {
    c.rth_alt = 12;
    c.mode_range[9] = {(uint8_t)ModeBox::GPS_RESCUE, 10, 1700, 2100};  // AUX7 high
    c.mode_range[2].box = 0;                                           // ALTHOLD needs the EKF height first
  });
  bool sawRth = false;
  float disarmT = -1, maxD = 0;
  run(150, [&](float t) {
    if (t >= 3.0f) rcIn.ch[4] = 2000;
    if (t >= 3.5f && t < 12.0f) rcIn.ch[RC_THROTTLE] = t < 6 ? 1600 : 1440;   // ANGLE, manual throttle
    rcIn.ch[RC_PITCH] = (t > 8 && t < 16) ? 1680 : 1500;                      // fly away
    rcIn.ch[RC_ROLL] = (t > 10 && t < 13) ? 1650 : 1500;
    if (t >= 12.0f && t < 20) rcIn.ch[RC_THROTTLE] = 1450;
    if (t >= 20.0f) rcIn.ch[10] = 2000;                                       // GPS RESCUE
  }, [&](float t) {
    VehicleStatus s = status();
    maxD = fmaxf(maxD, distHome());
    if (s.mode == FlightMode::RTH) sawRth = true;
    if (t > 20 && !s.armed && disarmT < 0) disarmT = t;
    if (fmodf(t, 10.0f) < 0.0005f) trace(t);
  });
  opt.baro = true;
  opt.mag = true;
  check("Flew away in ANGLE", maxD > 10.0f, "max %.1f m", maxD);
  check("GPS RESCUE switch -> RTH without baro / compass", sawRth, "%.0f", sawRth);
  check("Rescued home and landed", disarmT > 0 && distHome() < 4.0f, "distance %.2f m", distHome());
}

void scenarioRescueSanity() {
  printf("\n== rescue_sanity: headwind stronger than the craft -> no progress home -> lands ==\n");
  setup([](Config& c) { c.rth_alt = 8; c.rth_sanity_s = 8; c.pos_max_angle_deg = 15; });
  bool warned = false;
  float landT = -1, disarmT = -1;
  run(120, [&](float t) {
    if (t >= 3.0f) rcIn.ch[4] = 2000;
    if (t >= 3.5f && t < 6.0f) rcIn.ch[RC_THROTTLE] = 1580;
    if (t >= 6.0f) { rcIn.ch[5] = 2000; rcIn.ch[RC_THROTTLE] = 1500; }
    rcIn.ch[RC_PITCH] = (t > 7 && t < 13) ? 1750 : 1500;
    if (t >= 16.0f) { quad.wind = Vec3(1.8f, 0, 0); rcAlive = false; }  // strong wind away from home, RC lost
  }, [&](float t) {
    VehicleStatus s = status();
    if (s.warnings & WARN_RESCUE) warned = true;
    if (s.mode == FlightMode::LAND && landT < 0) landT = t;
    if (t > 16 && !s.armed && disarmT < 0) disarmT = t;
    if (fmodf(t, 10.0f) < 0.0005f) trace(t);
  });
  quad.wind = Vec3();
  check("No progress detected, rescue lands", warned && landT > 16, "at t=%.1f s", landT);
  check("Landed and disarmed", disarmT > 0 && alt() < 0.2f, "at t=%.1f s", disarmT);
}

void scenarioLaunch() {
  printf("\n== launch: launch control holds the craft on the ground, throttle up launches ==\n");
  setup([](Config& c) {
    c.mode_range[9] = {(uint8_t)ModeBox::LAUNCH, 10, 1700, 2100};
    c.launch_angle = 20;
    c.launch_thr = 25;
  }, Quat::fromEuler(0, 0, 0));
  bool waiting = false, stayed = true, flipped = false;
  float n5 = 0;
  run(12, [&](float t) {
    rcIn.ch[5] = 1000;                                 // ACRO
    rcIn.ch[10] = 2000;                                // launch control on
    if (t >= 3.0f) rcIn.ch[4] = 2000;
    if (t >= 3.5f && t < 6.0f) rcIn.ch[RC_THROTTLE] = 1150;   // below launch_thr: wait
    if (t >= 6.0f) rcIn.ch[RC_THROTTLE] = 1700;               // go
  }, [&](float t) {
    VehicleStatus s = status();
    if (t > 4 && t < 6 && (s.warnings & WARN_LAUNCH)) waiting = true;
    if (t > 4 && t < 6 && alt() > 0.1f) stayed = false;
    if (fabsf(quad.q.roll()) > 1.2f || fabsf(quad.q.pitch()) > 1.2f) flipped = true;
    if (fabsf(t - 8.0f) < 0.0005f) n5 = quad.pos.x;
  });
  check("Launch control waits on the ground", waiting && stayed, "%.0f", waiting && stayed);
  check("Throttle up launches forward", n5 > 2.0f && !flipped && alt() > 1.0f, "%.1f m forward", n5);
}

void scenarioVibeWarn() {
  printf("\n== vibe: heavy accelerometer vibration -> VIBRATION warning ==\n");
  setup([](Config&) {});
  bool warnLow = false, warnHigh = false;
  run(16, [&](float t) {
    if (t >= 3.0f) rcIn.ch[4] = 2000;
    if (t >= 3.5f && t < 6.0f) rcIn.ch[RC_THROTTLE] = 1580;
    if (t >= 6.0f) { rcIn.ch[5] = 2000; rcIn.ch[RC_THROTTLE] = 1500; }
    opt.accVib = t > 10 ? 60.0f : 0.0f;
  }, [&](float t) {
    VehicleStatus s = status();
    if (t > 7 && t < 9.5f && (s.warnings & WARN_VIBRATION)) warnLow = true;
    if (t > 12 && (s.warnings & WARN_VIBRATION)) warnHigh = true;
  });
  opt.accVib = 0;
  check("No warning with normal vibration", !warnLow, "%.0f", warnLow);
  check("VIBRATION warning with a loose / unbalanced prop", warnHigh, "%.0f", warnHigh);
}

void scenarioAccClip() {
  printf("\n== clip: accelerometer beyond its +-16 g range -> ACC CLIPPING warning ==\n");
  setup([](Config&) {});
  bool early = false, warn = false;
  uint32_t clips = 0;
  run(14, [&](float t) {
    if (t >= 3.0f) rcIn.ch[4] = 2000;
    if (t >= 3.5f && t < 6.0f) rcIn.ch[RC_THROTTLE] = 1580;
    if (t >= 6.0f) { rcIn.ch[5] = 2000; rcIn.ch[RC_THROTTLE] = 1500; }
    opt.accVib = t > 9 ? 200.0f : 0.0f;  // a loose FC: spikes past 16 g
  }, [&](float t) {
    VehicleStatus s = status();
    if (t > 4 && t < 8.5f && (s.warnings & WARN_ACC_CLIP)) early = true;
    if (t > 10 && (s.warnings & WARN_ACC_CLIP)) warn = true;
    clips = bus::attitude.get().accClips;
  });
  opt.accVib = 0;
  check("No clipping warning in normal flight", !early, "%.0f", early);
  check("ACC CLIPPING warning when the accelerometer saturates", warn && clips > 100, "%.0f samples", (float)clips);
}

void scenarioBeginner() {
  printf("\n== beginner: first-flight limits (ACRO switch flies ANGLE, 25 deg, fence -> RTH) ==\n");
  setup([](Config& c) { c.beginner = 1; c.beg_radius = 40; });
  float maxTilt = 0, maxDist = 0, thrMax = 0;
  bool angle = true, rth = false;
  run(70, [&](float t) {
    if (t >= 3.0f) rcIn.ch[4] = 2000;  // arm; the mode switch stays low (ACRO)
    if (t >= 3.5f) rcIn.ch[RC_THROTTLE] = t > 6 && t < 7 ? 2000 : t >= 8 ? 1720 : 1600;  // more throttle when tilted
    rcIn.ch[RC_ROLL] = t >= 8 ? 2000 : 1500;  // full roll and keep it there: drifts away
    if (t >= 12) { rcIn.ch[5] = 2000; rcIn.ch[RC_THROTTLE] = 1500; }  // ALTHOLD (allowed), still full roll
  }, [&](float t) {
    VehicleStatus s = status();
    float tilt = acosf(constrainf(quad.q.cosTilt(), -1.0f, 1.0f)) * RAD2DEG;
    if (t > 8.5f && t < 12 && s.mode != FlightMode::RTH) {
      maxTilt = fmaxf(maxTilt, tilt);
      if (s.mode != FlightMode::ANGLE) angle = false;
    }
    if (t > 6.2f && t < 7) thrMax = fmaxf(thrMax, bus::setpoint.get().throttle);
    if (s.mode == FlightMode::RTH) rth = true;
    maxDist = fmaxf(maxDist, hypotf(quad.pos.x, quad.pos.y));
    if (getenv("AQ_SIM_DEBUG") && fmodf(t, 2.0f) < 0.001f)
      printf("  t %.0f mode %d pos %.1f %.1f alt %.1f tilt %.1f warn %x\n", t, (int)s.mode, quad.pos.x, quad.pos.y, alt(), tilt, (unsigned)s.warnings);
  });
  check("First flight: ACRO switch flies ANGLE, tilt <= 27 deg", angle && maxTilt < 27.0f, "%.1f deg", maxTilt);
  check("First flight: full throttle stick gives 75 %", thrMax < 0.77f && thrMax > 0.7f, "%.2f", thrMax);
  check("First flight: leaving the 40 m fence starts RTH", rth && maxDist < 70.0f, "max %.0f m", maxDist);
}
}  // namespace


// ------------------------------------------------------------------ serve: virtual flight controller
// build/sim serve  -> the firmware runs in real time; its USB console is stdin/stdout (binary).
// tools/configurator.py connects the configurator page to it ("Mô phỏng (SITL)"), so the whole
// PC configurator can be tried and tested without a board. The craft sits on the ground, disarmed,
// with GPS and compass; nothing else is printed on stdout.
#if defined(AQ_WASM)
// ------------------------------------------------------------------ WebAssembly build (configurator demo)
// tools/build_wasm.sh compiles the whole firmware + this simulator with wasi-sdk; the configurator page
// runs it in a Web Worker (button "Demo") and talks to its USB console exactly like to a real board.
static uint8_t wasmIn[8192], wasmOut[65536];
static uint32_t wasmOutLen = 0;
extern "C" {
__attribute__((export_name("aq_init"))) void aq_init() { setup([](Config&) {}); }
__attribute__((export_name("aq_in_buf"))) uint8_t* aq_in_buf() { return wasmIn; }
__attribute__((export_name("aq_out_buf"))) uint8_t* aq_out_buf() { return wasmOut; }
__attribute__((export_name("aq_input"))) void aq_input(int n) {
  auto& in = hal::host::in(hal::Port::CONSOLE);
  for (int i = 0; i < n && i < (int)sizeof(wasmIn); i++) in.push_back(wasmIn[i]);
}
// run the firmware up to `us` microseconds of simulated time; returns the console bytes produced
__attribute__((export_name("aq_step"))) int aq_step(double us) {
  float sec = (float)(us * 1e-6);
  quad.q = Quat::fromEuler(0.05f * sinf(sec * 0.7f), 0.04f * sinf(sec * 0.43f), 30 * DEG2RAD);
  os::hostRunUntil((uint64_t)us);
  std::string& out = hal::host::out(hal::Port::CONSOLE);
  uint32_t n = out.size() < sizeof(wasmOut) ? (uint32_t)out.size() : (uint32_t)sizeof(wasmOut);
  memcpy(wasmOut, out.data(), n);
  out.erase(0, n);
  wasmOutLen = n;
  return (int)n;
}
}

#else
int serve() {
  setup([](Config&) {});
  fcntl(0, F_SETFL, fcntl(0, F_GETFL) | O_NONBLOCK);
  timespec t0;
  clock_gettime(CLOCK_MONOTONIC, &t0);
  std::string& out = hal::host::out(hal::Port::CONSOLE);
  auto& in = hal::host::in(hal::Port::CONSOLE);
  uint8_t buf[4096];
  for (;;) {
    ssize_t n = read(0, buf, sizeof(buf));
    if (n == 0) return 0;  // the bridge closed: stop
    for (ssize_t i = 0; i < n; i++) in.push_back(buf[i]);
    timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    uint64_t us = (uint64_t)(t.tv_sec - t0.tv_sec) * 1000000ull + (uint64_t)((t.tv_nsec - t0.tv_nsec) / 1000);
    // a little life on the bench: the frame rocks slowly, so the horizon and charts move
    float sec = us * 1e-6f;
    quad.q = Quat::fromEuler(0.05f * sinf(sec * 0.7f), 0.04f * sinf(sec * 0.43f), 30 * DEG2RAD);
    os::hostRunUntil(us);
    if (!out.empty()) {
      if (fwrite(out.data(), 1, out.size(), stdout) != out.size()) return 0;
      fflush(stdout);
      out.clear();
    }
    timespec nap{0, 1000000};
    nanosleep(&nap, nullptr);
  }
}

int main(int argc, char** argv) {
  const char* which = argc > 1 ? argv[1] : "all";
  if (!strcmp(which, "serve")) return serve();
  struct S { const char* name; void (*fn)(); } scenarios[] = {
      {"basic", scenarioBasic}, {"mission", scenarioMission}, {"fence", scenarioFence}, {"battery", scenarioBattery},
      {"flow", scenarioFlow}, {"vibration", scenarioVibration}, {"turtle", scenarioTurtle}, {"tune", scenarioTune}, {"wind", scenarioWind},
      {"glitch", scenarioGlitch}, {"hover", scenarioHover}, {"nomag", scenarioNoMag}, {"terrain", scenarioTerrain},
      {"cruise", scenarioCruise}, {"mission2", scenarioMission2}, {"autotune", scenarioAutotune},
      {"autotune_abort", scenarioAutotuneAbort}, {"motortest", scenarioMotorTest}, {"motormap", scenarioMotorMap}, {"bidir", scenarioBidir}, {"rpmloss", scenarioRpmLoss}, {"console", scenarioConsole}, {"edt", scenarioEdt},
      {"guided", scenarioGuided}, {"smartrtl", scenarioSmartRtl}, {"crash", scenarioCrash}, {"gcsfs", scenarioGcsFailsafe},
      {"spin", scenarioSpin}, {"logic", scenarioLogic}, {"profiles", scenarioProfiles}, {"polyfence", scenarioPolyFence},
      {"imufail", scenarioImuFail}, {"acc6", scenarioAcc6}, {"compassmot", scenarioCompassMot}, {"feel", scenarioFeel},
      {"ekf_faults", scenarioEkfFaults}, {"rescue", scenarioRescue}, {"rescue_sanity", scenarioRescueSanity},
      {"launch", scenarioLaunch}, {"vibe", scenarioVibeWarn}, {"clip", scenarioAccClip}, {"beginner", scenarioBeginner}};
  if (!strcmp(which, "all")) {  // every scenario in a fresh process (clean firmware state)
    int failed = 0;
    for (auto& s : scenarios) {
      char cmd[512];
      snprintf(cmd, sizeof(cmd), "\"%s\" %s", argv[0], s.name);
      if (system(cmd) != 0) failed++;
    }
    printf("\n%d of %d scenarios failed\n", failed, (int)(sizeof(scenarios) / sizeof(scenarios[0])));
    return failed ? 1 : 0;
  }
  for (auto& s : scenarios)
    if (!strcmp(which, s.name)) s.fn();
  int fails = 0;
  printf("\n==== RESULTS ====\n");
  for (int i = 0; i < nResults; i++) {
    printf("[%s] %-38s %s\n", results[i].ok ? "PASS" : "FAIL", results[i].name, results[i].detail);
    if (!results[i].ok) fails++;
  }
  printf("%d/%d checks passed\n", nResults - fails, nResults);
  return fails ? 1 : 0;
}
#endif  // AQ_WASM
