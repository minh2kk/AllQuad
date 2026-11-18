#include "app.h"

#include <stdio.h>
#include <string.h>

#if defined(ESP32) && defined(ARDUINO)
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#endif

#include "../bus/bus.h"
#include "../comms/aqlink.h"
#include "../comms/crsf_msp.h"
#include "../comms/fourway.h"
#include "../vtx/vtx.h"
#include "leds.h"
#include "../comms/cli.h"
#include "../comms/mavlink.h"
#include "../comms/msp.h"
#include "../control/fast_loop.h"
#include "../estimation/mag_fit.h"
#include "../gps/gps_driver.h"
#include "../hal/hal.h"
#include "../logging/blackbox.h"
#include "../logging/sdlog.h"
#include "../navigation/mission.h"
#include "../osd/osd.h"
#include "../rx/rx_parser.h"
#include "../web/credentials.h"
#include "../web/web.h"
#include "../web/webapi.h"
#include "../safety/safety.h"
#include "../sensors/serial_sensors.h"
#include "commander.h"

namespace app {

using namespace fc;
using hal::Port;

namespace {

Drivers drv_{};
uint16_t loopHz = 1000;

// ---- per-port protocol objects ----
constexpr int NPORTS = (int)Port::COUNT;
PortFn portFn[NPORTS];
drv::RxParser rx;
drv::GpsDriver gps;
drv::EscTelemetryParser escParser;
drv::TfminiParser tfmini;
comms::MavlinkLink mav[NPORTS];
comms::MspLink msp[NPORTS];
comms::Cli cli;
comms::AqLink aqlink;  // configurator over the USB console (Web Serial)
comms::MspLink mspCrsf;  // MSP requests from the radio's Lua scripts, carried in CRSF frames
comms::CrsfMsp crsfMsp;
comms::FourWay fourWay;  // ESC passthrough (esc-configurator.com) on the USB console
vtx::Driver vtxDrv;
bool ledsOn = false;
comms::SystemInfo sysInfo;
osd::Renderer osdRenderer;
osd::Max7456 osdMax;
osd::DisplayPort* osdDp = nullptr;
logging::Blackbox blackbox;
Port rxPort = Port::COUNT, gpsPort = Port::COUNT, escPort = Port::COUNT, rangePort = Port::COUNT;
Port cfgPort = Port::COUNT;  // UART with the "configurator" function (WiFi bridge / Bluetooth)
bool osdActive = false;

// every serial / network port except the USB console (it always speaks CLI, MSP, MAVLink)
inline bool configurablePort(int i) { return i != (int)Port::CONSOLE && i < NPORTS; }

Port findPort(PortFn f) {
  for (int i = 0; i < NPORTS; i++)
    if (configurablePort(i) && portFn[i] == f && hal::portExists((Port)i)) return (Port)i;
  return Port::COUNT;
}

// =========================================================== fast task
// profile accumulators (cycles), published once per second as bus::loopPerf
struct PerfAcc {
  uint64_t imu, filt, ahrs, pid, motor, total;
  uint32_t maxTotal, n, late;
};
static PerfAcc perfAcc;

#if defined(ESP32)
static constexpr uint16_t IO_STACK = 6144;
#else
static constexpr uint16_t IO_STACK = 4096;
#endif

// motor reordering: output channel (0-3) of each mixer motor; anything but a permutation = identity
FAST_CODE static void motorMap(const Config& c, uint8_t m[4]) {
  uint8_t seen = 0;
  for (int i = 0; i < 4; i++) {
    m[i] = (uint8_t)(c.motor_out[i] - 1);
    if (m[i] > 3) { seen = 0; break; }
    seen |= (uint8_t)(1 << m[i]);
  }
  if (seen != 0x0F) for (int i = 0; i < 4; i++) m[i] = (uint8_t)i;
}
FAST_CODE static void writeMotors(const uint8_t map[4], const float v[4], bool armed, int telemOut) {
  float o[4];
  for (int i = 0; i < 4; i++) o[map[i]] = v[i];
  hal::motorsWrite(o, armed, telemOut);
}

FAST_CODE void fastTask(void*) {
  static FastLoop loop;
  static uint32_t gen = 0, tick = 0;
  static uint64_t lastUs = 0;
  static MagSample mag{};
  static float motorHz[4] = {0, 0, 0, 0};
  static MotorMode lastMode = MotorMode::STOPPED;
  static int imuErrors = 0;
  static bool fastArmed = false;  // armed state of the previous iteration
  const Config& c = bus::config;
  static uint32_t gainsGen = 0;
  if (gen != bus::configGeneration) { gen = bus::configGeneration; gainsGen = bus::gainsGeneration; loop.configure(c, loopHz); }
  else if (gainsGen != bus::gainsGeneration) { gainsGen = bus::gainsGeneration; loop.setGains(c); }
  if (bus::gyroCalRequest) { bus::gyroCalRequest = false; loop.requestGyroCalibration(); }

  uint32_t cStart = cycles();
  uint64_t now = os::timeUs();
  if (lastUs && now - lastUs > (uint64_t)(2000000.0f / loopHz)) perfAcc.late++;
  float dt = lastUs ? (float)(now - lastUs) * 1e-6f : 1.0f / loopHz;
  if (dt <= 0 || dt > 0.02f) dt = 1.0f / loopHz;
  lastUs = now;
  tick++;
  bus::fastLoops = tick;

  static Vec3 g, a;  // last good sample is reused after a single failed read
  Vec3 gn, an;
  // IMU failover (ArduPilot redundancy): the second chip takes over when the first one stops answering
  // or freezes (identical samples); the switch is one-way until reboot and blocks the next arming.
  static drv::ImuDriver* active = nullptr;
  static uint32_t frozen = 0;
  static Vec3 lastRaw;
  if (!active) active = drv_.imu;
  bool ok = active && active->read(gn, an);
  if (ok && active == drv_.imu && drv_.imu2) {
    frozen = (gn.x == lastRaw.x && gn.y == lastRaw.y && gn.z == lastRaw.z) ? frozen + 1 : 0;
    lastRaw = gn;
    if (frozen > loopHz / 2) ok = false;  // 0.5 s of bit-identical gyro data: the chip has stopped
  }
  if (!ok && active == drv_.imu && drv_.imu2 && imuErrors >= 10) {
    Vec3 g2, a2;
    if (drv_.imu2->read(g2, a2)) {
      active = drv_.imu2;
      bus::imuActive = 1;
      bus::postMessage("IMU failed: switched to IMU 2");
      gn = g2; an = a2; ok = true;
    }
  }
  uint32_t cImu = cycles();
  imuErrors = ok ? 0 : imuErrors + 1;
  if (ok) {
    g = drv::applyAlign(gn, (Align)c.imu_align);
    a = drv::applyAlign(an, (Align)c.imu_align);
    // fine board alignment (degrees) on top of the 90-degree steps
    static float br = 0, bp = 0, by = 0;
    static Quat bq;
    static bool bOn = false;
    if (c.board_roll_deg != br || c.board_pitch_deg != bp || c.board_yaw_deg != by) {
      br = c.board_roll_deg; bp = c.board_pitch_deg; by = c.board_yaw_deg;
      bOn = br != 0 || bp != 0 || by != 0;
      bq = Quat::fromEuler(br * DEG2RAD, bp * DEG2RAD, by * DEG2RAD);
    }
    if (bOn) { g = bq.rotate(g); a = bq.rotate(a); }
  }
  bool imuOk = drv_.imu && imuErrors < 20 && (ok || tick > 1);

  ControlSetpoint sp;
  uint32_t spSeq = bus::setpoint.read(sp);
  sp.seq = spSeq;
  // low-latency sticks: new RC frame -> rate setpoint in this loop, not one commander cycle (4 ms) later
  static uint32_t rcSeq = 0, rcMs = 0;
  static float rcRate[3];
  static uint8_t rcProf = 255;
  if (sp.stickDirect) {
    uint32_t s = bus::rc.seq();
    if (s != rcSeq || rcProf != sp.profile) {
      RcInput rc;
      rcSeq = bus::rc.read(rc);
      rcProf = sp.profile;
      rcMs = rc.lastFrameMs;
      RateProfile rp[3];
      for (int a = 0; a < 3; a++) {
        rp[a] = profileRate(c, sp.profile, a);
        if (c.beginner) {  // first-flight rate limits (beginnerLimits)
          float mx = a == 2 ? 120.0f : 250.0f;
          rp[a].max = fminf(rp[a].max, mx);
          rp[a].center = fminf(rp[a].center, mx);
        }
      }
      rcRate[0] = stickRate(rcStickUs(rc.ch[RC_ROLL]), rp[0]);
      rcRate[1] = stickRate(-rcStickUs(rc.ch[RC_PITCH]), rp[1]);
      rcRate[2] = stickRate(rcStickUs(rc.ch[RC_YAW]), rp[2]);
    }
    if ((uint32_t)(now / 1000) - rcMs < 200) {  // stale RC: keep the commander's values (it handles the failsafe)
      if (sp.stickDirect & 1) { sp.rateDps[0] = rcRate[0]; sp.rateDps[1] = rcRate[1]; }
      if (sp.stickDirect & 2) sp.rateDps[2] = rcRate[2];
      sp.seq = rcSeq;  // RC smoothing follows the RC frame rate
    }
  }
  uint8_t omap[4];
  motorMap(c, omap);
  if (tick % 8 == 0) {
    bus::mag.read(mag);
    if (bus::magRejected) mag.healthy = false;  // EKF: compass inconsistent with the GPS heading
    // RPM per motor for the RPM filter; valid = the ESC answers (bidirectional DShot: within 100 ms,
    // serial ESC telemetry: within 200 ms). A motor without valid telemetry gets frequency 0: its
    // notches fade out; while armed that is reported (WARN_RPM_LOST) for the motors that had it.
    static RpmGate gate[4];
    static uint8_t hadRpm = 0;
    uint8_t valid = 0;
    bool source = false;
    if (hal::motorsBidirActive()) {
      // bidirectional DShot: eRPM straight from the ESCs, every motor every loop
      source = true;
      float erpm[4];
      hal::motorsErpm(erpm);
      float pairs = (c.motor_poles >= 2 ? c.motor_poles : 14) * 0.5f;
      for (int i = 0; i < 4; i++) {
        float rpm = erpm[omap[i]] >= 0 ? erpm[omap[i]] / pairs : -1.0f;
        bus::bidirRpm[i] = rpm;
        if (rpm >= 0) valid |= (uint8_t)(1 << i);
        motorHz[i] = gate[i].update(rpm > 0 ? rpm / 60.0f : 0.0f);
      }
    } else {
      EscTelemetry esc;
      if (bus::esc.read(esc)) {
        source = c.esc_telemetry != 0;
        for (int i = 0; i < 4; i++) {
          bool ok = (now / 1000 - esc.lastUpdateMs[omap[i]]) < 200 && esc.lastUpdateMs[omap[i]];
          if (ok) valid |= (uint8_t)(1 << i);
          motorHz[i] = gate[i].update(ok ? esc.rpm[omap[i]] / 60.0f : 0.0f);
        }
      }
    }
    bool armedNow = fastArmed;
    if (!armedNow || !source) hadRpm = armedNow ? hadRpm : valid;  // what worked at arming is expected in flight
    else hadRpm |= valid;
    bus::rpmLostMask = (armedNow && source) ? (uint8_t)(hadRpm & ~valid) : 0;
  }

  FastLoopOutput out;
  loop.run(g, a, imuOk, sp, &mag, c.mag_declination_deg * DEG2RAD, motorHz, dt, out);
  fastArmed = out.armed;

  // turtle mode: reverse / restore motor direction with DShot commands
  if (out.mode != lastMode) {
    if (out.mode == MotorMode::TURTLE) hal::motorsCommand(21);           // spin direction reversed
    else if (lastMode == MotorMode::TURTLE) hal::motorsCommand(20);      // normal
    lastMode = out.mode;
  }
  int telem = -1;
  if (c.esc_telemetry && !c.dshot_bidir && escPort != Port::COUNT && tick % 4 == 0) {
    telem = (int)((tick / 4) % 4);
    bus::escTelemMotor = (int8_t)telem;
  }
  uint32_t cMot = cycles();
  uint32_t nowMs = (uint32_t)(now / 1000);
  if (bus::escPassthrough) {
    // the motor pins carry the BLHeli bootloader line: nothing is written
  } else if (!out.armed && bus::escCmdPending) {
    // DShot command to some ESCs (beep / spin direction): ~10 ms once, motors stopped anyway
    uint8_t outMask = 0;  // the request names mixer motors
    for (int i = 0; i < 4; i++) if (bus::escCmdMask & (1 << i)) outMask |= (uint8_t)(1 << omap[i]);
    hal::motorsCommandMask(bus::escCmd, outMask);
    if (bus::escCmd2) { hal::delayMs(2); hal::motorsCommandMask(bus::escCmd2, outMask); }
    bus::escCmdPending = false;
  } else if (!out.armed && bus::escCalActive(nowMs) && !hal::motorsAreDshot()) {
    // ESC throttle-range calibration: maximum then minimum pulse on every output
    float t[4];
    for (int i = 0; i < 4; i++) t[i] = bus::escCalState == 1 ? 1.0f : 0.0f;
    hal::motorsWrite(t, true, -1);
  } else if (!out.armed && bus::motorTestActive(nowMs)) {
    float t[4];
    for (int i = 0; i < 4; i++) t[i] = bus::motorTest[i] > 0 ? bus::motorTest[i] : -1.0f;  // 0 = stopped
    writeMotors(omap, t, true, -1);
  } else {
    writeMotors(omap, out.motor, out.armed, telem);
  }
  uint32_t cEnd = cycles();
  hal::markFastLoopDone();
  perfAcc.imu += cImu - cStart;
  perfAcc.filt += loop.cycFilter();
  perfAcc.ahrs += loop.cycAhrs();
  perfAcc.pid += loop.cycPid();
  perfAcc.motor += cEnd - cMot;
  uint32_t total = cEnd - cStart;
  perfAcc.total += total;
  if (total > perfAcc.maxTotal) perfAcc.maxTotal = total;
  if (++perfAcc.n >= (uint32_t)loopHz) {  // once per second
    float k = 1.0f / (perfAcc.n * cyclesPerUs());
    bus::loopPerf.imuUs = perfAcc.imu * k;
    bus::loopPerf.filterUs = perfAcc.filt * k;
    bus::loopPerf.ahrsUs = perfAcc.ahrs * k;
    bus::loopPerf.pidUs = perfAcc.pid * k;
    bus::loopPerf.motorUs = perfAcc.motor * k;
    bus::loopPerf.totalUs = perfAcc.total * k;
    bus::loopPerf.maxTotalUs = perfAcc.maxTotal / cyclesPerUs();
    bus::loopPerf.periodUs = 1e6f / loopHz;
    bus::loopPerf.loadPct = bus::loopPerf.totalUs * loopHz * 1e-4f;
    bus::loopPerf.lateLoops = bus::loopPerf.lateLoops + perfAcc.late;
    perfAcc = PerfAcc{};
  }

  if (tick % 4 == 0) {
    AttitudeState st;
    const Ahrs& ah = loop.ahrs();
    st.q = ah.q();
    st.roll = ah.roll(); st.pitch = ah.pitch(); st.yaw = ah.yaw();
    st.rateDps = loop.gyroDps();
    st.accEarth = loop.takeAccEarthAverage();
    st.accBody = loop.accFiltered();
    loop.takeDelta(st.dAng, st.dVel, st.dt);
    st.vibration = loop.accVibration();
    st.accClips = loop.accClips();
    st.valid = imuOk && loop.gyroCalibrated();
    st.gyroCalibrated = loop.gyroCalibrated();
    st.timeUs = (uint32_t)now;
    bus::attitude.publish(st);
    MotorOutputs mo;
    memcpy(mo.out, out.motor, sizeof(mo.out));
    mo.armed = out.armed;
    bus::motors.publish(mo);
    PidDebug pd;
    for (int i = 0; i < 3; i++) {
      const RatePid& p = loop.pid(i);
      pd.p[i] = p.p; pd.i[i] = p.i; pd.d[i] = p.d; pd.f[i] = p.f;
      pd.dynNotchHz[i] = loop.dynNotch().centerHz(i);
    }
    pd.gyroRaw = loop.gyroUnfilteredDps();
    bus::pid.publish(pd);
  }
}

// =========================================================== control task
Commander commander;
void controlTask(void*) {
  static uint64_t lastUs = 0;
  uint64_t now = os::timeUs();
  float dt = lastUs ? (float)(now - lastUs) * 1e-6f : 0.004f;
  if (dt <= 0 || dt > 0.05f) dt = 0.004f;
  lastUs = now;
  commander.run((uint32_t)(now / 1000), dt);
  // hardware watchdog (STM32): armed once the flight loop has run for a while (a board whose loop never
  // starts stays reachable for the configurator), then fed only while the loop keeps counting
  static uint32_t seenLoops = 0;
  uint32_t loops = bus::fastLoops;
  if (loops != seenLoops) {
    if (!hal::watchdogRunning() && loops > 2000) hal::watchdogStart();
    hal::watchdogKick();
    seenLoops = loops;
  }
}

// =========================================================== sensors task
void sensorsTask(void*) {
  static uint32_t tick = 0, baroErr = 0, magErr = 0;
  static int refCount = 0;
  static float refSum = 0, refPa = 0;
  static BatteryMonitor battery;
  static uint32_t gen = 0;
  static Vec3 magMin, magMax;
  static bool magCalRunning = false;
  static bool wasArmed = false;
  static uint32_t armBeepUntil = 0, lastRcOkMs = 0;
  Config& c = bus::config;
  if (gen != bus::configGeneration) { gen = bus::configGeneration; battery.configure(c, 100.0f); }
  uint32_t now = os::timeMs();
  tick++;

  // ---- late detection: a baro / compass that was not ready at boot (module powered from the battery's
  // BEC while the board runs on USB, loose wire fixed): looked for every 2 s while disarmed ----
  // Cheap first: only addresses are pinged (a missing chip NACKs in ~0.1 ms); the full driver probe runs only
  // when something answers. A bus that times out instead (lines held low, no pull-ups) would block this
  // task: then the search backs off to every 30 s so the USB / web link keeps running.
  static uint32_t lastProbeMs = 0, probeEveryMs = 2000;
  if ((!drv_.baro || !drv_.mag) && !bus::status.get().armed && now - lastProbeMs >= probeEveryMs) {
    lastProbeMs = now;
    static const uint8_t baroAddr[] = {0x76, 0x77}, magAddr[] = {0x0D, 0x2C, 0x1E, 0x0E, 0x0C};
    uint32_t t0 = hal::millis();
    bool baroSeen = false, magSeen = false;
    if (!drv_.baro)
      for (uint8_t a : baroAddr) baroSeen |= hal::i2cProbe(a);
    if (!drv_.mag)
      for (uint8_t a : magAddr) magSeen |= hal::i2cProbe(a);
    bool stuck = hal::millis() - t0 > 20;
    if (stuck && probeEveryMs < 30000) bus::postMessage("I2C bus not responding: check SDA / SCL / pull-ups / power");
    probeEveryMs = stuck ? 30000 : 2000;
    if (!stuck && baroSeen && (drv_.baro = drv::probeBaro())) {
      sysInfo.baro = drv_.baro->name();
      refCount = 0;
      refSum = 0;
      bus::postMessage("Barometer found");
    }
    if (!stuck && magSeen && (drv_.mag = drv::probeMag())) {
      sysInfo.mag = drv_.mag->name();
      bus::postMessage("Compass found");
    }
  }
  // ---- barometer (50 Hz) ----
  if (drv_.baro && tick % 2 == 0) {
    float pa, t;
    if (drv_.baro->update(pa, t)) {
      baroErr = 0;
      if (refCount < 25) {
        refSum += pa;
        if (++refCount == 25) refPa = refSum / 25.0f;
      } else {
        BaroSample b{44330.0f * (1.0f - powf(pa / refPa, 0.190295f)), pa, t, true};
        bus::baro.publish(b);
      }
    } else if (++baroErr == 25 && refCount >= 25) {
      BaroSample b{0, 0, 0, false};
      bus::baro.publish(b);
    }
  }
  // ---- compass (50 Hz) + calibration ----
  if (drv_.mag && tick % 2 == 1) {
    Vec3 raw;
    if (drv_.mag->read(raw)) {
      magErr = 0;
      raw = drv::applyAlign(raw, (Align)c.mag_align);
      bool calActive = (int32_t)(bus::magCalUntilMs - now) > 0;
      // calibration samples: spread over the sphere (a new one only when it differs from the last)
      static Vec3 calPts[240];
      static int calN = 0;
      if (calActive) {
        if (!magCalRunning) { magMin = raw; magMax = raw; magCalRunning = true; calN = 0; }
        for (int i = 0; i < 3; i++) { magMin[i] = fminf(magMin[i], raw[i]); magMax[i] = fmaxf(magMax[i], raw[i]); }
        if (calN < 240 && (calN == 0 || (raw - calPts[calN - 1]).norm() > 0.06f * raw.norm())) calPts[calN++] = raw;
      } else if (magCalRunning) {
        magCalRunning = false;
        Vec3 half = (magMax - magMin) * 0.5f;
        float avg = (half.x + half.y + half.z) / 3.0f;
        Vec3 centre;
        float S[6];
        if (half.x > 0.05f && half.y > 0.05f && half.z > 0.05f && calN >= 40 && ellipsoidFit(calPts, calN, centre, S)) {
          // hard + soft iron (ellipsoid); scale so the corrected field keeps the min/max average size
          for (int i = 0; i < 3; i++) c.mag_bias[i] = centre[i];
          float k = 1.0f;
          {
            float sum = 0;
            for (int j = 0; j < calN; j++) {
              float b[3] = {c.mag_bias[0], c.mag_bias[1], c.mag_bias[2]}, d[3] = {S[0], S[1], S[2]}, o[3] = {S[3], S[4], S[5]};
              sum += magCorrect(calPts[j], b, d, o).norm();
            }
            k = sum > 0 ? avg * calN / sum : 1.0f;
          }
          for (int i = 0; i < 3; i++) { c.mag_scale[i] = S[i] * k; c.mag_offdiag[i] = S[3 + i] * k; }
          bus::configChanged();
          bus::postMessage("Compass calibrated (ellipsoid) - save to keep");
        } else if (half.x > 0.05f && half.y > 0.05f && half.z > 0.05f) {
          for (int i = 0; i < 3; i++) {
            c.mag_bias[i] = (magMax[i] + magMin[i]) * 0.5f;
            c.mag_scale[i] = avg / half[i];
            c.mag_offdiag[i] = 0;
          }
          bus::configChanged();
          bus::postMessage("Compass calibrated (min/max) - save to keep");
        } else {
          bus::postMessage("Compass calibration failed: rotate more");
        }
      }
      Vec3 f = magCorrect(raw, c.mag_bias, c.mag_scale, c.mag_offdiag);
      // motor / power-wire interference proportional to the current (compassmot)
      static float lastAmps = 0;
      if (c.mag_cmot[0] != 0 || c.mag_cmot[1] != 0 || c.mag_cmot[2] != 0) {
        BatteryState bs = bus::battery.get();
        lastAmps = bs.current > 0 ? bs.current : 0;
        for (int i = 0; i < 3; i++) f[i] -= c.mag_cmot[i] * lastAmps;
      }
      // ---- compassmot: motors ramp up (props on, craft held down), field change vs current ----
      static uint32_t cmStart = 0;
      static Vec3 cmBase, cmSxy;
      static float cmSxx = 0, cmMaxA = 0;
      static int cmBaseN = 0;
      if (bus::compassMotPct != 255) {
        if (!cmStart) { cmStart = now; cmBase = Vec3(); cmBaseN = 0; cmSxy = Vec3(); cmSxx = 0; cmMaxA = 0; }
        uint32_t t = now - cmStart;
        BatteryState bs = bus::battery.get();
        Vec3 fc0 = magCorrect(raw, c.mag_bias, c.mag_scale, c.mag_offdiag);  // without the old compensation
        float thr = t < 2000 ? 0.0f : t < 17000 ? (t - 2000) / 15000.0f * 0.6f : 0.6f;
        float mt[4] = {thr, thr, thr, thr};
        bool ok = bus::status.get().armed == false && bus::setMotorTest(mt, now);
        if (t < 2000) { cmBase += fc0; cmBaseN++; }
        else if (cmBaseN > 0) {
          Vec3 d = fc0 - cmBase * (1.0f / cmBaseN);
          float a = bs.current > 0 ? bs.current : 0;
          cmSxy += d * a;
          cmSxx += a * a;
          if (a > cmMaxA) cmMaxA = a;
        }
        bus::compassMotPct = (uint8_t)(t >= 20000 ? 100 : t / 200);
        if (!ok || t >= 20000) {
          float z[4] = {0, 0, 0, 0};
          bus::setMotorTest(z, now);
          if (ok && cmMaxA > 3.0f && cmSxx > 0) {
            for (int i = 0; i < 3; i++) c.mag_cmot[i] = constrainf(cmSxy[i] / cmSxx, -0.05f, 0.05f);
            bus::configChanged();
            char m[64];
            snprintf(m, sizeof(m), "compassmot: %.1f%% of field at %.0f A - save to keep",
                     (double)((Vec3(c.mag_cmot[0], c.mag_cmot[1], c.mag_cmot[2]) * cmMaxA).norm() * 100.0f /
                              fmaxf(cmBase.norm() / fmaxf((float)cmBaseN, 1.0f), 0.05f)), (double)cmMaxA);
            bus::postMessage(m);
          } else {
            bus::postMessage(ok ? "compassmot failed: no current reading (needs a current sensor or EDT)"
                                : "compassmot stopped");
          }
          bus::compassMotPct = 255;
          cmStart = 0;
        }
      }
      float n = f.norm();
      MagSample m{f, !calActive && n > 0.1f && n < 1.5f};
      bus::mag.publish(m);
    } else if (++magErr == 25) {
      MagSample m{Vec3(), false};
      bus::mag.publish(m);
    }
  }
  // ---- extended DShot telemetry (AM32 / Bluejay / BLHeli_32): temperature, voltage, current ----
  VehicleStatus st = bus::status.get();
  static uint32_t edtTryMs = 0;
  static uint8_t edtTries = 0;
  static EscTelemetry edtEsc{};
  bool edtFresh = false;
  float edtVolts = -1, edtAmps = -1;
  if (hal::motorsBidirActive()) {
    dshot::Edt e;
    bool any = false;
    float sumA = 0, maxV = -1;
    bool haveA = false;
    float pairs = (c.motor_poles >= 2 ? c.motor_poles : 14) * 0.5f;
    float erpm[4];
    hal::motorsErpm(erpm);
    for (int o = 0; o < 4; o++) {
      edtEsc.rpm[o] = erpm[o] >= 0 ? erpm[o] / pairs : -1.0f;
      static uint32_t edtFrames[4], edtChangeMs[4];
      if (!hal::motorsEdt(o, e)) { edtEsc.tempC[o] = -1; continue; }
      if (e.frames != edtFrames[o]) { edtFrames[o] = e.frames; edtChangeMs[o] = now; }
      if (now - edtChangeMs[o] > 1500) { edtEsc.tempC[o] = -1; continue; }  // ESC stopped sending EDT
      any = true;
      edtEsc.tempC[o] = e.tempC;
      edtEsc.lastUpdateMs[o] = now;
      if (e.volts > maxV) maxV = e.volts;
      if (e.amps >= 0) { sumA += e.amps; haveA = true; }
    }
    if (any) {
      edtFresh = true;
      edtVolts = maxV;
      edtAmps = haveA ? sumA : -1.0f;
      edtEsc.voltage = edtVolts;
      edtEsc.current = edtAmps;
      if (edtAmps > 0) edtEsc.mah += edtAmps * (0.01f / 3.6f);  // 100 Hz
      bus::esc.publish(edtEsc);
    } else if (c.dshot_edt && !st.armed && edtTries < 10 && now - edtTryMs > 2000 && now > 3000 &&
               !bus::escCmdPending && !bus::escPassthrough) {
      // ask the ESCs for EDT (DShot command 13) until they answer; ESCs that ignore it stay on eRPM
      edtTryMs = now;
      edtTries++;
      bus::escCmd = 13;
      bus::escCmd2 = 0;
      bus::escCmdMask = 0x0F;
      bus::escCmdPending = true;
    }
  }
  bool edtUse = edtFresh;

  // ---- battery ----
  float v = hal::adcVbatPin() * c.vbat_scale;
  float amps = c.cur_scale > 0 ? (hal::adcCurrentPin() - c.cur_offset) * c.cur_scale : -1.0f;
  // no divider / current sensor wired: the 4-in-1 ESC's own measurement (EDT) stands in
  if (edtUse && v < 2.0f && edtVolts > 0) v = edtVolts;
  if (edtUse && amps < 0 && edtAmps >= 0) amps = edtAmps;
  battery.update(v, amps, st.armed, now, 0.01f);
  bus::battery.publish(battery.state());

  // ---- LED & buzzer ----
  RcInput rc = bus::rc.get();
  bool rcOk = rc.everReceived && ageMs(now, rc.lastFrameMs) < c.fs_delay_ms;
  if (rcOk) lastRcOkMs = now;
  if (st.armed && !wasArmed) armBeepUntil = now + 300;
  wasArmed = st.armed;
  const BatteryState& b = battery.state();
  bool beep = (int32_t)(armBeepUntil - now) > 0;
  if (rcOk && boxOn(modeBoxes(c, rc.ch, RC_MAX_CH), ModeBox::BEEPER)) beep = (now / 250) % 2;
  if (st.armed && b.critical) beep = (now / 100) % 2;
  else if (st.armed && b.warn) beep = (now % 2000) < 150;
  if (st.failsafe) beep = (now / 200) % 2;
  if (bus::logicBeep) beep = (now / 300) % 2;
  if (!st.armed && rc.everReceived && !rcOk && now - lastRcOkMs > 10000) beep = (now % 2000) < 1000;  // lost model
  hal::buzzerSet(beep);
  bool led;
  if (st.armBlockers & ARM_CALIBRATING) led = (now / 50) % 2;
  else if (st.armed) led = true;
  else if (st.armBlockers) led = (now % 1000) < 100 || ((now % 1000) > 200 && (now % 1000) < 300);
  else led = (now / 500) % 2;
  hal::ledSet(led);
}

// =========================================================== io task
void ioTask(void*) {
  static uint32_t lastTelemMs = 0;
  static EscTelemetry esc{};
  static int8_t lastEscMotor = -1;
  uint32_t now = os::timeMs();
  int ch;
  if (rxPort != Port::COUNT) {
    bool got = false;
    for (int budget = 128; budget-- > 0 && (ch = hal::portRead(rxPort)) >= 0;) {
      got |= rx.feed((uint8_t)ch, now);
      const uint8_t* chunk;
      size_t cn;
      uint8_t origin;
      comms::MspFrame mf;
      if (rx.takeMsp(chunk, cn, origin) && crsfMsp.feed(chunk, cn, origin, mf)) mspCrsf.handle(mf, now);
    }
    if (got) bus::rc.publish(rx.rc());
    {
      const drv::RxParser::Counters& c = rx.counters();
      bus::rxLink.bytes = c.bytes;
      bus::rxLink.frames = c.frames;
      bus::rxLink.crcErrors = c.crcErrors;
    }
    static uint32_t lastMspTxMs = 0;
    if (crsfMsp.pending() && now - lastMspTxMs >= 8) {  // a Lua script waits for an answer: before telemetry
      lastMspTxMs = now;
      uint8_t frame[64];
      size_t n = crsfMsp.nextFrame(frame);
      if (n) hal::portWrite(rxPort, frame, n);
    } else if (rx.protocol() == RxProtocol::CRSF && now - lastTelemMs >= 40) {
      lastTelemMs = now;
      AttitudeState a = bus::attitude.get();
      BatteryState b = bus::battery.get();
      NavEstimate e = bus::nav.get();
      VehicleStatus st = bus::status.get();
      GpsData g = bus::gps.get();
      static char modeStr[12];
      snprintf(modeStr, sizeof(modeStr), "%s%s", flightModeName(st.mode), st.armed ? "" : "*");
      drv::RxTelemetry t{a.pitch, a.roll, e.yaw, b.voltage, b.current, b.mahDrawn, b.percent, e.vz, modeStr,
                         bus::gps.seq() != 0, g.lat, g.lon, g.groundSpeed, g.courseDeg, e.alt, g.sats};
      uint8_t frame[64];
      size_t n = rx.buildTelemetry(t, frame);
      if (n) hal::portWrite(rxPort, frame, n);
    }
  }
  bool gpsNew = gpsPort != Port::COUNT && gps.poll();
  if (gpsPort != Port::COUNT) {
    bus::gpsLink.port = (uint8_t)gpsPort;
    bus::gpsLink.baud = gps.baud();
    bus::gpsLink.frames = gps.frames();
    bus::gpsLink.proto = !strcmp(gps.protocol(), "UBX") ? 1 : !strcmp(gps.protocol(), "NMEA") ? 2 : 0;
  }
  if (gpsNew) {
    GpsData g = gps.data();
    g.timeMs = now;
    bus::gps.publish(g);
  }
  if (escPort != Port::COUNT) {
    int8_t m = bus::escTelemMotor;
    if (m != lastEscMotor) { escParser.reset(); lastEscMotor = m; }
    while ((ch = hal::portRead(escPort)) >= 0) {
      if (escParser.feed((uint8_t)ch) && m >= 0) {
        float poles = bus::config.motor_poles >= 2 ? bus::config.motor_poles : 14;
        esc.rpm[m] = escParser.erpm / (poles * 0.5f);
        esc.tempC[m] = escParser.tempC;
        esc.voltage = escParser.voltage;
        esc.current = escParser.current;
        esc.mah = escParser.mah;
        esc.lastUpdateMs[m] = now;
        bus::esc.publish(esc);
      }
    }
  }
  if (rangePort != Port::COUNT) {
    while ((ch = hal::portRead(rangePort)) >= 0)
      if (tfmini.feed((uint8_t)ch)) {
        RangeSample r{tfmini.distM, tfmini.distM > 0, (uint8_t)(tfmini.strength > 255 ? 255 : tfmini.strength)};
        bus::range.publish(r);
      }
  }
}

// =========================================================== telemetry task
void telemetryTask(void*) {
  static uint32_t lastOsdMs = 0;
  uint32_t now = os::timeMs();
  nav::persistPending();
  // first boot after a firmware update: keep the image once the flight tasks have run for 15 s;
  // if they never get there (stuck / crashing), go back to the previous firmware after 60 s
  static bool fwChecked = false;
  if (!fwChecked && now > 15000) {
    bool ok = bus::loopPerf.periodUs > 0 && os::taskCount() >= 4;
    if (ok) { hal::firmwareConfirm(); fwChecked = true; }
    else if (now > 60000 && hal::firmwareInfo().pendingVerify) hal::firmwareReject();
  }
  hal::logService(bus::status.get().armed);
  // USB console: configurator link + CLI + MSP + MAVLink on one stream
  int ch;
  int budget = 2048;
  if (fourWay.active()) {  // ESC passthrough owns the console until "exit"
    while (budget-- > 0 && (ch = hal::portRead(Port::CONSOLE)) >= 0) fourWay.feed((uint8_t)ch, now);
    fourWay.update(now);
    bus::cliActiveUntilMs = now + 3000;
    budget = 0;
  }
  // The configurator UART (WiFi bridge / Bluetooth) carries the same stream as USB. One of the two is
  // served at a time: the other side takes over once the current one has been quiet for 1.5 s (and no
  // configurator transfer is running); bytes from the side not served are dropped.
  static Port conPort = Port::CONSOLE;
  static uint32_t conRxMs = 0;
  static bool conSeen = false;  // anything received yet on the console in use
  for (int side = 0; side < 2; side++) {
    Port src = side == 0 ? Port::CONSOLE : cfgPort;
    if (src == Port::COUNT || (side == 0 && budget <= 0)) continue;
    int sb = side == 0 ? budget : 2048;
    while (sb-- > 0 && (ch = hal::portRead(src)) >= 0) {
      if (src != conPort) {
        if ((conSeen && now - conRxMs < 1500) || aqlink.active(now)) continue;  // the other side is in use
        conPort = src;  // switch every console protocol over
        aqlink.begin(src);
        cli.begin(src, &sysInfo);
        mav[(int)Port::CONSOLE].begin(src, true);
        msp[(int)Port::CONSOLE].begin(src);
        msp[(int)Port::CONSOLE].setFourWay(src == Port::CONSOLE ? &fourWay : nullptr);  // ESC passthrough: USB only
      }
      conRxMs = now;
      conSeen = true;
      uint8_t b = (uint8_t)ch;
      int ci = (int)Port::CONSOLE;
      bool aq = aqlink.feed(b, now);
      if (!aq) {
        mav[ci].feed(b, now);
        msp[ci].feed(b, now);
      }
      cli.feed(b, now, aq || aqlink.active(now) || mav[ci].seenRecently(now) || msp[ci].seenRecently(now));
    }
  }
  aqlink.update(now);
  mav[(int)Port::CONSOLE].update(now, false);
  for (int i = 0; i < NPORTS; i++) {
    if (!configurablePort(i) || !hal::portExists((Port)i)) continue;
    if (portFn[i] == PortFn::MAVLINK || portFn[i] == PortFn::MAVLINK_LOW) mav[i].update(now);
    else if (portFn[i] == PortFn::MSP || portFn[i] == PortFn::DISPLAYPORT) msp[i].update(now);
  }
  bool armedNow = bus::status.get().armed;
  vtxDrv.update(bus::config, armedNow, now);
  static uint32_t lastLedMs = 0;
  if (ledsOn && now - lastLedMs >= 50) {  // 20 Hz
    lastLedMs = now;
    static uint8_t grb[32 * 3];
    int n = bus::config.led_count > 32 ? 32 : bus::config.led_count;
    ledColors(bus::config, bus::status.get(), bus::battery.get(), now, grb, n);
    hal::ledStripShow(grb, (uint8_t)n);
  }
  if (osdActive && now - lastOsdMs >= 100) {
    lastOsdMs = now;
    osdRenderer.render(now);
  }
}

// =========================================================== logger task
void loggerTask(void*) { blackbox.update((uint32_t)os::timeUs()); }
// SD card writer: lowest priority, may block 100-250 ms inside a card write (the queue absorbs it)
void sdTask(void*) { sdlog::service(os::timeMs()); }

// =========================================================== web task (ESP32)
void webTask(void*) { web::poll(); }

uint32_t defaultBaud(PortFn f) {
  switch (f) {
    case PortFn::MAVLINK: return 57600;
    case PortFn::MAVLINK_LOW: return 9600;  // Ebyte E32 / most LoRa UART modules out of the box
    case PortFn::BLACKBOX: return 921600;
    case PortFn::CONFIGURATOR: return 460800;
    default: return 115200;
  }
}

}  // namespace

const Drivers& drivers() { return drv_; }

void loadConfig() {
  // an older firmware's config is shorter: the read may report "short", the header tells
  memset(&bus::config, 0, sizeof(Config));
  bool ok = hal::storageLoad(hal::Slot::CONFIG, &bus::config, sizeof(Config));
  if (!(ok && configValid(bus::config))) {
    if (configMigrate(bus::config)) bus::postMessage("Settings kept from the previous firmware");
    else configDefaults(bus::config);
    hal::storageSave(hal::Slot::CONFIG, &bus::config, sizeof(Config));
  }
  if (bus::config.loop_hz < 500 || bus::config.loop_hz > AQ_LOOP_HZ_MAX) bus::config.loop_hz = 1000;
  nav::load();
  nav::rallyLoad();
  nav::fenceLoad();
}

void start(const Drivers& d, bool configureGps) {
  drv_ = d;
  bus::imuSecondPresent = d.imu2 != nullptr;
  Config& c = bus::config;
  loopHz = c.loop_hz;
  sysInfo = {d.imu ? d.imu->name() : nullptr, d.baro ? d.baro->name() : nullptr, d.mag ? d.mag->name() : nullptr,
             VERSION};

  // ---- serial ports ----
  for (int i = 0; i < NPORTS; i++) portFn[i] = PortFn::NONE;
  for (int i = 0; i < NPORTS; i++) portFn[i] = configurablePort(i) ? (PortFn)portFnOf(c, i) : PortFn::NONE;
  creds::load();
  bool wifiUp = (c.wifi_enable || c.web_enable) && hal::portExists(Port::UDP) &&
                hal::wifiStart(creds::ssid(), creds::wifiPass());
  rxPort = findPort(PortFn::RX);
  gpsPort = findPort(PortFn::GPS);
  escPort = findPort(PortFn::ESC_TELEM);
  rangePort = findPort(PortFn::RANGEFINDER);
  cfgPort = findPort(PortFn::CONFIGURATOR);
  for (int i = 0; i < NPORTS; i++) {
    Port p = (Port)i;
    if (!configurablePort(i) || !hal::portExists(p) || portFn[i] == PortFn::NONE || portFn[i] == PortFn::GPS || portFn[i] == PortFn::VTX_SA ||
        portFn[i] == PortFn::VTX_TRAMP)
      continue;
    if (portFn[i] == PortFn::RX) {
      rx.begin((RxProtocol)c.rx_protocol);
      mspCrsf.begin(p);
      mspCrsf.setReplyHook([](void* ctx, const comms::MspFrame& req, const uint8_t* pl, uint16_t len, bool err) {
        static_cast<comms::CrsfMsp*>(ctx)->respond(req, pl, len, err);
      }, &crsfMsp);
      hal::portOpen(p, rx.baud(), rx.needs8E2() ? hal::UartFormat::F8E2 : hal::UartFormat::F8N1, rx.inverted());
      continue;
    }
    uint32_t baud = portBaudOf(c, i);
    hal::portOpen(p, baud ? baud : defaultBaud(portFn[i]));
    if (portFn[i] == PortFn::MAVLINK || portFn[i] == PortFn::MAVLINK_LOW) {
      mav[i].begin(p);
      mav[i].setLowRate(portFn[i] == PortFn::MAVLINK_LOW);
    }
    if (portFn[i] == PortFn::MSP || portFn[i] == PortFn::DISPLAYPORT) msp[i].begin(p);
    msp[i].setDisplayPort(portFn[i] == PortFn::DISPLAYPORT);
  }
  if (gpsPort != Port::COUNT && configureGps) gps.begin(gpsPort);
  else if (gpsPort != Port::COUNT) { hal::portOpen(gpsPort, 115200); gps.attach(gpsPort); }
  mav[(int)Port::CONSOLE].begin(Port::CONSOLE, true);
  msp[(int)Port::CONSOLE].begin(Port::CONSOLE);
  cli.begin(Port::CONSOLE, &sysInfo);
  aqlink.begin(Port::CONSOLE);
  fourWay.begin(Port::CONSOLE);
  msp[(int)Port::CONSOLE].setFourWay(&fourWay);

  // ---- video transmitter: SmartAudio / Tramp port, or MSP (digital, DisplayPort port) ----
  Port vtxPort = findPort(PortFn::VTX_SA);
  if (vtxPort != Port::COUNT) vtxDrv.begin(vtx::Type::SMARTAUDIO, vtxPort);
  else if ((vtxPort = findPort(PortFn::VTX_TRAMP)) != Port::COUNT) vtxDrv.begin(vtx::Type::TRAMP, vtxPort);
  for (int i = 0; i < NPORTS; i++)
    msp[i].setVtxType(vtxDrv.type() == vtx::Type::SMARTAUDIO ? 3 : vtxDrv.type() == vtx::Type::TRAMP ? 4 : 0xFF);


  // ---- OSD ----
  if (c.osd_backend == 1) {
    osdRenderer.begin(&osdMax, c.osd_pal != 0);
    osdActive = true;
  } else if (c.osd_backend == 2) {
    Port dp = findPort(PortFn::DISPLAYPORT);
    if (dp != Port::COUNT) {
      static osd::DisplayPort dpBackend(&msp[(int)dp]);
      osdDp = &dpBackend;
      osdRenderer.begin(osdDp, c.osd_pal != 0);
      osdActive = true;
    }
  }

  // ---- blackbox (SD card first: the backend checks that the card side is usable) ----
  sdlog::begin(c);
  Port bbPort = findPort(PortFn::BLACKBOX);
  blackbox.begin(c.bb_backend == 1 && bbPort == Port::COUNT ? 0 : c.bb_backend, bbPort, c.bb_rate_hz);

  // ---- loop trigger: the IMU's data-ready pin (the loop rate becomes ODR / divider, e.g. 6400 / 6) ----
  if (c.imu_sync && d.imu && hal::imuIntPin() >= 0) {
    uint8_t div = 1;
    uint32_t odr = d.imu->enableDataReady(loopHz, &div);
    if (odr && div) {
      loopHz = (uint16_t)((odr + div / 2) / div);
      os::setDataReadyTrigger(hal::imuIntPin(), odr, div);
    }
  }

  // ---- motors & modules ----
  hal::motorsInit((MotorProtocol)c.motor_protocol, loopHz, c.dshot_bidir != 0);
  // LED strip (after the motors: on the ESP32 bidirectional DShot takes every RMT channel)
  ledsOn = c.led_pin && hal::ledStripBegin(c.led_pin, c.led_count > 32 ? 32 : c.led_count);
  commander.init();

  // ---- tasks (fast task first: arming checks read its statistics as task 0) ----
  os::createTask({"fast", fastTask, nullptr, (1000000u + loopHz / 2) / loopHz, 6, 4096, 1, true});
  // io runs the CLI (diff / save) and MAVLink mission uploads (flash writes): 6 kB on ESP32
  os::createTask({"io", ioTask, nullptr, 2000, 5, IO_STACK, 0, false});
  // control (EKF ~1.3 ms per run, navigation, modes) on core 0: core 1 is left to the fast loop alone,
  // which then never waits behind a 4 ms EKF slice. On core 0 it outranks sensors, telemetry, logger and
  // web; WiFi (priority 23) and io still preempt it, its 4 ms period leaves room for both.
  os::createTask({"control", controlTask, nullptr, 4000, 4, 8192, 0, false});
  os::createTask({"sensors", sensorsTask, nullptr, 10000, 3, 4096, 0, false});
  os::createTask({"telemetry", telemetryTask, nullptr, 10000, 2, 8192, 0, false});
  if (c.bb_rate_hz) os::createTask({"logger", loggerTask, nullptr, 1000000u / c.bb_rate_hz, 1, 4096, 0, false});
  if (sdlog::enabled()) os::createTask({"sdcard", sdTask, nullptr, 10000, 1, 6144, 0, false});
  web::begin(&sysInfo);
  // the web configurator runs whenever the access point is up (password protected): wifi_enable alone
  // (MAVLink over WiFi) used to bring up an access point without a page at 192.168.4.1
  if ((c.web_enable || c.wifi_enable) && wifiUp && web::start())
    os::createTask({"web", webTask, nullptr, 50000, 1, 4096, 0, false});
}

void firmwareSetup() {
  hal::init();
  hal::delayMs(300);
  loadConfig();
  Drivers d;
  d.imu = drv::probeImu(bus::config.loop_hz);
  d.imu2 = bus::config.imu_dual ? drv::probeImu2(bus::config.loop_hz, d.imu) : nullptr;
  d.baro = drv::probeBaro();
  d.mag = drv::probeMag();
  if (!d.baro || !d.mag) {  // weak pull-ups / long wires: try the bus again at 100 kHz
    int before = (d.baro != nullptr) + (d.mag != nullptr);
    hal::i2cSetClock(100000);
    if (!d.baro) d.baro = drv::probeBaro();
    if (!d.mag) d.mag = drv::probeMag();
    if ((d.baro != nullptr) + (d.mag != nullptr) > before) bus::postMessage("I2C runs at 100 kHz: check the pull-ups");
    else hal::i2cSetClock(400000);  // nothing more at the slow speed either
  }
  start(d, true);
  os::startScheduler();  // STM32: never returns
}

void firmwareLoop() {
#if defined(ESP32)
  vTaskDelete(nullptr);  // all work happens in the tasks
#endif
}

}  // namespace app
