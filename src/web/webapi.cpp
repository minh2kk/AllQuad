#include "../common/size_opt.h"  // -Os on ESP32 (not flight critical)
#include "webapi.h"

#include "web.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../bus/bus.h"
#include "../estimation/nav_estimator.h"
#include "credentials.h"
#include "../hal/hal.h"
#include "../hal/pinmap.h"
#include "../logging/sdlog.h"
#include "../navigation/mission.h"

namespace web {

using namespace fc;

static const comms::SystemInfo* sys = nullptr;

void JsonOut::flush() {
  if (n_ && sink_) sink_(buf_, n_, ctx_);
  n_ = 0;
}

void JsonOut::raw(const char* s) {
  inKey_ = false;
  while (*s) put(*s++);
}

void JsonOut::str(const char* s) {
  inKey_ = false;
  put('"');
  for (; s && *s; s++) {
    char c = *s;
    if (c == '"' || c == '\\') { put('\\'); put(c); }
    else if ((unsigned char)c < 0x20) put(' ');
    else put(c);
  }
  put('"');
}

// Fixed-point formatting without printf: the status JSON has ~150 numbers and newlib's %f costs
// tens of microseconds each on the ESP32 (no double-precision FPU). Same output as "%.*f" with the
// trailing zeros of the fraction removed.
void JsonOut::num(double v, int decimals) {
  if (v != v || v > 1e15 || v < -1e15) v = 0;  // NaN / inf are not valid JSON
  if (decimals < 0) decimals = 0;
  if (decimals > 9) decimals = 9;
  static const uint32_t P10[10] = {1, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000, 1000000000};
  bool neg = v < 0;
  double a = neg ? -v : v;
  uint64_t scaled = (uint64_t)(a * P10[decimals] + 0.5);
  uint64_t ip = scaled / P10[decimals];
  uint32_t fp = (uint32_t)(scaled % P10[decimals]);
  char b[40];
  char* e = b + sizeof(b);
  *--e = 0;
  // fraction, trailing zeros dropped
  int fd = decimals;
  while (fd > 0 && fp % 10 == 0) { fp /= 10; fd--; }
  if (fd > 0) {
    for (int k = 0; k < fd; k++) { *--e = (char)('0' + fp % 10); fp /= 10; }
    *--e = '.';
  }
  do { *--e = (char)('0' + ip % 10); ip /= 10; } while (ip);
  if (neg && scaled) *--e = '-';  // no "-0"
  raw(e);
}

void begin(const comms::SystemInfo* info) { sys = info; }
bool armed() { return bus::status.get().armed; }

void paramsJson(Sink s, void* ctx) {
  JsonOut j(s, ctx);
  j.begin('{');
  j.key("armed"); j.raw(armed() ? "true" : "false");
  j.key("version"); j.str(sys ? sys->version : "?");
  j.key("board"); j.str(hal::platformName());
  uint32_t ports = 0;  // bit = hal::Port index that exists on this board
  for (int i = 0; i < (int)hal::Port::COUNT; i++) if (hal::portExists((hal::Port)i)) ports |= 1u << i;
  j.key("ports"); j.num(ports, 0);
  j.key("portpins");  // UART instance and pins per port index, for the wiring hints of the Ports tab
  j.begin('{');
  for (int i = 0; i < (int)hal::Port::COUNT; i++) {
    const char* l = hal::portLabel((hal::Port)i);
    if (!l[0]) continue;
    char k[4] = {(char)('0' + i), 0};
    j.key(k); j.str(l);
  }
  j.end('}');
  {  // board pin map (src/hal/board.h): [pin, group, arg, what] per used pin, for the pin map of the Ports tab
    hal::PinUse pm[hal::PINMAP_MAX];  // on the stack: no RAM kept for a rare request (F405 is tight)
    int n = hal::pinMap(pm, hal::PINMAP_MAX);
    j.key("pinmap");
    j.begin('{');
    j.key("board"); j.str(hal::pinMapBoard());
    j.key("pins");
    j.begin('[');
    for (int i = 0; i < n; i++) {
      char b[8];
      j.begin('[');
      j.comma(); j.str(hal::pinText(pm[i].pin, b, sizeof(b)));
      j.comma(); j.str(hal::pinGroupName(pm[i].group));
      j.comma(); j.num(pm[i].arg, 0);
      j.comma(); j.str(pm[i].what);
      j.end(']');
    }
    j.end(']');
    j.end('}');
  }
  {  // SD card modes this board can drive, fixed wiring, default pins
    uint8_t def[6];
    hal::sdDefaultPins(def);
    j.key("sd");
    j.begin('{');
    j.key("modes"); j.num(hal::sdModes(), 0);
    j.key("fixed"); j.raw(hal::sdPinsFixed() ? "true" : "false");
    j.key("def");
    j.begin('[');
    for (int i = 0; i < 6; i++) { j.comma(); j.num(def[i], 0); }
    j.end(']');
    j.end('}');
  }
  j.key("params");
  j.begin('[');
  for (int i = 0; i < paramCount(); i++) {
    const ParamInfo& p = paramInfo(i);
    j.begin('{');
    j.key("n"); j.str(p.name);
    j.key("v"); j.num(paramGet(bus::config, i), p.type == PType::F32 ? 4 : 0);
    j.key("min"); j.num(p.min, 4);
    j.key("max"); j.num(p.max, 4);
    j.key("f"); j.raw(p.type == PType::F32 ? "1" : "0");
    if (p.reboot) { j.key("r"); j.raw("1"); }
    j.end('}');
  }
  j.end(']');
  j.end('}');
}

static void vec(JsonOut& j, const char* k, const float* v, int n, int dec) {
  j.key(k);
  j.begin('[');
  for (int i = 0; i < n; i++) { j.comma(); j.num(v[i], dec); }
  j.end(']');
}

// status parts that change slowly: sent with every HTTP poll, over the WebSocket once per second
static void slowStatus(JsonOut& j) {
  j.key("tasks");
  j.begin('[');
  for (int i = 0; i < os::taskCount(); i++) {
    os::TaskStats t = os::taskStats(i);
    j.begin('{');
    j.key("n"); j.str(t.name);
    j.key("hz"); j.num(t.periodUs ? 1000000.0 / t.periodUs : 0, 0);
    j.key("us"); j.num(t.lastUs, 0);
    j.key("max"); j.num(t.maxUs, 0);
    j.key("over"); j.num(t.overruns, 0);
    j.key("stk"); j.num(t.stackFree, 0);
    j.end('}');
  }
  j.end(']');
  {
    const volatile bus::LoopPerf& p = bus::loopPerf;
    float pf[9] = {p.imuUs, p.filterUs, p.ahrsUs, p.pidUs, p.motorUs, p.totalUs, p.maxTotalUs, p.periodUs, p.loadPct};
    vec(j, "perf", pf, 9, 1);
    j.key("late"); j.num(p.lateLoops, 0);
  }
  // per-UART counters (Ports tab): [port, rx bytes, tx bytes, overruns, framing, noise, dropped]
  j.key("ports");
  j.begin('[');
  for (int i = 0; i < (int)hal::Port::COUNT; i++) {
    hal::PortStats ps;
    if ((hal::Port)i == hal::Port::UDP || (hal::Port)i == hal::Port::CONSOLE || !hal::portStats((hal::Port)i, ps)) continue;
    uint32_t v[7] = {(uint32_t)i, ps.rxBytes, ps.txBytes, ps.overruns, ps.framing, ps.noise, ps.dropped};
    j.begin('[');
    for (int k = 0; k < 7; k++) { j.comma(); j.num(v[k], 0); }
    j.end(']');
  }
  j.end(']');
  {
    uint32_t rc[3] = {bus::rxLink.bytes, bus::rxLink.frames, bus::rxLink.crcErrors};
    j.key("rxc");
    j.begin('[');
    for (int k = 0; k < 3; k++) { j.comma(); j.num(rc[k], 0); }
    j.end(']');
  }
  j.key("sensors");
  j.begin('{');
  j.key("imu"); j.str(sys && sys->imu ? sys->imu : "");
  j.key("baro"); j.str(sys && sys->baro ? sys->baro : "");
  j.key("mag"); j.str(sys && sys->mag ? sys->mag : "");
  j.key("gps"); j.raw(bus::gps.seq() ? "true" : "false");
  j.end('}');
  j.key("log");
  j.begin('{');
  j.key("size"); j.num(hal::logSize(), 0);
  j.key("cap"); j.num(hal::logCapacity(), 0);
  j.key("rate"); j.num(bus::config.bb_rate_hz, 0);
  j.key("busy"); j.raw(hal::logBusy() ? "true" : "false");
  j.end('}');
#if defined(ARDUINO) && defined(ESP32)
  // WiFi access point of the FC itself (ESP32 only): STM32 boards have no WiFi, nothing to secure
  j.key("ssid"); j.str(creds::ssid());
#endif
  // last reset (hal::resetReason numbering), crash record and watchdog (System tab, pre-flight)
  j.key("rr"); j.num(hal::resetReason(), 0);
  j.key("wdt"); j.raw(hal::watchdogRunning() ? "true" : "false");
  {
    hal::CrashInfo ci;
    if (hal::lastCrash(ci)) {
      j.key("crash");
      j.begin('{');
      j.key("kind"); j.str(hal::crashKindName(ci.kind));
      j.key("where"); j.str(ci.where);
      j.key("t"); j.num(ci.uptimeMs, 0);
      j.key("ago"); j.num(ci.bootsAgo, 0);
      j.key("n"); j.num(ci.count, 0);
      char hex[12];
      snprintf(hex, sizeof(hex), "0x%08lX", (unsigned long)ci.pc);
      j.key("pc"); j.str(hex);
      snprintf(hex, sizeof(hex), "0x%08lX", (unsigned long)ci.lr);
      j.key("lr"); j.str(hex);
      snprintf(hex, sizeof(hex), "0x%08lX", (unsigned long)ci.cfsr);
      j.key("cfsr"); j.str(hex);
      j.end('}');
    }
  }
  {
    hal::FirmwareInfo fw = hal::firmwareInfo();
    j.key("fw");
    j.begin('{');
    j.key("version"); j.str(sys ? sys->version : "?");
    j.key("slot"); j.str(fw.running);
    j.key("ota"); j.raw(fw.otaSupported ? "true" : "false");
    if (fw.otaSigned) { j.key("sig"); j.raw("true"); }
    j.key("pending"); j.raw(fw.pendingVerify ? "true" : "false");
    j.end('}');
  }
#if defined(ARDUINO) && defined(ESP32)
  j.key("defpw"); j.raw(creds::usingDefaultPassword() ? "true" : "false");
#endif
}

// RPM / ESC temperature per mixer motor (-1 = nothing from that ESC); true when bidirectional DShot runs
static bool escRpmTemp(const EscTelemetry& e, float rpm[4], float temp[4]) {
  uint32_t now = os::timeMs();
  bool bidir = hal::motorsBidirActive();
  const Config& c = bus::config;
  for (int i = 0; i < 4; i++) {
    // telemetry arrays are per output; the UI lists mixer motors (motor_out remaps them)
    int o = c.motor_out[i] >= 1 && c.motor_out[i] <= 4 ? c.motor_out[i] - 1 : i;
    bool fresh = bus::esc.seq() && ageMs(now, e.lastUpdateMs[o]) < 500;
    rpm[i] = bidir ? bus::bidirRpm[i] : (fresh ? e.rpm[o] : -1);
    temp[i] = fresh ? e.tempC[o] : -1;
  }
  return bidir;
}

void statusJson(Sink s, void* ctx, bool full) {
  VehicleStatus st = bus::status.get();
  AttitudeState a = bus::attitude.get();
  ControlSetpoint sp = bus::setpoint.get();
  MotorOutputs m = bus::motors.get();
  BatteryState b = bus::battery.get();
  RcInput rc = bus::rc.get();
  GpsData g = bus::gps.get();
  NavEstimate e = bus::nav.get();
  PidDebug pd = bus::pid.get();
  JsonOut j(s, ctx);
  // light frames (WebSocket, 10 Hz) carry the GPS / estimator blocks every other time and the
  // autotune block only while it runs: ~1/3 fewer bytes on the air; the page keeps the last values
  static uint8_t lightN = 0;
  const bool mid = full || (++lightN & 1);
  j.begin('{');
  j.key("t"); j.num(os::timeMs(), 0);
  if (bus::webGcs) {
    if (!st.armed) bus::webGcs = false;  // one flight only: a later MAVLink GCS loss must not be masked
    else bus::gcsHeartbeatMs = os::timeMs() | 1;  // the page that sent GUIDED is still here
  }
  j.key("armed"); j.raw(st.armed ? "true" : "false");
  j.key("mode"); j.str(flightModeName(st.mode));
  j.key("nav"); j.str(navPhaseName(st.navPhase));
  j.key("failsafe"); j.raw(st.failsafe ? "true" : "false");
  // calibration progress / logic: a6 = sides done bitmask (-1 idle), cm = compassmot % (-1 idle)
  j.key("a6"); j.num(bus::acc6Active ? (int)bus::acc6Mask : -1, 0);
  j.key("cm"); j.num(bus::compassMotPct == 255 ? -1 : (int)bus::compassMotPct, 0);
  j.key("lg"); j.num(bus::logicResults, 0);
  if (full || st.atState) {
  j.key("at");
  j.begin('{');
  j.key("state"); j.num(st.atState, 0);
  j.key("axis"); j.num(st.atAxis, 0);
  j.key("round"); j.num(st.atRound, 0);
  j.key("ov"); j.num(st.atOvershoot, 3);
  j.key("rise"); j.num(st.atRiseMs, 0);
  j.key("step"); j.num(st.atStep, 0);
  j.key("lag"); j.num(st.atLagMs, 1);
  j.key("ss"); j.num(st.atSteadyErr, 3);
  j.end('}');
  }
  j.key("blockers");
  j.begin('[');
  for (int i = 0; i < ARM_COUNT_BITS; i++)
    if (st.armBlockers & (1u << i)) { j.comma(); j.str(armCheckName(i)); }
  j.end(']');
  float att[3] = {a.roll * RAD2DEG, a.pitch * RAD2DEG, e.yaw * RAD2DEG};  // yaw: true heading
  vec(j, "att", att, 3, 1);
  float rate[3] = {a.rateDps.x, a.rateDps.y, a.rateDps.z};
  vec(j, "rate", rate, 3, 1);
  vec(j, "sp", sp.rateDps, 3, 1);
  j.key("thr"); j.num(sp.throttle, 3);
  float mo[4];
  for (int i = 0; i < 4; i++) mo[i] = m.armed && m.out[i] > 0 ? m.out[i] : 0;
  vec(j, "motors", mo, 4, 3);
  if (bus::motorTestActive(os::timeMs())) {  // motor test from the configurator is running
    float t[4] = {bus::motorTest[0], bus::motorTest[1], bus::motorTest[2], bus::motorTest[3]};
    vec(j, "mtest", t, 4, 2);
  }
  {  // ESC telemetry for the overview motor card (only when an ESC reports)
    float er[4], et[4];
    escRpmTemp(bus::esc.get(), er, et);
    if (er[0] >= 0 || er[1] >= 0 || er[2] >= 0 || er[3] >= 0) {
      vec(j, "erpm", er, 4, 0);
      vec(j, "etmp", et, 4, 0);
    }
  }
  vec(j, "notch", pd.dynNotchHz, 3, 0);
  float acc[3] = {a.accBody.x, a.accBody.y, a.accBody.z};
  vec(j, "acc", acc, 3, 2);
  j.key("bat");
  j.begin('{');
  j.key("pin"); j.num(bus::adcVbatV, 3);      // V on the ADC pins (wiring / calibration helpers)
  j.key("cpin"); j.num(bus::adcCurV, 3);
  j.key("v"); j.num(b.voltage, 2);
  j.key("a"); j.num(b.current, 1);
  j.key("mah"); j.num(b.mahDrawn, 0);
  j.key("cells"); j.num(b.cells, 0);
  j.key("pct"); j.num(b.percent, 0);
  j.key("warn"); j.raw(b.warn ? "true" : "false");
  j.end('}');
  j.key("rc");
  j.begin('[');
  for (int i = 0; i < RC_MAX_CH; i++) { j.comma(); j.num(rc.ch[i], 0); }
  j.end(']');
  j.key("boxes"); j.num(modeBoxes(bus::config, rc.ch, RC_MAX_CH), 0);
  j.key("lq"); j.num(rc.linkQuality, 0);
  j.key("rxok"); j.raw(rc.everReceived && ageMs(os::timeMs(), rc.lastFrameMs) < 500 ? "true" : "false");
  if (mid) {
  j.key("gps");
  j.begin('{');
  j.key("fix"); j.raw(g.fix3d ? "true" : "false");
  j.key("sats"); j.num(g.sats, 0);
  j.key("lat"); j.num(g.lat, 7);
  j.key("lon"); j.num(g.lon, 7);
  j.key("hacc"); j.num(g.hAcc, 1);
  j.end('}');
  j.key("est");
  j.begin('{');
  j.key("alt"); j.num(e.alt, 2);
  j.key("vz"); j.num(e.vz, 2);
  j.key("agl"); j.num(e.aglValid ? e.agl : -1, 2);
  j.key("n"); j.num(e.n, 2);
  j.key("e"); j.num(e.e, 2);
  j.key("pos"); j.raw(e.posValid ? "true" : "false");
  j.key("home"); j.raw(e.homeSet ? "true" : "false");
  j.key("dist"); j.num(st.homeDistance, 1);
  j.key("vn"); j.num(e.vn, 2);
  j.key("ve"); j.num(e.ve, 2);
  j.key("yaw"); j.num(e.yaw * RAD2DEG, 1);
  j.key("hsrc"); j.num(e.headingSrc, 0);
  j.key("hok"); j.raw(e.headingOk ? "true" : "false");
  float wind[2] = {e.windN, e.windE};
  vec(j, "wind", wind, 2, 2);
  j.key("hover"); j.num(e.hoverThr, 3);
  float tgt[3] = {e.tgtN, e.tgtE, e.tgtAlt};
  vec(j, "tgt", tgt, 3, 2);
  j.key("aglhold"); j.raw(e.aglHold ? "true" : "false");
  j.key("glitch"); j.raw(e.gpsGlitch ? "true" : "false");
  j.key("glitches"); j.num(e.glitchCount, 0);
  j.key("eph"); j.num(e.eph, 1);
  j.key("mi"); j.num(st.missionIndex, 0);
  j.key("mc"); j.num(st.missionCount, 0);
  j.end('}');
  }  // mid
  j.key("warn"); j.num(st.warnings, 0);
  j.key("rpml"); j.num(bus::rpmLostMask, 0);
  if (bus::sensorLink.port < (uint8_t)hal::Port::COUNT) {  // MSP sensor port: [port, bytes, MSP frames, Micolink frames, MAVLink starts]
    uint32_t sl[5] = {bus::sensorLink.port, bus::sensorLink.bytes, bus::sensorLink.msp, bus::sensorLink.mico, bus::sensorLink.mav};
    j.key("snsl");
    j.begin('[');
    for (int k = 0; k < 5; k++) { j.comma(); j.num(sl[k], 0); }
    j.end(']');
  }
  if (bus::gpsLink.port < (uint8_t)hal::Port::COUNT) {  // GPS link: [port, baud, valid frames, protocol 1 UBX / 2 NMEA]
    uint32_t gl[4] = {bus::gpsLink.port, bus::gpsLink.baud, bus::gpsLink.frames, bus::gpsLink.proto};
    j.key("gpsl");
    j.begin('[');
    for (int k = 0; k < 4; k++) { j.comma(); j.num(gl[k], 0); }
    j.end(']');
  }
  j.key("vibe"); j.num(a.vibration, 1);
  {
    EkfReport k = bus::ekf.get();
    j.key("ekf");
    j.begin('{');
    j.key("on"); j.raw(k.active ? "true" : "false");
    if (k.active) {
      float tr[6] = {k.trGpsPos, k.trGpsVel, k.trHeight, k.trYaw, k.trMag, k.trFlow};
      vec(j, "tr", tr, 6, 2);
      j.key("f"); j.num(k.faults, 0);
      float sd[4] = {k.posStd, k.velStd, k.hgtStd, k.yawStdDeg};
      vec(j, "sd", sd, 4, 2);
      j.key("hs"); j.num(k.heightSrc, 0);
      float rs[3] = {(float)k.posResets, (float)k.velResets, (float)k.filterResets};
      vec(j, "rs", rs, 3, 0);
      vec(j, "gb", k.gyroBias, 3, 3);
      vec(j, "ab", k.accBias, 3, 3);
    }
    j.end('}');
  }
  j.key("full"); j.raw(full ? "true" : "false");
  char msg[64];
  bus::readMessage(msg, sizeof(msg));
  j.key("msg"); j.str(msg);
  if (full) slowStatus(j);
  j.end('}');
}

// mission, rally points and fence for the map (local metres from home when home is known)
void missionJson(Sink s, void* ctx) {
  NavEstimate e = bus::nav.get();
  JsonOut j(s, ctx);
  j.begin('{');
  j.key("home"); j.raw(e.homeSet ? "true" : "false");
  j.key("lat"); j.num(e.homeLat, 7);
  j.key("lon"); j.num(e.homeLon, 7);
  auto items = [&](const char* key, int n, bool rally) {
    j.key(key);
    j.begin('[');
    for (int i = 0; i < n; i++) {
      nav::MissionItem it;
      if (rally ? !nav::rallyGet(i, it) : !nav::get(i, it)) continue;
      float nn = 0, ee = 0;
      bool pos = (it.lat || it.lon) && e.homeSet;
      if (pos) NavEstimator::llaToNed(it.lat * 1e-7, it.lon * 1e-7, e.homeLat, e.homeLon, nn, ee);
      j.begin('{');
      j.key("cmd"); j.num(it.cmd, 0);
      j.key("pos"); j.raw(pos ? "true" : "false");
      j.key("n"); j.num(nn, 1);
      j.key("e"); j.num(ee, 1);
      j.key("alt"); j.num(it.alt, 1);
      j.key("p1"); j.num(it.param1, 2);
      j.key("p2"); j.num(it.param2, 2);
      j.key("p3"); j.num(it.param3, 2);
      j.key("p4"); j.num(it.param4, 2);
      j.key("la"); j.num(it.lat, 0);  // 1e-7 degrees, for the mission editor
      j.key("lo"); j.num(it.lon, 0);
      j.end('}');
    }
    j.end(']');
  };
  items("items", nav::count(), false);
  items("rally", nav::rallyCount(), true);
  j.key("fence"); j.num(bus::config.fence_enable ? bus::config.fence_radius : 0, 0);
  // polygon fence vertices (metres from home), drawn and edited on the map
  j.key("poly"); j.begin('[');
  for (int i = 0; i < nav::fenceCount(); i++) {
    nav::MissionItem it;
    if (!nav::fenceGet(i, it)) continue;
    float nn = 0, ee = 0;
    if (e.homeSet) NavEstimator::llaToNed(it.lat * 1e-7, it.lon * 1e-7, e.homeLat, e.homeLon, nn, ee);
    j.begin('[');
    j.comma(); j.num(nn, 1); j.comma(); j.num(ee, 1); j.comma(); j.num(it.lat, 0); j.comma(); j.num(it.lon, 0);
    j.end(']');
  }
  j.end(']');
  j.key("polyOn"); j.raw(bus::config.fence_enable && bus::config.fence_poly ? "true" : "false");
  j.end('}');
}

int setParam(const char* name, const char* value) {
  int i = paramFind(name);
  if (i < 0) return 404;
  if (armed()) return 423;
  char* end = nullptr;
  float v = strtof(value, &end);
  if (!value[0] || (end && *end)) return 400;
  if (!paramSet(bus::config, i, v)) return 400;
  bus::configChanged();
  return 200;
}

bool save() {
  if (armed()) return false;
  configSeal(bus::config);
  return hal::storageSave(hal::Slot::CONFIG, &bus::config, sizeof(Config));
}

int command(const char* c, char* msg, size_t n) {
  if (armed() && strcmp(c, "status")) { snprintf(msg, n, "disarm first"); return 423; }
  if (!strcmp(c, "calib_acc")) { bus::postRequest(bus::Request::ACC_CAL); snprintf(msg, n, "accel calibration: keep level"); }
  else if (!strcmp(c, "calib_mag")) { bus::postRequest(bus::Request::MAG_CAL, 30); snprintf(msg, n, "rotate the craft for 30 s"); }
  else if (!strcmp(c, "calib_acc6")) { bus::postRequest(bus::Request::ACC_CAL, 6); snprintf(msg, n, "6-point: hold each side still"); }
  else if (!strcmp(c, "compassmot")) { bus::postRequest(bus::Request::COMPASSMOT); snprintf(msg, n, "compassmot started (20 s)"); }
  else if (!strcmp(c, "calib_gyro")) { bus::postRequest(bus::Request::GYRO_CAL); snprintf(msg, n, "gyro calibration: keep still"); }
  else if (!strcmp(c, "defaults")) { configDefaults(bus::config); bus::configChanged(); snprintf(msg, n, "defaults loaded (not saved)"); }
  else if (!strcmp(c, "bb_erase")) { hal::logErase(); snprintf(msg, n, "blackbox erase started"); }
  else if (!strcmp(c, "sd_mount")) {
    if (!sdlog::enabled()) { snprintf(msg, n, "SD card off (sd_mode) or not possible on this board"); return 409; }
    sdlog::remount();
    snprintf(msg, n, "looking for the card");
  }
  else if (!strncmp(c, "sd_rm_", 6)) {  // sd_rm_AQ00012.BBL
    if (!sdlog::validName(c + 6)) { snprintf(msg, n, "bad file name"); return 400; }
    if (!sdlog::remove(c + 6)) { snprintf(msg, n, "cannot delete (no card / busy)"); return 409; }
    snprintf(msg, n, "%s deleted", c + 6);
  }
  else if (!strcmp(c, "reboot")) { snprintf(msg, n, "rebooting"); return 202; }
  // ESC throttle calibration (PWM / OneShot): esccal_hi / esccal_lo, repeated as a keep-alive; esccal_off
  else if (!strncmp(c, "esccal_", 7)) {
    if (hal::motorsAreDshot()) { snprintf(msg, n, "DShot ESCs need no throttle calibration"); return 409; }
    uint8_t st = !strcmp(c + 7, "hi") ? 1 : !strcmp(c + 7, "lo") ? 2 : 0;
    if (!bus::setEscCal(st, os::timeMs())) { snprintf(msg, n, "not now"); return 423; }
    snprintf(msg, n, st == 1 ? "full throttle" : st == 2 ? "zero throttle" : "off");
  }
  // DShot commands: escbeep_<mask> (find a motor), escdir_<motor 1-4>_<0 normal | 1 reversed> (saved in the ESC)
  else if (!strncmp(c, "escbeep_", 8) || !strncmp(c, "escdir_", 7)) {
    if (!hal::motorsAreDshot()) { snprintf(msg, n, "needs DShot (motor_protocol 2 or 3)"); return 409; }
    bool beep = c[3] == 'b';
    int a = atoi(c + (beep ? 8 : 7));
    uint8_t mask = beep ? (uint8_t)(a & 0x0F) : (uint8_t)(a >= 1 && a <= 4 ? 1 << (a - 1) : 0);
    const char* us = beep ? nullptr : strchr(c + 7, '_');
    if (!mask || (!beep && !us)) { snprintf(msg, n, "bad motor"); return 400; }
    // DShot: 1 = beacon, 7 / 8 = spin direction 1 / 2, 12 = save settings
    bool ok = beep ? bus::requestEscCommand(1, mask) : bus::requestEscCommand(atoi(us + 1) ? 8 : 7, mask, 12);
    if (!ok) { snprintf(msg, n, "busy"); return 423; }
    snprintf(msg, n, beep ? "beep" : "direction saved in the ESC");
  }
  else if (!strcmp(c, "bootloader")) {
    if (!hal::bootloaderSupported()) { snprintf(msg, n, "not from software on this board"); return 400; }
    snprintf(msg, n, "rebooting into the bootloader");
    return 205;
  }
  else { snprintf(msg, n, "unknown command"); return 404; }
  return 200;
}

static void sdFileCb(const char* name, uint32_t size, void* ctx) {
  if (!sdlog::validName(name)) return;
  JsonOut& j = *static_cast<JsonOut*>(ctx);
  j.begin('{');
  j.key("n"); j.str(name);
  j.key("s"); j.num(size, 0);
  j.end('}');
}

// SD card: state, space, queue health, files (the list only while no log is being written)
void sdJson(Sink s, void* ctx) {
  sdlog::Info i = sdlog::info();
  JsonOut j(s, ctx);
  j.begin('{');
  j.key("state"); j.str(sdlog::stateName(i.state));
  j.key("mode"); j.num(i.mode, 0);
  j.key("type"); j.str(i.type);
  j.key("err"); j.str(i.err);
  j.key("total"); j.num((double)i.total, 0);
  j.key("used"); j.num((double)i.used, 0);
  j.key("file"); j.str(i.file);
  j.key("bytes"); j.num(i.fileBytes, 0);
  j.key("q"); j.num(i.queued, 0);
  j.key("qs"); j.num(i.queueSize, 0);
  j.key("drop"); j.num(i.dropped, 0);
  j.key("wms"); j.num(i.maxWriteMs, 0);
  j.key("pins");
  j.begin('[');
  for (int k = 0; k < 6; k++) { j.comma(); j.num(i.pin[k], 0); }
  j.end(']');
  j.key("files");
  j.begin('[');
  bool listed = !armed() && sdlog::list(sdFileCb, &j);
  j.end(']');
  j.key("listed"); j.raw(listed ? "true" : "false");
  j.end('}');
}

void sensorsJson(Sink s, void* ctx) {
  AttitudeState a = bus::attitude.get();
  MagSample m = bus::mag.get();
  BaroSample b = bus::baro.get();
  RangeSample r = bus::range.get();
  FlowSample f = bus::flow.get();
  EscTelemetry e = bus::esc.get();
  JsonOut j(s, ctx);
  j.begin('{');
  j.key("t"); j.num(os::timeMs(), 0);
  float g[3] = {a.rateDps.x, a.rateDps.y, a.rateDps.z};
  vec(j, "gyro", g, 3, 2);
  float ac[3] = {a.accBody.x, a.accBody.y, a.accBody.z};
  vec(j, "acc", ac, 3, 3);
  float mg[3] = {m.field.x, m.field.y, m.field.z};
  vec(j, "mag", mg, 3, 3);
  j.key("magok"); j.raw(bus::mag.seq() && m.healthy ? "true" : "false");
  j.key("baro");
  j.begin('{');
  j.key("ok"); j.raw(bus::baro.seq() && b.healthy ? "true" : "false");
  j.key("alt"); j.num(b.altM, 2);
  j.key("pa"); j.num(b.pressurePa, 0);
  j.key("temp"); j.num(b.tempC, 1);
  j.end('}');
  j.key("range"); j.num(bus::range.seq() && r.valid ? r.distM : -1, 2);
  float fl[2] = {f.rateX, f.rateY};
  vec(j, "flow", fl, 2, 3);
  j.key("flowq"); j.num(bus::flow.seq() ? f.quality : -1, 0);
  float q[4] = {a.q.w, a.q.x, a.q.y, a.q.z};
  vec(j, "q", q, 4, 4);
  float rpm[4], temp[4], volt[4], amp[4];
  bool bidir = escRpmTemp(e, rpm, temp);
  const Config& c = bus::config;
  for (int i = 0; i < 4; i++) {
    int o = c.motor_out[i] >= 1 && c.motor_out[i] <= 4 ? c.motor_out[i] - 1 : i;
    dshot::Edt d;
    bool edt = bidir && hal::motorsEdt(o, d);
    volt[i] = edt ? d.volts : -1;
    amp[i] = edt ? d.amps : -1;
  }
  j.key("bidir"); j.raw(bidir ? "true" : "false");
  if (bidir) {
    uint32_t fr, er;
    hal::motorsBidirStats(fr, er);
    j.key("bderr"); j.num(fr ? 100.0 * er / fr : 0, 1);
  }
  vec(j, "rpm", rpm, 4, 0);
  vec(j, "esct", temp, 4, 0);
  vec(j, "escv", volt, 4, 2);
  vec(j, "esca", amp, 4, 0);
  j.end('}');
}

void cliExec(const char* line, Sink s, void* ctx, bool trusted) {
  comms::Cli cli;
  cli.begin(hal::Port::CONSOLE, sys);
  cli.setOutput(s, ctx);
  while (*line == ' ') line++;
  if (!trusted && !strncmp(line, "wifi", 4) && (line[4] == 0 || line[4] == ' ')) {
    static const char m[] = "the wifi command works over USB only (WiFi settings: System tab)\r\n";
    s(m, sizeof(m) - 1, ctx);
    return;
  }
  cli.run(line);
}

int motorTest(const float v[4], char* msg, size_t n) {
  if (armed()) { snprintf(msg, n, "disarm first"); return 423; }
  if (!bus::setMotorTest(v, os::timeMs())) { snprintf(msg, n, "not now"); return 423; }
  snprintf(msg, n, "ok");
  return 200;
}

// ---------------------------------------------------------------- backup / restore
void backupText(Sink s, void* ctx, bool all) {
  comms::Cli cli;
  cli.begin(hal::Port::CONSOLE, sys);
  cli.setOutput(s, ctx);
  cli.diff(all);
}

struct RestoreLog {
  int errors = 0;
  char first[60] = "";
};
static void restoreSink(const char* d, size_t n, void* ctx) {
  RestoreLog& r = *static_cast<RestoreLog*>(ctx);
  static const char* const bad[] = {"unknown", "out of range", "disarm", "FAILED", "unsupported", "missing", "add <"};
  char line[80];
  size_t k = n < sizeof(line) - 1 ? n : sizeof(line) - 1;
  memcpy(line, d, k);
  line[k] = 0;
  for (const char* b : bad)
    if (strstr(line, b)) {
      if (!r.errors++) {
        snprintf(r.first, sizeof(r.first), "%.58s", line);
        for (char* p = r.first; *p; p++) if (*p == '\r' || *p == '\n') *p = 0;
      }
      break;
    }
}

int restoreText(const char* text, size_t len, char* msg, size_t n) {
  if (armed()) { snprintf(msg, n, "disarm first"); return 423; }
  static Config before;  // static: keeps the server task stack small
  before = bus::config;
  comms::Cli cli;
  cli.begin(hal::Port::CONSOLE, sys);
  RestoreLog log;
  cli.setOutput(restoreSink, &log);
  int lines = 0;
  char line[160];
  size_t start = 0;
  while (start < len) {
    size_t e = start;
    while (e < len && text[e] != '\n') e++;
    size_t k = e - start < sizeof(line) - 1 ? e - start : sizeof(line) - 1;
    memcpy(line, text + start, k);
    line[k] = 0;
    while (k && (line[k - 1] == '\r' || line[k - 1] == ' ')) line[--k] = 0;
    // "save" is done once at the end, and only if every line worked (no half-restored flash)
    if (k && line[0] != '#' && strcmp(line, "save")) { cli.run(line); lines++; }
    start = e + 1;
  }
  if (log.errors) {  // roll back: settings in RAM, mission and rally points from flash
    bus::config = before;
    bus::configChanged();
    if (!nav::load()) nav::clear();
    if (!nav::rallyLoad()) nav::rallySetCount(0);
    snprintf(msg, n, "%d commands, %d errors (nothing changed): %s", lines, log.errors, log.first);
    return 400;
  }
  cli.run("save");
  if (log.errors) { snprintf(msg, n, "save failed"); return 400; }
  snprintf(msg, n, "%d commands applied and saved", lines);
  return 200;
}

// ---------------------------------------------------------------- access control
struct Session {
  char token[33];
  uint32_t lastMs;
};
static Session sessions[4];
static uint8_t failures = 0;
static uint32_t lockUntilMs = 0;
static constexpr uint32_t SESSION_IDLE_MS = 2u * 3600u * 1000u;  // 2 h without any request

static const char* cookieToken(const char* cookie) {
  if (!cookie) return nullptr;
  for (const char* p = strstr(cookie, "aq="); p; p = strstr(p + 1, "aq="))
    if (p == cookie || p[-1] == ' ' || p[-1] == ';') return p + 3;
  return nullptr;
}

int login(const char* pw, uint32_t now, char token[33]) {
  if (lockUntilMs && (int32_t)(lockUntilMs - now) > 0) return 429;
  if (!pw || !creds::equalsConstTime(pw, creds::webPass())) {
    if (++failures >= 5) { failures = 0; lockUntilMs = now + 30000; }  // 5 wrong in a row: 30 s pause
    return 401;
  }
  failures = 0;
  lockUntilMs = 0;
  // replace the least recently used session
  int slot = -1;
  for (int i = 0; i < 4 && slot < 0; i++)
    if (!sessions[i].token[0]) slot = i;
  if (slot < 0) {
    slot = 0;
    for (int i = 1; i < 4; i++)
      if (sessions[i].lastMs < sessions[slot].lastMs) slot = i;
  }
  for (int i = 0; i < 4; i++) snprintf(sessions[slot].token + i * 8, 9, "%08lx", (unsigned long)hal::random32());
  sessions[slot].lastMs = now ? now : 1;
  memcpy(token, sessions[slot].token, 33);
  return 200;
}

bool authorized(const char* cookie, uint32_t now) {
  const char* t = cookieToken(cookie);
  if (!t) return false;
  for (Session& s : sessions) {
    if (!s.token[0]) continue;
    if (now - s.lastMs > SESSION_IDLE_MS) { s.token[0] = 0; continue; }
    if (!strncmp(t, s.token, 32) && (t[32] == 0 || t[32] == ';' || t[32] == ' ')) {
      s.lastMs = now ? now : 1;
      return true;
    }
  }
  return false;
}

void logout(const char* cookie) {
  const char* t = cookieToken(cookie);
  if (!t) return;
  for (Session& s : sessions)
    if (s.token[0] && !strncmp(t, s.token, 32)) s.token[0] = 0;
}

int changeWifi(const char* ssid, const char* wifiPass, const char* webPass, const char* current, char* msg,
               size_t n, bool trusted) {
  if (armed()) { snprintf(msg, n, "disarm first"); return 423; }
  if (!trusted && (!current || !creds::equalsConstTime(current, creds::webPass()))) { snprintf(msg, n, "wrong password"); return 401; }
  const char* err = nullptr;
  if (ssid && !err) err = creds::setSsid(ssid);
  if (wifiPass && wifiPass[0] && !err) err = creds::setWifiPass(wifiPass);
  if (webPass && !err) err = creds::setWebPass(webPass);
  if (err) { creds::load(); snprintf(msg, n, "%s", err); return 400; }  // drop partial changes
  if (!creds::save()) { snprintf(msg, n, "save failed"); return 400; }
  for (Session& s : sessions) s.token[0] = 0;  // everyone logs in again with the new password
  snprintf(msg, n, "saved, WiFi restarts");
  return 200;
}

// targets without the ESP32 web server
#if !(defined(ARDUINO) && defined(ESP32)) || defined(ALLQUAD_HOST)
bool start() { return false; }
void poll() {}
#endif

}  // namespace web
