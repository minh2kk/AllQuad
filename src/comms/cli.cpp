#include "../common/size_opt.h"  // -Os on ESP32 (not flight critical)
#include "cli.h"

#include "../web/credentials.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../bus/bus.h"
#include "../hal/pinmap.h"
#include "../logging/sdlog.h"
#include "../navigation/mission.h"

namespace {
const char* const kFnName[] = {"-", "receiver", "GPS", "MAVLink", "MSP", "DisplayPort", "ESC telemetry", "rangefinder",
                               "blackbox", "SmartAudio", "Tramp", "configurator", "MAVLink (low rate)"};
}  // namespace

namespace comms {

using namespace fc;

void Cli::print(const char* fmt, ...) {
  char buf[200];
  va_list ap, ap2;
  va_start(ap, fmt);
  va_copy(ap2, ap);
  int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  char* p = buf;
  if (n >= (int)sizeof(buf)) {  // long line (help text, EKF status): format it again on the heap
    char* big = (char*)malloc((size_t)n + 1);
    if (big) { vsnprintf(big, (size_t)n + 1, fmt, ap2); p = big; }
    else n = sizeof(buf) - 1;
  }
  va_end(ap2);
  if (n > 0) {
    if (out_) out_(p, (size_t)n, outCtx_);
    else hal::portWrite(port_, (const uint8_t*)p, (size_t)n);
  }
  if (p != buf) free(p);
}

void Cli::run(const char* line) {
  char b[160];
  strncpy(b, line, sizeof(b) - 1);
  b[sizeof(b) - 1] = 0;
  size_t n = strlen(b);
  while (n && (b[n - 1] == '\r' || b[n - 1] == '\n' || b[n - 1] == ' ')) b[--n] = 0;
  if (!n || b[0] == '#') return;
  exec(b, hal::millis());
}

void Cli::diff(bool all) {
  Config def;
  configDefaults(def);
  print("# AllQuad %s %s backup (%s)\r\n", info_ ? info_->version : "?", hal::platformName(), all ? "all" : "diff");
  print("# paste into the CLI (USB) or restore from the web configurator\r\n");
  for (int i = 0; i < paramCount(); i++) {
    const ParamInfo& p = paramInfo(i);
    float v = paramGet(bus::config, i), d = paramGet(def, i);
    if (!all && v == d) continue;
    if (p.type == PType::F32) print("set %s %.9g\r\n", p.name, (double)v);
    else print("set %s %lu\r\n", p.name, (unsigned long)v);
  }
  print("mission clear\r\n");
  for (int i = 0; i < nav::count(); i++) {
    nav::MissionItem it;
    if (nav::get(i, it))
      print("mission add %u %u %ld %ld %.9g %.9g %.9g %.9g %.9g\r\n", it.cmd, it.frame, (long)it.lat, (long)it.lon,
            (double)it.alt, (double)it.param1, (double)it.param2, (double)it.param3, (double)it.param4);
  }
  print("rally clear\r\n");
  for (int i = 0; i < nav::rallyCount(); i++) {
    nav::MissionItem it;
    if (nav::rallyGet(i, it)) print("rally add %ld %ld %.9g\r\n", (long)it.lat, (long)it.lon, (double)it.alt);
  }
  print("fence clear\r\n");
  for (int i = 0; i < nav::fenceCount(); i++) {
    nav::MissionItem it;
    if (nav::fenceGet(i, it)) print("fence add %ld %ld\r\n", (long)it.lat, (long)it.lon);
  }
  for (int i = 0; i < LOGIC_RULES; i++) {
    const LogicRule& r = bus::config.logic[i];
    if (!all && !r.enabled && !r.action) continue;
    print("logic %d %u %u %d %u %u %d %u %u %d\r\n", i + 1, r.enabled, r.aType, r.aVal, r.op, r.bType, r.bVal, r.action,
          r.actArg, r.actVal);
  }
  print("save\r\n");
}

void Cli::feed(uint8_t c, uint32_t now, bool binaryActive) {
  if (c == '\r' || c == '\n') {
    if (pos_ && !garbage_ && !binaryActive) {
      line_[pos_] = 0;
      bus::cliActiveUntilMs = now + 5000;
      exec(line_, now);
      print("# ");
    }
    pos_ = 0;
    garbage_ = false;
    return;
  }
  if (c == 8 || c == 127) { if (pos_) pos_--; return; }
  if (c < 32 || c > 126) { garbage_ = true; return; }  // binary protocol bytes
  if (pos_ < sizeof(line_) - 1) line_[pos_++] = (char)c;
}

void Cli::cmdStatus() {
  VehicleStatus st = bus::status.get();
  AttitudeState a = bus::attitude.get();
  NavEstimate e = bus::nav.get();
  BatteryState b = bus::battery.get();
  RcInput rc = bus::rc.get();
  GpsData g = bus::gps.get();
  static const char* const rrName[] = {"unknown", "power-on", "external pin", "software", "PANIC", "INT WATCHDOG",
                                       "TASK WATCHDOG", "WATCHDOG", "deep sleep", "BROWNOUT", "SDIO"};
  int rr = hal::resetReason();
  print("AllQuad %s on %s | up %lu s | last reset: %s\r\n", info_ ? info_->version : "?", hal::platformName(),
        (unsigned long)(hal::millis() / 1000), rr >= 0 && rr <= 10 ? rrName[rr] : "?");
  print("IMU %s | BARO %s | MAG %s | GPS %s\r\n", info_ && info_->imu ? info_->imu : "NONE",
        info_ && info_->baro ? info_->baro : "none", info_ && info_->mag ? info_->mag : "none",
        bus::gps.seq() ? "yes" : "none");
  print("attitude roll %.1f pitch %.1f yaw %.1f | alt %.2f m vz %.2f | agl %s %.2f\r\n", a.roll * RAD2DEG,
        a.pitch * RAD2DEG, e.yaw * RAD2DEG, e.alt, e.vz, e.aglValid ? "ok" : "-", e.agl);
  print("position %s (gps %d flow %d) N %.1f E %.1f | home %s %.0f m\r\n", e.posValid ? "valid" : "invalid",
        e.gpsValid, e.flowValid, e.n, e.e, e.homeSet ? "set" : "-", st.homeDistance);
  static const char* const hs[] = {"none", "compass", "GPS", "EKF"};
  print("heading %s %.0f deg | wind %.1f m/s from %.0f deg | hover thr %.3f | gps glitches %u%s\r\n",
        hs[e.headingSrc < 4 ? e.headingSrc : 0], e.yaw * RAD2DEG, sqrtf(e.windN * e.windN + e.windE * e.windE),
        fmodf(atan2f(-e.windE, -e.windN) * RAD2DEG + 360.0f, 360.0f), e.hoverThr, e.glitchCount,
        e.gpsGlitch ? " (now)" : "");
  EkfReport k = bus::ekf.get();
  if (k.active) {
    print("EKF: pos %.2f vel %.2f hgt %.2f m yaw %.1f deg (1 sigma) | test gps %.2f/%.2f hgt %.2f yaw %.2f mag %.2f | "
          "height %s | faults 0x%02x resets %u/%u/%u\r\n", k.posStd, k.velStd, k.hgtStd, k.yawStdDeg, k.trGpsPos,
          k.trGpsVel, k.trHeight, k.trYaw, k.trMag, k.heightSrc ? "GPS" : "baro", k.faults, k.posResets, k.velResets,
          k.filterResets);
  }
  print("vibration %.1f m/s^2 | motors %s\r\n", (double)a.vibration,
        hal::motorsBidirActive() ? "bidirectional DShot" : "");
  print("gps fix %d sats %u hAcc %.1f | battery %.2fV %uS %u%% %.1fA %.0fmAh\r\n", g.fix3d, g.sats, g.hAcc,
        b.voltage, b.cells, b.percent, b.current, b.mahDrawn);
  print("rx %s LQ %u  %u %u %u %u | %u %u %u %u %u %u\r\n", rc.everReceived ? "ok" : "no signal", rc.linkQuality,
        rc.ch[0], rc.ch[1], rc.ch[2], rc.ch[3], rc.ch[4], rc.ch[5], rc.ch[6], rc.ch[7], rc.ch[8], rc.ch[9]);
  print("mode %s nav %s armed %d failsafe %d mission %u/%u\r\n", flightModeName(st.mode), navPhaseName(st.navPhase),
        st.armed, st.failsafe, st.missionIndex, st.missionCount);
  if (!st.armBlockers) {
    print("arming: READY\r\n");
  } else {
    print("arming blocked:");
    for (int i = 0; i < ARM_COUNT_BITS; i++)
      if (st.armBlockers & (1u << i)) print(" %s", armCheckName(i));
    print("\r\n");
  }
}

// One-screen health check: each line [ OK ] / [WARN] / [FAIL] with what to look at. Read-only; it
// reads the same live values as "status" (nothing is reset, no bus traffic but one I2C line read).
void Cli::cmdSelftest() {
  int nWarn = 0, nFail = 0;
  auto line = [&](int lvl, const char* what, const char* fmt, ...) {
    char buf[150];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (lvl == 1) nWarn++;
    if (lvl == 2) nFail++;
    print("[%s] %-9s %s\r\n", lvl == 0 ? " OK " : lvl == 1 ? "WARN" : "FAIL", what, buf);
  };
  VehicleStatus st = bus::status.get();
  AttitudeState a = bus::attitude.get();
  RcInput rc = bus::rc.get();
  BatteryState b = bus::battery.get();
  const Config& c = bus::config;
  print("AllQuad %s on %s, up %lu s%s\r\n", info_ ? info_->version : "?", hal::platformName(),
        (unsigned long)(hal::millis() / 1000), st.armed ? " (ARMED: values are in flight)" : "");

  int rr = hal::resetReason();
  static const char* const rrName[] = {"unknown", "power-on", "external pin", "software", "PANIC", "INT WATCHDOG",
                                       "TASK WATCHDOG", "WATCHDOG", "deep sleep", "BROWNOUT", "SDIO"};
  const char* rrn = rr >= 0 && rr <= 10 ? rrName[rr] : "?";
  if (rr == 4 || rr == 5 || rr == 6 || rr == 7) line(2, "reset", "last reset: %s - the firmware crashed or hung (see 'crash')", rrn);
  else if (rr == 9) line(1, "reset", "last reset: BROWNOUT - supply dipped (weak 5 V / 3.3 V regulator, bad USB cable)");
  else line(0, "reset", "last reset: %s", rrn);
  {
    hal::CrashInfo ci;
    if (hal::lastCrash(ci))
      line(ci.bootsAgo ? 1 : 2, "crash", "%s in %s, %lu s after boot (%s) - 'crash' for details", hal::crashKindName(ci.kind),
           ci.where[0] ? ci.where : "?", (unsigned long)(ci.uptimeMs / 1000), ci.bootsAgo ? "an earlier run" : "last reset");
  }

  // fast loop
  {
    const volatile bus::LoopPerf& p = bus::loopPerf;
    uint32_t per = os::taskCount() ? os::taskStats(0).periodUs : 0;
    float hz = per ? 1e6f / per : 0;
    float want = c.loop_hz ? (float)c.loop_hz : hz;
    if (!per || hz < want * 0.9f) line(2, "loop", "%.0f Hz, set %u Hz - loop too slow (lower loop_hz)", (double)hz, (unsigned)c.loop_hz);
    else if (p.loadPct > 85) line(1, "loop", "%.0f Hz, load %.0f%% - close to the limit (lower loop_hz / filters)", (double)hz, (double)p.loadPct);
    else line(0, "loop", "%.0f Hz, load %.0f%%, worst %.0f us, started by %s", (double)hz, (double)p.loadPct, (double)p.maxTotalUs, os::triggerSource());
    if (p.lateLoops) line(1, "loop", "%lu late iterations since boot (flash writes / WiFi bursts cause a few)", (unsigned long)p.lateLoops);
  }
  // tasks: stack headroom and overruns
  {
    int worst = -1;
    uint32_t worstFree = 0xFFFFFFFF, over = 0;
    for (int i = 0; i < os::taskCount(); i++) {
      os::TaskStats t = os::taskStats(i);
      over += t.overruns;
      if (t.stackFree && t.stackFree < worstFree) { worstFree = t.stackFree; worst = i; }
    }
    if (worst >= 0 && worstFree < 256) line(2, "stack", "task %s has only %lu B free", os::taskStats(worst).name, (unsigned long)worstFree);
    else if (worst >= 0 && worstFree < 512) line(1, "stack", "task %s has %lu B free", os::taskStats(worst).name, (unsigned long)worstFree);
    else line(0, "stack", "%d tasks, lowest free %lu B", os::taskCount(), (unsigned long)(worst >= 0 ? worstFree : 0));
    uint32_t hf, hl;
    os::heapStats(hf, hl);
    if (hf) {
      if (hl < 2048) line(2, "heap", "%lu B free, lowest %lu B", (unsigned long)hf, (unsigned long)hl);
      else if (hl < 6144) line(1, "heap", "%lu B free, lowest %lu B", (unsigned long)hf, (unsigned long)hl);
      else line(0, "heap", "%lu B free, lowest %lu B", (unsigned long)hf, (unsigned long)hl);
    }
    if (over) line(1, "tasks", "%lu overruns (a task missed its period; 'tasks' shows which)", (unsigned long)over);
  }
  // IMU
  if (!info_ || !info_->imu) line(2, "imu", "no IMU detected - check SPI wiring / CS ('scan')");
  else if (!a.valid) line(2, "imu", "%s detected but no attitude yet", info_->imu);
  else {
    float g = sqrtf(a.accBody.x * a.accBody.x + a.accBody.y * a.accBody.y + a.accBody.z * a.accBody.z);
    float r = sqrtf(a.rateDps.x * a.rateDps.x + a.rateDps.y * a.rateDps.y + a.rateDps.z * a.rateDps.z);
    if (!st.armed && (g < 8.3f || g > 11.3f)) line(1, "imu", "%s: accel %.2f m/s^2 at rest (expect 9.8) - calibrate / check the scale", info_->imu, (double)g);
    else if (!st.armed && r > 5) line(1, "imu", "%s: gyro %.1f deg/s at rest - keep the quad still, recalibrate gyro", info_->imu, (double)r);
    else line(0, "imu", "%s: accel %.2f m/s^2, gyro %.1f deg/s, vibration %.1f", info_->imu, (double)g, (double)r, (double)a.vibration);
    if (!c.acc_calibrated) line(1, "imu", "accelerometer not calibrated (calib_acc, quad level)");
    if (a.accClips) line(1, "imu", "accelerometer clipped %lu times since boot (beyond +-16 g): hard vibration or a loose FC", (unsigned long)a.accClips);
    if (st.armBlockers & ARM_IMU_FAIL) line(2, "imu", "failed over to the second IMU - check the first one's wiring");
  }
  // I2C sensors
  {
    bool sda = true, scl = true;
    hal::i2cLineLevels(sda, scl);
    if (!sda || !scl) line(2, "i2c", "line held LOW (SDA %s, SCL %s): short, missing pull-up or a stuck chip", sda ? "high" : "LOW", scl ? "high" : "LOW");
    BaroSample bs = bus::baro.get();
    if (!info_ || !info_->baro) line(1, "baro", "none - no altitude hold ('scan' lists what answers on I2C)");
    else if (!bs.healthy) line(2, "baro", "%s detected but not reading", info_->baro);
    else line(0, "baro", "%s %.0f Pa %.1f C", info_->baro, (double)bs.pressurePa, (double)bs.tempC);
    MagSample ms = bus::mag.get();
    if (!info_ || !info_->mag) line(c.gps_heading ? 0 : 1, "compass", "none%s", c.gps_heading ? " (heading from GPS when moving)" : " - GPS modes need a heading source");
    else if (!ms.healthy) line(2, "compass", "%s detected but not reading", info_->mag);
    else {
      float f = sqrtf(ms.field.x * ms.field.x + ms.field.y * ms.field.y + ms.field.z * ms.field.z);
      if (f < 0.15f || f > 0.9f) line(1, "compass", "%s field %.2f G (Earth 0.25-0.65) - calibrate or move it away from wires", info_->mag, (double)f);
      else line(0, "compass", "%s field %.2f G", info_->mag, (double)f);
    }
  }
  // receiver
  {
    uint32_t by = bus::rxLink.bytes, fr = bus::rxLink.frames, ce = bus::rxLink.crcErrors;
    if (!by && !rc.everReceived) line(1, "rx", "no data on the receiver port (port function, TX/RX swapped, RX powered and bound?)");
    else if (by && !fr && !rc.everReceived) line(2, "rx", "%lu bytes but no valid frame - wrong protocol / baud / inversion", (unsigned long)by);
    else {
      float bad = fr + ce ? 100.0f * ce / (fr + ce) : 0;
      bool now = hal::millis() - rc.lastFrameMs < 500;
      if (!now) line(2, "rx", "signal lost (last frame %lu ms ago)", (unsigned long)(hal::millis() - rc.lastFrameMs));
      else if (bad > 2) line(1, "rx", "%.1f%% frames with bad checksum - wiring / ground / noise", (double)bad);
      else if (rc.linkQuality && rc.linkQuality < 70) line(1, "rx", "LQ %u%% - antenna position / range", rc.linkQuality);
      else line(0, "rx", "%lu frames, %lu bad, LQ %u%%", (unsigned long)fr, (unsigned long)ce, rc.linkQuality);
    }
  }
  // UARTs
  {
    static const char* const pn[] = {"SERIAL1", "SERIAL2", "SERIAL3", "UDP", "USB", "SERIAL4", "SERIAL5", "SERIAL6", "SERIAL7"};
    int open = 0;
    for (int i = 0; i < (int)hal::Port::COUNT; i++) {
      hal::PortStats ps;
      if (i == (int)hal::Port::UDP || i == (int)hal::Port::CONSOLE || !hal::portStats((hal::Port)i, ps)) continue;
      open++;
      if (ps.overruns || ps.dropped || ps.framing > 10 || ps.noise > 10)
        line(1, "uart", "%s: overrun %lu, dropped %lu, framing %lu, noise %lu", pn[i], (unsigned long)ps.overruns,
             (unsigned long)ps.dropped, (unsigned long)ps.framing, (unsigned long)ps.noise);
      else line(0, "uart", "%s: rx %lu B, tx %lu B, no line errors", pn[i], (unsigned long)ps.rxBytes, (unsigned long)ps.txBytes);
    }
    if (!open) line(0, "uart", "no UART opened");
    // port functions: set on a UART this board does not have, or a single-use function on two ports
    const Config& pc = bus::config;
    const char* const* fnName = kFnName;
    int first[13];
    for (int f = 0; f < 13; f++) first[f] = -1;
    int portIssues = 0;
    for (int i = 0; i < (int)hal::Port::COUNT; i++) {
      // UDP exists only with WiFi (ESP32); its default MAVLink function is harmless elsewhere
      if (i == (int)hal::Port::CONSOLE || i == (int)hal::Port::UDP) continue;
      uint8_t f = portFnOf(pc, i);
      if (!f || f >= 13) continue;
      if (!hal::portExists((hal::Port)i)) {
        line(1, "ports", "%s is set to %s, but this board has no %s (function ignored)", pn[i], fnName[f], pn[i]);
        portIssues++;
        continue;
      }
      // several MAVLink / MSP ports are fine; everything else talks to one device
      if (f != (uint8_t)PortFn::MAVLINK && f != (uint8_t)PortFn::MAVLINK_LOW && f != (uint8_t)PortFn::MSP && first[f] >= 0) {
        line(1, "ports", "%s on both %s and %s: only %s is used", fnName[f], pn[first[f]], pn[i], pn[first[f]]);
        portIssues++;
      }
      if (first[f] < 0) first[f] = i;
    }
    if (first[(int)PortFn::VTX_SA] >= 0 && first[(int)PortFn::VTX_TRAMP] >= 0) {
      line(1, "ports", "SmartAudio and Tramp both set: one VTX control at a time (SmartAudio is used)");
      portIssues++;
    }
    if (!portIssues) line(0, "ports", "functions on existing UARTs, no duplicates");
  }
  // GPS
  if (bus::gpsLink.port < (uint8_t)hal::Port::COUNT && !bus::gpsLink.frames) {
    static const char* const pn[] = {"SERIAL1", "SERIAL2", "SERIAL3", "UDP", "USB", "SERIAL4", "SERIAL5", "SERIAL6", "SERIAL7"};
    hal::PortStats ps{};
    bool open = hal::portStats((hal::Port)bus::gpsLink.port, ps);
    if (!open || !ps.rxBytes)
      line(2, "gps", "%s: no byte received - GPS TX must go to the FC RX pin of that port, GPS powered (5 V), GND common",
           pn[bus::gpsLink.port]);
    else
      line(2, "gps", "%s: %lu bytes but no valid GPS frame yet (trying %lu baud; framing errors %lu)", pn[bus::gpsLink.port],
           (unsigned long)ps.rxBytes, (unsigned long)bus::gpsLink.baud, (unsigned long)ps.framing);
  } else if (bus::gps.seq()) {
    GpsData g = bus::gps.get();
    const char* pr = bus::gpsLink.proto == 1 ? "UBX" : bus::gpsLink.proto == 2 ? "NMEA" : "?";
    if (!g.fix3d) line(1, "gps", "%s %lu baud, %u satellites, no 3D fix yet (needs open sky)", pr, (unsigned long)bus::gpsLink.baud, g.sats);
    else line(0, "gps", "%s %lu baud, 3D fix, %u satellites, hAcc %.1f m", pr, (unsigned long)bus::gpsLink.baud, g.sats, (double)g.hAcc);
  }
  // optical flow / rangefinder module on an MSP port
  {
    static const char* const pn[] = {"SERIAL1", "SERIAL2", "SERIAL3", "UDP", "USB", "SERIAL4", "SERIAL5", "SERIAL6", "SERIAL7"};
    const Config& sc = bus::config;
    uint8_t sp = bus::sensorLink.port;
    if (sp < (uint8_t)hal::Port::COUNT) {
      uint32_t by = bus::sensorLink.bytes, ms = bus::sensorLink.msp, mi = bus::sensorLink.mico, mv = bus::sensorLink.mav;
      RangeSample r = bus::range.get();
      FlowSample f = bus::flow.get();
      if (ms || mi)
        line(0, "flow", "%s: %s, range %s%.2f m, flow quality %u", pn[sp], mi > ms ? "Micolink" : "MSP", r.valid ? "" : "invalid ",
             (double)(r.valid ? r.distM : 0.0f), bus::flow.seq() ? f.quality : 0);
      else if (mv)
        line(2, "flow", "%s: %lu bytes of MAVLink - set the module to MSP (iNav) or Micolink with MicoAssistant", pn[sp],
             (unsigned long)by);
      else {
        hal::PortStats ps{};
        hal::portStats((hal::Port)sp, ps);
        line(2, "flow", "%s: %lu bytes but no MSP / Micolink frame - wrong baud (module: 115200) or protocol (framing errors %lu)",
             pn[sp], (unsigned long)by, (unsigned long)ps.framing);
      }
    } else {
      for (int i = 0; i < (int)hal::Port::COUNT; i++)
        if (i != (int)hal::Port::UDP && i != (int)hal::Port::CONSOLE && portFnOf(sc, i) == (uint8_t)PortFn::MSP &&
            hal::portExists((hal::Port)i)) {
          line(1, "flow", "%s (MSP): nothing received - a flow / range module: its TX to the FC RX pin, 5 V, GND common", pn[i]);
          break;
        }
    }
  }
  // battery
  if (b.voltage < 2.0f) line(1, "battery", "%.2f V - no battery (USB only) or voltage sensing not wired / scaled", (double)b.voltage);
  else line(0, "battery", "%.2f V, %u cells, %u%%", (double)b.voltage, b.cells, b.percent);
  // settings storage
  {
    int copy, copies;
    uint32_t seq;
    hal::storageInfo(copy, seq, copies);
    if (copies >= 2) line(0, "storage", "settings in copy %c of 2 (save #%lu) - a power loss while saving keeps the other", 'A' + copy, (unsigned long)seq);
    else if (copies == 1) line(0, "storage", "settings in one flash copy (no spare sector on this chip)");
    else line(0, "storage", "settings in NVS");
  }
  // watchdog (STM32 IWDG; ESP32 uses the ESP-IDF task watchdog)
#if defined(AQ_STM32)
  if (hal::watchdogRunning()) line(0, "watchdog", "running (resets a hung firmware after ~7 s)");
  else line(1, "watchdog", "not running yet (starts once the flight loop runs)");
#endif
  // arming
  if (!st.armBlockers) line(0, "arming", "ready");
  else {
    char buf[140];
    size_t n = 0;
    buf[0] = 0;
    for (int i = 0; i < ARM_COUNT_BITS && n < sizeof(buf) - 16; i++)
      if (st.armBlockers & (1u << i)) n += snprintf(buf + n, sizeof(buf) - n, " %s", armCheckName(i));
    line(1, "arming", "blocked by%s", buf);
  }
  print("selftest: %d warning(s), %d failure(s)%s\r\n", nWarn, nFail, nFail ? " - fix the FAIL lines before flying" : "");
}

void Cli::cmdTasks() {
  print("%-10s %7s %7s %7s %6s %8s %9s\r\n", "task", "rate_hz", "last_us", "max_us", "load%", "overruns", "stack_free");
  for (int i = 0; i < os::taskCount(); i++) {
    os::TaskStats t = os::taskStats(i);
    print("%-10s %7u %7u %7u %5u%% %8u %9u\r\n", t.name, (unsigned)(1000000 / t.periodUs), (unsigned)t.lastUs,
          (unsigned)t.maxUs, (unsigned)(t.lastUs * 100 / t.periodUs), (unsigned)t.overruns, (unsigned)t.stackFree);
  }
  os::resetTaskStats();
}

void Cli::cmdGet(const char* filter) {
  for (int i = 0; i < paramCount(); i++) {
    const ParamInfo& p = paramInfo(i);
    if (filter && !strstr(p.name, filter)) continue;
    float v = paramGet(bus::config, i);
    if (p.type == PType::F32) print("%s = %.4g\r\n", p.name, (double)v);
    else print("%s = %lu\r\n", p.name, (unsigned long)v);
  }
}

void Cli::cmdRally(const char* a1, char* rest) {
  if (a1 && !strcmp(a1, "clear")) { nav::rallySetCount(0); print("rally points cleared\r\n"); return; }
  if (a1 && !strcmp(a1, "add")) {
    nav::MissionItem it{};
    it.cmd = nav::CMD_RALLY_POINT;
    it.frame = 3;
    char* lat = strtok(rest, " ");
    char* lon = strtok(nullptr, " ");
    char* alt = strtok(nullptr, " ");
    if (!lat || !lon || nav::rallyCount() >= nav::RALLY_MAX) { print("rally add <lat_e7> <lon_e7> <alt>\r\n"); return; }
    it.lat = atol(lat);
    it.lon = atol(lon);
    it.alt = alt ? (float)atof(alt) : 0;
    int n = nav::rallyCount();
    nav::rallySet(n, it);
    nav::rallySetCount(n + 1);
    return;
  }
  print("%d rally points\r\n", nav::rallyCount());
  for (int i = 0; i < nav::rallyCount(); i++) {
    nav::MissionItem it;
    if (nav::rallyGet(i, it)) print("%d lat %.7f lon %.7f alt %.1f\r\n", i, it.lat * 1e-7, it.lon * 1e-7, it.alt);
  }
}

void Cli::cmdMission(const char* arg) {
  if (arg && !strcmp(arg, "clear")) {
    nav::clear();
    print("mission cleared\r\n");
    return;
  }
  print("%d items\r\n", nav::count());
  for (int i = 0; i < nav::count(); i++) {
    nav::MissionItem it;
    if (nav::get(i, it))
      print("%2d cmd %3u lat %.7f lon %.7f alt %.1f p1 %.1f p2 %.1f\r\n", i, it.cmd, it.lat * 1e-7, it.lon * 1e-7,
            it.alt, it.param1, it.param2);
  }
}

void Cli::exec(char* line, uint32_t now) {
  size_t lineLen = strlen(line);
  char* cmd = strtok(line, " ");
  char* a1 = strtok(nullptr, " ");
  char restBuf[128] = "";  // everything after the first argument (mission add / rally add)
  if (a1) {
    size_t off = (size_t)(a1 - line) + strlen(a1) + 1;
    if (off < lineLen) strncpy(restBuf, line + off, sizeof(restBuf) - 1);
  }
  char* a2 = strtok(nullptr, " ");
  if (!cmd) return;
  bool armed = bus::status.get().armed;
  if (!strcmp(cmd, "help")) {
    print("status | selftest | crash [clear|test|hang] | tasks | perf | esc | get [filter] | set <name> <value> | save | defaults\r\n"
          "calib_acc | calib_acc6 | calib_mag [s] | calib_gyro | compassmot | mission [clear|add ...] | rally [clear|add ...]\r\n"
          "fence [clear|add <latE7> <lonE7>] | logic [n ...] | log [erase] | sd [ls | rm <file> | mount] | goto <latE7> <lonE7> [alt] (GUIDED)\r\n"
          "diff (backup: changed settings) | dump (backup: everything)\r\n"
          "wifi [name <ssid> | pass <8-63 chars> | webpass <6-64 chars, - = same as wifi> | reset]\r\n"
          "version | reboot | bootloader (USB flashing: ESP32-S3 ROM / STM32 DFU) | scan (wiring check) | net (WiFi check)\r\n"
          "pins [filter] (what each pin of the board is wired to, and what uses it now)\r\n");
  } else if (!strcmp(cmd, "pins")) {
    // board pin map + what the settings use it for now; "pins uart", "pins PB" ... filter the lines
    hal::PinUse pm[hal::PINMAP_MAX];  // on the stack: no RAM kept for a rare request (F405 is tight)
    int n = hal::pinMap(pm, hal::PINMAP_MAX);
    const Config& c = bus::config;
    print("pin map of %s (src/hal/board.h)\r\n", hal::pinMapBoard());
    static const char* const pn[] = {"SERIAL1", "SERIAL2", "SERIAL3", "UDP", "USB", "SERIAL4", "SERIAL5", "SERIAL6", "SERIAL7"};
    for (int i = 0; i < n; i++) {
      const hal::PinUse& u = pm[i];
      char pin[8], what[40], use[48];
      hal::pinText(u.pin, pin, sizeof(pin));
      use[0] = 0;
      switch (u.group) {
        case hal::PinGroup::IMU:
          snprintf(what, sizeof(what), "IMU SPI %s", u.what);
          if (u.arg == 1) snprintf(use, sizeof(use), c.imu_dual ? "second IMU" : "off (imu_dual 0)");
          else if (!strcmp(u.what, "INT")) snprintf(use, sizeof(use), c.imu_sync ? "loop sync" : "off (imu_sync 0)");
          break;
        case hal::PinGroup::OSD: snprintf(what, sizeof(what), "OSD SPI %s", u.what);
          snprintf(use, sizeof(use), c.osd_backend == 1 ? "MAX7456" : "off (osd_backend)"); break;
        case hal::PinGroup::I2C: snprintf(what, sizeof(what), "I2C %s (baro, compass)", u.what); break;
        case hal::PinGroup::UART: {
          int f = portFnOf(c, u.arg);
          char inst[12] = "";  // "USART3 RX PD9 TX PD8" -> "USART3" (ESP32 labels have no instance name)
          const char* l = hal::portLabel((hal::Port)u.arg);
          if (strncmp(l, "RX", 2)) { int k = 0; while (l[k] && l[k] != ' ' && k < 11) { inst[k] = l[k]; k++; } inst[k] = 0; }
          snprintf(what, sizeof(what), "%s %s%s%s", u.arg < 9 ? pn[u.arg] : "?", u.what, inst[0] ? " " : "", inst);
          snprintf(use, sizeof(use), "%s", f >= 0 && f < 13 ? (f ? kFnName[f] : "free") : "?");
          break;
        }
        case hal::PinGroup::MOTOR: {
          int m = 0;
          for (int k = 0; k < 4; k++) if (c.motor_out[k] == u.arg) m = k + 1;
          snprintf(what, sizeof(what), "motor output %d", u.arg);
          if (m) snprintf(use, sizeof(use), "mixer motor %d", m);
          break;
        }
        case hal::PinGroup::ANALOG: snprintf(what, sizeof(what), u.arg ? "current sensor (ADC)" : "battery voltage (ADC)"); break;
        case hal::PinGroup::LED: snprintf(what, sizeof(what), "status LED"); break;
        case hal::PinGroup::BUZZER: snprintf(what, sizeof(what), "buzzer"); break;
        case hal::PinGroup::LEDSTRIP: snprintf(what, sizeof(what), "LED strip data");
          snprintf(use, sizeof(use), c.led_pin ? "on" : "off (led_pin 0)"); break;
        case hal::PinGroup::SD: snprintf(what, sizeof(what), "SD card %s", u.what);
          snprintf(use, sizeof(use), c.sd_mode ? "on" : "off (sd_mode 0)"); break;
        default: snprintf(what, sizeof(what), "board / chip (USB, SWD, flash ...)"); break;
      }
      if (a1) {  // filter: pin name or function text, case-insensitive
        char all[96];
        snprintf(all, sizeof(all), "%s %s %s %s", pin, what, use, hal::pinGroupName(u.group));
        bool hit = false;
        for (char* p = all; *p && !hit; p++) {
          int k = 0;
          while (a1[k] && p[k] && ((p[k] | 32) == (a1[k] | 32))) k++;
          hit = !a1[k];
        }
        if (!hit) continue;
      }
      print("  %-6s %-34s %s\r\n", pin, what, use);
    }
    if (c.led_pin) print("  LED strip parameter led_pin %d\r\n", c.led_pin);
  } else if (!strcmp(cmd, "net")) {
    char* b = (char*)malloc(900);
    if (b) { hal::netReport(b, 900); print("%s", b); free(b); }
  } else if (!strcmp(cmd, "scan")) {
    // wiring check: who answers on the I2C bus, what the IMU chip select reads back
    print("%s\r\n", hal::pinSummary());
    int found = 0;
    print("I2C:");
    for (uint8_t a = 0x08; a < 0x78; a++)
      if (hal::i2cProbe(a)) {
        found++;
        const char* n = a == 0x76 || a == 0x77 ? " (baro BMP/DPS/SPL/MS5611)" : a == 0x2C ? " (QMC5883P)" : a == 0x0D ? " (QMC5883L)"
                      : a == 0x1E ? " (HMC5883/LIS3MDL)" : a == 0x0E || a == 0x0C ? " (IST8310)" : a == 0x68 || a == 0x69 ? " (MPU/ICM)"
                      : a == 0x42 ? " (u-blox DDC)" : "";
        print(" 0x%02X%s", a, n);
      }
    print(found ? "\r\n" : " nothing answers (check SDA/SCL, 3.3 V, GND, pull-ups)\r\n");
    print("I2C lines at power-up: %s; clock %lu kHz\r\n", hal::i2cDiag(), (unsigned long)(hal::i2cClock() / 1000));
    // chip ID registers: tells "answers but is another chip" from "not there"
    struct IdReg { uint8_t addr, reg; const char* what; };
    static const IdReg ids[] = {{0x0D, 0x0D, "QMC5883L id (FF)"}, {0x2C, 0x00, "QMC5883P id (80)"},
                                {0x76, 0x00, "BMP388/390 id (50/60)"}, {0x77, 0x00, "BMP388/390 id (50/60)"},
                                {0x76, 0xD0, "BMP280/BME280 id (58/60)"}, {0x77, 0xD0, "BMP280/BME280 id (58/60)"},
                                {0x76, 0x0D, "DPS310/SPL06 id (10)"}, {0x77, 0x0D, "DPS310/SPL06 id (10)"},
                                {0x1E, 0x0F, "LIS3MDL id (3D)"}, {0x0E, 0x00, "IST8310 id (10)"}};
    for (const IdReg& r : ids) {
      uint8_t v;
      if (hal::i2cRead(r.addr, r.reg, &v, 1)) print("  0x%02X reg 0x%02X = 0x%02X  %s\r\n", r.addr, r.reg, v, r.what);
    }
    print("detected: baro %s, compass %s\r\n", info_ && info_->baro ? info_->baro : "-", info_ && info_->mag ? info_->mag : "-");
    {
      bool sda, scl;
      hal::i2cLineLevels(sda, scl);
      print("I2C lines now: SDA %s, SCL %s\r\n", sda ? "high" : "LOW", scl ? "high" : "LOW");
    }
    if (!found && !armed) {  // nobody answered the I2C peripheral: try the same pins by software, both ways
      uint8_t f[16];
      for (int sw = 0; sw < 2; sw++) {
        int n = hal::i2cSoftScan(sw == 1, f, 16);
        print("software scan, %s:", sw ? "SDA/SCL SWAPPED" : "wires as configured");
        for (int i = 0; i < n; i++) print(" 0x%02X", f[i]);
        print(n ? "\r\n" : " nothing\r\n");
        if (n && sw == 0) print("-> chips answer by software but not to the I2C peripheral: firmware / pin setup problem\r\n");
        if (n && sw == 1) print("-> SDA and SCL are swapped: SDA must go to the SDA pin, SCL to the SCL pin\r\n");
      }
    }
    if (hal::spiHasDevice(hal::SpiDev::IMU)) {
      const uint8_t regs[] = {0x72, 0x75, 0x00, 0x0F};
      const char* names[] = {"0x72 (ICM-456xx: E9/E5)", "0x75 (MPU/ICM-42xxx)", "0x00 (BMI088 gyro: 0F)", "0x0F (LSM6: 6C/6B/6A)"};
      for (int i = 0; i < 4; i++) {
        uint8_t v = 0;
        hal::spiRead(hal::SpiDev::IMU, regs[i], &v, 1);
        print("SPI IMU reg %s = 0x%02X\r\n", names[i], v);
      }
      print("(all 0xFF: MISO not connected / chip unpowered; all 0x00: MISO shorted low or CS wrong)\r\n");
    }
  } else if (!strcmp(cmd, "status")) {
    cmdStatus();
  } else if (!strcmp(cmd, "wifi")) {
    // USB only: the recovery path when the WiFi or web password is forgotten
    if (a1) {
      if (armed) { print("disarm first\r\n"); return; }
      const char* err = nullptr;
      if (!strcmp(a1, "reset")) creds::resetDefaults();
      else if (!a2) err = "missing value";
      else if (!strcmp(a1, "name")) err = creds::setSsid(a2);
      else if (!strcmp(a1, "pass")) err = creds::setWifiPass(a2);
      else if (!strcmp(a1, "webpass")) err = creds::setWebPass(strcmp(a2, "-") ? a2 : "");
      else err = "unknown option";
      if (err) { print("%s\r\n", err); creds::load(); return; }
      print(creds::save() ? "saved\r\n" : "save FAILED\r\n");
      hal::wifiStart(creds::ssid(), creds::wifiPass());
    }
    print("WiFi \"%s\" password %s | web password %s%s\r\n", creds::ssid(), creds::wifiPass(),
          strcmp(creds::webPass(), creds::wifiPass()) ? "(separate)" : "(same as WiFi)",
          creds::usingDefaultPassword() ? " | FACTORY DEFAULT - change it" : "");
  } else if (!strcmp(cmd, "perf")) {
    const volatile bus::LoopPerf& p = bus::loopPerf;
    print("fast loop (mean us per %.0f us period): imu %.1f | filters %.1f | attitude %.1f | pid+mixer %.1f | "
          "motors %.1f\r\n", (double)p.periodUs, (double)p.imuUs, (double)p.filterUs, (double)p.ahrsUs,
          (double)p.pidUs, (double)p.motorUs);
    print("total %.1f us (%.1f%% load), worst %.1f us, late loops %lu\r\n", (double)p.totalUs, (double)p.loadPct,
          (double)p.maxTotalUs, (unsigned long)p.lateLoops);
    {
      uint32_t per = os::taskCount() ? os::taskStats(0).periodUs : 0;  // task 0 = the fast loop
      print("loop %.0f Hz, started by %s\r\n", per ? 1e6 / per : 0.0, os::triggerSource());
    }
  } else if (!strcmp(cmd, "esc")) {
    // per output: bidirectional DShot eRPM + extended telemetry (AM32 / Bluejay / BLHeli_32)
    if (!hal::motorsBidirActive()) {
      print("bidirectional DShot is off (set dshot_bidir 1, DShot300/600, save, reboot)\r\n");
    } else {
      float erpm[4];
      hal::motorsErpm(erpm);
      uint32_t fr, er;
      hal::motorsBidirStats(fr, er);
      print("out  eRPM     temp   volt    amp  stress status  edt frames\r\n");
      for (int o = 0; o < 4; o++) {
        dshot::Edt e;
        bool edt = hal::motorsEdt(o, e);
        print("M%d %7.0f  %4.0fC %5.2fV %5.0fA  %5u   0x%02X  %lu\r\n", o + 1, (double)erpm[o],
              (double)(edt ? e.tempC : -1), (double)(edt ? e.volts : -1), (double)(edt ? e.amps : -1),
              edt ? (unsigned)e.stress : 0u, edt ? (unsigned)e.status : 0u, (unsigned long)(edt ? e.frames : 0));
      }
      print("answers %lu, errors %.1f%%%s\r\n", (unsigned long)fr, fr ? 100.0 * er / fr : 0.0,
            bus::config.dshot_edt ? "" : " (dshot_edt 0: extended telemetry not requested)");
    }
  } else if (!strcmp(cmd, "tasks")) {
    cmdTasks();
  } else if (!strcmp(cmd, "selftest")) {
    cmdSelftest();
  } else if (!strcmp(cmd, "crash")) {
    hal::CrashInfo ci;
    if (a1 && !strcmp(a1, "clear")) { hal::crashClear(); print("crash record cleared\r\n"); return; }
#if !defined(ALLQUAD_HOST)
    // bench check of the crash record / watchdog (disarmed, props off): "crash test" faults on purpose,
    // "crash hang" stops everything with interrupts off (only the hardware watchdog brings it back)
    if (a1 && (!strcmp(a1, "test") || !strcmp(a1, "hang"))) {
      if (armed) { print("disarm first\r\n"); return; }
      print(!strcmp(a1, "test") ? "faulting now...\r\n" : "hanging now (watchdog %s)...\r\n", hal::watchdogRunning() ? "running" : "OFF: power-cycle to recover");
      hal::delayMs(100);
      if (!strcmp(a1, "test")) __builtin_trap();
#if defined(AQ_STM32)
      __asm volatile("cpsid i");
#endif
      for (;;) {}
    }
#endif
    if (!hal::lastCrash(ci)) {
      print("no crash recorded since power-on%s\r\n", hal::watchdogRunning() ? " (watchdog running)" : "");
      return;
    }
    print("%s %s, %lu s after boot, in %s\r\n", hal::crashKindName(ci.kind),
          ci.bootsAgo ? "(an earlier run)" : "(caused the last reset)", (unsigned long)(ci.uptimeMs / 1000),
          ci.where[0] ? ci.where : "?");
    if (ci.pc) print("pc 0x%08lX lr 0x%08lX  (arm-none-eabi-addr2line -e firmware.elf <pc>)\r\n", (unsigned long)ci.pc, (unsigned long)ci.lr);
    if (ci.cfsr || ci.hfsr) print("cfsr 0x%08lX hfsr 0x%08lX addr 0x%08lX\r\n", (unsigned long)ci.cfsr, (unsigned long)ci.hfsr, (unsigned long)ci.addr);
    print("%u crash(es) recorded, %u boot(s) ago | 'crash clear' forgets it\r\n", ci.count, ci.bootsAgo);
  } else if (!strcmp(cmd, "get")) {
    cmdGet(a1);
  } else if (!strcmp(cmd, "set") && a1 && a2) {
    int i = paramFind(a1);
    if (i < 0) { print("unknown parameter '%s'\r\n", a1); return; }
    if (armed) { print("disarm first\r\n"); return; }
    const ParamInfo& p = paramInfo(i);
    if (!paramSet(bus::config, i, (float)atof(a2))) {
      print("out of range [%g .. %g]\r\n", (double)p.min, (double)p.max);
      return;
    }
    bus::configChanged();
    print("%s = %g%s\r\n", p.name, (double)paramGet(bus::config, i), p.reboot ? " (save + reboot to apply)" : "");
  } else if (!strcmp(cmd, "save")) {
    if (armed) { print("disarm first\r\n"); return; }
    configSeal(bus::config);
    bool ok = hal::storageSave(hal::Slot::CONFIG, &bus::config, sizeof(Config));
    ok = nav::save() && ok;       // mission and rally points edited from the CLI / a restore
    ok = nav::rallySave() && ok;
    print(ok ? "saved\r\n" : "save FAILED\r\n");
  } else if (!strcmp(cmd, "defaults")) {
    if (armed) { print("disarm first\r\n"); return; }
    configDefaults(bus::config);
    bus::configChanged();
    print("defaults loaded (not saved)\r\n");
  } else if (!strcmp(cmd, "calib_acc")) {
    bus::postRequest(bus::Request::ACC_CAL);
    print("keep the craft level and still, then 'save'\r\n");
  } else if (!strcmp(cmd, "calib_acc6")) {
    bus::postRequest(bus::Request::ACC_CAL, 6);
    print("6-point: put the craft on each of its 6 sides (level, upside down, nose down/up, left/right side down),\r\n"
          "hold each still ~1 s until it is confirmed, then 'save'\r\n");
  } else if (!strcmp(cmd, "compassmot")) {
    if (armed) { print("disarm first\r\n"); return; }
    if (!a1 || strcmp(a1, "go")) {
      print("compass interference vs current: PROPS ON, craft held / strapped down, current sensor or ESC telemetry\r\n"
            "needed. Motors ramp to 60 %% for 20 s. Type 'compassmot go' to start (to abort: unplug the battery).\r\n");
      return;
    }
    bus::postRequest(bus::Request::COMPASSMOT);
    print("compassmot started (20 s)\r\n");
  } else if (!strcmp(cmd, "fence")) {
    if (a1 && !strcmp(a1, "clear")) { nav::fenceSetCount(0); nav::fenceSave(); print("fence polygon cleared\r\n"); return; }
    if (a1 && !strcmp(a1, "add") && a2) {
      long lat = 0, lon = 0;
      if (sscanf(restBuf, "%ld %ld", &lat, &lon) != 2) { print("fence add <latE7> <lonE7>\r\n"); return; }
      int n = nav::fenceCount();
      if (n >= nav::FENCE_MAX) { print("fence full (%d)\r\n", nav::FENCE_MAX); return; }
      nav::MissionItem it{};
      it.cmd = nav::CMD_FENCE_POLYGON_INCLUSION;
      it.frame = 3;
      it.lat = (int32_t)lat; it.lon = (int32_t)lon;
      nav::fenceSet(n, it);
      nav::fenceSetCount(n + 1);
      nav::fenceSave();
      return;
    }
    print("fence polygon: %d vertices (fence_enable %u, fence_poly %u)\r\n", nav::fenceCount(), bus::config.fence_enable,
          bus::config.fence_poly);
  } else if (!strcmp(cmd, "goto")) {
    // goto <latE7> <lonE7> [alt m above home]: GUIDED fly-to (web map), armed and airborne only
    long lat = 0, lon = 0;
    float alt = NAN;
    if (!a1 || sscanf(restBuf, "%ld %ld %f", &lat, &lon, &alt) < 2) { print("goto <latE7> <lonE7> [alt]\r\n"); return; }
    if (!armed) { print("arm first\r\n"); return; }
    if (!bus::nav.get().posValid) { print("no position\r\n"); return; }
    bus::GuidedTarget g{};
    g.lat = (int32_t)lat;
    g.lon = (int32_t)lon;
    g.alt = alt;
    bus::webGcs = true;
    bus::gcsHeartbeatMs = now ? now : 1;
    bus::guided.publish(g);
    print("ok guided\r\n");
  } else if (!strcmp(cmd, "logic")) {
    // logic <1-8> <en> <aType> <aVal> <op> <bType> <bVal> <action> <arg> <val>
    if (a1 && a2) {
      int idx = atoi(a1) - 1, v[9];
      if (idx < 0 || idx >= LOGIC_RULES) { print("rule 1-%d\r\n", LOGIC_RULES); return; }
      if (sscanf(restBuf, "%d %d %d %d %d %d %d %d %d", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &v[6], &v[7], &v[8]) != 9) {
        print("logic <n> <enable> <A type> <A value> <op> <B type> <B value> <action> <arg> <value>\r\n"); return;
      }
      LogicRule& r = bus::config.logic[idx];
      r.enabled = (uint8_t)(v[0] != 0); r.aType = (uint8_t)v[1]; r.aVal = (int16_t)v[2]; r.op = (uint8_t)v[3];
      r.bType = (uint8_t)v[4]; r.bVal = (int16_t)v[5]; r.action = (uint8_t)v[6]; r.actArg = (uint8_t)v[7];
      r.actVal = (int16_t)v[8];
      if (r.aType > LOP_RULE || r.bType > LOP_RULE || r.op >= LO_COUNT || r.action >= LA_COUNT) {
        r = LogicRule{}; print("invalid rule (cleared)\r\n"); return;
      }
      bus::configChanged();
      print("ok (save to keep)\r\n");
      return;
    }
    print("operand: 0 const, 1 RC ch(1-16), 2 value (0 armed 1 alt dm 2 speed cm/s 3 home m 4 vbat cV 5 bat%% 6 amps dA\r\n"
          "  7 armed s 8 mode 9 link%% 10 roll 11 pitch 12 sats 13 mAh), 3 rule(0-7)\r\n"
          "op: 0 true 1 == 2 > 3 < 4 and 5 or 6 not 7 >= 8 <= 9 !=   action: 0 none 1 beep 2 mode box(arg)\r\n"
          "  3 RC override(arg ch, val us) 4 VTX power(val) 5 message 6 block arming\r\n");
    for (int i = 0; i < LOGIC_RULES; i++) {
      const LogicRule& r = bus::config.logic[i];
      print("logic %d %u %u %d %u %u %d %u %u %d  %s\r\n", i + 1, r.enabled, r.aType, r.aVal, r.op, r.bType, r.bVal,
            r.action, r.actArg, r.actVal, (bus::logicResults >> i) & 1 ? "TRUE" : "");
    }
  } else if (!strcmp(cmd, "calib_mag")) {
    uint32_t s = a1 ? (uint32_t)atoi(a1) : 30;
    bus::postRequest(bus::Request::MAG_CAL, s);
    print("rotate the craft in every direction for %u s, then 'save'\r\n", (unsigned)s);
  } else if (!strcmp(cmd, "calib_gyro")) {
    bus::postRequest(bus::Request::GYRO_CAL);
    print("gyro calibration restarted (keep still)\r\n");
  } else if (!strcmp(cmd, "mission") && a1 && !strcmp(a1, "add")) {
    if (armed) { print("disarm first\r\n"); return; }
    long v[4] = {0, 0, 0, 0};
    float f[5] = {0, 0, 0, 0, 0};
    char* t = strtok(restBuf, " ");
    for (int k = 0; k < 9 && t; k++, t = strtok(nullptr, " ")) {
      if (k < 4) v[k] = atol(t);
      else f[k - 4] = (float)atof(t);
    }
    nav::MissionItem it{};
    it.cmd = (uint16_t)v[0]; it.frame = (uint8_t)v[1]; it.lat = (int32_t)v[2]; it.lon = (int32_t)v[3];
    it.alt = f[0]; it.param1 = f[1]; it.param2 = f[2]; it.param3 = f[3]; it.param4 = f[4];
    it.autocontinue = 1;
    int n = nav::count();
    if (n >= nav::MISSION_MAX || !nav::supported(it.cmd)) { print("mission add: full or unsupported command\r\n"); return; }
    nav::set(n, it);
    nav::setCount(n + 1);
  } else if (!strcmp(cmd, "mission")) {
    cmdMission(a1);
  } else if (!strcmp(cmd, "rally")) {
    if (armed && a1) { print("disarm first\r\n"); return; }
    cmdRally(a1, restBuf);
  } else if (!strcmp(cmd, "diff") || !strcmp(cmd, "dump")) {
    diff(!strcmp(cmd, "dump") || (a1 && !strcmp(a1, "all")));
  } else if (!strcmp(cmd, "log")) {
    if (a1 && !strcmp(a1, "erase")) {
      if (armed) { print("disarm first\r\n"); return; }
      hal::logErase();
      print("log erase started (runs in the background, arming blocked until done)\r\n");
    }
    else print("onboard log: %lu bytes\r\n", (unsigned long)hal::logSize());
  } else if (!strcmp(cmd, "sd")) {
    sdlog::Info i = sdlog::info();
    if (a1 && !strcmp(a1, "mount")) { sdlog::remount(); print("looking for the card\r\n"); return; }
    if (a1 && !strcmp(a1, "rm") && a2) {
      if (armed) { print("disarm first\r\n"); return; }
      print(sdlog::remove(a2) ? "%s deleted\r\n" : "cannot delete %s\r\n", a2);
      return;
    }
    if (a1 && !strcmp(a1, "ls")) {
      if (armed) { print("disarm first\r\n"); return; }
      if (!sdlog::list([](const char* n, uint32_t sz, void* c) { static_cast<Cli*>(c)->print("%s %10lu\r\n", n, (unsigned long)sz); }, this))
        print("no card / busy\r\n");
      return;
    }
    static const char* modes[] = {"off", "SPI", "SD 1-bit", "SD 4-bit"};
    print("sd: %s, mode %s%s, %s %s\r\n", sdlog::stateName(i.state), modes[i.mode & 3], hal::sdDmaActive() ? " (DMA)" : "", i.type, i.err);
    if (i.total) print("  %.1f / %.1f GB used, %u logs\r\n", i.used / 1e9, i.total / 1e9, (unsigned)i.files);
    print("  file %s %lu bytes, queue %lu / %lu, dropped %lu, slowest write %lu ms\r\n", i.file, (unsigned long)i.fileBytes,
          (unsigned long)i.queued, (unsigned long)i.queueSize, (unsigned long)i.dropped, (unsigned long)i.maxWriteMs);
    print("  pins clk %d cmd %d d0 %d d1 %d d2 %d d3 %d (255 = unused)\r\n", i.pin[0], i.pin[1], i.pin[2], i.pin[3], i.pin[4], i.pin[5]);
  } else if (!strcmp(cmd, "version")) {
    print("AllQuad %s (%s), %d parameters\r\n", info_ ? info_->version : "?", hal::platformName(), paramCount());
  } else if (!strcmp(cmd, "bootloader") || !strcmp(cmd, "bl")) {
    if (armed) { print("disarm first\r\n"); return; }
    if (!hal::bootloaderSupported()) { print("not from software on this board: hold BOOT and press RESET\r\n"); return; }
    print("rebooting into the bootloader...\r\n");
    hal::delayMs(50);
    hal::rebootToBootloader();
  } else if (!strcmp(cmd, "reboot")) {
    if (armed) { print("disarm first\r\n"); return; }
    print("rebooting...\r\n");
    hal::delayMs(50);
    hal::reboot();
  } else {
    print("unknown command, type 'help'\r\n");
  }
  (void)now;
}

}  // namespace comms
