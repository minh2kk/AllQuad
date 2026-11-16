#include "../common/size_opt.h"  // -Os on ESP32 (not flight critical)
#include "cli.h"

#include "../web/credentials.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../bus/bus.h"
#include "../logging/sdlog.h"
#include "../navigation/mission.h"

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
    print("status | tasks | perf | esc | get [filter] | set <name> <value> | save | defaults\r\n"
          "calib_acc | calib_acc6 | calib_mag [s] | calib_gyro | compassmot | mission [clear|add ...] | rally [clear|add ...]\r\n"
          "fence [clear|add <latE7> <lonE7>] | logic [n ...] | log [erase] | sd [ls | rm <file> | mount] | goto <latE7> <lonE7> [alt] (GUIDED)\r\n"
          "diff (backup: changed settings) | dump (backup: everything)\r\n"
          "wifi [name <ssid> | pass <8-63 chars> | webpass <6-64 chars, - = same as wifi> | reset]\r\n"
          "version | reboot | bootloader (USB flashing: ESP32-S3 ROM / STM32 DFU) | scan (wiring check) | net (WiFi check)\r\n");
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
    print("sd: %s, mode %s, %s %s\r\n", sdlog::stateName(i.state), modes[i.mode & 3], i.type, i.err);
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
