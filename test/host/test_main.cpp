// AllQuad host unit tests.
//   build/tests                         run all unit tests
//   build/tests mavlink-io in out       MAVLink conversation driver used by tools/test_mavlink.py
#include "../../src/hal/pinmap.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <random>
#include <vector>
#include <map>
#include <set>
#include <string>

#include "../../src/bus/bus.h"
#include "../../src/comms/aqlink.h"
#include "../../src/comms/cli.h"
#include "../../src/comms/crsf_msp.h"
#include "../../src/hal/ee_ab.h"
#include "../../src/comms/fourway.h"
#include "../../src/app/leds.h"
#include "../../src/vtx/vtx.h"
#include "../../src/comms/mavlink.h"
#include "../../src/comms/msp.h"
#include "../../src/control/mixer.h"
#include "../../src/estimation/kalman.h"
#include "../../src/estimation/ekf.h"
#include "../../src/estimation/baro_comp.h"
#include "../../src/app/sticks.h"
#include "../../src/osd/menu.h"
#include "../../src/estimation/range_height.h"
#include "../../src/control/pos_control.h"
#include "../../src/config/config.h"
#include "../../src/estimation/mag_fit.h"
#include "../../src/control/pid.h"
#include "../../src/navigation/navigator.h"
#include "../../src/control/autotune.h"
#include "../../src/hal/dshot_codec.h"
#include "../../src/filter/dynamic_notch.h"
#include "../../src/gps/gps_driver.h"
#include "../../src/logging/blackbox.h"
#include "../../src/logging/gyro_tap.h"
#include "../../src/logging/sdlog.h"
#include "../../src/navigation/mission.h"
#include "../../src/osd/osd.h"
#include "../../src/rx/rx_parser.h"
#include "../../src/safety/safety.h"
#include "../../src/sensors/sensors.h"
#include "../../src/sensors/serial_sensors.h"
#include "../../src/web/credentials.h"
#include "../../src/web/webapi.h"
#include "hal_host.h"

using namespace fc;

static int failures = 0, total = 0;
#define CHECK(cond, msg)                                              \
  do {                                                                \
    total++;                                                          \
    if (cond) printf("[PASS] %s\n", msg);                             \
    else { printf("[FAIL] %s  (%s)\n", msg, #cond); failures++; }     \
  } while (0)

static void pack11(const uint16_t ch[16], uint8_t out[22]) {
  memset(out, 0, 22);
  uint32_t bit = 0;
  for (int i = 0; i < 16; i++)
    for (int b = 0; b < 11; b++, bit++)
      if (ch[i] & (1 << b)) out[bit / 8] |= (uint8_t)(1 << (bit % 8));
}

// ---------------------------------------------------------------- receivers
static void testRx() {
  drv::RxParser rx;
  rx.begin(RxProtocol::CRSF);
  uint16_t t[16];
  for (int i = 0; i < 16; i++) t[i] = 992;
  t[0] = 172; t[1] = 1811;
  uint8_t f[26] = {0xC8, 24, 0x16};
  pack11(t, f + 3);
  f[25] = drv::crsfCrc8(f + 2, 23);
  bool got = false;
  for (uint8_t c : f) got |= rx.feed(c, 10);
  CHECK(got && abs(rx.rc().ch[0] - 988) <= 1 && abs(rx.rc().ch[1] - 2012) <= 1, "CRSF channels decoded");
  f[9] ^= 0x40;
  got = false;
  for (uint8_t c : f) got |= rx.feed(c, 20);
  CHECK(!got, "CRSF bad CRC rejected");
  CHECK(rx.counters().frames == 1 && rx.counters().crcErrors == 1 && rx.counters().bytes == 52,
        "RX counters: good frames, checksum failures, bytes");
  drv::RxTelemetry tel{};
  tel.mode = "ANGLE";
  uint8_t out[64];
  size_t n = rx.buildTelemetry(tel, out);
  CHECK(n == 10 && out[2] == 0x1E && drv::crsfCrc8(out + 2, 7) == out[9], "CRSF telemetry frame valid");
  {  // the rotation: no GPS fix -> the GPS slot is skipped, barometric altitude follows
    bool seen09 = false, seen02 = false;
    for (int k = 0; k < 12; k++) {
      size_t m = rx.buildTelemetry(tel, out);
      if (m && out[2] == 0x09) { seen09 = true; uint16_t v = (uint16_t)(out[3] << 8 | out[4]); CHECK(m == 6 && drv::crsfCrc8(out + 2, 3) == out[5] && v == (uint16_t)lroundf(tel.altitude * 10.0f) + 10000, "CRSF baro altitude frame"); }
      if (m && out[2] == 0x02) seen02 = true;
    }
    CHECK(seen09 && seen02 == tel.gpsValid, "CRSF telemetry rotation: baro sent, GPS only with a fix");
  }

  rx.begin(RxProtocol::SBUS);
  uint8_t s[25] = {0x0F};
  t[2] = 172;
  pack11(t, s + 1);
  got = false;
  for (uint8_t c : s) got |= rx.feed(c, 30);
  CHECK(got && abs(rx.rc().ch[2] - 988) <= 1, "SBUS decoded");

  rx.begin(RxProtocol::IBUS);
  uint8_t ib[32] = {0x20, 0x40};
  for (int i = 0; i < 14; i++) { uint16_t v = 1000 + i * 50; ib[2 + i * 2] = v & 0xFF; ib[3 + i * 2] = v >> 8; }
  uint16_t sum = 0xFFFF;
  for (int i = 0; i < 30; i++) sum -= ib[i];
  ib[30] = sum & 0xFF; ib[31] = sum >> 8;
  got = false;
  for (uint8_t c : ib) got |= rx.feed(c, 40);
  CHECK(got && rx.rc().ch[13] == 1650, "iBUS decoded");
}

// ---------------------------------------------------------------- GPS
static void testGps() {
  drv::GpsDriver gps;
  uint8_t p[92] = {0};
  auto put32 = [&](int o, int32_t v) { memcpy(p + o, &v, 4); };
  p[20] = 3; p[21] = 1; p[23] = 12;
  put32(24, (int32_t)(105.8542 * 1e7)); put32(28, (int32_t)(21.0285 * 1e7));
  put32(40, 900); put32(48, 1500);
  uint8_t f[100] = {0xB5, 0x62, 0x01, 0x07, 92, 0};
  memcpy(f + 6, p, 92);
  uint8_t a = 0, b = 0;
  for (int i = 2; i < 98; i++) { a += f[i]; b += a; }
  f[98] = a; f[99] = b;
  bool got = false;
  for (uint8_t c : f) got |= gps.feed(c);
  CHECK(got && gps.data().fix3d && fabs(gps.data().lat - 21.0285) < 1e-6, "UBX NAV-PVT decoded");
  drv::GpsDriver g2;
  const char* body = "GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,";
  uint8_t cs = 0;
  for (const char* q = body; *q; q++) cs ^= (uint8_t)*q;
  char line[128];
  snprintf(line, sizeof(line), "$%s*%02X\r\n", body, cs);
  got = false;
  for (const char* q = line; *q; q++) got |= g2.feed((uint8_t)*q);
  CHECK(got && g2.data().sats == 8, "NMEA GGA decoded");
  {  // a 9600 baud NMEA-only receiver: found by trying the baud rates, then kept
    os::hostReset();
    hal::host::reset();
    drv::GpsDriver g3;
    g3.attach(hal::Port::SERIAL2);
    g3.poll();
    os::hostRunUntil(3000000);
    g3.poll();  // 2.5 s without a frame at 115200
    uint32_t tried = hal::host::openBaud[(int)hal::Port::SERIAL2];
    for (int s = 0; s < 6; s++) {
      for (const char* q = line; *q; q++) hal::host::in(hal::Port::SERIAL2).push_back((uint8_t)*q);
      os::hostRunUntil(3000000 + (uint64_t)(s + 1) * 1000000);
      g3.poll();
    }
    CHECK(tried == 9600 && g3.baud() == 9600 && g3.frames() == 6 && hal::host::openBaud[(int)hal::Port::SERIAL2] == 9600,
          "GPS: no frame at 115200 -> 9600 tried, NMEA found there and kept");
    os::hostReset();
  }
}

// ---------------------------------------------------------------- serial sensors
static void testSerialSensors() {
  drv::TfminiParser tf;
  uint8_t fr[9] = {0x59, 0x59, 0x2C, 0x01, 0xE8, 0x03, 0, 0, 0};  // 300 cm, strength 1000
  for (int i = 0; i < 8; i++) fr[8] += fr[i];
  bool got = false;
  for (uint8_t c : fr) got |= tf.feed(c);
  CHECK(got && fabsf(tf.distM - 3.0f) < 1e-4f, "TFmini frame decoded");
  uint8_t e[10] = {45, 0x06, 0x40, 0x03, 0xE8, 0x00, 0x96, 0x01, 0x2C, 0};  // 45C 16.0V 10A 150mAh 30000erpm
  e[9] = drv::EscTelemetryParser::crc8(e, 9);
  drv::EscTelemetryParser ep;
  got = false;
  for (uint8_t c : e) got |= ep.feed(c);
  CHECK(got && fabsf(ep.voltage - 16.0f) < 1e-3f && fabsf(ep.erpm - 30000) < 1, "ESC telemetry (KISS) decoded");
}

// ---------------------------------------------------------------- MSP
static void testMsp() {
  hal::host::reset();
  bus::config = Config{};
  configDefaults(bus::config);
  comms::MspLink link;
  link.begin(hal::Port::SERIAL3);
  uint8_t req[16];
  size_t n = comms::mspPack(req, false, '<', 108, nullptr, 0);  // MSP_ATTITUDE
  for (size_t i = 0; i < n; i++) link.feed(req[i], 100);
  std::string& o = hal::host::out(hal::Port::SERIAL3);
  CHECK(o.size() == 6 + 6 && o[0] == '$' && o[1] == 'M' && o[2] == '>' && (uint8_t)o[4] == 108, "MSP v1 request answered");
  // MSP v2 optical-flow sensor message
  uint8_t pl[9] = {200};
  int32_t mx = 105, my = -210;
  memcpy(pl + 1, &mx, 4);
  memcpy(pl + 5, &my, 4);
  uint8_t fr[32];
  n = comms::mspPack(fr, true, '<', 0x1F02, pl, 9);
  for (size_t i = 0; i < n; i++) link.feed(fr[i], 200);
  for (size_t i = 0; i < n; i++) link.feed(fr[i], 220);  // second sample -> dt = 20 ms
  FlowSample fs = bus::flow.get();
  CHECK(bus::flow.seq() == 1 && fabsf(fs.rateX - 105 / (10.5f * 0.02f)) < 1e-3f, "MSP2 optical flow -> rad/s");
  uint8_t rp[5] = {255};
  int32_t mm = 1234;
  memcpy(rp + 1, &mm, 4);
  n = comms::mspPack(fr, true, '<', 0x1F01, rp, 5);
  for (size_t i = 0; i < n; i++) link.feed(fr[i], 230);
  CHECK(fabsf(bus::range.get().distM - 1.234f) < 1e-4f, "MSP2 rangefinder");
}

// ---------------------------------------------------------------- filters
static void testDynamicNotch() {
  DynamicNotch dn;
  dn.configure(1000, 100, 450, 3.5f, 1);
  std::mt19937 rng(1);
  std::normal_distribution<float> nd(0, 2);
  double inSq = 0, outSq = 0;
  for (int i = 0; i < 4000; i++) {
    float f = 230.0f;
    float s = 30.0f * sinf(2 * PI_F * f * i / 1000.0f);
    Vec3 g(s + nd(rng), 0.5f * s + nd(rng), nd(rng));
    Vec3 o = dn.apply(g);
    if (i > 3000) { inSq += s * s; outSq += (o.x - (g.x - s)) * (o.x - (g.x - s)); }
  }
  CHECK(fabsf(dn.centerHz(0) - 230.0f) < 10.0f, "Dynamic notch locks on 230 Hz resonance");
  CHECK(outSq < inSq * 0.05, "Dynamic notch removes > 95% of the resonance power");
  RpmFilter rf;
  rf.configure(1000, 3, 80, 5);
  float hz[4] = {180, 180, 180, 180};
  rf.setMotorHz(hz);
  double rin = 0, rout = 0;
  for (int i = 0; i < 3000; i++) {
    float s = 20 * sinf(2 * PI_F * 180 * i / 1000.0f) + 8 * sinf(2 * PI_F * 360 * i / 1000.0f);
    Vec3 o = rf.apply(Vec3(s, s, s));
    if (i > 2000) { rin += s * s; rout += o.x * o.x; }
  }
  CHECK(rout < rin * 0.01, "RPM filter removes motor harmonics (1st + 2nd)");
}

static void testKalman() {
  Kalman3 k;
  k.init(0, 0, 1, 1, 0.1f);
  std::mt19937 rng(2);
  std::normal_distribution<float> nz(0, 0.3f);
  float truthPos = 0, truthVel = 0;
  for (int i = 0; i < 5000; i++) {
    float a = sinf(i * 0.002f);
    truthVel += a * 0.004f;
    truthPos += truthVel * 0.004f;
    k.predict(a + 0.3f, 0.004f, 0.5f, 0.02f);  // biased accelerometer
    if (i % 12 == 0) k.update(0, truthPos + nz(rng), 0.09f);
  }
  CHECK(fabsf(k.pos() - truthPos) < 0.3f && fabsf(k.bias() - 0.3f) < 0.1f, "Kalman: position tracked, accel bias learned");
}

// ---------------------------------------------------------------- misc modules
static void testParams() {
  std::set<std::string> names;
  bool ok = true;
  for (int i = 0; i < paramCount(); i++) {
    const ParamInfo& p = paramInfo(i);
    if (strlen(p.name) > 16 || !names.insert(p.name).second) { ok = false; printf("  bad param name %s\n", p.name); }
  }
  CHECK(ok, "Parameter names unique and <= 16 chars (MAVLink)");
  Config c;
  configDefaults(c);
  int i = paramFind("roll_p");
  CHECK(i >= 0 && paramSet(c, i, 55) && fabsf(c.pid[0].p - 55) < 1e-6f && !paramSet(c, i, 999), "Param set/get/range");
  CHECK(configValid(c) == false && (configSeal(c), configValid(c)), "Config CRC");
  printf("  %d parameters, Config %u bytes, MissionStore %u bytes\n", paramCount(), (unsigned)sizeof(Config),
         (unsigned)sizeof(nav::MissionStore));
}

static void testModeRanges() {
  Config c;
  configDefaults(c);
  uint16_t ch[RC_MAX_CH];
  for (auto& v : ch) v = 1000;
  CHECK(modeBoxes(c, ch, RC_MAX_CH) == 0, "Mode ranges: all switches low -> no box (ACRO, disarmed)");
  ch[4] = 2000; ch[5] = 1500; ch[6] = 2000;
  uint32_t b = modeBoxes(c, ch, RC_MAX_CH);
  CHECK(boxOn(b, ModeBox::ARM) && boxOn(b, ModeBox::ANGLE) && boxOn(b, ModeBox::RTH) && !boxOn(b, ModeBox::ALTHOLD),
        "Mode ranges: default layout (ARM, ANGLE, RTH)");
  int i = paramFind("mr10_mode");
  CHECK(i >= 0 && paramSet(c, i, (float)ModeBox::HORIZON) && paramSet(c, paramFind("mr10_ch"), 10) &&
        paramSet(c, paramFind("mr10_lo"), 1400) && paramSet(c, paramFind("mr10_hi"), 1600),
        "Mode ranges: configurable through parameters");
  ch[10] = 1500;
  CHECK(boxOn(modeBoxes(c, ch, RC_MAX_CH), ModeBox::HORIZON), "Mode ranges: custom range on CH11 activates HORIZON");
  ch[10] = 1600;
  CHECK(!boxOn(modeBoxes(c, ch, RC_MAX_CH), ModeBox::HORIZON), "Mode ranges: upper bound exclusive");
}

static void testDeferredSave() {
  hal::host::reset();
  VehicleStatus st{};
  st.armed = true;
  bus::status.publish(st);
  nav::MissionItem it{};
  it.cmd = nav::CMD_WAYPOINT; it.lat = 1; it.lon = 2; it.alt = 5;
  nav::set(0, it);
  nav::setCount(1);
  nav::save();  // armed: must not touch the flash (it would stall the CPU in flight)
  nav::MissionStore probe;
  bool written = hal::storageLoad(hal::Slot::MISSION, &probe, sizeof(probe));
  st.armed = false;
  bus::status.publish(st);
  nav::persistPending();
  bool after = hal::storageLoad(hal::Slot::MISSION, &probe, sizeof(probe)) && probe.count == 1;
  CHECK(!written && after, "Flash writes postponed while armed, done after disarm");
  nav::clear();
}

static void testAlignMixer() {
  Vec3 r = drv::applyAlign(Vec3(1, 0, 0), Align::CW90);
  CHECK(fabsf(r.y - 1) < 1e-6f, "CW90 alignment");
  float out[4];
  Mixer::turtle(1.0f, 0.0f, out);
  CHECK(out[0] < 0 && out[1] < 0 && out[2] > 0.5f && out[3] > 0.5f, "Turtle: roll right spins left motors");
  Mixer m;
  m.configure(false, true);
  m.mix(0.5f, 0.1f, 0, 0, out);
  CHECK(out[2] > out[0] && out[3] > out[1], "Mixer: positive roll raises left motors");
}

static void testBatteryFence() {
  CHECK(BatteryMonitor::lipoPercent(4.2f) == 100 && BatteryMonitor::lipoPercent(3.3f) == 0 &&
            BatteryMonitor::lipoPercent(3.8f) == 50, "LiPo percentage curve");
  Config c;
  configDefaults(c);
  c.fence_enable = 1;
  c.fence_radius = 50;
  NavEstimate e{};
  e.homeSet = e.posValid = e.altValid = true;
  e.n = 40; e.e = 40;
  CHECK(fenceBreached(c, e), "Geofence breach detected");
}

static void testOsdBlackbox() {
  configDefaults(bus::config);
  BatteryState b{};
  b.voltage = 16.4f;
  bus::battery.publish(b);
  osd::Renderer r;
  r.begin(nullptr, true);
  r.render(1000);
  const char volt[] = {osd::sym::BATT, '1', '6', '.', '4', osd::sym::VOLT, 0};
  CHECK(strstr(r.row(14), volt) != nullptr, "OSD renders battery voltage with Betaflight glyphs");
  bus::config.osd_font = 1;
  r.render(1100);
  CHECK(strstr(r.row(14), "16.4V") != nullptr, "OSD plain-text font");
  bus::config.osd_font = 0;
  {
    // battery bar / cell voltage (v26), and the low-battery blink while armed
    BatteryState lb{};
    lb.voltage = 14.0f; lb.cells = 4; lb.percent = 31;
    bus::battery.publish(lb);
    bus::config.osd_font = 1;
    bus::config.osd_pos3[OSD3_BAT_BAR] |= 0x8000;
    bus::config.osd_pos3[OSD3_CELL_V] |= 0x8000;
    VehicleStatus vs{};
    vs.armed = true;
    vs.warnings = WARN_BAT_LOW;
    bus::status.publish(vs);
    osd::Renderer br;
    br.begin(nullptr, true);
    br.render(400);   // blink phase "on"
    if (getenv("AQ_SHOW_OSD")) for (int q = 0; q < 16; q++) printf("|%s|\n", br.row(q));
    bool barOn = strstr(br.row(12), "[##=-----]") != nullptr && strstr(br.row(5), "3.50V/C") != nullptr && strstr(br.row(14), "14.0V");
    br.render(800);   // "off"
    bool barOff = !strstr(br.row(12), "[") && !strstr(br.row(14), "14.0V") && strstr(br.row(14), "MAH");
    bus::config.osd_blink = 0;
    br.render(1200);
    bool steady = strstr(br.row(12), "[") && strstr(br.row(14), "14.0V");
    CHECK(barOn && barOff && steady, "OSD battery bar + cell voltage; low battery blinks unless osd_blink 0");
    configDefaults(bus::config);
    bus::status.publish(VehicleStatus{});
    bus::battery.publish(b);
  }
  {
    // flight statistics: distance and maxima while armed, the stats screen after disarm
    osd::Renderer sr;
    sr.begin(nullptr, true);
    VehicleStatus vs{};
    NavEstimate ne{};
    ne.posValid = ne.altValid = ne.homeSet = true;
    vs.armed = true;
    for (int k = 0; k <= 100; k++) {  // 10 s at 10 Hz, 5 m/s east
      ne.e = k * 0.5f; ne.ve = 5.0f; ne.alt = 20.0f;
      vs.homeDistance = ne.e;
      bus::status.publish(vs);
      bus::nav.publish(ne);
      sr.render(5000 + k * 100);
    }
    vs.armed = false;
    bus::status.publish(vs);
    sr.render(15200);
    bool stats = false, dist = false;
    for (int r = 0; r < 16; r++) {
      if (strstr(sr.row(r), "FLIGHT STATS")) stats = true;
      if (strstr(sr.row(r), "DISTANCE   0.05 KM")) dist = true;
    }
    CHECK(stats && dist, "OSD statistics screen after disarm (distance flown 50 m)");
    sr.render(15200 + bus::config.osd_stats * 1000 + 200);
    bool gone = true;
    for (int r = 0; r < 16; r++) if (strstr(sr.row(r), "FLIGHT STATS")) gone = false;
    CHECK(gone, "OSD statistics screen clears after osd_stats seconds");
    bus::status.publish(VehicleStatus{});
    bus::nav.publish(NavEstimate{});
  }
  {
    // digital link: HD canvas, Betaflight identity, display-requested canvas, anchors kept
    hal::host::reset();
    comms::MspLink link;
    link.begin(hal::Port::SERIAL2);
    link.setDisplayPort(true);
    comms::MspFrame f{};
    f.dir = '<';
    f.cmd = 2;  // MSP_FC_VARIANT
    link.handle(f, 10);
    CHECK(hal::host::out(hal::Port::SERIAL2).find("BTFL") != std::string::npos,
          "DisplayPort link reports FC variant BTFL (goggles load the Betaflight font)");
    osd::DisplayPort dp(&link);
    osd::Renderer hd;
    hd.begin(&dp, true);
    hd.render(2000);
    CHECK(hd.cols() == 53 && hd.rows() == 20, "DisplayPort auto canvas: 53 x 20 (DJI O3 / O4, Walksnail)");
    const char* bottom = hd.row(18);  // battery row (14 of 16, 1 row above the bottom) keeps its gap
    CHECK(strstr(bottom, volt) == bottom + 1, "HD canvas: bottom-left element keeps its corner");
    f.cmd = 188;  // MSP_SET_OSD_CANVAS from an HDZero goggle
    f.len = 2;
    f.payload[0] = 50;
    f.payload[1] = 18;
    link.handle(f, 20);
    hd.render(2100);
    CHECK(hd.cols() == 50 && hd.rows() == 18, "display-requested canvas (HDZero 50 x 18) is followed");
    int c, rr;
    osd::Renderer::canvasSize(osd::Canvas::AUTO, false, false, 0, 0, c, rr);
    CHECK(c == 30 && rr == 13, "analog MAX7456 stays 30 x 13 NTSC");
    hal::host::out(hal::Port::SERIAL2).clear();
  }
  logging::Record rec;
  logging::Blackbox::fill(rec, 1234);
  uint8_t x = 0;
  for (size_t i = 0; i < sizeof(rec) - 1; i++) x ^= ((uint8_t*)&rec)[i];
  CHECK(rec.sync == 0xA5 && rec.len == sizeof(rec) && rec.crc == x, "Blackbox record framing");
}

static void jsonSink(const char* d, size_t n, void* ctx) { ((std::string*)ctx)->append(d, n); }
static void testJsonNumbers() {
  std::string out;
  {
    web::JsonOut j(jsonSink, &out);
    j.begin('[');
    const double v[] = {0, -0.0001, 1.5, -2.25, 16.4, 100, 21.0285123, 1e15 * 10, 0.1 + 0.2, -7.0};
    const int d[] = {3, 2, 2, 2, 1, 0, 7, 1, 3, 0};
    for (int i = 0; i < 10; i++) { j.comma(); j.num(v[i], d[i]); }
    j.end(']');
  }
  printf("  json: %s\n", out.c_str());
  CHECK(out == "[0,0,1.5,-2.25,16.4,100,21.0285123,0,0.3,-7]", "JSON numbers formatted without printf");
  std::string full, a, b;
  web::statusJson(jsonSink, &full, true);
  web::statusJson(jsonSink, &a, false);
  web::statusJson(jsonSink, &b, false);
  const std::string& withEst = a.find("\"est\"") != std::string::npos ? a : b;
  const std::string& light = a.find("\"est\"") != std::string::npos ? b : a;
  printf("  status bytes: full %zu, mid %zu, light %zu\n", full.size(), withEst.size(), light.size());
  CHECK(light.find("\"est\"") == std::string::npos && withEst.find("\"est\"") != std::string::npos &&
            light.find("\"at\"") == std::string::npos && light.size() < withEst.size() * 0.75,
        "WebSocket frames: estimator / GPS every other frame, autotune block only while it runs");
}

// STM32 settings A/B copies: what the next boot reads after a power loss at every point of a save
static void testEeAb() {
  static uint8_t A[ee::SIZE], B[ee::SIZE], img[ee::SIZE];
  uint8_t* cp[2] = {A, B};
  const uint8_t* cc[2] = {A, B};
  memset(A, 0x11, sizeof(A));        // settings saved by an older firmware: no footer
  memset(A + 7800, 0xFF, ee::SIZE - 7800);
  memset(B, 0xFF, sizeof(B));        // spare sector: erased (or old blackbox data)
  uint32_t seq;
  int act = ee::pick(cc, 2, seq);
  CHECK(act == 0 && seq == 0, "settings A/B: image without footer read from A");
  // save 1: written to B, A untouched
  memcpy(img, cp[act], ee::SIZE);
  img[0] = 0x22;
  uint32_t ns;
  int tgt = ee::seal(img, act, seq, 2, ns);
  bool okT = tgt == 1 && ns == 1;
  // power lost half way through programming B: still the old A
  memset(B, 0xFF, sizeof(B));
  memcpy(B, img, 4000);
  act = ee::pick(cc, 2, seq);
  bool okHalf = act == 0 && seq == 0 && A[0] == 0x11;
  memcpy(B, img, ee::SIZE);
  act = ee::pick(cc, 2, seq);
  CHECK(okT && okHalf && act == 1 && seq == 1 && B[0] == 0x22, "settings A/B: first save goes to B, a half-written B is ignored");
  // save 2 erases A: lost during the erase -> B (save 1) is used
  memcpy(img, cp[act], ee::SIZE);
  img[0] = 0x33;
  tgt = ee::seal(img, act, seq, 2, ns);
  memset(A, 0xFF, sizeof(A));
  int a2 = ee::pick(cc, 2, seq);
  bool okErase = tgt == 0 && a2 == 1 && seq == 1;
  memcpy(A, img, ee::SIZE);
  act = ee::pick(cc, 2, seq);
  CHECK(okErase && act == 0 && seq == 2 && A[0] == 0x33, "settings A/B: power loss while erasing keeps the previous save");
  // a flipped bit in the newest copy -> the older one
  A[100] ^= 4;
  act = ee::pick(cc, 2, seq);
  bool okFlip = act == 1 && seq == 1;
  A[100] ^= 4;
  // sequence wrap-around: 0xFFFFFFFF -> 1 is newer
  memcpy(img, A, ee::SIZE);
  ee::seal(img, 1, 0xFFFFFFFEu, 2, ns);
  memcpy(B, img, ee::SIZE);   // B = 0xFFFFFFFF
  memcpy(img, B, ee::SIZE);
  img[0] = 0x44;
  ee::seal(img, 1, 0xFFFFFFFFu, 2, ns);
  memcpy(A, img, ee::SIZE);   // A = 1
  act = ee::pick(cc, 2, seq);
  CHECK(okFlip && ns == 1 && act == 0 && A[0] == 0x44, "settings A/B: corrupt copy skipped, sequence wraps");
  // one copy (F722 / F411)
  const uint8_t* one[1] = {A};
  act = ee::pick(one, 1, seq);
  CHECK(act == 0 && ee::seal(img, 0, seq, 1, ns) == 0, "settings: single copy always rewritten in place");
}

// MSP over CRSF the way the radio's Lua script sends it: 8-byte chunks in 0x7A frames through the receiver
static std::vector<std::vector<uint8_t>> luaMspFrames(bool v2, uint16_t cmd, const std::vector<uint8_t>& data, uint8_t& seq) {
  std::vector<uint8_t> body;
  if (v2) body = {0, (uint8_t)cmd, (uint8_t)(cmd >> 8), (uint8_t)data.size(), (uint8_t)(data.size() >> 8)};
  else body = {(uint8_t)data.size(), (uint8_t)cmd};
  body.insert(body.end(), data.begin(), data.end());
  if (!v2) { uint8_t x = 0; for (size_t i = 0; i < body.size(); i++) x ^= body[i]; body.push_back(x); }
  std::vector<std::vector<uint8_t>> frames;
  for (size_t pos = 0; pos < body.size(); pos += 7) {
    std::vector<uint8_t> pl = {0xC8, 0xEA, (uint8_t)((v2 ? 0x40 : 0x20) | (pos == 0 ? 0x10 : 0) | (seq++ & 0x0F))};
    for (size_t i = pos; i < body.size() && i < pos + 7; i++) pl.push_back(body[i]);
    std::vector<uint8_t> f = {0xC8, (uint8_t)(pl.size() + 2), 0x7A};
    f.insert(f.end(), pl.begin(), pl.end());
    f.push_back(drv::crsfCrc8(f.data() + 2, (uint8_t)(f.size() - 2)));
    frames.push_back(f);
  }
  return frames;
}
// response frames -> {error, payload}; checks CRC, addresses and the chunk sequence like the Lua script
static bool luaMspAnswer(const std::string& wire, bool& err, std::vector<uint8_t>& out) {
  out.clear();
  size_t i = 0, size = 0;
  bool started = false;
  int lastSeq = -1;
  while (i + 2 < wire.size()) {
    const uint8_t* f = (const uint8_t*)wire.data() + i;
    uint8_t len = f[1];
    if (i + 2 + len > wire.size()) return false;
    if (f[0] != 0xC8 || drv::crsfCrc8(f + 2, (uint8_t)(len - 1)) != f[len + 1]) return false;
    i += 2 + len;
    if (f[2] != 0x7B) continue;  // telemetry
    if (f[3] != 0xEA || f[4] != 0xC8) return false;
    const uint8_t* c = f + 5;
    size_t n = len - 4;
    uint8_t st = c[0];
    if (st & 0x10) {
      err = st & 0x80;
      bool v2 = ((st >> 5) & 3) == 2;
      size = v2 ? (c[4] | c[5] << 8) : c[1];
      size_t h = v2 ? 6 : 3;
      out.assign(c + h, c + std::min(n, h + size));
      started = true;
    } else {
      if (!started || (st & 0x0F) != ((lastSeq + 1) & 0x0F)) return false;
      for (size_t k = 1; k < n && out.size() < size; k++) out.push_back(c[k]);
    }
    lastSeq = st & 0x0F;
  }
  return started && out.size() == size;
}
static void testCrsfMsp() {
  hal::host::reset();
  configDefaults(bus::config);
  bus::status.publish(VehicleStatus{});
  drv::RxParser rx;
  rx.begin(RxProtocol::CRSF);
  comms::MspLink link;
  comms::CrsfMsp cm;
  link.begin(hal::Port::SERIAL1);
  link.setReplyHook([](void* ctx, const comms::MspFrame& q, const uint8_t* p, uint16_t n, bool e) {
    static_cast<comms::CrsfMsp*>(ctx)->respond(q, p, n, e); }, &cm);
  uint8_t seq = 0;
  auto ask = [&](bool v2, uint16_t cmd, const std::vector<uint8_t>& d, bool& err, std::vector<uint8_t>& ans) {
    for (auto& f : luaMspFrames(v2, cmd, d, seq))
      for (uint8_t b : f) {
        rx.feed(b, 10);
        const uint8_t* ch; size_t cn; uint8_t org; comms::MspFrame mf;
        if (rx.takeMsp(ch, cn, org) && cm.feed(ch, cn, org, mf)) link.handle(mf, 10);
      }
    std::string wire;
    uint8_t fr[64];
    size_t n;
    while ((n = cm.nextFrame(fr))) wire.append((const char*)fr, n);
    return luaMspAnswer(wire, err, ans);
  };
  auto i32 = [](const std::vector<uint8_t>& a, size_t o) { return (int32_t)(a[o] | a[o + 1] << 8 | a[o + 2] << 16 | (uint32_t)a[o + 3] << 24); };
  bool err = true;
  std::vector<uint8_t> a;
  std::string nm = "dterm_lpf_hz";
  bool ok1 = ask(true, 0x4152, std::vector<uint8_t>(nm.begin(), nm.end()), err, a) && !err && a.size() >= 18 &&
             std::string(a.begin() + 18, a.end()) == nm && fabsf(i32(a, 6) * 0.001f - bus::config.dterm_lpf_hz) < 0.01f;
  int idx = a.size() >= 2 ? a[0] | a[1] << 8 : -1;
  CHECK(ok1 && idx == paramFind("dterm_lpf_hz"), "MSP over CRSF: parameter found by name (request in 3 chunks)");
  int32_t v = 87500;
  std::vector<uint8_t> set = {(uint8_t)idx, (uint8_t)(idx >> 8), (uint8_t)v, (uint8_t)(v >> 8), (uint8_t)(v >> 16), (uint8_t)(v >> 24)};
  bool ok2 = ask(true, 0x4153, set, err, a) && !err && fabsf(bus::config.dterm_lpf_hz - 87.5f) < 0.01f && i32(a, 6) == 87500;
  CHECK(ok2, "MSP over CRSF: parameter set (dterm_lpf_hz 87.5)");
  bool ok3 = ask(true, 0x4151, {0xFF, 0x7F}, err, a) && err;
  bool ok4 = ask(false, 250, {}, err, a) && !err && a.size() == 1 && a[0] == 1;
  CHECK(ok3 && ok4, "MSP over CRSF: bad index refused, settings saved (MSP_EEPROM_WRITE, v1)");
  VehicleStatus armedSt{};
  armedSt.armed = true;
  bus::status.publish(armedSt);
  bool ok5 = ask(true, 0x4153, set, err, a) && err;
  bus::status.publish(VehicleStatus{});
  // a long answer (MSP_BOARD_INFO / a name) spans several 0x7B frames; bad CRC frames are ignored
  bool ok6 = ask(false, 4, {}, err, a) && !err && a.size() >= 4;
  CHECK(ok5 && ok6, "MSP over CRSF: no changes while armed; v1 answers");
  {
    comms::MspFrame q{};
    q.v2 = true; q.cmd = 0x4151;
    uint8_t big[150];
    for (int k = 0; k < 150; k++) big[k] = (uint8_t)(k * 7);
    cm.respond(q, big, 150, false);
    std::string wire;
    uint8_t fr[64];
    size_t n, frames = 0;
    while ((n = cm.nextFrame(fr))) { wire.append((const char*)fr, n); frames++; if (n > 64) frames = 99; }
    bool same = luaMspAnswer(wire, err, a) && a.size() == 150 && !memcmp(a.data(), big, 150);
    CHECK(same && frames == 3, "MSP over CRSF: 150-byte answer in 3 frames of <= 64 bytes");
  }
}

static void testCli() {
  hal::host::reset();
  configDefaults(bus::config);
  comms::SystemInfo info{"SIM", nullptr, nullptr, "2.0.0"};
  comms::Cli cli;
  cli.begin(hal::Port::CONSOLE, &info);
  for (char ch : std::string("set roll_p 50\nget roll_p\nset nope 1\nstatus\n")) cli.feed((uint8_t)ch, 100, false);
  std::string& o = hal::host::out(hal::Port::CONSOLE);
  CHECK(o.find("roll_p = 50") != std::string::npos && o.find("unknown parameter") != std::string::npos &&
            o.find("arming") != std::string::npos, "CLI set/get/status");
  hal::host::out(hal::Port::CONSOLE).clear();
  for (char ch : std::string("status\n")) cli.feed((uint8_t)ch, 200, true);
  CHECK(hal::host::out(hal::Port::CONSOLE).empty(), "CLI silent while MAVLink/MSP active");
  hal::host::out(hal::Port::CONSOLE).clear();
  for (char ch : std::string("goto 210285000 1058542000 20\n")) cli.feed((uint8_t)ch, 300, false);
  CHECK(hal::host::out(hal::Port::CONSOLE).find("arm first") != std::string::npos && !bus::webGcs,
        "CLI goto (web tap-to-fly) refused while disarmed");
  hal::host::out(hal::Port::CONSOLE).clear();
  for (char ch : std::string("selftest\n")) cli.feed((uint8_t)ch, 400, false);
  {
    const std::string& so = hal::host::out(hal::Port::CONSOLE);
    CHECK(so.find("[ OK ] reset") != std::string::npos && so.find("[WARN] baro") != std::string::npos &&
              so.find("[WARN] rx") != std::string::npos && so.find("selftest: ") != std::string::npos,
          "CLI selftest: one line per check, summary");
    // two receivers configured: reported (the second one is ignored by the firmware)
    bus::config.port_fn[0] = (uint8_t)PortFn::RX;
    bus::config.port_fn_ext2[1] = (uint8_t)PortFn::RX;
    hal::host::out(hal::Port::CONSOLE).clear();
    for (char ch : std::string("selftest\n")) cli.feed((uint8_t)ch, 500, false);
    const std::string& s2 = hal::host::out(hal::Port::CONSOLE);
    CHECK(s2.find("[WARN] ports     receiver on both SERIAL1 and SERIAL7: only SERIAL1 is used") != std::string::npos,
          "CLI selftest: the same single-use function on two serial ports is reported");
    bus::config.port_fn_ext2[1] = 0;
    if (getenv("AQ_SHOW_SELFTEST")) printf("%s", so.c_str());
  }
}

// ---------------------------------------------------------------- MAVLink conversation (pymavlink)
static int mavlinkIo(const char* inPath, const char* outPath) {
  hal::host::reset();
  configDefaults(bus::config);
  bus::config.gcs_arm = 1;
  nav::clear();
  nav::rallySetCount(0);
  FILE* fi = fopen(inPath, "rb");
  if (!fi) return 2;
  comms::MavlinkLink link;
  link.begin(hal::Port::UDP);
  uint32_t t = 1000;
  int c;
  while ((c = fgetc(fi)) != EOF) {
    hal::host::in(hal::Port::UDP).push_back((uint8_t)c);
  }
  fclose(fi);
  for (int i = 0; i < 400; i++) { link.update(t); t += 10; }  // 4 s of link time
  bus::Request r;
  uint32_t arg;
  bool armReq = bus::takeRequest(r, arg) && r == bus::Request::ARM;
  FILE* fo = fopen(outPath, "wb");
  const std::string& o = hal::host::out(hal::Port::UDP);
  fwrite(o.data(), 1, o.size(), fo);
  fclose(fo);
  // report state for the python side
  printf("mission_count=%d\n", nav::count());
  for (int i = 0; i < nav::count(); i++) {
    nav::MissionItem it;
    nav::get(i, it);
    printf("item %d cmd=%u lat=%d lon=%d alt=%.2f p1=%.2f\n", i, it.cmd, it.lat, it.lon, it.alt, it.param1);
  }
  printf("rally_count=%d\n", nav::rallyCount());
  for (int i = 0; i < nav::rallyCount(); i++) {
    nav::MissionItem it;
    nav::rallyGet(i, it);
    printf("rally %d lat=%d lon=%d alt=%.2f\n", i, it.lat, it.lon, it.alt);
  }
  printf("roll_p=%.2f\n", bus::config.pid[0].p);
  printf("arm_request=%d\n", armReq ? 1 : 0);
  printf("param_count=%d\n", paramCount());
  return 0;
}

// ---------------------------------------------------------------- register-level driver tests
static void put16le(std::map<uint8_t, uint8_t>& r, uint8_t reg, int16_t v) {
  r[reg] = (uint8_t)v;
  r[(uint8_t)(reg + 1)] = (uint8_t)(v >> 8);
}

static void testIcm45686() {
  hal::host::reset();
  hal::host::fakeSpi = true;
  auto& r = hal::host::spiRegs;
  r[0x72] = 0xE9;  // WHO_AM_I
  // soft reset bit clears immediately; other writes are stored
  hal::host::spiOnWrite = [](uint8_t reg, uint8_t v) { hal::host::spiRegs[reg] = reg == 0x7F ? (uint8_t)(v & ~0x02) : v; };
  put16le(r, 0x00, 0);      // accel X
  put16le(r, 0x02, 0);      // accel Y
  put16le(r, 0x04, 2048);   // accel Z = +1 g (chip Z up)
  put16le(r, 0x06, 164);    // gyro X = +10 dps
  put16le(r, 0x08, -328);   // gyro Y = -20 dps
  put16le(r, 0x0A, 0);
  drv::ImuDriver* imu = drv::probeImu(1000);
  CHECK(imu && !strcmp(imu->name(), "ICM-45686"), "ICM-45686 detected (WHO_AM_I 0x72 = 0xE9)");
  CHECK(r[0x10] == 0x0F && r[0x1B] == 0x15 && r[0x1C] == 0x15, "ICM-45686 configured: LN mode, 16 g / 2000 dps @ 1.6 kHz");
  Vec3 g, a;
  bool ok = imu && imu->read(g, a);
  CHECK(ok && fabsf(a.z + GRAVITY) < 0.01f && fabsf(g.x * RAD2DEG - 10.0f) < 0.01f && fabsf(g.y * RAD2DEG - 20.0f) < 0.01f,
        "ICM-45686 data: little-endian, scaled, converted to FRD");
  // an ICM-42688 must still be detected as such (no false match)
  hal::host::reset();
  hal::host::fakeSpi = true;
  hal::host::spiRegs[0x75] = 0x47;
  imu = drv::probeImu(1000);
  CHECK(imu && !strcmp(imu->name(), "ICM-42688-P"), "ICM-42688-P still detected correctly");
}

static void testQmc5883p() {
  hal::host::reset();
  auto& r = hal::host::i2cRegs[0x2C];
  r[0x00] = 0x80;  // chip id
  put16le(r, 0x01, 1500);
  put16le(r, 0x03, -750);
  put16le(r, 0x05, 3000);
  drv::MagDriver* m = drv::probeMag();
  CHECK(m && !strcmp(m->name(), "QMC5883P"), "QMC5883P detected at 0x2C (id 0x80)");
  auto& w = hal::host::i2cWrites[0x2C];
  CHECK(w[0x06] == 0x29 && w[0x0A] == 0x0B && w[0x0B] == 0x08, "QMC5883P configured: axis sign, continuous 100 Hz, 8 G");
  Vec3 f;
  bool ok = m && m->read(f);
  CHECK(ok && fabsf(f.x - 0.4f) < 1e-4f && fabsf(f.y - 0.2f) < 1e-4f && fabsf(f.z + 0.8f) < 1e-4f,
        "QMC5883P data scaled to gauss, FRD");
}

// ---- web configurator API (JSON shape, parameter writes, armed lock)
struct JsonCheck {  // minimal recursive-descent JSON validator
  const char* p;
  void ws() { while (*p == ' ' || *p == '\n' || *p == '\t' || *p == '\r') p++; }
  bool str() {
    if (*p != '"') return false;
    for (p++; *p && *p != '"'; p++) { if ((unsigned char)*p < 0x20) return false; if (*p == '\\') p++; }
    return *p++ == '"';
  }
  bool num() {
    const char* s = p;
    if (*p == '-') p++;
    while ((*p >= '0' && *p <= '9') || *p == '.' || *p == 'e' || *p == 'E' || *p == '+' || *p == '-') p++;
    return p > s;
  }
  bool lit(const char* w) { size_t n = strlen(w); if (strncmp(p, w, n)) return false; p += n; return true; }
  bool value() {
    ws();
    if (*p == '{') {
      p++; ws();
      if (*p == '}') { p++; return true; }
      for (;;) { ws(); if (!str()) return false; ws(); if (*p++ != ':') return false; if (!value()) return false; ws();
        if (*p == ',') { p++; continue; } return *p++ == '}'; }
    }
    if (*p == '[') {
      p++; ws();
      if (*p == ']') { p++; return true; }
      for (;;) { if (!value()) return false; ws(); if (*p == ',') { p++; continue; } return *p++ == ']'; }
    }
    if (*p == '"') return str();
    if (lit("true") || lit("false") || lit("null")) return true;
    return num();
  }
  static bool valid(const std::string& s) { JsonCheck j{s.c_str()}; if (!j.value()) return false; j.ws(); return *j.p == 0; }
};
static void sinkString(const char* d, size_t n, void* ctx) { static_cast<std::string*>(ctx)->append(d, n); }

static void testWebApi() {
  configDefaults(bus::config);
  VehicleStatus st{};
  bus::status.publish(st);
  static comms::SystemInfo info{};
  info.version = "test";
  web::begin(&info);
  std::string pj, sj;
  web::paramsJson(sinkString, &pj);
  web::statusJson(sinkString, &sj);
  CHECK(JsonCheck::valid(pj) && pj.find("\"roll_p\"") != std::string::npos && pj.find("\"web_enable\"") != std::string::npos,
        "Web /api/params is valid JSON with all parameters");
  CHECK(JsonCheck::valid(sj) && sj.find("\"tasks\"") != std::string::npos && sj.find("\"wind\"") != std::string::npos,
        "Web /api/status is valid JSON");
  std::string lite;
  web::statusJson(sinkString, &lite, false);
  CHECK(JsonCheck::valid(lite) && lite.find("\"tasks\"") == std::string::npos && lite.find("\"fw\"") == std::string::npos &&
            lite.find("\"full\":false") != std::string::npos && lite.find("\"att\"") != std::string::npos &&
            lite.size() + 200 < sj.size(),
        "Web status lite frame (WebSocket) is valid and leaves out the slow fields");
  printf("  status json: full %u bytes, lite %u bytes\n", (unsigned)sj.size(), (unsigned)lite.size());
  nav::MissionItem it{};
  it.cmd = nav::CMD_LOITER_TURNS; it.lat = 210290000; it.lon = 1058550000; it.alt = 12; it.param3 = 15;
  nav::set(0, it);
  nav::setCount(1);
  std::string mj;
  web::missionJson(sinkString, &mj);
  CHECK(JsonCheck::valid(mj) && mj.find("\"cmd\":18") != std::string::npos, "Web /api/mission is valid JSON");
  nav::clear();
  int i = paramFind("roll_p");
  CHECK(web::setParam("roll_p", "52.5") == 200 && fabsf(bus::config.pid[0].p - 52.5f) < 1e-4f, "Web set parameter");
  CHECK(web::setParam("roll_p", "abc") == 400 && web::setParam("roll_p", "99999") == 400 && web::setParam("nope", "1") == 404,
        "Web rejects bad value / range / unknown name");
  st.armed = true;
  bus::status.publish(st);
  char msg[64];
  CHECK(web::setParam("roll_p", "40") == 423 && fabsf(paramGet(bus::config, i) - 52.5f) < 1e-4f && !web::save() &&
        web::command("defaults", msg, sizeof(msg)) == 423,
        "Web refuses parameter writes, save and commands while armed");
  st.armed = false;
  bus::status.publish(st);
  CHECK(web::save() && web::command("calib_acc", msg, sizeof(msg)) == 200 && web::command("xyz", msg, sizeof(msg)) == 404,
        "Web save and commands when disarmed");
  configDefaults(bus::config);
}

static void testWebAuth() {
  hal::host::reset();
  creds::load();
  VehicleStatus st{};
  bus::status.publish(st);
  char tok[33], msg[64];
  CHECK(creds::usingDefaultPassword() && !strncmp(creds::ssid(), "AllQuad-", 8), "Credentials: factory defaults");
  CHECK(web::login("wrong", 1000, tok) == 401 && !web::authorized("aq=0123", 1000), "Web login rejects a wrong password");
  CHECK(web::login("allquad123", 1000, tok) == 200 && strlen(tok) == 32, "Web login issues a 32-char session token");
  std::string cookie = std::string("theme=dark; aq=") + tok;
  CHECK(web::authorized(cookie.c_str(), 2000) && !web::authorized("aq=00000000000000000000000000000000", 2000),
        "Session cookie accepted, forged token refused");
  for (int i = 0; i < 5; i++) web::login("guess", 3000, tok);
  CHECK(web::login("allquad123", 4000, tok) == 429, "5 wrong passwords lock the login for 30 s");
  CHECK(web::login("allquad123", 40000, tok) == 200, "Login works again after the lockout");
  cookie = std::string("aq=") + tok;
  CHECK(web::changeWifi("MyQuad", "short", nullptr, "allquad123", msg, sizeof(msg)) == 400 &&
        !strncmp(creds::ssid(), "AllQuad-", 8), "WiFi change validates (password 8-63) and keeps old values");
  CHECK(web::changeWifi("MyQuad", "s3cret-pass", "", "bad", msg, sizeof(msg)) == 401, "WiFi change needs the current password");
  CHECK(web::changeWifi("MyQuad", "s3cret-pass", "", "allquad123", msg, sizeof(msg)) == 200 &&
        !web::authorized(cookie.c_str(), 41000), "WiFi change saved, old sessions logged out");
  creds::load();  // from flash
  CHECK(!strcmp(creds::ssid(), "MyQuad") && !strcmp(creds::webPass(), "s3cret-pass") && !creds::usingDefaultPassword(),
        "Credentials persist; web password follows the WiFi password by default");
  creds::resetDefaults();
}

static void strSink(const char* d, size_t n, void* ctx) { static_cast<std::string*>(ctx)->append(d, n); }

static void testBackupRestore() {
  hal::host::reset();
  VehicleStatus st{};
  bus::status.publish(st);
  configDefaults(bus::config);
  Config& c = bus::config;
  c.pid[0].p = 52.25f; c.pid[2].d = 3; c.rates[1].max = 812.5f; c.mag_declination_deg = -1.37f;
  c.acc_bias[2] = 0.123456f; c.acc_calibrated = 1; c.mode_range[9] = {(uint8_t)ModeBox::CRUISE, 10, 1700, 2100};
  nav::clear();
  nav::MissionItem it{};
  it.cmd = nav::CMD_TAKEOFF; it.frame = 3; it.alt = 12; nav::set(0, it);
  it = nav::MissionItem{}; it.cmd = nav::CMD_LOITER_TURNS; it.frame = 3; it.lat = 210290001; it.lon = 1058550002;
  it.alt = 15.5f; it.param1 = 2; it.param3 = -12.5f; nav::set(1, it);
  nav::setCount(2);
  it = nav::MissionItem{}; it.cmd = nav::CMD_RALLY_POINT; it.frame = 3; it.lat = 210300000; it.lon = 1058560000; it.alt = 8;
  nav::rallySet(0, it);
  nav::rallySetCount(1);
  Config before = c;

  comms::Cli cli;
  std::string text;
  cli.setOutput(strSink, &text);
  cli.diff(false);
  int setLines = 0;
  for (size_t p = text.find("\nset "); p != std::string::npos; p = text.find("\nset ", p + 1)) setLines++;
  // wipe, then replay the backup line by line
  configDefaults(c);
  nav::clear();
  nav::rallySetCount(0);
  std::string echo;
  cli.setOutput(strSink, &echo);
  size_t start = 0;
  while (start < text.size()) {
    size_t e = text.find('\n', start);
    if (e == std::string::npos) e = text.size();
    cli.run(text.substr(start, e - start).c_str());
    start = e + 1;
  }
  configSeal(before);
  configSeal(c);
  nav::MissionItem a, b;
  bool missionOk = nav::count() == 2 && nav::get(1, a) && a.lat == 210290001 && a.param3 == -12.5f && a.cmd == 18;
  bool rallyOk = nav::rallyCount() == 1 && nav::rallyGet(0, b) && b.lat == 210300000 && b.alt == 8;
  CHECK(setLines >= 8 && setLines < 20, "Backup (diff) lists only changed settings");
  CHECK(!memcmp(&before, &c, sizeof(Config)), "Restore reproduces the configuration bit for bit");
  CHECK(missionOk && rallyOk, "Restore reproduces mission and rally points");
  CHECK(echo.find("saved") != std::string::npos && hal::storageLoad(hal::Slot::MISSION, &a, sizeof(a)),
        "Restore ends with save (config, mission, rally)");
  // the web restore: a bad line saves nothing, a good file is applied and saved
  configDefaults(c);
  configSeal(c);
  hal::storageSave(hal::Slot::CONFIG, &c, sizeof(c));  // known stored state
  char msg[96];
  const char* bad = "set roll_p 61\nset no_such_param 1\nsave\n";
  int r1 = web::restoreText(bad, strlen(bad), msg, sizeof(msg));
  Config probe;
  bool savedBad = hal::storageLoad(hal::Slot::CONFIG, &probe, sizeof(probe)) && probe.pid[0].p == 61;
  const char* good = "# AllQuad backup\r\nset roll_p 61\r\nset pitch_p 62.5\r\nsave\r\n";
  int r2 = web::restoreText(good, strlen(good), msg, sizeof(msg));
  bool savedGood = hal::storageLoad(hal::Slot::CONFIG, &probe, sizeof(probe)) && probe.pid[1].p == 62.5f;
  CHECK(r1 == 400 && !savedBad && r2 == 200 && savedGood, "Web restore: all-or-nothing save");
  const char* bad2 = "set pitch_p 30\nbogus\n";
  CHECK(web::restoreText(bad2, strlen(bad2), msg, sizeof(msg)) == 400 && c.pid[1].p == 62.5f,
        "Web restore: a failed restore leaves the running settings unchanged");
  configDefaults(c);
  nav::clear();
  nav::rallySetCount(0);
}

// "web-dump params|status" prints the JSON (used by the UI tests' mock server)
static int webDump(const char* what) {
  configDefaults(bus::config);
  static comms::SystemInfo info{};
  info.version = "host";
  info.imu = "ICM-45686";
  info.baro = "SPL06";
  info.mag = "QMC5883P";
  web::begin(&info);
  std::string s;
  if (!strcmp(what, "params")) web::paramsJson(sinkString, &s);
  else if (!strcmp(what, "mission")) web::missionJson(sinkString, &s);
  else web::statusJson(sinkString, &s);
  fwrite(s.data(), 1, s.size(), stdout);
  return 0;
}


// ---------------------------------------------------------------- configurator over USB (AQ link)
namespace aqt {
struct Resp { int status = -1; uint32_t total = 0, off = 0; std::string data; uint8_t seq = 0; };
static uint8_t seqNo = 0;
static std::string frame(const std::string& path, const std::string& body = "") {
  std::string pl = path;
  pl.push_back('\0');
  pl += body;
  std::string f = "\xA7\x51";
  f.push_back((char)++seqNo);
  f.push_back((char)(pl.size() & 0xFF));
  f.push_back((char)(pl.size() >> 8));
  f += pl;
  uint16_t c = comms::AqLink::crc16((const uint8_t*)f.data() + 2, f.size() - 2);
  f.push_back((char)(c & 0xFF));
  f.push_back((char)(c >> 8));
  return f;
}
static uint16_t u16(const std::string& s, size_t i) { return (uint8_t)s[i] | (uint8_t)s[i + 1] << 8; }
static uint32_t u32(const std::string& s, size_t i) { return u16(s, i) | (uint32_t)u16(s, i + 2) << 16; }
// parse every response frame found in the console output (garbage in between is skipped)
static std::vector<Resp> parse(const std::string& out) {
  std::vector<Resp> v;
  for (size_t i = 0; i + 17 <= out.size(); i++) {
    if ((uint8_t)out[i] != 0xA7 || (uint8_t)out[i + 1] != 0x52) continue;
    size_t len = u16(out, i + 13);
    if (i + 17 + len > out.size()) break;
    uint16_t c = comms::AqLink::crc16((const uint8_t*)out.data() + i + 2, 13 + len);
    if (c != u16(out, i + 15 + len)) continue;
    Resp r;
    r.seq = (uint8_t)out[i + 2];
    r.status = u16(out, i + 3);
    r.total = u32(out, i + 5);
    r.off = u32(out, i + 9);
    r.data = out.substr(i + 15, len);
    v.push_back(r);
    i += 16 + len;
  }
  return v;
}
static comms::AqLink* L;
static uint32_t nowMs = 1000;
static Resp request(const std::string& path, const std::string& body = "", const std::string& noise = "") {
  std::string& out = hal::host::out(hal::Port::CONSOLE);
  out.clear();
  for (char ch : noise + frame(path, body)) L->feed((uint8_t)ch, nowMs);
  for (int k = 0; k < 200; k++) L->update(nowMs += 1);
  auto v = parse(out);
  return v.empty() ? Resp() : v.back();
}
// the client side of paging
static Resp fetchAll(const std::string& path) {
  Resp all;
  uint32_t off = 0;
  for (int k = 0; k < 400; k++) {
    Resp r = request(path + (path.find('?') == std::string::npos ? "?" : "&") + "o=" + std::to_string(off));
    if (r.status != 200 || r.off != off) return r;
    all.status = 200; all.total = r.total;
    all.data += r.data;
    off += (uint32_t)r.data.size();
    if (off >= r.total || r.data.empty()) break;
  }
  return all;
}
}  // namespace aqt

static void testAqLink() {
  using namespace aqt;
  static comms::AqLink link;
  link = comms::AqLink();
  link.begin(hal::Port::CONSOLE);
  L = &link;
  VehicleStatus st{};
  bus::status.publish(st);

  Resp h = request("/api/hello");
  CHECK(h.status == 200 && h.seq == seqNo && JsonCheck::valid(h.data) && h.data.find("\"usb\":false") != std::string::npos,
        "AQ link: hello answered, CRC and sequence correct");
  std::string full;
  web::paramsJson(sinkString, &full);
  Resp p = fetchAll("/api/params");
  CHECK(full.size() > comms::AqLink::DATA_MAX && p.status == 200 && p.data == full && p.total == full.size(),
        "AQ link: parameters paged in 2 kB frames, identical to the web API");
  std::string noise = "garbage\r\n";
  noise += std::string("\xA7\x51\x05\x03\x00" "abc" "\x00\x00", 10);  // bad CRC
  Resp n = request("/api/hello", "", noise);
  CHECK(n.status == 200 && link.crcErrors() == 1, "AQ link: noise and a corrupt frame are skipped, the next frame works");
  float before = bus::config.pid[0].p;
  Resp s = request("/api/set?roll_p=57.5&yaw_p=40");
  CHECK(s.status == 200 && fabsf(bus::config.pid[0].p - 57.5f) < 1e-3f, "AQ link: set parameters");
  Resp bad = request("/api/set?nope=1");
  CHECK(bad.status == 404 && bad.data.find("nope") != std::string::npos, "AQ link: unknown parameter reported");
  // restore a backup sent in small pieces
  std::string backup;
  web::backupText(sinkString, &backup, false);
  Resp b = fetchAll("/api/backup");
  CHECK(b.status == 200 && b.data == backup, "AQ link: backup download");
  size_t pos = backup.find("set roll_p");
  std::string edited = backup.substr(0, pos) + "set roll_p 61\n" + backup.substr(backup.find('\n', pos) + 1);
  Resp r;
  int pieces = 0;
  for (size_t o = 0; o < edited.size(); o += 40, pieces++)
    r = request("/api/restore?o=" + std::to_string(o) + "&n=" + std::to_string(edited.size()), edited.substr(o, 40));
  CHECK(pieces > 1 && r.status == 200 && fabsf(bus::config.pid[0].p - 61.0f) < 1e-3f,
        "AQ link: restore uploaded in pieces, applied and saved");
  Resp ooo = request("/api/restore?o=600&n=900", "x");
  CHECK(ooo.status == 400, "AQ link: restore piece out of order refused");
  // slow UART: the reply leaves in small pieces over several updates
  hal::host::consoleSpace = 64;
  Resp slow = request("/api/status");
  hal::host::consoleSpace = 4096;
  CHECK(slow.status == 200 && JsonCheck::valid(slow.data) && slow.total == slow.data.size(), "AQ link: status through a 64-byte TX buffer");
  Resp lite = request("/api/status?full=0");
  CHECK(lite.status == 200 && lite.data.find("\"full\":false") != std::string::npos && lite.data.size() < slow.data.size(),
        "AQ link: lite status");
  // armed: changes refused
  st.armed = true;
  bus::status.publish(st);
  Resp a = request("/api/set?roll_p=50");
  Resp bb = request("/api/blackbox");
  CHECK(a.status == 423 && bb.status == 423, "AQ link: changes and log download refused while armed");
  st.armed = false;
  bus::status.publish(st);
  hal::host::consoleBaudNow = 0;  // USB CDC board
  Resp usb = request("/api/hello");
  hal::host::consoleBaudNow = 921600;
  CHECK(usb.data.find("\"usb\":true") != std::string::npos && request("/api/hello").data.find("\"baud\":921600") != std::string::npos,
        "AQ link: hello tells native USB from a UART bridge");
  Resp w = request("/api/wifi", "ssid=BenchQuad&wifi=newpass99");
  CHECK(w.status == 200 && !strcmp(creds::ssid(), "BenchQuad"), "AQ link: WiFi name/password reset over USB without the old password");
  bus::config.pid[0].p = before;
  bus::configChanged();
}

// ---------------------------------------------------------------- configurator features (v16)
static void testConfigMigration() {
  // a config stored by the previous firmware (version 15): same layout, ends before the VTX fields
  Config old;
  configDefaults(old);
  old.pid[0].p = 77;
  old.mode_range[3].box = 6;
  size_t oldSize = offsetof(Config, vtx_band) + 4;
  old.version = 15;
  old.size = (uint16_t)oldSize;
  uint32_t crc = crc32(&old, oldSize - 4);
  memcpy((uint8_t*)&old + oldSize - 4, &crc, 4);
  memset((uint8_t*)&old + oldSize, 0xEE, sizeof(Config) - oldSize);  // whatever was behind it in flash
  Config c = old;
  bool ok = configMigrate(c);
  CHECK(ok && configValid(c) && c.pid[0].p == 77 && c.mode_range[3].box == 6 && c.vtx_band == 5 && c.led_brightness == 40,
        "Config v15 -> v16: every setting kept, new VTX/LED fields at their defaults");
  Config bad = old;
  ((uint8_t*)&bad)[20] ^= 1;
  CHECK(!configMigrate(bad), "Config migration refuses a corrupt old config");
  // every setting back inside its limits on load
  Config d;
  configDefaults(d);
  CHECK(configClamp(d) == 0 && configValid(d), "Config clamp: the defaults are all within their limits");
  d.pid[0].p = 1e6f;
  d.hover_throttle = NAN;
  d.loop_hz = 60000;
  int fixed = configClamp(d);
  CHECK(fixed == 3 && configValid(d) && d.pid[0].p < 1e6f && d.hover_throttle == d.hover_throttle && d.loop_hz < 60000,
        "Config clamp: out-of-range / NaN settings fixed and the config resealed");
}

static void testCliMotorSensors() {
  using namespace aqt;
  static comms::AqLink link;
  link = comms::AqLink();
  link.begin(hal::Port::CONSOLE);
  L = &link;
  VehicleStatus st{};
  bus::status.publish(st);
  Resp g = request("/api/cli", "get roll_p");
  CHECK(g.status == 200 && g.data.find("roll_p") != std::string::npos, "CLI over the link: one command, its output back");
  // a long output: the command runs once, the pages come from the buffer
  Resp d = request("/api/cli", "dump");
  std::string all = d.data;
  uint32_t off = (uint32_t)d.data.size();
  while (off < d.total) { Resp p = request("/api/cli?o=" + std::to_string(off)); all += p.data; off += (uint32_t)p.data.size(); if (p.data.empty()) break; }
  CHECK(d.total > comms::AqLink::DATA_MAX && all.size() == d.total && all.find("set led_count") != std::string::npos,
        "CLI dump paged from the buffer (runs once)");
  std::string w;
  web::cliExec("wifi reset", sinkString, &w, false);
  CHECK(w.find("USB only") != std::string::npos, "CLI over WiFi: the wifi command stays USB only");
  // motor test
  Resp m = request("/api/motor?m1=0.2&m3=0.05");
  CHECK(m.status == 200 && fabsf(bus::motorTest[0] - 0.2f) < 1e-4f && bus::motorTest[1] == 0 &&
            bus::motorTestActive(os::timeMs()) && (int32_t)(bus::cliActiveUntilMs - os::timeMs()) > 0,
        "Motor test: values set, arming blocked while it runs");
  CHECK(!bus::motorTestActive(os::timeMs() + bus::MOTOR_TEST_TIMEOUT_MS + 1), "Motor test stops by itself after 1 s");
  st.armed = true;
  bus::status.publish(st);
  CHECK(request("/api/motor?m1=0.3").status == 423, "Motor test refused while armed");
  st.armed = false;
  bus::status.publish(st);
  float zero[4] = {0, 0, 0, 0};
  bus::setMotorTest(zero, os::timeMs());
  Resp sj = request("/api/sensors");
  CHECK(sj.status == 200 && JsonCheck::valid(sj.data) && sj.data.find("\"gyro\"") != std::string::npos &&
            sj.data.find("\"q\"") != std::string::npos,
        "Sensors JSON (gyro, acc, mag, baro, range, flow, attitude, ESC)");
  Resp b = request("/api/cmd?c=bootloader");
  for (int k = 0; k < 50; k++) link.update(nowMs += 10);
  CHECK(b.status == 200 && hal::host::bootloaderRequests == 1, "Reboot into the USB bootloader on request");
}

static void testVtx() {
  uint8_t f[16];
  size_t n = vtx::saSetFrequency(f, 5800);
  CHECK(n == 9 && f[1] == 0xAA && f[2] == 0x55 && f[3] == 0x09 && f[4] == 2 && f[5] == 0x16 && f[6] == 0xA8 &&
            f[7] == vtx::saCrc8(f + 1, 6),
        "SmartAudio: set frequency 5800 MHz frame");
  n = vtx::saSetPowerDbm(f, 23);
  CHECK(f[3] == 0x05 && f[5] == (23 | 0x80), "SmartAudio 2.1: power in dBm");
  vtx::trampFrame(f, 'F', 5740);
  uint8_t sum = 0;
  for (int i = 1; i < 14; i++) sum = (uint8_t)(sum + f[i]);
  CHECK(f[0] == 0x0F && f[1] == 'F' && (f[2] | f[3] << 8) == 5740 && f[14] == sum && f[15] == 0, "Tramp: frequency frame");
  Config c;
  configDefaults(c);
  CHECK(vtx::frequency(c) == 5658, "VTX: Raceband 1 = 5658 MHz by default");
  c.vtx_freq = 5917;
  CHECK(vtx::frequency(c) == 5917, "VTX: a set frequency overrides band/channel");
  c.vtx_freq = 0;
  c.vtx_power = 4;
  vtx::Driver d;
  hal::host::out(hal::Port::SERIAL2).clear();
  d.begin(vtx::Type::SMARTAUDIO, hal::Port::SERIAL2);
  for (uint32_t t = 2000; t < 2400; t += 10) d.update(c, false, t);
  CHECK(d.frames() == 3 && d.sentPowerMw() == 25 && d.sentFreq() == 5658, "VTX: disarmed -> lowest power, frames spaced");
  for (uint32_t t = 2400; t < 4000; t += 10) d.update(c, false, t);
  CHECK(d.frames() == 6, "VTX: settings sent once more a second later");
  for (uint32_t t = 4000; t < 4600; t += 10) d.update(c, true, t);
  CHECK(d.sentPowerMw() == 400 && d.frames() == 9, "VTX: armed -> configured power (400 mW)");
  // MSP (DJI goggles): read and set
  comms::MspLink msp;
  msp.begin(hal::Port::SERIAL3);
  comms::MspFrame q{};
  q.dir = '<';
  q.cmd = 89;
  q.len = 4;
  uint16_t bc = 3 * 8 + 4;  // band F (4th), channel 5
  q.payload[0] = (uint8_t)bc; q.payload[1] = 0; q.payload[2] = 3; q.payload[3] = 0;
  msp.handle(q, 100);
  CHECK(bus::config.vtx_band == 4 && bus::config.vtx_channel == 5 && bus::config.vtx_power == 3 &&
            vtx::frequency(bus::config) == 5820,
        "MSP_SET_VTX_CONFIG (digital goggles) -> F5 5820 MHz");
  configDefaults(bus::config);
}

static void testLeds() {
  Config c;
  configDefaults(c);
  c.led_brightness = 100;
  VehicleStatus st{};
  BatteryState b{};
  uint8_t g[8 * 3];
  st.armed = true;
  st.mode = FlightMode::ANGLE;
  app::ledColors(c, st, b, 1000, g, 8);
  CHECK(g[0] == 255 && g[1] == 0 && g[2] == 0, "LED: armed ANGLE = green (GRB order)");
  st.failsafe = true;
  app::ledColors(c, st, b, 1000, g, 8);
  bool redOrOff = g[1] == 255 || g[1] == 0;
  app::ledColors(c, st, b, 1100, g + 0, 8);
  CHECK(redOrOff && g[0] == 0, "LED: failsafe blinks red");
  st.failsafe = false;
  c.led_mode = 1;
  c.led_brightness = 50;
  app::ledColors(c, st, b, 1000, g, 8);
  CHECK(g[0] == 127 && g[1] == 127 && g[21] == 0 && g[22] == 127, "LED: orientation (front white, rear red) at 50 %");
}

// byte-level fake of a BLHeli_S ESC bootloader on motor 2
namespace fakeesc {
std::vector<uint8_t> flash(16384, 0xFF);
std::deque<int> rx;
uint16_t addr = 0;
std::vector<uint8_t> buffer;
bool connected = false, expectBuffer = false;
size_t bufLen = 0;
int selected = -1, runs = 0;
uint16_t crc(const uint8_t* d, size_t n) { uint16_t c = 0; for (size_t i = 0; i < n; i++) c = comms::FourWay::crcBlb(c, d[i]); return c; }
void ack(uint8_t a) { rx.push_back(a); }
void tx(int esc, const uint8_t* d, size_t n) {
  if (esc != 2) return;  // only ESC 2 is connected
  if (!connected) {
    static const uint8_t init[] = {0x0D, 'B', 'L', 'H', 'e', 'l', 'i', 0xF4, 0x7D};
    if (n >= 9 && !memcmp(d + n - 9, init, 9)) {
      for (int b : {0x34, 0x37, 0x31, 0x64, 0xE8, 0xB2, 0x06, 0x04, 0x30}) rx.push_back(b);
      connected = true;
    }
    return;
  }
  if (n < 2 || crc(d, n - 2) != (uint16_t)(d[n - 2] | d[n - 1] << 8)) { ack(0xC2); return; }
  n -= 2;
  if (expectBuffer) { buffer.assign(d, d + n); expectBuffer = false; ack(n == bufLen ? 0x30 : 0xC2); return; }
  switch (d[0]) {
    case 0xFF: addr = (uint16_t)(d[2] << 8 | d[3]); ack(0x30); break;
    case 0xFE: bufLen = d[3] ? d[3] : (d[2] ? 256 : 0); expectBuffer = true; break;  // silent
    case 0x03: {
      size_t k = d[1] ? d[1] : 256;
      for (size_t i = 0; i < k; i++) rx.push_back(flash[addr + i]);
      uint16_t c = crc(flash.data() + addr, k);
      rx.push_back(c & 0xFF); rx.push_back(c >> 8); ack(0x30);
      break;
    }
    case 0x01: for (size_t i = 0; i < buffer.size(); i++) flash[addr + i] = buffer[i]; ack(0x30); break;
    case 0x02: for (int i = 0; i < 512; i++) flash[addr + i] = 0xFF; ack(0x30); break;
    case 0xFD: ack(0xC1); break;
    case 0x00: runs++; connected = false; break;
    default: ack(0xC1);
  }
}
int rxByte(int esc) { if (esc != 2 || rx.empty()) return -1; int b = rx.front(); rx.pop_front(); return b; }
}  // namespace fakeesc

static std::string fwFrame(uint8_t cmd, uint16_t addr, const std::vector<uint8_t>& p) {
  std::string f;
  f.push_back((char)0x2F); f.push_back((char)cmd); f.push_back((char)(addr >> 8)); f.push_back((char)addr);
  f.push_back((char)(p.size() == 256 ? 0 : p.size()));
  for (uint8_t b : p) f.push_back((char)b);
  uint16_t c = 0;
  for (char ch : f) c = comms::FourWay::crcXmodem(c, (uint8_t)ch);
  f.push_back((char)(c >> 8)); f.push_back((char)c);
  return f;
}
struct FwReply { uint8_t cmd = 0, ack = 0xFF; std::vector<uint8_t> p; bool crcOk = false; };
static FwReply fwParse(const std::string& o) {
  FwReply r;
  size_t i = o.find((char)0x2E);
  if (i == std::string::npos || o.size() < i + 8) return r;
  size_t len = (uint8_t)o[i + 4] ? (uint8_t)o[i + 4] : 256;
  if (o.size() < i + 5 + len + 3) return r;
  r.cmd = (uint8_t)o[i + 1];
  r.p.assign(o.begin() + i + 5, o.begin() + i + 5 + len);
  r.ack = (uint8_t)o[i + 5 + len];
  uint16_t c = 0;
  for (size_t k = i; k < i + 6 + len; k++) c = comms::FourWay::crcXmodem(c, (uint8_t)o[k]);
  r.crcOk = c == (uint16_t)((uint8_t)o[i + 6 + len] << 8 | (uint8_t)o[i + 7 + len]);
  return r;
}

static void testFourWay() {
  hal::host::escTx = fakeesc::tx;
  hal::host::escRx = fakeesc::rxByte;
  for (int i = 0; i < 16; i++) fakeesc::flash[0x1A00 + i] = (uint8_t)(0x40 + i);  // "settings"
  comms::FourWay fw;
  fw.begin(hal::Port::CONSOLE);
  comms::MspLink msp;
  msp.begin(hal::Port::CONSOLE);
  msp.setFourWay(&fw);
  VehicleStatus st{};
  bus::status.publish(st);
  std::string& out = hal::host::out(hal::Port::CONSOLE);
  out.clear();
  comms::MspFrame q{};
  q.dir = '<';
  q.cmd = 245;
  q.len = 0;
  msp.handle(q, 100);
  CHECK(fw.active() && hal::host::escActive && bus::escPassthrough && out.size() == 7 && (uint8_t)out[5] == 4,
        "MSP_SET_PASSTHROUGH: 4 ESCs, console switched to the 4-way interface, motor output off");
  auto run = [&](uint8_t cmd, uint16_t addr, std::vector<uint8_t> p) {
    out.clear();
    for (char ch : fwFrame(cmd, addr, p)) fw.feed((uint8_t)ch, 200);
    return fwParse(out);
  };
  FwReply v = run(0x31, 0, {0});
  FwReply nm = run(0x32, 0, {0});
  CHECK(v.crcOk && v.ack == 0 && v.p[0] == 108 && std::string(nm.p.begin(), nm.p.end()) == "m4wFCIntf",
        "4-way: protocol version 108, interface name");
  FwReply none = run(0x37, 0, {0});
  CHECK(none.ack == 0x0F, "4-way: no ESC answering on motor 1 -> general error");
  FwReply init = run(0x37, 0, {2});
  CHECK(init.ack == 0 && init.p.size() == 4 && init.p[0] == 0xB2 && init.p[1] == 0xE8 && init.p[3] == 1,
        "4-way: ESC 3 bootloader found, BLHeli_S signature E8B2, SiLabs mode");
  FwReply rd = run(0x3A, 0x1A00, {16});
  bool same = rd.p.size() == 16;
  for (int i = 0; i < 16 && same; i++) same = rd.p[i] == 0x40 + i;
  CHECK(rd.ack == 0 && same, "4-way: read 16 bytes of ESC flash (CRC checked)");
  FwReply er = run(0x39, 0, {0x0D});  // page 13 = 0x1A00
  FwReply wr = run(0x3B, 0x1A00, {1, 2, 3, 4});
  FwReply rd2 = run(0x3A, 0x1A00, {6});
  CHECK(er.ack == 0 && wr.ack == 0 && rd2.p == std::vector<uint8_t>({1, 2, 3, 4, 0xFF, 0xFF}),
        "4-way: erase page, write, read back");
  std::vector<uint8_t> big(256);
  for (int i = 0; i < 256; i++) big[i] = (uint8_t)i;
  FwReply wb = run(0x3B, 0x0200, big);
  FwReply rb = run(0x3A, 0x0200, {0});
  CHECK(wb.ack == 0 && rb.p == big, "4-way: 256-byte write and read (length 0 = 256)");
  FwReply alive = run(0x30, 0, {0});
  std::string bad = fwFrame(0x30, 0, {0});
  bad[bad.size() - 1] ^= 0x55;
  out.clear();
  for (char ch : bad) fw.feed((uint8_t)ch, 300);
  CHECK(alive.ack == 0 && fwParse(out).ack == 0x03, "4-way: keep-alive, and a frame with a bad CRC is refused");
  FwReply reset = run(0x35, 0, {2});
  FwReply ex = run(0x34, 0, {0});
  CHECK(reset.ack == 0 && fakeesc::runs == 1 && ex.ack == 0 && !fw.active() && !hal::host::escActive && !bus::escPassthrough,
        "4-way: ESC restarted, exit gives the motors back");
  fw.start(1000);
  fw.update(62000);
  CHECK(!fw.active(), "4-way: left by itself after 60 s without a frame");
  hal::host::escTx = nullptr;
  hal::host::escRx = nullptr;
}

// ---------------------------------------------------------------- version 17 features
static void testConfigMigrationV16() {
  Config old;
  configDefaults(old);
  old.pid[1].d = 41;
  old.vtx_channel = 6;
  old.led_count = 12;
  size_t oldSize = offsetof(Config, dshot_bidir) + 4;  // v16 ended with led_color, then crc
  // (padding: the v16 struct ended at led_color + crc; the crc sat right after led_color)
  oldSize = offsetof(Config, led_color) + sizeof(uint32_t) + 4;
  old.version = 16;
  old.size = (uint16_t)oldSize;
  uint32_t crc = crc32(&old, oldSize - 4);
  memcpy((uint8_t*)&old + oldSize - 4, &crc, 4);
  memset((uint8_t*)&old + oldSize, 0xEE, sizeof(Config) - oldSize);
  Config c = old;
  bool ok = configMigrate(c);
  CHECK(ok && configValid(c) && c.pid[1].d == 41 && c.vtx_channel == 6 && c.led_count == 12 && c.est_type == 1 &&
            c.dshot_bidir == 0 && c.at_steps == 15 && c.rth_sanity_s == 20.0f && c.vtxt_powers == 0,
        "Config v16 -> v17: settings kept, EKF / bidir / autotune / VTX table at their defaults");
  CHECK(paramFind("dshot_bidir") >= 0 && paramFind("vtxt_band8") >= 0 && paramFind("launch_angle") >= 0 &&
            paramInfo(paramFind("mr1_mode")).max >= 14,
        "Parameters: new v17 entries, mode ranges accept GPS RESCUE / LAUNCH");
}

static void testConfigMigrationV17() {
  Config old;
  configDefaults(old);
  old.pid[0].p = 52;
  old.yaw_reversed = 1;
  old.vtxt_powers = 3;
  size_t oldSize = offsetof(Config, board_roll_deg) + 4;  // v17: ... vtxt_band[8], (pad), crc
  old.version = 17;
  old.size = (uint16_t)oldSize;
  uint32_t crc = crc32(&old, oldSize - 4);
  memcpy((uint8_t*)&old + oldSize - 4, &crc, 4);
  memset((uint8_t*)&old + oldSize, 0xEE, sizeof(Config) - oldSize);
  Config c = old;
  bool ok = configMigrate(c);
  CHECK(ok && configValid(c) && c.pid[0].p == 52 && c.yaw_reversed == 1 && c.vtxt_powers == 3 &&
            c.board_roll_deg == 0 && c.board_pitch_deg == 0 && c.board_yaw_deg == 0,
        "Config v17 -> v18: settings kept, board alignment 0");
  CHECK(paramFind("board_roll") >= 0 && paramFind("board_yaw") >= 0, "Parameters: board_roll / board_pitch / board_yaw");
  // FC rolled 10 degrees right on a level frame: gravity seen by the FC, corrected back to level
  Quat q = Quat::fromEuler(10 * DEG2RAD, 0, 0);
  Vec3 fc = q.rotateInv(Vec3(0, 0, -9.81f));
  Vec3 back = q.rotate(fc);
  CHECK(fabsf(back.x) < 1e-4f && fabsf(back.y) < 1e-4f && fabsf(back.z + 9.81f) < 1e-4f && fabsf(fc.y) > 1.0f,
        "Board alignment: a 10 degree roll is corrected back to level");
}

static void testConfigMigrationV18() {
  Config old;
  configDefaults(old);
  old.pid[2].p = 61;
  old.motor_out[0] = 3;
  old.board_yaw_deg = 90;
  size_t oldSize = offsetof(Config, imu_sync) + 4;  // v18: ... motor_out[4], crc
  old.version = 18;
  old.size = (uint16_t)oldSize;
  uint32_t crc = crc32(&old, oldSize - 4);
  memcpy((uint8_t*)&old + oldSize - 4, &crc, 4);
  memset((uint8_t*)&old + oldSize, 0xEE, sizeof(Config) - oldSize);
  Config c = old;
  bool ok = configMigrate(c);
  CHECK(ok && configValid(c) && c.pid[2].p == 61 && c.motor_out[0] == 3 && c.board_yaw_deg == 90 && c.imu_sync == 1,
        "Config v18 -> v19: settings kept, IMU data-ready loop sync on");
  int i = paramFind("imu_sync");
  CHECK(i >= 0 && paramInfo(i).reboot, "Parameters: imu_sync (reboot to apply)");
}

static void testConfigMigrationV19() {
  Config old;
  configDefaults(old);
  old.pid[1].p = 57;
  old.imu_sync = 0;
  size_t oldSize = ((offsetof(Config, imu_sync) + 1 + 3) & ~(size_t)3) + 4;  // v19: ... imu_sync, pad, crc
  ((uint8_t*)&old)[offsetof(Config, dshot_edt)] = 0;  // old padding byte
  old.version = 19;
  old.size = (uint16_t)oldSize;
  uint32_t crc = crc32(&old, oldSize - 4);
  memcpy((uint8_t*)&old + oldSize - 4, &crc, 4);
  memset((uint8_t*)&old + oldSize, 0xEE, sizeof(Config) - oldSize);
  Config c = old;
  bool ok = configMigrate(c);
  CHECK(ok && configValid(c) && c.pid[1].p == 57 && c.imu_sync == 0 && c.dshot_edt == 1,
        "Config v19 -> v20: settings kept, ESC extended telemetry on");
}

static void testDshotEdt() {
  // AM32 extended telemetry frames are told apart from eRPM frames and decoded
  auto line = [](uint16_t v12, uint8_t* lv, float* du) {
    uint8_t r[24];
    int n = dshot::runsFromGcr(dshot::gcrEncode(dshot::withChecksum(v12)), r);
    int k = 0;
    lv[k] = 1; du[k++] = 10;  // gap before the answer
    for (int i = 0; i < n; i++) { lv[k] = (uint8_t)(i % 2 ? 1 : 0); du[k++] = r[i]; }
    return k;
  };
  uint8_t lv[32]; float du[32];
  dshot::Edt e;
  float erpm = 1234;
  int k = line(dshot::edtValue(0x2, 47), lv, du);
  uint8_t k1 = dshot::decodeRunsEx(lv, du, k, 1.0f, erpm, &e);
  k = line(dshot::edtValue(0x4, 64), lv, du);
  uint8_t k2 = dshot::decodeRunsEx(lv, du, k, 1.0f, erpm, &e);
  k = line(dshot::edtValue(0x6, 23), lv, du);
  uint8_t k3 = dshot::decodeRunsEx(lv, du, k, 1.0f, erpm, &e);
  k = line(dshot::edtValue(0xE, 0x83), lv, du);
  uint8_t k4 = dshot::decodeRunsEx(lv, du, k, 1.0f, erpm, &e);
  CHECK(k1 == dshot::EDT && k2 == dshot::EDT && k3 == dshot::EDT && k4 == dshot::EDT && e.tempC == 47 &&
        fabsf(e.volts - 16.0f) < 1e-3f && e.amps == 23 && e.status == 0x83 && e.frames == 4 && erpm == 1234,
        "DShot EDT: temperature / voltage / current / status frames decoded, eRPM untouched");
  // every eRPM value the ESC can send still decodes as eRPM (normalised mantissa)
  bool allErpm = true;
  for (float r = 1000; r < 300000; r *= 1.07f) {
    float out = -1;
    k = line(dshot::valueFromErpm(r), lv, du);
    allErpm &= dshot::decodeRunsEx(lv, du, k, 1.0f, out, &e) == dshot::ERPM && fabsf(out - r) / r < 0.01f;
  }
  CHECK(allErpm, "DShot EDT: no eRPM frame is mistaken for telemetry (1k..300k eRPM)");
  // the bool API keeps rejecting EDT frames (callers that only want eRPM)
  k = line(dshot::edtValue(0x2, 40), lv, du);
  CHECK(!dshot::decodeRuns(lv, du, k, 1.0f, erpm), "DShot EDT: eRPM-only decoder ignores telemetry frames");
  int i = paramFind("dshot_edt");
  CHECK(i >= 0, "Parameters: dshot_edt");
}


static void testConfigMigrationV20() {
  Config old;
  configDefaults(old);
  old.pid[0].p = 63;
  old.rates[1].max = 800;
  old.dshot_edt = 0;
  size_t oldSize = ((offsetof(Config, dshot_edt) + 1 + 3) & ~(size_t)3) + 4;  // v20: ... dshot_edt, pad, crc
  memset((uint8_t*)&old + offsetof(Config, rc_smooth_hz), 0, oldSize - 4 - offsetof(Config, rc_smooth_hz) > 0 &&
         oldSize - 4 > offsetof(Config, rc_smooth_hz) ? oldSize - 4 - offsetof(Config, rc_smooth_hz) : 0);
  old.version = 20;
  old.size = (uint16_t)oldSize;
  uint32_t crc = crc32(&old, oldSize - 4);
  memcpy((uint8_t*)&old + oldSize - 4, &crc, 4);
  memset((uint8_t*)&old + oldSize, 0xEE, sizeof(Config) - oldSize);
  Config c = old;
  bool ok = configMigrate(c);
  CHECK(ok && configValid(c) && c.pid[0].p == 63 && c.dshot_edt == 0 && c.prof_pid[0][0].p == 63 &&
            c.prof_rates[1][1].max == 800 && c.acc_scale[2] == 1.0f && c.yaw_spin_thr == 1950 && c.crash_detect == 1 &&
            c.logic[3].enabled == 0,
        "Config v20 -> v21: settings kept, profiles 2/3 copy profile 1, new features at their defaults");
  CHECK(paramFind("p3_yaw_expo") >= 0 && paramFind("thrust_expo") >= 0 && paramFind("gcs_fs") >= 0 &&
            paramInfo(paramFind("fs_action")).max == 3,
        "Parameters: v21 entries (profiles, thrust_expo, gcs_fs, SmartRTL failsafe action)");
}

static void testConfigMigrationV21() {
  Config old;
  configDefaults(old);
  old.pid[2].p = 41;
  old.port_fn[1] = (uint8_t)PortFn::GPS;
  size_t oldSize = ((offsetof(Config, logic) + sizeof(old.logic) + 3) & ~(size_t)3) + 4;  // v21: ... logic[], crc
  memset((uint8_t*)&old + offsetof(Config, port_fn_ext), 0x5A, oldSize - 4 > offsetof(Config, port_fn_ext)
         ? oldSize - 4 - offsetof(Config, port_fn_ext) : 0);  // garbage in old padding
  old.version = 21;
  old.size = (uint16_t)oldSize;
  uint32_t crc = crc32(&old, oldSize - 4);
  memcpy((uint8_t*)&old + oldSize - 4, &crc, 4);
  memset((uint8_t*)&old + oldSize, 0xEE, sizeof(Config) - oldSize);
  Config c = old;
  bool ok = configMigrate(c);
  CHECK(ok && configValid(c) && c.pid[2].p == 41 && portFnOf(c, 1) == (uint8_t)PortFn::GPS &&
            portFnOf(c, 5) == 0 && portFnOf(c, 6) == 0 && portBaudOf(c, 5) != 0x5A5A5A5Au,
        "Config v21 -> v22: settings kept, UART4/UART5 off");
  CHECK(c.osd_canvas == 0 && c.osd_font == 0 && c.rpm_weight[0] == 100 && c.rpm_weight[2] == 100 && c.osd_stats == 15 &&
            !(c.osd_pos2[OSD2_TIME_LEFT] & 0x8000),
        "Config v21 -> v24: OSD canvas auto, RPM weights 100 %, stats screen 15 s, new OSD elements hidden");
  CHECK(paramFind("serial4_fn") >= 0 && paramFind("serial5_baud") >= 0 && portFnOf(c, 4) == 0,
        "Parameters: serial4/serial5 entries; console index has no function");
}

static void testConfigMigrationV25() {
  Config old;
  configDefaults(old);
  old.pid[0].p = 47;
  old.sd_keep = 0;
  old.osd_stats = 9;
  size_t oldSize = ((offsetof(Config, sd_keep) + 1 + 3) & ~(size_t)3) + 4;  // v25: ... sd_keep, crc
  memset((uint8_t*)&old + offsetof(Config, sd_keep) + 1, 0x5A, oldSize - 4 - offsetof(Config, sd_keep) - 1);
  old.version = 25;
  old.size = (uint16_t)oldSize;
  uint32_t crc = crc32(&old, oldSize - 4);
  memcpy((uint8_t*)&old + oldSize - 4, &crc, 4);
  memset((uint8_t*)&old + oldSize, 0xEE, sizeof(Config) - oldSize);
  Config c = old;
  bool ok = configMigrate(c);
  CHECK(ok && configValid(c) && c.pid[0].p == 47 && c.sd_keep == 0 && c.osd_stats == 9 && c.osd_blink == 1 && c.sd_dma == 0 &&
            !(c.osd_pos3[OSD3_BAT_BAR] & 0x8000) && !(c.osd_pos3[OSD3_RSSI_DBM] & 0x8000),
        "Config v25 -> v26: settings kept, OSD extras hidden, blinking on, SD DMA off");
  CHECK(paramFind("osd_batbar_pos") >= 0 && paramFind("sd_dma") >= 0 && paramFind("osd_blink") >= 0,
        "Parameters: v26 entries");
}

static void testConfigMigrationV26() {
  Config old;
  configDefaults(old);
  old.pid[1].d = 33;
  old.sd_dma = 1;
  size_t oldSize = ((offsetof(Config, sd_dma) + 1 + 3) & ~(size_t)3) + 4;  // v26: ... sd_dma, crc
  memset((uint8_t*)&old + offsetof(Config, sd_dma) + 1, 0x5A, oldSize - 4 - offsetof(Config, sd_dma) - 1);
  old.version = 26;
  old.size = (uint16_t)oldSize;
  uint32_t crc = crc32(&old, oldSize - 4);
  memcpy((uint8_t*)&old + oldSize - 4, &crc, 4);
  memset((uint8_t*)&old + oldSize, 0xEE, sizeof(Config) - oldSize);
  Config c = old;
  bool ok = configMigrate(c);
  CHECK(ok && configValid(c) && c.pid[1].d == 33 && c.sd_dma == 1 && c.ff_jitter == 7 && c.ff_boost == 15 && c.ff_max_pct == 90,
        "Config v26 -> v27: settings kept, feed-forward shaping defaults");
}
static void testConfigMigrationV27() {
  Config old;
  configDefaults(old);
  old.pid[0].i = 91;
  old.port_fn_ext[1] = (uint8_t)PortFn::GPS;
  old.ff_boost = 22;
  size_t oldSize = ((offsetof(Config, ff_max_pct) + 1 + 3) & ~(size_t)3) + 4;  // v27: ... ff_max_pct, crc
  memset((uint8_t*)&old + offsetof(Config, ff_max_pct) + 1, 0x5A, oldSize - 4 - offsetof(Config, ff_max_pct) - 1);
  old.version = 27;
  old.size = (uint16_t)oldSize;
  uint32_t crc = crc32(&old, oldSize - 4);
  memcpy((uint8_t*)&old + oldSize - 4, &crc, 4);
  memset((uint8_t*)&old + oldSize, 0xEE, sizeof(Config) - oldSize);
  Config c = old;
  bool ok = configMigrate(c);
  CHECK(ok && configValid(c) && c.pid[0].i == 91 && portFnOf(c, 6) == (uint8_t)PortFn::GPS && c.ff_boost == 22 &&
            portFnOf(c, 7) == 0 && portFnOf(c, 8) == 0 && portBaudOf(c, 7) == 0 && portBaudOf(c, 8) == 0,
        "Config v27 -> v28: settings kept, SERIAL6 / SERIAL7 off");
  int f6 = paramFind("serial6_fn"), b7 = paramFind("serial7_baud");
  CHECK(f6 >= 0 && b7 >= 0 && paramInfo(f6).reboot && paramSet(c, f6, (float)PortFn::MAVLINK) && portFnOf(c, 7) == (uint8_t)PortFn::MAVLINK &&
            paramSet(c, b7, 57600) && portBaudOf(c, 8) == 57600,
        "Parameters: serial6 / serial7 function and baud");
}
static void testConfigMigrationV28() {
  Config old;
  configDefaults(old);
  old.pid[2].p = 77;
  old.port_baud_ext2[0] = 2;
  size_t oldSize = ((offsetof(Config, port_baud_ext2) + sizeof(old.port_baud_ext2) + 3) & ~(size_t)3) + 4;  // v28: ... port_baud_ext2, crc
  size_t end = offsetof(Config, port_baud_ext2) + sizeof(old.port_baud_ext2);
  memset((uint8_t*)&old + end, 0x5A, oldSize - 4 - end);
  old.version = 28;
  old.size = (uint16_t)oldSize;
  uint32_t crc = crc32(&old, oldSize - 4);
  memcpy((uint8_t*)&old + oldSize - 4, &crc, 4);
  memset((uint8_t*)&old + oldSize, 0xEE, sizeof(Config) - oldSize);
  Config c = old;
  bool ok = configMigrate(c);
  CHECK(ok && configValid(c) && c.pid[2].p == 77 && c.port_baud_ext2[0] == 2 && c.thr_limit_pct == 100 && c.motor_limit_pct == 100 &&
            c.beginner == 0 && c.beg_radius == 60,
        "Config v28 -> v29: settings kept, no limits, first-flight mode off");
  int t = paramFind("thr_limit_pct"), m = paramFind("motor_limit_pct"), b = paramFind("beginner"), r = paramFind("beg_radius");
  CHECK(t >= 0 && m >= 0 && b >= 0 && r >= 0 && paramSet(c, t, 80) && c.thr_limit_pct == 80 && !paramSet(c, m, 30) &&
            paramSet(c, b, 1) && c.beginner == 1 && paramSet(c, r, 100) && c.beg_radius == 100,
        "Parameters: throttle / motor limit, first-flight mode and radius");
}
static void testBeginnerLimits() {
  Config c;
  configDefaults(c);
  c.angle_limit_deg = 55;
  c.beginner = 1;
  c.thr_limit_pct = 100;
  c.fence_enable = 0;
  c.fence_radius = 500;
  c.beg_radius = 50;
  c.fence_alt_max = 0;
  Config e = c;
  beginnerLimits(e);
  CHECK(e.angle_limit_deg <= 25 && e.thr_limit_pct <= 75 && e.fence_enable == 1 && e.fence_radius <= 50.5f && e.fence_alt_max <= 40.5f &&
            e.pos_max_speed <= 3.01f && e.pos_max_angle_deg <= 20.5f,
        "First-flight mode: tilt, throttle, speed and a 50 m / 40 m fence");
  Config tame = c;
  tame.angle_limit_deg = 15;
  tame.thr_limit_pct = 60;
  tame.fence_enable = 1;
  tame.fence_radius = 30;
  Config t2 = tame;
  beginnerLimits(t2);
  CHECK(t2.angle_limit_deg == tame.angle_limit_deg && t2.thr_limit_pct == 60 && t2.fence_radius <= 30.5f,
        "First-flight mode only tightens: tamer settings stay as set");
}
static void testMavlinkLowRate() {
  // LoRa E32 at 9600 bit/s carries ~900 B/s at best: the low-rate stream must fit well below that
  auto rate = [](bool low) {
    hal::host::reset();
    configDefaults(bus::config);
    comms::MavlinkLink link;
    link.begin(hal::Port::SERIAL2);
    link.setLowRate(low);
    uint32_t t = 1000;
    for (int i = 0; i < 1000; i++) { link.update(t); t += 10; }  // 10 s
    return hal::host::out(hal::Port::SERIAL2).size() / 10.0;
  };
  double lo = rate(true), hi = rate(false);
  char msg[120];
  snprintf(msg, sizeof msg, "MAVLink low rate fits a 9600 bit/s LoRa link: %.0f B/s (normal %.0f B/s)", lo, hi);
  CHECK(lo > 20 && lo < 300 && hi > lo * 3, msg);
}
static void testPinMap() {
  hal::PinUse pm[hal::PINMAP_MAX];
  int n = hal::pinMap(pm, hal::PINMAP_MAX);
  bool dup = false, motors = false, rx1 = false;
  for (int i = 0; i < n; i++) {
    for (int k = i + 1; k < n; k++) dup |= pm[i].pin == pm[k].pin;
    char b[8];
    if (pm[i].group == hal::PinGroup::MOTOR && pm[i].arg == 1) motors = !strcmp(hal::pinText(pm[i].pin, b, sizeof(b)), "PA0");
    if (pm[i].group == hal::PinGroup::UART && pm[i].arg == 0 && !strcmp(pm[i].what, "RX")) rx1 = !strcmp(hal::pinText(pm[i].pin, b, sizeof(b)), "PD9");
  }
  CHECK(n > 40 && !dup && motors && rx1, "Pin map: every pin listed once (motor 1 PA0, SERIAL1 RX PD9 on the demo map)");
  configDefaults(bus::config);
  std::string pj;
  web::paramsJson(sinkString, &pj);
  CHECK(pj.find("\"pinmap\":{\"board\":") != std::string::npos && pj.find("[\"PD9\",\"uart\",0,\"RX\"]") != std::string::npos,
        "Pin map sent to the configurator (/api/params pinmap)");
  std::string out;
  web::cliExec("pins uart", sinkString, &out, true);
  CHECK(out.find("PD9") != std::string::npos && out.find("SERIAL1 RX USART3") != std::string::npos && out.find("RX") != std::string::npos &&
            out.find("PA0") == std::string::npos,
        "CLI pins: filtered list with the port function");
  if (getenv("AQ_PINS_DUMP")) { std::string all; web::cliExec("pins", sinkString, &all, true); printf("%s", all.c_str()); }
}
static void testFlowModuleProtocols() {
  // MicoAir MTF-01 / 01P on an MSP port: Micolink decoded, MAVLink recognised (wrong module setting)
  hal::host::reset();
  configDefaults(bus::config);
  bus::sensorLink.port = 0xFF;
  comms::MspLink link;
  link.begin(hal::Port::SERIAL1);
  auto frame = [](uint8_t seq, uint32_t mm, int16_t fx, int16_t fy, uint8_t q) {
    uint8_t f[27] = {0xEF, 0x01, 0x00, 0x51, seq, 20};
    uint8_t* p = f + 6;
    uint32_t t = seq * 10u;
    memcpy(p, &t, 4); memcpy(p + 4, &mm, 4);
    p[8] = 200; p[9] = 1; p[10] = 1;
    memcpy(p + 12, &fx, 2); memcpy(p + 14, &fy, 2);
    p[16] = q; p[17] = q ? 1 : 0;
    uint8_t sum = 0;
    for (int i = 0; i < 26; i++) sum = (uint8_t)(sum + f[i]);
    f[26] = sum;
    return std::vector<uint8_t>(f, f + 27);
  };
  uint32_t r0 = bus::range.seq(), f0 = bus::flow.seq();
  for (int i = 0; i < 5; i++) {
    for (uint8_t b : frame((uint8_t)i, 1234, 50, -20, 180)) hal::host::in(hal::Port::SERIAL1).push_back(b);
    hal::host::in(hal::Port::SERIAL1).push_back(0x55);  // noise between frames
    link.update(1000 + i * 10);
  }
  RangeSample r = bus::range.get();
  FlowSample f = bus::flow.get();
  CHECK(bus::range.seq() != r0 && r.valid && fabsf(r.distM - 1.234f) < 1e-4f && bus::flow.seq() != f0 && fabsf(f.rateX - 0.5f) < 1e-4f &&
            fabsf(f.rateY + 0.2f) < 1e-4f && f.quality == 180 && bus::sensorLink.port == 0 && bus::sensorLink.mico == 5,
        "MSP sensor port: Micolink range 1.234 m + flow 0.5 / -0.2 rad/s decoded (MTF-01 factory protocol)");
  // rangefinder-only module (flow fields zero): range yes, no flow samples
  {
    comms::MspLink lr;
    lr.begin(hal::Port::SERIAL3);
    uint32_t fs = bus::flow.seq(), rs = bus::range.seq();
    for (int i = 0; i < 3; i++) {
      for (uint8_t b : frame((uint8_t)i, 2500, 0, 0, 0)) hal::host::in(hal::Port::SERIAL3).push_back(b);
      lr.update(3000 + i * 10);
    }
    CHECK(bus::range.seq() != rs && fabsf(bus::range.get().distM - 2.5f) < 1e-4f && bus::flow.seq() == fs,
          "MSP sensor port: rangefinder-only Micolink module gives height, no flow samples");
  }
  // the same module set to MAVLink (ArduPilot): no data, but the selftest says why
  hal::host::reset();
  comms::MspLink link2;
  link2.begin(hal::Port::SERIAL2);
  bus::sensorLink.port = 0xFF;
  for (int i = 0; i < 6; i++) {
    const uint8_t mav[] = {0xFD, 14, 0, 0, (uint8_t)i, 200, 158, 132, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 0x12, 0x34};
    for (uint8_t b : mav) hal::host::in(hal::Port::SERIAL2).push_back(b);
    link2.update(2000 + i * 10);
  }
  bus::config.port_fn[1] = (uint8_t)PortFn::MSP;
  std::string st;
  web::cliExec("selftest", sinkString, &st, true);
  CHECK(bus::sensorLink.port == 1 && bus::sensorLink.mav >= 5 && !bus::sensorLink.mico && !bus::sensorLink.msp &&
            st.find("[FAIL] flow") != std::string::npos && st.find("MicoAssistant") != std::string::npos,
        "MSP sensor port: a module sending MAVLink is reported (selftest: switch it to MSP / Micolink)");
  bus::sensorLink.port = 0xFF;
}
static void testConfigMigrationV29() {
  Config old;
  configDefaults(old);
  old.beg_radius = 123;
  old.pid[0].d = 33;
  size_t end = offsetof(Config, beg_radius) + sizeof(old.beg_radius);
  size_t oldSize = ((end + 3) & ~(size_t)3) + 4;  // v29: ... beg_radius, crc
  memset((uint8_t*)&old + end, 0x5A, oldSize - 4 - end);
  old.version = 29;
  old.size = (uint16_t)oldSize;
  uint32_t crc = crc32(&old, oldSize - 4);
  memcpy((uint8_t*)&old + oldSize - 4, &crc, 4);
  memset((uint8_t*)&old + oldSize, 0xEE, sizeof(Config) - oldSize);
  Config c = old;
  CHECK(configMigrate(c) && configValid(c) && c.beg_radius == 123 && c.pid[0].d == 33 && c.rc_map[0] == 1 && c.rc_map[1] == 2 &&
            c.rc_map[2] == 3 && c.rc_map[3] == 4 && c.rc_deadband == 5,
        "Config v29 -> v30: settings kept, channel map AETR, dead-band 5 us (as before)");
  CHECK(c.bb_gyro_s == 0 && paramFind("bb_gyro_s") >= 0, "Config v31: high-rate gyro capture off after the update");
  // channel map: TAER receiver (throttle on channel 1) -> roll pitch throttle yaw
  uint16_t ch[8] = {1100, 1600, 1400, 1700, 1000, 2000, 1500, 1500};
  c.rc_map[0] = 2; c.rc_map[1] = 3; c.rc_map[2] = 1; c.rc_map[3] = 4;
  rcRemap(c, ch, 8);
  bool taer = ch[0] == 1600 && ch[1] == 1400 && ch[2] == 1100 && ch[3] == 1700 && ch[4] == 1000 && ch[5] == 2000;
  uint16_t ch2[4] = {1100, 1600, 1400, 1700};
  c.rc_map[0] = 1; c.rc_map[1] = 1;  // not a permutation: left as received
  rcRemap(c, ch2, 4);
  CHECK(taer && ch2[0] == 1100 && ch2[1] == 1600, "RC channel map: TAER reordered, AUX untouched, a broken map leaves the order alone");
  CHECK(rcStickUs(1503, 5) == 0.0f && rcStickUs(1503, 0) > 0.0f && rcStickUs(1520, 25) == 0.0f && fabsf(rcStickUs(2000, 5) - 1.0f) < 1e-6f,
        "Stick dead-band follows rc_deadband");
  int k = paramFind("rc_ch_thr");
  CHECK(k >= 0 && paramSet(c, k, 1) && c.rc_map[2] == 1 && !paramSet(c, k, 5) && paramFind("rc_deadband") >= 0, "Parameters rc_ch_* / rc_deadband");
}
static void testGyroTap() {
  using namespace logging;
  // 8 kHz loop -> captured at 4 kHz; 50 ms of samples -> 12 full frames, in order, 4 ms apart
  gyroTapConfigure(8000);
  CHECK(gyroTapAvailable() && gyroTapRate() == 4000, "Gyro capture: 8 kHz loop captured at 4 kHz");
  gyroTapStart(1000000, 100);
  for (int k = 0; k < 400; k++) gyroTapPush(Vec3((float)k, -(float)k, 1.5f), Vec3(0.25f, 0, 0), 1000000 + k * 125);
  GyroFrame f;
  int frames = 0;
  bool order = true, times = true;
  uint32_t t0 = 0;
  while (gyroTapFrame(f)) {
    uint8_t x = 0;
    for (size_t i = 0; i < sizeof(f) - 1; i++) x ^= ((const uint8_t*)&f)[i];
    for (int i = 0; i < GYRO_FRAME_N; i++) {
      int want = 2 * (frames * GYRO_FRAME_N + i) + 1;  // every 2nd loop sample (the 2nd of each pair)
      order &= f.s[i][0] == want * 10 && f.s[i][1] == -want * 10 && f.s[i][2] == 15 && f.s[i][3] == 2 && x == f.crc;
    }
    if (frames == 0) t0 = f.timeUs;
    else times &= f.timeUs - t0 == (uint32_t)frames * 4000u;
    frames++;
  }
  CHECK(frames == 12 && order && times && f.sync == 0xA8 && f.rateHz == 4000, "Gyro capture: 12 frames of 16 samples, raw + filtered, 4 ms apart");
  // the capture ends by itself; nothing is taken afterwards
  for (int k = 0; k < 64; k++) gyroTapPush(Vec3(), Vec3(), 1000000 + 100000 + k * 125);
  CHECK(!gyroTapActive() && !gyroTapFrame(f), "Gyro capture: stops after its duration");
  // the logger falls behind: the ring fills, samples are counted as dropped, the frames left stay whole
  gyroTapStart(2000000, 1000);
  uint32_t d0 = gyroTapDropped();
  for (int k = 0; k < 4000; k++) gyroTapPush(Vec3(1, 2, 3), Vec3(), 2000000 + k * 125);
  int n = 0;
  while (gyroTapFrame(f)) n++;
  CHECK(gyroTapDropped() > d0 && n > 0 && n <= 64, "Gyro capture: full ring drops samples, never half frames");
  gyroTapStop();
}
static void testBatteryPins() {
  // the selftest checks the wiring of the battery / current pins (ADC full scale 3.3 V)
  configDefaults(bus::config);
  bus::adcVbatV = 3.29f;
  bus::adcCurV = 0.5f;
  bus::config.cur_scale = 40;
  std::string st;
  web::cliExec("selftest", sinkString, &st, true);
  bool clip = st.find("[FAIL] battery") != std::string::npos && st.find("divider too weak") != std::string::npos;
  bus::adcVbatV = 1.40f;
  st.clear();
  web::cliExec("selftest", sinkString, &st, true);
  bool ok = st.find("divider too weak") == std::string::npos && st.find("pin 1.400 V") != std::string::npos;
  CHECK(clip && ok, "Selftest: a battery divider that saturates the ADC pin is reported, a normal one shows the pin voltage");
  configDefaults(bus::config);
  bus::adcVbatV = 0;
  bus::adcCurV = -1;
}
static void testFeedForward() {
  // stick snap: boost starts FF earlier; slow drift (below the jitter threshold) gets little FF;
  // near the max rate FF fades out
  AxisPid g{45, 80, 30, 120};
  auto run = [&](float jit, float boost, float maxPct, float (*sp)(int), int n, float* fAt, int at) {
    RatePid p;
    p.configure(g, 100, 300, 500, 15, 1000, 0, 0, 0, jit, boost, maxPct, 670);
    float last = 0;
    for (int k = 0; k < n; k++) {
      if (k % 4 == 0) p.rcFrame(sp(k));
      p.update(sp(k), 0, 0.001f, 1.0f, false);
      if (k == at) *fAt = p.f;
      last = p.f;
    }
    return last;
  };
  auto snap = [](int k) { return fminf(k * 10.0f, 600.0f); };  // 10 deg/s per ms up to 600
  auto slow = [](int k) { return k * 0.25f; };                  // 1 deg/s per 4 ms frame
  float f0 = 0, fb = 0, fs = 0, fsj = 0, fm = 0, fmx = 0;
  run(0, 0, 0, snap, 20, &f0, 15);
  run(0, 30, 0, snap, 20, &fb, 15);
  run(0, 0, 0, slow, 400, &fs, 399);
  run(7, 0, 0, slow, 400, &fsj, 399);
  auto nearMax = [](int k) { return 600.0f + fminf(k * 2.0f, 60.0f); };  // 600 -> 660 deg/s
  run(0, 0, 0, nearMax, 25, &fm, 20);
  run(0, 0, 90, nearMax, 25, &fmx, 20);
  CHECK(fb > f0 * 1.05f && fsj < fs * 0.5f && fmx < fm * 0.5f && f0 > 0,
        "Feed-forward: boost raises it on a snap, jitter cut on slow moves, fades near the max rate");
  // a snap, then the stick stops: the fading FF of the snap is not cut by the jitter reduction
  float fh = 0, fhj = 0;
  run(0, 0, 0, snap, 70, &fh, 64);
  run(7, 0, 0, snap, 70, &fhj, 64);
  CHECK(fh > 0 && fabsf(fhj - fh) < 0.05f * fh, "Feed-forward: jitter reduction keeps the FF of a real stick move after it stops");
}

static void testFlightFeel() {
  // thrust linearisation: the output gives exactly the requested thrust through (1-e)t + e t^2
  bool lin = true;
  for (float e : {0.3f, 0.65f, 0.9f})
    for (float T = 0.05f; T <= 1.0f; T += 0.05f) {
      float t = Mixer::linearize(T, e);
      lin &= fabsf((1 - e) * t + e * t * t - T) < 1e-4f && t >= T - 1e-6f;
    }
  CHECK(lin, "Thrust linearisation: output inverts the thrust curve (more output at low thrust)");
  // anti-windup: while the mixer is saturated the I-term cannot grow further
  RatePid a, b;
  AxisPid g{45, 80, 30, 0};
  a.configure(g, 100, 400, 500, 0, 1000);
  b.configure(g, 100, 400, 500, 0, 1000);
  for (int k = 0; k < 500; k++) {
    a.update(20, 0, 0.001f, 1, true);
    b.setSaturated(k > 100);
    b.update(20, 0, 0.001f, 1, true);
  }
  float iAt100 = 0;
  {
    RatePid c2;
    c2.configure(g, 100, 400, 500, 0, 1000);
    for (int k = 0; k <= 100; k++) c2.update(20, 0, 0.001f, 1, true);
    iAt100 = c2.i;
  }
  CHECK(a.i > b.i * 2 && fabsf(b.i - iAt100) < 1e-3f, "I-term anti-windup: frozen while the mixer is saturated");
  b.setSaturated(true);
  float before = b.i;
  for (int k = 0; k < 50; k++) b.update(-200, 0, 0.001f, 1, true);
  CHECK(b.i < before, "I-term anti-windup: still allowed to unwind");
  // D_min: calm -> D at d_min_pct, busy gyro -> full D
  RatePid dm, full;
  dm.configure(g, 200, 400, 500, 0, 1000, 50, 37, 20);
  full.configure(g, 200, 400, 500, 0, 1000);
  float dCalm = 0, dFullCalm = 0;
  for (int k = 0; k < 2000; k++) {
    float gy = 2.0f * sinf(k * 0.01f);  // slow drift
    dm.update(0, gy, 0.001f, 1, false); full.update(0, gy, 0.001f, 1, false);
    if (k > 1500) { dCalm += fabsf(dm.d); dFullCalm += fabsf(full.d); }
  }
  float dBusy = 0, dFullBusy = 0;
  for (int k = 0; k < 2000; k++) {
    float gy = 150.0f * sinf(k * 0.5f);  // prop wash oscillation
    dm.update(0, gy, 0.001f, 1, false); full.update(0, gy, 0.001f, 1, false);
    if (k > 1500) { dBusy += fabsf(dm.d); dFullBusy += fabsf(full.d); }
  }
  char m[96];
  snprintf(m, sizeof(m), "D_min: calm %.0f %% of D, prop wash %.0f %% of D", 100 * dCalm / dFullCalm, 100 * dBusy / dFullBusy);
  CHECK(dCalm / dFullCalm < 0.6f && dBusy / dFullBusy > 0.9f, m);
}

static void testEllipsoidFit() {
  // a compass with hard iron (0.15, -0.08, 0.3) and soft iron (stretch + skew) seen from every direction
  std::mt19937 rng(7);
  std::normal_distribution<float> nd(0, 1);
  const float H[3][3] = {{1.15f, 0.08f, -0.05f}, {0.08f, 0.9f, 0.04f}, {-0.05f, 0.04f, 1.02f}};
  const Vec3 off(0.15f, -0.08f, 0.3f);
  std::vector<Vec3> pts;
  for (int i = 0; i < 200; i++) {
    Vec3 u(nd(rng), nd(rng), nd(rng));
    u = u * (0.5f / u.norm());  // field 0.5 G
    Vec3 r(H[0][0] * u.x + H[0][1] * u.y + H[0][2] * u.z, H[1][0] * u.x + H[1][1] * u.y + H[1][2] * u.z,
           H[2][0] * u.x + H[2][1] * u.y + H[2][2] * u.z);
    pts.push_back(r + off + Vec3(nd(rng), nd(rng), nd(rng)) * 0.002f);
  }
  Vec3 c;
  float S[6];
  bool ok = ellipsoidFit(pts.data(), (int)pts.size(), c, S);
  float bias[3] = {c.x, c.y, c.z}, d[3] = {S[0], S[1], S[2]}, o[3] = {S[3], S[4], S[5]};
  float mn = 1e9f, mx = 0;
  for (const Vec3& p : pts) { float n = magCorrect(p, bias, d, o).norm(); mn = fminf(mn, n); mx = fmaxf(mx, n); }
  float mnRaw = 1e9f, mxRaw = 0;
  for (const Vec3& p : pts) { float n = (p - off).norm(); mnRaw = fminf(mnRaw, n); mxRaw = fmaxf(mxRaw, n); }
  char m[120];
  snprintf(m, sizeof(m), "Compass ellipsoid fit: offset error %.3f G, field spread %.1f %% (hard iron only %.0f %%)",
           (c - off).norm(), 100 * (mx - mn) / mx, 100 * (mxRaw - mnRaw) / mxRaw);
  CHECK(ok && (c - off).norm() < 0.01f && (mx - mn) / mx < 0.03f, m);
}

static void testPolygonFenceAndPath() {
  nav::fenceSetCount(0);
  // 100 m x 60 m rectangle around home (lat/lon offsets)
  const double lat0 = 21.0, lon0 = 105.0, mlat = 1.0 / 111319.49, mlon = mlat / cos(lat0 * 0.0174532925);
  double pts[4][2] = {{-50, -30}, {50, -30}, {50, 30}, {-50, 30}};
  for (int i = 0; i < 4; i++) {
    nav::MissionItem it{};
    it.cmd = nav::CMD_FENCE_POLYGON_INCLUSION;
    it.lat = (int32_t)((lat0 + pts[i][0] * mlat) * 1e7);
    it.lon = (int32_t)((lon0 + pts[i][1] * mlon) * 1e7);
    nav::fenceSet(i, it);
  }
  nav::fenceSetCount(4);
  bool in = !nav::fenceOutside(lat0, lon0, 10, 20) && !nav::fenceOutside(lat0, lon0, -45, -25);
  bool out = nav::fenceOutside(lat0, lon0, 60, 0) && nav::fenceOutside(lat0, lon0, 0, 35);
  Config c;
  configDefaults(c);
  c.fence_enable = 1;
  c.fence_radius = 500;
  NavEstimate e{};
  e.homeSet = true; e.posValid = true; e.homeLat = lat0; e.homeLon = lon0; e.n = 0; e.e = 40;
  bool breach = fenceBreached(c, e);
  nav::fenceSetCount(0);
  CHECK(in && out && breach, "Polygon geofence: inside / outside, breach inside the cylinder");
  // SmartRTL path: spacing, loop pruning, decimation when full
  nav::SmartPath p;
  for (int i = 0; i <= 30; i++) p.record((float)i, 0, 10, 3.0f);         // 30 m north -> points every 3 m
  int straight = p.count();
  for (int i = 0; i <= 20; i++) p.record(30.0f, (float)i, 10, 3.0f);     // 20 m east
  for (int i = 20; i >= 0; i--) p.record(30.0f + 0.2f, (float)i, 10, 3.0f);  // back west: the loop is cut
  int afterLoop = p.count();
  nav::SmartPath q;
  for (int i = 0; i < 2000; i++) q.record(i * 3.5f, 0, 10, 3.0f);
  float n0 = -1, e0, a0, nl = 0, el, al;
  q.get(0, n0, e0, a0);
  q.get(q.count() - 1, nl, el, al);
  char m[100];
  snprintf(m, sizeof(m), "SmartRTL path: %d points straight, %d after flying a loop back, %d when full", straight, afterLoop, q.count());
  CHECK(straight == 11 && afterLoop <= straight + 1 && q.count() <= nav::SmartPath::MAX && n0 == 0 && nl > 6900, m);
}

static void testEscCommands() {
  char msg[96];
  bus::escCmdPending = false;
  int r = web::command("escdir_3_1", msg, sizeof(msg));
  CHECK(r == 200 && bus::escCmdPending && bus::escCmd == 8 && bus::escCmdMask == 4 && bus::escCmd2 == 12,
        "ESC: spin direction of motor 3 queued as DShot 8 + save (12) for that motor only");
  bus::escCmdPending = false;
  r = web::command("escbeep_9", msg, sizeof(msg));
  CHECK(r == 200 && bus::escCmd == 1 && bus::escCmdMask == 9, "ESC: beep motors 1 + 4");
  bus::escCmdPending = false;
  r = web::command("esccal_hi", msg, sizeof(msg));
  CHECK(r == 409 && !bus::escCalState, "ESC calibration refused on DShot (would be full throttle)");
  CHECK(web::command("escdir_7_1", msg, sizeof(msg)) == 400, "ESC: motor number checked");
  bus::escCmdPending = false;
}

static void testDshotBidir() {
  // frames: normal vs inverted checksum
  uint16_t a = dshot::frame(1046, false, false), b = dshot::frame(1046, false, true);
  CHECK((a & 0xFFF0) == (b & 0xFFF0) && ((a ^ b) & 0xF) == 0xF, "DShot: bidirectional frame has the inverted checksum");
  // ESC answer round trip for a range of speeds
  bool allOk = true;
  float worst = 0;
  for (float erpm : {1000.0f, 5000.0f, 12345.0f, 30000.0f, 60000.0f, 150000.0f}) {
    uint32_t g = dshot::gcrEncode(dshot::withChecksum(dshot::valueFromErpm(erpm)));
    uint8_t runs[24];
    int n = dshot::runsFromGcr(g, runs);
    // the line as the FC sees it: its own frame (16 bits, max high run < 1 bit), the gap, the answer
    uint8_t lv[64];
    float du[64];
    int k = 0;
    for (int i = 0; i < 16; i++) { lv[k] = 0; du[k++] = 1.0f; lv[k] = 1; du[k++] = 1.5f; }
    lv[k] = 1; du[k++] = 30.0f;  // gap (in telemetry bits: ~11)
    du[k - 2] += 0;              // merge not needed: separate runs of the same level are allowed below
    // merge the last frame high run with the gap
    du[k - 2] += du[k - 1]; k--;
    for (int i = 0; i < n; i++) { lv[k] = (uint8_t)(i % 2 == 0 ? 0 : 1); du[k++] = runs[i] * (1.0f + 0.08f * ((i % 3) - 1)); }
    lv[k] = (uint8_t)(n % 2 == 0 ? 0 : 1);
    if (lv[k] == lv[k - 1]) du[k - 1] += 20; else du[k++] = 20;  // idle
    float out = -1;
    bool ok = dshot::decodeRuns(lv, du, k, 1.0f, out);
    float err = fabsf(out - erpm) / erpm;
    worst = fmaxf(worst, err);
    allOk &= ok && err < 0.01f;
  }
  char m[96];
  snprintf(m, sizeof(m), "Bidirectional DShot: eRPM answers decoded after the FC frame (worst %.2f %%)", worst * 100);
  CHECK(allOk, m);
  float z = -1;
  uint32_t g0 = dshot::gcrEncode(dshot::withChecksum(dshot::valueFromErpm(0)));
  uint8_t r0[24];
  int n0 = dshot::runsFromGcr(g0, r0);
  uint8_t lv0[32]; float du0[32]; int k0 = 0;
  lv0[k0] = 1; du0[k0++] = 10;
  for (int i = 0; i < n0; i++) { lv0[k0] = (uint8_t)(i % 2 ? 1 : 0); du0[k0++] = r0[i]; }
  CHECK(dshot::decodeRuns(lv0, du0, k0, 1.0f, z) && z == 0, "Bidirectional DShot: stopped motor reports 0 eRPM");
  // corrupted answer: checksum / GCR error is refused
  uint32_t gb = dshot::gcrEncode(dshot::withChecksum(dshot::valueFromErpm(20000))) ^ (1u << 7);
  n0 = dshot::runsFromGcr(gb, r0);
  k0 = 0; lv0[k0] = 1; du0[k0++] = 10;
  for (int i = 0; i < n0; i++) { lv0[k0] = (uint8_t)(i % 2 ? 1 : 0); du0[k0++] = r0[i]; }
  CHECK(!dshot::decodeRuns(lv0, du0, k0, 1.0f, z), "Bidirectional DShot: a corrupted answer is rejected");
  // STM32 bit-bang input: GPIO samples at 3x, two motors on one port
  uint16_t smp[150];
  int sN = 0;
  float e1 = 21000, e2 = 47000;
  uint8_t ra[24], rb[24];
  int na = dshot::runsFromGcr(dshot::gcrEncode(dshot::withChecksum(dshot::valueFromErpm(e1))), ra);
  int nb = dshot::runsFromGcr(dshot::gcrEncode(dshot::withChecksum(dshot::valueFromErpm(e2))), rb);
  auto level = [](const uint8_t* r, int n, int startSample, int s) {  // 3 samples per bit
    if (s < startSample) return 1;
    int bit = (s - startSample) / 3, acc = 0;
    for (int i = 0; i < n; i++) { acc += r[i]; if (bit < acc) return i % 2 ? 1 : 0; }
    return n % 2 ? 1 : 0;  // stays at the last level... then idle high
  };
  for (sN = 0; sN < 150; sN++) {
    int la = level(ra, na, 30, sN), lb = level(rb, nb, 36, sN);
    if (sN > 30 + 63 + 3) la = 1;
    if (sN > 36 + 63 + 3) lb = 1;
    smp[sN] = (uint16_t)((la ? 1u << 4 : 0) | (lb ? 1u << 5 : 0) | 0x0100);
  }
  float d1 = -1, d2 = -1;
  bool s1 = dshot::decodeSamples(smp, sN, 1u << 4, 3.0f, d1), s2 = dshot::decodeSamples(smp, sN, 1u << 5, 3.0f, d2);
  CHECK(s1 && s2 && fabsf(d1 - e1) / e1 < 0.01f && fabsf(d2 - e2) / e2 < 0.01f,
        "Bidirectional DShot (STM32 bit-bang): two motors decoded from one sample buffer");
  // the RPM filter is fed from bidir DShot without the ESC telemetry wire
  Config c;
  configDefaults(c);
  c.esc_telemetry = 0;
  c.dshot_bidir = 1;
  RpmFilter rf;
  rf.configure(1000, c.rpm_harmonics, c.rpm_min_hz, c.rpm_q);
  CHECK(rf.enabled(), "RPM filter enabled by bidirectional DShot");
}

static void testVtxTable() {
  Config c;
  configDefaults(c);
  c.vtxt_powers = 3;
  c.vtxt_mw[0] = 1; c.vtxt_mw[1] = 250; c.vtxt_mw[2] = 1000;
  c.vtxt_val[0] = 0; c.vtxt_val[1] = 24; c.vtxt_val[2] = 30;
  c.vtx_power = 3;
  CHECK(vtx::powerCount(c) == 3 && vtx::powerMw(c, 2) == 250 && vtx::powerSaValue(c, 3) == 30,
        "VTX table: custom power levels");
  c.vtx_band = 7;
  c.vtx_channel = 2;
  c.vtxt_band[1] = 5333;
  c.vtxt_band[1] = 5555;
  CHECK(vtx::frequency(c) == 5555, "VTX table: user band U2 = 5555 MHz");
  vtx::Driver d;
  hal::host::out(hal::Port::SERIAL2).clear();
  d.begin(vtx::Type::SMARTAUDIO, hal::Port::SERIAL2);
  for (uint32_t t = 2000; t < 2400; t += 10) d.update(c, true, t);
  const std::string& o = hal::host::out(hal::Port::SERIAL2);
  bool dbm = o.size() > 20 && (uint8_t)o[9 + 5] == (30 | 0x80);
  c.vtxt_sa_index = 1;
  c.vtxt_val[2] = 2;
  hal::host::out(hal::Port::SERIAL2).clear();
  vtx::Driver d2;
  d2.begin(vtx::Type::SMARTAUDIO, hal::Port::SERIAL2);
  for (uint32_t t = 2000; t < 2400; t += 10) d2.update(c, true, t);
  const std::string& o2 = hal::host::out(hal::Port::SERIAL2);
  bool idx = o2.size() > 20 && (uint8_t)o2[9 + 5] == 2;
  CHECK(d.sentPowerMw() == 1000 && dbm && idx, "VTX table: SmartAudio gets dBm, or the level index for old VTX");
}

// coning / sculling: delta angle and velocity summed over a 4 ms interval of 1 kHz samples, under a
// fast two-axis rotation, against a fine (100 kHz) exact integration
static fc::Quat qFromRot(const fc::Vec3& r) {
  float a = r.norm();
  if (a < 1e-9f) return fc::Quat{1, 0, 0, 0};
  float s = sinf(a * 0.5f) / a;
  return fc::Quat{cosf(a * 0.5f), r.x * s, r.y * s, r.z * s};
}
#define CHECKF(cond, ...)                         \
  do {                                            \
    char m_[160];                                 \
    snprintf(m_, sizeof(m_), __VA_ARGS__);        \
    CHECK(cond, m_);                              \
  } while (0)
static void testImuDelta() {
  using fc::Vec3;
  using fc::Quat;
  auto rate = [](double t) { return Vec3((float)(17.5 * sin(2 * M_PI * 3 * t)), (float)(17.5 * cos(2 * M_PI * 3 * t)), 3.0f); };  // ~1000 deg/s coning
  const Vec3 accE(0.5f, -1.0f, -9.81f);  // specific force, earth frame
  Quat qT{1, 0, 0, 0}, qC{1, 0, 0, 0}, qP{1, 0, 0, 0};
  double t = 0, errC = 0, errP = 0, vErrC = 0, vErrP = 0;
  const int FINE = 100;
  for (int iv = 0; iv < 250; iv++) {
    Quat q0 = qT;
    Vec3 dA, dV, pA, pV;
    for (int s = 0; s < 4; s++) {  // four 1 kHz samples per interval
      Vec3 dTh, dVs;
      for (int f = 0; f < FINE; f++) {  // exact: fine rotation steps, specific force in the body frame
        double h = 1e-3 / FINE;
        Vec3 w = rate(t + h * 0.5);
        Vec3 ab = qT.rotateInv(accE);
        dTh += w * (float)h;
        dVs += ab * (float)h;  // what an accelerometer sample integrates (averaged over the sample)
        qT = qT * qFromRot(w * (float)h);
        qT.normalize();
        t += h;
      }
      fc::imuDeltaAdd(dA, dV, dTh, dVs);
      pA += dTh;
      pV += dVs;
    }
    qC = qC * qFromRot(dA); qC.normalize();
    qP = qP * qFromRot(pA); qP.normalize();
    Vec3 vTrue = q0.rotateInv(accE * 0.004f);  // velocity change in the start frame
    vErrC = fmax(vErrC, (double)(dV - vTrue).norm());
    vErrP = fmax(vErrP, (double)(pV - vTrue).norm());
    auto ang = [&](const Quat& q) { Quat d = qT.conj() * q; return 2.0 * acos(fmin(1.0, fabs((double)d.w))) * 180 / M_PI; };
    errC = ang(qC);
    errP = ang(qP);
  }
  CHECKF(errC < 0.2 * errP && errC < 2.0, "IMU delta: coning term keeps attitude after 1 s at 1000 deg/s (%.2f deg vs %.1f deg plain)", errC, errP);
  CHECKF(vErrC < 0.5 * vErrP, "IMU delta: sculling term keeps the velocity in the start frame (%.4f vs %.4f m/s)", vErrC, vErrP);
  // joining two intervals equals one interval over both
  Vec3 a1, v1, a2, v2, aj, vj;
  for (int k = 0; k < 8; k++) {
    Vec3 th(0.004f * k, 0.01f, -0.003f * k), dv(0.01f, -0.005f * k, -0.04f);  // ~1000 deg/s samples
    fc::imuDeltaAdd(aj, vj, th, dv);
    if (k < 4) fc::imuDeltaAdd(a1, v1, th, dv); else fc::imuDeltaAdd(a2, v2, th, dv);
  }
  fc::imuDeltaJoin(a1, v1, a2, v2);
  CHECKF((a1 - aj).norm() < 2e-4f && (v1 - vj).norm() < 1e-3f, "IMU delta: joining two intervals ~ one interval (%.5f / %.5f)", (double)(a1 - aj).norm(), (double)(v1 - vj).norm());
}


static void testSticksMenu() {
  using app::StickCommands;
  fc::Config& c = bus::config;
  fc::configDefaults(c);
  uint16_t ch[16];
  auto set = [&](int t, int y, int p, int r) { for (int i = 0; i < 16; i++) ch[i] = 1500; ch[fc::RC_THROTTLE] = t; ch[fc::RC_YAW] = y; ch[fc::RC_PITCH] = p; ch[fc::RC_ROLL] = r; };
  set(1000, 1000, 1000, 1500);
  CHECK(StickCommands::decode(ch) == StickCommands::GYRO_CAL, "sticks: throttle low, yaw left, pitch down = gyro calibration");
  set(2000, 1000, 1000, 1500);
  CHECK(StickCommands::decode(ch) == StickCommands::ACC_CAL, "sticks: throttle high, yaw left, pitch down = accelerometer");
  set(1500, 1000, 2000, 1500);
  CHECK(StickCommands::decode(ch) == StickCommands::MENU, "sticks: throttle mid, yaw left, pitch up = OSD menu");
  set(1500, 1500, 1500, 1500);
  CHECK(StickCommands::decode(ch) == StickCommands::NONE, "sticks: centred = nothing");
  StickCommands sc;
  bus::Request rq; uint32_t arg;
  while (bus::takeRequest(rq, arg)) {}
  set(1000, 1000, 1000, 1500);
  bool early = sc.update(c, ch, true, false, false, 1000) != StickCommands::NONE || sc.update(c, ch, true, false, false, 1900) != StickCommands::NONE;
  StickCommands::Cmd k = sc.update(c, ch, true, false, false, 2010);
  CHECK(!early && k == StickCommands::GYRO_CAL && bus::takeRequest(rq, arg) && rq == bus::Request::GYRO_CAL,
        "sticks: gyro calibration after 1 s held, not before");
  CHECK(sc.update(c, ch, true, false, false, 2500) == StickCommands::NONE, "sticks: once per hold");
  StickCommands s2;
  s2.update(c, ch, true, true, false, 0);
  CHECK(s2.update(c, ch, true, true, false, 2000) == StickCommands::NONE, "sticks: nothing while armed");
  StickCommands s3;
  s3.update(c, ch, true, false, true, 0);
  CHECK(s3.update(c, ch, true, false, true, 2000) == StickCommands::NONE, "sticks: nothing with the arm switch on");
  // trim repeats while held
  StickCommands s4;
  c.level_trim_pitch_deg = 0;
  set(2000, 1500, 2000, 1500);
  for (uint32_t t = 0; t <= 2000; t += 10) s4.update(c, ch, true, false, false, t);
  CHECK(c.level_trim_pitch_deg < -0.55f && c.level_trim_pitch_deg > -0.85f, "sticks: trim forward repeats (0.1 deg / 0.15 s)");
  // profile 2
  StickCommands s5;
  set(1000, 1000, 2000, 1500);
  s5.update(c, ch, true, false, false, 0);
  s5.update(c, ch, true, false, false, 1100);
  CHECK(c.pid_profile == 1, "sticks: profile 2");
  c.pid_profile = 0;
  // OSD menu: open, PID page, change roll P, back, save + exit
  StickCommands s6;
  set(1500, 1000, 2000, 1500);
  s6.update(c, ch, true, false, false, 0);
  s6.update(c, ch, true, false, false, 1100);
  CHECK(osd::menu.isOpen() && bus::osdMenuOpen, "menu: opened by the sticks");
  osd::menu.input(0, 0.8f, 1200);   // still held from the command: ignored until centred
  osd::menu.input(0, 0, 1300);
  float p0 = c.pid[0].p;
  osd::menu.input(0.9f, 0, 1400);   // roll right on "PID": open the page
  osd::menu.input(0, 0, 1500);
  osd::menu.input(0.9f, 0, 1600);   // roll right on "ROLL P": +1
  osd::menu.input(0, 0, 1700);
  CHECK(osd::menu.page() == 1 && fabsf(c.pid[0].p - (p0 + 1)) < 1e-3f, "menu: PID page, roll P + 1");
  char ln[16][osd::Menu::LINE + 1];
  int n = osd::menu.lines(ln, 16);
  CHECK(n > 5 && strstr(ln[0], "PID") && strstr(ln[1], "ROLL P") && ln[1][0] == '>', "menu: lines drawn with the cursor");
  for (int i = 0; i < 11; i++) osd::menu.move(1);     // to BACK
  osd::menu.press(1);
  CHECK(osd::menu.page() == 0, "menu: BACK");
  for (int i = 0; i < 8; i++) osd::menu.move(1);      // SAVE + EXIT
  bus::saveRequest = false;
  osd::menu.press(1);
  CHECK(!osd::menu.isOpen() && !bus::osdMenuOpen && bus::saveRequest, "menu: save + exit");
  bus::saveRequest = false;
  c.pid[0].p = p0;
}

static void testAltitudeHelpers() {
  // rangefinder height: offset learned at first sight, step (kerb / table) absorbed, not passed on
  fc::RangeHeight rh;
  float z = 0;
  CHECK(!rh.measure(1.0f, 5.0f, 0.1f, 1000, z), "range height: first sample only sets the offset");
  bool ok = rh.measure(1.02f, 5.0f, 0.1f, 1020, z);
  CHECK(ok && fabsf(z - 5.02f) < 0.01f, "range height: follows the range on top of the offset");
  CHECK(!rh.measure(0.2f, 5.0f, 0.1f, 1040, z) && rh.steps == 1, "range height: a 0.8 m step goes into the offset");
  ok = rh.measure(0.21f, 5.0f, 0.1f, 1060, z);
  CHECK(ok && fabsf(z - 5.01f) < 0.02f, "range height: after the step the height stays the same");
  CHECK(!rh.measure(0.21f, 5.0f, 0.1f, 2000, z), "range height: a gap > 0.5 s starts over");
  // baro corrections
  float alt = 10, r = 0.4f;
  fc::baroCorrect(alt, r, 10, 100, 0.025f, false);
  CHECK(fabsf(alt - 12.5f) < 1e-4f && fabsf(r - 0.8f) < 1e-4f, "baro: dynamic pressure added back, less weight at 10 m/s");
  alt = 0.2f; r = 0.4f;
  fc::baroCorrect(alt, r, 0.5f, 0, 0, true);
  CHECK(fabsf(alt - 0.5f) < 1e-4f && r > 0.4f, "baro: ground-effect dip under 0.5 m ignored");
  alt = -0.5f;
  fc::baroCorrect(alt, r, 0.5f, 0, 0, true);
  CHECK(fabsf(alt + 0.0f) < 1e-4f, "baro: larger dip counts 0.5 m less");
  alt = 1.0f;
  fc::baroCorrect(alt, r, 0.5f, 0, 0, true);
  CHECK(fabsf(alt - 1.0f) < 1e-4f, "baro: readings above the estimate pass near the ground");
  // altitude control in thrust: reset gives back the throttle, hover gain unchanged, tilt exact
  fc::Config c;
  fc::configDefaults(c);
  c.thr_curve = 0.5f;
  c.thrust_expo = 0;
  c.alt_vel_i = 0;
  c.alt_vel_d = 0;
  fc::AltitudeControl a;
  a.configure(c);
  fc::NavEstimate e{};
  e.alt = 3;
  a.reset(0.55f, 1.0f);
  float t = a.update(3, 0, 1, 1, e, 0, 1.0f, 0.001f);
  CHECK(fabsf(t - 0.55f) < 0.002f, "altitude: reset in thrust gives the same throttle back");
  a.reset(c.hover_throttle, 1.0f);
  float t0 = a.update(3, 0, 1, 1, e, 0, 1.0f, 0.001f);
  e.vz = -0.02f;
  float t1 = a.update(3, 0, 1, 1, e, 0, 1.0f, 0.001f);
  c.thr_curve = 0;
  a.reset(c.hover_throttle, 1.0f);
  e.vz = 0;
  float l0 = a.update(3, 0, 1, 1, e, 0, 1.0f, 0.001f);
  e.vz = -0.02f;
  float l1 = a.update(3, 0, 1, 1, e, 0, 1.0f, 0.001f);
  CHECK(fabsf((t1 - t0) - (l1 - l0)) < 0.1f * fabsf(l1 - l0) + 1e-5f, "altitude: same gain at hover with or without the curve");
  c.thr_curve = 0.5f;
  a.reset(c.hover_throttle, 1.0f);
  e.vz = 0;
  float tt = a.update(3, 0, 1, 1, e, 0, 0.8f, 0.001f), h = c.hover_throttle;
  float f = (0.5f * tt + 0.5f * tt * tt) * 0.8f, fh = 0.5f * h + 0.5f * h * h;
  CHECK(fabsf(f - fh) < 0.003f, "altitude: tilt compensated in thrust (vertical thrust unchanged)");
}

static void testEkf() {
  Config c;
  configDefaults(c);
  Ekf k;
  k.configure(c);
  // level craft accelerating north at 1 m/s^2 for 5 s with GPS, baro and heading: tracks position
  k.init(Quat::fromEuler(0, 0, 0.3f), 0);
  std::mt19937 rng(3);
  std::normal_distribution<float> nz(0, 1);
  Vec3 p, v;
  float maxErr = 0;
  float yaw = 0.3f;
  Quat q = Quat::fromEuler(0, 0, yaw);
  uint32_t ms = 0;
  for (int i = 0; i < 250 * 20; i++) {
    float dt = 0.004f;
    Vec3 aN(i < 1250 ? 1.0f : (i < 2500 ? -1.0f : 0.0f), 0.5f * sinf(i * 0.01f), 0);
    v += aN * dt;
    p += v * dt;
    Vec3 f = q.rotateInv(aN - Vec3(0, 0, GRAVITY));  // specific force, body
    k.predict(Vec3(nz(rng) * 1e-4f, 0, 0), (f + Vec3(nz(rng) * 0.3f, nz(rng) * 0.3f, nz(rng) * 0.3f)) * dt, dt, ms);
    ms += 4;
    if (i % 25 == 0) {
      k.fuseGpsPos(p.x + nz(rng) * 0.5f, p.y + nz(rng) * 0.5f, 0.8f, ms);
      k.fuseGpsVel(v.x + nz(rng) * 0.1f, v.y + nz(rng) * 0.1f, v.z, 0.3f, ms);
      k.fuseYaw(yaw + nz(rng) * 0.03f, 0.1f);
    }
    if (i % 5 == 0) k.fuseHeight(-p.z + nz(rng) * 0.3f, 0.4f, 0, ms);
    if (i > 500) maxErr = fmaxf(maxErr, (k.pos() - p).norm());
  }
  char m[96];
  snprintf(m, sizeof(m), "EKF: position tracked through accelerations (max error %.2f m)", maxErr);
  CHECK(maxErr < 1.0f && k.status().faults == 0, m);
  // GPS jump: rejected first, accepted after gps_glitch_s
  bool rejected = false;
  for (int i = 0; i < 250 * 12; i++) {
    float dt = 0.004f;
    p += v * dt;
    k.predict(Vec3(), q.rotateInv(Vec3(0, 0, -GRAVITY)) * dt, dt, ms);
    ms += 4;
    if (i % 25 == 0) {
      k.fuseGpsPos(p.x + 30.0f, p.y, 0.8f, ms);
      k.fuseGpsVel(v.x, v.y, 0, 0.3f, ms);
      if (i < 250 * 5 && (k.status().faults & EKF_GPS_GLITCH) && fabsf(k.pos().x - p.x) < 3) rejected = true;
    }
  }
  CHECK(rejected && fabsf(k.pos().x - (p.x + 30)) < 2.0f && k.status().posResets >= 1,
        "EKF: 30 m GPS jump rejected, accepted as the new position after gps_glitch_s");
  // baro fault: the baro steps 15 m while GPS height stays consistent
  k.init(q, 0);
  bool fault = false;
  float altErr = 0;
  for (int i = 0; i < 250 * 30; i++) {
    float dt = 0.004f;
    k.predict(Vec3(), q.rotateInv(Vec3(0, 0, -GRAVITY)) * dt, dt, ms);
    ms += 4;
    float baro = i > 250 * 5 ? 15.0f : 0.0f;
    if (i % 5 == 0) k.fuseHeight(baro + nz(rng) * 0.2f, 0.4f, 0, ms);
    if (i % 25 == 0) {
      k.fuseGpsPos(nz(rng) * 0.3f, nz(rng) * 0.3f, 0.8f, ms);
      k.fuseGpsVel(0, 0, 0, 0.3f, ms);
      if (k.baroFault()) k.fuseHeight(nz(rng) * 1.0f, 1.5f, 1, ms);
    }
    if (k.baroFault()) fault = true;
    if (i > 250 * 20) altErr = fmaxf(altErr, fabsf(k.alt()));
  }
  snprintf(m, sizeof(m), "EKF: baro step detected as a fault, height from GPS (error %.2f m)", altErr);
  CHECK(fault && altErr < 2.5f, m);
  // compass fault: compass 70 deg off while GPS shows the true heading through accelerations
  k.init(q, 0);
  bool magFault = false;
  Vec3 magEarth(0.22f, 0, 0.42f);
  p = v = Vec3();
  for (int i = 0; i < 250 * 40; i++) {
    float dt = 0.004f;
    Vec3 aN(2.0f * sinf(i * 0.004f * 1.3f), 2.0f * cosf(i * 0.004f * 0.9f), 0);
    v += aN * dt;
    p += v * dt;
    k.predict(Vec3(), q.rotateInv(aN - Vec3(0, 0, GRAVITY)) * dt, dt, ms);
    ms += 4;
    if (i % 25 == 0) {
      k.fuseGpsPos(p.x, p.y, 0.8f, ms);
      k.fuseGpsVel(v.x, v.y, 0, 0.3f, ms);
    }
    if (i % 5 == 0) {
      k.fuseHeight(0, 0.4f, 0, ms);
      Quat bad = Quat::fromEuler(0, 0, yaw + (i > 250 * 5 ? 70.0f * DEG2RAD : 0.0f));
      Vec3 mb = bad.rotateInv(magEarth);
      k.checkMag(mb, 0, true, ms);
      if (!k.magFault()) {
        Vec3 mE = k.q().rotate(mb);
        k.fuseYaw(wrapPi(k.yaw() - atan2f(mE.y, mE.x)), 0.1f);
      }
    }
    if (k.magFault()) magFault = true;
  }
  CHECK(magFault, "EKF: compass 70 deg off detected while GPS observes the heading");
  // numerical safety
  k.init(q, 0);
  k.predict(Vec3(NAN, 0, 0), Vec3(), 0.004f, ms);
  CHECK(k.pos().x == k.pos().x && k.status().filterResets >= 1, "EKF: NaN input resets the filter instead of spreading");
}

static void testAutotuneSteps() {
  Config c;
  configDefaults(c);
  c.at_axes = 1;
  c.at_steps = 2;  // feed-forward only
  AutoTune at;
  at.begin(c);
  CHECK(at.active() && at.step() == AutoTune::Step::FF, "Autotune: at_steps 2 starts with the feed-forward step");
  // a "plant" with 40 ms lag: the rate follows the command through a first-order lag
  float rate = 0, f0 = c.pid[0].f;
  float att[2] = {0, 0};
  for (int i = 0; i < 250 * 120 && !at.done(); i++) {
    int ax; float r = 0;
    float rr[3] = {rate, 0, 0};
    bool tw = at.update(c, rr, att, false, 0.004f, ax, r);
    float cmd = tw ? r : 0;
    float tau = 0.04f * 60.0f / fmaxf(c.pid[0].f, 20.0f);  // more FF -> faster
    rate += (cmd - rate) * 0.004f / fmaxf(tau, 0.005f);
  }
  CHECK(at.done() && c.pid[0].f > f0 && c.pid[0].p == 45.0f, "Autotune: a laggy response raises feed-forward, P untouched");
  Config f;
  configDefaults(f);
  f.at_steps = 8;
  AutoTune n;
  n.begin(f);
  float raw[3], filt[3], notch[3] = {220, 230, 0};
  int ax; float r;
  float rr[3] = {0, 0, 0};
  for (int i = 0; i < 6000; i++) {
    n.update(f, rr, att, false, 0.004f, ax, r);
    for (int a = 0; a < 3; a++) { raw[a] = 3.0f * sinf(i * 2.1f + a); filt[a] = 0.1f * sinf(i * 1.7f); }
    n.noiseSample(raw, filt, notch);
  }
  float g0 = f.gyro_lpf_hz;
  bool ch = n.filterResult(f);
  CHECK(ch && f.gyro_lpf_hz > g0 && f.dyn_notch_min < 225 && f.dyn_notch_max > 225,
        "Autotune filters: clean gyro opens the low-pass, dynamic notch range centred on the resonance");
}

// ---------------------------------------------------------------- SD card blackbox (v25)
static void testSdCard() {
  using namespace aqt;
  hal::host::reset();
  configDefaults(bus::config);
  Config& c = bus::config;
  c.sd_mode = 0;
  sdlog::begin(c);
  CHECK(!sdlog::enabled() && sdlog::info().state == sdlog::State::OFF, "SD: off by default");
  c.sd_mode = 3;
  sdlog::begin(c);
  uint32_t t = 0;
  sdlog::service(t);
  CHECK(sdlog::enabled() && sdlog::info().state == sdlog::State::READY && hal::host::sd.lastMode == 3,
        "SD: 4-bit card mounted by the sdcard task");
  // a flight at 500 Hz: one file, every byte the blackbox produced lands in it
  logging::Blackbox bb;
  bb.begin(3, hal::Port::SERIAL3, 500);
  VehicleStatus st{};
  auto fly = [&](int records, int serviceEvery) {
    st.armed = true;
    bus::status.publish(st);
    for (int k = 0; k < records; k++) {
      AttitudeState a{};
      a.rateDps[0] = (float)(k % 37);  // something that changes
      bus::attitude.publish(a);
      bb.update((uint32_t)k * 2000);
      if (serviceEvery && k % serviceEvery == 0) sdlog::service(t += 10);
    }
    st.armed = false;
    bus::status.publish(st);
    bb.update(0);
    for (int k = 0; k < 20; k++) sdlog::service(t += 10);
  };
  uint32_t before = bb.bytesWritten();
  fly(2500, 5);
  auto& f1 = hal::host::sd.files["/AQLOG/AQ00001.BBL"];
  CHECK(f1.size() == bb.bytesWritten() - before && f1.size() > 2000 && !memcmp(f1.data(), "H AllQuad blackbox", 18) &&
            sdlog::info().state == sdlog::State::READY && hal::host::sd.syncs >= 4,
        "SD: one file per flight, complete, synced every second, closed after disarm");
  // the card stalls for 4 s (no service at all): the queue holds the data, nothing is lost
  before = bb.bytesWritten();
  uint32_t drop0 = sdlog::info().dropped;
  fly(2000, 0);
  auto& f2 = hal::host::sd.files["/AQLOG/AQ00002.BBL"];
  CHECK(f2.size() == bb.bytesWritten() - before && sdlog::info().dropped == drop0, "SD: a 4 s card stall is absorbed by the queue");
  // queue overflow: whole frames are refused (the blackbox then restarts with a full record)
  sdlog::startFile();
  uint8_t big[4096];
  memset(big, 0x55, sizeof(big));
  int ok = 0;
  while (sdlog::push(big, sizeof(big)) && ok < 1000) ok++;
  uint32_t qs = sdlog::info().queueSize;
  CHECK(ok == (int)(qs / sizeof(big)) && sdlog::info().dropped == drop0 + sizeof(big), "SD: full queue refuses whole frames");
  for (int k = 0; k < 20; k++) sdlog::service(t += 10);
  CHECK(sdlog::push(big, 100), "SD: queue usable again once the card caught up");
  sdlog::endFile();
  for (int k = 0; k < 20; k++) sdlog::service(t += 10);
  CHECK(hal::host::sd.files["/AQLOG/AQ00003.BBL"].size() == qs + 100 && sdlog::info().state == sdlog::State::READY,
        "SD: queued data written and the file closed");
  // card pulled mid-flight: error, logging stops; the next flight starts a new file
  hal::host::sd.failAfter = hal::host::sd.appends + 1;
  fly(3000, 5);
  CHECK(strstr(sdlog::info().err, "write failed") && hal::host::sd.mounts >= 2, "SD: write failure reported, card mounted again");
  hal::host::sd.failAfter = -1;
  fly(600, 5);
  CHECK(hal::host::sd.files.count("/AQLOG/AQ00005.BBL") && sdlog::info().state == sdlog::State::READY && !sdlog::info().err[0],
        "SD: next flight logs to a new file after an error");
  // configurator: list, download over USB in pages, delete; armed / bad names refused
  {
    static comms::AqLink link;
    link = comms::AqLink();
    link.begin(hal::Port::CONSOLE);
    L = &link;
    std::string js;
    web::sdJson(sinkString, &js);
    CHECK(JsonCheck::valid(js) && js.find("\"AQ00002.BBL\"") != std::string::npos && js.find("\"state\":\"ready\"") != std::string::npos,
          "SD: status JSON with the file list");
    Resp d = fetchAll("/api/sdfile?n=AQ00002.BBL");
    CHECK(d.status == 200 && d.data.size() == f2.size() && !memcmp(d.data.data(), f2.data(), f2.size()), "SD: file download over USB in pages");
    CHECK(request("/api/sdfile?n=../CONFIG").status == 400, "SD: path names refused");
    char msg[80];
    CHECK(web::command("sd_rm_AQ00001.BBL", msg, sizeof(msg)) == 200 && !hal::host::sd.files.count("/AQLOG/AQ00001.BBL"),
          "SD: delete a log");
    st.armed = true;
    bus::status.publish(st);
    CHECK(request("/api/sdfile?n=AQ00002.BBL").status == 423, "SD: no download while armed");
    st.armed = false;
    bus::status.publish(st);
  }
  // nearly full card: the oldest logs make room (sd_keep 1), numbering continues
  hal::host::sd.files.clear();
  for (int k = 1; k <= 3; k++) hal::host::sd.files["/AQLOG/AQ0000" + std::to_string(k) + ".BBL"].assign(10u << 20, 0);
  hal::host::sd.capacity = 40ull << 20;  // 10 MB free < 16 MB
  sdlog::remount();
  sdlog::service(t += 10);
  CHECK(!hal::host::sd.files.count("/AQLOG/AQ00001.BBL") && hal::host::sd.files.count("/AQLOG/AQ00002.BBL") &&
            sdlog::info().state == sdlog::State::READY,
        "SD: oldest log deleted to keep 16 MB free");
  fly(300, 5);
  CHECK(hal::host::sd.files.count("/AQLOG/AQ00004.BBL"), "SD: numbering continues after the highest file");
  c.sd_keep = 0;
  sdlog::begin(c);
  hal::host::sd.capacity = 32ull << 20;
  sdlog::service(t += 10);
  CHECK(sdlog::info().state == sdlog::State::FULL, "SD: sd_keep 0 stops at a full card instead of deleting");
  // no card / SPI needs its pins
  hal::host::sd = hal::host::SdCard();
  hal::host::sd.inserted = false;
  c.sd_keep = 1;
  c.sd_mode = 1;
  sdlog::begin(c);
  sdlog::service(t += 10);
  CHECK(sdlog::info().state == sdlog::State::NO_CARD && sdlog::info().pin[1] != 255 && sdlog::info().pin[3] == 255,
        "SD: no card reported; SPI uses SCK / MOSI / MISO / CS only");
  hal::host::sd.inserted = true;
  sdlog::service(t += 3000);
  CHECK(sdlog::info().state == sdlog::State::READY, "SD: a card inserted later is found (retry every 3 s)");
  {  // ESC frames (0xA7) between the records: bidirectional DShot RPM, ~50 Hz, delta chain untouched
    hal::host::bidir = true;
    for (int i = 0; i < 4; i++) hal::host::erpm[i] = 7000.0f * (bus::config.motor_poles * 0.5f) * (1 + 0.1f * i);
    size_t n0 = hal::host::sd.files.size();
    fly(1000, 5);  // 2 s at 500 Hz
    hal::host::bidir = false;
    const auto& f = (--hal::host::sd.files.end())->second;
    int frames = 0, good = 0;
    for (size_t k = 0; k + sizeof(logging::EscRecord) <= f.size(); k++) {
      if (f[k] != 0xA7 || f[k + 1] != sizeof(logging::EscRecord)) continue;
      uint8_t x = 0;
      for (size_t j = 0; j < sizeof(logging::EscRecord) - 1; j++) x ^= f[k + j];
      if (x != f[k + sizeof(logging::EscRecord) - 1]) continue;
      logging::EscRecord e;
      memcpy(&e, &f[k], sizeof(e));
      frames++;
      if (e.rpm10[0] == 700 && e.rpm10[3] == 910 && e.tempC[0] == 255) good++;
    }
    CHECK(hal::host::sd.files.size() == n0 + 1 && frames >= 90 && frames <= 110 && good == frames,
          "blackbox: ESC frames with motor RPM at ~50 Hz");
  }
  c.sd_mode = 0;
  sdlog::begin(c);
  configDefaults(bus::config);
  hal::host::reset();
}

// "crsf-msp": the FC side of MSP over CRSF on stdin / stdout, for the radio Lua script preview
// (tools/edgetx/preview.py): each input line = one CRSF frame in hex from the radio; the answer frames
// (hex, one per line) follow, then a line "."
static int crsfMspServe() {
  hal::host::reset();
  configDefaults(bus::config);
  bus::status.publish(VehicleStatus{});
  static drv::RxParser rx;
  rx.begin(RxProtocol::CRSF);
  static comms::MspLink link;
  static comms::CrsfMsp cm;
  link.begin(hal::Port::SERIAL1);
  link.setReplyHook([](void* ctx, const comms::MspFrame& q, const uint8_t* p, uint16_t n, bool e) {
    static_cast<comms::CrsfMsp*>(ctx)->respond(q, p, n, e); }, &cm);
  char line[512];
  uint32_t t = 1000;
  while (fgets(line, sizeof(line), stdin)) {
    t += 5;
    for (char* c = line; c[0] && c[1] && c[0] != '\n'; c += 2) {
      unsigned v;
      if (sscanf(c, "%2x", &v) != 1) break;
      rx.feed((uint8_t)v, t);
      const uint8_t* ch; size_t cn; uint8_t org; comms::MspFrame mf;
      if (rx.takeMsp(ch, cn, org) && cm.feed(ch, cn, org, mf)) link.handle(mf, t);
    }
    uint8_t fr[64];
    size_t n;
    while ((n = cm.nextFrame(fr))) {
      for (size_t i = 0; i < n; i++) printf("%02x", fr[i]);
      printf("\n");
    }
    printf(".\n");
    fflush(stdout);
  }
  return 0;
}

int main(int argc, char** argv) {
  if (argc == 3 && !strcmp(argv[1], "web-dump")) return webDump(argv[2]);
  if (argc == 2 && !strcmp(argv[1], "crsf-msp")) return crsfMspServe();
  if (argc == 4 && !strcmp(argv[1], "mavlink-io")) return mavlinkIo(argv[2], argv[3]);
  testRx();
  testGps();
  testSerialSensors();
  testMsp();
  testDynamicNotch();
  testKalman();
  testImuDelta();
  testAltitudeHelpers();
  testSticksMenu();
  testParams();
  testModeRanges();
  testDeferredSave();
  testAlignMixer();
  testBatteryFence();
  testOsdBlackbox();
  testJsonNumbers();
  testCli();
  testCrsfMsp();
  testIcm45686();
  testQmc5883p();
  testWebApi();
  testWebAuth();
  testBackupRestore();
  testAqLink();
  testConfigMigration();
  testCliMotorSensors();
  testVtx();
  testLeds();
  testFourWay();
  testConfigMigrationV16();
  testConfigMigrationV17();
  testConfigMigrationV18();
  testConfigMigrationV19();
  testDshotEdt();
  testConfigMigrationV20();
  testConfigMigrationV21();
  testConfigMigrationV25();
  testConfigMigrationV26();
  testConfigMigrationV27();
  testConfigMigrationV28();
  testConfigMigrationV29();
  testGyroTap();
  testBeginnerLimits();
  testMavlinkLowRate();
  testPinMap();
  testBatteryPins();
  testFlowModuleProtocols();
  testFeedForward();
  testFlightFeel();
  testEllipsoidFit();
  testPolygonFenceAndPath();
  testEscCommands();
  testDshotBidir();
  testVtxTable();
  testEkf();
  testAutotuneSteps();
  testSdCard();
  testEeAb();
  {  // RPM samples: a lone glitch is held back, a real change goes through one sample later, loss at once
    fc::RpmGate g;
    float a = g.update(200), b = g.update(205), c = g.update(900), d = g.update(210), e = g.update(420), f = g.update(430);
    float z = g.update(0), y = g.update(250);
    CHECK(a == 200 && b == 205 && c == 205 && d == 210 && e == 210 && f == 430 && z == 0 && y == 250 && g.rejected == 2,
          "RPM gate: decode glitch rejected, step confirmed by the next sample, telemetry loss taken at once");
  }
  {  // a stamp 1 ms "in the future" (published by a higher-priority task after this one read the clock)
    CHECK(fc::ageMs(1000, 1001) == 0 && fc::ageMs(1000, 900) == 100 && fc::ageMs(5, 0xFFFFFFFEu) == 7,
          "ageMs: newer stamps are age 0 (no false RC loss), wraps at 2^32");
  }
  printf("\n%d/%d tests passed\n", total - failures, total);
  return failures ? 1 : 0;
}
