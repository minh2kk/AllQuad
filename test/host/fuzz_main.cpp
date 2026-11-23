// AllQuad - protocol fuzzer (host build, run under AddressSanitizer + UBSan: tools/fuzz.sh)
//
// Every byte the flight controller takes from the outside world goes through one of these parsers:
// receiver (CRSF / SBUS / iBUS + CRSF MSP), MSP (console, sensor port, Micolink / MAVLink sniffing),
// MAVLink, GPS (UBX / NMEA), TFmini, ESC telemetry, the configurator link (AQ frames -> web API),
// the CLI, the ESC 4-way passthrough, stored settings (migration). A malformed or hostile frame must
// never crash, hang or write outside a buffer, and must never leave a setting outside its limits.
//
// Inputs: valid frames built with the firmware's own packers, then mutated (bit flips, wrong lengths,
// truncation, splices, random bytes). Deterministic (seeded), so a failure reproduces exactly.
//   build/fuzz [iterations per target, default 20000] [seed] [target name]
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <chrono>
#include <string>
#include <vector>

#include "../../src/bus/bus.h"
#include "../../src/comms/aqlink.h"
#include "../../src/comms/cli.h"
#include "../../src/comms/crsf_msp.h"
#include "../../src/comms/fourway.h"
#include "../../src/comms/hitl.h"
#include "../../src/comms/mavlink.h"
#include "../../src/comms/msp.h"
#include "../../src/config/config.h"
#include "../../src/gps/gps_driver.h"
#include "../../src/rx/rx_parser.h"
#include "../../src/sensors/serial_sensors.h"

#include "hal_host.h"

using Bytes = std::vector<uint8_t>;
using namespace fc;
using comms::MavFrame;
using comms::MspFrame;
using comms::crc8DvbS2;
using comms::mavPack;
using comms::mspPack;

// ---------------------------------------------------------------- random + mutation
static uint64_t rs = 0x9E3779B97F4A7C15ull;
static uint32_t rnd() { rs ^= rs << 13; rs ^= rs >> 7; rs ^= rs << 17; return (uint32_t)(rs >> 11); }
static uint32_t rnd(uint32_t n) { return n ? rnd() % n : 0; }
static const uint8_t kInteresting[] = {0x00, 0x01, 0x7F, 0x80, 0xFE, 0xFF, 0x20, 0x40, 0x0A, 0x0D, '$', 'M', 'X', '<', '>', 0xC8, 0xFD, 0xFE, 0xA7, 0xB5, 0x62};

static Bytes randomBytes(size_t maxLen) {
  Bytes b(rnd((uint32_t)maxLen) + 1);
  for (auto& x : b) x = rnd(4) ? (uint8_t)rnd(256) : kInteresting[rnd(sizeof(kInteresting))];
  return b;
}
static Bytes mutate(Bytes b, const std::vector<Bytes>& seeds) {
  int ops = 1 + (int)rnd(4);
  for (int k = 0; k < ops; k++) {
    switch (rnd(9)) {
      case 0: if (!b.empty()) b[rnd((uint32_t)b.size())] ^= (uint8_t)(1u << rnd(8)); break;
      case 1: if (!b.empty()) b[rnd((uint32_t)b.size())] = (uint8_t)rnd(256); break;
      case 2: if (!b.empty()) b[rnd((uint32_t)b.size())] = kInteresting[rnd(sizeof(kInteresting))]; break;
      case 3: b.insert(b.begin() + rnd((uint32_t)b.size() + 1), (uint8_t)rnd(256)); break;
      case 4: if (!b.empty()) b.erase(b.begin() + rnd((uint32_t)b.size())); break;
      case 5: if (b.size() > 2) b.resize(rnd((uint32_t)b.size())); break;  // cut short
      case 6: { const Bytes& s = seeds[rnd((uint32_t)seeds.size())]; b.insert(b.end(), s.begin(), s.end()); break; }  // two frames
      case 7: if (b.size() > 4) { size_t a = rnd((uint32_t)b.size()), n = 1 + rnd(8); Bytes d(b.begin() + a, b.begin() + std::min(b.size(), a + n)); b.insert(b.begin() + rnd((uint32_t)b.size()), d.begin(), d.end()); } break;
      case 8: { Bytes r = randomBytes(16); b.insert(b.begin() + rnd((uint32_t)b.size() + 1), r.begin(), r.end()); break; }
    }
  }
  if (b.size() > 4096) b.resize(4096);
  return b;
}
static Bytes pickInput(const std::vector<Bytes>& seeds) {
  uint32_t r = rnd(10);
  if (r == 0) return randomBytes(300);
  if (r == 1) return seeds[rnd((uint32_t)seeds.size())];  // valid frame: keeps deep paths reachable
  return mutate(seeds[rnd((uint32_t)seeds.size())], seeds);
}

// ---------------------------------------------------------------- invariants
static int failures = 0;
static uint32_t nowMs = 1000;
static const char* curTarget = "";
// every parameter inside its limits, no NaN: a handler that writes the settings directly (MSP, MAVLink,
// web API) must clamp like the CLI does
static void checkParams() {
  for (int i = 0; i < paramCount(); i++) {
    const ParamInfo& p = paramInfo(i);
    float v = paramGet(bus::config, i);
    if (v != v || v < p.min - 1e-3f * fabsf(p.min) - 1e-6f || v > p.max + 1e-3f * fabsf(p.max) + 1e-6f) {
      printf("[FAIL] %s: parameter %s = %g outside %g..%g\n", curTarget, p.name, v, p.min, p.max);
      failures++;
      configDefaults(bus::config);
      return;
    }
  }
}
static void drainOutputs() {
  for (int p = 0; p < (int)hal::Port::COUNT; p++) {
    hal::host::out((hal::Port)p).clear();
    hal::host::in((hal::Port)p).clear();
  }
}
static void freshState() {
  hal::host::reset();
  configDefaults(bus::config);
}

// ---------------------------------------------------------------- seed builders
static uint8_t crsfCrc(const uint8_t* d, size_t n) { uint8_t c = 0; for (size_t i = 0; i < n; i++) c = crc8DvbS2(c, d[i]); return c; }
static Bytes crsfFrame(uint8_t type, const Bytes& payload, uint8_t addr = 0xC8) {
  Bytes f = {addr, (uint8_t)(payload.size() + 2), type};
  f.insert(f.end(), payload.begin(), payload.end());
  f.push_back(crsfCrc(f.data() + 2, f.size() - 2));
  return f;
}
static Bytes crsfChannels(uint16_t v) {
  Bytes p(22, 0);
  uint32_t acc = 0; int bits = 0; size_t o = 0;
  for (int ch = 0; ch < 16; ch++) {
    acc |= (uint32_t)(v + ch * 37 % 600) << bits; bits += 11;
    while (bits >= 8) { p[o++] = (uint8_t)acc; acc >>= 8; bits -= 8; }
  }
  return crsfFrame(0x16, p);
}
static Bytes mspFrame(bool v2, char dir, uint16_t cmd, const Bytes& payload) {
  uint8_t buf[600];
  size_t n = mspPack(buf, v2, dir, cmd, payload.data(), (uint16_t)payload.size());
  return Bytes(buf, buf + n);
}
static Bytes mavFrame(uint32_t id, const Bytes& payload) {
  uint8_t buf[300];
  size_t n = mavPack(buf, id, payload.data(), (uint8_t)std::min<size_t>(payload.size(), 255), 255, 190, (uint8_t)rnd(256));
  return Bytes(buf, buf + n);
}
static Bytes ubx(uint8_t cls, uint8_t id, const Bytes& p) {
  Bytes f = {0xB5, 0x62, cls, id, (uint8_t)p.size(), (uint8_t)(p.size() >> 8)};
  f.insert(f.end(), p.begin(), p.end());
  uint8_t a = 0, b = 0;
  for (size_t i = 2; i < f.size(); i++) { a += f[i]; b += a; }
  f.push_back(a); f.push_back(b);
  return f;
}
static Bytes nmea(const char* body) {
  uint8_t x = 0;
  for (const char* p = body; *p; p++) x ^= (uint8_t)*p;
  char s[200];
  snprintf(s, sizeof(s), "$%s*%02X\r\n", body, x);
  return Bytes(s, s + strlen(s));
}
static Bytes aqFrame(uint8_t seq, const std::string& path, const std::string& body) {
  Bytes pl(path.begin(), path.end());
  pl.push_back(0);
  pl.insert(pl.end(), body.begin(), body.end());
  Bytes f = {0xA7, 0x51, seq, (uint8_t)pl.size(), (uint8_t)(pl.size() >> 8)};
  f.insert(f.end(), pl.begin(), pl.end());
  uint16_t c = comms::AqLink::crc16(f.data() + 2, f.size() - 2);
  f.push_back((uint8_t)c); f.push_back((uint8_t)(c >> 8));
  return f;
}
static Bytes randPayload(size_t maxLen) { Bytes p(rnd((uint32_t)maxLen + 1)); for (auto& x : p) x = (uint8_t)rnd(256); return p; }
static std::string randParamName() {
  if (rnd(5)) return paramInfo((int)rnd((uint32_t)paramCount())).name;
  std::string s;
  for (uint32_t i = 0, n = 1 + rnd(24); i < n; i++) s += (char)(rnd(3) ? 'a' + rnd(26) : rnd(256));
  return s;
}
static std::string randNumber() {
  static const char* const v[] = {"0", "-1", "1", "65535", "65536", "4294967295", "4294967296", "-2147483649", "1e38", "-1e38", "nan", "inf",
                                  "-inf", "0.0001", "3.4028235e38", "1e-45", "999999999999999999999", "0x10", "", "-0", "12abc", "1,5"};
  if (rnd(3)) return std::to_string((int)rnd(5000) - 1000);
  return v[rnd(sizeof(v) / sizeof(v[0]))];
}

// ---------------------------------------------------------------- targets
using Target = void (*)(int iters);

static void fuzzRx(int iters) {
  std::vector<Bytes> seeds = {crsfChannels(992), crsfChannels(172), crsfChannels(1811),
                              crsfFrame(0x14, {50, 60, 100, 5, 0, 2, 1, 40, 90, 3}),             // link statistics
                              crsfFrame(0x7A, {0xC8, 0xEA, 0x30, 0x00, 0x70, 0x00}, 0xC8),        // MSP request (ping)
                              crsfFrame(0x7C, {0xC8, 0xEA, 0x31, 0x02, 0x64, 0x00, 0x01, 0x02}, 0xC8),  // MSP write
                              crsfFrame(0x28, {0xC8, 0xEA}), crsfFrame(0x2C, {0xC8, 0xEA, 0x00, 0x00})};
  Bytes sbus(25, 0); sbus[0] = 0x0F; seeds.push_back(sbus);
  Bytes ibus(32, 0); ibus[0] = 0x20; ibus[1] = 0x40;
  { uint16_t s = 0xFFFF; for (int i = 0; i < 30; i++) s -= ibus[i]; ibus[30] = (uint8_t)s; ibus[31] = (uint8_t)(s >> 8); }
  seeds.push_back(ibus);
  const RxProtocol protos[] = {RxProtocol::CRSF, RxProtocol::SBUS, RxProtocol::IBUS};
  drv::RxParser rx;
  comms::CrsfMsp cm;
  int frames = 0, msp = 0;
  for (int it = 0; it < iters; it++) {
    if (it % 2000 == 0) rx.begin(protos[(it / 2000) % 3]);
    Bytes in = pickInput(seeds);
    for (uint8_t b : in) {
      if (rx.feed(b, nowMs)) frames++;
      const uint8_t* chunk; size_t n; uint8_t origin;
      if (rx.takeMsp(chunk, n, origin)) {
        MspFrame f;
        if (cm.feed(chunk, n, origin, f)) { msp++; uint8_t resp[300]; cm.respond(f, resp, (uint16_t)rnd(300), rnd(2)); uint8_t o[80]; while (cm.nextFrame(o)) {} }
      }
    }
    nowMs += 1 + rnd(20);
    if (it % 50 == 0) {  // telemetry with odd values
      drv::RxTelemetry t{};
      float odd[] = {0, -1, 1e30f, -1e30f, NAN, INFINITY, 3.3f};
      t.pitchRad = odd[rnd(7)]; t.rollRad = odd[rnd(7)]; t.yawRad = odd[rnd(7)];
      t.vbat = odd[rnd(7)]; t.current = odd[rnd(7)]; t.mah = odd[rnd(7)]; t.vario = odd[rnd(7)];
      t.groundSpeed = odd[rnd(7)]; t.courseDeg = odd[rnd(7)]; t.altitude = odd[rnd(7)];
      t.lat = rnd(2) ? 1e9 : -91; t.lon = rnd(2) ? 500 : NAN; t.sats = (uint8_t)rnd(256); t.gpsValid = rnd(2);
      t.batteryPct = (uint8_t)rnd(256);
      static char mode[80];
      memset(mode, 'A' + rnd(26), sizeof(mode) - 1); mode[rnd(sizeof(mode))] = 0;
      t.mode = mode;
      uint8_t out[128];
      for (int k = 0; k < 8; k++) { size_t n = rx.buildTelemetry(t, out); if (n > 64) { printf("[FAIL] rx: telemetry frame %zu bytes\n", n); failures++; } }
    }
  }
  printf("  rx: %d channel frames, %d MSP-over-CRSF requests\n", frames, msp);
}

static void fuzzMsp(int iters) {
  std::vector<Bytes> seeds;
  const uint16_t cmds[] = {1, 2, 3, 4, 5, 10, 36, 88, 89, 90, 101, 102, 104, 105, 106, 107, 108, 109, 110, 112, 124, 130, 131, 150, 160, 182, 188, 189,
                           200, 202, 203, 204, 205, 210, 211, 214, 217, 239, 245, 250, 0x4151, 0x4152, 0x4153, 0x1F01, 0x1F02};
  for (uint16_t c : cmds) {
    seeds.push_back(mspFrame(c > 255, '<', c, {}));
    seeds.push_back(mspFrame(c > 255, '<', c, randPayload(40)));
    seeds.push_back(mspFrame(true, '<', c, randPayload(200)));
  }
  { std::string n = "roll_p"; Bytes p(n.begin(), n.end()); p.push_back(0); seeds.push_back(mspFrame(true, '<', 0x4152, p)); }
  comms::MspLink console, sensor;
  console.begin(hal::Port::CONSOLE);
  sensor.begin(hal::Port::SERIAL1);
  comms::MspParser parser;
  int frames = 0;
  for (int it = 0; it < iters; it++) {
    Bytes in = pickInput(seeds);
    comms::MspLink& l = rnd(2) ? console : sensor;
    for (uint8_t b : in) { MspFrame f; if (parser.feed(b, f)) { frames++; l.handle(f, nowMs); } }
    // the sensor port also sniffs Micolink / MAVLink from the same bytes
    auto& q = hal::host::in(hal::Port::SERIAL1);
    for (uint8_t b : in) q.push_back(b);
    sensor.update(nowMs);
    console.update(nowMs);
    nowMs += 1 + rnd(30);
    if (it % 200 == 0) { checkParams(); drainOutputs(); }
  }
  printf("  msp: %d frames handled\n", frames);
}

// hardware in the loop: the first frame through the console's MSP, then the io task's own reader
// (hitl::poll) with whatever the PC sends; the replacement drivers are read like the fast loop would
static void fuzzHitl(int iters) {
  std::vector<Bytes> seeds;
  for (int i = 0; i < 6; i++) {
    Bytes p = randPayload(hitl::FRAME_LEN);
    p.resize(hitl::FRAME_LEN);
    p[44] = (uint8_t)(i == 0 ? 0 : rnd(16));  // flags without EXIT (that one reboots)
    seeds.push_back(mspFrame(true, '<', hitl::MSP_CMD, p));
  }
  seeds.push_back(mspFrame(true, '<', hitl::MSP_CMD, randPayload(20)));  // short frame
  comms::MspLink console;
  console.begin(hal::Port::CONSOLE);
  comms::MspParser parser;
  int frames = 0;
  for (int it = 0; it < iters; it++) {
    Bytes in = pickInput(seeds);
    for (size_t k = 0; k + 52 < in.size(); k++)
      if (in[k] == '$' && in[k + 1] == 'X') in[k + 52] &= 0x7F;  // keep the EXIT flag off (reboot)
    if (!hitl::active()) {
      for (uint8_t b : in) { MspFrame f; if (parser.feed(b, f)) { frames++; console.handle(f, nowMs); } }
    } else {
      auto& q = hal::host::in(hal::Port::CONSOLE);
      for (uint8_t b : in) q.push_back(b);
      hitl::poll(nowMs);
    }
    fc::Vec3 g, a, m;
    float pa, t, v, i;
    hitl::imu()->read(g, a);
    hitl::baro()->update(pa, t);
    hitl::mag()->read(m);
    hitl::battery(v, i);
    nowMs += 1 + rnd(3);
    if (it % 200 == 0) { checkParams(); drainOutputs(); }
  }
  printf("  hitl: %d console frames, active %d, %u frames applied\n", frames, hitl::active(), (unsigned)hitl::stats().frames);
}

static void fuzzMicolink(int iters) {
  std::vector<Bytes> seeds;
  for (int i = 0; i < 6; i++) {
    uint8_t f[27] = {0xEF, 0x01, 0x00, 0x51, (uint8_t)i, 20};
    for (int k = 6; k < 26; k++) f[k] = (uint8_t)rnd(256);
    uint8_t s = 0; for (int k = 0; k < 26; k++) s = (uint8_t)(s + f[k]);
    f[26] = s;
    seeds.emplace_back(f, f + 27);
  }
  seeds.push_back({0xFD, 0x1C, 0x00, 0x00, 0x01, 0x01, 0x64, 0x00, 0x00});  // MAVLink v2 header shape
  seeds.push_back({0xFE, 0x1C, 0x01, 0x01, 0x01, 0x84});                    // MAVLink v1 DISTANCE_SENSOR shape
  comms::MspLink sensor;
  sensor.begin(hal::Port::SERIAL1);
  for (int it = 0; it < iters; it++) {
    Bytes in = pickInput(seeds);
    auto& q = hal::host::in(hal::Port::SERIAL1);
    for (uint8_t b : in) q.push_back(b);
    sensor.update(nowMs);
    nowMs += 1 + rnd(10);
    if (it % 500 == 0) drainOutputs();
  }
  RangeSample r = bus::range.get();
  if (r.valid && !(r.distM >= 0 && r.distM < 1000)) { printf("[FAIL] micolink: range %g m\n", r.distM); failures++; }
  printf("  micolink: %u frames decoded, %u MAVLink frames recognised\n", (unsigned)bus::sensorLink.mico, (unsigned)bus::sensorLink.mav);
}

static void fuzzMavlink(int iters) {
  const uint32_t ids[] = {0, 1, 2, 11, 20, 21, 22, 23, 24, 30, 33, 39, 40, 41, 42, 43, 44, 45, 46, 47, 51, 65, 73, 74, 75, 76, 77, 84, 86, 148, 242, 253};
  std::vector<Bytes> seeds;
  for (uint32_t id : ids) { seeds.push_back(mavFrame(id, randPayload(60))); seeds.push_back(mavFrame(id, {})); }
  auto paramSet = [](const std::string& name, float v) {
    Bytes p(23, 0);
    memcpy(p.data(), &v, 4); p[4] = 1; p[5] = 1;
    memcpy(p.data() + 6, name.data(), std::min<size_t>(16, name.size()));
    p[22] = 9;
    return mavFrame(23, p);
  };
  auto cmdLong = [](uint16_t cmd, float p1, float p2) {
    Bytes p(33, 0);
    memcpy(p.data(), &p1, 4); memcpy(p.data() + 4, &p2, 4);
    memcpy(p.data() + 28, &cmd, 2); p[30] = 1; p[31] = 1;
    return mavFrame(76, p);
  };
  for (int i = 0; i < 30; i++) seeds.push_back(paramSet(paramInfo((int)rnd((uint32_t)paramCount())).name, (float)rnd(100000) - 50000));
  for (uint16_t c : {400, 20, 192, 22, 21, 300, 176, 241, 245, 512, 520, 511, 16, 178, 183, 200}) seeds.push_back(cmdLong(c, (float)rnd(3), (float)rnd(30)));
  comms::MavlinkLink link;
  link.begin(hal::Port::SERIAL2);
  comms::MavParser parser;
  int frames = 0;
  for (int it = 0; it < iters; it++) {
    Bytes in;
    if (rnd(8) == 0) in = paramSet(randParamName(), rnd(2) ? (float)(rnd(2000000) - 1000000) : (rnd(2) ? NAN : INFINITY));
    else in = pickInput(seeds);
    for (uint8_t b : in) { MavFrame f; if (parser.feed(b, f)) { frames++; link.handle(f, nowMs); } }
    link.update(nowMs, false);
    nowMs += 1 + rnd(50);
    if (it % 200 == 0) { checkParams(); drainOutputs(); }
  }
  printf("  mavlink: %d frames handled, %u CRC errors\n", frames, (unsigned)parser.errors());
}

static void fuzzGps(int iters) {
  Bytes pvt(92, 0);
  pvt[20] = 3; pvt[21] = 1; pvt[23] = 12;  // 3D fix, 12 satellites
  std::vector<Bytes> seeds = {ubx(0x01, 0x07, pvt), ubx(0x01, 0x07, randPayload(92)), ubx(0x05, 0x01, {0x06, 0x8A}), ubx(0x01, 0x35, randPayload(60)),
                              nmea("GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,"),
                              nmea("GNRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W"),
                              nmea("GPGSA,A,3,04,05,,09,12,,,24,,,,,2.5,1.3,2.1"), nmea("GNGGA,,,,,,0,00,99.99,,,,,,"),
                              nmea("GPGGA,999999,9999.999,X,99999.999,Y,9,99,999.9,99999.9,M,,M,,")};
  drv::GpsDriver gps;
  gps.attach(hal::Port::SERIAL2);
  int sol = 0;
  for (int it = 0; it < iters; it++) {
    Bytes in = pickInput(seeds);
    for (uint8_t b : in) if (gps.feed(b)) sol++;
    const GpsData& d = gps.data();
    if (d.fix3d && (!(d.lat == d.lat) || fabs(d.lat) > 90.0 || fabs(d.lon) > 180.0)) {
      printf("[FAIL] gps: fix with lat %g lon %g\n", d.lat, d.lon); failures++; break;
    }
  }
  printf("  gps: %d solutions (%s)\n", sol, gps.protocol());
}

static void fuzzSerialSensors(int iters) {
  std::vector<Bytes> seeds = {{0x59, 0x59, 0x10, 0x01, 0x20, 0x00, 0x00, 0x00, 0xDB}, {0x59, 0x59}};
  Bytes esc(10, 0); esc[0] = 40; esc[1] = 0x0F; esc[2] = 0xA0; seeds.push_back(esc);
  drv::TfminiParser tf;
  drv::EscTelemetryParser et;
  int a = 0, b2 = 0;
  for (int it = 0; it < iters; it++) {
    Bytes in = pickInput(seeds);
    for (uint8_t b : in) { if (tf.feed(b)) a++; if (et.feed(b)) b2++; }
  }
  printf("  tfmini / ESC telemetry: %d / %d frames\n", a, b2);
}

static void fuzzAqLink(int iters) {
  const char* routes[] = {"/api/hello", "/api/status", "/api/params", "/api/params?o=0", "/api/params?o=99999999", "/api/sensors", "/api/set",
                          "/api/save", "/api/cmd?c=calib_acc", "/api/cmd?c=reboot", "/api/cmd?c=", "/api/cmd?c=zzz", "/api/motor?m=1&v=5",
                          "/api/motor?m=99&v=-1", "/api/mission", "/api/mission?o=5", "/api/backup", "/api/backup?o=100", "/api/restore?o=0&n=10",
                          "/api/restore?o=4000000000&n=1", "/api/blackbox?o=0", "/api/blackbox?o=-1", "/api/cli", "/api/sd", "/api/sdfile?f=../../x",
                          "/api/wifi", "/api/login", "/api/logout", "/api/ota", "/api/unknown", "/api/set?"};
  auto body = []() -> std::string {
    switch (rnd(6)) {
      case 0: return randParamName() + "=" + randNumber();
      case 1: { std::string s; for (int i = 0; i < 1 + (int)rnd(12); i++) s += randParamName() + "=" + randNumber() + "&"; return s; }
      case 2: return "set " + randParamName() + " " + randNumber() + "\n";
      case 3: return "{\"wp\":[[" + randNumber() + "," + randNumber() + "," + randNumber() + "]]}";
      case 4: return std::string(rnd(1200), (char)('A' + rnd(26)));
      default: { Bytes r = randomBytes(200); return std::string(r.begin(), r.end()); }
    }
  };
  std::vector<Bytes> seeds;
  for (const char* r : routes) seeds.push_back(aqFrame((uint8_t)rnd(256), r, body()));
  comms::AqLink link;
  link.begin(hal::Port::CONSOLE);
  for (int it = 0; it < iters; it++) {
    Bytes in = rnd(3) ? aqFrame((uint8_t)rnd(256), routes[rnd(sizeof(routes) / sizeof(routes[0]))], body()) : pickInput(seeds);
    for (uint8_t b : in) link.feed(b, nowMs);
    for (int k = 0; k < 4; k++) link.update(nowMs);
    nowMs += 5 + rnd(100);
    if (it % 100 == 0) { checkParams(); drainOutputs(); }
  }
  printf("  aq link: %u frames, %u CRC errors\n", (unsigned)link.frames(), (unsigned)link.crcErrors());
}

static void fuzzCli(int iters) {
  const char* cmds[] = {"help", "status", "set", "get", "diff", "diff all", "dump", "defaults", "save", "version", "selftest", "pins", "pins uart",
                        "tasks", "perf", "esc", "sd", "sd ls", "sd rm", "mission", "mission clear", "mission add", "fence", "fence add", "fence clear",
                        "logic", "log", "gyrocap", "goto", "rally", "rally add", "crash", "crash clear", "calib", "motor", "beep", "vtx", "wifi", "led",
                        "serial", "profile", "rateprofile", "autotune", "mag", "acc", "baro", "gps", "osd", "bootloader", "dfu", "reboot", "exit", "#", ""};
  comms::Cli cli;
  static comms::SystemInfo info = {"SIM-IMU", "SIM-BARO", "SIM-MAG", "fuzz"};
  cli.begin(hal::Port::CONSOLE, &info);
  for (int it = 0; it < iters; it++) {
    std::string line = cmds[rnd(sizeof(cmds) / sizeof(cmds[0]))];
    int nargs = (int)rnd(5);
    for (int a = 0; a < nargs; a++) {
      line += ' ';
      switch (rnd(5)) {
        case 0: line += randParamName(); break;
        case 1: line += randNumber(); break;
        case 2: line += "="; break;
        case 3: line += std::string(rnd(400), 'x'); break;
        default: { Bytes r = randomBytes(30); for (uint8_t c : r) if (c != '\n' && c != '\r' && c) line += (char)c; }
      }
    }
    if (rnd(4) == 0) cli.run(line.c_str());
    else { for (char c : line) cli.feed((uint8_t)c, nowMs, false); cli.feed('\n', nowMs, false); }
    nowMs += 3;
    if (it % 100 == 0) { checkParams(); drainOutputs(); }
  }
  printf("  cli: %d command lines\n", iters);
}

static void fuzzFourWay(int iters) {
  comms::FourWay fw;
  fw.begin(hal::Port::CONSOLE);
  std::vector<Bytes> seeds = {{0x2F, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00}, {0x2F, 0x37, 0x00, 0x00, 0x01, 0x00}, {0x2F, 0x3A, 0x10, 0x00, 0x04, 1, 2, 3, 4}};
  for (int it = 0; it < iters; it++) {
    if (!fw.active()) fw.start(nowMs);
    Bytes in = pickInput(seeds);
    for (uint8_t b : in) fw.feed(b, nowMs);
    fw.update(nowMs);
    nowMs += 1 + rnd(40);
    if (it % 500 == 0) drainOutputs();
  }
  fw.stop();
  printf("  4-way: %d inputs\n", iters);
}

// stored settings from flash: any bytes, with a valid CRC, any version -> migrate must not crash
static void fuzzConfig(int iters) {
  int migrated = 0;
  for (int it = 0; it < iters; it++) {
    Config c;
    configDefaults(c);
    uint8_t* p = (uint8_t*)&c;
    if (rnd(2)) for (size_t i = 0; i < sizeof(c); i++) p[i] = (uint8_t)rnd(256);
    else for (int k = 0; k < 20; k++) p[rnd(sizeof(c))] = (uint8_t)rnd(256);
    c.version = (uint16_t)(rnd(3) ? 15 + rnd(CONFIG_VERSION - 15 + 2) : rnd(65536));
    c.size = (uint16_t)(rnd(3) ? 64 + rnd(sizeof(c) - 64 + 8) : rnd(65536));
    if (c.size >= 8 && c.size <= sizeof(c)) { uint32_t crc = crc32(&c, c.size - 4); memcpy(p + c.size - 4, &crc, 4); }
    if (configMigrate(c)) {
      migrated++;
      if (!configValid(c)) { printf("[FAIL] config: migrated copy not valid\n"); failures++; break; }
      int fixed = configClamp(c);
      (void)fixed;
      bus::config = c;
      curTarget = "config (after clamp)";
      checkParams();
    }
  }
  printf("  config: %d of %d stored images migrated (and clamped)\n", migrated, iters);
}

int main(int argc, char** argv) {
  int iters = argc > 1 ? atoi(argv[1]) : 20000;
  if (argc > 2) rs ^= strtoull(argv[2], nullptr, 0) * 0x2545F4914F6CDD1Dull;
  const char* only = argc > 3 ? argv[3] : nullptr;
  struct { const char* name; Target fn; int scale; } targets[] = {
      {"rx", fuzzRx, 1}, {"msp", fuzzMsp, 1}, {"micolink", fuzzMicolink, 1}, {"mavlink", fuzzMavlink, 1}, {"gps", fuzzGps, 2},
      {"sensors", fuzzSerialSensors, 2}, {"aqlink", fuzzAqLink, 1}, {"cli", fuzzCli, 1}, {"fourway", fuzzFourWay, 1}, {"config", fuzzConfig, 2},
      {"hitl", fuzzHitl, 1}};
  for (auto& t : targets) {
    if (only && strcmp(only, t.name)) continue;
    freshState();
    curTarget = t.name;
    auto t0 = std::chrono::steady_clock::now();
    int f0 = failures;
    t.fn(iters * t.scale);
    checkParams();
    double s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    printf("[%s] %-9s %d inputs, %.1f s\n", failures == f0 ? "PASS" : "FAIL", t.name, iters * t.scale, s);
  }
  printf("\n%s\n", failures ? "FUZZ FAILURES" : "fuzz: no crash, no sanitizer error, all settings within limits");
  return failures ? 1 : 0;
}
