#include "../common/size_opt.h"  // -Os on ESP32 (not flight critical)
#include "gps_driver.h"

#include <stdlib.h>
#include <string.h>


namespace drv {

static inline uint32_t u32le(const uint8_t* p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static inline int32_t i32le(const uint8_t* p) { return (int32_t)u32le(p); }

void GpsDriver::sendUbx(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t len) {
  uint8_t hdr[6] = {0xB5, 0x62, cls, id, (uint8_t)len, (uint8_t)(len >> 8)};
  uint8_t a = 0, b = 0;
  for (int i = 2; i < 6; i++) { a += hdr[i]; b += a; }
  for (uint16_t i = 0; i < len; i++) { a += payload[i]; b += a; }
  uint8_t ck[2] = {a, b};
  hal::portWrite(port_, hdr, 6);
  if (len) hal::portWrite(port_, payload, len);
  hal::portWrite(port_, ck, 2);
}

void GpsDriver::configure() {
  // --- u-blox M8 and older (legacy CFG messages) ---
  // CFG-PRT: UART1, 8N1, 115200, in UBX+NMEA, out UBX
  uint8_t prt[20] = {0x01, 0, 0, 0, 0xD0, 0x08, 0, 0, 0x00, 0xC2, 0x01, 0, 0x03, 0, 0x01, 0, 0, 0, 0, 0};
  sendUbx(0x06, 0x00, prt, sizeof(prt));
  // --- u-blox M9/M10 (CFG-VALSET, RAM + BBR) ---
  uint8_t val[] = {
      0x00, 0x03, 0x00, 0x00,                          // version, layers RAM|BBR, reserved
      0x01, 0x00, 0x52, 0x40, 0x00, 0xC2, 0x01, 0x00,  // CFG-UART1-BAUDRATE = 115200
      0x02, 0x00, 0x74, 0x10, 0x00,                    // CFG-UART1OUTPROT-NMEA = 0
      0x01, 0x00, 0x74, 0x10, 0x01,                    // CFG-UART1OUTPROT-UBX = 1
  };
  sendUbx(0x06, 0x8A, val, sizeof(val));
}

void GpsDriver::begin(hal::Port port) {
  port_ = port;
  // Receivers boot at 9600 (M8) or 38400 (M10); modules set up by another firmware are often at 115200 or
  // 230400 (Betaflight / INAV): send the baud-rate change at each
  const uint32_t bauds[] = {9600, 38400, 57600, 115200, 230400};
  for (uint32_t b : bauds) {
    hal::portOpen(port_, b);
    hal::delayMs(50);
    configure();
    hal::delayMs(100);
  }
  hal::portOpen(port_, 115200);
  baud_ = 115200;
  baudIdx_ = 0;
  switchMs_ = hal::millis();
  timing_ = true;
  hal::delayMs(100);

  // 10 Hz navigation rate (M8: CFG-RATE)
  uint8_t rate[6] = {100, 0, 1, 0, 1, 0};
  sendUbx(0x06, 0x08, rate, sizeof(rate));
  // enable NAV-PVT on UART1 (M8: CFG-MSG)
  uint8_t msg[8] = {0x01, 0x07, 0, 1, 0, 0, 0, 0};
  sendUbx(0x06, 0x01, msg, sizeof(msg));
  // dynamic model "airborne < 1 g" (M8: CFG-NAV5)
  uint8_t nav5[36] = {0};
  nav5[0] = 0x01;  // mask: dyn
  nav5[2] = 6;
  sendUbx(0x06, 0x24, nav5, sizeof(nav5));
  // M9/M10: rate 100 ms, NAV-PVT on UART1, dyn model 6, GPS+Galileo+BeiDou default
  uint8_t val[] = {
      0x00, 0x03, 0x00, 0x00,
      0x01, 0x00, 0x21, 0x30, 100, 0x00,  // CFG-RATE-MEAS = 100 ms
      0x07, 0x00, 0x91, 0x20, 0x01,       // CFG-MSGOUT-UBX_NAV_PVT_UART1 = 1
      0x21, 0x00, 0x11, 0x20, 0x06,       // CFG-NAVSPG-DYNMODEL = 6
  };
  sendUbx(0x06, 0x8A, val, sizeof(val));
}

bool GpsDriver::poll() {
  bool got = false;
  int c, budget = 256;
  uint32_t before = frames_;
  while (budget-- > 0 && (c = hal::portRead(port_)) >= 0) got |= feed((uint8_t)c);
  uint32_t now = hal::millis();
  if (frames_ != before) lastFrameMs_ = now;
  if (!timing_) { timing_ = true; switchMs_ = now; }  // attach(): count from the first poll
  // nothing valid for 2.5 s at this baud rate: try the next one
  uint32_t since = lastFrameMs_ > switchMs_ ? lastFrameMs_ : switchMs_;
  if (now - since > 2500) {
    static const uint32_t bauds[] = {115200, 9600, 38400, 57600, 230400};
    baudIdx_ = (uint8_t)((baudIdx_ + 1) % (sizeof(bauds) / sizeof(bauds[0])));
    baud_ = bauds[baudIdx_];
    hal::portOpen(port_, baud_);
    ubxState_ = 0;
    inNmea_ = false;
    switchMs_ = now;
  }
  return got;
}

bool GpsDriver::feed(uint8_t b) {
  bool got = ubxByte(b);
  got |= nmeaByte(b);
  return got;
}

// ---------------------------------------------------------------- UBX
bool GpsDriver::ubxByte(uint8_t b) {
  switch (ubxState_) {
    case 0: if (b == 0xB5) ubxState_ = 1; return false;
    case 1: ubxState_ = b == 0x62 ? 2 : 0; ubxPos_ = 0; return false;
    case 2:  // class, id, len (4 bytes)
      ubx_[ubxPos_++] = b;
      if (ubxPos_ == 4) {
        ubxLen_ = ubx_[2] | (ubx_[3] << 8);
        if (ubxLen_ + 6 > (uint16_t)sizeof(ubx_)) { ubxState_ = 0; return false; }
        ubxState_ = 3;
      }
      return false;
    case 3:
      ubx_[ubxPos_++] = b;
      if (ubxPos_ < ubxLen_ + 6) return false;  // header(4) + payload + ck(2)
      ubxState_ = 0;
      break;
  }
  uint8_t a = 0, c = 0;
  for (uint16_t i = 0; i < ubxLen_ + 4; i++) { a += ubx_[i]; c += a; }
  if (a != ubx_[ubxLen_ + 4] || c != ubx_[ubxLen_ + 5]) return false;
  detected_ = ubxSeen_ = true;
  frames_++;
  if (ubx_[0] != 0x01 || ubx_[1] != 0x07 || ubxLen_ < 92) return false;  // NAV-PVT only
  const uint8_t* p = ubx_ + 4;
  uint8_t fixType = p[20];
  bool gnssFixOk = p[21] & 0x01;
  data_.fix3d = gnssFixOk && (fixType == 3 || fixType == 4);
  data_.sats = p[23];
  data_.lon = i32le(p + 24) * 1e-7;
  data_.lat = i32le(p + 28) * 1e-7;
  data_.altMsl = i32le(p + 36) * 1e-3f;
  data_.hAcc = u32le(p + 40) * 1e-3f;
  data_.velN = i32le(p + 48) * 1e-3f;
  data_.velE = i32le(p + 52) * 1e-3f;
  data_.velD = i32le(p + 56) * 1e-3f;
  data_.groundSpeed = i32le(p + 60) * 1e-3f;
  course_ = i32le(p + 64) * 1e-5f;
  data_.courseDeg = course_;
  data_.vAcc = u32le(p + 44) * 1e-3f;
  sanitize();
  return true;
}

// a corrupted frame that still passes its checksum (or a receiver bug) must not become a position:
// coordinates outside the globe, NaN or absurd speeds drop the fix for this solution
void GpsDriver::sanitize() {
  auto fin = [](double v) { return v == v && v < 1e9 && v > -1e9; };
  bool ok = fin(data_.lat) && fin(data_.lon) && data_.lat >= -90.0 && data_.lat <= 90.0 && data_.lon >= -180.0 && data_.lon <= 180.0 &&
            fin(data_.altMsl) && data_.altMsl > -1000.0f && data_.altMsl < 50000.0f && fin(data_.velN) && fin(data_.velE) && fin(data_.velD) &&
            fabsf(data_.velN) < 300.0f && fabsf(data_.velE) < 300.0f && fabsf(data_.velD) < 300.0f && fin(data_.hAcc) && data_.hAcc >= 0;
  if (!ok) {
    data_.fix3d = false;
    if (!(fin(data_.lat) && fin(data_.lon))) data_.lat = data_.lon = 0;
    if (!(fin(data_.velN) && fin(data_.velE) && fin(data_.velD))) data_.velN = data_.velE = data_.velD = 0;
    if (!fin(data_.groundSpeed)) data_.groundSpeed = 0;
    if (!fin(data_.altMsl)) data_.altMsl = 0;
    if (!fin(data_.hAcc) || data_.hAcc < 0) data_.hAcc = 999;
  }
}

// ---------------------------------------------------------------- NMEA (GGA + RMC)
bool GpsDriver::nmeaByte(uint8_t b) {
  if (b == '$') { inNmea_ = true; nmeaPos_ = 0; return false; }
  if (!inNmea_) return false;
  if (b == '\r' || b == '\n') {
    inNmea_ = false;
    nmea_[nmeaPos_] = 0;
    return parseNmea();
  }
  if (nmeaPos_ < sizeof(nmea_) - 1) nmea_[nmeaPos_++] = (char)b;
  else inNmea_ = false;
  return false;
}

static double nmeaCoord(const char* f, const char* hemi) {
  if (!*f) return 0;
  double v = atof(f);
  if (!(v >= 0.0 && v <= 18000.0)) return 999.0;  // not ddmm.mmmm / dddmm.mmmm: sanitize() drops the fix
  int deg = (int)(v / 100);
  double d = deg + (v - deg * 100) / 60.0;
  if (*hemi == 'S' || *hemi == 'W') d = -d;
  return d;
}

bool GpsDriver::parseNmea() {
  // checksum
  char* star = strchr(nmea_, '*');
  if (!star) return false;
  uint8_t cs = 0;
  for (char* p = nmea_; p < star; p++) cs ^= (uint8_t)*p;
  if (strtol(star + 1, nullptr, 16) != cs) return false;
  *star = 0;
  // split fields
  char* f[20];
  int n = 0;
  f[n++] = nmea_;
  for (char* p = nmea_; *p && n < 20; p++)
    if (*p == ',') { *p = 0; f[n++] = p + 1; }
  if (strlen(f[0]) < 5) return false;
  const char* type = f[0] + 2;  // skip talker (GP, GN, ...)
  detected_ = nmeaSeen_ = true;
  frames_++;
  if (!strcmp(type, "GGA") && n >= 10) {
    int q = atoi(f[6]);
    data_.lat = nmeaCoord(f[2], f[3]);
    data_.lon = nmeaCoord(f[4], f[5]);
    data_.sats = (uint8_t)atoi(f[7]);
    data_.fix3d = q >= 1 && data_.sats >= 4;
    float hdop = (float)atof(f[8]);
    data_.hAcc = hdop * 2.5f;  // rough estimate
    data_.altMsl = (float)atof(f[9]);
    sanitize();
    return true;  // published with or without a fix (satellite count shows progress)
  }
  if (!strcmp(type, "RMC") && n >= 9) {
    float knots = (float)atof(f[7]);
    course_ = (float)atof(f[8]);
    data_.courseDeg = course_;
    data_.groundSpeed = knots * 0.514444f;
    data_.velN = data_.groundSpeed * cosf(course_ * fc::DEG2RAD);
    data_.velE = data_.groundSpeed * sinf(course_ * fc::DEG2RAD);
    data_.velD = 0;
    sanitize();
  }
  return false;
}

}  // namespace drv
