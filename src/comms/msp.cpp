#include "../common/size_opt.h"  // -Os on ESP32 (not flight critical)
#include "msp.h"

#include <string.h>

#include "../bus/bus.h"
#include "../vtx/vtx.h"
#include "fourway.h"

namespace comms {

using namespace fc;

enum : uint16_t {
  MSP_API_VERSION = 1, MSP_FC_VARIANT = 2, MSP_FC_VERSION = 3, MSP_BOARD_INFO = 4, MSP_BUILD_INFO = 5,
  MSP_NAME = 10, MSP_STATUS = 101, MSP_RAW_IMU = 102, MSP_MOTOR = 104, MSP_RC = 105, MSP_RAW_GPS = 106,
  MSP_COMP_GPS = 107, MSP_ATTITUDE = 108, MSP_ALTITUDE = 109, MSP_ANALOG = 110, MSP_PID = 112,
  MSP_BATTERY_STATE = 130, MSP_STATUS_EX = 150, MSP_DISPLAYPORT = 182, MSP_SET_OSD_CANVAS = 188, MSP_OSD_CANVAS = 189,
  MSP_FEATURE_CONFIG = 36, MSP_VTX_CONFIG = 88, MSP_SET_VTX_CONFIG = 89, MSP_ADVANCED_CONFIG = 90,
  MSP_MOTOR_3D_CONFIG = 124, MSP_MOTOR_CONFIG = 131, MSP_UID = 160, MSP_SET_MOTOR = 214,
  MSP_SET_PASSTHROUGH = 245, MSP_EEPROM_WRITE = 250,
  MSP2_AQ_PARAM_INFO = 0x4151, MSP2_AQ_PARAM_FIND = 0x4152, MSP2_AQ_PARAM_SET = 0x4153,  // AllQuad ("AQ" range)
  MSP2_SENSOR_RANGEFINDER = 0x1F01, MSP2_SENSOR_OPTIC_FLOW = 0x1F02,
};

uint8_t crc8DvbS2(uint8_t crc, uint8_t b) {
  crc ^= b;
  for (int i = 0; i < 8; i++) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
  return crc;
}

size_t mspPack(uint8_t* o, bool v2, char dir, uint16_t cmd, const uint8_t* p, uint16_t len) {
  size_t n = 0;
  o[n++] = '$';
  o[n++] = v2 ? 'X' : 'M';
  o[n++] = (uint8_t)dir;
  if (!v2) {
    uint8_t chk = 0;
    o[n++] = (uint8_t)len; chk ^= (uint8_t)len;
    o[n++] = (uint8_t)cmd; chk ^= (uint8_t)cmd;
    for (uint16_t i = 0; i < len; i++) { o[n++] = p[i]; chk ^= p[i]; }
    o[n++] = chk;
  } else {
    uint8_t crc = 0;
    uint8_t hdr[5] = {0, (uint8_t)cmd, (uint8_t)(cmd >> 8), (uint8_t)len, (uint8_t)(len >> 8)};
    for (uint8_t b : hdr) { o[n++] = b; crc = crc8DvbS2(crc, b); }
    for (uint16_t i = 0; i < len; i++) { o[n++] = p[i]; crc = crc8DvbS2(crc, p[i]); }
    o[n++] = crc;
  }
  return n;
}

bool MspParser::feed(uint8_t b, MspFrame& out) {
  switch (st_) {
    case IDLE: if (b == '$') st_ = HDR_M_X; return false;
    case HDR_M_X:
      if (b == 'M' || b == 'X') { f_.v2 = b == 'X'; st_ = DIR; } else st_ = IDLE;
      return false;
    case DIR:
      if (b == '<' || b == '>' || b == '!') { f_.dir = (char)b; st_ = f_.v2 ? V2_FLAG : V1_LEN; } else st_ = IDLE;
      return false;
    case V1_LEN: f_.len = b; chk_ = b; st_ = V1_CMD; return false;
    case V1_CMD:
      f_.cmd = b; chk_ ^= b; idx_ = 0;
      st_ = f_.len ? PAYLOAD : CRC;
      return false;
    case V2_FLAG: chk_ = crc8DvbS2(0, b); st_ = V2_CMD1; return false;
    case V2_CMD1: f_.cmd = b; chk_ = crc8DvbS2(chk_, b); st_ = V2_CMD2; return false;
    case V2_CMD2: f_.cmd |= (uint16_t)(b << 8); chk_ = crc8DvbS2(chk_, b); st_ = V2_LEN1; return false;
    case V2_LEN1: f_.len = b; chk_ = crc8DvbS2(chk_, b); st_ = V2_LEN2; return false;
    case V2_LEN2:
      f_.len |= (uint16_t)(b << 8);
      chk_ = crc8DvbS2(chk_, b);
      idx_ = 0;
      if (f_.len > sizeof(f_.payload)) { st_ = IDLE; return false; }
      st_ = f_.len ? PAYLOAD : CRC;
      return false;
    case PAYLOAD:
      f_.payload[idx_++] = b;
      chk_ = f_.v2 ? crc8DvbS2(chk_, b) : (uint8_t)(chk_ ^ b);
      if (idx_ >= f_.len) st_ = CRC;
      return false;
    case CRC:
      st_ = IDLE;
      if (b != chk_) return false;
      out = f_;
      return true;
  }
  return false;
}

namespace {
struct B {
  uint8_t b[256];
  uint16_t n = 0;
  void u8(uint8_t v) { b[n++] = v; }
  void u16(uint16_t v) { u8((uint8_t)v); u8((uint8_t)(v >> 8)); }
  void u32(uint32_t v) { u16((uint16_t)v); u16((uint16_t)(v >> 16)); }
  void str(const char* s) { while (*s) u8((uint8_t)*s++); }
};
// parameter answer: u16 index, u16 count, u8 type, u8 flags (1 = needs a reboot), i32 value, min, max (x 1000),
// name (up to 16 characters, no terminator: the rest of the payload)
void paramAnswer(B& o, int i) {
  const ParamInfo& p = paramInfo(i);
  auto x1000 = [](float v) { float s = v * 1000.0f; return (uint32_t)(int32_t)(s > 2.0e9f ? 2.0e9f : s < -2.0e9f ? -2.0e9f : (s < 0 ? s - 0.5f : s + 0.5f)); };
  o.u16((uint16_t)i);
  o.u16((uint16_t)paramCount());
  o.u8((uint8_t)p.type);
  o.u8(p.reboot ? 1 : 0);
  o.u32(x1000(paramGet(bus::config, i)));
  o.u32(x1000(p.min));
  o.u32(x1000(p.max));
  for (int k = 0; p.name[k] && k < 16; k++) o.u8((uint8_t)p.name[k]);
}
int32_t rd32(const uint8_t* p) { return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24)); }
}  // namespace

void MspLink::error(const MspFrame& req) {
  if (hook_) { hook_(hookCtx_, req, nullptr, 0, true); return; }
  uint8_t buf[16];
  size_t n = mspPack(buf, req.v2, '!', req.cmd, nullptr, 0);
  if (hal::portWriteSpace(port_) >= n) hal::portWrite(port_, buf, n);
}

void MspLink::reply(const MspFrame& req, const uint8_t* payload, uint16_t len) {
  if (hook_) { hook_(hookCtx_, req, payload, len, false); return; }
  uint8_t buf[300];
  size_t n = mspPack(buf, req.v2, '>', req.cmd, payload, len);
  if (hal::portWriteSpace(port_) >= n) hal::portWrite(port_, buf, n);
}

void MspLink::handle(const MspFrame& f, uint32_t now) {
  lastRxMs_ = now;
  if (f.dir != '<') return;
  const Config& c = bus::config;
  B o;
  switch (f.cmd) {
    case MSP_API_VERSION: o.u8(0); o.u8(1); o.u8(46); break;
    case MSP_FC_VARIANT: o.str(displayPort_ ? "BTFL" : "AQAD"); break;
    case MSP_FC_VERSION:  // DisplayPort: a Betaflight 4.5 level (DJI O3 / O4 need 4.4+ for the HD canvas)
      if (displayPort_) { o.u8(4); o.u8(5); o.u8(0); } else { o.u8(2); o.u8(0); o.u8(0); }
      break;
    case MSP_SET_OSD_CANVAS:  // sent by the display: its grid in characters
      if (f.len >= 2) { canvasCols_ = f.payload[0]; canvasRows_ = f.payload[1]; }
      break;
    case MSP_OSD_CANVAS: {
      int cols = canvasCols_, rows = canvasRows_;
      if (!cols) { cols = 53; rows = 20; }
      o.u8((uint8_t)cols); o.u8((uint8_t)rows);
      break;
    }
    case MSP_BOARD_INFO: o.str(hal::platformName()[0] == 'E' ? "AQE3" : "AQS4"); o.u16(0); o.u8(0); break;
    case MSP_BUILD_INFO: o.str("Oct 22 2026"); o.str("12:00:00"); o.str("allquad"); break;
    case MSP_NAME: o.str("AllQuad"); break;
    case MSP_STATUS:
    case MSP_STATUS_EX: {
      VehicleStatus st = bus::status.get();
      uint16_t sensors = 1 | (bus::baro.seq() ? 2 : 0) | (bus::mag.seq() ? 4 : 0) | (bus::gps.seq() ? 8 : 0) |
                         (bus::range.seq() ? 16 : 0);
      uint32_t flags = (st.armed ? 1u : 0u) | (1u << ((uint8_t)st.mode + 1));
      if (displayPort_)  // Betaflight box order: 0 ARM, 1 ANGLE, 2 HORIZON
        flags = (st.armed ? 1u : 0u) | (st.mode == FlightMode::ANGLE ? 2u : 0u) | (st.mode == FlightMode::HORIZON ? 4u : 0u);
      o.u16(os::taskCount() ? (uint16_t)os::taskStats(0).periodUs : 1000);
      o.u16(0);
      o.u16(sensors);
      o.u32(flags);
      o.u8(c.pid_profile);
      if (f.cmd == MSP_STATUS_EX) {  // Betaflight layout: cpu load, profile count, rate profile, arming flags
        o.u16(0); o.u8(3); o.u8(c.pid_profile); o.u8(0);
        o.u8(32); o.u32(st.armBlockers); o.u8(0);
      }
      break;
    }
    case MSP_RAW_IMU: {
      AttitudeState a = bus::attitude.get();
      MagSample m = bus::mag.get();
      for (int i = 0; i < 3; i++) o.u16((uint16_t)(int16_t)(a.accBody[i] / GRAVITY * 512.0f));
      for (int i = 0; i < 3; i++) o.u16((uint16_t)(int16_t)a.rateDps[i]);
      for (int i = 0; i < 3; i++) o.u16((uint16_t)(int16_t)(m.field[i] * 1000.0f));
      break;
    }
    case MSP_MOTOR: {
      MotorOutputs m = bus::motors.get();
      for (int i = 0; i < 8; i++) o.u16(i < 4 ? (uint16_t)(m.armed && m.out[i] >= 0 ? 1000 + m.out[i] * 1000 : 1000) : 0);
      break;
    }
    case MSP_RC: {
      RcInput rc = bus::rc.get();
      for (int i = 0; i < rc.count && i < RC_MAX_CH; i++) o.u16(rc.ch[i]);
      break;
    }
    case MSP_RAW_GPS: {
      GpsData g = bus::gps.get();
      o.u8(g.fix3d ? 2 : 0); o.u8(g.sats);
      o.u32((uint32_t)(int32_t)(g.lat * 1e7)); o.u32((uint32_t)(int32_t)(g.lon * 1e7));
      o.u16((uint16_t)g.altMsl); o.u16((uint16_t)(g.groundSpeed * 100)); o.u16((uint16_t)(g.courseDeg * 10));
      o.u16((uint16_t)(g.hAcc * 100));
      break;
    }
    case MSP_COMP_GPS: {
      VehicleStatus st = bus::status.get();
      float b = st.homeBearingDeg < 0 ? st.homeBearingDeg + 360 : st.homeBearingDeg;
      o.u16((uint16_t)st.homeDistance); o.u16((uint16_t)b); o.u8(1);
      break;
    }
    case MSP_ATTITUDE: {
      AttitudeState a = bus::attitude.get();
      float y = bus::nav.get().yaw * RAD2DEG;  // true heading
      if (y < 0) y += 360;
      o.u16((uint16_t)(int16_t)(a.roll * RAD2DEG * 10)); o.u16((uint16_t)(int16_t)(a.pitch * RAD2DEG * 10));
      o.u16((uint16_t)(int16_t)y);
      break;
    }
    case MSP_ALTITUDE: {
      NavEstimate e = bus::nav.get();
      o.u32((uint32_t)(int32_t)(e.alt * 100)); o.u16((uint16_t)(int16_t)(e.vz * 100));
      break;
    }
    case MSP_ANALOG: {
      BatteryState b = bus::battery.get();
      RcInput rc = bus::rc.get();
      o.u8((uint8_t)constrainf(b.voltage * 10, 0, 255)); o.u16((uint16_t)b.mahDrawn);
      o.u16((uint16_t)(rc.linkQuality * 1023 / 100)); o.u16((uint16_t)(int16_t)(b.current * 100));
      o.u16((uint16_t)(b.voltage * 100));
      break;
    }
    case MSP_BATTERY_STATE: {
      BatteryState b = bus::battery.get();
      o.u8(b.cells); o.u16(c.bat_capacity); o.u8((uint8_t)constrainf(b.voltage * 10, 0, 255));
      o.u16((uint16_t)b.mahDrawn); o.u16((uint16_t)(b.current * 100));
      o.u8(b.critical ? 2 : (b.warn ? 1 : 0)); o.u16((uint16_t)(b.voltage * 100));
      break;
    }
    case MSP_UID: {
      uint32_t id = hal::deviceId();
      o.u32(id); o.u32(id ^ 0xA5A5A5A5u); o.u32(~id);
      break;
    }
    case MSP_FEATURE_CONFIG: o.u32(0); break;
    case MSP_MOTOR_CONFIG:  // min / max throttle, min command, motor count, poles, DShot telemetry, ESC sensor
      o.u16(1070); o.u16(2000); o.u16(1000); o.u8(4); o.u8(c.motor_poles); o.u8(0); o.u8(c.esc_telemetry);
      break;
    case MSP_MOTOR_3D_CONFIG: o.u16(1406); o.u16(1514); o.u16(1460); break;
    case MSP_ADVANCED_CONFIG: {
      static const uint8_t proto[] = {0, 1, 6, 7};  // Betaflight enum: PWM, ONESHOT125, DSHOT300, DSHOT600
      o.u8(1); o.u8(1); o.u8(0); o.u8(proto[c.motor_protocol & 3]); o.u16(480);
      o.u16((uint16_t)(c.motor_idle * 10000)); o.u8(0); o.u8(0);
      break;
    }
    case MSP_SET_MOTOR: {  // esc-configurator / Betaflight motor sliders: 1000..2000 us, disarmed only
      float v[4];
      for (int i = 0; i < 4; i++) {
        uint16_t us = f.len >= (i + 1) * 2 ? (uint16_t)(f.payload[i * 2] | f.payload[i * 2 + 1] << 8) : 1000;
        v[i] = us > 1000 ? (us - 1000) / 1000.0f : 0.0f;
      }
      if (!bus::setMotorTest(v, now)) break;
      break;
    }
    case MSP_SET_PASSTHROUGH: {
      // ESC 4-way interface (esc-configurator.com, BLHeliSuite): mode 0xFF or no payload
      bool fourWay = f.len == 0 || f.payload[0] == 0xFF;
      if (!fourWay_ || !fourWay || bus::status.get().armed) { o.u8(0); break; }
      o.u8(4);
      reply(f, o.b, o.n);
      fourWay_->start(now);  // from now on the console speaks the 4-way protocol
      return;
    }
    case MSP_VTX_CONFIG: {
      o.u8(vtxType_); o.u8(c.vtx_band); o.u8(c.vtx_channel); o.u8(c.vtx_power); o.u8(c.vtx_pit);
      o.u16(vtx::frequency(c)); o.u8(1); o.u8(c.vtx_low_disarmed); o.u16(0);
      o.u8(0); o.u8(0); o.u8(0); o.u8(0);  // no VTX table
      break;
    }
    case MSP_SET_VTX_CONFIG: {
      // digital systems (DJI goggles menu) set band/channel or frequency, power, pit mode
      if (f.len < 2 || bus::status.get().armed) break;
      Config& w = bus::config;
      uint16_t v = (uint16_t)(f.payload[0] | f.payload[1] << 8);
      if (v < 64) { w.vtx_band = (uint8_t)(v / 8 + 1); w.vtx_channel = (uint8_t)(v % 8 + 1); w.vtx_freq = 0; }
      else w.vtx_freq = v;
      if (f.len >= 3 && f.payload[2] >= 1 && f.payload[2] <= vtx::powerCount(w)) w.vtx_power = f.payload[2];
      if (f.len >= 4) w.vtx_pit = f.payload[3] ? 1 : 0;
      if (f.len >= 5) w.vtx_low_disarmed = f.payload[4] ? 1 : 0;
      break;
    }
    case MSP_PID:
      for (int i = 0; i < 3; i++) { o.u8((uint8_t)c.pid[i].p); o.u8((uint8_t)c.pid[i].i); o.u8((uint8_t)c.pid[i].d); }
      break;
    case MSP2_SENSOR_RANGEFINDER: {  // quality u8, distance mm i32 (<0 = out of range)
      if (f.len < 5) return;
      int32_t mm = rd32(f.payload + 1);
      RangeSample r{mm > 0 ? mm * 0.001f : -1.0f, mm > 0 && f.payload[0] > 0, f.payload[0]};
      bus::range.publish(r);
      return;  // sensor messages are not answered
    }
    case MSP2_SENSOR_OPTIC_FLOW: {  // quality u8, motionX i32, motionY i32
      if (f.len < 9) return;
      float dt = lastFlowMs_ ? (now - lastFlowMs_) * 0.001f : 0.0f;
      lastFlowMs_ = now;
      if (dt <= 0.001f || dt > 0.25f) return;
      float k = 1.0f / (c.flow_scale * dt);  // iNav convention: raw / opflow_scale / dt = rad/s
      FlowSample s{rd32(f.payload + 1) * k, rd32(f.payload + 5) * k, f.payload[0], dt};
      bus::flow.publish(s);
      return;
    }
    // ---- AllQuad parameters (radio Lua script over CRSF, or any MSP port): value / min / max x 1000
    case MSP2_AQ_PARAM_INFO:
    case MSP2_AQ_PARAM_FIND: {
      int i = -1;
      if (f.cmd == MSP2_AQ_PARAM_INFO && f.len >= 2) i = f.payload[0] | (f.payload[1] << 8);
      if (f.cmd == MSP2_AQ_PARAM_FIND && f.len >= 1 && f.len <= 24) {
        char name[25];
        memcpy(name, f.payload, f.len);
        name[f.len] = 0;
        i = paramFind(name);
      }
      if (i < 0 || i >= paramCount()) { error(f); return; }
      paramAnswer(o, i);
      break;
    }
    case MSP2_AQ_PARAM_SET: {  // u16 index, i32 value x 1000 -> the answer of PARAM_INFO with the new value
      if (f.len < 6) { error(f); return; }
      int i = f.payload[0] | (f.payload[1] << 8);
      if (i < 0 || i >= paramCount() || bus::status.get().armed) { error(f); return; }
      if (!paramSet(bus::config, i, rd32(f.payload + 2) * 0.001f)) { error(f); return; }
      bus::configChanged();
      paramAnswer(o, i);
      break;
    }
    case MSP_EEPROM_WRITE: {  // save the settings (disarmed); answers u8 1 = saved
      if (bus::status.get().armed) { error(f); return; }
      configSeal(bus::config);
      o.u8(hal::storageSave(hal::Slot::CONFIG, &bus::config, sizeof(Config)) ? 1 : 0);
      break;
    }
    default:
      error(f);  // unsupported
      return;
  }
  reply(f, o.b, o.n);
}

void MspLink::update(uint32_t now) {
  MspFrame f;
  int c, budget = 512;
  while (budget-- > 0 && (c = hal::portRead(port_)) >= 0)
    if (parser_.feed((uint8_t)c, f)) handle(f, now);
}

// ---------------------------------------------------------------- DisplayPort
void MspLink::dpHeartbeat() {
  uint8_t p[1] = {0};
  uint8_t buf[16];
  hal::portWrite(port_, buf, mspPack(buf, false, '>', MSP_DISPLAYPORT, p, 1));
}
void MspLink::dpClear() {
  uint8_t p[1] = {2};
  uint8_t buf[16];
  hal::portWrite(port_, buf, mspPack(buf, false, '>', MSP_DISPLAYPORT, p, 1));
}
bool MspLink::dpWrite(uint8_t row, uint8_t col, const char* text, uint8_t attr) {
  uint8_t p[68] = {3, row, col, attr};
  uint16_t n = 4;
  while (*text && n < sizeof(p)) p[n++] = (uint8_t)*text++;
  uint8_t buf[80];
  size_t len = mspPack(buf, false, '>', MSP_DISPLAYPORT, p, n);
  if (hal::portWriteSpace(port_) < len + 8) return false;  // keep room for the draw command
  hal::portWrite(port_, buf, len);
  return true;
}
void MspLink::dpOptions(uint8_t font, uint8_t resolution) {
  uint8_t p[3] = {5, font, resolution};
  uint8_t buf[16];
  hal::portWrite(port_, buf, mspPack(buf, false, '>', MSP_DISPLAYPORT, p, 3));
}
void MspLink::dpDraw() {
  uint8_t p[1] = {4};
  uint8_t buf[16];
  hal::portWrite(port_, buf, mspPack(buf, false, '>', MSP_DISPLAYPORT, p, 1));
}

}  // namespace comms
