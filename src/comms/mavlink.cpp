#include "../common/size_opt.h"  // -Os on ESP32 (not flight critical)
#include "mavlink.h"

#include <string.h>

#include "../bus/bus.h"
#include "../navigation/mission.h"

namespace comms {

using namespace fc;

// ---------------------------------------------------------------- message ids / CRC extras
enum : uint32_t {
  MSG_HEARTBEAT = 0, MSG_SYS_STATUS = 1, MSG_SYSTEM_TIME = 2, MSG_SET_MODE = 11, MSG_PARAM_REQUEST_READ = 20,
  MSG_PARAM_REQUEST_LIST = 21, MSG_PARAM_VALUE = 22, MSG_PARAM_SET = 23, MSG_GPS_RAW_INT = 24,
  MSG_ATTITUDE = 30, MSG_GLOBAL_POSITION_INT = 33, MSG_MISSION_ITEM = 39, MSG_MISSION_REQUEST = 40,
  MSG_MISSION_SET_CURRENT = 41, MSG_MISSION_CURRENT = 42, MSG_MISSION_REQUEST_LIST = 43, MSG_MISSION_COUNT = 44,
  MSG_MISSION_CLEAR_ALL = 45, MSG_MISSION_ITEM_REACHED = 46, MSG_MISSION_ACK = 47, MSG_MISSION_REQUEST_INT = 51,
  MSG_RC_CHANNELS = 65, MSG_MISSION_ITEM_INT = 73, MSG_VFR_HUD = 74, MSG_COMMAND_INT = 75, MSG_COMMAND_LONG = 76,
  MSG_COMMAND_ACK = 77, MSG_SET_POSITION_TARGET_LOCAL_NED = 84, MSG_SET_POSITION_TARGET_GLOBAL_INT = 86,
  MSG_AUTOPILOT_VERSION = 148, MSG_HOME_POSITION = 242, MSG_STATUSTEXT = 253,
};

int mavCrcExtra(uint32_t id) {
  switch (id) {
    case 0: return 50;    case 1: return 124;   case 2: return 137;   case 11: return 89;   case 20: return 214;
    case 21: return 159;  case 22: return 220;  case 23: return 168;  case 24: return 24;
    case 30: return 39;   case 33: return 104;  case 39: return 254;  case 40: return 230;
    case 41: return 28;   case 42: return 28;   case 43: return 132;  case 44: return 221;
    case 45: return 232;  case 46: return 11;   case 47: return 153;  case 51: return 196;
    case 65: return 118;  case 73: return 38;   case 74: return 20;   case 75: return 158;
    case 76: return 152;  case 77: return 143;  case 84: return 143;  case 86: return 5;
    case 148: return 178; case 242: return 104;
    case 253: return 83;
    default: return -1;
  }
}

uint16_t mavCrcAccumulate(uint8_t b, uint16_t crc) {
  uint8_t tmp = b ^ (uint8_t)(crc & 0xFF);
  tmp ^= (uint8_t)(tmp << 4);
  return (uint16_t)((crc >> 8) ^ ((uint16_t)tmp << 8) ^ ((uint16_t)tmp << 3) ^ (tmp >> 4));
}

size_t mavPack(uint8_t* o, uint32_t msgid, const uint8_t* payload, uint8_t len, uint8_t sysid, uint8_t compid,
               uint8_t seq) {
  while (len > 1 && payload[len - 1] == 0) len--;  // MAVLink 2 payload truncation
  o[0] = 0xFD; o[1] = len; o[2] = 0; o[3] = 0; o[4] = seq; o[5] = sysid; o[6] = compid;
  o[7] = (uint8_t)msgid; o[8] = (uint8_t)(msgid >> 8); o[9] = (uint8_t)(msgid >> 16);
  memcpy(o + 10, payload, len);
  uint16_t crc = 0xFFFF;
  for (size_t i = 1; i < 10u + len; i++) crc = mavCrcAccumulate(o[i], crc);
  crc = mavCrcAccumulate((uint8_t)mavCrcExtra(msgid), crc);
  o[10 + len] = (uint8_t)crc;
  o[11 + len] = (uint8_t)(crc >> 8);
  return 12u + len;
}

bool MavParser::feed(uint8_t b, MavFrame& out) {
  if (pos_ == 0) {
    if (b != 0xFD && b != 0xFE) return false;
    v2_ = b == 0xFD;
    buf_[pos_++] = b;
    return false;
  }
  buf_[pos_++] = b;
  if (pos_ == 2) need_ = (uint16_t)(v2_ ? 12 + b : 8 + b);
  if (pos_ < 2 || pos_ < need_) return false;
  pos_ = 0;
  uint8_t len = buf_[1];
  uint32_t id;
  const uint8_t* pl;
  if (v2_) {
    if (buf_[2] & ~0x01) { errors_++; return false; }  // unsupported incompat flags (signing allowed but ignored)
    id = buf_[7] | (buf_[8] << 8) | ((uint32_t)buf_[9] << 16);
    pl = buf_ + 10;
  } else {
    id = buf_[5];
    pl = buf_ + 6;
  }
  int extra = mavCrcExtra(id);
  if (extra < 0) return false;  // unknown message: cannot validate, ignore silently
  uint16_t crc = 0xFFFF;
  size_t hdr = v2_ ? 10 : 6;
  for (size_t i = 1; i < hdr + len; i++) crc = mavCrcAccumulate(buf_[i], crc);
  crc = mavCrcAccumulate((uint8_t)extra, crc);
  if ((uint8_t)crc != buf_[hdr + len] || (uint8_t)(crc >> 8) != buf_[hdr + len + 1]) { errors_++; return false; }
  out.msgid = id;
  out.len = len;
  out.seq = v2_ ? buf_[4] : buf_[2];
  out.sysid = v2_ ? buf_[5] : buf_[3];
  out.compid = v2_ ? buf_[6] : buf_[4];
  memset(out.payload, 0, sizeof(out.payload));  // re-expand truncated payloads
  memcpy(out.payload, pl, len);
  return true;
}

// ---------------------------------------------------------------- little-endian helpers
namespace {
struct W {
  uint8_t b[255];
  uint8_t n = 0;
  W() { memset(b, 0, sizeof(b)); }
  void u8(uint8_t v) { b[n++] = v; }
  void i8(int8_t v) { b[n++] = (uint8_t)v; }
  void u16(uint16_t v) { b[n++] = (uint8_t)v; b[n++] = (uint8_t)(v >> 8); }
  void i16(int16_t v) { u16((uint16_t)v); }
  void u32(uint32_t v) { for (int i = 0; i < 4; i++) b[n++] = (uint8_t)(v >> (8 * i)); }
  void i32(int32_t v) { u32((uint32_t)v); }
  void u64(uint64_t v) { for (int i = 0; i < 8; i++) b[n++] = (uint8_t)(v >> (8 * i)); }
  void f32(float v) { uint32_t u; memcpy(&u, &v, 4); u32(u); }
  void str(const char* s, int len) { for (int i = 0; i < len; i++) b[n++] = (s && *s) ? (uint8_t)*s++ : 0; }
};
inline float rf(const uint8_t* p) { float v; memcpy(&v, p, 4); return v; }
inline uint16_t ru16(const uint8_t* p) { return (uint16_t)(p[0] | (p[1] << 8)); }
inline int16_t ri16(const uint8_t* p) { return (int16_t)ru16(p); }
inline int32_t ri32(const uint8_t* p) { return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24)); }

// MAV_MODE_FLAG
constexpr uint8_t MODE_CUSTOM = 1, MODE_AUTO = 4, MODE_GUIDED = 8, MODE_STABILIZE = 16, MODE_MANUAL = 64,
                  MODE_ARMED = 128;
}  // namespace

// ---------------------------------------------------------------- link
void MavlinkLink::begin(hal::Port p, bool streamOnlyWhenConnected) {
  port_ = p;
  streamOnlyWhenConnected_ = streamOnlyWhenConnected;
}

bool MavlinkLink::forUs(uint8_t t) const { return t == 0 || t == bus::config.mav_sysid; }

void MavlinkLink::send(uint32_t msgid, const uint8_t* payload, uint8_t len) {
  uint8_t buf[280];
  size_t n = mavPack(buf, msgid, payload, len, bus::config.mav_sysid, 1, txSeq_++);
  if (hal::portWriteSpace(port_) < n) return;  // never block the telemetry task
  hal::portWrite(port_, buf, n);
}

void MavlinkLink::sendHeartbeat() {
  VehicleStatus st = bus::status.get();
  uint8_t base = MODE_CUSTOM | MODE_STABILIZE;
  bool autoMode = st.mode == FlightMode::RTH || st.mode == FlightMode::MISSION || st.mode == FlightMode::LAND ||
                  st.mode == FlightMode::GUIDED || st.mode == FlightMode::SMART_RTL;
  if (autoMode) base |= MODE_AUTO | MODE_GUIDED;
  else base |= MODE_MANUAL;
  if (st.armed) base |= MODE_ARMED;
  W w;
  w.u32((uint32_t)st.mode);  // custom_mode
  w.u8(2);                   // MAV_TYPE_QUADROTOR
  w.u8(0);                   // MAV_AUTOPILOT_GENERIC
  w.u8(base);
  w.u8(st.failsafe ? 5 : (st.armed ? 4 : 3));  // CRITICAL / ACTIVE / STANDBY
  w.u8(3);
  send(MSG_HEARTBEAT, w.b, 9);
}

void MavlinkLink::sendSysStatus() {
  BatteryState b = bus::battery.get();
  VehicleStatus st = bus::status.get();
  uint32_t present = 1 | 2 | (1u << 10) | (1u << 11) | (1u << 15) | (1u << 21);
  if (bus::baro.seq()) present |= 8 | (1u << 13);
  if (bus::mag.seq()) present |= 4;
  if (bus::gps.seq()) present |= 32 | (1u << 14);
  if (bus::rc.seq()) present |= 1u << 16;
  if (b.voltage > 1) present |= 1u << 25;
  uint32_t health = present;
  if (st.armBlockers & ARM_NO_RX) health &= ~(1u << 16);
  if (st.warnings & WARN_GPS_LOST) health &= ~32u;
  if (st.warnings & WARN_BAT_CRIT) health &= ~(1u << 25);
  uint16_t load = 0;
  if (os::taskCount()) {
    os::TaskStats t = os::taskStats(0);
    load = (uint16_t)(t.periodUs ? (uint32_t)t.lastUs * 1000 / t.periodUs : 0);
  }
  W w;
  w.u32(present); w.u32(present); w.u32(health);
  w.u16(load);
  w.u16((uint16_t)(b.voltage * 1000.0f));
  w.i16(b.current > 0 ? (int16_t)(b.current * 100.0f) : -1);
  w.u16(0); w.u16(0);
  w.u16(0); w.u16(0); w.u16(0); w.u16(0);
  w.i8(b.cells ? (int8_t)b.percent : -1);
  send(MSG_SYS_STATUS, w.b, 31);
}

void MavlinkLink::sendAttitude(uint32_t now) {
  AttitudeState a = bus::attitude.get();
  float yaw = bus::nav.get().yaw;  // true heading (compass / GPS / EKF corrected), not the raw AHRS yaw
  W w;
  w.u32(now);
  w.f32(a.roll); w.f32(a.pitch); w.f32(yaw);
  w.f32(a.rateDps.x * DEG2RAD); w.f32(a.rateDps.y * DEG2RAD); w.f32(a.rateDps.z * DEG2RAD);
  send(MSG_ATTITUDE, w.b, 28);
}

void MavlinkLink::sendPosition(uint32_t now) {
  NavEstimate e = bus::nav.get();
  double lat = 0, lon = 0;
  if (e.homeSet) {
    lat = e.homeLat + e.n / 6371000.0 * RAD2DEG;
    lon = e.homeLon + e.e / (6371000.0 * cos(e.homeLat * DEG2RAD)) * RAD2DEG;
  }
  float hdg = e.yaw * RAD2DEG;
  if (hdg < 0) hdg += 360.0f;
  W w;
  w.u32(now);
  w.i32((int32_t)(lat * 1e7)); w.i32((int32_t)(lon * 1e7));
  w.i32((int32_t)((e.homeAltMsl + e.alt) * 1000.0f));
  w.i32((int32_t)(e.alt * 1000.0f));
  w.i16((int16_t)(e.vn * 100.0f)); w.i16((int16_t)(e.ve * 100.0f)); w.i16((int16_t)(-e.vz * 100.0f));
  w.u16((uint16_t)(hdg * 100.0f));
  send(MSG_GLOBAL_POSITION_INT, w.b, 28);
}

void MavlinkLink::sendGpsRaw(uint32_t now) {
  GpsData g;
  if (!bus::gps.read(g)) return;
  W w;
  w.u64((uint64_t)now * 1000);
  w.i32((int32_t)(g.lat * 1e7)); w.i32((int32_t)(g.lon * 1e7)); w.i32((int32_t)(g.altMsl * 1000.0f));
  w.u16((uint16_t)constrainf(g.hAcc * 100.0f, 0, 65535)); w.u16((uint16_t)constrainf(g.vAcc * 100.0f, 0, 65535));
  w.u16((uint16_t)(g.groundSpeed * 100.0f)); w.u16((uint16_t)(g.courseDeg * 100.0f));
  w.u8(g.fix3d ? 3 : (g.sats ? 1 : 0));
  w.u8(g.sats);
  send(MSG_GPS_RAW_INT, w.b, 30);
}

void MavlinkLink::sendVfrHud() {
  NavEstimate e = bus::nav.get();
  MotorOutputs m = bus::motors.get();
  float hdg = e.yaw * RAD2DEG;
  if (hdg < 0) hdg += 360.0f;
  float gs = sqrtf(e.vn * e.vn + e.ve * e.ve);
  float thr = m.armed ? (m.out[0] + m.out[1] + m.out[2] + m.out[3]) * 25.0f : 0;
  W w;
  w.f32(gs); w.f32(gs); w.f32(e.alt); w.f32(e.vz);
  w.i16((int16_t)hdg);
  w.u16((uint16_t)constrainf(thr, 0, 100));
  send(MSG_VFR_HUD, w.b, 20);
}

void MavlinkLink::sendRcChannels(uint32_t now) {
  RcInput rc;
  if (!bus::rc.read(rc)) return;
  W w;
  w.u32(now);
  for (int i = 0; i < 18; i++) w.u16(i < RC_MAX_CH && i < rc.count ? rc.ch[i] : 0xFFFF);
  w.u8(rc.count);
  w.u8(rc.linkQuality ? (uint8_t)(rc.linkQuality * 254 / 100) : 255);
  send(MSG_RC_CHANNELS, w.b, 42);
}

void MavlinkLink::sendHome() {
  NavEstimate e = bus::nav.get();
  if (!e.homeSet) return;
  W w;
  w.i32((int32_t)(e.homeLat * 1e7)); w.i32((int32_t)(e.homeLon * 1e7)); w.i32((int32_t)(e.homeAltMsl * 1000.0f));
  w.f32(0); w.f32(0); w.f32(0);
  w.f32(1); w.f32(0); w.f32(0); w.f32(0);
  w.f32(0); w.f32(0); w.f32(0);
  send(MSG_HOME_POSITION, w.b, 52);
}

void MavlinkLink::sendStatusText(const char* text, uint8_t severity) {
  W w;
  w.u8(severity);
  w.str(text, 50);
  send(MSG_STATUSTEXT, w.b, 51);
}

void MavlinkLink::sendParam(int index) {
  if (index < 0 || index >= paramCount()) return;
  const ParamInfo& p = paramInfo(index);
  W w;
  w.f32(paramGet(bus::config, index));
  w.u16((uint16_t)paramCount());
  w.u16((uint16_t)index);
  w.str(p.name, 16);
  w.u8(9);  // MAV_PARAM_TYPE_REAL32
  send(MSG_PARAM_VALUE, w.b, 25);
}

void MavlinkLink::sendCommandAck(uint16_t cmd, uint8_t result) {
  W w;
  w.u16(cmd);
  w.u8(result);
  w.u8(0); w.i32(0); w.u8(gcsSys_); w.u8(gcsComp_);
  send(MSG_COMMAND_ACK, w.b, 10);
}

void MavlinkLink::sendMissionAck(uint8_t type, uint8_t missionType) {
  W w;
  w.u8(gcsSys_); w.u8(gcsComp_); w.u8(type); w.u8(missionType);
  send(MSG_MISSION_ACK, w.b, 4);
}

void MavlinkLink::sendMissionCount(uint8_t missionType) {
  W w;
  w.u16(missionType == 0 ? (uint16_t)nav::count()
        : missionType == 2 ? (uint16_t)nav::rallyCount() : missionType == 1 ? (uint16_t)nav::fenceCount() : 0);
  w.u8(gcsSys_); w.u8(gcsComp_); w.u8(missionType);
  send(MSG_MISSION_COUNT, w.b, 5);
}

void MavlinkLink::sendMissionRequest(uint16_t seq) {
  W w;
  w.u16(seq); w.u8(gcsSys_); w.u8(gcsComp_); w.u8(upType_);
  send(upUseInt_ ? MSG_MISSION_REQUEST_INT : MSG_MISSION_REQUEST, w.b, 5);
}

void MavlinkLink::sendMissionItem(uint16_t seq, uint8_t missionType) {
  nav::MissionItem it;
  bool got = missionType == 2 ? nav::rallyGet(seq, it) : missionType == 1 ? nav::fenceGet(seq, it) : nav::get(seq, it);
  if (!got) return;
  VehicleStatus st = bus::status.get();
  W w;
  w.f32(it.param1); w.f32(it.param2); w.f32(it.param3); w.f32(it.param4);
  w.i32(it.lat); w.i32(it.lon); w.f32(it.alt);
  w.u16(seq); w.u16(it.cmd);
  w.u8(gcsSys_); w.u8(gcsComp_);
  w.u8(it.frame ? it.frame : 6);  // MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
  w.u8(missionType == 0 && st.mode == FlightMode::MISSION && st.missionIndex == seq ? 1 : 0);
  w.u8(it.autocontinue);
  w.u8(missionType);
  send(MSG_MISSION_ITEM_INT, w.b, 38);
}

void MavlinkLink::sendAutopilotVersion() {
  W w;
  // MISSION_INT | PARAM_FLOAT | COMMAND_INT | SET_POSITION_TARGET_LOCAL_NED / GLOBAL_INT | MAVLINK2 |
  // MISSION_FENCE | MISSION_RALLY
  w.u64(4 | 2 | 8 | 128 | 256 | 8192 | 16384 | 32768);
  w.u64(0);
  w.u32((2u << 24) | (0u << 16) | (0u << 8) | 255);  // 2.0.0 official
  w.u32(0); w.u32(0); w.u32(0);
  w.u16(0); w.u16(0);
  w.str("AllQuad", 8);
  w.str(nullptr, 8);
  w.str(nullptr, 8);
  send(MSG_AUTOPILOT_VERSION, w.b, 60);
}

// ---------------------------------------------------------------- input handling
void MavlinkLink::handleCommand(uint16_t cmd, const float p[7], uint32_t nowMs) {
  VehicleStatus st = bus::status.get();
  uint8_t res = 0;  // MAV_RESULT_ACCEPTED
  switch (cmd) {
    case 400:  // MAV_CMD_COMPONENT_ARM_DISARM
      if (p[0] > 0.5f) {
        if (!bus::config.gcs_arm) res = 2;
        else bus::postRequest(bus::Request::ARM);
      } else {
        bus::postRequest(bus::Request::DISARM);
      }
      break;
    case 20: bus::postRequest(bus::Request::SET_MODE, (uint32_t)FlightMode::RTH); break;    // RETURN_TO_LAUNCH
    case 192: {  // DO_REPOSITION: p5 lat, p6 lon (deg), p7 alt (m above home); NaN = keep
      bus::GuidedTarget g{};
      g.lat = p[4] == p[4] ? (int32_t)(p[4] * 1e7) : 0;
      g.lon = p[5] == p[5] ? (int32_t)(p[5] * 1e7) : 0;
      g.alt = p[6];
      if (!st.armed) { res = 1; break; }
      bus::guided.publish(g);
      break;
    }
    case 22: {  // NAV_TAKEOFF (GUIDED): climb to p7 above home where we are
      if (!st.armed) { res = 1; break; }
      bus::GuidedTarget g{};
      g.alt = p[6] > 1.0f ? p[6] : 2.5f;
      g.takeoff = true;
      bus::guided.publish(g);
      break;
    }
    case 21: bus::postRequest(bus::Request::SET_MODE, (uint32_t)FlightMode::LAND); break;   // NAV_LAND
    case 300: bus::postRequest(bus::Request::SET_MODE, (uint32_t)FlightMode::MISSION); break;  // MISSION_START
    case 176:  // DO_SET_MODE (custom mode in param2)
      if (p[1] >= 0 && (p[1] < (float)FlightMode::TURTLE || p[1] == (float)FlightMode::CRUISE ||
                        p[1] == (float)FlightMode::GUIDED || p[1] == (float)FlightMode::SMART_RTL))
        bus::postRequest(bus::Request::SET_MODE, (uint32_t)p[1]);
      else res = 2;
      break;
    case 241:  // PREFLIGHT_CALIBRATION
      if (st.armed) { res = 1; break; }
      if (p[0] > 0.5f) bus::postRequest(bus::Request::GYRO_CAL);
      else if (p[1] > 0.5f) bus::postRequest(bus::Request::MAG_CAL, 30);
      else if (p[5] > 0.5f) bus::postRequest(bus::Request::COMPASSMOT);  // ArduPilot: param6 = compassmot
      else if (p[4] > 0.5f && p[4] < 1.5f) bus::postRequest(bus::Request::ACC_CAL, 6);  // 1 = full (6-point)
      else if (p[4] > 1.5f) bus::postRequest(bus::Request::ACC_CAL);                   // 2 / 4 = level only
      else res = 3;
      break;
    case 245:  // PREFLIGHT_REBOOT_SHUTDOWN
      if (st.armed || p[0] < 0.5f) { res = 2; break; }
      sendCommandAck(cmd, 0);
      hal::delayMs(20);
      hal::reboot();
      return;
    case 512:  // REQUEST_MESSAGE
      if ((uint32_t)p[0] == MSG_AUTOPILOT_VERSION) sendAutopilotVersion();
      else if ((uint32_t)p[0] == MSG_HOME_POSITION) sendHome();
      else res = 3;
      break;
    case 520: sendAutopilotVersion(); break;  // REQUEST_AUTOPILOT_CAPABILITIES
    case 511: break;                          // SET_MESSAGE_INTERVAL: accepted, fixed rates
    default: res = 3;                         // UNSUPPORTED
  }
  (void)nowMs;
  sendCommandAck(cmd, res);
}

void MavlinkLink::handle(const MavFrame& f, uint32_t now) {
  const uint8_t* p = f.payload;
  lastRxMs_ = now;
  if (f.sysid != bus::config.mav_sysid) { gcsSys_ = f.sysid; gcsComp_ = f.compid; }
  switch (f.msgid) {
    case MSG_PARAM_REQUEST_LIST:
      if (forUs(p[0])) paramStreamIdx_ = 0;
      break;
    case MSG_PARAM_REQUEST_READ: {
      if (!forUs(p[2])) break;
      int16_t idx = ri16(p);
      if (idx < 0) {
        char name[17];
        memcpy(name, p + 4, 16);
        name[16] = 0;
        idx = (int16_t)paramFind(name);
      }
      sendParam(idx);
      break;
    }
    case MSG_PARAM_SET: {
      if (!forUs(p[4])) break;
      char name[17];
      memcpy(name, p + 6, 16);
      name[16] = 0;
      int idx = paramFind(name);
      if (idx < 0) break;
      if (!bus::status.get().armed && paramSet(bus::config, idx, rf(p))) {
        bus::configChanged();
        paramDirty_ = true;
        paramDirtyMs_ = now;
      }
      sendParam(idx);  // echo the (possibly unchanged) value
      break;
    }
    case MSG_COMMAND_LONG: {
      if (!forUs(p[30])) break;
      float prm[7];
      for (int i = 0; i < 7; i++) prm[i] = rf(p + 4 * i);
      handleCommand(ru16(p + 28), prm, now);
      break;
    }
    case MSG_COMMAND_INT: {
      if (!forUs(p[30])) break;
      // x / y are degE7 integers: kept exact for DO_REPOSITION (a float holds only ~7 digits)
      uint16_t cmd = ru16(p + 28);
      if (cmd == 192) {
        if (!bus::status.get().armed) { sendCommandAck(cmd, 1); break; }
        bus::GuidedTarget g{};
        g.lat = ri32(p + 16); g.lon = ri32(p + 20); g.alt = rf(p + 24);
        bus::guided.publish(g);
        sendCommandAck(cmd, 0);
        break;
      }
      float prm[7] = {rf(p), rf(p + 4), rf(p + 8), rf(p + 12), (float)ri32(p + 16) * 1e-7f, (float)ri32(p + 20) * 1e-7f, rf(p + 24)};
      handleCommand(cmd, prm, now);
      break;
    }
    case MSG_SET_POSITION_TARGET_GLOBAL_INT:
    case MSG_SET_POSITION_TARGET_LOCAL_NED: {
      if (!forUs(p[50])) break;
      uint16_t mask = ru16(p + 48);
      if (mask & 0x7) break;  // position ignored: velocity / attitude targets are not supported
      VehicleStatus st = bus::status.get();
      if (!st.armed || st.mode != FlightMode::GUIDED) break;
      bus::GuidedTarget g{};
      if (f.msgid == MSG_SET_POSITION_TARGET_GLOBAL_INT) {
        g.lat = ri32(p + 4); g.lon = ri32(p + 8); g.alt = rf(p + 12);
      } else {
        g.local = true; g.n = rf(p + 4); g.e = rf(p + 8); g.alt = -rf(p + 12);
      }
      bus::guided.publish(g);
      break;
    }
    case MSG_SET_MODE:  // custom_mode u32, target_system u8, base_mode u8
      if (forUs(p[4]) && ri32(p) >= 0 && (ri32(p) < (int32_t)FlightMode::TURTLE || ri32(p) == (int32_t)FlightMode::CRUISE ||
                                          ri32(p) == (int32_t)FlightMode::GUIDED || ri32(p) == (int32_t)FlightMode::SMART_RTL))
        bus::postRequest(bus::Request::SET_MODE, (uint32_t)ri32(p));
      break;
    // ---- mission protocol ----
    case MSG_MISSION_REQUEST_LIST:
      if (forUs(p[0])) sendMissionCount(f.len > 2 ? p[2] : 0);
      break;
    case MSG_MISSION_REQUEST_INT:
    case MSG_MISSION_REQUEST:
      if (forUs(p[2])) sendMissionItem(ru16(p), f.len > 4 ? p[4] : 0);
      break;
    case MSG_MISSION_COUNT: {
      if (!forUs(p[2])) break;
      uint8_t mtype = f.len > 4 ? p[4] : 0;
      uint16_t n = ru16(p);
      VehicleStatus st = bus::status.get();
      if (mtype == 2) {  // rally points
        if (n > nav::RALLY_MAX) { sendMissionAck(4, 2); break; }
        if (n == 0) { nav::rallySetCount(0); nav::rallySave(); sendMissionAck(0, 2); break; }
      } else if (mtype == 1) {  // geofence polygon
        if (n > nav::FENCE_MAX) { sendMissionAck(4, 1); break; }
        if (n == 0) { nav::fenceSetCount(0); nav::fenceSave(); sendMissionAck(0, 1); break; }
      } else if (mtype != 0) {
        sendMissionAck(n == 0 ? 0 : 3, mtype);
        break;
      } else {
        if (st.armed && st.mode == FlightMode::MISSION) { sendMissionAck(14); break; }  // DENIED
        if (n > nav::MISSION_MAX) { sendMissionAck(4); break; }                        // NO_SPACE
        if (n == 0) { nav::clear(); nav::save(); sendMissionAck(0); break; }
      }
      upType_ = mtype;
      upCount_ = n; upNext_ = 0; upRetries_ = 0; upLastMs_ = now;
      upUseInt_ = true;
      sendMissionRequest(0);
      break;
    }
    case MSG_MISSION_ITEM_INT:
    case MSG_MISSION_ITEM: {
      if (!forUs(p[32]) || upCount_ < 0) break;
      uint16_t seq = ru16(p + 28);
      if (seq != upNext_) { sendMissionRequest((uint16_t)upNext_); break; }
      nav::MissionItem it{};
      it.param1 = rf(p); it.param2 = rf(p + 4); it.param3 = rf(p + 8); it.param4 = rf(p + 12);
      if (f.msgid == MSG_MISSION_ITEM_INT) { it.lat = ri32(p + 16); it.lon = ri32(p + 20); }
      else { it.lat = (int32_t)(rf(p + 16) * 1e7); it.lon = (int32_t)(rf(p + 20) * 1e7); upUseInt_ = false; }
      it.alt = rf(p + 24);
      it.cmd = ru16(p + 30);
      it.frame = p[34];
      it.autocontinue = p[36];
      bool okCmd = upType_ == 2 ? it.cmd == nav::CMD_RALLY_POINT
                 : upType_ == 1 ? it.cmd == nav::CMD_FENCE_POLYGON_INCLUSION : nav::supported(it.cmd);
      if (!okCmd) { sendMissionAck(3, upType_); upCount_ = -1; break; }   // UNSUPPORTED
      if (it.frame != 2 && it.frame != 3 && it.frame != 6) { sendMissionAck(2, upType_); upCount_ = -1; break; }
      if (upType_ == 2) nav::rallySet(seq, it);
      else if (upType_ == 1) nav::fenceSet(seq, it);
      else nav::set(seq, it);
      upNext_++;
      upRetries_ = 0;
      upLastMs_ = now;
      if (upNext_ >= upCount_) {
        if (upType_ == 2) {
          nav::rallySetCount(upCount_);
          nav::rallySave();
          bus::postMessage("Rally points received");
        } else if (upType_ == 1) {
          nav::fenceSetCount(upCount_);
          nav::fenceSave();
          bus::postMessage("Fence polygon received");
        } else {
          nav::setCount(upCount_);
          nav::save();
          bus::postMessage("Mission received");
        }
        upCount_ = -1;
        sendMissionAck(0, upType_);
      } else {
        sendMissionRequest((uint16_t)upNext_);
      }
      break;
    }
    case MSG_MISSION_CLEAR_ALL:
      if (!forUs(p[0])) break;
      if (f.len > 2 && p[2] == 2) { nav::rallySetCount(0); nav::rallySave(); }
      else if (f.len > 2 && p[2] == 1) { nav::fenceSetCount(0); nav::fenceSave(); }
      else { nav::clear(); nav::save(); }
      sendMissionAck(0, f.len > 2 ? p[2] : 0);
      break;
    case MSG_HEARTBEAT:
      if (f.len > 4 && p[4] == 6) bus::gcsHeartbeatMs = now ? now : 1;  // MAV_TYPE_GCS
      break;
    case MSG_MISSION_ACK:
    default:
      break;
  }
}

void MavlinkLink::update(uint32_t now, bool readPort) {
  if (readPort) {
    int c, budget = 512;
    while (budget-- > 0 && (c = hal::portRead(port_)) >= 0) feed((uint8_t)c, now);
  }
  if (streamOnlyWhenConnected_ && !seenRecently(now)) return;

  // mission upload retries
  if (upCount_ >= 0 && now - upLastMs_ > 1000) {
    if (++upRetries_ > 5) { upCount_ = -1; sendMissionAck(1, upType_); }
    else { upLastMs_ = now; sendMissionRequest((uint16_t)upNext_); }
  }
  // parameter streaming (a few per call, bounded by TX space)
  for (int k = 0; k < 6 && paramStreamIdx_ >= 0; k++) {
    if (hal::portWriteSpace(port_) < 40) break;
    sendParam(paramStreamIdx_++);
    if (paramStreamIdx_ >= paramCount()) paramStreamIdx_ = -1;
  }
  // persist parameter changes 1 s after the last PARAM_SET (only while disarmed)
  if (paramDirty_ && now - paramDirtyMs_ > 1000 && !bus::status.get().armed) {
    configSeal(bus::config);
    hal::storageSave(hal::Slot::CONFIG, &bus::config, sizeof(bus::config));
    paramDirty_ = false;
  }

  bool fast = port_ == hal::Port::UDP || port_ == hal::Port::CONSOLE;
  if (now - tHb_ >= 1000) { tHb_ = now; sendHeartbeat(); }
  if (now - tSys_ >= 500) { tSys_ = now; sendSysStatus(); }
  if (now - tAtt_ >= (fast ? 50u : 100u)) { tAtt_ = now; sendAttitude(now); }
  if (now - tPos_ >= 200) { tPos_ = now; sendPosition(now); }
  if (now - tGps_ >= 500) { tGps_ = now; sendGpsRaw(now); }
  if (now - tHud_ >= 250) { tHud_ = now; sendVfrHud(); }
  if (now - tRc_ >= 500) { tRc_ = now; sendRcChannels(now); }
  if (now - tHome_ >= 2000) { tHome_ = now; sendHome(); }
  if (now - tMc_ >= 1000) {
    tMc_ = now;
    VehicleStatus st = bus::status.get();
    W w;
    w.u16(st.missionIndex);
    send(MSG_MISSION_CURRENT, w.b, 2);
    if (st.mode == FlightMode::MISSION && st.missionIndex != lastReached_ && st.missionIndex > 0) {
      lastReached_ = st.missionIndex;
      W r;
      r.u16((uint16_t)(st.missionIndex - 1));
      send(MSG_MISSION_ITEM_REACHED, r.b, 2);
    }
  }
  if (bus::messageSeq() != msgSeq_) {
    msgSeq_ = bus::messageSeq();
    char m[64];
    bus::readMessage(m, sizeof(m));
    sendStatusText(m, 6);  // MAV_SEVERITY_INFO
  }
}

}  // namespace comms
