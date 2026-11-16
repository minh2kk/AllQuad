#include "rx_parser.h"

#include <string.h>


namespace drv {

using fc::RxProtocol;

uint8_t crsfCrc8(const uint8_t* data, uint8_t len) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
  }
  return crc;
}

void RxParser::begin(RxProtocol p) {
  proto_ = p;
  pos_ = 0;
  rc_ = fc::RcInput{};
  for (int i = 0; i < fc::RC_MAX_CH; i++) rc_.ch[i] = 1500;
  rc_.ch[fc::RC_THROTTLE] = 1000;
}

uint32_t RxParser::baud() const {
  switch (proto_) {
    case RxProtocol::CRSF: return 420000;
    case RxProtocol::SBUS: return 100000;
    case RxProtocol::IBUS: return 115200;
  }
  return 115200;
}

bool RxParser::feed(uint8_t b, uint32_t nowMs) {
  // a gap > 3 ms means a new frame starts
  if (nowMs - lastByteMs_ > 3) pos_ = 0;
  lastByteMs_ = nowMs;
  switch (proto_) {
    case RxProtocol::CRSF: return crsfByte(b, nowMs);
    case RxProtocol::SBUS: return sbusByte(b, nowMs);
    case RxProtocol::IBUS: return ibusByte(b, nowMs);
  }
  return false;
}

// 16 x 11-bit little-endian packed channels (CRSF and SBUS use the same packing)
static void unpack11(const uint8_t* p, uint16_t out[16]) {
  uint32_t acc = 0;
  int bits = 0, idx = 0;
  for (int i = 0; i < 22 && idx < 16; i++) {
    acc |= (uint32_t)p[i] << bits;
    bits += 8;
    while (bits >= 11 && idx < 16) {
      out[idx++] = acc & 0x7FF;
      acc >>= 11;
      bits -= 11;
    }
  }
}

// ---------------------------------------------------------------- CRSF
bool RxParser::crsfByte(uint8_t b, uint32_t nowMs) {
  if (pos_ == 0 && b != 0xC8 && b != 0xEE && b != 0xEA) return false;
  buf_[pos_++] = b;
  if (pos_ == 2 && (buf_[1] < 2 || buf_[1] > 62)) { pos_ = 0; return false; }
  if (pos_ < 2 || pos_ < buf_[1] + 2) return false;
  uint8_t len = buf_[1];
  pos_ = 0;
  if (crsfCrc8(buf_ + 2, len - 1) != buf_[len + 1]) return false;
  uint8_t type = buf_[2];
  const uint8_t* pl = buf_ + 3;
  if (type == 0x16 && len >= 24) {  // RC_CHANNELS_PACKED
    uint16_t raw[16];
    unpack11(pl, raw);
    for (int i = 0; i < 16; i++) rc_.ch[i] = (uint16_t)(raw[i] * 0.62477120f + 881.0f);
    rc_.count = 16;
    rc_.lastFrameMs = nowMs;
    rc_.everReceived = true;
    rc_.failsafeFlag = false;
    return true;
  }
  if (type == 0x14 && len >= 12) {  // LINK_STATISTICS
    rc_.rssiDbm = -(int8_t)pl[0];
    rc_.linkQuality = pl[2];
  }
  return false;
}

size_t RxParser::crsfFrame(uint8_t type, const uint8_t* payload, uint8_t len, uint8_t* f) {
  f[0] = 0xC8;
  f[1] = len + 2;
  f[2] = type;
  memcpy(f + 3, payload, len);
  f[3 + len] = crsfCrc8(f + 2, len + 1);
  return len + 4;
}

static void put16(uint8_t* p, int32_t v) { p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t)v; }
static void put32(uint8_t* p, int32_t v) {
  p[0] = (uint8_t)(v >> 24); p[1] = (uint8_t)(v >> 16); p[2] = (uint8_t)(v >> 8); p[3] = (uint8_t)v;
}

size_t RxParser::buildTelemetry(const RxTelemetry& t, uint8_t* out) {
  if (proto_ != RxProtocol::CRSF) return 0;
  uint8_t p[24];
  switch (telemSlot_++ % 5) {
    case 0:  // attitude (rad * 10000)
      put16(p + 0, (int32_t)(t.pitchRad * 10000.0f));
      put16(p + 2, (int32_t)(t.rollRad * 10000.0f));
      put16(p + 4, (int32_t)(t.yawRad * 10000.0f));
      return crsfFrame(0x1E, p, 6, out);
    case 1: {  // battery
      put16(p + 0, (int32_t)(t.vbat * 10.0f + 0.5f));
      put16(p + 2, (int32_t)(t.current * 10.0f + 0.5f));
      uint32_t mah = (uint32_t)t.mah;
      p[4] = (uint8_t)(mah >> 16); p[5] = (uint8_t)(mah >> 8); p[6] = (uint8_t)mah;
      p[7] = t.batteryPct;
      return crsfFrame(0x08, p, 8, out);
    }
    case 2: {  // flight mode
      uint8_t n = (uint8_t)strnlen(t.mode, 14);
      memcpy(p, t.mode, n);
      p[n] = 0;
      return crsfFrame(0x21, p, n + 1, out);
    }
    case 3:  // vario (cm/s)
      put16(p, (int32_t)(t.vario * 100.0f));
      return crsfFrame(0x07, p, 2, out);
    case 4:  // GPS
      if (!t.gpsValid) return 0;
      put32(p + 0, (int32_t)(t.lat * 1e7));
      put32(p + 4, (int32_t)(t.lon * 1e7));
      put16(p + 8, (int32_t)(t.groundSpeed * 36.0f));  // km/h * 10
      put16(p + 10, (int32_t)(t.courseDeg * 100.0f));
      put16(p + 12, (int32_t)(t.altitude + 1000.0f));
      p[14] = t.sats;
      return crsfFrame(0x02, p, 15, out);
  }
  return 0;
}

// ---------------------------------------------------------------- SBUS
bool RxParser::sbusByte(uint8_t b, uint32_t nowMs) {
  if (pos_ == 0 && b != 0x0F) return false;
  buf_[pos_++] = b;
  if (pos_ < 25) return false;
  pos_ = 0;
  uint8_t end = buf_[24];
  if (end != 0x00 && (end & 0x0F) != 0x04) return false;
  uint16_t raw[16];
  unpack11(buf_ + 1, raw);
  for (int i = 0; i < 16; i++) rc_.ch[i] = (uint16_t)(raw[i] * 0.625f + 880.0f);
  rc_.count = 16;
  uint8_t flags = buf_[23];
  rc_.failsafeFlag = (flags & 0x08) != 0;
  if (!rc_.failsafeFlag) {
    rc_.lastFrameMs = nowMs;
    rc_.everReceived = true;
  }
  return !rc_.failsafeFlag;
}

// ---------------------------------------------------------------- iBUS
bool RxParser::ibusByte(uint8_t b, uint32_t nowMs) {
  if (pos_ == 0 && b != 0x20) return false;
  if (pos_ == 1 && b != 0x40) { pos_ = 0; return false; }
  buf_[pos_++] = b;
  if (pos_ < 32) return false;
  pos_ = 0;
  uint16_t sum = 0xFFFF;
  for (int i = 0; i < 30; i++) sum -= buf_[i];
  if (sum != (uint16_t)(buf_[30] | (buf_[31] << 8))) return false;
  for (int i = 0; i < 14; i++) rc_.ch[i] = (uint16_t)(buf_[2 + i * 2] | (buf_[3 + i * 2] << 8));
  rc_.count = 14;
  rc_.lastFrameMs = nowMs;
  rc_.everReceived = true;
  rc_.failsafeFlag = false;
  return true;
}

}  // namespace drv
