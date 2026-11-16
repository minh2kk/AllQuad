// AllQuad - byte-stream parsers for serial sensors (pure, no hardware access)
//   TfminiParser : Benewake TFmini / TFmini-S / TF-Luna / TF02 (9-byte frames, 115200)
//   EscTelemetryParser : KISS / BLHeli32 / AM32 serial telemetry (10-byte frames, 115200)
#pragma once
#include <stddef.h>
#include <stdint.h>

namespace drv {

class TfminiParser {
 public:
  // returns true when a frame was decoded; distM < 0 means "no valid target"
  bool feed(uint8_t b) {
    if (pos_ == 0 && b != 0x59) return false;
    if (pos_ == 1 && b != 0x59) { pos_ = 0; return false; }
    buf_[pos_++] = b;
    if (pos_ < 9) return false;
    pos_ = 0;
    uint8_t sum = 0;
    for (int i = 0; i < 8; i++) sum += buf_[i];
    if (sum != buf_[8]) return false;
    uint16_t distCm = (uint16_t)(buf_[2] | (buf_[3] << 8));
    strength = (uint16_t)(buf_[4] | (buf_[5] << 8));
    bool ok = strength >= 100 && strength != 0xFFFF && distCm > 0 && distCm < 4000;
    distM = ok ? distCm * 0.01f : -1.0f;
    return true;
  }
  float distM = -1.0f;
  uint16_t strength = 0;

 private:
  uint8_t buf_[9];
  uint8_t pos_ = 0;
};

class EscTelemetryParser {
 public:
  static uint8_t crc8(const uint8_t* d, size_t n) {
    uint8_t crc = 0;
    for (size_t i = 0; i < n; i++) {
      uint8_t c = crc ^ d[i];
      for (int b = 0; b < 8; b++) c = (c & 0x80) ? (uint8_t)(0x07 ^ (c << 1)) : (uint8_t)(c << 1);
      crc = c;
    }
    return crc;
  }
  void reset() { pos_ = 0; }
  // Frames are only expected after a telemetry request, so the caller resets
  // the parser when it requests the next motor.
  bool feed(uint8_t b) {
    buf_[pos_++] = b;
    if (pos_ < 10) return false;
    pos_ = 0;
    if (crc8(buf_, 9) != buf_[9]) return false;
    tempC = buf_[0];
    voltage = ((buf_[1] << 8) | buf_[2]) * 0.01f;
    current = ((buf_[3] << 8) | buf_[4]) * 0.01f;
    mah = (float)((buf_[5] << 8) | buf_[6]);
    erpm = ((buf_[7] << 8) | buf_[8]) * 100.0f;
    return true;
  }
  float tempC = 0, voltage = 0, current = 0, mah = 0, erpm = 0;

 private:
  uint8_t buf_[10];
  uint8_t pos_ = 0;
};

}  // namespace drv
