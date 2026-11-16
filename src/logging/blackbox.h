// AllQuad - blackbox flight recorder
// Fixed-size binary records (little-endian) behind a text header; decode with
// tools/blackbox_decode.py. Recording runs while armed.
// Backends: 1 serial port with function BLACKBOX (OpenLog), 2 onboard flash, 3 SD card (logging/sdlog).
#pragma once
#include <stdint.h>

#include "../hal/hal.h"

namespace logging {

#pragma pack(push, 1)
struct Record {
  uint8_t sync;          // 0xA5
  uint8_t len;           // sizeof(Record)
  uint32_t timeUs;
  int16_t gyro[3];       // 0.1 deg/s (filtered)
  int16_t gyroRaw[3];    // 0.1 deg/s
  int16_t setpoint[3];   // 0.1 deg/s
  int16_t pidP[3], pidI[3], pidD[3], pidF[3];  // PID-sum units x10
  uint16_t motor[4];     // 0..1000
  uint16_t throttle;     // 0..1000
  uint16_t rc[4];        // us
  int16_t att[3];        // 0.1 deg
  int32_t altCm;
  int16_t vzCms;
  int32_t posN, posE;    // cm
  uint16_t vbatCv;
  int16_t currentCa;
  uint8_t mode, navPhase, flags;  // flags: 1 armed, 2 failsafe, 4 landed
  uint8_t crc;           // xor of all previous bytes
};
// ESC frame (0xA7, "H esc_frame 23"): per-motor RPM / temperature from bidirectional DShot, EDT or
// KISS / BLHeli telemetry, ~50 Hz while any ESC reports. Older readers skip it (unknown sync byte).
struct EscRecord {
  uint8_t sync;        // 0xA7
  uint8_t len;         // sizeof(EscRecord)
  uint32_t timeUs;
  uint16_t rpm10[4];   // mechanical RPM / 10, 0xFFFF = no data
  uint8_t tempC[4];    // 255 = no data
  uint16_t voltCv;     // ESC-reported voltage x100, 0 = none
  int16_t currentDa;   // total ESC current x10, -32768 = none
  uint8_t crc;         // xor of all previous bytes
};
#pragma pack(pop)

class Blackbox {
 public:
  void begin(uint8_t backend, hal::Port port, uint16_t rateHz);
  void update(uint32_t nowUs);   // call at rateHz
  bool active() const { return active_; }
  uint32_t bytesWritten() const { return bytes_; }
  static void fill(Record& r, uint32_t nowUs);
  static bool fillEsc(EscRecord& r, uint32_t nowUs);  // false: no ESC reports anything
  // Delta frame (0xA6): checksum byte of the record it builds on (so a lost full record is noticed),
  // changed-field bitmask, zig-zag varint differences to that record, xor checksum. A full record (0xA5) every KEY_EVERY records lets a reader resync after damage.
  // ~3-4x smaller than full records. Returns the frame length.
  static constexpr int KEY_EVERY = 32;
  static constexpr int FIELDS = 43;
  static constexpr int MAX_DELTA = 3 + (FIELDS + 7) / 8 + FIELDS * 5;
  static size_t encodeDelta(const Record& prev, const Record& cur, uint8_t* out);

 private:
  bool write(const uint8_t* d, size_t n);  // false: (partly) dropped - flash full / busy, port buffer full
  void writeHeader();
  uint8_t backend_ = 0;
  hal::Port port_ = hal::Port::SERIAL3;
  bool active_ = false, wasArmed_ = false;
  uint32_t bytes_ = 0;
  Record prev_;
  uint32_t count_ = 0;
  uint16_t escEvery_ = 10, escN_ = 0;  // records between ESC frames (~50 Hz)
};

}  // namespace logging
