// AllQuad - high-rate gyro capture ("gyro burst") into the SD-card blackbox
//
// The regular blackbox records at bb_rate_hz (<= 1 kHz): noise above 500 Hz is invisible in it. For a few
// seconds after take-off (bb_gyro_s) the fast loop also hands every gyro sample (raw and filtered, up to
// 4 kHz) to a lock-free ring; the logger task packs them 16 at a time into 0xA8 frames between the
// records. One producer (fast loop), one consumer (logger task). SD card backend only (48 kB/s at 4 kHz).
#pragma once
#include <stdint.h>

#include "../common/math3d.h"

namespace logging {

constexpr int GYRO_FRAME_N = 16;
#pragma pack(push, 1)
struct GyroFrame {
  uint8_t sync;      // 0xA8
  uint8_t n;         // samples in the frame (GYRO_FRAME_N)
  uint32_t timeUs;   // time of the first sample
  uint16_t rateHz;   // sample rate
  int16_t s[GYRO_FRAME_N][6];  // raw x y z, filtered x y z, 0.1 deg/s
  uint8_t crc;       // xor of all previous bytes
};
#pragma pack(pop)

bool gyroTapAvailable();                   // false: no RAM for the ring on this board (F411)
void gyroTapConfigure(uint16_t loopHz);    // capture rate = loop rate, halved until <= 4 kHz
void gyroTapStart(uint32_t nowUs, uint32_t durMs);
void gyroTapStop();
bool gyroTapActive();
uint16_t gyroTapRate();
uint32_t gyroTapDropped();                 // samples lost (ring full: the card could not keep up)
// fast loop: one call per loop (returns at once when no capture runs)
void gyroTapPush(const fc::Vec3& rawDps, const fc::Vec3& filtDps, uint32_t nowUs);
// logger task: true + a sealed frame when 16 samples are waiting
bool gyroTapFrame(GyroFrame& f);

}  // namespace logging
