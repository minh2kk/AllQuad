#include "gyro_tap.h"

#include <string.h>

#include "../common/cycles.h"

namespace logging {

#if defined(STM32F411xE)
constexpr uint32_t RING = 0;     // 128 kB RAM board: no room for the ring
#else
constexpr uint32_t RING = 1024;  // samples (12 kB): ~250 ms at 4 kHz before the logger must drain it
#endif

namespace {
struct Sample { int16_t v[6]; };
constexpr uint32_t STAMPS = RING ? RING / GYRO_FRAME_N : 1;
Sample ring[RING ? RING : 1];
uint32_t stamp[STAMPS];                 // time of the first sample of every 16 (written by the fast loop)
volatile uint32_t head = 0, tail = 0;   // head: fast loop only, tail: logger only (free-running counters)
volatile uint32_t base = 0;             // head when the capture started: frames count from here
volatile bool active = false;
volatile uint32_t endUs = 0, dropped = 0;
uint16_t rate = 0;
uint8_t div = 1, divN = 0;

inline int16_t q(float v) {
  float x = v * 10.0f;
  return (int16_t)(x > 32767.0f ? 32767 : x < -32767.0f ? -32767 : x);
}
}  // namespace

bool gyroTapAvailable() { return RING > 0; }

void gyroTapConfigure(uint16_t loopHz) {
  div = 1;
  while (loopHz / div > 4000) div++;
  rate = (uint16_t)(loopHz / div);
}

void gyroTapStart(uint32_t nowUs, uint32_t durMs) {
  if (!RING || !rate || active) return;
  // whole frames only: anything left from an earlier capture is dropped
  uint32_t h = __atomic_load_n(&head, __ATOMIC_ACQUIRE);
  tail = h;
  base = h;
  endUs = nowUs + durMs * 1000u;
  divN = 0;
  __atomic_store_n(&active, true, __ATOMIC_RELEASE);
}
void gyroTapStop() { active = false; }
bool gyroTapActive() { return active; }
uint16_t gyroTapRate() { return rate; }
uint32_t gyroTapDropped() { return dropped; }

FAST_CODE void gyroTapPush(const fc::Vec3& r, const fc::Vec3& f, uint32_t nowUs) {
  if (!RING || !active) return;
  if ((int32_t)(nowUs - endUs) >= 0) { active = false; return; }
  if (++divN < div) return;
  divN = 0;
  uint32_t h = head;
  if (h - __atomic_load_n(&tail, __ATOMIC_ACQUIRE) >= RING - GYRO_FRAME_N) { dropped++; return; }
  Sample& s = ring[h % RING];
  s.v[0] = q(r.x); s.v[1] = q(r.y); s.v[2] = q(r.z);
  s.v[3] = q(f.x); s.v[4] = q(f.y); s.v[5] = q(f.z);
  uint32_t k = h - base;
  if (k % GYRO_FRAME_N == 0) stamp[(k / GYRO_FRAME_N) % STAMPS] = nowUs;
  __atomic_store_n(&head, h + 1, __ATOMIC_RELEASE);
}

bool gyroTapFrame(GyroFrame& fr) {
  if (!RING) return false;
  uint32_t t = tail, h = __atomic_load_n(&head, __ATOMIC_ACQUIRE);
  if (h - t < (uint32_t)GYRO_FRAME_N) return false;
  fr.sync = 0xA8;
  fr.n = GYRO_FRAME_N;
  fr.rateHz = rate;
  fr.timeUs = stamp[((t - base) / GYRO_FRAME_N) % STAMPS];
  for (int k = 0; k < GYRO_FRAME_N; k++) memcpy(fr.s[k], ring[(t + k) % RING].v, sizeof(fr.s[k]));
  __atomic_store_n(&tail, t + GYRO_FRAME_N, __ATOMIC_RELEASE);
  const uint8_t* p = (const uint8_t*)&fr;
  uint8_t x = 0;
  for (size_t i = 0; i < sizeof(fr) - 1; i++) x ^= p[i];
  fr.crc = x;
  return true;
}

}  // namespace logging
