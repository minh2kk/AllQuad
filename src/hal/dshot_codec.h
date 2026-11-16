// AllQuad - DShot frame encoding and bidirectional DShot (RPM telemetry) decoding.
// Pure functions shared by the ESP32 (RMT) and STM32 (GPIO bit-bang) HALs, the simulator and the tests.
//
// Bidirectional DShot (Betaflight "dshot_bidir"): the FC sends an *inverted* frame (idle high, the
// 4-bit checksum inverted). About 30 us after the frame the ESC answers on the same wire with 21 bits
// at 5/4 of the DShot bit rate:
//   12-bit value  eee mmmmmmmmm  (period of one electrical revolution = m << e microseconds)
//   + 4-bit checksum (xor of all nibbles == 0xF), GCR-encoded 16 -> 20 bits,
//   sent as transitions: a 1 is a level change ("NRZI"), preceded by the start transition (falling edge).
// eRPM = 60e6 / period. Mechanical RPM = eRPM / (motor poles / 2).
#pragma once
#include <math.h>
#include <stdint.h>

namespace dshot {

// 16-bit frame: 11-bit value, telemetry-request bit, 4-bit checksum (inverted for bidirectional DShot)
inline uint16_t frame(uint16_t value, bool telemetry, bool bidir) {
  uint16_t p = (uint16_t)((value << 1) | (telemetry ? 1 : 0));
  uint16_t csum = (uint16_t)(p ^ (p >> 4) ^ (p >> 8));
  if (bidir) csum = (uint16_t)~csum;
  return (uint16_t)((p << 4) | (csum & 0x0F));
}

static const uint8_t GCR_ENC[16] = {0x19, 0x1B, 0x12, 0x13, 0x1D, 0x15, 0x16, 0x17,
                                    0x1A, 0x09, 0x0A, 0x0B, 0x1E, 0x0D, 0x0E, 0x0F};
inline int gcrDecode5(uint32_t q) {
  for (int i = 0; i < 16; i++)
    if (GCR_ENC[i] == q) return i;
  return -1;
}

// ---- encoder (ESC side: used by the simulator and the tests) ----
inline uint16_t valueFromErpm(float erpm) {
  if (erpm < 1.0f) return 0x0FFF;  // stopped
  uint32_t period = (uint32_t)(60000000.0f / erpm + 0.5f);
  uint32_t e = 0;
  while ((period >> e) > 511 && e < 7) e++;
  uint32_t m = period >> e;
  if (m > 511) m = 511;
  return (uint16_t)((e << 9) | m);
}
inline uint16_t withChecksum(uint16_t v12) {
  uint16_t c = (uint16_t)(v12 ^ (v12 >> 4) ^ (v12 >> 8));
  return (uint16_t)((v12 << 4) | (~c & 0x0F));
}
inline uint32_t gcrEncode(uint16_t v16) {
  uint32_t g = 0;
  for (int i = 3; i >= 0; i--) g = (g << 5) | GCR_ENC[(v16 >> (i * 4)) & 0xF];
  return g;
}
// run lengths (in telemetry bits) of the answer, first run = low after the start edge.
// The last run continues into the idle level. Returns the number of runs.
inline int runsFromGcr(uint32_t gcr20, uint8_t* runs) {
  uint32_t t = (1u << 20) | gcr20;  // transitions: start + 20 GCR bits, MSB first
  int n = 0, len = 0;
  for (int b = 20; b >= 0; b--) {
    if ((t >> b) & 1) {
      if (b != 20) runs[n++] = (uint8_t)len;
      len = 1;
    } else {
      len++;
    }
  }
  runs[n++] = (uint8_t)len;  // ends at the bit-21 boundary; the line then stays at this level
  return n;
}

// ---- decoder (FC side) ----
// Extended DShot Telemetry (EDT: AM32, Bluejay, BLHeli_32), switched on by DShot command 13: the ESC
// slips status frames in between the eRPM frames. The 12-bit value is eee m mmmm mmmm; eRPM periods
// are always sent normalised (bit 8 set whenever the exponent is not 0), so exponent != 0 with bit 8
// clear marks an EDT frame: type = bits 11-8, payload = bits 7-0.
struct Edt {
  float tempC = -1;   // type 0x2, degrees C
  float volts = -1;   // type 0x4, 0.25 V per step
  float amps = -1;    // type 0x6, 1 A per step
  uint8_t stress = 0; // type 0xC (AM32: demag / desync stress level)
  uint8_t status = 0; // type 0xE (bit 7 alert, bit 6 warning, bit 5 error, bits 3-0 max stress)
  uint32_t frames = 0;
};
enum Kind : uint8_t { BAD = 0, ERPM = 1, EDT = 2 };

inline uint8_t classify(uint32_t v12, float& erpm, Edt* edt) {
  if (v12 == 0x0FFF) { erpm = 0; return ERPM; }
  if (!(v12 & 0x100) && (v12 & 0xE00)) {
    uint8_t d = (uint8_t)(v12 & 0xFF);
    if (edt) {
      switch ((v12 >> 8) & 0xF) {
        case 0x2: edt->tempC = d; break;
        case 0x4: edt->volts = d * 0.25f; break;
        case 0x6: edt->amps = d; break;
        case 0xC: edt->stress = d; break;
        case 0xE: edt->status = d; break;
        default: break;  // 0x8 / 0xA: debug values
      }
      edt->frames++;
    }
    return EDT;
  }
  uint32_t period = (v12 & 0x1FF) << (v12 >> 9);
  if (!period) return BAD;
  erpm = 60000000.0f / (float)period;
  return ERPM;
}

// transitions (21 bits: start + GCR) -> eRPM or EDT; BAD when the checksum or the GCR code is wrong
inline uint8_t decodeTransitions(uint32_t t21, float& erpm, Edt* edt) {
  if (!((t21 >> 20) & 1)) return BAD;
  uint32_t g = t21 & 0xFFFFF;
  uint32_t v = 0;
  for (int i = 3; i >= 0; i--) {
    int nib = gcrDecode5((g >> (i * 5)) & 0x1F);
    if (nib < 0) return BAD;
    v = (v << 4) | (uint32_t)nib;
  }
  uint32_t c = v ^ (v >> 4) ^ (v >> 8) ^ (v >> 12);
  if ((c & 0xF) != 0xF) return BAD;
  return classify(v >> 4, erpm, edt);
}
inline bool erpmFromTransitions(uint32_t t21, float& erpm) { return decodeTransitions(t21, erpm, nullptr) == ERPM; }

// Line levels as alternating runs (level of the first run given, durations in any unit).
// bit = telemetry bit length in the same unit. The FC's own frame may precede the answer: everything
// before the first long high run (> 4 bits: the gap before the answer, or idle) is skipped.
inline uint8_t decodeRunsEx(const uint8_t* level, const float* dur, int n, float bit, float& erpm, Edt* edt) {
  int i = 0;
  while (i < n && !(level[i] && dur[i] > 4.0f * bit)) i++;
  i++;  // first run of the answer (low)
  if (i >= n || level[i]) return BAD;
  uint32_t t = 0;
  int bits = 0;
  for (; i < n && bits < 21; i++) {
    int len = (int)floorf(dur[i] / bit + 0.5f);
    if (len < 1) len = 1;
    if (bits + len > 21) len = 21 - bits;  // final run merges into the idle level
    t = (t << len) | (1u << (len - 1));
    bits += len;
  }
  if (bits < 21) t <<= (21 - bits);  // capture ended in the last level: no further transitions
  return decodeTransitions(t, erpm, edt);
}
inline bool decodeRuns(const uint8_t* level, const float* dur, int n, float bit, float& erpm) {
  return decodeRunsEx(level, dur, n, bit, erpm, nullptr) == ERPM;
}

// STM32 bit-bang input: GPIO port samples (IDR), one pin = mask; samplesPerBit = oversampling.
inline uint8_t decodeSamplesEx(const uint16_t* s, int n, uint16_t mask, float samplesPerBit, float& erpm, Edt* edt) {
  uint8_t lv[48];
  float du[48];
  int k = 0;
  uint8_t cur = (s[0] & mask) ? 1 : 0;
  int len = 0;
  for (int j = 0; j < n; j++) {
    uint8_t l = (s[j] & mask) ? 1 : 0;
    if (l != cur) {
      if (k >= 47) break;
      lv[k] = cur; du[k++] = (float)len;
      cur = l;
      len = 0;
    }
    len++;
  }
  lv[k] = cur; du[k++] = (float)len + 8.0f * samplesPerBit;  // trailing level runs into idle
  // the samples start right after the FC frame: the first run is the gap (idle high)
  if (lv[0] && du[0] < 4.5f * samplesPerBit) du[0] = 4.5f * samplesPerBit;
  return decodeRunsEx(lv, du, k, samplesPerBit, erpm, edt);
}
inline bool decodeSamples(const uint16_t* s, int n, uint16_t mask, float samplesPerBit, float& erpm) {
  return decodeSamplesEx(s, n, mask, samplesPerBit, erpm, nullptr) == ERPM;
}

// ESC side of EDT (simulator / tests): 12-bit value of a status frame
inline uint16_t edtValue(uint8_t type, uint8_t payload) { return (uint16_t)(((type & 0xF) << 8) | payload); }

}  // namespace dshot
