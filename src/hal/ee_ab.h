// AllQuad - settings image with a footer, kept in one or two flash copies (A/B). Pure functions: the
// STM32 storage (hal_stm32cube.cpp) uses them on the flash copies, the host tests on byte arrays.
// Image: EE_SIZE bytes, the last 16 are {magic, sequence, crc32 of the rest, ~magic}.
#pragma once
#include <stdint.h>
#include <string.h>

#include "../config/config.h"  // fc::crc32

namespace ee {
constexpr uint32_t SIZE = 8192, FOOT = SIZE - 16, MAGIC = 0x41514545u;  // "AQEE"
struct Foot { uint32_t magic, seq, crc, inv; };

inline bool valid(const uint8_t* img, uint32_t& seq) {
  Foot f;
  memcpy(&f, img + FOOT, sizeof(f));
  if (f.magic != MAGIC || f.inv != ~MAGIC || fc::crc32(img, FOOT) != f.crc) return false;
  seq = f.seq;
  return true;
}
// The copy to use: the valid one with the newest sequence (wraps at 2^32); none valid -> copy 0 read
// as it is (an image saved before the footer existed), seq 0.
inline int pick(const uint8_t* const* copies, int n, uint32_t& seq) {
  int best = -1;
  uint32_t bestSeq = 0;
  for (int i = 0; i < n; i++) {
    uint32_t q;
    if (valid(copies[i], q) && (best < 0 || (int32_t)(q - bestSeq) > 0)) { best = i; bestSeq = q; }
  }
  seq = best < 0 ? 0 : bestSeq;
  return best < 0 ? 0 : best;
}
// next sequence number and the footer for it; returns the copy to write (the one not in use)
inline int seal(uint8_t* img, int active, uint32_t activeSeq, int n, uint32_t& seq) {
  seq = activeSeq + 1 ? activeSeq + 1 : 1;
  Foot f = {MAGIC, seq, fc::crc32(img, FOOT), ~MAGIC};
  memcpy(img + FOOT, &f, sizeof(f));
  return n > 1 ? 1 - active : 0;
}
}  // namespace ee
