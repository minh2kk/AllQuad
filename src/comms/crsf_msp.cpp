#include "../common/size_opt.h"  // -Os on ESP32 (not flight critical)
#include "crsf_msp.h"

#include <string.h>

namespace comms {

bool CrsfMsp::feed(const uint8_t* d, size_t n, uint8_t origin, MspFrame& out) {
  if (n < 1) return false;
  uint8_t st = d[0], ver = (uint8_t)((st >> 5) & 3), seq = (uint8_t)(st & 0x0F);
  size_t i = 1;
  if (st & 0x10) {  // first chunk of a request
    rxActive_ = false;
    if (ver == 1) {
      if (n < 3) return false;
      rx_.v2 = false;
      rx_.len = d[1];
      rx_.cmd = d[2];
      i = 3;
    } else if (ver == 2) {
      if (n < 6) return false;
      rx_.v2 = true;
      rx_.cmd = (uint16_t)(d[2] | (d[3] << 8));
      rx_.len = (uint16_t)(d[4] | (d[5] << 8));
      i = 6;
    } else {
      return false;
    }
    if (rx_.len > sizeof(rx_.payload)) return false;
    rx_.dir = '<';
    rxGot_ = 0;
    rxVer_ = ver;
    rxSeq_ = seq;
    rxActive_ = true;
  } else {
    // a continuation must follow the previous chunk of the same request
    if (!rxActive_ || ver != rxVer_ || seq != ((rxSeq_ + 1) & 0x0F)) { rxActive_ = false; return false; }
    rxSeq_ = seq;
  }
  while (i < n && rxGot_ < rx_.len) rx_.payload[rxGot_++] = d[i++];
  if (rxGot_ < rx_.len) return false;  // more chunks to come (a v1 checksum byte after the data is ignored)
  rxActive_ = false;
  out = rx_;
  txDest_ = origin;
  return true;
}

void CrsfMsp::respond(const MspFrame& req, const uint8_t* p, uint16_t len, bool error) {
  if (len > 256) len = 256;
  uint16_t k = 0;
  if (!req.v2) {
    uint8_t chk = (uint8_t)(len ^ (uint8_t)req.cmd);
    tx_[k++] = (uint8_t)len;
    tx_[k++] = (uint8_t)req.cmd;
    for (uint16_t j = 0; j < len; j++) { tx_[k++] = p[j]; chk ^= p[j]; }
    tx_[k++] = chk;
  } else {
    tx_[k++] = 0;  // flags
    tx_[k++] = (uint8_t)req.cmd;
    tx_[k++] = (uint8_t)(req.cmd >> 8);
    tx_[k++] = (uint8_t)len;
    tx_[k++] = (uint8_t)(len >> 8);
    if (len) memcpy(tx_ + k, p, len);
    k = (uint16_t)(k + len);
  }
  txVer_ = req.v2 ? 2 : 1;
  txError_ = error;
  txLen_ = k;
  txPos_ = 0;
}

size_t CrsfMsp::nextFrame(uint8_t* f) {
  if (!pending()) return 0;
  bool start = txPos_ == 0;
  uint16_t n = (uint16_t)((size_t)(txLen_ - txPos_) < RESP_CHUNK - 1 ? txLen_ - txPos_ : RESP_CHUNK - 1);
  size_t k = 0;
  f[k++] = ADDR_FC;           // sync
  f[k++] = (uint8_t)(n + 5);  // type + dest + origin + status + data + crc
  f[k++] = TYPE_RESP;
  f[k++] = txDest_;
  f[k++] = ADDR_FC;
  f[k++] = (uint8_t)((txError_ ? 0x80 : 0) | (txVer_ << 5) | (start ? 0x10 : 0) | (txSeq_ & 0x0F));
  txSeq_ = (uint8_t)((txSeq_ + 1) & 0x0F);
  memcpy(f + k, tx_ + txPos_, n);
  k += n;
  txPos_ = (uint16_t)(txPos_ + n);
  uint8_t crc = 0;
  for (size_t j = 2; j < k; j++) crc = crc8DvbS2(crc, f[j]);
  f[k++] = crc;
  return k;
}

}  // namespace comms
