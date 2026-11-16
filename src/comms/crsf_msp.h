// AllQuad - MSP over CRSF (the radio's Lua scripts reach the FC through the ELRS / Crossfire link).
// CRSF extended frames: 0x7A MSP request, 0x7C MSP write (radio -> FC), 0x7B MSP response (FC -> radio),
// payload = [dest][origin][chunk]. A chunk starts with a status byte (Betaflight msp_shared layout):
//   bit 7 error (responses), bits 5-6 MSP version (1 = v1, 2 = v2), bit 4 start of a message, bits 0-3 sequence
// start chunk, v1: [status][size][cmd][data...]   (+ an xor checksum after the data, not required here)
// start chunk, v2: [status][flags][cmd lo][cmd hi][size lo][size hi][data...]
// later chunks:    [status][data...]
// The CRSF frame has its own CRC. Requests are answered by the same MspLink code as on a serial port.
#pragma once
#include <stddef.h>
#include <stdint.h>

#include "msp.h"

namespace comms {

class CrsfMsp {
 public:
  static constexpr uint8_t ADDR_FC = 0xC8, ADDR_RADIO = 0xEA;
  static constexpr uint8_t TYPE_REQ = 0x7A, TYPE_RESP = 0x7B, TYPE_WRITE = 0x7C;
  static constexpr size_t RESP_CHUNK = 56;  // response chunk incl. status (frame: 4 + 2 + chunk <= 64)

  // one request chunk (frame payload after dest / origin); true when a whole request is assembled in out
  bool feed(const uint8_t* d, size_t n, uint8_t origin, MspFrame& out);
  // the answer to the request (error = command not supported / refused): sent as response chunks
  void respond(const MspFrame& req, const uint8_t* p, uint16_t len, bool error);
  // next 0x7B frame (complete CRSF frame with CRC) to put on the receiver port, 0 = nothing to send
  size_t nextFrame(uint8_t* out);
  bool pending() const { return txPos_ < txLen_; }

 private:
  // request being assembled
  uint8_t rxVer_ = 0, rxSeq_ = 0;
  bool rxActive_ = false;
  MspFrame rx_{};
  uint16_t rxGot_ = 0;
  // response being sent
  uint8_t tx_[4 + 256 + 8];
  uint16_t txLen_ = 0, txPos_ = 0;
  uint8_t txVer_ = 1, txSeq_ = 0, txDest_ = ADDR_RADIO;
  bool txError_ = false;
};

}  // namespace comms
