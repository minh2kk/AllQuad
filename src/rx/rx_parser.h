// AllQuad - RC receiver: CRSF (ExpressLRS / Crossfire, with telemetry), SBUS, iBUS
#pragma once
#include <stddef.h>
#include "../bus/topics.h"
#include "../config/config.h"

namespace drv {

struct RxTelemetry {
  float pitchRad, rollRad, yawRad;
  float vbat, current, mah;
  uint8_t batteryPct;
  float vario;          // m/s
  const char* mode;     // e.g. "ANGLE" (append '*' when disarmed)
  bool gpsValid;
  double lat, lon;
  float groundSpeed;    // m/s
  float courseDeg;
  float altitude;       // m
  uint8_t sats;
};

class RxParser {
 public:
  void begin(fc::RxProtocol p);
  fc::RxProtocol protocol() const { return proto_; }
  // UART settings for the protocol
  uint32_t baud() const;
  bool needs8E2() const { return proto_ == fc::RxProtocol::SBUS; }
  bool inverted() const { return proto_ == fc::RxProtocol::SBUS; }
  // Feed one received byte; returns true when a new channel frame was decoded
  bool feed(uint8_t b, uint32_t nowMs);
  const fc::RcInput& rc() const { return rc_; }
  // CRSF only: builds the next telemetry frame (rotating) into out[], returns its length (0 = none)
  size_t buildTelemetry(const RxTelemetry& t, uint8_t* out);

 private:
  bool crsfByte(uint8_t b, uint32_t nowMs);
  bool sbusByte(uint8_t b, uint32_t nowMs);
  bool ibusByte(uint8_t b, uint32_t nowMs);
  size_t crsfFrame(uint8_t type, const uint8_t* payload, uint8_t len, uint8_t* out);

  fc::RxProtocol proto_ = fc::RxProtocol::CRSF;
  fc::RcInput rc_;
  uint8_t buf_[64];
  uint8_t pos_ = 0;
  uint32_t lastByteMs_ = 0;
  uint8_t telemSlot_ = 0;
};

uint8_t crsfCrc8(const uint8_t* data, uint8_t len);

}  // namespace drv
