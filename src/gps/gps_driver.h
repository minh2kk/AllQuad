// AllQuad - GPS: u-blox UBX (NAV-PVT, auto-configured for M8/M9/M10) with NMEA fallback
#pragma once
#include "../bus/topics.h"
#include "../hal/hal.h"

namespace drv {

class GpsDriver {
 public:
  // Configures the receiver on `port` (blocking ~1.5 s) and leaves the port at 115200
  void begin(hal::Port port);
  void attach(hal::Port port) { port_ = port; }  // receiver already configured
  // Drain the port; returns true when a new solution arrived (also without a fix: satellites count).
  // No valid UBX / NMEA frame for 2.5 s: the port tries the next baud rate (115200, 9600, 38400,
  // 57600, 230400) - receivers that ignored the configuration or are NMEA only (ATGM336H, BN-xxx
  // clones at 9600) are found this way.
  bool poll();
  uint32_t baud() const { return baud_; }
  uint32_t frames() const { return frames_; }
  const fc::GpsData& data() const { return data_; }
  bool detected() const { return detected_; }
  const char* protocol() const { return ubxSeen_ ? "UBX" : (nmeaSeen_ ? "NMEA" : "none"); }
  float courseDeg() const { return course_; }
  // receiver: u-blox generation (7 / 8 / 9 / 10, 0 unknown), protocol version, measured NAV-PVT rate
  uint8_t generation() const { return gen_; }
  const char* protVer() const { return protVer_; }
  bool versionKnown() const { return versionKnown_; }
  float navRateHz(uint32_t nowMs);
  uint16_t valsetAcks() const { return valsetAck_; }
  uint16_t valsetNaks() const { return valsetNak_; }
  void pollVersion();

  bool feed(uint8_t b);  // exposed for tests

 private:
  bool ubxByte(uint8_t b);
  bool nmeaByte(uint8_t b);
  bool parseNmea();
  void sanitize();
  void sendUbx(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t len);
  void configure();
  void parseMonVer(const uint8_t* p, uint16_t len);

  fc::GpsData data_{};
  hal::Port port_ = hal::Port::SERIAL2;
  float course_ = 0;
  bool detected_ = false, ubxSeen_ = false, nmeaSeen_ = false;
  uint32_t baud_ = 115200, frames_ = 0, lastFrameMs_ = 0, switchMs_ = 0;
  uint8_t baudIdx_ = 0;
  bool timing_ = false;
  uint8_t gen_ = 0;
  char protVer_[8] = "";
  bool versionKnown_ = false;
  uint16_t valsetAck_ = 0, valsetNak_ = 0;
  uint32_t pvtCount_ = 0, rateCount_ = 0, rateMs_ = 0;
  float rateHz_ = 0;

  // UBX state
  uint8_t ubx_[260];  // NAV-PVT 92, MON-VER up to ~250
  uint16_t ubxPos_ = 0, ubxLen_ = 0;
  uint8_t ubxState_ = 0;
  // NMEA state
  char nmea_[96];
  uint8_t nmeaPos_ = 0;
  bool inNmea_ = false;
};

}  // namespace drv
