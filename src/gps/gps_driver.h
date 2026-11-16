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
  // Drain the port; returns true when a new position solution arrived
  bool poll();
  const fc::GpsData& data() const { return data_; }
  bool detected() const { return detected_; }
  const char* protocol() const { return ubxSeen_ ? "UBX" : (nmeaSeen_ ? "NMEA" : "none"); }
  float courseDeg() const { return course_; }

  bool feed(uint8_t b);  // exposed for tests

 private:
  bool ubxByte(uint8_t b);
  bool nmeaByte(uint8_t b);
  bool parseNmea();
  void sendUbx(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t len);
  void configure();

  fc::GpsData data_{};
  hal::Port port_ = hal::Port::SERIAL2;
  float course_ = 0;
  bool detected_ = false, ubxSeen_ = false, nmeaSeen_ = false;

  // UBX state
  uint8_t ubx_[100];
  uint16_t ubxPos_ = 0, ubxLen_ = 0;
  uint8_t ubxState_ = 0;
  // NMEA state
  char nmea_[96];
  uint8_t nmeaPos_ = 0;
  bool inNmea_ = false;
};

}  // namespace drv
