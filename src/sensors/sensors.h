// AllQuad - sensor driver interfaces + auto-detection
//
// Raw sensor axes follow the chip marking (X, Y, Z right-handed, Z out of the top).
// Drivers convert to FRD (x, -y, -z); board alignment is applied afterwards.
#pragma once
#include "../common/math3d.h"
#include "../config/config.h"
#include "../hal/hal.h"

namespace drv {

using fc::Vec3;

class ImuDriver {
 public:
  virtual ~ImuDriver() {}
  virtual bool init(uint16_t loopHz) = 0;
  virtual bool read(Vec3& gyroRad, Vec3& accMs2) = 0;  // FRD, before board alignment
  virtual const char* name() const = 0;
  // Data-ready pulses on the chip's INT1 pin, used to start the fast loop (the sample is read the
  // moment it exists instead of up to one period late). Sets the output data rate high (several kHz)
  // with the chip's own low-pass filter as anti-alias, and *divider = pulses per loop so the loop runs
  // near loopHz. Returns the pulse rate in Hz, 0 = not supported (the loop stays on the timer).
  virtual uint32_t enableDataReady(uint16_t loopHz, uint8_t* divider) { (void)loopHz; (void)divider; return 0; }
  void setDev(hal::SpiDev d) { dev_ = d; }

 protected:
  hal::SpiDev dev_ = hal::SpiDev::IMU;  // chip select (IMU, or IMU2 for the second IMU)
};

class BaroDriver {
 public:
  virtual ~BaroDriver() {}
  virtual bool init() = 0;
  // Call at 50 Hz. Returns true and fills pa/tempC when a new sample is available.
  virtual bool update(float& pa, float& tempC) = 0;
  virtual const char* name() const = 0;
};

class MagDriver {
 public:
  virtual ~MagDriver() {}
  virtual bool init() = 0;
  virtual bool read(Vec3& gauss) = 0;  // FRD, ~gauss
  virtual const char* name() const = 0;
};

// Probe every supported chip; return the first one found (nullptr if none).
ImuDriver* probeImu(uint16_t loopHz);
ImuDriver* probeImu2(uint16_t loopHz, const ImuDriver* first);  // second IMU (IMU2 chip select)
BaroDriver* probeBaro();
MagDriver* probeMag();

Vec3 applyAlign(const Vec3& v, fc::Align a);
inline Vec3 toFrd(float x, float y, float z) { return Vec3(x, -y, -z); }
inline int16_t be16(const uint8_t* p) { return (int16_t)((p[0] << 8) | p[1]); }
inline int16_t le16(const uint8_t* p) { return (int16_t)(p[0] | (p[1] << 8)); }
inline uint16_t le16u(const uint8_t* p) { return (uint16_t)(p[0] | (p[1] << 8)); }

constexpr float GYRO_2000DPS = 1.0f / 16.4f * fc::DEG2RAD;  // rad/s per LSB
constexpr float ACC_16G = fc::GRAVITY / 2048.0f;            // m/s^2 per LSB

}  // namespace drv
