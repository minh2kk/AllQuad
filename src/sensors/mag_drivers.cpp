// AllQuad - magnetometer drivers (I2C): QMC5883L, QMC5883P, HMC5883L, IST8310, LIS3MDL
#include "../common/size_opt.h"  // -Os on ESP32 (not flight critical)
#include "../hal/hal.h"
#include "sensors.h"

namespace drv {

class Qmc5883l : public MagDriver {
 public:
  bool init() override {
    uint8_t id = 0;
    if (!hal::i2cRead(ADDR, 0x0D, &id, 1) || id != 0xFF) return false;
    hal::i2cWrite(ADDR, 0x0A, 0x80);  // soft reset
    hal::delayMs(10);
    hal::i2cWrite(ADDR, 0x0B, 0x01);  // SET/RESET period
    hal::i2cWrite(ADDR, 0x09, 0x1D);  // OSR 512, 8 G, 200 Hz, continuous
    return true;
  }
  bool read(Vec3& f) override {
    uint8_t b[6];
    if (!hal::i2cRead(ADDR, 0x00, b, 6)) return false;
    float x = (int16_t)(b[0] | (b[1] << 8));
    float y = (int16_t)(b[2] | (b[3] << 8));
    float z = (int16_t)(b[4] | (b[5] << 8));
    f = Vec3(x, -y, -z) * (1.0f / 3000.0f);  // ~gauss
    return true;
  }
  const char* name() const override { return "QMC5883L"; }

 private:
  static constexpr uint8_t ADDR = 0x0D;
};

class Hmc5883l : public MagDriver {
 public:
  bool init() override {
    uint8_t id[3];
    if (!hal::i2cRead(ADDR, 0x0A, id, 3) || id[0] != 'H' || id[1] != '4' || id[2] != '3') return false;
    hal::i2cWrite(ADDR, 0x00, 0x78);  // 8 samples averaged, 75 Hz
    hal::i2cWrite(ADDR, 0x01, 0x20);  // +-1.3 Ga
    hal::i2cWrite(ADDR, 0x02, 0x00);  // continuous
    return true;
  }
  bool read(Vec3& f) override {
    uint8_t b[6];
    if (!hal::i2cRead(ADDR, 0x03, b, 6)) return false;  // X, Z, Y (big endian)
    float x = (int16_t)((b[0] << 8) | b[1]);
    float z = (int16_t)((b[2] << 8) | b[3]);
    float y = (int16_t)((b[4] << 8) | b[5]);
    f = Vec3(x, -y, -z) * (1.0f / 1090.0f);
    return true;
  }
  const char* name() const override { return "HMC5883L"; }

 private:
  static constexpr uint8_t ADDR = 0x1E;
};

// QMC5883P (address 0x2C, chip id 0x80) - a different chip from the QMC5883L.
// Reference: ArduPilot AP_Compass_QMC5883P, Adafruit_QMC5883P
class Qmc5883p : public MagDriver {
 public:
  bool init() override {
    uint8_t id = 0;
    if (!hal::i2cRead(ADDR, 0x00, &id, 1) || id != 0x80) return false;
    hal::i2cWrite(ADDR, 0x0B, 0x80);  // CONF2: soft reset
    hal::delayMs(5);
    hal::i2cWrite(ADDR, 0x06, 0x29);  // axis sign setting required for continuous mode (datasheet 7.2)
    hal::i2cWrite(ADDR, 0x0B, 0x08);  // CONF2: +-8 gauss
    hal::i2cWrite(ADDR, 0x0A, 0x0B);  // CONF1: continuous, 100 Hz, OSR 8
    return true;
  }
  bool read(Vec3& f) override {
    uint8_t b[6];
    if (!hal::i2cRead(ADDR, 0x01, b, 6)) return false;  // X, Y, Z little-endian
    float x = (int16_t)(b[0] | (b[1] << 8)), y = (int16_t)(b[2] | (b[3] << 8)), z = (int16_t)(b[4] | (b[5] << 8));
    f = Vec3(x, -y, -z) * (1.0f / 3750.0f);  // 3750 LSB/gauss at +-8 G
    return true;
  }
  const char* name() const override { return "QMC5883P"; }

 private:
  static constexpr uint8_t ADDR = 0x2C;
};

// IST8310: single-measurement mode, re-triggered after every read
class Ist8310 : public MagDriver {
 public:
  bool init() override {
    static const uint8_t addrs[] = {0x0E, 0x0C};
    for (uint8_t a : addrs) {
      uint8_t id = 0;
      if (hal::i2cRead(a, 0x00, &id, 1) && id == 0x10) { addr_ = a; break; }
    }
    if (!addr_) return false;
    hal::i2cWrite(addr_, 0x0B, 0x01);  // CNTL2 soft reset
    hal::delayMs(10);
    hal::i2cWrite(addr_, 0x41, 0x24);  // 16x averaging x/y and z
    hal::i2cWrite(addr_, 0x42, 0xC0);  // pulse duration (datasheet recommended)
    hal::i2cWrite(addr_, 0x0A, 0x01);  // first single measurement
    return true;
  }
  bool read(Vec3& f) override {
    uint8_t b[6];
    if (!hal::i2cRead(addr_, 0x03, b, 6)) return false;
    hal::i2cWrite(addr_, 0x0A, 0x01);  // trigger next
    float x = (int16_t)(b[0] | (b[1] << 8)), y = (int16_t)(b[2] | (b[3] << 8)), z = (int16_t)(b[4] | (b[5] << 8));
    f = Vec3(x, -y, -z) * 0.003f;  // 0.3 uT/LSB -> gauss
    return true;
  }
  const char* name() const override { return "IST8310"; }

 private:
  uint8_t addr_ = 0;
};

// LIS3MDL: continuous, +-4 gauss, ultra-high-performance
class Lis3mdl : public MagDriver {
 public:
  bool init() override {
    static const uint8_t addrs[] = {0x1C, 0x1E};
    for (uint8_t a : addrs) {
      uint8_t id = 0;
      if (hal::i2cRead(a, 0x0F, &id, 1) && id == 0x3D) { addr_ = a; break; }
    }
    if (!addr_) return false;
    hal::i2cWrite(addr_, 0x20, 0x7C);  // CTRL1: UHP x/y, 80 Hz
    hal::i2cWrite(addr_, 0x21, 0x00);  // CTRL2: +-4 gauss
    hal::i2cWrite(addr_, 0x23, 0x0C);  // CTRL4: UHP z
    hal::i2cWrite(addr_, 0x22, 0x00);  // CTRL3: continuous
    return true;
  }
  bool read(Vec3& f) override {
    uint8_t b[6];
    if (!hal::i2cRead(addr_, 0x28 | 0x80, b, 6)) return false;  // auto-increment
    float x = (int16_t)(b[0] | (b[1] << 8)), y = (int16_t)(b[2] | (b[3] << 8)), z = (int16_t)(b[4] | (b[5] << 8));
    f = Vec3(x, -y, -z) * (1.0f / 6842.0f);
    return true;
  }
  const char* name() const override { return "LIS3MDL"; }

 private:
  uint8_t addr_ = 0;
};

MagDriver* probeMag() {
  static Qmc5883l qmc;
  static Qmc5883p qmcp;
  static Hmc5883l hmc;
  static Ist8310 ist;
  static Lis3mdl lis;
  MagDriver* order[] = {&qmc, &qmcp, &hmc, &ist, &lis};
  for (MagDriver* d : order)
    if (d->init()) return d;
  return nullptr;
}

}  // namespace drv
