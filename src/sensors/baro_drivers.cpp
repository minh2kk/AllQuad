// AllQuad - barometer drivers (I2C): BMP280/BME280, BMP388/BMP390, DPS310/SPL06, MS5611
#include "../common/size_opt.h"  // -Os on ESP32 (not flight critical)
#include "../hal/hal.h"
#include "sensors.h"

#include <stdint.h>

namespace drv {

static inline int16_t le16s(const uint8_t* p) { return (int16_t)(p[0] | (p[1] << 8)); }

// ------------------------------------------------------------------ BMP280 / BME280
class Bmp280 : public BaroDriver {
 public:
  bool init() override {
    static const uint8_t addrs[] = {0x76, 0x77};
    for (uint8_t a : addrs) {
      uint8_t id = 0;
      if (hal::i2cRead(a, 0xD0, &id, 1) && (id == 0x58 || id == 0x60)) { addr_ = a; bme_ = id == 0x60; break; }
    }
    if (!addr_) return false;
    uint8_t c[24];
    if (!hal::i2cRead(addr_, 0x88, c, 24)) return false;
    T1 = le16u(c + 0); T2 = le16s(c + 2); T3 = le16s(c + 4);
    P1 = le16u(c + 6); P2 = le16s(c + 8); P3 = le16s(c + 10); P4 = le16s(c + 12); P5 = le16s(c + 14);
    P6 = le16s(c + 16); P7 = le16s(c + 18); P8 = le16s(c + 20); P9 = le16s(c + 22);
    hal::i2cWrite(addr_, 0xF5, 0x08);  // standby 0.5 ms, IIR x4
    hal::i2cWrite(addr_, 0xF4, 0x53);  // temp x2, press x8, normal mode (~50 Hz)
    return true;
  }
  bool update(float& pa, float& tempC) override {
    uint8_t d[6];
    if (!hal::i2cRead(addr_, 0xF7, d, 6)) return false;
    int32_t adcP = ((int32_t)d[0] << 12) | ((int32_t)d[1] << 4) | (d[2] >> 4);
    int32_t adcT = ((int32_t)d[3] << 12) | ((int32_t)d[4] << 4) | (d[5] >> 4);
    if (adcP == 0x80000) return false;  // no measurement yet
    // Bosch datasheet floating point compensation
    double v1 = (adcT / 16384.0 - T1 / 1024.0) * T2;
    double v2 = (adcT / 131072.0 - T1 / 8192.0);
    v2 = v2 * v2 * T3;
    double tFine = v1 + v2;
    tempC = (float)(tFine / 5120.0);
    v1 = tFine / 2.0 - 64000.0;
    v2 = v1 * v1 * P6 / 32768.0;
    v2 = v2 + v1 * P5 * 2.0;
    v2 = v2 / 4.0 + P4 * 65536.0;
    v1 = (P3 * v1 * v1 / 524288.0 + P2 * v1) / 524288.0;
    v1 = (1.0 + v1 / 32768.0) * P1;
    if (v1 == 0.0) return false;
    double p = 1048576.0 - adcP;
    p = (p - v2 / 4096.0) * 6250.0 / v1;
    v1 = P9 * p * p / 2147483648.0;
    v2 = p * P8 / 32768.0;
    p = p + (v1 + v2 + P7) / 16.0;
    pa = (float)p;
    return true;
  }
  const char* name() const override { return bme_ ? "BME280" : "BMP280"; }

 private:
  uint8_t addr_ = 0;
  bool bme_ = false;
  uint16_t T1, P1;
  int16_t T2, T3, P2, P3, P4, P5, P6, P7, P8, P9;
};

// ------------------------------------------------------------------ BMP388 / BMP390
class Bmp388 : public BaroDriver {
 public:
  bool init() override {
    static const uint8_t addrs[] = {0x76, 0x77};
    for (uint8_t a : addrs) {
      uint8_t id = 0;
      if (hal::i2cRead(a, 0x00, &id, 1) && (id == 0x50 || id == 0x60)) { addr_ = a; is390_ = id == 0x60; break; }
    }
    if (!addr_) return false;
    hal::i2cWrite(addr_, 0x7E, 0xB6);  // soft reset
    hal::delayMs(10);
    uint8_t c[21];
    if (!hal::i2cRead(addr_, 0x31, c, 21)) return false;
    T1 = le16u(c + 0) * 256.0;                       // / 2^-8
    T2 = le16u(c + 2) / 1073741824.0;                // / 2^30
    T3 = (int8_t)c[4] / 281474976710656.0;           // / 2^48
    P1 = (le16s(c + 5) - 16384.0) / 1048576.0;       // (x - 2^14) / 2^20
    P2 = (le16s(c + 7) - 16384.0) / 536870912.0;     // (x - 2^14) / 2^29
    P3 = (int8_t)c[9] / 4294967296.0;                // / 2^32
    P4 = (int8_t)c[10] / 137438953472.0;             // / 2^37
    P5 = le16u(c + 11) * 8.0;                        // / 2^-3
    P6 = le16u(c + 13) / 64.0;                       // / 2^6
    P7 = (int8_t)c[15] / 256.0;                      // / 2^8
    P8 = (int8_t)c[16] / 32768.0;                    // / 2^15
    P9 = le16s(c + 17) / 281474976710656.0;          // / 2^48
    P10 = (int8_t)c[19] / 281474976710656.0;         // / 2^48
    P11 = (int8_t)c[20] / 36893488147419103232.0;    // / 2^65
    hal::i2cWrite(addr_, 0x1C, 0x03);  // OSR: press x8, temp x1
    hal::i2cWrite(addr_, 0x1D, 0x02);  // ODR 50 Hz
    hal::i2cWrite(addr_, 0x1F, 0x04);  // IIR coef 3
    hal::i2cWrite(addr_, 0x1B, 0x33);  // press + temp enabled, normal mode
    return true;
  }
  bool update(float& pa, float& tempC) override {
    uint8_t d[6];
    if (!hal::i2cRead(addr_, 0x04, d, 6)) return false;
    double uP = (double)((uint32_t)d[0] | ((uint32_t)d[1] << 8) | ((uint32_t)d[2] << 16));
    double uT = (double)((uint32_t)d[3] | ((uint32_t)d[4] << 8) | ((uint32_t)d[5] << 16));
    if (uP == 0) return false;
    double pd1 = uT - T1;
    double t = pd1 * T2 + pd1 * pd1 * T3;
    tempC = (float)t;
    double o1 = P5 + P6 * t + P7 * t * t + P8 * t * t * t;
    double o2 = uP * (P1 + P2 * t + P3 * t * t + P4 * t * t * t);
    double o3 = uP * uP * (P9 + P10 * t) + uP * uP * uP * P11;
    pa = (float)(o1 + o2 + o3);
    return true;
  }
  const char* name() const override { return is390_ ? "BMP390" : "BMP388"; }

 private:
  uint8_t addr_ = 0;
  bool is390_ = false;
  double T1, T2, T3, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11;
};

// ------------------------------------------------------------------ DPS310
class Dps310 : public BaroDriver {
 public:
  bool init() override {
    static const uint8_t addrs[] = {0x77, 0x76};
    for (uint8_t a : addrs) {
      uint8_t id = 0;
      if (hal::i2cRead(a, 0x0D, &id, 1) && id == 0x10) { addr_ = a; break; }
    }
    if (!addr_) return false;
    hal::i2cWrite(addr_, 0x0C, 0x09);  // soft reset
    hal::delayMs(40);
    uint8_t st = 0;
    for (int i = 0; i < 50; i++) {  // wait COEF_RDY + SENSOR_RDY
      hal::i2cRead(addr_, 0x08, &st, 1);
      if ((st & 0xC0) == 0xC0) break;
      hal::delayMs(5);
    }
    uint8_t c[18];
    if (!hal::i2cRead(addr_, 0x10, c, 18)) return false;
    c0 = sext(((uint32_t)c[0] << 4) | (c[1] >> 4), 12);
    c1 = sext((((uint32_t)c[1] & 0x0F) << 8) | c[2], 12);
    c00 = sext(((uint32_t)c[3] << 12) | ((uint32_t)c[4] << 4) | (c[5] >> 4), 20);
    c10 = sext((((uint32_t)c[5] & 0x0F) << 16) | ((uint32_t)c[6] << 8) | c[7], 20);
    c01 = sext(((uint32_t)c[8] << 8) | c[9], 16);
    c11 = sext(((uint32_t)c[10] << 8) | c[11], 16);
    c20 = sext(((uint32_t)c[12] << 8) | c[13], 16);
    c21 = sext(((uint32_t)c[14] << 8) | c[15], 16);
    c30 = sext(((uint32_t)c[16] << 8) | c[17], 16);
    uint8_t src = 0;
    hal::i2cRead(addr_, 0x28, &src, 1);  // COEF_SRCE: which temperature sensor
    // Infineon temperature-sensor workaround
    hal::i2cWrite(addr_, 0x0E, 0xA5);
    hal::i2cWrite(addr_, 0x0F, 0x96);
    hal::i2cWrite(addr_, 0x62, 0x02);
    hal::i2cWrite(addr_, 0x0E, 0x00);
    hal::i2cWrite(addr_, 0x0F, 0x00);
    hal::i2cWrite(addr_, 0x06, 0x54);                 // PRS_CFG: 32 Hz, 16x oversampling
    hal::i2cWrite(addr_, 0x07, 0x50 | (src & 0x80));  // TMP_CFG: 32 Hz, 1x
    hal::i2cWrite(addr_, 0x09, 0x04);                 // CFG_REG: P_SHIFT (needed for >8x)
    hal::i2cWrite(addr_, 0x08, 0x07);                 // continuous P + T
    return true;
  }
  bool update(float& pa, float& tempC) override {
    uint8_t d[6];
    if (!hal::i2cRead(addr_, 0x00, d, 6)) return false;
    float pr = (float)sext(((uint32_t)d[0] << 16) | ((uint32_t)d[1] << 8) | d[2], 24) / 253952.0f;
    float tr = (float)sext(((uint32_t)d[3] << 16) | ((uint32_t)d[4] << 8) | d[5], 24) / 524288.0f;
    tempC = c0 * 0.5f + c1 * tr;
    pa = c00 + pr * (c10 + pr * (c20 + pr * c30)) + tr * c01 + tr * pr * (c11 + pr * c21);
    return pa > 10000.0f;
  }
  const char* name() const override { return "DPS310/SPL06"; }

 private:
  static int32_t sext(uint32_t v, int bits) {
    uint32_t m = 1u << (bits - 1);
    v &= (1u << bits) - 1;
    return (int32_t)((v ^ m) - m);
  }
  uint8_t addr_ = 0;
  float c0, c1, c00, c10, c01, c11, c20, c21, c30;
};


// ------------------------------------------------------------------ MS5611 (TE)
// Command-based: D1 (pressure) and D2 (temperature) conversions alternate, one per update().
class Ms5611 : public BaroDriver {
 public:
  bool init() override {
    static const uint8_t addrs[] = {0x77, 0x76};
    for (uint8_t a : addrs) {
      if (!hal::i2cProbe(a)) continue;
      hal::i2cCommand(a, 0x1E);  // reset
      hal::delayMs(4);
      bool ok = true, allZero = true;
      for (int i = 0; i < 8; i++) {
        uint8_t b[2];
        if (!hal::i2cRead(a, (uint8_t)(0xA0 + i * 2), b, 2)) { ok = false; break; }
        prom_[i] = (uint16_t)((b[0] << 8) | b[1]);
        if (prom_[i] != 0 && prom_[i] != 0xFFFF) allZero = false;
      }
      if (ok && !allZero && crc4(prom_) == (prom_[7] & 0x0F)) { addr_ = a; break; }
    }
    if (!addr_) return false;
    hal::i2cCommand(addr_, 0x58);  // start D2 (temperature, OSR 4096)
    state_ = 0;
    return true;
  }
  bool update(float& pa, float& tempC) override {
    uint8_t b[3];
    if (!hal::i2cRead(addr_, 0x00, b, 3)) return false;  // ADC result of the last conversion
    uint32_t v = ((uint32_t)b[0] << 16) | ((uint32_t)b[1] << 8) | b[2];
    if (state_ == 0) { d2_ = v; hal::i2cCommand(addr_, 0x48); state_ = 1; return false; }
    d1_ = v;
    hal::i2cCommand(addr_, 0x58);
    state_ = 0;
    if (!d1_ || !d2_) return false;
    int64_t dT = (int64_t)d2_ - ((int64_t)prom_[5] << 8);
    int64_t temp = 2000 + (dT * prom_[6] >> 23);
    int64_t off = ((int64_t)prom_[2] << 16) + ((prom_[4] * dT) >> 7);
    int64_t sens = ((int64_t)prom_[1] << 15) + ((prom_[3] * dT) >> 8);
    if (temp < 2000) {  // second order compensation
      int64_t t2 = (dT * dT) >> 31;
      int64_t d = (temp - 2000) * (temp - 2000);
      int64_t off2 = 5 * d / 2, sens2 = 5 * d / 4;
      if (temp < -1500) {
        int64_t d2 = (temp + 1500) * (temp + 1500);
        off2 += 7 * d2;
        sens2 += 11 * d2 / 2;
      }
      temp -= t2; off -= off2; sens -= sens2;
    }
    int64_t p = ((((int64_t)d1_ * sens) >> 21) - off) >> 15;  // Pa
    pa = (float)p;
    tempC = temp / 100.0f;
    return true;
  }
  const char* name() const override { return "MS5611"; }

  static uint8_t crc4(const uint16_t prom[8]) {
    uint16_t n[8];
    for (int i = 0; i < 8; i++) n[i] = prom[i];
    uint16_t rem = 0;
    n[7] &= 0xFF00;
    for (int cnt = 0; cnt < 16; cnt++) {
      rem ^= (cnt & 1) ? (uint16_t)(n[cnt >> 1] & 0x00FF) : (uint16_t)(n[cnt >> 1] >> 8);
      for (int bit = 8; bit > 0; bit--) rem = (rem & 0x8000) ? (uint16_t)((rem << 1) ^ 0x3000) : (uint16_t)(rem << 1);
    }
    return (uint8_t)((rem >> 12) & 0x0F);
  }

 private:
  uint8_t addr_ = 0;
  uint16_t prom_[8];
  uint32_t d1_ = 0, d2_ = 0;
  uint8_t state_ = 0;
};

BaroDriver* probeBaro() {
  static Dps310 dps;
  static Bmp388 bmp388;
  static Bmp280 bmp280;
  static Ms5611 ms5611;
  if (dps.init()) return &dps;
  if (bmp388.init()) return &bmp388;
  if (bmp280.init()) return &bmp280;
  if (ms5611.init()) return &ms5611;  // last: has no ID register (validated by PROM CRC)
  return nullptr;
}

}  // namespace drv
