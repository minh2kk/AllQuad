// AllQuad - IMU drivers
//   SPI : ICM-45686, ICM-45605, ICM-42688-P, ICM-42605, MPU6000, MPU6500, MPU9250, ICM-20602, ICM-20689,
//         BMI088 (separate gyro/accel chip selects), LSM6DSO, LSM6DSR, LSM6DSL, ISM330DHCX
//   I2C : MPU6050 (fallback)
#include "../common/size_opt.h"  // -Os on ESP32 (not flight critical)
#include "../hal/hal.h"
#include "sensors.h"

#include <string.h>

namespace drv {

using hal::SpiDev;

// ------------------------------------------------------------------ InvenSense ICM-456xx
// ICM-45686 / ICM-45605 (register map differs from ICM-426xx: no bank switching,
// WHO_AM_I at 0x72, little-endian data from 0x00). Reference: Betaflight accgyro_spi_icm456xx.c
class Icm456xx : public ImuDriver {
 public:
  bool init(uint16_t loopHz) override {
    hal::spiSetSpeed(dev_, 1000000);
    hal::spiWrite(dev_, 0x7F, 0x02);  // REG_MISC2: soft reset
    for (int i = 0; i < 20; i++) {           // reset bit self-clears (max ~20 ms)
      hal::delayMs(1);
      uint8_t misc = 0xFF;
      hal::spiRead(dev_, 0x7F, &misc, 1);
      if (!(misc & 0x02)) break;
    }
    hal::spiWrite(dev_, 0x10, 0x00);  // PWR_MGMT0: sensors off
    uint8_t who = 0;
    for (int i = 0; i < 5 && who != 0xE9 && who != 0xE5; i++) {
      hal::delayMs(1);
      hal::spiRead(dev_, 0x72, &who, 1);
    }
    if (who == 0xE9) name_ = "ICM-45686";
    else if (who == 0xE5) name_ = "ICM-45605";
    else return false;
    hal::spiWrite(dev_, 0x10, 0x0F);  // PWR_MGMT0: gyro + accel low-noise
    hal::delayMs(1);
    hal::spiWrite(dev_, 0x1B, 0x10 | 0x05);  // ACCEL_CONFIG0: 16 g, 1.6 kHz
    hal::delayMs(10);
    // GYRO_CONFIG0: 2000 dps, ODR 1.6 kHz (1 kHz loop) / 3.2 kHz (2 kHz loop)
    hal::spiWrite(dev_, 0x1C, 0x10 | (loopHz >= 2000 ? 0x04 : 0x05));
    hal::delayMs(35);
    hal::spiSetSpeed(dev_, 12000000);
    return true;
  }
  bool read(Vec3& gyro, Vec3& acc) override {
    uint8_t b[12];
    if (!hal::spiRead(dev_, 0x00, b, 12)) return false;  // ACCEL_DATA_X1_UI .. GYRO_DATA_Z0_UI
    acc = toFrd(le16(b + 0), le16(b + 2), le16(b + 4)) * ACC_16G;
    gyro = toFrd(le16(b + 6), le16(b + 8), le16(b + 10)) * GYRO_2000DPS;
    return true;
  }
  const char* name() const override { return name_; }

  // Betaflight setup: gyro 6.4 kHz, DRDY on INT1 (pulsed, push-pull, active high). The loop takes every
  // N-th sample; the chip's UI low-pass (an IREG register) keeps everything above the loop's Nyquist out.
  uint32_t enableDataReady(uint16_t loopHz, uint8_t* divider) override {
    const uint32_t odr = 6400;
    uint32_t div = (odr + loopHz / 2) / loopHz;  // 1 kHz -> 6 (1067 Hz), 2 kHz -> 3 (2133 Hz)
    if (div < 1) div = 1;
    if (div > 32) div = 32;
    uint32_t loop = odr / div;
    // UI LPF bandwidth ODR/4..ODR/32 = 1600/800/400/200 Hz: the widest one below 0.45 x loop rate
    uint8_t bw = 4;
    if (1600 * 100 <= loop * 45) bw = 1;
    else if (800 * 100 <= loop * 45) bw = 2;
    else if (400 * 100 <= loop * 45) bw = 3;
    hal::spiSetSpeed(dev_, 1000000);
    uint8_t lpf = 0;
    bool ok = iregRead(0xA4AC, lpf) && iregWrite(0xA4AC, (uint8_t)((lpf & ~0x07) | bw));
    hal::spiWrite(dev_, 0x1C, 0x10 | 0x03);  // GYRO_CONFIG0: 2000 dps, 6.4 kHz
    hal::delayMs(35);                                // gyro start-up after an ODR change
    hal::spiWrite(dev_, 0x18, 0x01);          // INT1_CONFIG2: pulsed, push-pull, active high
    hal::spiWrite(dev_, 0x16, 0x04);          // INT1_CONFIG0: data ready
    hal::spiSetSpeed(dev_, 12000000);
    (void)ok;  // without the LPF the loop still works, only with the chip's default (wider) filter
    *divider = (uint8_t)div;
    return odr;
  }

 private:
  // Indirect registers (Betaflight icm456xx_read/write_ireg): wait for IREG_DONE, then the address
  // (and data) as one burst from 0x7C; accesses at least 4 us apart.
  bool iregReady() {
    for (int i = 0; i < 5; i++) {
      uint8_t m = 0;
      hal::spiRead(dev_, 0x7F, &m, 1);
      if (m & 0x01) return true;
      hal::delayMs(1);
    }
    return false;
  }
  bool iregWrite(uint16_t addr, uint8_t v) {
    if (!iregReady()) return false;
    const uint8_t b[4] = {0x7C, (uint8_t)(addr >> 8), (uint8_t)addr, v};
    bool ok = hal::spiWriteRaw(dev_, b, 4);
    hal::delayMs(1);
    return ok;
  }
  bool iregRead(uint16_t addr, uint8_t& v) {
    if (!iregReady()) return false;
    const uint8_t b[3] = {0x7C, (uint8_t)(addr >> 8), (uint8_t)addr};
    if (!hal::spiWriteRaw(dev_, b, 3)) return false;
    hal::delayMs(1);
    bool ok = hal::spiRead(dev_, 0x7E, &v, 1);
    hal::delayMs(1);
    return ok;
  }
  const char* name_ = "ICM-456xx";
};

// ------------------------------------------------------------------ InvenSense ICM-426xx
class Icm426xx : public ImuDriver {
 public:
  bool init(uint16_t loopHz) override {
    hal::spiSetSpeed(dev_, 1000000);
    hal::spiWrite(dev_, 0x76, 0x00);  // bank 0
    hal::spiWrite(dev_, 0x11, 0x01);  // soft reset
    hal::delayMs(2);
    uint8_t who = 0;
    hal::spiRead(dev_, 0x75, &who, 1);
    if (who == 0x47) name_ = "ICM-42688-P";
    else if (who == 0x42) name_ = "ICM-42605";
    else return false;
    uint8_t odr = loopHz >= 2000 ? 0x05 : 0x06;  // 2 kHz / 1 kHz
    hal::spiWrite(dev_, 0x4F, odr);       // GYRO_CONFIG0: 2000 dps
    hal::spiWrite(dev_, 0x50, odr);       // ACCEL_CONFIG0: 16 g
    hal::spiWrite(dev_, 0x52, 0x11);      // UI filter BW ODR/4
    hal::spiWrite(dev_, 0x4E, 0x0F);      // gyro + accel low-noise
    hal::delayMs(50);
    hal::spiSetSpeed(dev_, 10000000);
    return true;
  }
  bool read(Vec3& gyro, Vec3& acc) override {
    uint8_t b[12];
    if (!hal::spiRead(dev_, 0x1F, b, 12)) return false;
    acc = toFrd(be16(b + 0), be16(b + 2), be16(b + 4)) * ACC_16G;
    gyro = toFrd(be16(b + 6), be16(b + 8), be16(b + 10)) * GYRO_2000DPS;
    return true;
  }
  // Gyro 8 kHz, UI data ready on INT1 (Betaflight: pulsed, push-pull, active high, 8 us pulse, no
  // de-assert time, async reset off - required at ODR >= 4 kHz). Accel 1 kHz (aligned with the gyro).
  uint32_t enableDataReady(uint16_t loopHz, uint8_t* divider) override {
    const uint32_t odr = 8000;
    uint32_t div = (odr + loopHz / 2) / loopHz;  // 1 kHz -> 8, 2 kHz -> 4
    if (div < 1) div = 1;
    if (div > 32) div = 32;
    uint32_t loop = odr / div;
    // GYRO_UI_FILT_BW: 4 = ODR/10 (800 Hz), 6 = ODR/20 (400 Hz), 7 = ODR/40 (200 Hz)
    uint8_t bw = 800 * 100 <= loop * 45 ? 4 : 400 * 100 <= loop * 45 ? 6 : 7;
    hal::spiSetSpeed(dev_, 1000000);
    hal::spiWrite(dev_, 0x4E, 0x00);         // sensors off while reconfiguring
    hal::delayMs(1);
    hal::spiWrite(dev_, 0x4F, 0x03);         // GYRO_CONFIG0: 2000 dps, 8 kHz
    hal::spiWrite(dev_, 0x50, 0x06);         // ACCEL_CONFIG0: 16 g, 1 kHz
    hal::spiWrite(dev_, 0x52, (uint8_t)(0x10 | bw));
    hal::spiWrite(dev_, 0x14, 0x03);         // INT_CONFIG: INT1 pulsed, push-pull, active high
    hal::spiWrite(dev_, 0x63, 0x00);         // INT_CONFIG0: DRDY clears on status read
    hal::spiWrite(dev_, 0x65, 0x08);         // INT_SOURCE0: UI data ready -> INT1
    uint8_t c1 = 0;
    hal::spiRead(dev_, 0x64, &c1, 1);         // INT_CONFIG1
    hal::spiWrite(dev_, 0x64, (uint8_t)((c1 & ~0x10) | 0x60));
    hal::spiWrite(dev_, 0x4E, 0x0F);         // gyro + accel low-noise
    hal::delayMs(50);
    hal::spiSetSpeed(dev_, 10000000);
    *divider = (uint8_t)div;
    return odr;
  }
  const char* name() const override { return name_; }

 private:
  const char* name_ = "ICM-426xx";
};

// ------------------------------------------------------------------ MPU6000 / 6500 family (SPI)
class MpuSpi : public ImuDriver {
 public:
  bool init(uint16_t) override {
    hal::spiSetSpeed(dev_, 1000000);
    hal::spiWrite(dev_, 0x6B, 0x80);
    hal::delayMs(100);
    hal::spiWrite(dev_, 0x68, 0x07);
    hal::delayMs(100);
    uint8_t who = 0;
    hal::spiRead(dev_, 0x75, &who, 1);
    switch (who) {
      case 0x68: name_ = "MPU6000"; break;
      case 0x70: name_ = "MPU6500"; break;
      case 0x71: name_ = "MPU9250"; break;
      case 0x12: name_ = "ICM-20602"; break;
      case 0x98: name_ = "ICM-20689"; break;
      default: return false;
    }
    hal::spiWrite(dev_, 0x6B, 0x01);
    hal::delayMs(10);
    hal::spiWrite(dev_, 0x6A, 0x10);  // disable I2C
    hal::spiWrite(dev_, 0x19, 0x00);
    hal::spiWrite(dev_, 0x1A, 0x01);
    hal::spiWrite(dev_, 0x1B, 0x18);
    hal::spiWrite(dev_, 0x1C, 0x18);
    if (who != 0x68) hal::spiWrite(dev_, 0x1D, 0x01);
    hal::delayMs(20);
    hal::spiSetSpeed(dev_, who == 0x68 ? 10000000 : 8000000);
    return true;
  }
  bool read(Vec3& gyro, Vec3& acc) override {
    uint8_t b[14];
    if (!hal::spiRead(dev_, 0x3B, b, 14)) return false;
    acc = toFrd(be16(b + 0), be16(b + 2), be16(b + 4)) * ACC_16G;
    gyro = toFrd(be16(b + 8), be16(b + 10), be16(b + 12)) * GYRO_2000DPS;
    return true;
  }
  const char* name() const override { return name_; }

 private:
  const char* name_ = "MPU";
};

// ------------------------------------------------------------------ Bosch BMI088
// Gyro on SpiDev::IMU, accelerometer on SpiDev::IMU2. The accelerometer returns a
// dummy byte before data on SPI reads.
class Bmi088 : public ImuDriver {
 public:
  bool init(uint16_t loopHz) override {
    if (!hal::spiHasDevice(SpiDev::IMU2)) return false;
    hal::spiSetSpeed(SpiDev::IMU, 1000000);
    hal::spiSetSpeed(SpiDev::IMU2, 1000000);
    uint8_t id = 0;
    hal::spiRead(SpiDev::IMU2, 0x00, &id, 1, true);  // rising CS edge switches accel to SPI
    hal::delayMs(1);
    hal::spiRead(SpiDev::IMU2, 0x00, &id, 1, true);
    if (id != 0x1E && id != 0x1A) return false;      // BMI088 / BMI090L
    hal::spiRead(SpiDev::IMU, 0x00, &id, 1);
    if (id != 0x0F) return false;
    // accelerometer
    hal::spiWrite(SpiDev::IMU2, 0x7E, 0xB6);  // soft reset
    hal::delayMs(2);
    hal::spiRead(SpiDev::IMU2, 0x00, &id, 1, true);
    hal::spiWrite(SpiDev::IMU2, 0x7C, 0x00);  // ACC_PWR_CONF active
    hal::delayMs(1);
    hal::spiWrite(SpiDev::IMU2, 0x7D, 0x04);  // ACC_PWR_CTRL on
    hal::delayMs(5);
    hal::spiWrite(SpiDev::IMU2, 0x41, 0x03);  // ACC_RANGE 24 g
    hal::spiWrite(SpiDev::IMU2, 0x40, 0xAC);  // ACC_CONF normal BW, 1600 Hz
    // gyroscope
    hal::spiWrite(SpiDev::IMU, 0x14, 0xB6);   // soft reset
    hal::delayMs(30);
    hal::spiWrite(SpiDev::IMU, 0x0F, 0x00);   // 2000 dps
    hal::spiWrite(SpiDev::IMU, 0x10, loopHz >= 2000 ? 0x01 : 0x02);  // ODR 2000/1000 Hz
    hal::spiWrite(SpiDev::IMU, 0x11, 0x00);   // normal power
    hal::delayMs(30);
    hal::spiSetSpeed(SpiDev::IMU, 10000000);
    hal::spiSetSpeed(SpiDev::IMU2, 10000000);
    return true;
  }
  bool read(Vec3& gyro, Vec3& acc) override {
    uint8_t g[6], a[6];
    if (!hal::spiRead(SpiDev::IMU, 0x02, g, 6)) return false;
    if (!hal::spiRead(SpiDev::IMU2, 0x12, a, 6, true)) return false;
    gyro = toFrd(le16(g), le16(g + 2), le16(g + 4)) * (fc::DEG2RAD / 16.384f);
    acc = toFrd(le16(a), le16(a + 2), le16(a + 4)) * (24.0f * fc::GRAVITY / 32768.0f);
    return true;
  }
  const char* name() const override { return "BMI088"; }
};

// ------------------------------------------------------------------ ST LSM6DSx / ISM330
class Lsm6 : public ImuDriver {
 public:
  bool init(uint16_t) override {
    hal::spiSetSpeed(dev_, 1000000);
    uint8_t who = 0;
    hal::spiRead(dev_, 0x0F, &who, 1);
    if (who == 0x6C) name_ = "LSM6DSO";
    else if (who == 0x6B) name_ = "LSM6DSR/ISM330DHCX";
    else if (who == 0x6A) name_ = "LSM6DSL";
    else return false;
    hal::spiWrite(dev_, 0x12, 0x01);  // CTRL3_C: SW reset
    hal::delayMs(10);
    hal::spiWrite(dev_, 0x12, 0x44);  // BDU + auto-increment
    hal::spiWrite(dev_, 0x10, 0x84);  // CTRL1_XL: 1.66 kHz, 16 g
    hal::spiWrite(dev_, 0x11, 0x8C);  // CTRL2_G: 1.66 kHz, 2000 dps
    hal::delayMs(20);
    hal::spiSetSpeed(dev_, 10000000);
    return true;
  }
  bool read(Vec3& gyro, Vec3& acc) override {
    uint8_t b[12];
    if (!hal::spiRead(dev_, 0x22, b, 12)) return false;  // OUTX_L_G .. OUTZ_H_A
    gyro = toFrd(le16(b), le16(b + 2), le16(b + 4)) * (0.070f * fc::DEG2RAD);
    acc = toFrd(le16(b + 6), le16(b + 8), le16(b + 10)) * (0.000488f * fc::GRAVITY);
    return true;
  }
  const char* name() const override { return name_; }

 private:
  const char* name_ = "LSM6";
};

// ------------------------------------------------------------------ MPU6050 (I2C)
class Mpu6050 : public ImuDriver {
 public:
  bool init(uint16_t) override {
    static const uint8_t addrs[] = {0x68, 0x69};
    for (uint8_t a : addrs) {
      uint8_t who = 0;
      if (hal::i2cRead(a, 0x75, &who, 1) && (who == 0x68 || who == 0x72 || who == 0x70)) { addr_ = a; break; }
    }
    if (!addr_) return false;
    hal::i2cWrite(addr_, 0x6B, 0x80);
    hal::delayMs(100);
    hal::i2cWrite(addr_, 0x6B, 0x01);
    hal::i2cWrite(addr_, 0x19, 0x00);
    hal::i2cWrite(addr_, 0x1A, 0x02);
    hal::i2cWrite(addr_, 0x1B, 0x18);
    hal::i2cWrite(addr_, 0x1C, 0x18);
    hal::delayMs(20);
    return true;
  }
  bool read(Vec3& gyro, Vec3& acc) override {
    uint8_t b[14];
    if (!hal::i2cRead(addr_, 0x3B, b, 14)) return false;
    acc = toFrd(be16(b + 0), be16(b + 2), be16(b + 4)) * ACC_16G;
    gyro = toFrd(be16(b + 8), be16(b + 10), be16(b + 12)) * GYRO_2000DPS;
    return true;
  }
  const char* name() const override { return "MPU6050 (I2C)"; }

 private:
  uint8_t addr_ = 0;
};

ImuDriver* probeImu(uint16_t loopHz) {
  static Bmi088 bmi088;
  static Icm456xx icm45;
  static Icm426xx icm;
  static Lsm6 lsm;
  static MpuSpi mpu;
  static Mpu6050 mpu6050;
  ImuDriver* order[] = {&bmi088, &icm45, &icm, &lsm, &mpu, &mpu6050};
  for (ImuDriver* d : order)
    if (d->init(loopHz)) return d;
  return nullptr;
}

// Second IMU on the IMU2 chip select (ArduPilot IMU redundancy). Not with a BMI088 as the first
// chip: its accelerometer uses that chip select.
ImuDriver* probeImu2(uint16_t loopHz, const ImuDriver* first) {
  if (!hal::spiHasDevice(SpiDev::IMU2) || (first && !strcmp(first->name(), "BMI088"))) return nullptr;
  static Icm456xx icm45;
  static Icm426xx icm;
  static Lsm6 lsm;
  static MpuSpi mpu;
  ImuDriver* order[] = {&icm45, &icm, &lsm, &mpu};
  for (ImuDriver* d : order) {
    d->setDev(SpiDev::IMU2);
    if (d->init(loopHz)) return d;
  }
  return nullptr;
}

Vec3 applyAlign(const Vec3& in, fc::Align a) {
  uint8_t idx = (uint8_t)a;
  Vec3 v = in;
  if (idx >= 4) { v.y = -v.y; v.z = -v.z; idx -= 4; }
  switch (idx) {
    case 1: return Vec3(-v.y, v.x, v.z);
    case 2: return Vec3(-v.x, -v.y, v.z);
    case 3: return Vec3(v.y, -v.x, v.z);
    default: return v;
  }
}

}  // namespace drv
