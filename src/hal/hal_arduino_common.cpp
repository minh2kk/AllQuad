// AllQuad - HAL parts shared by every Arduino-based target (ESP32 + STM32)
#if defined(ARDUINO) && !defined(ALLQUAD_HOST)
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <string.h>

#include "board.h"
#include "hal.h"
#include "hal_internal.h"

namespace hal {

static uint32_t spiHz[3] = {1000000, 1000000, 1000000};

uint32_t micros() { return ::micros(); }
uint32_t millis() { return ::millis(); }
void delayMs(uint32_t ms) { ::delay(ms); }

// ---------------- I2C ----------------
bool i2cProbe(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

bool i2cRead(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)addr, (int)len) != (int)len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = (uint8_t)Wire.read();
  return true;
}

bool i2cReadRaw(uint8_t addr, uint8_t* buf, uint8_t len) {
  if (Wire.requestFrom((int)addr, (int)len) != (int)len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = (uint8_t)Wire.read();
  return true;
}

bool i2cWrite(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static uint32_t i2cHz = 400000;
void i2cSetClock(uint32_t hz) {
  i2cHz = hz;
  Wire.setClock(hz);
}
uint32_t i2cClock() { return i2cHz; }

// SDA / SCL before the I2C peripheral takes the pins. Internal pull-downs (~40 k) are on: a line reads
// high only if an external pull-up (the sensor module's 4.7 k / 10 k) holds it. A slave that was reset in
// the middle of a read can hold SDA low forever: up to 9 clocks + STOP release it (I2C bus recovery).
static char i2cDiagText[96] = "";
void internal::i2cPrepare(int sda, int scl) {
  pinMode(sda, INPUT_PULLDOWN);
  pinMode(scl, INPUT_PULLDOWN);
  delayMicroseconds(200);
  bool sdaHi = digitalRead(sda), sclHi = digitalRead(scl);
  int clocks = 0;
  if (sclHi && !sdaHi) {
    pinMode(sda, INPUT_PULLUP);
    pinMode(scl, OUTPUT_OPEN_DRAIN);
    for (; clocks < 9 && !digitalRead(sda); clocks++) {
      digitalWrite(scl, LOW);
      delayMicroseconds(5);
      digitalWrite(scl, HIGH);
      delayMicroseconds(5);
    }
    pinMode(sda, OUTPUT_OPEN_DRAIN);  // STOP: SDA low -> high while SCL is high
    digitalWrite(sda, LOW);
    delayMicroseconds(5);
    digitalWrite(sda, HIGH);
    delayMicroseconds(5);
    pinMode(sda, INPUT_PULLUP);
    pinMode(scl, INPUT_PULLUP);
  }
  snprintf(i2cDiagText, sizeof(i2cDiagText), "SDA %s, SCL %s%s%s", sdaHi ? "high" : "LOW", sclHi ? "high" : "LOW",
           !sdaHi || !sclHi ? " (no pull-up / short to GND / module unpowered)" : " (pull-ups ok)",
           clocks ? " - bus was stuck, freed" : "");
}
const char* i2cDiag() { return i2cDiagText; }

void i2cLineLevels(bool& sda, bool& scl) {
  sda = digitalRead(PIN_I2C_SDA);
  scl = digitalRead(PIN_I2C_SCL);
}

// ~10 kHz bit-banged I2C with the internal pull-ups (open drain: drive low, release high)
static void odSet(int p, bool high) {
  if (high) pinMode(p, INPUT_PULLUP);
  else { digitalWrite(p, LOW); pinMode(p, OUTPUT); }
  delayMicroseconds(50);
}
int i2cSoftScan(bool swapped, uint8_t* out, int max) {
  int sda = swapped ? PIN_I2C_SCL : PIN_I2C_SDA, scl = swapped ? PIN_I2C_SDA : PIN_I2C_SCL;
  Wire.end();
  odSet(sda, true);
  odSet(scl, true);
  int n = 0;
  for (uint8_t a = 0x08; a < 0x78; a++) {
    odSet(sda, false);  // START
    odSet(scl, false);
    uint8_t b = (uint8_t)(a << 1);  // write
    for (int i = 7; i >= 0; i--) {
      odSet(sda, (b >> i) & 1);
      odSet(scl, true);
      odSet(scl, false);
    }
    odSet(sda, true);  // release for ACK
    odSet(scl, true);
    bool ack = !digitalRead(sda);
    odSet(scl, false);
    odSet(sda, false);  // STOP
    odSet(scl, true);
    odSet(sda, true);
    if (ack && n < max) out[n++] = a;
  }
  internal::i2cStart();
  return n;
}

bool i2cCommand(uint8_t addr, uint8_t cmd) {
  Wire.beginTransmission(addr);
  Wire.write(cmd);
  return Wire.endTransmission() == 0;
}

// ---------------- SPI ----------------
const char* pinSummary() {
  static char b[160];
  if (!b[0])
    snprintf(b, sizeof(b), "SPI SCK %d MISO %d MOSI %d | IMU CS %d | I2C SDA %d SCL %d", (int)PIN_SPI_SCK,
             (int)PIN_SPI_MISO, (int)PIN_SPI_MOSI, (int)PIN_IMU_CS, (int)PIN_I2C_SDA, (int)PIN_I2C_SCL);
  return b;
}

bool spiHasDevice(SpiDev d) { return internal::csPin(d) >= 0 && internal::spiBus(d) != nullptr; }
void spiSetSpeed(SpiDev d, uint32_t hz) { spiHz[(int)d] = hz; }

bool spiRead(SpiDev d, uint8_t reg, uint8_t* buf, size_t len, bool dummy) {
  SPIClass* bus = internal::spiBus(d);
  int cs = internal::csPin(d);
  if (!bus || cs < 0) return false;
  bus->beginTransaction(SPISettings(spiHz[(int)d], MSBFIRST, SPI_MODE0));
  digitalWrite(cs, LOW);
  bus->transfer(reg | 0x80);
  if (dummy) bus->transfer(0x00);
  // one hardware burst instead of a call per byte (the IMU sample is read 1000-2000 times a second)
#if defined(ESP32)
  static const uint8_t zeros[64] = {0};
  for (size_t o = 0; o < len; o += 64) {
    size_t k = len - o < 64 ? len - o : 64;
    bus->transferBytes(zeros, buf + o, (uint32_t)k);
  }
#else
  memset(buf, 0, len);
  bus->transfer(buf, len);  // STM32duino: in-place buffer transfer
#endif
  digitalWrite(cs, HIGH);
  bus->endTransaction();
  return true;
}

bool spiWrite(SpiDev d, uint8_t reg, uint8_t val) {
  SPIClass* bus = internal::spiBus(d);
  int cs = internal::csPin(d);
  if (!bus || cs < 0) return false;
  uint32_t hz = spiHz[(int)d] < 1000000 ? spiHz[(int)d] : 1000000;  // registers: slow & safe
  if (d == SpiDev::OSD) hz = spiHz[(int)d];
  bus->beginTransaction(SPISettings(hz, MSBFIRST, SPI_MODE0));
  digitalWrite(cs, LOW);
  bus->transfer(reg & 0x7F);
  bus->transfer(val);
  digitalWrite(cs, HIGH);
  bus->endTransaction();
  return true;
}

bool spiWriteRaw(SpiDev d, const uint8_t* data, size_t len) {
  SPIClass* bus = internal::spiBus(d);
  int cs = internal::csPin(d);
  if (!bus || cs < 0) return false;
  bus->beginTransaction(SPISettings(spiHz[(int)d], MSBFIRST, SPI_MODE0));
  digitalWrite(cs, LOW);
  for (size_t i = 0; i < len; i++) bus->transfer(data[i]);
  digitalWrite(cs, HIGH);
  bus->endTransaction();
  return true;
}

// ---------------- serial ports ----------------
bool portExists(Port p) {
  if (p == Port::CONSOLE) return true;
  if (p == Port::UDP) return internal::udpSupported();
  return internal::serialPort(p) != nullptr;
}

int portRead(Port p) {
#if defined(AQ_CONSOLE_DUAL)
  if (p == Port::CONSOLE) return internal::consoleRead();
#endif
  if (p == Port::CONSOLE) return Serial.available() ? Serial.read() : -1;
  if (p == Port::UDP) return internal::udpRead();
  HardwareSerial* s = internal::serialPort(p);
  if (!s || !s->available()) return -1;
  return s->read();
}

size_t portWrite(Port p, const uint8_t* data, size_t len) {
#if defined(AQ_CONSOLE_DUAL)
  if (p == Port::CONSOLE) return internal::consoleWrite(data, len);
#endif
  if (p == Port::CONSOLE) return Serial.write(data, len);
  if (p == Port::UDP) return internal::udpWrite(data, len);
  HardwareSerial* s = internal::serialPort(p);
  return s ? s->write(data, len) : 0;
}

size_t portWriteSpace(Port p) {
#if defined(AQ_CONSOLE_DUAL)
  if (p == Port::CONSOLE) return internal::consoleWriteSpace();
#endif
  if (p == Port::CONSOLE) return (size_t)Serial.availableForWrite();
  if (p == Port::UDP) return 1024;
  HardwareSerial* s = internal::serialPort(p);
  return s ? (size_t)s->availableForWrite() : 0;
}

// ---------------- indicators ----------------
void ledSet(bool on) {
  if (PIN_LED >= 0) digitalWrite(PIN_LED, (on ^ (LED_ACTIVE_LOW != 0)) ? HIGH : LOW);
}
void buzzerSet(bool on) { digitalWrite(PIN_BUZZER, on ? HIGH : LOW); }

void internal::commonInit() {
  if (PIN_LED >= 0) pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  ledSet(false);
  buzzerSet(false);
  const SpiDev devs[] = {SpiDev::IMU, SpiDev::IMU2, SpiDev::OSD};
  for (SpiDev d : devs) {
    int cs = csPin(d);
    if (cs >= 0) { pinMode(cs, OUTPUT); digitalWrite(cs, HIGH); }
  }
}

}  // namespace hal
#endif
