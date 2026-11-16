// AllQuad - glue between the shared Arduino HAL and the chip-specific files
#pragma once
#ifdef ARDUINO
#include <Arduino.h>
#include <SPI.h>

#include "hal.h"

namespace hal {
namespace internal {
HardwareSerial* serialPort(Port p);  // SERIAL1..3, nullptr if absent
SPIClass* spiBus(SpiDev d);          // bus for a device
void i2cPrepare(int sda, int scl);  // line check + bus recovery before Wire.begin
void i2cStart();                    // (re)start the I2C peripheral on PIN_I2C_SDA / SCL at i2cClock()
int csPin(SpiDev d);                 // -1 if absent
// UDP link (ESP32 WiFi); other chips: unsupported
bool udpSupported();
int udpRead();
size_t udpWrite(const uint8_t* data, size_t len);
void commonInit();
PortStats& portStatsRef(Port p);    // counters kept for each opened UART
void portMarkOpen(Port p);
#if defined(AQ_CONSOLE_DUAL)
// console on two physical ports (native USB + USB-UART bridge on UART0)
int consoleRead();
size_t consoleWrite(const uint8_t* data, size_t len);
size_t consoleWriteSpace();
#endif
}  // namespace internal
}  // namespace hal
#endif
