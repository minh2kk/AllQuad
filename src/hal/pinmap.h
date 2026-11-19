// AllQuad - what every pin of the board is wired to (from src/hal/board.h), for the configurator's pin map
// and the CLI command "pins". Static wiring only: what is in use now (port functions, OSD, SD card, LED strip
// ...) comes from the configuration and is put next to it by the reader.
#pragma once
#include <stdint.h>

namespace hal {

enum class PinGroup : uint8_t {
  IMU = 0,    // SPI bus of the IMU + chip selects + data-ready
  OSD,        // SPI bus of the MAX7456
  I2C,        // baro / compass
  UART,       // arg = hal::Port index, what = "RX" / "TX"
  MOTOR,      // arg = output 1-4
  ANALOG,     // battery voltage / current (ADC)
  LED,        // status LED
  BUZZER,
  LEDSTRIP,   // WS2812 data (STM32: fixed pin; ESP32: led_pin parameter, not listed here)
  SD,         // SD card (arg = 0 CLK, 1 CMD, 2 D0, 3 D1, 4 D2, 5 D3); board default (ESP32: sd_ parameters)
  RESERVED,   // used by the board / chip itself: USB, SWD, flash, PSRAM, crystal, keys
  COUNT
};

struct PinUse {
  int16_t pin;       // board.h number (ESP32: GPIO, STM32: port << 4 | n)
  PinGroup group;
  uint8_t arg;
  const char* what;  // "SCK", "CS", "RX", "VBAT" ... ("" for RESERVED)
};

// every pin the board uses (pins of -1 left out), in a fixed order; returns the count (<= max)
int pinMap(PinUse* out, int max);
// "GPIO18" / "PB13"
const char* pinText(int pin, char* buf, int len);
// board name of the map (the PC simulator / demo drone uses the STM32H743 DevEBox map)
const char* pinMapBoard();
const char* pinGroupName(PinGroup g);  // "imu", "osd", ... (JSON / CLI)
constexpr int PINMAP_MAX = 96;

}  // namespace hal
