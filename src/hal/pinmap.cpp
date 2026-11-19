#include "../common/size_opt.h"  // -Os on ESP32 (not flight critical)
#include "pinmap.h"

#include <stdio.h>
#include <string.h>

#if defined(ALLQUAD_HOST)
// PC simulator / WebAssembly demo drone: the map of the STM32H743 DevEBox (src/hal/board.h)
#define AQ_PIN(port, n) (((port) << 4) | (n))
#define AQ_PIN_STM 1
#define AQ_MAP_BOARD "STM32H743-DevEBox (drone ảo)"
#define AQ_PINS_RESERVED "PA11 PA12 PA13 PA14 PC14 PC15 PC8 PC9 PC10 PC11 PC12 PD2 PB2 PB6 PD11 PD12 PD13 PE2 " \
                         "PB3 PB4 PD7 PE10 PE11 PE12 PE13 PE14 PE3 PC5"
#define PIN_SPI_SCK AQ_PIN(1, 13)
#define PIN_SPI_MISO AQ_PIN(1, 14)
#define PIN_SPI_MOSI AQ_PIN(1, 15)
#define PIN_IMU_CS AQ_PIN(1, 12)
#define PIN_IMU2_CS AQ_PIN(4, 15)
#define PIN_IMU_INT AQ_PIN(2, 4)
#define PIN_OSD_SCK -1
#define PIN_OSD_MISO -1
#define PIN_OSD_MOSI -1
#define PIN_OSD_CS -1
#define PIN_I2C_SDA AQ_PIN(1, 11)
#define PIN_I2C_SCL AQ_PIN(1, 10)
#define PIN_SERIAL1_RX AQ_PIN(3, 9)
#define PIN_SERIAL1_TX AQ_PIN(3, 8)
#define PIN_SERIAL2_RX AQ_PIN(4, 7)
#define PIN_SERIAL2_TX AQ_PIN(4, 8)
#define PIN_SERIAL3_RX AQ_PIN(4, 0)
#define PIN_SERIAL3_TX AQ_PIN(4, 1)
#define PIN_SERIAL4_RX AQ_PIN(0, 10)
#define PIN_SERIAL4_TX AQ_PIN(0, 9)
#define PIN_SERIAL5_RX AQ_PIN(3, 6)
#define PIN_SERIAL5_TX AQ_PIN(3, 5)
#define PIN_SERIAL6_RX AQ_PIN(3, 0)
#define PIN_SERIAL6_TX AQ_PIN(3, 1)
#define PIN_SERIAL7_RX AQ_PIN(2, 7)
#define PIN_SERIAL7_TX AQ_PIN(2, 6)
#define PIN_MOTOR1 AQ_PIN(0, 0)
#define PIN_MOTOR2 AQ_PIN(0, 1)
#define PIN_MOTOR3 AQ_PIN(0, 2)
#define PIN_MOTOR4 AQ_PIN(0, 3)
#define PIN_VBAT AQ_PIN(2, 0)
#define PIN_CURRENT AQ_PIN(2, 1)
#define PIN_LED -1
#define PIN_BUZZER AQ_PIN(3, 14)
#define PIN_LED_STRIP AQ_PIN(1, 8)
#define PIN_SD_CLK AQ_PIN(2, 12)
#define PIN_SD_CMD AQ_PIN(3, 2)
#define PIN_SD_D0 AQ_PIN(2, 8)
#define PIN_SD_D1 AQ_PIN(2, 9)
#define PIN_SD_D2 AQ_PIN(2, 10)
#define PIN_SD_D3 AQ_PIN(2, 11)
#else
#include "board.h"
#if defined(AQ_STM32)
#define AQ_PIN_STM 1
#endif
#define AQ_MAP_BOARD AQ_BOARD_NAME
#endif

#if defined(AQ_STM32) && defined(AQ_HAS_SDMMC) && PIN_SD_CLK < 0
// the board's microSD slot on SDMMC1 (fixed pins, listed as reserved too)
#undef PIN_SD_CLK
#undef PIN_SD_CMD
#undef PIN_SD_D0
#undef PIN_SD_D1
#undef PIN_SD_D2
#undef PIN_SD_D3
#define PIN_SD_CLK AQ_PIN(2, 12)
#define PIN_SD_CMD AQ_PIN(3, 2)
#define PIN_SD_D0 AQ_PIN(2, 8)
#define PIN_SD_D1 AQ_PIN(2, 9)
#define PIN_SD_D2 AQ_PIN(2, 10)
#define PIN_SD_D3 AQ_PIN(2, 11)
#endif
#ifndef PIN_IMU_INT
#define PIN_IMU_INT -1
#endif
#ifndef PIN_LED_STRIP
#define PIN_LED_STRIP -1
#endif
#ifndef AQ_PINS_RESERVED
#define AQ_PINS_RESERVED ""
#endif

namespace hal {

const char* pinText(int pin, char* buf, int len) {
  if (pin < 0) snprintf(buf, len, "-");
#if defined(AQ_PIN_STM)
  else snprintf(buf, len, "P%c%d", 'A' + (pin >> 4), pin & 15);
#else
  else snprintf(buf, len, "GPIO%d", pin);
#endif
  return buf;
}

const char* pinMapBoard() { return AQ_MAP_BOARD; }

const char* pinGroupName(PinGroup g) {
  static const char* const n[] = {"imu", "osd", "i2c", "uart", "motor", "adc", "led", "buzzer", "ledstrip", "sd", "reserved"};
  return (int)g < (int)PinGroup::COUNT ? n[(int)g] : "?";
}

// "PA11 PC14" / "1 3 6": board.h spelling -> pin number (-1 when not understood)
static int parsePin(const char* s, int n) {
#if defined(AQ_PIN_STM)
  if (n < 3 || s[0] != 'P' || s[1] < 'A' || s[1] > 'K') return -1;
  int k = 0;
  for (int i = 2; i < n; i++) {
    if (s[i] < '0' || s[i] > '9') return -1;
    k = k * 10 + (s[i] - '0');
  }
  return k < 16 ? ((s[1] - 'A') << 4) | k : -1;
#else
  int k = 0;
  for (int i = 0; i < n; i++) {
    if (s[i] < '0' || s[i] > '9') return -1;
    k = k * 10 + (s[i] - '0');
  }
  return n ? k : -1;
#endif
}

int pinMap(PinUse* out, int max) {
  int n = 0;
  auto add = [&](int pin, PinGroup g, uint8_t arg, const char* what) {
    if (pin >= 0 && n < max) out[n++] = PinUse{(int16_t)pin, g, arg, what};
  };
  add(PIN_SPI_SCK, PinGroup::IMU, 0, "SCK");
  add(PIN_SPI_MISO, PinGroup::IMU, 0, "MISO");
  add(PIN_SPI_MOSI, PinGroup::IMU, 0, "MOSI");
  add(PIN_IMU_CS, PinGroup::IMU, 0, "CS");
  add(PIN_IMU2_CS, PinGroup::IMU, 1, "CS2");
  add(PIN_IMU_INT, PinGroup::IMU, 0, "INT");
  if (PIN_OSD_CS >= 0) {  // no chip select: no OSD chip, its bus pins stay free
    add(PIN_OSD_SCK, PinGroup::OSD, 0, "SCK");
    add(PIN_OSD_MISO, PinGroup::OSD, 0, "MISO");
    add(PIN_OSD_MOSI, PinGroup::OSD, 0, "MOSI");
    add(PIN_OSD_CS, PinGroup::OSD, 0, "CS");
  }
  add(PIN_I2C_SDA, PinGroup::I2C, 0, "SDA");
  add(PIN_I2C_SCL, PinGroup::I2C, 0, "SCL");
  static const int16_t uart[7][3] = {
      {0, PIN_SERIAL1_RX, PIN_SERIAL1_TX}, {1, PIN_SERIAL2_RX, PIN_SERIAL2_TX}, {2, PIN_SERIAL3_RX, PIN_SERIAL3_TX},
      {5, PIN_SERIAL4_RX, PIN_SERIAL4_TX}, {6, PIN_SERIAL5_RX, PIN_SERIAL5_TX}, {7, PIN_SERIAL6_RX, PIN_SERIAL6_TX},
      {8, PIN_SERIAL7_RX, PIN_SERIAL7_TX}};
  for (const auto& u : uart) {
    if (u[1] < 0 || u[2] < 0) continue;
    add(u[1], PinGroup::UART, (uint8_t)u[0], "RX");
    add(u[2], PinGroup::UART, (uint8_t)u[0], "TX");
  }
  const int16_t mot[4] = {PIN_MOTOR1, PIN_MOTOR2, PIN_MOTOR3, PIN_MOTOR4};
  for (int i = 0; i < 4; i++) add(mot[i], PinGroup::MOTOR, (uint8_t)(i + 1), "OUT");
  add(PIN_VBAT, PinGroup::ANALOG, 0, "VBAT");
  add(PIN_CURRENT, PinGroup::ANALOG, 1, "CURRENT");
  add(PIN_LED, PinGroup::LED, 0, "LED");
  add(PIN_BUZZER, PinGroup::BUZZER, 0, "BUZZER");
  add(PIN_LED_STRIP, PinGroup::LEDSTRIP, 0, "DATA");
  static const char* const sdn[6] = {"CLK", "CMD", "D0", "D1", "D2", "D3"};
  const int16_t sd[6] = {PIN_SD_CLK, PIN_SD_CMD, PIN_SD_D0, PIN_SD_D1, PIN_SD_D2, PIN_SD_D3};
  for (int i = 0; i < 6; i++) add(sd[i], PinGroup::SD, (uint8_t)i, sdn[i]);
  // reserved pins not already listed (the H743 microSD pins are both)
  const char* r = AQ_PINS_RESERVED;
  while (*r) {
    while (*r == ' ') r++;
    int len = 0;
    while (r[len] && r[len] != ' ') len++;
    int p = parsePin(r, len);
    bool seen = false;
    for (int i = 0; i < n && !seen; i++) seen = out[i].pin == p;
    if (p >= 0 && !seen) add(p, PinGroup::RESERVED, 0, "");
    r += len;
  }
  return n;
}

}  // namespace hal
