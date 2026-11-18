// AllQuad - board pin maps. Select with -D BOARD_xxx in platformio.ini. Edit to match your wiring.
// A pin of -1 means "not present" (feature disabled on that board).
#pragma once
#if defined(AQ_STM32)
#include "stm32/stm32_mcu.h"  // PA0 ... PE15 for the STM32Cube builds
#endif

#if defined(BOARD_ESP32_DEVKIT)
// ---------------- ESP32 (WROOM-32 DevKit) ----------------
#define AQ_BOARD_NAME "ESP32-DevKit"
// kept by the chip / board (tools/check_pins.py): UART0 console, SPI flash
#define AQ_PINS_RESERVED "1 3 6 7 8 9 10 11"
// IMU SPI bus (VSPI)
#define PIN_SPI_SCK 18
#define PIN_SPI_MISO 19
#define PIN_SPI_MOSI 23
#define PIN_IMU_CS 5
#define PIN_IMU2_CS -1      // BMI088 accelerometer CS (gyro uses PIN_IMU_CS)
// OSD SPI bus (HSPI) - MAX7456
#define PIN_OSD_SCK 14
#define PIN_OSD_MISO 12
#define PIN_OSD_MOSI 15
#define PIN_OSD_CS -1       // set to e.g. 0 to enable an analog OSD chip
// I2C
#define PIN_I2C_SDA 21
#define PIN_I2C_SCL 22
// serial ports: SERIAL1 = UART2, SERIAL2 = UART1, SERIAL3 = none (UART0 is the USB console)
#define PIN_SERIAL1_RX 16
#define PIN_SERIAL1_TX 17
#define PIN_SERIAL2_RX 4
#define PIN_SERIAL2_TX 27
#define PIN_SERIAL3_RX -1
#define PIN_SERIAL3_TX -1
// outputs / analog
#define PIN_MOTOR1 13
#define PIN_MOTOR2 25
#define PIN_MOTOR3 32
#define PIN_MOTOR4 33
#define PIN_VBAT 34
#define PIN_CURRENT 35
#define PIN_LED 2
#define PIN_BUZZER 26
#define LED_ACTIVE_LOW 0

#elif defined(BOARD_ESP32S3_N16R8)
// ---------------- ESP32-S3 N16R8 (16 MB flash, 8 MB octal PSRAM) - AllQuad build of the owner ----------------
// Unusable on this module: GPIO 19/20 (native USB), 33-37 (octal PSRAM), 43/44 (UART0 -> CH343 "COM"
// port, used as a second console), 0/3/45/46 (strapping pins), 48 (on-board WS2812 RGB LED).
// Motors: outputs 1-4 = GPIO 4/5/6/7 = ESC "M1" front-left, "M2" front-right, "M3" rear-right,
// "M4" rear-left. The mixer numbers motors Betaflight style (M1 rear-right, M2 front-right, M3
// rear-left, M4 front-left): this build defaults motor1..4_out to 3, 2, 4, 1 (changeable in the
// configurator, Motor tab). Spin directions FL CCW, FR CW, RR CCW, RL CW = props-out: yaw_reversed 1.
#define AQ_BOARD_NAME "ESP32-S3-N16R8"
#define AQ_PINS_RESERVED "19 20 26 27 28 29 30 31 32 33 34 35 36 37 43 44"  // USB, flash, octal PSRAM, UART0 console
#define AQ_CONSOLE_DUAL 1
#define PIN_SPI_SCK 12      // ICM-45686
#define PIN_SPI_MISO 13
#define PIN_SPI_MOSI 11
#define PIN_IMU_CS 10
#define PIN_IMU2_CS -1
#define PIN_IMU_INT 14      // ICM-45686 INT1: data-ready pulses start the fast loop (imu_sync 1)
#define PIN_OSD_SCK 39      // MAX7456 (optional): set PIN_OSD_CS to 38 to enable
#define PIN_OSD_MISO 40
#define PIN_OSD_MOSI 41
#define PIN_OSD_CS -1
#define PIN_I2C_SDA 8       // baro / compass (GPS module)
#define PIN_I2C_SCL 9
#define PIN_SERIAL1_RX 18   // UART1: CRSF receiver (RP4TD TX -> 18, 17 -> RP4TD RX)
#define PIN_SERIAL1_TX 17
#define PIN_SERIAL2_RX 15   // UART2: GPS (GPS TX -> 15, 16 -> GPS RX)
#define PIN_SERIAL2_TX 16
#define PIN_SERIAL3_RX -1   // UART0 is the second console (COM port)
#define PIN_SERIAL3_TX -1
#define PIN_MOTOR1 4        // output 1: ESC "M1" front-left
#define PIN_MOTOR2 5        // output 2: ESC "M2" front-right
#define PIN_MOTOR3 6        // output 3: ESC "M3" rear-right
#define PIN_MOTOR4 7        // output 4: ESC "M4" rear-left
#define PIN_VBAT 1          // divider 3.3k / 1k: vbat_scale 4.3
#define PIN_CURRENT 2       // current sensor (optional)
#define PIN_LED 21          // status LED (optional, active high)
#define PIN_BUZZER 42       // buzzer through a transistor (optional)
#define LED_ACTIVE_LOW 0
#define PIN_LED_STRIP_HINT 48  // on-board RGB LED: set led_pin 48, led_count 1
// SD card module (sd_mode, v25): the free pins 38-41 / 47 (no MAX7456 fitted). Card pin -> GPIO:
// CLK/SCK 39, CMD/MOSI 38, D0/MISO 40, D3/CS 41, D1 47. SPI and 1-bit SD bus work as is; 4-bit needs one
// more free pin for D2 (sd_d2, e.g. 21 if the status LED is not used).
#define PIN_SD_CLK 39
#define PIN_SD_CMD 38
#define PIN_SD_D0 40
#define PIN_SD_D1 47
#define PIN_SD_D2 -1
#define PIN_SD_D3 41

#elif defined(BOARD_ESP32S3_DEVKIT)
// ---------------- ESP32-S3 DevKitC-1 ----------------
#define AQ_BOARD_NAME "ESP32-S3-DevKitC"
#define AQ_PINS_RESERVED "19 20 26 27 28 29 30 31 32"  // native USB, SPI flash
// DevKitC-1 (and the N16R8 clones) have two USB-C ports: "USB" (native USB) and "COM/UART" (a
// CH343 / CP2102 bridge on UART0). The console answers on both: whichever port last sent a byte
// gets the replies. UART0 is therefore not a free serial port (SERIAL3 does not exist).
#define AQ_CONSOLE_DUAL 1
#define PIN_SPI_SCK 12
#define PIN_SPI_MISO 13
#define PIN_SPI_MOSI 11
#define PIN_IMU_CS 10
#define PIN_IMU2_CS 9       // BMI088 accel (unused otherwise)
#if defined(AQ_S3_OPI_PSRAM)  // N8R8 / N16R8: GPIO 33-37 are wired to the octal PSRAM
#define PIN_OSD_SCK 39
#define PIN_OSD_MISO 40
#define PIN_OSD_MOSI 41
#else
#define PIN_OSD_SCK 36
#define PIN_OSD_MISO 37
#define PIN_OSD_MOSI 35
#endif
#define PIN_OSD_CS -1
#define PIN_I2C_SDA 8
#define PIN_I2C_SCL 3
// SERIAL1 = UART1, SERIAL2 = UART2, SERIAL3 = UART0 (console is native USB)
#define PIN_SERIAL1_RX 18
#define PIN_SERIAL1_TX 17
#define PIN_SERIAL2_RX 16
#define PIN_SERIAL2_TX 15
#define PIN_SERIAL3_RX 44
#define PIN_SERIAL3_TX 43
#define PIN_MOTOR1 4
#define PIN_MOTOR2 5
#define PIN_MOTOR3 6
#define PIN_MOTOR4 7
#define PIN_VBAT 1
#define PIN_CURRENT 2
#define PIN_LED 21
#define PIN_BUZZER 14
#define LED_ACTIVE_LOW 0
// SD card module (sd_mode): CLK/SCK 39, CMD/MOSI 38, D0/MISO 40, D1 47, D2 42, D3/CS 41
#define PIN_SD_CLK 39
#define PIN_SD_CMD 38
#define PIN_SD_D0 40
#define PIN_SD_D1 47
#define PIN_SD_D2 42
#define PIN_SD_D3 41

#elif defined(BOARD_STM32F411_BLACKPILL)
// ---------------- STM32F411CEU6 "Black Pill" ----------------
// Motors MUST stay on GPIOB (DShot DMA bit-bang on one port) and on TIM3 (PWM).
// TIM1 + DMA2 Stream5 (DShot) and TIM5 (scheduler) are reserved.
#define AQ_BOARD_NAME "STM32F411-BlackPill"
#define AQ_PINS_RESERVED "PA11 PA12 PA13 PA14 PC14 PC15"  // USB, SWD, 32 kHz crystal
#define PIN_SPI_SCK PA5     // SPI1
#define PIN_SPI_MISO PA6
#define PIN_SPI_MOSI PA7
#define PIN_IMU_CS PA4
#define PIN_IMU2_CS PB12
#define PIN_OSD_SCK PB13    // SPI2
#define PIN_OSD_MISO PB14
#define PIN_OSD_MOSI PB15
#define PIN_OSD_CS -1       // e.g. PA8
#define PIN_I2C_SDA PB7
#define PIN_I2C_SCL PB6
#define PIN_SERIAL1_RX PA3  // USART2
#define PIN_SERIAL1_TX PA2
#define PIN_SERIAL2_RX PA10 // USART1
#define PIN_SERIAL2_TX PA9
#define PIN_SERIAL3_RX -1   // USART6 shares pins with USB on this board
#define PIN_SERIAL3_TX -1
#define PIN_MOTOR1 PB4      // TIM3_CH1
#define PIN_MOTOR2 PB5      // TIM3_CH2
#define PIN_MOTOR3 PB0      // TIM3_CH3
#define PIN_MOTOR4 PB1      // TIM3_CH4
#define PIN_VBAT PA0
#define PIN_CURRENT PA1
#define PIN_LED PC13
#define PIN_BUZZER PB10
#define LED_ACTIVE_LOW 1
#define PIN_LED_STRIP PB8   // TIM4_CH3 + DMA1 Stream6 (WS2812)

#elif defined(BOARD_STM32F405) || defined(BOARD_STM32F722)
// ---------------- STM32F405RG / STM32F722RE flight controller (generic layout) ----------------
// Same rules as the Black Pill: motors on GPIOB / TIM3 (DShot bit-bang + PWM), TIM1 + DMA2 Stream5
// for DShot, TIM5 for the scheduler, TIM4 + DMA1 Stream6 for the LED strip. Commercial FCs route
// the motor pins differently: move them in this block (they must share one GPIO port).
#if defined(BOARD_STM32F722)
#define AQ_BOARD_NAME "STM32F722"
#else
#define AQ_BOARD_NAME "STM32F405"
#endif
#define AQ_PINS_RESERVED "PA11 PA12 PA13 PA14"  // USB, SWD
#define PIN_SPI_SCK PA5     // SPI1: IMU
#define PIN_SPI_MISO PA6
#define PIN_SPI_MOSI PA7
#define PIN_IMU_CS PA4
#define PIN_IMU2_CS PC4
#define PIN_OSD_SCK PB13    // SPI2: MAX7456
#define PIN_OSD_MISO PB14
#define PIN_OSD_MOSI PB15
#define PIN_OSD_CS PB12
#define PIN_I2C_SDA PB7     // I2C1: baro, compass
#define PIN_I2C_SCL PB6
#define PIN_SERIAL1_RX PA3  // USART2 (receiver)
#define PIN_SERIAL1_TX PA2
#define PIN_SERIAL2_RX PA10 // USART1 (GPS)
#define PIN_SERIAL2_TX PA9
#define PIN_SERIAL3_RX PC11 // UART4 on these pins (MAVLink / VTX / ...)
#define PIN_SERIAL3_TX PC10
#define PIN_SERIAL4_RX PD2  // UART5
#define PIN_SERIAL4_TX PC12
#define PIN_SERIAL5_RX PC7  // USART6
#define PIN_SERIAL5_TX PC6
#define PIN_SERIAL6_RX PB11 // USART3 (free on this layout: I2C is I2C1 on PB6 / PB7)
#define PIN_SERIAL6_TX PB10
#define PIN_MOTOR1 PB4      // TIM3_CH1
#define PIN_MOTOR2 PB5      // TIM3_CH2
#define PIN_MOTOR3 PB0      // TIM3_CH3
#define PIN_MOTOR4 PB1      // TIM3_CH4
#define PIN_VBAT PC1
#define PIN_CURRENT PC2
#define PIN_LED PC14
#define PIN_BUZZER PC13
#define LED_ACTIVE_LOW 0
#define PIN_LED_STRIP PB8   // TIM4_CH3

#elif defined(BOARD_STM32H743_WEACT) || defined(BOARD_STM32H743_DEVEBOX)
// ---------------- STM32H743VIT6, WeAct Studio MiniSTM32H743 (480 MHz Cortex-M7) ----------------
// Also MCUDev DevEBox STM32H7XX_M V2.0 (BOARD_STM32H743_DEVEBOX, env stm32h743_devebox): same chip, crystal
// (25 MHz), microSD (SDMMC1) and QSPI flash pins, so the same pin map; differences below (LED D2 sits on
// PA1 = motor 2, keys K1 PE3 / K2 PC5).
// Chosen around what the board already uses: USB (PA11/PA12), SWD, microSD (PC8-PC12, PD2), QSPI /
// SPI flash (PB2, PB6, PB3/PB4/PD7, PD11-PD13, PE2), LCD header (PE10-PE14), key PC13, LED PE3.
// The camera (DVP) header pins are reused for UART8 and the LED strip: leave the camera unplugged.
// 4-in-1 ESC (8-pin JST-SH): M1-M4 -> PA0-PA3, CURR -> PC1, TLM -> SERIAL3 RX (PE0) if wanted.
// The ESC's VBAT pin is the raw battery: NEVER to a GPIO, use a 10k/1k divider into PC0 (vbat_scale 11).
// The ESC has no BEC: power the board's 5V pin from a 5 V BEC (or the flight controller's).
#if defined(BOARD_STM32H743_DEVEBOX)
#define AQ_BOARD_NAME "STM32H743-DevEBox"
#define AQ_PINS_KEYS "PE3 PC5"  // keys K1 / K2
#else
#define AQ_BOARD_NAME "STM32H743-WeAct"
#define AQ_PINS_KEYS "PC13"     // key K1
#endif
// USB, SWD, 32 kHz crystal, microSD (SDMMC1), QSPI flash, SPI flash, LCD header, keys (tools/check_pins.py)
#define AQ_PINS_RESERVED "PA11 PA12 PA13 PA14 PC14 PC15 PC8 PC9 PC10 PC11 PC12 PD2 PB2 PB6 PD11 PD12 PD13 PE2 " \
                         "PB3 PB4 PD7 PE10 PE11 PE12 PE13 PE14 " AQ_PINS_KEYS
#define PIN_SPI_SCK PB13    // SPI2: IMU
#define PIN_SPI_MISO PB14
#define PIN_SPI_MOSI PB15
#define PIN_IMU_CS PB12
#define PIN_IMU2_CS PE15   // optional second IMU on the same SPI bus (imu_dual: failover)
#define PIN_IMU_INT PC4     // IMU INT1 (EXTI4) / data ready: the fast loop starts on its pulses (imu_sync 1)
#define PIN_OSD_SCK -1      // no MAX7456 on this board (HDZero / DJI / Walksnail: MSP DisplayPort)
#define PIN_OSD_MISO -1
#define PIN_OSD_MOSI -1
#define PIN_OSD_CS -1
#define PIN_I2C_SDA PB11    // I2C2: baro, compass
#define PIN_I2C_SCL PB10
#define PIN_SERIAL1_RX PD9  // USART3 (receiver: CRSF / ELRS)
#define PIN_SERIAL1_TX PD8
#define PIN_SERIAL2_RX PE7  // UART7 (GPS)
#define PIN_SERIAL2_TX PE8
#define PIN_SERIAL3_RX PE0  // UART8 (ESC telemetry / MSP DisplayPort / VTX / MAVLink)
#define PIN_SERIAL3_TX PE1
#define PIN_SERIAL4_RX PA10 // USART1 (header pins PA9 / PA10; VBUS sensing is off, PA9 stays a UART pin)
#define PIN_SERIAL4_TX PA9
#define PIN_SERIAL5_RX PD6  // USART2 (header pins; the other USART2 pins PA2 / PA3 are motors 3 / 4)
#define PIN_SERIAL5_TX PD5
#define PIN_SERIAL6_RX PD0  // UART4 (header pins; its other pins: PA0 / PA1 = motors, PB8 = LED strip)
#define PIN_SERIAL6_TX PD1
#define PIN_SERIAL7_RX PC7  // USART6 (camera header D0 / D1, like UART8: leave the camera unplugged)
#define PIN_SERIAL7_TX PC6
// UART5 stays unused: its pins are SDMMC (PC12 / PD2), SPI2 (PB12 / PB13) or QSPI / flash (PB5 / PB6)
#define PIN_MOTOR1 PA0      // TIM2_CH1 (DShot: bit-bang on GPIOA, TIM1 + DMA2 Stream5)
#define PIN_MOTOR2 PA1      // TIM2_CH2
#define PIN_MOTOR3 PA2      // TIM2_CH3
#define PIN_MOTOR4 PA3      // TIM2_CH4
#define PIN_VBAT PC0        // through a divider (10k/1k: 6S max ~36 V)
#define PIN_CURRENT PC1     // 4-in-1 ESC current sensor output (CURR)
#if defined(BOARD_STM32H743_DEVEBOX)
#define PIN_LED -1          // the board's LED D2 is on PA1 (motor 2: it glows with the motor signal); PE3 is key K1
#else
#define PIN_LED PE3         // onboard blue LED
#endif
#define PIN_BUZZER PD14     // via a transistor (5 V active buzzer)
#define LED_ACTIVE_LOW 0
#define PIN_LED_STRIP PB8   // TIM4_CH3 (DMA1 Stream6)
#define AQ_HAS_FLASH_LOG 1  // blackbox in the internal flash (bank 2, 896 kB)
#define AQ_HAS_SDMMC 1      // microSD slot on the board: SDMMC1 4-bit (PC8-PC11, PC12 CLK, PD2 CMD)

#elif defined(BOARD_ESP32C6_DEVKIT)
// ---------------- ESP32-C6 DevKitC-1 (single core RISC-V, WiFi 6) ----------------
// The C6 has 2 RMT transmit channels: 4 DShot outputs do not fit, motors run OneShot125 / PWM.
#define AQ_BOARD_NAME "ESP32-C6-DevKitC"
#define AQ_PINS_RESERVED "12 13 24 25 26 27 28 29 30"  // native USB, SPI flash
#define PIN_SPI_SCK 20
#define PIN_SPI_MISO 19
#define PIN_SPI_MOSI 18
#define PIN_IMU_CS 21
#define PIN_IMU2_CS -1
#define PIN_OSD_SCK -1      // one general-purpose SPI bus only
#define PIN_OSD_MISO -1
#define PIN_OSD_MOSI -1
#define PIN_OSD_CS -1
#define PIN_I2C_SDA 22
#define PIN_I2C_SCL 23
#define PIN_SERIAL1_RX 4    // UART1
#define PIN_SERIAL1_TX 5
#define PIN_SERIAL2_RX 17   // UART0 (the console is the native USB port)
#define PIN_SERIAL2_TX 16
#define PIN_SERIAL3_RX -1
#define PIN_SERIAL3_TX -1
#define PIN_MOTOR1 10
#define PIN_MOTOR2 11
#define PIN_MOTOR3 2
#define PIN_MOTOR4 3
#define PIN_VBAT 0
#define PIN_CURRENT 1
#define PIN_LED 15
#define PIN_BUZZER 7
#define LED_ACTIVE_LOW 0

#elif defined(BOARD_ESP32P4)
// ---------------- ESP32-P4 (dual core RISC-V 400 MHz, no radio on chip) ----------------
// WiFi needs an ESP32-C6 companion (ESP-Hosted); without it the configurator runs over USB.
#define AQ_BOARD_NAME "ESP32-P4"
#define AQ_PINS_RESERVED "24 25"  // USB serial / JTAG
#define PIN_SPI_SCK 36
#define PIN_SPI_MISO 37
#define PIN_SPI_MOSI 38
#define PIN_IMU_CS 39
#define PIN_IMU2_CS 40
#define PIN_OSD_SCK 41
#define PIN_OSD_MISO 42
#define PIN_OSD_MOSI 43
#define PIN_OSD_CS -1
#define PIN_I2C_SDA 7
#define PIN_I2C_SCL 8
#define PIN_SERIAL1_RX 21
#define PIN_SERIAL1_TX 20
#define PIN_SERIAL2_RX 23
#define PIN_SERIAL2_TX 22
#define PIN_SERIAL3_RX -1
#define PIN_SERIAL3_TX -1
#define PIN_MOTOR1 2
#define PIN_MOTOR2 3
#define PIN_MOTOR3 4
#define PIN_MOTOR4 5
#define PIN_VBAT 16
#define PIN_CURRENT 17
#define PIN_LED 47
#define PIN_BUZZER 48
#define LED_ACTIVE_LOW 0

#else
#error "No board selected: see the BOARD_xxx blocks in src/hal/board.h (platformio.ini sets one per env)"
#endif

// SD card pins (ESP32 boards; -1 = none: set the sd_ parameters)
#ifndef PIN_SD_CLK
#define PIN_SD_CLK -1
#define PIN_SD_CMD -1
#define PIN_SD_D0 -1
#define PIN_SD_D1 -1
#define PIN_SD_D2 -1
#define PIN_SD_D3 -1
#endif

// serial ports 4-7: only boards with spare UARTs define them
#ifndef PIN_SERIAL4_RX
#define PIN_SERIAL4_RX -1
#define PIN_SERIAL4_TX -1
#endif
#ifndef PIN_SERIAL5_RX
#define PIN_SERIAL5_RX -1
#define PIN_SERIAL5_TX -1
#endif
#ifndef PIN_SERIAL6_RX
#define PIN_SERIAL6_RX -1
#define PIN_SERIAL6_TX -1
#endif
#ifndef PIN_SERIAL7_RX
#define PIN_SERIAL7_RX -1
#define PIN_SERIAL7_TX -1
#endif
