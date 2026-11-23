// AllQuad - Hardware Abstraction Layer
// Everything above this layer is chip independent. Implementations:
//   hal_arduino_common.cpp  Arduino code of the ESP32 targets (time, I2C, SPI, ports)
//   hal_esp32.cpp           RMT DShot, LEDC PWM, NVS, raw-partition log, WiFi UDP
//   hal_stm32cube.cpp       STM32 on STM32Cube (no Arduino): interrupt UARTs, HAL I2C, SPI, USB CDC,
//                           TIM1+DMA DShot, timer PWM, flash settings (+ src/hal/stm32/: clocks, USB, SD)
//   test/host/hal_host.cpp  PC simulator / unit tests
//
// Bus ownership (no locking needed): IMU SPI bus -> fast task, I2C -> sensors task,
// OSD SPI bus -> telemetry task, each serial port -> the task running its function.
#pragma once
#include <stddef.h>
#include <stdint.h>

#include "../config/config.h"
#include "dshot_codec.h"

namespace hal {

void init();
const char* platformName();
// the faster code paths of this chip and whether they are in use (CLI perf), "" = none
const char* fastPaths();

// ---- time ----
uint32_t micros();
uint32_t millis();
void delayMs(uint32_t ms);

// ---- I2C ----
bool i2cProbe(uint8_t addr);
bool i2cRead(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t len);
bool i2cWrite(uint8_t addr, uint8_t reg, uint8_t val);
bool i2cCommand(uint8_t addr, uint8_t cmd);           // single command byte (MS5611)
bool i2cReadRaw(uint8_t addr, uint8_t* buf, uint8_t len);
void i2cSetClock(uint32_t hz);                        // 400 kHz at start; 100 kHz fallback (weak pull-ups, long wires)
uint32_t i2cClock();
// wiring check before the bus started: SDA / SCL level with only the external pull-ups (internal pull-downs
// on), and whether a device holding SDA low had to be clocked free. Text for `scan` / `status`.
const char* i2cDiag();
// wiring check without the I2C peripheral: bit-banged address scan on the same two pins, normal or with
// SDA / SCL swapped (finds swapped wires and a peripheral / pin-mux problem). Stops the bus meanwhile:
// only while no sensor is in use. Returns the number of addresses found (written to out).
int i2cSoftScan(bool swapped, uint8_t* out, int max);
void i2cLineLevels(bool& sda, bool& scl);   // the lines right now

// ---- SPI ----
enum class SpiDev : uint8_t { IMU = 0, IMU2 = 1, OSD = 2 };  // IMU2 = BMI088 accelerometer CS
bool spiHasDevice(SpiDev d);
void spiSetSpeed(SpiDev d, uint32_t hz);
// reads `len` bytes from reg (bit7 set). dummy = discard one byte first (BMI088 accel).
bool spiRead(SpiDev d, uint8_t reg, uint8_t* buf, size_t len, bool dummy = false);
bool spiWrite(SpiDev d, uint8_t reg, uint8_t val);
bool spiWriteRaw(SpiDev d, const uint8_t* data, size_t len);  // chip-select around the whole buffer

// ---- serial ports ----
// SERIAL4 / 5 come after CONSOLE so the stored port indices (0-3) keep their meaning
enum class Port : uint8_t {
  SERIAL1 = 0, SERIAL2 = 1, SERIAL3 = 2, UDP = 3, CONSOLE = 4, SERIAL4 = 5, SERIAL5 = 6, SERIAL6 = 7, SERIAL7 = 8, COUNT = 9
};
enum class UartFormat : uint8_t { F8N1, F8E2, F8N2 };
bool portExists(Port p);
// what a serial port is on this board, for the configurator: "USART1 RX PA10 TX PA9" ("" = no such UART)
const char* portLabel(Port p);
void portOpen(Port p, uint32_t baud, UartFormat fmt = UartFormat::F8N1, bool inverted = false);
int portRead(Port p);  // -1 if empty
size_t portWrite(Port p, const uint8_t* data, size_t len);
size_t portWriteSpace(Port p);
// counters of an opened UART (since it was opened): bytes, line errors, bytes lost because the
// receive ring was full. false = no such port / not opened.
struct PortStats {
  uint32_t rxBytes, txBytes;
  uint32_t overruns;   // hardware overrun (a byte arrived before the previous one was read)
  uint32_t framing;    // framing errors / breaks (wrong baud, inverted signal, noise)
  uint32_t noise;      // noise / parity errors
  uint32_t dropped;    // receive ring full: the task did not read in time
};
bool portStats(Port p, PortStats& out);
// USB console: UART baud rate (921600 on ESP32 boards with a USB-UART bridge), 0 = native USB (CDC)
uint32_t consoleBaud();
// why the chip last started: 1 power-on, 3 software reset, 4 panic / exception, 5-7 watchdog,
// 8 deep sleep, 9 brownout, 0 unknown (ESP-IDF esp_reset_reason_t numbering; STM32 maps its flags)
int resetReason();
// ---- crash record and watchdog (STM32; ESP32 keeps its own: reset reason PANIC / WATCHDOG + coredump) ----
// A fault, failed assert, stack overflow, out-of-memory or watchdog reset leaves one record in RAM
// that survives the reset (not a power cycle). Shown by the CLI "crash" and the configurator.
struct CrashInfo {
  uint8_t kind;          // CrashKind
  uint32_t pc, lr;       // where the fault happened (fault kinds), 0 = unknown
  uint32_t cfsr, hfsr;   // Cortex-M fault status registers
  uint32_t addr;         // faulting data address when the CPU latched one, else 0
  uint32_t uptimeMs;     // time since boot when it happened
  uint16_t bootsAgo;     // 0 = the reset just before this boot
  uint16_t count;        // crashes recorded since the record was last cleared
  char where[24];        // task name, or file:line of an assert
};
enum CrashKind : uint8_t { CRASH_NONE = 0, CRASH_HARDFAULT, CRASH_MEMMANAGE, CRASH_BUSFAULT, CRASH_USAGEFAULT,
                           CRASH_STACK, CRASH_NOMEM, CRASH_ASSERT, CRASH_WATCHDOG };
inline const char* crashKindName(uint8_t kind) {
  static const char* const n[] = {"none", "hard fault", "memory fault", "bus fault", "usage fault", "stack overflow",
                                  "out of memory", "assert", "watchdog"};
  return kind < sizeof(n) / sizeof(n[0]) ? n[kind] : "?";
}
bool lastCrash(CrashInfo& out);  // false = no record
void crashClear();
// Hardware watchdog (STM32 IWDG, ~6.5 s): started once the tasks run, then kept alive only while the
// flight loop makes progress (app control task). It cannot be stopped until the next reset.
void watchdogStart();
void watchdogKick();
bool watchdogRunning();
int imuIntPin();  // GPIO wired to the IMU's INT1 (data ready), -1 = none
// one line with the bus pins of this board (CLI "scan")
const char* pinSummary();

// ---- motors ----
// out[]: 0..1 (idle already applied); out < 0 stops that motor. armed == false stops all.
// telemetryMotor: 0..3 sets the DShot telemetry-request bit for that motor (-1 none).
// bidir = bidirectional DShot (RPM telemetry on the signal wire, Betaflight dshot_bidir): DShot only
bool motorsInit(fc::MotorProtocol p, uint16_t loopHz, bool bidir = false);
void motorsWrite(const float out[4], bool armed, int telemetryMotor = -1);
// DShot special command (e.g. 20 = spin normal, 21 = reversed) sent 10x to all motors. Disarmed only.
void motorsCommand(uint8_t cmd);
void motorsCommandMask(uint8_t cmd, uint8_t mask);  // only the motors in mask (bit 0 = motor 1)
bool motorsAreDshot();
// bidirectional DShot: latest eRPM per motor from the ESC answers (< 0 = no valid answer for 100 ms)
bool motorsBidirActive();
void motorsErpm(float erpm[4]);
void motorsBidirStats(uint32_t& frames, uint32_t& errors);  // decoded answers / invalid ones
// Extended DShot telemetry of one output (AM32 / Bluejay / BLHeli_32 after DShot command 13):
// temperature, voltage, current, stress, status. false until the first EDT frame from that ESC.
bool motorsEdt(int output, dshot::Edt& out);

// ---- analog ----
float adcVbatPin();
float adcCurrentPin();  // < 0 if not wired

// ---- indicators ----
void ledSet(bool on);
void buzzerSet(bool on);

// ---- non-volatile storage ----
enum class Slot : uint8_t { CONFIG = 0, MISSION = 1, RALLY = 2, WIFI = 3, FENCE = 4 };
bool storageLoad(Slot s, void* data, size_t len);
bool storageSave(Slot s, const void* data, size_t len);
// where the settings live: copy in use, its save counter, copies kept (2 = A/B, survives a power loss
// during a save; 0 copies = key-value store such as the ESP32 NVS)
void storageInfo(int& copy, uint32_t& seq, int& copies);

// ---- onboard log storage (ESP32: raw flash partition; false elsewhere) ----
// In flight only 256-byte page programs happen (no erase, no filesystem metadata). Erasing is done
// in the background by logService() while disarmed.
bool logOpen();
size_t logWrite(const uint8_t* data, size_t len);
void logFlush();
void logErase();          // request: the space is erased sector by sector while disarmed
uint32_t logSize();
uint32_t logCapacity();   // bytes available for the log (0 = no onboard storage)
bool logRead(uint32_t offset, uint8_t* buf, size_t len);
void logService(bool armed);  // background erase; call often from a low-priority task
bool logBusy();               // erase in progress (arming is blocked: an erase stalls the CPU)
// The fast task calls this at the end of every iteration; flash page writes are timed into the
// idle gap that follows, so a write stall does not delay the next control loop.
void markFastLoopDone();

// ---- SD card (FAT32), used by logging/sdlog only ----
// Calls block for the card (a write can take 100-250 ms while the card erases internally), so only the
// low-priority "sdcard" task writes, and the configurator reads / deletes only while no log is open
// (sdlog's lock). One file open for writing and one for reading at a time.
// sdModes(): bit (1 << SdMode) for every mode this board can drive; sdPinsFixed(): the pins are wired on
// the PCB / fixed by the chip (the sd_pin parameters are ignored).
uint8_t sdModes();
bool sdPinsFixed();
void sdDefaultPins(uint8_t pin[6]);  // 255 = none
// pin[] already resolved (no 255 for the pins the mode needs). err: reason when false.
bool sdMount(uint8_t mode, uint8_t mhz, const uint8_t pin[6], char* err, size_t n);
void sdUnmount();
bool sdMounted();
// STM32H7 SDMMC: move the blocks with the controller's DMA (falls back to polled transfers by itself
// after a DMA error); other targets ignore it. Takes effect at the next mount.
void sdSetDma(bool on);
bool sdDmaActive();
const char* sdCardType();             // "SDHC", "SDSC", "MMC" ...
bool sdSpace(uint64_t& total, uint64_t& used);
// RAM for the log queue: as much as the chip can spare up to `want` (PSRAM on the S3 N16R8)
uint8_t* sdQueueAlloc(size_t want, size_t& got);
bool sdCreate(const char* path);      // open for writing (truncates); makes the folder if needed
size_t sdAppend(const uint8_t* d, size_t n);
bool sdSync();                        // file length + FAT on the card (survives a power cut)
void sdClose();
// directory listing: cb(name, size) for every file in `dir`
void sdList(const char* dir, void (*cb)(const char* name, uint32_t size, void* ctx), void* ctx);
bool sdRemove(const char* path);
// read part of a file (kept open between calls of the same path); size = file length
bool sdRead(const char* path, uint32_t off, uint8_t* buf, size_t len, size_t& got, uint32_t& size);
void sdReadClose();

// ---- wireless (ESP32: WiFi access point, MAVLink on UDP 14550) ----
bool wifiStart(const char* ssid, const char* pass);
void wifiSetEnabled(bool on);  // radio off/on (access point restarted with the last name/password)
// network diagnostics for the CLI "net": access point, connected phones and their DHCP address,
// and an HTTP request from the FC to its own web server (port 80). Writes text into out.
void netReport(char* out, size_t n);
uint32_t deviceId();           // chip-unique number (default network name)
uint32_t random32();           // hardware random (session tokens)

// ---- firmware slots (ESP32 over-the-air update with rollback; elsewhere: single image) ----
struct FirmwareInfo {
  const char* running;   // partition label, e.g. "ota_0"
  bool otaSupported;     // a second slot exists
  bool pendingVerify;    // new image on its first boot: rolled back if it does not confirm itself
  bool otaSigned = false;  // WiFi updates must be signed (public key built in, tools/ota_sign.py)
};
FirmwareInfo firmwareInfo();
void firmwareConfirm();  // self-test passed: keep this image
void firmwareReject();   // self-test failed: mark invalid, reboot into the previous image

void reboot();
// restart into the chip's own USB bootloader (ESP32-S3: ROM download mode on the same USB port,
// STM32: system DFU bootloader). false = not possible from software (ESP32: the flasher pulses
// DTR/RTS of the USB-UART bridge instead).
bool bootloaderSupported();
void rebootToBootloader();

// ---- ESC passthrough (BLHeli / Bluejay / AM32 4-way interface) ----
// The motor pins become one-wire 19200 baud serial lines to the ESC bootloaders.
bool escSerialBegin();                                    // motor output stopped, lines high (pull-up)
void escSerialEnd();                                      // motor output restored
void escSerialWrite(int esc, const uint8_t* data, size_t len);  // blocking, ~0.52 ms per byte
int escSerialRead(int esc, uint32_t timeoutMs);           // one byte, -1 on timeout

// ---- LED strip (WS2812 / SK6812, GRB) ----
bool ledStripBegin(uint8_t pin, uint8_t count);
void ledStripShow(const uint8_t* grb, uint8_t count);    // non-blocking

}  // namespace hal
