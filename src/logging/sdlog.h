// AllQuad - blackbox on an SD card (FAT32), one file per flight: /AQLOG/AQ00001.BBL ...
//
// The flight side never waits for the card. The logger task copies each blackbox frame into a RAM queue
// (push: whole frame or nothing, no lock - one producer, one consumer); the low-priority "sdcard" task
// (service) mounts the card, opens a file when the craft arms, writes the queue out in 4-16 kB pieces,
// commits the file length to the card every second (a crash or a cut battery loses at most ~1 s) and
// closes the file after disarm. A card that pauses for a few hundred ms while it erases internally is
// absorbed by the queue (32 kB ESP32, 64 kB STM32H7, 256 kB in the PSRAM of the S3 N16R8).
// Card modes (sd_mode): SPI, SD bus 1-bit, SD bus 4-bit - see fc::SdMode and hal::sdModes().
#pragma once
#include <stddef.h>
#include <stdint.h>

#include "../config/config.h"

namespace sdlog {

enum class State : uint8_t { OFF = 0, UNSUPPORTED, NO_CARD, READY, LOGGING, FULL, ERROR };
const char* stateName(State s);

struct Info {
  State state;
  uint8_t mode;
  const char* type;      // card type
  const char* err;       // last error (mount / write)
  uint64_t total, used;  // bytes
  char file[24];         // current or last log file
  uint32_t fileBytes;
  uint32_t queued, queueSize, dropped;  // bytes waiting, queue size, bytes lost (queue full / no card)
  uint32_t maxWriteMs;   // slowest card write so far
  uint16_t files;
  uint8_t pin[6];        // pins in use (255 = none)
};

constexpr const char* DIR = "/AQLOG";
constexpr uint32_t MIN_FREE = 16u << 20;  // stop / make room below 16 MB free

void begin(const fc::Config& c);  // pins, queue; the card is mounted later by service()
bool enabled();

// ---- logger task (never blocks) ----
void startFile();                          // armed: a new file (its header is the next push)
void endFile();                            // disarmed: the file closes once its data is written
bool push(const uint8_t* d, size_t n);     // false = dropped (queue full, no card)

// ---- "sdcard" task ----
void service(uint32_t nowMs);
void remount();                            // try the card again now (configurator)

Info info();

// ---- configurator; false = busy (a log is being written) or no card ----
bool list(void (*cb)(const char* name, uint32_t size, void* ctx), void* ctx);
bool read(const char* name, uint32_t off, uint8_t* buf, size_t len, size_t& got, uint32_t& size);
bool remove(const char* name);
bool validName(const char* name);          // AQnnnnn.BBL only (no paths)

}  // namespace sdlog
