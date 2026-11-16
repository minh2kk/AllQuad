// AllQuad - host (PC) HAL test hooks
#pragma once
#include <deque>
#include <functional>
#include <map>
#include <string>
#include <vector>

#include "../../src/hal/hal.h"

namespace hal {
namespace host {
std::deque<uint8_t>& in(Port p);   // bytes the firmware will read from a port
std::string& out(Port p);          // bytes the firmware wrote to a port
extern float motors[4];
extern bool motorsArmed;
extern int lastMotorCommand;
extern int lastMotorMask;
extern int motorCommandCount;
extern int lastTelemetryMotor;
extern std::vector<uint8_t> log;
extern float vbatPin, currentPin;
void reset();
// register-level fake chips for driver tests
extern std::map<uint8_t, uint8_t> spiRegs;               // IMU chip-select registers
extern std::function<void(uint8_t reg, uint8_t val)> spiOnWrite;
extern std::map<uint8_t, std::map<uint8_t, uint8_t>> i2cRegs;  // addr -> registers
extern std::map<uint8_t, std::map<uint8_t, uint8_t>> i2cWrites;  // last value written
extern bool fakeSpi;
extern size_t consoleSpace;       // room in the console TX buffer (tests throttle it)
extern uint32_t consoleBaudNow;    // 0 = behaves like USB CDC
// ESC passthrough: tests plug a byte-level fake ESC bootloader in here
extern std::function<void(int esc, const uint8_t* d, size_t n)> escTx;
extern std::function<int(int esc)> escRx;  // next byte from the ESC, -1 = nothing (timeout)
extern bool escActive;
extern std::vector<uint8_t> ledFrame;     // last LED strip frame (GRB)
extern int bootloaderRequests;
// bidirectional DShot: eRPM the simulator / tests put on the motor wires
extern bool bidir;
extern float erpm[4];
extern uint32_t bidirFrames;
extern dshot::Edt edt[4];  // extended DShot telemetry the fake ESCs report
// fake SD card: files in memory; tests insert / pull the card, make writes stall or fail, shrink it
struct SdCard {
  bool inserted = true;
  uint64_t capacity = 64ull << 20;
  int failAfter = -1;            // appends that succeed before the card stops accepting data (-1 never)
  int appends = 0, syncs = 0, mounts = 0;
  uint8_t lastMode = 0;
  std::map<std::string, std::vector<uint8_t>> files;
};
extern SdCard sd;
extern CrashInfo crash;     // the record lastCrash() reports (kind 0 = none)
extern bool wdtRunning;
extern uint32_t wdtKicks;
}  // namespace host
}  // namespace hal
