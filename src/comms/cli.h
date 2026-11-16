// AllQuad - text command line on the USB console (shares the port with MSP & MAVLink)
#pragma once
#include <stddef.h>
#include <stdint.h>

#include "../hal/hal.h"

namespace comms {

struct SystemInfo {
  const char* imu;
  const char* baro;
  const char* mag;
  const char* version;
};

class Cli {
 public:
  void begin(hal::Port p, const SystemInfo* info) { port_ = p; info_ = info; }
  // feed one byte; binaryActive suppresses replies while MSP/MAVLink traffic is present
  void feed(uint8_t c, uint32_t nowMs, bool binaryActive);
  void print(const char* fmt, ...);
  // redirect the output (web backup / restore); nullptr = back to the port
  using Out = void (*)(const char* data, size_t len, void* ctx);
  void setOutput(Out o, void* ctx) { out_ = o; outCtx_ = ctx; }
  // execute one command line (restore from a backup file)
  void run(const char* line);
  // backup: "set" lines (all = every parameter, else only those that differ from the defaults),
  // mission and rally points; pasting it back into the CLI restores the configuration
  void diff(bool all);

 private:
  void exec(char* line, uint32_t nowMs);
  void cmdRally(const char* a1, char* rest);
  void cmdStatus();
  void cmdTasks();
  void cmdGet(const char* filter);
  void cmdMission(const char* arg);
  hal::Port port_ = hal::Port::CONSOLE;
  const SystemInfo* info_ = nullptr;
  char line_[128];
  Out out_ = nullptr;
  void* outCtx_ = nullptr;
  uint8_t pos_ = 0;
  bool garbage_ = false;
};

}  // namespace comms
