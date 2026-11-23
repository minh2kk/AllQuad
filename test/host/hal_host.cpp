// AllQuad - HAL implementation for the PC simulator and unit tests.
// No real buses: sensors are provided by simulated drivers; ports are in-memory queues.
#include "hal_host.h"

#include <stdio.h>
#include <string.h>

#include "../../src/os/os.h"

namespace hal {
namespace host {
static std::deque<uint8_t> inQ[(int)Port::COUNT];
static std::string outQ[(int)Port::COUNT];
float motors[4] = {0, 0, 0, 0};
bool motorsArmed = false;
int lastMotorCommand = -1;
int lastMotorMask = 0;
int motorCommandCount = 0;
int lastTelemetryMotor = -1;
std::vector<uint8_t> log;
float vbatPin = 1.5f, currentPin = -1.0f;
std::map<uint8_t, uint8_t> spiRegs;
std::function<void(uint8_t, uint8_t)> spiOnWrite;
std::map<uint8_t, std::map<uint8_t, uint8_t>> i2cRegs;
std::map<uint8_t, std::map<uint8_t, uint8_t>> i2cWrites;
bool fakeSpi = false;
std::deque<uint8_t>& in(Port p) { return inQ[(int)p]; }
std::string& out(Port p) { return outQ[(int)p]; }
size_t consoleSpace = 4096;
std::function<void(int, const uint8_t*, size_t)> escTx;
std::function<int(int)> escRx;
bool escActive = false;
std::vector<uint8_t> ledFrame;
int bootloaderRequests = 0;
CrashInfo crash{};
bool wdtRunning = false;
uint32_t wdtKicks = 0;
uint32_t openBaud[(int)Port::COUNT] = {};
uint32_t consoleBaudNow = 115200;
bool bidir = false;
float erpm[4] = {-1, -1, -1, -1};
uint32_t bidirFrames = 0;
dshot::Edt edt[4];
SdCard sd;
void reset() {
  sd = SdCard();
  crash = CrashInfo{};
  wdtRunning = false;
  wdtKicks = 0;
  consoleSpace = 4096;
  consoleBaudNow = 115200;
  bidir = false;
  for (float& e : erpm) e = -1;
  for (dshot::Edt& d : edt) d = dshot::Edt();
  for (int i = 0; i < (int)Port::COUNT; i++) { inQ[i].clear(); outQ[i].clear(); }
  memset(motors, 0, sizeof(motors));
  motorsArmed = false;
  lastMotorCommand = -1;
  motorCommandCount = 0;
  log.clear();
  spiRegs.clear();
  spiOnWrite = nullptr;
  i2cRegs.clear();
  i2cWrites.clear();
  fakeSpi = false;
}
}  // namespace host

static uint8_t slots[5][4096];
static bool slotValid[5] = {false, false, false, false, false};

void init() {}
const char* platformName() { return "host-sim"; }
const char* fastPaths() { return ""; }
uint32_t micros() { return (uint32_t)os::timeUs(); }
uint32_t millis() { return os::timeMs(); }
void delayMs(uint32_t) {}

static uint32_t hostI2cHz = 400000;
void i2cSetClock(uint32_t hz) { hostI2cHz = hz; }
uint32_t i2cClock() { return hostI2cHz; }
const char* i2cDiag() { return "SDA high, SCL high (pull-ups ok)"; }
int i2cSoftScan(bool, uint8_t*, int) { return 0; }
void i2cLineLevels(bool& sda, bool& scl) { sda = scl = true; }
bool i2cProbe(uint8_t a) { return host::i2cRegs.count(a) > 0; }
bool i2cRead(uint8_t a, uint8_t reg, uint8_t* b, uint8_t n) {
  auto it = host::i2cRegs.find(a);
  if (it == host::i2cRegs.end()) return false;
  for (uint8_t i = 0; i < n; i++) b[i] = it->second.count((uint8_t)(reg + i)) ? it->second[(uint8_t)(reg + i)] : 0;
  return true;
}
bool i2cWrite(uint8_t a, uint8_t reg, uint8_t v) {
  if (!host::i2cRegs.count(a)) return false;
  host::i2cWrites[a][reg] = v;
  return true;
}
bool i2cCommand(uint8_t, uint8_t) { return false; }
bool i2cReadRaw(uint8_t, uint8_t*, uint8_t) { return false; }

bool spiHasDevice(SpiDev d) { return host::fakeSpi && d == SpiDev::IMU; }
void spiSetSpeed(SpiDev, uint32_t) {}
bool spiRead(SpiDev d, uint8_t reg, uint8_t* buf, size_t len, bool) {
  if (!host::fakeSpi || d != SpiDev::IMU) { memset(buf, 0xFF, len); return false; }
  for (size_t i = 0; i < len; i++) {
    uint8_t r = (uint8_t)((reg & 0x7F) + i);
    buf[i] = host::spiRegs.count(r) ? host::spiRegs[r] : 0x00;
  }
  return true;
}
bool spiWrite(SpiDev d, uint8_t reg, uint8_t v) {
  if (!host::fakeSpi || d != SpiDev::IMU) return false;
  reg &= 0x7F;
  if (host::spiOnWrite) host::spiOnWrite(reg, v);
  else host::spiRegs[reg] = v;
  return true;
}
bool spiWriteRaw(SpiDev, const uint8_t*, size_t) { return false; }

bool portExists(Port) { return true; }
// the UARTs of the STM32H743 DevEBox map (src/hal/pinmap.cpp uses the same map for the demo drone)
const char* portLabel(Port p) {
  static const char* const l[] = {"USART3 RX PD9 TX PD8", "UART7 RX PE7 TX PE8", "UART8 RX PE0 TX PE1", "", "",
                                  "USART1 RX PA10 TX PA9", "USART2 RX PD6 TX PD5", "UART4 RX PD0 TX PD1", "USART6 RX PC7 TX PC6"};
  return (int)p < (int)(sizeof(l) / sizeof(l[0])) ? l[(int)p] : "";
}
void portOpen(Port p, uint32_t baud, UartFormat, bool) { if ((int)p < (int)Port::COUNT) host::openBaud[(int)p] = baud; }
int portRead(Port p) {
  auto& q = host::inQ[(int)p];
  if (q.empty()) return -1;
  int c = q.front();
  q.pop_front();
  return c;
}
size_t portWrite(Port p, const uint8_t* d, size_t n) {
  host::outQ[(int)p].append((const char*)d, n);
  return n;
}
bool portStats(Port p, PortStats& o) {
  if ((int)p >= (int)Port::COUNT) return false;
  o = PortStats{};
  o.txBytes = (uint32_t)host::outQ[(int)p].size();
  return true;
}
size_t portWriteSpace(Port p) { return p == Port::CONSOLE ? host::consoleSpace : 4096; }
uint32_t consoleBaud() { return host::consoleBaudNow; }
int resetReason() { return 1; }
bool lastCrash(CrashInfo& o) {
  if (!host::crash.kind) return false;
  o = host::crash;
  return true;
}
void crashClear() { host::crash = CrashInfo{}; }
void watchdogStart() { host::wdtRunning = true; }
void watchdogKick() { host::wdtKicks++; }
bool watchdogRunning() { return host::wdtRunning; }
int imuIntPin() { return -1; }
const char* pinSummary() { return "host"; }

bool motorsInit(fc::MotorProtocol, uint16_t, bool bidir) { host::bidir = bidir; return true; }
bool motorsBidirActive() { return host::bidir; }
void motorsErpm(float e[4]) { for (int i = 0; i < 4; i++) e[i] = host::bidir ? host::erpm[i] : -1; }
void motorsBidirStats(uint32_t& f, uint32_t& e) { f = host::bidirFrames; e = 0; }
bool motorsEdt(int o, dshot::Edt& out) {
  if (o < 0 || o > 3 || !host::edt[o].frames) return false;
  out = host::edt[o];
  return true;
}
void motorsWrite(const float out[4], bool armed, int telemetryMotor) {
  for (int i = 0; i < 4; i++) host::motors[i] = (armed && out[i] >= 0) ? out[i] : 0.0f;
  host::motorsArmed = armed;
  host::lastTelemetryMotor = telemetryMotor;
}
void motorsCommandMask(uint8_t cmd, uint8_t mask) {
  host::lastMotorCommand = cmd;
  host::lastMotorMask = mask;
  host::motorCommandCount++;
}
void motorsCommand(uint8_t cmd) {
  host::lastMotorCommand = cmd;
  host::motorCommandCount++;
}
bool motorsAreDshot() { return true; }

float adcVbatPin() { return host::vbatPin; }
float adcCurrentPin() { return host::currentPin; }
void ledSet(bool) {}
void buzzerSet(bool) {}

bool storageLoad(Slot s, void* d, size_t n) {
  int i = (int)s;
  if (!slotValid[i] || n > sizeof(slots[i])) return false;
  memcpy(d, slots[i], n);
  return true;
}
void storageInfo(int& copy, uint32_t& seq, int& copies) { copy = 0; seq = 0; copies = 1; }
bool storageSave(Slot s, const void* d, size_t n) {
  int i = (int)s;
  if (n > sizeof(slots[i])) return false;
  memcpy(slots[i], d, n);
  slotValid[i] = true;
  return true;
}

// ---- SD card (host fake) ----
static bool sdUp = false;
static std::string sdWr, sdRd;
uint8_t sdModes() { return 0x0E; }
bool sdPinsFixed() { return false; }
void sdDefaultPins(uint8_t pin[6]) { for (int i = 0; i < 6; i++) pin[i] = (uint8_t)(30 + i); }
bool sdMount(uint8_t mode, uint8_t, const uint8_t*, char* err, size_t n) {
  host::sd.mounts++;
  host::sd.lastMode = mode;
  if (!host::sd.inserted) { snprintf(err, n, "no card"); return sdUp = false; }
  return sdUp = true;
}
void sdUnmount() { sdUp = false; sdWr.clear(); }
bool sdMounted() { return sdUp && host::sd.inserted; }
void sdSetDma(bool) {}
bool sdDmaActive() { return false; }
const char* sdCardType() { return "SDHC"; }
bool sdSpace(uint64_t& total, uint64_t& used) {
  if (!sdMounted()) return false;
  total = host::sd.capacity;
  used = 0;
  for (auto& f : host::sd.files) used += f.second.size();
  return true;
}
uint8_t* sdQueueAlloc(size_t want, size_t& got) {
  static uint8_t q[64 * 1024];
  got = want < sizeof(q) ? want : sizeof(q);
  return q;
}
bool sdCreate(const char* path) {
  if (!sdMounted()) return false;
  sdWr = path;
  host::sd.files[sdWr].clear();
  return true;
}
size_t sdAppend(const uint8_t* d, size_t n) {
  if (!sdMounted() || sdWr.empty()) return 0;
  if (host::sd.failAfter >= 0 && host::sd.appends >= host::sd.failAfter) return 0;
  uint64_t t, u;
  sdSpace(t, u);
  if (u + n > t) return 0;
  host::sd.appends++;
  auto& f = host::sd.files[sdWr];
  f.insert(f.end(), d, d + n);
  return n;
}
bool sdSync() { host::sd.syncs++; return sdMounted(); }
void sdClose() { sdWr.clear(); }
void sdList(const char* dir, void (*cb)(const char*, uint32_t, void*), void* ctx) {
  std::string pre = std::string(dir) + "/";
  for (auto& f : host::sd.files)
    if (!f.first.compare(0, pre.size(), pre)) cb(f.first.c_str() + pre.size(), (uint32_t)f.second.size(), ctx);
}
bool sdRemove(const char* path) { return host::sd.files.erase(path) > 0; }
bool sdRead(const char* path, uint32_t off, uint8_t* buf, size_t len, size_t& got, uint32_t& size) {
  auto it = host::sd.files.find(path);
  if (!sdMounted() || it == host::sd.files.end()) return false;
  sdRd = path;
  size = (uint32_t)it->second.size();
  got = off >= size ? 0 : (size - off < len ? size - off : len);
  if (got) memcpy(buf, it->second.data() + off, got);
  return true;
}
void sdReadClose() { sdRd.clear(); }

bool logOpen() { return true; }
size_t logWrite(const uint8_t* d, size_t n) { host::log.insert(host::log.end(), d, d + n); return n; }
void logFlush() {}
void logErase() { host::log.clear(); }
uint32_t logSize() { return (uint32_t)host::log.size(); }
uint32_t logCapacity() { return 2u * 1024 * 1024; }
bool logRead(uint32_t off, uint8_t* buf, size_t len) {
  if (off + len > host::log.size()) return false;
  memcpy(buf, host::log.data() + off, len);
  return true;
}
void logService(bool) {}
bool logBusy() { return false; }
void markFastLoopDone() {}
void wifiSetEnabled(bool) {}
void netReport(char* out, size_t n) { snprintf(out, n, "host: no WiFi\r\n"); }
FirmwareInfo firmwareInfo() { return FirmwareInfo{"host", false, false}; }
void firmwareConfirm() {}
void firmwareReject() {}
bool wifiStart(const char*, const char*) { return true; }
uint32_t deviceId() { return 0x1234ABCD; }
uint32_t random32() { static uint32_t s = 2463534242u; s ^= s << 13; s ^= s >> 17; s ^= s << 5; return s; }
void reboot() {}
bool bootloaderSupported() { return true; }
void rebootToBootloader() { host::bootloaderRequests++; }
bool escSerialBegin() { host::escActive = true; return true; }
void escSerialEnd() { host::escActive = false; }
void escSerialWrite(int esc, const uint8_t* d, size_t n) { if (host::escTx) host::escTx(esc, d, n); }
int escSerialRead(int esc, uint32_t) { return host::escRx ? host::escRx(esc) : -1; }
bool ledStripBegin(uint8_t pin, uint8_t count) { return pin && count; }
void ledStripShow(const uint8_t* grb, uint8_t count) { host::ledFrame.assign(grb, grb + count * 3); }

}  // namespace hal
