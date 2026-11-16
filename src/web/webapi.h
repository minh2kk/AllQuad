// AllQuad - web configurator API (pure logic, transport independent).
// The ESP32 web server (web_esp32.cpp) maps HTTP routes onto these functions, the USB configurator
// link (comms/aqlink.cpp) maps its frames onto the same ones; the host tests call them directly.
//
//   GET  /                 configurator page (gzip, embedded)
//   GET  /api/params       all parameters with ranges
//   GET  /api/status       live state (attitude, rates, motors, battery, RC, GPS, tasks...)
//   POST /api/set?a=1&b=2  change parameters (disarmed only, not saved)
//   POST /api/save         store parameters in flash
//   POST /api/cmd?c=...    calib_acc | calib_mag | calib_gyro | defaults | reboot | bb_erase
//   GET  /api/mission      mission, rally points, fence for the map
//   GET  /api/blackbox     download the onboard blackbox log
//   GET  /api/backup       configuration backup (CLI text: set / mission add / rally add / save)
//   POST /api/restore      run a backup file (disarmed)
//   POST /api/ota          firmware image (ESP32, disarmed): written to the other slot, rollback on failure
//   POST /api/login /api/logout /api/wifi   access control (see web_esp32.cpp)
//   GET  /api/sensors      raw sensor values (Sensors tab)
//   POST /api/cli          one CLI line in the body, its output back as text
//   POST /api/motor?m1=..  motor test (disarmed)
//   WS   /ws               live status pushed 10 Hz (2 Hz while armed); polling /api/status is the fallback
#pragma once
#include <stddef.h>
#include <stdint.h>

#include "../comms/cli.h"

namespace web {

using Sink = void (*)(const char* data, size_t len, void* ctx);

// Buffered JSON text writer that flushes through a sink (keeps RAM use small).
class JsonOut {
 public:
  JsonOut(Sink s, void* ctx) : sink_(s), ctx_(ctx) {}
  ~JsonOut() { flush(); }
  void raw(const char* s);
  void str(const char* s);  // quoted + escaped
  void num(double v, int decimals = 3);
  // key then exactly one value (scalar via raw/str/num, or begin()...end())
  void key(const char* k) { comma(); str(k); put(':'); first_ = false; inKey_ = true; }
  void begin(char c) { if (inKey_) inKey_ = false; else comma(); put(c); first_ = true; }
  void end(char c) { put(c); first_ = false; }
  void comma() { if (!first_) put(','); first_ = false; }
  void flush();

 private:
  void put(char c) { if (n_ >= sizeof(buf_)) flush(); buf_[n_++] = c; }
  Sink sink_;
  void* ctx_;
  char buf_[1400];  // ~ one TCP segment per flush
  size_t n_ = 0;
  bool first_ = true;
  bool inKey_ = false;
};

void begin(const comms::SystemInfo* info);
void paramsJson(Sink s, void* ctx);
// full = false leaves out what changes slowly (task table, loop profile, sensors, log, firmware,
// WiFi): the WebSocket sends that only once per second, the page keeps the last values
void statusJson(Sink s, void* ctx, bool full = true);
void missionJson(Sink s, void* ctx);
// Sets one parameter from text. Returns an HTTP status (200 ok, 400 bad, 404 unknown, 423 armed).
int setParam(const char* name, const char* value);
bool save();
// Runs a command, returns HTTP status; msg receives a short human readable result.
int command(const char* name, char* msg, size_t msgLen);
bool armed();
// live raw sensors for the Sensors tab (gyro deg/s, acc m/s^2, mag, baro, rangefinder, flow, ESC)
void sensorsJson(Sink s, void* ctx);
void sdJson(Sink s, void* ctx);
// runs one CLI line, its output goes to the sink. trusted = USB (the "wifi" command is USB only)
void cliExec(const char* line, Sink s, void* ctx, bool trusted);
// motor test: "m1=0.1&m2=0&m3=0&m4=0" (0..1), stops by itself after 1 s without a new request
int motorTest(const float v[4], char* msg, size_t n);

// ---- access control: password login -> session token (sent back as the "aq" cookie) ----
// returns 200 (token filled, 32 hex chars), 401 wrong password, 429 too many attempts (lockout)
int login(const char* password, uint32_t nowMs, char token[33]);
// cookie header ("a=1; aq=<token>") -> valid session (refreshes its idle timer)
bool authorized(const char* cookieHeader, uint32_t nowMs);
void logout(const char* cookieHeader);
// change network name / WiFi password / web password; current = web password for confirmation.
// 200 saved (access point restarts), 400 invalid (msg says why), 401 wrong password, 423 armed
// ---- configuration backup / restore (CLI command text, also usable over USB) ----
void backupText(Sink s, void* ctx, bool all);
// runs the lines (disarmed only); msg = "N commands, M errors[: first error]"; 200 ok, 400 errors, 423 armed
int restoreText(const char* text, size_t len, char* msg, size_t msgLen);

// trusted = over USB (comms/aqlink): the current password is not asked (resetting a forgotten one)
int changeWifi(const char* ssid, const char* wifiPass, const char* webPass, const char* current, char* msg,
               size_t msgLen, bool trusted = false);

}  // namespace web
