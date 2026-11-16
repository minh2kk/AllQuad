#include "../common/size_opt.h"  // -Os on ESP32 (not flight critical)
#include "credentials.h"

#include <stdio.h>
#include <string.h>

#include "../config/config.h"
#include "../hal/hal.h"

namespace creds {

static constexpr uint32_t MAGIC = 0x41514352;  // "AQCR"
static Credentials c;
static char defSsid[33];

static uint32_t crcOf(const Credentials& x) { return fc::crc32(&x, offsetof(Credentials, crc)); }

void resetDefaults() {
  memset(&c, 0, sizeof(c));
  c.magic = MAGIC;
  strncpy(c.wifiPass, DEFAULT_WIFI_PASS, sizeof(c.wifiPass) - 1);
}

void load() {
  snprintf(defSsid, sizeof(defSsid), "AllQuad-%04X", (unsigned)(hal::deviceId() & 0xFFFF));
  Credentials t;
  if (hal::storageLoad(hal::Slot::WIFI, &t, sizeof(t)) && t.magic == MAGIC && t.crc == crcOf(t)) {
    t.ssid[sizeof(t.ssid) - 1] = t.wifiPass[sizeof(t.wifiPass) - 1] = t.webPass[sizeof(t.webPass) - 1] = 0;
    c = t;
  } else {
    resetDefaults();
  }
}

bool save() {
  c.magic = MAGIC;
  c.crc = crcOf(c);
  return hal::storageSave(hal::Slot::WIFI, &c, sizeof(c));
}

const char* ssid() {
  if (!defSsid[0]) snprintf(defSsid, sizeof(defSsid), "AllQuad-%04X", (unsigned)(hal::deviceId() & 0xFFFF));
  return c.ssid[0] ? c.ssid : defSsid;
}
const char* wifiPass() { return c.wifiPass[0] ? c.wifiPass : DEFAULT_WIFI_PASS; }
const char* webPass() { return c.webPass[0] ? c.webPass : wifiPass(); }
bool usingDefaultPassword() {
  return !strcmp(wifiPass(), DEFAULT_WIFI_PASS) || !strcmp(webPass(), DEFAULT_WIFI_PASS);
}

static bool printable(const char* s) {
  for (; *s; s++)
    if ((unsigned char)*s < 0x20 || (unsigned char)*s > 0x7E) return false;
  return true;
}

const char* setSsid(const char* s) {
  size_t n = strlen(s);
  if (n > 32) return "name longer than 32";
  if (!printable(s)) return "name: ASCII only";
  strcpy(c.ssid, s);
  return nullptr;
}
const char* setWifiPass(const char* s) {
  size_t n = strlen(s);
  if (n < 8 || n > 63) return "WiFi password needs 8-63 characters";
  if (!printable(s)) return "password: ASCII only";
  strcpy(c.wifiPass, s);
  return nullptr;
}
const char* setWebPass(const char* s) {
  size_t n = strlen(s);
  if (n && (n < 6 || n > 64)) return "web password needs 6-64 characters";
  if (!printable(s)) return "password: ASCII only";
  strcpy(c.webPass, s);
  return nullptr;
}

bool equalsConstTime(const char* a, const char* b) {
  size_t la = strlen(a), lb = strlen(b);
  unsigned diff = (unsigned)(la ^ lb);
  for (size_t i = 0; i < la; i++) diff |= (unsigned char)a[i] ^ (unsigned char)(i < lb ? b[i] : 0);
  return diff == 0;
}

}  // namespace creds
