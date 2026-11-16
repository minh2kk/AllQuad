// AllQuad - WiFi access point name/password and web configurator password.
// Stored in their own flash slot (not in the parameter table: strings, never sent over MAVLink).
// Recovery when forgotten: USB CLI "wifi" commands (see cli.cpp).
#pragma once
#include <stddef.h>
#include <stdint.h>

namespace creds {

constexpr const char* DEFAULT_WIFI_PASS = "allquad123";

struct Credentials {
  uint32_t magic;
  char ssid[33];       // empty = "AllQuad-XXXX" from the chip id
  char wifiPass[65];   // WPA2: 8..63 characters
  char webPass[65];    // empty = same as the WiFi password
  uint32_t crc;
};

void load();                     // from flash (defaults if missing)
bool save();                     // disarmed only (flash write)
void resetDefaults();
const char* ssid();              // effective name
const char* wifiPass();
const char* webPass();           // effective web password
bool usingDefaultPassword();     // WiFi or web password is still the factory default
// validation: 0 ok, else a short reason
const char* setSsid(const char* s);
const char* setWifiPass(const char* s);
const char* setWebPass(const char* s);  // "" = same as WiFi
bool equalsConstTime(const char* a, const char* b);

}  // namespace creds
