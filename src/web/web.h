// AllQuad - web configurator server (ESP32). Other targets: start() returns false.
#pragma once

namespace web {
bool start();  // after hal::wifiStart(); serves http://192.168.4.1
void poll();   // called by the "web" task
}  // namespace web
