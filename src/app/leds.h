// AllQuad - LED strip colours (WS2812 / SK6812 on led_pin, ESP32).
//   led_mode 0 "status": the whole strip tells the state
//       failsafe ............ red, fast blink        battery critical .. red, slow blink
//       autotune ............ magenta blink          battery low ....... orange blink every 2 s
//       armed ............... flight-mode colour: ACRO white, ANGLE green, HORIZON cyan, ALTHOLD blue,
//                             POSHOLD / CRUISE purple, RTH / MISSION / LAND yellow, TURTLE orange
//       disarmed, ready ..... green breathing        disarmed, blocked . amber breathing
//   led_mode 1 "orientation": first half white (front), second half red (rear); failsafe / battery
//       critical still take over the whole strip
//   led_mode 2 fixed colour (led_color), failsafe still blinks red
#pragma once
#include <stdint.h>

#include "../bus/topics.h"
#include "../config/config.h"

namespace app {

// fills grb[count * 3] (brightness applied)
void ledColors(const fc::Config& c, const fc::VehicleStatus& st, const fc::BatteryState& b, uint32_t nowMs,
               uint8_t* grb, int count);

}  // namespace app
