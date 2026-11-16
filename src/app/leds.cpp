#include "../common/size_opt.h"  // -Os on ESP32 (not flight critical)
#include "leds.h"

namespace app {

using namespace fc;

static uint32_t modeColor(FlightMode m) {
  switch (m) {
    case FlightMode::ACRO: return 0xFFFFFF;
    case FlightMode::ANGLE: return 0x00FF00;
    case FlightMode::HORIZON: return 0x00FFFF;
    case FlightMode::ALTHOLD: return 0x0040FF;
    case FlightMode::POSHOLD: case FlightMode::CRUISE: return 0x8000FF;
    case FlightMode::RTH: case FlightMode::MISSION: case FlightMode::LAND: return 0xFFC000;
    case FlightMode::TURTLE: return 0xFF6000;
    case FlightMode::AUTOTUNE: return 0xFF00FF;
    default: return 0xFF0000;
  }
}

void ledColors(const Config& c, const VehicleStatus& st, const BatteryState& b, uint32_t now, uint8_t* grb, int n) {
  bool fast = (now / 100) % 2, slow = (now / 400) % 2;
  float breathe = 0.25f + 0.75f * (float)((now / 20) % 100 < 50 ? (now / 20) % 50 : 50 - (now / 20) % 50) / 50.0f;
  uint32_t whole = 0;  // a colour for every LED (0 = per LED)
  float level = 1.0f;
  bool override = true;
  if (st.failsafe) whole = fast ? 0xFF0000 : 0x000000;
  else if (b.critical) whole = slow ? 0xFF0000 : 0x000000;
  else override = false;
  if (!override) {
    if (c.led_mode == 2) whole = c.led_color;
    else if (c.led_mode == 1) whole = 0;  // orientation, below
    else if (st.atState == 1) whole = fast ? 0xFF00FF : 0x000000;
    else if (b.warn && (now / 250) % 8 == 0) whole = 0xFF6000;
    else if (st.armed) whole = modeColor(st.mode);
    else { whole = st.armBlockers ? 0xFFA000 : 0x00FF00; level = breathe; }
  }
  float k = level * (c.led_brightness > 100 ? 100 : c.led_brightness) / 100.0f;
  for (int i = 0; i < n; i++) {
    uint32_t col = whole;
    if (!override && c.led_mode == 1) col = i < (n + 1) / 2 ? 0xFFFFFF : 0xFF0000;  // front white, rear red
    grb[i * 3 + 0] = (uint8_t)(((col >> 8) & 0xFF) * k);   // G
    grb[i * 3 + 1] = (uint8_t)(((col >> 16) & 0xFF) * k);  // R
    grb[i * 3 + 2] = (uint8_t)((col & 0xFF) * k);           // B
  }
}

}  // namespace app
