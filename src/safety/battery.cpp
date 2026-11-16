#include "safety.h"

namespace fc {

void BatteryMonitor::configure(const Config& c, float rateHz) {
  cfg_ = &c;
  vLpf_.setCutoff(0.5f, 1.0f / rateHz);
  aLpf_.setCutoff(2.0f, 1.0f / rateHz);
}

uint8_t BatteryMonitor::lipoPercent(float v) {
  // resting LiPo discharge curve (per cell)
  static const float V[] = {3.30f, 3.50f, 3.60f, 3.70f, 3.75f, 3.80f, 3.85f, 3.95f, 4.05f, 4.20f};
  static const float P[] = {0, 5, 10, 20, 35, 50, 60, 75, 88, 100};
  if (v <= V[0]) return 0;
  for (int i = 1; i < 10; i++)
    if (v < V[i]) return (uint8_t)(P[i - 1] + (P[i] - P[i - 1]) * (v - V[i - 1]) / (V[i] - V[i - 1]));
  return 100;
}

void BatteryMonitor::update(float volts, float amps, bool armed, uint32_t nowMs, float dt) {
  const Config& c = *cfg_;
  if (volts < 1.0f) {  // no battery sensing
    st_ = BatteryState{};
    return;
  }
  if (!init_) { vLpf_.reset(volts); aLpf_.reset(amps > 0 ? amps : 0); init_ = true; }
  st_.voltage = vLpf_.apply(volts);
  st_.current = amps >= 0 ? aLpf_.apply(amps) : 0;
  st_.mahDrawn += st_.current * dt * (1000.0f / 3600.0f);
  if (st_.cells == 0 && nowMs > 1500) st_.cells = c.cell_count ? c.cell_count : (uint8_t)ceilf(st_.voltage / 4.35f);
  if (!st_.cells) return;
  st_.cellVoltage = st_.voltage / st_.cells;
  if (c.bat_capacity > 0) {
    float pct = 100.0f * (1.0f - st_.mahDrawn / c.bat_capacity);
    st_.percent = (uint8_t)constrainf(pct, 0, 100);
    st_.capacityLow = pct < c.bat_reserve_pct;
  } else {
    st_.percent = lipoPercent(st_.cellVoltage);
    st_.capacityLow = false;
  }
  // latch while flying; clear only when disarmed and recovered
  if (st_.cellVoltage < c.vbat_warn_cell) st_.warn = true;
  else if (!armed && st_.cellVoltage > c.vbat_warn_cell + 0.1f) st_.warn = false;
  if (st_.cellVoltage < c.vbat_crit_cell) st_.critical = true;
  else if (!armed && st_.cellVoltage > c.vbat_crit_cell + 0.1f) st_.critical = false;
}

}  // namespace fc
