#include "safety.h"

namespace fc {

void BatteryMonitor::configure(const Config& c, float rateHz) {
  cfg_ = &c;
  vLpf_.setCutoff(0.5f, 1.0f / rateHz);
  aLpf_.setCutoff(2.0f, 1.0f / rateHz);
}

float BatteryMonitor::lipoPercentF(float v) {
  // resting LiPo discharge curve (per cell)
  static const float V[] = {3.30f, 3.50f, 3.60f, 3.70f, 3.75f, 3.80f, 3.85f, 3.95f, 4.05f, 4.20f};
  static const float P[] = {0, 5, 10, 20, 35, 50, 60, 75, 88, 100};
  if (v <= V[0]) return 0;
  for (int i = 1; i < 10; i++)
    if (v < V[i]) return P[i - 1] + (P[i] - P[i - 1]) * (v - V[i - 1]) / (V[i] - V[i - 1]);
  return 100;
}
uint8_t BatteryMonitor::lipoPercent(float v) { return (uint8_t)lipoPercentF(v); }

// latched flag once `low` held for holdMs in a row
void BatteryMonitor::lowFor(bool low, uint32_t& since, uint32_t holdMs, uint32_t nowMs, bool& flag) {
  if (!low) { since = 0; return; }
  if (!since) since = nowMs ? nowMs : 1;
  if (nowMs - since >= holdMs) flag = true;
}

// Pack resistance (ArduPilot BATT_ resistance estimate): over a flight the voltage follows V = V0 - R x,
// x the load (current, or the motor outputs^1.5 without a current sensor). Both are high-passed (10 s)
// and R is the regression of the voltage changes on the load changes (60 s memory): throttle steps
// are what teaches it, the slow discharge cancels out. Counted once the load has moved enough.
void BatteryMonitor::learnResistance(float volts, float x, float dt) {
  float a = dt / (10.0f + dt);
  if (!rInit_) { vSlow_ = volts; xSlow_ = x; rInit_ = true; return; }
  vSlow_ += (volts - vSlow_) * a;
  xSlow_ += (x - xSlow_) * a;
  float dv = volts - vSlow_, dx = x - xSlow_;
  float m = dt / (60.0f + dt);
  sxv_ += (dx * dv - sxv_) * m;
  sxx_ += (dx * dx - sxx_) * m;
  float span = st_.loadIsCurrent ? 4.0f : 0.004f;  // load variance it needs (A^2 / output^2)
  if (sxx_ > span) {
    float r = -sxv_ / sxx_;
    float rMax = st_.loadIsCurrent ? 0.3f : 0.5f * volts;
    if (r > 0 && r < rMax) { st_.resistance = r; st_.restLearned = true; }
  }
}

void BatteryMonitor::update(float volts, float amps, bool armed, uint32_t nowMs, float dt, float load) {
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
  // load: the current when there is a sensor, else the motor outputs (thrust ~ power)
  st_.loadIsCurrent = amps >= 0;
  float x = st_.loadIsCurrent ? st_.current : load;
  if (!xInit_) { xLpf_ = x; xInit_ = true; }
  xLpf_ += (x - xLpf_) * (dt / (0.32f + dt));  // the same lag as the voltage filter (0.5 Hz)
  if (armed) learnResistance(st_.voltage, xLpf_, dt);
  float rest = st_.voltage + (st_.restLearned ? st_.resistance * xLpf_ : 0.0f);
  st_.restCell = st_.restLearned ? rest / st_.cells : st_.cellVoltage;
  if (c.bat_capacity > 0 && st_.loadIsCurrent) {
    float pct = 100.0f * (1.0f - st_.mahDrawn / c.bat_capacity);
    st_.percentF = constrainf(pct, 0, 100);
    st_.capacityLow = pct < c.bat_reserve_pct;
  } else {
    // from the resting voltage, smoothed (20 s): a punch no longer reads as an empty pack
    float p = lipoPercentF(st_.restCell);
    pctLpf_ = pctInit_ ? pctLpf_ + (p - pctLpf_) * (dt / (20.0f + dt)) : p;
    pctInit_ = true;
    st_.percentF = pctLpf_;
    st_.capacityLow = false;
  }
  st_.percent = (uint8_t)(st_.percentF + 0.5f);
  // latch while flying; clear only when disarmed and recovered
  // a punch sags the pack for a moment: below the limit for 3 s in a row (warning 2 s) counts, a short dip
  // does not trigger the battery failsafe (ArduPilot BATT_LOW_TIMER)
  lowFor(st_.cellVoltage < c.vbat_warn_cell, warnMs_, 2000, nowMs, st_.warn);
  if (!armed && st_.cellVoltage > c.vbat_warn_cell + 0.1f) st_.warn = false;
  lowFor(st_.cellVoltage < c.vbat_crit_cell, critMs_, 3000, nowMs, st_.critical);
  if (!armed && st_.cellVoltage > c.vbat_crit_cell + 0.1f) st_.critical = false;
}

}  // namespace fc
