// AllQuad - safety building blocks: battery monitor, landing detector, geofence
#pragma once
#include "../bus/topics.h"
#include "../common/filters.h"
#include "../config/config.h"
#include "../navigation/mission.h"

namespace fc {

// Battery: voltage/current filtering, cell auto-detect, remaining %, warning levels
class BatteryMonitor {
 public:
  void configure(const Config& c, float rateHz);
  // load: mean motor output^1.5 (0..1), the load proxy without a current sensor
  void update(float volts, float amps, bool armed, uint32_t nowMs, float dt, float load = 0);
  const BatteryState& state() const { return st_; }
  static uint8_t lipoPercent(float cellV);
  static float lipoPercentF(float cellV);

 private:
  const Config* cfg_ = nullptr;
  BatteryState st_{};
  static void lowFor(bool low, uint32_t& since, uint32_t holdMs, uint32_t nowMs, bool& flag);
  Pt1 vLpf_, aLpf_;
  bool init_ = false;
  uint32_t warnMs_ = 0, critMs_ = 0;
  void learnResistance(float volts, float x, float dt);
  float vSlow_ = 0, xSlow_ = 0, sxv_ = 0, sxx_ = 0, xLpf_ = 0, pctLpf_ = 0;
  bool rInit_ = false, pctInit_ = false, xInit_ = false;
};

// ArduPilot-style landing detector: low throttle demand + no vertical motion for 1 s
class LandDetector {
 public:
  bool update(bool armed, float throttleOut, float hoverThrottle, float vz, const NavEstimate& est,
              bool descending, uint32_t nowMs) {
    if (!armed) { since_ = 0; landed_ = true; return landed_; }
    bool lowThrottle = throttleOut < hoverThrottle * (descending ? 0.95f : 0.8f);
    bool still = fabsf(vz) < 0.3f;
    bool nearGround = !est.aglValid || est.agl < 0.5f;
    if (lowThrottle && still && nearGround) {
      if (!since_) since_ = nowMs ? nowMs : 1;
      landed_ = (nowMs - since_) > 1000;
    } else {
      since_ = 0;
      landed_ = false;
    }
    return landed_;
  }
  bool landed() const { return landed_; }
  void reset() { since_ = 0; landed_ = false; }

 private:
  uint32_t since_ = 0;
  bool landed_ = true;
};

// Geofence: cylinder around home, plus the polygon uploaded from the GCS (fence_poly)
inline bool fenceBreached(const Config& c, const NavEstimate& est) {
  if (!c.fence_enable || !est.homeSet) return false;
  float d = sqrtf(est.n * est.n + est.e * est.e);
  if ((est.posValid && d > c.fence_radius) || (est.altValid && est.alt > c.fence_alt_max)) return true;
  return c.fence_poly && est.posValid && nav::fenceOutside(est.homeLat, est.homeLon, est.n, est.e);
}

}  // namespace fc
