// AllQuad - position / altitude estimator (Kalman filters), runs in the control task.
//   vertical  : accelerometer + barometer (+ rangefinder for height above ground)
//   horizontal: accelerometer + GPS position & velocity + optical-flow velocity
#pragma once
#include "../bus/topics.h"
#include "../config/config.h"
#include "kalman.h"

namespace fc {

class NavEstimator {
 public:
  void configure(const Config& c) { cfg_ = &c; }
  // horizTrusted = false: heading unknown (no compass yet), horizontal acceleration is not usable
  void predict(const Vec3& accEarth, float dt, bool horizTrusted = true);
  // drop the horizontal state (e.g. after a large heading correction); GPS/flow re-initialise it
  void resetHorizontal() { if (gpsUsable_ || flowUsable_) hReset_ = true; }
  void fuseBaro(const BaroSample& b);
  void fuseRange(const RangeSample& r, float cosTilt, uint32_t nowMs);
  void fuseGps(const GpsData& g, uint32_t nowMs);
  void fuseFlow(const FlowSample& f, const Vec3& rateDps, float yaw, float cosTilt, uint32_t nowMs);
  void update(uint32_t nowMs);  // validity bookkeeping, call once per cycle
  // Re-zero: altitude to 0 at the current height and (if possible) home to the current GPS position.
  void setHomeHere();

  const NavEstimate& state() const { return st_; }
  float baroOffset() const { return baroOffset_; }
  // last optical-flow velocity (NED) for the EKF: true once per new sample
  bool takeFlowVel(float& vn, float& ve, float& sigma) {
    if (!flowNew_) return false;
    flowNew_ = false; vn = flowVn_; ve = flowVe_; sigma = flowSigma_;
    return true;
  }
  static void llaToNed(double lat, double lon, double refLat, double refLon, float& n, float& e);
  static void nedToLla(float n, float e, double refLat, double refLon, double& lat, double& lon);

 private:
  const Config* cfg_ = nullptr;
  Kalman3 kz_, kn_, ke_;
  NavEstimate st_{};
  bool zInit_ = false, hInit_ = false;
 public:
  uint16_t baroResets() const { return baroResets_; }
 private:
  float baroOffset_ = 0;
  uint32_t lastGpsMs_ = 0, lastFlowMs_ = 0, lastRangeMs_ = 0, lastBaroMs_ = 0;
  uint32_t baroRejSince_ = 0;   // baro rejected by the gate since (0 = accepted)
  uint16_t baroResets_ = 0;
  bool gpsUsable_ = false, flowUsable_ = false;
  bool rehomePending_ = false;
  bool glitch_ = false, hReset_ = false;
  uint32_t glitchStartMs_ = 0;
  GpsData lastGps_{};
  uint32_t nowMs_ = 0;
  float flowVn_ = 0, flowVe_ = 0, flowSigma_ = 1;
  bool flowNew_ = false;
};

}  // namespace fc
