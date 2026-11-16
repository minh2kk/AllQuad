// AllQuad - heading from GPS for craft without a (healthy) compass.
// Idea of ArduPilot's EKF-GSF yaw estimator / Betaflight GPS Rescue: over a 1 s window, the change of
// GPS velocity (true NED) is compared with the change of velocity integrated from the accelerometer
// in the AHRS frame (whose yaw is only gyro-propagated). The angle between both vectors is the
// AHRS yaw error. Needs some horizontal acceleration (any manoeuvre, or RTH_ALIGN flies forward).
#pragma once
#include "../common/math3d.h"

namespace fc {

class GpsHeading {
 public:
  void reset() { *this = GpsHeading(); }
  // accN/accE: AHRS-frame earth acceleration (gravity removed); every control cycle
  void predict(float accN, float accE, float dt) {
    vIns_[0] += accN * dt;
    vIns_[1] += accE * dt;
  }
  void fuseGps(float velN, float velE, uint32_t nowMs) {
    if (!t0_) { start(velN, velE, nowMs); return; }
    uint32_t span = nowMs - t0_;
    if (span < 900) return;
    if (span > 1600) { start(velN, velE, nowMs); return; }  // GPS gap
    float gN = velN - vGps0_[0], gE = velE - vGps0_[1];
    float iN = vIns_[0] - vIns0_[0], iE = vIns_[1] - vIns0_[1];
    start(velN, velE, nowMs);
    float gl = sqrtf(gN * gN + gE * gE), il = sqrtf(iN * iN + iE * iE);
    if (gl < 1.2f || il < 1.2f || il > gl * 1.7f || il < gl * 0.6f) return;  // not enough excitation
    // rotate the inertial change by the current correction, measure the residual angle
    float c = cosf(offset_), s = sinf(offset_);
    float rN = iN * c - iE * s, rE = iN * s + iE * c;
    float err = atan2f(rN * gE - rE * gN, rN * gN + rE * gE);
    float k = good_ >= 2 ? 0.35f : 1.0f;  // first fixes: jump straight to the measurement
    offset_ = wrapPi(offset_ + k * err);
    if (fabsf(err) < 20.0f * DEG2RAD) { if (good_ < 255) good_++; }
    else if (good_ >= 2) good_--;
    lastGoodMs_ = nowMs;
  }
  float offset() const { return offset_; }   // add to the AHRS yaw
  bool aligned() const { return good_ >= 2; }

 private:
  void start(float vn, float ve, uint32_t now) {
    t0_ = now ? now : 1;
    vGps0_[0] = vn; vGps0_[1] = ve;
    vIns0_[0] = vIns_[0]; vIns0_[1] = vIns_[1];
  }
  float vIns_[2] = {0, 0}, vIns0_[2] = {0, 0}, vGps0_[2] = {0, 0};
  uint32_t t0_ = 0, lastGoodMs_ = 0;
  float offset_ = 0;
  uint8_t good_ = 0;
};

// Wind observer: horizontal acceleration that the thrust tilt does not explain is drag in the moving
// air, d = k * (wind - v) with k = wind_drag (linear drag / mass). Wind = d / k + v, low-pass filtered.
// accN/accE return the wind force per mass on a craft at rest (k * wind): the position controller
// cancels it, so it does not have to wait for its integrator.
class WindEstimator {
 public:
  void reset() { w_[0] = w_[1] = 0; valid_ = false; }
  // q: attitude (AHRS frame), yawCorr: heading correction, accEarth: AHRS-frame NED accel (gravity removed),
  // vN/vE: ground velocity (NED), k: wind_drag
  void update(const Quat& q, float yawCorr, const Vec3& accEarth, float vN, float vE, float k, bool airborne,
              float dt) {
    Vec3 zb = q.rotate(Vec3(0, 0, 1));  // body z (down) in the AHRS earth frame
    if (!airborne || zb.z < 0.7f || k <= 0.0f) return;
    float tm = (GRAVITY - accEarth.z) / zb.z;  // thrust per mass from the vertical balance
    float tN = -zb.x * tm, tE = -zb.y * tm;
    float c = cosf(yawCorr), s = sinf(yawCorr);
    float aN = accEarth.x * c - accEarth.y * s, aE = accEarth.x * s + accEarth.y * c;
    float thN = tN * c - tE * s, thE = tN * s + tE * c;
    k_ = k;
    float f = dt / (1.5f + dt);
    w_[0] += ((aN - thN) / k + vN - w_[0]) * f;
    w_[1] += ((aE - thE) / k + vE - w_[1]) * f;
    valid_ = true;
  }
  float windN() const { return w_[0]; }
  float windE() const { return w_[1]; }
  float accN() const { return valid_ ? k_ * w_[0] : 0; }
  float accE() const { return valid_ ? k_ * w_[1] : 0; }
  bool valid() const { return valid_; }

 private:
  float w_[2] = {0, 0};
  float k_ = 0.45f;
  bool valid_ = false;
};

}  // namespace fc
