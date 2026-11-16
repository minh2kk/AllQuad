// AllQuad - outer loops: self-level (angle/horizon), altitude and horizontal position
#pragma once
#include "../bus/topics.h"
#include "../config/config.h"
#include "pid.h"

namespace fc {

// ArduPilot-style square-root controller: linear near the target, constant
// deceleration further away (never overshoots the acceleration limit).
inline float sqrtController(float err, float p, float accMax) {
  if (p <= 0.0f) return 0.0f;
  if (accMax <= 0.0f) return err * p;
  float linDist = accMax / (p * p);
  if (err > linDist) return sqrtf(2.0f * accMax * (err - linDist * 0.5f));
  if (err < -linDist) return -sqrtf(2.0f * accMax * (-err - linDist * 0.5f));
  return err * p;
}

class AttitudeControl {
 public:
  // target angles (deg) -> body rate setpoints (deg/s)
  static void level(const Config& c, float tgtRollDeg, float tgtPitchDeg, float rollDeg, float pitchDeg,
                    float out[2]);
  // Horizon: self-level near center stick, rate (acro) control at full deflection
  static void horizon(const Config& c, float rollStick, float pitchStick, float rollDeg, float pitchDeg,
                      float out[2], int profile = 0);
};

class AltitudeControl {
 public:
  void configure(const Config& c);
  // start from the current throttle so the transition is bump-free
  void reset(float currentThrottle, float cosTilt);
  // returns collective throttle 0..1
  float update(float targetAlt, float climbFF, float maxUp, float maxDown, const NavEstimate& est, float accUp,
               float cosTilt, float dt);
  float hoverEstimate() const;
  // learned hover throttle (feed-forward); moves the integrator so the output does not jump
  void setHover(float h);
  float hover() const { return hover_; }

 private:
  void limits();
  const Config* cfg_ = nullptr;
  Pid velPid_;
  float hover_ = 0.4f;
};

class PositionControl {
 public:
  void configure(const Config& c);
  void reset();
  // target position + velocity feed-forward (NED) -> desired roll/pitch angles (deg)
  // windAccN/E: disturbance acceleration (wind, drag) estimated by WindEstimator, compensated when wind_comp
  void update(float tgtN, float tgtE, float ffN, float ffE, float maxSpeed, const NavEstimate& est, float yaw,
              float windAccN, float windAccE, float dt, float& rollDeg, float& pitchDeg, float accFfN = 0,
              float accFfE = 0);
  float velSpN() const { return vsp_[0]; }
  float velSpE() const { return vsp_[1]; }

 private:
  const Config* cfg_ = nullptr;
  Pid velPid_[2];
  float vsp_[2] = {0, 0};
  float corr_[2] = {0, 0};  // rate-limited correction on top of the feed-forward
};

}  // namespace fc
