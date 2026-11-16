// AllQuad - Quad X mixer (Betaflight motor numbering, props-in by default)
//
//      M4 (FL, CW)    M2 (FR, CCW)      ^ forward
//               [ FC ]
//      M3 (RL, CCW)   M1 (RR, CW)
#pragma once
#include "../common/math3d.h"

namespace fc {

class Mixer {
 public:
  static constexpr float R[4] = {-1, -1, +1, +1};  // roll  (+ = right wing down)
  static constexpr float P[4] = {-1, +1, -1, +1};  // pitch (+ = nose up)
  static constexpr float Y[4] = {-1, +1, +1, -1};  // yaw   (+ = clockwise from above)

  void configure(bool yawReversed, bool airmode) { yawSign_ = yawReversed ? -1.0f : 1.0f; airmode_ = airmode; }
  void setAirmode(bool on) { airmode_ = on; }
  bool saturated() const { return saturated_; }  // last mix() had to scale the attitude correction

  // ArduPilot MOT_THST_EXPO: thrust ~ (1-e)*t + e*t^2. Returns the output t that gives thrust T.
  static float linearize(float T, float e) {
    if (e <= 0.0f || T <= 0.0f) return T;
    float a = 1.0f - e;
    return (-a + sqrtf(a * a + 4.0f * e * T)) / (2.0f * e);
  }

  // throttle 0..1, roll/pitch/yaw normalised (+-1 = full motor range) -> out[] 0..1
  AQ_HOT void mix(float throttle, float roll, float pitch, float yaw, float out[4]) const {
    float m[4], mn = 1e9f, mx = -1e9f;
    for (int i = 0; i < 4; i++) {
      m[i] = R[i] * roll + P[i] * pitch + Y[i] * yaw * yawSign_;
      if (m[i] < mn) mn = m[i];
      if (m[i] > mx) mx = m[i];
    }
    float range = mx - mn;
    saturated_ = range > 1.0f;
    if (range > 1.0f) {  // not enough authority: scale the attitude correction
      for (int i = 0; i < 4; i++) m[i] /= range;
      mn /= range; mx /= range;
    }
    if (!airmode_ && throttle < 0.2f) {  // without airmode, fade authority near zero throttle
      float k = throttle / 0.2f;
      for (int i = 0; i < 4; i++) m[i] *= k;
      mn *= k; mx *= k;
    }
    throttle = constrainf(throttle, -mn, 1.0f - mx);
    for (int i = 0; i < 4; i++) out[i] = constrainf(throttle + m[i], 0.0f, 1.0f);
  }

  // Betaflight "crash flip" / turtle mode: motors run reversed; the stick direction
  // selects which side spins up to flip the craft back over. out < 0 = motor stopped.
  static void turtle(float rollStick, float pitchStick, float out[4]) {
    float mag = fmaxf(fabsf(rollStick), fabsf(pitchStick));
    for (int i = 0; i < 4; i++) {
      // roll right -> left motors (R = +1) spin; pitch forward -> front motors spin
      float v = rollStick * R[i] + pitchStick * P[i];
      out[i] = (mag > 0.15f && v > 0.1f) ? constrainf(v, 0.0f, 1.0f) * mag : -1.0f;
    }
  }

 private:
  float yawSign_ = 1.0f;
  bool airmode_ = true;
  mutable bool saturated_ = false;
};

}  // namespace fc
