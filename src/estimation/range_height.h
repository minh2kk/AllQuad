// AllQuad - the rangefinder as a height source for the estimators (classic Kalman and EKF).
//
// height = range (tilt corrected) + T, T = where the ground lies in the height frame (baro / home).
//  - short term the range wins (cm instead of the baro's tens of cm of prop wash and ground effect),
//  - T follows the estimate slowly (10 s) so the long-term height stays on the baro / home frame,
//  - a sudden range jump (a table, a step, a wall edge) moves T, not the height: no bump in ALTHOLD.
// A few multiplications per sample (25-50 Hz): nothing for any core.
#pragma once
#include <math.h>
#include <stdint.h>

namespace fc {

struct RangeHeight {
  float T = 0;
  bool valid = false;
  uint32_t lastMs = 0;
  uint16_t steps = 0;    // ground steps absorbed (diagnostics)

  // agl: tilt-corrected range (m), hEst / sigmaEst: the estimator's height and its 1-sigma.
  // true = z is a height measurement to fuse (noise rangeSigma(agl))
  bool measure(float agl, float hEst, float sigmaEst, uint32_t nowMs, float& z) {
    uint32_t dtMs = nowMs - lastMs;
    if (!valid || dtMs > 500) {  // first sample, or back after a gap: line up, no correction yet
      T = hEst - agl;
      valid = true;
      lastMs = nowMs;
      return false;
    }
    lastMs = nowMs;
    float innov = agl + T - hEst;
    if (fabsf(innov) > fmaxf(0.4f, 4.0f * sigmaEst)) {  // the ground moved under the craft
      T = hEst - agl;
      steps++;
      return false;
    }
    float k = dtMs * 1e-3f / 10.0f;
    T += (k < 0.1f ? k : 0.1f) * (hEst - agl - T);
    z = agl + T;
    return true;
  }
  static float sigma(float agl) { return 0.05f + 0.02f * agl; }
  void drop() { valid = false; }
};

}  // namespace fc
