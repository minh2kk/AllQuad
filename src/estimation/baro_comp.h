// AllQuad - barometer height corrections shared by the classic estimator and the EKF (a few flops).
#pragma once

namespace fc {

// alt: baro height (m), r: its noise (m, 1 sigma), hEst: the estimator's height, vh2: horizontal speed^2.
// dyn: baro_dyn, height error per (m/s)^2 added back (the static port sees the air flow at speed);
//      the leftover is not modelled well either, so the baro counts less at speed (x2 noise at 10 m/s).
// ground: armed near the ground (ground effect / prop wash raises the pressure under the craft: the baro
//      reads low). A dip of up to 0.5 m below the estimate is ignored, a larger one counts 0.5 m less, and
//      the baro counts half: the accelerometer carries take-off and touch-down.
inline void baroCorrect(float& alt, float& r, float hEst, float vh2, float dyn, bool ground) {
  alt += dyn * vh2;
  r *= 1.0f + vh2 * 0.01f;
  if (ground) {
    float y = alt - hEst;
    if (y < 0) alt = hEst + (y < -0.5f ? y + 0.5f : 0.0f);
    r *= 1.4f;
  }
}

}  // namespace fc
