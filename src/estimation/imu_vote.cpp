#include "imu_vote.h"

#include <math.h>

namespace fc {

float ImuVote::noise(int i) const { return sqrtf(var_[i & 1]); }

void ImuVote::add(const Vec3& g1, const Vec3& a1, const Vec3& g2, const Vec3& a2, float dt) {
  const float gn[2] = {g1.norm(), g2.norm()};
  const float an1 = a1.norm(), an2 = a2.norm();
  const float kSlow = dt / (0.05f + dt), kVar = dt / (1.0f + dt), kAcc = dt / (0.5f + dt);
  for (int i = 0; i < 2; i++) {
    if (!n_) slow_[i] = gn[i];
    slow_[i] += kSlow * (gn[i] - slow_[i]);  // ~3 Hz: motion both chips see
    float d = gn[i] - slow_[i];
    var_[i] += kVar * (d * d - var_[i]);
  }
  if (!n_) { acc1_ = an1; acc2_ = an2; }
  acc1_ += kAcc * (an1 - acc1_);
  acc2_ += kAcc * (an2 - acc2_);
  n_++;
  if (n_ < 50) return;
  const float G = 9.81f;
  const bool sane2 = fabsf(acc2_ - G) < 1.5f && noise(1) < 5.0f;
  const bool bad1 = noise(0) > 3.0f * noise(1) + 0.5f || fabsf(acc1_ - G) > 2.5f;
  preferS_ = sane2 && bad1 ? preferS_ + dt : 0;
  mismatchS_ = fabsf(acc1_ - acc2_) > 1.5f ? mismatchS_ + dt : 0;
}

}  // namespace fc
