// AllQuad - picks the better of two IMUs while disarmed (imu_dual 1)
//
// The failover in the fast loop only reacts to a chip that stops or freezes. Before arming, both chips are
// also compared on quantities that do not depend on how each one is mounted: the length of the gyro
// vector (its fast wobble = noise) and the length of the accelerometer vector (gravity, 9.81 m/s^2 at rest).
//  - prefer(): the first chip is clearly noisier (vibration-damaged, bad solder) or reads a wrong gravity,
//    while the second one is sane -> fly on the second (no arming block: it was chosen on the ground)
//  - mismatch(): the two accelerometers disagree on gravity: one of them is wrong -> arming blocked
#pragma once
#include "../common/math3d.h"

namespace fc {

class ImuVote {
 public:
  void reset() { *this = ImuVote(); }
  // gyro deg/s, accel m/s^2, dt s between calls
  void add(const Vec3& g1, const Vec3& a1, const Vec3& g2, const Vec3& a2, float dt);
  bool prefer() const { return preferS_ > 2.0f; }
  bool mismatch() const { return mismatchS_ > 2.0f; }
  float noise(int i) const;   // gyro noise (deg/s RMS of the fast part of |gyro|)
  float gravity(int i) const { return i ? acc2_ : acc1_; }  // |accel| (m/s^2, slow)
  bool ready() const { return n_ > 200; }

 private:
  float slow_[2] = {0, 0}, var_[2] = {0, 0};
  float acc1_ = 9.81f, acc2_ = 9.81f;
  float preferS_ = 0, mismatchS_ = 0;
  int n_ = 0;
};

}  // namespace fc
