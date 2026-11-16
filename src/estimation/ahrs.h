// AllQuad - attitude estimator (Mahony complementary filter on quaternions)
#pragma once
#include "../common/cycles.h"
#include "../common/math3d.h"

namespace fc {

class Ahrs {
 public:
  void reset();
  // Initialise attitude from a still accelerometer reading (and optional heading)
  void initFromAcc(const Vec3& acc, float yaw = 0.0f);
  // gyro rad/s FRD, acc m/s^2 FRD (specific force), mag optional (calibrated, any unit)
  // accKin: optional true acceleration (NED, gravity-free) - removes centripetal / linear acceleration
  // from the gravity reference (ArduPilot DCM GPS acceleration compensation)
  void update(const Vec3& gyro, const Vec3& acc, const Vec3* mag, float declinationRad, float dt,
              const Vec3* accKin = nullptr);

  const Quat& q() const { return q_; }
  // Euler angles on demand (3 trig calls): read a few hundred times a second, not every loop
  float roll() const { return q_.roll(); }
  float pitch() const { return q_.pitch(); }
  float yaw() const { return q_.yaw(); }
  // accelerometer / compass correction every n-th update (~1 kHz is plenty for a complementary
  // filter); the gyro is integrated on every update. The correction rate is held in between.
  void setCorrectionDivider(int n) { corrDiv_ = n < 1 ? 1 : n; }
  // Earth-frame (NED) acceleration with gravity removed, m/s^2
  const Vec3& accEarth() const { return accEarth_; }
  bool accTrusted() const { return accTrusted_; }

  float kp = 0.15f;    // accel correction gain
  float ki = 0.003f;   // gyro bias learning
  float kpMag = 0.4f;

 private:
  Quat q_;
  Vec3 integral_;
  Vec3 accEarth_;
  Vec3 corr_;
  int corrDiv_ = 1, corrCount_ = 0;
  float corrDt_ = 0;
  float settleTime_ = 0;
  bool accTrusted_ = false;
};

}  // namespace fc
