#include "ahrs.h"

namespace fc {

void Ahrs::reset() {
  q_ = Quat();
  integral_ = Vec3();
  accEarth_ = Vec3();
  corr_ = Vec3(); corrCount_ = 0; corrDt_ = 0;
  settleTime_ = 0;
}

void Ahrs::initFromAcc(const Vec3& a, float yaw) {
  // At rest: a = -g * (down vector expressed in body)
  float roll = atan2f(-a.y, -a.z);
  float pitch = atan2f(a.x, sqrtf(a.y * a.y + a.z * a.z));
  q_ = Quat::fromEuler(roll, pitch, yaw);
  integral_ = Vec3();
  settleTime_ = 0;
}

FAST_CODE void Ahrs::update(const Vec3& gyroIn, const Vec3& accIn, const Vec3* mag, float declinationRad, float dt,
                  const Vec3* accKin) {
  const Vec3& gyro = gyroIn;
  settleTime_ += dt;
  corrDt_ += dt;
  if (++corrCount_ >= corrDiv_) {
  corrCount_ = 0;
  const float cdt = corrDt_;
  corrDt_ = 0;
  Vec3 corr;
  // gravity reference: specific force minus the known kinematic acceleration
  Vec3 acc = accKin ? accIn - q_.rotateInv(*accKin) : accIn;
  float accNorm = acc.norm();
  // Trust the accelerometer only when it measures ~1 g
  accTrusted_ = accNorm > 0.80f * GRAVITY && accNorm < 1.20f * GRAVITY;

  // Faster convergence during the first seconds after boot
  float gain = settleTime_ < 3.0f ? 2.5f : kp;

  if (accTrusted_) {
    // Weight drops quickly when |acc| deviates from 1 g (manoeuvres, sustained tilt)
    float dev = fabsf(accNorm - GRAVITY) / GRAVITY;
    float weight = constrainf(1.0f - dev / 0.06f, 0.0f, 1.0f);
    // ...and when rotating fast (centripetal errors, gyro is reliable then)
    float rate = gyro.norm();
    if (!accKin) weight *= constrainf(1.0f - rate / 3.0f, 0.0f, 1.0f);
    if (settleTime_ < 3.0f) weight = 1.0f;
    Vec3 downMeas = acc * (-1.0f / accNorm);
    Vec3 downEst = q_.rotateInv(Vec3(0, 0, 1));
    Vec3 e = downMeas.cross(downEst);
    corr += e * (gain * weight);
    if (ki > 0.0f && settleTime_ > 3.0f) integral_ += e * (ki * cdt);
  }

  if (mag) {
    Vec3 mEarth = q_.rotate(*mag);
    if (mEarth.x * mEarth.x + mEarth.y * mEarth.y > 1e-6f) {
      float headingErr = wrapPi(atan2f(mEarth.y, mEarth.x) - declinationRad);
      // rotate estimate about earth z to cancel the error
      Vec3 eEarth(0, 0, -headingErr);
      float mg = settleTime_ < 3.0f ? 2.0f : kpMag;
      corr += q_.rotateInv(eEarth) * mg;
    }
  }
  corr_ = corr;
  }  // correction

  Vec3 w = gyro + corr_ + integral_;
  Quat dq(0.0f, w.x, w.y, w.z);
  Quat qd = q_ * dq;
  q_.w += 0.5f * qd.w * dt;
  q_.x += 0.5f * qd.x * dt;
  q_.y += 0.5f * qd.y * dt;
  q_.z += 0.5f * qd.z * dt;
  q_.normalize();
  accEarth_ = q_.rotate(accIn) + Vec3(0, 0, GRAVITY);
}

}  // namespace fc
