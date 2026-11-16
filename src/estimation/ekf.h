// AllQuad - navigation EKF (error-state Kalman filter, 15 states), runs in the control task.
//
//   nominal state : position NED, velocity NED, attitude quaternion, gyro bias, accelerometer bias
//   error state   : dp(3) dv(3) dtheta(3, earth frame) dbg(3) dba(3)
//   prediction    : delta angles / delta velocities integrated by the fast loop (body frame)
//   measurements  : GPS position, GPS velocity (3D), barometric height, GPS height (no baro / baro
//                   fault), heading (compass or GPS-learned), optical-flow velocity, zero velocity
//                   while landed / without any horizontal aiding (keeps the covariance bounded)
//
// Fault handling (ArduPilot EKF3 style): every measurement is checked against the prediction with an
// innovation consistency test (test ratio = innovation^2 / (gate^2 * innovation variance)); a test
// ratio > 1 rejects the sample. A rejection that persists is resolved per sensor:
//   GPS position  > gps_glitch_s : the position is reset to GPS (a real jump, not a glitch)
//   GPS velocity  > 2 s          : velocity (and position) reset to GPS - the filter had diverged
//   barometer     > 5 s          : baro fault, height from GPS until the baro agrees again for 10 s
//   compass       > 5 s          : compass fault while GPS shows the heading is wrong (moving):
//                                  the commander switches the heading source to GPS
// A NaN or a negative variance resets the whole filter. The classic per-axis Kalman filters keep
// running next to it (est_type 0 selects them for control, the EKF is then a monitor only).
#pragma once
#include <stdint.h>

#include "../common/math3d.h"
#include "../config/config.h"

namespace fc {

enum EkfFault : uint16_t {
  EKF_GPS_GLITCH = 1 << 0,   // GPS position rejected right now
  EKF_GPS_VEL = 1 << 1,      // GPS velocity rejected right now
  EKF_BARO_FAULT = 1 << 2,   // baro rejected persistently: height from GPS
  EKF_MAG_FAULT = 1 << 3,    // compass disagrees with the GPS-observed heading
  EKF_NO_AIDING = 1 << 4,    // no position / velocity measurements: dead reckoning
  EKF_RESET = 1 << 5,        // whole filter reset (numerical) since boot
  EKF_FLOW_REJECT = 1 << 6,
  EKF_HEAD_REJECT = 1 << 7,
};

struct EkfStatus {
  float trGpsPos, trGpsVel, trHeight, trYaw, trMag, trFlow;  // last test ratios (> 1 = rejected)
  uint16_t faults;
  uint16_t posResets, velResets, filterResets;
  float posStd, velStd, hgtStd, yawStdDeg;                   // 1-sigma from the covariance
  uint8_t heightSrc;                                          // 0 baro, 1 GPS, 2 none
};

class Ekf {
 public:
  static constexpr int N = 15;
  void configure(const Config& c) { cfg_ = &c; }
  void reset() { init_ = false; }
  bool initialized() const { return init_; }
  void init(const Quat& q, float altUp);
  // body-frame delta angle (rad) and delta velocity (m/s, specific force * dt) over dt
  void predict(const Vec3& dAng, const Vec3& dVel, float dt, uint32_t nowMs);

  void fuseGpsPos(float n, float e, float sigma, uint32_t nowMs);
  void fuseGpsVel(float vn, float ve, float vd, float sigma, uint32_t nowMs);
  // height above the origin (up positive); src 0 = baro, 1 = GPS
  void fuseHeight(float altUp, float sigma, int src, uint32_t nowMs);
  void fuseYaw(float yaw, float sigma);
  // compass consistency only (no fusion: the heading enters through fuseYaw)
  void checkMag(const Vec3& magBody, float declinationRad, bool yawObservable, uint32_t nowMs);
  void fuseFlowVel(float vn, float ve, float sigma, uint32_t nowMs);
  void fuseZeroVel(float sigma);

  void shift(float dn, float de, float dUp) { p_.x += dn; p_.y += de; p_.z -= dUp; }
  void setYaw(float yaw);  // hard heading reset (heading source changed)
  // heading unknown (no compass, GPS heading not learned): the attitude follows the AHRS and is kept
  // out of the estimation, so GPS innovations cannot drag the tilt / accelerometer bias around
  void slaveAttitude(const Quat& q, float tiltVar, float yawVar);

  Vec3 pos() const { return p_; }   // NED
  Vec3 vel() const { return v_; }
  float alt() const { return -p_.z; }
  const Quat& q() const { return q_; }
  float yaw() const { return q_.yaw(); }
  Vec3 gyroBias() const { return bg_; }
  Vec3 accBias() const { return ba_; }
  bool horizAided(uint32_t nowMs) const { return lastAidMs_ && nowMs - lastAidMs_ < 1000; }
  bool gpsUsed(uint32_t nowMs) const { return lastGpsMs_ && nowMs - lastGpsMs_ < 1000; }
  bool heightOk(uint32_t nowMs) const { return lastHgtMs_ && nowMs - lastHgtMs_ < 1000; }
  bool baroFault() const { return st_.faults & EKF_BARO_FAULT; }
  bool magFault() const { return st_.faults & EKF_MAG_FAULT; }
  const EkfStatus& status() const { return st_; }
  float var(int i) const { return P_[i][i]; }

 private:
  // scalar measurement: h = row of H (dense), returns the test ratio; fused only when accepted
  float fuse(const float* h, float innov, float R, float gate, bool apply = true);
  void inject();          // add the error state to the nominal state, zero it
  void sanitize();
  void resetPos(float n, float e, float var);
  void resetVel(float vn, float ve, float vd, float var);

  const Config* cfg_ = nullptr;
  bool init_ = false;
  Vec3 p_, v_, bg_, ba_;
  Quat q_;
  float dx_[N];
  float P_[N][N];
  EkfStatus st_{};
  uint32_t nowMs_ = 0;
  uint32_t lastAidMs_ = 0, lastGpsMs_ = 0, lastHgtMs_ = 0;
  uint32_t posRejSince_ = 0, velRejSince_ = 0, hgtRejSince_ = 0, baroGoodSince_ = 0;
  uint32_t magBadSince_ = 0, magGoodSince_ = 0;
};

}  // namespace fc
