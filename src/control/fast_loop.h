// AllQuad - the hard real-time loop (gyro task, 1-2 kHz):
//   gyro calibration -> filter chain (RPM, static & dynamic notches, low-pass)
//   -> attitude estimate (AHRS) -> rate PIDs (+TPA, anti-gravity, I-relax) -> mixer.
// Pure computation: the task wrapper reads the IMU and writes the motors.
#pragma once
#include "../bus/topics.h"
#include "../common/cycles.h"
#include "../common/filters.h"
#include "../config/config.h"
#include "../estimation/ahrs.h"
#include "../filter/dynamic_notch.h"
#include "mixer.h"
#include "pid.h"

namespace fc {

struct FastLoopOutput {
  float motor[4];       // 0..1 incl. idle; < 0 = stop this motor (turtle mode)
  bool armed;
  MotorMode mode;
};

class FastLoop {
 public:
  void configure(const Config& c, float loopHz);
  void setGains(const Config& c);  // rate PID gains only (no filter re-initialisation, safe in flight)
  void run(const Vec3& gyroRad, const Vec3& accMs2, bool imuOk, const ControlSetpoint& sp,
           const MagSample* mag, float declinationRad, const float motorHz[4], float dt, FastLoopOutput& out);

  const Ahrs& ahrs() const { return ahrs_; }
  const Vec3& gyroDps() const { return gyroDps_; }
  const Vec3& gyroUnfilteredDps() const { return gyroRawDps_; }
  const Vec3& accFiltered() const { return accFilt_; }
  bool gyroCalibrated() const { return calDone_; }
  void requestGyroCalibration() { calDone_ = false; calCount_ = 0; }
  const RatePid& pid(int axis) const { return pid_[axis]; }
  const DynamicNotch& dynNotch() const { return dyn_; }
  bool yawSpinRecovery() const { return spin_; }
  float dynIdle() const { return dynIdleOut_; }
  float setpointCutoffHz() const { return spCutoff_; }
  int profile() const { return profile_; }
  // earth-frame acceleration averaged since the last call (for the estimator)
  Vec3 takeAccEarthAverage();
  // delta angle / delta velocity (body) integrated since the last call (for the navigation EKF)
  void takeDelta(Vec3& dAng, Vec3& dVel, float& dt);
  float accVibration() const { return vibration_; }
  uint32_t accClips() const { return accClips_; }       // accelerometer samples at the end of the range
  static constexpr float ACC_CLIP = 15.5f * GRAVITY;    // +-16 g IMU range
  // CPU cycles of the last run(): gyro filter chain, attitude, PID + mixer
  uint32_t cycFilter() const { return cycFilter_; }
  uint32_t cycAhrs() const { return cycAhrs_; }
  uint32_t cycPid() const { return cycPid_; }

 private:
  const Config* cfg_ = nullptr;
  float loopHz_ = 1000;
  // calibration
  bool calDone_ = false;
  int calCount_ = 0;
  Vec3 calSum_, calMin_, calMax_, bias_;
  // filters
  Biquad notch1_[3], notch2_[3], lpf1_[3];
  Pt1 lpf2_[3], accLpf_[3], thrLpf_, vibLpf_;
  DynamicNotch dyn_;
  RpmFilter rpm_;
  // state
  Ahrs ahrs_;
  RatePid pid_[3];
  Mixer mixer_;
  Vec3 gyroDps_, gyroRawDps_, accFilt_;
  Vec3 accEarthSum_;
  int accEarthN_ = 0;
  Vec3 dAngSum_, dVelSum_;
  float dtSum_ = 0;
  float vibration_ = 0;
  uint32_t accClips_ = 0;
  bool wasArmed_ = false;
  uint32_t cycFilter_ = 0, cycAhrs_ = 0, cycPid_ = 0;
  // RC smoothing: the setpoint arrives in steps (control task / RC frames); PT2 per axis
  Pt1 spLpf1_[3], spLpf2_[3];
  uint32_t spSeq_ = 0;
  float spAge_ = 0, spInterval_ = 0.004f, spCutoff_ = 0;
  uint32_t spTick_ = 0;
  // dynamic gyro low-pass, throttle boost, dynamic idle, yaw spin recovery
  float lpfCutoff_ = -1;
  Pt1 thrBoostLpf_;
  float dynIdleI_ = 0, dynIdleOut_ = 0;
  bool spin_ = false;
  bool saturated_ = false;
  int profile_ = 0;
  void applyGains(const Config& c);
};

}  // namespace fc
