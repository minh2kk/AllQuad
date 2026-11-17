#include "fast_loop.h"

namespace fc {

static constexpr int GYRO_CAL_SAMPLES = 1000;
static constexpr float GYRO_CAL_MAX_SPREAD = 0.05f;  // rad/s

void FastLoop::configure(const Config& c, float loopHz) {
  cfg_ = &c;
  loopHz_ = loopHz;
  float dt = 1.0f / loopHz;
  for (int i = 0; i < 3; i++) {
    notch1_[i].notch(c.notch1_hz, c.notch1_cut, loopHz);
    notch2_[i].notch(c.notch2_hz, c.notch2_cut, loopHz);
    lpf1_[i].lowpass(c.gyro_lpf_hz, loopHz);
    lpf2_[i].setCutoff(c.gyro_lpf2_hz, dt);
    accLpf_[i].setCutoff(c.acc_lpf_hz, dt);
  }
  lpfCutoff_ = -1;
  spCutoff_ = 0;
  applyGains(c);
  thrLpf_.setCutoff(5.0f, dt);
  thrBoostLpf_.setCutoff(15.0f, dt);
  vibLpf_.setCutoff(2.0f, dt);
  dyn_.configure(loopHz, c.dyn_notch_min, c.dyn_notch_max, c.dyn_notch_q, c.dyn_notch_count);
  rpm_.configure(loopHz, (c.esc_telemetry || c.dshot_bidir) ? c.rpm_harmonics : 0, c.rpm_min_hz, c.rpm_q, c.rpm_weight);
  mixer_.configure(c.yaw_reversed != 0, c.airmode != 0);
  ahrs_.setCorrectionDivider((int)(loopHz / 1000.0f + 0.5f));  // attitude correction at ~1 kHz
}

void FastLoop::applyGains(const Config& c) {
  for (int i = 0; i < 3; i++) {
    float lim = i == 2 ? c.pidsum_limit_yaw : c.pidsum_limit;
    pid_[i].configure(profilePid(c, profile_, i), c.dterm_lpf_hz, c.iterm_limit, lim, c.iterm_relax_cutoff, loopHz_,
                      c.d_min_pct, c.d_min_gain, c.d_min_adv, c.ff_jitter, c.ff_boost, c.ff_max_pct,
                      profileRate(c, profile_, i).max);
  }
}

void FastLoop::setGains(const Config& c) { applyGains(c); }

void FastLoop::takeDelta(Vec3& dAng, Vec3& dVel, float& dt) {
  dAng = dAngSum_;
  dVel = dVelSum_;
  dt = dtSum_;
  dAngSum_ = dVelSum_ = Vec3();
  dtSum_ = 0;
}

Vec3 FastLoop::takeAccEarthAverage() {
  Vec3 a = accEarthN_ ? accEarthSum_ * (1.0f / accEarthN_) : ahrs_.accEarth();
  accEarthSum_ = Vec3();
  accEarthN_ = 0;
  return a;
}

FAST_CODE void FastLoop::run(const Vec3& gyroRad, const Vec3& acc, bool imuOk, const ControlSetpoint& sp,
                   const MagSample* mag, float declinationRad, const float motorHz[4], float dt,
                   FastLoopOutput& out) {
  const Config& c = *cfg_;
  out.armed = false;
  out.mode = MotorMode::STOPPED;
  for (int i = 0; i < 4; i++) out.motor[i] = 0;
  if (!imuOk) return;

  // ---- gyro calibration: average while still, restart if moved ----
  if (!calDone_) {
    if (calCount_ == 0) { calSum_ = Vec3(); calMin_ = gyroRad; calMax_ = gyroRad; }
    calSum_ += gyroRad;
    for (int i = 0; i < 3; i++) {
      calMin_[i] = fminf(calMin_[i], gyroRad[i]);
      calMax_[i] = fmaxf(calMax_[i], gyroRad[i]);
    }
    if (++calCount_ >= GYRO_CAL_SAMPLES) {
      Vec3 s = calMax_ - calMin_;
      if (s.x < GYRO_CAL_MAX_SPREAD && s.y < GYRO_CAL_MAX_SPREAD && s.z < GYRO_CAL_MAX_SPREAD) {
        bias_ = calSum_ * (1.0f / GYRO_CAL_SAMPLES);
        calDone_ = true;
        Vec3 a = acc - Vec3(c.acc_bias[0], c.acc_bias[1], c.acc_bias[2]);
        for (int i = 0; i < 3; i++) accLpf_[i].reset(a[i]);
        ahrs_.reset();
        ahrs_.initFromAcc(a);
      }
      calCount_ = 0;
    }
  }

  // ---- gyro filter chain (deg/s) ----
  uint32_t c0 = cycles();
  Vec3 g = (gyroRad - bias_) * RAD2DEG;
  gyroRawDps_ = g;
  rpm_.setMotorHz(motorHz);
  g = rpm_.apply(g);
  for (int i = 0; i < 3; i++) g[i] = notch2_[i].apply(notch1_[i].apply(g[i]));
  g = dyn_.apply(g);
  // dynamic gyro low-pass (Betaflight dyn_lpf): the second stage follows the throttle, low cutoff
  // (more filtering, motor noise is low-frequency) at low throttle, high cutoff at high throttle
  if (c.dyn_lpf_max > 0 && (spTick_ & 7) == 0) {
    float t = constrainf(sp.throttle, 0.0f, 1.0f);
    float hz = c.dyn_lpf_min + (float)(c.dyn_lpf_max - c.dyn_lpf_min) * t * (2.0f - t);
    if (fabsf(hz - lpfCutoff_) > 1.0f) {
      lpfCutoff_ = hz;
      for (int i = 0; i < 3; i++) lpf2_[i].setCutoff(hz, dt);
    }
  }
  for (int i = 0; i < 3; i++) g[i] = lpf2_[i].apply(lpf1_[i].apply(g[i]));
  gyroDps_ = g;

  uint32_t c1 = cycles();
  cycFilter_ = c1 - c0;

  // ---- accelerometer & attitude ----
  Vec3 a = acc - Vec3(c.acc_bias[0], c.acc_bias[1], c.acc_bias[2]);
  for (int i = 0; i < 3; i++) a[i] *= c.acc_scale[i];  // 6-point calibration (1 = none)
  for (int i = 0; i < 3; i++) accFilt_[i] = accLpf_[i].apply(a[i]);
  vibration_ = vibLpf_.apply((a - accFilt_).norm());
  if (calDone_) {
    ahrs_.update(gyroDps_ * DEG2RAD, accFilt_, (mag && mag->healthy) ? &mag->field : nullptr, declinationRad, dt,
                 sp.accKinValid ? &sp.accKin : nullptr);
    accEarthSum_ += ahrs_.accEarth();
    accEarthN_++;
    dAngSum_ += gyroDps_ * (DEG2RAD * dt);
    dVelSum_ += a * dt;
    dtSum_ += dt;
  }

  uint32_t c2 = cycles();
  cycAhrs_ = c2 - c1;
  cycPid_ = 0;

  // ---- PID / rate profile switch (gains only, safe in flight) ----
  if (sp.profile != profile_ && sp.profile <= 2) {
    profile_ = sp.profile;
    applyGains(c);
  }

  // ---- RC smoothing: PT2 on the rate setpoint, cutoff from the measured setpoint update rate ----
  spTick_++;
  spAge_ += dt;
  if (sp.seq != spSeq_) {
    spSeq_ = sp.seq;
    for (int i = 0; i < 3; i++) pid_[i].rcFrame(sp.rateDps[i]);
    if (spAge_ < 0.1f) spInterval_ += 0.05f * (spAge_ - spInterval_);
    spAge_ = 0;
  }
  if ((spTick_ & 63) == 1) {
    float hz = c.rc_smooth_hz ? (float)c.rc_smooth_hz : constrainf(0.25f / fmaxf(spInterval_, 1e-3f), 15.0f, 120.0f);
    if (fabsf(hz - spCutoff_) > 0.5f) {
      spCutoff_ = hz;
      for (int i = 0; i < 3; i++) { spLpf1_[i].setCutoff(hz, dt); spLpf2_[i].setCutoff(hz, dt); }
    }
  }
  float rateSp[3];
  for (int i = 0; i < 3; i++) rateSp[i] = spLpf2_[i].apply(spLpf1_[i].apply(sp.rateDps[i]));

  // ---- yaw spin recovery (Betaflight): beyond the gyro range the rate loop cannot win; aim for zero
  // rotation with full authority and no I / feed-forward until the spin is below half the threshold ----
  if (c.yaw_spin_thr > 0) {
    float gz = fabsf(gyroRawDps_.z);
    if (!spin_ && gz > c.yaw_spin_thr && sp.motorMode == MotorMode::NORMAL) spin_ = true;
    else if (spin_ && gz < c.yaw_spin_thr * 0.5f) spin_ = false;
  } else {
    spin_ = false;
  }

  // ---- motors ----
  bool armed = sp.motorMode != MotorMode::STOPPED && calDone_;
  if (!armed) {
    for (int i = 0; i < 3; i++) pid_[i].update(0, gyroDps_[i], dt, 1.0f, false);  // keep filters warm
    if (wasArmed_) for (int i = 0; i < 3; i++) pid_[i].reset();
    wasArmed_ = false;
    thrLpf_.reset(0);
    thrBoostLpf_.reset(0);
    dynIdleI_ = dynIdleOut_ = 0;
    spin_ = false;
    for (int i = 0; i < 3; i++) { spLpf1_[i].reset(0); spLpf2_[i].reset(0); }
    return;
  }
  if (!wasArmed_) for (int i = 0; i < 3; i++) pid_[i].reset();
  wasArmed_ = true;
  out.armed = true;
  out.mode = sp.motorMode;

  if (sp.motorMode == MotorMode::TURTLE) {
    Mixer::turtle(sp.turtleStick[0], sp.turtleStick[1], out.motor);
    return;
  }

  float thr = sp.throttle;
  // throttle boost (Betaflight): adds the fast part of throttle moves for a quicker response
  float thrFast = thr - thrBoostLpf_.apply(thr);
  if (c.thr_boost) thr = constrainf(thr + c.thr_boost * 0.1f * thrFast, 0.0f, 1.0f);
  // TPA: reduce P and D at high throttle
  float tpa = 1.0f;
  if (thr > c.tpa_breakpoint && c.tpa_breakpoint < 1.0f)
    tpa = 1.0f - c.tpa_rate * (thr - c.tpa_breakpoint) / (1.0f - c.tpa_breakpoint);
  // anti-gravity: boost I while the throttle moves quickly
  float thrHp = fabsf(thr - thrLpf_.apply(thr));
  float iBoost = 1.0f + c.anti_gravity * constrainf(thrHp * 8.0f, 0.0f, 1.0f);

  bool integrate = sp.integrate && !spin_;
  if (spin_) {
    for (int i = 0; i < 3; i++) { rateSp[i] = 0; pid_[i].resetI(); }
  }
  float pidOut[3];
  for (int i = 0; i < 3; i++) {
    pid_[i].setSaturated(saturated_);
    pidOut[i] = pid_[i].update(rateSp[i], gyroDps_[i], dt, tpa, integrate, iBoost) / 1000.0f;
  }
  // spin recovery: full yaw authority against the spin (the PID sum limit would cap it)
  if (spin_) pidOut[2] = gyroRawDps_.z > 0 ? -1.0f : 1.0f;

  float mix[4];
  mixer_.setAirmode(c.airmode != 0 || spin_);
  mixer_.mix(thr, pidOut[0], pidOut[1], pidOut[2], mix);
  saturated_ = mixer_.saturated();

  // dynamic idle (Betaflight dyn_idle_min_rpm): keep the slowest motor above a minimum RPM so the
  // props never stall in dives / flips; needs motor RPM (bidirectional DShot or ESC telemetry)
  float idle = constrainf(c.motor_idle, 0.0f, 0.2f);
  if (c.dyn_idle_rpm && motorHz) {
    float mn = 1e9f;
    for (int i = 0; i < 4; i++) mn = fminf(mn, motorHz[i]);
    if (mn > 0.0f && mn < 1e8f) {
      float err = c.dyn_idle_rpm * 100.0f - mn * 60.0f;  // rpm
      dynIdleI_ = constrainf(dynIdleI_ + err * 2e-5f * dt * 60.0f, 0.0f, 0.15f);
      dynIdleOut_ = constrainf(err * 5e-5f + dynIdleI_, 0.0f, 0.2f);
    }
    idle = fmaxf(idle, dynIdleOut_);
  }
  float sag = (sp.sagFactor > 1.0f && sp.sagFactor < 1.5f) ? sp.sagFactor : 1.0f;
  for (int i = 0; i < 4; i++) {
    float m = Mixer::linearize(mix[i], c.thrust_expo);  // thrust -> motor command
    out.motor[i] = constrainf((idle + m * (1.0f - idle)) * sag, 0.0f, 1.0f);
  }
  cycPid_ = cycles() - c2;
}

}  // namespace fc
