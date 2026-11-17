// AllQuad - PID controllers
#pragma once
#include "../config/config.h"
#include "../common/filters.h"

namespace fc {

// Rate (inner loop) PID with Betaflight-compatible gain scaling.
// Inputs in deg/s, output in "PID-sum units" (1000 = full motor range).
class RatePid {
 public:
  static constexpr float P_SCALE = 0.032029f;
  static constexpr float I_SCALE = 0.244381f;
  static constexpr float D_SCALE = 0.000529f;
  static constexpr float F_SCALE = 0.013754f / 100.0f;

  void configure(const AxisPid& g, float dtermLpfHz, float itermLimit, float outLimit,
                 float relaxCutoff, float loopHz, float dMinPct = 0, float dMinGain = 0, float dMinAdv = 0,
                 float ffJitter = 0, float ffBoost = 0, float ffMaxPct = 0, float maxRate = 0) {
    // feed-forward shaping (Betaflight 4.3+): jitter reduction (slow / shaky stick moves get less FF),
    // boost (stick acceleration: earlier start of a flip), and less FF close to the maximum rate
    // (no overshoot past the max rate at the end of a snap)
    ffJitterDps_ = ffJitter * 1.3f;  // ~ff_jitter stick units of 500 at a 650 deg/s rate
    ffBoostK_ = ffBoost * 1e-4f;
    ffMaxRate_ = maxRate;
    ffMaxFrom_ = (ffMaxPct > 0 && ffMaxPct < 100 && maxRate > 0) ? maxRate * ffMaxPct * 0.01f : 0.0f;
    ffBoostLpf_.setCutoff(15.0f, 1.0f / loopHz);
    kp_ = g.p * P_SCALE; ki_ = g.i * I_SCALE; kd_ = g.d * D_SCALE; kf_ = g.f * F_SCALE;
    // Betaflight d_min: D sits at d_min_pct % of D in calm flight and rises to the full D when the gyro
    // gets busy (prop wash, bounce-back) or the sticks move quickly
    dMinFrac_ = (dMinPct > 0 && dMinPct < 100) ? dMinPct * 0.01f : 1.0f;
    dMinGyro_ = dMinGain * 2e-6f;
    dMinSp_ = dMinAdv * 1e-5f;
    boostLpf_.setCutoff(10.0f, 1.0f / loopHz);
    iLimit_ = itermLimit; outLimit_ = outLimit; relaxCutoff_ = relaxCutoff;
    float dt = 1.0f / loopHz;
    invRelax_ = relaxCutoff > 0.0f ? 1.0f / relaxCutoff : 0.0f;
    dLpf_.lowpass(dtermLpfHz, loopHz);
    dLpf2_.setCutoff(dtermLpfHz * 1.5f, dt);
    ffLpf_.setCutoff(30.0f, dt);
    spLpf_.setCutoff(10.0f, dt);  // iterm relax reference
  }

  // iBoost: anti-gravity multiplier for the I accumulation (1 = off)
  AQ_HOT float update(float setpoint, float gyro, float dt, float tpa, bool integrate, float iBoost = 1.0f) {
    float err = setpoint - gyro;
    p = kp_ * err * tpa;

    float relax = 1.0f;
    if (relaxCutoff_ > 0.0f) {
      float hp = setpoint - spLpf_.apply(setpoint);
      relax = constrainf(1.0f - fabsf(hp) * invRelax_, 0.0f, 1.0f);
    }
    // anti-windup: while the mixer is out of authority, the I-term may only shrink (ArduPilot limit flags)
    if (integrate && !(saturated_ && err * i > 0.0f)) {
      i = constrainf(i + ki_ * err * dt * relax * iBoost, -iLimit_, iLimit_);
    }

    // measured interval: with a timer-driven loop the sample spacing follows the task, not the IMU clock
    const float invDt = 1.0f / dt;
    float dGyro = first_ ? 0.0f : (gyro - prevGyro_) * invDt;
    float dSp = first_ ? 0.0f : (setpoint - prevSp_) * invDt;
    prevGyro_ = gyro; prevSp_ = setpoint; first_ = false;
    float dg = dLpf2_.apply(dLpf_.apply(dGyro));
    float kd = kd_;
    if (dMinFrac_ < 1.0f) {
      float b = constrainf(fabsf(dg) * dMinGyro_ + fabsf(dSp) * dMinSp_, 0.0f, 1.0f);
      float boost = boostLpf_.apply(b);
      kd = kd_ * (dMinFrac_ + (1.0f - dMinFrac_) * boost);
    }
    d = -kd * dg * tpa;
    float ffv = ffLpf_.apply(dSp);
    float ffAcc = (ffv - prevFf_) * invDt;
    prevFf_ = ffv;
    float fsum = ffv + (ffBoostK_ > 0 ? ffBoostLpf_.apply(ffAcc) * ffBoostK_ : 0.0f);
    float scale = ffAtt_;
    if (ffMaxFrom_ > 0 && fabsf(setpoint) > ffMaxFrom_ && setpoint * fsum > 0.0f)
      scale *= constrainf((ffMaxRate_ - fabsf(setpoint)) / (ffMaxRate_ - ffMaxFrom_), 0.0f, 1.0f);
    f = kf_ * fsum * scale;

    return constrainf(p + i + d + f, -outLimit_, outLimit_);
  }

  // a new stick sample (raw rate setpoint, before smoothing): FF jitter reduction follows how far the
  // stick moved in the last two samples (as Betaflight). A sample without movement keeps the factor:
  // the smoothed FF of the last real move is still fading out and must not be cut half way.
  void rcFrame(float rawSp) {
    float delta = fabsf(rawSp - prevRaw_);
    prevRaw_ = rawSp;
    float d2 = 0.5f * (delta + prevDelta_);
    prevDelta_ = delta;
    if (delta < 0.01f) return;
    ffAtt_ = ffJitterDps_ > 0 ? constrainf(d2 / ffJitterDps_, 0.0f, 1.0f) : 1.0f;
  }
  void resetI() { i = 0.0f; }
  void setSaturated(bool s) { saturated_ = s; }
  void reset() {
    i = 0.0f; first_ = true; prevFf_ = 0; dLpf_.reset(); dLpf2_.reset(); ffLpf_.reset(); spLpf_.reset(); ffBoostLpf_.reset();
  }

  float p = 0, i = 0, d = 0, f = 0;  // last terms (debug / telemetry)

 private:
  float kp_ = 0, ki_ = 0, kd_ = 0, kf_ = 0, iLimit_ = 0, outLimit_ = 0, relaxCutoff_ = 0;
  float invRelax_ = 0.0f;
  float prevGyro_ = 0, prevSp_ = 0;
  float dMinFrac_ = 1.0f, dMinGyro_ = 0, dMinSp_ = 0;
  float ffJitterDps_ = 0, ffBoostK_ = 0, ffMaxRate_ = 0, ffMaxFrom_ = 0, ffAtt_ = 1.0f, prevRaw_ = 0, prevDelta_ = 0, prevFf_ = 0;
  bool first_ = true, saturated_ = false;
  Pt1 boostLpf_;
  Biquad dLpf_;
  Pt1 dLpf2_, ffLpf_, spLpf_, ffBoostLpf_;
};

// Generic PID for outer loops (altitude, position)
class Pid {
 public:
  void set(float kp, float ki, float kd, float iLimit, float outMin, float outMax) {
    kp_ = kp; ki_ = ki; kd_ = kd; iLimit_ = iLimit; outMin_ = outMin; outMax_ = outMax;
  }
  // measDot: derivative of the measurement (D on measurement)
  AQ_HOT float update(float err, float measDot, float dt, bool integrate = true) {
    float out = kp_ * err - kd_ * measDot;
    if (integrate) {
      float newI = constrainf(i + ki_ * err * dt, -iLimit_, iLimit_);
      float unclamped = out + newI;
      // conditional integration (anti-windup)
      if ((unclamped < outMax_ || err < 0.0f) && (unclamped > outMin_ || err > 0.0f)) i = newI;
    }
    return constrainf(out + i, outMin_, outMax_);
  }
  void reset(float iInit = 0.0f) { i = constrainf(iInit, -iLimit_, iLimit_); }
  float i = 0.0f;

 private:
  float kp_ = 0, ki_ = 0, kd_ = 0, iLimit_ = 0, outMin_ = -1, outMax_ = 1;
};

}  // namespace fc
