#include "autotune.h"

namespace fc {

static constexpr int TWITCHES_PER_GROUP = 4;   // alternating directions
static constexpr int MAX_ROUNDS = 20;          // groups per axis before accepting the result
static constexpr float TWITCH_S = 0.18f;       // step length
static constexpr float OV_LOW = 0.04f, OV_HIGH = 0.12f;
static constexpr float MIN_SCALE = 0.5f, MAX_SCALE = 2.5f;
static constexpr float RAMP_S = 0.06f, HOLD_S = 0.30f;  // FF / I twitches
static constexpr float LAG_HIGH_MS = 10.0f, LAG_LOW_MS = 3.0f, SS_HIGH = 0.04f, SS_LOW = -0.03f;

void AutoTune::begin(const Config& c) {
  for (int a = 0; a < 3; a++) start_[a] = c.pid[a];
  axesMask_ = c.at_axes ? c.at_axes : 3;
  steps_ = c.at_steps ? c.at_steps : 1;
  sRaw_ = sFilt_ = notchSum_ = 0;
  nRaw_ = notchN_ = 0;
  axis_ = -1;
  hoverOnly_ = !(steps_ & 7);
  if (hoverOnly_) {  // filters only: measure the noise in a 20 s hover, no twitches
    axis_ = 0;
    t_ = 0;
    settle_ = 0;
    state_ = State::SETTLE;
    return;
  }
  nextAxis(c);
}

void AutoTune::restore(Config& c) const {
  for (int a = 0; a < 3; a++) c.pid[a] = start_[a];
}

void AutoTune::startStep(Step st) {
  step_ = st;
  ffRefOv_ = -1;
  ffPrevLag_ = -1;
  ffPlateau_ = false;
  round_ = twitch_ = good_ = n_ = ringing_ = 0;
  sumOv_ = sumRise_ = sumPeakT_ = sumLag_ = sumSs_ = 0;
  dir_ = 1;
  settle_ = 0;
  state_ = State::SETTLE;
}

bool AutoTune::nextStep() {
  // order per axis: P/D, feed-forward (roll / pitch), I-term
  for (int s = (int)step_ + 1; s <= (int)Step::I; s++) {
    if (!(steps_ & (1 << s))) continue;
    if (s == (int)Step::FF && axis_ == 2) continue;  // yaw: no feed-forward tuning
    startStep((Step)s);
    return true;
  }
  return false;
}

void AutoTune::nextAxis(const Config&) {
  do { axis_++; } while (axis_ < 3 && !(axesMask_ & (1 << axis_)));
  if (axis_ >= 3) { state_ = State::DONE; return; }
  if (steps_ & 1) { startStep(Step::PD); return; }
  step_ = Step::PD;  // nextStep() starts after PD
  if (!nextStep()) nextAxis(Config());
}

float AutoTune::shape() const {
  if (step_ == Step::PD) return 1.0f;
  return t_ < RAMP_S ? t_ / RAMP_S : 1.0f;
}

void AutoTune::noiseSample(const float raw[3], const float filt[3], const float notch[3]) {
  if (state_ != State::SETTLE && state_ != State::RETURN) return;
  if (settle_ < 0.12f) return;  // only in a quiet hover between twitches
  for (int a = 0; a < 2; a++) {
    lpRaw_[a] += (raw[a] - lpRaw_[a]) * 0.4f;
    lpFilt_[a] += (filt[a] - lpFilt_[a]) * 0.4f;
    sRaw_ += squaref(raw[a] - lpRaw_[a]);
    sFilt_ += squaref(filt[a] - lpFilt_[a]);
  }
  nRaw_ += 2;
  float nh = 0;
  int k = 0;
  for (int a = 0; a < 2; a++)
    if (notch[a] > 0) { nh += notch[a]; k++; }
  if (k) { notchSum_ += nh / k; notchN_++; }
}

bool AutoTune::filterResult(Config& c) const {
  if (!(steps_ & 8) || nRaw_ < 500) return false;
  float nf = noiseFilt(), nr = noiseRaw();
  bool ch = false;
  if (nf < 0.6f && nr < 8.0f) {          // clean: open the low-pass filters (less delay)
    float g = fminf(c.gyro_lpf_hz * 1.25f, 300.0f), d = fminf(c.dterm_lpf_hz * 1.15f, 150.0f);
    ch |= g != c.gyro_lpf_hz || d != c.dterm_lpf_hz;
    c.gyro_lpf_hz = g; c.dterm_lpf_hz = d;
  } else if (nf > 2.0f) {                // noisy after the filters: more filtering
    float g = fmaxf(c.gyro_lpf_hz * 0.8f, 80.0f), d = fmaxf(c.dterm_lpf_hz * 0.85f, 60.0f);
    ch |= g != c.gyro_lpf_hz || d != c.dterm_lpf_hz;
    c.gyro_lpf_hz = g; c.dterm_lpf_hz = d;
    if (!c.dyn_notch_count) { c.dyn_notch_count = 1; ch = true; }
  }
  if (notchN_ > 100 && c.dyn_notch_count) {  // centre the dynamic notch range on the resonance seen
    float hz = notchSum_ / notchN_;
    float lo = constrainf(hz * 0.6f, 60.0f, 250.0f), hi = constrainf(hz * 1.8f, lo + 100.0f, 600.0f);
    if (fabsf(lo - c.dyn_notch_min) > 5 || fabsf(hi - c.dyn_notch_max) > 5) {
      c.dyn_notch_min = lo; c.dyn_notch_max = hi;
      ch = true;
    }
  }
  return ch;
}

void AutoTune::evaluateFfI(Config& c) {
  AxisPid& g = c.pid[axis_];
  const AxisPid& s = start_[axis_];
  float ov = sumOv_ / n_, lag = sumLag_ / n_, ss = sumSs_ / n_;
  lastLag_ = lag;
  lastSs_ = ss;
  bool inBand = false;
  if (step_ == Step::FF) {
    // the P/D step left some overshoot (ovRef): feed-forward may shorten the lag as long as it does
    // not add more than 6 % overshoot of its own. Target lag: a quarter of the P/D rise time.
    if (ffRefOv_ < 0) ffRefOv_ = ov;
    float lagTarget = fmaxf(LAG_HIGH_MS, 0.25f * (lastRise_ > 0 ? lastRise_ : 40.0f));
    float lo = fmaxf(s.f * MIN_SCALE, 20.0f), hi = fminf(fmaxf(s.f * MAX_SCALE, 60.0f), 400.0f);
    float f0 = g.f;
    // the last increase bought (almost) nothing: the remaining lag is the motors / filters, not
    // something feed-forward can remove. Go back to the previous value and accept it.
    bool plateau = ffPrevLag_ > 0 && ffPrevLag_ - lag < fmaxf(1.0f, 0.04f * ffPrevLag_);
    if (ov > ffRefOv_ + 0.06f || lag < LAG_LOW_MS) { g.f = constrainf(g.f * 0.9f, lo, hi); ffPrevLag_ = -1; }
    else if (plateau) { g.f = ffPrevF_; ffPrevLag_ = -1; ffPlateau_ = true; inBand = true; }
    else if (lag > lagTarget && !ffPlateau_) { ffPrevLag_ = lag; ffPrevF_ = g.f; g.f = constrainf(fmaxf(g.f, 20.0f) * 1.15f, lo, hi); }
    else inBand = true;
    if (!inBand && g.f == f0) inBand = true;  // at the limit: accept
  } else {
    float lo = fmaxf(s.i * MIN_SCALE, 20.0f), hi = fmaxf(s.i * MAX_SCALE, 40.0f);
    if (ss > SS_HIGH) g.i = constrainf(g.i * 1.2f, lo, hi);
    else if (ss < SS_LOW) g.i = constrainf(g.i * 0.85f, lo, hi);
    else inBand = true;
    if (!inBand && (g.i == lo || g.i == hi)) inBand = true;
  }
  changed_ = !inBand;
  good_ = inBand ? good_ + 1 : 0;
  round_++;
  n_ = ringing_ = 0;
  sumOv_ = sumRise_ = sumPeakT_ = sumLag_ = sumSs_ = 0;
  if (good_ >= 2 || round_ >= MAX_ROUNDS / 2) {
    if (!nextStep()) nextAxis(c);
  }
}

void AutoTune::evaluate(Config& c) {
  float ov = sumOv_ / n_, rise = sumRise_ / n_, peakFrac = sumPeakT_ / n_ / TWITCH_S;
  const float prevOv = lastOv_;
  const bool loweredP = loweredP_;
  loweredP_ = false;
  lastOv_ = ov;
  lastRise_ = rise;
  AxisPid& g = c.pid[axis_];
  const AxisPid& s = start_[axis_];
  auto clampGain = [](float v, float ref, float floor) {
    float lo = fmaxf(ref * MIN_SCALE, floor), hi = fmaxf(ref * MAX_SCALE, floor * 2);
    return constrainf(v, lo, hi);
  };
  bool hasD = axis_ < 2;  // yaw: no D (Betaflight default), damp with P instead
  // D may also grow up to ~P when the start value was very low
  auto clampD = [&](float v) { return constrainf(v, fmaxf(s.d * MIN_SCALE, 10.0f), fmaxf(s.d * MAX_SCALE, g.p * 0.95f)); };
  float ratio = g.p > 1 ? g.d / g.p : 1.0f;
  bool inBand = false;
  if (ringing_ >= TWITCHES_PER_GROUP / 2) {          // oscillation: too much P for the damping
    g.p = clampGain(g.p * 0.85f, s.p, 10);
    if (hasD) g.d = clampD(g.d * 1.10f);
  } else if (ov > OV_HIGH && peakFrac > 0.75f) {
    // the response is still climbing at the end of the step: I-term / feed-forward carry it past the
    // target while P is too weak to get there quickly -> more P, less I
    g.p = clampGain(g.p * 1.12f, s.p, 10);
    g.i = clampGain(g.i * 0.9f, s.i, 20);
  } else if (ov > OV_HIGH && loweredP && ov > prevOv * 1.05f) {
    // lowering P made the overshoot worse: it comes from I / feed-forward against a weak P, not from
    // too much P -> go the other way (the case of a start far below the right P)
    g.p = clampGain(g.p * 1.15f, s.p, 10);
    g.i = clampGain(g.i * 0.9f, s.i, 20);
  } else if (ov > OV_HIGH) {                           // early overshoot: P too strong for the damping
    loweredP_ = !hasD || ratio < 0.55f || ratio >= 0.95f;
    if (!hasD) g.p = clampGain(g.p * 0.9f, s.p, 10);
    else if (ratio < 0.55f) { g.p = clampGain(g.p * 0.92f, s.p, 10); g.d = clampD(g.d * 1.12f); }
    else if (ratio < 0.95f) g.d = clampD(fmaxf(g.d, 10.0f) * 1.12f);
    else g.p = clampGain(g.p * 0.9f, s.p, 10);       // D is already high: keep it, lower P
  } else if (ov < OV_LOW) {                            // well damped but could be faster
    g.p = clampGain(g.p * 1.12f, s.p, 10);
    if (hasD && ratio < 0.6f) g.d = clampD(g.d * 1.08f);
  } else {
    inBand = true;
  }
  changed_ = !inBand;
  good_ = inBand ? good_ + 1 : 0;
  round_++;
  n_ = ringing_ = 0;
  sumOv_ = sumRise_ = sumPeakT_ = 0;
  if (good_ >= 2 || round_ >= MAX_ROUNDS) {
    if (!nextStep()) nextAxis(c);
  }
}

bool AutoTune::update(Config& c, const float rate[3], const float att[2], bool pilot, float dt, int& axisOut,
                      float& rateOut) {
  axisOut = -1;
  if (state_ == State::IDLE || state_ == State::DONE) return false;
  if (pilot) {  // the pilot flies: abandon the current twitch, start over when the sticks are centred
    if (state_ == State::TWITCH) state_ = State::RETURN;
    settle_ = 0;
    return false;
  }
  t_ += dt;
  if (hoverOnly_) {
    bool quiet = fabsf(rate[0]) < 25 && fabsf(rate[1]) < 25 && fabsf(att[0]) < 6 && fabsf(att[1]) < 6;
    settle_ = quiet ? settle_ + dt : 0;
    if (t_ > 20.0f) state_ = State::DONE;
    return false;
  }
  switch (state_) {
    case State::RETURN:
    case State::SETTLE: {
      // level and quiet for 0.3 s before the next twitch; a craft that never gets quiet (a weak
      // tune wobbles) twitches anyway after 2 s, the measurement then shows what is wrong
      bool quiet = fabsf(rate[0]) < 25 && fabsf(rate[1]) < 25 && fabsf(rate[2]) < 25 && fabsf(att[0]) < 6 &&
                   fabsf(att[1]) < 6;
      settle_ = quiet ? settle_ + dt : 0;
      wait_ += dt;
      if (settle_ > 0.3f || (wait_ > 2.0f && fabsf(att[0]) < 12 && fabsf(att[1]) < 12)) {
        wait_ = 0;
        state_ = State::TWITCH;
        t_ = 0;
        target_ = (axis_ == 2 ? 0.6f : 1.0f) * (c.at_rate > 0 ? c.at_rate : 200.0f);
        peak_ = 0;
        peakT_ = 0;
        rise_ = -1;
        crossings_ = 0;
        peaked_ = false;
        prevErr_ = -1;
        lagT_ = -1;
        holdSum_ = 0;
        holdN_ = 0;
      }
      return false;
    }
    case State::TWITCH: {
      float r = rate[axis_] * dir_;  // response in the twitch direction
      if (step_ != Step::PD) {
        // ramp twitch: lag at 50 %, overshoot during the hold, mean of the last 150 ms of the hold
        if (lagT_ < 0 && r >= 0.5f * target_) lagT_ = t_;
        if (t_ > RAMP_S && r > peak_) peak_ = r;
        if (t_ > RAMP_S + HOLD_S - 0.15f) { holdSum_ += r; holdN_++; }
        bool tooFar = axis_ < 2 && fabsf(att[axis_]) > 35.0f;
        if (t_ >= RAMP_S + HOLD_S || tooFar) {
          float ov = peak_ / target_ - 1.0f;
          sumOv_ += ov > 0 ? ov : 0;
          sumLag_ += lagT_ >= 0 ? (lagT_ - 0.5f * RAMP_S) * 1000.0f : 50.0f;
          sumSs_ += holdN_ ? 1.0f - holdSum_ / holdN_ / target_ : 0.0f;
          n_++;
          twitch_++;
          dir_ = -dir_;
          state_ = State::RETURN;
          settle_ = 0;
          if (n_ >= TWITCHES_PER_GROUP) evaluateFfI(c);
          return false;
        }
        axisOut = axis_;
        rateOut = target_ * dir_ * shape();
        return true;
      }
      if (rise_ < 0 && r >= 0.9f * target_) rise_ = t_ * 1000.0f;
      if (r > peak_) { peak_ = r; peakT_ = t_; }
      if (rise_ >= 0) {  // ringing: the error changes sign repeatedly after the first rise
        float e = r - target_;
        float hyst = 0.04f * target_;
        if (e > hyst && prevErr_ < 0) { crossings_++; prevErr_ = 1; }
        else if (e < -hyst && prevErr_ > 0) { crossings_++; prevErr_ = -1; }
      }
      bool tooFar = axis_ < 2 && fabsf(att[axis_]) > 35.0f;
      if (t_ >= TWITCH_S || tooFar) {
        float ov = peak_ / target_ - 1.0f;
        sumOv_ += ov > 0 ? ov : 0;
        sumRise_ += rise_ >= 0 ? rise_ : TWITCH_S * 1000.0f * 1.5f;
        sumPeakT_ += peakT_;
        if (crossings_ >= 3) ringing_++;
        n_++;
        twitch_++;
        dir_ = -dir_;
        state_ = State::RETURN;
        settle_ = 0;
        if (n_ >= TWITCHES_PER_GROUP) evaluate(c);
        return false;
      }
      axisOut = axis_;
      rateOut = target_ * dir_;
      return true;
    }
    default:
      return false;
  }
}

}  // namespace fc
