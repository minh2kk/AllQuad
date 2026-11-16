// AllQuad - in-flight PID autotune (ArduPilot-style "twitch" autotune for the rate loop).
//
// The craft hovers (altitude hold + self level). Repeatedly, on one axis at a time, it commands a
// short rate step ("twitch", alternating left/right), records the gyro response at the control rate
// and measures rise time, overshoot and ringing. After every group of twitches the P and D gains of
// that axis are adjusted, until the response is fast with 4-12 % overshoot and no ringing:
//   ringing ............................................ P down, D up
//   overshoot, peak still at the end of the step ...... P up, I down (I / feed-forward carry it over)
//   overshoot, early peak ............................... D up while D/P < 0.95 (and P down while
//                                                         D/P < 0.55), else P down
//   no overshoot ........................................ P up (D a little if D/P is low)
// Then (at_steps) the same axis gets feed-forward and I-term steps, flown as "ramp" twitches (setpoint
// ramps up in 60 ms, holds 300 ms) so that the feed-forward sees a realistic stick motion:
//   feed-forward: tracking lag of the ramp (gyro 50 % vs setpoint 50 %) above a quarter of the P/D
//                 rise time: FF up; < 3 ms or 6 % more overshoot than the first FF group: FF down
//   I-term      : steady error at the end of the hold: > 4 % I up, overshoot past the target I down
// Filters (at_steps 8): the gyro noise is measured while hovering between twitches; the result
// (gyro / D-term low-pass, dynamic notch range) is applied only when saved.
// Gains change only inside [0.5x, 2.5x] of the start values.
//
// Pilot: switch AUTOTUNE on in a hover, hands off the sticks (moving a stick pauses twitching and
// gives full control). Done for all axes: the craft just holds; land and disarm WITH the switch on
// to save, switch it off before disarming to throw the result away (the old gains come back).
#pragma once
#include "../common/math3d.h"
#include "../config/config.h"

namespace fc {

class AutoTune {
 public:
  enum class State : uint8_t { IDLE, SETTLE, TWITCH, RETURN, DONE };
  enum class Step : uint8_t { PD = 0, FF = 1, I = 2 };

  void begin(const Config& c);      // remember the current gains, start with the first axis
  // dt in s, rate/setpoint in deg/s (filtered gyro), pilotActive: sticks moved
  // returns true when the axis rate is overridden by a twitch (rateOut = commanded rate)
  bool update(Config& c, const float rateDps[3], const float attDeg[2], bool pilotActive, float dt,
              int& axisOut, float& rateOut);
  void restore(Config& c) const;    // back to the gains from begin()
  // gyro noise while hovering between twitches (deg/s, before and after the filters, control rate)
  void noiseSample(const float rawDps[3], const float filtDps[3], const float notchHz[3]);
  // filter changes measured during the tune (copied into c when saving); false = nothing to change
  bool filterResult(Config& c) const;
  bool done() const { return state_ == State::DONE; }
  bool active() const { return state_ != State::IDLE; }
  int axis() const { return axis_; }
  int round() const { return round_; }
  State state() const { return state_; }
  Step step() const { return step_; }
  float lastLagMs() const { return lastLag_; }
  float lastSteadyErr() const { return lastSs_; }
  float noiseRaw() const { return nRaw_ ? sqrtf(sRaw_ / nRaw_) : 0; }
  float noiseFilt() const { return nRaw_ ? sqrtf(sFilt_ / nRaw_) : 0; }
  bool gainsChanged() { bool g = changed_; changed_ = false; return g; }
  // last measured response of the current axis (for status / tests)
  float lastOvershoot() const { return lastOv_; }
  float lastRiseMs() const { return lastRise_; }

 private:
  void nextAxis(const Config& c);
  void startStep(Step s);
  bool nextStep();  // false = axis finished
  void evaluate(Config& c);
  void evaluateFfI(Config& c);
  float shape() const;  // setpoint shape of the current twitch (0..1)

  State state_ = State::IDLE;
  AxisPid start_[3];
  uint8_t axesMask_ = 3, steps_ = 1;
  bool hoverOnly_ = false;
  Step step_ = Step::PD;
  int axis_ = 0, round_ = 0, twitch_ = 0, good_ = 0;
  float t_ = 0, settle_ = 0, wait_ = 0;
  float target_ = 0;
  int dir_ = 1;
  // per-twitch measurement
  float peak_ = 0, peakT_ = 0, rise_ = -1, prevErr_ = 0;
  int crossings_ = 0;
  bool peaked_ = false;
  // per-group accumulation
  float sumOv_ = 0, sumRise_ = 0, sumPeakT_ = 0;
  int ringing_ = 0, n_ = 0;
  float lastOv_ = 0, lastRise_ = 0, lastLag_ = 0, lastSs_ = 0;
  bool loweredP_ = false;  // last round lowered P for overshoot (to notice when that made it worse)
  bool changed_ = false;
  // ramp twitch (FF / I) measurement
  float lagT_ = -1, holdSum_ = 0, sumLag_ = 0, sumSs_ = 0, ffRefOv_ = -1;
  float ffPrevLag_ = -1, ffPrevF_ = 0;
  bool ffPlateau_ = false;  // more F found useless: do not raise it again on this axis  // last round that raised F: lag before, F before
  int holdN_ = 0;
  // noise measurement for the filter step
  float lpRaw_[3] = {0, 0, 0}, lpFilt_[3] = {0, 0, 0};
  float sRaw_ = 0, sFilt_ = 0, notchSum_ = 0;
  int nRaw_ = 0, notchN_ = 0;
};

}  // namespace fc
