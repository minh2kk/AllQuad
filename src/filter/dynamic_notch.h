// AllQuad - dynamic notch filter (Betaflight-style)
//
// Gyro samples are collected per axis, a 128-point FFT is computed in small
// slices spread over several loop iterations (bounded CPU per loop), the
// strongest resonance between minHz and maxHz is located with parabolic
// interpolation, smoothed, and notches are re-tuned to it.
#pragma once
#include "../common/cycles.h"
#include "../common/filters.h"

namespace fc {

class DynamicNotch {
 public:
  static constexpr int N = 128;       // FFT length
  static constexpr int MAX_PEAKS = 2; // notches per axis

  void configure(float loopHz, float minHz, float maxHz, float q, int count);
  // Feed the (pre-filtered) gyro sample for all axes, returns the notched values.
  Vec3 apply(const Vec3& gyroDps);
  float centerHz(int axis, int peak = 0) const { return center_[axis][peak]; }
  bool enabled() const { return enabled_; }

 private:
  void step();  // advance the FFT / peak-detect pipeline by one bounded slice
  void butterflies(int len);  // one radix-2 stage of the FFT

  bool enabled_ = false;
  float loopHz_ = 1000, minHz_ = 100, maxHz_ = 600, q_ = 3.5f;
  int count_ = 1;
  int decim_ = 1, decimCount_ = 0;
  Vec3 accum_;
  float sampleHz_ = 1000;

  float buf_[3][N];
  int bufPos_ = 0;
  float work_[N], workIm_[N];
  float window_[N];
  float twRe_[N / 2], twIm_[N / 2];  // twiddle factors (no sin/cos in the loop)
  uint8_t rev_[N];                   // bit-reversal permutation
  int stage_ = 0, axis_ = 0;
  bool ready_ = false;

  float center_[3][MAX_PEAKS];
  Biquad notch_[3][MAX_PEAKS];
  Pt1 smooth_[3][MAX_PEAKS];
};

// RPM filter: notches on motor rotation frequency harmonics, for every motor and
// axis. Coefficients are refreshed round-robin (one motor/harmonic per loop).
class RpmFilter {
 public:
  static constexpr int MOTORS = 4;
  static constexpr int HARMONICS = 3;
  void configure(float loopHz, int harmonics, float minHz, float q, const uint8_t* weightPct = nullptr);
  void setMotorHz(const float hz[MOTORS]);  // mechanical rotation frequency per motor
  Vec3 apply(const Vec3& g);
  bool enabled() const { return harmonics_ > 0; }

 private:
  float loopHz_ = 1000, minHz_ = 100, q_ = 5.0f;
  float weight_[HARMONICS] = {1, 1, 1};  // per harmonic (Betaflight rpm_filter_weights)
  int harmonics_ = 0;
  float motorHz_[MOTORS] = {0};
  int updateIdx_ = 0;
  Biquad3 f_[MOTORS][HARMONICS];
  Pt1 hzLpf_[MOTORS];
};

}  // namespace fc
