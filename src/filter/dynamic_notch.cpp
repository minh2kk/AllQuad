#include "dynamic_notch.h"

#include <stdlib.h>
#include <string.h>

namespace fc {

// ------------------------------------------------------------ DynamicNotch
void DynamicNotch::configure(float loopHz, float minHz, float maxHz, float q, int count) {
  loopHz_ = loopHz;
  minHz_ = minHz;
  maxHz_ = constrainf(maxHz, minHz + 20.0f, loopHz * 0.45f);
  q_ = q > 0.5f ? q : 3.5f;
  count_ = count < 0 ? 0 : (count > MAX_PEAKS ? MAX_PEAKS : count);
  enabled_ = count_ > 0 && minHz > 0.0f;
  decim_ = (int)(loopHz / (maxHz_ * 2.2f));
  if (decim_ < 1) decim_ = 1;
  sampleHz_ = loopHz / decim_;
  memset(buf_, 0, sizeof(buf_));
  bufPos_ = 0;
  decimCount_ = 0;
  stage_ = 0;
  axis_ = 0;
  ready_ = false;
  for (int i = 0; i < N; i++) window_[i] = 0.5f - 0.5f * cosf(2.0f * PI_F * i / (N - 1));  // Hann
  for (int k = 0; k < N / 2; k++) {
    twRe_[k] = cosf(-2.0f * PI_F * k / N);
    twIm_[k] = sinf(-2.0f * PI_F * k / N);
  }
  int bits = 0;
  while ((1 << bits) < N) bits++;
  for (int i = 0; i < N; i++) {
    int r = 0;
    for (int b = 0; b < bits; b++) r |= ((i >> b) & 1) << (bits - 1 - b);
    rev_[i] = (uint8_t)r;
  }
  for (int a = 0; a < 3; a++)
    for (int p = 0; p < MAX_PEAKS; p++) {
      center_[a][p] = 0;
      notch_[a][p].setBypass(true);
      notch_[a][p].reset();
      smooth_[a][p].setCutoff(4.0f, (N / 2) / sampleHz_);
    }
}

FAST_CODE Vec3 DynamicNotch::apply(const Vec3& g) {
  if (!enabled_) return g;
  // decimated, averaged input for the analyser
  accum_ += g;
  if (++decimCount_ >= decim_) {
    float k = 1.0f / decimCount_;
    for (int a = 0; a < 3; a++) buf_[a][bufPos_] = accum_[a] * k;
    bufPos_ = (bufPos_ + 1) % N;
    decimCount_ = 0;
    accum_ = Vec3();
    // start a new analysis every N/2 samples (50 % overlap)
    if (bufPos_ % (N / 2) == 0) ready_ = true;
  }
  step();
  Vec3 out = g;
  for (int a = 0; a < 3; a++)
    for (int p = 0; p < count_; p++) out[a] = notch_[a][p].apply(out[a]);
  return out;
}

// One radix-2 stage (N/2 butterflies) with table twiddles: every call costs the same.
FAST_CODE void DynamicNotch::butterflies(int len) {
  int half = len / 2, step = N / len;
  for (int i = 0; i < N; i += len) {
    for (int k = 0; k < half; k++) {
      float cr = twRe_[k * step], ci = twIm_[k * step];
      int u = i + k, v = u + half;
      float tr = work_[v] * cr - workIm_[v] * ci;
      float ti = work_[v] * ci + workIm_[v] * cr;
      work_[v] = work_[u] - tr; workIm_[v] = workIm_[u] - ti;
      work_[u] += tr; workIm_[u] += ti;
    }
  }
}

FAST_CODE void DynamicNotch::step() {
  // pipeline per axis: 0 copy+window (bit-reversed), 1..7 FFT stages, 8 peak search
  constexpr int LOG2N = 7;
  static_assert((1 << LOG2N) == N, "N must be 2^LOG2N");
  if (stage_ == 0) {
    if (!ready_) return;
    // copy the ring buffer (oldest first), remove mean, window, store in bit-reversed order
    const float* b = buf_[axis_];
    float mean = 0;
    for (int i = 0; i < N; i++) mean += b[i];
    mean *= 1.0f / N;
    int src = bufPos_;
    for (int i = 0; i < N; i++) {
      int d = rev_[i];
      work_[d] = (b[src] - mean) * window_[i];
      workIm_[d] = 0;
      if (++src == N) src = 0;
    }
    stage_ = 1;
  } else if (stage_ <= LOG2N) {
    butterflies(1 << stage_);
    stage_++;
  } else {
    float binHz = sampleHz_ / N;
    int lo = (int)(minHz_ / binHz), hi = (int)(maxHz_ / binHz);
    if (lo < 2) lo = 2;
    if (hi > N / 2 - 2) hi = N / 2 - 2;
    float mag[N / 2];
    float sum = 0;
    for (int k = 1; k < N / 2; k++) {
      mag[k] = work_[k] * work_[k] + workIm_[k] * workIm_[k];
      if (k >= lo && k <= hi) sum += mag[k];
    }
    float mean = sum / (hi - lo + 1);
    int used = -100;
    for (int p = 0; p < count_; p++) {
      int best = -1;
      for (int k = lo; k <= hi; k++) {
        if (abs(k - used) <= 2) continue;
        if (best < 0 || mag[k] > mag[best]) best = k;
      }
      if (best < 0 || mag[best] < 4.0f * mean) continue;  // nothing stands out
      used = best;
      float y0 = sqrtf(mag[best - 1]), y1 = sqrtf(mag[best]), y2 = sqrtf(mag[best + 1]);
      float den = y0 - 2.0f * y1 + y2;
      float d = fabsf(den) > 1e-9f ? 0.5f * (y0 - y2) / den : 0.0f;
      float f = constrainf((best + constrainf(d, -0.5f, 0.5f)) * binHz, minHz_, maxHz_);
      if (center_[axis_][p] <= 0) smooth_[axis_][p].reset(f);
      center_[axis_][p] = smooth_[axis_][p].apply(f);
      notch_[axis_][p].notchQ(center_[axis_][p], q_, loopHz_);
    }
    axis_++;
    stage_ = 0;
    if (axis_ >= 3) { axis_ = 0; ready_ = false; }
  }
}

// ------------------------------------------------------------ RpmFilter
void RpmFilter::configure(float loopHz, int harmonics, float minHz, float q, const uint8_t* weightPct) {
  loopHz_ = loopHz;
  for (int h = 0; h < HARMONICS; h++) weight_[h] = weightPct ? weightPct[h] * 0.01f : 1.0f;
  harmonics_ = harmonics < 0 ? 0 : (harmonics > HARMONICS ? HARMONICS : harmonics);
  minHz_ = minHz;
  q_ = q > 0.5f ? q : 5.0f;
  for (int m = 0; m < MOTORS; m++) {
    motorHz_[m] = 0;
    hzLpf_[m].setCutoff(100.0f, (float)(MOTORS * (harmonics_ ? harmonics_ : 1)) / loopHz);
    for (int h = 0; h < HARMONICS; h++) { f_[m][h].setBypass(true); f_[m][h].reset(); }
  }
}

FAST_CODE void RpmFilter::setMotorHz(const float hz[MOTORS]) {
  for (int m = 0; m < MOTORS; m++) motorHz_[m] = hz[m];
}

FAST_CODE Vec3 RpmFilter::apply(const Vec3& g) {
  if (!harmonics_) return g;
  // refresh one motor/harmonic per call
  int m = updateIdx_ / harmonics_, h = updateIdx_ % harmonics_;
  updateIdx_ = (updateIdx_ + 1) % (MOTORS * harmonics_);
  float hz = hzLpf_[m].apply(motorHz_[m]) * (h + 1);
  // fade in over FADE_HZ above the minimum (Betaflight rpm_filter_fade_range_hz)
  constexpr float FADE_HZ = 50.0f;
  float w = (hz - minHz_) * (1.0f / FADE_HZ);
  if (w > 1.0f) w = 1.0f;
  w *= weight_[h];  // a weak harmonic: less notch, less delay
  if (w <= 0.0f || hz > loopHz_ * 0.45f) {
    if (!f_[m][h].bypassed()) { f_[m][h].setBypass(true); f_[m][h].reset(); }
  } else {
    f_[m][h].notchQ(hz, q_, loopHz_);  // one coefficient set shared by the 3 axes
    f_[m][h].setWeight(w);
  }
  Vec3 out = g;
  for (int mm = 0; mm < MOTORS; mm++)
    for (int hh = 0; hh < harmonics_; hh++) f_[mm][hh].apply(out);
  return out;
}

}  // namespace fc
