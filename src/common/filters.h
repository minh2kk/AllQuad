// AllQuad - digital filters
#pragma once
#include "math3d.h"

namespace fc {

// First order low pass
class Pt1 {
 public:
  void setCutoff(float cutoffHz, float dt) {
    if (cutoffHz <= 0.0f) { k_ = 1.0f; return; }
    float rc = 1.0f / (2.0f * PI_F * cutoffHz);
    k_ = dt / (rc + dt);
  }
  AQ_HOT float apply(float x) { y_ += k_ * (x - y_); return y_; }
  void reset(float v = 0.0f) { y_ = v; }
  float value() const { return y_; }

 private:
  float k_ = 1.0f, y_ = 0.0f;
};

// Second order section (transposed direct form II). Coefficients can be
// changed on the fly (dynamic / RPM notches) without resetting the state.
class Biquad {
 public:
  void lowpass(float cutoffHz, float sampleHz, float q = 0.70710678f) {
    if (cutoffHz <= 0.0f || cutoffHz >= sampleHz * 0.48f) { bypass_ = true; return; }
    float w = 2.0f * PI_F * cutoffHz / sampleHz;
    float sn = sinf(w), cs = cosf(w), alpha = sn / (2.0f * q);
    float a0 = 1.0f + alpha;
    b0_ = (1.0f - cs) * 0.5f / a0; b1_ = (1.0f - cs) / a0; b2_ = b0_;
    a1_ = -2.0f * cs / a0; a2_ = (1.0f - alpha) / a0;
    bypass_ = false;
  }
  // Notch defined by quality factor
  void notchQ(float centerHz, float q, float sampleHz) {
    if (centerHz <= 0.0f || q <= 0.0f || centerHz >= sampleHz * 0.48f) { bypass_ = true; return; }
    float w = 2.0f * PI_F * centerHz / sampleHz;
    float sn = sinf(w), cs = cosf(w), alpha = sn / (2.0f * q);
    float a0 = 1.0f + alpha;
    b0_ = 1.0f / a0; b1_ = -2.0f * cs / a0; b2_ = b0_;
    a1_ = b1_; a2_ = (1.0f - alpha) / a0;
    bypass_ = false;
  }
  // Betaflight-style notch definition (center + lower cutoff)
  void notch(float centerHz, float cutoffHz, float sampleHz) {
    if (centerHz <= 0.0f || cutoffHz <= 0.0f || cutoffHz >= centerHz) { bypass_ = true; return; }
    notchQ(centerHz, centerHz * cutoffHz / (centerHz * centerHz - cutoffHz * cutoffHz), sampleHz);
  }
  void setBypass(bool b) { bypass_ = b; }
  AQ_HOT float apply(float x) {
    if (bypass_) return x;
    float y = b0_ * x + z1_;
    z1_ = b1_ * x - a1_ * y + z2_;
    z2_ = b2_ * x - a2_ * y;
    return y;
  }
  void reset() { z1_ = z2_ = 0.0f; }
  bool bypassed() const { return bypass_; }

 private:
  bool bypass_ = true;
  float b0_ = 1, b1_ = 0, b2_ = 0, a1_ = 0, a2_ = 0, z1_ = 0, z2_ = 0;
};

// Three axes sharing one set of coefficients (RPM filter: same motor frequency on every axis).
// One coefficient update per call instead of three, and the coefficients stay in registers.
class Biquad3 {
 public:
  void notchQ(float centerHz, float q, float sampleHz) {
    if (centerHz <= 0.0f || q <= 0.0f || centerHz >= sampleHz * 0.48f) { bypass_ = true; return; }
    float w = 2.0f * PI_F * centerHz / sampleHz;
    float sn = sinf(w), cs = cosf(w), alpha = sn / (2.0f * q);
    float ia0 = 1.0f / (1.0f + alpha);
    b0_ = ia0; b1_ = -2.0f * cs * ia0; a2_ = (1.0f - alpha) * ia0;
    bypass_ = false;
  }
  void setBypass(bool b) { bypass_ = b; }
  // 0..1 mix of the notched signal (RPM fade near the lowest notch frequency: no step when a notch
  // switches on or off, and less phase delay at low rpm where the noise is small)
  void setWeight(float w) { weight_ = w; }
  bool bypassed() const { return bypass_; }
  void reset() { for (int i = 0; i < 3; i++) z1_[i] = z2_[i] = 0.0f; }
  // notch: b2 = b0, a1 = b1
  void apply(Vec3& v) {
    if (bypass_) return;
    for (int i = 0; i < 3; i++) {
      float x = v[i];
      float y = b0_ * x + z1_[i];
      z1_[i] = b1_ * (x - y) + z2_[i];
      z2_[i] = b0_ * x - a2_ * y;
      v[i] = weight_ >= 1.0f ? y : x + weight_ * (y - x);
    }
  }

 private:
  bool bypass_ = true;
  float b0_ = 1, b1_ = 0, a2_ = 0, weight_ = 1.0f;
  float z1_[3] = {0, 0, 0}, z2_[3] = {0, 0, 0};
};

}  // namespace fc
