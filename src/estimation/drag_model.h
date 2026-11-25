// AllQuad - multirotor drag model for dead reckoning without GPS (ArduPilot EK3_DRAG_*).
//
// A multirotor's rotors are dragged sideways by the air flowing through them (momentum drag): the
// horizontal specific force the accelerometer feels in the body frame is about -k times the velocity
// through the air. Rotated to north / east with the heading:
//     aN = -k (vN - wN),   aE = -k (vE - wE)        (w: wind)
// which is linear in (k, k wN, k wE). While GPS works, recursive least squares (memory ~3 min) learns
// them from the accelerometer and the EKF velocity; once GPS is lost the same equation gives the ground
// velocity from the accelerometer alone: v = w - a / k. The EKF fuses that as a velocity measurement:
// the position no longer runs away with the wind, it drifts with the error of k and of the wind.
// Cost: a 3x3 update at 10 Hz.
#pragma once
#include <math.h>

namespace fc {

class DragModel {
 public:
  void reset() { *this = DragModel(); }
  // aN, aE: horizontal body specific force rotated by the heading (m/s^2); vN, vE: ground velocity
  void learn(float aN, float aE, float vN, float vE) {
    update(-vN, 1, 0, aN);
    update(-vE, 0, 1, aE);
    float v2 = vN * vN + vE * vE;
    if (v2 > 9.0f && fast_ < 60000) fast_++;  // samples at > 3 m/s: k is only seen while moving
    if (n_ < 60000) n_++;
  }
  // learned, plausible, and seen at speed
  bool ready() const { return n_ >= 150 && fast_ >= 50 && th_[0] > 0.05f && th_[0] < 2.0f && P_[0][0] < 0.05f; }
  float k() const { return th_[0]; }
  float pk() const { return P_[0][0]; }
  unsigned samples() const { return n_; }
  unsigned fastSamples() const { return fast_; }
  float windN() const { return th_[0] > 0.05f ? th_[1] / th_[0] : 0.0f; }
  float windE() const { return th_[0] > 0.05f ? th_[2] / th_[0] : 0.0f; }
  // ground velocity from the accelerometer (valid when ready())
  void velocity(float aN, float aE, float& vN, float& vE) const {
    vN = (th_[1] - aN) / th_[0];
    vE = (th_[2] - aE) / th_[0];
  }

 private:
  // y = phi . theta, phi = (p0, p1, p2)
  void update(float p0, float p1, float p2, float y) {
    float phi[3] = {p0, p1, p2}, Pp[3];
    for (int i = 0; i < 3; i++) Pp[i] = P_[i][0] * phi[0] + P_[i][1] * phi[1] + P_[i][2] * phi[2];
    float den = kLambda + phi[0] * Pp[0] + phi[1] * Pp[1] + phi[2] * Pp[2];
    if (!(den > 1e-6f)) return;
    float err = y - (phi[0] * th_[0] + phi[1] * th_[1] + phi[2] * th_[2]);
    if (fabsf(err) > 8.0f) return;  // a bump / landing: not drag
    for (int i = 0; i < 3; i++) th_[i] += Pp[i] / den * err;
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) P_[i][j] = (P_[i][j] - Pp[i] * Pp[j] / den) / kLambda;
    // keep it bounded (no excitation for a long time inflates P with the forgetting factor)
    for (int i = 0; i < 3; i++) if (P_[i][i] > 10.0f) for (int j = 0; j < 3; j++) { P_[i][j] *= 0.5f; P_[j][i] *= 0.5f; }
  }
  static constexpr float kLambda = 0.9997f;  // two equations per sample at 10 Hz: ~3 min memory
  float th_[3] = {0.3f, 0.0f, 0.0f};
  float P_[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  unsigned n_ = 0, fast_ = 0;
};

}  // namespace fc
