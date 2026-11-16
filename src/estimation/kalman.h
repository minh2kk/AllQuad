// AllQuad - 3-state Kalman filter: [position, velocity, accelerometer bias] along one axis.
// Used for altitude (baro) and for each horizontal axis (GPS position/velocity, optical flow).
#pragma once

namespace fc {

class Kalman3 {
 public:
  void init(float pos, float vel, float pPos, float pVel, float pBias) {
    x[0] = pos; x[1] = vel; x[2] = 0;
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) P[i][j] = 0;
    P[0][0] = pPos; P[1][1] = pVel; P[2][2] = pBias;
  }

  // acc: measured acceleration along the axis (includes bias), sigmaAcc: accel noise (m/s^2)
  void predict(float acc, float dt, float sigmaAcc, float sigmaBias) {
    float a = acc - x[2];
    x[0] += x[1] * dt + 0.5f * a * dt * dt;
    x[1] += a * dt;
    // F = [[1, dt, -dt²/2], [0, 1, -dt], [0, 0, 1]]
    float F[3][3] = {{1, dt, -0.5f * dt * dt}, {0, 1, -dt}, {0, 0, 1}};
    float FP[3][3], N[3][3];
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) FP[i][j] = F[i][0] * P[0][j] + F[i][1] * P[1][j] + F[i][2] * P[2][j];
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) N[i][j] = FP[i][0] * F[j][0] + FP[i][1] * F[j][1] + FP[i][2] * F[j][2];
    // discrete white-noise acceleration + bias random walk
    float qa = sigmaAcc * sigmaAcc;
    N[0][0] += qa * dt * dt * dt * dt * 0.25f;
    N[0][1] += qa * dt * dt * dt * 0.5f;
    N[1][0] += qa * dt * dt * dt * 0.5f;
    N[1][1] += qa * dt * dt;
    N[2][2] += sigmaBias * sigmaBias * dt;
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) P[i][j] = N[i][j];
  }

  // Scalar measurement of state `idx` (0 = position, 1 = velocity). Returns the innovation.
  float update(int idx, float z, float R, float gate = 0.0f) {
    float y = z - x[idx];
    float S = P[idx][idx] + R;
    if (gate > 0.0f && y * y > gate * gate * S) return y;  // reject outliers (n-sigma gate)
    float K[3] = {P[0][idx] / S, P[1][idx] / S, P[2][idx] / S};
    for (int i = 0; i < 3; i++) x[i] += K[i] * y;
    float Prow[3] = {P[idx][0], P[idx][1], P[idx][2]};
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) P[i][j] -= K[i] * Prow[j];
    // keep symmetric
    for (int i = 0; i < 3; i++)
      for (int j = i + 1; j < 3; j++) P[i][j] = P[j][i] = 0.5f * (P[i][j] + P[j][i]);
    return y;
  }

  void shift(float dPos) { x[0] += dPos; }
  float pos() const { return x[0]; }
  float vel() const { return x[1]; }
  float bias() const { return x[2]; }
  float posVar() const { return P[0][0]; }

  float x[3] = {0, 0, 0};
  float P[3][3] = {{0}};
};

}  // namespace fc
