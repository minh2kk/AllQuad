// AllQuad - compass offsets learned in flight (ArduPilot COMPASS_LEARN).
//
// The field the compass reads is the earth field turned into the body frame plus what the craft adds
// (hard iron: magnets, steel, a current loop that is always there):  f = R^T m + b.
// With R from the attitude, this is linear in the 6 unknowns (m: the earth field in that frame, b: the
// offset). R only has to be right up to a constant heading: the heading here is the gyro integrated
// from the start of the flight, not the compass (that is the thing being fixed) - a constant heading
// error just turns m. Turning the craft round (yaw over most of the circle) separates m from b.
// Least squares on 6x6 normal equations: a few multiply-adds per sample at 5 Hz, one solve at the end.
#pragma once
#include <math.h>
#include <stdint.h>

#include "../common/math3d.h"

namespace fc {

class MagLearn {
 public:
  void reset() { *this = MagLearn(); }
  // f: compass field (current calibration applied), q: attitude whose heading is the integrated gyro
  void add(const Vec3& f, const Quat& q, float yaw) {
    Vec3 c0 = q.rotateInv(Vec3(1, 0, 0)), c1 = q.rotateInv(Vec3(0, 1, 0)), c2 = q.rotateInv(Vec3(0, 0, 1));
    for (int i = 0; i < 3; i++) {
      float row[6] = {c0[i], c1[i], c2[i], 0, 0, 0};
      row[3 + i] = 1;
      float y = f[i];
      for (int a = 0; a < 6; a++) {
        b_[a] += row[a] * y;
        for (int c = a; c < 6; c++) A_[a][c] += row[a] * row[c];
      }
      yy_ += y * y;
    }
    float d = yaw * 57.29578f;
    while (d < 0) d += 360.0f;
    int bin = (int)(d / 30.0f) % 12;
    if (bins_[bin] < 255) bins_[bin]++;
    n_++;
  }
  unsigned samples() const { return n_; }
  // 30-degree heading sectors with at least 5 samples
  int sectors() const {
    int k = 0;
    for (int i = 0; i < 12; i++) k += bins_[i] >= 5;
    return k;
  }
  // m: earth field in the learning frame, b: offset (calibrated frame), rms: residual per axis
  bool solve(Vec3& m, Vec3& b, float& rms) const {
    float M[6][7];
    for (int a = 0; a < 6; a++) {
      for (int c = 0; c < 6; c++) M[a][c] = c >= a ? A_[a][c] : A_[c][a];
      M[a][6] = b_[a];
    }
    for (int col = 0; col < 6; col++) {  // Gauss-Jordan with partial pivoting
      int p = col;
      for (int r = col + 1; r < 6; r++) if (fabsf(M[r][col]) > fabsf(M[p][col])) p = r;
      if (fabsf(M[p][col]) < 1e-6f * (float)(n_ + 1)) return false;
      if (p != col) for (int c = 0; c < 7; c++) { float t = M[p][c]; M[p][c] = M[col][c]; M[col][c] = t; }
      for (int r = 0; r < 6; r++) {
        if (r == col) continue;
        float k = M[r][col] / M[col][col];
        for (int c = col; c < 7; c++) M[r][c] -= k * M[col][c];
      }
    }
    float x[6];
    for (int a = 0; a < 6; a++) x[a] = M[a][6] / M[a][a];
    m = Vec3(x[0], x[1], x[2]);
    b = Vec3(x[3], x[4], x[5]);
    // residual: sum (y - row.x)^2 = yy - 2 x.b + x.A.x
    float xb = 0, xAx = 0;
    for (int a = 0; a < 6; a++) {
      xb += x[a] * b_[a];
      for (int c = 0; c < 6; c++) xAx += x[a] * (c >= a ? A_[a][c] : A_[c][a]) * x[c];
    }
    float ss = yy_ - 2 * xb + xAx;
    rms = sqrtf(fmaxf(ss, 0.0f) / (3.0f * (float)(n_ ? n_ : 1)));
    return true;
  }

 private:
  float A_[6][6] = {};
  float b_[6] = {};
  float yy_ = 0;
  unsigned n_ = 0;
  uint8_t bins_[12] = {};
};

// offset in the calibrated frame -> change of the raw bias: raw = S^-1 f + bias, S the soft-iron matrix
inline Vec3 magBiasDelta(const Vec3& db, const float diag[3], const float off[3]) {
  float a = diag[0], b = off[0], c = off[1], d = diag[1], e = off[2], f = diag[2];
  // S = [a b c; b d e; c e f]
  float A = d * f - e * e, B = -(b * f - c * e), C = b * e - c * d;
  float det = a * A + b * B + c * C;
  if (fabsf(det) < 1e-9f) return db;
  float D = a * f - c * c, E = -(a * e - b * c), F = a * d - b * b;
  // inverse (symmetric): [A B C; B D E; C E F] / det
  return Vec3((A * db.x + B * db.y + C * db.z) / det, (B * db.x + D * db.y + E * db.z) / det,
              (C * db.x + E * db.y + F * db.z) / det);
}

}  // namespace fc
