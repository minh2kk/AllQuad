// AllQuad - compass calibration fits
//   ellipsoidFit: hard + soft iron (ArduPilot-style): general quadric least squares,
//                 centre = hard-iron offset, symmetric matrix = soft-iron correction
#pragma once
#include <math.h>

#include "../common/math3d.h"

namespace fc {

namespace magfit {

// Jacobi eigen-decomposition of a symmetric 3x3 matrix: a = v * diag(d) * v^T
inline void eigen3(const double a_in[3][3], double d[3], double v[3][3]) {
  double a[3][3];
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) { a[i][j] = a_in[i][j]; v[i][j] = i == j ? 1.0 : 0.0; }
  for (int sweep = 0; sweep < 50; sweep++) {
    double off = fabs(a[0][1]) + fabs(a[0][2]) + fabs(a[1][2]);
    if (off < 1e-15) break;
    for (int p = 0; p < 2; p++)
      for (int q = p + 1; q < 3; q++) {
        if (fabs(a[p][q]) < 1e-18) continue;
        double theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
        double t = (theta >= 0 ? 1.0 : -1.0) / (fabs(theta) + sqrt(theta * theta + 1.0));
        double c = 1.0 / sqrt(t * t + 1.0), s = t * c;
        for (int k = 0; k < 3; k++) {  // rotate columns p, q
          double akp = a[k][p], akq = a[k][q];
          a[k][p] = c * akp - s * akq;
          a[k][q] = s * akp + c * akq;
        }
        for (int k = 0; k < 3; k++) {  // rotate rows p, q
          double apk = a[p][k], aqk = a[q][k];
          a[p][k] = c * apk - s * aqk;
          a[q][k] = s * apk + c * aqk;
        }
        for (int k = 0; k < 3; k++) {
          double vkp = v[k][p], vkq = v[k][q];
          v[k][p] = c * vkp - s * vkq;
          v[k][q] = s * vkp + c * vkq;
        }
      }
  }
  for (int i = 0; i < 3; i++) d[i] = a[i][i];
}

// Solve n x n (n <= 9) linear system in place (Gaussian elimination, partial pivoting)
inline bool solve(double m[9][10], int n) {
  for (int c = 0; c < n; c++) {
    int piv = c;
    for (int r = c + 1; r < n; r++) if (fabs(m[r][c]) > fabs(m[piv][c])) piv = r;
    if (fabs(m[piv][c]) < 1e-12) return false;
    if (piv != c) for (int k = 0; k <= n; k++) { double t = m[c][k]; m[c][k] = m[piv][k]; m[piv][k] = t; }
    for (int r = 0; r < n; r++) {
      if (r == c) continue;
      double f = m[r][c] / m[c][c];
      for (int k = c; k <= n; k++) m[r][k] -= f * m[c][k];
    }
  }
  for (int r = 0; r < n; r++) m[r][n] /= m[r][r];
  return true;
}

}  // namespace magfit

// Fits x^T A x + 2 g^T x = 1 to the points. Returns the centre and the soft-iron matrix
// S = [xx, yy, zz, xy, xz, yz] (symmetric, determinant 1: field magnitude kept on average) so that
// corrected = S * (raw - centre) lies on a sphere. false when the points do not define an ellipsoid.
inline bool ellipsoidFit(const Vec3* p, int n, Vec3& centre, float S[6]) {
  if (n < 20) return false;
  // normalise for conditioning
  double mean[3] = {0, 0, 0}, scl = 0;
  for (int i = 0; i < n; i++) for (int k = 0; k < 3; k++) mean[k] += p[i][k] / n;
  for (int i = 0; i < n; i++) for (int k = 0; k < 3; k++) scl += fabs(p[i][k] - mean[k]) / (3.0 * n);
  if (scl < 1e-6) return false;
  double m[9][10] = {};
  for (int i = 0; i < n; i++) {
    double x = (p[i].x - mean[0]) / scl, y = (p[i].y - mean[1]) / scl, z = (p[i].z - mean[2]) / scl;
    double v[9] = {x * x, y * y, z * z, 2 * x * y, 2 * x * z, 2 * y * z, 2 * x, 2 * y, 2 * z};
    for (int r = 0; r < 9; r++) {
      for (int c = 0; c < 9; c++) m[r][c] += v[r] * v[c];
      m[r][9] += v[r];
    }
  }
  if (!magfit::solve(m, 9)) return false;
  double a = m[0][9], b = m[1][9], c = m[2][9], d = m[3][9], e = m[4][9], f = m[5][9];
  double g = m[6][9], h = m[7][9], k = m[8][9];
  double A[3][3] = {{a, d, e}, {d, b, f}, {e, f, c}};
  // centre: A c = -[g h k]
  double M3[9][10] = {};
  for (int r = 0; r < 3; r++) { for (int q = 0; q < 3; q++) M3[r][q] = A[r][q]; }
  M3[0][3] = -g; M3[1][3] = -h; M3[2][3] = -k;
  if (!magfit::solve(M3, 3)) return false;
  double cc[3] = {M3[0][3], M3[1][3], M3[2][3]};
  double cAc = 0;
  for (int r = 0; r < 3; r++) for (int q = 0; q < 3; q++) cAc += cc[r] * A[r][q] * cc[q];
  double kk = 1.0 + cAc;
  if (kk <= 0) return false;
  double M[3][3];
  for (int r = 0; r < 3; r++) for (int q = 0; q < 3; q++) M[r][q] = A[r][q] / kk;
  double ev[3], V[3][3];
  magfit::eigen3(M, ev, V);
  if (ev[0] <= 0 || ev[1] <= 0 || ev[2] <= 0) return false;
  // axis ratio sanity: a real compass ellipsoid is not extreme
  double emax = fmax(ev[0], fmax(ev[1], ev[2])), emin = fmin(ev[0], fmin(ev[1], ev[2]));
  if (emax / emin > 9.0) return false;  // radii ratio > 3
  double det = ev[0] * ev[1] * ev[2];
  double norm = pow(det, -1.0 / 6.0);  // det(S) = 1
  double sq[3] = {sqrt(ev[0]) * norm, sqrt(ev[1]) * norm, sqrt(ev[2]) * norm};
  double W[3][3];
  for (int r = 0; r < 3; r++)
    for (int q = 0; q < 3; q++) W[r][q] = V[r][0] * sq[0] * V[q][0] + V[r][1] * sq[1] * V[q][1] + V[r][2] * sq[2] * V[q][2];
  centre = Vec3((float)(cc[0] * scl + mean[0]), (float)(cc[1] * scl + mean[1]), (float)(cc[2] * scl + mean[2]));
  S[0] = (float)W[0][0]; S[1] = (float)W[1][1]; S[2] = (float)W[2][2];
  S[3] = (float)W[0][1]; S[4] = (float)W[0][2]; S[5] = (float)W[1][2];
  return true;
}

// corrected field: S * (raw - bias), S from mag_scale (diagonal) + mag_offdiag
inline Vec3 magCorrect(const Vec3& raw, const float bias[3], const float diag[3], const float off[3]) {
  float x = raw.x - bias[0], y = raw.y - bias[1], z = raw.z - bias[2];
  return Vec3(diag[0] * x + off[0] * y + off[1] * z, off[0] * x + diag[1] * y + off[2] * z,
              off[1] * x + off[2] * y + diag[2] * z);
}

}  // namespace fc
