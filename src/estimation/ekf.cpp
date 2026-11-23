#include "ekf.h"

#include <string.h>

namespace fc {

enum : int { IP = 0, IV = 3, IT = 6, IBG = 9, IBA = 12 };

static Quat fromRotVec(const Vec3& r) {
  float a = r.norm();
  if (a < 1e-6f) return Quat(1.0f, 0.5f * r.x, 0.5f * r.y, 0.5f * r.z);
  float s = sinf(0.5f * a) / a;
  return Quat(cosf(0.5f * a), r.x * s, r.y * s, r.z * s);
}

static void rotMat(const Quat& q, float R[3][3]) {
  float w = q.w, x = q.x, y = q.y, z = q.z;
  R[0][0] = 1 - 2 * (y * y + z * z); R[0][1] = 2 * (x * y - w * z);     R[0][2] = 2 * (x * z + w * y);
  R[1][0] = 2 * (x * y + w * z);     R[1][1] = 1 - 2 * (x * x + z * z); R[1][2] = 2 * (y * z - w * x);
  R[2][0] = 2 * (x * z - w * y);     R[2][1] = 2 * (y * z + w * x);     R[2][2] = 1 - 2 * (x * x + y * y);
}

void Ekf::init(const Quat& q, float altUp) {
  q_ = q;
  q_.normalize();
  p_ = Vec3(0, 0, -altUp);
  v_ = Vec3();
  bg_ = ba_ = Vec3();
  memset(dx_, 0, sizeof(dx_));
  memset(P_, 0, sizeof(P_));
  for (int i = 0; i < 3; i++) {
    P_[IP + i][IP + i] = i < 2 ? 100.0f : 1.0f;
    P_[IV + i][IV + i] = 1.0f;
    P_[IT + i][IT + i] = i < 2 ? squaref(0.05f) : squaref(0.6f);
    P_[IBG + i][IBG + i] = squaref(0.01f);
    P_[IBA + i][IBA + i] = squaref(0.3f);
  }
  posRejSince_ = velRejSince_ = hgtRejSince_ = magBadSince_ = magGoodSince_ = 0;
  uint16_t keep = st_.faults & EKF_RESET;
  uint16_t pr = st_.posResets, vr = st_.velResets, fr = st_.filterResets;
  st_ = EkfStatus{};
  st_.faults = keep;
  st_.posResets = pr; st_.velResets = vr; st_.filterResets = fr;
  init_ = true;
}

void Ekf::predict(const Vec3& dAng, const Vec3& dVel, float dt, uint32_t now) {
  nowMs_ = now;
  if (!init_ || dt <= 0.0f || dt > 0.1f) return;
  const float sa = cfg_ ? cfg_->est_acc_noise : 0.6f;
  Vec3 da = dAng - bg_ * dt, dvb = dVel - ba_ * dt;
  Vec3 dvE = q_.rotate(dvb);  // dVel is in the body frame at the start of the interval
  q_ = q_ * fromRotVec(da);
  q_.normalize();
  Vec3 vNew = v_ + dvE + Vec3(0, 0, GRAVITY * dt);
  p_ += (v_ + vNew) * (0.5f * dt);
  v_ = vNew;

  // ---- covariance: P = F P F' + Q with F = I + A dt, A sparse ----
  float R[3][3];
  rotMat(q_, R);
  Vec3 a = dvE * (1.0f / dt);  // earth-frame specific force
  // Avt = -[a]x, Avb = -R, Atg = -R
  const float Avt[3][3] = {{0, a.z, -a.y}, {-a.z, 0, a.x}, {a.y, -a.x, 0}};
  // row i of A*X (X given by column accessor)
  auto aRow = [&](int i, auto&& X) -> float {
    if (i < IV) return X(IV + i);
    if (i < IT) {
      int k = i - IV;
      return Avt[k][0] * X(IT) + Avt[k][1] * X(IT + 1) + Avt[k][2] * X(IT + 2) -
             (R[k][0] * X(IBA) + R[k][1] * X(IBA + 1) + R[k][2] * X(IBA + 2));
    }
    if (i < IBG) {
      int k = i - IT;
      return -(R[k][0] * X(IBG) + R[k][1] * X(IBG + 1) + R[k][2] * X(IBG + 2));
    }
    return 0.0f;
  };
  static float M[N][N];
  for (int i = 0; i < N; i++)
    for (int j = 0; j < N; j++) M[i][j] = P_[i][j] + dt * aRow(i, [&](int r) { return P_[r][j]; });
  for (int i = 0; i < N; i++)
    for (int j = 0; j < N; j++) P_[i][j] = M[i][j] + dt * aRow(j, [&](int c) { return M[i][c]; });
  float qv = squaref(sa * dt), qp = 0.25f * qv * dt * dt;
  float qt = squaref(0.004f * dt), qbg = squaref(2e-5f) * dt, qba = squaref(0.004f) * dt;
  for (int i = 0; i < 3; i++) {
    P_[IP + i][IP + i] += qp;
    P_[IV + i][IV + i] += qv * (i == 2 ? 1.5f : 1.0f);
    P_[IT + i][IT + i] += qt * (i == 2 ? 2.0f : 1.0f);
    P_[IBG + i][IBG + i] += qbg;
    P_[IBA + i][IBA + i] += qba;
  }
  // variance limits (keep the filter responsive and numerically sane)
  for (int i = 0; i < 3; i++) {
    P_[IP + i][IP + i] = fminf(P_[IP + i][IP + i], 1e6f);
    P_[IV + i][IV + i] = fminf(P_[IV + i][IV + i], 1e4f);
    P_[IT + i][IT + i] = fminf(P_[IT + i][IT + i], 1.0f);
    P_[IBG + i][IBG + i] = fminf(P_[IBG + i][IBG + i], squaref(0.02f));
    P_[IBA + i][IBA + i] = fminf(P_[IBA + i][IBA + i], squaref(0.5f));
  }
  sanitize();
  // status
  st_.posStd = sqrtf(fmaxf(P_[0][0], P_[1][1]));
  st_.velStd = sqrtf(fmaxf(P_[3][3], P_[4][4]));
  st_.hgtStd = sqrtf(P_[2][2]);
  st_.yawStdDeg = sqrtf(P_[8][8]) * RAD2DEG;
  if (horizAided(now)) st_.faults &= ~EKF_NO_AIDING;
  else st_.faults |= EKF_NO_AIDING;
}

void Ekf::sanitize() {
  bool bad = false;
  for (int i = 0; i < N && !bad; i++) {
    if (!(P_[i][i] >= 0.0f) || P_[i][i] != P_[i][i]) bad = true;
    for (int j = i + 1; j < N; j++) P_[i][j] = P_[j][i] = 0.5f * (P_[i][j] + P_[j][i]);
  }
  float s = p_.x + p_.y + p_.z + v_.x + v_.y + v_.z + q_.w + q_.x + q_.y + q_.z;
  if (bad || s != s) {
    Quat q = (q_.w == q_.w && q_.x == q_.x) ? q_ : Quat();
    float alt = p_.z == p_.z ? -p_.z : 0.0f;
    init(q, alt);
    st_.faults |= EKF_RESET;
    st_.filterResets++;
  }
}

float Ekf::fuse(const float* h, float innov, float R, float gate, bool apply) {
  float PH[N];
  float S = R;
  for (int i = 0; i < N; i++) {
    float s = 0;
    for (int j = 0; j < N; j++) s += P_[i][j] * h[j];
    PH[i] = s;
  }
  for (int i = 0; i < N; i++) S += h[i] * PH[i];
  if (S <= 0) return 1e6f;
  float tr = innov * innov / (gate * gate * S);
  if (tr > 1.0f || !apply) return tr;
  float K[N];
  for (int i = 0; i < N; i++) K[i] = PH[i] / S;
  for (int i = 0; i < N; i++) dx_[i] = K[i] * innov;
  for (int i = 0; i < N; i++)
    for (int j = 0; j < N; j++) P_[i][j] -= K[i] * PH[j];
  inject();
  sanitize();
  return tr;
}

void Ekf::inject() {
  p_ += Vec3(dx_[0], dx_[1], dx_[2]);
  v_ += Vec3(dx_[3], dx_[4], dx_[5]);
  q_ = fromRotVec(Vec3(dx_[6], dx_[7], dx_[8])) * q_;
  q_.normalize();
  bg_ += Vec3(dx_[9], dx_[10], dx_[11]);
  ba_ += Vec3(dx_[12], dx_[13], dx_[14]);
  for (int i = 0; i < 3; i++) {
    bg_[i] = constrainf(bg_[i], -0.1f, 0.1f);
    ba_[i] = constrainf(ba_[i], -2.0f, 2.0f);
  }
  memset(dx_, 0, sizeof(dx_));
}

static void unit(float* h, int i) {
  memset(h, 0, sizeof(float) * Ekf::N);
  h[i] = 1.0f;
}

void Ekf::resetPos(float n, float e, float var) {
  p_.x = n; p_.y = e;
  for (int i = 0; i < N; i++) { P_[0][i] = P_[i][0] = 0; P_[1][i] = P_[i][1] = 0; }
  P_[0][0] = P_[1][1] = var;
  st_.posResets++;
}

void Ekf::resetVel(float vn, float ve, float vd, float var) {
  v_ = Vec3(vn, ve, vd);
  for (int k = IV; k < IV + 3; k++)
    for (int i = 0; i < N; i++) P_[k][i] = P_[i][k] = 0;
  for (int k = IV; k < IV + 3; k++) P_[k][k] = var;
  st_.velResets++;
}

void Ekf::fuseGpsPos(float n, float e, float sigma, uint32_t now) {
  if (!init_) return;
  const float gate = 5.0f;
  float R = sigma * sigma;
  float h[N];
  bool first = !lastGpsMs_ || now - lastGpsMs_ > 3000;
  if (first) {  // (re)acquired: start from the fix
    resetPos(n, e, R);
    lastGpsMs_ = lastAidMs_ = now;
    posRejSince_ = 0;
    return;
  }
  unit(h, 0);
  float t1 = fuse(h, n - p_.x, R, gate, false);
  unit(h, 1);
  float t2 = fuse(h, e - p_.y, R, gate, false);
  st_.trGpsPos = fmaxf(t1, t2);
  float limitMs = (cfg_ ? cfg_->gps_glitch_s : 10.0f) * 1000.0f;
  float jump = sqrtf(squaref(n - p_.x) + squaref(e - p_.y));
  float minJump = cfg_ ? cfg_->gps_glitch_m : 8.0f;
  if (st_.trGpsPos > 1.0f && jump > minJump * 0.5f) {
    if (!posRejSince_) posRejSince_ = now ? now : 1;
    st_.faults |= EKF_GPS_GLITCH;
    if (now - posRejSince_ > (uint32_t)limitMs) {  // persistent: accept the new position
      resetPos(n, e, R);
      posRejSince_ = 0;
      st_.faults &= ~EKF_GPS_GLITCH;
    }
    lastGpsMs_ = now;
    return;
  }
  posRejSince_ = 0;
  st_.faults &= ~EKF_GPS_GLITCH;
  unit(h, 0);
  fuse(h, n - p_.x, R, 1e3f);
  unit(h, 1);
  fuse(h, e - p_.y, R, 1e3f);
  lastGpsMs_ = lastAidMs_ = now;
}

void Ekf::fuseGpsVel(float vn, float ve, float vd, float sigma, uint32_t now) {
  if (!init_) return;
  const float gate = 5.0f;
  float R = sigma * sigma;
  float z[3] = {vn, ve, vd};
  float h[N];
  float tr = 0;
  for (int i = 0; i < 3; i++) {
    unit(h, IV + i);
    tr = fmaxf(tr, fuse(h, z[i] - v_[i], R * (i == 2 ? 4.0f : 1.0f), gate, false));
  }
  st_.trGpsVel = tr;
  if (tr > 1.0f) {
    if (!velRejSince_) velRejSince_ = now ? now : 1;
    st_.faults |= EKF_GPS_VEL;
    if (now - velRejSince_ > 2000) {  // the filter diverged: start over from GPS
      resetVel(vn, ve, vd, R);
      velRejSince_ = 0;
    }
    return;
  }
  velRejSince_ = 0;
  st_.faults &= ~EKF_GPS_VEL;
  for (int i = 0; i < 3; i++) {
    unit(h, IV + i);
    fuse(h, z[i] - v_[i], R * (i == 2 ? 4.0f : 1.0f), 1e3f);
  }
  lastAidMs_ = now;
}

void Ekf::fuseHeight(float altUp, float sigma, int src, uint32_t now) {
  if (!init_) return;
  float h[N];
  memset(h, 0, sizeof(h));
  h[2] = -1.0f;
  float innov = altUp - (-p_.z);
  float R = sigma * sigma;
  float tr = fuse(h, innov, R, 5.0f, false);
  if (src == 0) {
    // baro: persistent disagreement (with GPS height available) = baro fault
    if (tr > 1.0f) {
      if (!hgtRejSince_) hgtRejSince_ = now ? now : 1;
      baroGoodSince_ = 0;
      if (now - hgtRejSince_ > 5000) {
        if (gpsUsed(now)) st_.faults |= EKF_BARO_FAULT;
        else { fuse(h, innov, R, 1e3f); hgtRejSince_ = 0; }  // nothing better: follow the baro
      }
    } else {
      hgtRejSince_ = 0;
      if (st_.faults & EKF_BARO_FAULT) {
        if (!baroGoodSince_) baroGoodSince_ = now ? now : 1;
        if (now - baroGoodSince_ > 10000) st_.faults &= ~EKF_BARO_FAULT;
      }
    }
    st_.trHeight = tr;
    if ((st_.faults & EKF_BARO_FAULT) || tr > 1.0f) return;
  } else {
    st_.trHeight = tr;
    if (tr > 1.0f) {  // GPS height: no alternative, accept after 5 s
      if (!hgtRejSince_) hgtRejSince_ = now ? now : 1;
      if (now - hgtRejSince_ < 5000) return;
      hgtRejSince_ = 0;
    }
  }
  st_.heightSrc = (uint8_t)src;
  fuse(h, innov, R, 1e3f);
  lastHgtMs_ = now;
}

void Ekf::fuseYaw(float yaw, float sigma) {
  if (!init_) return;
  float h[N];
  unit(h, IT + 2);
  float innov = wrapPi(yaw - q_.yaw());
  st_.trYaw = fuse(h, innov, sigma * sigma, 5.0f);
  if (st_.trYaw > 1.0f) st_.faults |= EKF_HEAD_REJECT;
  else st_.faults &= ~EKF_HEAD_REJECT;
}

void Ekf::setYaw(float yaw) {
  float d = wrapPi(yaw - q_.yaw());
  q_ = fromRotVec(Vec3(0, 0, d)) * q_;
  q_.normalize();
  for (int i = 0; i < N; i++) P_[8][i] = P_[i][8] = 0;
  P_[8][8] = squaref(0.1f);
}

void Ekf::fuseTilt(const Quat& qRef, float sigma) {
  if (!init_) return;
  Quat e = qRef * q_.conj();  // earth-frame rotation from the EKF attitude to the reference
  if (e.w < 0) { e.w = -e.w; e.x = -e.x; e.y = -e.y; e.z = -e.z; }
  float h[N];
  unit(h, IT);
  fuse(h, 2.0f * e.x, sigma * sigma, 5.0f);
  unit(h, IT + 1);
  fuse(h, 2.0f * e.y, sigma * sigma, 5.0f);
}

void Ekf::relearnAccBias(float sigma) {
  if (!init_) return;
  for (int k = IBA; k < IBA + 3; k++) {
    for (int i = 0; i < N; i++) P_[k][i] = P_[i][k] = 0;
    P_[k][k] = sigma * sigma;
  }
}

void Ekf::slaveAttitude(const Quat& q, float tiltVar, float yawVar) {
  if (!init_) return;
  q_ = q;
  q_.normalize();
  for (int k = IT; k < IT + 3; k++)
    for (int i = 0; i < N; i++) P_[k][i] = P_[i][k] = 0;
  P_[IT][IT] = P_[IT + 1][IT + 1] = tiltVar;
  P_[IT + 2][IT + 2] = yawVar;
}

void Ekf::checkMag(const Vec3& m, float decl, bool observable, uint32_t now) {
  if (!init_) return;
  Vec3 mE = q_.rotate(m);
  if (mE.x * mE.x + mE.y * mE.y < 1e-6f) return;
  float innov = -wrapPi(atan2f(mE.y, mE.x) - decl);  // compass heading - EKF heading
  float S = P_[8][8] + squaref(0.12f);
  st_.trMag = innov * innov / (9.0f * S);
  if (observable && st_.trMag > 1.0f) {
    magGoodSince_ = 0;
    if (!magBadSince_) magBadSince_ = now ? now : 1;
    if (now - magBadSince_ > 5000) st_.faults |= EKF_MAG_FAULT;
  } else if (st_.trMag < 0.5f) {
    magBadSince_ = 0;
    if (st_.faults & EKF_MAG_FAULT) {
      if (!magGoodSince_) magGoodSince_ = now ? now : 1;
      if (now - magGoodSince_ > 10000) st_.faults &= ~EKF_MAG_FAULT;
    }
  }
}

void Ekf::fuseFlowVel(float vn, float ve, float sigma, uint32_t now) {
  if (!init_) return;
  float h[N];
  float R = sigma * sigma;
  unit(h, IV);
  float t1 = fuse(h, vn - v_.x, R, 5.0f);
  unit(h, IV + 1);
  float t2 = fuse(h, ve - v_.y, R, 5.0f);
  st_.trFlow = fmaxf(t1, t2);
  if (st_.trFlow > 1.0f) st_.faults |= EKF_FLOW_REJECT;
  else { st_.faults &= ~EKF_FLOW_REJECT; lastAidMs_ = now; }
}

void Ekf::fuseZeroVel(float sigma) {
  if (!init_) return;
  float h[N];
  float R = sigma * sigma;
  unit(h, IV);
  fuse(h, -v_.x, R, 1e3f);
  unit(h, IV + 1);
  fuse(h, -v_.y, R, 1e3f);
}

}  // namespace fc
