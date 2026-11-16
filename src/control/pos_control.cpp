#include "pos_control.h"

namespace fc {

static constexpr float LEVEL_MAX_RATE = 300.0f;

void AttitudeControl::level(const Config& c, float tgtRoll, float tgtPitch, float roll, float pitch, float out[2]) {
  roll -= c.level_trim_roll_deg;
  pitch -= c.level_trim_pitch_deg;
  out[0] = constrainf(c.angle_p * (tgtRoll - roll), -LEVEL_MAX_RATE, LEVEL_MAX_RATE);
  out[1] = constrainf(c.angle_p * (tgtPitch - pitch), -LEVEL_MAX_RATE, LEVEL_MAX_RATE);
}

void AttitudeControl::horizon(const Config& c, float rs, float ps, float roll, float pitch, float out[2], int profile) {
  float lvl[2];
  level(c, rs * c.angle_limit_deg, -ps * c.angle_limit_deg, roll, pitch, lvl);
  float defl = fmaxf(fabsf(rs), fabsf(ps));
  // full self-level below the transition point, fading to pure rate at full stick
  float t = c.horizon_transition;
  float k = defl <= t ? 1.0f : constrainf(1.0f - (defl - t) / (1.0f - t + 1e-3f), 0.0f, 1.0f);
  out[0] = k * lvl[0] + (1.0f - k) * stickRate(rs, profileRate(c, profile, 0));
  out[1] = k * lvl[1] + (1.0f - k) * stickRate(-ps, profileRate(c, profile, 1));
}

// ---------------------------------------------------------------- altitude
void AltitudeControl::configure(const Config& c) {
  cfg_ = &c;
  hover_ = c.hover_throttle;
  limits();
}

void AltitudeControl::limits() {
  velPid_.set(cfg_->alt_vel_p, cfg_->alt_vel_i, cfg_->alt_vel_d, 0.35f, -hover_ + 0.08f, 0.9f - hover_);
}

void AltitudeControl::setHover(float h) {
  h = constrainf(h, 0.1f, 0.8f);
  velPid_.i -= h - hover_;
  hover_ = h;
  limits();
}

void AltitudeControl::reset(float thr, float cosTilt) {
  velPid_.reset(thr * constrainf(cosTilt, 0.6f, 1.0f) - hover_);
}

float AltitudeControl::hoverEstimate() const { return hover_ + velPid_.i; }

float AltitudeControl::update(float targetAlt, float climbFF, float maxUp, float maxDown, const NavEstimate& est,
                              float accUp, float cosTilt, float dt) {
  float velSp = climbFF + sqrtController(targetAlt - est.alt, cfg_->alt_pos_p, 3.0f);
  velSp = constrainf(velSp, -maxDown, maxUp);
  float thr = hover_ + velPid_.update(velSp - est.vz, accUp, dt);
  return constrainf(thr / constrainf(cosTilt, 0.6f, 1.0f), 0.05f, 0.95f);
}

// ---------------------------------------------------------------- position
void PositionControl::configure(const Config& c) {
  cfg_ = &c;
  float maxAcc = GRAVITY * tanf(c.pos_max_angle_deg * DEG2RAD);
  for (int i = 0; i < 2; i++) velPid_[i].set(c.pos_vel_p, c.pos_vel_i, 0.0f, maxAcc * 0.5f, -maxAcc, maxAcc);
}

void PositionControl::reset() {
  velPid_[0].reset();
  velPid_[1].reset();
  vsp_[0] = vsp_[1] = 0;
  corr_[0] = corr_[1] = 0;
}

void PositionControl::update(float tgtN, float tgtE, float ffN, float ffE, float maxSpeed, const NavEstimate& est,
                             float yaw, float windAccN, float windAccE, float dt, float& rollDeg, float& pitchDeg,
                             float accFfN, float accFfE) {
  const Config& c = *cfg_;
  // position error -> velocity (sqrt shaped), plus feed-forward, limited to maxSpeed
  float en = tgtN - est.n, ee = tgtE - est.e;
  float dist = sqrtf(en * en + ee * ee);
  float v = sqrtController(dist, c.pos_p, c.pos_max_accel * 0.5f);
  float want[2] = {ffN, ffE};
  if (dist > 1e-3f) { want[0] += en / dist * v; want[1] += ee / dist * v; }
  float sp = sqrtf(want[0] * want[0] + want[1] * want[1]);
  if (sp > maxSpeed) { want[0] *= maxSpeed / sp; want[1] *= maxSpeed / sp; }
  // acceleration limit on the velocity set-point (smooth starts and stops). A smooth feed-forward
  // (orbit, cruise) passes straight through; only the step-like part is rate limited.
  float dv = c.pos_max_accel * dt;
  float ff[2] = {ffN, ffE};
  float ffSp = sqrtf(ffN * ffN + ffE * ffE);
  if (ffSp > maxSpeed) { ff[0] *= maxSpeed / ffSp; ff[1] *= maxSpeed / ffSp; }
  for (int i = 0; i < 2; i++) {
    corr_[i] += constrainf((want[i] - ff[i]) - corr_[i], -dv, dv);
    vsp_[i] = ff[i] + corr_[i];
  }

  float accN = velPid_[0].update(vsp_[0] - est.vn, 0, dt) + accFfN;
  float accE = velPid_[1].update(vsp_[1] - est.ve, 0, dt) + accFfE;
  if (c.wind_comp) {
    // air drag = wind_drag * (wind - velocity): cancel the wind part (observer) and pre-compensate the
    // drag of the velocity we are about to fly (feed-forward part only)
    float lim = GRAVITY * tanf(c.pos_max_angle_deg * DEG2RAD) * 0.7f;
    accN += constrainf(c.wind_drag * vsp_[0] - windAccN, -lim, lim);
    accE += constrainf(c.wind_drag * vsp_[1] - windAccE, -lim, lim);
  }
  float cy = cosf(yaw), sy = sinf(yaw);
  float aFwd = accN * cy + accE * sy;
  float aRight = -accN * sy + accE * cy;
  float lim = c.pos_max_angle_deg;
  pitchDeg = constrainf(-atanf(aFwd / GRAVITY) * RAD2DEG, -lim, lim);
  rollDeg = constrainf(atanf(aRight / GRAVITY) * RAD2DEG, -lim, lim);
}

}  // namespace fc
