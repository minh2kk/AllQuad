#include "nav_estimator.h"

namespace fc {

static constexpr double EARTH_R = 6371000.0;

void NavEstimator::llaToNed(double lat, double lon, double refLat, double refLon, float& n, float& e) {
  n = (float)((lat - refLat) * DEG2RAD * EARTH_R);
  e = (float)((lon - refLon) * DEG2RAD * EARTH_R * cos(refLat * DEG2RAD));
}

void NavEstimator::nedToLla(float n, float e, double refLat, double refLon, double& lat, double& lon) {
  lat = refLat + n / EARTH_R * RAD2DEG;
  lon = refLon + e / (EARTH_R * cos(refLat * DEG2RAD)) * RAD2DEG;
}

void NavEstimator::predict(const Vec3& accEarth, float dt, bool horizTrusted) {
  float sa = cfg_ ? cfg_->est_acc_noise : 0.6f;
  if (zInit_) {
    kz_.predict(-accEarth.z, dt, sa, 0.02f);
    if (st_.aglValid) st_.agl += kz_.vel() * dt;
  }
  if (hInit_) {
    if (horizTrusted) {
      kn_.predict(accEarth.x, dt, sa, 0.01f);
      ke_.predict(accEarth.y, dt, sa, 0.01f);
    } else {  // constant-velocity model with a large acceleration uncertainty (GPS carries the estimate)
      kn_.predict(kn_.bias(), dt, 4.0f, 0.0f);
      ke_.predict(ke_.bias(), dt, 4.0f, 0.0f);
    }
  }
}

void NavEstimator::fuseBaro(const BaroSample& b) {
  if (!b.healthy) return;
  // first baro sample after a GPS-only start (late chip): join the GPS height without a jump
  if (zInit_ && !lastBaroMs_) baroOffset_ = b.altM - kz_.pos();
  float alt = b.altM - baroOffset_;
  if (!zInit_) {
    kz_.init(alt, 0, 1.0f, 0.5f, 0.1f);
    zInit_ = true;
  }
  float r = cfg_ ? cfg_->est_baro_noise : 0.4f;
  if (cfg_) baroCorrect(alt, r, kz_.pos(), baroVh2_, cfg_->baro_dyn, baroGround_ && cfg_->baro_gnd);
  float S = kz_.P[0][0] + r * r, y = alt - kz_.pos();
  if (y * y > 36.0f * S) {
    // outside the 6-sigma gate: a baro spike is skipped. Rejected for 2 s in a row means the estimate
    // ran away (accelerometer clipping on an impact, a long vibration burst), not the baro: reset the
    // height to the baro, keep the velocity but trust it less. Without this the gate kept the baro out
    // until the covariance had grown enough (tens of seconds) while ALTHOLD / LAND followed a wrong height.
    if (!baroRejSince_) baroRejSince_ = nowMs_ ? nowMs_ : 1;
    if (nowMs_ - baroRejSince_ > 2000) {
      kz_.x[0] = alt;
      kz_.x[1] *= 0.5f;
      kz_.P[0][0] = r * r * 4.0f;
      kz_.P[1][1] = kz_.P[1][1] > 1.0f ? kz_.P[1][1] : 1.0f;
      kz_.P[0][1] = kz_.P[1][0] = kz_.P[0][2] = kz_.P[2][0] = 0;
      baroRejSince_ = 0;
      baroResets_++;
    }
  } else {
    baroRejSince_ = 0;
    kz_.update(0, alt, r * r);
  }
  lastBaroMs_ = nowMs_;
}

void NavEstimator::fuseRange(const RangeSample& r, float cosTilt, uint32_t nowMs) {
  float maxM = cfg_ ? cfg_->rf_max_m : 8.0f;
  if (!r.valid || r.distM <= 0.02f || r.distM > maxM || cosTilt < 0.86f) { rangeUsed_ = false; return; }
  float agl = r.distM * cosTilt;
  if (!st_.aglValid) st_.agl = agl;
  else st_.agl += 0.35f * (agl - st_.agl);
  st_.aglValid = true;
  lastRangeMs_ = nowMs;
  // the range is also a height measurement (short term it beats the baro by an order of magnitude)
  float z;
  rangeUsed_ = zInit_ && rh_.measure(agl, kz_.pos(), sqrtf(kz_.P[0][0]), nowMs, z);
  if (rangeUsed_) {
    float s = RangeHeight::sigma(agl);
    kz_.update(0, z, s * s);
  }
}

void NavEstimator::fuseGps(const GpsData& g, uint32_t nowMs) {
  lastGps_ = g;
  uint8_t minSats = cfg_ ? cfg_->gps_min_sats : 7;
  float maxHacc = cfg_ ? cfg_->gps_max_hacc : 5.0f;
  bool good = g.fix3d && g.sats >= minSats && g.hAcc < maxHacc;
  if (!good) return;
  if (!st_.homeSet || rehomePending_) {
    if (st_.homeSet && hInit_) {
      // move the local frame origin: shift the states by the old->new home offset
      float dn, de;
      llaToNed(g.lat, g.lon, st_.homeLat, st_.homeLon, dn, de);
      kn_.shift(-dn);
      ke_.shift(-de);
    }
    st_.homeLat = g.lat;
    st_.homeLon = g.lon;
    if (st_.homeSet) gpsHOff_ += g.altMsl - st_.homeAltMsl;  // same GPS height in the height frame
    st_.homeAltMsl = g.altMsl;
    st_.homeSet = true;
    rehomePending_ = false;
  }
  // GPS height: learns its offset to the baro height while the baro works (slowly: GPS height wanders),
  // carries the height when the baro is gone (no jump, the offset is already there)
  if (!zInit_ && !lastBaroMs_ && st_.homeSet && nowMs > 5000) {  // no barometer at all: GPS height only
    kz_.init(g.altMsl - st_.homeAltMsl, 0, 4.0f, 0.5f, 0.1f);
    zInit_ = true;
    gpsHInit_ = true;
    gpsHOff_ = 0;
  }
  if (zInit_) {
    float gh = g.altMsl - st_.homeAltMsl;
    bool baroFresh = lastBaroMs_ && nowMs - lastBaroMs_ < 1000;
    if (baroFresh) {
      if (!gpsHInit_) { gpsHOff_ = kz_.pos() - gh; gpsHInit_ = true; }
      else gpsHOff_ += 0.01f * (kz_.pos() - gh - gpsHOff_);
    } else if (gpsHInit_) {
      float r = g.vAcc > 1.5f ? g.vAcc : 1.5f;
      kz_.update(0, gh + gpsHOff_, r * r);
      lastGpsHMs_ = nowMs;
    }
  }
  float n, e;
  llaToNed(g.lat, g.lon, st_.homeLat, st_.homeLon, n, e);
  float rp = g.hAcc > 0.8f ? g.hAcc : 0.8f;
  float rv = cfg_ ? cfg_->est_gps_vel_noise : 0.3f;
  if (hReset_) { hInit_ = false; hReset_ = false; glitch_ = false; }
  // inertial prediction disagrees with GPS in velocity too: the filter diverged, it is not a GPS glitch
  if (hInit_ && gpsUsable_) {
    float vin = sqrtf((g.velN - kn_.vel()) * (g.velN - kn_.vel()) + (g.velE - ke_.vel()) * (g.velE - ke_.vel()));
    if (vin > 3.0f) { hInit_ = false; glitch_ = false; }
  }
  if (!hInit_ || (!gpsUsable_ && !flowUsable_)) {
    kn_.init(n, g.velN, rp * rp, 1.0f, 0.05f);
    ke_.init(e, g.velE, rp * rp, 1.0f, 0.05f);
    hInit_ = true;
  } else if (!gpsUsable_) {
    // GPS returns after a flow-only period: snap position to GPS
    kn_.x[0] = n;
    ke_.x[0] = e;
    glitch_ = false;
  }
  kn_.update(1, g.velN, rv * rv, 5.0f);
  ke_.update(1, g.velE, rv * rv, 5.0f);

  // ---- glitch protection (ArduPilot style): a position that jumps away from the inertial
  // prediction is not fused; the craft keeps navigating on GPS velocity + accelerometers.
  // A jump that persists for gps_glitch_s is accepted as the new truth (position reset).
  float dn = n - kn_.pos(), de = e - ke_.pos();
  float jump = sqrtf(dn * dn + de * de);
  float sigma = sqrtf(fminf(fmaxf(kn_.posVar(), ke_.posVar()), 4.0f) + rp * rp);  // capped: no drift-in
  float gate = fmaxf(cfg_ ? cfg_->gps_glitch_m : 8.0f, 5.0f * sigma);
  if (jump > gate) {
    if (!glitch_) { glitch_ = true; glitchStartMs_ = nowMs; st_.glitchCount++; }
    float limitS = cfg_ ? cfg_->gps_glitch_s : 10.0f;
    if (nowMs - glitchStartMs_ > (uint32_t)(limitS * 1000.0f)) {
      kn_.x[0] = n;
      ke_.x[0] = e;
      kn_.P[0][0] = ke_.P[0][0] = rp * rp;
      glitch_ = false;
    }
    lastGpsMs_ = nowMs;
    st_.eph = fmaxf(rp, jump);
    return;
  }
  glitch_ = false;
  kn_.update(0, n, rp * rp);
  ke_.update(0, e, rp * rp);
  lastGpsMs_ = nowMs;
  st_.eph = rp;
}

void NavEstimator::fuseFlow(const FlowSample& f, const Vec3& rateDps, float yaw, float cosTilt, uint32_t nowMs) {
  if (!st_.aglValid || st_.agl < 0.15f || f.quality < 30 || cosTilt < 0.86f) return;
  // sensor alignment
  float ix = f.rateX, iy = f.rateY;
  uint8_t align = cfg_ ? cfg_->flow_align : 0;
  for (uint8_t i = 0; i < align; i++) { float t = ix; ix = -iy; iy = t; }
  uint8_t flip = cfg_ ? cfg_->flow_flip : 0;
  if (flip & 1) ix = -ix;
  if (flip & 2) iy = -iy;
  // remove the apparent motion caused by rotation, scale by height:
  // ix = -vx/h - q,  iy = -vy/h + p
  float p = rateDps.x * DEG2RAD, q = rateDps.y * DEG2RAD;
  float h = st_.agl;
  float vx = -(ix + q) * h;
  float vy = -(iy - p) * h;
  float cy = cosf(yaw), sy = sinf(yaw);
  float vn = vx * cy - vy * sy;
  float ve = vx * sy + vy * cy;
  if (!hInit_) {
    kn_.init(0, vn, 0.1f, 0.5f, 0.05f);
    ke_.init(0, ve, 0.1f, 0.5f, 0.05f);
    hInit_ = true;
  }
  float r = 0.15f + 0.1f * h + (255 - f.quality) * 0.002f;
  flowVn_ = vn; flowVe_ = ve; flowSigma_ = r; flowNew_ = true;
  kn_.update(1, vn, r * r, 5.0f);
  ke_.update(1, ve, r * r, 5.0f);
  lastFlowMs_ = nowMs;
}

void NavEstimator::update(uint32_t nowMs) {
  nowMs_ = nowMs;
  gpsUsable_ = hInit_ && lastGpsMs_ && (nowMs - lastGpsMs_) < 1000;
  flowUsable_ = hInit_ && lastFlowMs_ && (nowMs - lastFlowMs_) < 300;
  if (lastRangeMs_ && nowMs - lastRangeMs_ > 300) { st_.aglValid = false; rangeUsed_ = false; }
  if (hInit_ && !gpsUsable_ && !flowUsable_ && (nowMs - lastGpsMs_) > 3000 && (nowMs - lastFlowMs_) > 3000) hInit_ = false;

  st_.altValid = zInit_ && ((nowMs - lastBaroMs_) < 1000 || (lastGpsHMs_ && nowMs - lastGpsHMs_ < 1000));
  st_.alt = kz_.pos();
  st_.vz = kz_.vel();
  st_.gpsValid = gpsUsable_;
  st_.gpsGlitch = glitch_ && gpsUsable_;
  st_.flowValid = flowUsable_;
  st_.posValid = gpsUsable_ || flowUsable_;
  st_.n = kn_.pos();
  st_.e = ke_.pos();
  st_.vn = kn_.vel();
  st_.ve = ke_.vel();
  if (!gpsUsable_) st_.eph = flowUsable_ ? 1.0f + 0.01f * (nowMs - lastGpsMs_) / 1000.0f : 99.0f;
}

void NavEstimator::setHomeHere() {
  if (zInit_) {
    float h = kz_.pos();
    baroOffset_ += h;
    kz_.shift(-h);
    rh_.T -= h;  // the ground moves with the height frame
    gpsHOff_ -= h;
  }
  if (gpsUsable_) rehomePending_ = true;
  else if (flowUsable_ && hInit_) {  // relative navigation: current point becomes origin
    kn_.shift(-kn_.pos());
    ke_.shift(-ke_.pos());
  }
}

}  // namespace fc
