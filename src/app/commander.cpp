#include "commander.h"

#include <string.h>

#include <stdio.h>
#include <stdlib.h>

#include "../comms/hitl.h"
#include "../hal/hal.h"

namespace app {

using namespace fc;

// ---------------------------------------------------------------- stick helpers
static float stick(uint16_t us) { return rcStickUs(us, bus::config.rc_deadband); }
static float thrStick(uint16_t us) { return constrainf(((float)us - 1000.0f) / 1000.0f, 0.0f, 1.0f); }
// throttle stick -> climb rate with a center dead-band (alt-hold style)
static float climbFromStick(float thr, float maxRate) {
  if (thr > 0.6f) return (thr - 0.6f) / 0.4f * maxRate;
  if (thr < 0.4f) return (thr - 0.4f) / 0.4f * maxRate;
  return 0.0f;
}
static bool isAltMode(FlightMode m) {
  return m == FlightMode::ALTHOLD || m == FlightMode::POSHOLD || m == FlightMode::RTH || m == FlightMode::MISSION ||
         m == FlightMode::LAND || m == FlightMode::CRUISE || m == FlightMode::AUTOTUNE || m == FlightMode::GUIDED ||
         m == FlightMode::SMART_RTL || m == FlightMode::BRAKE || m == FlightMode::CIRCLE || m == FlightMode::FOLLOW;
}
static bool isNavMode(FlightMode m) {
  return m == FlightMode::POSHOLD || m == FlightMode::RTH || m == FlightMode::MISSION || m == FlightMode::LAND ||
         m == FlightMode::CRUISE || m == FlightMode::GUIDED || m == FlightMode::SMART_RTL || m == FlightMode::BRAKE ||
         m == FlightMode::CIRCLE || m == FlightMode::FOLLOW;
}
static bool gcsSettable(uint32_t m) {
  return m < (uint32_t)FlightMode::TURTLE || m == (uint32_t)FlightMode::CRUISE || m == (uint32_t)FlightMode::GUIDED ||
         m == (uint32_t)FlightMode::SMART_RTL || m == (uint32_t)FlightMode::BRAKE || m == (uint32_t)FlightMode::CIRCLE ||
         m == (uint32_t)FlightMode::FOLLOW;
}
static FlightMode actionFor(uint8_t a) {  // failsafe / fence / battery action
  return a == (uint8_t)FailsafeAction::SMART_RTL ? FlightMode::SMART_RTL
       : a == (uint8_t)FailsafeAction::RTH ? FlightMode::RTH : FlightMode::LAND;
}

// ---------------------------------------------------------------- setup
void Commander::init() { reconfigure(); }

void Commander::reconfigure() {
  const Config& c = bus::config;
  est_.configure(c);
  nav_.configure(c);
  alt_.configure(c);
  pos_.configure(c);
  cfgGen_ = bus::configGeneration;
}

// ---------------------------------------------------------------- estimation
void Commander::fuseSensors(uint32_t now, float dt) {
  const Config& c = bus::config;
  // ---- heading source: compass, else GPS-learned correction of the gyro-only AHRS yaw ----
  bool useEkf = c.est_type == 1;
  bool magCal = c.mag_bias[0] != 0 || c.mag_bias[1] != 0 || c.mag_bias[2] != 0 || hitl::active();  // HITL field needs none
  bool magFault = useEkf && ekf_.initialized() && ekf_.magFault();
  bus::magRejected = magFault;
  bool magOk = magSeen_ && mag_.healthy && magCal && !magFault;
  useGpsHeading_ = !magOk && c.gps_heading;
  float prevCorr = yawCorr_;
  if (att_.valid) gpsRef_.predict(att_.accEarth.x, att_.accEarth.y, dt);  // referee, always running
  if (useGpsHeading_) {
    if (att_.valid) gpsHeading_.predict(att_.accEarth.x, att_.accEarth.y, dt);
    yawCorr_ = gpsHeading_.offset();
    headingOk_ = gpsHeading_.aligned();
    headingSrc_ = headingOk_ ? 2 : 0;
  } else {
    gpsHeading_.reset();
    yawCorr_ = 0;
    headingOk_ = magOk;
    headingSrc_ = magOk ? 1 : 0;
  }
  // EKF heading (compass and GPS-observed, fault checked) once it has converged
  // without a usable compass the EKF heading is trusted only after the GPS heading learner aligned
  // it (a large initial yaw error would make the linearised filter converge to a wrong heading)
  ekfYawOk_ = useEkf && ekf_.initialized() && ekf_.status().yawStdDeg < 8.0f &&
              (magOk || (useGpsHeading_ && gpsHeading_.aligned() && ekfGpsYawInit_));
  if (ekfYawOk_) {
    yawCorr_ = wrapPi(ekf_.yaw() - att_.yaw);
    headingOk_ = true;
    headingSrc_ = 3;
  }
  if (yawCorr_ != prevCorr) {
    yawHold_ = wrapPi(yawHold_ + (yawCorr_ - prevCorr));  // keep the physical heading
    if (fabsf(wrapPi(yawCorr_ - prevCorr)) > 20.0f * DEG2RAD) est_.resetHorizontal();
  }
  yawNav_ = wrapPi(att_.yaw + yawCorr_);
  float cc = cosf(yawCorr_), sc = sinf(yawCorr_);
  accNav_ = Vec3(att_.accEarth.x * cc - att_.accEarth.y * sc, att_.accEarth.x * sc + att_.accEarth.y * cc,
                 att_.accEarth.z);

  if (att_.valid) est_.predict(accNav_, dt, headingOk_ || !useGpsHeading_);
  BaroSample b;
  bool newBaro = false, newGps = false, newMag = false;
  uint32_t s = bus::baro.read(b);
  if (s != seqBaro_) {
    seqBaro_ = s;
    if (armed_ && baroSeen_ && !b.healthy)
      bus::postMessage(navEst_.gpsValid ? "Baro lost - height from GPS" : "Baro lost - no height hold");
    baroSeen_ = b.healthy;
    // baro corrections: speed from the last estimate (only with a position source), near the ground
    // when armed below 1 m (rangefinder height when it sees the ground, else above home)
    const NavEstimate& pe = navEst_;
    baroVh2_ = pe.posValid ? pe.vn * pe.vn + pe.ve * pe.ve : 0.0f;
    baroGround_ = armed_ && (pe.aglValid ? pe.agl : pe.alt) < 1.0f;
    est_.setBaroContext(baroVh2_, baroGround_);
    est_.fuseBaro(b);
    newBaro = true;
  }
  s = bus::mag.read(mag_);
  if (s != seqMag_) {
    bool was = magSeen_ && magHealthy_;
    seqMag_ = s;
    magSeen_ = true;
    newMag = true;
    magHealthy_ = mag_.healthy;
    if (armed_ && was && !mag_.healthy && !bus::magRejected && (int32_t)(bus::magCalUntilMs - now) <= 0)
      bus::postMessage("Compass lost - heading from GPS / gyro");
  }
  float cosTilt = att_.q.cosTilt();
  RangeSample r;
  s = bus::range.read(r);
  bool newRange = false;
  if (s != seqRange_) { seqRange_ = s; est_.fuseRange(r, cosTilt, now); newRange = true; }
  GpsData g;
  s = bus::gps.read(g);
  if (s != seqGps_) {
    seqGps_ = s;
    newGps = true;
    est_.fuseGps(g, now);
    if (useGpsHeading_ && g.fix3d && g.sats >= c.gps_min_sats) gpsHeading_.fuseGps(g.velN, g.velE, now);
    if (g.fix3d && g.sats >= c.gps_min_sats) gpsRef_.fuseGps(g.velN, g.velE, now);
    if (g.fix3d && g.sats >= c.gps_min_sats) {
      uint32_t dtMs = now - gpsVelPrevMs_;
      if (gpsVelPrevMs_ && dtMs > 30 && dtMs < 500) {
        float gdt = dtMs * 1e-3f, k = gdt / (0.25f + gdt);
        // GPS velocity is true NED; rotate into the AHRS frame (heading correction) for the leveling
        float aN = (g.velN - gpsVelPrev_[0]) / gdt, aE = (g.velE - gpsVelPrev_[1]) / gdt;
        float cy = cosf(-yawCorr_), sy = sinf(-yawCorr_);
        accKin_.x += (aN * cy - aE * sy - accKin_.x) * k;
        accKin_.y += (aN * sy + aE * cy - accKin_.y) * k;
        accKinMs_ = now;
      }
      gpsVelPrev_[0] = g.velN;
      gpsVelPrev_[1] = g.velE;
      gpsVelPrevMs_ = now;
    }
  }
  FlowSample f;
  s = bus::flow.read(f);
  if (s != seqFlow_) { seqFlow_ = s; est_.fuseFlow(f, att_.rateDps, yawNav_, cosTilt, now); }
  est_.update(now);
  magLearnStep(now, dt, newMag);
  if (useEkf) ekfFuse(now, newGps ? &g : nullptr, newBaro ? &b : nullptr, newMag, newRange ? &r : nullptr);
  else ekf_.reset();
  buildNavEstimate(now);
  if (navEst_.altValid) altValidMs_ = now;
  const NavEstimate& e = navEst_;
  wind_.update(att_.q, yawCorr_, att_.accEarth, e.vn, e.ve, c.wind_drag, airborne_ && armed_ && e.posValid, dt);
  if (e.glitchCount != lastGlitchCount_) {
    lastGlitchCount_ = e.glitchCount;
    bus::postMessage("GPS glitch: position held on inertial");
  }
}

// ---------------------------------------------------------------- navigation EKF
void Commander::ekfFuse(uint32_t now, const GpsData* g, const BaroSample* b, bool newMag, const RangeSample* rng) {
  const Config& c = bus::config;
  const NavEstimate& est = est_.state();
  if (!att_.valid || !att_.gyroCalibrated) return;
  if (!ekf_.initialized()) {
    // start from the AHRS attitude, rotated onto the navigation heading
    Quat q = Quat::fromEuler(0, 0, yawCorr_) * att_.q;
    ekf_.init(q, est.altValid ? est.alt : 0.0f);
    seqAtt_ = 0;
    ekfHomeKnown_ = false;
    return;
  }
  bool magCal = c.mag_bias[0] != 0 || c.mag_bias[1] != 0 || c.mag_bias[2] != 0 || hitl::active();
  bool magUsable = magSeen_ && mag_.healthy && magCal && !ekf_.magFault();
  bool headingKnown = magUsable || (useGpsHeading_ && gpsHeading_.aligned());
  if (!headingKnown) {
    ekf_.slaveAttitude(Quat::fromEuler(0, 0, yawCorr_) * att_.q, squaref(0.05f), 1.0f);
    ekfGpsYawInit_ = false;
  }
  if (imuNew_ && imuDt_ > 0 && imuDt_ < 0.1f) ekf_.predict(imuDAng_, imuDVel_, imuDt_, now);
#ifdef ALLQUAD_HOST
  if (bus::simEkfYawKickDeg) { ekf_.setYaw(ekf_.yaw() + bus::simEkfYawKickDeg * DEG2RAD); bus::simEkfYawKickDeg = 0; }
#endif
  // ---- tilt cross-check against the AHRS: the complementary filter is the robust one (gravity
  // always pulls it back); an EKF tilt that stays far from it (after a tumble, saturated rates,
  // a long acceleration) has diverged - its tilt is reset to the AHRS one, heading and
  // position / velocity kept. Without this a wrong tilt turns into a wrong acceleration and the
  // position estimate runs away while the filter's own covariance still looks confident.
  // While the craft is steady (hovering: no acceleration, slow rates for 2 s) the AHRS tilt is
  // accurate to a degree or two, so the limit drops from 15 to 4 deg: that catches the quieter trap
  // where a tilt error and a wrong accelerometer bias explain each other.
  bool steady = att_.valid && att_.accEarth.norm() < 0.6f && fabsf(att_.rateDps.x) < 15 && fabsf(att_.rateDps.y) < 15 &&
                fabsf(att_.rateDps.z) < 30;
  if (!steady) steadySince_ = 0;
  else if (!steadySince_) steadySince_ = now ? now : 1;
  bool calm = steadySince_ && now - steadySince_ > 2000;
  if (headingKnown && att_.valid) {
    // gravity direction in the body frame: independent of heading (EKF and AHRS headings differ)
    Vec3 zk = ekf_.q().rotateInv(Vec3(0, 0, 1)), za = att_.q.rotateInv(Vec3(0, 0, 1));
    float dotz = zk.x * za.x + zk.y * za.y + zk.z * za.z;
    if (dotz < cosf((calm ? 4.0f : 15.0f) * DEG2RAD)) {
      if (!ekfTiltBadSince_) ekfTiltBadSince_ = now ? now : 1;
      if (now - ekfTiltBadSince_ > (calm ? 3000u : 1000u)) {
        float yawK = ekf_.yaw();
        ekf_.slaveAttitude(Quat::fromEuler(0, 0, wrapPi(yawK - att_.yaw)) * att_.q, squaref(0.05f), ekf_.var(8));
        ekf_.relearnAccBias(0.3f);
        ekfTiltBadSince_ = 0;
        ekfTiltResets_++;
        bus::postMessage("EKF: attitude disagreed with the AHRS - tilt reset");
      }
    } else ekfTiltBadSince_ = 0;
    // calm hover: a weak pull towards the AHRS tilt (the attitude the craft is flown on anyway)
    if (calm && now - ekfTiltFuseMs_ >= 100) ekfTiltFuseMs_ = now, ekf_.fuseTilt(Quat::fromEuler(0, 0, wrapPi(ekf_.yaw() - att_.yaw)) * att_.q, 2.0f * DEG2RAD);
  } else ekfTiltBadSince_ = 0;
  // ---- height: barometer, GPS when the baro is missing or faulty ----
  if (b && b->healthy) {
    float alt = b->altM - est_.baroOffset(), r = c.est_baro_noise;
    baroCorrect(alt, r, ekf_.alt(), baroVh2_, c.baro_dyn, baroGround_ && c.baro_gnd);
    ekf_.fuseHeight(alt, r, 0, now);
  }
  // ---- rangefinder height (below rf_max_m, craft not tilted much) ----
  if (rng) {
    float ct = att_.q.cosTilt();
    ekfRangeUsed_ = false;
    if (rng->valid && rng->distM > 0.02f && rng->distM <= c.rf_max_m && ct >= 0.86f) {
      float agl = rng->distM * ct, z;
      if (ekfRange_.measure(agl, ekf_.alt(), sqrtf(ekf_.posVarDown()), now, z)) {
        ekf_.fuseHeight(z, RangeHeight::sigma(agl), 2, now);
        ekfRangeUsed_ = true;
      }
    }
  }
  bool gpsGood = g && g->fix3d && g->sats >= c.gps_min_sats && g->hAcc < c.gps_max_hacc;
  if (gpsGood && est.homeSet) {
    // the local frame follows the home position of the per-axis estimator (rehomed when arming)
    if (ekfHomeKnown_ && (ekfHomeLat_ != est.homeLat || ekfHomeLon_ != est.homeLon)) {
      float dn, de;
      NavEstimator::llaToNed(ekfHomeLat_, ekfHomeLon_, est.homeLat, est.homeLon, dn, de);
      ekf_.shift(dn, de, 0);
    }
    ekfHomeLat_ = est.homeLat;
    ekfHomeLon_ = est.homeLon;
    ekfHomeKnown_ = true;
    float n, e;
    NavEstimator::llaToNed(g->lat, g->lon, est.homeLat, est.homeLon, n, e);
    ekf_.fuseGpsPos(n, e, fmaxf(g->hAcc, 0.8f), now);
    // speed accuracy reported by the receiver (u-blox sAcc): poor sky / multipath counts less
    ekf_.fuseGpsVel(g->velN, g->velE, g->velD, fmaxf(c.est_gps_vel_noise, g->sAcc), now);
    bool baroBad = !seqBaro_ || !baroSeen_ || ekf_.baroFault() || !ekf_.heightOk(now);
    if (baroBad) ekf_.fuseHeight(g->altMsl - est.homeAltMsl, fmaxf(g->vAcc, 1.5f), 1, now);
  }
  // ---- heading ----
  if (newMag && magSeen_ && mag_.healthy && magCal) {
    float decl = c.mag_declination_deg * DEG2RAD;
    Vec3 v = ekf_.vel();
    bool observable = ekf_.gpsUsed(now) && (v.x * v.x + v.y * v.y > 9.0f || accNav_.x * accNav_.x + accNav_.y * accNav_.y > 4.0f);
    ekf_.checkMag(mag_.field, decl, observable, now);
    if (!ekf_.magFault()) {
      Vec3 mE = ekf_.q().rotate(mag_.field);
      if (mE.x * mE.x + mE.y * mE.y > 1e-6f)
        ekf_.fuseYaw(wrapPi(ekf_.yaw() - wrapPi(atan2f(mE.y, mE.x) - decl)), 0.1f);
    }
  }
  // ---- GPS referee for the EKF heading while a compass is fitted: the heading learned from GPS
  // accelerations (independent of compass and EKF) says who is wrong when they disagree. Compass
  // agrees with it, EKF 30 deg off for 2 s: the EKF heading diverged (rejecting a good compass, or
  // about to blame it with a compass fault) -> heading reset to the compass, compass fault cleared.
  // The other way round (compass wrong) is the EKF's own compass fault check.
  bool compassFitted = magSeen_ && mag_.healthy && magCal;
  if (compassFitted && gpsRef_.aligned() && now - gpsRef_.lastGoodMs() < 5000) {
    float refYaw = wrapPi(att_.yaw + gpsRef_.offset());
    bool compassAgrees = fabsf(gpsRef_.offset()) < 15.0f * DEG2RAD;
    if (compassAgrees && fabsf(wrapPi(ekf_.yaw() - refYaw)) > 30.0f * DEG2RAD) {
      if (!ekfYawBadSince_) ekfYawBadSince_ = now ? now : 1;
      if (now - ekfYawBadSince_ > 2000) {
        ekf_.setYaw(att_.yaw);  // compass heading (the AHRS follows the compass)
        ekf_.relearnAccBias(0.3f);
        ekf_.clearMagFault();
        ekfYawBadSince_ = 0;
        ekfYawResets_++;
        bus::postMessage("EKF: heading disagreed with compass + GPS - reset");
      }
    } else ekfYawBadSince_ = 0;
  } else ekfYawBadSince_ = 0;
  if (!magUsable && headingKnown) {  // GPS-learned heading (the attitude was slaved until now)
    ekfGpsYawInit_ = true;
    if (g) ekf_.fuseYaw(wrapPi(att_.yaw + gpsHeading_.offset()), 0.15f);
  }
  dragStep(now);
  // ---- optical flow ----
  float fvn, fve, fs;
  if (est_.takeFlowVel(fvn, fve, fs)) ekf_.fuseFlowVel(fvn, fve, fs, now);
  // ---- no horizontal aiding: hold the velocity near zero (on the ground firmly, in flight loosely)
  if (!ekf_.horizAided(now) && now - ekfZeroVelMs_ >= 100) {
    ekfZeroVelMs_ = now;
    ekf_.fuseZeroVel(armed_ && airborne_ ? 3.0f : 0.3f);
  }
  // ---- fault messages ----
  uint16_t f = ekf_.status().faults, rise = f & ~ekfFaultsPrev_;
  if (rise & EKF_BARO_FAULT) bus::postMessage("EKF: baro inconsistent - height from GPS");
  if (rise & EKF_MAG_FAULT) bus::postMessage("EKF: compass inconsistent - heading from GPS");
  if (rise & EKF_RESET) bus::postMessage("EKF: numerical reset");
  ekfFaultsPrev_ = f;
}

void Commander::buildNavEstimate(uint32_t now) {
  navEst_ = est_.state();
  if (bus::config.est_type != 1 || !ekf_.initialized()) return;
  Vec3 p = ekf_.pos(), v = ekf_.vel();
  const EkfStatus& st = ekf_.status();
  // dead reckoning: from the first moment without GPS (the drag fusion takes over within 0.1 s, the EKF
  // stays aided meanwhile) until dr_time_s
  bool dr = drActive_ || (bus::config.dr_time_s && drag_.ready() && armed_ && airborne_ &&
                          (!gpsLostMs_ || now - gpsLostMs_ < bus::config.dr_time_s * 1000u));
  if (ekf_.horizAided(now) && (navEst_.posValid || ekf_.gpsUsed(now) || dr)) {
    navEst_.n = p.x;
    navEst_.e = p.y;
    navEst_.vn = v.x;
    navEst_.ve = v.y;
    navEst_.posValid = true;
    navEst_.eph = st.posStd;
    navEst_.gpsGlitch = (st.faults & EKF_GPS_GLITCH) != 0;
  }
  if (ekf_.heightOk(now)) {
    navEst_.alt = ekf_.alt();
    navEst_.vz = -v.z;
    navEst_.altValid = true;
  }
}

// ---------------------------------------------------------------- requests & arming
void Commander::handleRequests(uint32_t now) {
  const Config& c = bus::config;
  bus::Request r;
  uint32_t arg;
  if (!bus::takeRequest(r, arg)) return;
  switch (r) {
    case bus::Request::ARM:
      if (armed_) break;
      if (!c.gcs_arm) bus::postMessage("Arm: GCS arming disabled (gcs_arm)");
      else if (blockers_ & ~ARM_SWITCH_AT_BOOT) bus::postMessage("Arm: pre-arm checks failed");
      else arm(now, false);
      break;
    case bus::Request::DISARM:
      if (armed_) disarm("GCS");
      break;
    case bus::Request::SET_MODE:
      if (gcsSettable(arg)) { gcsMode_ = (FlightMode)arg; gcsSwitchState_ = switchState_; }
      break;
    case bus::Request::ACC_CAL:
      if (armed_) break;
      if (arg == 6) {
        acc6_ = true;
        acc6StartMs_ = now;
        acc6Side_ = -1;
        acc6Count_ = 0;
        bus::acc6Mask = 0;
        bus::acc6Active = true;
        bus::postMessage("Accel 6-point: place on each side, hold still");
      } else {
        accCalCount_ = 0; accCalSum_ = Vec3(); bus::postMessage("Accel calibration: keep level");
      }
      break;
    case bus::Request::MAG_CAL:
      if (!armed_) { bus::magCalUntilMs = now + (arg ? arg : 30) * 1000; bus::postMessage("Compass calibration: rotate"); }
      break;
    case bus::Request::GYRO_CAL:
      if (!armed_) bus::gyroCalRequest = true;
      break;
    case bus::Request::COMPASSMOT:
      if (!armed_ && bus::compassMotPct == 255) {
        bus::compassMotPct = 0;
        bus::postMessage("compassmot: PROPS ON, hold the craft down - motors ramp to 60 %");
      }
      break;
    default:
      break;
  }
}

uint32_t Commander::computeArmBlockers(bool rcOk) {
  const Config& c = cfg();
  const NavEstimate& e = navEst_;
  uint32_t b = 0;
  uint32_t now = os::timeMs();
  if (!rcOk) b |= ARM_NO_RX;
  if (rcOk && rc_.ch[RC_THROTTLE] > c.arm_throttle_max_us) b |= ARM_THROTTLE;
  if (fabsf(att_.roll) * RAD2DEG > c.arm_max_angle || fabsf(att_.pitch) * RAD2DEG > c.arm_max_angle) b |= ARM_ANGLE;
  if (!armSwSeenLow_) b |= ARM_SWITCH_AT_BOOT;
  if (!att_.gyroCalibrated || accCalCount_ >= 0 || acc6_ || (int32_t)(bus::magCalUntilMs - now) > 0 || hal::logBusy() ||
      bus::otaActive || bus::compassMotPct != 255)
    b |= ARM_CALIBRATING;  // (log erase stalls the CPU for ~45 ms per sector: only on the ground)
  if (bat_.critical || bat_.capacityLow) b |= ARM_BATTERY;
  if (!att_.valid) b |= ARM_NO_IMU;
  if ((int32_t)(bus::cliActiveUntilMs - now) > 0 || bus::osdMenuOpen) b |= ARM_CLI;  // CLI / configurator / OSD menu
  if (rcFailsafe_) b |= ARM_FAILSAFE;
  if (!c.acc_calibrated) b |= ARM_ACC_NOT_CAL;
  if (seqBaro_ && !baroSeen_) b |= ARM_BARO;
  bool magCal = c.mag_bias[0] != 0 || c.mag_bias[1] != 0 || c.mag_bias[2] != 0 || hitl::active();
  // an uncalibrated compass blocks arming, unless it is to be learned in flight (mag_learn): then the
  // heading comes from GPS until the turns have taught the offsets
  if (magSeen_ && (!mag_.healthy || (!magCal && !c.mag_learn))) b |= ARM_MAG;
  if (c.arm_require_gps && !(e.gpsValid && e.homeSet)) b |= ARM_GPS;
  if (fenceBreached(c, e)) b |= ARM_FENCE;
  if (os::taskCount() > 0) {
    os::TaskStats ts = os::taskStats(0);
    if (ts.runs > 1000 && ts.lastUs > ts.periodUs * 9 / 10) b |= ARM_CPU_LOAD;
  }
  FlightMode want = pilotMode(rcOk);
  if (want == FlightMode::RTH || want == FlightMode::LAND || want == FlightMode::SMART_RTL) b |= ARM_NAV_MODE;
  if (logicNoArm_) b |= ARM_LOGIC;
  if (bus::imuActive == 1) b |= ARM_IMU_FAIL;  // 2 = picked on the ground (ImuVote): no block
  if (bus::imuMismatch) b |= ARM_IMU_MISMATCH;
  return b;
}

void Commander::arm(uint32_t now, bool turtle) {
  armed_ = true;
  turtle_ = turtle;
  airborne_ = false;
  armTimeMs_ = now;
  est_.setHomeHere();
  if (ekf_.initialized()) ekf_.shift(0, 0, -ekf_.alt());  // height zero at the take-off point
  launched_ = false;
  clipTold_ = false;
  clipsFlight_ = 0;
  rthBestDist_ = 1e9f;
  rescueLanding_ = false;
  land_.reset();
  rcFailsafe_ = false;
  actionMode_ = FlightMode::COUNT;
  batFsDone_ = false;
  gcsFsDone_ = false;
  crashSinceMs_ = impactMs_ = 0;
  nav_.path().reset();
  pathMs_ = 0;
  headfreeRef_ = yawNav_;  // head-free "forward": where the nose points at arming
  magLearn_.reset();
  mlYaw_ = 0;
  mlDone_ = mlPending_ = false;
  if (magSeen_ && bus::config.mag_learn && bus::config.mag_bias[0] == 0 && bus::config.mag_bias[1] == 0 &&
      bus::config.mag_bias[2] == 0)
    bus::postMessage("Compass not calibrated: fly some turns to learn it");
  flipActive_ = false;
  smartDone_ = smartWarn_ = false;
  smartLowN_ = 0;
  ampsAvg_ = -1;
  pctRefMs_ = 0;
  pctRate_ = 0;
  motorBadSince_ = thrustBadSince_ = 0;
  motorWarn_ = thrustWarn_ = false;
  for (float& m : motorAvg_) m = 0;
  mode_ = FlightMode::COUNT;  // force mode entry
  yawHold_ = yawNav_;
  wind_.reset();
  if (hoverSamples_ < 250 || !bus::config.hover_learn) hoverLearn_ = bus::config.hover_throttle;  // keep what this session learned
  hoverSamples_ = 0;
  alt_.setHover(hoverLearn_);
  aglHold_ = false;
  bus::postMessage(turtle ? "Armed (turtle)" : "Armed");
}

void Commander::disarm(const char* why) {
  if (armed_) saveHover();
  if (armed_) magLearnSave();
  if (autotune_.active()) finishAutotune(autotune_.done() && boxOn(boxes_, ModeBox::AUTOTUNE));
  armed_ = false;
  turtle_ = false;
  airborne_ = false;
  rcFailsafe_ = false;
  actionMode_ = FlightMode::COUNT;
  lastThrottle_ = 0;
  launched_ = false;
  launchWaiting_ = false;
  char m[48];
  snprintf(m, sizeof(m), "Disarmed: %s", why);
  bus::postMessage(m);
}

// ---------------------------------------------------------------- modes
FlightMode Commander::pilotMode(bool rcOk) const {
  const Config& c = cfg();
  if (!rcOk) return mode_ == FlightMode::COUNT ? FlightMode::ANGLE : mode_;
  const uint32_t b = boxes_;
  if (boxOn(b, ModeBox::LAND)) return FlightMode::LAND;
  if (boxOn(b, ModeBox::SMART_RTL)) return FlightMode::SMART_RTL;
  if (boxOn(b, ModeBox::RTH) || boxOn(b, ModeBox::GPS_RESCUE)) return FlightMode::RTH;
  if (boxOn(b, ModeBox::MISSION)) return FlightMode::MISSION;
  if (boxOn(b, ModeBox::BRAKE)) return FlightMode::BRAKE;
  if (boxOn(b, ModeBox::FOLLOW)) return FlightMode::FOLLOW;
  if (boxOn(b, ModeBox::CIRCLE)) return FlightMode::CIRCLE;
  if (flipActive_) return FlightMode::FLIP;
  if (boxOn(b, ModeBox::AUTOTUNE)) return FlightMode::AUTOTUNE;
  if (boxOn(b, ModeBox::CRUISE)) return FlightMode::CRUISE;
  if (boxOn(b, ModeBox::POSHOLD)) return FlightMode::POSHOLD;
  if (boxOn(b, ModeBox::ALTHOLD)) return FlightMode::ALTHOLD;
  if (c.beginner) return FlightMode::ANGLE;  // first-flight mode: ACRO / HORIZON fly as ANGLE
  if (boxOn(b, ModeBox::HORIZON)) return FlightMode::HORIZON;
  if (boxOn(b, ModeBox::ANGLE)) return FlightMode::ANGLE;
  return FlightMode::ACRO;
}

FlightMode Commander::validate(FlightMode m) const {
  const Config& c = cfg();
  const NavEstimate& e = navEst_;
  (void)c;
  bool altOk = e.altValid;
  // absolute (GPS) navigation needs a heading; optical-flow navigation is relative and does not
  bool posOk = e.posValid && altOk && (headingOk_ || (e.flowValid && !e.gpsValid));
  bool gpsNav = e.gpsValid && e.homeSet && altOk && headingOk_;
  bool rthOk = e.gpsValid && e.homeSet && altOk && (headingOk_ || useGpsHeading_);  // RTH can align heading
  bool forced = rcFailsafe_ || actionMode_ != FlightMode::COUNT;
  switch (m) {
    case FlightMode::CRUISE:
      if (posOk) return m;
      return altOk ? FlightMode::ALTHOLD : FlightMode::ANGLE;
    case FlightMode::GUIDED:
      if (gpsNav) return m;
      m = FlightMode::POSHOLD;
      if (posOk) return m;
      return altOk ? FlightMode::ALTHOLD : FlightMode::ANGLE;
    case FlightMode::FOLLOW:
      if (gpsNav) return m;
      m = FlightMode::POSHOLD;
      if (posOk) return m;
      return altOk ? FlightMode::ALTHOLD : FlightMode::ANGLE;
    case FlightMode::BRAKE:
    case FlightMode::CIRCLE:
      if (posOk) return m;
      return altOk ? FlightMode::ALTHOLD : FlightMode::ANGLE;
    case FlightMode::SMART_RTL:
      if (rthOk) return m;
      return altOk ? FlightMode::LAND : (forced ? FlightMode::FAILSAFE : FlightMode::ANGLE);
    case FlightMode::MISSION:
      if (gpsNav && nav::count() > 0) return m;
      m = FlightMode::POSHOLD;
      // fall through
    case FlightMode::POSHOLD:
      if (posOk) return m;
      return altOk ? FlightMode::ALTHOLD : FlightMode::ANGLE;
    case FlightMode::RTH:
      if (rthOk) return m;
      if (altOk) return FlightMode::LAND;
      return forced ? FlightMode::FAILSAFE : FlightMode::ANGLE;
    case FlightMode::LAND:  // asked for a landing: without a height the inertial descent still lands it
      return altOk ? m : FlightMode::FAILSAFE;
    case FlightMode::ALTHOLD:
    case FlightMode::AUTOTUNE:  // twitches from an altitude-held hover
      return altOk ? m : FlightMode::ANGLE;
    default:
      return m;
  }
}

void Commander::updateFailsafes(bool rcOk, uint32_t now) {
  const Config& c = cfg();
  if (!armed_) {
    rcFailsafe_ = false;
    actionMode_ = FlightMode::COUNT;
    return;
  }
  // FAILSAFE (descent without a height): disarmed fs_off_delay after it began, however it was asked for
  if (mode_ != FlightMode::FAILSAFE) fsModeMs_ = now;
  else if (!rcFailsafe_ && now - fsModeMs_ > c.fs_off_delay_ms) { disarm("failsafe timeout"); return; }
  // ---- RC link ----
  if (!rcOk && !rcFailsafe_) {
    if (!airborne_) { disarm("RC lost on ground"); return; }
    if (c.fs_action == (uint8_t)FailsafeAction::DROP) { disarm("failsafe (drop)"); return; }
    rcFailsafe_ = true;
    failsafeStartMs_ = now;
    rcRecoverMs_ = 0;
    bus::postMessage("Failsafe: RC lost");
  }
  if (rcFailsafe_) {
    if (rcOk) {
      if (!rcRecoverMs_) rcRecoverMs_ = now;
      if (now - rcRecoverMs_ > 1000) { rcFailsafe_ = false; bus::postMessage("RC recovered"); }
    } else {
      rcRecoverMs_ = 0;
    }
    if (mode_ == FlightMode::FAILSAFE && now - fsModeMs_ > c.fs_off_delay_ms) { disarm("failsafe timeout"); return; }
  }
  // ---- battery ----
  if (!batFsDone_ && c.bat_fs_action && airborne_ && (bat_.critical || bat_.capacityLow)) {
    batFsDone_ = true;
    actionMode_ = actionFor(c.bat_fs_action);
    actionSwitchState_ = switchState_;
    bus::postMessage(c.bat_fs_action == 3 ? "Battery failsafe: SmartRTL"
                     : c.bat_fs_action == 2 ? "Battery failsafe: RTH" : "Battery failsafe: landing");
  }
  smartRth(now);
  // ---- GCS link (ArduPilot FS_GCS): only for modes that depend on the ground station ----
  bool gcsMode = mode_ == FlightMode::GUIDED || mode_ == FlightMode::MISSION;
  uint32_t hb = bus::gcsHeartbeatMs;
  if (c.gcs_fs && airborne_ && gcsMode && hb && !gcsFsDone_ && actionMode_ == FlightMode::COUNT &&
      now - hb > (uint32_t)(c.gcs_fs_s * 1000.0f)) {
    gcsFsDone_ = true;
    actionMode_ = actionFor(c.gcs_fs == 1 ? 2 : c.gcs_fs == 3 ? 3 : 1);
    actionSwitchState_ = switchState_;
    bus::postMessage("Failsafe: GCS lost");
  }
  // ---- geofence ----
  if (airborne_ && actionMode_ == FlightMode::COUNT && (int32_t)(now - fenceSuppressUntilMs_) > 0 &&
      mode_ != FlightMode::RTH && mode_ != FlightMode::LAND && fenceBreached(c, navEst_)) {
    actionMode_ = c.fence_action == (uint8_t)FailsafeAction::LAND ? FlightMode::LAND : FlightMode::RTH;  // never SmartRTL (the path is inside the breach)
    actionSwitchState_ = switchState_;
    bus::postMessage("Geofence breach");
  }
  // pilot takes over a latched action by moving a mode switch
  if (actionMode_ != FlightMode::COUNT && rcOk && switchState_ != actionSwitchState_) {
    actionMode_ = FlightMode::COUNT;
    fenceSuppressUntilMs_ = now + 10000;
    bus::postMessage("Pilot override");
  }
}

// ---------------------------------------------------------------- main cycle
void Commander::run(uint32_t now, float dt) {
  if (cfgGen_ != bus::configGeneration) reconfigure();
  if (bus::config.beginner) {  // first-flight limits on a copy: the stored settings stay as they are
    eff_ = bus::config;
    beginnerLimits(eff_);
  }
  Config& c = bus::config;
  bus::rc.read(rc_);
  uint32_t sa = bus::attitude.read(att_);
  attNew_ = sa != seqAtt_;
  seqAtt_ = sa;
  imuNew_ = bus::takeImuDelta(imuDAng_, imuDVel_, imuDt_);
  bus::battery.read(bat_);
  fuseSensors(now, dt);

  bool rcOk = rc_.everReceived && !rc_.failsafeFlag && ageMs(now, rc_.lastFrameMs) < c.fs_delay_ms;
  // logic conditions: RC overrides go in before anything reads the sticks / switches
  for (int i = 0; i < 16 && i < RC_MAX_CH; i++) if (logicRc_[i]) rc_.ch[i] = logicRc_[i];
  // PID / rate profile from its switch (low / mid / high), else the stored choice
  uint8_t prof = c.pid_profile > 2 ? 0 : c.pid_profile;
  if (c.prof_ch >= 5 && c.prof_ch <= RC_MAX_CH && rcOk) {
    uint16_t us = rc_.ch[c.prof_ch - 1];
    prof = us < 1300 ? 0 : us < 1700 ? 1 : 2;
  }
  if (prof != profile_) {
    profile_ = prof;
    char m[24];
    snprintf(m, sizeof(m), "PID profile %d", prof + 1);
    bus::postMessage(m);
  }
  bus::pidProfile = profile_;
  if (rcOk) {
    boxes_ = modeBoxes(c, rc_.ch, RC_MAX_CH) | logicBoxes_;
    // flight-mode boxes only: moving any mode switch hands control back to the pilot
    switchState_ = boxes_ & ~((1u << (int)ModeBox::ARM) | (1u << (int)ModeBox::BEEPER) | (1u << (int)ModeBox::TURTLE));
  }
  if (gcsMode_ != FlightMode::COUNT && switchState_ != gcsSwitchState_) gcsMode_ = FlightMode::COUNT;
  handleRequests(now);
  guidedInput(now);
  followInput(now);
  flipTrigger(now);
  if (now - logicMs_ >= 20) { logicMs_ = now; logicUpdate(now); }
  if (acc6_) acc6Update(now);

  // ---- accelerometer calibration (level, still) ----
  if (accCalCount_ >= 0) {
    accCalSum_ += Vec3(att_.accBody.x / c.acc_scale[0], att_.accBody.y / c.acc_scale[1], att_.accBody.z / c.acc_scale[2]) +
                  Vec3(c.acc_bias[0], c.acc_bias[1], c.acc_bias[2]);
    if (++accCalCount_ >= 250) {
      Vec3 avg = accCalSum_ * (1.0f / 250.0f);  // raw (bias added back; the scale stays)
      c.acc_bias[0] = avg.x; c.acc_bias[1] = avg.y; c.acc_bias[2] = avg.z + GRAVITY / c.acc_scale[2];
      c.acc_calibrated = 1;
      c.level_trim_roll_deg = c.level_trim_pitch_deg = 0;
      accCalCount_ = -1;
      bus::configChanged();
      bus::postMessage("Accel calibrated - save to keep");
    }
  }

  // ---- arming ----
  blockers_ = computeArmBlockers(rcOk);
  bool sw = rcOk && boxOn(boxes_, ModeBox::ARM);
  if (rcOk && !sw) armSwSeenLow_ = true;
  if (rcOk) {
    if (sw && !armSwPrev_ && !armed_) {
      bool turtle = boxOn(boxes_, ModeBox::TURTLE) && c.motor_protocol >= (uint8_t)MotorProtocol::DSHOT300;
      uint32_t relevant = turtle ? (blockers_ & ~(uint32_t)ARM_ANGLE) : blockers_;
      if (!relevant) {
        arm(now, turtle);
      } else {
        for (int i = 0; i < ARM_COUNT_BITS; i++)
          if (relevant & (1u << i)) {
            char m[48];
            snprintf(m, sizeof(m), "Arming blocked: %s", armCheckName(i));
            bus::postMessage(m);
            break;
          }
      }
    } else if (!sw && armSwPrev_ && armed_) {
      disarm("switch");
    }
    armSwPrev_ = sw;
  }
  updateFailsafes(rcOk, now);
  crashCheck(now);

  // ---- SmartRTL breadcrumbs (5 Hz) ----
  if (armed_ && airborne_ && navEst_.gpsValid && navEst_.posValid && navEst_.homeSet && mode_ != FlightMode::SMART_RTL &&
      now - pathMs_ >= 200) {
    pathMs_ = now;
    nav_.path().record(navEst_.n, navEst_.e, navEst_.alt, c.srtl_point_m);
  }

  // ---- flight mode ----
  FlightMode want;
  if (turtle_) want = FlightMode::TURTLE;
  else if (rcFailsafe_) want = (c.fs_action == (uint8_t)FailsafeAction::RTH || c.fs_action == (uint8_t)FailsafeAction::SMART_RTL) &&
                                 !rescueLanding_ ? actionFor(c.fs_action) : FlightMode::LAND;
  else if (actionMode_ != FlightMode::COUNT) want = actionMode_;
  else if (gcsMode_ != FlightMode::COUNT) want = gcsMode_;
  else want = pilotMode(rcOk);
  FlightMode asked = want;
  want = validate(want);
  // a mode the sensors cannot fly (GPS / height lost): tell the pilot once per change
  if (armed_ && want != asked && (asked != downAsked_ || want != downGot_)) {
    char m[48];
    snprintf(m, sizeof(m), "%s unavailable: %s", flightModeName(asked), flightModeName(want));
    bus::postMessage(m);
  }
  downAsked_ = want != asked ? asked : FlightMode::COUNT;
  downGot_ = want;
  if (want != mode_) {
    FlightMode old = mode_;
    mode_ = want;
    prevMode_ = old;
    const NavEstimate& e = navEst_;
    if (isAltMode(mode_) && (old == FlightMode::COUNT || !isAltMode(old))) {
      alt_.reset(lastThrottle_, att_.q.cosTilt());
      altTarget_ = e.alt;
    }
    if (isNavMode(mode_)) {
      if (old == FlightMode::COUNT || !isNavMode(old)) pos_.reset();
      nav_.begin(mode_, e, yawNav_, headingOk_);
    }
    if (isAltMode(mode_)) aglHold_ = false;
    yawHold_ = yawNav_;
    if (mode_ == FlightMode::AUTOTUNE && !autotune_.active()) {
      autotune_.begin(bus::config);
      bus::postMessage("Autotune: hands off, twitching roll");
    } else if (old == FlightMode::AUTOTUNE && autotune_.active() && !autotune_.done()) {
      finishAutotune(false);  // left before the end: the old gains come back
    }
  }

  control(dt, now);
  motorHealth(now, dt);
  publish(now);
}

void Commander::control(float dt, uint32_t now) {
  const Config& c = cfg();
  ControlSetpoint sp{};
  if (!armed_) {
    sp.motorMode = MotorMode::STOPPED;
    lastThrottle_ = 0;
    bus::setpoint.publish(sp);
    return;
  }
  if (mode_ == FlightMode::TURTLE) {
    sp.motorMode = MotorMode::TURTLE;
    sp.turtleStick[0] = stick(rc_.ch[RC_ROLL]);
    sp.turtleStick[1] = stick(rc_.ch[RC_PITCH]);
    bus::setpoint.publish(sp);
    return;
  }

  const NavEstimate& e = navEst_;
  bool autonomous = mode_ == FlightMode::RTH || mode_ == FlightMode::MISSION || mode_ == FlightMode::LAND ||
                    mode_ == FlightMode::FAILSAFE || mode_ == FlightMode::GUIDED || mode_ == FlightMode::SMART_RTL ||
                    mode_ == FlightMode::BRAKE || mode_ == FlightMode::FOLLOW || rcFailsafe_;
  float rs = autonomous ? 0 : stick(rc_.ch[RC_ROLL]);
  float ps = autonomous ? 0 : stick(rc_.ch[RC_PITCH]);
  float ys = autonomous ? 0 : stick(rc_.ch[RC_YAW]);
  float thr = autonomous ? 0 : thrStick(rc_.ch[RC_THROTTLE]);
  // head-free: the roll / pitch sticks are turned into the pilot's frame (self-levelling modes only)
  bool hfBox = boxOn(boxes_, ModeBox::HEADFREE);
  if (hfBox != headfreeOn_) { headfreeOn_ = hfBox; bus::postMessage(hfBox ? "Headfree on" : "Headfree off"); }
  if (hfBox && !autonomous && (mode_ == FlightMode::ANGLE || mode_ == FlightMode::HORIZON || mode_ == FlightMode::ALTHOLD ||
                               mode_ == FlightMode::POSHOLD))
    headfreeSticks(rs, ps);
  float rollDeg = att_.roll * RAD2DEG, pitchDeg = att_.pitch * RAD2DEG;
  float cosTilt = att_.q.cosTilt();
  float accUp = -att_.accEarth.z;
  float rate[3] = {0, 0, stickRate(ys, profileRate(c, profile_, 2))};
  // throttle_limit: the stick's range scaled down in the manual modes (altitude modes compute their own)
  float throttle = thr * (c.thr_limit_pct ? c.thr_limit_pct : 100) * 0.01f;
  bool descending = false;

  // ---- launch control (Betaflight): armed on the ground with the LAUNCH switch on, the craft holds a
  // forward pitch at low throttle with the I-term off; raising the throttle past launch_thr starts
  bool manualMode = mode_ == FlightMode::ACRO || mode_ == FlightMode::ANGLE || mode_ == FlightMode::HORIZON;
  launchWaiting_ = false;
  if (manualMode && boxOn(boxes_, ModeBox::LAUNCH) && !launched_ && !airborne_) {
    if (thr * 100.0f >= c.launch_thr) {
      launched_ = true;
      bus::postMessage("Launch!");
    } else {
      launchWaiting_ = true;
      AttitudeControl::level(c, 0, -(float)c.launch_angle, rollDeg, pitchDeg, rate);
      rate[2] = 0;
    }
  }

  tgtTiltDeg_ = 180.0f;
  switch (launchWaiting_ ? FlightMode::COUNT : mode_) {
    case FlightMode::ACRO:
      rate[0] = stickRate(rs, profileRate(c, profile_, 0));
      rate[1] = stickRate(-ps, profileRate(c, profile_, 1));
      break;
    case FlightMode::ANGLE:
      AttitudeControl::level(c, rs * c.angle_limit_deg, -ps * c.angle_limit_deg, rollDeg, pitchDeg, rate);
      tgtTiltDeg_ = hypotf(rs, ps) * c.angle_limit_deg;
      break;
    case FlightMode::HORIZON:
      AttitudeControl::horizon(c, rs, ps, rollDeg, pitchDeg, rate, profile_);
      break;
    case FlightMode::ALTHOLD: {
      AttitudeControl::level(c, rs * c.angle_limit_deg, -ps * c.angle_limit_deg, rollDeg, pitchDeg, rate);
      tgtTiltDeg_ = hypotf(rs, ps) * c.angle_limit_deg;
      float climb = climbFromStick(thr, c.climb_rate_max);
      // still on the ground (low, not rising yet): the target leads by 0.5 m at most, so the climb asked
      // while the motors spool up does not pile up into an overshoot right after lift-off
      float lead = (e.alt < 1.0f && e.vz < 0.3f) ? 1.0f : 2.0f;
      altTarget_ = constrainf(altTarget_ + climb * dt, e.alt - 2.0f, e.alt + lead);
      altTarget_ = surfaceTarget(altTarget_, climb, dt);
      throttle = alt_.update(altTarget_, climb, c.climb_rate_max, c.climb_rate_max, e, accUp, cosTilt, dt);
      descending = climb < 0;
      learnHover(throttle, cosTilt, climb, dt);
      break;
    }
    case FlightMode::AUTOTUNE: {
      AttitudeControl::level(c, rs * c.angle_limit_deg, -ps * c.angle_limit_deg, rollDeg, pitchDeg, rate);
      float climb = climbFromStick(thr, c.climb_rate_max);
      altTarget_ = constrainf(altTarget_ + climb * dt, e.alt - 2.0f, e.alt + 2.0f);
      throttle = alt_.update(altTarget_, climb, c.climb_rate_max, c.climb_rate_max, e, accUp, cosTilt, dt);
      learnHover(throttle, cosTilt, climb, dt);
      bool pilot = fabsf(rs) > 0.05f || fabsf(ps) > 0.05f || fabsf(ys) > 0.05f || fabsf(climb) > 0.01f;
      float att[2] = {rollDeg, pitchDeg};
      float meas[3] = {att_.rateDps.x, att_.rateDps.y, att_.rateDps.z};
      int ax = -1;
      float r = 0;
      bool twitch = airborne_ && autotune_.update(bus::config, meas, att, pilot, dt, ax, r);
      if (airborne_) {
        PidDebug pd = bus::pid.get();
        float raw[3] = {pd.gyroRaw.x, pd.gyroRaw.y, pd.gyroRaw.z};
        autotune_.noiseSample(raw, meas, pd.dynNotchHz);
      }
      if (autotune_.gainsChanged()) bus::gainsChanged();
      if (fabsf(ys) > 0.02f) yawHold_ = yawNav_;  // pilot yaws
      else rate[2] = constrainf(c.nav_yaw_p * wrapPi(yawHold_ - yawNav_) * RAD2DEG, -90.0f, 90.0f);
      if (twitch) {
        rate[ax] = r;
        if (ax == 2) yawHold_ = yawNav_;  // do not fight a yaw twitch
      }
      if (autotune_.done() && !atAnnounced_) { atAnnounced_ = true; bus::postMessage("Autotune done: land + disarm to save"); }
      break;
    }
    case FlightMode::POSHOLD:
    case FlightMode::CRUISE:
    case FlightMode::RTH:
    case FlightMode::MISSION:
    case FlightMode::GUIDED:
    case FlightMode::SMART_RTL:
    case FlightMode::BRAKE:
    case FlightMode::CIRCLE:
    case FlightMode::FOLLOW:
    case FlightMode::LAND: {
      bool pilotNav = mode_ == FlightMode::POSHOLD || mode_ == FlightMode::CRUISE || mode_ == FlightMode::CIRCLE;
      float climbStick = pilotNav ? climbFromStick(thr, 1.0f) : 0.0f;
      nav_.update(e, yawNav_, rs, ps, ys, climbStick, land_.landed() && airborne_, headingOk_, dt, now, navOut_);
      // RTH / GPS rescue sanity check: the distance to the RTH destination (home / rally point) must keep
      // shrinking. (Not the position target: on the first return cycle that was still the climb's hold
      // point at the craft, and a best distance of 0 m could never be beaten: every return longer than
      // rth_sanity_s ended in a landing on the way.)
      if (mode_ == FlightMode::RTH && nav_.phase() == NavPhase::RTH_RETURN && c.rth_sanity_s > 0) {
        float dn, de;
        nav_.rthDestination(dn, de);
        float d = sqrtf(squaref(dn - e.n) + squaref(de - e.e));
        if (d < rthBestDist_ - 2.0f || rthBestDist_ > 1e8f) { rthBestDist_ = d; rthProgressMs_ = now; }
        else if (now - rthProgressMs_ > (uint32_t)(c.rth_sanity_s * 1000.0f) && !rescueLanding_) {
          rescueLanding_ = true;
          actionMode_ = FlightMode::LAND;
          actionSwitchState_ = switchState_;
          bus::postMessage("RTH: no progress home - landing");
        }
      } else if (mode_ != FlightMode::RTH) {
        rthBestDist_ = 1e9f;
      }
      float tr, tp;
      if (navOut_.attOverride) {
        tr = navOut_.ovRollDeg;
        tp = navOut_.ovPitchDeg;
      } else if (navOut_.horizontal) {
        pos_.update(navOut_.tgtN, navOut_.tgtE, navOut_.ffN, navOut_.ffE, navOut_.maxSpeed, e, yawNav_,
                    wind_.accN(), wind_.accE(), dt, tr, tp, navOut_.accFfN, navOut_.accFfE);
      } else {
        tr = rs * c.angle_limit_deg;
        tp = -ps * c.angle_limit_deg;
      }
      AttitudeControl::level(c, tr, tp, rollDeg, pitchDeg, rate);
      tgtTiltDeg_ = hypotf(tr, tp);
      float tgtAlt = pilotNav ? surfaceTarget(navOut_.tgtAlt, navOut_.climbFF, dt) : navOut_.tgtAlt;
      if (!pilotNav) aglHold_ = false;
      throttle = alt_.update(tgtAlt, navOut_.climbFF, navOut_.maxUp, navOut_.maxDown, e, accUp, cosTilt, dt);
      // ALTHOLD after this (GPS lost, pilot switch) holds the height the craft is at, not an old target
      altTarget_ = navOut_.landing ? e.alt : tgtAlt;
      descending = navOut_.landing || navOut_.climbFF < 0;
      if (!navOut_.landing) learnHover(throttle, cosTilt, navOut_.climbFF, dt);
      bool yawFree = navOut_.yawStickFree && fabsf(ys) > 0.02f;
      if (navOut_.yawControl) yawHold_ = navOut_.tgtYaw;
      else if (yawFree) yawHold_ = yawNav_;
      if (!yawFree) rate[2] = constrainf(c.nav_yaw_p * wrapPi(yawHold_ - yawNav_) * RAD2DEG, -90.0f, 90.0f);
      break;
    }
    case FlightMode::FLIP:
      if (!flipControl(now, dt, rollDeg, pitchDeg, cosTilt, rate, throttle)) flipActive_ = false;  // done: back to the mode below
      break;
    case FlightMode::FAILSAFE: {  // no altitude sensor: level and descend
      AttitudeControl::level(c, 0, 0, rollDeg, pitchDeg, rate);
      rate[2] = 0;
      // descent began less than 30 s after the height was lost: about 1 m/s down on the inertial vertical speed (accelerometer
      // only, good for a while), at most 5 % over the learned hover throttle. Later the
      // inertial speed has no anchor: a fixed 90 % of hover. Nothing learned: the fixed fs_throttle.
      float h = alt_.hoverEstimate(), ct = constrainf(cosTilt, 0.6f, 1.0f);
      bool learned = airborne_ && h > 0.12f && h < 0.8f;
      fsInertial_ = learned && fsModeMs_ - altValidMs_ < 30000 && fabsf(e.vz) < 10.0f;
      if (fsInertial_) throttle = constrainf(h - 0.15f * (e.vz + 1.0f), h * 0.75f, h * 1.05f) / ct;
      else throttle = learned ? h * 0.9f / ct : c.fs_throttle;
      descending = true;
      break;
    }
    default:
      break;
  }

  if (!airborne_ && (throttle > 0.25f || (e.altValid && e.alt > 0.5f))) airborne_ = true;

  // ---- landing detection & auto-disarm ----
  bool landed = land_.update(armed_, throttle, alt_.hoverEstimate(), e.vz, e, descending, now);
  if (landed && airborne_) {
    if (isNavMode(mode_) && navOut_.landing) { disarm("landed"); }
    else if (mode_ == FlightMode::FAILSAFE && fsInertial_) {  // low throttle, no vertical motion: on the ground
      if (!landedLowThrSinceMs_) landedLowThrSinceMs_ = now;
      if (now - landedLowThrSinceMs_ > 2000) disarm("landed");
    }
    else if (isAltMode(mode_) && thrStick(rc_.ch[RC_THROTTLE]) < 0.1f) {
      if (!landedLowThrSinceMs_) landedLowThrSinceMs_ = now;
      if (now - landedLowThrSinceMs_ > 3000) disarm("landed");
    }
  } else {
    landedLowThrSinceMs_ = 0;
  }
  if (!armed_) { sp.motorMode = MotorMode::STOPPED; bus::setpoint.publish(sp); return; }

  sp.motorMode = MotorMode::NORMAL;
  // horizontal acceleration compensation for the AHRS only when the heading is known (else the
  // vector would be rotated wrongly) and GPS is fresh
  sp.accKinValid = airborne_ && (now - accKinMs_) < 400 && (headingOk_ || !useGpsHeading_) && e.gpsValid;
  sp.accKin = Vec3(constrainf(accKin_.x, -8.0f, 8.0f), constrainf(accKin_.y, -8.0f, 8.0f), 0.0f);
  for (int i = 0; i < 3; i++) sp.rateDps[i] = rate[i];
  // the yaw rate is a plain stick curve in ACRO / ANGLE / HORIZON / ALTHOLD (nothing overrides it there)
  bool pilotYaw = !autonomous && !launchWaiting_ &&
                  (mode_ == FlightMode::ACRO || mode_ == FlightMode::ANGLE || mode_ == FlightMode::HORIZON ||
                   mode_ == FlightMode::ALTHOLD);
  sp.stickDirect = (uint8_t)((pilotYaw && mode_ == FlightMode::ACRO ? 1 : 0) | (pilotYaw ? 2 : 0));
  sp.throttle = throttle;
  sp.integrate = airborne_ && !launchWaiting_;
  sp.profile = profile_;
  // battery sag compensation (Betaflight vbat_sag_compensation): scale the outputs as if the pack
  // were full (4.2 V / cell), so the response and the hover throttle stay the same as it discharges
  sp.sagFactor = 1.0f;
  if (c.vbat_sag_comp && bat_.cellVoltage > 3.0f && bat_.cellVoltage < 4.2f)
    sp.sagFactor = 1.0f + c.vbat_sag_comp * 0.01f * (4.2f / bat_.cellVoltage - 1.0f);
  lastThrottle_ = throttle;
  bus::setpoint.publish(sp);
}

// ---------------------------------------------------------------- hover learning (ArduPilot MOT_HOVER_LEARN)
void Commander::learnHover(float throttle, float cosTilt, float climbCmd, float dt) {
  const Config& c = cfg();
  const NavEstimate& e = navEst_;
  if (!c.hover_learn || !airborne_ || !e.altValid) return;
  if (fabsf(e.vz) > 0.3f || fabsf(climbCmd) > 0.1f || cosTilt < 0.94f || throttle < 0.08f || throttle > 0.9f) return;
  // the throttle that holds altitude (tilt removed); the 2 s filter lets the integrator settle first
  hoverLearn_ += (throttle * cosTilt - hoverLearn_) * dt / 2.0f;
  hoverSamples_++;
  if (hoverSamples_ > 250) alt_.setHover(hoverLearn_);  // after ~1 s of steady hover
}

void Commander::saveHover() {
  Config& c = bus::config;
  if (c.hover_learn != 2 || hoverSamples_ < 1250 || fabsf(hoverLearn_ - c.hover_throttle) < 0.005f) return;
  c.hover_throttle = constrainf(hoverLearn_, 0.1f, 0.8f);
  // patch only this value into the stored configuration (unsaved edits elsewhere stay unsaved)
  Config stored;
  if (hal::storageLoad(hal::Slot::CONFIG, &stored, sizeof(stored)) && configValid(stored)) {
    stored.hover_throttle = c.hover_throttle;
    configSeal(stored);
    hal::storageSave(hal::Slot::CONFIG, &stored, sizeof(stored));
  }
  char m[48];
  snprintf(m, sizeof(m), "Hover throttle learned: %.3f", (double)c.hover_throttle);
  bus::postMessage(m);
}

// ---------------------------------------------------------------- autotune result
void Commander::finishAutotune(bool keep) {
  Config& c = bus::config;
  if (keep) {
    bool filters = autotune_.filterResult(c);  // gyro / D-term low-pass, dynamic notch range
    Config stored;  // patch only the tuned values into the stored configuration
    if (hal::storageLoad(hal::Slot::CONFIG, &stored, sizeof(stored)) && configValid(stored)) {
      for (int a = 0; a < 3; a++) stored.pid[a] = c.pid[a];
      if (filters) {
        stored.gyro_lpf_hz = c.gyro_lpf_hz;
        stored.dterm_lpf_hz = c.dterm_lpf_hz;
        stored.dyn_notch_count = c.dyn_notch_count;
        stored.dyn_notch_min = c.dyn_notch_min;
        stored.dyn_notch_max = c.dyn_notch_max;
      }
      configSeal(stored);
      hal::storageSave(hal::Slot::CONFIG, &stored, sizeof(stored));
    }
    if (filters) bus::configChanged();  // disarmed now: the fast loop may re-initialise its filters
    if (c.at_steps & 8) {
      char fm[64];
      snprintf(fm, sizeof(fm), "Autotune noise %.1f/%.2f dps: lpf %.0f/%.0f", (double)autotune_.noiseRaw(),
               (double)autotune_.noiseFilt(), (double)c.gyro_lpf_hz, (double)c.dterm_lpf_hz);
      bus::postMessage(fm);
    }
    char m[64];
    snprintf(m, sizeof(m), "Autotune saved: R %.0f/%.0f P %.0f/%.0f", (double)c.pid[0].p, (double)c.pid[0].d,
             (double)c.pid[1].p, (double)c.pid[1].d);
    bus::postMessage(m);
  } else {
    autotune_.restore(c);
    bus::postMessage("Autotune discarded, previous gains restored");
  }
  bus::gainsChanged();
  autotune_ = AutoTune();
  atAnnounced_ = false;
}

// ---------------------------------------------------------------- surface tracking (rangefinder)
float Commander::surfaceTarget(float baroTarget, float climb, float dt) {
  const Config& c = cfg();
  const NavEstimate& e = navEst_;
  if (!c.alt_surface || !e.aglValid) { aglHold_ = false; return baroTarget; }
  if (!aglHold_) { aglHold_ = true; aglTarget_ = e.agl; }
  aglTarget_ = constrainf(aglTarget_ + climb * dt, 0.3f, c.rf_max_m * 0.9f);
  // altitude target that puts the craft aglTarget above the ground currently below it
  float t = e.alt + (aglTarget_ - e.agl);
  altTarget_ = t;  // when the rangefinder drops out, the baro hold continues from here
  return t;
}

// ---------------------------------------------------------------- GUIDED input (GCS)
void Commander::guidedInput(uint32_t now) {
  (void)now;
  bus::GuidedTarget g;
  uint32_t seq = bus::guided.read(g);
  if (seq == guidedSeq_) return;
  guidedSeq_ = seq;
  const NavEstimate& e = navEst_;
  if (!armed_ || !e.homeSet || !e.posValid) { bus::postMessage("Guided: needs armed + position"); return; }
  float n = e.n, ea = e.e;
  if (g.local) { n = g.n; ea = g.e; }
  else if (g.lat || g.lon) NavEstimator::llaToNed(g.lat * 1e-7, g.lon * 1e-7, e.homeLat, e.homeLon, n, ea);
  float alt = g.alt == g.alt ? g.alt : (mode_ == FlightMode::GUIDED ? navOut_.tgtAlt : e.alt);
  if (alt < 1.0f) alt = 1.0f;
  if (fenceBreached(cfg(), e) == false && cfg().fence_enable) {
    // refuse targets outside the fence
    NavEstimate t = e;
    t.n = n; t.e = ea; t.alt = alt;
    if (fenceBreached(cfg(), t)) { bus::postMessage("Guided: target outside the fence"); return; }
  }
  nav_.setGuided(n, ea, alt);
  if (mode_ != FlightMode::GUIDED) { gcsMode_ = FlightMode::GUIDED; gcsSwitchState_ = switchState_; }
}

// ---------------------------------------------------------------- crash detection
// ArduPilot crash check (attitude far from what self-levelling commands for 2 s) + an impact
// followed by stillness, or lying upside down without turning. Motors are stopped.
void Commander::crashCheck(uint32_t now) {
  const Config& c = cfg();
  if (!c.crash_detect || !armed_ || !airborne_ || mode_ == FlightMode::TURTLE) { crashSinceMs_ = 0; return; }
  float tiltDeg = acosf(constrainf(att_.q.cosTilt(), -1.0f, 1.0f)) * RAD2DEG;
  float rates = att_.rateDps.norm();
  float accN = att_.accBody.norm();
  if (accN > 4.0f * GRAVITY) impactMs_ = now;
  bool selfLevel = mode_ != FlightMode::ACRO;
  (void)c;
  // ArduPilot: more than 30 deg away from the commanded attitude for 2 s
  bool levelLost = selfLevel && tgtTiltDeg_ < 90.0f && tiltDeg > tgtTiltDeg_ + 30.0f;
  bool upsideDown = tiltDeg > 110.0f && rates < 60.0f;
  bool stillAfterImpact = impactMs_ && now - impactMs_ < 3000 && rates < 30.0f && fabsf(accN - GRAVITY) < 2.5f &&
                          tiltDeg > 45.0f;
  if (levelLost || upsideDown || stillAfterImpact) {
    if (!crashSinceMs_) crashSinceMs_ = now;
    if (now - crashSinceMs_ > (stillAfterImpact ? 1000u : 2000u)) disarm("crash detected");
  } else {
    crashSinceMs_ = 0;
  }
}

// ---------------------------------------------------------------- 6-point accelerometer calibration
// Each of the six faces down for ~1 s (still): the mean of each side gives bias (mid-point) and
// scale (2 g / span) per axis, like ArduPilot's accelerometer calibration.
void Commander::acc6Update(uint32_t now) {
  Config& c = bus::config;
  if (now - acc6StartMs_ > 180000) {
    acc6_ = false; bus::acc6Active = false;
    bus::postMessage("Accel 6-point: timed out");
    return;
  }
  // raw specific force (without the stored bias / scale)
  Vec3 raw(att_.accBody.x / c.acc_scale[0] + c.acc_bias[0], att_.accBody.y / c.acc_scale[1] + c.acc_bias[1],
           att_.accBody.z / c.acc_scale[2] + c.acc_bias[2]);
  float n = raw.norm();
  bool still = att_.rateDps.norm() < 4.0f && fabsf(n - GRAVITY) < 1.5f;
  int ax = 0;
  for (int i = 1; i < 3; i++) if (fabsf(raw[i]) > fabsf(raw[ax])) ax = i;
  int side = ax * 2 + (raw[ax] > 0 ? 0 : 1);  // FRD: level (z down) reads -g -> side 5
  bool upright = fabsf(raw[ax]) > 0.85f * n;
  if (!still || !upright || (bus::acc6Mask & (1u << side))) { acc6Side_ = -1; acc6Count_ = 0; return; }
  if (side != acc6Side_) { acc6Side_ = side; acc6Count_ = 0; acc6Sum_ = Vec3(); }
  acc6Sum_ += raw;
  if (++acc6Count_ < 250) return;
  acc6Mean_[side] = acc6Sum_ * (1.0f / acc6Count_);
  bus::acc6Mask = (uint8_t)(bus::acc6Mask | (1u << side));
  acc6Side_ = -1;
  int done = 0;
  for (int i = 0; i < 6; i++) done += (bus::acc6Mask >> i) & 1;
  static const char* names[6] = {"nose down", "nose up", "right side down", "left side down", "upside down", "level"};
  char m[48];
  snprintf(m, sizeof(m), "Accel 6-point: %s ok (%d/6)", names[side], done);
  bus::postMessage(m);
  if (bus::acc6Mask != 0x3F) return;
  bool ok = true;
  float bias[3], scale[3];
  for (int k = 0; k < 3; k++) {
    float pos = acc6Mean_[k * 2][k], neg = acc6Mean_[k * 2 + 1][k];
    float span = pos - neg;
    bias[k] = (pos + neg) * 0.5f;
    scale[k] = span > 1.0f ? 2.0f * GRAVITY / span : 0;
    if (scale[k] < 0.8f || scale[k] > 1.2f || fabsf(bias[k]) > 3.0f) ok = false;
  }
  acc6_ = false;
  bus::acc6Active = false;
  if (!ok) { bus::postMessage("Accel 6-point: failed (values out of range)"); return; }
  for (int k = 0; k < 3; k++) { c.acc_bias[k] = bias[k]; c.acc_scale[k] = scale[k]; }
  c.acc_calibrated = 1;
  c.level_trim_roll_deg = c.level_trim_pitch_deg = 0;
  bus::configChanged();
  bus::postMessage("Accel 6-point calibrated - save to keep");
}

// ---------------------------------------------------------------- logic conditions
int32_t Commander::logicValue(uint8_t type, int16_t v) const {
  const NavEstimate& e = navEst_;
  switch (type) {
    case LOP_CONST: return v;
    case LOP_RC: return v >= 1 && v <= RC_MAX_CH ? rc_.ch[v - 1] : 0;
    case LOP_RULE: return v >= 0 && v < LOGIC_RULES ? (logicRes_ >> v) & 1 : 0;
    case LOP_FLIGHT:
      switch (v) {
        case LV_ARMED: return armed_;
        case LV_ALT_DM: return (int32_t)(e.alt * 10.0f);
        case LV_SPEED_CMS: return (int32_t)(sqrtf(e.vn * e.vn + e.ve * e.ve) * 100.0f);
        case LV_HOME_M: return e.homeSet ? (int32_t)sqrtf(e.n * e.n + e.e * e.e) : 0;
        case LV_VBAT_CV: return (int32_t)(bat_.voltage * 100.0f);
        case LV_BAT_PCT: return bat_.percent;
        case LV_AMPS_DA: return (int32_t)(bat_.current * 10.0f);
        case LV_ARM_TIME_S: return armed_ ? (int32_t)((os::timeMs() - armTimeMs_) / 1000) : 0;
        case LV_MODE: return (int32_t)mode_;
        case LV_LINK_PCT: return rc_.linkQuality;
        case LV_ROLL_DEG: return (int32_t)(att_.roll * RAD2DEG);
        case LV_PITCH_DEG: return (int32_t)(att_.pitch * RAD2DEG);
        case LV_SATS: return bus::gps.get().sats;
        case LV_MAH: return (int32_t)bat_.mahDrawn;
        default: return 0;
      }
    default: return 0;
  }
}

void Commander::logicUpdate(uint32_t now) {
  (void)now;
  const Config& c = cfg();
  uint8_t res = 0;
  uint32_t boxes = 0;
  uint16_t rc[16] = {0};
  bool beep = false, noArm = false;
  uint8_t vtx = 0;
  for (int i = 0; i < LOGIC_RULES; i++) {
    const LogicRule& r = c.logic[i];
    if (!r.enabled) continue;
    int32_t a = logicValue(r.aType, r.aVal), b = logicValue(r.bType, r.bVal);
    bool t = false;
    switch (r.op) {
      case LO_TRUE: t = true; break;
      case LO_EQ: t = a == b; break;
      case LO_GT: t = a > b; break;
      case LO_LT: t = a < b; break;
      case LO_AND: t = a && b; break;
      case LO_OR: t = a || b; break;
      case LO_NOT: t = !a; break;
      case LO_GE: t = a >= b; break;
      case LO_LE: t = a <= b; break;
      case LO_NE: t = a != b; break;
      default: break;
    }
    if (!t) continue;
    res |= (uint8_t)(1u << i);
    logicRes_ = (uint8_t)(logicRes_ | (1u << i));  // later rules can use this result in the same pass
    switch (r.action) {
      case LA_BEEP: beep = true; break;
      case LA_MODE_BOX:
        if (r.actArg > 0 && r.actArg < (uint8_t)ModeBox::COUNT && r.actArg != (uint8_t)ModeBox::ARM) boxes |= 1u << r.actArg;
        break;
      case LA_RC:
        if (r.actArg >= 1 && r.actArg <= 16) rc[r.actArg - 1] = (uint16_t)constrainf(r.actVal, 800, 2200);
        break;
      case LA_VTX_POWER: vtx = (uint8_t)constrainf(r.actVal, 1, 8); break;
      case LA_MESSAGE:
        if (!(logicPrev_ & (1u << i))) { char m[16]; snprintf(m, sizeof(m), "Logic %d", i + 1); bus::postMessage(m); }
        break;
      case LA_NO_ARM: noArm = true; break;
      default: break;
    }
  }
  logicRes_ = res;
  logicPrev_ = res;
  logicBoxes_ = boxes;
  memcpy(logicRc_, rc, sizeof(rc));
  logicNoArm_ = noArm;
  bus::logicBeep = beep;
  bus::logicVtxPower = vtx;
  bus::logicResults = res;
}

void Commander::publish(uint32_t now) {
  NavEstimate e = navEst_;
  e.yaw = yawNav_;
  e.headingSrc = headingSrc_;
  e.headingOk = headingOk_;
  e.windN = wind_.windN();
  e.windE = wind_.windE();
  e.hoverThr = alt_.hover();
  bool navActive = armed_ && isNavMode(mode_);
  e.tgtN = navActive ? navOut_.tgtN : e.n;
  e.tgtE = navActive ? navOut_.tgtE : e.e;
  e.tgtAlt = armed_ && isAltMode(mode_) ? (navActive && !aglHold_ ? navOut_.tgtAlt : altTarget_) : e.alt;
  e.aglHold = aglHold_ && armed_;
  bus::nav.publish(e);
  VehicleStatus st{};
  st.armed = armed_;
  st.mode = mode_ == FlightMode::COUNT ? FlightMode::ANGLE : mode_;
  st.navPhase = isNavMode(mode_) ? nav_.phase() : NavPhase::IDLE;
  st.armBlockers = blockers_;
  st.failsafe = rcFailsafe_ || actionMode_ != FlightMode::COUNT;
  st.landed = land_.landed();
  st.armedTimeMs = armed_ ? now - armTimeMs_ : 0;
  st.missionIndex = (uint16_t)nav_.missionIndex();
  st.missionCount = (uint16_t)nav::count();
  st.homeDistance = e.homeSet ? sqrtf(e.n * e.n + e.e * e.e) : 0;
  st.homeBearingDeg = e.homeSet ? atan2f(-e.e, -e.n) * RAD2DEG : 0;
  st.atState = autotune_.done() ? 2 : (autotune_.active() ? 1 : 0);
  st.atAxis = (uint8_t)(autotune_.axis() < 0 ? 0 : autotune_.axis());
  st.atRound = (uint8_t)autotune_.round();
  st.atOvershoot = autotune_.lastOvershoot();
  st.atRiseMs = autotune_.lastRiseMs();
  st.atStep = (uint8_t)autotune_.step();
  st.atLagMs = autotune_.lastLagMs();
  st.atSteadyErr = autotune_.lastSteadyErr();
  uint32_t w = 0;
  if (bat_.warn) w |= WARN_BAT_LOW;
  if (bat_.critical) w |= WARN_BAT_CRIT;
  if (armed_ && rcFailsafe_) w |= WARN_RX_LOST;
  if (e.homeSet && !e.gpsValid) w |= WARN_GPS_LOST;
  if (e.gpsGlitch) w |= WARN_GPS_GLITCH;
  if (useGpsHeading_ && !headingOk_ && e.gpsValid) w |= WARN_NO_HEADING;
  if (fenceBreached(cfg(), e)) w |= WARN_FENCE;
  if (navOut_.landing && isNavMode(mode_)) w |= WARN_LANDING;
  const Config& c = cfg();
  if (c.vibe_warn > 0 && armed_ && att_.vibration > c.vibe_warn) vibeWarnUntilMs_ = now + 2000;
  if ((int32_t)(vibeWarnUntilMs_ - now) > 0) w |= WARN_VIBRATION;
  if (att_.accClips != clipsSeen_) {
    if (armed_) {
      clipWarnUntilMs_ = now + 3000;
      if (!clipTold_) { clipTold_ = true; bus::postMessage("Accel clipping: check FC mounting / vibration"); }
      clipsFlight_ += att_.accClips - clipsSeen_;
    }
    clipsSeen_ = att_.accClips;
  }
  if ((int32_t)(clipWarnUntilMs_ - now) > 0) w |= WARN_ACC_CLIP;
  if (c.beginner) w |= WARN_BEGINNER;
  if (motorWarn_ || rpmWarn_) w |= WARN_MOTOR;
  if (thrustWarn_) w |= WARN_THRUST;
  if (smartWarn_ && armed_) w |= WARN_RTH_BAT;
  bool ekfOn = c.est_type == 1 && ekf_.initialized();
  if (ekfOn && (ekf_.status().faults & (EKF_BARO_FAULT | EKF_MAG_FAULT))) w |= WARN_EKF;
  if (launchWaiting_) w |= WARN_LAUNCH;
  if (rescueLanding_ && armed_) w |= WARN_RESCUE;
  if (gcsFsDone_ && armed_) w |= WARN_GCS_LOST;
  if (bus::imuActive) w |= WARN_IMU2;
  if (armed_ && bus::rpmLostMask) w |= WARN_RPM_LOST;
  st.warnings = w;
  bus::status.publish(st);
  EkfReport r{};
  r.active = ekfOn;
  r.tiltResets = ekfTiltResets_;
  r.yawResets = ekfYawResets_;
  r.baroResets = est_.baroResets();
  if (ekfOn) {
    const EkfStatus& k = ekf_.status();
    r.trGpsPos = k.trGpsPos; r.trGpsVel = k.trGpsVel; r.trHeight = k.trHeight;
    r.trYaw = k.trYaw; r.trMag = k.trMag; r.trFlow = k.trFlow;
    r.faults = k.faults;
    r.posResets = k.posResets; r.velResets = k.velResets; r.filterResets = k.filterResets;
    r.posStd = k.posStd; r.velStd = k.velStd; r.hgtStd = k.hgtStd; r.yawStdDeg = k.yawStdDeg;
    r.heightSrc = k.heightSrc;
    Vec3 gb = ekf_.gyroBias(), ab = ekf_.accBias();
    for (int i = 0; i < 3; i++) { r.gyroBias[i] = gb[i] * RAD2DEG; r.accBias[i] = ab[i]; }
    r.yawDeg = ekf_.yaw() * RAD2DEG;
    r.rollDeg = ekf_.q().roll() * RAD2DEG;
    r.pitchDeg = ekf_.q().pitch() * RAD2DEG;
  }
  bus::ekf.publish(r);
}

// ---------------------------------------------------------------- head-free
// The roll / pitch sticks move the craft in the pilot's frame: "forward" is where the nose pointed at
// arming (headfree_type 0), or straight away from home once 5 m out (1, home lock: pull back = come home).
void Commander::headfreeSticks(float& rs, float& ps) {
  const Config& c = cfg();
  const NavEstimate& e = navEst_;
  float ref = headfreeRef_;
  if (c.headfree_type == 1 && e.homeSet && e.posValid && e.n * e.n + e.e * e.e > 25.0f) ref = atan2f(e.e, e.n);
  float d = wrapPi(yawNav_ - ref), cd = cosf(d), sd = sinf(d);
  float f = ps, r = rs;
  ps = f * cd + r * sd;
  rs = -f * sd + r * cd;
}

// ---------------------------------------------------------------- FLIP
// ArduPilot FLIP: switch on -> climb kick 0.25 s, 400 deg/s round the roll axis (pitch when the pitch stick
// is pushed further than roll; left / back with the stick that way), little thrust upside down, then
// level again and back to the mode underneath. Only from ACRO / ANGLE / HORIZON / ALTHOLD, 3 m up.
void Commander::flipTrigger(uint32_t now) {
  bool box = boxOn(boxes_, ModeBox::FLIP);
  bool rising = box && !flipBoxPrev_;
  flipBoxPrev_ = box;
  if (!rising || flipActive_ || !armed_) return;
  const NavEstimate& e = navEst_;
  bool fromOk = mode_ == FlightMode::ACRO || mode_ == FlightMode::ANGLE || mode_ == FlightMode::HORIZON ||
                mode_ == FlightMode::ALTHOLD;
  if (!fromOk) { bus::postMessage("Flip: from ACRO / ANGLE / HORIZON / ALTHOLD only"); return; }
  if (!airborne_ || (e.altValid && e.alt < 3.0f)) { bus::postMessage("Flip: too low (3 m)"); return; }
  float rs = stick(rc_.ch[RC_ROLL]), ps = stick(rc_.ch[RC_PITCH]);
  if (fabsf(ps) > 0.3f && fabsf(ps) > fabsf(rs)) { flipAxis_ = 1; flipDir_ = ps > 0 ? 1 : -1; }
  else { flipAxis_ = 0; flipDir_ = rs < -0.3f ? -1 : 1; }
  flipActive_ = true;
  flipPhase_ = FLIP_START;
  flipAngle_ = 0;
  flipStartMs_ = flipPhaseMs_ = now;
  bus::postMessage("Flip!");
}

bool Commander::flipControl(uint32_t now, float dt, float rollDeg, float pitchDeg, float cosTilt, float rate[3],
                            float& throttle) {
  const Config& c = cfg();
  float h = constrainf(alt_.hoverEstimate(), 0.15f, 0.8f), ct = constrainf(cosTilt, 0.6f, 1.0f);
  uint32_t inPhase = now - flipPhaseMs_;
  if (now - flipStartMs_ > 3000 && flipPhase_ != FLIP_RECOVER) { flipPhase_ = FLIP_RECOVER; flipPhaseMs_ = now; inPhase = 0; }
  switch (flipPhase_) {
    case FLIP_START:  // climb kick: height to lose on the way round
      AttitudeControl::level(c, 0, 0, rollDeg, pitchDeg, rate);
      rate[2] = 0;
      throttle = fminf(h * 1.6f, 0.9f) / ct;
      if (inPhase > 250) { flipPhase_ = FLIP_ROLL; flipPhaseMs_ = now; }
      return true;
    case FLIP_ROLL: {
      float meas = flipAxis_ == 0 ? att_.rateDps.x : att_.rateDps.y;
      float want = 400.0f * flipDir_ * (flipAxis_ == 1 ? -1.0f : 1.0f);  // pitch: forward = nose down
      rate[0] = flipAxis_ == 0 ? want : 0.0f;
      rate[1] = flipAxis_ == 1 ? want : 0.0f;
      rate[2] = 0;
      flipAngle_ += fabsf(meas) * dt;
      throttle = cosTilt < 0 ? h * 0.3f : h * 0.8f;  // upside down thrust only pushes it down
      if (flipAngle_ > 300.0f) { flipPhase_ = FLIP_RECOVER; flipPhaseMs_ = now; }
      return true;
    }
    default:  // level again, then hand back
      AttitudeControl::level(c, 0, 0, rollDeg, pitchDeg, rate);
      rate[2] = 0;
      throttle = fminf(h * 1.3f, 0.9f) / ct;
      if ((fabsf(rollDeg) < 15.0f && fabsf(pitchDeg) < 15.0f && inPhase > 200) || inPhase > 2000) return false;
      return true;
  }
}

// ---------------------------------------------------------------- FOLLOW leader
void Commander::followInput(uint32_t now) {
  bus::FollowTarget f;
  uint32_t seq = bus::follow.read(f);
  const NavEstimate& e = navEst_;
  if (seq != followSeq_) { followSeq_ = seq; followMs_ = now; }
  bool fresh = followMs_ && now - followMs_ < 3000 && e.homeSet && (f.lat || f.lon);
  float n = 0, ea = 0;
  if (fresh) {
    NavEstimator::llaToNed(f.lat * 1e-7, f.lon * 1e-7, e.homeLat, e.homeLon, n, ea);
    float age = (now - followMs_) * 1e-3f;  // where the leader is by now
    n += f.vn * age;
    ea += f.ve * age;
    if (n * n + ea * ea > 1000.0f * 1000.0f) fresh = false;  // more than 1 km from home: not a leader to follow
  }
  nav_.setFollow(n, ea, f.vn, f.ve, fresh);
  if (mode_ == FlightMode::FOLLOW && !fresh && !followTold_) { followTold_ = true; bus::postMessage("Follow: no leader - holding"); }
  if (fresh) followTold_ = false;
}

// ---------------------------------------------------------------- battery needed to come home
// DJI smart RTH / iNav: once a second, the charge the way home takes (climb to rth_alt, fly back at
// rth_speed with the wind, loiter, land) at the average current of the flight, plus 20 %. When the charge
// left above bat_reserve_pct drops below it for 3 s: warn and (bat_smart_rth 2) start RTH.
// With bat_capacity and a current sensor it counts mAh. Without them it uses the charge estimate from
// the resting voltage (the learned sag added back) and how fast it falls: the time left above the
// reserve against the time the way home takes.
void Commander::smartRth(uint32_t now) {
  const Config& c = cfg();
  if (!c.bat_smart_rth || !airborne_ || now - smartMs_ < 1000) return;
  smartMs_ = now;
  if (bat_.loadIsCurrent && bat_.current >= 0) ampsAvg_ = ampsAvg_ < 0 ? bat_.current : ampsAvg_ + (bat_.current - ampsAvg_) * 0.1f;
  // charge rate from the resting-voltage estimate (%/s, 30 s memory, sampled every 5 s)
  if (!pctRefMs_) { pctRefMs_ = now; pctRef_ = bat_.percentF; }
  else if (now - pctRefMs_ >= 5000) {
    float r = (pctRef_ - bat_.percentF) * 1000.0f / (float)(now - pctRefMs_);
    pctRate_ = pctRate_ <= 0 ? r : pctRate_ + (r - pctRate_) * 0.17f;
    pctRefMs_ = now;
    pctRef_ = bat_.percentF;
  }
  const NavEstimate& e = navEst_;
  smartWarn_ = false;
  bool counting = c.bat_capacity > 0 && bat_.loadIsCurrent && ampsAvg_ > 0.5f;
  bool byVoltage = !counting && bat_.restLearned && bat_.cells && now - armTimeMs_ > 60000 && pctRate_ > 0.005f;
  if ((!counting && !byVoltage) || !e.homeSet || !e.posValid || !e.gpsValid) return;
  float d = sqrtf(e.n * e.n + e.e * e.e), alt = fmaxf(e.alt, 0.0f);
  float rthAlt = fmaxf(alt, c.rth_alt);
  if (c.fence_enable && c.fence_alt_max > 3.0f) rthAlt = fminf(rthAlt, c.fence_alt_max - 2.0f);
  float speed = fmaxf(c.rth_speed, 1.0f);
  float head = 0;  // wind against the way home (m/s): more air speed for the same ground speed
  if (wind_.valid() && d > 1.0f) head = (wind_.windN() * e.n + wind_.windE() * e.e) / d;
  float tClimb = fmaxf(rthAlt - alt, 0.0f) / fmaxf(c.climb_rate_max, 0.5f);
  float tHome = d / speed * (1.0f + 0.5f * fmaxf(head, 0.0f) / speed);
  float tLand = fmaxf(rthAlt - c.land_slow_alt, 0.0f) / fmaxf(c.land_speed, 0.2f) +
                fminf(rthAlt, c.land_slow_alt) / fmaxf(c.land_slow_speed, 0.1f);
  float t = tClimb + tHome + c.rth_loiter_s + tLand;
  if (counting) {  // mAh
    rthNeedMah_ = ampsAvg_ * t / 3.6f * 1.2f;
    rthLeftMah_ = c.bat_capacity * (100 - c.bat_reserve_pct) * 0.01f - bat_.mahDrawn;
  } else {  // seconds: the charge left above the reserve at the rate it falls
    rthNeedMah_ = t * 1.2f;
    rthLeftMah_ = (bat_.percentF - c.bat_reserve_pct) / pctRate_;
  }
  smartWarn_ = rthLeftMah_ < rthNeedMah_ * 1.15f;
  smartLowN_ = rthLeftMah_ < rthNeedMah_ ? (uint8_t)(smartLowN_ < 250 ? smartLowN_ + 1 : 250) : 0;
  if (smartLowN_ >= 3 && !smartDone_ && d > c.rth_home_radius) {
    smartDone_ = true;
    char m[48];
    if (counting) snprintf(m, sizeof(m), "Battery: home now (%d mAh needed)", (int)rthNeedMah_);
    else snprintf(m, sizeof(m), "Battery: home now (%d s needed)", (int)rthNeedMah_);
    bus::postMessage(m);
    bool returning = mode_ == FlightMode::RTH || mode_ == FlightMode::LAND || mode_ == FlightMode::SMART_RTL ||
                     actionMode_ != FlightMode::COUNT || rcFailsafe_;
    if (c.bat_smart_rth == 2 && !returning) {
      actionMode_ = FlightMode::RTH;
      actionSwitchState_ = switchState_;
    }
  }
}

// ---------------------------------------------------------------- compass offsets learned in flight
// While flying (mag_learn 1): 5 compass samples a second against the attitude with a gyro-integrated
// heading. Once the turns cover 9 of 12 heading sectors: solve. Offset under 5 % of the field: the
// calibration is fine. Larger: an uncalibrated compass gets it at once (it was not used anyway: the
// EKF now has a compass); a calibrated one is corrected after landing (a heading change in flight
// would only fight the EKF) unless the EKF has already rejected it as wrong; the stored settings get
// the new offsets either way.
void Commander::magLearnStep(uint32_t now, float dt, bool newMag) {
  Config& c = bus::config;
  if (!c.mag_learn || !armed_ || !airborne_ || mlDone_ || hitl::active() || !att_.valid) return;
  Vec3 wE = att_.q.rotate(att_.rateDps * DEG2RAD);
  mlYaw_ = wrapPi(mlYaw_ + wE.z * dt);
  if (!newMag || !magSeen_ || !mag_.healthy || now - mlSampleMs_ < 200) return;
  if (att_.rateDps.norm() > 150 || mode_ == FlightMode::FLIP || mode_ == FlightMode::TURTLE) return;
  if ((int32_t)(bus::magCalUntilMs - now) > 0) return;
  mlSampleMs_ = now;
  Quat q = Quat::fromEuler(att_.roll, att_.pitch, mlYaw_);
  magLearn_.add(mag_.field, q, mlYaw_);
  if (magLearn_.sectors() < 9 || magLearn_.samples() < 150 || now - mlSolveMs_ < 5000) return;
  mlSolveMs_ = now;
  Vec3 m, b;
  float rms;
  if (!magLearn_.solve(m, b, rms)) return;
  float fm = m.norm();
  if (fm < 0.1f || fm > 1.5f || rms > 0.06f * fm) return;  // not a consistent fit (yet)
  mlDone_ = true;
  int pct = (int)(b.norm() / fm * 100.0f + 0.5f);
  char msg[48];
  if (pct < 5) {
    snprintf(msg, sizeof(msg), "Compass: in-flight check OK (%d%%)", pct);
    bus::postMessage(msg);
    return;
  }
  mlDelta_ = magBiasDelta(b, c.mag_scale, c.mag_offdiag);
  bool uncalibrated = c.mag_bias[0] == 0 && c.mag_bias[1] == 0 && c.mag_bias[2] == 0;
  // in flight when the compass is not in use anyway: never calibrated, or the EKF has rejected it
  if (uncalibrated || ekf_.magFault()) {
    for (int i = 0; i < 3; i++) c.mag_bias[i] += mlDelta_[i];
    if (c.mag_bias[0] == 0) c.mag_bias[0] = 1e-6f;  // marks it calibrated
    bus::configChanged();
    mlDelta_ = Vec3();
    snprintf(msg, sizeof(msg), "Compass: offsets learned in flight (%d%%)", pct);
  } else {
    snprintf(msg, sizeof(msg), "Compass off by %d%%: fixed at landing", pct);
  }
  mlPending_ = true;
  bus::postMessage(msg);
}

void Commander::magLearnSave() {
  if (!mlPending_) return;
  mlPending_ = false;
  Config& c = bus::config;
  for (int i = 0; i < 3; i++) c.mag_bias[i] += mlDelta_[i];
  mlDelta_ = Vec3();
  bus::configChanged();
  // only the offsets go into the stored settings (unsaved edits elsewhere stay unsaved)
  Config stored;
  if (hal::storageLoad(hal::Slot::CONFIG, &stored, sizeof(stored)) && configValid(stored)) {
    for (int i = 0; i < 3; i++) stored.mag_bias[i] = c.mag_bias[i];
    configSeal(stored);
    hal::storageSave(hal::Slot::CONFIG, &stored, sizeof(stored));
  }
  bus::postMessage("Compass offsets saved");
}

// ---------------------------------------------------------------- dead reckoning (drag model)
// 10 Hz: while GPS is used and the craft flies, the drag model learns k and the wind from the body
// accelerometer against the EKF velocity. When GPS goes, for dr_time_s the drag velocity is fused into
// the EKF and the EKF position stays valid: POSHOLD keeps holding (with the wind) instead of dropping to
// ALTHOLD and drifting downwind. After dr_time_s, or without a learned model: as before (ALTHOLD).
void Commander::dragStep(uint32_t now) {
  const Config& c = bus::config;
  bool gps = ekf_.gpsUsed(now);
  // 10 Hz, but at once on the cycle GPS times out (the EKF would otherwise be unaided for a moment)
  if (now - dragMs_ < 100 && !(!gps && !drActive_ && !gpsLostMs_)) return;
  dragMs_ = now;
  if (gps) { gpsLostMs_ = 0; drActive_ = false; drTold_ = false; }
  else if (!gpsLostMs_) gpsLostMs_ = now;
  if (!armed_ || !airborne_ || !att_.valid) { drActive_ = false; return; }
  // body specific force minus the EKF accelerometer bias, rotated north / east with the heading
  Vec3 ba = ekf_.accBias();
  float ax = att_.accBody.x - ba.x, ay = att_.accBody.y - ba.y;
  float yaw = ekf_.yaw(), cy = cosf(yaw), sy = sinf(yaw);
  float aN = cy * ax - sy * ay, aE = sy * ax + cy * ay;
  Vec3 v = ekf_.vel();
  bool calmRates = fabsf(att_.rateDps.x) < 90 && fabsf(att_.rateDps.y) < 90 && att_.q.cosTilt() > 0.82f;
  if (gps && calmRates && mode_ != FlightMode::FLIP && mode_ != FlightMode::TURTLE) drag_.learn(aN, aE, v.x, v.y);
  drActive_ = false;
  if (gps || !c.dr_time_s || !drag_.ready() || !calmRates) return;
  if (now - gpsLostMs_ > c.dr_time_s * 1000u) {
    if (drTold_) { drTold_ = false; bus::postMessage("Dead reckoning ended - no position"); }
    return;
  }
  float vn, ve;
  drag_.velocity(aN, aE, vn, ve);
  // the error grows with speed (k) and with time (the wind moves on)
  float sigma = 0.4f + 0.08f * sqrtf(vn * vn + ve * ve) + 0.02f * (now - gpsLostMs_) * 1e-3f;
  ekf_.fuseDragVel(vn, ve, sigma, now);
  if (!(ekf_.status().faults & EKF_DRAG_REJECT)) { drFusedMs_ = now; }
  drActive_ = drFusedMs_ && now - drFusedMs_ < 1000;
  if (drActive_ && !drTold_) { drTold_ = true; bus::postMessage("GPS lost - dead reckoning (drag)"); }
}

// ---------------------------------------------------------------- motor health from the RPM telemetry
// Bidirectional DShot (or serial ESC telemetry) gives each motor's RPM. Per motor: RPM per unit of command,
// 1 s average, against the median of the four. A motor turning 15 % slower than the others for the same
// command for 2 s: a failing motor / ESC, a bearing ("Motor N slow"). 12 % faster: it is not loaded -
// a damaged or lost prop ("prop damaged?"). Commanded above 25 % but under a third of the expected RPM
// for 0.3 s: stalled / desync, at once.
void Commander::rpmHealth(uint32_t now, float dt, const MotorOutputs& mo, bool steady) {
  float rpm[4];
  bool have = true;
  for (int i = 0; i < 4; i++) {
    rpm[i] = bus::bidirRpm[i];
    if (!(rpm[i] >= 0) || !hal::motorsBidirActive()) have = false;
  }
  if (!have) {
    EscTelemetry t = bus::esc.get();
    have = true;
    for (int i = 0; i < 4; i++) {
      rpm[i] = t.rpm[i];
      if (!t.lastUpdateMs[i] || now - t.lastUpdateMs[i] > 300 || !(rpm[i] >= 0)) have = false;
    }
  }
  if (!have) { rpmSlowSince_ = rpmFreeSince_ = rpmStallSince_ = 0; return; }
  // RPM and command averaged separately (1 s) and divided: the motor's lag behind a changing command
  // cancels out (a ratio of instantaneous values swings with every correction)
  float k = dt / (1.0f + dt), r[4];
  for (int i = 0; i < 4; i++) {
    if (rpmAvg_[i] <= 0) { rpmAvg_[i] = rpm[i]; cmdAvg_[i] = mo.out[i]; }
    rpmAvg_[i] += (rpm[i] - rpmAvg_[i]) * k;
    cmdAvg_[i] += (mo.out[i] - cmdAvg_[i]) * k;
    r[i] = rpmAvg_[i] / fmaxf(cmdAvg_[i], 0.05f);
  }
  // median of four: mean of the middle two
  float s[4] = {r[0], r[1], r[2], r[3]};
  for (int a = 0; a < 3; a++)
    for (int b = a + 1; b < 4; b++)
      if (s[b] < s[a]) { float x = s[a]; s[a] = s[b]; s[b] = x; }
  float med = 0.5f * (s[1] + s[2]);
  if (med <= 0) return;
  int lo = 0, hi = 0, stall = -1;
  for (int i = 1; i < 4; i++) { if (r[i] < r[lo]) lo = i; if (r[i] > r[hi]) hi = i; }
  for (int i = 0; i < 4; i++)
    if (mo.out[i] > 0.25f && rpm[i] < med * mo.out[i] * 0.33f) stall = i;  // instantaneous RPM
  char m[48];
  if (stall >= 0) {
    if (!rpmStallSince_) rpmStallSince_ = now;
    if (now - rpmStallSince_ > 300 && (!rpmStallToldMs_ || now - rpmStallToldMs_ > 5000)) {
      rpmStallToldMs_ = now;
      snprintf(m, sizeof(m), "Motor %d stalled / desync!", stall + 1);
      bus::postMessage(m);
    }
  } else rpmStallSince_ = 0;
  rpmStall_ = rpmStallSince_ && now - rpmStallSince_ > 300;
  if (steady && r[lo] < med * 0.85f) { if (!rpmSlowSince_) rpmSlowSince_ = now; }
  else if (r[lo] > med * 0.9f) rpmSlowSince_ = 0;
  if (rpmSlowSince_ && now - rpmSlowSince_ > 2000 && (!rpmSlowToldMs_ || now - rpmSlowToldMs_ > 30000)) {
    rpmSlowToldMs_ = now;
    snprintf(m, sizeof(m), "Motor %d slow (RPM -%d%%)", lo + 1, (int)((1.0f - r[lo] / med) * 100.0f + 0.5f));
    bus::postMessage(m);
  }
  if (steady && r[hi] > med * 1.12f) { if (!rpmFreeSince_) rpmFreeSince_ = now; }
  else if (r[hi] < med * 1.06f) rpmFreeSince_ = 0;
  if (rpmFreeSince_ && now - rpmFreeSince_ > 2000 && (!rpmFreeToldMs_ || now - rpmFreeToldMs_ > 30000)) {
    rpmFreeToldMs_ = now;
    snprintf(m, sizeof(m), "Motor %d spins free: prop damaged?", hi + 1);
    bus::postMessage(m);
  }
  rpmWarn_ = rpmStall_ || (rpmSlowSince_ && now - rpmSlowSince_ > 2000) || (rpmFreeSince_ && now - rpmFreeSince_ > 2000);
}

// ---------------------------------------------------------------- weak motor / thrust loss
// One motor that runs much harder than the others for 3 s in steady flight (2 s average): a weak motor,
// a damaged prop or a failing ESC. All motors near full power for 1.5 s while sinking: thrust loss
// (ArduPilot). Warnings only: the pilot decides (land now).
void Commander::motorHealth(uint32_t now, float dt) {
  const Config& c = cfg();
  if (!c.motor_warn || !armed_ || !airborne_ || mode_ == FlightMode::TURTLE || mode_ == FlightMode::FLIP) {
    motorBadSince_ = thrustBadSince_ = 0;
    motorWarn_ = thrustWarn_ = rpmWarn_ = rpmStall_ = false;
    rpmSlowSince_ = rpmFreeSince_ = rpmStallSince_ = 0;
    for (int i = 0; i < 4; i++) rpmAvg_[i] = cmdAvg_[i] = 0;
    return;
  }
  MotorOutputs mo = bus::motors.get();
  float k = dt / (2.0f + dt), mean = 0, inst = 0;
  for (int i = 0; i < 4; i++) {
    motorAvg_[i] += (mo.out[i] - motorAvg_[i]) * k;
    mean += motorAvg_[i] * 0.25f;
    inst += mo.out[i] * 0.25f;
  }
  int worst = 0;
  for (int i = 1; i < 4; i++) if (motorAvg_[i] > motorAvg_[worst]) worst = i;
  float dev = motorAvg_[worst] - mean;
  bool steady = fabsf(att_.rateDps.x) < 60 && fabsf(att_.rateDps.y) < 60 && fabsf(att_.rateDps.z) < 60 &&
                att_.q.cosTilt() > 0.87f;
  if (steady && mean > 0.15f && dev > 0.10f) { if (!motorBadSince_) motorBadSince_ = now; }
  else if (dev < 0.07f) motorBadSince_ = 0;
  motorWarn_ = motorBadSince_ && now - motorBadSince_ > 3000;
  if (motorWarn_ && (!motorToldMs_ || now - motorToldMs_ > 30000)) {
    motorToldMs_ = now;
    char m[48];
    snprintf(m, sizeof(m), "Motor %d weak? (+%d%% output)", worst + 1, (int)(dev * 100.0f + 0.5f));
    bus::postMessage(m);
  }
  rpmHealth(now, dt, mo, steady && mean > 0.15f);
  // full power (motors / the throttle asked near the top), sinking, and not even slowing the descent: a
  // full-throttle catch out of a dive decelerates hard and does not count
  bool full = inst > 0.8f || lastThrottle_ > 0.8f;
  bool notSlowing = -att_.accEarth.z < 1.0f;
  if (navEst_.altValid && full && navEst_.vz < -0.5f && notSlowing) { if (!thrustBadSince_) thrustBadSince_ = now; }
  else thrustBadSince_ = 0;
  thrustWarn_ = thrustBadSince_ && now - thrustBadSince_ > 1500;
  if (thrustWarn_ && (!thrustToldMs_ || now - thrustToldMs_ > 10000)) {
    thrustToldMs_ = now;
    bus::postMessage("Thrust loss: full power, still sinking");
  }
}

}  // namespace app
