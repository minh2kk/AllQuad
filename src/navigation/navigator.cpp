#include "navigator.h"

#include "../estimation/nav_estimator.h"

namespace nav {

using fc::FlightMode;
using fc::NavPhase;

enum WpKind : uint8_t { WP_NEXT = 0, WP_LAND = 1, WP_ORBIT_AFTER = 2, WP_FOREVER = 3 };

void Navigator::holdHere(const fc::NavEstimate& est) {
  tgtN_ = est.n;
  tgtE_ = est.e;
  tgtAlt_ = est.alt;
}

bool Navigator::toNed(const fc::NavEstimate& est, int32_t lat, int32_t lon, float& n, float& e) const {
  if ((lat == 0 && lon == 0) || !est.homeSet) return false;
  fc::NavEstimator::llaToNed(lat * 1e-7, lon * 1e-7, est.homeLat, est.homeLon, n, e);
  return true;
}

void SmartPath::record(float n, float e, float alt, float spacing) {
  if (spacing <= 0) return;
  if (n_ > 0) {
    float dn = n - pn_[n_ - 1], de = e - pe_[n_ - 1];
    if (dn * dn + de * de < spacing * spacing && fabsf(alt - pa_[n_ - 1]) < spacing) return;
    // loop pruning: back near an older point -> forget everything after it
    for (int i = 0; i < n_ - 2; i++) {
      float qn = n - pn_[i], qe = e - pe_[i];
      if (qn * qn + qe * qe < spacing * spacing * 0.5f && fabsf(alt - pa_[i]) < spacing) { n_ = i + 1; return; }
    }
  }
  if (n_ >= MAX) {  // full: keep every second point (the first one is the take-off point)
    int k = 1;
    for (int i = 2; i < n_; i += 2) { pn_[k] = pn_[i]; pe_[k] = pe_[i]; pa_[k] = pa_[i]; k++; }
    n_ = k;
  }
  pn_[n_] = n; pe_[n_] = e; pa_[n_] = alt;
  n_++;
}

void Navigator::setGuided(float n, float e, float alt) {
  gN_ = n; gE_ = e; gAlt_ = alt;
  guidedSet_ = true;
  if (mode_ == FlightMode::GUIDED) {
    tgtN_ = n; tgtE_ = e; tgtAlt_ = alt;
    phase_ = NavPhase::GUIDED;
  }
}

void Navigator::begin(FlightMode m, const fc::NavEstimate& est, float yaw, bool headingOk) {
  mode_ = m;
  tgtYaw_ = yaw;
  speed_ = cfg_->wp_speed;
  wpKind_ = WP_NEXT;
  switch (m) {
    case FlightMode::POSHOLD:
      holdHere(est);
      phase_ = NavPhase::POSHOLD;
      break;
    case FlightMode::CRUISE: {
      holdHere(est);
      crsCourse_ = yaw;
      crsSpeed_ = fc::constrainf(est.vn * cosf(yaw) + est.ve * sinf(yaw), 0.0f, cfg_->pos_max_speed);
      phase_ = NavPhase::CRUISE;
      break;
    }
    case FlightMode::RTH:
      startRth(est, headingOk);
      break;
    case FlightMode::MISSION:
      wpIndex_ = 0;
      roi_ = yawFixed_ = false;
      for (int i = 0; i < MISSION_MAX; i++) jumpLeft_[i] = -1;
      holdHere(est);
      loadWaypoint(est);
      break;
    case FlightMode::LAND:
    case FlightMode::FAILSAFE:
      holdHere(est);
      startLanding(est, est.n, est.e);
      break;
    case FlightMode::GUIDED:
      holdHere(est);
      if (guidedSet_) { tgtN_ = gN_; tgtE_ = gE_; tgtAlt_ = gAlt_; }
      phase_ = NavPhase::GUIDED;
      break;
    case FlightMode::SMART_RTL:
      holdHere(est);
      srtlIdx_ = path_.count() - 1;
      if (srtlIdx_ < 1 || !est.gpsValid) { startRth(est, headingOk); break; }  // no path: plain RTH
      phase_ = NavPhase::SRTL_RETURN;
      break;
    case FlightMode::BRAKE: {
      // the stopping point at twice the normal deceleration: the position loop brakes straight to it
      float v = sqrtf(est.vn * est.vn + est.ve * est.ve), a = fmaxf(cfg_->pos_max_accel * 2.0f, 2.0f);
      holdHere(est);
      if (v > 0.2f) {
        float d = v * v / (2.0f * a);
        tgtN_ += est.vn / v * d;
        tgtE_ += est.ve / v * d;
      }
      phase_ = NavPhase::BRAKE;
      break;
    }
    case FlightMode::CIRCLE: {
      holdHere(est);
      orbR_ = fmaxf(cfg_->circle_radius, 2.0f);
      orbN_ = est.n + orbR_ * cosf(yaw);
      orbE_ = est.e + orbR_ * sinf(yaw);
      crcSpeed_ = cfg_->circle_speed;
      orbAngle_ = atan2f(est.e - orbE_, est.n - orbN_);
      phase_ = NavPhase::CIRCLE;
      break;
    }
    case FlightMode::FOLLOW:
      holdHere(est);
      if (cfg_->follow_alt > 0) tgtAlt_ = cfg_->follow_alt;
      phase_ = NavPhase::FOLLOW;
      break;
    default:
      phase_ = NavPhase::IDLE;
      break;
  }
}

void Navigator::startRth(const fc::NavEstimate& est, bool headingOk) {
  holdHere(est);
  if (!est.homeSet || !est.gpsValid) {  // cannot navigate home: land where we are
    startLanding(est, est.n, est.e);
    return;
  }
  // destination: home, or the nearest rally point that is closer than home (ArduPilot rally / iNav safehome)
  rtlN_ = rtlE_ = 0;
  rtlAlt_ = cfg_->rth_alt;
  rtlRally_ = -1;
  float best = sqrtf(est.n * est.n + est.e * est.e);
  if (cfg_->rally_max_m > 0) {
    for (int i = 0; i < rallyCount(); i++) {
      MissionItem it;
      float n, e;
      if (!rallyGet(i, it) || !toNed(est, it.lat, it.lon, n, e)) continue;
      float d = sqrtf((n - est.n) * (n - est.n) + (e - est.e) * (e - est.e));
      if (d < best && d < cfg_->rally_max_m) {
        best = d;
        rtlN_ = n;
        rtlE_ = e;
        rtlAlt_ = it.alt > 0.5f ? it.alt : cfg_->rth_alt;
        rtlRally_ = i;
      }
    }
  }
  tgtAlt_ = fmaxf(est.alt, rtlAlt_);
  // a height ceiling (geofence): the return flies under it, also when RTH started because of it
  if (cfg_->fence_enable && cfg_->fence_alt_max > 3.0f) tgtAlt_ = fminf(tgtAlt_, cfg_->fence_alt_max - 2.0f);
  phaseStartMs_ = nowMs_;
  if (!headingOk) {  // no compass and GPS heading unknown: fly a few forward/back legs to learn it
    phase_ = NavPhase::RTH_ALIGN;
    return;
  }
  phase_ = cfg_->rth_climb_first ? NavPhase::RTH_CLIMB : NavPhase::RTH_RETURN;
}

void Navigator::startLanding(const fc::NavEstimate& est, float n, float e) {
  tgtN_ = n;
  tgtE_ = e;
  tgtAlt_ = est.alt;
  phase_ = NavPhase::LANDING;
  phaseStartMs_ = nowMs_;
}

void Navigator::loadWaypoint(const fc::NavEstimate& est) {
  for (int guard = 0; guard < MISSION_MAX * 2; guard++) {
    if (!get(wpIndex_, wp_)) {
      phase_ = NavPhase::MISSION_DONE;  // hold the last target
      return;
    }
    float n, e;
    switch (wp_.cmd) {
      case CMD_CHANGE_SPEED:
        if (wp_.param2 > 0.1f) speed_ = fc::constrainf(wp_.param2, 0.5f, cfg_->pos_max_speed);
        wpIndex_++;
        continue;
      case CMD_DO_JUMP: {
        int target = (int)wp_.param1, reps = (int)wp_.param2;
        if (target < 0 || target >= count() || target == wpIndex_) { wpIndex_++; continue; }
        if (reps < 0) { wpIndex_ = target; continue; }
        int16_t& left = jumpLeft_[wpIndex_];
        if (left < 0) left = (int16_t)reps;
        if (left > 0) { left--; wpIndex_ = target; }
        else wpIndex_++;
        continue;
      }
      case CMD_DO_SET_ROI_LOC:
      case CMD_DO_SET_ROI:
        roi_ = toNed(est, wp_.lat, wp_.lon, roiN_, roiE_);
        wpIndex_++;
        continue;
      case CMD_DO_SET_ROI_NONE:
        roi_ = false;
        wpIndex_++;
        continue;
      case CMD_CONDITION_YAW: {
        float a = wp_.param1 * fc::DEG2RAD;
        yawFixedRad_ = wp_.param4 > 0.5f ? fc::wrapPi(tgtYaw_ + (wp_.param3 < 0 ? -a : a)) : fc::wrapPi(a);
        yawFixed_ = true;
        roi_ = false;
        wpIndex_++;
        continue;
      }
      case CMD_TAKEOFF:
        tgtAlt_ = wp_.alt > 1.0f ? wp_.alt : 2.0f;
        phase_ = NavPhase::WP_TAKEOFF;
        return;
      case CMD_RTL:
        startRth(est, true);
        return;
      case CMD_LOITER_TURNS: {
        float cn = est.n, ce = est.e;
        toNed(est, wp_.lat, wp_.lon, cn, ce);
        orbN_ = cn;
        orbE_ = ce;
        orbR_ = fmaxf(fabsf(wp_.param3), 2.0f);
        orbDir_ = wp_.param3 < 0 ? -1 : 1;
        orbTotal_ = fmaxf(wp_.param1, 0.25f) * 2.0f * fc::PI_F;
        // fly to the nearest point of the circle first
        float dn = est.n - cn, de = est.e - ce, d = sqrtf(dn * dn + de * de);
        if (d < 0.5f) { dn = 1; de = 0; d = 1; }
        tgtN_ = cn + dn / d * orbR_;
        tgtE_ = ce + de / d * orbR_;
        if (wp_.alt > 0.5f) tgtAlt_ = wp_.alt;
        wpKind_ = WP_ORBIT_AFTER;
        phase_ = NavPhase::WP_ENROUTE;
        return;
      }
      case CMD_LAND:
      case CMD_WAYPOINT:
      case CMD_LOITER_TIME:
      case CMD_LOITER_UNLIM:
      default:
        if (toNed(est, wp_.lat, wp_.lon, n, e)) { tgtN_ = n; tgtE_ = e; }
        if (wp_.alt > 0.5f) tgtAlt_ = wp_.alt;
        wpKind_ = wp_.cmd == CMD_LAND ? WP_LAND : (wp_.cmd == CMD_LOITER_UNLIM ? WP_FOREVER : WP_NEXT);
        phase_ = NavPhase::WP_ENROUTE;
        return;
    }
  }
  phase_ = NavPhase::MISSION_DONE;  // jump loop without a navigation item
}

void Navigator::update(const fc::NavEstimate& est, float yaw, float rollStick, float pitchStick, float yawStick,
                       float climbStick, bool landed, bool headingOk, float dt, uint32_t nowMs, NavOutput& o) {
  const fc::Config& c = *cfg_;
  nowMs_ = nowMs;
  o.horizontal = est.posValid;
  o.ffN = o.ffE = 0;
  o.accFfN = o.accFfE = 0;
  o.maxSpeed = c.pos_max_speed;
  o.climbFF = 0;
  o.maxUp = c.climb_rate_max;
  o.maxDown = c.climb_rate_max;
  o.yawControl = false;
  o.landing = false;
  o.attOverride = false;
  o.yawStickFree = false;

  float dn = tgtN_ - est.n, de = tgtE_ - est.e;
  float dist = sqrtf(dn * dn + de * de);
  auto faceTravel = [&](float tn, float te) {
    if (c.nav_yaw_mode == 1 && dist > 3.0f) {
      tgtYaw_ = atan2f(te - est.e, tn - est.n);
      o.yawControl = true;
    }
  };
  // mission heading: region of interest > fixed heading > direction of travel
  auto missionYaw = [&](float tn, float te) {
    float rn = roiN_ - est.n, re = roiE_ - est.e;
    if (roi_ && rn * rn + re * re > 4.0f) { tgtYaw_ = atan2f(re, rn); o.yawControl = true; }
    else if (yawFixed_) { tgtYaw_ = yawFixedRad_; o.yawControl = true; }
    else faceTravel(tn, te);
  };
  auto pilotClimb = [&]() {
    float climb = climbStick * c.climb_rate_max;
    tgtAlt_ = fc::constrainf(tgtAlt_ + climb * dt, est.alt - 2.0f, est.alt + 2.0f);
    o.climbFF = climb;
  };

  switch (phase_) {
    case NavPhase::POSHOLD: {
      if (fabsf(rollStick) > 0.05f || fabsf(pitchStick) > 0.05f) {
        float fwd = pitchStick * c.pos_max_speed, right = rollStick * c.pos_max_speed;
        float cy = cosf(yaw), sy = sinf(yaw);
        o.ffN = fwd * cy - right * sy;
        o.ffE = fwd * sy + right * cy;
        tgtN_ = est.n + est.vn * 0.3f;
        tgtE_ = est.e + est.ve * 0.3f;
      }
      pilotClimb();
      o.yawStickFree = true;
      break;
    }
    case NavPhase::CRUISE: {
      if (fabsf(yawStick) > 0.03f) crsCourse_ = fc::wrapPi(crsCourse_ + yawStick * 90.0f * fc::DEG2RAD * dt);
      if (fabsf(pitchStick) > 0.05f)
        crsSpeed_ = fc::constrainf(crsSpeed_ + pitchStick * c.pos_max_accel * dt, 0.0f, c.pos_max_speed);
      float lateral = fabsf(rollStick) > 0.05f ? rollStick * c.pos_max_speed * 0.5f : 0.0f;
      float cc = cosf(crsCourse_), sc = sinf(crsCourse_);
      float vN = crsSpeed_ * cc - lateral * sc, vE = crsSpeed_ * sc + lateral * cc;
      // a carrot that moves along the course: wind cannot push the craft off the track
      tgtN_ += vN * dt;
      tgtE_ += vE * dt;
      float cn = tgtN_ - est.n, ce = tgtE_ - est.e, cd = sqrtf(cn * cn + ce * ce);
      if (cd > 5.0f) { tgtN_ = est.n + cn / cd * 5.0f; tgtE_ = est.e + ce / cd * 5.0f; }
      o.ffN = vN;
      o.ffE = vE;
      tgtYaw_ = crsCourse_;
      o.yawControl = true;
      pilotClimb();
      break;
    }
    case NavPhase::RTH_ALIGN: {
      // forward 2.5 s, brake 2.5 s: the accelerations let the GPS heading estimator converge
      uint32_t t = nowMs - phaseStartMs_;
      o.attOverride = true;
      o.horizontal = false;
      o.ovRollDeg = 0;
      o.ovPitchDeg = (t / 2500) % 2 == 0 ? -12.0f : 12.0f;
      if (headingOk) startRth(est, true);
      else if (t > 30000) startLanding(est, est.n, est.e);
      break;
    }
    case NavPhase::RTH_CLIMB:
      if (est.alt >= tgtAlt_ - 1.0f) { phase_ = NavPhase::RTH_RETURN; phaseStartMs_ = nowMs; }
      break;
    case NavPhase::RTH_RETURN:
      tgtN_ = rtlN_; tgtE_ = rtlE_;
      dn = tgtN_ - est.n; de = tgtE_ - est.e;
      dist = sqrtf(dn * dn + de * de);
      o.maxSpeed = c.rth_speed;
      faceTravel(tgtN_, tgtE_);
      if (dist < c.rth_home_radius) { phase_ = NavPhase::RTH_LOITER; phaseStartMs_ = nowMs; }
      break;
    case NavPhase::RTH_LOITER:
      if (c.rth_land && (nowMs - phaseStartMs_) > (uint32_t)(c.rth_loiter_s * 1000.0f)) startLanding(est, rtlN_, rtlE_);
      break;
    case NavPhase::LANDING:
    case NavPhase::LANDED: {
      float h = est.aglValid ? est.agl : est.alt;
      float v = h > c.land_slow_alt ? c.land_speed : c.land_slow_speed;
      tgtAlt_ = est.alt;
      o.climbFF = -v;
      o.maxDown = v;
      o.landing = true;
      if (landed) phase_ = NavPhase::LANDED;
      break;
    }
    case NavPhase::WP_TAKEOFF:
      if (est.alt > tgtAlt_ - 0.5f) { wpIndex_++; loadWaypoint(est); }
      break;
    case NavPhase::WP_ENROUTE:
      o.maxSpeed = speed_;
      missionYaw(tgtN_, tgtE_);
      if (dist < c.wp_radius && fabsf(est.alt - tgtAlt_) < 1.5f) {
        if (wpKind_ == WP_LAND) {
          startLanding(est, tgtN_, tgtE_);
        } else if (wpKind_ == WP_ORBIT_AFTER) {
          orbAngle_ = atan2f(est.e - orbE_, est.n - orbN_);
          orbDone_ = 0;
          phase_ = NavPhase::WP_ORBIT;
        } else if (wpKind_ == WP_FOREVER || wp_.param1 > 0.0f) {
          phase_ = NavPhase::WP_HOLD;
          phaseStartMs_ = nowMs;
        } else {
          wpIndex_++;
          loadWaypoint(est);
        }
      }
      break;
    case NavPhase::WP_ORBIT: {
      // circle: the target sits on the circle at the craft's own angle, so tracking lag cannot
      // accumulate into a radius error; progress is measured on the craft's angle
      float r = orbR_;
      float v = fminf(speed_, sqrtf(c.pos_max_accel * 0.35f * r));
      float w = v / r;
      float ang = atan2f(est.e - orbE_, est.n - orbN_);
      float step = fc::wrapPi(ang - orbAngle_) * orbDir_;
      if (step > 0) orbDone_ += step;
      orbAngle_ = ang;
      tgtN_ = orbN_ + r * cosf(ang);  // radial correction only; the feed-forward carries the craft around
      tgtE_ = orbE_ + r * sinf(ang);
      float ca = cosf(ang), sa = sinf(ang);
      o.ffN = -orbDir_ * v * sa;
      o.ffE = orbDir_ * v * ca;
      o.accFfN = -v * w * ca;  // centripetal, towards the centre
      o.accFfE = -v * w * sa;
      o.maxSpeed = v + 2.0f;
      if (roi_ || yawFixed_) missionYaw(tgtN_, tgtE_);
      else { tgtYaw_ = atan2f(orbE_ - est.e, orbN_ - est.n); o.yawControl = true; }  // nose to the centre
      if (orbDone_ >= orbTotal_) { wpIndex_++; loadWaypoint(est); }
      break;
    }
    case NavPhase::WP_HOLD:
      missionYaw(tgtN_, tgtE_);
      if (wpKind_ != WP_FOREVER && (nowMs - phaseStartMs_) > (uint32_t)(wp_.param1 * 1000.0f)) {
        wpIndex_++;
        loadWaypoint(est);
      }
      break;
    case NavPhase::GUIDED:
      o.maxSpeed = c.wp_speed;
      faceTravel(tgtN_, tgtE_);
      break;
    case NavPhase::SRTL_RETURN: {
      float pn, pe, pa;
      if (!path_.get(srtlIdx_, pn, pe, pa)) { startRth(est, headingOk); break; }
      tgtN_ = pn; tgtE_ = pe;
      tgtAlt_ = fmaxf(pa, 2.0f);
      dn = tgtN_ - est.n; de = tgtE_ - est.e;
      dist = sqrtf(dn * dn + de * de);
      o.maxSpeed = c.rth_speed;
      // carrot ahead: the next point is fed forward so the craft does not stop at every breadcrumb
      float r = fmaxf(c.wp_radius, 2.0f);
      if (dist < r && fabsf(est.alt - tgtAlt_) < 2.0f) {
        srtlIdx_--;
        if (srtlIdx_ < 0) {  // back at the take-off point: hold, then land like RTH (no climb)
          rtlN_ = tgtN_; rtlE_ = tgtE_; rtlRally_ = -1;
          phase_ = NavPhase::RTH_LOITER;
          phaseStartMs_ = nowMs;
          break;
        }
      }
      faceTravel(tgtN_, tgtE_);
      break;
    }
    case NavPhase::BRAKE:
      o.maxSpeed = c.pos_max_speed;
      break;
    case NavPhase::CIRCLE: {
      // pilot: roll changes the speed (and the direction through zero), pitch forward shrinks the radius
      if (fabsf(rollStick) > 0.05f) crcSpeed_ = fc::constrainf(crcSpeed_ + rollStick * 2.0f * dt, -c.pos_max_speed, c.pos_max_speed);
      if (fabsf(pitchStick) > 0.05f) orbR_ = fc::constrainf(orbR_ - pitchStick * 3.0f * dt, 2.0f, 200.0f);
      pilotClimb();
      float r = orbR_;
      float sp = fabsf(crcSpeed_);
      int dir = crcSpeed_ < 0 ? -1 : 1;
      float v = fminf(sp, sqrtf(c.pos_max_accel * 0.35f * r));
      float w = v / r;
      float ang = atan2f(est.e - orbE_, est.n - orbN_);
      tgtN_ = orbN_ + r * cosf(ang);
      tgtE_ = orbE_ + r * sinf(ang);
      float ca = cosf(ang), sa = sinf(ang);
      o.ffN = -dir * v * sa;
      o.ffE = dir * v * ca;
      o.accFfN = -v * w * ca;
      o.accFfE = -v * w * sa;
      o.maxSpeed = v + 2.0f;
      tgtYaw_ = atan2f(orbE_ - est.e, orbN_ - est.n);  // nose to the centre
      o.yawControl = true;
      break;
    }
    case NavPhase::FOLLOW: {
      o.maxSpeed = c.pos_max_speed;
      if (!fFresh_) break;  // leader quiet: hold where we are
      float ln = fN_, le = fE_;
      float dx = est.n - ln, dy = est.e - le, d = sqrtf(dx * dx + dy * dy);
      if (d > 0.5f) { tgtN_ = ln + dx / d * c.follow_dist; tgtE_ = le + dy / d * c.follow_dist; }
      o.ffN = fVn_;
      o.ffE = fVe_;
      if (d > 2.0f) { tgtYaw_ = atan2f(le - est.e, ln - est.n); o.yawControl = true; }  // nose to the leader
      break;
    }
    case NavPhase::MISSION_DONE:
    case NavPhase::IDLE:
      break;
  }
  if (!o.yawControl) tgtYaw_ = yaw;  // remember the current heading for relative CONDITION_YAW

  o.tgtN = tgtN_;
  o.tgtE = tgtE_;
  o.tgtAlt = tgtAlt_;
  o.tgtYaw = tgtYaw_;
}

}  // namespace nav
