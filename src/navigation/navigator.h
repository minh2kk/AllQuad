// AllQuad - autonomous navigation state machine (iNav / ArduPilot style)
//   POSHOLD : hold position, sticks command velocity
//   CRUISE  : hold course and speed (iNav CRUISE / course hold): pitch changes speed, yaw turns
//   RTH     : (no compass: align heading) -> climb -> return to home or the nearest rally point
//             -> loiter -> land
//   MISSION : TAKEOFF / WAYPOINT / LOITER_TIME / LOITER_UNLIM / LOITER_TURNS (orbit) / LAND / RTL
//             + DO_JUMP / CHANGE_SPEED / CONDITION_YAW / DO_SET_ROI
//   LAND    : descend in place, slow near the ground, report touchdown
//   GUIDED  : fly to the point the GCS sends, hold there (ArduPilot GUIDED)
//   SMART_RTL : back along the recorded path to the take-off point, then RTH (ArduPilot SmartRTL)
//   BRAKE   : stop as fast as possible at the stopping point and hold, sticks ignored (ArduPilot BRAKE)
//   CIRCLE  : orbit a centre circle_radius ahead of the nose, nose to the centre; roll changes the speed
//             (through zero: the other way round), pitch the radius, throttle the height (ArduPilot CIRCLE)
//   FOLLOW  : keep follow_dist from the leader (GCS / phone position), nose to it; holds when it goes quiet
#pragma once
#include "../bus/topics.h"
#include "../config/config.h"
#include "mission.h"

namespace nav {

struct NavOutput {
  bool horizontal;      // position controller active
  float tgtN, tgtE;     // position target (NED, m)
  float ffN, ffE;       // velocity feed-forward
  float accFfN, accFfE; // acceleration feed-forward (orbit: centripetal)
  float maxSpeed;
  float tgtAlt;         // altitude target (m above home)
  float climbFF;
  float maxUp, maxDown;
  bool yawControl;      // follow tgtYaw
  float tgtYaw;         // rad
  bool landing;         // in final descent (landing detector armed)
  bool attOverride;     // fly fixed angles instead of the position controller (RTH_ALIGN)
  float ovRollDeg, ovPitchDeg;
  bool yawStickFree;    // pilot yaw stick controls the yaw rate (POSHOLD)
};

// SmartRTL breadcrumbs: a point every srtl_point_m metres while flying with a position fix. When the
// craft comes back near an older point the loop in between is cut out (the way home stays short);
// when the buffer is full every second point is dropped.
class SmartPath {
 public:
#if defined(STM32F4xx) || (defined(AQ_STM32) && !defined(STM32H7xx))
  static constexpr int MAX = 120;
#else
  static constexpr int MAX = 300;
#endif
  void reset() { n_ = 0; }
  void record(float n, float e, float alt, float spacing);
  int count() const { return n_; }
  bool get(int i, float& n, float& e, float& alt) const {
    if (i < 0 || i >= n_) return false;
    n = pn_[i]; e = pe_[i]; alt = pa_[i];
    return true;
  }

 private:
  float pn_[MAX], pe_[MAX], pa_[MAX];
  int n_ = 0;
};

class Navigator {
 public:
  void configure(const fc::Config& c) { cfg_ = &c; }
  // (re)start a navigation mode from the current state
  void begin(fc::FlightMode m, const fc::NavEstimate& est, float yaw, bool headingOk);
  // sticks -1..1: roll/pitch nudge (POSHOLD, CRUISE), yaw (CRUISE turns), climb (POSHOLD/CRUISE altitude)
  void update(const fc::NavEstimate& est, float yaw, float rollStick, float pitchStick, float yawStick,
              float climbStick, bool landed, bool headingOk, float dt, uint32_t nowMs, NavOutput& out);
  fc::NavPhase phase() const { return phase_; }
  int missionIndex() const { return wpIndex_; }
  fc::FlightMode mode() const { return mode_; }
  int rtlTarget() const { return rtlRally_; }  // -1 home, else rally index
  void rthDestination(float& n, float& e) const { n = rtlN_; e = rtlE_; }
  // GUIDED target (NE metres from home, altitude above home); applies at once when in GUIDED
  void setGuided(float n, float e, float alt);
  bool guidedSet() const { return guidedSet_; }
  SmartPath& path() { return path_; }
  // FOLLOW leader: NE metres from home, velocity, fresh = updated within the last 3 s
  void setFollow(float n, float e, float vn, float ve, bool fresh) { fN_ = n; fE_ = e; fVn_ = vn; fVe_ = ve; fFresh_ = fresh; }
  float circleSpeed() const { return crcSpeed_; }
  float circleRadius() const { return orbR_; }

 private:
  void startRth(const fc::NavEstimate& est, bool headingOk);
  void startLanding(const fc::NavEstimate& est, float n, float e);
  void loadWaypoint(const fc::NavEstimate& est);
  void holdHere(const fc::NavEstimate& est);
  bool toNed(const fc::NavEstimate& est, int32_t lat, int32_t lon, float& n, float& e) const;

  const fc::Config* cfg_ = nullptr;
  fc::FlightMode mode_ = fc::FlightMode::ACRO;
  fc::NavPhase phase_ = fc::NavPhase::IDLE;
  float tgtN_ = 0, tgtE_ = 0, tgtAlt_ = 0, tgtYaw_ = 0;
  float speed_ = 5;
  uint32_t phaseStartMs_ = 0, nowMs_ = 0;
  uint8_t wpKind_ = 0;  // what happens when the current waypoint is reached
  int wpIndex_ = 0;
  MissionItem wp_{};
  int16_t jumpLeft_[MISSION_MAX];
  // RTH destination (home or rally point)
  float rtlN_ = 0, rtlE_ = 0, rtlAlt_ = 0;
  int rtlRally_ = -1;
  // yaw in missions: region of interest / fixed heading
  bool roi_ = false, yawFixed_ = false;
  float roiN_ = 0, roiE_ = 0, yawFixedRad_ = 0;
  // orbit
  float orbN_ = 0, orbE_ = 0, orbR_ = 10, orbAngle_ = 0, orbDone_ = 0, orbTotal_ = 0;
  int orbDir_ = 1;
  // cruise
  float crsCourse_ = 0, crsSpeed_ = 0;
  // guided / SmartRTL
  float gN_ = 0, gE_ = 0, gAlt_ = 0;
  bool guidedSet_ = false;
  SmartPath path_;
  int srtlIdx_ = -1;
  // circle / follow
  float crcSpeed_ = 0;
  float fN_ = 0, fE_ = 0, fVn_ = 0, fVe_ = 0;
  bool fFresh_ = false;
};

}  // namespace nav
