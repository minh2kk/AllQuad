// AllQuad - commander: the control task (default 250 Hz).
// Owns estimation (Kalman), arming & pre-arm checks, failsafes (RC / battery / geofence),
// flight-mode logic, navigation and the outer control loops. Output: ControlSetpoint.
#pragma once
#include "../bus/bus.h"
#include "../control/autotune.h"
#include "../control/pos_control.h"
#include "../estimation/ekf.h"
#include "../estimation/gps_heading.h"
#include "../estimation/nav_estimator.h"
#include "../navigation/navigator.h"
#include "../safety/safety.h"

namespace app {

class Commander {
 public:
  void init();
  void run(uint32_t nowMs, float dt);

 private:
  // the settings the flight code uses: bus::config, or a copy with the first-flight limits (beginner 1)
  const fc::Config& cfg() const { return bus::config.beginner ? eff_ : bus::config; }
  fc::Config eff_{};
  void reconfigure();
  void fuseSensors(uint32_t nowMs, float dt);
  void handleRequests(uint32_t nowMs);
  uint32_t computeArmBlockers(bool rcOk);
  void arm(uint32_t nowMs, bool turtle);
  void disarm(const char* why);
  fc::FlightMode pilotMode(bool rcOk) const;
  fc::FlightMode validate(fc::FlightMode m) const;
  void updateFailsafes(bool rcOk, uint32_t nowMs);
  void control(float dt, uint32_t nowMs);
  void publish(uint32_t nowMs);

  uint32_t cfgGen_ = 0;
  fc::NavEstimator est_;
  // navigation EKF (est_type 1): position / velocity / height / heading for control; est_ keeps
  // running for the rangefinder height, the home position and as the fallback
  fc::Ekf ekf_;
  fc::NavEstimate navEst_{};   // what the controllers use (est_ or EKF)
  uint32_t seqAtt_ = 0, ekfZeroVelMs_ = 0, ekfTiltBadSince_ = 0;
  uint16_t ekfTiltResets_ = 0, ekfYawResets_ = 0;
  fc::GpsHeading gpsRef_;          // GPS-acceleration heading, as an independent referee for the EKF heading
  uint32_t ekfYawBadSince_ = 0, steadySince_ = 0, ekfTiltFuseMs_ = 0;
  bool attNew_ = false;
  bool imuNew_ = false;
  fc::Vec3 imuDAng_, imuDVel_;
  float imuDt_ = 0;
  double ekfHomeLat_ = 0, ekfHomeLon_ = 0;
  bool ekfHomeKnown_ = false, ekfYawOk_ = false, ekfGpsYawInit_ = false;
  uint16_t ekfFaultsPrev_ = 0;
  void ekfFuse(uint32_t nowMs, const fc::GpsData* gps, const fc::BaroSample* baro, bool newMag);
  void buildNavEstimate(uint32_t nowMs);
  // RTH / GPS rescue progress check
  float rthBestDist_ = 1e9f;
  uint32_t rthProgressMs_ = 0;
  bool rescueLanding_ = false;
  // launch control
  bool launched_ = false;
  bool launchWaiting_ = false;
  // vibration warning (held for 2 s)
  uint32_t vibeWarnUntilMs_ = 0;
  uint32_t clipsSeen_ = 0, clipWarnUntilMs_ = 0, clipsFlight_ = 0;  // accelerometer clipping
  bool clipTold_ = false;
  nav::Navigator nav_;
  fc::AltitudeControl alt_;
  fc::PositionControl pos_;
  fc::LandDetector land_;

  // inputs (latest snapshots)
  fc::RcInput rc_{};
  fc::AttitudeState att_{};
  fc::BatteryState bat_{};
  fc::MagSample mag_{};
  uint32_t seqBaro_ = 0, seqGps_ = 0, seqRange_ = 0, seqFlow_ = 0, seqMag_ = 0;
  bool baroSeen_ = false, magSeen_ = false;

  // state
  bool armed_ = false, airborne_ = false, turtle_ = false;
  bool armSwPrev_ = false, armSwSeenLow_ = false;
  uint32_t armTimeMs_ = 0;
  uint32_t blockers_ = 0;
  fc::FlightMode mode_ = fc::FlightMode::ACRO, prevMode_ = fc::FlightMode::ACRO;
  fc::FlightMode gcsMode_ = fc::FlightMode::COUNT;  // COUNT = no GCS override
  uint32_t switchState_ = 0, gcsSwitchState_ = 0;
  uint32_t boxes_ = 0;  // active mode boxes (ModeBox bits)

  // failsafes
  bool rcFailsafe_ = false;
  uint32_t rcRecoverMs_ = 0, failsafeStartMs_ = 0;
  fc::FlightMode actionMode_ = fc::FlightMode::COUNT;  // latched battery / fence action
  bool batFsDone_ = false;
  uint32_t fenceSuppressUntilMs_ = 0, actionSwitchState_ = 0;
  uint16_t warnings_ = 0;

  // control
  float altTarget_ = 0;
  float lastThrottle_ = 0;
  float yawHold_ = 0;
  uint32_t landedLowThrSinceMs_ = 0;
  nav::NavOutput navOut_{};

  // heading (compass or GPS), wind, hover learning, surface tracking
  fc::GpsHeading gpsHeading_;
  fc::WindEstimator wind_;
  float yawCorr_ = 0, yawNav_ = 0;
  fc::Vec3 accNav_;
  bool headingOk_ = false, useGpsHeading_ = false;
  uint8_t headingSrc_ = 0;
  float hoverLearn_ = 0;
  uint32_t hoverSamples_ = 0;
  float aglTarget_ = 0;
  bool aglHold_ = false;
  uint16_t lastGlitchCount_ = 0;
  // GPS-derived kinematic acceleration for the AHRS
  fc::Vec3 accKin_;
  float gpsVelPrev_[2] = {0, 0};
  uint32_t gpsVelPrevMs_ = 0, accKinMs_ = 0;
  void learnHover(float throttle, float cosTilt, float climbCmd, float dt);
  float surfaceTarget(float baroTarget, float climb, float dt);
  void saveHover();
  void finishAutotune(bool keep);
  fc::AutoTune autotune_;
  bool atAnnounced_ = false;

  // acc calibration
  int accCalCount_ = -1;
  fc::Vec3 accCalSum_;
  // 6-point accelerometer calibration (ArduPilot): one average per side, bias + scale per axis
  bool acc6_ = false;
  uint32_t acc6StartMs_ = 0;
  int acc6Side_ = -1, acc6Count_ = 0;
  fc::Vec3 acc6Sum_, acc6Mean_[6];
  void acc6Update(uint32_t nowMs);
  // crash detection
  uint32_t crashSinceMs_ = 0, impactMs_ = 0;
  float tgtTiltDeg_ = 180.0f;  // tilt the self-levelling asks for (180 = not self-levelling)
  void crashCheck(uint32_t nowMs);
  // GCS failsafe
  bool gcsFsDone_ = false;
  // SmartRTL path recording
  uint32_t pathMs_ = 0;
  uint32_t guidedSeq_ = 0;
  void guidedInput(uint32_t nowMs);
  // logic conditions (iNav programming framework)
  uint32_t logicMs_ = 0;
  uint8_t logicRes_ = 0, logicPrev_ = 0;
  uint32_t logicBoxes_ = 0;
  uint16_t logicRc_[16] = {0};
  bool logicNoArm_ = false;
  int32_t logicValue(uint8_t type, int16_t v) const;
  void logicUpdate(uint32_t nowMs);
  // profiles
  uint8_t profile_ = 0;
};

}  // namespace app
