// AllQuad - data exchanged between tasks (publish/subscribe "topics").
// Every struct is plain data; one task writes, any task may read a snapshot.
#pragma once
#include <stdint.h>

#include "../common/math3d.h"
#include "../os/os.h"

namespace fc {

constexpr int RC_MAX_CH = 16;
enum RcCh { RC_ROLL = 0, RC_PITCH = 1, RC_THROTTLE = 2, RC_YAW = 3 };

// ---------------------------------------------------------------- inputs
// Age of a timestamp taken by another task. Each task reads the clock once at its start; a higher-priority
// task can publish a stamp a millisecond newer before this one reads the topic, and plain `now - stamp`
// then wraps to ~4e9 ("lost"). Negative ages count as 0.
inline uint32_t ageMs(uint32_t now, uint32_t stamp) {
  int32_t d = (int32_t)(now - stamp);
  return d < 0 ? 0u : (uint32_t)d;
}

struct RcInput {
  uint16_t ch[RC_MAX_CH];  // microseconds, 1000..2000 nominal
  uint8_t count;
  bool failsafeFlag;       // receiver reported failsafe
  uint32_t lastFrameMs;
  uint8_t linkQuality;     // 0..100
  int8_t rssiDbm;
  bool everReceived;
};

struct GpsData {
  bool fix3d;
  uint8_t sats;
  double lat, lon;          // deg
  float altMsl;             // m
  float velN, velE, velD;   // m/s
  float groundSpeed, courseDeg;
  float hAcc, vAcc;         // m
  uint32_t timeMs;
  float sAcc;               // speed accuracy (m/s, 1 sigma; 0 = not reported)
};

struct BaroSample { float altM; float pressurePa; float tempC; bool healthy; };
struct MagSample { Vec3 field; bool healthy; };                 // calibrated, body FRD
struct RangeSample { float distM; bool valid; uint8_t quality; };
struct FlowSample { float rateX, rateY; uint8_t quality; float dt; };  // rad/s (raw, sensor frame)

struct EscTelemetry {
  float rpm[4];        // mechanical RPM
  float tempC[4];
  float voltage, current, mah;
  uint32_t lastUpdateMs[4];
};

struct BatteryState {
  float voltage, current, mahDrawn;
  float cellVoltage;
  uint8_t cells, percent;
  bool warn, critical, capacityLow;
  // resting voltage: the sag under load added back with the learned pack resistance (ohm with a
  // current sensor, else volts per unit motor load), the charge estimate from it when there is no
  // capacity / current counting
  float restCell;      // V per cell (0 until learned)
  float resistance;    // ohm (current sensor) or V per unit load (motor outputs)
  bool restLearned, loadIsCurrent;
  float percentF;      // percent as a float (0..100, for the smart RTH rate)
};

// ---------------------------------------------------------------- estimates
struct AttitudeState {
  Quat q;
  float roll, pitch, yaw;   // rad
  Vec3 rateDps;             // filtered body rates
  Vec3 accEarth;            // NED, gravity removed, averaged since the last publish
  Vec3 accBody;             // filtered specific force
  // integrated since the previous publish (for the navigation EKF): body delta angle (rad),
  // delta velocity (specific force * dt, m/s) and the time they cover
  Vec3 dAng, dVel;
  float dt;
  float vibration;          // accelerometer vibration (m/s^2, high-pass RMS)
  uint32_t accClips;        // accelerometer samples at the end of the range since boot
  bool valid;
  bool gyroCalibrated;
  uint32_t timeUs;
};

struct NavEstimate {
  // vertical (up positive, relative to boot/home ground level)
  float alt, vz;
  bool altValid;
  float agl;                // above ground (rangefinder), valid if aglValid
  bool aglValid;
  // horizontal (local NED relative to home)
  float n, e, vn, ve;
  bool posValid;            // absolute (GPS) or relative (optical flow)
  bool gpsValid;
  bool flowValid;
  bool homeSet;
  double homeLat, homeLon;
  float homeAltMsl;
  float eph;                // horizontal accuracy estimate (m)
  bool gpsGlitch;           // GPS position rejected (jump), navigating on GPS velocity + inertial
  uint16_t glitchCount;
  // filled by the commander
  float yaw;                // navigation heading (rad), compass or GPS-corrected
  uint8_t headingSrc;       // 0 none (gyro only), 1 compass, 2 GPS
  bool headingOk;
  float windN, windE;       // estimated wind (m/s, direction the air moves to)
  float hoverThr;           // learned hover throttle
  float tgtN, tgtE, tgtAlt; // navigation targets (when a nav/alt mode is active)
  bool aglHold;             // altitude controller is holding height above ground
};

// ---------------------------------------------------------------- control
enum class MotorMode : uint8_t { STOPPED = 0, NORMAL, TURTLE };

struct ControlSetpoint {
  float rateDps[3];         // roll, pitch, yaw body rates
  float throttle;           // 0..1 collective
  MotorMode motorMode;
  bool integrate;           // allow I-term accumulation (airborne)
  float turtleStick[2];     // roll/pitch stick for turtle mode
  Vec3 accKin;              // kinematic acceleration (NED, m/s^2) from GPS velocity, for AHRS leveling
  bool accKinValid;
  float sagFactor;          // battery sag compensation: output scale (1 = full battery / off)
  uint8_t profile;          // active PID / rate profile (0-2)
  // low-latency sticks: the fast loop turns the newest RC frame into rates itself instead of waiting
  // for the next commander cycle (bit0 = roll + pitch, ACRO; bit1 = yaw, every mode the pilot yaws in)
  uint8_t stickDirect;
  uint32_t seq;
};

// rate-loop internals for the blackbox / tuning (published by the fast task)
struct PidDebug {
  float p[3], i[3], d[3], f[3];
  Vec3 gyroRaw;        // deg/s before filtering
  float dynNotchHz[3];
};

struct MotorOutputs {
  float out[4];
  bool armed;
};

// ---------------------------------------------------------------- status
enum class FlightMode : uint8_t {
  ACRO = 0, ANGLE, HORIZON, ALTHOLD, POSHOLD, RTH, MISSION, LAND, FAILSAFE, TURTLE, CRUISE, AUTOTUNE,
  GUIDED,     // ArduPilot GUIDED: fly to the point the GCS sends ("fly here" / companion computer)
  SMART_RTL,  // ArduPilot SmartRTL: back along the recorded path, then RTH
  BRAKE,      // ArduPilot BRAKE: stop hard and hold, sticks ignored
  CIRCLE,     // ArduPilot CIRCLE: orbit a point, nose to the centre
  FLIP,       // ArduPilot FLIP: one automatic flip, then back to the mode underneath
  FOLLOW,     // follow the GCS / phone position (MAVLink FOLLOW_TARGET)
  COUNT
};
const char* flightModeName(FlightMode m);

enum ArmCheck : uint32_t {
  ARM_NO_RX = 1u << 0,
  ARM_THROTTLE = 1u << 1,
  ARM_ANGLE = 1u << 2,
  ARM_SWITCH_AT_BOOT = 1u << 3,
  ARM_CALIBRATING = 1u << 4,
  ARM_BATTERY = 1u << 5,
  ARM_NO_IMU = 1u << 6,
  ARM_CLI = 1u << 7,
  ARM_FAILSAFE = 1u << 8,
  ARM_ACC_NOT_CAL = 1u << 9,
  ARM_BARO = 1u << 10,
  ARM_MAG = 1u << 11,
  ARM_GPS = 1u << 12,
  ARM_FENCE = 1u << 13,
  ARM_CPU_LOAD = 1u << 14,
  ARM_NAV_MODE = 1u << 15,    // armed directly into an autonomous mode
  ARM_LOGIC = 1u << 16,       // a logic condition blocks arming
  ARM_IMU_FAIL = 1u << 17,    // IMU failed over to the second chip (check the wiring before flying)
  ARM_IMU_MISMATCH = 1u << 18,  // the two IMUs disagree on gravity and neither is clearly wrong
};
constexpr int ARM_COUNT_BITS = 19;
const char* armCheckName(int bit);

enum Warning : uint32_t {
  WARN_BAT_LOW = 1 << 0,
  WARN_BAT_CRIT = 1 << 1,
  WARN_RX_LOST = 1 << 2,
  WARN_GPS_LOST = 1 << 3,
  WARN_FENCE = 1 << 4,
  WARN_LANDING = 1 << 5,
  WARN_NO_MAG = 1 << 6,
  WARN_GPS_GLITCH = 1 << 7,
  WARN_NO_HEADING = 1 << 8,     // no compass and GPS heading not learned yet
  WARN_VIBRATION = 1 << 9,      // accelerometer vibration above vibe_warn
  WARN_EKF = 1 << 10,           // EKF fault (baro / compass / GPS rejected persistently)
  WARN_LAUNCH = 1 << 11,        // launch control armed: waiting for the throttle
  WARN_RESCUE = 1 << 12,        // GPS rescue / RTH without progress: landing
  WARN_GCS_LOST = 1 << 13,      // GCS heartbeat lost (GUIDED / MISSION)
  WARN_IMU2 = 1 << 14,          // flying on the second IMU
  WARN_RPM_LOST = 1 << 15,      // a motor's RPM telemetry stopped in flight (its RPM notches are off)
  WARN_ACC_CLIP = 1 << 16,      // accelerometer hit the end of its range (hard vibration / loose FC)
  WARN_BEGINNER = 1 << 17,      // first-flight limits active (information)
  WARN_MOTOR = 1 << 18,         // one motor runs much harder than the others (weak motor / damaged prop)
  WARN_THRUST = 1 << 19,        // motors at full power and still sinking (thrust loss)
  WARN_RTH_BAT = 1 << 20,       // the battery left is about what the way home needs
};

// navigation EKF health (published by the control task)
struct EkfReport {
  bool active;              // est_type 1 and initialised
  float trGpsPos, trGpsVel, trHeight, trYaw, trMag, trFlow;
  uint16_t faults;          // EkfFault bits
  uint16_t posResets, velResets, filterResets;
  uint16_t tiltResets;      // EKF tilt reset to the AHRS one (they disagreed > 15 deg for 1 s)
  uint16_t yawResets;       // EKF heading reset to the compass: the GPS referee agreed with the compass, not the EKF
  uint16_t baroResets;      // classic estimator: height reset after the baro was rejected for 2 s
  float posStd, velStd, hgtStd, yawStdDeg;
  uint8_t heightSrc;        // 0 baro, 1 GPS
  float gyroBias[3];        // deg/s
  float accBias[3];         // m/s^2
  float yawDeg, rollDeg, pitchDeg;  // EKF attitude
};

enum class NavPhase : uint8_t {
  IDLE = 0, POSHOLD, RTH_CLIMB, RTH_RETURN, RTH_LOITER, LANDING, LANDED, WP_TAKEOFF, WP_ENROUTE, WP_HOLD, MISSION_DONE,
  RTH_ALIGN,  // no compass: fly forward until the GPS heading is learned
  WP_ORBIT, CRUISE,
  GUIDED,       // flying to / holding the GCS target
  SRTL_RETURN,  // SmartRTL: following the recorded path backwards
  BRAKE,        // stopping / holding where it stopped
  CIRCLE,       // orbiting the circle centre
  FOLLOW        // keeping follow_dist from the leader (holds when the leader goes quiet)
};
const char* navPhaseName(NavPhase p);

struct VehicleStatus {
  bool armed;
  FlightMode mode;
  NavPhase navPhase;
  uint32_t armBlockers;
  uint32_t warnings;
  bool failsafe;
  bool landed;
  uint32_t armedTimeMs;
  uint16_t missionIndex, missionCount;
  float homeDistance, homeBearingDeg;
  uint8_t atState;   // autotune: 0 off, 1 tuning, 2 done (land + disarm with the switch on to save)
  uint8_t atAxis, atRound;
  uint8_t atStep;    // autotune step: 0 P/D twitches, 1 feed-forward, 2 I-term
  float atLagMs, atSteadyErr;   // last feed-forward / I-term measurement
  float atOvershoot, atRiseMs;  // last measured response
};

}  // namespace fc
