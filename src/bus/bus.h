// AllQuad - global topic instances (the "bus") and config access shared by all tasks.
#pragma once
#include "../config/config.h"
#include "topics.h"

namespace bus {

using os::Topic;

// writer task noted after each topic
extern Topic<fc::RcInput> rc;               // io
extern Topic<fc::GpsData> gps;              // io
extern Topic<fc::BaroSample> baro;          // sensors
extern Topic<fc::MagSample> mag;            // sensors
extern Topic<fc::RangeSample> range;        // io (TFmini / MSP rangefinder)
extern Topic<fc::FlowSample> flow;          // io (MSP optical flow)
extern Topic<fc::EscTelemetry> esc;         // io
extern Topic<fc::BatteryState> battery;     // sensors
extern Topic<fc::AttitudeState> attitude;   // fast
extern Topic<fc::MotorOutputs> motors;      // fast
extern Topic<fc::PidDebug> pid;             // fast
extern Topic<fc::NavEstimate> nav;          // control
extern Topic<fc::ControlSetpoint> setpoint; // control
extern Topic<fc::VehicleStatus> status;     // control
extern Topic<fc::EkfReport> ekf;            // control

// GUIDED target from the GCS (MAVLink: SET_POSITION_TARGET_*, DO_REPOSITION, NAV_TAKEOFF)
struct GuidedTarget {
  int32_t lat, lon;   // degE7; 0,0 = keep the current horizontal position
  float alt;          // m above home; NaN = keep the current altitude
  bool takeoff;       // NAV_TAKEOFF: climb to alt here
  bool local;         // n / e (metres from home) instead of lat / lon
  float n, e;
};
extern Topic<GuidedTarget> guided;          // telemetry -> control
// FOLLOW leader (MAVLink FOLLOW_TARGET from QGroundControl "Follow Me" / a phone app)
struct FollowTarget {
  int32_t lat, lon;   // degE7
  float vn, ve;       // m/s (0 when unknown)
  uint32_t timeMs;    // when it arrived (0 = never)
};
extern Topic<FollowTarget> follow;          // telemetry -> control
// stick commands / OSD menu -> the telemetry task (the same task the CLI saves from)
extern volatile bool saveRequest;
extern volatile uint32_t stickBeepUntilMs;   // short buzzer / LED blink: a stick command was taken
extern volatile bool osdMenuOpen;            // the OSD shows the menu (sticks drive it, arming blocked)

// Configuration: tasks keep a pointer; writers (CLI / MAVLink / MSP) only change it
// while disarmed and then bump the generation so every task re-reads it.
extern fc::Config config;
extern volatile uint32_t configGeneration;
void configChanged();
// only the rate PID gains changed (in-flight autotune): the fast loop updates them without
// re-initialising its filters
extern volatile uint32_t gainsGeneration;
void gainsChanged();

// Requests from telemetry / CLI to the control task (consumed once)
enum class Request : uint8_t { NONE = 0, ARM, DISARM, SET_MODE, ACC_CAL, MAG_CAL, GYRO_CAL, COMPASSMOT };
void postRequest(Request r, uint32_t arg = 0);
bool takeRequest(Request& r, uint32_t& arg);

// IMU delta angle / delta velocity for the navigation EKF. The fast loop adds every attitude publish,
// the control task takes the sum since its last take: the attitude topic only keeps the latest
// sample, and at loop rates where it publishes faster than the control task runs (2 kHz loop ->
// 500 Hz vs 250 Hz) the EKF would otherwise integrate only part of the rotation and diverge.
void addImuDelta(const fc::Vec3& dAng, const fc::Vec3& dVel, float dt);
#ifdef ALLQUAD_HOST
extern float simEkfYawKickDeg;  // simulator: turn the EKF heading by this much once (divergence test)
#endif
bool takeImuDelta(fc::Vec3& dAng, fc::Vec3& dVel, float& dt);

// cross-task flags
extern volatile uint32_t cliActiveUntilMs;   // CLI in use -> arming blocked
extern volatile uint8_t gyroCapReq;          // CLI / web: start a high-rate gyro capture of this many s (logger task)
extern volatile uint32_t magCalUntilMs;      // sensors task collects compass extremes until then
extern volatile bool magCalDone;
extern volatile bool gyroCalRequest;         // fast task re-runs gyro calibration

// Fast-loop profile (written by the fast task once per second, read by CLI/web; diagnostics only)
struct LoopPerf {
  float imuUs, filterUs, ahrsUs, pidUs, motorUs, totalUs;  // mean per iteration
  float maxTotalUs;                                          // worst iteration in the window
  float periodUs, loadPct;                                   // loop period, mean total / period
  uint32_t lateLoops;                                        // iterations that started > 1 period late
};
extern volatile LoopPerf loopPerf;
// RC receiver link counters since the port was opened (io task; diagnostics only)
struct RxLinkStats { uint32_t bytes, frames, crcErrors; };
extern volatile RxLinkStats rxLink;
extern volatile bool otaActive;  // firmware update being written: arming blocked (flash writes stall the CPU)
extern volatile uint32_t fastLoops;
// GPS link diagnostics: port (hal::Port, 0xFF = none), baud rate in use, valid frames, protocol 0 / 1 UBX / 2 NMEA
// gen: u-blox generation (7..10, 0 unknown), rateHz: measured navigation solutions per second
struct GpsLink { uint8_t port, proto; uint32_t baud, frames; uint8_t gen; float rateHz; char protVer[8]; };
extern volatile GpsLink gpsLink;
// MSP sensor port (optical flow / rangefinder modules, e.g. MicoAir MTF-01): what arrives on it. port 0xFF = none;
// msp / mico = valid MSP / Micolink frames, mav = MAVLink frame starts seen (module set to the wrong protocol)
struct SensorLink { uint8_t port; uint32_t bytes, msp, mico, mav; };
extern volatile SensorLink sensorLink;
// voltage on the battery / current ADC pins (V, before vbat_scale / cur_scale; current -1 = no pin): wiring checks
// in the configurator (divider saturating the pin, current zero offset)
extern volatile float adcVbatV, adcCurV;
extern volatile int8_t escTelemMotor;        // motor whose ESC telemetry was last requested
// Motor test from the configurator (disarmed, props off): 0..1 per motor, stops by itself when no
// new value arrives for MOTOR_TEST_TIMEOUT_MS. Arming is blocked while it runs.
constexpr uint32_t MOTOR_TEST_TIMEOUT_MS = 1000;
extern volatile float motorTest[4];
extern volatile uint32_t motorTestUntilMs;
bool setMotorTest(const float v[4], uint32_t nowMs);  // false when armed / not allowed
inline bool motorTestActive(uint32_t nowMs) { return (int32_t)(motorTestUntilMs - nowMs) > 0; }
// ESC passthrough (BLHeli 4-way): the fast loop leaves the motor pins alone
extern volatile bool escPassthrough;
// ESC throttle-range calibration (PWM / OneShot ESCs, motors unarmed): 1 = full throttle, 2 = zero
// throttle on every motor; expires ESC_CAL_TIMEOUT_MS after the last keep-alive
constexpr uint32_t ESC_CAL_TIMEOUT_MS = 1500;
extern volatile uint8_t escCalState;
extern volatile uint32_t escCalUntilMs;
inline bool escCalActive(uint32_t nowMs) { return escCalState && (int32_t)(escCalUntilMs - nowMs) > 0; }
bool setEscCal(uint8_t state, uint32_t now);
// DShot command for some motors (beep, spin direction + save), sent by the fast loop when unarmed
extern volatile uint8_t escCmd, escCmdMask, escCmd2;
extern volatile bool escCmdPending;
bool requestEscCommand(uint8_t cmd, uint8_t mask, uint8_t thenCmd = 0);
// the EKF found the compass inconsistent with the GPS-observed heading: the fast-loop AHRS ignores it
extern volatile bool magRejected;
// bidirectional DShot RPM (written by the fast task, mechanical RPM; < 0 = no answer)
extern volatile float bidirRpm[4];
extern volatile uint8_t rpmLostMask;  // armed: motors whose RPM telemetry stopped (bit per mixer motor)
// active PID / rate profile (control task, from prof_ch or pid_profile)
extern volatile uint8_t pidProfile;
// last MAVLink heartbeat from a ground station (GCS failsafe), 0 = never
extern volatile uint32_t gcsHeartbeatMs;
extern volatile bool webGcs;  // the web page sent a GUIDED target: its status polling counts as the GCS link
// logic conditions: buzzer request, VTX power level override (0 = none)
extern volatile bool logicBeep;
extern volatile uint8_t logicVtxPower;
extern volatile uint8_t logicResults;      // bit i = rule i true
// IMU failover: 0 first chip, 1 second chip (IMU2 chip select)
extern volatile uint8_t imuActive;      // 2: second chip chosen on the ground as the better one (ImuVote)
extern volatile bool imuMismatch;     // the two IMUs disagree (arming blocked)
extern volatile bool imuSecondPresent;
// 6-point accelerometer calibration progress (bit per side), 0x3F = done
extern volatile uint8_t acc6Mask;
extern volatile bool acc6Active;
// compassmot (compass interference vs current) progress 0-100, 255 = idle
extern volatile uint8_t compassMotPct;

// short text messages for GCS / OSD (latest only)
void postMessage(const char* text);
uint32_t messageSeq();
void readMessage(char* out, int len);
// next message after `seq` (the reader's own counter, start at messageSeq()); false when none is new
bool messageAfter(uint32_t& seq, char* out, int len);

}  // namespace bus
