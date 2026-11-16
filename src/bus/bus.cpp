#include "bus.h"

#include <string.h>

namespace fc {

const char* flightModeName(FlightMode m) {
  static const char* n[] = {"ACRO", "ANGLE", "HORIZON", "ALTHOLD", "POSHOLD", "RTH", "MISSION", "LAND", "FAILSAFE", "TURTLE", "CRUISE", "AUTOTUNE", "GUIDED", "SMARTRTL"};
  return (uint8_t)m < (uint8_t)FlightMode::COUNT ? n[(uint8_t)m] : "?";
}

const char* armCheckName(int bit) {
  static const char* n[] = {"NO_RX", "THROTTLE", "ANGLE", "ARM_SWITCH", "CALIBRATING", "BATTERY",
                            "NO_IMU", "CLI", "FAILSAFE", "ACC_NOT_CALIBRATED", "BARO", "MAG",
                            "GPS", "FENCE", "CPU_LOAD", "NAV_MODE", "LOGIC", "IMU_FAILOVER"};
  return bit >= 0 && bit < ARM_COUNT_BITS ? n[bit] : "?";
}

const char* navPhaseName(NavPhase p) {
  static const char* n[] = {"IDLE", "POSHOLD", "RTH_CLIMB", "RTH_RETURN", "RTH_LOITER", "LANDING",
                            "LANDED", "WP_TAKEOFF", "WP_ENROUTE", "WP_HOLD", "MISSION_DONE",
                            "RTH_ALIGN", "WP_ORBIT", "CRUISE", "GUIDED", "SRTL_RETURN"};
  return (uint8_t)p <= (uint8_t)NavPhase::SRTL_RETURN ? n[(uint8_t)p] : "?";
}

}  // namespace fc

namespace bus {

Topic<fc::RcInput> rc;
Topic<fc::GpsData> gps;
Topic<fc::BaroSample> baro;
Topic<fc::MagSample> mag;
Topic<fc::RangeSample> range;
Topic<fc::FlowSample> flow;
Topic<fc::EscTelemetry> esc;
Topic<fc::BatteryState> battery;
Topic<fc::AttitudeState> attitude;
Topic<fc::MotorOutputs> motors;
Topic<fc::PidDebug> pid;
Topic<fc::NavEstimate> nav;
Topic<fc::ControlSetpoint> setpoint;
Topic<fc::VehicleStatus> status;
Topic<fc::EkfReport> ekf;

fc::Config config;
volatile uint32_t configGeneration = 1;
volatile uint32_t gainsGeneration = 1;
void gainsChanged() { gainsGeneration = gainsGeneration + 1; }
void configChanged() {
  os::CriticalGuard g;
  configGeneration = configGeneration + 1;
}

volatile uint32_t cliActiveUntilMs = 0;
volatile uint32_t magCalUntilMs = 0;
volatile bool magCalDone = false;
volatile LoopPerf loopPerf;
volatile uint32_t fastLoops = 0;
volatile GpsLink gpsLink = {7, 0, 0, 0};
volatile RxLinkStats rxLink;
volatile bool otaActive = false;
volatile bool gyroCalRequest = false;
volatile int8_t escTelemMotor = -1;
volatile bool magRejected = false;
volatile float bidirRpm[4] = {-1, -1, -1, -1};
volatile uint8_t rpmLostMask = 0;
volatile uint8_t pidProfile = 0;
volatile uint32_t gcsHeartbeatMs = 0;
volatile bool webGcs = false;
volatile bool logicBeep = false;
volatile uint8_t logicVtxPower = 0;
volatile uint8_t logicResults = 0;
volatile uint8_t imuActive = 0;
volatile bool imuSecondPresent = false;
volatile uint8_t acc6Mask = 0;
volatile bool acc6Active = false;
volatile uint8_t compassMotPct = 255;
Topic<GuidedTarget> guided;
volatile float motorTest[4] = {0, 0, 0, 0};
volatile uint32_t motorTestUntilMs = 0;
volatile bool escPassthrough = false;
volatile uint8_t escCalState = 0;
volatile uint32_t escCalUntilMs = 0;
volatile uint8_t escCmd = 0, escCmdMask = 0, escCmd2 = 0;
volatile bool escCmdPending = false;
bool setEscCal(uint8_t state, uint32_t now) {
  if (status.get().armed || escPassthrough) return false;
  escCalState = state;
  escCalUntilMs = now + ESC_CAL_TIMEOUT_MS;
  if (state) cliActiveUntilMs = now + ESC_CAL_TIMEOUT_MS + 2000;  // no arming during / right after
  return true;
}
bool requestEscCommand(uint8_t cmd, uint8_t mask, uint8_t thenCmd) {
  if (status.get().armed || escPassthrough || escCmdPending) return false;
  escCmd = cmd; escCmdMask = mask; escCmd2 = thenCmd;
  escCmdPending = true;
  return true;
}

bool setMotorTest(const float v[4], uint32_t now) {
  if (status.get().armed || escPassthrough) return false;
  bool any = false;
  for (int i = 0; i < 4; i++) {
    float x = v[i] > 0.0f ? (v[i] < 1.0f ? v[i] : 1.0f) : 0.0f;  // NaN -> 0
    motorTest[i] = x;
    any |= x > 0;
  }
  motorTestUntilMs = any ? now + MOTOR_TEST_TIMEOUT_MS : 0;
  if (any) cliActiveUntilMs = now + MOTOR_TEST_TIMEOUT_MS + 2000;  // no arming during / right after a test
  return true;
}

static Request reqPending = Request::NONE;
static uint32_t reqArg = 0;
void postRequest(Request r, uint32_t arg) {
  os::CriticalGuard g;
  reqPending = r;
  reqArg = arg;
}
bool takeRequest(Request& r, uint32_t& arg) {
  os::CriticalGuard g;
  if (reqPending == Request::NONE) return false;
  r = reqPending;
  arg = reqArg;
  reqPending = Request::NONE;
  return true;
}

static char msg[64];
static volatile uint32_t msgSeq = 0;
void postMessage(const char* text) {
  os::CriticalGuard g;
  strncpy(msg, text, sizeof(msg) - 1);
  msg[sizeof(msg) - 1] = 0;
  msgSeq = msgSeq + 1;
}
uint32_t messageSeq() { return msgSeq; }
void readMessage(char* out, int len) {
  os::CriticalGuard g;
  strncpy(out, msg, len - 1);
  out[len - 1] = 0;
}

}  // namespace bus
