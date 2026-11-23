// AllQuad - hardware in the loop (HITL)
//
// The real flight controller runs its real firmware, tasks, timing and estimators, but the sensors
// come from a physics simulation on the PC (tools/hitl.py) over the USB console, and the motor outputs
// go back to it. The motors themselves stay stopped the whole time (props can stay on, nothing spins).
//
//   PC -> FC  MSP2_AQ_HITL (0x4160) at ~500 Hz: gyro, accelerometer, pressure, compass (body FRD),
//             optionally GPS (10 Hz), rangefinder, RC channels, battery
//   FC -> PC  answer to every frame: mixer outputs 0..1, armed, mode, estimated attitude / height
//
// The first frame switches HITL on (disarmed only): the IMU / baro / compass drivers are replaced,
// the gyro is calibrated again, the console belongs to HITL (read by the io task at 500 Hz) and the
// real GPS / rangefinder / optical flow are ignored (the real receiver too when the PC sends RC).
// HITL ends with a reboot: when the PC stops sending for 1 s and the craft is disarmed.
#pragma once
#include <stddef.h>
#include <stdint.h>

#include "../sensors/sensors.h"

namespace hitl {

constexpr uint16_t MSP_CMD = 0x4160;
constexpr size_t FRAME_LEN = 108;   // PC -> FC payload
constexpr size_t REPLY_LEN = 36;    // FC -> PC payload
// flags in the PC frame
enum : uint8_t { F_GPS = 1, F_RANGE = 2, F_RC = 4, F_BATTERY = 8, F_EXIT = 0x80 };

bool active();
// telemetry task, MSP handler of the console: the first frame (switches HITL on). false = refused (armed)
bool begin(const uint8_t* payload, size_t len, uint8_t* reply, size_t& replyLen);
// io task: owns the console while HITL runs
void poll(uint32_t nowMs);

// replacements for the real drivers
drv::ImuDriver* imu();
drv::BaroDriver* baro();
drv::MagDriver* mag();
// fast task: the mixer outputs (mixer order, before motor_out remapping) that would have gone to the ESCs
void motorsOut(const float v[4], bool armed);
// sensors task: battery from the PC (false = none sent, use the ADC)
bool battery(float& volts, float& amps);
bool ownsRc();   // the PC sends the RC channels (the real receiver is ignored)

struct Stats { uint32_t frames, bad, lastMs; float rateHz; };
Stats stats();

}  // namespace hitl
