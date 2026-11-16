// AllQuad - application wiring: creates the tasks and connects drivers, links and modules.
//
//  task       rate        prio  core  owns
//  fast       1-2 kHz     6     1     IMU SPI, filters, AHRS, rate PID, mixer, motors
//  io         500 Hz      5     0     RX (CRSF/SBUS/iBUS + CRSF telemetry), GPS, ESC telemetry, rangefinder
//  control    250 Hz      4     1     estimator (Kalman), arming, failsafe, modes, navigation, outer loops
//  sensors    100 Hz      3     0     I2C: baro, compass (+calibration); battery; LED & buzzer
//  telemetry  100 Hz      2     0     MAVLink, MSP, CLI, OSD (MAX7456 / DisplayPort)
//  logger     bb_rate_hz  1     0     blackbox
#pragma once
#include "../sensors/sensors.h"

namespace app {

constexpr const char* VERSION = "2.0.0";

struct Drivers {
  drv::ImuDriver* imu;
  drv::BaroDriver* baro;
  drv::MagDriver* mag;
  drv::ImuDriver* imu2 = nullptr;  // second IMU (failover), optional
};

void loadConfig();
// Opens ports, configures GPS, creates all tasks. Does not start the STM32 scheduler.
void start(const Drivers& d, bool configureGps = true);
// Firmware entry points (called from Arduino setup()/loop())
void firmwareSetup();
void firmwareLoop();

const Drivers& drivers();

}  // namespace app
