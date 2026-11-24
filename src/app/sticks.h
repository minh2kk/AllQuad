// AllQuad - stick commands (Betaflight / KK2): calibrate, pick a profile, trim, save and open the OSD menu
// with the sticks, no computer. Disarmed, arm switch off, RC link up, stick_cmds 1; each held 1 s
// (trim repeats every 0.15 s while held). Mode 2 sticks: LO = left / down, HI = right / up.
//
//   throttle LO  yaw LO  pitch LO  roll CE   calibrate the gyro
//   throttle HI  yaw LO  pitch LO  roll CE   calibrate the accelerometer (craft level)
//   throttle HI  yaw HI  pitch LO  roll CE   calibrate the compass (30 s, turn the craft around)
//   throttle LO  yaw LO  pitch LO  roll HI   save the settings
//   throttle LO  yaw LO  roll LO / pitch HI / roll HI   PID / rate profile 1 / 2 / 3
//   throttle HI  yaw CE  pitch or roll       level trim 0.1 deg per step (ANGLE drifts the other way)
//   throttle CE  yaw LO  pitch HI  roll CE   open the OSD menu
#pragma once
#include <stdint.h>

#include "../bus/topics.h"
#include "../config/config.h"

namespace app {

class StickCommands {
 public:
  enum Cmd : uint8_t { NONE = 0, GYRO_CAL, ACC_CAL, MAG_CAL, SAVE, PROFILE1, PROFILE2, PROFILE3, TRIM_FWD, TRIM_BACK,
                       TRIM_LEFT, TRIM_RIGHT, MENU };
  // ch: RC channels (us), armSwitch: the ARM box is on; returns the command taken this call (or NONE)
  Cmd update(const fc::Config& c, const uint16_t* ch, bool rcOk, bool armed, bool armSwitch, uint32_t nowMs);
  static Cmd decode(const uint16_t* ch);  // the pattern the sticks show now

 private:
  Cmd held_ = NONE;
  uint32_t sinceMs_ = 0, lastMs_ = 0;
  bool done_ = false;
};

}  // namespace app
