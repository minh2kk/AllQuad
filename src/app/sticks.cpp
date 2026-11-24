#include "../common/size_opt.h"
#include "sticks.h"

#include <math.h>
#include <stdio.h>

#include "../bus/bus.h"
#include "../osd/menu.h"

namespace app {

using fc::RC_PITCH;
using fc::RC_ROLL;
using fc::RC_THROTTLE;
using fc::RC_YAW;

namespace {
enum Pos { LO = 0, CE = 1, HI = 2 };
Pos pos(uint16_t us) { return us < 1150 ? LO : us > 1850 ? HI : (us > 1350 && us < 1650) ? CE : (Pos)3; }
}  // namespace

StickCommands::Cmd StickCommands::decode(const uint16_t* ch) {
  Pos t = pos(ch[RC_THROTTLE]), y = pos(ch[RC_YAW]), p = pos(ch[RC_PITCH]), r = pos(ch[RC_ROLL]);
  if (t == LO && y == LO) {
    if (p == LO && r == CE) return GYRO_CAL;
    if (p == LO && r == HI) return SAVE;
    if (p == CE && r == LO) return PROFILE1;
    if (p == HI && r == CE) return PROFILE2;
    if (p == CE && r == HI) return PROFILE3;
  }
  if (t == HI && y == LO && p == LO && r == CE) return ACC_CAL;
  if (t == HI && y == HI && p == LO && r == CE) return MAG_CAL;
  if (t == HI && y == CE) {
    if (p == HI && r == CE) return TRIM_FWD;
    if (p == LO && r == CE) return TRIM_BACK;
    if (r == LO && p == CE) return TRIM_LEFT;
    if (r == HI && p == CE) return TRIM_RIGHT;
  }
  if (t == CE && y == LO && p == HI && r == CE) return MENU;
  return NONE;
}

StickCommands::Cmd StickCommands::update(const fc::Config& c, const uint16_t* ch, bool rcOk, bool armed,
                                         bool armSwitch, uint32_t now) {
  if (!c.stick_cmds || !rcOk || armed || armSwitch || osd::menu.isOpen()) { held_ = NONE; done_ = false; return NONE; }
  Cmd k = decode(ch);
  if (k != held_) { held_ = k; sinceMs_ = now; done_ = false; }
  if (k == NONE || now - sinceMs_ < 1000) return NONE;
  bool trim = k >= TRIM_FWD && k <= TRIM_RIGHT;
  if (done_ && !(trim && now - lastMs_ >= 150)) return NONE;  // once per hold (trim repeats)
  done_ = true;
  lastMs_ = now;
  fc::Config& w = bus::config;
  char m[40];
  switch (k) {
    case GYRO_CAL: bus::postRequest(bus::Request::GYRO_CAL); break;
    case ACC_CAL: bus::postRequest(bus::Request::ACC_CAL); break;
    case MAG_CAL: bus::postRequest(bus::Request::MAG_CAL, 30); break;
    case SAVE: bus::saveRequest = true; bus::postMessage("Sticks: saving"); break;
    case PROFILE1: case PROFILE2: case PROFILE3:
      if (c.prof_ch) { bus::postMessage("Profile is on a switch (prof_ch)"); break; }
      w.pid_profile = (uint8_t)(k - PROFILE1);
      bus::configChanged();
      break;
    case TRIM_FWD: case TRIM_BACK: case TRIM_LEFT: case TRIM_RIGHT: {
      // pitch forward: nose lower (the craft drifted back); roll right: right side lower
      if (k == TRIM_FWD) w.level_trim_pitch_deg -= 0.1f;
      if (k == TRIM_BACK) w.level_trim_pitch_deg += 0.1f;
      if (k == TRIM_RIGHT) w.level_trim_roll_deg += 0.1f;
      if (k == TRIM_LEFT) w.level_trim_roll_deg -= 0.1f;
      w.level_trim_pitch_deg = fc::constrainf(w.level_trim_pitch_deg, -10.0f, 10.0f);
      w.level_trim_roll_deg = fc::constrainf(w.level_trim_roll_deg, -10.0f, 10.0f);
      bus::configChanged();
      snprintf(m, sizeof(m), "Trim R %.1f P %.1f", (double)w.level_trim_roll_deg, (double)w.level_trim_pitch_deg);
      bus::postMessage(m);
      break;
    }
    case MENU: osd::menu.openMenu(now); break;
    default: break;
  }
  bus::stickBeepUntilMs = now + 150;
  return k;
}

}  // namespace app
