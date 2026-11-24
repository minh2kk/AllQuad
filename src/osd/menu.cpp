#include "../common/size_opt.h"  // -Os on ESP32 (menu, not flight critical)
#include "menu.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "../bus/bus.h"
#include "../config/config.h"

namespace osd {

Menu menu;

namespace {

enum Kind : uint8_t { VALUE, PAGE, ACTION, BACK };
enum Action : uint8_t { A_EXIT = 0, A_SAVE_EXIT, A_GYRO_CAL, A_ACC_CAL };
enum Page : uint8_t { P_MAIN = 0, P_PID, P_RATES, P_FILTERS, P_VTX, P_BATTERY, P_MISC, P_COUNT };

struct Item {
  const char* label;
  Kind kind;
  const char* param;   // base name (profile 1)
  const char* prof;    // name in profiles 2 / 3 without the "pN_" prefix (nullptr: not per profile)
  float step;
  uint8_t arg;         // PAGE: page, ACTION: action
};

const Item kMain[] = {
    {"PID", PAGE, nullptr, nullptr, 0, P_PID},           {"RATES", PAGE, nullptr, nullptr, 0, P_RATES},
    {"FILTERS", PAGE, nullptr, nullptr, 0, P_FILTERS},   {"VTX", PAGE, nullptr, nullptr, 0, P_VTX},
    {"BATTERY", PAGE, nullptr, nullptr, 0, P_BATTERY},   {"FLIGHT", PAGE, nullptr, nullptr, 0, P_MISC},
    {"CALIBRATE GYRO", ACTION, nullptr, nullptr, 0, A_GYRO_CAL},
    {"CALIBRATE ACC (LEVEL)", ACTION, nullptr, nullptr, 0, A_ACC_CAL},
    {"SAVE + EXIT", ACTION, nullptr, nullptr, 0, A_SAVE_EXIT}, {"EXIT", ACTION, nullptr, nullptr, 0, A_EXIT},
};
const Item kPid[] = {
    {"ROLL P", VALUE, "roll_p", "roll_p", 1, 0},   {"ROLL I", VALUE, "roll_i", "roll_i", 1, 0},
    {"ROLL D", VALUE, "roll_d", "roll_d", 1, 0},   {"ROLL F", VALUE, "roll_f", "roll_f", 1, 0},
    {"PITCH P", VALUE, "pitch_p", "pitch_p", 1, 0}, {"PITCH I", VALUE, "pitch_i", "pitch_i", 1, 0},
    {"PITCH D", VALUE, "pitch_d", "pitch_d", 1, 0}, {"PITCH F", VALUE, "pitch_f", "pitch_f", 1, 0},
    {"YAW P", VALUE, "yaw_p", "yaw_p", 1, 0},       {"YAW I", VALUE, "yaw_i", "yaw_i", 1, 0},
    {"YAW F", VALUE, "yaw_f", "yaw_f", 1, 0},       {"BACK", BACK, nullptr, nullptr, 0, 0},
};
const Item kRates[] = {
    {"ROLL RATE", VALUE, "roll_rc_rate", "roll_rc", 10, 0},   {"ROLL MAX", VALUE, "roll_max_rate", "roll_max", 10, 0},
    {"ROLL EXPO", VALUE, "roll_expo", "roll_expo", 0.01f, 0}, {"PITCH RATE", VALUE, "pitch_rc_rate", "pitch_rc", 10, 0},
    {"PITCH MAX", VALUE, "pitch_max_rate", "pitch_max", 10, 0}, {"PITCH EXPO", VALUE, "pitch_expo", "pitch_expo", 0.01f, 0},
    {"YAW RATE", VALUE, "yaw_rc_rate", "yaw_rc", 10, 0},      {"YAW MAX", VALUE, "yaw_max_rate", "yaw_max", 10, 0},
    {"YAW EXPO", VALUE, "yaw_expo", "yaw_expo", 0.01f, 0},    {"BACK", BACK, nullptr, nullptr, 0, 0},
};
const Item kFilters[] = {
    {"GYRO LPF HZ", VALUE, "gyro_lpf_hz", nullptr, 5, 0},   {"DTERM LPF HZ", VALUE, "dterm_lpf_hz", nullptr, 5, 0},
    {"DYN NOTCHES", VALUE, "dyn_notch_count", nullptr, 1, 0}, {"RPM HARMONICS", VALUE, "rpm_harmonics", nullptr, 1, 0},
    {"BACK", BACK, nullptr, nullptr, 0, 0},
};
const Item kVtx[] = {
    {"BAND", VALUE, "vtx_band", nullptr, 1, 0},   {"CHANNEL", VALUE, "vtx_channel", nullptr, 1, 0},
    {"POWER", VALUE, "vtx_power", nullptr, 1, 0}, {"PIT MODE", VALUE, "vtx_pit", nullptr, 1, 0},
    {"BACK", BACK, nullptr, nullptr, 0, 0},
};
const Item kBattery[] = {
    {"CELLS (0 AUTO)", VALUE, "cell_count", nullptr, 1, 0}, {"WARN V/CELL", VALUE, "vbat_warn_cell", nullptr, 0.05f, 0},
    {"CRIT V/CELL", VALUE, "vbat_crit_cell", nullptr, 0.05f, 0}, {"CAPACITY MAH", VALUE, "bat_capacity", nullptr, 50, 0},
    {"RESERVE %", VALUE, "bat_reserve_pct", nullptr, 1, 0},  {"SMART RTH", VALUE, "bat_smart_rth", nullptr, 1, 0},
    {"BACK", BACK, nullptr, nullptr, 0, 0},
};
const Item kMisc[] = {
    {"ANGLE LIMIT", VALUE, "angle_limit", nullptr, 1, 0},    {"AIRMODE", VALUE, "airmode", nullptr, 1, 0},
    {"RTH ALT M", VALUE, "rth_alt", nullptr, 1, 0},          {"HEADFREE TYPE", VALUE, "headfree_type", nullptr, 1, 0},
    {"CIRCLE RADIUS", VALUE, "circle_radius", nullptr, 1, 0}, {"CIRCLE SPEED", VALUE, "circle_speed", nullptr, 0.5f, 0},
    {"FOLLOW DIST", VALUE, "follow_dist", nullptr, 1, 0},     {"BEACON S", VALUE, "beacon_s", nullptr, 1, 0},
    {"BACK", BACK, nullptr, nullptr, 0, 0},
};

struct PageDef { const char* title; const Item* items; int n; };
const PageDef kPages[P_COUNT] = {
    {"ALLQUAD MENU", kMain, sizeof(kMain) / sizeof(Item)},
    {"PID", kPid, sizeof(kPid) / sizeof(Item)},
    {"RATES", kRates, sizeof(kRates) / sizeof(Item)},
    {"FILTERS", kFilters, sizeof(kFilters) / sizeof(Item)},
    {"VTX", kVtx, sizeof(kVtx) / sizeof(Item)},
    {"BATTERY", kBattery, sizeof(kBattery) / sizeof(Item)},
    {"FLIGHT", kMisc, sizeof(kMisc) / sizeof(Item)},
};

// the parameter of a line for the active PID / rate profile (-1: none)
int paramOf(const Item& it) {
  if (it.kind != VALUE) return -1;
  int prof = bus::pidProfile;
  if (it.prof && prof >= 1 && prof <= 2) {
    char n[32];
    snprintf(n, sizeof(n), "p%d_%s", prof + 1, it.prof);
    return fc::paramFind(n);
  }
  return fc::paramFind(it.param);
}

}  // namespace

int Menu::itemCount() const { return kPages[page_].n; }

void Menu::openMenu(uint32_t nowMs) {
  open_ = true;
  page_ = P_MAIN;
  cur_ = top_ = 0;
  heldDir_ = 0;
  heldAxis_ = -1;
  openedMs_ = nowMs;
  neutralSeen_ = false;
  bus::osdMenuOpen = true;
  bus::postMessage("OSD menu");
}

void Menu::close(bool save) {
  open_ = false;
  bus::osdMenuOpen = false;
  if (save) bus::saveRequest = true;
  bus::postMessage(save ? "Menu: saved" : "Menu closed");
}

void Menu::move(int dir) {
  int n = itemCount();
  cur_ = (cur_ + dir + n) % n;
}

void Menu::press(int dir) {
  const Item& it = kPages[page_].items[cur_];
  switch (it.kind) {
    case PAGE:
      if (dir > 0) { page_ = it.arg; cur_ = top_ = 0; }
      break;
    case BACK:
      if (dir > 0 || dir < 0) { int from = page_; page_ = P_MAIN; top_ = 0; cur_ = from - 1 >= 0 ? from - 1 : 0; }
      break;
    case ACTION:
      if (dir <= 0) break;
      if (it.arg == A_EXIT) close(false);
      else if (it.arg == A_SAVE_EXIT) close(true);
      else if (it.arg == A_GYRO_CAL) bus::postRequest(bus::Request::GYRO_CAL);
      else if (it.arg == A_ACC_CAL) bus::postRequest(bus::Request::ACC_CAL);
      break;
    case VALUE: {
      int i = paramOf(it);
      if (i < 0) break;
      float step = it.step;
      // held for a while: ten steps at a time
      if (heldDir_ && heldSinceMs_ && lastRepeatMs_ - heldSinceMs_ > 1500) step *= 10;
      float v = fc::paramGet(bus::config, i) + dir * step;
      const fc::ParamInfo& p = fc::paramInfo(i);
      if (v < p.min) v = p.min;
      if (v > p.max) v = p.max;
      if (fc::paramSet(bus::config, i, v)) bus::configChanged();
      break;
    }
  }
}

void Menu::input(float roll, float pitch, uint32_t now) {
  if (!open_) return;
  // the stick command that opened the menu is still held: wait for centred sticks
  bool centred = fabsf(roll) < 0.3f && fabsf(pitch) < 0.3f;
  if (!neutralSeen_) { if (centred) neutralSeen_ = true; return; }
  int axis = -1, dir = 0;
  if (fabsf(pitch) > 0.6f && fabsf(pitch) >= fabsf(roll)) { axis = 0; dir = pitch > 0 ? -1 : 1; }  // forward = up the list
  else if (fabsf(roll) > 0.6f) { axis = 1; dir = roll > 0 ? 1 : -1; }
  if (axis < 0) { heldDir_ = 0; heldAxis_ = -1; heldSinceMs_ = 0; return; }
  bool fresh = axis != heldAxis_ || dir != heldDir_;
  uint32_t gap = (!fresh && now - heldSinceMs_ > 400) ? 120 : 400;  // first repeat after 0.4 s, then fast
  if (!fresh && now - lastRepeatMs_ < gap) return;
  if (fresh) { heldAxis_ = axis; heldDir_ = dir; heldSinceMs_ = now; }
  lastRepeatMs_ = now;
  if (axis == 0) move(dir);
  else if (fresh || kPages[page_].items[cur_].kind == VALUE) press(dir);  // pages / actions: once per push
}

int Menu::lines(char out[][LINE + 1], int maxLines) const {
  if (!open_ || maxLines < 3) return 0;
  const PageDef& pg = kPages[page_];
  int n = 0;
  if (page_ == P_PID || page_ == P_RATES) snprintf(out[n++], LINE + 1, "  %s (PROFILE %d)", pg.title, bus::pidProfile + 1);
  else snprintf(out[n++], LINE + 1, "  %s", pg.title);
  int rows = maxLines - 2;  // title + hint
  int top = top_;
  if (cur_ < top) top = cur_;
  if (cur_ >= top + rows) top = cur_ - rows + 1;
  const_cast<Menu*>(this)->top_ = top;
  for (int k = top; k < pg.n && n < maxLines - 1; k++) {
    const Item& it = pg.items[k];
    char val[12] = "";
    int i = paramOf(it);
    if (i >= 0) {
      float v = fc::paramGet(bus::config, i);
      if (fabsf(v - roundf(v)) < 1e-4f) snprintf(val, sizeof(val), "%d", (int)roundf(v));
      else snprintf(val, sizeof(val), "%.2f", (double)v);
    } else if (it.kind == PAGE) {
      strcpy(val, ">");
    }
    char* l = out[n++];
    snprintf(l, LINE + 1, "%c%-20s%7s", k == cur_ ? '>' : ' ', it.label, val);
  }
  snprintf(out[n++], LINE + 1, " PITCH:MOVE ROLL:CHANGE");
  return n;
}

}  // namespace osd
