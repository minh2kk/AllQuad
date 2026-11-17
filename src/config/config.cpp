#include "../common/size_opt.h"  // -Os on ESP32 (not flight critical)
#include "config.h"

#include <string.h>

namespace fc {

static uint16_t osdPos(int row, int col, bool visible = true) {
  return (uint16_t)((visible ? 0x8000 : 0) | (row * 32 + col));
}

// fields added in version 21 (also used by configMigrate for older stored settings)
static void v21Defaults(Config& c) {
  c.rc_smooth_hz = 0;      // auto
  c.crash_detect = 1;
  c.yaw_spin_thr = 1950;
  c.d_min_pct = 0;         // off: constant D (as before)
  c.d_min_gain = 37;
  c.d_min_adv = 20;
  c.dyn_lpf_min = 0;       // off: static gyro_lpf2_hz
  c.dyn_lpf_max = 0;
  c.dyn_idle_rpm = 0;
  c.thr_boost = 0;
  c.thrust_expo = 0.0f;
  c.vbat_sag_comp = 0;
  c.pid_profile = 0;
  c.prof_ch = 0;
  for (int p = 0; p < 2; p++)
    for (int a = 0; a < 3; a++) { c.prof_pid[p][a] = c.pid[a]; c.prof_rates[p][a] = c.rates[a]; }
  c.gcs_fs = 0;
  c.gcs_fs_s = 5.0f;
  c.srtl_point_m = 3.0f;
  for (int i = 0; i < 3; i++) { c.acc_scale[i] = 1.0f; c.mag_offdiag[i] = 0.0f; c.mag_cmot[i] = 0.0f; }
  c.imu_dual = 1;
  c.fence_poly = 1;
  for (int i = 0; i < LOGIC_RULES; i++) c.logic[i] = LogicRule{};
}
static void v22Defaults(Config& c) {
  for (int i = 0; i < NUM_PORTS_EXT; i++) { c.port_fn_ext[i] = 0; c.port_baud_ext[i] = 0; }
}
static void v24Defaults(Config& c) {
  for (int i = 0; i < 3; i++) c.rpm_weight[i] = 100;
  c.osd_stats = 15;
  c.osd_pos2[OSD2_ESC_TEMP] = osdPos(2, 24, false);
  c.osd_pos2[OSD2_DIST_TOTAL] = osdPos(3, 1, false);
  c.osd_pos2[OSD2_EFFICIENCY] = osdPos(13, 23, false);
  c.osd_pos2[OSD2_TIME_LEFT] = osdPos(13, 1, false);
}
static void v25Defaults(Config& c) {
#if defined(AQ_DEFAULT_SD_MODE)  // boards with a card slot on the PCB (WeAct H743)
  c.sd_mode = AQ_DEFAULT_SD_MODE;
#else
  c.sd_mode = 0;
#endif
  c.sd_mhz = 0;
  for (int i = 0; i < SD_PINS; i++) c.sd_pin[i] = 255;
  c.sd_keep = 1;
}
static void v26Defaults(Config& c) {
  // hidden until ticked in the OSD tab (then shown at these places)
  c.osd_pos3[OSD3_BAT_BAR] = osdPos(12, 1, false);
  c.osd_pos3[OSD3_CELL_V] = osdPos(5, 1, false);
  c.osd_pos3[OSD3_RSSI_DBM] = osdPos(3, 23, false);
  c.osd_blink = 1;
  c.sd_dma = 0;
}
static void v27Defaults(Config& c) {
  c.ff_jitter = 7;   // Betaflight defaults
  c.ff_boost = 15;
  c.ff_max_pct = 90;
}
static void v23Defaults(Config& c) {
  c.osd_canvas = 0;  // auto
  c.osd_font = 0;    // Betaflight glyphs
}

void configDefaults(Config& c) {
  memset(&c, 0, sizeof(c));
#if defined(AQ_DEFAULT_LOOP_HZ)
  c.loop_hz = AQ_DEFAULT_LOOP_HZ;
#else
  c.loop_hz = 1000;
#endif
  c.rx_protocol = (uint8_t)RxProtocol::CRSF;
  c.motor_protocol = (uint8_t)MotorProtocol::DSHOT300;
  c.airmode = 1;
  c.motor_poles = 14;
  c.motor_idle = 0.055f;

  c.port_fn[0] = (uint8_t)PortFn::RX;
  c.port_fn[1] = (uint8_t)PortFn::GPS;
  c.port_fn[2] = (uint8_t)PortFn::MAVLINK;
  c.port_fn[3] = (uint8_t)PortFn::MAVLINK;
  c.wifi_enable = 0;
  c.wifi_armed = 0;
  c.mav_sysid = 1;

  c.gyro_lpf_hz = 150.0f;
  c.gyro_lpf2_hz = 250.0f;
  c.dterm_lpf_hz = 90.0f;
  c.acc_lpf_hz = 15.0f;
  c.dyn_notch_count = 1;
  c.dyn_notch_min = 100.0f;
  c.dyn_notch_max = 450.0f;
  c.dyn_notch_q = 3.5f;
  c.rpm_harmonics = 3;
  c.rpm_min_hz = 100.0f;
  c.rpm_q = 5.0f;

  c.pid[0] = {45.0f, 80.0f, 30.0f, 60.0f};
  c.pid[1] = {47.0f, 84.0f, 34.0f, 60.0f};
  c.pid[2] = {45.0f, 80.0f, 0.0f, 60.0f};
  c.iterm_limit = 400.0f;
  c.pidsum_limit = 500.0f;
  c.pidsum_limit_yaw = 400.0f;
  c.tpa_rate = 0.4f;
  c.tpa_breakpoint = 0.6f;
  c.iterm_relax_cutoff = 40.0f;
  c.anti_gravity = 3.5f;
  for (int i = 0; i < 3; i++) c.rates[i] = {70.0f, 670.0f, 0.0f};

  c.angle_p = 6.0f;
  c.at_axes = 3;
  c.at_rate = 100.0f;
  c.angle_limit_deg = 45.0f;
  c.horizon_transition = 0.75f;

  c.alt_pos_p = 1.2f;
  c.alt_vel_p = 0.12f;
  c.alt_vel_i = 0.08f;
  c.alt_vel_d = 0.01f;
  c.climb_rate_max = 2.0f;
  c.hover_throttle = 0.40f;
  c.hover_learn = 2;
  c.alt_surface = 1;

  c.pos_p = 0.8f;
  c.pos_vel_p = 1.6f;
  c.pos_vel_i = 0.3f;
  c.pos_max_speed = 6.0f;
  c.pos_max_angle_deg = 25.0f;
  c.pos_max_accel = 4.0f;
  c.gps_min_sats = 7;
  c.gps_max_hacc = 5.0f;
  c.gps_glitch_m = 8.0f;
  c.gps_glitch_s = 10.0f;
  c.gps_heading = 1;
  c.wind_comp = 1;
  c.wind_drag = 0.45f;

  c.rth_alt = 20.0f;
  c.rth_speed = 6.0f;
  c.rth_loiter_s = 3.0f;
  c.rth_home_radius = 2.0f;
  c.rth_climb_first = 1;
  c.rth_land = 1;
  c.land_speed = 1.0f;
  c.land_slow_speed = 0.4f;
  c.land_slow_alt = 3.0f;
  c.wp_radius = 2.0f;
  c.wp_speed = 5.0f;
  c.nav_yaw_mode = 1;
  c.rally_max_m = 500.0f;
  c.nav_yaw_p = 2.0f;

  c.fence_enable = 0;
  c.fence_action = (uint8_t)FailsafeAction::RTH;
  c.fence_radius = 150.0f;
  c.fence_alt_max = 100.0f;

  // default switch layout (3-position switches; low = off)
  static const ModeRange defRanges[] = {
      {(uint8_t)ModeBox::ARM, 4, 1700, 2100},      // AUX1 high
      {(uint8_t)ModeBox::ANGLE, 5, 1300, 1700},    // AUX2: low ACRO / mid ANGLE / high ALTHOLD
      {(uint8_t)ModeBox::ALTHOLD, 5, 1700, 2100},
      {(uint8_t)ModeBox::POSHOLD, 6, 1300, 1700},  // AUX3: off / POSHOLD / RTH
      {(uint8_t)ModeBox::RTH, 6, 1700, 2100},
      {(uint8_t)ModeBox::MISSION, 7, 1300, 1700},  // AUX4: off / MISSION / LAND
      {(uint8_t)ModeBox::LAND, 7, 1700, 2100},
      {(uint8_t)ModeBox::BEEPER, 8, 1700, 2100},   // AUX5
      {(uint8_t)ModeBox::TURTLE, 9, 1700, 2100},   // AUX6
  };
  for (int i = 0; i < MODE_RANGES; i++)
    c.mode_range[i] = i < (int)(sizeof(defRanges) / sizeof(defRanges[0])) ? defRanges[i] : ModeRange{0, 4, 1700, 2100};
  c.arm_throttle_max_us = 1050;

  c.fs_delay_ms = 300;
  c.fs_off_delay_ms = 30000;
  c.fs_action = (uint8_t)FailsafeAction::RTH;
  c.fs_throttle = 0.35f;

#if defined(AQ_DEFAULT_VBAT_SCALE)  // board builds with their own divider (platformio.ini)
  c.vbat_scale = AQ_DEFAULT_VBAT_SCALE;
#else
  c.vbat_scale = 11.0f;
#endif
#if defined(AQ_DEFAULT_MOTOR_OUT)  // e.g. 3241: mixer motor 1 on output 3, motor 2 on output 2 ...
  for (int i = 0, v = AQ_DEFAULT_MOTOR_OUT; i < 4; i++, v *= 10) c.motor_out[i] = (uint8_t)((v / 1000) % 10);
#else
  for (int i = 0; i < 4; i++) c.motor_out[i] = (uint8_t)(i + 1);
#endif
  c.imu_sync = 1;
  c.dshot_edt = 1;
#if defined(AQ_DEFAULT_YAW_REVERSED)  // props-out frames
  c.yaw_reversed = AQ_DEFAULT_YAW_REVERSED;
#endif
  c.vbat_warn_cell = 3.5f;
  c.vbat_crit_cell = 3.3f;
  c.bat_capacity = 0;
  c.bat_fs_action = 1;
  c.bat_reserve_pct = 20;

  c.arm_require_gps = 0;
  c.gcs_arm = 0;
  c.arm_max_angle = 25.0f;

  c.est_baro_noise = 0.4f;
  c.est_acc_noise = 0.6f;
  c.est_gps_vel_noise = 0.3f;
  c.rf_max_m = 8.0f;
  c.flow_scale = 10.5f;

  c.osd_backend = 0;
  c.osd_pal = 1;
  c.osd_pos[OSD_VBAT] = osdPos(14, 1);
  c.osd_pos[OSD_CURRENT] = osdPos(14, 9);
  c.osd_pos[OSD_MAH] = osdPos(14, 16);
  c.osd_pos[OSD_MODE] = osdPos(13, 12);
  c.osd_pos[OSD_ARM_TIME] = osdPos(14, 23);
  c.osd_pos[OSD_ALTITUDE] = osdPos(7, 23);
  c.osd_pos[OSD_VARIO] = osdPos(8, 23);
  c.osd_pos[OSD_HOME_DIST] = osdPos(1, 1);
  c.osd_pos[OSD_HOME_DIR] = osdPos(1, 14);
  c.osd_pos[OSD_GPS_SATS] = osdPos(1, 24);
  c.osd_pos[OSD_SPEED] = osdPos(7, 1);
  c.osd_pos[OSD_LINK] = osdPos(2, 1);
  c.osd_pos[OSD_CROSSHAIR] = osdPos(7, 14);
  c.osd_pos[OSD_HORIZON] = osdPos(7, 9, false);
  c.osd_pos[OSD_WARNINGS] = osdPos(10, 9);
  c.osd_pos[OSD_COORDS] = osdPos(15, 1, false);
  c.bb_rate_hz = 0;
#if defined(ESP32)
  c.bb_backend = 2;  // onboard flash partition - readable from the web configurator
#else
  c.bb_backend = 1;
#endif

  for (int i = 0; i < 3; i++) c.mag_scale[i] = 1.0f;

  c.vtx_band = 5;  // Raceband
  c.vtx_channel = 1;
  c.vtx_freq = 0;
  c.vtx_power = 1;
  c.vtx_pit = 0;
  c.vtx_low_disarmed = 1;
  c.led_pin = 0;
  c.led_count = 8;
  c.led_brightness = 40;
  c.led_mode = 0;
  c.led_color = 0x0040FF;

  c.dshot_bidir = 0;
#if defined(AQ_DEFAULT_DSHOT600)  // 4-in-1 32-bit ESC boards (AM32 / BLHeli_32): DShot600 + RPM telemetry
  c.motor_protocol = (uint8_t)MotorProtocol::DSHOT600;
  c.dshot_bidir = 1;
#endif
  c.est_type = 1;
  c.at_steps = 1 | 2 | 4 | 8;
  c.vibe_warn = 30.0f;
  c.launch_angle = 15;
  c.launch_thr = 20;
  c.rth_sanity_s = 20.0f;
  c.vtxt_powers = 0;
  static const uint16_t mw[8] = {25, 100, 200, 400, 600, 800, 1000, 1600};
  static const uint8_t dbm[8] = {14, 20, 23, 26, 28, 29, 30, 32};
  static const uint16_t band[8] = {5645, 5665, 5685, 5705, 5885, 5905, 5925, 5945};
  for (int i = 0; i < 8; i++) { c.vtxt_mw[i] = mw[i]; c.vtxt_val[i] = dbm[i]; c.vtxt_band[i] = band[i]; }
  v21Defaults(c);
  v22Defaults(c);
  v23Defaults(c);
  v24Defaults(c);
  v25Defaults(c);
  v26Defaults(c);
  v27Defaults(c);
  configSeal(c);
}

bool configMigrate(Config& c) {
  // version 15 is the first layout that only grows at the end
  // (a new 1-byte field can land in the old padding before crc: same size, newer version)
  if (c.version < 15 || c.version >= CONFIG_VERSION || c.size < 64 || c.size > sizeof(Config)) return false;
  uint32_t oldCrc;
  memcpy(&oldCrc, (const uint8_t*)&c + c.size - 4, 4);
  if (crc32(&c, c.size - 4) != oldCrc) return false;
  Config d, def;
  configDefaults(def);
  d = def;
  memcpy(&d, &c, c.size - 4);
  // fields that sat in the old padding: their bytes were copied above, give them their default
  if (c.version < 20) d.dshot_edt = def.dshot_edt;
  // version 21: profiles 2 / 3 start as copies of the stored profile 1
  if (c.version < 21) {
    memcpy((uint8_t*)&d + offsetof(Config, rc_smooth_hz), (const uint8_t*)&def + offsetof(Config, rc_smooth_hz),
           offsetof(Config, crc) - offsetof(Config, rc_smooth_hz));
    v21Defaults(d);
  }
  if (c.version < 22) v22Defaults(d);
  if (c.version < 23) v23Defaults(d);
  if (c.version < 24) v24Defaults(d);
  if (c.version < 25) v25Defaults(d);
  if (c.version < 26) v26Defaults(d);
  if (c.version < 27) v27Defaults(d);
  configSeal(d);
  c = d;
  return true;
}

uint32_t crc32(const void* data, size_t n, uint32_t crc) {
  const uint8_t* p = static_cast<const uint8_t*>(data);
  crc = ~crc;
  for (size_t i = 0; i < n; i++) {
    crc ^= p[i];
    for (int b = 0; b < 8; b++) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
  }
  return ~crc;
}

uint32_t configCrc(const Config& c) { return crc32(&c, offsetof(Config, crc)); }

void configSeal(Config& c) {
  c.version = CONFIG_VERSION;
  c.size = sizeof(Config);
  c.crc = configCrc(c);
}

bool configValid(const Config& c) {
  return c.version == CONFIG_VERSION && c.size == sizeof(Config) && c.crc == configCrc(c);
}

uint32_t modeBoxes(const Config& c, const uint16_t* ch, int nch) {
  uint32_t m = 0;
  for (int i = 0; i < MODE_RANGES; i++) {
    const ModeRange& r = c.mode_range[i];
    if (!r.box || r.box >= (uint8_t)ModeBox::COUNT || r.ch >= nch) continue;
    uint16_t v = ch[r.ch];
    if (v >= r.lo && v < r.hi) m |= 1u << r.box;
  }
  return m;
}

const char* modeBoxName(int box) {
  static const char* const names[] = {"NONE", "ARM", "ANGLE", "HORIZON", "ALTHOLD", "POSHOLD",
                                      "RTH", "MISSION", "LAND", "TURTLE", "BEEPER", "CRUISE", "AUTOTUNE",
                                      "GPS RESCUE", "LAUNCH", "SMART RTL"};
  return box >= 0 && box < (int)ModeBox::COUNT ? names[box] : "?";
}

}  // namespace fc
