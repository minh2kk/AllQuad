// AllQuad - persistent configuration (single struct, stored with CRC)
#pragma once
#include <stddef.h>
#include <stdint.h>

namespace fc {

// Version history: 16 = VTX, LED strip; 17 = bidirectional DShot, EKF, autotune steps, launch control,
// RTH sanity check, vibration warning, custom VTX table (appended). New fields are ONLY appended before `crc`:
// configMigrate() then keeps every setting of an older (shorter) stored config.
constexpr uint16_t CONFIG_VERSION = 28;

// fastest flight loop the target sustains (platformio.ini: the H743 runs 3.2-4 kHz)
#ifndef AQ_LOOP_HZ_MAX
#define AQ_LOOP_HZ_MAX 2000
#endif

enum class RxProtocol : uint8_t { CRSF = 0, SBUS = 1, IBUS = 2 };
enum class MotorProtocol : uint8_t { PWM = 0, ONESHOT125 = 1, DSHOT300 = 2, DSHOT600 = 3 };
enum class Align : uint8_t { CW0 = 0, CW90, CW180, CW270, CW0_FLIP, CW90_FLIP, CW180_FLIP, CW270_FLIP };

// Function assigned to a serial port (Betaflight "Ports" tab / ArduPilot SERIALx_PROTOCOL)
enum class PortFn : uint8_t {
  NONE = 0,
  RX = 1,           // CRSF / SBUS / iBUS receiver (CRSF telemetry goes back on the same port)
  GPS = 2,
  MAVLINK = 3,      // QGroundControl, Mission Planner, companion computers
  MSP = 4,          // MSP requests + MSP sensors (optical flow / rangefinder modules)
  DISPLAYPORT = 5,  // MSP DisplayPort OSD (DJI O3 / Walksnail / HDZero)
  ESC_TELEM = 6,    // BLHeli32 / AM32 / KISS serial ESC telemetry
  RANGEFINDER = 7,  // Benewake TFmini / TF-Luna / TF02
  BLACKBOX = 8,     // OpenLog style serial logger
  VTX_SA = 9,       // analog video transmitter, TBS SmartAudio (half duplex on the TX pin)
  VTX_TRAMP = 10,   // analog video transmitter, ImmersionRC Tramp
  CONFIGURATOR = 11,  // a second configurator link (ESP32-C3 WiFi bridge, Bluetooth serial): same as USB
};
constexpr int NUM_PORTS = 4;  // UART1, UART2, UART3, UDP (ESP32 WiFi)
constexpr int NUM_PORTS_EXT = 2;  // UART4, UART5 (v22, boards with more UARTs: H743, F405, F722)
constexpr int NUM_PORTS_EXT2 = 2;  // SERIAL6, SERIAL7 (v28: H743 7 UARTs, F405 / F722 6)

// Mode switches (Betaflight "Modes" tab / iNav): a box is active while its AUX channel is inside [lo, hi] us.
// Several ranges may drive the same box (any active range activates it). Flight mode priority when several
// boxes are active: LAND > RTH > MISSION > AUTOTUNE > CRUISE > POSHOLD > ALTHOLD > HORIZON > ANGLE > ACRO (no box).
enum class ModeBox : uint8_t {
  NONE = 0, ARM, ANGLE, HORIZON, ALTHOLD, POSHOLD, RTH, MISSION, LAND, TURTLE, BEEPER, CRUISE, AUTOTUNE,
  GPS_RESCUE,  // Betaflight-style rescue: RTH that needs neither compass nor barometer
  LAUNCH,      // launch control (racing start): hold a pitch angle on the ground until the throttle is raised
  SMART_RTL,   // ArduPilot SmartRTL: fly home along the recorded path, then RTH (land)
  COUNT
};
struct ModeRange { uint8_t box, ch; uint16_t lo, hi; };  // ch: 4 = AUX1 (CH5) ... 15 = CH16
constexpr int MODE_RANGES = 12;
inline bool boxOn(uint32_t boxes, ModeBox b) { return boxes & (1u << (int)b); }

// SD card wiring (v25). The SD bus modes use the chip's SD host (ESP32 / S3 / P4, STM32 SDMMC);
// SPI works on any 4 free pins but needs a free SPI bus (the OSD bus when no MAX7456 is fitted).
enum class SdMode : uint8_t { OFF = 0, SPI = 1, SDIO1 = 2, SDIO4 = 3 };
enum SdPin : uint8_t { SD_CLK = 0, SD_CMD, SD_D0, SD_D1, SD_D2, SD_D3, SD_PINS };

enum class FailsafeAction : uint8_t { DROP = 0, LAND = 1, RTH = 2, SMART_RTL = 3 };

// OSD elements (positions stored as row*32+col, bit15 = visible)
enum OsdElement : uint8_t {
  OSD_VBAT = 0, OSD_CURRENT, OSD_MAH, OSD_MODE, OSD_ARM_TIME, OSD_ALTITUDE, OSD_VARIO, OSD_HOME_DIST,
  OSD_HOME_DIR, OSD_GPS_SATS, OSD_SPEED, OSD_LINK, OSD_CROSSHAIR, OSD_HORIZON, OSD_WARNINGS, OSD_COORDS,
  OSD_ELEMENT_COUNT
};
// v24 OSD elements (second position array, appended to the config)
enum Osd2Element : uint8_t {
  OSD2_ESC_TEMP = 0, OSD2_DIST_TOTAL, OSD2_EFFICIENCY, OSD2_TIME_LEFT, OSD2_COUNT
};
// v26 OSD elements
enum Osd3Element : uint8_t {
  OSD3_BAT_BAR = 0, OSD3_CELL_V, OSD3_RSSI_DBM, OSD3_COUNT
};

struct AxisPid { float p, i, d, f; };  // Betaflight-compatible scale (e.g. P45 I80 D30 F60)
struct RateProfile { float center, max, expo; };

// iNav-style logic condition ("programming framework"): result = A <op> B, evaluated at 50 Hz;
// while true the action runs. Operands: type 0 constant (val), 1 RC channel (val 1-16, us),
// 2 flight value (val = LogicValue), 3 result of another rule (val = rule 0-7, 0/1).
enum LogicOperand : uint8_t { LOP_CONST = 0, LOP_RC = 1, LOP_FLIGHT = 2, LOP_RULE = 3 };
enum LogicValue : uint8_t {
  LV_ARMED = 0, LV_ALT_DM, LV_SPEED_CMS, LV_HOME_M, LV_VBAT_CV, LV_BAT_PCT, LV_AMPS_DA, LV_ARM_TIME_S,
  LV_MODE, LV_LINK_PCT, LV_ROLL_DEG, LV_PITCH_DEG, LV_SATS, LV_MAH, LV_COUNT
};
enum LogicOp : uint8_t { LO_TRUE = 0, LO_EQ, LO_GT, LO_LT, LO_AND, LO_OR, LO_NOT, LO_GE, LO_LE, LO_NE, LO_COUNT };
enum LogicAction : uint8_t {
  LA_NONE = 0,   // result only (usable by other rules)
  LA_BEEP,       // buzzer
  LA_MODE_BOX,   // activate mode box arg (ModeBox number) as if its switch were on
  LA_RC,         // override RC channel arg (1-16) with val (us)
  LA_VTX_POWER,  // VTX power level val (1-5)
  LA_MESSAGE,    // "Logic N" message to the OSD / GCS when it becomes true
  LA_NO_ARM,     // block arming
  LA_COUNT
};
struct LogicRule {
  uint8_t enabled, aType, op, bType;
  int16_t aVal, bVal;
  uint8_t action, actArg;
  int16_t actVal;
};
constexpr int LOGIC_RULES = 8;
// stick (-1..1) -> rate (deg/s), "actual" rates: centre sensitivity, max rate, expo.
// One definition for ACRO (commander), HORIZON (attitude control) and the web curve.
inline float stickRate(float s, const RateProfile& r) {
  float a = s < 0 ? -s : s;
  float s5 = s * s * s * s * s;
  float expof = a * (s5 * r.expo + s * (1.0f - r.expo));
  float move = r.max - r.center;
  return s * r.center + (move > 0 ? move : 0) * expof;
}

// RC pulse (us) -> stick -1..1 with a 1 % centre dead-band (commander and the fast loop share it)
inline float rcStickUs(uint16_t us) {
  float v = ((float)us - 1500.0f) / 500.0f;
  if (v > -0.01f && v < 0.01f) v = 0.0f;
  return v < -1.0f ? -1.0f : v > 1.0f ? 1.0f : v;
}

struct Config {
  uint16_t version;
  uint16_t size;

  // ---- system / outputs ----
  uint16_t loop_hz;
  uint8_t rx_protocol, motor_protocol, imu_align, mag_align, yaw_reversed, airmode;
  uint8_t motor_poles;
  uint8_t esc_telemetry;        // request KISS/BLHeli32 telemetry (needs ESC_TELEM port)
  float motor_idle;

  // ---- serial ports ----
  uint8_t port_fn[NUM_PORTS];
  uint32_t port_baud[NUM_PORTS];  // 0 = protocol default
  uint8_t wifi_enable;            // ESP32: access point + MAVLink over UDP 14550
  uint8_t web_enable;             // ESP32: web configurator on http://192.168.4.1 (starts the access point)
  uint8_t wifi_armed;             // 0: switch the access point off while armed (unless wifi_enable), 1: keep it
  uint8_t mav_sysid;

  // ---- gyro / D-term filters ----
  float gyro_lpf_hz, gyro_lpf2_hz, dterm_lpf_hz, acc_lpf_hz;
  float notch1_hz, notch1_cut, notch2_hz, notch2_cut;
  uint8_t dyn_notch_count;
  float dyn_notch_min, dyn_notch_max, dyn_notch_q;
  uint8_t rpm_harmonics;
  float rpm_min_hz, rpm_q;

  // ---- rate controller ----
  AxisPid pid[3];
  float iterm_limit, pidsum_limit, pidsum_limit_yaw;
  float tpa_rate, tpa_breakpoint, iterm_relax_cutoff, anti_gravity;
  RateProfile rates[3];

  // ---- self level ----
  float angle_p, angle_limit_deg, horizon_transition;
  uint8_t at_axes;              // autotune axes bitmask: 1 roll, 2 pitch, 4 yaw
  float at_rate;                // autotune twitch rate (deg/s, yaw uses 60 %)

  // ---- altitude ----
  float alt_pos_p, alt_vel_p, alt_vel_i, alt_vel_d, climb_rate_max, hover_throttle;
  uint8_t hover_learn;          // 0 off, 1 learn in flight, 2 learn and save on disarm (ArduPilot MOT_THST_HOVER)
  uint8_t alt_surface;          // 1: ALTHOLD/POSHOLD/CRUISE hold height above ground when the rangefinder sees it

  // ---- position ----
  float pos_p, pos_vel_p, pos_vel_i, pos_max_speed, pos_max_angle_deg, pos_max_accel;
  uint8_t gps_min_sats;
  float gps_max_hacc;           // reject fixes with worse horizontal accuracy (m)
  float gps_glitch_m;           // minimum position jump treated as a glitch (m)
  float gps_glitch_s;           // a glitch lasting longer than this resets the position to GPS (s)
  uint8_t gps_heading;          // 1: without a healthy compass, learn heading from GPS velocity
  uint8_t wind_comp;            // 1: feed the estimated wind/drag acceleration forward in position control
  float wind_drag;              // linear drag / mass (1/s), used to convert wind acceleration into m/s

  // ---- navigation (RTH / missions / landing) ----
  float rth_alt, rth_speed, rth_loiter_s, rth_home_radius;
  uint8_t rth_climb_first, rth_land;
  float land_speed, land_slow_speed, land_slow_alt;
  float wp_radius, wp_speed;
  uint8_t nav_yaw_mode;         // 0 hold heading, 1 face direction of travel
  float rally_max_m;            // RTH uses the nearest rally point closer than this (0 = always home)
  float nav_yaw_p;

  // ---- geofence ----
  uint8_t fence_enable, fence_action;
  float fence_radius, fence_alt_max;

  // ---- RC ----
  ModeRange mode_range[MODE_RANGES];
  uint16_t arm_throttle_max_us;

  // ---- failsafe ----
  uint16_t fs_delay_ms, fs_off_delay_ms;
  uint8_t fs_action;
  float fs_throttle;

  // ---- battery ----
  float vbat_scale, cur_scale, cur_offset;
  uint8_t cell_count;
  float vbat_warn_cell, vbat_crit_cell;
  uint16_t bat_capacity;        // mAh, 0 = unknown
  uint8_t bat_fs_action;        // 0 warn only, 1 land, 2 RTH
  uint8_t bat_reserve_pct;

  // ---- arming ----
  uint8_t arm_require_gps, gcs_arm;
  float arm_max_angle;

  // ---- estimator ----
  float est_baro_noise, est_acc_noise, est_gps_vel_noise;
  float rf_max_m;               // rangefinder usable range
  float flow_scale;
  uint8_t flow_align;           // 0..3 = 0/90/180/270 deg
  uint8_t flow_flip;            // bit0 flip X, bit1 flip Y

  // ---- OSD / logging ----
  uint8_t osd_backend;          // 0 off, 1 MAX7456 (analog), 2 MSP DisplayPort (digital)
  uint8_t osd_pal;
  uint16_t osd_pos[OSD_ELEMENT_COUNT];
  uint16_t bb_rate_hz;          // 0 = off
  uint8_t bb_backend;           // 1 serial (BLACKBOX port), 2 onboard flash, 3 SD card (v25)

  // ---- calibration ----
  float acc_bias[3];
  uint8_t acc_calibrated;
  float mag_bias[3], mag_scale[3];
  float mag_declination_deg;
  float level_trim_roll_deg, level_trim_pitch_deg;

  // ======== appended in version 16 (keep appending: see configMigrate) ========
  // ---- video transmitter (SmartAudio / Tramp port, or MSP for digital systems) ----
  uint8_t vtx_band;             // 1 A, 2 B, 3 E, 4 F (Fatshark), 5 R (Raceband), 6 L (Lowband)
  uint8_t vtx_channel;          // 1..8
  uint16_t vtx_freq;            // MHz; 0 = from band/channel
  uint8_t vtx_power;            // power level index 1..5 (VTX table: 25/100/200/400/600 mW)
  uint8_t vtx_pit;              // 1: pit mode (minimum power, for the bench)
  uint8_t vtx_low_disarmed;     // 1: lowest power while disarmed
  // ---- LED strip (WS2812 / SK6812) ----
  uint8_t led_pin;              // GPIO number, 0 = off (ESP32 only)
  uint8_t led_count;            // up to 32
  uint8_t led_brightness;       // percent
  uint8_t led_mode;             // 0 status colours, 1 orientation (front white, rear red), 2 fixed colour
  uint32_t led_color;           // 0xRRGGBB for mode 2

  // ======== appended in version 17 ========
  uint8_t dshot_bidir;          // 1: bidirectional DShot - motor RPM from the ESCs on the signal wire (RPM filter)
  uint8_t est_type;             // 0: per-axis Kalman filters, 1: 15-state navigation EKF with fault handling
  uint8_t at_steps;             // autotune steps: 1 P/D twitches, 2 feed-forward, 4 I-term, 8 filters
  float vibe_warn;              // vibration warning above this accelerometer vibration (m/s^2), 0 = off
  uint8_t launch_angle;         // launch control: pitch angle held on the ground (deg)
  uint8_t launch_thr;           // launch control: throttle (%) that starts the launch
  float rth_sanity_s;           // RTH / GPS rescue: no progress towards home for this long -> land (0 = off)
  // custom VTX table (Betaflight vtxtable): power levels and one user band "U" (band 7)
  uint8_t vtxt_powers;          // number of custom power levels (0 = built-in 25/100/200/400/600 mW)
  uint16_t vtxt_mw[8];          // power level in mW (shown in the UI, sent to Tramp)
  uint8_t vtxt_val[8];          // value sent to SmartAudio: dBm (SmartAudio 2.1), or the level index (1.x / 2.0)
  uint8_t vtxt_sa_index;        // 1: SmartAudio expects level indices instead of dBm (old VTX firmware)
  uint16_t vtxt_band[8];        // user band frequencies (MHz), used with vtx_band 7
  // ---- v18 ----
  float board_roll_deg;         // fine board alignment on top of imu_align (FC mounted at an angle)
  float board_pitch_deg;
  float board_yaw_deg;
  uint8_t motor_out[4];         // output (1-4) driving mixer motor 1-4: motor reordering without rewiring
  // ---- v19 ----
  uint8_t imu_sync;             // 1: the fast loop starts on the IMU's data-ready pin (boards with PIN_IMU_INT)
  // ---- v20 ----
  uint8_t dshot_edt;            // 1: ask bidirectional-DShot ESCs for extended telemetry (temp / volt / amp)
  // ---- v21: Betaflight flight feel ----
  uint8_t rc_smooth_hz;         // setpoint smoothing cutoff (0 = auto from the setpoint update rate)
  uint8_t crash_detect;         // 1: disarm after a crash (ArduPilot crash check + impact detection)
  uint16_t yaw_spin_thr;        // yaw spin recovery above this rate (deg/s), 0 = off
  uint8_t d_min_pct;            // D at rest in % of D (Betaflight d_min), 0 or 100 = constant D
  uint8_t d_min_gain;           // how strongly gyro activity (prop wash) raises D back to full
  uint8_t d_min_adv;            // how strongly stick movement raises D (advance)
  uint16_t dyn_lpf_min;         // dynamic gyro low-pass: cutoff at zero throttle (Hz), 0 = static gyro_lpf2_hz
  uint16_t dyn_lpf_max;         // ... cutoff at full throttle (Hz)
  uint8_t dyn_idle_rpm;         // dynamic idle: minimum motor RPM / 100 while armed (needs RPM), 0 = off
  uint8_t thr_boost;            // throttle boost (Betaflight throttle_boost, 0-10)
  float thrust_expo;            // thrust linearisation (ArduPilot MOT_THST_EXPO, 0 = off, typ. 0.6)
  uint8_t vbat_sag_comp;        // battery sag compensation (%): outputs behave like on a full battery
  // ---- v21: PID / rate profiles (profile 1 = pid / rates above) ----
  uint8_t pid_profile;          // active profile 0-2 when no switch is assigned
  uint8_t prof_ch;              // AUX channel (5-16) selecting the profile: low / mid / high, 0 = none
  AxisPid prof_pid[2][3];       // profiles 2 and 3
  RateProfile prof_rates[2][3];
  // ---- v21: ArduPilot ----
  uint8_t gcs_fs;               // GCS link lost (after a heartbeat was seen) in GUIDED / MISSION: 0 off, 1 RTH, 2 LAND, 3 SmartRTL
  float gcs_fs_s;               // seconds without a GCS heartbeat
  float srtl_point_m;           // SmartRTL: path point spacing (m), 0 = SmartRTL off
  float acc_scale[3];           // 6-point accelerometer calibration: scale per axis (1 = none)
  float mag_offdiag[3];         // ellipsoid compass calibration: soft-iron off-diagonal (xy, xz, yz)
  float mag_cmot[3];            // compass motor interference per amp (gauss / A, compassmot)
  uint8_t imu_dual;             // 1: second IMU (IMU2 chip select) takes over when the first fails
  uint8_t fence_poly;           // 1: polygon fence (uploaded from the GCS) in addition to the cylinder
  // ---- v21: iNav logic conditions ----
  LogicRule logic[LOGIC_RULES];
  // ---- v22: serial ports 4 and 5 ----
  uint8_t port_fn_ext[NUM_PORTS_EXT];
  uint32_t port_baud_ext[NUM_PORTS_EXT];
  // ---- v23: OSD canvas / font ----
  uint8_t osd_canvas;  // 0 auto, 1 SD 30x16, 2 HD 50x18 (HDZero), 3 HD 53x20 (DJI O3 / O4, Walksnail), 4 HD 60x22
  uint8_t osd_font;    // 0 Betaflight glyphs (icons), 1 plain text
  // ---- v24: RPM filter harmonic weights, more OSD elements, post-flight statistics ----
  uint8_t rpm_weight[3];          // % of each harmonic's notch (Betaflight rpm_filter_weights)
  uint8_t osd_stats;              // statistics screen after disarm (s, 0 = off)
  uint16_t osd_pos2[OSD2_COUNT];  // ESC temperature, total distance, mAh/km, flight time left
  // ---- v25: SD card ----
  uint8_t sd_mode;    // SdMode: 0 off, 1 SPI, 2 SD bus 1-bit, 3 SD bus 4-bit
  uint8_t sd_mhz;     // bus clock (MHz), 0 = default for the mode
  uint8_t sd_pin[6];  // CLK, CMD, D0, D1, D2, D3 (SPI: SCK, MOSI, MISO, -, -, CS); 255 = board default
  uint8_t sd_keep;    // card nearly full: 1 = delete the oldest logs to make room, 0 = stop logging
  // ---- v26: OSD extras, SD card DMA ----
  uint16_t osd_pos3[OSD3_COUNT];  // battery bar, average cell voltage, RSSI (dBm)
  uint8_t osd_blink;              // 1: critical warnings (failsafe, battery critical...) and a low battery blink
  uint8_t sd_dma;                 // STM32H7 SDMMC: 1 = blocks moved by the controller's DMA, 0 = by the CPU (polled)
  // ---- v27: feed-forward shaping ----
  uint8_t ff_jitter;              // stick moves smaller than this per RC frame get less FF (0 = off)
  uint8_t ff_boost;               // stick acceleration added to FF (0 = off)
  uint8_t ff_max_pct;             // FF fades out above this % of the max rate (0 / 100 = off)
  // ---- v28: serial ports 6 and 7 ----
  uint8_t port_fn_ext2[NUM_PORTS_EXT2];
  uint32_t port_baud_ext2[NUM_PORTS_EXT2];

  uint32_t crc;
};

void configDefaults(Config& c);
// active boxes (bit i = ModeBox i) for the given RC channels
uint32_t modeBoxes(const Config& c, const uint16_t* ch, int nch);
const char* modeBoxName(int box);
uint32_t configCrc(const Config& c);
void configSeal(Config& c);
bool configValid(const Config& c);
// an older stored config (same layout, fewer fields at the end): keep its values, defaults for the rest
bool configMigrate(Config& c);
uint32_t crc32(const void* data, size_t len, uint32_t crc = 0);

// ---------------------------------------------------------------- parameters
enum class PType : uint8_t { F32, U8, U16, U32 };
struct ParamInfo {
  const char* name;     // <= 16 chars (MAVLink limit)
  PType type;
  uint16_t offset;
  float min, max;
  bool reboot;          // change needs a reboot
};
int paramCount();
const ParamInfo& paramInfo(int i);
// function / baud of a port by hal::Port index (0-2 SERIAL1-3, 3 UDP, 4 console, 5-6 SERIAL4-5)
inline uint8_t portFnOf(const Config& c, int port) {
  if (port >= 0 && port < NUM_PORTS) return c.port_fn[port];
  if (port >= 5 && port < 5 + NUM_PORTS_EXT) return c.port_fn_ext[port - 5];
  if (port >= 7 && port < 7 + NUM_PORTS_EXT2) return c.port_fn_ext2[port - 7];
  return 0;
}
inline uint32_t portBaudOf(const Config& c, int port) {
  if (port >= 0 && port < NUM_PORTS) return c.port_baud[port];
  if (port >= 5 && port < 5 + NUM_PORTS_EXT) return c.port_baud_ext[port - 5];
  if (port >= 7 && port < 7 + NUM_PORTS_EXT2) return c.port_baud_ext2[port - 7];
  return 0;
}
// gains of the active PID / rate profile (0 = pid / rates, 1-2 = prof_pid / prof_rates)
inline const AxisPid& profilePid(const Config& c, int profile, int axis) {
  return profile <= 0 || profile > 2 ? c.pid[axis] : c.prof_pid[profile - 1][axis];
}
inline const RateProfile& profileRate(const Config& c, int profile, int axis) {
  return profile <= 0 || profile > 2 ? c.rates[axis] : c.prof_rates[profile - 1][axis];
}
int paramFind(const char* name);
float paramGet(const Config& c, int i);
bool paramSet(Config& c, int i, float v);  // clamps, false if out of range

}  // namespace fc
