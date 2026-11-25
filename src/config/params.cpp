// AllQuad - parameter table shared by the CLI, MAVLink PARAM protocol and MSP
#include "../common/size_opt.h"  // -Os on ESP32 (not flight critical)
#include <string.h>

#include "config.h"

namespace fc {

#define P(n, t, field, lo, hi) {n, PType::t, (uint16_t)offsetof(Config, field), lo, hi, false}
#define PR(n, t, field, lo, hi) {n, PType::t, (uint16_t)offsetof(Config, field), lo, hi, true}

static const ParamInfo PARAMS[] = {
    // system / outputs
    PR("loop_hz", U16, loop_hz, 500, AQ_LOOP_HZ_MAX),
    PR("rx_protocol", U8, rx_protocol, 0, 2),
    PR("motor_protocol", U8, motor_protocol, 0, 3),
    P("imu_align", U8, imu_align, 0, 7),
    P("board_roll", F32, board_roll_deg, -180, 180), P("board_pitch", F32, board_pitch_deg, -180, 180),
    P("board_yaw", F32, board_yaw_deg, -180, 180),
    P("motor1_out", U8, motor_out[0], 1, 4), P("motor2_out", U8, motor_out[1], 1, 4),
    P("motor3_out", U8, motor_out[2], 1, 4), P("motor4_out", U8, motor_out[3], 1, 4),
    PR("imu_sync", U8, imu_sync, 0, 1),
    P("dshot_edt", U8, dshot_edt, 0, 1),
    P("mag_align", U8, mag_align, 0, 7),
    P("yaw_reversed", U8, yaw_reversed, 0, 1),
    P("airmode", U8, airmode, 0, 1),
    P("motor_poles", U8, motor_poles, 2, 40),
    PR("esc_telemetry", U8, esc_telemetry, 0, 1),
    P("motor_idle", F32, motor_idle, 0, 0.2f),
    // ports
    PR("serial1_fn", U8, port_fn[0], 0, 12), PR("serial2_fn", U8, port_fn[1], 0, 12),
    PR("serial3_fn", U8, port_fn[2], 0, 12), PR("udp_fn", U8, port_fn[3], 0, 8),
    PR("serial1_baud", U32, port_baud[0], 0, 2000000), PR("serial2_baud", U32, port_baud[1], 0, 2000000),
    PR("serial3_baud", U32, port_baud[2], 0, 2000000),
    PR("serial4_fn", U8, port_fn_ext[0], 0, 12), PR("serial5_fn", U8, port_fn_ext[1], 0, 12),
    PR("serial4_baud", U32, port_baud_ext[0], 0, 2000000), PR("serial5_baud", U32, port_baud_ext[1], 0, 2000000),
    PR("serial6_fn", U8, port_fn_ext2[0], 0, 12), PR("serial7_fn", U8, port_fn_ext2[1], 0, 12),
    PR("serial6_baud", U32, port_baud_ext2[0], 0, 2000000), PR("serial7_baud", U32, port_baud_ext2[1], 0, 2000000),
    // ---- v29 ----
    P("thr_limit_pct", U8, thr_limit_pct, 25, 100),
    P("motor_limit_pct", U8, motor_limit_pct, 50, 100),
    P("beginner", U8, beginner, 0, 1),
    P("beg_radius", U16, beg_radius, 20, 500),
    P("rc_ch_roll", U8, rc_map[0], 1, 4), P("rc_ch_pitch", U8, rc_map[1], 1, 4),
    P("rc_ch_thr", U8, rc_map[2], 1, 4), P("rc_ch_yaw", U8, rc_map[3], 1, 4),
    P("rc_deadband", U8, rc_deadband, 0, 50),
    PR("wifi_enable", U8, wifi_enable, 0, 1),
    PR("web_enable", U8, web_enable, 0, 1),
    P("wifi_armed", U8, wifi_armed, 0, 1),
    P("mav_sysid", U8, mav_sysid, 1, 250),
    // filters
    P("gyro_lpf_hz", F32, gyro_lpf_hz, 0, 500),
    P("gyro_lpf2_hz", F32, gyro_lpf2_hz, 0, 1000),
    P("dterm_lpf_hz", F32, dterm_lpf_hz, 0, 300),
    P("acc_lpf_hz", F32, acc_lpf_hz, 1, 100),
    P("notch1_hz", F32, notch1_hz, 0, 900), P("notch1_cut", F32, notch1_cut, 0, 900),
    P("notch2_hz", F32, notch2_hz, 0, 900), P("notch2_cut", F32, notch2_cut, 0, 900),
    P("dyn_notch_count", U8, dyn_notch_count, 0, 2),
    P("dyn_notch_min", F32, dyn_notch_min, 50, 300),
    P("dyn_notch_max", F32, dyn_notch_max, 150, 900),
    P("dyn_notch_q", F32, dyn_notch_q, 1, 10),
    P("rpm_harmonics", U8, rpm_harmonics, 0, 3),
    P("rpm_min_hz", F32, rpm_min_hz, 50, 300),
    P("rpm_q", F32, rpm_q, 1, 10),
    P("rpm_w1", U8, rpm_weight[0], 0, 100), P("rpm_w2", U8, rpm_weight[1], 0, 100), P("rpm_w3", U8, rpm_weight[2], 0, 100),
    // rate controller
    P("roll_p", F32, pid[0].p, 0, 250), P("roll_i", F32, pid[0].i, 0, 250),
    P("roll_d", F32, pid[0].d, 0, 250), P("roll_f", F32, pid[0].f, 0, 1000),
    P("pitch_p", F32, pid[1].p, 0, 250), P("pitch_i", F32, pid[1].i, 0, 250),
    P("pitch_d", F32, pid[1].d, 0, 250), P("pitch_f", F32, pid[1].f, 0, 1000),
    P("yaw_p", F32, pid[2].p, 0, 250), P("yaw_i", F32, pid[2].i, 0, 250),
    P("yaw_d", F32, pid[2].d, 0, 250), P("yaw_f", F32, pid[2].f, 0, 1000),
    P("iterm_limit", F32, iterm_limit, 0, 500),
    P("pidsum_limit", F32, pidsum_limit, 100, 1000),
    P("pidsum_lim_yaw", F32, pidsum_limit_yaw, 100, 1000),
    P("tpa_rate", F32, tpa_rate, 0, 1),
    P("tpa_breakpoint", F32, tpa_breakpoint, 0, 1),
    P("iterm_relax", F32, iterm_relax_cutoff, 0, 500),
    P("anti_gravity", F32, anti_gravity, 0, 10),
    P("roll_rc_rate", F32, rates[0].center, 0, 500), P("roll_max_rate", F32, rates[0].max, 0, 2000),
    P("roll_expo", F32, rates[0].expo, 0, 1),
    P("pitch_rc_rate", F32, rates[1].center, 0, 500), P("pitch_max_rate", F32, rates[1].max, 0, 2000),
    P("pitch_expo", F32, rates[1].expo, 0, 1),
    P("yaw_rc_rate", F32, rates[2].center, 0, 500), P("yaw_max_rate", F32, rates[2].max, 0, 2000),
    P("yaw_expo", F32, rates[2].expo, 0, 1),
    // level
    P("angle_p", F32, angle_p, 0, 20),
    P("angle_limit", F32, angle_limit_deg, 10, 80),
    P("horizon_trans", F32, horizon_transition, 0.1f, 1),
    P("at_axes", U8, at_axes, 1, 7),
    P("at_rate", F32, at_rate, 90, 400),
    // altitude
    P("alt_pos_p", F32, alt_pos_p, 0, 5),
    P("alt_vel_p", F32, alt_vel_p, 0, 1),
    P("alt_vel_i", F32, alt_vel_i, 0, 1),
    P("alt_vel_d", F32, alt_vel_d, 0, 0.5f),
    P("climb_rate_max", F32, climb_rate_max, 0.2f, 10),
    P("hover_throttle", F32, hover_throttle, 0.1f, 0.8f),
    P("hover_learn", U8, hover_learn, 0, 2),
    P("alt_surface", U8, alt_surface, 0, 1),
    P("thr_curve", F32, thr_curve, 0, 1),
    // position
    P("pos_p", F32, pos_p, 0, 5),
    P("pos_vel_p", F32, pos_vel_p, 0, 10),
    P("pos_vel_i", F32, pos_vel_i, 0, 5),
    P("pos_max_speed", F32, pos_max_speed, 0.5f, 25),
    P("pos_max_angle", F32, pos_max_angle_deg, 5, 45),
    P("pos_max_accel", F32, pos_max_accel, 0.5f, 10),
    P("gps_min_sats", U8, gps_min_sats, 4, 20),
    P("gps_max_hacc", F32, gps_max_hacc, 1, 50),
    P("gps_glitch_m", F32, gps_glitch_m, 2, 100),
    P("gps_glitch_s", F32, gps_glitch_s, 1, 60),
    P("gps_heading", U8, gps_heading, 0, 1),
    P("wind_comp", U8, wind_comp, 0, 1),
    P("wind_drag", F32, wind_drag, 0.05f, 3),
    // navigation
    P("rth_alt", F32, rth_alt, 2, 200),
    P("rth_speed", F32, rth_speed, 1, 20),
    P("rth_loiter_s", F32, rth_loiter_s, 0, 120),
    P("rth_home_radius", F32, rth_home_radius, 0.5f, 20),
    P("rth_climb_first", U8, rth_climb_first, 0, 1),
    P("rth_land", U8, rth_land, 0, 1),
    P("land_speed", F32, land_speed, 0.2f, 3),
    P("land_slow_speed", F32, land_slow_speed, 0.1f, 1),
    P("land_slow_alt", F32, land_slow_alt, 0.5f, 20),
    P("wp_radius", F32, wp_radius, 0.5f, 20),
    P("wp_speed", F32, wp_speed, 0.5f, 20),
    P("nav_yaw_mode", U8, nav_yaw_mode, 0, 1),
    P("rally_max_m", F32, rally_max_m, 0, 10000),
    P("nav_yaw_p", F32, nav_yaw_p, 0, 10),
    // geofence
    P("fence_enable", U8, fence_enable, 0, 1),
    P("fence_action", U8, fence_action, 1, 2),
    P("fence_radius", F32, fence_radius, 10, 5000),
    P("fence_alt_max", F32, fence_alt_max, 5, 1000),
    // RC
    // mode ranges: mrN_mode 0 unused, 1 ARM, 2 ANGLE, 3 HORIZON, 4 ALTHOLD, 5 POSHOLD, 6 RTH, 7 MISSION,
    //              8 LAND, 9 TURTLE, 10 BEEPER, 11 CRUISE, 12 AUTOTUNE, 13 GPS RESCUE, 14 LAUNCH; mrN_ch 4 = AUX1 (CH5); active while lo <= us < hi
    P("mr1_mode", U8, mode_range[0].box, 0, 20), P("mr1_ch", U8, mode_range[0].ch, 4, 15),
    P("mr1_lo", U16, mode_range[0].lo, 900, 2100), P("mr1_hi", U16, mode_range[0].hi, 900, 2100),
    P("mr2_mode", U8, mode_range[1].box, 0, 20), P("mr2_ch", U8, mode_range[1].ch, 4, 15),
    P("mr2_lo", U16, mode_range[1].lo, 900, 2100), P("mr2_hi", U16, mode_range[1].hi, 900, 2100),
    P("mr3_mode", U8, mode_range[2].box, 0, 20), P("mr3_ch", U8, mode_range[2].ch, 4, 15),
    P("mr3_lo", U16, mode_range[2].lo, 900, 2100), P("mr3_hi", U16, mode_range[2].hi, 900, 2100),
    P("mr4_mode", U8, mode_range[3].box, 0, 20), P("mr4_ch", U8, mode_range[3].ch, 4, 15),
    P("mr4_lo", U16, mode_range[3].lo, 900, 2100), P("mr4_hi", U16, mode_range[3].hi, 900, 2100),
    P("mr5_mode", U8, mode_range[4].box, 0, 20), P("mr5_ch", U8, mode_range[4].ch, 4, 15),
    P("mr5_lo", U16, mode_range[4].lo, 900, 2100), P("mr5_hi", U16, mode_range[4].hi, 900, 2100),
    P("mr6_mode", U8, mode_range[5].box, 0, 20), P("mr6_ch", U8, mode_range[5].ch, 4, 15),
    P("mr6_lo", U16, mode_range[5].lo, 900, 2100), P("mr6_hi", U16, mode_range[5].hi, 900, 2100),
    P("mr7_mode", U8, mode_range[6].box, 0, 20), P("mr7_ch", U8, mode_range[6].ch, 4, 15),
    P("mr7_lo", U16, mode_range[6].lo, 900, 2100), P("mr7_hi", U16, mode_range[6].hi, 900, 2100),
    P("mr8_mode", U8, mode_range[7].box, 0, 20), P("mr8_ch", U8, mode_range[7].ch, 4, 15),
    P("mr8_lo", U16, mode_range[7].lo, 900, 2100), P("mr8_hi", U16, mode_range[7].hi, 900, 2100),
    P("mr9_mode", U8, mode_range[8].box, 0, 20), P("mr9_ch", U8, mode_range[8].ch, 4, 15),
    P("mr9_lo", U16, mode_range[8].lo, 900, 2100), P("mr9_hi", U16, mode_range[8].hi, 900, 2100),
    P("mr10_mode", U8, mode_range[9].box, 0, 20), P("mr10_ch", U8, mode_range[9].ch, 4, 15),
    P("mr10_lo", U16, mode_range[9].lo, 900, 2100), P("mr10_hi", U16, mode_range[9].hi, 900, 2100),
    P("mr11_mode", U8, mode_range[10].box, 0, 20), P("mr11_ch", U8, mode_range[10].ch, 4, 15),
    P("mr11_lo", U16, mode_range[10].lo, 900, 2100), P("mr11_hi", U16, mode_range[10].hi, 900, 2100),
    P("mr12_mode", U8, mode_range[11].box, 0, 20), P("mr12_ch", U8, mode_range[11].ch, 4, 15),
    P("mr12_lo", U16, mode_range[11].lo, 900, 2100), P("mr12_hi", U16, mode_range[11].hi, 900, 2100),
    P("arm_throttle_max", U16, arm_throttle_max_us, 1000, 1300),
    // failsafe
    P("fs_delay_ms", U16, fs_delay_ms, 100, 2000),
    P("fs_off_delay_ms", U16, fs_off_delay_ms, 1000, 60000),
    P("fs_action", U8, fs_action, 0, 3),
    P("fs_throttle", F32, fs_throttle, 0, 0.8f),
    // battery
    P("vbat_scale", F32, vbat_scale, 1, 50),
    P("cur_scale", F32, cur_scale, 0, 500),
    P("cur_offset", F32, cur_offset, -3.3f, 3.3f),
    P("cell_count", U8, cell_count, 0, 12),
    P("vbat_warn_cell", F32, vbat_warn_cell, 2.8f, 4.2f),
    P("vbat_crit_cell", F32, vbat_crit_cell, 2.8f, 4.2f),
    P("bat_capacity", U16, bat_capacity, 0, 30000),
    P("bat_fs_action", U8, bat_fs_action, 0, 3),
    P("bat_reserve_pct", U8, bat_reserve_pct, 0, 50),
    // arming
    P("arm_require_gps", U8, arm_require_gps, 0, 1),
    P("gcs_arm", U8, gcs_arm, 0, 1),
    P("arm_max_angle", F32, arm_max_angle, 5, 180),
    // estimator / sensors
    P("est_baro_noise", F32, est_baro_noise, 0.05f, 5),
    P("est_acc_noise", F32, est_acc_noise, 0.05f, 5),
    P("est_gpsvel_noise", F32, est_gps_vel_noise, 0.05f, 5),
    P("baro_gnd", U8, baro_gnd, 0, 1),
    P("bat_smart_rth", U8, bat_smart_rth, 0, 2),
    P("dr_time_s", U8, dr_time_s, 0, 120),
    P("mag_learn", U8, mag_learn, 0, 1),
    P("motor_warn", U8, motor_warn, 0, 1),
    P("beacon_s", U8, beacon_s, 0, 120),
    P("stick_cmds", U8, stick_cmds, 0, 1),
    P("headfree_type", U8, headfree_type, 0, 1),
    P("circle_radius", F32, circle_radius, 2, 200),
    P("circle_speed", F32, circle_speed, -15, 15),
    P("follow_dist", F32, follow_dist, 2, 100),
    P("follow_alt", F32, follow_alt, 0, 120),
    P("baro_dyn", F32, baro_dyn, -0.1f, 0.1f),
    P("rf_max_m", F32, rf_max_m, 0.5f, 40),
    P("flow_scale", F32, flow_scale, 0.1f, 100),
    P("flow_align", U8, flow_align, 0, 3),
    P("flow_flip", U8, flow_flip, 0, 3),
    // OSD / blackbox
    PR("osd_backend", U8, osd_backend, 0, 2),
    P("osd_pal", U8, osd_pal, 0, 1),
    PR("osd_canvas", U8, osd_canvas, 0, 4),
    P("osd_font", U8, osd_font, 0, 1),
    P("osd_stats", U8, osd_stats, 0, 60),
    P("osd_esct_pos", U16, osd_pos2[OSD2_ESC_TEMP], 0, 65535),
    P("osd_dist_pos", U16, osd_pos2[OSD2_DIST_TOTAL], 0, 65535),
    P("osd_eff_pos", U16, osd_pos2[OSD2_EFFICIENCY], 0, 65535),
    P("osd_tleft_pos", U16, osd_pos2[OSD2_TIME_LEFT], 0, 65535),
    P("osd_batbar_pos", U16, osd_pos3[OSD3_BAT_BAR], 0, 65535),
    P("osd_cell_pos", U16, osd_pos3[OSD3_CELL_V], 0, 65535),
    P("osd_rssi_pos", U16, osd_pos3[OSD3_RSSI_DBM], 0, 65535),
    P("osd_blink", U8, osd_blink, 0, 1),
    P("osd_vbat_pos", U16, osd_pos[OSD_VBAT], 0, 65535),
    P("osd_cur_pos", U16, osd_pos[OSD_CURRENT], 0, 65535),
    P("osd_mah_pos", U16, osd_pos[OSD_MAH], 0, 65535),
    P("osd_mode_pos", U16, osd_pos[OSD_MODE], 0, 65535),
    P("osd_time_pos", U16, osd_pos[OSD_ARM_TIME], 0, 65535),
    P("osd_alt_pos", U16, osd_pos[OSD_ALTITUDE], 0, 65535),
    P("osd_vario_pos", U16, osd_pos[OSD_VARIO], 0, 65535),
    P("osd_hdist_pos", U16, osd_pos[OSD_HOME_DIST], 0, 65535),
    P("osd_hdir_pos", U16, osd_pos[OSD_HOME_DIR], 0, 65535),
    P("osd_sats_pos", U16, osd_pos[OSD_GPS_SATS], 0, 65535),
    P("osd_speed_pos", U16, osd_pos[OSD_SPEED], 0, 65535),
    P("osd_link_pos", U16, osd_pos[OSD_LINK], 0, 65535),
    P("osd_cross_pos", U16, osd_pos[OSD_CROSSHAIR], 0, 65535),
    P("osd_horizon_pos", U16, osd_pos[OSD_HORIZON], 0, 65535),
    P("osd_warn_pos", U16, osd_pos[OSD_WARNINGS], 0, 65535),
    P("osd_coord_pos", U16, osd_pos[OSD_COORDS], 0, 65535),
    PR("bb_rate_hz", U16, bb_rate_hz, 0, 1000),
    P("bb_gyro_s", U8, bb_gyro_s, 0, 30),
    PR("bb_backend", U8, bb_backend, 1, 3),
    PR("sd_mode", U8, sd_mode, 0, 3), PR("sd_mhz", U8, sd_mhz, 0, 40),
    PR("sd_clk", U8, sd_pin[SD_CLK], 0, 255), PR("sd_cmd", U8, sd_pin[SD_CMD], 0, 255),
    PR("sd_d0", U8, sd_pin[SD_D0], 0, 255), PR("sd_d1", U8, sd_pin[SD_D1], 0, 255),
    PR("sd_d2", U8, sd_pin[SD_D2], 0, 255), PR("sd_d3", U8, sd_pin[SD_D3], 0, 255),
    P("sd_keep", U8, sd_keep, 0, 1),
    PR("sd_dma", U8, sd_dma, 0, 1),
    // calibration
    P("acc_bias_x", F32, acc_bias[0], -5, 5), P("acc_bias_y", F32, acc_bias[1], -5, 5),
    P("acc_bias_z", F32, acc_bias[2], -5, 5),
    P("acc_calibrated", U8, acc_calibrated, 0, 1),
    P("mag_bias_x", F32, mag_bias[0], -10, 10), P("mag_bias_y", F32, mag_bias[1], -10, 10),
    P("mag_bias_z", F32, mag_bias[2], -10, 10),
    P("mag_scale_x", F32, mag_scale[0], 0.2f, 5), P("mag_scale_y", F32, mag_scale[1], 0.2f, 5),
    P("mag_scale_z", F32, mag_scale[2], 0.2f, 5),
    P("mag_declination", F32, mag_declination_deg, -180, 180),
    P("level_trim_roll", F32, level_trim_roll_deg, -10, 10),
    P("level_trim_pitch", F32, level_trim_pitch_deg, -10, 10),
    // video transmitter
    P("vtx_band", U8, vtx_band, 1, 7), P("vtx_channel", U8, vtx_channel, 1, 8),
    P("vtx_freq", U16, vtx_freq, 0, 6000), P("vtx_power", U8, vtx_power, 1, 8),
    P("vtx_pit", U8, vtx_pit, 0, 1), P("vtx_low_disarm", U8, vtx_low_disarmed, 0, 1),
    // LED strip
    PR("led_pin", U8, led_pin, 0, 48), P("led_count", U8, led_count, 1, 32),
    P("led_brightness", U8, led_brightness, 1, 100), P("led_mode", U8, led_mode, 0, 2),
    P("led_color", U32, led_color, 0, 16777215),
    // version 17
    PR("dshot_bidir", U8, dshot_bidir, 0, 1),
    P("est_type", U8, est_type, 0, 1),
    P("at_steps", U8, at_steps, 1, 15),
    P("vibe_warn", F32, vibe_warn, 0, 200),
    P("launch_angle", U8, launch_angle, 0, 80),
    P("launch_thr", U8, launch_thr, 5, 70),
    P("rth_sanity_s", F32, rth_sanity_s, 0, 120),
    P("vtxt_powers", U8, vtxt_powers, 0, 8),
    P("vtxt_sa_index", U8, vtxt_sa_index, 0, 1),
#define VTXT(i) P("vtxt_mw" #i, U16, vtxt_mw[i - 1], 0, 10000), P("vtxt_val" #i, U8, vtxt_val[i - 1], 0, 255), \
                P("vtxt_band" #i, U16, vtxt_band[i - 1], 0, 6000)
    VTXT(1), VTXT(2), VTXT(3), VTXT(4), VTXT(5), VTXT(6), VTXT(7), VTXT(8),
#undef VTXT
    // ---- v21: flight feel / safety (Betaflight) ----
    P("rc_smooth_hz", U8, rc_smooth_hz, 0, 250),
    P("crash_detect", U8, crash_detect, 0, 1),
    P("yaw_spin_thr", U16, yaw_spin_thr, 0, 2000),
    P("d_min_pct", U8, d_min_pct, 0, 100),
    P("d_min_gain", U8, d_min_gain, 0, 100),
    P("d_min_adv", U8, d_min_adv, 0, 200),
    P("ff_jitter", U8, ff_jitter, 0, 20),
    P("ff_boost", U8, ff_boost, 0, 50),
    P("ff_max_pct", U8, ff_max_pct, 0, 100),
    P("dyn_lpf_min", U16, dyn_lpf_min, 0, 1000),
    P("dyn_lpf_max", U16, dyn_lpf_max, 0, 1000),
    P("dyn_idle_rpm", U8, dyn_idle_rpm, 0, 100),
    P("thr_boost", U8, thr_boost, 0, 10),
    P("thrust_expo", F32, thrust_expo, 0, 1),
    P("vbat_sag_comp", U8, vbat_sag_comp, 0, 100),
    P("pid_profile", U8, pid_profile, 0, 2),
    P("prof_ch", U8, prof_ch, 0, 16),
    P("p2_roll_p", F32, prof_pid[0][0].p, 0, 250), P("p2_roll_i", F32, prof_pid[0][0].i, 0, 250),
    P("p2_roll_d", F32, prof_pid[0][0].d, 0, 250), P("p2_roll_f", F32, prof_pid[0][0].f, 0, 1000),
    P("p2_roll_rc", F32, prof_rates[0][0].center, 0, 500), P("p2_roll_max", F32, prof_rates[0][0].max, 0, 2000),
    P("p2_roll_expo", F32, prof_rates[0][0].expo, 0, 1),
    P("p2_pitch_p", F32, prof_pid[0][1].p, 0, 250), P("p2_pitch_i", F32, prof_pid[0][1].i, 0, 250),
    P("p2_pitch_d", F32, prof_pid[0][1].d, 0, 250), P("p2_pitch_f", F32, prof_pid[0][1].f, 0, 1000),
    P("p2_pitch_rc", F32, prof_rates[0][1].center, 0, 500), P("p2_pitch_max", F32, prof_rates[0][1].max, 0, 2000),
    P("p2_pitch_expo", F32, prof_rates[0][1].expo, 0, 1),
    P("p2_yaw_p", F32, prof_pid[0][2].p, 0, 250), P("p2_yaw_i", F32, prof_pid[0][2].i, 0, 250),
    P("p2_yaw_d", F32, prof_pid[0][2].d, 0, 250), P("p2_yaw_f", F32, prof_pid[0][2].f, 0, 1000),
    P("p2_yaw_rc", F32, prof_rates[0][2].center, 0, 500), P("p2_yaw_max", F32, prof_rates[0][2].max, 0, 2000),
    P("p2_yaw_expo", F32, prof_rates[0][2].expo, 0, 1),
    P("p3_roll_p", F32, prof_pid[1][0].p, 0, 250), P("p3_roll_i", F32, prof_pid[1][0].i, 0, 250),
    P("p3_roll_d", F32, prof_pid[1][0].d, 0, 250), P("p3_roll_f", F32, prof_pid[1][0].f, 0, 1000),
    P("p3_roll_rc", F32, prof_rates[1][0].center, 0, 500), P("p3_roll_max", F32, prof_rates[1][0].max, 0, 2000),
    P("p3_roll_expo", F32, prof_rates[1][0].expo, 0, 1),
    P("p3_pitch_p", F32, prof_pid[1][1].p, 0, 250), P("p3_pitch_i", F32, prof_pid[1][1].i, 0, 250),
    P("p3_pitch_d", F32, prof_pid[1][1].d, 0, 250), P("p3_pitch_f", F32, prof_pid[1][1].f, 0, 1000),
    P("p3_pitch_rc", F32, prof_rates[1][1].center, 0, 500), P("p3_pitch_max", F32, prof_rates[1][1].max, 0, 2000),
    P("p3_pitch_expo", F32, prof_rates[1][1].expo, 0, 1),
    P("p3_yaw_p", F32, prof_pid[1][2].p, 0, 250), P("p3_yaw_i", F32, prof_pid[1][2].i, 0, 250),
    P("p3_yaw_d", F32, prof_pid[1][2].d, 0, 250), P("p3_yaw_f", F32, prof_pid[1][2].f, 0, 1000),
    P("p3_yaw_rc", F32, prof_rates[1][2].center, 0, 500), P("p3_yaw_max", F32, prof_rates[1][2].max, 0, 2000),
    P("p3_yaw_expo", F32, prof_rates[1][2].expo, 0, 1),
    // ---- v21: ArduPilot ----
    P("gcs_fs", U8, gcs_fs, 0, 3),
    P("gcs_fs_s", F32, gcs_fs_s, 1, 60),
    P("srtl_point_m", F32, srtl_point_m, 0, 20),
    P("acc_scale_x", F32, acc_scale[0], 0.8f, 1.2f), P("acc_scale_y", F32, acc_scale[1], 0.8f, 1.2f),
    P("acc_scale_z", F32, acc_scale[2], 0.8f, 1.2f),
    P("mag_odiag_xy", F32, mag_offdiag[0], -0.5f, 0.5f), P("mag_odiag_xz", F32, mag_offdiag[1], -0.5f, 0.5f),
    P("mag_odiag_yz", F32, mag_offdiag[2], -0.5f, 0.5f),
    P("mag_cmot_x", F32, mag_cmot[0], -0.05f, 0.05f), P("mag_cmot_y", F32, mag_cmot[1], -0.05f, 0.05f),
    P("mag_cmot_z", F32, mag_cmot[2], -0.05f, 0.05f),
    PR("imu_dual", U8, imu_dual, 0, 1),
    P("fence_poly", U8, fence_poly, 0, 1),
};
static constexpr int NPARAMS = sizeof(PARAMS) / sizeof(PARAMS[0]);

int paramCount() { return NPARAMS; }
const ParamInfo& paramInfo(int i) { return PARAMS[i]; }

int paramFind(const char* name) {
  for (int i = 0; i < NPARAMS; i++)
    if (!strncmp(PARAMS[i].name, name, 16)) return i;
  return -1;
}

float paramGet(const Config& c, int i) {
  const ParamInfo& p = PARAMS[i];
  const uint8_t* base = reinterpret_cast<const uint8_t*>(&c) + p.offset;
  switch (p.type) {
    case PType::F32: { float v; memcpy(&v, base, 4); return v; }
    case PType::U8: return *base;
    case PType::U16: { uint16_t v; memcpy(&v, base, 2); return v; }
    case PType::U32: { uint32_t v; memcpy(&v, base, 4); return (float)v; }
  }
  return 0;
}

bool paramSet(Config& c, int i, float v) {
  const ParamInfo& p = PARAMS[i];
  if (v != v || v < p.min || v > p.max) return false;  // NaN or out of range
  uint8_t* base = reinterpret_cast<uint8_t*>(&c) + p.offset;
  switch (p.type) {
    case PType::F32: memcpy(base, &v, 4); break;
    case PType::U8: *base = (uint8_t)(v + 0.5f); break;
    case PType::U16: { uint16_t u = (uint16_t)(v + 0.5f); memcpy(base, &u, 2); break; }
    case PType::U32: { uint32_t u = (uint32_t)(v + 0.5f); memcpy(base, &u, 4); break; }
  }
  return true;
}

int configClamp(Config& c) {
  Config def;
  int fixed = 0;
  for (int i = 0; i < NPARAMS; i++) {
    const ParamInfo& p = PARAMS[i];
    float v = paramGet(c, i);
    if (v == v && v >= p.min && v <= p.max) continue;
    if (!fixed) configDefaults(def);
    float d = paramGet(def, i);
    if (!(v == v)) v = d;  // NaN: the default
    else v = v < p.min ? p.min : p.max;
    if (!paramSet(c, i, v)) paramSet(c, i, d);
    fixed++;
  }
  if (fixed) configSeal(c);
  return fixed;
}

}  // namespace fc
