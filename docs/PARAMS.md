# Bảng tham số AllQuad (cấu hình v25)

Tạo tự động bằng `node tools/gen_params_doc.js` từ firmware (giới hạn, mặc định) và trang cấu hình (mô tả). 395 tham số. Đổi bằng trang cấu hình (tab Tham số, hoặc thẻ của từng tab), CLI `set <tên> <giá trị>` rồi `save`, MAVLink (QGroundControl / Mission Planner) hoặc MSP. Cột "KĐL" = cần khởi động lại sau khi lưu.

Nhóm: [PID & rates](#pid-rates) · [Profile 2 & 3](#profile-2-3) · [Bộ lọc](#bộ-lọc) · [Độ cao & vị trí](#độ-cao-vị-trí) · [Điều hướng & geofence](#điều-hướng-geofence) · [Failsafe & pin](#failsafe-pin) · [Tay điều khiển](#tay-điều-khiển) · [Chế độ (công tắc)](#chế-độ-công-tắc) · [Cổng & kết nối](#cổng-kết-nối) · [Motor & hệ thống](#motor-hệ-thống) · [VTX & LED](#vtx-led) · [OSD & blackbox](#osd-blackbox) · [Hiệu chuẩn & cảm biến](#hiệu-chuẩn-cảm-biến) · [Khác](#khác)

## PID & rates

| Tham số | Mặc định | Giới hạn | Mô tả |
|---|---|---|---|
| `yaw_reversed` | 0 | 0 … 1 | Đảo chiều quay cánh (props out) |
| `roll_p` | 45 | 0 … 250 | P (bám theo sai lệch) trục roll |
| `roll_i` | 80 | 0 … 250 | I (triệt sai lệch kéo dài) trục roll |
| `roll_d` | 30 | 0 … 250 | D (giảm chấn) trục roll |
| `roll_f` | 60 | 0 … 1000 | F / feedforward (bám cần nhanh) trục roll |
| `pitch_p` | 47 | 0 … 250 | P (bám theo sai lệch) trục pitch |
| `pitch_i` | 84 | 0 … 250 | I (triệt sai lệch kéo dài) trục pitch |
| `pitch_d` | 34 | 0 … 250 | D (giảm chấn) trục pitch |
| `pitch_f` | 60 | 0 … 1000 | F / feedforward (bám cần nhanh) trục pitch |
| `yaw_p` | 45 | 0 … 250 | P (bám theo sai lệch) trục yaw |
| `yaw_i` | 80 | 0 … 250 | I (triệt sai lệch kéo dài) trục yaw |
| `yaw_d` | 0 | 0 … 250 | D (giảm chấn) trục yaw |
| `yaw_f` | 60 | 0 … 1000 | F / feedforward (bám cần nhanh) trục yaw |
| `iterm_limit` | 400 | 0 … 500 | Giới hạn I |
| `pidsum_limit` | 500 | 100 … 1000 | Giới hạn tổng PID roll / pitch |
| `pidsum_lim_yaw` | 400 | 100 … 1000 | Giới hạn tổng PID yaw |
| `tpa_rate` | 0.4 | 0 … 1 | Giảm P/D ở ga cao (0–1) |
| `tpa_breakpoint` | 0.6 | 0 … 1 | Ga bắt đầu TPA |
| `iterm_relax` | 40 °/s | 0 … 500 | Ngưỡng I-relax (°/s) |
| `anti_gravity` | 3.5 | 0 … 10 | Tăng I khi giật ga |
| `roll_rc_rate` | 70 °/s | 0 … 500 | Rate tâm cần (°/s ở giữa cần, rates Actual), trục roll |
| `roll_max_rate` | 670 °/s | 0 … 2000 | Tốc độ quay tối đa khi gạt hết cần (°/s), trục roll |
| `roll_expo` | 0 | 0 … 1 | Expo (độ cong quanh tâm cần), trục roll |
| `pitch_rc_rate` | 70 °/s | 0 … 500 | Rate tâm cần (°/s ở giữa cần, rates Actual), trục pitch |
| `pitch_max_rate` | 670 °/s | 0 … 2000 | Tốc độ quay tối đa khi gạt hết cần (°/s), trục pitch |
| `pitch_expo` | 0 | 0 … 1 | Expo (độ cong quanh tâm cần), trục pitch |
| `yaw_rc_rate` | 70 °/s | 0 … 500 | Rate tâm cần (°/s ở giữa cần, rates Actual), trục yaw |
| `yaw_max_rate` | 670 °/s | 0 … 2000 | Tốc độ quay tối đa khi gạt hết cần (°/s), trục yaw |
| `yaw_expo` | 0 | 0 … 1 | Expo (độ cong quanh tâm cần), trục yaw |
| `angle_p` | 6 ° | 0 … 20 | Độ cứng tự cân bằng |
| `angle_limit` | 45 ° | 10 … 80 | Góc nghiêng tối đa (ANGLE) |
| `horizon_trans` | 0.75 | 0.1 … 1 | Điểm chuyển HORIZON |
| `at_axes` | 3 | 1 … 7 | Autotune trục: 1 roll, 2 pitch, 4 yaw (cộng lại, 3 = roll + pitch) |
| `at_rate` | 100 °/s | 90 … 400 | Autotune: tốc độ cú giật (°/s) |
| `at_steps` | 15 | 1 … 15 | Autotune bước: 1 P/D, 2 feedforward, 4 I, 8 bộ lọc (cộng lại, 15 = tất cả; chỉ 8 = đo nhiễu khi treo) |
| `launch_angle` | 15 ° | 0 … 80 | Launch control: góc chúi khi chờ phóng (°) |
| `launch_thr` | 20 | 5 … 70 | Launch control: ga (%) để phóng |
| `rc_smooth_hz` | 0 Hz | 0 … 250 | Làm mượt tay điều khiển (Hz), 0 = tự động theo tốc độ cập nhật |
| `crash_detect` | 1 | 0 … 1 | Phát hiện rơi / va chạm và tắt motor |
| `yaw_spin_thr` | 1950 °/s | 0 … 2000 | Phục hồi khi xoay yaw quá nhanh (°/s), 0 = tắt |
| `d_min_pct` | 0 % | 0 … 100 | D khi bay êm (% của D), 0 = D cố định |
| `d_min_gain` | 37 | 0 … 100 | Mức tăng D khi gyro rung (prop wash) |
| `d_min_adv` | 20 | 0 … 200 | Mức tăng D khi gạt cần nhanh |
| `thr_boost` | 0 | 0 … 10 | Throttle boost: tăng phản ứng ga (0-10) |
| `thrust_expo` | 0 | 0 … 1 | Tuyến tính hoá lực đẩy (0 = tắt, thường 0,6) |
| `vbat_sag_comp` | 0 | 0 … 100 | Bù sụt áp pin (%): phản ứng như pin đầy |

## Profile 2 & 3

| Tham số | Mặc định | Giới hạn | Mô tả |
|---|---|---|---|
| `pid_profile` | 0 | 0 … 2 | Profile PID / rates đang dùng khi không gán công tắc (0-2 = 1-3). 0 = Profile 1, 1 = Profile 2, 2 = Profile 3 |
| `prof_ch` | 0 | 0 … 16 | Kênh chọn profile (thấp / giữa / cao = 1 / 2 / 3), 0 = không |
| `p2_roll_p` | 45 | 0 … 250 | P (bám theo sai lệch) trục roll, profile 2 |
| `p2_roll_i` | 80 | 0 … 250 | I (triệt sai lệch kéo dài) trục roll, profile 2 |
| `p2_roll_d` | 30 | 0 … 250 | D (giảm chấn) trục roll, profile 2 |
| `p2_roll_f` | 60 | 0 … 1000 | F / feedforward (bám cần nhanh) trục roll, profile 2 |
| `p2_roll_rc` | 70 | 0 … 500 | Rate tâm cần (°/s ở giữa cần, rates Actual), trục roll, profile 2 |
| `p2_roll_max` | 670 | 0 … 2000 | Tốc độ quay tối đa khi gạt hết cần (°/s), trục roll, profile 2 |
| `p2_roll_expo` | 0 | 0 … 1 | Expo (độ cong quanh tâm cần), trục roll, profile 2 |
| `p2_pitch_p` | 47 | 0 … 250 | P (bám theo sai lệch) trục pitch, profile 2 |
| `p2_pitch_i` | 84 | 0 … 250 | I (triệt sai lệch kéo dài) trục pitch, profile 2 |
| `p2_pitch_d` | 34 | 0 … 250 | D (giảm chấn) trục pitch, profile 2 |
| `p2_pitch_f` | 60 | 0 … 1000 | F / feedforward (bám cần nhanh) trục pitch, profile 2 |
| `p2_pitch_rc` | 70 | 0 … 500 | Rate tâm cần (°/s ở giữa cần, rates Actual), trục pitch, profile 2 |
| `p2_pitch_max` | 670 | 0 … 2000 | Tốc độ quay tối đa khi gạt hết cần (°/s), trục pitch, profile 2 |
| `p2_pitch_expo` | 0 | 0 … 1 | Expo (độ cong quanh tâm cần), trục pitch, profile 2 |
| `p2_yaw_p` | 45 | 0 … 250 | P (bám theo sai lệch) trục yaw, profile 2 |
| `p2_yaw_i` | 80 | 0 … 250 | I (triệt sai lệch kéo dài) trục yaw, profile 2 |
| `p2_yaw_d` | 0 | 0 … 250 | D (giảm chấn) trục yaw, profile 2 |
| `p2_yaw_f` | 60 | 0 … 1000 | F / feedforward (bám cần nhanh) trục yaw, profile 2 |
| `p2_yaw_rc` | 70 | 0 … 500 | Rate tâm cần (°/s ở giữa cần, rates Actual), trục yaw, profile 2 |
| `p2_yaw_max` | 670 | 0 … 2000 | Tốc độ quay tối đa khi gạt hết cần (°/s), trục yaw, profile 2 |
| `p2_yaw_expo` | 0 | 0 … 1 | Expo (độ cong quanh tâm cần), trục yaw, profile 2 |
| `p3_roll_p` | 45 | 0 … 250 | P (bám theo sai lệch) trục roll, profile 3 |
| `p3_roll_i` | 80 | 0 … 250 | I (triệt sai lệch kéo dài) trục roll, profile 3 |
| `p3_roll_d` | 30 | 0 … 250 | D (giảm chấn) trục roll, profile 3 |
| `p3_roll_f` | 60 | 0 … 1000 | F / feedforward (bám cần nhanh) trục roll, profile 3 |
| `p3_roll_rc` | 70 | 0 … 500 | Rate tâm cần (°/s ở giữa cần, rates Actual), trục roll, profile 3 |
| `p3_roll_max` | 670 | 0 … 2000 | Tốc độ quay tối đa khi gạt hết cần (°/s), trục roll, profile 3 |
| `p3_roll_expo` | 0 | 0 … 1 | Expo (độ cong quanh tâm cần), trục roll, profile 3 |
| `p3_pitch_p` | 47 | 0 … 250 | P (bám theo sai lệch) trục pitch, profile 3 |
| `p3_pitch_i` | 84 | 0 … 250 | I (triệt sai lệch kéo dài) trục pitch, profile 3 |
| `p3_pitch_d` | 34 | 0 … 250 | D (giảm chấn) trục pitch, profile 3 |
| `p3_pitch_f` | 60 | 0 … 1000 | F / feedforward (bám cần nhanh) trục pitch, profile 3 |
| `p3_pitch_rc` | 70 | 0 … 500 | Rate tâm cần (°/s ở giữa cần, rates Actual), trục pitch, profile 3 |
| `p3_pitch_max` | 670 | 0 … 2000 | Tốc độ quay tối đa khi gạt hết cần (°/s), trục pitch, profile 3 |
| `p3_pitch_expo` | 0 | 0 … 1 | Expo (độ cong quanh tâm cần), trục pitch, profile 3 |
| `p3_yaw_p` | 45 | 0 … 250 | P (bám theo sai lệch) trục yaw, profile 3 |
| `p3_yaw_i` | 80 | 0 … 250 | I (triệt sai lệch kéo dài) trục yaw, profile 3 |
| `p3_yaw_d` | 0 | 0 … 250 | D (giảm chấn) trục yaw, profile 3 |
| `p3_yaw_f` | 60 | 0 … 1000 | F / feedforward (bám cần nhanh) trục yaw, profile 3 |
| `p3_yaw_rc` | 70 | 0 … 500 | Rate tâm cần (°/s ở giữa cần, rates Actual), trục yaw, profile 3 |
| `p3_yaw_max` | 670 | 0 … 2000 | Tốc độ quay tối đa khi gạt hết cần (°/s), trục yaw, profile 3 |
| `p3_yaw_expo` | 0 | 0 … 1 | Expo (độ cong quanh tâm cần), trục yaw, profile 3 |

## Bộ lọc

| Tham số | Mặc định | Giới hạn | Mô tả |
|---|---|---|---|
| `gyro_lpf_hz` | 150 Hz | 0 … 500 | Lọc thông thấp gyro 1 (Hz) |
| `gyro_lpf2_hz` | 250 Hz | 0 … 1000 | Lọc thông thấp gyro 2 (Hz) |
| `dterm_lpf_hz` | 90 Hz | 0 … 300 | Lọc D-term (Hz) |
| `acc_lpf_hz` | 15 Hz | 1 … 100 | Lọc thông thấp accel (Hz) |
| `notch1_hz` | 0 Hz | 0 … 900 | Notch 1 tâm (Hz, 0 = tắt) |
| `notch1_cut` | 0 Hz | 0 … 900 | Notch 1 cạnh dưới (Hz) |
| `notch2_hz` | 0 Hz | 0 … 900 | Notch 2 tâm (Hz) |
| `notch2_cut` | 0 Hz | 0 … 900 | Notch 2 cạnh dưới (Hz) |
| `dyn_notch_count` | 1 | 0 … 2 | Số notch động (0–2). 0 = Tắt, 1 = 1 notch, 2 = 2 notch |
| `dyn_notch_min` | 100 Hz | 50 … 300 | Notch động thấp nhất (Hz) |
| `dyn_notch_max` | 450 Hz | 150 … 900 | Notch động cao nhất (Hz) |
| `dyn_notch_q` | 3.5 | 1 … 10 | Độ hẹp notch động (Q) |
| `rpm_harmonics` | 3 | 0 … 3 | Số hoạ âm RPM filter |
| `rpm_min_hz` | 100 Hz | 50 … 300 | RPM filter tần số thấp nhất |
| `rpm_q` | 5 | 1 … 10 | Độ hẹp RPM filter (Q) |
| `rpm_w1` | 100 | 0 … 100 | RPM filter: mức notch hài bậc 1 (%) |
| `rpm_w2` | 100 | 0 … 100 | RPM filter: mức notch hài bậc 2 (%) |
| `rpm_w3` | 100 | 0 … 100 | RPM filter: mức notch hài bậc 3 (%, thấp = ít trễ hơn) |
| `dyn_lpf_min` | 0 | 0 … 1000 | Lọc gyro động: tần số cắt ở ga 0 (Hz), 0 = tắt |
| `dyn_lpf_max` | 0 | 0 … 1000 | Lọc gyro động: tần số cắt ở ga tối đa (Hz) |
| `dyn_idle_rpm` | 0 | 0 … 100 | Dynamic idle: RPM tối thiểu ÷ 100 (cần RPM từ ESC), 0 = tắt |

## Độ cao & vị trí

| Tham số | Mặc định | Giới hạn | Mô tả |
|---|---|---|---|
| `alt_pos_p` | 1.2 | 0 … 5 | P độ cao → tốc độ leo |
| `alt_vel_p` | 0.12 | 0 … 1 | P tốc độ leo |
| `alt_vel_i` | 0.08 | 0 … 1 | I tốc độ leo |
| `alt_vel_d` | 0.01 | 0 … 0.5 | D tốc độ leo |
| `climb_rate_max` | 2 m/s | 0.2 … 10 | Tốc độ leo/hạ tối đa (m/s) |
| `hover_throttle` | 0.4 | 0.1 … 0.8 | Ga treo (tự học nếu bật) |
| `hover_learn` | 2 | 0 … 2 | Tự học ga treo: 0 tắt, 1 học, 2 học và lưu khi disarm |
| `alt_surface` | 1 | 0 … 1 | Bám mặt đất bằng cảm biến khoảng cách (ALTHOLD/POSHOLD/CRUISE) |
| `pos_p` | 0.8 | 0 … 5 | P vị trí → tốc độ |
| `pos_vel_p` | 1.6 | 0 … 10 | P tốc độ ngang |
| `pos_vel_i` | 0.3 | 0 … 5 | I tốc độ ngang (chống trôi) |
| `pos_max_speed` | 6 m/s | 0.5 … 25 | Tốc độ ngang tối đa (m/s) |
| `pos_max_angle` | 25 ° | 5 … 45 | Góc nghiêng tối đa khi tự bay (°) |
| `pos_max_accel` | 4 m/s² | 0.5 … 10 | Gia tốc ngang tối đa (m/s²) |
| `gps_min_sats` | 7 | 4 … 20 | Số vệ tinh tối thiểu |

## Điều hướng & geofence

| Tham số | Mặc định | Giới hạn | Mô tả |
|---|---|---|---|
| `rth_alt` | 20 m | 2 … 200 | Độ cao bay về (m) |
| `rth_speed` | 6 m/s | 1 … 20 | Tốc độ bay về (m/s) |
| `rth_loiter_s` | 3 s | 0 … 120 | Chờ trên điểm về trước khi hạ (s) |
| `rth_home_radius` | 2 m | 0.5 … 20 | Bán kính coi là đã về (m) |
| `rth_climb_first` | 1 | 0 … 1 | Leo đủ độ cao rồi mới bay về |
| `rth_land` | 1 | 0 … 1 | Tự hạ cánh sau khi về |
| `land_speed` | 1 m/s | 0.2 … 3 | Tốc độ hạ (m/s) |
| `land_slow_speed` | 0.4 m/s | 0.1 … 1 | Tốc độ hạ gần đất (m/s) |
| `land_slow_alt` | 3 m | 0.5 … 20 | Độ cao bắt đầu hạ chậm (m) |
| `wp_radius` | 2 m | 0.5 … 20 | Bán kính đến waypoint (m) |
| `wp_speed` | 5 m/s | 0.5 … 20 | Tốc độ nhiệm vụ (m/s) |
| `nav_yaw_mode` | 1 | 0 … 1 | Hướng mũi: 0 giữ nguyên, 1 theo hướng bay |
| `nav_yaw_p` | 2 | 0 … 10 | P giữ hướng khi tự bay |
| `fence_enable` | 0 | 0 … 1 | Bật geofence |
| `fence_action` | 2 | 1 … 2 | Vượt rào: 1 hạ cánh, 2 bay về. 1 = Hạ cánh, 2 = Bay về |
| `fence_radius` | 150 m | 10 … 5000 | Bán kính rào (m) |
| `fence_alt_max` | 100 m | 5 … 1000 | Độ cao tối đa (m) |
| `rth_sanity_s` | 20 s | 0 … 120 | GPS Rescue: bay về không lại gần nhà trong chừng này giây thì hạ cánh tại chỗ (0 = tắt) |
| `srtl_point_m` | 3 m | 0 … 20 | SmartRTL: khoảng cách giữa các điểm ghi đường bay (m), 0 = tắt |
| `fence_poly` | 1 | 0 … 1 | Dùng geofence đa giác tải lên từ GCS |

## Failsafe & pin

| Tham số | Mặc định | Giới hạn | Mô tả |
|---|---|---|---|
| `arm_throttle_max` | 1050 µs | 1000 … 1300 | Ga tối đa để được arm (µs) |
| `fs_delay_ms` | 300 ms | 100 … 2000 | Mất sóng bao lâu thì vào failsafe (ms) |
| `fs_off_delay_ms` | 30000 ms | 1000 … 60000 | Failsafe kéo dài bao lâu thì tắt motor (ms) |
| `fs_action` | 2 | 0 … 3 | Mất sóng tay điều khiển thì làm gì. 0 = Tắt motor, 1 = Hạ cánh, 2 = Bay về (RTH), 3 = SmartRTL |
| `fs_throttle` | 0.35 | 0 … 0.8 | Ga khi hạ cánh failsafe không có baro |
| `vbat_scale` | 11 | 1 … 50 | Hệ số chia áp đo pin |
| `cur_scale` | 0 | 0 … 500 | Hệ số cảm biến dòng (A / V) |
| `cur_offset` | 0 | -3.3 … 3.3 | Độ lệch cảm biến dòng (V) |
| `cell_count` | 0 | 0 … 12 | Số cell pin, 0 = tự nhận |
| `vbat_warn_cell` | 3.5 V | 2.8 … 4.2 | Cảnh báo pin yếu (V / cell) |
| `vbat_crit_cell` | 3.3 V | 2.8 … 4.2 | Pin cạn (V / cell) |
| `bat_capacity` | 0 mAh | 0 … 30000 | Dung lượng pin (mAh), 0 = không theo dõi |
| `bat_fs_action` | 1 | 0 … 3 | Pin cạn thì làm gì. 0 = Chỉ cảnh báo, 1 = Hạ cánh, 2 = Bay về (RTH), 3 = SmartRTL |
| `bat_reserve_pct` | 20 % | 0 … 50 | Giữ lại % pin để bay về (RTH thông minh) |
| `arm_require_gps` | 0 | 0 … 1 | Chỉ arm khi có GPS fix (để RTH chắc chắn có điểm về) |
| `gcs_arm` | 0 | 0 … 1 | Cho phép arm từ trạm mặt đất (MAVLink) |
| `arm_max_angle` | 25 ° | 5 … 180 | Góc nghiêng tối đa để được arm (°) |
| `bat_smart_rth` | 2 | 0 … 2 | Pin đủ để bay về (tính theo khoảng cách, độ cao, gió): 0 tắt, 1 chỉ cảnh báo, 2 cảnh báo và tự bay về |
| `gcs_fs` | 0 | 0 … 3 | Mất kết nối trạm mặt đất khi GUIDED / MISSION: 0 tắt, 1 RTH, 2 hạ cánh, 3 SmartRTL |
| `gcs_fs_s` | 5 s | 1 … 60 | Số giây mất heartbeat GCS |

## Tay điều khiển

| Tham số | Mặc định | Giới hạn | Mô tả |
|---|---|---|---|
| `rx_protocol` (KĐL) | 0 | 0 … 2 | Giao thức receiver. 0 = CRSF / ELRS, 1 = SBUS, 2 = iBUS |
| `rc_ch_roll` | 1 | 1 … 4 | Kênh receiver mang roll |
| `rc_ch_pitch` | 2 | 1 … 4 | Kênh receiver mang pitch |
| `rc_ch_thr` | 3 | 1 … 4 | Kênh receiver mang ga |
| `rc_ch_yaw` | 4 | 1 … 4 | Kênh receiver mang yaw |
| `rc_deadband` | 5 µs | 0 … 50 | Vùng chết quanh tâm cần roll / pitch / yaw (µs) |

## Chế độ (công tắc)

| Tham số | Mặc định | Giới hạn | Mô tả |
|---|---|---|---|
| `mr1_mode` | 1 | 0 … 20 | Dòng chế độ 1: chế độ được bật (mã như tab Chế độ: 1 ARM, 2 ANGLE, 3 HORIZON, 4 ALTHOLD, 5 POSHOLD, 6 RTH, 7 MISSION, 8 LAND, 9 TURTLE, 10 BEEPER, 11 CRUISE, 12 AUTOTUNE, 13 GPS RESCUE, 14 LAUNCH, 15 SMART RTL, 16 HEADFREE, 17 BRAKE, 18 CIRCLE, 19 FLIP, 20 FOLLOW; 0 = không dùng) |
| `mr1_ch` | 4 | 4 … 15 | Dòng chế độ 1: kênh RC (0-based, 4 = AUX1 / CH5) |
| `mr1_lo` | 1700 µs | 900 … 2100 | Dòng chế độ 1: đầu dưới của khoảng (µs) |
| `mr1_hi` | 2100 µs | 900 … 2100 | Dòng chế độ 1: đầu trên của khoảng (µs) |
| `mr2_mode` | 2 | 0 … 20 | Dòng chế độ 2: chế độ được bật (mã như tab Chế độ: 1 ARM, 2 ANGLE, 3 HORIZON, 4 ALTHOLD, 5 POSHOLD, 6 RTH, 7 MISSION, 8 LAND, 9 TURTLE, 10 BEEPER, 11 CRUISE, 12 AUTOTUNE, 13 GPS RESCUE, 14 LAUNCH, 15 SMART RTL, 16 HEADFREE, 17 BRAKE, 18 CIRCLE, 19 FLIP, 20 FOLLOW; 0 = không dùng) |
| `mr2_ch` | 5 | 4 … 15 | Dòng chế độ 2: kênh RC (0-based, 4 = AUX1 / CH5) |
| `mr2_lo` | 1300 µs | 900 … 2100 | Dòng chế độ 2: đầu dưới của khoảng (µs) |
| `mr2_hi` | 1700 µs | 900 … 2100 | Dòng chế độ 2: đầu trên của khoảng (µs) |
| `mr3_mode` | 4 | 0 … 20 | Dòng chế độ 3: chế độ được bật (mã như tab Chế độ: 1 ARM, 2 ANGLE, 3 HORIZON, 4 ALTHOLD, 5 POSHOLD, 6 RTH, 7 MISSION, 8 LAND, 9 TURTLE, 10 BEEPER, 11 CRUISE, 12 AUTOTUNE, 13 GPS RESCUE, 14 LAUNCH, 15 SMART RTL, 16 HEADFREE, 17 BRAKE, 18 CIRCLE, 19 FLIP, 20 FOLLOW; 0 = không dùng) |
| `mr3_ch` | 5 | 4 … 15 | Dòng chế độ 3: kênh RC (0-based, 4 = AUX1 / CH5) |
| `mr3_lo` | 1700 µs | 900 … 2100 | Dòng chế độ 3: đầu dưới của khoảng (µs) |
| `mr3_hi` | 2100 µs | 900 … 2100 | Dòng chế độ 3: đầu trên của khoảng (µs) |
| `mr4_mode` | 5 | 0 … 20 | Dòng chế độ 4: chế độ được bật (mã như tab Chế độ: 1 ARM, 2 ANGLE, 3 HORIZON, 4 ALTHOLD, 5 POSHOLD, 6 RTH, 7 MISSION, 8 LAND, 9 TURTLE, 10 BEEPER, 11 CRUISE, 12 AUTOTUNE, 13 GPS RESCUE, 14 LAUNCH, 15 SMART RTL, 16 HEADFREE, 17 BRAKE, 18 CIRCLE, 19 FLIP, 20 FOLLOW; 0 = không dùng) |
| `mr4_ch` | 6 | 4 … 15 | Dòng chế độ 4: kênh RC (0-based, 4 = AUX1 / CH5) |
| `mr4_lo` | 1300 µs | 900 … 2100 | Dòng chế độ 4: đầu dưới của khoảng (µs) |
| `mr4_hi` | 1700 µs | 900 … 2100 | Dòng chế độ 4: đầu trên của khoảng (µs) |
| `mr5_mode` | 6 | 0 … 20 | Dòng chế độ 5: chế độ được bật (mã như tab Chế độ: 1 ARM, 2 ANGLE, 3 HORIZON, 4 ALTHOLD, 5 POSHOLD, 6 RTH, 7 MISSION, 8 LAND, 9 TURTLE, 10 BEEPER, 11 CRUISE, 12 AUTOTUNE, 13 GPS RESCUE, 14 LAUNCH, 15 SMART RTL, 16 HEADFREE, 17 BRAKE, 18 CIRCLE, 19 FLIP, 20 FOLLOW; 0 = không dùng) |
| `mr5_ch` | 6 | 4 … 15 | Dòng chế độ 5: kênh RC (0-based, 4 = AUX1 / CH5) |
| `mr5_lo` | 1700 µs | 900 … 2100 | Dòng chế độ 5: đầu dưới của khoảng (µs) |
| `mr5_hi` | 2100 µs | 900 … 2100 | Dòng chế độ 5: đầu trên của khoảng (µs) |
| `mr6_mode` | 7 | 0 … 20 | Dòng chế độ 6: chế độ được bật (mã như tab Chế độ: 1 ARM, 2 ANGLE, 3 HORIZON, 4 ALTHOLD, 5 POSHOLD, 6 RTH, 7 MISSION, 8 LAND, 9 TURTLE, 10 BEEPER, 11 CRUISE, 12 AUTOTUNE, 13 GPS RESCUE, 14 LAUNCH, 15 SMART RTL, 16 HEADFREE, 17 BRAKE, 18 CIRCLE, 19 FLIP, 20 FOLLOW; 0 = không dùng) |
| `mr6_ch` | 7 | 4 … 15 | Dòng chế độ 6: kênh RC (0-based, 4 = AUX1 / CH5) |
| `mr6_lo` | 1300 µs | 900 … 2100 | Dòng chế độ 6: đầu dưới của khoảng (µs) |
| `mr6_hi` | 1700 µs | 900 … 2100 | Dòng chế độ 6: đầu trên của khoảng (µs) |
| `mr7_mode` | 8 | 0 … 20 | Dòng chế độ 7: chế độ được bật (mã như tab Chế độ: 1 ARM, 2 ANGLE, 3 HORIZON, 4 ALTHOLD, 5 POSHOLD, 6 RTH, 7 MISSION, 8 LAND, 9 TURTLE, 10 BEEPER, 11 CRUISE, 12 AUTOTUNE, 13 GPS RESCUE, 14 LAUNCH, 15 SMART RTL, 16 HEADFREE, 17 BRAKE, 18 CIRCLE, 19 FLIP, 20 FOLLOW; 0 = không dùng) |
| `mr7_ch` | 7 | 4 … 15 | Dòng chế độ 7: kênh RC (0-based, 4 = AUX1 / CH5) |
| `mr7_lo` | 1700 µs | 900 … 2100 | Dòng chế độ 7: đầu dưới của khoảng (µs) |
| `mr7_hi` | 2100 µs | 900 … 2100 | Dòng chế độ 7: đầu trên của khoảng (µs) |
| `mr8_mode` | 10 | 0 … 20 | Dòng chế độ 8: chế độ được bật (mã như tab Chế độ: 1 ARM, 2 ANGLE, 3 HORIZON, 4 ALTHOLD, 5 POSHOLD, 6 RTH, 7 MISSION, 8 LAND, 9 TURTLE, 10 BEEPER, 11 CRUISE, 12 AUTOTUNE, 13 GPS RESCUE, 14 LAUNCH, 15 SMART RTL, 16 HEADFREE, 17 BRAKE, 18 CIRCLE, 19 FLIP, 20 FOLLOW; 0 = không dùng) |
| `mr8_ch` | 8 | 4 … 15 | Dòng chế độ 8: kênh RC (0-based, 4 = AUX1 / CH5) |
| `mr8_lo` | 1700 µs | 900 … 2100 | Dòng chế độ 8: đầu dưới của khoảng (µs) |
| `mr8_hi` | 2100 µs | 900 … 2100 | Dòng chế độ 8: đầu trên của khoảng (µs) |
| `mr9_mode` | 9 | 0 … 20 | Dòng chế độ 9: chế độ được bật (mã như tab Chế độ: 1 ARM, 2 ANGLE, 3 HORIZON, 4 ALTHOLD, 5 POSHOLD, 6 RTH, 7 MISSION, 8 LAND, 9 TURTLE, 10 BEEPER, 11 CRUISE, 12 AUTOTUNE, 13 GPS RESCUE, 14 LAUNCH, 15 SMART RTL, 16 HEADFREE, 17 BRAKE, 18 CIRCLE, 19 FLIP, 20 FOLLOW; 0 = không dùng) |
| `mr9_ch` | 9 | 4 … 15 | Dòng chế độ 9: kênh RC (0-based, 4 = AUX1 / CH5) |
| `mr9_lo` | 1700 µs | 900 … 2100 | Dòng chế độ 9: đầu dưới của khoảng (µs) |
| `mr9_hi` | 2100 µs | 900 … 2100 | Dòng chế độ 9: đầu trên của khoảng (µs) |
| `mr10_mode` | 0 | 0 … 20 | Dòng chế độ 10: chế độ được bật (mã như tab Chế độ: 1 ARM, 2 ANGLE, 3 HORIZON, 4 ALTHOLD, 5 POSHOLD, 6 RTH, 7 MISSION, 8 LAND, 9 TURTLE, 10 BEEPER, 11 CRUISE, 12 AUTOTUNE, 13 GPS RESCUE, 14 LAUNCH, 15 SMART RTL, 16 HEADFREE, 17 BRAKE, 18 CIRCLE, 19 FLIP, 20 FOLLOW; 0 = không dùng) |
| `mr10_ch` | 4 | 4 … 15 | Dòng chế độ 10: kênh RC (0-based, 4 = AUX1 / CH5) |
| `mr10_lo` | 1700 µs | 900 … 2100 | Dòng chế độ 10: đầu dưới của khoảng (µs) |
| `mr10_hi` | 2100 µs | 900 … 2100 | Dòng chế độ 10: đầu trên của khoảng (µs) |
| `mr11_mode` | 0 | 0 … 20 | Dòng chế độ 11: chế độ được bật (mã như tab Chế độ: 1 ARM, 2 ANGLE, 3 HORIZON, 4 ALTHOLD, 5 POSHOLD, 6 RTH, 7 MISSION, 8 LAND, 9 TURTLE, 10 BEEPER, 11 CRUISE, 12 AUTOTUNE, 13 GPS RESCUE, 14 LAUNCH, 15 SMART RTL, 16 HEADFREE, 17 BRAKE, 18 CIRCLE, 19 FLIP, 20 FOLLOW; 0 = không dùng) |
| `mr11_ch` | 4 | 4 … 15 | Dòng chế độ 11: kênh RC (0-based, 4 = AUX1 / CH5) |
| `mr11_lo` | 1700 µs | 900 … 2100 | Dòng chế độ 11: đầu dưới của khoảng (µs) |
| `mr11_hi` | 2100 µs | 900 … 2100 | Dòng chế độ 11: đầu trên của khoảng (µs) |
| `mr12_mode` | 0 | 0 … 20 | Dòng chế độ 12: chế độ được bật (mã như tab Chế độ: 1 ARM, 2 ANGLE, 3 HORIZON, 4 ALTHOLD, 5 POSHOLD, 6 RTH, 7 MISSION, 8 LAND, 9 TURTLE, 10 BEEPER, 11 CRUISE, 12 AUTOTUNE, 13 GPS RESCUE, 14 LAUNCH, 15 SMART RTL, 16 HEADFREE, 17 BRAKE, 18 CIRCLE, 19 FLIP, 20 FOLLOW; 0 = không dùng) |
| `mr12_ch` | 4 | 4 … 15 | Dòng chế độ 12: kênh RC (0-based, 4 = AUX1 / CH5) |
| `mr12_lo` | 1700 µs | 900 … 2100 | Dòng chế độ 12: đầu dưới của khoảng (µs) |
| `mr12_hi` | 2100 µs | 900 … 2100 | Dòng chế độ 12: đầu trên của khoảng (µs) |

## Cổng & kết nối

| Tham số | Mặc định | Giới hạn | Mô tả |
|---|---|---|---|
| `serial1_fn` (KĐL) | 1 | 0 … 12 | Chức năng của SERIAL1: 0 tắt, 1 receiver, 2 GPS, 3 MAVLink, 4 MSP, 5 DisplayPort (OSD kính số), 6 telemetry ESC, 7 TFmini, 8 blackbox, 9 VTX SmartAudio, 10 VTX Tramp |
| `serial2_fn` (KĐL) | 2 | 0 … 12 | Chức năng của SERIAL2: 0 tắt, 1 receiver, 2 GPS, 3 MAVLink, 4 MSP, 5 DisplayPort (OSD kính số), 6 telemetry ESC, 7 TFmini, 8 blackbox, 9 VTX SmartAudio, 10 VTX Tramp |
| `serial3_fn` (KĐL) | 3 | 0 … 12 | Chức năng của SERIAL3: 0 tắt, 1 receiver, 2 GPS, 3 MAVLink, 4 MSP, 5 DisplayPort (OSD kính số), 6 telemetry ESC, 7 TFmini, 8 blackbox, 9 VTX SmartAudio, 10 VTX Tramp |
| `udp_fn` (KĐL) | 3 | 0 … 8 | Chức năng của cổng UDP (WiFi): 0 tắt, 1 receiver, 2 GPS, 3 MAVLink, 4 MSP, 5 DisplayPort (OSD kính số), 6 telemetry ESC, 7 TFmini, 8 blackbox, 9 VTX SmartAudio, 10 VTX Tramp |
| `serial1_baud` (KĐL) | 0 | 0 … 2000000 | Tốc độ baud của SERIAL1 |
| `serial2_baud` (KĐL) | 0 | 0 … 2000000 | Tốc độ baud của SERIAL2 |
| `serial3_baud` (KĐL) | 0 | 0 … 2000000 | Tốc độ baud của SERIAL3 |
| `serial4_fn` (KĐL) | 0 | 0 … 12 | Chức năng của SERIAL4: 0 tắt, 1 receiver, 2 GPS, 3 MAVLink, 4 MSP, 5 DisplayPort (OSD kính số), 6 telemetry ESC, 7 TFmini, 8 blackbox, 9 VTX SmartAudio, 10 VTX Tramp |
| `serial5_fn` (KĐL) | 0 | 0 … 12 | Chức năng của SERIAL5: 0 tắt, 1 receiver, 2 GPS, 3 MAVLink, 4 MSP, 5 DisplayPort (OSD kính số), 6 telemetry ESC, 7 TFmini, 8 blackbox, 9 VTX SmartAudio, 10 VTX Tramp |
| `serial4_baud` (KĐL) | 0 | 0 … 2000000 | Tốc độ baud của SERIAL4 |
| `serial5_baud` (KĐL) | 0 | 0 … 2000000 | Tốc độ baud của SERIAL5 |
| `serial6_fn` (KĐL) | 0 | 0 … 12 | Chức năng của SERIAL6: 0 tắt, 1 receiver, 2 GPS, 3 MAVLink, 4 MSP, 5 DisplayPort (OSD kính số), 6 telemetry ESC, 7 TFmini, 8 blackbox, 9 VTX SmartAudio, 10 VTX Tramp |
| `serial7_fn` (KĐL) | 0 | 0 … 12 | Chức năng của SERIAL7: 0 tắt, 1 receiver, 2 GPS, 3 MAVLink, 4 MSP, 5 DisplayPort (OSD kính số), 6 telemetry ESC, 7 TFmini, 8 blackbox, 9 VTX SmartAudio, 10 VTX Tramp |
| `serial6_baud` (KĐL) | 0 | 0 … 2000000 | Tốc độ baud của SERIAL6 |
| `serial7_baud` (KĐL) | 0 | 0 … 2000000 | Tốc độ baud của SERIAL7 |
| `wifi_enable` (KĐL) | 0 | 0 … 1 | WiFi + MAVLink qua UDP 14550 (QGroundControl không dây), kèm trang 192.168.4.1 |
| `web_enable` (KĐL) | 0 | 0 … 1 | WiFi + trang cấu hình 192.168.4.1 |
| `wifi_armed` | 0 | 0 … 1 | Giữ WiFi khi đang bay (tắt = bớt nhiễu, tiết kiệm CPU) |
| `mav_sysid` | 1 | 1 … 250 | MAVLink system ID |

## Motor & hệ thống

| Tham số | Mặc định | Giới hạn | Mô tả |
|---|---|---|---|
| `loop_hz` (KĐL) | 1000 Hz | 500 … 2000 | Tần số vòng PID |
| `motor_protocol` (KĐL) | 2 | 0 … 3 | Giao thức ESC. 0 = PWM, 1 = OneShot125, 2 = DShot300, 3 = DShot600 |
| `motor1_out` | 1 | 1 … 4 | Đầu ra điều khiển motor 1 (sau phải) |
| `motor2_out` | 2 | 1 … 4 | Đầu ra điều khiển motor 2 (trước phải) |
| `motor3_out` | 3 | 1 … 4 | Đầu ra điều khiển motor 3 (sau trái) |
| `motor4_out` | 4 | 1 … 4 | Đầu ra điều khiển motor 4 (trước trái) |
| `imu_sync` (KĐL) | 1 | 0 … 1 | Vòng bay chạy theo chân ngắt INT1 của IMU (mẫu gyro mới nhất, 6,4 kHz + lọc trong chip) |
| `dshot_edt` | 1 | 0 … 1 | Xin ESC (AM32 / Bluejay / BLHeli_32) gửi nhiệt độ, điện áp, dòng qua DShot hai chiều |
| `airmode` | 1 | 0 … 1 | Airmode: giữ điều khiển khi ga 0 |
| `motor_poles` | 14 | 2 … 40 | Số cực từ của motor (thường 14) |
| `esc_telemetry` (KĐL) | 0 | 0 … 1 | Đọc telemetry ESC (cần cho RPM filter) |
| `motor_idle` | 0.055 | 0 … 0.2 | Ga không tải khi arm (0,055 = 5,5 %) |
| `motor_limit_pct` | 100 % | 50 … 100 | Giới hạn công suất motor (% dải ra motor; cho motor / pin mạnh hơn khung cần) |
| `motor_warn` | 1 | 0 … 1 | Cảnh báo motor yếu (một motor phải chạy mạnh hơn hẳn) và mất lực đẩy |
| `dshot_bidir` (KĐL) | 0 | 0 … 1 | Bidirectional DShot: đọc RPM qua dây tín hiệu (RPM filter không cần dây telemetry; ESC cần Bluejay/BLHeli_32/AM32) |

## VTX & LED

| Tham số | Mặc định | Giới hạn | Mô tả |
|---|---|---|---|
| `vtx_band` | 5 | 1 … 7 | VTX: băng tần (1 A, 2 B, 3 E, 4 F, 5 R, 6 L, 7 U) |
| `vtx_channel` | 1 | 1 … 8 | VTX: kênh 1–8 |
| `vtx_freq` | 0 | 0 … 6000 | VTX: tần số tự chọn (MHz, 0 = theo băng/kênh) |
| `vtx_power` | 1 | 1 … 8 | VTX: mức công suất (theo bảng VTX) |
| `vtx_pit` | 0 | 0 … 1 | VTX: pit mode (phát rất yếu khi ở bàn) |
| `vtx_low_disarm` | 1 | 0 … 1 | VTX: công suất thấp nhất khi disarm |
| `led_pin` (KĐL) | 0 | 0 … 48 | Dải LED WS2812: chân GPIO (ESP32; STM32 cố định PB8), 0 = tắt |
| `led_count` | 8 | 1 … 32 | Số LED |
| `led_brightness` | 40 | 1 … 100 | Độ sáng LED (%) |
| `led_mode` | 0 | 0 … 2 | Kiểu LED: 0 màu theo trạng thái, 1 hướng (trước trắng, sau đỏ), 2 màu cố định |
| `led_color` | 16639 | 0 … 16777215 | Màu LED cố định |
| `vtxt_powers` | 0 | 0 … 8 | Bảng VTX: số mức công suất tự đặt (0 = mặc định 25/100/200/400/600 mW) |
| `vtxt_sa_index` | 0 | 0 … 1 | SmartAudio gửi số thứ tự mức thay vì dBm (VTX firmware cũ) |
| `vtxt_mw1` | 25 | 0 … 10000 | Bảng VTX: công suất mức 1 (mW, 0 = không dùng) |
| `vtxt_val1` | 14 | 0 … 255 | Bảng VTX: giá trị gửi cho VTX ở mức 1 |
| `vtxt_band1` | 5645 | 0 … 6000 | Băng U tự chọn: tần số kênh 1 (MHz) |
| `vtxt_mw2` | 100 | 0 … 10000 | Bảng VTX: công suất mức 2 (mW, 0 = không dùng) |
| `vtxt_val2` | 20 | 0 … 255 | Bảng VTX: giá trị gửi cho VTX ở mức 2 |
| `vtxt_band2` | 5665 | 0 … 6000 | Băng U tự chọn: tần số kênh 2 (MHz) |
| `vtxt_mw3` | 200 | 0 … 10000 | Bảng VTX: công suất mức 3 (mW, 0 = không dùng) |
| `vtxt_val3` | 23 | 0 … 255 | Bảng VTX: giá trị gửi cho VTX ở mức 3 |
| `vtxt_band3` | 5685 | 0 … 6000 | Băng U tự chọn: tần số kênh 3 (MHz) |
| `vtxt_mw4` | 400 | 0 … 10000 | Bảng VTX: công suất mức 4 (mW, 0 = không dùng) |
| `vtxt_val4` | 26 | 0 … 255 | Bảng VTX: giá trị gửi cho VTX ở mức 4 |
| `vtxt_band4` | 5705 | 0 … 6000 | Băng U tự chọn: tần số kênh 4 (MHz) |
| `vtxt_mw5` | 600 | 0 … 10000 | Bảng VTX: công suất mức 5 (mW, 0 = không dùng) |
| `vtxt_val5` | 28 | 0 … 255 | Bảng VTX: giá trị gửi cho VTX ở mức 5 |
| `vtxt_band5` | 5885 | 0 … 6000 | Băng U tự chọn: tần số kênh 5 (MHz) |
| `vtxt_mw6` | 800 | 0 … 10000 | Bảng VTX: công suất mức 6 (mW, 0 = không dùng) |
| `vtxt_val6` | 29 | 0 … 255 | Bảng VTX: giá trị gửi cho VTX ở mức 6 |
| `vtxt_band6` | 5905 | 0 … 6000 | Băng U tự chọn: tần số kênh 6 (MHz) |
| `vtxt_mw7` | 1000 | 0 … 10000 | Bảng VTX: công suất mức 7 (mW, 0 = không dùng) |
| `vtxt_val7` | 30 | 0 … 255 | Bảng VTX: giá trị gửi cho VTX ở mức 7 |
| `vtxt_band7` | 5925 | 0 … 6000 | Băng U tự chọn: tần số kênh 7 (MHz) |
| `vtxt_mw8` | 1600 | 0 … 10000 | Bảng VTX: công suất mức 8 (mW, 0 = không dùng) |
| `vtxt_val8` | 32 | 0 … 255 | Bảng VTX: giá trị gửi cho VTX ở mức 8 |
| `vtxt_band8` | 5945 | 0 … 6000 | Băng U tự chọn: tần số kênh 8 (MHz) |

## OSD & blackbox

| Tham số | Mặc định | Giới hạn | Mô tả |
|---|---|---|---|
| `osd_backend` (KĐL) | 0 | 0 … 2 | Loại OSD. 0 = Tắt, 1 = MAX7456 (analog), 2 = MSP DisplayPort (digital) |
| `osd_pal` | 1 | 0 … 1 | Camera chuẩn PAL (tắt = NTSC) |
| `osd_canvas` (KĐL) | 0 | 0 … 4 | Lưới chữ OSD (canvas). 0 = Tự động, 1 = SD 30×16, 2 = HD 50×18 (HDZero), 3 = HD 53×20 (DJI, Walksnail), 4 = HD 60×22 |
| `osd_font` | 0 | 0 … 1 | Ký hiệu OSD. 0 = Icon Betaflight, 1 = Chỉ chữ |
| `osd_stats` | 15 | 0 … 60 | Màn hình thống kê sau khi disarm (giây, 0 = tắt) |
| `osd_esct_pos` | 88 | 0 … 65535 | Vị trí OSD: nhiệt độ ESC |
| `osd_dist_pos` | 97 | 0 … 65535 | Vị trí OSD: tổng quãng đường |
| `osd_eff_pos` | 439 | 0 … 65535 | Vị trí OSD: mAh/km |
| `osd_tleft_pos` | 417 | 0 … 65535 | Vị trí OSD: thời gian bay còn lại |
| `osd_batbar_pos` | 385 | 0 … 65535 | Vị trí OSD: thanh pin |
| `osd_cell_pos` | 161 | 0 … 65535 | Vị trí OSD: điện áp mỗi cell |
| `osd_rssi_pos` | 119 | 0 … 65535 | Vị trí OSD: RSSI (dBm) |
| `osd_blink` | 1 | 0 … 1 | OSD: cảnh báo và pin yếu nhấp nháy |
| `osd_vbat_pos` | 33217 | 0 … 65535 | Vị trí phần tử OSD "vbat" (cột + 32 × hàng trên lưới 30×16; bit 15 = hiện). Kéo thả trong tab OSD |
| `osd_cur_pos` | 33225 | 0 … 65535 | Vị trí phần tử OSD "cur" (cột + 32 × hàng trên lưới 30×16; bit 15 = hiện). Kéo thả trong tab OSD |
| `osd_mah_pos` | 33232 | 0 … 65535 | Vị trí phần tử OSD "mah" (cột + 32 × hàng trên lưới 30×16; bit 15 = hiện). Kéo thả trong tab OSD |
| `osd_mode_pos` | 33196 | 0 … 65535 | Vị trí phần tử OSD "mode" (cột + 32 × hàng trên lưới 30×16; bit 15 = hiện). Kéo thả trong tab OSD |
| `osd_time_pos` | 33239 | 0 … 65535 | Vị trí phần tử OSD "time" (cột + 32 × hàng trên lưới 30×16; bit 15 = hiện). Kéo thả trong tab OSD |
| `osd_alt_pos` | 33015 | 0 … 65535 | Vị trí phần tử OSD "alt" (cột + 32 × hàng trên lưới 30×16; bit 15 = hiện). Kéo thả trong tab OSD |
| `osd_vario_pos` | 33047 | 0 … 65535 | Vị trí phần tử OSD "vario" (cột + 32 × hàng trên lưới 30×16; bit 15 = hiện). Kéo thả trong tab OSD |
| `osd_hdist_pos` | 32801 | 0 … 65535 | Vị trí phần tử OSD "hdist" (cột + 32 × hàng trên lưới 30×16; bit 15 = hiện). Kéo thả trong tab OSD |
| `osd_hdir_pos` | 32814 | 0 … 65535 | Vị trí phần tử OSD "hdir" (cột + 32 × hàng trên lưới 30×16; bit 15 = hiện). Kéo thả trong tab OSD |
| `osd_sats_pos` | 32824 | 0 … 65535 | Vị trí phần tử OSD "sats" (cột + 32 × hàng trên lưới 30×16; bit 15 = hiện). Kéo thả trong tab OSD |
| `osd_speed_pos` | 32993 m/s | 0 … 65535 | Vị trí phần tử OSD "speed" (cột + 32 × hàng trên lưới 30×16; bit 15 = hiện). Kéo thả trong tab OSD |
| `osd_link_pos` | 32833 | 0 … 65535 | Vị trí phần tử OSD "link" (cột + 32 × hàng trên lưới 30×16; bit 15 = hiện). Kéo thả trong tab OSD |
| `osd_cross_pos` | 33006 | 0 … 65535 | Vị trí phần tử OSD "cross" (cột + 32 × hàng trên lưới 30×16; bit 15 = hiện). Kéo thả trong tab OSD |
| `osd_horizon_pos` | 233 | 0 … 65535 | Vị trí phần tử OSD "horizon" (cột + 32 × hàng trên lưới 30×16; bit 15 = hiện). Kéo thả trong tab OSD |
| `osd_warn_pos` | 33097 | 0 … 65535 | Vị trí phần tử OSD "warn" (cột + 32 × hàng trên lưới 30×16; bit 15 = hiện). Kéo thả trong tab OSD |
| `osd_coord_pos` | 481 | 0 … 65535 | Vị trí phần tử OSD "coord" (cột + 32 × hàng trên lưới 30×16; bit 15 = hiện). Kéo thả trong tab OSD |
| `bb_rate_hz` (KĐL) | 0 Hz | 0 … 1000 | Tần số ghi blackbox (0 = tắt, khuyên 500) |
| `bb_gyro_s` | 0 s | 0 … 30 | Ghi gyro tần số cao sau cất cánh (s, thẻ SD; 0 = tắt): thấy nhiễu motor trên 250 Hz |
| `bb_backend` (KĐL) | 1 | 1 … 3 | Nơi ghi: 1 cổng serial, 2 flash trên board, 3 thẻ SD. 1 = Cổng serial, 2 = Flash trên board, 3 = Thẻ SD |

## Hiệu chuẩn & cảm biến

| Tham số | Mặc định | Giới hạn | Mô tả |
|---|---|---|---|
| `imu_align` | 0 | 0 … 7 | Hướng lắp FC |
| `board_roll` | 0 ° | -180 … 180 | Chỉnh tinh hướng lắp FC: roll (°), thêm vào imu_align |
| `board_pitch` | 0 ° | -180 … 180 | Chỉnh tinh hướng lắp FC: pitch (°) |
| `board_yaw` | 0 ° | -180 … 180 | Chỉnh tinh hướng lắp FC: yaw (°) |
| `mag_align` | 0 | 0 … 7 | Hướng lắp compass |
| `est_baro_noise` | 0.4 | 0.05 … 5 | Độ nhiễu baro cho bộ ước lượng |
| `est_acc_noise` | 0.6 | 0.05 … 5 | Độ nhiễu accel cho bộ ước lượng |
| `est_gpsvel_noise` | 0.3 | 0.05 … 5 | Độ nhiễu vận tốc GPS |
| `mag_learn` | 1 | 0 … 1 | Tự học độ lệch la bàn khi bay (cần GPS và vài vòng quay), lưu khi hạ cánh |
| `rf_max_m` | 8 m | 0.5 … 40 | Tầm xa tối đa cảm biến khoảng cách (m) |
| `flow_scale` | 10.5 | 0.1 … 100 | Hệ số optical flow |
| `flow_align` | 0 | 0 … 3 | Hướng lắp optical flow |
| `flow_flip` | 0 | 0 … 3 | Lật trục optical flow |
| `acc_bias_x` | 0 | -5 … 5 | Kết quả hiệu chuẩn gia tốc: độ lệch trục x (tự ghi khi hiệu chuẩn) |
| `acc_bias_y` | 0 | -5 … 5 | Kết quả hiệu chuẩn gia tốc: độ lệch trục y (tự ghi khi hiệu chuẩn) |
| `acc_bias_z` | 0 | -5 … 5 | Kết quả hiệu chuẩn gia tốc: độ lệch trục z (tự ghi khi hiệu chuẩn) |
| `acc_calibrated` | 0 | 0 … 1 | Accel đã hiệu chuẩn |
| `mag_bias_x` | 0 | -10 … 10 | Kết quả hiệu chuẩn la bàn: độ lệch trục x (tự ghi khi hiệu chuẩn) |
| `mag_bias_y` | 0 | -10 … 10 | Kết quả hiệu chuẩn la bàn: độ lệch trục y (tự ghi khi hiệu chuẩn) |
| `mag_bias_z` | 0 | -10 … 10 | Kết quả hiệu chuẩn la bàn: độ lệch trục z (tự ghi khi hiệu chuẩn) |
| `mag_scale_x` | 1 | 0.2 … 5 | Kết quả hiệu chuẩn la bàn: hệ số trục x (tự ghi khi hiệu chuẩn) |
| `mag_scale_y` | 1 | 0.2 … 5 | Kết quả hiệu chuẩn la bàn: hệ số trục y (tự ghi khi hiệu chuẩn) |
| `mag_scale_z` | 1 | 0.2 … 5 | Kết quả hiệu chuẩn la bàn: hệ số trục z (tự ghi khi hiệu chuẩn) |
| `mag_declination` | 0 ° | -180 … 180 | Độ lệch từ (°) tại nơi bay |
| `level_trim_roll` | 0 ° | -10 … 10 | Cân bằng ANGLE: roll (°) |
| `level_trim_pitch` | 0 ° | -10 … 10 | Cân bằng ANGLE: pitch (°) |
| `est_type` | 1 | 0 … 1 | Bộ ước lượng: 0 Kalman từng trục (cũ), 1 EKF 15 trạng thái có xử lý lỗi cảm biến |
| `vibe_warn` | 30 | 0 … 200 | Cảnh báo rung trên OSD khi rung accel vượt (m/s²) |
| `acc_scale_x` | 1 | 0.8 … 1.2 | Hệ số gia tốc trục X (hiệu chuẩn 6 mặt) |
| `acc_scale_y` | 1 | 0.8 … 1.2 | Kết quả hiệu chuẩn gia tốc: hệ số trục y (tự ghi khi hiệu chuẩn) |
| `acc_scale_z` | 1 | 0.8 … 1.2 | Kết quả hiệu chuẩn gia tốc: hệ số trục z (tự ghi khi hiệu chuẩn) |
| `mag_odiag_xy` | 0 | -0.5 … 0.5 | Hiệu chuẩn la bàn ellipsoid: hệ số chéo xy (soft iron, tự ghi) |
| `mag_odiag_xz` | 0 | -0.5 … 0.5 | Hiệu chuẩn la bàn ellipsoid: hệ số chéo xz (soft iron, tự ghi) |
| `mag_odiag_yz` | 0 | -0.5 … 0.5 | Hiệu chuẩn la bàn ellipsoid: hệ số chéo yz (soft iron, tự ghi) |
| `mag_cmot_x` | 0 | -0.05 … 0.05 | Nhiễu la bàn theo dòng điện, trục X (compassmot) |
| `mag_cmot_y` | 0 | -0.05 … 0.05 | compassmot: nhiễu la bàn theo dòng điện, trục y (G/A, tự ghi) |
| `mag_cmot_z` | 0 | -0.05 … 0.05 | compassmot: nhiễu la bàn theo dòng điện, trục z (G/A, tự ghi) |
| `imu_dual` (KĐL) | 1 | 0 … 1 | IMU thứ hai (chân CS IMU2): thay khi IMU chính hỏng, và trước khi arm chọn chip tốt hơn |

## Khác

| Tham số | Mặc định | Giới hạn | Mô tả |
|---|---|---|---|
| `thr_limit_pct` | 100 % | 25 … 100 | Giới hạn ga (% cần ga ở ACRO / ANGLE / HORIZON; 100 = không giới hạn) |
| `beginner` | 0 | 0 … 1 | Chế độ bay lần đầu: tự cân bằng, nghiêng 25°, quay 250 / 120 °/s, ga 75 %, rào ảo |
| `beg_radius` | 60 m | 20 … 500 | Bay lần đầu: bán kính rào ảo quanh điểm cất cánh (m, cần GPS) |
| `thr_curve` | 0.5 | 0 … 1 | Độ cong lực đẩy theo ga (0 thẳng, 0.5 thường gặp): giữ độ cao đều ở mọi mức ga |
| `gps_max_hacc` | 5 | 1 … 50 | Bỏ fix có sai số lớn hơn (m) |
| `gps_glitch_m` | 8 m | 2 … 100 | Nhảy vị trí lớn hơn mức này coi là nhiễu (m) |
| `gps_glitch_s` | 10 s | 1 … 60 | Nhiễu kéo dài quá thời gian này thì chấp nhận vị trí mới (s) |
| `gps_heading` | 1 | 0 … 1 | Không có compass: học hướng từ GPS |
| `wind_comp` | 1 | 0 … 1 | Bù gió: 1 bật |
| `wind_drag` | 0.45 | 0.05 … 3 | Hệ số cản (1/s): đổi gió từ m/s² sang m/s |
| `rally_max_m` | 500 m | 0 … 10000 | Bay về điểm rally gần hơn nhà trong phạm vi này (m, 0 = luôn về nhà) |
| `baro_gnd` | 1 | 0 … 1 | Bỏ qua baro tụt do hiệu ứng mặt đất (đã arm, dưới khoảng 1 m) |
| `dr_time_s` | 30 s | 0 … 120 | Mất GPS: giữ vị trí bằng mô hình cản gió (EKF) trong bấy nhiêu giây, 0 tắt |
| `beacon_s` | 10 s | 0 … 120 | Motor kêu bíp (DShot beacon) sau bấy nhiêu giây mất sóng khi đang disarm, để tìm drone (0 tắt) |
| `stick_cmds` | 1 | 0 … 1 | Lệnh bằng cần khi disarm: hiệu chuẩn, chọn profile, trim, lưu, mở menu OSD |
| `headfree_type` | 0 | 0 … 1 | HEADFREE: 0 hướng mũi lúc arm là phía trước, 1 khoá theo nhà (kéo cần về = bay về nhà) |
| `circle_radius` | 10 m | 2 … 200 | CIRCLE: bán kính (m), tâm ở phía trước mũi lúc bật |
| `circle_speed` | 3 m/s | -15 … 15 | CIRCLE: tốc độ (m/s), âm = ngược chiều kim đồng hồ |
| `follow_dist` | 8 | 2 … 100 | FOLLOW: khoảng cách giữ với người dẫn (m) |
| `follow_alt` | 0 | 0 … 120 | FOLLOW: độ cao so với nhà (m), 0 = giữ độ cao lúc bật |
| `baro_dyn` | 0 | -0.1 … 0.1 | Sửa baro theo tốc độ ngang (m trên (m/s)², 0 tắt; dương khi baro đọc thấp lúc bay nhanh) |
| `sd_mode` (KĐL) | 0 | 0 … 3 | Kiểu nối thẻ SD: 0 tắt, 1 SPI, 2 SD bus 1-bit, 3 SD bus 4-bit |
| `sd_mhz` (KĐL) | 0 | 0 … 40 | Xung nhịp thẻ (0 = mặc định 20 MHz; hạ xuống 10 nếu dây dài / lỗi ghi) |
| `sd_clk` (KĐL) | 255 | 0 … 255 | Chân CLK / SCK (255 = mặc định của board) |
| `sd_cmd` (KĐL) | 255 | 0 … 255 | Chân CMD / MOSI (255 = mặc định) |
| `sd_d0` (KĐL) | 255 | 0 … 255 | Chân D0 / MISO (255 = mặc định) |
| `sd_d1` (KĐL) | 255 | 0 … 255 | Chân D1 (chỉ 4-bit, 255 = mặc định) |
| `sd_d2` (KĐL) | 255 | 0 … 255 | Chân D2 (chỉ 4-bit, 255 = mặc định) |
| `sd_d3` (KĐL) | 255 | 0 … 255 | Chân D3 / CS (4-bit và SPI, 255 = mặc định) |
| `sd_keep` | 1 | 0 … 1 | Thẻ gần đầy: tự xoá log cũ nhất (tắt = ngừng ghi) |
| `sd_dma` (KĐL) | 0 | 0 … 1 | Thẻ SD dùng DMA (STM32H7: CPU rảnh hơn khi ghi; tắt = cách cũ, chắc chắn nhất) |
| `ff_jitter` | 7 | 0 … 20 | FF: lọc rung cần (gạt chậm / tay run thì giảm FF; 0 = tắt) |
| `ff_boost` | 15 | 0 … 50 | FF: tăng tốc khi giật cần (0 = tắt) |
| `ff_max_pct` | 90 % | 0 … 100 | FF: giảm dần trên % tốc độ tối đa (chống vọt lố cuối cú lật; 0 = tắt) |
