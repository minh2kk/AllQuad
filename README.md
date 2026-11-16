# AllQuad 2.0 – Firmware điều khiển bay đa nhiệm cho ESP32 & STM32

Một mã nguồn chạy trên **ESP32, ESP32-S3, ESP32-C6, ESP32-P4, STM32F411, STM32F405, STM32F722 và STM32H743**. Firmware gom những phần mạnh nhất của ba dự án lớn:
- **Betaflight:** bay tay, lọc nhiễu.
- **iNav:** điều hướng, optical flow, logic conditions.
- **ArduPilot:** EKF, an toàn, MAVLink.

Firmware chạy trên FreeRTOS với các task tách bạch; thuật toán bay là C++ thuần, không phụ thuộc chip. Kèm theo là **trang cấu hình** chạy ngay trên drone (ESP32, qua WiFi) hoặc trên laptop qua cáp USB (mọi board), bộ phân tích blackbox có gợi ý chỉnh PID, và trình mô phỏng toàn hệ thống.

> **Chưa bay thử trên phần cứng thật.** Mọi thứ đã kiểm chứng bằng mô phỏng và test tự động (xem [Kiểm chứng](#kiểm-chứng)). Thử trên bàn, **tháo cánh**, trước khi bay. Thứ tự an toàn: không cánh → chiều motor → ANGLE → ALTHOLD → POSHOLD → RTH, ở nơi trống.

## Mục lục
1. [Bắt đầu nhanh](#bắt-đầu-nhanh)
2. [Tính năng](#tính-năng)
3. [Board, cảm biến, ngoại vi](#board-cảm-biến-ngoại-vi)
4. [Build & nạp firmware](#build--nạp-firmware)
5. [Trang cấu hình (web / laptop)](#trang-cấu-hình)
6. [Cài đặt lần đầu](#cài-đặt-lần-đầu)
7. [Chế độ bay & công tắc](#chế-độ-bay--công-tắc)
8. [Tinh chỉnh: PID, rates, bộ lọc, autotune](#tinh-chỉnh)
9. [Blackbox, thẻ SD & phân tích log](#blackbox-thẻ-sd--phân-tích-log)
10. [GPS, EKF & điều hướng](#gps-ekf--điều-hướng)
11. [Motor & ESC](#motor--esc)
12. [OSD, VTX, LED](#osd-vtx-led)
13. [An toàn & failsafe](#an-toàn--failsafe)
14. [Cổng, trạm mặt đất, WiFi](#cổng-trạm-mặt-đất-wifi)
15. [Hiệu chuẩn](#hiệu-chuẩn)
16. [Logic conditions](#logic-conditions)
17. [Lệnh CLI](#lệnh-cli)
18. [Sao lưu, cập nhật firmware, nâng cấu hình](#sao-lưu-cập-nhật-firmware-nâng-cấu-hình)
19. [Kiến trúc & hiệu năng](#kiến-trúc--hiệu-năng)
20. [Kiểm chứng & CI](#kiểm-chứng)
21. [Hướng phát triển](#hướng-phát-triển)

**Tài liệu kèm theo:**
- [`docs/HARDWARE.md`](docs/HARDWARE.md): sơ đồ đấu dây từng board, thẻ SD, bidirectional DShot.
- [`docs/PARAMS.md`](docs/PARAMS.md): **toàn bộ 359 tham số** (mặc định, giới hạn, mô tả), tạo tự động.
- [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md): task, topic, đường dữ liệu.

---

## Bắt đầu nhanh

1. **Đấu dây** theo [`docs/HARDWARE.md`](docs/HARDWARE.md). Với **ESP32-S3 N16R8**:
   - IMU ICM-45686 trên SPI.
   - BMP388 + QMC5883P trên I2C.
   - GPS ở SERIAL2, receiver CRSF ở SERIAL1.
   - Thẻ SD (tuỳ chọn) ở chân 38–41.
2. **Nạp firmware:**
   - Windows: `tools\aqflash.bat` (tự build và nạp board đang cắm).
   - Hoặc lệnh: `pio run -e esp32s3_n16r8 -t upload`.
   - Hoặc qua trang cấu hình: nút *Nạp firmware qua USB*.
3. **Mở trang cấu hình:**
   - Qua USB: mở file `build\web\AllQuad-Configurator.html` bằng Chrome / Edge → *Chọn cổng COM*.
   - Qua WiFi (ESP32): `set web_enable 1`, `save`, `reboot`, nối WiFi `AllQuad-XXXX`, mở http://192.168.4.1.
4. **Tab Thiết lập:** trình thiết lập 9 bước: hướng FC, accel, compass, baro, GPS, tay điều khiển, từng motor, lưu.
5. **Tab Chế độ:** gán ARM, ANGLE, ALTHOLD, POSHOLD, RTH vào công tắc.
6. **Thử motor** (tháo cánh), kiểm tra failsafe trên bàn, rồi bay thử ở ANGLE.
7. **Sau mỗi chuyến:** tab **Blackbox** → phân tích → *Kế hoạch chỉnh* → *Áp dụng* → bay lại.

Kiểm tra nhanh trên board: `tools\aqtest.bat` (Windows) gửi `status` / `perf` / `tasks` qua USB và ghi kết quả ra `tools\aqtest_log.txt`.

---

## Tính năng

| Nhóm | Tính năng | Cảm hứng |
|---|---|---|
| **Bay tay** | PID tốc độ 1–4 kHz, hệ số tương thích Betaflight (P/I/D/F), feedforward, I-term relax, TPA, anti-gravity, airmode, rates "Actual", D_min, RC smoothing, throttle boost, dynamic idle, tuyến tính hoá lực đẩy, bù sụt áp pin, **3 profile PID / rates** chọn bằng công tắc, chống tích luỹ I khi motor bão hoà | Betaflight / ArduPilot |
| **Lọc nhiễu** | **RPM filter** (3 hoạ âm × 4 motor, trọng số từng hoạ âm, mờ dần ở vòng quay thấp), **notch động FFT**, 2 notch tĩnh, lọc thông thấp gyro 2 tầng (tầng 2 động theo ga), lọc D-term | Betaflight |
| **Chế độ** | ACRO, ANGLE, HORIZON, ALTHOLD, POSHOLD, CRUISE, AUTOTUNE, RTH, SMART RTL, GUIDED, GPS RESCUE, LAUNCH, MISSION (120 điểm), LAND, TURTLE, BEEPER | BF / iNav / ArduPilot |
| **Ước lượng** | **EKF 15 trạng thái** có xử lý lỗi cảm biến (GPS nhảy, baro lệch, compass sai → loại và chuyển nguồn); AHRS quaternion bù gia tốc bằng GPS; Kalman 3 trạng thái (chế độ cũ); ước lượng gió; học hướng từ GPS khi không có compass; IMU dự phòng | ArduPilot |
| **Điều hướng** | RTH về nhà hoặc điểm rally gần nhất; nhiệm vụ TAKEOFF / WAYPOINT / LOITER (giờ, vô hạn, bay vòng) / DO_JUMP / ROI / CONDITION_YAW / CHANGE_SPEED / LAND / RTL; **quét khu vực**; bù gió; tự học ga treo; bám mặt đất bằng cảm biến khoảng cách; giữ vị trí bằng optical flow khi không GPS | iNav / ArduPilot |
| **An toàn** | 18 kiểm tra trước arm, failsafe mất sóng / pin / trạm mặt đất, geofence trụ + **đa giác**, phát hiện va chạm, phục hồi xoay yaw, phát hiện hạ cánh, GPS Rescue kiểm tra tiến triển | ArduPilot / BF |
| **Blackbox** | Ghi vào **cổng serial (OpenLog), flash trên board hoặc thẻ SD** (SPI / SD bus 1-bit / 4-bit). Nén chênh lệch (~4×). **Khung ESC** (RPM + nhiệt độ từng motor, 50 Hz) | Betaflight |
| **Phân tích** | Đáp ứng bước, phổ nhiễu, độ trễ, prop wash, TPA, trôi, cân bằng motor, **motor yếu theo RPM**, nhiệt ESC, điện trở pin, cộng hưởng khung → **gợi ý + kế hoạch chỉnh có giới hạn ±25 %**; so sánh hai chuyến bay; xuất CSV | — |
| **OSD** | Analog MAX7456 / AT7456E; kính số qua **MSP DisplayPort**: DJI O3 / O4 / O4 Pro, Walksnail, HDZero; lưới SD / 50×18 / 53×20 / 60×22; font Betaflight; màn hình thống kê sau chuyến bay | Betaflight |
| **Trạm mặt đất** | **MAVLink 2**: QGroundControl / Mission Planner (tham số, nhiệm vụ, rally, geofence, lệnh), qua USB, UART hoặc **WiFi UDP** | ArduPilot |
| **Trang cấu hình** | 12 tab, trình thiết lập phần cứng, thử motor, ESC passthrough, OSD kéo thả, soạn nhiệm vụ trên bản đồ, CLI, nạp firmware qua USB, cập nhật qua WiFi, **drone ảo** (firmware thật dạng WebAssembly), **mô phỏng SITL** | BF / iNav Configurator |
| **Lập trình** | **Logic conditions**: 8 luật "A so sánh B → hành động" | iNav |
| **Hiệu chuẩn** | Accel 6 mặt, compass ellipsoid (hard + soft iron), **compassmot**, hướng lắp tự tìm | ArduPilot |

---

## Board, cảm biến, ngoại vi

| Env PlatformIO | Board | Vòng bay | Ghi chú |
|---|---|---|---|
| `esp32dev` | ESP32 DevKit (4 MB) | 1–2 kHz | WiFi, OTA, blackbox flash ~1,1 MB |
| `esp32s3` | ESP32-S3 (8 MB, không PSRAM) | 1–2 kHz | USB gốc |
| `esp32s3_n16r8` | **ESP32-S3 N16R8** (16 MB, 8 MB PSRAM) | 1–2 kHz | blackbox flash 9,9 MB, **thẻ SD** (hàng đợi 256 kB PSRAM) |
| `esp32c6` | ESP32-C6 | 1 kHz | OneShot125, không thẻ SD |
| `esp32p4` | ESP32-P4 | 1–2 kHz | không WiFi: cấu hình qua USB |
| `blackpill_f411ce` | STM32F411 BlackPill | 1 kHz | |
| `stm32f405` / `stm32f722` | FC F405 / F722 | 1–2 kHz | SERIAL4/5 |
| `stm32h743_weact` | **WeAct MiniSTM32H743** | **3,2 kHz** (tối đa 4) | khe **microSD có sẵn** (bật mặc định), DShot600 + bidir + EDT |
| `stm32h743_devebox` | **MCUDev DevEBox STM32H743VIT6** | **3,2 kHz** | như WeAct; IMU INT1 → PC4, I2C2 PB11/PB10, RX trên PA9/PA10 = SERIAL4 |
| `stm32f405_qemu` | (máy ảo QEMU) | – | chỉ để kiểm tra khởi động: `tools/qemu_f405.sh` |

| Loại | Thiết bị (tự nhận diện) |
|---|---|
| IMU | BMI088, **ICM-45686**, ICM-45605, ICM-42688-P, ICM-42605, LSM6DSO / DSR / DSL, ISM330DHCX, MPU6000/6500/9250, ICM-20602/20689 (SPI); MPU6050 (I2C) |
| Baro | DPS310 / SPL06, **BMP388**, BMP390, BMP280, BME280, MS5611 |
| Compass | QMC5883L, **QMC5883P**, HMC5883L, IST8310, LIS3MDL |
| GPS | u-blox M8/M9/M10 (UBX NAV-PVT, tự cấu hình 10 Hz) + NMEA |
| Receiver | **CRSF / ExpressLRS** (kèm telemetry ngược), SBUS, iBUS |
| ESC | DShot300/600 (lệnh đặc biệt), **bidirectional DShot**, **EDT** (nhiệt / áp / dòng qua dây tín hiệu), PWM, OneShot125; telemetry serial BLHeli32 / AM32 / KISS |
| Khác | TFmini / TF-Luna / TF02, optical flow MSP (MTF-01, 3901-L0X), VTX SmartAudio / Tramp / MSP, LED WS2812, **thẻ microSD** |

Sơ đồ chân từng board: [`docs/HARDWARE.md`](docs/HARDWARE.md).

---

## Build & nạp firmware

Cài [PlatformIO](https://platformio.org), rồi:
```bash
pio run -e esp32s3_n16r8 -t upload       # ESP32-S3 N16R8 (đổi env theo bảng trên)
pio run -e stm32h743_weact -t upload     # WeAct H743 (giữ BOOT0 + bấm RESET để vào DFU lần đầu)
pio device monitor                       # CLI qua USB (ESP32 921600 baud, STM32 USB)
```
- ESP32 dùng pioarduino (Arduino-ESP32 3.x).
- **STM32 dùng STM32Cube (không còn Arduino / STM32duino)**: thư viện HAL của ST, CMSIS, nhân FreeRTOS, thư viện USB device của ST và FatFs nằm sẵn trong `lib/` — build không cần tải thư viện nào (chỉ toolchain GCC). Chi tiết ở mục *STM32 trên STM32Cube* bên dưới.
- **Trang cấu hình nhúng trong firmware** (`src/web/web_ui_data.h`) được tạo lại tự động khi build nếu bạn sửa `src/web/ui/*`. Có Node thì chạy `npm install` một lần trong thư mục dự án để trang được nén bằng terser + csso (nhỏ hơn ~5 %); không có Node vẫn build được.
- Các module không gấp về thời gian (CLI, web, MAVLink, OSD, thẻ SD…) biên dịch tối ưu kích thước; vòng bay `-O2`.
- Máy không vào được registry PlatformIO: đặt sẵn gói vào `~/.platformio/packages`, platform lấy từ GitHub.

### STM32 trên STM32Cube (không Arduino)

| Phần | Cách làm | Tệp |
|---|---|---|
| Khởi động, xung nhịp | `main()` riêng: H743 400 MHz (VOS1, thạch anh 25 MHz, USB 48 MHz từ PLL3), F405 168 MHz, F722 216 MHz (overdrive), F411 96 MHz. F4/F7 **đo tần số thạch anh** lúc khởi động (8 / 25 MHz…), không có thạch anh thì chạy HSI | `src/hal/stm32/system_stm32.cpp` |
| Thời gian | timer 32-bit 1 MHz (H7 TIM5, F4/F7 TIM2) + đếm tràn → `micros()` 64-bit không bao giờ quay vòng; SysTick chỉ dành cho FreeRTOS | `system_stm32.cpp` |
| Cổng UART | driver ngắt riêng, **vòng đệm nhận 1 kB mỗi cổng**, cờ overrun / framing được xoá trong ngắt (F7/H7 bật `OVRDIS`): một byte lỗi không bao giờ làm cổng ngừng nhận — đây là nguyên nhân hay gặp của "mất RX" | `src/hal/hal_stm32cube.cpp` |
| I2C | HAL của ST, chế độ polling, timeout ngắn theo độ dài; lỗi bus → reset ngoại vi + 9 xung SCL giải phóng SDA rồi khởi tạo lại. F7/H7 tự tính thanh ghi `TIMINGR` cho 100 / 400 kHz | `hal_stm32cube.cpp` |
| SPI | F4/F7 truy cập thanh ghi trực tiếp (nhanh cho 1–4 kHz), H7 HAL; CS bằng GPIO | `hal_stm32cube.cpp` |
| USB | cổng COM ảo CDC (VID 0483 / PID 5740), vòng đệm 2–4 kB, **không làm rơi byte** (host phải chờ khi đệm đầy); không dùng VBUS nên **PA9 / PA10 là UART bình thường** | `src/hal/stm32/usb_cdc.c` |
| Thẻ SD (H743) | SDMMC1 4-bit + FatFs, điều khiển luồng phần cứng | `src/hal/stm32/sd_stm32.cpp` |
| Cấu hình | 8 kB cuối sector flash cuối, **cùng bố cục với bản STM32duino cũ** → nâng cấp không mất cấu hình | `hal_stm32cube.cpp` |
| Định tuyến ngoại vi | UART / I2C / SPI / timer / kênh ADC của từng board | `src/hal/stm32/board_stm32.h` |
| Linker / cấu hình | `ld/*.ld`, `src/hal/stm32/conf/` (HAL, FreeRTOS, USB, FatFs) | |

Kiểm tra: `pio run -e stm32f405_qemu && tools/qemu_f405.sh` chạy firmware F405 trong QEMU và gọi CLI (`version`, `status`).

**Nạp không cần PlatformIO:** trang cấu hình → *Nạp firmware qua USB* (cả khi board trống):
- **ESP32 / S3 / C6 / P4:** qua ROM bootloader, có MD5; từ chối file sai chip.
- **STM32:** qua DFU (WebUSB). Chỉ xoá các sector cần, giữ sector cấu hình. Windows cần driver WinUSB một lần (Zadig).
- Trang **tự tải file sao lưu cấu hình** trước khi nạp.

---

## Trang cấu hình

Cùng một giao diện, ba cách kết nối:

| Cách | Mở | Ghi chú |
|---|---|---|
| **USB (mọi board)** | `build/web/AllQuad-Configurator.html` trên Chrome / Edge, hoặc `python3 tools/configurator.py` (http://localhost:8180) | Web Serial; không cần mật khẩu; arm bị chặn khi đang nối |
| **WiFi (ESP32)** | `set web_enable 1` → `save` → `reboot`; WiFi `AllQuad-XXXX`, mật khẩu mặc định `allquad123`; http://192.168.4.1 | Đăng nhập; **đổi mật khẩu ngay** (tab Hệ thống); WiFi tự tắt khi arm (`wifi_armed 0`) |
| **Không cần board** | *Thử với drone ảo* (firmware WebAssembly trong trình duyệt) hoặc *Mô phỏng trên máy* (SITL qua `tools/configurator.py`) | Mọi tab dùng được |

**Quy tắc chung:**
- Thay đổi vào hàng chờ (thanh vàng dưới đáy) → *Xem lại* (danh sách cũ → mới, hoàn tác từng dòng) → *Áp dụng* (có hiệu lực ngay, chỉ khi disarm) → *Lưu vào flash*. Đã áp dụng mà chưa lưu thì thanh chuyển xanh "chưa lưu vào flash" và trình duyệt hỏi lại khi đóng trang.
- Thanh trên cùng: **Tìm** (`Ctrl+K` hoặc `/`: tìm tham số theo tên / mô tả có hoặc không dấu, Enter mở đúng tab và tô sáng ô cần sửa), **Đơn giản / Nâng cao** (đơn giản chỉ hiện ~50 thông số chính; tab Tham số luôn đủ), **giao diện** Tự động / Sáng / Tối / **Ngoài trời** (tương phản tối đa, chữ to: đọc được dưới nắng), chấm kết nối kèm **tốc độ cập nhật thực tế** (Hz).
- Thanh bên trên laptop thu gọn được (nút *Thu gọn* ở cuối) để nhường chỗ cho đồ thị.
- Nhãn "khởi động lại" = cần lưu rồi khởi động lại FC.
- Mỗi tham số hiện tên kỹ thuật (`roll_p`…), giới hạn và giải thích.
- Giao diện sáng / tối; trên điện thoại menu nằm dưới đáy.

| Tab | Chức năng chính | Trường / tham số quan trọng |
|---|---|---|
| **Tổng quan** | Khung sẵn sàng arm (xanh = sẵn sàng, đỏ = liệt kê từng lý do kèm cách sửa, bấm để tới đúng tab), ô trạng thái (pin, tín hiệu, GPS, chế độ, độ cao, thời gian bay còn lại, tải CPU), chân trời nhân tạo lớn có thang nghiêng, gyro so với setpoint, motor, kênh RC, danh sách tình trạng cảm biến có đèn trạng thái, cảnh báo rung / EKF | `bat_capacity` (để tính thời gian còn lại) |
| **Trước bay** | Danh sách kiểm tra trước khi bay: tự kiểm tra (kết nối, IMU, accel / compass đã hiệu chuẩn, baro, tay điều khiển + số lần mất sóng, công tắc ARM, failsafe, pin / V-cell, GPS khi dùng chế độ cần GPS, điều kiện arm, cấu hình đã lưu, chỗ trống blackbox, rung) và 7 mục kiểm tra bằng mắt (tự bỏ tích sau 12 giờ); điểm tổng và nút *Sửa* tới đúng tab | — |
| **Thiết lập** | Trình thiết lập 9 bước, kiểm tra bằng số liệu thật | `imu_align`, `board_roll/pitch/yaw` |
| **PID & lọc** | Bảng P/I/D/F 3 profile, thanh trượt chỉnh nhanh (độ mạnh chung, giảm chấn, feedforward, I); cảm giác bay & bảo vệ; rates kèm đồ thị; bộ lọc kèm đồ thị đáp ứng tần số & độ trễ | `roll_p…yaw_f`, `p2_*`, `p3_*`, `*_rc_rate`, `*_max_rate`, `*_expo`, `gyro_lpf_hz`, `dterm_lpf_hz`, `dyn_notch_*`, `rpm_harmonics`, `rpm_w1..3` |
| **Chế độ & an toàn** | **Tín hiệu tay điều khiển**: LQ 2 phút gần nhất, trung bình / thấp nhất, số lần và thời gian mất sóng dài nhất; gán chế độ cho khoảng kênh AUX (kéo, Thấp / Giữa / Cao), xem công tắc trực tiếp; failsafe, pin, geofence | `mrN_mode/ch/lo/hi`, `fs_action`, `bat_fs_action`, `fence_*` |
| **Điều hướng** | Bản đồ (nhà, drone, vệt bay, waypoint, rally, geofence, gió; bản đồ nền OSM); **soạn nhiệm vụ** (chạm thêm, kéo dời), **quét khu vực**, **geofence đa giác**, **bay tới điểm chạm** (GUIDED), nhập / xuất `.waypoints`; **thống kê nhiệm vụ** (quãng đường, thời gian, xa nhà nhất, cao nhất, mAh ước tính theo dòng đo được, cảnh báo khi > 70 % pin); bảng EKF | `rth_alt`, `wp_speed`, `nav_*`, `fence_poly`, `gcs_fs` |
| **Cảm biến** | Mô hình 3D, đồ thị gyro / accel / compass / độ cao, cảm biến khoảng cách, optical flow, RPM + nhiệt ESC; hiệu chuẩn & hướng lắp | `acc_*`, `mag_*`, `level_trim_*` |
| **Motor & ESC** | **Thử motor** (khoá đến khi tích "đã tháo cánh"), đo rung từng motor, kiểm tra phản ứng (không cần arm), thứ tự & chiều quay (tìm tự động), giao thức ESC, bíp / đảo chiều từng motor, hiệu chuẩn ESC PWM, **ESC passthrough** (esc-configurator.com) | `motor_protocol`, `dshot_bidir`, `dshot_edt`, `motor_poles`, `yaw_reversed`, `motorN_out` |
| **Cổng & thiết bị** | Chức năng + baud từng UART (cảnh báo trùng), VTX (băng / kênh / công suất / pit, bảng VTX riêng), dải LED | `serialN_fn`, `serialN_baud`, `vtx_*`, `vtxt_*`, `led_*` |
| **OSD** | Kéo thả phần tử, bật / tắt, analog hay kính số, lưới chữ, icon hay chữ, xem trước HD, hướng dẫn DJI / Walksnail / HDZero | `osd_backend`, `osd_canvas`, `osd_font`, `osd_stats`, `osd_*_pos` |
| **Blackbox** | Tải log từ drone hoặc mở file, **thẻ SD** (trạng thái, danh sách, phân tích / lưu / xoá từng file), phân tích, **kế hoạch chỉnh**, bảng ESC, ghim chuyến mốc để so sánh, xuất CSV, **bản đồ nhiệt nhiễu theo mức ga** (vệt chéo = nhiễu motor, vệt đứng = cộng hưởng khung, có vạch cắt lọc), **sự kiện chuyến bay** (arm / disarm, đổi chế độ, motor chạm 100 %, điện áp thấp nhất, tốc độ quay lớn nhất; bấm để nhảy tới đoạn đó) | `bb_rate_hz`, `bb_backend`, `sd_*` |
| **Tham số** | **Hồ sơ drone** (lưu cấu hình theo tên trong trình duyệt, nạp vào hàng chờ, xuất / nhập file `.json`; không chép hiệu chuẩn cảm biến, WiFi, cổng), preset (5 inch, 3 inch, cinewhoop, 7 inch, bộ lọc, rates, an toàn GPS), mọi tham số theo nhóm, tìm kiếm, lịch sử thay đổi + hoàn tác + CSV | — |
| **CLI** | Dòng lệnh như Betaflight, nhớ lệnh, Tab tự hoàn thành, nút nhanh | — |
| **Hệ thống** | Lưu / khởi động lại / mặc định / sáng-tối, sao lưu & khôi phục, so sánh với file sao lưu, nạp firmware USB, cập nhật WiFi (OTA), WiFi & bảo mật, **cài đặt blackbox + thẻ SD**, cảnh báo giọng nói, logic conditions, bảng task và thời gian từng khâu vòng bay | `wifi_enable`, `web_enable`, `wifi_armed`, `mav_sysid` |

**Không đè tải lên FC:**
- Trạng thái được đẩy qua **một WebSocket**: 10 Hz ở tab cần dữ liệu nhanh, 2 Hz ở các tab khác, 2 Hz khi armed.
- Bản tin được chia tầng: GPS / ước lượng gửi xen kẽ, autotune chỉ khi đang chạy.
- Trang có **ETag**, nên tải lại chỉ tốn một phản hồi 304.
- Danh sách thẻ SD chỉ đọc khi mở tab Blackbox.
- Mọi phân tích, đồ thị, bản đồ chạy trong trình duyệt.
- Web server chạy ở nhân 0, ưu tiên thấp.

**Bảo mật WiFi:**
- Phiên đăng nhập bằng cookie `HttpOnly; SameSite=Strict`, hết hạn sau 2 giờ không dùng.
- Sai 5 lần thì khoá 30 s.
- Không mở CORS.
- Mật khẩu nằm ngoài bảng tham số, không gửi qua MAVLink.
- Quên mật khẩu: cắm USB, dùng lệnh `wifi`.

---

### Dùng trang cấu hình trên điện thoại

| Board | Cách nối | Ghi chú |
|---|---|---|
| ESP32 / ESP32-S3 | WiFi `AllQuad-xxxx`, mở `192.168.4.1` | Không cần cáp; mọi điện thoại |
| **STM32** (F411 / F405 / F722 / H743) | **Android + cáp OTG** (USB-C/micro-USB OTG ↔ cổng USB của FC), Chrome mở trang cấu hình | Chrome 148+ dùng Web Serial; Chrome cũ hơn dùng WebUSB (nút *Kết nối USB (điện thoại / WebUSB)*). iPhone không có Web Serial / WebUSB nên không nối USB được |

Trang cấu hình cho STM32 là một file (`build/web/AllQuad-Configurator.html`, gồm cả drone ảo). Web Serial / WebUSB chỉ chạy trên trang **https** (hoặc file mở trực tiếp trên máy tính), nên với điện thoại hãy đặt file này lên một chỗ https một lần (GitHub Pages, Netlify…, đổi tên thành `index.html`), rồi mở link đó bằng Chrome và *Thêm vào màn hình chính*. Trang tự nhận ra mình là bản USB (không đi tìm drone qua WiFi).

## Cài đặt lần đầu

**THÁO CÁNH QUẠT.** Dùng tab **Thiết lập**, hoặc làm tay qua CLI:

1. `status` kiểm tra cảm biến; `scan` để kiểm tra dây:
   - Liệt kê thiết bị I2C, mức SDA/SCL, mã chip và mã IMU.
   - Baro / compass cắm muộn (ví dụ module ăn nguồn từ pin khi board chạy USB) được tự tìm lại mỗi 2 s khi disarm.
   - Không thấy gì ở 400 kHz thì FC tự thử 100 kHz.
2. Hướng lắp: nghiêng phải → roll dương, ngóc mũi → pitch dương. Sai thì `set imu_align ...` hoặc dùng bước "tìm hướng lắp" của trình thiết lập.
3. `calib_acc` (hoặc `calib_acc6`), `calib_mag 30`, `set mag_declination <độ>`, `save`.
4. Gán cổng: `set serial1_fn 1` (receiver), `set serial2_fn 2` (GPS)… (mã ở [Cổng](#cổng-trạm-mặt-đất-wifi)). `save` → `reboot`.
5. `set rx_protocol 0` (CRSF; 1 SBUS, 2 iBUS); `set motor_protocol 2` (DShot300; 3 DShot600).
6. Pin:
   - Chỉnh `vbat_scale` theo đồng hồ đo.
   - Đặt `bat_capacity` (mAh) để có thời gian bay còn lại và failsafe theo dung lượng.
   - Board không có cầu chia áp: điện áp / dòng lấy từ ESC (EDT).
7. Thử motor ở tab Motor: thứ tự và chiều quay. Sai thứ tự: `motorN_out` (không cần đổi dây). Sai chiều: đảo chiều trong ESC (DShot).
8. Thử failsafe: arm trên bàn không cánh, tắt tay điều khiển, motor phải dừng hoặc vào chế độ đã chọn.

---

## Chế độ bay & công tắc

Mỗi chế độ gán cho **một khoảng giá trị của một kênh AUX**, tối đa 12 dòng (`mrN_mode`, `mrN_ch`, `mrN_lo`, `mrN_hi`). Ví dụ: `set mr10_mode 3`, `set mr10_ch 10`, `set mr10_lo 1300`, `set mr10_hi 1700`. Kênh 4 = AUX1 / CH5.

| Mã | Chế độ | Làm gì |
|---|---|---|
| 1 | ARM | Arm motor |
| 2 | ANGLE | Tự cân bằng, giới hạn góc `angle_limit` |
| 3 | HORIZON | Tự cân bằng ở giữa cần, lật được khi gạt hết cần (`horizon_trans`) |
| 4 | ALTHOLD | Giữ độ cao (baro / GPS / cảm biến khoảng cách) |
| 5 | POSHOLD | Giữ vị trí (GPS hoặc optical flow) |
| 6 | RTH | Bay về nhà / điểm rally gần nhất, hạ cánh |
| 7 | MISSION | Chạy nhiệm vụ (arm ngay trong MISSION để tự cất cánh) |
| 8 | LAND | Hạ cánh tại chỗ |
| 9 | TURTLE | Lật drone khi úp (bật rồi arm, dùng cần roll / pitch) |
| 10 | BEEPER | Còi tìm drone |
| 11 | CRUISE | Giữ hướng và tốc độ; cần pitch tăng / giảm tốc, yaw rẽ |
| 12 | AUTOTUNE | Tự chỉnh PID khi bay (xem [Tinh chỉnh](#tinh-chỉnh)) |
| 13 | GPS RESCUE | Như RTH, thêm kiểm tra tiến triển: `rth_sanity_s` giây không lại gần nhà → hạ cánh |
| 14 | LAUNCH | Launch control: giữ góc chúi `launch_angle`, phóng khi ga > `launch_thr` |
| 15 | SMART RTL | Bay về theo đường đã bay (`srtl_point_m`) |

- **Ưu tiên khi nhiều chế độ cùng bật:** LAND > RTH / GPS RESCUE > MISSION > AUTOTUNE > CRUISE > POSHOLD > ALTHOLD > HORIZON > ANGLE. Không bật gì là ACRO.
- **GUIDED** (không cần công tắc): QGC "Go to location", nút *Bay tới đây* trên bản đồ web, hoặc `goto <latE7> <lonE7> [alt]`.
- **Bố trí mặc định:**
  - CH5 cao = ARM.
  - CH6: ACRO / ANGLE / ALTHOLD.
  - CH7: — / POSHOLD / RTH.
  - CH8: — / MISSION / LAND.
  - CH9 cao = còi; CH10 cao = TURTLE.
- **Phi công luôn lấy lại quyền:** gạt bất kỳ công tắc chế độ nào khi đang failsafe pin / geofence.

---

## Tinh chỉnh

### PID, rates, profile
- **PID:** `roll_p`, `roll_i`, `roll_d`, `roll_f` (tương tự pitch / yaw). Cùng thang với Betaflight.
- **Rates "Actual":**
  - `*_rc_rate`: tốc độ ở tâm cần (°/s).
  - `*_max_rate`: tốc độ khi gạt hết cần.
  - `*_expo`: độ cong quanh tâm cần.
- **3 profile:**
  - Profile 2, 3 nằm ở các tham số `p2_*`, `p3_*`.
  - Chọn bằng `pid_profile`, hoặc bằng công tắc `prof_ch` (thấp / giữa / cao = profile 1 / 2 / 3).
  - Trang PID có nút chuyển profile ngay trên bảng.
- **Cảm giác bay:**

  | Tham số | Ví dụ / tác dụng |
  |---|---|
  | `anti_gravity` | |
  | `iterm_relax` | |
  | `tpa_rate`, `tpa_breakpoint` | |
  | `d_min_pct` | vd 60 |
  | `thr_boost` | |
  | `thrust_expo` | vd 0,6 |
  | `vbat_sag_comp` | 0–100 |
  | `rc_smooth_hz` | 0 = tự động |
  | `dyn_idle_rpm` | đơn vị ×100 rpm, cần RPM |
  | `airmode` | |

### Bộ lọc
- **Lọc thông thấp gyro:** `gyro_lpf_hz`, `gyro_lpf2_hz`. Tầng động theo ga: `dyn_lpf_min` / `dyn_lpf_max`.
- **D-term:** `dterm_lpf_hz`.
- **Notch động FFT:** `dyn_notch_count`, `dyn_notch_min` / `dyn_notch_max` (Hz), `dyn_notch_q`.
- **Notch tĩnh:** `notch1_hz` / `notch1_cut`, `notch2_*`.
- **RPM filter:** `rpm_harmonics` (0–3). Cần `dshot_bidir 1` hoặc telemetry ESC.
  - `rpm_w1..3`: trọng số từng hoạ âm (%).
  - `rpm_min_hz`: dưới mức này notch mờ dần.
- Tab PID → Bộ lọc vẽ đáp ứng tần số và độ trễ ước tính khi bạn chỉnh.

### Quy trình chỉnh bằng blackbox (khuyên dùng)
1. Bật blackbox: `bb_rate_hz 500` (thẻ SD cho phép bay lâu, xem phần sau).
2. Bay ACRO 60 s trở lên: mỗi trục vài cú lắc cần nhanh qua lại, một đoạn bay êm, vài lần hạ ga đột ngột.
3. Hạ cánh, disarm → tab **Blackbox** → *Tải log* (hoặc chọn file trên thẻ SD → *Phân tích*).
4. Đọc **Kế hoạch chỉnh từ chuyến bay này**:
   - Mọi gợi ý được gộp thành một bộ thay đổi.
   - Mỗi tham số đổi tối đa ±25 % một lần và nằm trong giới hạn.
   - Có lý do cho từng thay đổi.
5. Tích các mục muốn dùng → *Đưa vào hàng chờ* → *Áp dụng* → bay lại. Thường 2–4 vòng.
6. *Ghim chuyến mốc* để so sánh đáp ứng bước, vọt lố, nhiễu giữa hai lần bay.

**Bộ phân tích đo:**
- **Đáp ứng bước:** vọt lố, thời gian lên, mức bám.
- **Phổ nhiễu:** gyro thô, sau lọc, D-term.
- **Độ trễ bám cần.**
- **Prop wash.**
- **Nhiễu ga cao (TPA).**
- **Trôi chậm:** dấu hiệu I yếu.
- **Cân bằng motor:** trọng tâm lệch.
- **Motor yếu theo RPM:** cánh hỏng, bạc đạn.
- **Nhiệt ESC.**
- **Điện trở pin / sụt áp.**
- **Cộng hưởng khung.**

### Autotune (khi bay)
1. Gán AUTOTUNE vào công tắc (cần baro).
2. Bay ALTHOLD 5–10 m nơi trống, bật công tắc, **buông cần**. Drone tự giật roll rồi pitch và chỉnh P/D, feedforward, I, rồi bộ lọc.
3. Đẩy cần để tạm dừng; buông cần thì tiếp tục.
4. Khi OSD báo "AUTOTUNE DONE":
   - Hạ cánh và **disarm khi công tắc vẫn bật** → kết quả được lưu.
   - Tắt công tắc trước khi disarm → bỏ kết quả.
5. Tham số:
   - `at_axes`: 1 roll, 2 pitch, 4 yaw (cộng lại).
   - `at_rate`: tốc độ cú giật.
   - `at_steps`: 1 P/D, 2 FF, 4 I, 8 bộ lọc.

---

## Blackbox, thẻ SD & phân tích log

| `bb_backend` | Nơi ghi | Dung lượng (500 Hz, đã nén) | Lấy log |
|---|---|---|---|
| 1 | Cổng serial chức năng 8 (OpenLog) | theo thẻ của OpenLog | rút thẻ OpenLog |
| 2 | Flash trên board | ESP32 4 MB ~1,5 phút; S3 N16R8 ~13 phút; H743 ~80 s | tab Blackbox → *Tải log* |
| 3 | **Thẻ SD** | hàng chục giờ | tab Blackbox → Thẻ SD, hoặc rút thẻ |

**Tham số chung:**
- `bb_rate_hz`: 0 = tắt; 500 là khuyến nghị; tối đa 1000.
- Chỉ ghi khi armed.

### Thẻ SD (`sd_mode`, cấu hình v25)

| Tham số | Ý nghĩa |
|---|---|
| `sd_mode` | 0 tắt, 1 SPI, 2 SD bus 1-bit, 3 SD bus 4-bit (trang chỉ hiện kiểu board làm được) |
| `sd_mhz` | xung nhịp, 0 = mặc định 20 MHz; hạ xuống 10 nếu dây dài / lỗi ghi |
| `sd_clk`, `sd_cmd`, `sd_d0`…`sd_d3` | chân GPIO (SPI: SCK, MOSI, MISO, CS = D3); 255 = mặc định của board |
| `sd_keep` | 1: thẻ còn < 16 MB thì tự xoá log cũ nhất; 0: ngừng ghi |

**Kiểu nối theo board:**
- **H743 WeAct:** khe có sẵn, 4-bit, bật mặc định.
- **ESP32-S3 N16R8:**
  - SPI hoặc 1-bit: CLK/SCK 39, CMD/MOSI 38, D0/MISO 40, D3/CS 41.
  - 4-bit cần thêm chân D2.
- **SPI:** cần bus SPI thứ hai, nghĩa là không lắp chip OSD MAX7456.
- **Kiểm tra chân:** firmware từ chối chân trùng motor / cảm biến / UART hoặc chân strapping.
- Chi tiết: [`docs/HARDWARE.md`](docs/HARDWARE.md#thẻ-nhớ-sd-blackbox-từ-config-v25).

**Cách hoạt động:**
- **Mỗi lần arm một file:** `/AQLOG/AQ00001.BBL`…
- **Vòng bay không chờ thẻ:** dữ liệu vào hàng đợi RAM (256 kB PSRAM trên S3 N16R8, 64 kB H743, 32 kB ESP32); task `sdcard` ưu tiên thấp ghi ra thẻ theo khối 4–16 kB.
- **Thẻ khựng:** khựng vài trăm ms khi tự xoá bên trong không mất dữ liệu (test: khựng 4 s không mất byte nào).
- **Mất điện / rơi:** mỗi giây chốt độ dài file, nên chỉ mất tối đa ~1 s cuối.
- **Rút thẻ / lỗi ghi:** dừng ghi chuyến đó, tìm lại thẻ mỗi 3 s, chuyến sau ghi file mới.
- **Tab Blackbox → Thẻ SD:**
  - Trạng thái, dung lượng, lần ghi chậm nhất, số byte mất.
  - Danh sách log kèm nút *Phân tích* / *Lưu* / *Xoá* (khi disarm).
- **CLI:** `sd`, `sd ls`, `sd rm AQ00003.BBL`, `sd mount`.
- Log lớn: rút thẻ cắm vào máy tính rồi dùng *Mở file log*, nhanh hơn nhiều so với tải qua USB.

### Định dạng log
- **Header chữ** (`H ...`): phiên bản, PID, bộ lọc, `encoding delta1`, `esc_frame 23`.
- **0xA5:** bản ghi đầy đủ 94 byte, mỗi 32 bản ghi một lần.
  - Nội dung: gyro thô / lọc, setpoint, P/I/D/F, 4 motor, ga, 4 kênh RC, tư thế, độ cao, vận tốc đứng, vị trí N/E, điện áp, dòng, chế độ, pha điều hướng, cờ.
- **0xA6:** khung chênh lệch.
  - Gồm: byte kiểm của bản ghi gốc, mặt nạ trường thay đổi, zig-zag varint, checksum.
  - Hỏng một byte chỉ mất tới bản ghi đầy đủ kế tiếp, không bao giờ giải sai.
- **0xA7 (mới):** khung ESC 23 byte, ~50 Hz.
  - Gồm: RPM cơ khí từng motor (từ bidirectional DShot, EDT hoặc telemetry serial), nhiệt độ từng ESC, điện áp và dòng ESC.
  - Trình đọc cũ bỏ qua khung này.
- **Giải mã trên máy tính:** `python3 tools/blackbox_decode.py LOG.BBL` → `LOG.csv` (bản ghi) + `LOG_esc.csv` (RPM / nhiệt).

---

## GPS, EKF & điều hướng

- **GPS:**
  - Cổng chức năng 2. FC tự chuyển u-blox sang 115200 baud, 10 Hz, NAV-PVT, airborne.
  - `gps_min_sats`.
  - `gps_max_hacc`: loại fix sai số lớn.
  - **Chống nhảy:** `gps_glitch_m` / `gps_glitch_s`.
- **EKF:**
  - `est_type 1`: EKF 15 trạng thái. GPS nhảy → nhận vị trí mới; baro lệch 5 s → độ cao từ GPS; compass sai 5 s → hướng từ GPS. Web hiện tỉ số kiểm định từng cảm biến.
  - `est_type 0`: bộ Kalman tách trục cũ.
- **Không compass:** `gps_heading 1` học hướng từ chuyển động. Trước khi học xong, RTH bay tới – lui để học.
- **Độ cao:**
  - `hover_learn 2`: tự học ga treo.
  - `alt_surface 1`: bám mặt đất bằng cảm biến khoảng cách, trong tầm `rf_max_m`.
- **RTH:**
  - `rth_alt`, rally (`rally_max_m`), `rth_sanity_s`.
  - Hạ cánh: `land_*`.
- **Nhiệm vụ:** tối đa 120 điểm.
  - Soạn trên QGC / Mission Planner, hoặc tab Điều hướng.
  - Có quét khu vực zig-zag, nhập / xuất `.waypoints`.
  - Tham số: `wp_speed`, `wp_radius`, `nav_yaw_mode`.
- **Gió:** ước lượng và bù trước (`wind_comp`, `wind_drag`).
- **Geofence:**
  - Hình trụ: `fence_enable`, `fence_radius`, `fence_alt_max`, `fence_action`.
  - Đa giác: `fence_poly 1`, vẽ trên web / QGC, hoặc `fence add`.

---

## Motor & ESC

- **Giao thức:** `motor_protocol` (0 PWM, 1 OneShot125, 2 DShot300, 3 DShot600).
- **Bidirectional DShot:** `dshot_bidir 1`.
  - RPM đọc qua dây tín hiệu, RPM filter chạy không cần dây telemetry.
  - Đặt `motor_poles` (số cực motor, thường 14).
- **EDT:** `dshot_edt 1`. ESC AM32 / Bluejay / BLHeli_32 gửi nhiệt / áp / dòng qua dây tín hiệu.
- **Telemetry serial:** `esc_telemetry 1` + cổng chức năng 6.
- **Thứ tự motor:**
  - `motor1_out` … `motor4_out`: đầu ra nào điều khiển motor nào.
  - `yaw_reversed`: cánh ra ngoài (props out).
  - Trang có trình tìm thứ tự tự động.
- **Lệnh ESC (DShot):** bíp từng motor, đảo chiều từng motor. ESC PWM: hiệu chuẩn dải ga 5 bước.
- **ESC passthrough:** *Nhả cổng USB* → esc-configurator.com (BLHeli_S / Bluejay / AM32 / BLHeli_32).
- **CLI `esc`:** RPM, nhiệt, áp, dòng, trạng thái từng ESC.

---

## OSD, VTX, LED

- **Kính số (DJI O3 / O4 / O4 Pro, Walksnail, HDZero):**
  - Nối TX/RX của air unit chéo vào một UART; cổng đó chức năng 5 (DisplayPort), 115200.
  - `osd_backend 2`, `osd_canvas 0` (tự động: 53×20 DJI / Walksnail, theo kính với HDZero).
  - FC tự báo là Betaflight 4.5 để kính dùng font Betaflight và thoát chế độ công suất thấp khi arm.
  - Trên kính DJI: bật Custom OSD.
- **Analog:** chip MAX7456 / AT7456E. `osd_backend 1`; PAL / NTSC theo camera. `osd_font 1` = chỉ chữ (nếu chip chưa có font Betaflight).
- **Phần tử OSD:** kéo thả ở tab OSD.
  - Có: pin, dòng, mAh, chế độ, thời gian, độ cao, vario, khoảng cách / hướng về nhà, vệ tinh, tốc độ, link, tâm ngắm, chân trời, cảnh báo, toạ độ, nhiệt ESC, tổng quãng đường, mAh/km, thời gian bay còn lại.
  - `osd_stats`: giây hiện màn hình thống kê sau khi disarm.
- **VTX:** SmartAudio / Tramp (cổng chức năng 9 / 10) hoặc MSP.
  - `vtx_band`, `vtx_channel`, `vtx_power`, `vtx_pit`, `vtx_low_disarm`.
  - Bảng công suất riêng: `vtxt_*`.
- **LED WS2812:** `led_pin` (STM32 cố định PB8), `led_count`, `led_mode` (trạng thái / hướng / màu cố định), `led_brightness`.

---

## An toàn & failsafe

| Sự cố | Tham số | Hành động |
|---|---|---|
| Mất sóng | `fs_delay_ms`, `fs_action` (0 tắt motor, 1 hạ cánh, **2 RTH**, 3 SmartRTL), `fs_throttle` | không GPS → hạ cánh; không baro → ga cố định; sóng về 1 s → trả quyền |
| Pin yếu / cạn | `vbat_warn_cell`, `vbat_crit_cell`, `bat_capacity`, `bat_reserve_pct`, `bat_fs_action` | cảnh báo OSD / giọng nói, rồi hành động |
| Mất trạm mặt đất | `gcs_fs`, `gcs_fs_s` | RTH / hạ cánh / SmartRTL |
| Vượt rào | `fence_*` | hạ cánh / bay về |
| Va chạm | `crash_detect` | tắt motor |
| Xoay yaw mất kiểm soát | `yaw_spin_thr` | phục hồi |
| Rung | `vibe_warn` | OSD "VIBRATION" |
| IMU hỏng | `imu_dual 1` | IMU 2 thay thế |

- **Kiểm tra trước arm:** 18 mục, hiện ở tab Tổng quan và trên OSD ("Chưa thể arm").
- **Arm từ trạm mặt đất:** cần `gcs_arm 1`.
- **Configurator USB đang nối:** không arm được.

---

## Cổng, trạm mặt đất, WiFi

**Chức năng cổng** (`serial1_fn`…`serial5_fn`, `udp_fn`; baud `serialN_baud`):

| Mã | Chức năng | Mã | Chức năng |
|---|---|---|---|
| 0 | tắt | 6 | telemetry ESC (BLHeli32 / AM32 / KISS) |
| 1 | receiver (CRSF / SBUS / iBUS) | 7 | cảm biến khoảng cách TFmini |
| 2 | GPS | 8 | blackbox (OpenLog) |
| 3 | MAVLink | 9 | VTX SmartAudio |
| 4 | MSP (cả optical flow / rangefinder MSP) | 10 | VTX Tramp |
| 5 | MSP DisplayPort (OSD kính số) | | |

- **USB:** CLI, MSP, MAVLink và giao thức của trang cấu hình dùng chung một cổng (tự nhận).
- **WiFi:**
  - `wifi_enable 1`: MAVLink UDP 14550 cho QGC + trang cấu hình.
  - `web_enable 1`: chỉ trang cấu hình.
  - `wifi_armed 0` (mặc định): tắt WiFi khi bay.
- **CLI `wifi`** (chỉ qua USB): `wifi name <tên>`, `wifi pass <mật khẩu>`, `wifi webpass <mật khẩu|->`, `wifi reset`.
- **MAVLink:** nhiều drone thì đặt `mav_sysid` khác nhau.

---

## Hiệu chuẩn

| Việc | Cách |
|---|---|
| Gyro | `calib_gyro` (tự làm khi khởi động nếu đứng yên) |
| Accel | `calib_acc` (đặt phẳng) hoặc `calib_acc6` (6 mặt, mỗi mặt giữ ~1 s; QGC / Mission Planner cũng dùng được) |
| La bàn | `calib_mag 30` rồi xoay mọi hướng 30 s (khớp ellipsoid); `mag_declination` |
| compassmot | **lắp cánh, buộc chặt drone**, cần cảm biến dòng hoặc EDT: `compassmot go` |
| Hướng lắp | `imu_align`, `mag_align` (8 hướng), chỉnh tinh `board_roll/pitch/yaw`, cân bằng `level_trim_*` |

Kết quả (`acc_bias_*`, `mag_bias_*`, `mag_scale_*`, `mag_odiag_*`, `mag_cmot_*`) được ghi tự động.

---

## Logic conditions

8 luật, mỗi luật: `logic <n> <bật> <loại A> <giá trị A> <phép> <loại B> <giá trị B> <hành động> <tham số> <giá trị>`. Soạn trên web: tab Hệ thống.

| Trường | Giá trị |
|---|---|
| Loại toán hạng | 0 hằng số, 1 kênh RC (µs), 2 giá trị bay, 3 kết quả luật khác |
| Giá trị bay (loại 2) | 0 armed, 1 độ cao (dm), 2 tốc độ (cm/s), 3 cách nhà (m), 4 điện áp (cV), 5 % pin, 6 dòng (dA), 7 thời gian arm (s), 8 chế độ, 9 % link, 10 roll (°), 11 pitch (°), 12 số vệ tinh, 13 mAh |
| Phép | 0 luôn đúng, 1 =, 2 >, 3 <, 4 AND, 5 OR, 6 NOT, 7 ≥, 8 ≤, 9 ≠ |
| Hành động | 0 chỉ tính, 1 kêu còi, 2 bật chế độ (tham số = mã chế độ), 3 ghi đè kênh RC (tham số = kênh, giá trị = µs), 4 công suất VTX, 5 thông báo, 6 chặn arm |

Ví dụ: `logic 1 1 2 1 2 0 1200 1 0 0` = độ cao > 120 m → kêu còi.

---

## Lệnh CLI

| Lệnh | Làm gì |
|---|---|
| `help`, `version`, `status` | trợ giúp, phiên bản, trạng thái cảm biến / arm / pin / GPS / gió, thời gian chạy, lý do khởi động lại |
| `tasks`, `perf` | tải và thời gian từng task; thời gian từng khâu vòng bay nhanh |
| `get [lọc]`, `set <tên> <giá trị>`, `save`, `defaults` | tham số |
| `diff`, `dump` | sao lưu (chỉ phần đã đổi / tất cả) dạng lệnh CLI |
| `calib_acc`, `calib_acc6`, `calib_mag [s]`, `calib_gyro`, `compassmot` | hiệu chuẩn |
| `esc` | telemetry từng ESC |
| `mission [clear\|add …]`, `rally [clear\|add …]`, `fence [clear\|add <latE7> <lonE7>]` | nhiệm vụ, điểm rally, geofence đa giác |
| `goto <latE7> <lonE7> [alt]` | GUIDED: bay tới điểm |
| `logic [n …]` | logic conditions |
| `log [erase]` | log trong flash |
| `sd [ls \| rm <file> \| mount]` | thẻ SD |
| `wifi …`, `net` | WiFi (chỉ qua USB), kiểm tra kết nối WiFi |
| `scan` | kiểm tra dây: thiết bị I2C, mức SDA/SCL lúc khởi động (thiếu pull-up / chạm GND / module chưa cấp nguồn), mã chip ở các địa chỉ baro / compass, tốc độ I2C, mã IMU |
| `reboot`, `bootloader` | khởi động lại; vào chế độ nạp (ESP32-S3 ROM / STM32 DFU) |

---

## Sao lưu, cập nhật firmware, nâng cấu hình

- **Sao lưu:**
  - Hệ thống → *Tải file sao lưu*, hoặc `diff` / `dump`.
  - File là lệnh CLI: tham số, chế độ, cổng, hiệu chuẩn, nhiệm vụ, rally. Không chứa mật khẩu.
- **Khôi phục:** *Khôi phục từ file* ("được cả hoặc không gì"), hoặc dán vào CLI. Có *So sánh với file sao lưu* trước khi khôi phục.
- **Cập nhật qua WiFi (ESP32):**
  - Chọn `firmware.bin`; ghi vào ô OTA còn lại.
  - Không chạy bình thường trong 15 s đầu thì tự quay về bản cũ.
- **Nâng cấp giữ cấu hình:** tham số mới chỉ thêm vào cuối. Cấu hình cũ (v15 trở lên) tự chuyển sang **v25** với giá trị mặc định cho trường mới.

---

## Kiến trúc & hiệu năng

| Task | Tần số | Ưu tiên | Nhân (ESP32) | Việc |
|---|---|---|---|---|
| `fast` | 1–4 kHz (timer / ngắt IMU) | 6 | 1 | IMU → lọc → AHRS → PID → mixer → motor |
| `io` | 500 Hz | 5 | 0 | receiver, GPS, telemetry ESC, cảm biến khoảng cách |
| `control` | 250 Hz | 4 | 0 | EKF, arm, failsafe, chế độ, điều hướng |
| `sensors` | 100 Hz | 3 | 0 | baro, compass, pin, LED, còi |
| `telemetry` | 100 Hz | 2 | 0 | MAVLink, MSP, CLI, OSD, trang cấu hình qua USB |
| `logger` | `bb_rate_hz` | 1 | 0 | blackbox (không bao giờ chờ thiết bị lưu) |
| `sdcard` | 100 Hz | 1 | 0 | ghi thẻ SD (chỉ khi `sd_mode` bật) |
| `web` | 20 Hz | 1 | 0 | web server (ESP32, khi bật WiFi) |

- **Trao đổi dữ liệu:** các task dùng **topic** publish / subscribe, mỗi topic một task ghi.
- **Đo hiệu năng trên board:** `perf` / `tasks`, hoặc Hệ thống → Hiệu năng.
- **Vòng bay đã tối ưu:**
  - AHRS sửa sai ở ~1 kHz, góc Euler tính khi cần.
  - PID một phép chia mỗi vòng.
  - Blackbox ghi chênh lệch.
  - OSD / DisplayPort không chặn.

```
src/  os/ bus/ config/ common/ filter/ estimation/ control/ navigation/ safety/ sensors/ rx/ gps/
      comms/ (MAVLink, MSP, CLI, aqlink) osd/ logging/ (blackbox, sdlog) web/ (API, server, ui/) app/ hal/ sim/
test/host/  HAL giả lập + unit test        test/web/  bộ phân tích (Node), giao diện (Playwright)
tools/      build/test, giải mã blackbox, tạo docs/PARAMS.md, configurator, nạp / kiểm tra (aqflash, aqtest)
```

---

## Kiểm chứng

```bash
tools/run_tests.sh       # unit test + MAVLink + mô phỏng + WebAssembly + bộ phân tích + giao diện
tools/run_sim.sh tune    # một kịch bản mô phỏng
node tools/gen_params_doc.js   # cập nhật docs/PARAMS.md sau khi thêm tham số
```

| Hạng mục | Kết quả |
|---|---|
| Biên dịch PlatformIO | 9 / 9 board, 0 cảnh báo trong mã dự án |
| Unit test (kèm AddressSanitizer + UBSan) | **186 / 186** |
| Mô phỏng toàn hệ thống | 38 kịch bản đạt (EKF với cảm biến lỗi, RTH, nhiệm vụ, gió, optical flow, autotune, GPS Rescue, launch, bidir DShot, EDT…) |
| MAVLink đối chiếu pymavlink | 17 / 17 |
| Bộ phân tích blackbox | 18 / 18 (gồm: nén, log hỏng, khung ESC, phát hiện motor yếu 15 %, hội tụ khi chỉnh lặp) |
| Firmware WebAssembly | 5 / 5 |
| Giao diện (Playwright) | đạt toàn bộ: web, configurator USB + SITL, công cụ laptop, tính năng mở rộng, nạp firmware, thẻ SD |

**CI:** `.github/workflows/ci.yml` chạy unit test (cả sanitizer), mô phỏng, bộ phân tích và build 9 board cho mỗi lần push. File firmware giữ 14 ngày. Để dùng: đưa thư mục dự án lên GitHub (`git init`, `git add .`, `git commit`, `git push`).

**Chưa kiểm chứng trên phần cứng:** DShot / bidir trên chip thật, RMT, OSD MAX7456, khởi tạo từng cảm biến, thẻ SD trên chip thật, các board F405 / F722 / C6 / P4. Khi thử trên bàn, gửi kèm kết quả `status`, `perf`, `tasks`, `sd`, `esc` để kiểm tra.

---

## Hướng phát triển

- **Sau khi có log bay thật:** chỉnh mặc định PID / bộ lọc / kích thước hàng đợi thẻ SD theo số đo.
- **Ghi gyro thô tần số cao** (2–4 kHz, từng đợt vài giây) lên thẻ SD để phân tích phổ sâu.
- **EKF nhiều IMU:** chọn IMU tốt nhất.
- **WiFi cho ESP32-P4** qua ESP32-C6 (esp-hosted).
- **Bản đồ nền offline** (tải sẵn tile).
- **Ký số firmware OTA.**
