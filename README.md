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
- [`docs/PARAMS.md`](docs/PARAMS.md): **toàn bộ 381 tham số** (mặc định, giới hạn, mô tả), tạo tự động.
- [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md): task, topic, đường dữ liệu.
- [`docs/HARDWARE_TEST.md`](docs/HARDWARE_TEST.md): danh sách kiểm tra trên phần cứng thật sau mỗi lần nạp firmware (bàn → cắm pin → bay).

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
| `stm32f405` / `stm32f722` | FC F405 / F722 | 1–2 kHz | 6 UART (SERIAL1–6) |
| `stm32h743_weact` | **WeAct MiniSTM32H743** | **3,2 kHz** (tối đa 4) | khe **microSD có sẵn** (bật mặc định), DShot600 + bidir + EDT |
| `stm32h743_devebox` | **MCUDev DevEBox STM32H743VIT6** | **3,2 kHz** | như WeAct; IMU INT1 → PC4, I2C2 PB11/PB10, RX trên PA9/PA10 = SERIAL4; 7 UART (SERIAL1–7) |
| `stm32f405_qemu` | (máy ảo QEMU) | – | chỉ để kiểm tra khởi động: `tools/qemu_f405.sh` |

| Loại | Thiết bị (tự nhận diện) |
|---|---|
| IMU | BMI088, **ICM-45686**, ICM-45605, ICM-42688-P, ICM-42605, LSM6DSO / DSR / DSL, ISM330DHCX, MPU6000/6500/9250, ICM-20602/20689 (SPI); MPU6050 (I2C) |
| Baro | DPS310 / SPL06, **BMP388**, BMP390, BMP280, BME280, MS5611 |
| Compass | QMC5883L, **QMC5883P**, HMC5883L, IST8310, LIS3MDL |
| GPS | u-blox M8/M9/M10 (UBX NAV-PVT, tự cấu hình 10 Hz) + NMEA |
| Receiver | **CRSF / ExpressLRS** (kèm telemetry ngược), SBUS, iBUS |
| ESC | DShot300/600 (lệnh đặc biệt), **bidirectional DShot**, **EDT** (nhiệt / áp / dòng qua dây tín hiệu), PWM, OneShot125; telemetry serial BLHeli32 / AM32 / KISS |
| Khác | TFmini / TF-Luna / TF02, optical flow MSP (3901-L0X, MicoAir MTF-01 / 01P / 02: MSP hoặc Micolink), VTX SmartAudio / Tramp / MSP, LED WS2812, **thẻ microSD** |

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
- Mỗi tham số hiện tên kỹ thuật (`roll_p`…), giới hạn và giải thích; các thông số hay chỉnh (P / I / D / F, lọc, idle, rates, failsafe…) có thêm dòng **↑ tăng thì… · ↓ giảm thì…**.
- **Thuật ngữ:** nút **?** cạnh tiêu đề có thuật ngữ (PID, notch, RPM filter, failsafe, EKF…) mở giải thích ngắn; tìm "thuật ngữ" (Ctrl+K) mở cả danh sách ~40 mục.
- Chế độ **Đơn giản** còn gập tab Tham số và CLI dưới nhãn *Công cụ* (bấm nhãn để hiện lại); tìm kiếm và liên kết vẫn mở được.
- **Giao diện buồng lái (mặc định, tối):** nền xám than, màu xanh lơ chỉ cho nút bấm và mục đang chọn; xanh lá / vàng / đỏ chỉ dùng cho trạng thái (ARMED là nhãn đỏ nhấp nháy). Nút mặt trời trên thanh trên cùng đổi giữa *Buồng lái* / *Sáng* / *Theo máy* (theo cài đặt hệ điều hành) / *Ngoài trời* (tương phản tối đa dưới nắng).
- **Bố cục gọn:** Tổng quan có một dải trạng thái lớn (Sẵn sàng / bị chặn vì…, kèm "Cấu hình còn thiếu"), một hàng 6 chỉ số, rồi tư thế, cảm biến, tay điều khiển, motor. Tiêu đề thẻ có biểu tượng; đoạn giải thích dài thu còn 1–2 dòng (bấm để đọc hết); mỗi cổng serial có biểu tượng theo chức năng, cổng không dùng mờ đi; thông báo hiện ở đáy màn hình. Trên điện thoại menu nằm dưới đáy, nhãn ngắn một dòng.
- **English:** nút 🌐 VI / EN trên thanh trên cùng (và ở màn hình kết nối / đăng nhập): dịch ~1 350 câu chữ của giao diện, giữ nguyên số đo; bật lại VI là về nguyên văn. Câu chưa có trong bảng vẫn hiện tiếng Việt; thêm câu: `node tools/i18n/collect.js` rồi bổ sung `tools/i18n/en.json`.

| Tab | Chức năng chính | Trường / tham số quan trọng |
|---|---|---|
| **Tổng quan** | Khung sẵn sàng arm (xanh = sẵn sàng, đỏ = liệt kê từng lý do kèm cách sửa, bấm để tới đúng tab; đang nối trang cấu hình chỉ là thông tin), thẻ **Cấu hình còn thiếu** (hiệu chuẩn accel / compass, công tắc ARM, receiver, failsafe cần GPS, dung lượng pin, blackbox… kèm nút tới đúng chỗ; gợi ý ẩn được), ô trạng thái (pin, tín hiệu, GPS, chế độ, độ cao, thời gian bay còn lại, tải CPU), chân trời nhân tạo hoặc mô hình 3D (nút *Chân trời / 3D*, bấm mô hình để nhìn lại từ phía sau), gyro so với setpoint, motor (chưa arm: giao thức, RPM / nhiệt ESC, nút thử motor; arm: thanh ga), kênh RC, danh sách tình trạng cảm biến có đèn trạng thái, cảnh báo rung / EKF | `bat_capacity` (để tính thời gian còn lại) |
| **Trước bay** | Danh sách kiểm tra trước khi bay: tự kiểm tra (kết nối, IMU, accel / compass đã hiệu chuẩn, baro, tay điều khiển + số lần mất sóng, công tắc ARM, failsafe, pin / V-cell, GPS khi dùng chế độ cần GPS, điều kiện arm, cấu hình đã lưu, chỗ trống blackbox, rung) và 7 mục kiểm tra bằng mắt (tự bỏ tích sau 12 giờ); điểm tổng và nút *Sửa* tới đúng tab. **Hướng dẫn bay lần đầu**: 9 bước theo thứ tự (đấu dây, cảm biến, hướng lắp FC, tay điều khiển, ARM / chế độ, đo pin, motor, failsafe, điều kiện arm), bước đang làm có lý do vì sao; **thử nghiêng** (nghiêng phải, ngóc mũi: trang nhận ra FC lắp lệch 90° / ngược và chỉ `imu_align`) và **thử failsafe** (tắt tay điều khiển: FC phải báo mất sóng; receiver vẫn gửi kênh sau 20 s thì báo đặt "No pulses") chạy trực tiếp | `imu_align`, `fs_action` |
| **Thiết lập** | Trình thiết lập 9 bước, kiểm tra bằng số liệu thật; bước hướng lắp FC có mô hình 3D chạy theo tư thế đo được | `imu_align`, `board_roll/pitch/yaw` |
| **PID & lọc** | **Trợ lý chỉnh PID** (chọn khung → bài bay thử → phân tích blackbox → đưa kế hoạch vào hàng chờ, lặp lại; bảng vọt lố / nhiễu từng vòng), bảng P/I/D/F 3 profile, thanh trượt chỉnh nhanh (độ mạnh chung, giảm chấn, feedforward, I); cảm giác bay & bảo vệ; rates kèm đồ thị; bộ lọc kèm đồ thị đáp ứng tần số & độ trễ; **trạng thái RPM filter** bằng lời (đang chạy 4/4 ESC, thiếu ESC, chưa có nguồn RPM, tắt) | `roll_p…yaw_f`, `p2_*`, `p3_*`, `*_rc_rate`, `*_max_rate`, `*_expo`, `gyro_lpf_hz`, `dterm_lpf_hz`, `dyn_notch_*`, `rpm_harmonics`, `rpm_w1..3` |
| **Chế độ & an toàn** | **Tín hiệu tay điều khiển**: LQ 2 phút gần nhất, trung bình / thấp nhất, số lần và thời gian mất sóng dài nhất; gán chế độ cho khoảng kênh AUX (kéo, Thấp / Giữa / Cao), xem công tắc trực tiếp; failsafe, pin, geofence | `mrN_mode/ch/lo/hi`, `fs_action`, `bat_fs_action`, `fence_*` |
| **Điều hướng** | Bản đồ (nhà, drone, vệt bay, waypoint, rally, geofence, gió; bản đồ nền OSM); **soạn nhiệm vụ** (chạm thêm, kéo dời), **quét khu vực**, **geofence đa giác**, **bay tới điểm chạm** (GUIDED), nhập / xuất `.waypoints`; **thống kê nhiệm vụ** (quãng đường, thời gian, xa nhà nhất, cao nhất, mAh ước tính theo dòng đo được, cảnh báo khi > 70 % pin); bảng EKF | `rth_alt`, `wp_speed`, `nav_*`, `fence_poly`, `gcs_fs` |
| **Tay điều khiển** | Thanh từng kênh (4 cần + AUX, kênh vừa gạt được tô sáng), **kiểm tra hành trình** (min / max đã đi, tâm 1500 µs, gợi ý chỉnh endpoint / subtrim trên tay), **thứ tự kênh** (AETR / TAER / AERT / RETA hoặc **tự dò**: lần lượt đẩy ga, roll, pitch, yaw, kèm báo kênh bị đảo chiều), vùng chết quanh tâm cần | `rc_ch_roll/pitch/thr/yaw`, `rc_deadband`, `rc_smooth_hz` |
| **Cảm biến** | Mô hình 3D, đồ thị gyro / accel / compass / độ cao, cảm biến khoảng cách, optical flow, RPM + nhiệt ESC; hiệu chuẩn & hướng lắp; **kiểm tra optical flow** (cầm drone trên nền có hoa văn, nghiêng qua lại: trang so flow với gyro, tìm `flow_align` / `flow_flip` đúng, báo biên độ, nút Áp dụng; module chỉ đo khoảng cách thì nói rõ) | `acc_*`, `mag_*`, `level_trim_*` |
| **Motor & ESC** | **Thử motor** (khoá đến khi tích "đã tháo cánh"), đo rung từng motor, kiểm tra phản ứng (không cần arm), thứ tự & chiều quay (tìm tự động), giao thức ESC, bíp / đảo chiều từng motor, hiệu chuẩn ESC PWM, **ESC passthrough** (esc-configurator.com) | `motor_protocol`, `dshot_bidir`, `dshot_edt`, `motor_poles`, `yaw_reversed`, `motorN_out` |
| **Cổng & thiết bị** | Chức năng + baud từng UART (cảnh báo trùng), **lưu lượng & lỗi từng UART** (byte/s, tràn, lỗi khung, nhiễu; khung receiver/s + % sai checksum, kèm gợi ý nguyên nhân), chức năng **Configurator** cho cầu WiFi, VTX (băng / kênh / công suất / pit, bảng VTX riêng), dải LED | `serialN_fn`, `serialN_baud`, `vtx_*`, `vtxt_*`, `led_*` |
| **OSD** | Kéo thả phần tử (thêm thanh pin, V/cell, RSSI dBm), nhấp nháy cảnh báo + pin yếu (`osd_blink`), bật / tắt, analog hay kính số, lưới chữ, icon hay chữ, xem trước HD, hướng dẫn DJI / Walksnail / HDZero | `osd_backend`, `osd_canvas`, `osd_font`, `osd_stats`, `osd_*_pos` |
| **Blackbox** | **Tóm tắt chuyến bay** (thời gian, pin đầu → cuối và V/cell thấp nhất, dòng max / TB, mAh, ga lơ lửng, độ cao / xa nhất, rung, nhiệt ESC, kèm nhận xét: pin bị ép, sụt áp, drone nặng, motor chạm 100 %, rung cao…), tải log từ drone hoặc mở file, **thẻ SD** (trạng thái, danh sách, phân tích / lưu / xoá từng file), phân tích, **kế hoạch chỉnh**, bảng ESC, ghim chuyến mốc để so sánh, xuất CSV, **bản đồ nhiệt nhiễu theo mức ga** (vệt chéo = nhiễu motor, vệt đứng = cộng hưởng khung, có vạch cắt lọc), **sự kiện chuyến bay** (arm / disarm, đổi chế độ, motor chạm 100 %, điện áp thấp nhất, tốc độ quay lớn nhất; bấm để nhảy tới đoạn đó), **đường bay nhìn từ trên** (màu theo độ cao, chấm đi theo thanh thời gian), so sánh với chuyến mốc cả **phổ nhiễu** (đường đứt nét) | `bb_rate_hz`, `bb_backend`, `sd_*` |
| **Hệ thống: gói chẩn đoán** | Một nút gom selftest, status, cấu hình đã đổi (`diff`, không mật khẩu), sơ đồ chân, task / hiệu năng, ESC, thẻ SD, lưu lượng cổng vào một file `.txt` (hoặc sao chép): gửi file này khi nhờ hỗ trợ | — |
| **Tham số** | **Hồ sơ drone** (lưu cấu hình theo tên trong trình duyệt, nạp vào hàng chờ, xuất / nhập file `.json`; không chép hiệu chuẩn cảm biến, WiFi, cổng), preset (5 inch, 3 inch, cinewhoop, 7 inch, bộ lọc, rates, an toàn GPS), mọi tham số theo nhóm, tìm kiếm, lịch sử thay đổi + hoàn tác + CSV | — |
| **CLI** | Dòng lệnh như Betaflight, nhớ lệnh, Tab tự hoàn thành, nút nhanh | — |
| **Hệ thống** | **Khởi động & sự cố** (lý do reset, bản ghi crash STM32 kèm PC / LR, watchdog), cài đặt cầu WiFi (khi nối qua cầu), Lưu / khởi động lại / mặc định / sáng-tối, sao lưu & khôi phục, so sánh với file sao lưu, nạp firmware USB, cập nhật WiFi (OTA), WiFi & bảo mật, **cài đặt blackbox + thẻ SD**, cảnh báo giọng nói, logic conditions, bảng task và thời gian từng khâu vòng bay | `wifi_enable`, `web_enable`, `wifi_armed`, `mav_sysid` |

**Thao tác:**
- Tab dài chia thành **trang**: thanh nút dưới đầu trang (ví dụ PID · Cảm giác bay · Rates · Bộ lọc · Tất cả) chỉ hiện một phần; trang nhớ phần đang mở. Tìm kiếm (Ctrl K) và các liên kết tự mở đúng phần.
- **Lưu một bước**: sửa xong bấm **Lưu** (áp dụng + ghi flash) hoặc Ctrl+S; *Chỉ áp dụng* để thử trước (mất khi tắt nguồn). Tham số cần khởi động lại thì sau khi lưu có nút **Khởi động lại FC**.
- Tham số dạng bit (trục / bước autotune, lật cảm biến quang) là ô tích; kênh chọn profile là danh sách AUX.
- Điện thoại: thanh dưới giữ Tổng quan, Thiết lập, PID & lọc, Chế độ; các tab khác ở nút **Thêm**.
- Màn hình kết nối: nút *Kết nối lại* ghi tên thiết bị (STM32, ESP32 CP210x…); tốc độ baud và mẹo nằm trong *Không kết nối được?*.
- Nạp firmware: kéo thả file vào hộp thoại; trang tự nhận ra ESP32 hay STM32 từ file, đọc tên board trong file và cảnh báo nếu khác board FC đang nối.
- Tab Tham số: ô tìm kiếm và nhóm tham số đứng đầu; preset + hồ sơ, lịch sử + so sánh ở các trang riêng.

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

**Cầu WiFi cho STM32** (không cần cáp, cả iPhone): một ESP32-C3 SuperMini (~2 USD) nạp `pio run -e bridge_esp32c3 -t upload`. Nối GPIO21 (TX) → RX một UART của FC, GPIO20 (RX) ← TX, GND chung, nguồn 5 V. Trên FC đặt cổng đó chức năng **Configurator** (460800 baud). Điện thoại vào WiFi `AllQuad-Link-xxxx` (mật khẩu `allquad123`, đổi ngay ở tab Hệ thống), trang tự mở (hoặc vào `192.168.4.1`) và tự nối với FC. USB và cầu dùng chung console: bên nào nói trước thì được phục vụ, bên kia đợi 1,5 s im lặng. Nạp firmware FC và ESC passthrough vẫn qua USB. Cầu chưa thử trên phần cứng.

Trang cấu hình cho STM32 là một file (`build/web/AllQuad-Configurator.html`, gồm cả drone ảo). Web Serial / WebUSB chỉ chạy trên trang **https** (hoặc file mở trực tiếp trên máy tính), nên với điện thoại hãy đặt file này lên một chỗ https một lần (GitHub Pages, Netlify…, đổi tên thành `index.html`), rồi mở link đó bằng Chrome và *Thêm vào màn hình chính*. Trang tự nhận ra mình là bản USB (không đi tìm drone qua WiFi).

**Cài làm ứng dụng (PWA, bản GitHub Pages):** `tools/make_pages.py` thêm vào bản Pages manifest, biểu tượng, `sw.js` và `version.json`. Mở `https://<tên>.github.io/AllQuad/` bằng Chrome/Edge → nút *Cài làm ứng dụng* ở màn hình kết nối (điện thoại: *Thêm vào màn hình chính*).
- **Không có mạng:** app mở bản đã lưu lần trước (kết nối USB, phân tích blackbox, drone ảo đều chạy).
- **Có mạng:** mỗi lần mở, khi quay lại app, và 15 phút một lần, app hỏi `version.json` trên GitHub Pages. Có bản mới → tải ngay nếu chưa nối FC; đang nối FC thì hiện thanh *Có bản mới* và tự cập nhật khi ngắt kết nối. Push lên `main` (đã chạy `make_pages.py`) là app ở mọi máy tự lên bản đó.
- Bản trang nằm trong firmware (ESP32 qua WiFi) không cài làm app.

## Cài đặt lần đầu

**THÁO CÁNH QUẠT.** Dùng tab **Thiết lập** rồi **Trước bay → Hướng dẫn bay lần đầu**, hoặc làm tay qua CLI:

1. `status` kiểm tra cảm biến; `scan` để kiểm tra dây:
   - Liệt kê thiết bị I2C, mức SDA/SCL, mã chip và mã IMU.
   - Baro / compass cắm muộn (ví dụ module ăn nguồn từ pin khi board chạy USB) được tự tìm lại mỗi 2 s khi disarm.
   - Không thấy gì ở 400 kHz thì FC tự thử 100 kHz.
2. Hướng lắp: nghiêng phải → roll dương, ngóc mũi → pitch dương. Sai thì `set imu_align ...` hoặc dùng bước "tìm hướng lắp" của trình thiết lập.
3. `calib_acc` (hoặc `calib_acc6`), `calib_mag 30`, `set mag_declination <độ>`, `save`.
4. Gán cổng: `set serial1_fn 1` (receiver), `set serial2_fn 2` (GPS)… (mã ở [Cổng](#cổng-trạm-mặt-đất-wifi)). `save` → `reboot`.
5. `set rx_protocol 0` (CRSF; 1 SBUS, 2 iBUS); `set motor_protocol 2` (DShot300; 3 DShot600).
6. Pin: tab **Chế độ & an toàn → "Đo điện áp & dòng: cách nối, hiệu chuẩn"** (xem [Đo điện áp & dòng](#đo-điện-áp--dòng)).
   - Đặt `bat_capacity` (mAh) để có thời gian bay còn lại và failsafe theo dung lượng.
7. Thử motor ở tab Motor: thứ tự và chiều quay. Sai thứ tự: `motorN_out` (không cần đổi dây). Sai chiều: đảo chiều trong ESC (DShot).
8. Thử failsafe: arm trên bàn không cánh, tắt tay điều khiển, motor phải dừng hoặc vào chế độ đã chọn.

**Bay lần đầu** (cấu hình v29, tab PID → "Giới hạn & bay lần đầu"): `set beginner 1`.
- Luôn tự cân bằng (ANGLE), kể cả khi công tắc đang ở ACRO / HORIZON.
- Nghiêng tối đa 25°, quay tối đa 250 °/s (yaw 120 °/s), ga tối đa 75 %, leo tối đa 1,5 m/s, bay vị trí 3 m/s.
- Rào ảo bật tự động: bán kính `beg_radius` (mặc định 60 m), cao 40 m; ra khỏi rào thì bay về (RTH).
- Chỉ siết chặt hơn, không nới: thông số đã hiền hơn thì giữ nguyên. Cấu hình lưu không bị đổi, tắt `beginner` là về như cũ.
- OSD hiện "BEGINNER MODE" khi chưa arm, web hiện thẻ cảnh báo, để không quên đang bật.

**Giới hạn ga / motor:**
- `thr_limit_pct` (25–100): cần ga 100 % chỉ ra từng ấy % ga ở ACRO / ANGLE / HORIZON. Hợp cho motor quá mạnh, pin 6S trên khung 4S…
- `motor_limit_pct` (50–100): trần đầu ra mỗi motor, áp cả lúc bộ trộn cần thêm (yaw, giữ thăng bằng). Giảm nhiệt motor / ESC.

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
- **Gán nhanh:** nút **Dò** trên mỗi dòng chế độ: gạt công tắc trên tay trong 10 s, trang tự chọn đúng kênh AUX.
- **Thứ tự kênh** khác AETR (ví dụ TAER của receiver cũ): `rc_ch_roll`, `rc_ch_pitch`, `rc_ch_thr`, `rc_ch_yaw` (cấu hình v30), hoặc tab Tay điều khiển → *Tự dò bằng cách gạt cần*. Kiểm tra tháo cánh trước khi bay.
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

**Feedforward nâng cao** (cấu hình v27, tab PID):
- `ff_jitter` (0–20, mặc định 7): gạt cần chậm / tay run thì FF giảm theo độ lớn chuyển động cần, cú gạt thật vẫn đủ FF. 0 = tắt.
- `ff_boost` (0–50, mặc định 15): cộng thêm theo gia tốc cần, FF vào sớm hơn khi gạt mạnh.
- `ff_max_pct` (0–100, mặc định 90): từ % này của tốc độ quay tối đa, FF giảm dần về 0 (đỡ vọt khi đánh cần hết cỡ). 0 = tắt.

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

**Xuất log cho công cụ khác** (tab Blackbox, sau khi mở log): *Xuất .bbl* mở được bằng Betaflight Blackbox Explorer và PIDtoolbox (định dạng Betaflight, đơn vị như Betaflight 4.4; đã kiểm bằng bộ đọc của Blackbox Explorer). *Xuất CSV (Betaflight)* có cột giống `blackbox_decode` (`gyroADC[0]`, `setpoint[0]`, `motor[0]`…) cộng tư thế, độ cao, chế độ.

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
- **0xA8 (v2.2):** khung gyro tần số cao, 16 mẫu gyro thô + sau lọc (tới 4 kHz) mỗi khung; chỉ trên thẻ SD.
  - `bb_gyro_s` (giây, 0 = tắt): ghi một đợt sau mỗi lần cất cánh (lần đầu ga > 25 %); CLI `gyrocap <s>` ghi ngay khi đang bay.
  - Log thường (≤ 1 kHz) chỉ thấy nhiễu dưới 500 Hz; đợt này thấy tới 2 kHz: tab Blackbox → *Gyro tần số cao* (phổ thô / sau lọc, đỉnh nhiễu và độ suy giảm của bộ lọc).
  - Không có ở F411 (thiếu RAM cho bộ đệm 12 kB).
- **0xA9 (v2.2):** khung điều hướng 41 byte, ~10 Hz: bộ ước lượng nào đang chạy, cờ lỗi EKF, tỉ số kiểm định từng cảm biến, tư thế EKF và hướng AHRS, độ lệch chuẩn, số vệ tinh và sai số GPS, bộ đếm đặt lại (vị trí, vận tốc, cả bộ lọc, tư thế, hướng, độ cao baro).
  - Tab Blackbox → *Tóm tắt chuyến bay* tự đọc: baro bị loại bao lâu, la bàn lệch GPS, GPS nhảy, EKF phải tự đặt lại tư thế / hướng mấy lần.
- **Giải mã trên máy tính:** `python3 tools/blackbox_decode.py LOG.BBL` → `LOG.csv` (bản ghi) + `LOG_esc.csv` (RPM / nhiệt) + `LOG_nav.csv` (sức khoẻ bộ ước lượng).

---

## GPS, EKF & điều hướng

- **GPS:**
  - Cổng chức năng 2. FC tự chuyển u-blox sang 115200 baud, 10 Hz, NAV-PVT, airborne.
  - `gps_min_sats`.
  - `gps_max_hacc`: loại fix sai số lớn.
  - **Chống nhảy:** `gps_glitch_m` / `gps_glitch_s`.
- **EKF:**
  - `est_type 1`: EKF 15 trạng thái. GPS nhảy → nhận vị trí mới; baro lệch 5 s → độ cao từ GPS; compass sai 5 s → hướng từ GPS. Web hiện tỉ số kiểm định từng cảm biến.
  - Tự sửa khi chính EKF sai (v2.2):
    - Tư thế EKF lệch AHRS quá 15° trong 1 s (sau lộn nhào, va chạm) → đặt lại tư thế theo AHRS và học lại bias gia tốc kế. Khi treo yên thì ngưỡng 4°, và EKF được kéo nhẹ về tư thế AHRS (lúc treo yên, lỗi nghiêng và bias gia tốc kế không tách được bằng GPS).
    - Trọng tài GPS cho hướng: hướng học từ gia tốc GPS (độc lập với la bàn và EKF). La bàn khớp nó mà EKF lệch quá 30° trong 2 s → hướng EKF đặt lại theo la bàn, la bàn không bị đổ lỗi oan.
    - Delta góc / vận tốc từ vòng bay được cộng dồn kèm bù coning / sculling, không mất mẫu ở bất kỳ tần số vòng bay nào.
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
- **Bản đồ offline** (tab Điều hướng → "Bản đồ nền", "Offline…"):
  - Ô bản đồ OpenStreetMap đã xem được lưu trong trình duyệt (tối đa khoảng 6000 ô, ô cũ nhất xoá trước). Xem trước khu bay khi ở nhà có mạng, ra bãi không có mạng vẫn có bản đồ.
  - Nhập ZIP ô dạng `z/x/y.png|jpg` (MOBAC, QGIS "Generate XYZ tiles"). Thiếu mức zoom thì dùng ô zoom thấp hơn phóng to.
  - Bộ nhớ riêng theo từng địa chỉ: trang trên GitHub Pages, file HTML và trang trong drone ESP32 (192.168.4.1) mỗi nơi giữ bản đồ riêng.

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
  - Trang có trình tìm thứ tự tự động (quay từng đầu ra, bạn bấm vị trí; hiện RPM nếu ESC báo).
- **Kiểm tra trên bàn bằng RPM** (tab Motor & ESC, tháo cánh): quay lần lượt M1–M4 ở ga 6 %, đọc RPM cả 4 ESC. Báo ESC không trả telemetry, motor không quay, hoặc motor khác cũng có RPM (đầu ra / dây telemetry lẫn).
- **Tab Tổng quan:** khi chưa arm, ô Motor hiện giao thức, RPM / nhiệt từng ESC theo vị trí và nút *Thử motor*; khi arm (hoặc đang thử) hiện thanh ga.
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

### Đo điện áp & dòng

Chân ADC đo tối đa **3,3 V**. Công thức: điện áp = V ở chân × `vbat_scale`; dòng = (V ở chân dòng − `cur_offset`) × `cur_scale` (A/V). Chân của từng board: tab Cổng & thiết bị → Sơ đồ chân (DevEBox H743: VBAT **PC0**, CURRENT **PC1**).

| Trường hợp | Đấu dây | Hệ số |
|---|---|---|
| **Board phát triển** (DevEBox, WeAct, BlackPill, ESP32 DevKit): không có cầu chia | Pin (+) → R1 → điểm giữa → chân VBAT; điểm giữa → R2 → GND; tụ 100 nF chân–GND; (tuỳ chọn) 1 kΩ nối tiếp + zener 3,3 V | `vbat_scale` = (R1 + R2) / R2. 10 kΩ / 1 kΩ = 11 (đến 6S, chân 2,4 V khi pin đầy). ESP32 / 8S: 20 kΩ / 1,5 kΩ = 14,3 (giữ chân ≤ 2,5 V vì ADC ESP32 không tuyến tính phía trên) |
| **FC thương mại** (cầu chia có sẵn) | Pad VBAT của FC ← pin (+) hoặc chân VBAT của ESC 4-in-1 | Betaflight `vbat_scale` ÷ 10 (110 → 11) |
| **Power module / PDB analog** | V → chân VBAT, I → chân CURRENT, GND chung (loại I2C / CAN không dùng được) | theo datasheet |
| **Không nối chân** | `dshot_bidir 1` + `dshot_edt 1` (AM32 / Bluejay / BLHeli_32) hoặc dây telemetry ESC | FC tự dùng áp / dòng ESC báo khi chân VBAT không có điện áp |
| **Dòng (CURR của ESC / module)** | CURR → chân CURRENT, nối thẳng (đã là 0–3,3 V) | `cur_scale` = 1000 / (mV/A); từ Betaflight: `cur_scale` = 10000 / `ibata_scale` (400 → 25 A/V), `cur_offset` = −`ibata_offset`(mA) / 1000 / `cur_scale` |

**Hiệu chỉnh cho chuẩn** (trang có sẵn nút, kết quả vào hàng chờ → Lưu):
- Điện áp: đo cọc pin bằng đồng hồ, nhập số → `vbat_scale` = V đồng hồ ÷ V ở chân.
- Điểm 0 dòng: pin cắm, motor dừng → `cur_offset` = V ở chân dòng lúc đó.
- Hệ số dòng: bay hết một pin, so mAh FC báo với mAh bộ sạc nạp vào → `cur_scale` × (mAh sạc ÷ mAh FC báo). Làm 2–3 lần.
- `selftest` báo lỗi khi chân đo áp gần 3,3 V (mạch chia quá yếu: điện áp hiển thị sai, có thể hỏng chân) và khi đang disarm mà vẫn báo trên 1,5 A (chưa đặt điểm 0). STM32 lấy trung bình 8 lần đo mỗi lượt để bớt nhiễu motor.

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
| Gia tốc kế bị kịch (> ±15,5 g) | – | OSD "ACC CLIPPING", web, blackbox (cờ bit 8): rung quá mạnh, ước lượng độ cao / vị trí sai. Kiểm tra cách gắn FC, cánh, motor |
| IMU hỏng | `imu_dual 1` | IMU 2 thay thế |
| Hai IMU, một chip kém (khi chưa arm) | `imu_dual 1` | so độ nhiễu gyro và trọng lực đo được (không phụ thuộc hướng lắp): chip chính nhiễu gấp 3 hoặc sai trọng lực > 2,5 m/s² thì bay bằng chip 2 (không chặn arm); hai chip lệch trọng lực > 1,5 m/s² mà không rõ chip nào sai thì chặn arm `IMU_MISMATCH` |

- **Kiểm tra trước arm:** 18 mục, hiện ở tab Tổng quan và trên OSD ("Chưa thể arm").
- **Arm từ trạm mặt đất:** cần `gcs_arm 1`.
- **Configurator USB đang nối:** không arm được.

---

## Cổng, trạm mặt đất, WiFi

**Chức năng cổng** (`serial1_fn`…`serial7_fn`, `udp_fn`; baud `serialN_baud`; SERIAL6/7 từ cấu hình v28):

| Mã | Chức năng | Mã | Chức năng |
|---|---|---|---|
| 0 | tắt | 6 | telemetry ESC (BLHeli32 / AM32 / KISS) |
| 1 | receiver (CRSF / SBUS / iBUS) | 7 | cảm biến khoảng cách TFmini |
| 2 | GPS | 8 | blackbox (OpenLog) |
| 3 | MAVLink | 9 | VTX SmartAudio |
| 4 | MSP (cả optical flow / rangefinder MSP) | 10 | VTX Tramp |
| 5 | MSP DisplayPort (OSD kính số) | 11 | configurator (cầu WiFi ESP32-C3 / Bluetooth) |
| | | 12 | MAVLink tốc độ thấp (LoRa / radio chậm, cấu hình v29) |

**Cổng serial theo board** (tab Cổng hiện UART và chân dưới tên từng cổng):

| Cổng | H743 WeAct / DevEBox | F405 / F722 | F411 BlackPill |
|---|---|---|---|
| SERIAL1 | USART3 RX PD9 / TX PD8 | USART2 PA3 / PA2 | USART2 PA3 / PA2 |
| SERIAL2 | UART7 PE7 / PE8 | USART1 PA10 / PA9 | USART1 PA10 / PA9 |
| SERIAL3 | UART8 PE0 / PE1 | UART4 PC11 / PC10 | – |
| SERIAL4 | USART1 PA10 / PA9 | UART5 PD2 / PC12 | – |
| SERIAL5 | USART2 PD6 / PD5 | USART6 PC7 / PC6 | – |
| SERIAL6 | UART4 PD0 / PD1 | USART3 PB11 / PB10 | – |
| SERIAL7 | USART6 PC7 / PC6 | – | – |

- H743: UART5 không dùng (chân của nó là thẻ SD, SPI2 của IMU hoặc flash QSPI). SERIAL3 và SERIAL7 nằm trên header camera: không cắm camera.
- Chống trùng chân: `tools/check_pins.py` (chạy trong `run_tests.sh` và CI) kiểm mọi board: không chân nào dùng hai lần, không đụng chân board / chip giữ riêng (`AQ_PINS_RESERVED`: USB, SWD, thẻ SD, flash, PSRAM…), và mỗi chân RX / TX đúng là chân của UART đó với đúng alternate function.
- `selftest` báo cổng được gán chức năng nhưng board không có, và chức năng dùng một lần (receiver, GPS, ESC, VTX…) gán cho hai cổng (chỉ cổng đầu được dùng). ESP32: `led_pin` trùng chân board / chip thì dải LED không bật.

- **USB:** CLI, MSP, MAVLink và giao thức của trang cấu hình dùng chung một cổng (tự nhận).
- **Optical flow + đo khoảng cách (MicoAir MTF-01 / 01P / 02, Matek 3901-L0X):** cổng chức năng 4 (MSP), 115200.
  - Nối TX của module vào RX của cổng (đấu chéo), 5 V, GND chung.
  - Firmware nhận cả **MSP** (chế độ iNav) lẫn **Micolink** (giao thức riêng của MicoAir), không cần đổi giao thức trong MicoAssistant nếu module đang ở một trong hai.
  - Module để chế độ MAVLink (bản cho ArduPilot / PX4) thì không dùng được: `selftest` (dòng `flow`) và bảng lưu lượng ở tab Cổng báo "đang nhận MAVLink". Dùng MicoAssistant + cáp USB-TTL chuyển sang MSP hoặc Micolink.
  - Kiểm tra chiều trước khi bay (`flow_flip`, `flow_align`): xem [`docs/HARDWARE.md`](docs/HARDWARE.md).
- **Sơ đồ chân (GPIO)** (tab Cổng & thiết bị → "Sơ đồ chân", hoặc tìm "chân" / "gpio", CLI `pins`):
  - Từng chân của board: nối với gì (IMU, I2C, cổng serial RX / TX kèm tên UART, đầu ra motor, đo pin, LED, còi, dải LED, thẻ SD, chân board giữ cho USB / SWD / flash).
  - Cột "Đang dùng cho" theo cài đặt hiện tại, kể cả thay đổi chưa áp dụng: chức năng cổng, motor nào của bộ trộn, OSD, thẻ SD, dải LED.
  - Lọc (đang dùng, cổng, motor, cảm biến, khác), tìm theo tên chân hoặc chức năng, xếp theo tên chân.
  - Chân dùng hai lần (ESP32: `led_pin`, chân thẻ SD tự đặt) được báo đỏ.
  - Nút "Xem chân…" ở tab Motor, Cảm biến, Cổng và phần thẻ SD mở thẳng phần liên quan.
  - Theo `src/hal/board.h`: FC thương mại có thể đi dây khác, so với sơ đồ của nhà sản xuất.
- **WiFi:**
  - `wifi_enable 1`: MAVLink UDP 14550 cho QGC + trang cấu hình.
  - `web_enable 1`: chỉ trang cấu hình.
  - `wifi_armed 0` (mặc định): tắt WiFi khi bay.
- **CLI `wifi`** (chỉ qua USB): `wifi name <tên>`, `wifi pass <mật khẩu>`, `wifi webpass <mật khẩu|->`, `wifi reset`.
- **MAVLink:** nhiều drone thì đặt `mav_sysid` khác nhau.
- **MAVLink tốc độ thấp** (chức năng 12): cho module LoRa trong suốt như E32 / E22 (SX1278 / SX1268) hoặc radio 9600 baud.
  - Gửi ít hơn khoảng 4 lần: trạng thái 2 s, tư thế / vị trí / HUD 1 s, GPS 3 s, không gửi kênh RC, tham số mỗi 200 ms. Khoảng 90 B/s, vừa với 9600 bit/s.
  - Nối E32: TX module → RX cổng, RX module → TX cổng, M0 = M1 = GND (chế độ thường), VCC 5 V, GND chung. Hai module cùng kênh, cùng địa chỉ, cùng tốc độ không trung (air rate). Baud mặc định 9600.
  - Module thứ hai cắm USB-UART vào máy tính, mở QGC / Mission Planner ở cổng COM đó, 9600 baud.

---

### Telemetry trên tay điều khiển (CRSF / ELRS, EdgeTX)

FC gửi về tay điều khiển qua ExpressLRS (CRSF), luân phiên mỗi 40 ms một gói: tư thế (`Ptch` `Roll` `Yaw`), pin (`RxBt` `Curr` `Capa` `Bat%`), chế độ bay (`FM`, có `*` khi chưa arm), vario (`VSpd`), độ cao baro so với điểm cất cánh (`Alt`), GPS (`GPS` `GSpd` `Hdg` `Sats`, chỉ khi có fix). Tay điều khiển tự thêm `RQly` `1RSS` `2RSS` `RSNR` `TPWR`.

**Màn hình telemetry cho tay điều khiển màn đen trắng 128×64** (RadioMaster Pocket, Zorro, Boxer, TX12…): `tools/edgetx/SCRIPTS/TELEMETRY/aqtlm.lua`.

| Trang | Nội dung |
|---|---|
| 1 Bay | Chế độ, đồng hồ bay, LQ (nhấp nháy khi < 50 %); điện áp to, V/cell, số cell, % pin + thanh; độ cao, vario, tốc độ, số vệ tinh; khoảng cách về nhà + mũi tên chỉ hướng nhà; dòng, mAh, công suất phát |
| 2 Sóng & GPS | Chân trời nhỏ, RSSI 2 ăng-ten, SNR, công suất, roll / pitch, hướng, toạ độ |
| 3 Tổng kết | Tự hiện khi disarm sau chuyến bay: thời gian, cao nhất, nhanh nhất, xa nhất, điện áp thấp nhất, dòng lớn nhất, mAh đã dùng |

- Cảnh báo: pin < 3,5 V/cell nhấp nháy + rung một lần; < 3,3 V/cell rung ba lần (sửa `CELL_WARN`, `CELL_CRIT` ở đầu file).
- Điểm nhà = vị trí GPS lúc arm (cần ≥ 6 vệ tinh).
- Cài: chép thư mục `SCRIPTS` vào thẻ SD của tay điều khiển → MODEL → DISPLAY (Telemetry screens) → Screen 1: *Script* → `aqtlm` → MODEL → TELEMETRY → *Discover new* (bật drone) → về màn hình chính, giữ **TELE** (Pocket) để mở. Nút ENTER / PAGE đổi trang.
- ELRS: bật Telemetry Ratio (mặc định `Std` là đủ; `1:8` cập nhật nhanh hơn).
- Xem trước trên máy tính: `python3 tools/edgetx/preview.py` (chạy script bằng Lua 5.2 như EdgeTX, xuất ảnh màn hình).

**Chỉnh FC từ tay điều khiển** (`tools/edgetx/SCRIPTS/TOOLS/aqcfg.lua`, cần CRSF trên cổng receiver): 4 trang PID / RATES / FILTER / VTX, đọc và ghi qua chính link ELRS (MSP trong khung CRSF 0x7A / 0x7B, cùng cách các script Lua của Betaflight dùng). Cài: chép vào `/SCRIPTS/TOOLS/` → SYS → Tools → `aqcfg`. Con lăn / +- chọn, ENTER sửa rồi ENTER gửi, PAGE đổi trang, mục **SAVE TO FC** để giữ sau khi tắt nguồn. Chỉ đổi được khi đã disarm. Script được thử với chính mã MSP-over-CRSF của firmware (`preview.py`), chưa thử trên tay thật.

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
| `selftest` | kiểm tra nhanh một màn hình, mỗi dòng `[ OK ]` / `[WARN]` / `[FAIL]`: lý do reset, crash, vòng bay, stack, IMU, I2C, baro, compass, receiver (khung / lỗi checksum / LQ), lỗi từng UART, GPS, pin, nơi lưu cấu hình, watchdog, điều kiện arm |
| `crash [clear\|test\|hang]` | bản ghi lỗi firmware gần nhất (STM32: loại lỗi, task, PC / LR để tra bằng `arm-none-eabi-addr2line`); `test` / `hang` (đã disarm, tháo cánh) cố ý gây lỗi / treo để thử bản ghi và watchdog |
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
| `pins [lọc]` | sơ đồ chân của board: mỗi chân nối với gì và cài đặt đang dùng nó cho việc gì (`pins uart`, `pins PB`, `pins motor`) |
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
  - **Ký số (tuỳ chọn):** `python3 tools/ota_sign.py keygen` tạo khoá riêng ở `~/.allquad/ota_private.pem` (không nằm trong thư mục dự án; **sao lưu nó**) và khoá công khai `src/web/ota_pubkey.h`. Nạp bản có khoá qua USB một lần; từ đó drone **chỉ nhận qua WiFi file `firmware.signed.bin`** (PlatformIO tự tạo cạnh `firmware.bin` khi có khoá; ECDSA P-256 trên SHA-256 của ảnh firmware). File không ký / ký khoá khác bị từ chối, firmware cũ giữ nguyên. Mất khoá thì chỉ còn nạp USB. Không chạy `keygen` thì mọi thứ như cũ.
- **Nạp firmware từ trang (GitHub Pages):** `python3 tools/make_pages.py` tạo thư mục `pages/` = trang + `firmware/*.bin` + `manifest.json` (kích thước, SHA-256). Đưa thư mục lên Pages; hộp *Nạp firmware* có mục *Bản phát hành trên trang này*, tự chọn đúng board, tải về và kiểm SHA-256 trước khi nạp.
- **Nâng cấp giữ cấu hình:** tham số mới chỉ thêm vào cuối. Cấu hình cũ (v15 trở lên) tự chuyển sang **v31** với giá trị mặc định cho trường mới.
- **v2.2, ESP32 DevKit 4 MB:** ô firmware tăng 1,5 → 1,625 MB (flash đang dùng 95 % → 88 %, còn chỗ cho bản sau), blackbox 896 → 640 KB (~14 s ở 500 Hz; dùng thẻ SD nếu cần log dài). Board đang chạy bảng cũ vẫn chạy và vẫn cập nhật qua WiFi được chừng nào firmware còn vừa ô 1,5 MB; muốn có bảng mới thì nạp **file gộp** qua USB một lần (sao lưu cấu hình trước). Cấu hình v31 (`bb_gyro_s`). ESP32-S3 / C6 / P4 không đổi.
- **v2.1, ESP32 DevKit 4 MB:** ô firmware tăng 1,375 → 1,5 MB (blackbox còn 896 KB). Board đang chạy bảng phân vùng cũ phải nạp **file gộp** qua USB một lần (cấu hình về mặc định: sao lưu trước); cập nhật qua WiFi sẽ báo lỗi chứ không ghi hỏng. ESP32-S3 / C6 / P4 không đổi.
- **STM32 lưu cấu hình hai bản (H743, F405):** mỗi lần lưu ghi vào bản không dùng, có số thứ tự + CRC; mất điện giữa lúc lưu thì lần sau dùng bản cũ còn nguyên. F405 vì vậy giới hạn firmware 768 KB (đang ~300 KB); H743 blackbox flash còn 6 sector (768 KB). F722 / F411 không còn sector trống: một bản như trước.

---

## Kiến trúc & hiệu năng

**Bộ nhớ (build hiện tại):** F405 đặt heap FreeRTOS (task stack, `new`) vào 64 kB CCM RAM, RAM chính còn ~53 % trống (heap không bao giờ là bộ đệm DMA: DShot, LED, thẻ SD dùng bộ đệm tĩnh). F411 báo RAM ~95 % vì heap 66 kB được tính là đã dùng ngay từ đầu; chỗ trống thật trong heap xem bằng `selftest` (dòng `heap`; bản F405 trong QEMU còn ~31 kB trống trên 64 kB). Trang cấu hình trong firmware ESP32 nén bằng zopfli khi có (`pip install zopfli`): nhỏ hơn gzip ~4 %.

| Task | Tần số | Ưu tiên | Nhân (ESP32) | Việc |
|---|---|---|---|---|
| `fast` | 1–6.4 kHz (timer / ngắt IMU) | 6 | 1 | IMU → lọc → AHRS → PID → mixer → motor |
| `io` | 500 Hz | 5 | 0 | receiver, GPS, telemetry ESC, cảm biến khoảng cách |
| `control` | 250 Hz | 4 | 0 | EKF, arm, failsafe, chế độ, điều hướng |
| `sensors` | 100 Hz | 3 | 0 | baro, compass, pin, LED, còi |
| `telemetry` | 100 Hz | 2 | 0 | MAVLink, MSP, CLI, OSD, trang cấu hình qua USB |
| `logger` | `bb_rate_hz` | 1 | 0 | blackbox (không bao giờ chờ thiết bị lưu) |
| `sdcard` | 100 Hz | 1 | 0 | ghi thẻ SD (chỉ khi `sd_mode` bật) |
| `web` | 20 Hz | 1 | 0 | web server (ESP32, khi bật WiFi) |

- **Trao đổi dữ liệu:** các task dùng **topic** publish / subscribe, mỗi topic một task ghi.
- **Đo hiệu năng trên board:** `perf` / `tasks`, hoặc Hệ thống → Hiệu năng.
- **H743, vòng bay tới 6.4 kHz** (`loop_hz` tối đa 6400 = tốc độ dữ liệu của ICM-45686; mặc định vẫn 3200):
  - Đọc IMU trên H7 không qua HAL: địa chỉ và dữ liệu trong một lần truyền SPI. Được so với HAL ở những lần đọc đầu; hết thời gian chờ hoặc lệch thì quay về HAL luôn.
  - Bidirectional DShot không còn chờ: ngắt DMA đảo chân và bắt đầu lấy mẫu ngay khi khung DShot gửi xong (trước đây vòng bay đứng chờ 27–53 µs mỗi vòng). Ngắt không tới 3 lần thì quay về cách cũ.
  - Sau khi đổi `loop_hz`, xem `perf`: tải vòng bay dưới ~70 % là ổn.
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

## Bay thử trên bàn (HITL)

FC thật bay một chiếc drone mô phỏng trên máy tính qua USB, motor đứng yên: `python tools\hitl.py COM5 --auto` (chuyến bay kiểm tra tự động) hoặc `python tools\hitl.py COM5` (bay bằng tay điều khiển thật). Hướng dẫn đầy đủ: [docs/HITL.md](docs/HITL.md).

---

## Kiểm chứng

```bash
tools/run_tests.sh       # unit test + MAVLink + mô phỏng + WebAssembly + bộ phân tích + giao diện
tools/run_sim.sh tune    # một kịch bản mô phỏng
node tools/gen_params_doc.js   # cập nhật docs/PARAMS.md sau khi thêm tham số
```

| Hạng mục | Kết quả |
|---|---|
| Biên dịch PlatformIO | 12 / 12 môi trường (6 ESP32, 5 STM32 + QEMU, cầu WiFi C3) |
| Unit test (kèm AddressSanitizer + UBSan) | **239 / 239** (gồm coning / sculling, kẹp cấu hình về giới hạn khi nạp, A/B cấu hình khi mất điện, MSP qua CRSF, bộ lọc gai RPM, migrate tới v31, ghi gyro tần số cao, thứ tự kênh RC, chế độ bay lần đầu, băng thông MAVLink tốc độ thấp) |
| Mô phỏng toàn hệ thống | 44 kịch bản đạt (hướng EKF quay 178° được trọng tài GPS sửa, chọn IMU tốt hơn khi chưa arm, EKF với cảm biến lỗi, RTH, nhiệm vụ, gió, optical flow, autotune, GPS Rescue, launch, bidir DShot, mất RPM telemetry khi bay, console USB / cầu WiFi, EDT, gia tốc kế bị kịch, bay lần đầu…) |
| QEMU STM32F405 | khởi động firmware thật + bản ghi crash còn sau reset + báo cáo stack còn trống từng task (CI fail nếu task nào < 256 B) |
| Fuzz bộ đọc giao thức (`tools/fuzz.sh`, ASan + UBSan) | 11 bộ đọc (CRSF/SBUS…, MSP, Micolink, MAVLink, GPS UBX/NMEA, cảm biến, AqLink, CLI, 4-way, cấu hình, khung HITL) × 3 seed × 1 triệu mẫu: không crash, không lỗi sanitizer, mọi tham số trong giới hạn. Đã sửa: tần số VTX ngoài dải, rangefinder Micolink 3000 km, GPS fix với toạ độ ngoài Trái Đất, tràn số NMEA, memcpy null trong MAVLink |
| Phân tích tĩnh (`tools/static_check.sh`) | cppcheck sạch (cảnh báo giả đã xem xét nằm trong `tools/cppcheck.supp`); tuỳ chọn clang-tidy |
| Soak (`./build/sim soak N seed`) | **360 / 360** + **240 / 240** sau các thay đổi EKF v2.2 (coning, trọng tài GPS, kéo tư thế khi treo yên): chuyến bay ngẫu nhiên 170 s (ACRO/ANGLE/POSHOLD/RTH, gió, mất sóng, GPS nhảy, baro lệch, rung, nhiễu IMU, EKF / Kalman, loop 1/2 kHz): không NaN, motor trong 0..1, failsafe đúng hạn, không bay mất, LAND xong và disarm. Đã sửa 3 lỗi thật chỉ lộ ra sau va chạm / lộn nhào: EKF mất một nửa delta góc ở loop 2 kHz (tư thế EKF trôi), EKF không tự sửa khi tư thế lệch AHRS, Kalman độ cao kẹt cổng 6σ sau va chạm |
| Script Lua EdgeTX | `aqtlm` + `aqcfg` chạy trong Lua 5.2, `aqcfg` nói chuyện với mã MSP-over-CRSF thật |
| MAVLink đối chiếu pymavlink | 17 / 17 |
| Bộ phân tích blackbox | 29 / 29 (khung điều hướng: baro / la bàn bị loại, EKF đặt lại tư thế / hướng; khung gyro 4 kHz, gồm: nén, log hỏng, khung ESC, phát hiện motor yếu 15 %, hội tụ khi chỉnh lặp, tóm tắt chuyến bay, pin yếu) |
| Firmware WebAssembly | 5 / 5 |
| HITL (`tools/hitl.py --exec "build/sim serve" --auto`) | 6 / 6: 500 khung/s, arm, ALTHOLD (dao động 0.03 m), POSHOLD, tư thế ước lượng lệch ≤ 1.3°, LAND và disarm |
| Ký số firmware OTA | 6 / 6 (`tools/test_ota_sign.py`; kiểm chéo với mbedtls như firmware) |
| Giao diện (Playwright) | đạt toàn bộ: web, configurator USB + SITL, công cụ laptop, tính năng mở rộng, bản đồ offline, PWA, nạp firmware, thẻ SD, hướng dẫn bay lần đầu (thử nghiêng / failsafe), tìm hướng optical flow (16 kiểu lắp), thuật ngữ, gyro tần số cao, giao diện tiếng Anh |

**CI:** `.github/workflows/ci.yml` chạy kiểm tra sơ đồ chân, unit test (cả sanitizer), mô phỏng, bộ phân tích, cppcheck, fuzz và soak (mỗi lần chạy một seed mới; bản đêm chạy dài 300 chuyến), build 11 môi trường, khởi động F405 trong QEMU kèm bảng stack từng task và toàn bộ test giao diện (Playwright, drone ảo WebAssembly) cho mỗi lần push. File firmware giữ 14 ngày. Để dùng: đưa thư mục dự án lên GitHub (`git init`, `git add .`, `git commit`, `git push`).

**Chưa kiểm chứng trên phần cứng:** DShot / bidir trên chip thật, RMT, OSD MAX7456, khởi tạo từng cảm biến, thẻ SD trên chip thật (cả `sd_dma`), watchdog IWDG và lưu A/B trên flash thật, cầu WiFi ESP32-C3, script `aqcfg` trên tay thật, các board F405 / F722 / C6 / P4. Khi thử trên bàn, gửi kèm kết quả `status`, `perf`, `tasks`, `sd`, `esc` để kiểm tra.

---

## Hướng phát triển

- **Sau khi có log bay thật:** chỉnh mặc định PID / bộ lọc / kích thước hàng đợi thẻ SD theo số đo.
- **Ghi gyro thô tần số cao** (2–4 kHz, từng đợt vài giây) lên thẻ SD để phân tích phổ sâu.
- **EKF nhiều IMU:** chọn IMU tốt nhất.
- **WiFi cho ESP32-P4** qua ESP32-C6 (esp-hosted).
- **Ký số firmware OTA.**
