# Khả năng và giới hạn theo từng FC

Cùng một firmware chạy trên nhiều chip, nhưng không phải chip nào cũng làm được mọi thứ. Chỗ nào một tính
năng không an toàn hoặc vô ích trên một board, firmware tự tắt hoặc tự giới hạn nó. Bảng này liệt kê các giới
hạn đó, để bạn không phải tự dò. Số liệu bộ nhớ là của bản build hiện tại (CI, tháng 11/2026).

## Bảng tổng hợp

| Board (`pio run -e …`) | Chip | `loop_hz` mặc định / tối đa | Motor | DShot hai chiều (RPM) | Blackbox | Thẻ SD | Gyro tần số cao | Trang cấu hình trong chip | Cập nhật qua WiFi | Bay thử trên bàn (HITL) |
|---|---|---|---|---|---|---|---|---|---|---|
| `esp32dev` | ESP32, 240 MHz, 2 nhân | 1000 / 2000 | DShot300/600, OneShot, PWM | có (RMT) | flash trong chip 640 kB, serial, SD | SPI / SDMMC | có (chỉ khi ghi SD) | **bản rút gọn**: không có tiếng Anh, không có HITL | có (2 ô 1.625 MB) | có, qua USB |
| `esp32s3` | ESP32-S3, 240 MHz, 2 nhân, 8 MB | 1000 / 2000 | như trên | có | như trên | SPI / SDMMC | có | đầy đủ | có | có |
| `esp32s3_n16r8` | ESP32-S3, 16 MB + PSRAM 8 MB | 1000 / 2000 | như trên | có | như trên (hàng đợi SD lớn trong PSRAM) | SPI / SDMMC | có | đầy đủ | có | có |
| `esp32c6` | ESP32-C6, 160 MHz, 1 nhân, WiFi 6 | 1000 / 2000 | **OneShot125 / PWM** (DShot bị đổi sang OneShot) | **không** | flash trong chip, serial, SD | SPI | có | đầy đủ | có | có |
| `esp32p4` | ESP32-P4, không có radio | 1000 / 2000 | DShot300/600, OneShot, PWM | có | flash trong chip, serial, SD | SPI / SDMMC | có | không có WiFi: dùng trang trên máy tính qua USB | không (nạp qua USB) | có |
| `blackpill_f411ce` | STM32F411, 100 MHz, 128 kB RAM | 1000 / 2000 | DShot300/600, OneShot, PWM | có | **chỉ serial** (OpenLog) | không | **không** (thiếu RAM) | không có (trang trên máy tính qua USB) | không | có |
| `stm32f405` | STM32F405, 168 MHz | 1000 / 2000 | như trên | có | chỉ serial | không | không (cần SD) | không có | không | có |
| `stm32f722` | STM32F722, 216 MHz | 1000 / 2000 | như trên | có | chỉ serial | không | không (cần SD) | không có | không | có |
| `stm32h743_weact`, `stm32h743_devebox` | STM32H743, 480 MHz | **3200 / 6400** | DShot600 mặc định | có, đảo chiều bằng ngắt DMA | flash trong chip 896 kB, serial, **SD 4-bit** | SDMMC (cố định) | có | không có | không | có |
| `bridge_esp32c3` | ESP32-C3 | – | – | – | – | – | – | cầu WiFi cho FC STM32, không phải FC | – | – |

Bộ nhớ: `esp32dev` dùng 94 % ô firmware 1.625 MB trước khi cắt trang cấu hình (khoảng 92 % sau khi cắt).
`blackpill_f411ce` báo RAM 95 % vì heap 66 kB được tính là đã dùng; chỗ trống thật xem bằng lệnh `selftest`.
`bridge_esp32c3` dùng 92 % ô firmware.

## Từng board

### ESP32 DevKit (`esp32dev`, module 4 MB)
- **Trang cấu hình trong chip là bản rút gọn** (`AQ_WEB_LITE`): bỏ chế độ tiếng Anh và bảng HITL, bớt
  khoảng 52 kB flash. Lý do: ô firmware 1.625 MB đã đầy 94 %, thêm tính năng nữa thì không build được.
  Mọi tab và công cụ khác vẫn đủ. Cần tiếng Anh hoặc HITL: dùng trang trên GitHub Pages hoặc file
  `AllQuad-Configurator.html` trên máy tính qua cáp USB.
- Không bỏ một ô OTA để lấy chỗ: hai ô OTA là thứ cho phép cập nhật qua WiFi mà tự quay về bản cũ khi
  bản mới hỏng. Mất nó nguy hiểm hơn mất tiếng Anh.
- `loop_hz` tối đa 2000 Hz.

### ESP32-C6 (`esp32c6`)
- Chỉ có 2 kênh RMT phát: không đủ cho 4 dây DShot. Đặt DShot thì firmware **tự đổi sang OneShot125**.
  Vì vậy không có DShot hai chiều, không có lọc RPM từ DShot (vẫn dùng được telemetry ESC qua dây
  serial). Dùng ESC chấp nhận OneShot125 / PWM.
- Một nhân 160 MHz: giữ `loop_hz` 1000.

### ESP32-P4 (`esp32p4`)
- Chip không có WiFi / Bluetooth. Cấu hình qua cáp USB bằng trang trên máy tính. Không cập nhật qua WiFi.

### STM32F411 BlackPill (`blackpill_f411ce`)
- 128 kB RAM: không có bộ đệm gyro tần số cao (`bb_gyro_s` không có tác dụng), không có thẻ SD,
  blackbox chỉ ra cổng serial (OpenLog).
- `loop_hz` tối đa 2000 Hz; nếu `perf` báo tải trên 70 % thì về 1000.

### STM32F405 / F722 (`stm32f405`, `stm32f722`)
- Không có khe SD: blackbox ra cổng serial (OpenLog). Gyro tần số cao chỉ ghi lên thẻ SD nên không dùng được.
- `loop_hz` tối đa 2000 Hz.

### STM32H743 WeAct / DevEBox (`stm32h743_weact`, `stm32h743_devebox`)
- Vòng bay mặc định 3200 Hz, tối đa 6400 Hz (đúng tốc độ dữ liệu của ICM-45686). Với ngắt data-ready của
  IMU, tần số thật là tốc độ dữ liệu của IMU chia cho một số nguyên: ICM-45686 6400 / 3200 / 2133…,
  ICM-42688-P 8000 / 4000 / 2667…
- Đọc IMU bằng thanh ghi SPI trực tiếp, không qua HAL. Firmware so với HAL ở những lần đọc đầu; nếu lệch
  hoặc quá thời gian chờ thì quay về HAL luôn. Lệnh `perf` cho biết đang dùng đường nào.
- DShot hai chiều: ngắt DMA đảo chân ngay khi khung DShot gửi xong, vòng bay không đứng chờ. Nếu ngắt
  không tới 3 lần thì quay về cách đứng chờ. `perf` cho biết cách đang dùng.
- Thẻ SD 4-bit (SDMMC) cố định trên board, cùng blackbox trong flash 896 kB.

## Theo từng loại cấu hình

### Tần số vòng bay và motor
| Cấu hình | Giới hạn | Lý do |
|---|---|---|
| `loop_hz` vượt mức tối đa của board | về 1000 khi khởi động | board không theo kịp |
| DShot300 + `dshot_bidir 1` + `loop_hz` > 4000 | **tự về 3200**, có tin báo | khung DShot300 mất 53 µs, cửa sổ nghe trả lời khoảng 130 µs: không vừa chu kỳ dưới 250 µs. DShot600 vừa tới 6400 Hz |
| Vòng bay không theo kịp (`perf`: tải cao, chạy chậm hơn `loop_hz`) | **không cho arm** (`ARM_CPU_LOAD`) | bay với vòng bay trễ là mất ổn định |
| DShot trên ESP32-C6 | tự đổi OneShot125 | thiếu kênh RMT |

### Bộ ước lượng (`est_type`)
- `0`: Kalman tách trục, nhẹ, đủ cho ANGLE / ALTHOLD / POSHOLD có GPS.
- `1`: EKF 15 trạng thái. Có trên mọi board. Tự sửa khi chính nó sai: đặt lại tư thế theo AHRS, đặt lại
  hướng theo la bàn khi hướng tính từ GPS đồng ý với la bàn. Mọi lần đặt lại được ghi vào blackbox
  (khung điều hướng) và hiện trong tab Blackbox.

### Giữ độ cao (baro, cảm biến khoảng cách)
Mọi board như nhau; phần tính thêm chỉ vài phép nhân mỗi mẫu baro / mẫu khoảng cách và một căn bậc hai mỗi
vòng điều khiển độ cao, không đáng kể cả trên F411.

| Cấu hình | Mặc định | Làm gì / giới hạn |
|---|---|---|
| Cảm biến khoảng cách (MT-01P…) trong tầm `rf_max_m`, nghiêng dưới 30° | tự dùng | độ cao lấy chi tiết từ cảm biến, baro chỉ giữ dài hạn. Mặt đất đổi bậc (bậc thềm, mép bàn) **không** làm drone nhảy: bước nhảy lớn hơn 0.4 m được coi là đổi mặt đất. Ra ngoài tầm: về baro, không giật |
| `baro_gnd 1` | bật | đã arm và dưới khoảng 1 m: baro tụt (luồng gió cánh đập xuống đất) tới 0.5 m bị bỏ qua, phần lớn hơn tính bớt 0.5 m. Cất / hạ cánh không bị nảy. Đang disarm thì không áp dụng |
| `baro_dyn` | **0 (tắt)** | sửa sai số baro khi bay nhanh (gió thổi qua lỗ baro). Mỗi khung và cách đặt baro khác nhau nên không đặt sẵn. Chỉ dùng khi có GPS hoặc optical flow (cần tốc độ ngang). Cách chỉnh bên dưới |
| `thr_curve` | 0.5 | vòng độ cao tính theo lực đẩy thay vì theo ga: cùng độ nhạy ở ga treo như trước, đều hơn khi ga xa ga treo và khi nghiêng. **Tự tắt khi `thrust_expo` > 0** (mixer đã làm việc này). 0 = như bản cũ |

Chỉnh `baro_dyn`: bay POSHOLD ở độ cao vài mét, đẩy cần pitch hết cỡ vài giây rồi thả. Nếu drone tự **leo**
lúc bay nhanh (baro đọc thấp), tăng `baro_dyn` (thử 0.01, 0.02…); nếu tự **hạ**, đặt âm. Đúng khi độ cao
gần như không đổi lúc lao. Trong mô phỏng (`./build/sim alt_real`) đặt đúng giá trị làm sai lệch độ cao lúc
lao giảm từ 1.4 m xuống 0.3 m.

Ngoài ra, ALTHOLD cất cánh từ mặt đất: khi drone chưa rời đất, đích độ cao chỉ đi trước tối đa 1 m (trước là
2 m) để lượng leo dồn lại lúc motor đang tăng tốc không thành vọt quá sau khi rời đất.

### Khi cảm biến hỏng giữa chuyến bay
Mọi board như nhau. Mỗi lần firmware phải đổi chế độ vì thiếu cảm biến, OSD / trạm mặt đất hiện tin
"`<chế độ>` unavailable: `<chế độ thay>`".

| Sự cố | Firmware làm gì |
|---|---|
| Mất GPS (POSHOLD, MISSION, CRUISE, GUIDED) | về ALTHOLD giữ đúng độ cao lúc mất; GPS về thì chế độ cũ chạy lại |
| Mất GPS khi đang RTH / SmartRTL | LAND tại chỗ (không bay mò về nhà) |
| Baro chết, có GPS | độ cao lấy từ GPS (cả `est_type` 0 và 1), không giật; tin "Baro lost - height from GPS" |
| Baro chết, không có GPS | ALTHOLD về ANGLE (tin "Baro lost - no height hold"). Công tắc LAND hoặc mất RC: hạ không cần độ cao (FAILSAFE). Nếu bắt đầu hạ trong vòng 30 s sau khi mất baro: khoảng 1 m/s theo gia tốc kế, ga quanh ga treo đã học, tự disarm khi chạm đất. Muộn hơn (gia tốc kế không còn đáng tin): ga cố định 90 % ga treo. Luôn disarm sau `fs_off_delay_ms` (mặc định 30 s): **đặt đủ dài cho độ cao bạn hay bay** |
| La bàn chết | hướng từ GPS khi bay / con quay; POSHOLD, RTH vẫn chạy; tin "Compass lost" |
| Motor yếu / cánh hỏng nhẹ (mất khoảng 30 % lực) | vẫn giữ thăng bằng, độ cao, vị trí, RTH hạ cánh được |
| Áp pin sụt khi đẩy ga | cảnh báo chỉ tính khi thấp liên tục 2 s, mức nguy hiểm 3 s: đẩy ga ngắn không kích failsafe pin |
| Vượt trần độ cao `fence_alt_max` | RTH bay về **dưới** trần (trần − 2 m), kể cả khi `rth_alt` cao hơn |

### Blackbox (`bb_backend`)
| Giá trị | Board | Ghi chú |
|---|---|---|
| 1 serial (OpenLog) | mọi board | cần một cổng ở chức năng Blackbox |
| 2 flash trong chip | ESP32 (640 kB), H743 (896 kB) | xoá nền khi disarm |
| 3 thẻ SD | ESP32 (SPI / SDMMC theo board), H743 (SDMMC) | gyro tần số cao (`bb_gyro_s`) chỉ ghi ở đây |

### Bay thử trên bàn (HITL)
- Chỉ qua cổng USB của FC (không qua WiFi, không qua cổng UART khác), chỉ khi đang disarm.
- Motor luôn bị giữ dừng trong suốt HITL. Thoát HITL bằng cách khởi động lại; FC tự khởi động lại khi máy
  tính ngừng gửi 1 s mà đang disarm.
- Có trên mọi FC. Dùng `tools/hitl.py`, hoặc bảng HITL trong tab Hệ thống của trang trên máy tính /
  GitHub Pages (không có trong trang rút gọn của `esp32dev`).

### Trang cấu hình
| Nơi mở | Có |
|---|---|
| GitHub Pages / `AllQuad-Configurator.html` trên máy tính | mọi thứ, gồm tiếng Anh, HITL, drone ảo, nạp firmware |
| Trang trong ESP32-S3 / C6 (qua WiFi của drone) | mọi thứ trừ drone ảo; HITL ẩn (cần USB); nạp firmware cần Web Serial trên máy tính |
| Trang trong `esp32dev` (qua WiFi) | như trên, trừ tiếng Anh |
