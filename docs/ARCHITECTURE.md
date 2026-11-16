# AllQuad – Kiến trúc phần mềm

## Phân lớp

```
 ┌──────────────── app (task wiring, commander) ────────────────┐
 │ control │ estimation │ navigation │ safety │ comms │ osd │ log │   C++ thuần, không biết chip
 ├─────────────── bus (topics) ── config (params) ───────────────┤
 │ sensors / rx / gps drivers (chỉ gọi HAL)                       │
 ├──────────────────────── os ───────────────────────────────────┤   FreeRTOS hoặc bộ lập lịch PC
 └──────────────────────── hal ──────────────────────────────────┘   ESP32 | STM32 | host (PC)
```

- **hal/**: mọi thứ phụ thuộc chip. Thêm chip mới = một file `hal_<chip>.cpp` + một khối chân trong `board.h`.
- **os/**: `createTask` (task tuần hoàn, task nhanh do timer phần cứng đánh thức qua task notification), `CriticalGuard`, `Topic<T>` (mỗi topic một spinlock riêng: trên ESP32 hai nhân, task nhân 0 chép một topic không làm vòng bay ở nhân 1 phải chờ topic khác). `TaskStats.stackFree` = stack chưa từng dùng (high-water mark), xem bằng CLI `tasks` hoặc tab Hệ thống. Trên PC là bộ lập lịch tất định: khi nhiều task đến hạn cùng lúc, task ưu tiên cao chạy trước — giống RTOS.
- **bus/**: dữ liệu chia sẻ. Mỗi topic chỉ có **một task ghi**; đọc là sao chép snapshot (khoá rất ngắn). Thay đổi cấu hình tăng `configGeneration` để mọi task tự nạp lại.

## Luồng dữ liệu

```
 IMU ─► [fast] lọc + AHRS + PID + mixer ─► motor
           │ attitude, motors, pid                 ▲ setpoint
           ▼                                       │
 baro, mag, pin ─► [sensors] ──► [control] Kalman ─► chế độ bay ─► điều hướng ─► vòng ngoài
 RC, GPS, ESC, TFmini ─► [io] ─┘        │ nav, status
                                         ▼
                 [telemetry] MAVLink · MSP · CLI · OSD      [logger] blackbox
```

| Topic | Ghi bởi | Đọc bởi |
|---|---|---|
| `rc`, `gps`, `esc`, `range` | io | control, telemetry, logger |
| `flow`, `range` (module MSP) | telemetry (MSP) | control |
| `baro`, `mag`, `battery` | sensors | control, fast (mag), telemetry |
| `attitude`, `motors`, `pid` | fast | control, telemetry, logger |
| `setpoint`, `nav`, `status` | control | fast, telemetry, sensors, logger |

Lệnh từ CLI/MAVLink tới bộ chỉ huy đi qua `bus::postRequest` (arm, disarm, đổi chế độ, hiệu chuẩn).

## Thời gian thực
- Task `fast` được đánh thức bởi timer phần cứng (ESP32: GPTimer phần cứng, ngắt đặt trên nhân 1 và báo thẳng cho task — không qua task `esp_timer` ở nhân 0 vốn rung 10–50 µs khi WiFi bận, `esp_timer` chỉ là dự phòng; STM32 TIM5 / TIM7 trên H7, hoặc chân INT1 của IMU qua EXTI) → 1–2 kHz không phụ thuộc tick 1 ms.
- Thống kê mỗi task (thời gian chạy, tối đa, số lần trễ) xem bằng lệnh CLI `tasks`; arm bị chặn nếu task nhanh dùng > 90% chu kỳ.
- Bus không chia sẻ giữa các task: SPI IMU → fast; I2C → sensors; SPI thứ hai (OSD) → telemetry; mỗi cổng serial → task phụ trách chức năng của nó.
- DShot trên STM32 dùng DMA (không chặn CPU); trên ESP32 dùng RMT bất đồng bộ.
- **Ghi flash không bao giờ xảy ra giữa chuyến bay** (trừ blackbox ESP32, xem dưới): lưu cấu hình, nhiệm vụ, điểm rally khi đang armed chỉ được ghi nhớ và thực hiện sau khi disarm (`nav::persistPending()`), vì xoá một sector flash làm STM32F411 đứng toàn bộ 0,5–2 s và làm ESP32 dừng cả hai nhân. Xoá log blackbox cũng chỉ chạy nền khi disarmed, và chặn arm cho tới khi xong.
- **Blackbox ESP32** ghi vào phân vùng flash thô (không phải LittleFS): trong lúc bay chỉ ghi từng trang 256 byte (khoảng 0,5 ms), và mỗi lần ghi được canh vào khoảng trống ngay sau khi vòng nhanh vừa chạy xong (`hal::markFastLoopDone()`), nên không làm trễ vòng PID ở 1 kHz. Ở 2 kHz khe trống hẹp hơn: nên ghi blackbox qua cổng serial (OpenLog).
- **ESP32**: toàn bộ phần tính của vòng nhanh (bộ lọc, notch động, RPM filter, AHRS, PID, mixer, task) nằm trong IRAM (`FAST_CODE`), không phụ thuộc bộ đệm flash. **STM32**: bật ART accelerator (cache lệnh/dữ liệu + prefetch; H7: I-cache + D-cache) để chạy từ flash không chờ.
- Notch động: FFT 128 điểm chia thành 9 bước nhỏ bằng nhau (chép + cửa sổ theo thứ tự đảo bit, 7 tầng butterfly dùng bảng twiddle tính sẵn, tìm đỉnh), mỗi vòng chỉ làm một bước. RPM filter dùng một bộ hệ số chung cho 3 trục. Biên dịch với `-fno-math-errno` để `sqrtf` thành một lệnh FPU.
- **Đo trên board**: bộ đếm chu kỳ CPU (ESP32 CCOUNT, STM32 DWT) đo từng khâu của vòng nhanh; xem bằng lệnh CLI `perf` hoặc tab Hệ thống trên web (đọc IMU, bộ lọc, AHRS, PID + mixer, xuất motor, tải %, vòng chậm nhất, số vòng trễ).

## Vòng điều khiển
1. **fast**: hiệu chuẩn gyro (tự làm lại nếu bị rung) → RPM filter → notch tĩnh → notch động (FFT 128 điểm, chia nhỏ qua nhiều vòng lặp) → biquad + PT1 → AHRS Mahony (giảm tin cậy accelerometer khi |a| ≠ 1g hoặc quay nhanh) → PID tốc độ (D trên phép đo, feedforward, I-relax, TPA, anti-gravity) → mixer (airmode, co giãn khi bão hoà) hoặc turtle.
2. **control**: Kalman 3 trạng thái mỗi trục (vị trí, vận tốc, bias gia tốc; cổng loại ngoại lai), AGL từ cảm biến khoảng cách, vận tốc từ optical flow (khử chuyển động quay) → kiểm tra arm → failsafe → chọn chế độ (có chuỗi dự phòng: MISSION → POSHOLD → ALTHOLD → ANGLE; RTH → LAND → FAILSAFE) → navigator → điều khiển vị trí (căn bậc hai + giới hạn gia tốc) → điều khiển độ cao (vận tốc PID + bù nghiêng) → điều khiển góc.

## Kiểm thử
- `build/sim <scenario>`: toàn bộ firmware + mô hình vật lý (quán tính, trễ motor, lực đẩy phi tuyến, kéo khí động, va chạm đất), nhiễu, bias gyro, rung theo vòng quay motor, gió.
- `build/tests`: unit test các module thuần.
- `tools/test_mavlink.py`: đối chiếu MAVLink với pymavlink.
- `test/web/test_analysis.js`, `test/web/test_ui.py`: bộ phân tích blackbox và giao diện web (máy chủ giả `tools/web_mock.py`).
- `test/web/test_wasm.js`: firmware biên dịch sang WebAssembly trả lời giao thức link (hello, tham số, trạng thái EKF, ghi tham số). `test/web/test_extras.py`: trình thiết lập, demo, tile bản đồ, so sánh blackbox, lịch sử tham số, bảng VTX trên drone ảo.

## Điều hướng & ước lượng (bổ sung)
- `estimation/nav_estimator`: cổng chặn GPS nhảy (đổi vị trí lớn nhưng vận tốc GPS khớp → bỏ vị trí, giữ vận tốc; kéo dài `gps_glitch_s` → nhận vị trí mới). Lệch cả vận tốc > 3 m/s = bộ lọc phân kỳ → khởi tạo lại theo GPS.
- `estimation/gps_heading.h`: `GpsHeading` so ΔvGPS với Δv quán tính trong khung AHRS (cửa sổ 1 s) → độ lệch hướng; `WindEstimator` tính gió = (gia tốc đo − gia tốc do lực đẩy nghiêng)/`wind_drag` + vận tốc.
- Commander quay gia tốc AHRS theo độ lệch hướng trước khi đưa vào Kalman; khi chưa có hướng thì dự đoán ngang không dùng gia tốc kế. Gia tốc thật từ GPS được gửi cho AHRS qua `ControlSetpoint.accKin` để tách gia tốc ly tâm khỏi trọng lực.
- `control/pos_control`: feed-forward vận tốc đi thẳng (không qua giới hạn gia tốc), feed-forward gia tốc (hướng tâm khi bay vòng), bù gió + cản không khí; `AltitudeControl::setHover()` nhận ga treo đã học mà không giật.
- `navigation/navigator`: CRUISE (cà rốt chạy dọc hướng bay), RTH_ALIGN, chọn điểm rally, orbit (mục tiêu luôn trên vòng tròn tại góc hiện tại), DO_JUMP, ROI, CONDITION_YAW. Điểm rally lưu ở `Slot::RALLY` và trao đổi bằng giao thức mission MAVLink `mission_type = 2`.

## EKF (`estimation/ekf.*`, `est_type 1` mặc định)
- Bộ lọc Kalman sai số (error-state) 15 trạng thái: vị trí, vận tốc NED, sai số góc (khung đất), bias gyro, bias accel. Dự đoán bằng góc delta / vận tốc delta tích phân trong vòng nhanh (`AttitudeState.dAng/dVel/dt`), hiệp phương sai F = I + A·dt tính thưa (chỉ các khối khác 0), chạy trong task control.
- Đo: vị trí + vận tốc 3D GPS, độ cao baro (hoặc GPS khi không có / baro lỗi), hướng (compass hoặc GPS đã học), vận tốc optical flow, vận tốc 0 khi đứng yên / không có nguồn ngang (giữ hiệp phương sai không nổ).
- Xử lý lỗi kiểu EKF3 của ArduPilot: mỗi phép đo có tỉ số kiểm định = đổi mới² / (cổng² · phương sai đổi mới); > 1 thì bỏ. Bỏ kéo dài: GPS vị trí > `gps_glitch_s` → reset vị trí; vận tốc GPS > 2 s → reset vận tốc; baro > 5 s → lỗi baro, độ cao từ GPS cho tới khi baro khớp lại 10 s; compass > 5 s khi hướng quan sát được → lỗi compass, `bus::magRejected` báo AHRS bỏ compass, nguồn hướng chuyển sang GPS. NaN / phương sai âm → khởi tạo lại.
- Chưa có hướng (không compass, GPS chưa học): `slaveAttitude()` cho tư thế EKF theo AHRS và loại các trạng thái góc khỏi ước lượng, tránh đổi mới GPS kéo nghiêng / bias accel sai.
- Commander (`ekfFuse`, `buildNavEstimate`): `navEst_` thay cho Kalman tách trục ở mọi chỗ dùng (giữ vị trí, RTH, nhiệm vụ, failsafe); hướng EKF chỉ dùng khi độ lệch chuẩn yaw < 8°. Kalman tách trục vẫn chạy song song (`est_type 0` để dùng lại). Topic `EkfReport` → web (`ekf` trong /api/status), CLI `status`, OSD cảnh báo "EKF".
- Kịch bản mô phỏng `ekf_faults`: baro lệch 15 m, compass xoay 60°, GPS nhảy, cùng lúc — bộ ước lượng cũ rơi xuống đất, EKF giữ được.

## GPS Rescue, launch control, rung
- Box `GPS RESCUE` (13) dùng RTH; kiểm tra tiến triển: `rth_sanity_s` giây không lại gần nhà (cả khi failsafe mất sóng) → hạ cánh tại chỗ, cảnh báo `WARN_RESCUE`.
- Box `LAUNCH` (14) trong ACRO/ANGLE/HORIZON: khi arm, giữ góc chúi `launch_angle`, tắt tích phân I, ga cầm chừng; ga > `launch_thr` thì phóng và tự tắt.
- Rung: độ lệch chuẩn gia tốc (lọc thông cao) trong vòng nhanh → `AttitudeState.vibration`; > `vibe_warn` giữ cảnh báo `WARN_VIBRATION` 2 s (OSD "VIBRATION", chip trên web).

## Bidirectional DShot
- `hal/dshot_codec.h` (thuần, có test): khung DShot đảo checksum, mã hoá GCR 5→4 bit, chuỗi độ dài mức → giá trị, eRPM ↔ chu kỳ, giải mã từ mẫu (STM32) hoặc từ độ dài xung RMT (ESP32).
- ESP32: kênh RMT thu tạo trước, kênh phát `invert_out + io_loop_back + io_od_mode` trên cùng chân, encoder copy; sau mỗi khung bật thu 30 µs. STM32: DMA2 Stream5/TIM1 ghi BSRR, chờ khung xong, đổi chân sang input, DMA lấy 150 mẫu IDR ở 3,75× tốc độ bit.
- eRPM → `bus::bidirRpm` → RPM filter (`esc_telemetry` hoặc `dshot_bidir`). ESC passthrough tắt kênh bidir và bật lại khi thoát.

## Autotune
- Bước theo `at_steps` (1 P/D, 2 feedforward, 4 I, 8 bộ lọc). FF và I dùng cú giật dạng dốc (60 ms rồi giữ 300 ms): FF tăng 15 %/vòng tới khi trễ ở 50 % ≤ max(10 ms, ¼ thời gian lên của bước P/D), giảm nếu thêm > 6 % vọt lố; tăng mà trễ giảm < 1 ms (4 %) thì quay về giá trị trước và chốt (phần trễ còn lại là motor / bộ lọc). I: sai lệch trung bình cuối pha giữ > 4 % → I↑, < −3 % → I↓. Bộ lọc: đo nhiễu gyro thô / sau lọc giữa các cú giật (hoặc 20 s bay treo nếu chỉ chọn 8), mở rộng lọc khi sạch, siết khi nhiễu, đặt dải notch động quanh cộng hưởng; lưu khi disarm.
- `control/autotune.*` (P/D): máy trạng thái SETTLE → TWITCH → RETURN theo từng trục, chạy trong task control (250 Hz) trên tốc độ góc đã lọc. Mỗi nhóm 4 cú giật: vọt lố trung bình, thời gian lên, thời điểm đỉnh, số lần dao động quanh đích → luật chỉnh (dao động: P↓ D↑; vọt lố mà đỉnh còn ở cuối bước: P↑ I↓; vọt lố sớm: D↑ khi D/P < 0,95, kèm P↓ khi D/P < 0,55, ngược lại P↓; không vọt lố: P↑). Hai nhóm liên tiếp trong dải 4–12 % thì sang trục sau (tối đa 20 nhóm).
- Hệ số mới chỉ đổi phần PID của vòng nhanh (`bus::gainsChanged()` → `FastLoop::setGains`), không khởi tạo lại bộ lọc giữa chuyến bay. Lưu khi disarm với công tắc bật (chỉ vá PID vào cấu hình đã lưu), ngược lại trả hệ số cũ.
- Biên độ cú giật mặc định 100 °/s: ở 200 °/s motor gần bão hoà làm đo vọt lố thấp hơn thực tế.

## Cập nhật firmware & sao lưu
- OTA (ESP32): `/api/ota` ghi tuần tự vào ô `ota_x` còn lại (`esp_ota_*`), kiểm tra header ảnh và app descriptor ngay khối đầu, `esp_ota_end` kiểm checksum/SHA; ảnh mới khởi động ở trạng thái *pending verify* (`verifyRollbackLater()` trả `true` thay vì để core Arduino xác nhận ngay). Task telemetry xác nhận ảnh sau 15 s khi vòng nhanh và các task đã chạy; không đạt trong 60 s thì đánh dấu lỗi và khởi động lại về bản cũ; reset/treo trước đó thì bootloader tự quay về.
- Sao lưu = văn bản lệnh CLI (`Cli::diff`), khôi phục = chạy lại từng dòng (`Cli::run`), `save` chỉ thực hiện cuối cùng nếu mọi dòng thành công; lỗi thì trả cấu hình RAM và nạp lại nhiệm vụ/rally từ flash.

## Web configurator (ESP32)
- `src/web/webapi.*`: phần không phụ thuộc phần cứng (JSON tham số/trạng thái, ghi tham số, lệnh) — biên dịch và test trên PC.
- `src/web/credentials.*`: tên/mật khẩu WiFi và mật khẩu web (slot flash `WIFI`, có CRC); `webapi.cpp`: đăng nhập → token 128 bit ngẫu nhiên phần cứng, tối đa 4 phiên, so mật khẩu thời gian hằng, khoá 30 s sau 5 lần sai; tất cả kiểm thử được trên PC.
- `src/web/web_esp32.cpp`: ESP-IDF `esp_http_server` (keep-alive, tối đa 5 kết nối, task ưu tiên 2 trên nhân 0, timeout 3 s), WebSocket `/ws` đẩy trạng thái qua `httpd_queue_work` (không khoá với các handler); route `/`, `/api/params`, `/api/status`, `/api/mission`, `/api/set?ten=gia_tri`, `/api/save`, `/api/cmd?c=calib_acc|calib_mag|calib_gyro|defaults|bb_erase|reboot`, `/api/blackbox`. Mọi thao tác ghi trả 423 khi armed.
- `src/comms/aqlink.*`: configurator qua cổng USB (mọi board). Khung `A7 51 seq len "đường dẫn?query" 00 thân crc16` → `A7 52 seq status total offset len dữ liệu crc16` (CRC-16/CCITT). Cùng các hàm `web::` như server WiFi; tài liệu dài chia trang 2 kB (`o=`), file khôi phục gửi từng mảnh (`o=&n=`). Trả lời không chặn (gửi theo chỗ trống bộ đệm TX), chạy trong task telemetry cùng CLI/MSP/MAVLink; CLI im khi khung đang tới; arm bị chặn 3 s sau khung cuối. ESP32 UART console 921600 baud, bộ đệm RX/TX 2 kB.
- `src/comms/fourway.*`: ESC passthrough, giao thức BLHeli 4-way giống Betaflight (`MSP_SET_PASSTHROUGH` trên cổng USB, rồi khung `2F … crc` ↔ `2E … ack crc`), lớp bootloader ESC (BLB: BootInit "BLHeli", đặt địa chỉ/bộ đệm, đọc/ghi/xoá trang, keep-alive, CRC 0xA001). HAL: `escSerialBegin/Write/Read/End` bit-bang 19200 baud trên chân motor (ESP32: tắt ngắt từng byte, chờ start bit theo lát 1 ms; STM32: ghi thanh ghi GPIO trực tiếp, đếm chu kỳ DWT). Fast loop không ghi motor khi `bus::escPassthrough`.
- Thử motor: `bus::setMotorTest` (chỉ khi disarm, tự hết hạn sau 1 s, chặn arm thêm 2 s); fast loop ghi giá trị thử qua `hal::motorsWrite(…, armed=true)` thay cho đầu ra mixer. `/api/motor`, MSP_SET_MOTOR.
- `src/vtx/*`: SmartAudio 2.1 (dBm), Tramp, MSP_VTX_CONFIG / MSP_SET_VTX_CONFIG; gửi từng khung cách 150 ms khi đổi, gửi lại một lần sau 1 s. `src/app/leds.*`: màu LED theo trạng thái (thuần, có test), `hal::ledStripShow` (RMT 10 MHz).
- Bootloader: `cmd bootloader` / `hal::rebootToBootloader` (ESP32-S3: RTC FORCE_DOWNLOAD_BOOT; STM32: cờ ở RTC BKP19R, kiểm tra ngay đầu `main()` khi xung nhịp còn ở trạng thái reset, remap system memory (F4), nhảy vào ROM: F4 0x1FFF0000, F7 0x1FF00000, H7 0x1FF09800).
- Cấu hình v17 (v16 + `dshot_bidir`, `est_type`, `at_steps`, `vibe_warn`, `launch_*`, `rth_sanity_s`, bảng VTX `vtxt_*`): trường mới chỉ thêm vào cuối trước `crc`; `configMigrate` giữ mọi giá trị của cấu hình ngắn hơn (v15, v16) và lấy mặc định cho phần mới.
- Bảng VTX: `vtxt_powers` mức (0 = bảng có sẵn), mỗi mức mW + giá trị gửi (dBm SmartAudio 2.1 hoặc chỉ số với `vtxt_sa_index`), băng U (`vtx_band 7`) 8 tần số `vtxt_band1..8`. Dải LED STM32: TIM4_CH3 (PB8) PWM 800 kHz, DMA1 Stream6 nạp độ rộng xung từng bit.
- Trang: `src/web/ui/tools.js` (ghép vào trang bởi `embed_web.py`): cảm biến + mô hình 3D (vẽ đa giác theo quaternion, sắp theo độ sâu), thử motor, cổng/VTX/LED, OSD kéo thả, CLI, soạn nhiệm vụ (gửi bằng `/api/restore` các lệnh `mission clear` / `mission add`, được-cả-hoặc-không), preset, trình nạp firmware (ESP32 ROM loader: SLIP, SYNC, READ_REG nhận chip, SPI_ATTACH, SPI_SET_PARAMS, FLASH_DEFL_* nén bằng CompressionStream, SPI_FLASH_MD5; STM32 DfuSe qua WebUSB: bảng sector từ tên interface, 0x41 xoá, 0x21 đặt địa chỉ, khối 2048 B, UPLOAD đọc lại, leave).
- Laptop: `tools/configurator.py` (phục vụ trang + cầu `/sitl` WebSocket ↔ `build/sim serve`, firmware thật chạy thời gian thực với console là stdin/stdout) hoặc mở `build/web/AllQuad-Configurator.html`. Trang tự chọn: mở từ file / `?usb` → màn hình kết nối Web Serial; do ESP32 phục vụ → đăng nhập WiFi.
- `src/web/ui/index.html` + `analysis.js`: giao diện một file, không tài nguyên ngoài. `tools/embed_web.py` (PlatformIO tự chạy trước khi build) ghép, thu gọn an toàn (bỏ chú thích, thụt lề; không đổi tên biến), nén gzip vào `web_ui_data.h` (~29 kB); bản đọc được ở `build/web/index.dev.html`. `/api/status` luôn đủ trường; WebSocket gửi khung rút gọn (`full:false`, bỏ tasks/perf/sensors/log/fw/ssid) 9/10 lần, trang ghép với khung đủ gần nhất.
- `src/web/ui/extras.js` (ghép sau tools.js): **trình thiết lập** 9 bước (hướng FC giải từ 2 tư thế: các hướng lắp là hoán vị có dấu nên nghịch đảo = chuyển vị, chọn hướng làm "phẳng" ≈ (0,0,−g) và "mũi lên" ≈ (+g,0,0); accel, compass, baro nhấc 1 m, GPS, kiểm tra cần + công tắc ARM, thử từng motor 2 s), **demo** (Web Worker chạy `allquad.wasm` với shim WASI tối thiểu; `aq_input` / `aq_step(µs)` mỗi 4 ms; LINK kind `demo`), **tile OSM** (Web Mercator quanh nhà, zoom theo tỉ lệ bản đồ, cache 300 ảnh, ghi công), **so sánh blackbox** (ghim chuyến mốc: đáp ứng bước đứt nét + bảng chỉ số), **xuất CSV**, **lịch sử tham số** (localStorage `aq-history`, ghi cũ→mới khi Áp dụng / khôi phục / mặc định, hoàn tác), bảng EKF.
- WebAssembly: `tools/build_wasm.sh` (wasi-sdk, `-mexec-model=reactor`) biên dịch toàn bộ firmware + mô phỏng (~300 kB). `embed_web.py` nhúng base64 chỉ vào `AllQuad-Configurator.html` và trang của `configurator.py`, không vào bản gzip trên ESP32.
- `analysis.js` dùng chung cho trình duyệt và Node: giải mã log, Welch FFT, đáp ứng bước bằng giải chập Wiener (cách của PID-Analyzer), luật gợi ý.
- Phân vùng `partitions_allquad.csv` (4 MB): 2 ô app 1.5 MB (OTA), 896 KB blackbox thô, 64 KB coredump.

## Thêm cảm biến mới
Viết lớp kế thừa `ImuDriver` / `BaroDriver` / `MagDriver` trong `src/sensors/`, chỉ dùng hàm `hal::`, đổi trục về FRD bằng `toFrd()`, rồi thêm vào danh sách `probe…()`. Cảm biến serial: viết parser thuần trong `serial_sensors.h` và thêm một `PortFn`.

## Blackbox và thẻ SD (v25)

```
logger task (bb_rate_hz, ưu tiên 1)
  Blackbox::fill()        -> bản ghi 94 B từ các topic (attitude, pid, setpoint, motors, rc, nav, battery, status)
  encodeDelta()           -> 0xA6 (mỗi 32 bản ghi một 0xA5 đầy đủ)
  fillEsc() ~50 Hz        -> 0xA7: RPM (hal::motorsErpm / topic esc), nhiệt, áp, dòng
  write() -> backend 1 hal::portWrite | 2 hal::logWrite | 3 sdlog::push   (không bao giờ chờ)
                                                          |
                                     hàng đợi vòng SPSC (32 / 64 / 256 kB, không khoá)
                                                          |
sdcard task (100 Hz, ưu tiên 1)  sdlog::service(): gắn thẻ, mở /AQLOG/AQnnnnn.BBL khi arm,
  ghi 4-16 kB / lần, sdSync mỗi giây, đóng file sau disarm, dọn log cũ (sd_keep), tìm lại thẻ mỗi 3 s
```

- **Khoá thẻ:** khoá thử (atomic) giữa task ghi và trang cấu hình.
  - Trang cấu hình (web task / telemetry task qua aqlink) chỉ liệt kê / đọc / xoá khi không có file đang ghi.
  - Bận thì trả 409; trang báo lại cho người dùng.
- **Khi một khung không vào được hàng đợi** (đầy, không có thẻ):
  - Blackbox ghi một bản ghi đầy đủ ở lần sau, nên trình đọc không bao giờ giải sai.
  - Số byte mất hiện ở `sd` / tab Blackbox.

## STM32 trên STM32Cube (thay cho STM32duino)

- Không dùng framework của PlatformIO (`framework` để trống): `tools/pio_stm32cube.py` thêm cờ FPU (`-mfpu`, `-mfloat-abi=hard`) cho cả trình biên dịch lẫn linker, newlib-nano, `printf` số thực. Thư viện trong `lib/` (HAL ST, CMSIS, FreeRTOS ARM_CM4F, USB device, FatFs), `lib_ldf_mode = off` và liệt kê rõ trong `lib_deps` từng env; các env ESP32 bỏ qua chúng (`lib_ignore`).
- `main()` (`src/hal/stm32/system_stm32.cpp`): kiểm tra yêu cầu vào DFU → bật DWT, cache (H7 I+D, F7 I) → `HAL_Init` → xung nhịp → `app::firmwareSetup()` (tạo task, `vTaskStartScheduler`).
- `HAL_GetTick()` được thay: trước khi timer 1 MHz chạy thì đếm từ DWT, sau đó từ timer (liền mạch). `HAL_InitTick` rỗng: SysTick thuộc FreeRTOS. Mọi timeout của HAL dùng được cả trước lẫn sau khi scheduler chạy.
- `malloc` / `new` → `pvPortMalloc` (heap_4: H7 192 kB, F722 112 kB, F405 / F411 66 kB); `_sbrk` không cấp phát. Lỗi cứng, tràn stack, hết heap, `configASSERT` → khởi động lại chip.
- FreeRTOS: `uxCriticalNesting` khởi tạo 0 (thay vì 0xAAAAAAAA) để vùng găng trước khi scheduler chạy (cấp phát heap, tạo task) trả lại BASEPRI — nếu không, ngắt USB / timer ưu tiên ≥ 5 bị chặn tới khi scheduler chạy.
- Ưu tiên ngắt (0 = khẩn nhất): UART 1, timer thời gian 2, timer vòng bay 5, EXTI IMU 5, USB 6. Ngắt ≤ 4 không gọi FreeRTOS.
- QEMU (`env:stm32f405_qemu`, `-DAQ_QEMU`): bỏ cấu hình RCC, console ở USART1, ID chip cố định, không kiểm tra ưu tiên ngắt (QEMU mô phỏng 8 bit ưu tiên).

## Độ tin cậy (v2.1)

- **Bản ghi crash (STM32, `hal/stm32/system_stm32.cpp`):** HardFault / MemManage / BusFault / UsageFault (handler naked lấy khung stack MSP/PSP), tràn stack, hết heap, assert → một bản ghi `{loại, PC, LR, CFSR, HFSR, địa chỉ lỗi, thời gian, task / file:dòng, số lần}` trong RAM section `.noinit` (linker: không bị startup xoá), có magic + checksum, H7 clean D-cache trước `NVIC_SystemReset`. Lúc khởi động `crashBoot()` đọc cờ RCC một lần (power-on/brownout → xoá bản ghi; IWDG → tạo bản ghi "watchdog" từ breadcrumb mà tick hook ghi 64 ms/lần: task đang chạy). `resetReason()` trả 4 (PANIC) khi reset do bản ghi vừa tạo. CLI `crash`, JSON `crash`/`rr`/`wdt`, QEMU kiểm chứng (`crash test`).
- **Watchdog IWDG:** LSI /64, reload 3400 ≈ 6,8 s (≥ 4,6 s với LSI nhanh nhất của F4). Bật trong task control khi vòng bay đã chạy > 2000 lần (board không có vòng bay vẫn vào được configurator), chỉ kick khi `bus::fastLoops` tăng. Kick ngay trước mỗi lần xoá sector flash (F4/F7 dừng CPU tới 4 s). Dừng khi debugger halt. ESP32 dùng watchdog của ESP-IDF.
- **Cấu hình A/B (`hal/ee_ab.h`):** ảnh 8 KB + footer `{magic, seq, crc32, ~magic}`; H743 bank 2 sector 7/6, F405 sector 11/10 (firmware ≤ 768 KB). Lưu: ghi bản không dùng, đọc lại so sánh rồi mới chuyển. Ảnh cũ không footer đọc như bản A, lần lưu đầu sang B. Lưu đồng thời bị chặn bằng cờ; blackbox flash H7 nhường bank 2 khi đang lưu. Unit test giả lập mất điện ở từng bước.
- **Bộ đếm UART:** `hal::portStats` (byte, tràn phần cứng, lỗi khung, nhiễu, đầy ring) đếm trong ISR (STM32) / `onReceiveError` (ESP32); receiver: byte, khung đúng, lỗi checksum (`RxParser::counters`). CLI `selftest`, tab Cổng.
- **RPM:** `RpmGate` giữ lại một mẫu lệch xa (giải mã sai) đến khi mẫu sau xác nhận; motor mất telemetry → tần số 0 (notch tắt dần). Đang bay mà motor đã có RPM lúc arm mất telemetry → `WARN_RPM_LOST` (OSD, web).

## Liên kết phụ

- **Console thứ hai (`PortFn::CONFIGURATOR`):** UART mang cùng dòng byte như USB (aqlink, CLI, MAVLink, MSP). Mỗi lúc phục vụ một bên; bên kia chiếm khi bên đang dùng im 1,5 s và không có truyền aqlink. ESC passthrough chỉ qua USB.
- **Cầu WiFi (`src/bridge/`, env `bridge_esp32c3`):** AP + captive portal, phục vụ trang (gzip như ESP32 FC), WebSocket `/link` ↔ UART 460800; `/bridge/info`, `/bridge/wifi`. Trang mở với `?bridge` tự nối và tự nối lại.
- **MSP qua CRSF (`comms/crsf_msp.*`):** khung 0x7A/0x7C từ tay điều khiển (định dạng chunk của Betaflight `msp_shared`, v1 và v2) → `MspLink` thứ hai với reply hook → chunk 0x7B gửi trước telemetry (8 ms/khung). Lệnh MSP2 `0x4151` / `0x4152` / `0x4153` (tham số theo chỉ số / tên / ghi, giá trị ×1000) và `MSP_EEPROM_WRITE` (250). Ghi và lưu chỉ khi disarm.
