# AllQuad – Phần cứng & đấu dây

> **An toàn:** luôn tháo cánh quạt khi cấu hình, nạp firmware hoặc thử motor trên bàn.

## Bộ linh kiện khuyến nghị

| Hạng mục | Tốt nhất | Phổ biến / rẻ | Ghi chú |
|---|---|---|---|
| IMU | **BMI088**, **ICM-45686** hoặc **ICM-42688-P** (SPI) | MPU6000, ICM-20602, LSM6DSO | Luôn dùng SPI cho vòng 1–2 kHz. MPU6050 (I2C) chỉ để thử. |
| Baro | **DPS310**, **BMP390**, MS5611 | BMP280, SPL06 | Che baro bằng mút xốp. |
| Compass | **IST8310**, **LIS3MDL** | QMC5883P, QMC5883L, HMC5883L | Đặt trên cột GPS, xa dây nguồn. Bắt buộc cho RTH/MISSION. |
| GPS | **u-blox M10** | M8, NMEA | Tự cấu hình UBX 10 Hz, 115200. |
| Receiver | **ExpressLRS (CRSF)** | SBUS, iBUS | CRSF có telemetry về tay điều khiển. |
| ESC | **AM32 / BLHeli32** + DShot300 + dây telemetry | BLHeli_S / Bluejay, PWM | Dây telemetry ESC bật RPM filter. |
| Optical flow | **Matek 3901-L0X**, MicoAir MTF-01 (MSP) | — | Giữ vị trí trong nhà / không GPS. |
| Khoảng cách | TF-Luna / TFmini-S | — | Hạ cánh êm, đo độ cao mặt đất. |
| OSD | DJI O3 / Walksnail / HDZero (DisplayPort) | MAX7456 / AT7456E (analog) | |

## Thứ tự & chiều quay motor (Betaflight Quad X, props-in)

```
   M4 (trước-trái, CW)     M2 (trước-phải, CCW)
               [ FC  ↑ ]
   M3 (sau-trái, CCW)      M1 (sau-phải, CW)
```
Props-out: `set yaw_reversed 1`.

## Cổng serial

Mỗi board có tối đa 3 UART + USB (+ UDP trên ESP32). Gán chức năng bằng `serial1_fn`, `serial2_fn`, `serial3_fn`, `udp_fn`:

| Mã | Chức năng | Baud mặc định |
|---|---|---|
| 0 | tắt | – |
| 1 | Receiver (CRSF 420000 / SBUS 100000 8E2 đảo / iBUS 115200) | theo giao thức |
| 2 | GPS | tự cấu hình 115200 |
| 3 | MAVLink | 57600 |
| 4 | MSP (+ module optical flow / khoảng cách) | 115200 |
| 5 | MSP DisplayPort (OSD kính số) | 115200 |
| 6 | Telemetry ESC | 115200 |
| 7 | TFmini / TF-Luna | 115200 |
| 8 | Blackbox (OpenLog) | 921600 |
| 9 | VTX SmartAudio (dây SA của VTX nối vào chân **TX** của UART) | 4800 8N2 |
| 10 | VTX Tramp (dây điều khiển vào chân **TX**) | 9600 |

Đổi baud: `serialN_baud` (0 = mặc định). Mặc định: SERIAL1 = receiver, SERIAL2 = GPS, SERIAL3 = MAVLink, UDP = MAVLink.

VTX: `vtx_band` (1 A, 2 B, 3 E, 4 F, 5 R, 6 L), `vtx_channel` 1–8 hoặc `vtx_freq` (MHz), `vtx_power` 1–5 (25/100/200/400/600 mW), `vtx_pit`, `vtx_low_disarm`. Chỉ dùng chân TX (không đọc lại trạng thái VTX). Kính số (DJI / Walksnail / HDZero) đọc và đổi các giá trị này qua MSP trên cổng OSD kính số.

LED: `led_pin` (GPIO, 0 = tắt), `led_count` ≤ 32, `led_brightness`, `led_mode` (0 trạng thái, 1 hướng, 2 một màu), `led_color`. Chỉ ESP32 (RMT); ESP32-S3 dùng DShot thì không còn kênh RMT trống.

ESC passthrough: dây tín hiệu motor nối thẳng tới ESC như bình thường (không cần dây thêm); ESC phải có pin.

## Sơ đồ chân

### ESP32 DevKit (WROOM-32)
| Chức năng | GPIO |
|---|---|
| IMU SPI SCK / MISO / MOSI / CS / CS2 (BMI088 accel) | 18 / 19 / 23 / 5 / – |
| OSD SPI (HSPI) SCK / MISO / MOSI / CS | 14 / 12 / 15 / – (đặt `PIN_OSD_CS` để bật) |
| I2C SDA / SCL | 21 / 22 |
| SERIAL1 RX / TX (UART2) | 16 / 17 |
| SERIAL2 RX / TX (UART1) | 4 / 27 |
| SERIAL3 | – (UART0 là cổng USB) |
| Motor M1 / M2 / M3 / M4 | 13 / 25 / 32 / 33 |
| Pin (ADC) / dòng (ADC) | 34 / 35 |
| LED / Buzzer (qua transistor) | 2 / 26 |

### ESP32-S3 DevKitC-1
| Chức năng | GPIO |
|---|---|
| IMU SPI SCK / MISO / MOSI / CS / CS2 | 12 / 13 / 11 / 10 / 9 |
| OSD SPI SCK / MISO / MOSI | 36 / 37 / 35 |
| I2C SDA / SCL | 8 / 3 |
| SERIAL1 RX / TX | 18 / 17 |
| SERIAL2 RX / TX | 16 / 15 |
| SERIAL3 RX / TX (UART0) | 44 / 43 |
| Motor M1–M4 | 4 / 5 / 6 / 7 |
| Pin / dòng | 1 / 2 |
| LED / Buzzer | 21 / 14 |
| Cổng CLI | USB gốc |

### ESP32-S3 N16R8 (`pio run -e esp32s3_n16r8`, board `BOARD_ESP32S3_N16R8`)
Bản build theo cách đấu của chủ dự án (ICM-45686, receiver CRSF RadioMaster RP4TD, GPS + compass, khung props-out).

| Chức năng | GPIO | Đấu nối |
|---|---:|---|
| ESC "M1" trước trái, CCW | 4 | đầu ra 1 = firmware motor 4 |
| ESC "M2" trước phải, CW | 5 | đầu ra 2 = firmware motor 2 |
| ESC "M3" sau phải, CCW | 6 | đầu ra 3 = firmware motor 1 |
| ESC "M4" sau trái, CW | 7 | đầu ra 4 = firmware motor 3 |
| ICM-45686 CS / MOSI / SCK / MISO | 10 / 11 / 12 / 13 | MOSI → SDI, MISO ← SDO |
| ICM-45686 INT1 | 14 | đấu sẵn, firmware đọc IMU theo timer |
| I2C SDA / SCL (baro, compass) | 8 / 9 | cần trở kéo lên 3,3 V (module GPS-compass thường có sẵn) |
| GPS (SERIAL2) | RX 15 / TX 16 | GPIO15 ← TX của GPS, GPIO16 → RX của GPS |
| CRSF (SERIAL1) | RX 18 / TX 17 | GPIO18 ← TX của RP4TD, GPIO17 → RX của RP4TD |
| Pin | 1 | Pin+ → 3,3 kΩ → GPIO1 → 1 kΩ → GND, `vbat_scale 4.3` |
| Cảm biến dòng (tuỳ chọn) | 2 | |
| LED trạng thái (tuỳ chọn) | 21 | |
| Còi (qua transistor, tuỳ chọn) | 42 | |
| LED RGB có sẵn trên board | 48 | `set led_pin 48`, `set led_count 1` |
| OSD MAX7456 (tuỳ chọn) | SCK 39 / MISO 40 / MOSI 41, CS 38 | sửa `PIN_OSD_CS` thành 38 để bật |

- Không dùng: GPIO 19/20 (USB gốc), 33–37 (PSRAM octal), 43/44 (UART0 → cổng COM, làm cổng console thứ hai), 0/3/45/46 (chân strapping).
- Thứ tự motor: firmware đánh số kiểu Betaflight (1 sau phải, 2 trước phải, 3 sau trái, 4 trước trái). Bản này mặc định `motor1_out 3`, `motor2_out 2`, `motor3_out 4`, `motor4_out 1` để đầu ra 1–4 (GPIO 4–7) khớp nhãn ESC M1–M4. Đổi được trong configurator: tab Motor → **Vị trí & chiều quay motor** (bảng đầu ra từng vị trí, hoặc "Tìm thứ tự tự động": trang quay từng đầu ra, bạn bấm vào vị trí motor đang quay). Chiều quay props-out: `yaw_reversed 1` là mặc định của bản này.
- **Phân áp pin 3,3 kΩ / 1 kΩ (×4,3) chỉ an toàn tới 3S** (12,6 V → 2,9 V ở GPIO1). Pin 4S đầy (16,8 V) cho 3,9 V, vượt giới hạn 3,3 V của chân ESP32: dùng 10 kΩ / 1 kΩ (×11, tới 6S) và `set vbat_scale 11`.
- Cả hai cổng USB-C đều là console (configurator, CLI, MSP, MAVLink): cổng "USB" gốc và cổng "COM" (CH343, 921600 baud).
- Bảng phân vùng 16 MB (`partitions_allquad_16mb.csv`): 2 × 3 MB firmware + 9,9 MB blackbox (~3,5 phút ở 500 Hz).

### STM32F411CEU6 "Black Pill"
| Chức năng | Chân |
|---|---|
| IMU SPI1 SCK / MISO / MOSI / CS / CS2 | PA5 / PA6 / PA7 / PA4 / PB12 |
| OSD SPI2 SCK / MISO / MOSI | PB13 / PB14 / PB15 |
| I2C1 SDA / SCL | PB7 / PB6 |
| SERIAL1 (USART2) RX / TX | PA3 / PA2 |
| SERIAL2 (USART1) RX / TX | PA10 / PA9 |
| Motor M1–M4 | PB4 / PB5 / PB0 / PB1 |
| Pin / dòng | PA0 / PA1 |
| LED / Buzzer | PC13 / PB10 |
| Dải LED WS2812 | PB8 (TIM4_CH3 + DMA1) |
| Cổng CLI | USB-C |

### STM32F405 / STM32F722 (board FC chung, `pio run -e stm32f405` / `stm32f722`)
Bố trí theo kiểu phổ biến của FC F4/F7 (nhiều board thương mại dùng đúng các chân này; board khác thì sửa `src/hal/board.h`).

| Chức năng | Chân |
|---|---|
| IMU SPI1 SCK / MISO / MOSI / CS / CS2 | PA5 / PA6 / PA7 / PA4 / PC4 |
| OSD SPI2 SCK / MISO / MOSI / CS (MAX7456) | PB13 / PB14 / PB15 / PB12 |
| I2C1 SDA / SCL (baro, compass) | PB7 / PB6 |
| SERIAL1 (USART2) RX / TX | PA3 / PA2 |
| SERIAL2 (USART1) RX / TX | PA10 / PA9 |
| SERIAL3 (USART3) RX / TX | PC11 / PC10 |
| Motor M1–M4 | PB4 / PB5 / PB0 / PB1 (TIM3) |
| Pin / dòng | PC1 / PC2 |
| LED / Buzzer | PC14 / PC13 |
| Dải LED WS2812 | PB8 (TIM4_CH3 + DMA1 Stream6) |
| Cổng CLI | USB |

F405: 168 MHz, flash 1 MB (firmware dùng ~20 %). F722: 216 MHz, I-cache + ART bật, D-cache tắt (DMA DShot/LED đọc thẳng RAM), bootloader hệ thống ở 0x1FF00000 (lệnh `bootloader` vào DFU như F4).

### STM32H743VIT6 WeAct MiniSTM32H743 (`pio run -e stm32h743_weact`)
| Chức năng | Chân |
|---|---|
| IMU SPI2 SCK / MISO / MOSI / CS / CS2 | PB13 / PB14 / PB15 / PB12 / PE15 |
| IMU INT1 (data ready, `imu_sync 1`) | **PC4** (EXTI4) |
| I2C2 SDA / SCL (baro, compass) | PB11 / PB10 |
| SERIAL1 USART3 RX / TX (receiver) | PD9 / PD8 |
| SERIAL2 UART7 RX / TX (GPS) | PE7 / PE8 |
| SERIAL3 UART8 RX / TX | PE0 / PE1 |
| SERIAL4 LPUART1 RX / TX | PA10 / PA9 |
| Motor 1–4 | PA0 / PA1 / PA2 / PA3 |
| VBAT (cầu chia 10k/1k) / dòng | PC0 / PC1 |
| LED / còi / LED strip | PE3 / PD14 / PB8 |
| microSD (SDMMC1) | PC8–PC11, PC12 CLK, PD2 CMD (khe trên board) |

INT1 không nối thì đặt `imu_sync 0` (vòng bay chạy theo timer).

### ESP32-C6 DevKitC-1 (`pio run -e esp32c6`)
| Chức năng | GPIO |
|---|---|
| IMU SPI SCK / MISO / MOSI / CS | 20 / 19 / 18 / 21 |
| I2C SDA / SCL | 22 / 23 |
| SERIAL1 (UART1) RX / TX | 4 / 5 |
| SERIAL2 (UART0) RX / TX | 17 / 16 |
| Motor M1–M4 | 10 / 11 / 2 / 3 |
| Pin / dòng | 0 / 1 |
| LED / Buzzer | 15 / 7 |
| Cổng CLI | USB gốc (USB-Serial-JTAG) |

Một nhân RISC-V 160 MHz (mọi task chạy trên nhân 0), chỉ có **2 kênh RMT phát**: không đủ cho 4 motor DShot nên firmware tự dùng **OneShot125** qua LEDC (không có DShot / bidirectional DShot; kênh RMT còn trống dùng cho dải LED). Không có OSD analog (một bus SPI). Dùng module flash 8 MB (bảng phân vùng `partitions_allquad_8mb.csv`). Hợp cho drone nhỏ, bay chậm; drone đua nên dùng ESP32-S3 hoặc STM32.

### ESP32-P4 (`pio run -e esp32p4`)
| Chức năng | GPIO |
|---|---|
| IMU SPI SCK / MISO / MOSI / CS / CS2 | 36 / 37 / 38 / 39 / 40 |
| OSD SPI SCK / MISO / MOSI | 41 / 42 / 43 |
| I2C SDA / SCL | 7 / 8 |
| SERIAL1 RX / TX | 21 / 20 |
| SERIAL2 RX / TX | 23 / 22 |
| Motor M1–M4 | 2 / 3 / 4 / 5 |
| Pin / dòng | 16 / 17 |
| LED / Buzzer | 47 / 48 |

Hai nhân RISC-V 360 MHz, 4 kênh RMT phát (DShot + bidirectional đủ 4 motor). P4 **không có WiFi** trong chip (board dev thường kèm ESP32-C6 làm modem qua SDIO, firmware chưa dùng): cấu hình qua configurator trên laptop bằng USB.

Ràng buộc STM32: 4 chân motor phải **cùng GPIOB** (DShot qua DMA ghi BSRR) và **cùng TIM3** (PWM). TIM1 + DMA2 Stream5 (DShot, và lấy mẫu IDR khi bidirectional DShot), TIM4 + DMA1 Stream6 (dải LED) và TIM5 (đồng hồ task nhanh) được dành riêng. USART của F4 không đảo được tín hiệu: SBUS cần mạch đảo.

Tất cả cảm biến dùng **logic 3.3 V**. Sửa chân trong `src/hal/board.h`; chân `-1` = không dùng.

## Định hướng cảm biến

| `imu_align` / `mag_align` / `flow_align` | Ý nghĩa |
|---|---|
| 0 | CW0 (mặc định, trục X chip hướng về trước) |
| 1 / 2 / 3 | Xoay 90° / 180° / 270° theo chiều kim đồng hồ |
| 4–7 | Như 0–3 nhưng lật úp (chỉ IMU/compass) |

Kiểm tra bằng `status`: nghiêng phải → roll dương; ngóc mũi → pitch dương; xoay theo chiều kim đồng hồ → yaw tăng. Optical flow: cầm drone đứng yên ở độ cao ~1 m rồi dịch về trước — `status` phải báo N tăng (khi mũi hướng Bắc); sai dấu thì chỉnh `flow_flip`.

## Bidirectional DShot (`dshot_bidir 1`)

ESC trả eRPM ngược lại trên chính dây tín hiệu motor ngay sau mỗi khung DShot, nên **RPM filter chạy được mà không cần dây telemetry ESC**. Cần firmware ESC hỗ trợ: Bluejay (BLHeli_S), BLHeli_32, AM32. Bật: `set motor_protocol 2` (DShot300) hoặc 3 (DShot600), `set dshot_bidir 1`, `save`, `reboot`; tab Cảm biến hiện RPM từng motor và tỉ lệ khung lỗi (nên < 1 %).

- ESP32 / ESP32-S3 / ESP32-P4: mỗi motor một kênh RMT phát (open-drain, đảo mức) + một kênh RMT thu trên cùng chân. ESP32-S3 dùng hết kênh RMT nên không còn dải LED.
- STM32: DMA ghi khung, rồi đổi 4 chân sang input và DMA lấy mẫu thanh ghi IDR ở 3,75 lần tốc độ bit; giải mã GCR + checksum trong task nhanh.
- Cần điện trở kéo lên trên dây tín hiệu (đa số ESC có sẵn). ESC không hỗ trợ sẽ không quay: tắt `dshot_bidir`.

## Ghi chú theo chip

- **ICM-45686 / ICM-45605:** đấu SPI như ICM-42688 (cùng chân CS). Bảng thanh ghi khác dòng 426xx; firmware tự nhận (WHO_AM_I 0x72 = 0xE9 / 0xE5). Gyro 2000 °/s, ODR 1.6 kHz (vòng 1 kHz) hoặc 3.2 kHz (`loop_hz 2000`); accel 16 g.
- **QMC5883P:** I2C địa chỉ **0x2C** (khác QMC5883L ở 0x0D), chip ID 0x80, dải ±8 gauss, 100 Hz. Chiều trục trên module GPS thay đổi tuỳ hãng: sau khi lắp, kiểm tra bằng `status` và chỉnh `mag_align`, rồi `calib_mag`.

## Web configurator & blackbox trên ESP32

- Cần module có **flash 4 MB trở lên** (ESP32-C6 cần 8 MB). Bảng phân vùng có hai ô firmware để cập nhật qua WiFi: 4 MB (`partitions_allquad.csv`) = 2 × 1.5 MB firmware + 896 KB blackbox (~19 s ở 500 Hz; từ v2.1 — board đang chạy bảng cũ 1.375 MB phải nạp qua USB một lần, web sẽ từ chối file lớn hơn ô firmware); ESP32-S3 8 MB (`partitions_allquad_8mb.csv`) = 2 × 2 MB + 3.9 MB blackbox. Lần nạp đầu (hoặc khi bảng phân vùng đổi) dùng `pio run -t upload` qua USB; sau đó cập nhật qua web được. Log và firmware cũ trong flash sẽ mất khi đổi bảng phân vùng.
- Xoá log (web hoặc `log erase`) chạy nền khoảng 25 s khi drone đứng yên; trong lúc đó chưa arm được. Ghi blackbox vào flash chạy tốt ở vòng 1 kHz; ở 2 kHz nên dùng OpenLog qua cổng serial.
- WiFi chỉ bật khi `wifi_enable 1` hoặc `web_enable 1`. WiFi làm sụt áp và gây nhiễu: dùng nguồn 3.3 V ổn định (LDO ≥ 500 mA), tụ 10–100 µF gần ESP32, để anten xa dây motor. Bay xong, tắt `web_enable` nếu không cần.
- Phạm vi WiFi AP vài chục mét: dùng để cấu hình và tải log khi drone nằm trên mặt đất, không phải để điều khiển.
- STM32: không có WiFi; blackbox ghi qua OpenLog (cổng `serial8`), mở file log bằng nút *Mở file log* trong giao diện web (chạy được cả khi mở file `build/web/index.html` trực tiếp trên PC).

### Thẻ nhớ SD (blackbox, từ config v25)

Mỗi lần arm ghi ra một file `/AQLOG/AQ00001.BBL`, `AQ00002.BBL`… Cùng định dạng với log flash/OpenLog, nên tab Blackbox và `tools/blackbox_decode.py` mở được trực tiếp.

| Board | Kiểu nối (`sd_mode`) | Chân |
|---|---|---|
| STM32H743 WeAct | SD bus 4-bit (mặc định bật) | khe thẻ có sẵn trên board (SDMMC1) |
| ESP32-S3 N16R8 | SPI, SD bus 1-bit (4-bit cần thêm 1 chân D2) | CLK/SCK 39, CMD/MOSI 38, D0/MISO 40, D3/CS 41, D1 47 |
| ESP32-S3 DevKit | SPI, SD bus 1-bit / 4-bit | CLK 39, CMD 38, D0 40, D1 47, D2 42, D3 41 |
| ESP32, ESP32-P4 | SPI, SD bus | tự chọn chân (`sd_clk`, `sd_cmd`, `sd_d0`… ; 255 = mặc định) |
| ESP32-C6, STM32 F4/F7 | không hỗ trợ | dùng OpenLog qua cổng serial |

- Cài đặt: tab **Hệ thống → Blackbox**. Chọn *Kiểu nối thẻ SD* và *Nơi ghi = Thẻ SD*, lưu rồi khởi động lại. Giao diện chỉ hiện những kiểu nối mà board làm được, kèm sơ đồ đấu dây.
- SPI dùng bus SPI thứ hai của chip, nên chỉ được khi không lắp chip OSD MAX7456 (kính số DJI/Walksnail/HDZero thì không sao). Firmware từ chối chân đang dùng cho motor/cảm biến/UART hoặc chân strapping/PSRAM.
- Vòng bay không bao giờ chờ thẻ. Dữ liệu vào hàng đợi RAM (256 kB PSRAM trên S3 N16R8, 64 kB trên H743, 32 kB ESP32 thường), tác vụ `sdcard` ưu tiên thấp ghi ra thẻ theo khối 4–16 kB. Thẻ khựng vài trăm ms khi tự xoá bên trong cũng không mất dữ liệu.
- Mỗi giây chốt độ dài file xuống thẻ, nên rơi máy/rút pin chỉ mất tối đa khoảng 1 s cuối.
- Thẻ còn dưới 16 MB trống: `sd_keep = 1` tự xoá log cũ nhất (khi disarm), `sd_keep = 0` thì ngừng ghi.
- Thẻ rút ra hoặc lỗi ghi: dừng ghi chuyến đó, tự tìm lại thẻ mỗi 3 s, chuyến sau ghi file mới.
- Xem/tải/xoá log: tab **Blackbox → Thẻ SD** (chỉ khi disarm), hoặc CLI `sd`, `sd ls`, `sd rm AQ00003.BBL`, `sd mount`. Log lớn thì rút thẻ cắm vào máy tính rồi dùng *Mở file log* sẽ nhanh hơn nhiều so với tải qua USB.
- Thẻ nên dùng: microSD 8–32 GB, FAT32, loại A1/A2 hoặc U3. Module thẻ dùng nguồn 3.3 V (module có ổn áp thì 5 V).
