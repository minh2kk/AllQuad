# Kiểm tra trên phần cứng thật

Test tự động (host, simulator, QEMU, giao diện web) không thay được phần cứng thật: nhiễu thật, receiver thật,
ESC thật, GPS ngoài trời. Danh sách dưới đây đi lại **mỗi lần nạp firmware mới** hoặc đổi phần cứng.
Làm theo thứ tự; mục nào hỏng thì dừng, sửa xong mới đi tiếp.

Trang cấu hình có sẵn công cụ cho hầu hết các bước: tab **Trước bay → Hướng dẫn bay lần đầu**.

## A. Trên bàn, chưa cắm pin (chỉ USB)

| # | Kiểm tra | Cách làm | Đạt khi |
|---|---|---|---|
| A1 | Firmware lên đúng | CLI `version` | đúng board, đúng phiên bản, `ô nhớ` đúng |
| A2 | Tự kiểm tra | CLI `selftest` | không có `FAIL`; `WARN` thì đọc từng dòng |
| A3 | Không reset ngầm | tab Hệ thống → Khởi động & sự cố | không có crash mới (`crash clear` sau khi đọc) |
| A4 | Tốc độ CPU | CLI `perf` | loop đủ `loop_hz`, CPU < 60 % |
| A5 | Hướng lắp FC | Hướng dẫn → **Thử nghiêng** | nghiêng phải và ngóc mũi đều qua |
| A6 | Accel / compass | tab Cảm biến | accel đã hiệu chuẩn; compass xoay đủ mọi hướng, hướng Bắc đúng |
| A7 | Baro | tab Cảm biến, nâng FC 1 m | độ cao tăng khoảng 1 m, không nhảy |
| A8 | Sơ đồ chân | CLI `pins`, tab Cổng → Sơ đồ chân | không chân nào trùng; mỗi thiết bị đúng UART |

## B. Trên bàn, cắm pin, **THÁO CÁNH**

| # | Kiểm tra | Cách làm | Đạt khi |
|---|---|---|---|
| B1 | Điện áp | đồng hồ đo ở cọc pin so với FC | lệch < 0.1 V (sai thì Trợ giúp pin) |
| B2 | Dòng khi nghỉ | tab Tổng quan | 0–1.5 A khi motor dừng |
| B3 | Receiver | tab Tay điều khiển → Dò cần | 4 trục đúng kênh, đúng chiều, về giữa 1500 ± 5 |
| B4 | Công tắc | tab Chế độ → Dò | ARM, ANGLE, các chế độ đổi đúng khi gạt |
| B5 | Motor | tab Motor, từng motor 5–10 % | đúng vị trí, đúng chiều quay theo hình |
| B6 | RPM / telemetry ESC | tab Motor khi thử | 4/4 ESC trả RPM (nếu dùng DShot hai chiều) |
| B7 | Failsafe | Hướng dẫn → **Thử failsafe** | FC báo mất sóng khi tắt tay điều khiển, hết khi bật lại |
| B8 | Arm / disarm | arm bằng công tắc (vẫn tháo cánh) | arm được, motor quay idle đều, disarm dừng ngay |
| B9 | Cổng thiết bị | tab Cổng → Lưu lượng & lỗi đường truyền | GPS / optical flow / VTX có lưu lượng và gói hợp lệ |

## C. Lắp cánh, bãi trống

| # | Kiểm tra | Cách làm | Đạt khi |
|---|---|---|---|
| C1 | Cất cánh | ANGLE, ga từ từ, cao 1 m | không lật, không xoay, không trôi mạnh |
| C2 | Lơ lửng 30 s | thả cần | đứng tương đối yên, không rung nghe thấy |
| C3 | Hạ cánh, đo nhiệt | sờ motor | ấm; nóng tay ngay sau 30 s = nhiễu / D quá cao |
| C4 | Log | tab Blackbox → tải log | **Tóm tắt chuyến bay** không có mục đỏ; rung thấp |
| C5 | Giữ độ cao | ALTHOLD | cần ga giữa giữ độ cao ± 0.5 m |
| C6 | GPS | ngoài trời, ≥ 8 vệ tinh | hướng và vị trí đúng trên bản đồ |
| C7 | Giữ vị trí | POSHOLD (GPS hoặc optical flow) | đứng trong vòng 1–2 m, không xoáy vòng (xoáy = compass / flow lệch) |
| C8 | RTH | bật RTH ở 20–30 m | quay đầu đúng hướng, về và hạ cánh |
| C9 | Failsafe khi bay | ở 10 m, tắt tay điều khiển (sẵn sàng bật lại) | làm đúng `fs_action` |

## Sau khi đạt

- Lưu bản sao cấu hình (tab Hệ thống → Sao lưu & khôi phục) kèm phiên bản firmware.
- Ghi lại: board, firmware, pin, cánh, kết quả C4 (rung, ga lơ lửng). Lần nạp sau so lại với số này.

## Còn treo cho DevEBox H743 (cấu hình hiện tại)

- 480 MHz: chạy C1–C4 với bản 480 MHz. Nếu ổn định thì bỏ bản dự phòng 400 MHz (`AllQuad-stm32h743_devebox-400mhz.bin`).
- ELRS trên SERIAL4: B3 và B7.
- Compass QMC5883P: A6 ngoài trời, xa sắt thép; rồi C6–C7.
- MT-01P (chỉ đo khoảng cách): C5 trên nền phẳng và nền cỏ.
