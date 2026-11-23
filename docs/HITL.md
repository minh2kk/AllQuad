# Bay thử trên bàn: hardware in the loop (HITL)

FC thật chạy firmware thật: các task, thời gian thực, bộ lọc, EKF, failsafe, chế độ bay. Còn chiếc drone
được mô phỏng trên máy tính. Máy tính gửi cho FC gyro, gia tốc kế, baro, la bàn, GPS (tuỳ chọn: cảm biến
khoảng cách, kênh RC, pin) qua USB khoảng 500 lần/giây; FC trả lại lệnh motor để máy tính tính chuyển động.

**Motor không quay trong suốt quá trình.** FC giữ đầu ra motor ở mức dừng (DShot 0) từ lúc vào HITL tới khi
khởi động lại. Không cần pin, không cần tháo cánh (lần đầu vẫn nên tháo cho chắc).

## Cần gì

- FC cắm USB vào máy tính, đã nạp firmware AllQuad (bản có HITL: 2.2.0 trở lên).
- Python 3 và `pip install pyserial`.
- Đóng trang cấu hình / Mission Planner: cổng USB chỉ một chương trình dùng được một lúc.

## Chạy

```bat
REM chuyến bay kiểm tra tự động: RC do máy tính gửi (không cần tay điều khiển)
python tools\hitl.py COM5 --auto

REM tự bay bằng tay điều khiển thật (receiver ELRS trên FC vẫn hoạt động)
python tools\hitl.py COM5

REM gió 3 m/s, ghi lại chuyến bay ra CSV
python tools\hitl.py COM5 --wind 3 --log hitl.csv
```

`COM5` là cổng của FC (xem trong Device Manager, hoặc trang cấu hình → Kết nối).

Mỗi giây script in một dòng: chế độ bay, đã arm chưa, độ cao thật và độ cao FC ước lượng, vị trí, góc
roll thật và ước lượng, số khung/giây qua USB, độ trễ, tần số vòng bay của FC.

`--auto` bay một kịch bản cố định rồi chấm điểm:

| Thời điểm | Việc |
|---|---|
| 4 s | arm (AUX1) |
| 4.5–7 s | tăng ga, lên khoảng 4 m |
| 7 s | ALTHOLD (AUX2), ga giữa |
| 10 s | POSHOLD (AUX3) |
| 18 s | đẩy cần pitch 1.5 s |
| 28 s | LAND (AUX4), chờ hạ cánh và tự disarm |

`--auto` dùng cách gán công tắc mặc định của AllQuad (AUX1 arm, AUX2 ANGLE / ALTHOLD, AUX3 POSHOLD,
AUX4 LAND). Nếu bạn đã đổi Chế độ bay, dùng tay điều khiển thật thay cho `--auto`.

Kết quả mẫu (PC build, `--exec "build/sim serve"`, chạy trong CI):

```
[PASS] board answers HITL frames: 500 answers/s, latency 2.0 ms
[PASS] armed and flew: hover height 4.5 m
[PASS] ALTHOLD steady: height spread 0.03 m
[PASS] POSHOLD kept its place (with a stick push): max 3.2 m
[PASS] attitude estimate follows the simulated craft: max error 1.3 deg
[PASS] LAND: down and disarmed: at 39.1 s
6/6 HITL checks passed
```

## Kết thúc

Ctrl+C. Script báo FC thoát HITL và FC tự khởi động lại về cảm biến thật. Nếu rút USB hoặc script dừng
giữa chừng, FC cũng tự khởi động lại sau 1 s (khi đang disarm).

## Khi vào HITL, FC thay đổi những gì

- IMU, baro, la bàn thật được thay bằng dữ liệu từ máy tính. Gyro được hiệu chuẩn lại (drone mô phỏng
  đứng yên lúc đầu). Hướng lắp IMU / la bàn và hiệu chuẩn la bàn không áp dụng (dữ liệu mô phỏng đã ở hệ
  trục thân).
- GPS thật, cảm biến khoảng cách (MT-01P) và optical flow thật bị bỏ qua.
- Receiver thật vẫn dùng, trừ khi máy tính gửi RC (`--auto`, `--rc`).
- Pin lấy từ máy tính (16.4 V giảm dần, dòng theo ga) để không bị chặn arm vì không có pin.
- Cổng USB thuộc về HITL: CLI, trang cấu hình, MAVLink qua USB tạm không dùng được.
- Blackbox vẫn ghi bình thường (thẻ SD): chuyến bay HITL phân tích được trong tab Blackbox như bay thật.

## Nên xem gì

HITL không thay được bay thật: mô hình vật lý là một quad 5 inch chung chung (đổi bằng `--mass`, `--tmax`),
không có rung và độ trễ USB khoảng 2 ms. PID chỉnh cho máy thật có thể khác. Nó trả lời những câu hỏi mà
mô phỏng trên PC không trả lời được:

- Tần số vòng bay thật có giữ được không (`fc loop ... Hz` trong dòng trạng thái, lệnh `perf` sau khi thoát).
- EKF, ALTHOLD, POSHOLD, RTH, LAND, failsafe chạy trên chip thật với thời gian thật.
- Tắt tay điều khiển giữa chừng (không dùng `--rc`): FC phải vào failsafe và hạ cánh.
- Sau chuyến bay: tab Blackbox → khung điều hướng (EKF đặt lại, loại cảm biến, số vệ tinh).
