# PID cascade và kiểm tra chiều phản hồi

Tài liệu mô tả mixer đang khai báo trong `DroneFC_CRSF_DShot/control_task.ino` và cùng mixer được giữ trong bản ManualAcc after_change. **Không thể kết luận firmware đúng chiều hoặc bay ổn chỉ từ tiếng motor khi dùng tay giữ drone.** Tháo cánh trước mọi kiểm tra trên bàn; không thả drone có cánh đang quay để thử cân bằng.

## 1. Hai tầng điều khiển roll/pitch

```text
Góc đặt − góc IMU → PID góc → tốc độ góc đặt
Tốc độ góc đặt − tốc độ gyro → PID rate → u_roll/u_pitch
Collective + roll + pitch + yaw → mixer → clamp → lệnh ESC
```

- Tầng góc dùng `error=angle_sp-angle`, đầu ra có đơn vị độ/giây.
- Tầng rate dùng `error=rate_sp-rate`, đầu ra dùng đơn vị điều khiển nội bộ, không phải lực đẩy hay RPM.
- Yaw bám tốc độ, không có vòng giữ góc yaw tuyệt đối.
- Ví dụ chỉ xét P: angle_sp0°, angle+20°, KP_TILT10 → rate_sp−200°/s. Nếu gyro0 và KP_RATE0.5 thì u_roll≈−100. I, D, trạng thái trước đó và saturation có thể làm giá trị thực khác.

## 2. Mixer và thứ tự motor hiện tại

```cpp
m1 = throttle + yaw - roll + pitch;
m2 = throttle - yaw - roll - pitch;
m3 = throttle - yaw + roll + pitch;
m4 = throttle + yaw + roll - pitch;
```

| Motor | GPIO | Vị trí theo khai báo | Chiều theo khai báo, nhìn từ trên |
|---|---:|---|---|
| M1 | 27 | Sau phải | CW |
| M2 | 26 | Trước phải | CCW |
| M3 | 25 | Sau trái | CCW |
| M4 | 33 | Trước trái | CW |

```text
                 TRƯỚC
      M4 trái/CW       M2 phải/CCW
                 +
      M3 trái/CCW      M1 phải/CW
                  SAU
```

Với u_roll−100, chưa có pitch/yaw và chưa clamp: M1/M2 bên phải **tăng100**, M3/M4 bên trái **giảm100**. Với u_pitch−100: M1/M3 phía sau **giảm100**, M2/M4 phía trước **tăng100**. Đây là kết quả đại số của mixer, không xác nhận trục IMU/dây thực đã khớp.

## 3. Vì sao không được coi “nâng phía cao hơn” là phản hồi đúng

Trong mô hình thông thường, khi bên phải đang thấp, muốn cân bằng lại cần moment nâng bên phải tương đối so với bên trái. Tăng lực phía đã cao không tự nhiên đưa thân về phẳng; giải thích ngược trong tài liệu cũ là sai.

Chiều hiệu chỉnh còn phụ thuộc dấu góc/rate thật, hướng gắn IMU, vị trí/chiều motor và chiều lực đẩy. Nếu dấu của một khối không khớp, vòng kín có thể thành phản hồi dương. Không sửa dấu một trục chỉ theo tên “roll dương” mà chưa kiểm toàn chuỗi.

Khi giữ frame bằng tay, moment không làm drone quay tự do, sai số có thể kéo dài và I có thể tích lũy. Anti-windup chỉ giới hạn một phần tác động của bão hòa, không phát hiện “đang bị tay giữ”. Motor mạnh dần **không chứng minh đúng hướng**, cũng không có cơ sở hứa khi thả tay sẽ tự cân bằng.

## 4. Kiểm tra bàn trước khi cân nhắc bay

1. Tháo toàn bộ cánh, cố định môi trường thử và có cách cắt nguồn an toàn; không tự chạy motor chỉ để đọc tài liệu.
2. Khi disarmed, dùng debug góc/rate để ghi dấu thực khi nghiêng/quay từng trục. Kiểm góc accel và gyro có cùng chiều biến thiên; giữ yên khác với đang quay.
3. Đối chiếu GPIO với đúng motor thực và chiều quay ESC theo quy trình phần cứng riêng, từng motor một, không dựa vào sơ đồ firmware thay cho kiểm tra dây.
4. Đối chiếu góc đo → sai số → rate đặt → output PID → chênh lệch motor: moment dự kiến phải chống lại sai lệch khi tốc độ góc gần0.
5. Tách ảnh hưởng giới hạn motor/ga thấp. Khi motor đã clamp, không kỳ vọng chênh lệch đầu ra đúng như ví dụ tuyến tính.
6. Nếu dấu/trục/thứ tự chưa rõ, dừng kiểm tra; không gắn cánh hoặc dùng bay thử để xác định chiều phản hồi.

## 5. Phân biệt bản gốc và ManualAcc after_change đã sửa

Công thức mixer giống nhau không có nghĩa luồng an toàn giống nhau. Bản after_change đã sửa reset PID khi disarmed, phản hồi bão hòa motor, health cảm biến và mode DEGRADED; không mặc định các cơ chế đó có trong sketch `DroneFC_CRSF_DShot` ở thư mục này.

Trong after_change: nội bộ800→DShot0; armed900–1600→DShot48–2047. Mapping ANGLE `RC×0.6` chưa được thay đổi hoặc xác nhận bằng đo lực đẩy. ALT_HOLD là điều khiển tốc độ dọc với IMU+baro, không có cảm biến vị trí ngang nên không bảo đảm loại bỏ trôi ngang.

Báo cáo của bản đã sửa: [BAO_CAO_XU_LY_MANUALACC.md](../DroneFC_CRSF_DShot_ManualAcc_after_change/BAO_CAO_XU_LY_MANUALACC.md).
