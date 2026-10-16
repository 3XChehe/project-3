# Từ cảm biến qua PID và mixer đến motor

Tài liệu giải thích **firmware DroneFC_CRSF_DShot_ManualAcc_after_change hiện tại**, đối chiếu mã ngày **2026-10-16**. Mục tiêu là hiểu dữ liệu đi đâu, công thức nào được tính và vì sao motor thay đổi ở mỗi chế độ; không phải hướng dẫn bảo đảm drone đã đủ an toàn để bay.

**Phân biệt ba loại nội dung:** công thức ghi “trong firmware” phản ánh mã đang có; ví dụ tính số là tình huống giả định để giải thích; mô hình vật lý là xấp xỉ, không phải thông số đã đo của drone. Khối lượng, quán tính và hệ số lực đẩy thực chưa được xác định. Không suy ra lực đẩy/RPM chỉ từ lệnh DShot. Mọi kiểm tra phần cứng trên bàn phải tháo toàn bộ cánh.

## 1. Nhìn toàn bộ hệ thống trước khi đi vào công thức

```text
Tay điều khiển → RP3/CRSF → kênh ga/góc/yaw + công tắc ARM/MODE
                                  │
                                  ├── tạo giá trị đặt theo chế độ
                                  │
ICM-20602                         ▼
  gyro → tốc độ góc ───────────→ PID rate roll/pitch/yaw ───────┐
             │                       ▲                        │
             └→ dự đoán góc          │                        │
  accel → góc từ trọng lực → Kalman góc → PID góc roll/pitch ───┘
             │                  │                             │
             └── chiếu gia tốc bằng roll/pitch                 │
                        │                                     │
BMP388                  ▼                                     │
  áp suất/nhiệt → độ cao → Kalman độ cao/tốc độ dọc → PID climb │
                                                    │         │
Ga ANGLE / ramp takeoff / collective DEGRADED ───────┤         │
                                                    ▼         ▼
                                              collective + roll/pitch/yaw
                                                         │
                                                       mixer
                                                         │
                                            clamp + phản hồi chống windup
                                                         │
                                          đổi thang nội bộ sang DShot600
                                                         │
                                                    ESC → motor
                                                         │
                                             lực đẩy/moment làm thân chuyển động
                                                         │
                                              cảm biến đo lại ở vòng sau
```

Đây là **vòng kín**: motor không chạy theo một lần tính duy nhất. Chuyển động thực làm số đo thay đổi, PID tính lại rồi phát lệnh tiếp theo.

Ba điều cần nhớ:
1. **Gyro/accel điều khiển tư thế khi đang hoạt động; riêng WAIT_TAKEOFF giữ bốn motor bằng nhau, không chạy PID.**
2. **Baro chỉ tham gia đường điều khiển ga tự động ở ALT_HOLD/FAILSAFE**, dù KF có thể vẫn được cập nhật trong ANGLE nếu dữ liệu tốt.
3. Tên **ALT_HOLD ở đây là điều khiển tốc độ lên/xuống**, không có vòng ngoài chụp một độ cao rồi điều khiển sai số độ cao; cũng không có giữ vị trí ngang.

## 2. Ký hiệu, đơn vị và ý nghĩa của “P, I, D, K”

| Ký hiệu | Nghĩa | Không được nhầm với |
|---|---|---|
| `setpoint`, hậu tố `_sp` | Giá trị mong muốn | Số đo thực tế |
| `e = setpoint - measurement` | Sai số điều khiển | Sai số cảm biến riêng lẻ |
| `Kp`, `Ki`, `Kd` | Các **hệ số khuếch đại PID** | Ba giá trị P/I/D đã tính ra |
| `P_term`, `I_term`, `D_term` | Ba phần đóng góp vào đầu ra PID | Ma trận covariance `P` của Kalman |
| `K` trong Kalman | Độ lợi hiệu chỉnh ước lượng | Kp/Ki/Kd của bộ điều khiển |
| `Q`, `R` trong Kalman | Mô hình bất định quá trình và phép đo | Gain motor hoặc lực đẩy |
| `roll`, `pitch`, `yaw` | Nghiêng ngang, chúi/ngẩng, xoay hướng | Vị trí ngang x/y |
| `_deg`, `_dps` | Độ; độ/giây | Radian dùng trong công thức cơ học |
| `_cm`, `_cms`, `_g` | cm; cm/s; gia tốc theo bội của g | Mét; lệnh ga |
| `u_throttle`, `u_roll`… | Lệnh theo **đơn vị nội bộ** | Newton, RPM, microsecond hay phần trăm lực đẩy |

Trong tài liệu, `T=u_throttle`, `r=u_roll`, `p=u_pitch`, `y=u_yaw` khi viết mixer. Chữ `p` ở mixer là **lệnh pitch**, không phải áp suất hay P_term.

**PID trả lời:** “Cần thay đổi lệnh điều khiển bao nhiêu để bám mục tiêu?”

**Kalman trả lời:** “Từ dự đoán và số đo, hiện trạng thái có khả năng là bao nhiêu?”

Tăng Kp không làm cảm biến chính xác hơn. Tăng/giảm Kalman K không trực tiếp ra lệnh tăng ga, nhưng thay đổi trạng thái ước lượng đi vào PID nên có thể gián tiếp thay đổi motor.

## 3. Thời gian và thứ tự thực thi

### 3.1 Các nhịp hiện tại

| Thành phần | Nhịp dự kiến | Ý nghĩa |
|---|---:|---|
| `control_task` trên core1 | 500Hz, 2ms | Đọc IMU, ước lượng, chọn trạng thái, PID, mixer, ESC |
| `rc_task` trên core0 | 500Hz | Đọc UART/giải mã CRSF; không có nghĩa receiver luôn gửi frame mới500Hz |
| `baro_task` trên core0 | 100Hz | Kiểm tra baro và xuất debug |
| BMP388 | 50Hz | Khoảng20ms mới có một mẫu áp suất mới |
| KF độ cao khi health tốt | 500Hz | Hiện dùng lại mẫu baro mới nhất còn hợp lệ ở các vòng giữa hai mẫu mới |

`DT_CTRL=0.002s` được dùng cố định trong tích phân/đạo hàm. `loop_period_us` chỉ đo nhịp thực để quan sát, **không thay DT_CTRL trong thuật toán**. Ví dụ vòng thực3ms nhưng vẫn dùng2ms: với rate không đổi, tích phân góc chỉ cộng2/3 lượng cần thiết; cùng một thay đổi sai số qua3ms, D tính chia2ms sẽ lớn gấp1.5 so với chia thời gian thật.

### 3.2 Một lượt `control_step()`

1. Đọc IMU; ước lượng roll/pitch và lấy gyro đã trừ bias.
2. Chiếu gia tốc lên phương thẳng đứng, trừ trọng lực; lỗi IMU/số không hữu hạn làm khóa `control_fault`.
3. Lấy snapshot RC; giải mã công tắc từ dữ liệu thô, lọc các giá trị đặt; ARM mới đồng bộ với lệnh hiện tại.
4. Lấy snapshot baro, xét driver fault, tính hữu hạn và tuổi mẫu. Cập nhật KF cùng readiness: lịch sử nguồn còn mới, trend baro theo cửa sổ và các điều kiện tĩnh trước cất cánh. Sau stale/driver/KF fault, chỉ reacquire/reset khi nguồn collective không còn là CLIMB_PI; phải tích lại lịch sử mới1s, không reset rồi mở PI ngay giữa bay.
5. Cập nhật bằng chứng airborne trên mẫu baro timestamp mới, sau đó xử lý ARM, mất/phục hồi RC, yêu cầu mode và DEGRADED. Bằng chứng được dùng chung trong phiên armed, kể cả khi đang ANGLE.
6. Nếu disarmed: reset PID, gửi DShot0 và kết thúc lượt; không tính PID/mixer.
7. Nếu armed: chọn phase/nguồn collective, chạy mode; kiểm hữu hạn; clamp/slew collective nếu là nguồn dọc, mixer, phản hồi bão hòa, tính lại có giới hạn rồi gửi ESC. Không tiến limiter hai lần trong một tick khi rollback.

Kalman góc giữ `R_acc=9` cố định; không có bước chọn dynamic trust, quarantine hay guard innovation trong luồng hiện tại.

Snapshot bị hụt mutex chỉ giữ dữ liệu cũ; không được biến thành một mẫu mới. RC và baro có tuổi nguồn tối đa200ms. Đây là ngưỡng dữ liệu còn được dùng, không phải bảo đảm mọi số đo trong200ms đều chính xác.

Đối chiếu: `control_step()` trong [control_task.ino](control_task.ino), [cấu hình và setup](DroneFC_CRSF_DShot_ManualAcc_after_change.ino).

### 3.3 Trước vòng điều khiển: firmware khởi động ra sao?

Luồng startup bình thường, không bật `CALIBRATE_ACCEL`:

```text
Serial/SPI → kiểm ID ICM-20602
                  │
                  ├─ sai/vắng → dừng, chưa khởi tạo các task bay
                  │
                  └─ đúng → khởi tạo ESC, phát disarm
                            → khởi tạo UART CRSF
                            → cấu hình IMU, calibration gyro
                            → khởi tạo BMP388, lấy mốc áp suất
                            → khởi tạo KF dọc
                            → tạo mutex và ba task RC/BARO/CTRL
                            → chờ điều kiện ARM hợp lệ
```

Calibration gyro phải xong trước khi vào vòng điều khiển. Nếu baro init không thành công, firmware vẫn có thể chạy ANGLE nhưng không được vào mode dọc. Không có tự quay thử lần lượt motor lúc startup. Nhánh `CALIBRATE_ACCEL` là công cụ đo offset riêng và dừng trước khi tạo các task bay, không phải một mode điều khiển trong không trung.

## 4. ICM-20602: gyro và accel tác động khác nhau

### 4.1 Gyro đo tốc độ quay, không đo trực tiếp góc nghiêng

Firmware quy đổi:

```text
rate_raw_dps = raw_gyro / 16.4
rate_dps     = rate_raw_dps - gyro_bias_dps
```

Ví dụ raw164 và bias0.2°/s: rate đưa vào thuật toán là `164/16.4 - 0.2 = 9.8°/s`.

Nếu giữ rate9.8°/s trong2ms, phần dự đoán góc tăng `9.8 × 0.002 = 0.0196°`. Nếu bias còn sai0.5°/s, tích phân thuần10s có thể lệch5° dù thân không quay; đó là lý do cần accel kéo ước lượng góc về tham chiếu trọng lực.

**Hai đường gyro:** rate đã trừ bias vừa đi vào Kalman góc, vừa đi **trực tiếp vào PID rate**. Vì vậy góc trông mượt không có nghĩa đường D của rate PID đã hết nhiễu; Kalman góc không lọc thay cho mọi tín hiệu gyro.

Bias được lấy từ calibration khởi động, mỗi đợt500mẫu ×2ms cộng overhead. So hai đợt ổn định không chứng minh đứng yên: quay chậm đều vẫn có thể bị nhận làm bias. Bản hiện tại không tự học bias tiếp khi disarmed, tránh học chuyển động thật thành sai lệch; đổi lại chưa có bù drift nhiệt online.

### 4.2 Accel đo lực riêng, không chỉ đo gia tốc chuyển động

Đứng yên trên bàn, accel theo quy ước của firmware phải gần `(0,0,+1g)`, không phải `(0,0,0)`. Giá đỡ đang tác dụng lực chống trọng lực lên cảm biến. Trong rơi tự do lý tưởng, số đo lực riêng có thể gần0 dù vận tốc đang tăng.

Dạng mô hình vật lý tổng quát, khi chọn trục thế giới z hướng lên:

```text
f_body = R_body_to_worldᵀ × (a_world - g_world) + bias + noise
```

`f_body` là số đo lực riêng; `g_world=(0,0,-9.81)m/s²`. Trục cảm biến lắp thật vẫn phải đối chiếu dấu với firmware, không suy ra từ tên biến.

Quy đổi đang dùng:

```text
ax = raw_acc_x / 2048 + 0.0152       (g)
ay = raw_acc_y / 2048 + 0.0141       (g)
az = raw_acc_z / 2048 + 0.0337       (g)
```

Ví dụ rawZ1980: `az = 1980/2048 + 0.0337 ≈ 1.000497g`. Offset cộng/trừ vào số đo, không phải gain PID. Offset giúp bù lệch tĩnh; không tự sửa scale, lệch trục lắp, rung, mọi ảnh hưởng nhiệt hoặc mọi tư thế calibration.

### 4.3 Góc đo từ accel trong firmware

```text
roll_acc  = atan2(ay, √(ax² + az²)) × 180/π
pitch_acc = -atan2(ax, √(ay² + az²)) × 180/π
```

Khi chỉ nghiêng roll và gần như không có gia tốc chuyển động, `ay≈sin(roll)`, `az≈cos(roll)`, nên accel cho thông tin tư thế theo trọng lực. Khi có gia tốc ngang, nó không còn là thước đo góc thuần.

Ví dụ thân phẳng nhưng số đo có thành phần ngang0.2g và đứng1g:

```text
góc accel có thể lệch ≈ atan(0.2) = 11.31°
|a| = √(1² + 0.2²) = 1.019804g
```

Norm chỉ lệch1g rất ít nhưng góc đo đã lệch đáng kể. **Norm gần1g không chứng minh số đo chỉ còn trọng lực hoặc không bị gia tốc ngang.** Kiểm tra norm trong mã chỉ loại vector quá gần0 hoặc số không hữu hạn; không phải heuristic tự thay đổi độ tin cậy accel.

Đối chiếu: `imu_read_raw()`, `imu_kalman_1d()` và các getter có bù bias/offset trong [imu_icm20602.ino](imu_icm20602.ino).

### 4.4 Kalman góc: K phối hợp gyro và accel như thế nào?

Mỗi trục roll/pitch có một bộ lọc riêng. Ký hiệu `V` là variance để không nhầm P_term của PID:

```text
Giai đoạn dự đoán:
  angle_pred = angle_old + rate_gyro × dt
  V_pred     = V_old + dt² × (1°/s)²

Giai đoạn hiệu chỉnh:
  R_acc      = 9                                  (độ², cố định)
  K_angle    = V_pred / (V_pred + R_acc)
  angle_new  = angle_pred + K_angle × (angle_acc - angle_pred)
  V_new      = (1 - K_angle) × V_pred
```

- K lớn: kéo nhiều hơn về góc accel ở lần cập nhật đó.
- K nhỏ: giữ nhiều hơn phần dự đoán bằng gyro, nhưng drift bias có thể tồn tại lâu hơn.
- V thay đổi qua từng lượt nên K vẫn thay đổi dù R cố định. K không phải gain PID hay một phần trăm accel được đặt theo mode.

Ví dụ một lượt: `angle_old=5°`, `rate=10°/s`, `dt=0.002`, `V_old=4độ²`, `angle_acc=4.5°`:

```text
angle_pred = 5 + 10×0.002 = 5.02°
V_pred = 4 + 0.002² = 4.000004độ²
R_acc = 9độ²
K_angle = 4.000004 / (4.000004+9) ≈ 0.307693
angle_new = 5.02 + K_angle×(4.5-5.02) ≈ 4.860000°
```

`IMU_ACC_VAR_DEG2=9` giữ mô hình phép đo gốc ở mọi mode/phase. Firmware không có trust schedule theo ARM/lệnh nghiêng, không đổi R theo norm accel. Giá trị R cố định cũng không bảo đảm số đo accel luôn phản ánh trọng lực thuần; ví dụ gia tốc ngang ở mục4.3 vẫn có thể làm sai góc.

Bộ lọc này không phải AHRS quaternion đầy đủ: ở góc lớn/chuyển động kết hợp, rate trục thân không đồng nhất hoàn toàn với đạo hàm góc Euler. Không suy ra khả năng giữ tư thế ở mọi góc từ công thức một chiều này.

## 5. Từ IMU và baro thành tốc độ dọc đưa vào PID climb

### 5.1 Trừ trọng lực đúng hướng, không chỉ lấy az−1

Trong `control_step()` của [control_task.ino](control_task.ino), với `φ=roll`, `θ=pitch`, đổi độ sang radian trước sin/cos:

```text
acc_earth_z_g = -sinθ × ax
                + cosθ × sinφ × ay
                + cosθ × cosφ × az
                - 1

a_z = acc_earth_z_g × 981                         (cm/s²)
```

Khi phẳng, công thức về `az−1`. Khi nghiêng, trọng lực xuất hiện ở nhiều trục, nên cần chiếu cả ax/ay/az trước khi trừ1g. Số đo accel ở đây phải là số đã bù offset.

Ví dụ roll10°, pitch0°, đứng yên lý tưởng, `ax=0`, `ay=sin10°`, `az=cos10°`:

```text
acc_earth_z_g = sin²10° + cos²10° - 1 = 0
```

**Góc sai có thể làm sai cả ga tự động:** nếu thân thật phẳng, accel `(0,0,1)` nhưng ước lượng roll sai+5°:

```text
acc_earth_z_g = cos5° - 1 ≈ -0.0038053g
a_z ≈ -3.7330cm/s²
```

Tích phân thuần1s sẽ tạo sai lệch vận tốc≈−3.733cm/s; KF có baro sẽ hiệu chỉnh nên không được coi con số tích phân thuần này là kết quả thực cuối cùng. Offset dọc còn dư0.02g tương ứng19.62cm/s², cũng đủ ảnh hưởng đáng kể đến ước lượng tốc độ.

Chuỗi ảnh hưởng là: **accel/ước lượng góc sai → roll/pitch sai → trừ trọng lực sai → tốc độ dọc sai → climb PID đổi ga**, ngoài đường tác động trực tiếp đến PID góc.

### 5.2 BMP388: áp suất/nhiệt thành độ cao

Driver đọc raw nhiệt độ và áp suất, dùng hệ số hiệu chỉnh riêng trong chip. Viết gọn đúng dạng đa thức đang dùng, với `d=raw_temp-t1`:

```text
T_c = d×t2 + d²×t3

pressure = (p5 + p6×T_c + p7×T_c² + p8×T_c³)
         + raw_press × (p1 + p2×T_c + p3×T_c² + p4×T_c³)
         + raw_press² × (p9 + p10×T_c)
         + raw_press³ × p11
```

`t1…t3`, `p1…p11` là hệ số cảm biến, **không phải các P/Kp của PID**. Nhiệt độ ở đây dùng bù phép đo áp suất, không đi trực tiếp vào một lệnh motor.

Sau bù:

```text
h_abs = 44330 × [1 - (pressure / 101325)^0.1902949571836346]   (m)
h_rel = h_abs - h_reference                                  (m)
z_baro = 100 × h_rel                                         (cm)
```

`h_reference` là trung bình50 mẫu hợp lệ lúc startup. Đây là mốc áp suất tương đối, không phải đo khoảng cách thật từ drone tới mặt đất. Gần101325Pa, giảm1Pa tương ứng tăng khoảng0.08325m; giảm12Pa cho gần0.9991m theo công thức này. Gió, luồng cánh, nhiệt hoặc thời tiết làm đổi áp suất cũng có thể bị hiểu là đổi độ cao.

BMP388 đang đặt50Hz, pressure OSRx8, temperature OSRx1, IIR3. Lọc/oversampling giảm nhiễu nhưng thêm độ trễ; không có nghĩa500 lần điều khiển/giây nhận500 mẫu áp suất độc lập.

Đối chiếu: `baro_compensate_temp()`, `baro_compensate_press()`, `baro_press_to_altitude_m()`, `baro_update()` và `baro_measure_reference()` trong [baro_bmp388.ino](baro_bmp388.ino).

### 5.3 Kalman độ cao có hai trạng thái, khác Kalman góc

Trạng thái và các ma trận trong [altitude_kf.ino](altitude_kf.ino):

```text
x = [z, vz]ᵀ                 z:cm; vz:cm/s
F = [1, dt; 0, 1]
G = [0.5×dt²; dt]
H = [1, 0]
Q = G×Gᵀ×4²
R = 30² = 900                (cm²)
```

Với dt0.002:

```text
G = [0.000002; 0.002]
Q = [6.4e-11, 6.4e-8;
     6.4e-8,  6.4e-5]
```

Thuật toán:

```text
Dự đoán:
  x_pred = F×x_old + G×a_z
  P_pred = F×P_old×Fᵀ + Q

Hiệu chỉnh bằng độ cao baro:
  innovation = z_baro - H×x_pred
  S = H×P_pred×Hᵀ + R
  K = P_pred×Hᵀ / S                       K có hai phần tử
  x_new = x_pred + K×innovation
  P_new = (Identity - K×H)×P_pred
```

Phần tử K thứ nhất hiệu chỉnh z, phần tử thứ hai hiệu chỉnh vz. Với innovation tính bằng cm, phần tử thứ nhất không có đơn vị, phần tử thứ hai có đơn vị1/s để tạo hiệu chỉnh cm/s. Vì vậy không được coi mọi phần tử của Kalman K đều là một tỷ lệ phần trăm giống nhau. Dù baro không đo trực tiếp vận tốc, tương quan giữa z/vz trong P cho phép sai lệch độ cao tác động vào ước lượng tốc độ. P ở đây là **ma trận covariance**, không phải lực điều khiển P_term.

Ví dụ dự đoán, **chưa hiệu chỉnh baro**: `z=100cm`, `vz=20cm/s`, `a_z=50cm/s²`:

```text
z_pred  = 100 + 20×0.002 + 0.5×50×0.002² = 100.0401cm
vz_pred = 20 + 50×0.002 = 20.1cm/s
```

- IMU cung cấp biến thiên nhanh; chỉ tích phân IMU sẽ trôi do bias.
- Baro cung cấp tham chiếu độ cao chậm/nhiễu hơn, giúp sửa trôi; không đơn giản là lấy vi phân hai mẫu baro rồi đưa vào PID.
- R lớn hơn thường làm hiệu chỉnh baro yếu hơn khi P giữ nguyên. Q lớn hơn làm bất định dự đoán tăng nhanh hơn, thường làm K tăng; đáp ứng thực phụ thuộc toàn bộ mô hình và nhịp gọi.
- Hiện vẫn correct500Hz với mẫu baro mới nhất còn tốt để giữ bộ Q/R hiện có. Mẫu lặp không trở thành phép đo độc lập chỉ vì gọi lại KF; đây là giới hạn thiết kế đang giữ, không phải khẳng định đúng thống kê. Không tự giảm correction xuống50Hz mà coi là không ảnh hưởng tuning.
- Mẫu quá200ms hoặc driver/KF fault đều xóa lịch sử readiness và yêu cầu reacquire. Chỉ reset khi `collective_source != COLLECTIVE_CLIMB_PI`, đưa z về phép đo và vz0; nếu đang dùng PI phải khóa/rời nguồn đó trước. Sau lỗi trong mode dọc hoặc khi đã confirmed airborne, khóa `vertical_inhibited`; cần lịch sử mới1s và chuyển MODE rõ ràng qua ANGLE rồi ALT để mở PI lại. ANGLE đang được pilot điều khiển có thể vẫn giữ đường ga thủ công dù latch này đã khóa đường PI. Không bỏ qua driver fault chỉ vì chưa quá200ms. Số0 là **trạng thái khởi tạo lại**, không chứng minh drone đã đứng yên.
- `healthy` là đủ lịch sử nguồn còn mới/KF hữu hạn, không phải đánh giá động học bảo đảm cảm biến đúng. `ready` bổ sung điều kiện tĩnh trước cất cánh. Hiện không có jump guard, innovation gate, so chênh vận tốc KF–baro hoặc tự học/sửa bias dọc.

## 6. PID: mỗi hệ số làm gì và phép tính đang thực hiện

### 6.1 Công thức liên tục và rời rạc

Dạng quen thuộc:

```text
u(t) = Kp×e(t) + Ki×∫e(t)dt + Kd×de(t)/dt
```

Trong `pid_step()`, với dt0.002:

```text
P_term = Kp × e[n]
D_term = Kd × (e[n] - e[n-1]) / dt
I_try  = clamp(I_old + Ki × (e[n] + e[n-1]) × dt/2, -I_limit, +I_limit)
u_raw  = P_term + D_term + I_try
```

I dùng quy tắc hình thang. `integral` đang lưu **phần đóng góp I đã nhân Ki**, không phải tổng sai số thô để lại nhân Ki lần nữa.

Nếu u_raw vượt giới hạn trên và I_try tăng, hoặc vượt dưới và I_try giảm, bước I đó bị từ chối. Sau đó u bị chặn ở ±u_limit. Đây là conditional integration. Giới hạn I và giới hạn tổng output là hai việc khác nhau.

Mẫu đầu sau reset đặt sai số trước bằng sai số hiện tại cho tích phân và bỏ D:

```text
D_first = 0
ΔI_first = Ki × e_first × dt
```

Điều này loại cú D do history cũ; **không loại mọi D-kick do setpoint đổi ở các lượt sau**. Firmware lấy D theo sai số, không dùng một bộ D theo measurement riêng.

### 6.2 P — phản ứng với sai số đang có

P giống “mức thúc đẩy để quay về mục tiêu”. Sai số gấp đôi thì phần P gấp đôi, trước saturation.

Ví dụ rate error20°/s, Kp0.5 → P10 đơn vị motor nội bộ. Kp lớn có thể bám mạnh/nhanh hơn, nhưng quá lớn cùng độ trễ/quán tính có thể gây vượt lố hoặc dao động. Kp không tự biết motor nào cần tăng; mixer quyết định phân bố theo trục.

Nếu cần một moment cố định để chống lệch trọng tâm/gió mà chỉ dùng P, thường phải còn sai số để tạo moment đó; I có thể duy trì phần bù khi sai số tiến gần0.

### 6.3 I — nhớ sai số kéo dài, không phải “chữa mọi loại trôi”

Nếu error20°/s gần như không đổi, Ki_rate1.4:

```text
ΔI mỗi lượt = 1.4×20×0.002 = 0.056
Sau1s: ΔI tổng ≈ 28 đơn vị, nếu không chạm giới hạn/anti-windup/bleed.
```

I giúp chống nhiễu kéo dài và duy trì moment hoặc ga nền. I quá lớn có thể tạo lệnh tồn dư khi sai số đã đổi dấu, làm hồi phục chậm/vượt lố. Giữ thân bằng tay vẫn có thể tích I trước khi saturation rõ rệt; anti-windup không phải cảm biến phát hiện đang bị giữ.

Trong climb, I còn duy trì một phần collective cần để treo. Nó không có đơn vị lực đẩy và không tự đo được ga hover. Đặc biệt, **I của rate không tích phân sai số vị trí ngang**, nên tăng Ki_rate không biến drone thành giữ vị trí GPS.

### 6.4 D — phản ứng với tốc độ thay đổi sai số

Ví dụ rate error từ20 xuống18°/s trong2ms:

```text
D = 0.03 × (18-20) / 0.002 = -30 đơn vị
```

Trong khi P vẫn dương, D âm có thể giảm lệnh khi đang tiến gần mục tiêu, hỗ trợ hãm. Nhưng khi mục tiêu thay đổi đột ngột, D cũng phản ứng với chính thay đổi mục tiêu. Nhiễu gyro chỉ1°/s giữa hai lượt có thể tạo |D|15 đơn vị với Kd0.03.

D quá lớn có thể khuếch đại nhiễu/rung và tăng nóng motor. Firmware đặt DLPF gyro `0x06`, accel `0x05`: lọc nội cảm biến giảm thành phần tần số cao nhưng cũng tạo độ trễ. Đây là mã cấu hình thanh ghi, không phải Kd hoặc thời gian trễ tính bằng ms. DLPF không có nghĩa D đã được tune/lọc riêng đầy đủ. Không xem D như “càng lớn càng ổn định”.

### 6.5 Các gain và giới hạn thực tế

| Vòng | Sai số đầu vào | Đầu ra | Kp | Ki | Kd | Giới hạn |
|---|---|---|---:|---:|---:|---|
| Góc roll/pitch | độ | tốc độ đặt°/s | 10 | 0 | 0 | I±200; u±400 |
| Rate roll/pitch | °/s | lệnh trục nội bộ | 0.5 | 1.4 | 0.03 | I±200; u±400 |
| Rate yaw | °/s | lệnh yaw nội bộ | 1 | 10 | 0 | I±100; u±100 |
| Climb | cm/s | phần cộng vào ga800 | 3 | 15 | 0 | P±200; I0…700; xét thêm ga900…1600 |

Vì Ki/Kd vòng góc bằng0, tầng góc hiện **thực chất là P controller**. Climb và yaw hiện là PI, dù các hàm có dạng PID. Các con số Kp ở những vòng khác đơn vị không thể so trực tiếp để kết luận vòng nào “mạnh hơn”.

Ví dụ đơn vị: Kp_tilt có `(°/s)/°=1/s`; Kp_rate có `đơn_vị_motor/(°/s)`; Ki_rate có `đơn_vị_motor/°`; Kp_climb có `đơn_vị_motor/(cm/s)`.

Trong ANGLE, **chỉ khi chưa `airborne_confirmed`** và `RC_throttle×0.6 <830`, I rate và tilt mới nhân0.9 mỗi lượt trước khi chạy PID; climb I không bị bleed theo hàm này. Nếu chỉ xét phép nhân,10 lượt còn `0.9^10≈0.3487` phần I cũ, nhưng lượt PID sau đó vẫn có thể tích thêm I mới. Sau FLIGHT, ga thấp không xóa/bleed I theo điều kiện này, không đưa phase về WAIT và không tự disarm. Không nhầm bleed trước airborne với reset đầy đủ khi disarmed.

Đối chiếu: `pid_step()` và `pid_step_climb()` trong [pid.ino](pid.ino), [cấu hình gain](DroneFC_CRSF_DShot_ManualAcc_after_change.ino).

## 7. RC tạo mục tiêu như thế nào?

CRSF raw khoảng172…1811 được đổi về kênh nội bộ khoảng990…2010, giữa1500. Đây không phải xung PWM thực đang gửi ESC.

Trước lọc:

```text
roll_target  = clamp((CH_ROLL-1500)×20/500, -20,20)       (độ)
pitch_target = clamp((CH_PITCH-1500)×20/500, -20,20)       (độ)
yaw_target   = clamp(-(CH_YAW-1500)×100/500, -100,100)    (°/s)
x = CH_THROTTLE-1500
climb_target = sign(x)×clamp((|x|-40)/460, 0,1)×100       (cm/s)
```

Mỗi lượt RC hợp lệ, firmware lọc:

```text
sp_new = sp_old + 0.05 × (target - sp_old)
```

Ví dụ CH_ROLL1750 → target10°. Từ sp0: lượt đầu0.5°, lượt hai0.975°. Với nhịp2ms đều, hằng số thời gian tương đương≈39ms, đạt95% bước sau≈117ms. Đây là làm mượt lệnh, không phải độ trễ duy nhất của toàn hệ thống.

**Sau bổ sung bảo vệ ALT:** setpoint góc/yaw/climb đã clamp đúng±20°/±100°/s/±100cm/s dù RC đạt990/2010. Ga1460–1540 có climb_target0; CH_THROTTLE1750 cho `(250−40)/460×100≈45,65cm/s`. Deadband chỉ thuộc đường dọc, không đổi mapping ga ANGLE. Disarm xóa LPF; ARM mới đồng bộ từ RC hiện tại nên không mang lệnh lớn của lần trước.

Trong ANGLE, ga dùng trực tiếp `CH_THROTTLE×0.6`, không qua LPF0.05 của climb. Cùng một cần ga nhưng ý nghĩa khác nhau giữa ANGLE và ALT_HOLD.

Mapping MODE hiện tại: **SW0=ANGLE, SW1=ANGLE, SW2=ALT_HOLD**. Không có nấc công tắc yêu cầu FAILSAFE; FAILSAFE chỉ được tự chọn khi mất RC và đủ điều kiện airborne/dữ liệu dọc/latch.

Đối chiếu: `control_*_target()`, `control_sync_commands()` và `rc_fetch_and_map()` trong [control_task.ino](control_task.ino).

## 8. Luồng và tính toán của từng chế độ

### 8.1 DISARMED — vẫn quan sát cảm biến, không chạy điều khiển motor

IMU/health/KF có thể vẫn cập nhật, nhưng sau kiểm tra trạng thái:

```text
reset toàn bộ PID
u_roll=u_pitch=u_yaw=0
u_throttle=800
motor1…4=800 → DShot0
```

Không có chuyện PID cố cân bằng bằng motor khi disarmed. Ngoài PID, firmware còn xóa setpoint lọc, phase/ramp, airborne evidence và các latch của chuyến cũ. ARM cần RC hợp lệ, công tắc OFF→ON và ga<1150. **Cho ARM trực tiếp ALT ở SW2**, nhưng phải có readiness tĩnh tốt1s và bắt đầu WAIT_TAKEOFF, không vào PI ngay. Bị từ chối không tự arm khi readiness hồi; cần OFF→ON mới. Boot/reconnect với ARM đã ON không tự arm; control_fault đã khóa thì không re-arm cho tới khởi động lại và kiểm tra lỗi. Khi RC hợp lệ báo ARM OFF, disarm có ưu tiên trước mọi xử lý mode/takeover.

### 8.2 ANGLE — giữ góc, pilot quyết định collective

```text
RC roll/pitch → góc đặt
IMU → góc đo → P góc → tốc độ góc đặt
Gyro → tốc độ góc đo → PID rate → r,p
RC yaw + gyro yaw → PI yaw → y
RC throttle ×0.6 → T
T,r,p,y → mixer → motor
```

Hai tầng roll/pitch:

```text
rate_sp_roll = PID_tilt(roll_sp - roll_measured)
r = PID_rate(rate_sp_roll - gyro_roll)
```

**Ví dụ trọn một lượt**, giả sử đang armed, angle setpoint đã về0, gyro roll0, ước lượng roll+5°, pitch/yaw và các lỗi khác0; CH_THROTTLE2000. Rate PID vừa reset nên D0, I ban đầu0; chưa saturation:

```text
T = 2000×0.6 = 1200
error_angle = 0-5 = -5°
rate_sp_roll = 10×(-5) = -50°/s
error_rate = -50-0 = -50°/s
P_rate = 0.5×(-50) = -25
ΔI_rate = 1.4×(-50)×0.002 = -0.14
r = -25.14; p=0; y=0
```

Mixer sẽ cho:

```text
M1=M2=1200-(-25.14)=1225.14 → int1225 → DShot976
M3=M4=1200+(-25.14)=1174.86 → int1174 → DShot830
```

Theo vị trí khai báo, bên phải tăng tương đối so với bên trái. Đó là kết quả dấu của code; phải kiểm dấu góc đo và dây thật để xác nhận moment chống sai lệch. Không suy ra “nghiêng phải chắc chắn là roll dương” nếu chưa kiểm IMU lắp thật.

**Ảnh hưởng baro:** KF vẫn có thể chạy, nhưng kết quả vz/z không đi vào công thức T của ANGLE. Baro nhiễu không trực tiếp làm climb PID đổi ga trong mode này. IMU sai vẫn ảnh hưởng PID góc/rate.

**Hạn chế vật lý:** khi thân nghiêng, cùng tổng lực đẩy cho thành phần nâng thẳng đứng nhỏ hơn. ANGLE hiện không có bù ga theo `1/(cosφcosθ)`; pilot phải điều khiển ga. Dải RC990…2010 cho T594…1206 trước mixer; khi armed motor bị chặn900…1600. T thấp hơn900 không có nghĩa đã disarm, vì clamp armed vẫn nâng motor lên tối thiểu900.

### 8.3 ALT_HOLD — chờ/ramp cất cánh, sau đó pilot đặt tốc độ dọc

**ALT không còn luôn gọi climb PI ngay khi armed:**

| Pha | Collective và PID |
|---|---|
| WAIT_TAKEOFF | Arm mới trực tiếp ALT: bốn motor900, I toàn bộ0, không tính PID. Motor vẫn quay ở mức armed. |
| TAKEOFF | Ga>1580 liên tục200ms, readiness đạt: dùng cần ga tăng/giảm collective tương đối. Attitude chỉ PD, I0; không có climb PI để tích lũy khi bị giữ trên nền. |
| FLIGHT | Mẫu baro mới cao hơn mốc takeoff>30cm, trend>0 và healthy, duy trì bằng chứng300ms: latch airborne/FLIGHT. Nếu đang TAKEOFF ALT thì ngay lượt xác nhận seed climb PI, không tích thêm climb I trong lượt đó; ANGLE vẫn là ANGLE cho tới yêu cầu ALT hợp lệ. Phase latch đến disarm; hạ ga không quay WAIT. |

Readiness tĩnh dùng lịch sử tốt1s, góc<10°, mỗi gyro rate<5°/s, `|mean(a_z)|<0,01g`, baro peak-to-peak<20cm trong cửa sổ ít nhất500ms có ít nhất3 mẫu. `healthy` chỉ cần lịch sử nguồn mới/KF hữu hạn đủ1s, không bắt buộc còn đứng tĩnh khi bay; `ready` cần thêm điều kiện tĩnh. Trend và thời gian bằng chứng airborne dựa trên timestamp baro mới, không đếm lại snapshot500Hz; gia tốc/góc/rate của cửa sổ vẫn được thu mỗi tick điều khiển. Đây không phải kiểm tra động học bảo đảm cảm biến đúng hoặc xác nhận drone đang ở trên đất.

Trong TAKEOFF, `dT/dt = c×80` nếu c>0 và `c×150` nếu c<0, với `c=climb_target/100`; giữa deadband c0 giữ T. Ví dụ c0,5 trong1s cho thêm40 đơn vị, không phải tăng40% lực đẩy. Nếu chưa nhấc lên thì không tự bật PI theo timeout; pilot phải giảm/disarm.

**Mọi yêu cầu ANGLE→ALT trước khi có airborne evidence đều bị từ chối và vẫn ANGLE**, kể cả phiên ban đầu ARM trực tiếp ALT rồi đã chuyển sang ANGLE. Bằng chứng cất cánh được theo dõi cả trong ANGLE và được dùng chung xuyên chuyển mode trong cùng phiên armed; mốc được lấy khi ARM, hoặc khi bắt đầu TAKEOFF ALT. Khi đã confirmed airborne, yêu cầu ANGLE→ALT hợp lệ seed PI ở FLIGHT, không qua WAIT.

Mất RC ở WAIT của phiên ARM trực tiếp ALT thì disarm. Mất RC ở TAKEOFF trước airborne chuyển DEGRADED, giữ collective khi chưa lấy lại quyền, attitude PD/I0 và latch `launch_aborted` đến DISARM. Dù RC hồi và đã xác nhận MODE, preflight DEGRADED chỉ cho giữ/giảm collective, không cho tăng hoặc tự thử lại takeoff.

**Luồng dưới đây chỉ là FLIGHT:**

```text
RC throttle → climb_sp_cms
IMU accel + góc → a_z ─┐
Baro → z_baro ─────────┴→ KF → vz_est
climb_sp - vz_est → PI climb → T=800+output_climb
RC roll/pitch/yaw + IMU → r,p,y như ANGLE
T,r,p,y → mixer → motor
```

Tính climb trong firmware:

```text
e_v = climb_sp - vz_est
P_climb = clamp(3×e_v, -200, +200)
I_try = clamp(I_old + 15×(e_v+e_prev)×0.002/2, 0, 700)
T_raw = 800 + P_climb + I_try                  (Kd hiện bằng0)
```

Chỉ nhận bước I nếu nó không đẩy tổng ga sâu thêm ngoài900…1600; sau đó xét clamp/slew collective và saturation motor ở mục10. Nguồn dọc tăng tối đa0,16/giảm0,30 đơn vị mỗi tick2ms. Giới hạn này không giới hạn riêng từng motor khi P/D tư thế đổi.

Ví dụ **một lượt bình thường, không phải lượt vừa vào mode**: climb_sp20cm/s, vz_est0, e_prev20, I_old300:

```text
e_v=20
P=60
ΔI_candidate=15×20×0.002=0.60
I_try=300.60
T_requested=800+60+300.60=1160.60
```

Nếu collective tick trước1160 và r=p=y0, slew chỉ cho1160,16: residual−0,44. Feedback giữ phần I khả thi `I_new=300,60−0,44=300,16`, rồi tính lại T1160,16 từ cùng mốc limiter, không thêm một bước slew thứ hai. Mỗi motor sau đổi int là1160 → DShot790. Không thể từ DShot790 kết luận lực đẩy đủ để lên20cm/s; đó còn phụ thuộc pin, motor, cánh, khối lượng và phản hồi các lượt sau.

Cần ga giữa1500 → climb_sp tiến về0cm/s. Bộ điều khiển cố đưa **vận tốc dọc** về0, nhưng không tính `z_target-z_est`. Nếu vz_est0 mà drone đã lệch khỏi độ cao ban đầu, không có vòng vị trí kéo về đúng độ cao đó.

**Vai trò I nền:** khi vz_est=climb_sp0 thì P0. Muốn T1200, cần I400 vì `800+400=1200`. 800 chỉ là mốc lệnh nội bộ, không phải ga hover vật lý. Ki_climb15 giúp I điều chỉnh theo sai số kéo dài, không tự cung cấp một giá trị hover đã đo sẵn.

Nếu KF báo sai vz_est−30cm/s trong khi mục tiêu0, P tăng90 và I có xu hướng tăng dù drone không thực đang hạ. Firmware hiện chỉ xét freshness/driver fault, tính hữu hạn/KF và lịch sử readiness; **không có guard nhảy độ cao, innovation gate, so chênh vận tốc KF–trend baro hay tự sửa bias**. Vì vậy sai lệch vẫn hữu hạn và được publish mới có thể không bị phát hiện. Trend cửa sổ dùng cho bằng chứng cất cánh, không phải một phép đo vận tốc tức thời độc lập để bảo đảm KF đúng. Không tự reset I/disarm khi đang FLIGHT vì ga thấp hoặc vz gần0 có thể là lệnh hạ/hover thật.

**Sau đặt xuống phải ARM OFF trước lần thử tiếp theo, không cần reset ESP32.** Nếu vẫn armed, I tư thế có thể còn; firmware không có cảm biến xác nhận tiếp đất để xóa I chắc chắn mà không ảnh hưởng hover.

### 8.4 FAILSAFE — đặt mục tiêu hạ và mục tiêu tư thế phẳng

**Chỉ tự chọn khi mất RC**, đã `airborne_confirmed`, vertical healthy, không `vertical_inhibited` và không `launch_aborted`. SW2 là ALT_HOLD, không phải FAILSAFE. Khi mất RC mà chưa có bằng chứng airborne hoặc không đủ điều kiện dọc thì dùng DEGRADED; riêng WAIT của phiên ARM trực tiếp ALT thì disarm. Không dùng việc “đã armed/chọn mode” thay cho bằng chứng airborne:


```text
climb_sp = -60cm/s
roll_sp = 0°; pitch_sp = 0°; yaw_rate_sp = 0°/s
IMU+baro → KF → PI climb → T
IMU góc/rate → cascade tư thế → r,p,y
```

Ví dụ lượt bình thường: vz_est−20cm/s, e_prev−40, I_old450:

```text
e_v = -60-(-20) = -40cm/s
P = 3×(-40) = -120
ΔI_candidate = 15×(-40)×0.002 = -1.20
I_try = 448.80
T_requested = 800-120+448.80 = 1128.80
```

Nếu collective tick trước1130 và tư thế/lỗi yaw đều0, slew chỉ cho giảm0,30 xuống1129,70. Residual+0,90 làm feedback giữ phần giảm I khả thi: `I_new=448,80+0,90=449,70`, output tính lại T1129,70, motor int1129 → DShot701. Lệnh giảm so với khi cùng I mà P0, để yêu cầu hạ nhanh hơn. Khi vz_est−100, e_v+40 nên PID có xu hướng **tăng** ga để giảm tốc độ rơi về−60; failsafe không chỉ là giảm ga liên tục.

Yaw rate đặt0 là yêu cầu ngừng quay, **không khóa một hướng la bàn tuyệt đối**. Không có cảm biến tiếp đất nên mode không tự disarm dựa vào một mẫu vz gần0/ga thấp sau2s. Motor có thể tiếp tục quay khi đã xuống đất; điều này không phải hạ cánh an toàn đã kiểm chứng.

### 8.5 DEGRADED — ngừng PI dọc, pilot điều chỉnh collective tương đối

Vào khi mode dọc mất dữ liệu hợp lệ/lịch sử healthy, hoặc mất RC mà không đủ điều kiện dùng PI. Seed từ collective giới hạn cuối; nếu nguồn trước là ANGLE trực tiếp thì seed bằng average motor cuối sau clamp. Không tiếp tục dùng climb I để quyết định ga.

```text
c = climb_target/100
rate = c×80 nếu c>0, c×150 nếu c<0
T_next = clamp(T_previous + rate×0.002, 900,1600)
```

RC hợp lệ đã tiếp quản và đã confirmed airborne: giữa deadband giữ T, lên tăng, xuống giảm. Ví dụ T1260, cần thấp hết c−1: sau1s giảm xấp xỉ150 về1110 nếu chưa chạm minimum. Không có sensor dọc đáng tin nên đây **không phải yêu cầu hạ100cm/s**; tốc độ hạ thật chưa biết. RC mất hoặc vừa hồi nhưng chưa đổi MODE xác nhận thì giữ T cuối của DEGRADED: **không nhận cả lệnh ga tăng lẫn giảm**. Trước airborne, sau khi lấy lại quyền chỉ được giữ/giảm, không tăng.

Nếu r−50,p20,y0 và T1260, mixer vẫn cho `[1330,1290,1230,1190]`; giữ collective không phải cố định bốn motor. Khi còn RC đã tiếp quản, tư thế theo pilot, nếu không theo mục tiêu0. Trước FLIGHT, I tư thế vẫn khóa; nếu còn ở WAIT thì giữ cả collective900 và không sinh torque, không cho cần ga tăng thrust. Cần disarm/arm ALT mới để bắt đầu lại quy trình chờ/ramp.

Baro hồi không tự mở PI khi đã latch `vertical_inhibited`; cần lịch sử mới1s, confirmed airborne và thao tác MODE ANGLE rồi ALT. Nếu `launch_aborted` trước airborne thì không chuỗi MODE nào mở lại PI/takeoff trong cùng phiên: phải DISARM. Mất cả RC+baro chỉ giữ collective/cân bằng khi IMU còn tốt, không bảo đảm hạ an toàn. IMU/control output lỗi vẫn khóa fault/disarm, không qua slew.

### 8.6 Chuyển mode không được hiểu như một bước nhảy PID tùy ý

Khi bắt đầu dùng climb PI của FLIGHT ALT_HOLD hoặc FAILSAFE:

```text
T_previous = average bốn lệnh motor cuối sau clamp/đổi int
error_new = setpoint_new - vz_est
P_new = clamp(Kp_climb×error_new, -200, +200)
I_needed = T_previous - 800 - P_new
I_seed = clamp(I_needed, 0, 700)
```

Lượt chuyển vào mode dọc từ mode khác dùng output seed, chưa tích thêm climb; riêng arm trực tiếp ALT dùng WAIT chứ không seed PI. Ví dụ T_previous1200, error20 → P60, I_seed340 → T_raw1200. Nếu T_previous900, error100 thì P200, I_needed−100 không hợp lệ: I_seed0, T_raw1000 và seed_limited=true; với mốc limiter900 thì tick đầu chỉ được collective900,16, không nhảy thẳng1000. **TAKEOFF→FLIGHT dùng chính average motor đã áp dụng để seed**, không tích thêm climb I trên tick xác nhận.

Phân biệt hai lịch sử: `applied_collective` là trung bình motor sau clamp/**đổi int**, còn `limited_collective` giữ lịch sử float của limiter. Chỉ khi DIRECT→PI mới đồng bộ mốc limiter về collective thực áp dụng; RAMP→PI và PI→PI vẫn giữ lịch sử float đang có. Sai số lượng tử khi seed từ motor nguyên không được reset lịch sử limiter mỗi lần, nếu không các bước tăng0,16 có thể liên tục bị mất. **Không có bảo đảm bumpless cho mọi motor:** seed/slew collective không triệt thay đổi attitude P/D hoặc xác nhận lực đẩy thực.

Sau RC phục hồi, cần **đổi nấc MODE thật sau thời điểm hồi**, khác nấc ghi nhận trên frame hồi, để lấy lại toàn quyền; đổi khi đang mất RC không đủ. Khi chưa xác nhận, cả ga cao lẫn ga thấp đều không tiếp quản collective: DEGRADED giữ collective, FAILSAFE vẫn chạy mục tiêu tự động−60cm/s chứ không theo ga pilot. ARM OFF hợp lệ vẫn disarm ngay. Nếu PI bị latch do lỗi dọc, còn cần lịch sử mới1s và chuyển qua ANGLE rồi ALT; không tự mở lại chỉ vì baro có mẫu. ANGLE thủ công đang hoạt động có thể tiếp tục khi đường PI bị khóa. Mất RC trước FLIGHT không khởi động climb PI tự tăng ga; `launch_aborted` chỉ xóa khi DISARM.

## 9. Mixer: từ bốn lệnh T/r/p/y thành bốn motor

### 9.1 Vị trí đang khai báo

Nhìn từ trên, hướng trước ở phía trên:

```text
              TRƯỚC
    M4: trái/CW     M2: phải/CCW
              +
    M3: trái/CCW    M1: phải/CW
               SAU
```

| Motor | GPIO | Vị trí |
|---|---:|---|
| M1 | 27 | Sau phải |
| M2 | 26 | Trước phải |
| M3 | 25 | Sau trái |
| M4 | 33 | Trước trái |

Đây là cấu hình phần mềm, chưa thay cho kiểm tra dây/chiều quay thực tế.

### 9.2 Ma trận mixer đúng trong firmware

```text
M1 = T + y - r + p
M2 = T - y - r - p
M3 = T - y + r + p
M4 = T + y + r - p

[M1]   [1 -1  1  1] [T]
[M2] = [1 -1 -1 -1] [r]
[M3]   [1  1  1 -1] [p]
[M4]   [1  1 -1  1] [y]
```

Trong miền tuyến tính trước clamp:
- T tăng: cả bốn tăng bằng nhau, chủ yếu đổi tổng lực đẩy.
- r dương: M3/M4 bên trái tăng, M1/M2 bên phải giảm.
- p dương: M1/M3 phía sau tăng, M2/M4 phía trước giảm.
- y dương: M1/M4 tăng, M2/M3 giảm, tạo chênh moment phản lực giữa hai nhóm chiều quay.

Tổng luôn bằng4T trước clamp vì đóng góp r/p/y cộng lại bằng0. Tuy nhiên **tổng lệnh không đổi không chứng minh tổng lực đẩy thực không đổi**, do đặc tính motor/cánh phi tuyến và không đồng nhất.

Dấu r/p/y chỉ là quy ước phần mềm. Muốn phản hồi ổn định, chuỗi dấu từ IMU → PID → vị trí/chiều motor → moment thân phải khớp. Khi một bên đang thấp và tốc độ quay gần0, moment phục hồi phải nâng bên thấp tương đối; không được giải thích “tăng phía đang cao làm tự cân bằng”.

### 9.3 Clamp và thang DShot

Khi armed, motor cuối bị chặn900…1600 rồi chuyển sang int. Trong mode dọc, collective còn bị chặn900…1600 trước khi cộng r/p/y; requested cho anti-windup vẫn lấy trước các clamp này.

Driver đổi số nguyên nội bộ m:

```text
m < 900       → DShot0 trong driver
900…1600      → 48 + floor[(m-900)×1999/700]
trên1600      → tối đa2047
```

Ở đường điều khiển armed bình thường, clamp motor đã bảo đảm m≥900; khi disarmed firmware chủ động gửi m800 cho cả bốn để tạo DShot0. **Không thay clamp armed bằng DShot0 chỉ vì ga pilot đang thấp.**

| Nội bộ | DShot | Diễn giải đúng |
|---:|---:|---|
| 800 | 0 | Lệnh disarm |
| 900 | 48 | Đầu dải throttle DShot, không phải bảo đảm motor có đủ lực nâng |
| 1206 | 921 | Collective ANGLE lớn nhất theo RC2010 nếu không có hiệu chỉnh |
| 1600 | 2047 | Cuối dải lệnh đang dùng, không phải số RPM |

DShot600 dùng bit khoảng1.675µs,16bit khoảng26.8µs trước khoảng kết thúc; RMT phát xung tới từng ESC. Packet gồm11bit giá trị,1bit yêu cầu telemetry và4bit CRC. Code hiện không yêu cầu telemetry. Gửi lệnh thành công ở phần mềm không chứng minh ESC đã tạo đúng lực đẩy; chưa có vòng phản hồi RPM/thrust tại đây.

Đối chiếu: `control_motor_values()`, `control_limit_collective()` và `control_mix_and_write()` trong [control_task.ino](control_task.ino), [driver DShot](esc_dshot.ino).

## 10. Vì sao saturation ảnh hưởng PID, và phản hồi chống windup tính gì?

Motor có giới hạn. PID yêu cầu thêm moment nhưng một motor đã1600 thì phần tăng tiếp không thể được thực hiện. Nếu I vẫn tích theo cùng chiều, PID đang “ghi nợ” một tác động actuator không làm được; khi điều kiện đổi, phần I tồn dư có thể gây vượt lố.

Firmware giới hạn collective riêng trước khi mixer:

```text
T_limited = clamp(clamp(T_requested,900,1600),
                  max(900,T_previous−150×0.002), min(1600,T_previous+80×0.002))
requested_motor = mixer(T_limited,r,p,y)
d1…d4 = clamp(requested_motor,900,1600) − requested_motor
residualR = (−d1−d2+d3+d4)/4
residualP = ( d1−d2+d3−d4)/4
residualY = ( d1−d2−d3+d4)/4
residualClimb = (T_limited−T_requested) + Σ min(0,di)/4
```

Rate dùng nghịch đảo mixer; climb dùng cả hạn chế collective và phần thiếu do motor chạm trần, **không lấy floor do rate để cản I giảm**. Chỉ nguồn CLIMB_PI mới nhận feedback climb; RAMP/ANGLE không bị output climb cũ ghi đè. ANGLE không dùng slew collective dọc.

```text
ΔI = I_candidate − I_before
nếu ΔI × residual_tương_ứng < −0.0001:
    rate: I_new = I_before
    climb: I_new = clamp(I_candidate + residualClimb,
                        min(I_before,I_candidate), max(I_before,I_candidate))
    tính lại output
```

Climb giữ phần bước I khả thi. Ví dụ T_previous1200, P200/I_before200, error và e_prev đều+100cm/s: ΔI_candidate=`15×100×0,002=3`, I_try203 cho T_requested1203. Slew chỉ cho1200,16 nên residual−2,84; I_new200,16 giữ được tiến triển0,16. Nếu rollback hết về200 thì mỗi tick lại T1200, PI bị đứng dù pilot yêu cầu lên liên tục. Hiệu chỉnh chỉ nằm giữa I_before và I_candidate, không tự xóa I cũ ngoài bước hiện tại.

Rate rollback rồi remix trước khi xét climb; nếu climb hiệu chỉnh thì limiter dùng lại mốc T_previous của tick, không tiến thêm bước thời gian. Có thể rà lại rate khi collective vừa đổi. Đây là số lượt hữu hạn, không phải bộ tối ưu phân bổ lực đẩy tới hội tụ. Feedback không đi vào tilt PID vì output tilt là°/s.

Ví dụ collective trước1580, T_requested1580 nên limiter không đổi, r40,p=y0:

```text
requested = [1540,1540,1620,1620]
applied   = [1540,1540,1600,1600]
delta     = [0,0,-20,-20]
residualT = -10; residualR = -10; residualP=residualY=0
```

- Nếu climb I đang tăng, ΔI dương nhân residualT âm → bỏ phần bước không khả thi; nếu độ lớn residual≥ΔI thì rollback hết bước đó.
- Nếu roll I đang tăng, residualR âm → từ chối bước đó.
- Nếu I đang giảm theo hướng thoát bão hòa, tích cùng dấu không âm → cho phép bước đó theo kiểm tra residual.

**Ở biên thấp thì ngược lại:** raw T860 bị nâng lên900 cho residualT+40. Bước climb I tăng là cần thiết để bớt thiếu ga nên được phép; bước I giảm làm thiếu ga thêm sẽ bị chặn. Ví dụ error và e_prev20 cho P60, I0: candidate mỗi lượt tăng0,60. Bỏ qua các thay đổi trạng thái khác, sau67 lượt/0,134s I đạt40,20 và raw900,20. Nếu mốc limiter vẫn900 ở lượt vượt biên, slew chỉ cho900,16 và feedback giữ I40,16 ở lượt đó. Đây chỉ là minh họa tích phân/giới hạn, không phải thời gian cất cánh thực và không tính seed khi vừa vào mode.

**Phân biệt floor của collective với floor do rate:** T1048,r200,p=y0 cho requested `[848,848,1248,1248]`, clamp `[900,900,1248,1248]`. Average residual+26 từng hủy bước giảm climb I dù T1048 vẫn>900. Nay ceiling residual0 và collective residual0 nên ΔI_climb âm được nhận; không được nhầm với raw T860 thật sự thấp hơn minimum. I rate200 cũ không tự biến mất chỉ vì climb được unwind; đặt xuống phải disarm.

Giới hạn chống windup chỉ biết lệnh đã giới hạn, không biết motor hỏng, pin sụt, cánh thiếu lực hay thân bị tay giữ. Bảo vệ WAIT/TAKEOFF giảm tích I trước chuyến mới nhưng không chứng minh tiếp đất ở FLIGHT; không bảo đảm đồng thời mọi yêu cầu T/r/p/y hoặc an toàn bay.

Đối chiếu: `control_mix_and_write()` trong [control_task.ino](control_task.ino), `pid_motor_feedback()` và `pid_climb_feedback()` trong [pid.ino](pid.ino).

## 11. Liên hệ với vật lý: lực đẩy, trọng lực, moment và hiện tượng trôi

### 11.1 Tại sao đổi cùng bốn motor làm lên/xuống?

Mô hình gần đúng, bỏ qua lực cản và các chi tiết khí động:

```text
F_i ≈ k_f × ω_i²
F_total = F1+F2+F3+F4
m×a_z = F_total×cosφ×cosθ - m×g
```

ω là tốc độ quay thực theo rad/s, k_f phụ thuộc cánh/mật độ không khí. DShot là lệnh cho ESC, **không phải ω**; không có k_f hoặc quan hệ DShot→ω được đo trong firmware để tính Newton chính xác từ lệnh.

Ví dụ giả định m0.8kg:

```text
mg = 0.8×9.81 = 7.848N
Hover phẳng, bốn motor giống nhau: mỗi motor≈1.962N.
Nếu nghiêng10° một trục, để giữ độ cao:
F_total = 7.848/cos10° ≈ 7.969N.
```

ANGLE không tự có phép bù hình học này. ALT_HOLD có thể tăng ga **sau khi sai số vận tốc/ước lượng tác động vào PI**, nhưng đó là phản hồi, không phải feed-forward biết trước chính xác lực đẩy cần thiết.

Tăng ga không đổi vận tốc ngay: lực tạo gia tốc, rồi vận tốc thay đổi theo thời gian; ESC/motor/cánh còn có độ trễ. Giảm ga không nhất thiết làm drone hạ ngay nếu trước đó đang lên nhanh. P/I quá mạnh với độ trễ có thể gây lên xuống dao động.

### 11.2 Vì sao chênh lực motor làm đổi tư thế?

Công thức tổng quát:

```text
τ = Σ(r_i × F_i) + moment phản lực cánh
J×ω_dot + ω×(J×ω) = τ
```

J là tensor quán tính, **không phải I_term của PID**. Gần tư thế nhỏ và tốc độ quay không lớn, có thể xấp xỉ một trục `angular_acceleration ≈ torque / J_axis`.

Ví dụ giả định khoảng cách vuông góc từ trục roll đến mỗi motor l0.12m. Mỗi motor bên phải tăng0.2N, mỗi motor bên trái giảm0.2N:

```text
chênh lực hai bên tăng = 2×0.2 - 2×(-0.2) = 0.8N
|τ_roll| ≈ 0.12×0.8 = 0.096N·m
nếu J_roll=0.01kg·m²:
|angular_acceleration| ≈ 9.6rad/s² ≈ 550.04°/s²
```

Đó là ví dụ về độ nhạy của moment, không phải dự đoán drone thật. Hướng moment xác định bằng vị trí, chiều lực và quy ước trục; kiểm bằng tích có hướng, không suy từ cảm giác tiếng motor.

Yaw dựa vào moment phản lực của hai nhóm motor quay ngược chiều. Với chiều quay nhìn từ trên như khai báo, tăng nhóm cánh CW có xu hướng tăng moment phản lực CCW lên thân. Dấu gyro và mixer yaw phải tương thích; sai chiều một motor có thể phá cả yaw và lực nâng, không chỉ làm sai một con số PID.

### 11.3 Tại sao cân bằng góc rồi vẫn trôi ngang?

Ở mô hình nghiêng một góc α, giữ độ cao, bỏ lực cản:

```text
F_total×cosα = mg
a_horizontal = F_total×sinα/m = g×tanα
```

Lệch thật chỉ1° có thể tạo `a_horizontal≈0.17123m/s²`. Nếu giữ điều kiện đó2s từ vận tốc0, dịch chuyển lý tưởng `0.5×a×t²≈0.34247m`. Đây là ước tính lý tưởng, không phải số đo drift hiện tại.

Nhưng **ngay cả khi góc đã về0 và gia tốc ngang về0, vận tốc ngang đã có vẫn chưa tự về0** nếu bỏ lực cản. Cần thông tin vận tốc/vị trí ngang để chủ động hãm và trở lại điểm cũ. Firmware không có GPS/optical flow hoặc vòng vị trí ngang, nên:

- PID góc/rate tốt không đồng nghĩa giữ tọa độ x/y.
- I của rate bù moment, không biết drone đang ở cách vị trí ban đầu bao nhiêu mét.
- Baro chỉ là tham chiếu dọc, không giúp xác định trôi trái/phải.
- Giảm drift tư thế do gyro/accel có ích, nhưng không thay thế vòng giữ vị trí.

## 12. Bảng ảnh hưởng cảm biến theo chế độ và khi lỗi

| Trạng thái | Gyro | Accel/góc | Baro/KF dọc | Nguồn collective |
|---|---|---|---|---|
| Disarmed | Vẫn có thể đo/ước lượng | Vẫn có thể cập nhật | Nếu đủ health có thể cập nhật | Cố định800, DShot0 |
| ANGLE | Đầu vào rate PID và dự đoán góc | Phép đo góc → tilt PID | Không trực tiếp điều khiển ga | RC×0.6 |
| ALT_HOLD | WAIT không PID; TAKEOFF PD; FLIGHT PID | Góc/chiếu a_z, readiness | Freshness/history và bằng chứng airborne; PI chỉ FLIGHT | SW2; ARM trực tiếp vào WAIT900; TAKEOFF ramp; FLIGHT PI qua slew |
| FAILSAFE | Giữ mục tiêu tốc độ quay | Cố phẳng + chiếu a_z | Điều khiển tốc độ hạ−60cm/s | Tự động khi mất RC, confirmed airborne và dọc hợp lệ/không khóa; PI qua slew |
| DEGRADED | Cân bằng, khóa I trước FLIGHT; WAIT không torque | Vẫn cung cấp góc | Không dùng climb PI | Sau tiếp quản RC: airborne tăng/giảm, preflight chỉ giữ/giảm; mất/chưa tiếp quản giữ cuối; riêng WAIT900 |

| Sự cố/ảnh hưởng | Hệ quả cần hiểu |
|---|---|
| Gyro bias/rung | Sai rate PID trực tiếp; có thể làm góc trôi, D nhiễu và motor dao động. |
| Accel lệch/nhiễu/gia tốc ngang | Có thể làm góc đo sai; ngoài tư thế còn làm sai phép tách gia tốc dọc. |
| Baro nhiễu/luồng cánh | Có thể tạo đổi z/vz ước lượng rồi đổi ga trong mode dọc; không trực tiếp ra lệnh roll/pitch. |
| Baro vắng/cũ/lỗi | Chặn vào mode dọc; đang mode dọc thì DEGRADED. Khi đã confirmed airborne, lỗi cũng khóa PI dù đang ANGLE; ANGLE đang điều khiển thủ công có thể còn hoạt động. |
| RC mất khi armed | WAIT phiên ARM trực tiếp ALT thì disarm; chỉ confirmed airborne và dọc healthy/không khóa/không abort mới FAILSAFE, còn lại DEGRADED. Không tự re-arm khi nối lại; phải đổi MODE sau hồi để lấy lại quyền. |
| IMU/ước lượng tư thế/control output không hữu hạn | Khóa control_fault, disarm; không còn bảo đảm cân bằng/hạ an toàn. |
| KF/quality dọc lỗi | Rời PI sang DEGRADED; không đồng nhất với lỗi tư thế bắt buộc disarm. |
| Dữ liệu sai nhưng hữu hạn/đóng băng | Kiểm tra số và ID chưa đủ phát hiện hết; không mặc định mọi lỗi sẽ gây disarm đúng lúc. |

Khi thấy motor dao động, phải phân biệt **lệnh pilot đổi**, **góc/rate ước lượng đổi**, **vz_est đổi**, **PID tự tích I** hay **motor bị clamp**. Một triệu chứng không đủ để kết luận cần giảm Kp hoặc tăng Ki.

## 13. Cách đọc log và tra mã mà không nhầm tầng

1. Xác định `armed`, `flight_mode`, `rc_ok`, `baro_healthy`, `control_fault` trước; biết đang dùng đường ga nào.
2. So `roll_sp_deg/pitch_sp_deg` với góc đo: nếu góc đo sai khi phẳng, kiểm IMU/offset/ước lượng trước khi đổ cho rate PID.
3. So output tilt (`rate_*_sp_dps`) với gyro rate: đây mới là sai số vào PID rate, không phải sai số góc tính thêm lần nữa.
4. Trong mode dọc, so `climb_sp_cms` thực dùng với `kf_climb_cms`, quan sát baro/acc_earth_z_g. FAILSAFE dùng−60cm/s, không theo cần ga pilot.
5. Phân biệt u_throttle trước clamp, bốn lệnh motor sau clamp, average đã áp dụng và DShot. Không đọc “1200” rồi gọi đó là1200µs hay60% lực đẩy.
6. Nếu thấy `seed_limited`, kiểm chuyển mode và giới hạn I, không chỉ so ga trước/sau rồi coi seed đã giữ mọi thứ liên tục.
7. DEBUG_ALTITUDE/RC in phase, history_ok/reason, ready, takeover, denied, I bốn vòng và collective requested/limited; chưa in mọi P/D/residual. Phase0 WAIT,1 TAKEOFF,2 FLIGHT. Reason0=ACQUIRING,1=OK,2=STALE,3=KF,4=PREARM; PREARM có thể nghĩa là lịch sử dọc đã healthy nhưng chưa đạt điều kiện tĩnh, không phải tự khẳng định sensor hỏng. Không có subsystem LED cảnh báo phase/fault/takeover trong vòng điều khiển; LED14 thuộc các thao tác khởi động/calibration IMU. Không có cảnh báo gửi lên tay điều khiển; cần đo tải runtime trên ESP32.

| Nơi tra | Nội dung |
|---|---|
| [File cấu hình chính](DroneFC_CRSF_DShot_ManualAcc_after_change.ino) | Chân, mode, gain, offset, nhịp, ngưỡng ALT và `setup()` |
| [Vòng điều khiển và mode](control_task.ino) | `control_step()`, `control_update_state()`, `mode_angle()`, `mode_alt_hold()`, `mode_failsafe()`, `mode_degraded()` |
| [Readiness dọc](control_task.ino) | `vertical_readiness_update()`, `vertical_reset_readiness()`, `control_update_launch_evidence()` |
| [IMU](imu_icm20602.ino) | `imu_update()`, `imu_read_raw()`, `imu_kalman_1d()` với R cố định9 |
| [Baro](baro_bmp388.ino) | Bù nhiệt/áp suất, đổi độ cao, `baro_update()` và mốc reference |
| [KF dọc](altitude_kf.ino) | `alt_kf_update()`, `alt_kf_reset_to_measurement()`; không có innovation gate |
| [PID](pid.ino) | `pid_step()`, `pid_step_climb()`, `pid_seed_climb()`, feedback chống windup |
| [Mixer](control_task.ino) | `control_motor_values()`, `control_mix_and_write()`, residual, clamp/slew |
| [DShot](esc_dshot.ino) | Mã hóa frame, đổi thang lệnh |
| [Báo cáo xử lý](BAO_CAO_XU_LY_MANUALACC.md) | Lỗi đã sửa, test host và các giới hạn xác minh |

**Tóm lại:** cảm biến không trực tiếp “chọn motor”; chúng tạo trạng thái ước lượng. Mode quyết định mục tiêu và nguồn collective; PID biến sai số thành yêu cầu lực/moment theo **đơn vị lệnh**; mixer phân yêu cầu ra bốn motor có giới hạn; ESC/motor/cánh mới tạo chuyển động vật lý. Độ ổn định phụ thuộc toàn chuỗi này, không chỉ một hệ số P, I hay K.
