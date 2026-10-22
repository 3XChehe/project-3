# Changelog – DroneFC_CRSF_DShot_ManualAcc

> Tổng hợp tất cả thay đổi so với commit `2cb5cbf`, bao gồm thay đổi do người dùng thực hiện và các fix bổ sung trong phiên review này.

---

## Tổng quan file bị thay đổi

| File | Số thay đổi |
|------|-------------|
| `DroneFC_CRSF_DShot_ManualAcc.ino` | 3 |
| `control_task.ino` | 5 |
| `imu_icm20602.ino` | 4 |

---

## DroneFC_CRSF_DShot_ManualAcc.ino

### [1] Ánh xạ công tắc Flight Mode
**Vị trí:** dòng ~94–96

```cpp
// Trước
#define MODE_FOR_SW_1     MODE_ANGLE
#define MODE_FOR_SW_2     MODE_ALT_HOLD

// Sau
#define MODE_FOR_SW_1     MODE_ALT_HOLD
#define MODE_FOR_SW_2     MODE_FAILSAFE
```

**Tác dụng:** SW_1 giờ vào ALT_HOLD (trước là ANGLE). SW_2 là công tắc khẩn cấp — pilot bật để drone tự hạ cánh khi mất kiểm soát.

> [!IMPORTANT]
> `MODE_FAILSAFE` được kích qua công tắc khi `rc_ok = true`. Đây là hành vi hợp lệ vì `mode_failsafe()` không phụ thuộc vào `rc_ok`, và đã được thêm logic khởi tạo `failsafe_start_ms` + guard tự disarm tương ứng trong `control_task.ino`.

---

### [2] Tắt DEBUG_ACC_OFFSET
**Vị trí:** dòng ~155

```cpp
// #define DEBUG_ACC_OFFSET
```

**Tác dụng:** Tắt debug sau khi đã hiệu chỉnh xong ACC offset. Không ảnh hưởng runtime.

---

### [3] Giảm KI_CLIMB: 15.0 → 4.0
**Vị trí:** dòng ~124

```cpp
// Trước
const float KP_CLIMB = 3.0f, KI_CLIMB = 15.0f, KD_CLIMB = 0.0f;

// Sau
const float KP_CLIMB = 3.0f, KI_CLIMB = 4.0f,  KD_CLIMB = 0.0f;
```

**Tác dụng:** Giảm tốc độ tích phân pid_climb từ ~150 units/giây → ~40 units/giây (khi error = 10 cm/s). Khắc phục hiện tượng drone nhảy vọt khi tăng ga trong ALT_HOLD.

**Vấn đề đã xử lý:** `KI_CLIMB = 15` quá lớn — khi drone ở trên đất với motor rung, integral windup tích lũy rất nhanh rồi phóng drone lên khi cất cánh.

---

## control_task.ino

### [4] Tracking gyro bias liên tục khi chưa arm
**Vị trí:** dòng ~91–92, trong block `if (rc_ok)`

```cpp
// Bù được phần bias trôi theo nhiệt trong lúc chờ cất cánh.
// Tốt nhất cắm pin để drone nằm yên 30–60s cho IMU ấm trước khi bay.
if (!armed) imu_track_gyro_bias();
```

**Tác dụng:** Sau khi hiệu chỉnh gyro lúc khởi động, bias tiếp tục trôi theo nhiệt (thermal drift). Hàm này cập nhật bias liên tục khi đứng yên (`|rate| < 1.5 dps`), giúp bias chính xác hơn đến lúc cất cánh.

> [!NOTE]
> Chỉ chạy khi `rc_ok = true && !armed`. Không ảnh hưởng khi đang bay.

---

### [5] Khởi tạo failsafe_start_ms khi bật công tắc SW_2
**Vị trí:** dòng ~98–102

```cpp
else {
    // SW_2 = failsafe thủ công (pilot bật khi mất kiểm soát)
    if (flight_mode != MODE_FAILSAFE) failsafe_start_ms = millis();
    flight_mode = MODE_FOR_SW_2;
}
```

**Tác dụng:** Ghi nhận thời điểm bắt đầu hạ cánh failsafe thủ công. Đảm bảo bộ đếm giờ tự disarm hoạt động đúng ngay cả khi `rc_ok = true`.

**Vấn đề đã xử lý:** Trước đây `failsafe_start_ms` chỉ được set khi `!rc_ok`. Khi pilot bật công tắc (rc_ok vẫn true), timer không chạy → drone không bao giờ tự disarm sau khi hạ.

---

### [6] Tự disarm sau khi hạ xong — bắt cả failsafe thủ công
**Vị trí:** dòng ~114–121

```cpp
// Trước
if (!rc_ok && armed) { ... tự disarm ... }

// Sau
if (armed && flight_mode == MODE_FAILSAFE) { ... tự disarm ... }
```

**Tác dụng:** Tự disarm khi drone đã hạ xong (`u_throttle < ESC_MIN_ARMED+1 && |climb| < 5 cm/s && đã qua 2 giây`), áp dụng cho cả mất sóng lẫn pilot bật SW_2.

---

### [7] Dynamic acc trust + biến static local
**Vị trí:** dòng ~205–237, trong `rc_fetch_and_map()`

```cpp
// Trước: khai báo toàn cục lởng giữa file
uint32_t acc_hold_until_ms = 0;

// Sau: static local, chỉ thuộc hàm này
static uint32_t acc_hold_until_ms = 0;

if (fabsf(roll_sp_deg) > 2.0f || fabsf(pitch_sp_deg) > 2.0f)
    acc_hold_until_ms = millis() + 1500;

// Trên đất tin accel hoàn toàn, đang bay tin 50%,
// khi đang vận động tin 10%.
float trust = !armed ? 1.0f : (millis() < acc_hold_until_ms ? 0.1f : 0.5f);
imu_set_acc_trust(trust);
```

**Tác dụng:** Giảm mức tin tưởng accel khi đang bay (tránh accel bị lực quán tính làm nhiễu góc estimate) và giảm mạnh hơn trong 1.5s sau khi ra lệnh nghiêng. Chuyển biến thành `static` để tránh pollute namespace.

| Trạng thái | trust | R Kalman |
|---|---|---|
| Chưa arm | 1.0 | 9.0 |
| Đang bay, ổn định | 0.5 | 36.0 |
| Đang nghiêng / 1.5s sau | 0.1 | 900.0 |

---

### [8] Back-calculation anti-windup trong mode_alt_hold()
**Vị trí:** dòng ~176–192

```cpp
void mode_alt_hold() {
  int raw = ESC_IDLE + (int)pid_climb(alt_kf_get_climb_cms(), climb_sp_cms);

  // Khi drone trên đất, output bị chặn bởi ESC_MIN_ARMED.
  // Hoàn trả integral thừa để tránh windup → nhảy vọt khi cất cánh.
  if (raw < ESC_MIN_ARMED) {
    pid_climb_st.integral -= (ESC_MIN_ARMED - raw);
    if (pid_climb_st.integral < 0) pid_climb_st.integral = 0;
    u_throttle = ESC_MIN_ARMED;
  } else {
    u_throttle = raw;
  }
  ...
}
```

**Tác dụng:** Lớp bảo vệ thứ hai chống integral windup. Khi drone còn trên đất và motor chưa đủ lực nâng, `raw < ESC_MIN_ARMED` → integral bị hoàn trả phần thừa → cất cánh mượt.

**Vấn đề đã xử lý:** Anti-windup bên trong `pid_step_climb` chỉ bảo vệ khi output vượt `u_limit` nội bộ, không bảo vệ được khi output bị chặn **bên ngoài** bởi phần cứng ESC (`ESC_MIN_ARMED`).

---

## imu_icm20602.ino

### [9] Dynamic Kalman trust theo acc norm
**Vị trí:** dòng ~259–274, trong `imu_update()`

```cpp
float dev = fabsf(sqrtf(ax*ax + ay*ay + az*az) - 1.0f);

// Ngưỡng 0.15g để không bị kích hoạt bởi rung motor bình thường (~0.05-0.1g)
float k = dev / 0.15f;
float trust = imu_acc_trust / (1.0f + k * k);

imu_kalman_1d(..., trust);
```

**Tác dụng:** Tự động giảm tin accel khi `|a|` lệch khỏi 1g (drone đang có gia tốc thực sự — maneuver). Khi bay ổn định, `|a| ≈ 1g` → trust giữ nguyên.

**Vấn đề đã xử lý:** Ngưỡng ban đầu `0.04g` quá nhạy — rung motor bình thường (0.05–0.1g) đã kích hoạt, làm trust giảm từ 0.5 xuống ~0.04, khiến Kalman bỏ qua accel hoàn toàn → angle estimate drift trên mặt đất → `acc_earth_z_g` noise → `climb_cms` dao động → integral pid_climb tích lũy giả → drone nhảy vọt khi cất cánh.

> [!CAUTION]
> Nếu tăng ngưỡng quá cao (> 0.3g), cơ chế bảo vệ khi maneuver thật sẽ mất tác dụng. Giá trị 0.15g là cân bằng phù hợp cho drone cỡ vừa.

---

### [10] Kalman R động theo trust
**Vị trí:** dòng ~307–318, trong `imu_kalman_1d()`

```cpp
// Trước: R cố định
float gain = variance / (variance + 3.0f * 3.0f);

// Sau: R thay đổi theo trust
float r    = IMU_ACC_VAR_DEG2 / (trust * trust);   // IMU_ACC_VAR_DEG2 = 9.0
float gain = variance / (variance + r);
```

**Tác dụng:** Khi `trust = 1.0` → `R = 9.0` (giống cũ). Khi `trust = 0.5` → `R = 36`, gain nhỏ hơn, tin accel ít hơn. Công thức `R = R_base / trust²` đúng về mặt Kalman (phương sai đo lường tỉ lệ nghịch với bình phương độ tin cậy).

---

### [11] Hàm imu_track_gyro_bias()
**Vị trí:** dòng ~365–377

```cpp
void imu_track_gyro_bias() {
  const float a = 0.0005f;   // hằng số thời gian ~4s ở 500Hz
  float dr = imu_rate_roll_dps  - gyro_bias_roll_dps;
  float dp = imu_rate_pitch_dps - gyro_bias_pitch_dps;
  float dy = imu_rate_yaw_dps   - gyro_bias_yaw_dps;
  if (fabsf(dr) < 1.5f && fabsf(dp) < 1.5f && fabsf(dy) < 1.5f) {
    gyro_bias_roll_dps  += a * dr;
    gyro_bias_pitch_dps += a * dp;
    gyro_bias_yaw_dps   += a * dy;
  }
}
```

**Tác dụng:** Exponential Moving Average cập nhật bias gyro liên tục khi đứng yên (`|rate| < 1.5 dps`). Bù thermal drift trong 30–60 giây chờ trước khi bay.

---

### [12] Thay delayMicroseconds(10) → delay(2) trong hiệu chỉnh gyro
**Vị trí:** dòng ~226, trong `imu_calibrate_gyro()`

```cpp
// Trước: 500 mẫu × 10µs = 5ms tổng → quá nhanh, không lọc được nhiễu thấp
delayMicroseconds(10);

// Sau: 500 mẫu × 2ms = ~1 giây → đủ để trung bình hóa nhiễu tần số thấp
delay(2);
```

**Tác dụng:** Chất lượng bias gyro khởi động tốt hơn nhiều. Thời gian calibrate tăng từ 5ms lên 1 giây — hoàn toàn chấp nhận được trong `setup()`.

---

## Lưu ý còn lại

| # | Vấn đề | Mức độ |
|---|--------|--------|
| A | `KI_CLIMB = 4.0` là giá trị khởi điểm — nếu drone giữ độ cao kém (trôi chậm), tăng lên 5–6; nếu vẫn còn giật nhẹ, giảm xuống 2.5–3.0 | Cần test thực tế |
| B | `acc_trust = 0.5` khi bay bình thường — nếu angle estimate còn drift, có thể tăng lên 0.7 | Cần test thực tế |
| C | Ngưỡng `1.5 dps` trong `imu_track_gyro_bias()` — nếu môi trường rung nhẹ (gió), có thể bị ngắt sớm; giảm xuống 1.0 nếu cần | Tùy môi trường |
| D | `pid_climb_st` được truy cập trực tiếp từ `control_task.ino` (biến file-level trong `pid.ino`) — nếu refactor sau này nên bọc qua hàm setter | Kỹ thuật phần mềm |
