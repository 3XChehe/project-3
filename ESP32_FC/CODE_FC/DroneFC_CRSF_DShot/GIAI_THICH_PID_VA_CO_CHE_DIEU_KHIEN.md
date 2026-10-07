# Giải thích Cơ chế PID Cascade và Hiện tượng "Nghiêng càng mạnh hơn" khi Test trên Bàn

> Tài liệu này giải thích tại sao drone hoạt động đúng khi bay, nhưng khi bạn **dùng tay nghiêng drone trên bàn**, bạn lại cảm thấy 2 motor phía dưới mạnh hơn — tưởng như "đẩy ngược" làm drone càng nghiêng thêm.

---

## 1. Kiến trúc PID Cascade (PID Xếp Tầng)

Firmware này dùng **PID 2 tầng** (cascade / nested PID) cho trục roll và pitch:

```
Tay điều khiển (cần gạt)
        │
        ▼  angle_sp_deg (góc mong muốn, ví dụ 0°)
   ┌──────────────┐
   │  Tầng NGOÀI  │  pid_tilt_roll / pid_tilt_pitch
   │  (góc nghiêng)│  KP_TILT = 10.0
   └──────────────┘
        │
        ▼  rate_sp_dps (tốc độ góc mong muốn, đơn vị °/s)
   ┌──────────────┐
   │  Tầng TRONG  │  pid_rate_roll / pid_rate_pitch
   │ (tốc độ góc) │  KP_RATE = 0.5, KI_RATE = 1.4, KD_RATE = 0.03
   └──────────────┘
        │
        ▼  u_roll / u_pitch (tín hiệu điều khiển)
   ┌──────────────┐
   │    MIXER     │  Phân phối lực cho 4 motor
   └──────────────┘
        │
        ▼
  ESC 1, 2, 3, 4
```

---

## 2. Giải thích từng tầng

### Tầng Ngoài — Điều khiển Góc (Angle Controller)

```c
// pid.ino, line 123-128
float pid_tilt_roll(float angle_deg, float angle_sp_deg) {
  tilt_roll_out_dps = pid_step(pid_tilt_roll_st, angle_sp_deg - angle_deg,
                               KP_TILT_ROLL, ...);
  return tilt_roll_out_dps;
}
```

- **Đầu vào**: `angle_sp_deg` (góc mong muốn từ cần gạt) và `angle_deg` (góc thực từ IMU)
- **Sai số**: `error = angle_sp_deg - angle_deg`
- **Đầu ra**: Tốc độ góc mong muốn (°/s) để tầng trong bám theo
- **Ý nghĩa**: Nếu drone đang nghiêng phải 10° trong khi setpoint là 0°, tầng này ra lệnh "hãy xoay trái với tốc độ = KP × 10 = **100 °/s**"

### Tầng Trong — Điều khiển Tốc Độ Góc (Rate Controller)

```c
// pid.ino, line 104-108
float pid_rate_roll(float rate_dps, float rate_sp_dps) {
  return pid_step(pid_rate_roll_st, rate_sp_dps - rate_dps,
                  KP_RATE_ROLL, KI_RATE_ROLL, KD_RATE_ROLL, ...);
}
```

- **Đầu vào**: `rate_sp_dps` (từ tầng ngoài) và `rate_dps` (tốc độ góc thực từ gyro)
- **Đầu ra**: `u_roll` — tín hiệu điều khiển cuối cùng đưa vào mixer

### Mixer — Phân phối lực cho 4 motor

```c
// control_task.ino, line 136-139
esc_1 = u_throttle - u_yaw - u_pitch + u_roll;   // M1 front-right CCW
esc_2 = u_throttle + u_yaw + u_pitch + u_roll;   // M2 rear-left   CCW
esc_3 = u_throttle + u_yaw - u_pitch - u_roll;   // M3 front-left  CW
esc_4 = u_throttle - u_yaw + u_pitch - u_roll;   // M4 rear-right  CW
```

**Sơ đồ vị trí motor (nhìn từ trên):**

```
        TRƯỚC (PITCH -)
            │
   M3(CW)  ─┼─  M1(CCW)
   front-L  │  front-R
            │
   ──────── + ────────    ROLL: trái(-) / phải(+)
            │
   M2(CCW) ─┼─  M4(CW)
   rear-L   │  rear-R
            │
        SAU (PITCH +)
```

---

## 3. Giải thích hiện tượng "Motor mạnh hơn khi dùng tay nghiêng" ← QUAN TRỌNG

### Tình huống:
- Drone đang ở trạng thái **cân bằng** (roll_sp = 0°, roll_deg ≈ 0°)
- Bạn **dùng tay nghiêng drone sang phải** (roll_deg tăng lên, ví dụ +20°)
- Bạn thấy **2 motor bên trái** (phía cao hơn) quay **mạnh hơn**
- Cảm giác: chúng đang "đẩy ngược" — bên trái đẩy lên còn bên phải bị kéo xuống

### Giải thích — Tại sao đây là ĐÚNG:

**Bước 1: Tính sai số tầng ngoài**
```
error = angle_sp_deg - angle_deg = 0° - (+20°) = -20°
```
Sai số âm → PID muốn **quay về bên trái** (giảm roll)

**Bước 2: Tầng ngoài tính tốc độ góc mong muốn**
```
rate_sp = KP_TILT × error = 10.0 × (-20) = -200 °/s
```
Tầng ngoài muốn gyro đọc **-200 °/s** (xoay nhanh về bên trái)

**Bước 3: Tầng trong tính u_roll**
```
u_roll = PID(rate_sp - rate_dps) = PID(-200 - 0) → âm lớn
u_roll ≈ KP_RATE × (-200) = 0.5 × (-200) = -100
```
→ `u_roll` là số **âm** (lệnh "nghiêng trái")

**Bước 4: Mixer phân phối lực**
```
esc_1 (front-right) = throttle + u_roll  = throttle + (-100)  → GIẢM
esc_3 (front-left)  = throttle - u_roll  = throttle - (-100)  → TĂNG
esc_2 (rear-left)   = throttle + u_roll  = throttle + (-100)  → GIẢM
esc_4 (rear-right)  = throttle - u_roll  = throttle - (-100)  → TĂNG
```

> ✅ **Motor bên TRÁI (M2, M3) TĂNG** — để nâng phần trái lên, tức là **kéo drone về vị trí cân bằng** từ trạng thái đang nghiêng sang phải.

### Tại sao CẢM GIÁC "đẩy ngược"?

```
Trạng thái: Bạn nghiêng drone sang PHẢI (+20°)

          TRÁI (cao)      PHẢI (thấp)
            M3, M2   |   M1, M4
              ▲  ▲   |   ↓  ↓
         TĂNG lực    |   GIẢM lực

  PID cố NÂNG phần bên TRÁI (phía cao) và HẠ bên PHẢI
  để kéo trọng tâm về 0°.
```

Nghe có vẻ lạ phải không — sao phải nâng bên **cao** lên? Thực ra:

- Drone đang nghiêng PHẢI (+20°)
- Bên TRÁI đang ở vị trí **cao hơn** → cần giảm góc → tăng motor bên trái để nâng thân bên trái, **xoay frame theo chiều ngược lại** về 0°
- Đây là nguyên lý **moment lực quay ngược chiều** để lấy lại cân bằng

**Thêm nữa — Tích phân I tích lũy khi bạn giữ tay:**

Khi bạn giữ drone bằng tay, bạn đang **kháng lại lực** mà PID tạo ra:
1. PID tăng motor trái → bạn giữ frame, drone không quay được
2. Sai số vẫn tồn tại → khâu tích phân I **tiếp tục tích lũy**
3. Motor trái **mạnh lên mãi** theo thời gian
4. Cảm giác: "Càng giữ lâu, motor càng mạnh hơn, đẩy ngược"

**Thực ra motor đang ĐÚNG hướng** — chúng đang cố lấy lại cân bằng. Khi bạn thả tay, drone sẽ tự về 0°.

---

## 4. Ví dụ số cụ thể với firmware hiện tại

| Thông số | Giá trị |
|---|---|
| `roll_sp_deg` | 0° (cần gạt giữa) |
| `roll_deg` (bạn đang nghiêng) | +20° (nghiêng phải) |
| Error tầng ngoài | 0 - 20 = **-20°** |
| `KP_TILT_ROLL` | 10.0 |
| `rate_sp` từ tầng ngoài | 10 × (-20) = **-200 °/s** |
| `rate_dps` (gyro, bạn giữ yên) | **0 °/s** |
| Error tầng trong | -200 - 0 = **-200 °/s** |
| `u_roll` (KP_RATE = 0.5, chỉ P) | ≈ **-100** |
| M2, M3 (trái) | throttle **+ 100** → TĂNG |
| M1, M4 (phải) | throttle **- 100** → GIẢM |

---

## 5. Kiểm tra nhanh trên bàn (không cần bay)

Bật `DEBUG_ATTITUDE` (đã bật mặc định trong config):

```c
// DroneFC_CRSF_DShot.ino, line 136
#define DEBUG_ATTITUDE      // góc hiện tại so với góc mong muốn
```

Output: `roll_sp_deg, roll_deg, pitch_sp_deg, pitch_deg`

**Quy trình kiểm tra:**
1. Không arm, chỉ quan sát
2. Nghiêng drone sang phải → `roll_deg` phải **tăng dương**
3. Nghiêng sang trái → `roll_deg` phải **âm**
4. Nếu ngược lại → IMU bị gắn ngược trục X, cần đổi dấu khi đọc

Sau đó bật `DEBUG_RATE`:
```c
// #define DEBUG_RATE          // tốc độ góc mong muốn so với thực tế
```
Output: `rate_roll_sp_dps, rate_roll_dps, ...`

5. Arm + ga thấp + nghiêng phải → `rate_roll_sp_dps` phải **âm** (muốn xoay trái)

---

## 6. Nguyên nhân phổ biến khiến điều khiển bị ĐẢO NGƯỢC

Nếu drone **không tự ổn định** khi thả tay (ngã thêm thay vì về cân bằng):

### 6.1 Trục IMU lắp ngược
```c
// imu_icm20602.ino — kiểm tra dấu khi đọc gyro/acc
float imu_get_roll_deg();   // Nghiêng phải → phải trả về DƯƠNG
```
**Test nhanh**: Nghiêng phải → `roll_deg` phải tăng dương (xem DEBUG_ATTITUDE)

### 6.2 Vị trí motor trong mixer bị sai
```c
// control_task.ino, line 136-139
// Kiểm tra M1 có thực sự là front-right không
esc_1 = u_throttle - u_yaw - u_pitch + u_roll;
```
**Lưu ý từ code**: `// u_roll và u_yaw đã được hoán đổi vì thực tế lắp đặt motor bị lệch`

### 6.3 Chiều quay motor sai
- M1 front-right: phải quay **CCW** (ngược chiều kim đồng hồ nhìn từ trên)
- M2 rear-left: phải quay **CCW**
- M3 front-left: phải quay **CW**
- M4 rear-right: phải quay **CW**

Nếu motor quay sai chiều → hiệu ứng torque yaw bị đảo, gây ảnh hưởng toàn bộ mixer.

---

## 7. Tóm tắt

```
╔══════════════════════════════════════════════════════════════╗
║  KẾT LUẬN                                                   ║
║                                                             ║
║  Khi bạn nghiêng drone sang PHẢI (roll = +20°):            ║
║                                                             ║
║  PID tính: error = 0 - 20 = -20 → muốn xoay TRÁI          ║
║  Motor TRÁI (M2,M3) TĂNG → Nâng phần trái lên             ║
║  Motor PHẢI (M1,M4) GIẢM → Hạ phần phải xuống             ║
║  → Frame xoay về 0° = ĐÚNG                                 ║
║                                                             ║
║  Cảm giác "motor mạnh hơn khi giữ tay nghiêng" = ĐÚNG     ║
║  Bởi vì bạn KHÁNG LẠI lực PID, khâu I tích lũy.          ║
║  Khi thả tay → drone tự về cân bằng.                       ║
╚══════════════════════════════════════════════════════════════╝
```

---

## 8. Sơ đồ tổng thể luồng điều khiển

```
┌──────────────────────────────────────────────────────────────────┐
│  VÒNG ĐIỀU KHIỂN CHÍNH (500Hz, core 1)                          │
│                                                                   │
│  RC cần gạt ─────────────────────────────────────┐              │
│  (roll_sp_deg = 0°)                               ▼              │
│                                             [TẦNG NGOÀI]        │
│  IMU (acc+gyro)                             pid_tilt_roll        │
│   → roll_deg ────────────────────────────►  error = sp - deg    │
│                                             KP_TILT = 10.0      │
│                                                  │               │
│                                                  ▼ rate_sp (°/s)│
│                                             [TẦNG TRONG]        │
│   → rate_roll_dps ───────────────────────►  pid_rate_roll       │
│                                             KP_RATE = 0.5       │
│                                             KI_RATE = 1.4       │
│                                             KD_RATE = 0.03      │
│                                                  │               │
│                                                  ▼ u_roll        │
│  u_throttle ────────────────────────────► [MIXER QUAD-X]        │
│  u_pitch ───────────────────────────────►       │                │
│  u_yaw ─────────────────────────────────►  esc_1,2,3,4         │
│                                                  │               │
│                                        [DShot600 → ESC]         │
│                                                  │               │
│                                           M1, M2, M3, M4        │
└──────────────────────────────────────────────────────────────────┘
```

---

*Tài liệu tương ứng firmware: `DroneFC_CRSF_DShot` — PID cascade 2 tầng, 500Hz, ESP32 + ICM20602 + CRSF + DShot600*
