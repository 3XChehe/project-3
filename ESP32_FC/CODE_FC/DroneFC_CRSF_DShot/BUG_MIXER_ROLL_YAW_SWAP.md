# BUG REPORT: Mixer Quad-X — u_roll và u_yaw bị hoán đổi

**File liên quan:** `control_task.ino` dòng 133–139  
**Ngày phát hiện:** 2026-10-06  
**Mức độ:** 🔴 Nghiêm trọng — drone không thể tự ổn định trục roll

---

## 1. Triệu chứng quan sát được

- Giữ drone ở vị trí cân bằng, nghiêng tay để **trái cao hơn phải**
- Kỳ vọng: motor bên **phải** phải mạnh hơn để nâng phải lên → lấy lại cân bằng
- Thực tế quan sát: **cả hai motor bên trái (M2 + M3) đều mạnh hơn** → đẩy trái lên cao thêm → không ổn định

---

## 2. Nguyên nhân gốc rễ

### 2.1 Mixer hiện tại (SAI)

```c
// control_task.ino, line 136-139
esc_1 = u_throttle - u_yaw - u_pitch + u_roll;   // M1 front-right CCW
esc_2 = u_throttle + u_yaw + u_pitch + u_roll;   // M2 rear-left   CCW
esc_3 = u_throttle + u_yaw - u_pitch - u_roll;   // M3 front-left  CW
esc_4 = u_throttle - u_yaw + u_pitch - u_roll;   // M4 rear-right  CW
```

### 2.2 Phân tích nhóm u_roll trong mixer hiện tại

```
             TRƯỚC
    M3(trái) |  M1(phải)
    ──────────────────────
    M2(trái) |  M4(phải)
             SAU

Nhóm u_roll HIỆN TẠI:
  +u_roll → M1(phải) + M2(TRÁI)  ← ĐƯỜNG CHÉO ✗
  -u_roll → M3(TRÁI) + M4(phải)  ← ĐƯỜNG CHÉO ✗
```

**Motor trên cùng một cạnh (trái/phải) đang bị nhóm VỚI nhau theo đường chéo, không phải theo cạnh.**

Nhóm theo đường chéo = cách hoạt động của **YAW** trong quad-X chuẩn (vì motor trên đường chéo cùng chiều quay).

→ **u_roll đang tạo ra moment xoay YAW, không phải lực lật ROLL.**

### 2.3 Ví dụ số: trái cao (+15°)

| Bước | Tính toán | Kết quả |
|---|---|---|
| `roll_deg` khi trái cao | IMU: `atanf(ay/...)` với ay dương | `+15°` |
| Error tầng ngoài | `0 - 15 = -15°` | Muốn xoay về trái |
| `rate_sp` | `KP_TILT × (-15) = -150 °/s` | |
| `u_roll` | `KP_RATE × (-150) ≈ -75` | **Âm** |
| **M1** (front-right) | `throttle + (-75)` | **GIẢM** |
| **M2** (rear-**left**) | `throttle + (-75)` | **GIẢM** ← trái giảm |
| **M3** (front-**left**) | `throttle - (-75)` | **TĂNG** ← trái tăng |
| **M4** (rear-right) | `throttle - (-75)` | **TĂNG** |

Kết quả thực tế: M2(trái) giảm nhưng M3(trái) tăng ngược nhau → **tổng lực bên trái không đổi**, chỉ tạo moment xoắn theo đường chéo M3↑–M4↑ / M1↓–M2↓ = **YAW spin**, không phải ROLL correction.

### 2.4 Nhóm u_roll ĐÚNG phải là

```
Nhóm u_roll ĐÚNG (theo cạnh):
  +u_roll → M1(phải) + M4(phải)  ← cùng bên phải ✓
  -u_roll → M2(trái) + M3(trái)  ← cùng bên trái ✓
```

### 2.5 Mâu thuẫn thêm trong tài liệu

Chiều quay motor ghi ở **2 nơi khác nhau, không nhất quán:**

| Motor | PIN definitions (dòng 53-56) | Mixer comment (dòng 136-139) |
|---|---|---|
| M1 front-right | **CW** | **CCW** |
| M2 rear-left | **CW** | **CCW** |
| M3 front-left | **CCW** | **CW** |
| M4 rear-right | **CCW** | **CW** |

Chiều quay thực tế ảnh hưởng trực tiếp đến YAW mixer — cần xác minh bằng thực tế.

---

## 3. Mixer chuẩn cho layout này (cần xác minh thực tế)

Giả sử chiều quay thực tế là M1,M2 = CCW và M3,M4 = CW (theo comment mixer):

| | Roll phải (+u_roll) | Pitch (mũi lên = +u_pitch) | Yaw phải (+u_yaw) |
|---|---|---|---|
| M1 front-right CCW | +u_roll | -u_pitch | +u_yaw |
| M2 rear-left CCW | -u_roll | +u_pitch | +u_yaw |
| M3 front-left CW | -u_roll | -u_pitch | -u_yaw |
| M4 rear-right CW | +u_roll | +u_pitch | -u_yaw |

**Mixer đề xuất (cần test):**
```c
esc_1 = u_throttle + u_yaw - u_pitch + u_roll;   // M1 front-right CCW
esc_2 = u_throttle + u_yaw + u_pitch - u_roll;   // M2 rear-left   CCW
esc_3 = u_throttle - u_yaw - u_pitch - u_roll;   // M3 front-left  CW
esc_4 = u_throttle - u_yaw + u_pitch + u_roll;   // M4 rear-right  CW
```

> ⚠️ **Chưa áp dụng** — cần xác minh thực tế vật lý trước.

---

## 4. Các bước cần làm trước khi sửa

- [ ] **Bước 1:** Test từng motor riêng lẻ (dùng Configurator hoặc code debug) để xác nhận GPIO27/26/25/33 điều khiển motor vật lý nào
- [ ] **Bước 2:** Xác nhận chiều quay thực tế của từng motor (CW hay CCW nhìn từ trên)
- [ ] **Bước 3:** Xác nhận quy ước dấu roll_deg của IMU (trái cao → `roll_deg` dương hay âm?)
- [ ] **Bước 4:** Xác nhận quy ước dấu pitch (mũi lên → `pitch_deg` dương hay âm?)
- [ ] **Bước 5:** Áp dụng mixer mới, test trên bàn không cánh trước khi bay

---

## 5. Tóm tắt một dòng

> **`u_roll` trong mixer đang nhóm motor theo đường chéo (hành vi YAW) thay vì theo cạnh trái/phải (hành vi ROLL đúng). Drone không thể tự ổn định trục roll.**
