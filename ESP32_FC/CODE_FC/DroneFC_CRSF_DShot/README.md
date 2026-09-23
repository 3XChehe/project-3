# DroneFC_CRSF_DShot — Firmware Quadcopter ESP32

Firmware điều khiển bay cho drone quadcopter, chạy trên ESP32 WROOM32 38Pin.
Phiên bản điều chỉnh từ [KyThuatUAV FC](../KyThuatUAV_FC_level_1/) cho bộ linh kiện cụ thể.

## Linh kiện sử dụng

| Thành phần | Linh kiện | Giao tiếp |
|---|---|---|
| Vi điều khiển | ESP32 WROOM32 38Pin | — |
| Cảm biến quán tính | ICM-20602 | SPI, 10 MHz |
| Cảm biến áp suất | BMP388 | I2C, 400 kHz |
| Bộ thu RC | Radiomaster RP3 ExpressLRS 2.4GHz | **CRSF**, UART2, 420000 baud |
| ESC | GOKU G55M 4-in-1 55A BLHeli32 | **DShot600**, RMT |
| Động cơ | RS2205 2300KV | 4 motor |
| Khung | 5 inch | — |
| Cánh | 5 inch (2 cặp CW/CCW) | — |
| Pin | LiPo 3S 2200mAh 75C | — |
| Tay điều khiển | Pocket Crush ELRS 2.4GHz (EdgeTX) | — |
| Nguồn 5V | Flywoo BEC 5V/2A | — |
| Chống rung | Bộ đệm cao su M3 | — |

## Thay đổi so với bản gốc

### 1. RC: SBUS → CRSF
- **Lý do**: Radiomaster RP3 ELRS xuất giao thức **CRSF** (Crossfire Serial Protocol), không phải SBUS
- **Kỹ thuật**: 420000 baud, 8N1, tín hiệu **KHÔNG đảo** (SBUS là 100000, 8E2, đảo)
- **File**: `rc_crsf.ino` (thay `rc_sbus.ino`)
- **Chân**: RX=GPIO16, TX=GPIO17 (SBUS chỉ cần RX=GPIO35)

### 2. ESC: PWM → DShot600
- **Lý do**: GOKU G55M BLHeli32 hỗ trợ DShot — chính xác, nhanh, không cần hiệu chỉnh ESC range
- **Kỹ thuật**: Dùng RMT peripheral của ESP32 để tạo tín hiệu DShot chính xác
- **File**: `esc_dshot.ino` (thay `esc_output.ino`)
- **Ưu điểm**: Không bị trôi xung, không cần calibrate ESC, response nhanh hơn PWM

### 3. Pin mapping
- Tối ưu cho ESP32 WROOM32 38Pin
- Chân CRSF dùng GPIO16/17 (UART2 hardware) thay vì GPIO35 (SBUS)

## Sơ đồ nối dây

```
ESP32 WROOM32 38Pin
├── GPIO27 ──→ GOKU G55M Motor 1 signal
├── GPIO26 ──→ GOKU G55M Motor 2 signal
├── GPIO25 ──→ GOKU G55M Motor 3 signal
├── GPIO33 ──→ GOKU G55M Motor 4 signal
├── GPIO5  ──→ ICM-20602 CS (SPI)
├── GPIO18 ──→ ICM-20602 SCK (SPI) [tự động]
├── GPIO23 ──→ ICM-20602 MOSI (SPI) [tự động]
├── GPIO19 ──→ ICM-20602 MISO (SPI) [tự động]
├── GPIO14 ──→ LED trạng thái
├── GPIO16 ──→ RP3 TX (CRSF RX)
├── GPIO17 ──→ RP3 RX (CRSF TX, telemetry)
├── GPIO21 ──→ BMP388 SDA (I2C)
├── GPIO22 ──→ BMP388 SCL (I2C)
├── 5V     ←── Flywoo BEC 5V output
└── GND    ──→ GND chung (ESC + BEC + cảm biến)
```

## Nguồn cấp

```
Pin 3S ──→ GOKU G55M ESC (cấp trực tiếp cho motor)
Pin 3S ──→ Flywoo BEC 5V/2A ──→ ESP32 + ICM-20602 + BMP388 + RP3
                                  (KHÔNG lấy 5V từ ESC, dùng BEC riêng)
```

## Cài đặt

1. Arduino IDE 2.x
2. Board Manager URL: `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
3. Cài gói **esp32 phiên bản 2.0.17** — **KHÔNG dùng bản 3.x**
4. Cài thư viện `BasicLinearAlgebra`
5. Board: **ESP32 Dev Module** hoặc **ESP32-WROOM-DA Module**
6. Mở `DroneFC_CRSF_DShot.ino`

> ⚠️ **Phải đúng core 2.0.17.** Từ bản 3.x, Espressif đổi hoàn toàn cách gọi hàm
> RMT và LEDC, cài nhầm là biên dịch lỗi.

## Cấu hình tay điều khiển (EdgeTX)

Pocket Crush chạy EdgeTX, cấu hình trong Mixer:
- **CH1** = Aileron (Roll)
- **CH2** = Elevator (Pitch)
- **CH3** = Throttle
- **CH4** = Rudder (Yaw)
- **CH5** = Switch SB (Arm) — nấc trên = arm
- **CH6** = Switch SC (Mode) — 3 nấc: Angle / Angle / Alt Hold

ELRS output mode: **500Hz** (cài trong ExpressLRS Configurator hoặc Lua script)

## Cấu hình ESC (BLHeli Configurator)

Kết nối ESC với BLHeli Configurator (qua Betaflight passthrough hoặc trực tiếp):
- **Motor Protocol**: DShot600
- **Motor Direction**: Normal (đảo nếu motor quay sai chiều)
- **Timing**: 16° (mặc định)

## Cấu trúc file

```
DroneFC_CRSF_DShot/
├── DroneFC_CRSF_DShot.ino   Khối cấu hình + setup + các tác vụ RTOS
├── control_task.ino          Vòng điều khiển chính, các chế độ bay
├── imu_icm20602.ino          Driver IMU + ước lượng góc + hiệu chỉnh
├── baro_bmp388.ino           Driver áp suất BMP388, viết bằng thanh ghi
├── altitude_kf.ino           Kalman 2 trạng thái cho độ cao
├── pid.ino                   PID xếp tầng
├── rc_crsf.ino               ★ MỚI — Giải mã CRSF (thay rc_sbus.ino)
├── esc_dshot.ino             ★ MỚI — DShot600 qua RMT (thay esc_output.ino)
├── types.h                   Kiểu dữ liệu dùng chung
└── README.md                 File này
```

## An toàn

**Luôn tháo cánh quạt** khi thử nghiệm trên bàn.

Trước khi bay:
1. Kiểm tra các kênh tay điều khiển đúng thứ tự (bật DEBUG_RC)
2. Kiểm tra thứ tự motor 1 → 2 → 3 → 4 (mở khối test trong esc_init)
3. Kiểm tra dấu của cảm biến và cần gạt **cùng chiều**
4. Bay đầu tiên luôn **buộc dây** và bay thấp

## Ghi công

Mã nguồn gốc: **Nguyễn Văn Quý** — [Kỹ Thuật UAV](https://github.com/CarbonAeronautics)
License: GPL-3.0
