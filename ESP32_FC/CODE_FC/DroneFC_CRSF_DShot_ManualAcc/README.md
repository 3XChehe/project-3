# DroneFC_CRSF_DShot_ManualAcc — Firmware Quadcopter ESP32 (Hiệu Chỉnh Acc Thủ Công)

Firmware điều khiển bay cho drone quadcopter chạy trên **ESP32 WROOM32 38Pin**.  
Phiên bản này kết hợp:
1. Nền tảng điều khiển hiện đại từ **`DroneFC_CRSF_DShot`** (sử dụng **Radiomaster RP3 ELRS CRSF** và **GOKU G55M DShot600** qua RMT).
2. Cơ chế **Hiệu chỉnh gia tốc kế (Accelerometer) THỦ CÔNG** từ **`KyThuatUAV_FC_level_1`** (bù sai số tĩnh qua `#define ACC_OFFSET_*_G`, khắc phục hoàn toàn lỗi trôi góc khi cắm pin trên địa hình dốc/cầm tay).

---

## Điểm Khác Biệt Giữa 2 Bản

| Đặc điểm | `DroneFC_CRSF_DShot` (Bản tự động) | `DroneFC_CRSF_DShot_ManualAcc` (Bản này) |
|---|---|---|
| **Hiệu chỉnh Accel** | Tự động đo 500 mẫu mỗi lần cắm pin | **Thủ công 1 lần**: lưu 3 hằng số vào code |
| **Yêu cầu khi cắm pin** | Bắt buộc drone phải nằm phẳng tuyệt đối | **Không cần phẳng**: cắm pin góc nào cũng bay chuẩn |
| **Hiệu chỉnh Gyro** | Tự động khi khởi động | Tự động khi khởi động (chỉ cần đứng yên 1-2s) |
| **Giao thức RC** | CRSF (420000 baud, UART2) | CRSF (420000 baud, UART2) |
| **Giao thức ESC** | DShot600 (RMT) | DShot600 (RMT) |
| **Mixer Quad X** | Chuẩn Betaflight / GOKU G55M | Chuẩn Betaflight / GOKU G55M |

---

## Linh Kiện Sử Dụng

| Thành phần | Linh kiện | Giao tiếp |
|---|---|---|
| Vi điều khiển | ESP32 WROOM32 38Pin | — |
| Cảm biến quán tính | ICM-20602 | SPI, 10 MHz |
| Cảm biến áp suất | BMP388 | I2C, 400 kHz |
| Bộ thu RC | Radiomaster RP3 ExpressLRS 2.4GHz | **CRSF**, UART2, 420000 baud |
| ESC | GOKU G55M 4-in-1 55A BLHeli32 | **DShot600**, RMT |
| Động cơ | RS2205 2300KV | 4 motor |
| Khung & Cánh | 5 inch | — |
| Pin | LiPo 3S 2200mAh | — |
| Tay điều khiển | Pocket Crush ELRS 2.4GHz (EdgeTX) | — |

---

## Hướng Dẫn Hiệu Chỉnh Gia Tốc Kế

Chi tiết xem tại file: [**HUONG_DAN_HIEU_CHINH_ACC.md**](HUONG_DAN_HIEU_CHINH_ACC.md)

1. Mở file `DroneFC_CRSF_DShot_ManualAcc.ino`, bật `#define CALIBRATE_ACCEL`.
2. Đặt drone lên bàn thật phẳng, mở Serial Monitor (500000 baud).
3. Đợi đo 2000 mẫu, copy 3 dòng `#define ACC_OFFSET_*_G` in ra dán đè vào mục Cấu hình trong `DroneFC_CRSF_DShot_ManualAcc.ino`.
4. Comment lại `// #define CALIBRATE_ACCEL` và nạp lại code.

---

## Cấu Trúc Mã Nguồn

```
DroneFC_CRSF_DShot_ManualAcc/
├── DroneFC_CRSF_DShot_ManualAcc.ino   Khối cấu hình (chứa 3 offset acc) + setup + RTOS tasks
├── control_task.ino                   Vòng điều khiển chính (500Hz), mixer quad X
├── imu_icm20602.ino                   Driver ICM20602 + bù lệch acc thủ công + cali routine
├── baro_bmp388.ino                    Driver BMP388 I2C
├── altitude_kf.ino                    Bộ lọc Kalman 2 trạng thái ước lượng độ cao
├── pid.ino                            Bộ điều khiển PID xếp tầng (Cascade PID)
├── rc_crsf.ino                        Driver CRSF cho Radiomaster RP3 ELRS
├── esc_dshot.ino                      Driver DShot600 qua ESP32 RMT peripheral
├── types.h                            Các cấu trúc dữ liệu dùng chung
├── HUONG_DAN_HIEU_CHINH_ACC.md        Hướng dẫn chi tiết hiệu chỉnh gia tốc kế
└── README.md                          Tài liệu tổng quan này
```

---

## Sơ Đồ Chân Kết Nối (ESP32 WROOM32 38Pin)

```
ESP32 WROOM32 38Pin
├── GPIO27 ──→ GOKU G55M Motor 1 signal (RR, CW)
├── GPIO26 ──→ GOKU G55M Motor 2 signal (FR, CCW)
├── GPIO25 ──→ GOKU G55M Motor 3 signal (RL, CCW)
├── GPIO33 ──→ GOKU G55M Motor 4 signal (FL, CW)
├── GPIO5  ──→ ICM-20602 CS (SPI)
├── GPIO18 ──→ ICM-20602 SCK (SPI) [tự động]
├── GPIO23 ──→ ICM-20602 MOSI (SPI) [tự động]
├── GPIO19 ──→ ICM-20602 MISO (SPI) [tự động]
├── GPIO14 ──→ LED trạng thái / cali
├── GPIO16 ──→ RP3 TX (CRSF RX)
├── GPIO17 ──→ RP3 RX (CRSF TX, telemetry)
├── GPIO21 ──→ BMP388 SDA (I2C)
├── GPIO22 ──→ BMP388 SCL (I2C)
├── 5V     ←── Flywoo BEC 5V output
└── GND    ──→ GND chung
```

---

## Giấy Phép & Ghi Công

- Dựa trên mã nguồn gốc của **Nguyễn Văn Quý** — [Kỹ Thuật UAV](https://github.com/CarbonAeronautics).
- Giấy phép: **GNU General Public License v3.0 (GPL-3.0)**.
