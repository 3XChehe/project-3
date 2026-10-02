# HƯỚNG DẪN LẮP RÁP, CẤU HÌNH VÀ NẠP FIRMWARE DRONE QUADCOPTER ESP32

Tài liệu kỹ thuật chi tiết dành cho dự án bay Quadcopter sử dụng bộ linh kiện thực tế:
- **Board điều khiển**: ESP32 WROOM32 (38 Pin)
- **IMU**: ICM-20602 (SPI 10MHz)
- **Barometer**: BMP388 (I2C)
- **ESC**: GOKU G55M 4-in-1 55A BLHeli_32 (DShot600)
- **Motor**: 4x RS2205 2300KV
- **Cánh**: 5 inch (2 cặp CW/CCW)
- **Khung**: 5 inch X-Frame (kèm đệm chống rung cao su M3)
- **Pin**: LiPo 3S 2200mAh 75C
- **Nguồn hạ áp**: Flywoo BEC 5V/12V 2A V2.0 (cấp 5V cho FC và cảm biến)
- **Mạch thu RC**: Radiomaster RP3 ExpressLRS 2.4GHz Diversity
- **Tay điều khiển**: Radiomaster Pocket Crush ELRS 2.4GHz (EdgeTX)

---

## I. SƠ ĐỒ ĐẤU DÂY CHI TIẾT (WIRING DIAGRAM)

### 1. Phân phối nguồn điện (Power Distribution)
> ⚠️ **CỰC KỲ QUAN TRỌNG**: Động cơ và ESC tiêu thụ dòng điện rất lớn (lên tới 30-50A khi thốc ga), gây sụt áp và nhiễu xung nhọn rất mạnh trên đường pin. **TUYỆT ĐỐI KHÔNG** cấp nguồn cho ESP32 và cảm biến trực tiếp từ chân 5V của ESC nếu không qua mạch lọc/BEC riêng.

- **Dây nguồn chính từ Pin LiPo 3S (11.1V - 12.6V)**:
  - Cực `+` (Đỏ) và `-` (Đen) từ giắc XT60 hàn trực tiếp vào 2 pad nguồn `BAT+` và `GND` của ESC GOKU G55M.
  - Hàn song song 1 tụ điện chống nhiễu (Low ESR Electrolytic Capacitor 35V 470µF-1000µF) ngay tại 2 cực nguồn của ESC.
  - Từ 2 cực nguồn `BAT+` và `GND` của ESC, trích 2 dây nhỏ (khoảng 22-24 AWG) dẫn vào chân `VIN` và `GND` của module **Flywoo BEC**.
- **Flywoo BEC 5V/2A**:
  - Gạt jumper/hàn pad chọn mức điện áp đầu ra là **5V** (không để 12V).
  - Đầu ra `5V` của BEC cấp vào chân `5V` (hoặc `VIN`) của ESP32.
  - Đầu ra `GND` của BEC nối vào chân `GND` của ESP32.
  - Cấp 5V và GND từ BEC sang mạch thu **Radiomaster RP3**.
  - Cấp 3.3V từ chân `3V3` của ESP32 sang module **ICM-20602** và **BMP388** (các cảm biến chạy chuẩn logic 3.3V).

---

### 2. Sơ đồ nối chân ESP32 WROOM32 (38 Pin)

```
                            ESP32 WROOM32 (38 Pin)
                               +----------------+
                               | 3V3        GND |--- GND chung
            GOKU G55M M1 <-----| GPIO27     GPIO23 |----> ICM-20602 MOSI (SPI)
            GOKU G55M M2 <-----| GPIO26     GPIO22 |----> BMP388 SCL (I2C)
            GOKU G55M M3 <-----| GPIO25     GPIO21 |----> BMP388 SDA (I2C)
            GOKU G55M M4 <-----| GPIO33     GPIO19 |<---- ICM-20602 MISO (SPI)
         ICM-20602 CS(SPI) <---| GPIO5      GPIO18 |----> ICM-20602 SCK (SPI)
          LED trạng thái <-----| GPIO14     GPIO5  |
           RP3 TX (CRSF) ----->| GPIO16(RX2)GPIO17 |----> RP3 RX (CRSF Telem - tuỳ chọn)
            Flywoo BEC 5V ---->| VIN/5V      EN |
                               +----------------+
```

| Tên Module | Chân trên Module | Chân kết nối trên ESP32 | Chú thích |
|---|---|---|---|
| **GOKU G55M ESC** | Motor 1 (M1) | **GPIO 27** | DShot600 Channel 0 |
| | Motor 2 (M2) | **GPIO 26** | DShot600 Channel 1 |
| | Motor 3 (M3) | **GPIO 25** | DShot600 Channel 2 |
| | Motor 4 (M4) | **GPIO 33** | DShot600 Channel 3 |
| | GND | **GND** | Phải nối chung GND với ESP32 |
| **ICM-20602 (IMU)** | VCC | **3V3** | Nguồn 3.3V ổn định từ ESP32 |
| | GND | **GND** | GND |
| | CS | **GPIO 5** | SPI Chip Select |
| | SCK / SCL | **GPIO 18** | VSPI Clock |
| | SDI / MOSI | **GPIO 23** | VSPI Data In (MOSI) |
| | SDO / MISO | **GPIO 19** | VSPI Data Out (MISO) |
| **BMP388 (Baro)** | VCC | **3V3** | Nguồn 3.3V |
| | GND | **GND** | GND |
| | SDA | **GPIO 21** | I2C Data |
| | SCL | **GPIO 22** | I2C Clock |
| | SDO | **GND** | Đặt địa chỉ I2C là `0x76` (nếu nối 3.3V là 0x77) |
| **Radiomaster RP3** | 5V | **5V (từ BEC)** | Nguồn 5V cho RX |
| | GND | **GND** | GND |
| | TX | **GPIO 16 (RX2)** | CRSF Serial RX (tín hiệu điều khiển) |
| | RX | **GPIO 17 (TX2)** | CRSF Serial TX (Telemetry) |
| **Chỉ báo LED** | LED Anode (+) | **GPIO 14** (qua trở 330Ω)| LED báo hiệu trạng thái cân bằng gyro |

---

## II. THỨ TỰ ĐỘNG CƠ VÀ CHIỀU QUAY (QUAD X)

Firmware tuân theo chuẩn Quad X Betaflight:

```
        ĐẦU DRONE (HƯỚNG BAY TỚI)
               ▲
               │
    M3 (CCW) ┌───┐ M1 (CW)
         \   │   │   /
          \  └───┘  /
           \       /
            \     /
             \   /
              \ /
              / \
             /   \
            /     \
           /       \
          /  ┌───┐  \
         /   │   │   \
    M2 (CW)  └───┘ M4 (CCW)
               │
              ĐUÔI
```

- **M1 (Trước Phải)**: Quay ngược chiều kim đồng hồ (**CCW**) — Cánh CCW
- **M2 (Sau Trái)**: Quay ngược chiều kim đồng hồ (**CCW**) — Cánh CCW
- **M3 (Trước Trái)**: Quay thuận chiều kim đồng hồ (**CW**) — Cánh CW
- **M4 (Sau Phải)**: Quay thuận chiều kim đồng hồ (**CW**) — Cánh CW

> **Lưu ý**: Cấu hình chiều quay này KHÁC chuẩn Betaflight Quad X. Ma trận mixer trong `control_task.ino` đã được điều chỉnh phù hợp (đảo dấu u_yaw).

> **Đảo chiều động cơ**: Vì ESC GOKU G55M chạy firmware BLHeli_32, bạn có thể đảo chiều bất kỳ động cơ nào bằng **BLHeli Configurator** trên máy tính mà không cần phải tháo mối hàn dây motor!

---

## III. THIẾT LẬP TAY ĐIỀU KHIỂN RADIOMASTER POCKET (EDGETX + ELRS)

### 1. Thứ tự kênh (Channel Order)
Trên tay cầm Radiomaster Pocket (chạy EdgeTX), vào trang **MDL -> MIXER**:
- **CH1**: `Ail` (Roll) — Cần phải gạt trái/phải
- **CH2**: `Ele` (Pitch) — Cần phải gạt lên/xuống
- **CH3**: `Thr` (Throttle) — Cần trái gạt lên/xuống (Mode 2)
- **CH4**: `Rud` (Yaw) — Cần trái gạt trái/phải
- **CH5**: Gán vào công tắc **SA** hoặc **SB** (2 nấc hoặc 3 nấc) để làm **ARM / DISARM**.
  - Gạt xuống (DISARM): Giá trị gửi đi ~1000µs.
  - Gạt lên (ARM): Giá trị gửi đi ~2000µs.
- **CH6**: Gán vào công tắc 3 nấc **SC** để chọn **CHẾ ĐỘ BAY (Flight Mode)**.
  - Nấc 0 (1000µs): Chế độ `ANGLE` (Tự cân bằng góc)
  - Nấc 1 (1500µs): Chế độ `ANGLE`
  - Nấc 2 (2000µs): Chế độ `ALT HOLD` (Giữ độ cao bằng khí áp kế BMP388)

### 2. Cấu hình ExpressLRS
- Chạy Script Lua ExpressLRS trên tay cầm:
  - **Packet Rate**: Chọn `500Hz` (đây là tốc độ tối ưu tương thích hoàn hảo với chu kỳ `rc_task` 500Hz trong code).
  - **Telem Ratio**: Chọn `1:64` hoặc `1:32` hoặc `Off`.
  - **Switch Mode**: Chọn `Wide` hoặc `8ch`.
- **Bind tay cầm với RP3**:
  - Cắm/rút nguồn RP3 3 lần liên tiếp để vào chế độ Binding (LED RP3 nhấp nháy đôi liên tục).
  - Trên tay cầm, chạy Lua script ELRS và chọn `[Bind]`. Khi LED RP3 sáng đứng là đã kết nối thành công.

---

## IV. CÀI ĐẶT MÔI TRƯỜNG VÀ NẠP FIRMWARE

### 1. Chuẩn bị Arduino IDE
1. Mở Arduino IDE (khuyên dùng bản 2.x).
2. Vào `File` -> `Preferences` -> dán URL sau vào ô *Additional boards manager URLs*:
   `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
3. Vào `Tools` -> `Board` -> `Boards Manager...`, tìm `esp32`.
4. **BẮT BUỘC**: Chọn cài phiên bản **`2.0.17`** (Không cài 3.x vì Espressif đã thay đổi toàn bộ API LEDC/RMT ở core 3.x).
5. Cài thư viện: Vào `Tools` -> `Manage Libraries...`, tìm và cài:
   - **`BasicLinearAlgebra`** (phiên bản 3.x hoặc 4.x đều chạy tốt với code này).

### 2. Thiết lập Board trong Arduino IDE
- **Board**: `ESP32 Dev Module` hoặc `ESP32-WROOM-DA Module`
- **CPU Frequency**: `240MHz (WiFi/BT)`
- **Flash Frequency**: `80MHz`
- **Flash Mode**: `QIO` hoặc `DIO`
- **Upload Speed**: `921600`
- **Port**: Chọn đúng cổng COM của chip ESP32 (cài driver CP210x trong thư mục `driver esp32/` nếu máy chưa nhận).

### 3. Nạp code
1. Mở file: `D:\Drone\ESP32_FC\CODE_FC\DroneFC_CRSF_DShot\DroneFC_CRSF_DShot.ino`
2. Bấm nút **Verify (Biên dịch)** để đảm bảo không có lỗi.
3. Cắm cáp USB kết nối ESP32 vào máy tính.
4. Bấm **Upload (Nạp code)**.

---

## V. QUY TRÌNH KIỂM TRA AN TOÀN TRƯỚC KHI BAY (QUAN TRỌNG NHẤT)

> 🔴 **CẢNH BÁO NGUY HIỂM: THÁO TOÀN BỘ CÁNH QUẠT KHI THỰC HIỆN CÁC BƯỚC NÀY!**

### Bước 1: Kiểm tra tay điều khiển (RC Check)
1. Trong file `DroneFC_CRSF_DShot.ino`, bỏ dấu comment dòng:
   ```cpp
   #define DEBUG_RC
   ```
2. Nạp lại code, bật tay cầm, mở Serial Monitor ở baudrate `500000`.
3. Quan sát các giá trị:
   - Gạt cần Roll, Pitch, Throttle, Yaw: giá trị phải biến thiên từ ~1000 đến ~2000 (ở giữa là 1500).
   - Gạt công tắc ARM: `arm` đổi từ `0` sang `2`.
   - Gạt công tắc MODE: `mode` đổi giữa `0`, `1`, `2`.
   - `rc_ok` phải bằng `1`.
4. Comment lại `#define DEBUG_RC` sau khi kiểm tra xong.

### Bước 2: Kiểm tra chiều cảm biến (Sensor Direction Check)
1. Bỏ dấu comment dòng:
   ```cpp
   #define DEBUG_ATTITUDE
   ```
2. Mở Serial Monitor:
   - Nghiêng drone sang phải -> `roll` phải tăng giá trị dương.
   - Chúi mũi drone xuống -> `pitch` phải tăng giá trị âm/dương theo góc tương ứng.
3. Đảm bảo góc ước lượng khớp đúng với chuyển động thực tế.

### Bước 3: Kiểm tra thứ tự và chiều quay động cơ (Motor Direction Check)
1. Trong `esc_dshot.ino`, mở đoạn code test trong hàm `esc_init()`:
   ```cpp
   esc_write(ESC_MIN_ARMED, ESC_IDLE, ESC_IDLE, ESC_IDLE); delay(400); // M1 quay
   esc_write(ESC_IDLE, ESC_MIN_ARMED, ESC_IDLE, ESC_IDLE); delay(400); // M2 quay
   esc_write(ESC_IDLE, ESC_IDLE, ESC_MIN_ARMED, ESC_IDLE); delay(400); // M3 quay
   esc_write(ESC_IDLE, ESC_IDLE, ESC_IDLE, ESC_MIN_ARMED); delay(400); // M4 quay
   ```
2. Cấp nguồn pin (chưa lắp cánh), quan sát từng motor quay lần lượt 1 -> 2 -> 3 -> 4.
3. Kiểm tra chiều quay: nếu motor nào quay ngược chiều so với sơ đồ Quad X, mở **BLHeli Configurator** để đảo chiều motor đó.
4. Đóng lại khối test trong `esc_init()` trước khi bay.

### Bước 4: Chuyến bay đầu tiên (Maiden Flight)
1. Luôn lắp cánh quạt đúng chiều (chữ trên mặt cánh hướng lên trên).
2. Tìm bãi cỏ trống rộng, không có người xung quanh.
3. Có thể buộc dây neo nhẹ dưới chân drone trong lần nhấc bổng đầu tiên để kiểm tra độ ổn định.
4. Để drone ở mặt phẳng tĩnh 3-5 giây sau khi cắm pin để chip gyro tự cali bias hoàn tất (LED tắt).
5. Gạt cần ga xuống đáy -> Gạt công tắc ARM -> Tăng nhẹ ga để cất cánh ở chế độ `ANGLE`.
