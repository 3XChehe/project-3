// ============================================================================
//  DroneFC_CRSF_DShot — Xuất DShot600 cho 4 ESC
//
//  Dựa trên mã nguồn gốc: KyThuatUAV FC — Copyright (C) 2026 Nguyễn Văn Quý
//  Chỉnh sửa: thay PWM analog bằng DShot600 cho GOKU G55M 4-in-1 ESC BLHeli32
//  License: GPL-3.0
// ============================================================================
// ============================================================================
//  DShot600 — giao thức tín hiệu số cho ESC.
//
//  GOKU G55M 4-in-1 ESC 55A BLHeli32 hỗ trợ DShot150/300/600/1200.
//  Chọn DShot600 vì cân bằng giữa tốc độ và độ tin cậy trên ESP32.
//
//  Cấu trúc frame DShot:
//    - 16 bit: [11 bit throttle] [1 bit telemetry request] [4 bit CRC]
//    - Tổng cộng 16 bit, mỗi bit được mã hoá bằng độ rộng xung
//    - DShot600: tốc độ bit 600 kbit/s → mỗi bit ≈ 1.67 µs
//      - bit '1': high 1.25 µs, low 0.42 µs (T1H ≈ 75% duty)
//      - bit '0': high 0.625 µs, low 1.045 µs (T0H ≈ 37.5% duty)
//
//  Giá trị throttle DShot:
//    0     = disarmed (motor tắt)
//    1-47  = lệnh đặc biệt (beep, đảo chiều, v.v.)
//    48    = motor quay idle (ga thấp nhất khi armed)
//    2047  = ga tối đa
//
//  Triển khai: dùng RMT (Remote Control Transceiver) của ESP32.
//  ESP32 có 8 kênh RMT, mỗi kênh phát được chuỗi xung chính xác mà không
//  chiếm CPU. Hoàn hảo cho DShot vì timing rất chặt.
//
//  LƯU Ý KHI DÙNG VỚI GOKU G55M:
//  - ESC 4-in-1 có 4 pad tín hiệu motor (M1-M4), mỗi pad nối vào 1 chân GPIO
//  - Cấp 5V cho ESP32 từ BEC module Flywoo 5V/2A (KHÔNG lấy từ ESC trực tiếp)
//  - GND của ESC và ESP32 phải nối chung
//  - Đầu tiên cấu hình ESC qua BLHeli Configurator: chọn DShot600
// ============================================================================

#include <driver/rmt.h>

// ---- Thông số DShot600 cho RMT ----------------------------------------------
// RMT clock mặc định 80 MHz → 1 tick = 12.5 ns
// DShot600: bit period = 1.67 µs = 133.6 ticks (~134 ticks)

#define DSHOT_T1H_TICKS     100   // bit '1' high duration: 1.25 µs = 100 ticks
#define DSHOT_T1L_TICKS      34   // bit '1' low duration:  0.42 µs = 34 ticks
#define DSHOT_T0H_TICKS      50   // bit '0' high duration: 0.625 µs = 50 ticks
#define DSHOT_T0L_TICKS      84   // bit '0' low duration:  1.045 µs = 84 ticks

#define DSHOT_FRAME_BITS     16
#define DSHOT_THROTTLE_MIN   48     // giá trị throttle nhỏ nhất khi armed
#define DSHOT_THROTTLE_MAX   2047

// Mapping RMT channels cho 4 ESC
static const rmt_channel_t dshot_rmt_ch[4] = {
  RMT_CHANNEL_0,   // Motor 1
  RMT_CHANNEL_1,   // Motor 2
  RMT_CHANNEL_2,   // Motor 3
  RMT_CHANNEL_3    // Motor 4
};

static const gpio_num_t dshot_pins[4] = {
  (gpio_num_t)PIN_ESC_1,
  (gpio_num_t)PIN_ESC_2,
  (gpio_num_t)PIN_ESC_3,
  (gpio_num_t)PIN_ESC_4
};

// Buffer cho RMT items (16 bit + 1 end marker)
static rmt_item32_t dshot_items[4][DSHOT_FRAME_BITS + 1];


// ---- Tính CRC DShot ---------------------------------------------------------
// CRC = XOR của ba nhóm 4 bit: (throttle >> 7) ^ (throttle >> 3) ^ (throttle & 0x0F)
// Nhưng thực tế tính trên 12 bit gốc (11 bit throttle + 1 bit telemetry):
//   value = (throttle << 1) | telem
//   crc = (value ^ (value >> 4) ^ (value >> 8)) & 0x0F
static uint16_t dshot_encode(uint16_t throttle, bool telem_request) {
  uint16_t value = (throttle << 1) | (telem_request ? 1 : 0);
  uint16_t crc = (value ^ (value >> 4) ^ (value >> 8)) & 0x0F;
  return (value << 4) | crc;
}


// ---- Chuyển frame 16 bit thành RMT items ------------------------------------
static void dshot_build_rmt_items(rmt_item32_t *items, uint16_t frame) {
  for (int i = 0; i < DSHOT_FRAME_BITS; i++) {
    // MSB first
    if (frame & (1 << (15 - i))) {
      // Bit '1'
      items[i].duration0 = DSHOT_T1H_TICKS;
      items[i].level0    = 1;
      items[i].duration1 = DSHOT_T1L_TICKS;
      items[i].level1    = 0;
    } else {
      // Bit '0'
      items[i].duration0 = DSHOT_T0H_TICKS;
      items[i].level0    = 1;
      items[i].duration1 = DSHOT_T0L_TICKS;
      items[i].level1    = 0;
    }
  }
  // End marker
  items[DSHOT_FRAME_BITS].duration0 = 0;
  items[DSHOT_FRAME_BITS].level0    = 0;
  items[DSHOT_FRAME_BITS].duration1 = 0;
  items[DSHOT_FRAME_BITS].level1    = 0;
}


// ---- Gửi DShot frame cho 1 ESC qua RMT -------------------------------------
static void dshot_send_one(int motor_idx, uint16_t throttle) {
  uint16_t frame = dshot_encode(throttle, false);  // không yêu cầu telemetry
  dshot_build_rmt_items(dshot_items[motor_idx], frame);
  rmt_write_items(dshot_rmt_ch[motor_idx], dshot_items[motor_idx],
                  DSHOT_FRAME_BITS + 1, false);    // false = không chờ xong
}


// ---- Chuyển đổi từ thang nội bộ 800-1600 sang DShot 0-2047 -----------------
// Giữ nguyên thang 800-1600 trong toàn bộ code điều khiển (PID, mixer, v.v.)
// để không phải sửa logic đã kiểm chứng. Chỉ chuyển đổi ở đây trước khi gửi.
static uint16_t internal_to_dshot(int internal_value) {
  if (internal_value <= ESC_IDLE) return 0;         // disarmed
  if (internal_value < ESC_MIN_ARMED) return 0;     // dưới mức arm

  // Map ESC_MIN_ARMED..ESC_MAX → DSHOT_THROTTLE_MIN..DSHOT_THROTTLE_MAX
  long dshot_val = map((long)internal_value, ESC_MIN_ARMED, ESC_MAX,
                       DSHOT_THROTTLE_MIN, DSHOT_THROTTLE_MAX);

  if (dshot_val < DSHOT_THROTTLE_MIN) dshot_val = DSHOT_THROTTLE_MIN;
  if (dshot_val > DSHOT_THROTTLE_MAX) dshot_val = DSHOT_THROTTLE_MAX;

  return (uint16_t)dshot_val;
}


// ============================================================================
//  API công khai — cùng giao diện với bản PWM cũ
// ============================================================================

void esc_init() {
  // Cấu hình RMT cho 4 kênh DShot
  for (int i = 0; i < 4; i++) {
    rmt_config_t cfg = RMT_DEFAULT_CONFIG_TX(dshot_pins[i], dshot_rmt_ch[i]);
    cfg.clk_div = 1;                // clock divider = 1 → 80 MHz, 12.5 ns/tick
    cfg.mem_block_num = 1;           // 1 block = 64 items, đủ cho 16 bit DShot
    cfg.tx_config.carrier_en = false;
    cfg.tx_config.loop_en = false;
    cfg.tx_config.idle_output_en = true;
    cfg.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;

    rmt_config(&cfg);
    rmt_driver_install(dshot_rmt_ch[i], 0, 0);
  }

  // Gửi lệnh disarm (throttle = 0) vài lần để ESC nhận ra DShot
  for (int j = 0; j < 10; j++) {
    for (int i = 0; i < 4; i++) {
      dshot_send_one(i, 0);
    }
    delay(10);
  }

  // Chờ ESC khởi tạo (thường cần 2-3 giây để nhận DShot và arm)
  for (int j = 0; j < 300; j++) {
    esc_write(ESC_IDLE, ESC_IDLE, ESC_IDLE, ESC_IDLE);
    delay(10);
  }

  Serial.println("[DShot600 4 kenh khoi tao xong]");

  // THÁO CÁNH rồi mở khối này để dò chân nào ra motor nào.
  // Mỗi motor sẽ quay nhẹ lần lượt theo đúng thứ tự 1 → 2 → 3 → 4.
    esc_write(ESC_MIN_ARMED, ESC_IDLE, ESC_IDLE, ESC_IDLE); delay(4000);
    esc_write(ESC_IDLE, ESC_MIN_ARMED, ESC_IDLE, ESC_IDLE); delay(4000);
    esc_write(ESC_IDLE, ESC_IDLE, ESC_MIN_ARMED, ESC_IDLE); delay(4000);
    esc_write(ESC_IDLE, ESC_IDLE, ESC_IDLE, ESC_MIN_ARMED); delay(4000);
    esc_write(ESC_IDLE, ESC_IDLE, ESC_IDLE, ESC_IDLE);      delay(1000);
}


void esc_write(int m1, int m2, int m3, int m4) {
  dshot_send_one(0, internal_to_dshot(m1));
  dshot_send_one(1, internal_to_dshot(m2));
  dshot_send_one(2, internal_to_dshot(m3));
  dshot_send_one(3, internal_to_dshot(m4));
}
