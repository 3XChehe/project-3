// ============================================================================
//  DroneFC_CRSF_DShot — Giải mã CRSF (Crossfire Protocol)
//
//  Dựa trên mã nguồn gốc: KyThuatUAV FC — Copyright (C) 2026 Nguyễn Văn Quý
//  Chỉnh sửa: thay SBUS bằng CRSF cho Radiomaster RP3 ExpressLRS
//  License: GPL-3.0
// ============================================================================
// ============================================================================
//  Giải mã CRSF (Crossfire Serial Protocol).
//
//  CRSF là giao thức nối tiếp KHÔNG ĐẢO, 420000 baud, 8N1.
//  Radiomaster RP3 ELRS 2.4GHz xuất tín hiệu CRSF qua chân TX.
//  Nối TX của RP3 vào RX (GPIO16) của ESP32 UART2.
//  Nối RX của RP3 vào TX (GPIO17) của ESP32 UART2 (cho telemetry).
//
//  Cấu trúc khung CRSF:
//    [addr] [len] [type] [payload...] [crc8]
//    - addr: 0xC8 (flight controller) hoặc 0xEE (broadcast)
//    - len:  tổng số byte từ type tới hết crc (bao gồm cả type và crc)
//    - type: 0x16 = RC channels packed
//    - payload: 16 kênh x 11 bit = 22 byte
//    - crc8: CRC-8/DVB-S2 từ type tới hết payload
//
//  Giá trị kênh CRSF: 172-1811 (giống SBUS), quy đổi sang 990-2010.
//
//  ELRS ở 500Hz gửi frame mỗi 2ms — đồng bộ tự nhiên với rc_task 500Hz.
// ============================================================================

#include <HardwareSerial.h>

HardwareSerial crsf_uart(2);   // UART2

// ---- Hằng số giao thức CRSF ------------------------------------------------
#define CRSF_BAUDRATE          420000
#define CRSF_MAX_FRAME_LEN     64
#define CRSF_ADDR_FC           0xC8      // địa chỉ flight controller
#define CRSF_ADDR_BROADCAST    0xEE
#define CRSF_TYPE_RC_CHANNELS  0x16      // loại frame chứa dữ liệu kênh
#define CRSF_CHANNEL_COUNT     16
#define CRSF_CHANNEL_BITS      11

// Quy đổi giá trị kênh
#define CRSF_RAW_MIN           172
#define CRSF_RAW_MAX           1811
#define CRSF_OUT_MIN           990       // quy đổi ra dải quen thuộc 1000-2000
#define CRSF_OUT_MAX           2010
#define CRSF_TIMEOUT_MS        200       // quá thời gian này = mất sóng

// ---- Biến nội bộ ------------------------------------------------------------
static uint8_t  crsf_buf[CRSF_MAX_FRAME_LEN];
static uint8_t  crsf_buf_idx = 0;
static uint8_t  crsf_frame_len = 0;      // len field từ frame hiện tại
static bool     crsf_in_frame = false;

static uint16_t crsf_raw_ch[CRSF_CHANNEL_COUNT];   // giá trị thô 172-1811
static int      crsf_ch_us[17];                      // quy đổi sang 990-2010, index 1-16
static unsigned long crsf_last_frame_ms = 0;


// ---- CRC-8 DVB-S2 ----------------------------------------------------------
// Bảng tra CRC tính trước, đa thức 0xD5 (chuẩn DVB-S2), dùng cho CRSF.
static const uint8_t crsf_crc8_tab[256] = {
    0x00,0xD5,0x7F,0xAA,0xFE,0x2B,0x81,0x54,
    0x29,0xFC,0x56,0x83,0xD7,0x02,0xA8,0x7D,
    0x52,0x87,0x2D,0xF8,0xAC,0x79,0xD3,0x06,
    0x7B,0xAE,0x04,0xD1,0x85,0x50,0xFA,0x2F,
    0xA4,0x71,0xDB,0x0E,0x5A,0x8F,0x25,0xF0,
    0x8D,0x58,0xF2,0x27,0x73,0xA6,0x0C,0xD9,
    0xF6,0x23,0x89,0x5C,0x08,0xDD,0x77,0xA2,
    0xDF,0x0A,0xA0,0x75,0x21,0xF4,0x5E,0x8B,
    0x9D,0x48,0xE2,0x37,0x63,0xB6,0x1C,0xC9,
    0xB4,0x61,0xCB,0x1E,0x4A,0x9F,0x35,0xE0,
    0xCF,0x1A,0xB0,0x65,0x31,0xE4,0x4E,0x9B,
    0xE6,0x33,0x99,0x4C,0x18,0xCD,0x67,0xB2,
    0x39,0xEC,0x46,0x93,0xC7,0x12,0xB8,0x6D,
    0x10,0xC5,0x6F,0xBA,0xEE,0x3B,0x91,0x44,
    0x6B,0xBE,0x14,0xC1,0x95,0x40,0xEA,0x3F,
    0x42,0x97,0x3D,0xE8,0xBC,0x69,0xC3,0x16,
    0xEF,0x3A,0x90,0x45,0x11,0xC4,0x6E,0xBB,
    0xC6,0x13,0xB9,0x6C,0x38,0xED,0x47,0x92,
    0xBD,0x68,0xC2,0x17,0x43,0x96,0x3C,0xE9,
    0x94,0x41,0xEB,0x3E,0x6A,0xBF,0x15,0xC0,
    0x4B,0x9E,0x34,0xE1,0xB5,0x60,0xCA,0x1F,
    0x62,0xB7,0x1D,0xC8,0x9C,0x49,0xE3,0x36,
    0x19,0xCC,0x66,0xB3,0xE7,0x32,0x98,0x4D,
    0x30,0xE5,0x4F,0x9A,0xCE,0x1B,0xB1,0x64,
    0xD2,0x07,0xAD,0x78,0x2C,0xF9,0x53,0x86,
    0xFB,0x2E,0x84,0x51,0x05,0xD0,0x7A,0xAF,
    0x80,0x55,0xFF,0x2A,0x7E,0xAB,0x01,0xD4,
    0xA9,0x7C,0xD6,0x03,0x57,0x82,0x28,0xFD,
    0x76,0xA3,0x09,0xDC,0x88,0x5D,0xF7,0x22,
    0x5F,0x8A,0x20,0xF5,0xA1,0x74,0xDE,0x0B,
    0x24,0xF1,0x5B,0x8E,0xDA,0x0F,0xA5,0x70,
    0x0D,0xD8,0x72,0xA7,0xF3,0x26,0x8C,0x59
};

static uint8_t crsf_crc8(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++) {
    crc = crsf_crc8_tab[crc ^ data[i]];
  }
  return crc;
}


// ---- Khởi tạo UART cho CRSF ------------------------------------------------
void rc_init() {
  // CRSF: 420000 baud, 8N1, KHÔNG đảo tín hiệu
  crsf_uart.begin(CRSF_BAUDRATE, SERIAL_8N1, PIN_CRSF_RX, PIN_CRSF_TX);

  // Xoá buffer ban đầu
  while (crsf_uart.available()) crsf_uart.read();

  crsf_buf_idx = 0;
  crsf_in_frame = false;

  // Khởi tạo kênh về giá trị giữa
  for (int i = 0; i < CRSF_CHANNEL_COUNT; i++) {
    crsf_raw_ch[i] = 992;     // CRSF mid = 992
  }
  for (int i = 0; i <= 16; i++) {
    crsf_ch_us[i] = 1500;
  }
}


// ---- Tách 16 kênh từ payload 22 byte ----------------------------------------
// Cấu trúc giống hệt SBUS: 16 kênh x 11 bit = 176 bit = 22 byte, xếp liên tục.
static void crsf_decode_channels(const uint8_t *payload) {
  crsf_raw_ch[0]  = ((uint16_t)payload[0]       | ((uint16_t)payload[1]  << 8))  & 0x07FF;
  crsf_raw_ch[1]  = ((uint16_t)payload[1]  >> 3 | ((uint16_t)payload[2]  << 5))  & 0x07FF;
  crsf_raw_ch[2]  = ((uint16_t)payload[2]  >> 6 | ((uint16_t)payload[3]  << 2)
                                                 | ((uint16_t)payload[4]  << 10)) & 0x07FF;
  crsf_raw_ch[3]  = ((uint16_t)payload[4]  >> 1 | ((uint16_t)payload[5]  << 7))  & 0x07FF;
  crsf_raw_ch[4]  = ((uint16_t)payload[5]  >> 4 | ((uint16_t)payload[6]  << 4))  & 0x07FF;
  crsf_raw_ch[5]  = ((uint16_t)payload[6]  >> 7 | ((uint16_t)payload[7]  << 1)
                                                 | ((uint16_t)payload[8]  << 9))  & 0x07FF;
  crsf_raw_ch[6]  = ((uint16_t)payload[8]  >> 2 | ((uint16_t)payload[9]  << 6))  & 0x07FF;
  crsf_raw_ch[7]  = ((uint16_t)payload[9]  >> 5 | ((uint16_t)payload[10] << 3))  & 0x07FF;
  crsf_raw_ch[8]  = ((uint16_t)payload[11]      | ((uint16_t)payload[12] << 8))  & 0x07FF;
  crsf_raw_ch[9]  = ((uint16_t)payload[12] >> 3 | ((uint16_t)payload[13] << 5))  & 0x07FF;
  crsf_raw_ch[10] = ((uint16_t)payload[13] >> 6 | ((uint16_t)payload[14] << 2)
                                                 | ((uint16_t)payload[15] << 10)) & 0x07FF;
  crsf_raw_ch[11] = ((uint16_t)payload[15] >> 1 | ((uint16_t)payload[16] << 7))  & 0x07FF;
  crsf_raw_ch[12] = ((uint16_t)payload[16] >> 4 | ((uint16_t)payload[17] << 4))  & 0x07FF;
  crsf_raw_ch[13] = ((uint16_t)payload[17] >> 7 | ((uint16_t)payload[18] << 1)
                                                 | ((uint16_t)payload[19] << 9))  & 0x07FF;
  crsf_raw_ch[14] = ((uint16_t)payload[19] >> 2 | ((uint16_t)payload[20] << 6))  & 0x07FF;
  crsf_raw_ch[15] = ((uint16_t)payload[20] >> 5 | ((uint16_t)payload[21] << 3))  & 0x07FF;
}


// ---- Đọc và xử lý frame CRSF -----------------------------------------------
// Máy trạng thái gom byte:
//   1. Chờ byte addr (0xC8 hoặc 0xEE)
//   2. Đọc byte len
//   3. Gom đúng len byte (type + payload + crc)
//   4. Kiểm tra CRC
//   5. Nếu type = 0x16 thì giải mã kênh
static bool crsf_process_byte(uint8_t b) {
  if (!crsf_in_frame) {
    // Chờ byte địa chỉ
    if (b == CRSF_ADDR_FC || b == CRSF_ADDR_BROADCAST) {
      crsf_buf[0] = b;
      crsf_buf_idx = 1;
      crsf_in_frame = true;
    }
    return false;
  }

  crsf_buf[crsf_buf_idx++] = b;

  // Byte thứ 2: len
  if (crsf_buf_idx == 2) {
    crsf_frame_len = b;
    // Kiểm tra len hợp lệ (tối thiểu 2: type + crc, tối đa 62)
    if (crsf_frame_len < 2 || crsf_frame_len > CRSF_MAX_FRAME_LEN - 2) {
      crsf_in_frame = false;
      crsf_buf_idx = 0;
    }
    return false;
  }

  // Chưa đủ frame
  if (crsf_buf_idx < (uint8_t)(crsf_frame_len + 2)) {
    return false;
  }

  // Đủ frame — kiểm tra CRC
  crsf_in_frame = false;

  // CRC tính từ type (byte index 2) đến hết payload (không bao gồm crc)
  uint8_t crc_calc = crsf_crc8(&crsf_buf[2], crsf_frame_len - 1);
  uint8_t crc_recv = crsf_buf[crsf_frame_len + 1];

  if (crc_calc != crc_recv) {
    return false;
  }

  // Frame hợp lệ — kiểm tra type
  uint8_t frame_type = crsf_buf[2];
  if (frame_type == CRSF_TYPE_RC_CHANNELS) {
    // Payload bắt đầu từ byte index 3 (sau addr, len, type)
    crsf_decode_channels(&crsf_buf[3]);
    return true;
  }

  return false;   // frame hợp lệ nhưng không phải RC channels
}


// ---- API công khai (cùng giao diện với bản SBUS cũ) -------------------------

// Trả về true khi vẫn còn sóng, false khi mất sóng.
bool rc_update() {
  bool got_channels = false;

  while (crsf_uart.available()) {
    uint8_t b = crsf_uart.read();
    if (crsf_process_byte(b)) {
      got_channels = true;
    }
  }

  if (got_channels) {
    // Quy đổi 16 kênh từ 172-1811 sang 990-2010
    for (int i = 0; i < CRSF_CHANNEL_COUNT; i++) {
      crsf_ch_us[i + 1] = map(crsf_raw_ch[i], CRSF_RAW_MIN, CRSF_RAW_MAX,
                               CRSF_OUT_MIN, CRSF_OUT_MAX);
    }
    crsf_last_frame_ms = millis();
  }

  return (millis() - crsf_last_frame_ms <= CRSF_TIMEOUT_MS);
}

// Lấy giá trị kênh (index 1-16), đã quy đổi sang 990-2010
int rc_get_channel(int index) {
  if (index < 1 || index > 16) return 1500;
  return crsf_ch_us[index];
}
