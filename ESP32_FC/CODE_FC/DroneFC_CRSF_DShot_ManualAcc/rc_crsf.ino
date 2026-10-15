// ============================================================================
// rc_crsf.ino
// ============================================================================
// DroneFC_CRSF_DShot — CRSF Receiver Driver
//
// Thay thế SBUS receiver bằng CRSF cho RadioMaster RP3 ExpressLRS.
//
// RP3:
//   TX -> ESP32 GPIO16 (UART2 RX)
//   RX -> ESP32 GPIO17 (UART2 TX, telemetry)
//   GND -> ESP32 GND
//
// CRSF:
//   Baudrate : 420000
//   Format   : 8N1
//   Signal   : NON-INVERTED
//
// RC frame:
//   Address  : 0xC8
//   Length   : 24
//   Type     : 0x16
//   Payload  : 22 bytes = 16 channels x 11 bits
//   CRC      : CRC-8/DVB-S2, polynomial 0xD5
//
// Raw CRSF channel:
//   approximately 172 ... 1811
//   center approximately 992
//
// FC output:
//   approximately 990 ... 2010
//   center approximately 1500
//
// API tương thích với module SBUS cũ:
//
//   rc_init();
//   rc_update();
//   rc_get_channel(1..16);
//   rc_get_rx_byte_count();
//   rc_get_frame_count();
//   rc_get_crc_error_count();
//
// ============================================================================

#include <Arduino.h>
#include <HardwareSerial.h>

// ============================================================================
// PIN CONFIGURATION
// ============================================================================

#ifndef PIN_CRSF_RX
#define PIN_CRSF_RX 16
#endif

#ifndef PIN_CRSF_TX
#define PIN_CRSF_TX 17
#endif

// ============================================================================
// CRSF CONSTANTS
// ============================================================================

#define CRSF_BAUDRATE          420000

#define CRSF_MAX_FRAME_LEN     64

#define CRSF_ADDR_FC           0xC8
#define CRSF_ADDR_BROADCAST    0xEE

#define CRSF_TYPE_RC_CHANNELS  0x16

#define CRSF_CHANNEL_COUNT     16
#define CRSF_CHANNEL_BITS      11

#define CRSF_RC_PAYLOAD_SIZE   22

// ============================================================================
// CRSF CHANNEL RANGE
//
// Đây là miền raw của giao thức CRSF.
//
// Không phải PWM.
// Không phải microsecond.
//
// 172  -> minimum
// 992  -> center
// 1811 -> maximum
//
// ============================================================================

#define CRSF_RAW_MIN           172
#define CRSF_RAW_CENTER        992
#define CRSF_RAW_MAX           1811

// ============================================================================
// FC CHANNEL RANGE
//
// Đây là miền mà flight controller cũ sử dụng.
//
// CRSF raw sẽ được scale sang miền này.
//
// 990  -> minimum
// 1500 -> center
// 2010 -> maximum
//
// ============================================================================

#define CRSF_OUT_MIN           990
#define CRSF_OUT_CENTER        1500
#define CRSF_OUT_MAX           2010

// ============================================================================
// FAILSAFE / RX TIMEOUT
//
// Nếu không nhận được RC frame trong khoảng thời gian này:
//
// rc_update() -> false
//
// ============================================================================

#define CRSF_TIMEOUT_MS        200

// ============================================================================
// UART
// ============================================================================

HardwareSerial crsf_uart(2);

// ============================================================================
// INTERNAL BUFFER
// ============================================================================

static uint8_t crsf_buf[CRSF_MAX_FRAME_LEN];

static uint8_t crsf_buf_idx = 0;

static uint8_t crsf_frame_len = 0;

static bool crsf_in_frame = false;

// ============================================================================
// CHANNEL DATA
//
// crsf_raw_ch[0]  = CH1
// crsf_raw_ch[1]  = CH2
// ...
// crsf_raw_ch[15] = CH16
//
// ============================================================================

static uint16_t crsf_raw_ch[CRSF_CHANNEL_COUNT];

// ============================================================================
// FC OUTPUT
//
// Index:
//
// crsf_ch_us[1]  = CH1
// crsf_ch_us[2]  = CH2
// ...
// crsf_ch_us[16] = CH16
//
// Index 0 không sử dụng.
//
// ============================================================================

static int crsf_ch_us[17];

// ============================================================================
// RX STATUS
// ============================================================================

static unsigned long crsf_last_frame_ms = 0;

static volatile uint32_t crsf_rx_byte_count = 0;

static volatile uint32_t crsf_rc_frame_count = 0;

static volatile uint32_t crsf_crc_error_count = 0;

static volatile uint8_t crsf_uplink_lq = 100;

// ============================================================================
// CRC-8/DVB-S2
//
// Polynomial:
//   x^8 + x^7 + x^6 + x^4 + x^2 + 1
//
// Polynomial = 0xD5
//
// CRSF CRC được tính từ:
//   TYPE + PAYLOAD
//
// Không tính:
//   ADDRESS
//   LENGTH
//   CRC byte
//
// ============================================================================

static uint8_t crsf_crc8(
    const uint8_t *data,
    uint8_t len
)
{
    uint8_t crc = 0;

    while (len--)
    {
        crc ^= *data++;

        for (uint8_t i = 0; i < 8; i++)
        {
            if (crc & 0x80)
            {
                crc = (crc << 1) ^ 0xD5;
            }
            else
            {
                crc <<= 1;
            }
        }
    }

    return crc;
}

// ============================================================================
// INITIALIZE
// ============================================================================

void rc_init()
{
    // --------------------------------------------------------
    // UART2
    //
    // RX = GPIO16
    // TX = GPIO17
    //
    // CRSF:
    // 420000 baud
    // 8N1
    // NON-INVERTED
    // --------------------------------------------------------

    crsf_uart.setRxBufferSize(1024);

    crsf_uart.begin(
        CRSF_BAUDRATE,
        SERIAL_8N1,
        PIN_CRSF_RX,
        PIN_CRSF_TX
    );

    // --------------------------------------------------------
    // Xóa dữ liệu cũ
    // --------------------------------------------------------

    while (crsf_uart.available())
    {
        crsf_uart.read();
    }

    // --------------------------------------------------------
    // Reset parser
    // --------------------------------------------------------

    crsf_buf_idx = 0;

    crsf_frame_len = 0;

    crsf_in_frame = false;

    // --------------------------------------------------------
    // Reset statistics
    // --------------------------------------------------------

    crsf_rx_byte_count = 0;

    crsf_rc_frame_count = 0;

    crsf_crc_error_count = 0;
    
    crsf_uplink_lq = 100;

    // --------------------------------------------------------
    // Initial channel state
    //
    // Nếu chưa nhận frame:
    // tất cả channel = center
    // --------------------------------------------------------

    for (int i = 0; i < CRSF_CHANNEL_COUNT; i++)
    {
        crsf_raw_ch[i] = CRSF_RAW_CENTER;
    }

    for (int i = 0; i <= 16; i++)
    {
        crsf_ch_us[i] = CRSF_OUT_CENTER;
    }

    crsf_last_frame_ms = millis();
}

// ============================================================================
// DECODE 16 CHANNELS
//
// Input:
//   payload[22]
//
// Output:
//   crsf_raw_ch[16]
//
// Mỗi channel = 11 bit.
//
// Tổng:
//   16 × 11 = 176 bit
//           = 22 byte
//
// ============================================================================

static void crsf_decode_channels(
    const uint8_t *payload
)
{
    crsf_raw_ch[0] =
        (
            (uint16_t)payload[0]
            |
            ((uint16_t)payload[1] << 8)
        ) & 0x07FF;

    crsf_raw_ch[1] =
        (
            ((uint16_t)payload[1] >> 3)
            |
            ((uint16_t)payload[2] << 5)
        ) & 0x07FF;

    crsf_raw_ch[2] =
        (
            ((uint16_t)payload[2] >> 6)
            |
            ((uint16_t)payload[3] << 2)
            |
            ((uint16_t)payload[4] << 10)
        ) & 0x07FF;

    crsf_raw_ch[3] =
        (
            ((uint16_t)payload[4] >> 1)
            |
            ((uint16_t)payload[5] << 7)
        ) & 0x07FF;

    crsf_raw_ch[4] =
        (
            ((uint16_t)payload[5] >> 4)
            |
            ((uint16_t)payload[6] << 4)
        ) & 0x07FF;

    crsf_raw_ch[5] =
        (
            ((uint16_t)payload[6] >> 7)
            |
            ((uint16_t)payload[7] << 1)
            |
            ((uint16_t)payload[8] << 9)
        ) & 0x07FF;

    crsf_raw_ch[6] =
        (
            ((uint16_t)payload[8] >> 2)
            |
            ((uint16_t)payload[9] << 6)
        ) & 0x07FF;

    crsf_raw_ch[7] =
        (
            ((uint16_t)payload[9] >> 5)
            |
            ((uint16_t)payload[10] << 3)
        ) & 0x07FF;

    crsf_raw_ch[8] =
        (
            (uint16_t)payload[11]
            |
            ((uint16_t)payload[12] << 8)
        ) & 0x07FF;

    crsf_raw_ch[9] =
        (
            ((uint16_t)payload[12] >> 3)
            |
            ((uint16_t)payload[13] << 5)
        ) & 0x07FF;

    crsf_raw_ch[10] =
        (
            ((uint16_t)payload[13] >> 6)
            |
            ((uint16_t)payload[14] << 2)
            |
            ((uint16_t)payload[15] << 10)
        ) & 0x07FF;

    crsf_raw_ch[11] =
        (
            ((uint16_t)payload[15] >> 1)
            |
            ((uint16_t)payload[16] << 7)
        ) & 0x07FF;

    crsf_raw_ch[12] =
        (
            ((uint16_t)payload[16] >> 4)
            |
            ((uint16_t)payload[17] << 4)
        ) & 0x07FF;

    crsf_raw_ch[13] =
        (
            ((uint16_t)payload[17] >> 7)
            |
            ((uint16_t)payload[18] << 1)
            |
            ((uint16_t)payload[19] << 9)
        ) & 0x07FF;

    crsf_raw_ch[14] =
        (
            ((uint16_t)payload[19] >> 2)
            |
            ((uint16_t)payload[20] << 6)
        ) & 0x07FF;

    crsf_raw_ch[15] =
        (
            ((uint16_t)payload[20] >> 5)
            |
            ((uint16_t)payload[21] << 3)
        ) & 0x07FF;
}

// ============================================================================
// MAP CRSF RAW -> FC CHANNEL
//
// CRSF:
//
//   172  -------- 992 -------- 1811
//
// FC:
//
//   990 -------- 1500 -------- 2010
//
// Lưu ý:
// đây là scaling số, không phải chuyển đổi đơn vị vật lý.
//
// ============================================================================

static int crsf_map_channel(
    uint16_t raw
)
{
    raw = constrain(
        raw,
        CRSF_RAW_MIN,
        CRSF_RAW_MAX
    );

    return map(
        raw,
        CRSF_RAW_MIN,
        CRSF_RAW_MAX,
        CRSF_OUT_MIN,
        CRSF_OUT_MAX
    );
}

// ============================================================================
// PROCESS ONE BYTE
//
// State machine:
//
// IDLE
//   |
//   | addr = C8 / EE
//   v
// GOT ADDRESS
//   |
//   | length
//   v
// RECEIVING FRAME
//   |
//   | đủ length
//   v
// CRC CHECK
//   |
//   +---- ERROR -> discard
//   |
//   +---- OK
//          |
//          +-- type 0x16 -> decode RC
//
// ============================================================================

static bool crsf_process_byte(
    uint8_t b
)
{
    // --------------------------------------------------------
    // WAIT FOR ADDRESS
    // --------------------------------------------------------

    if (!crsf_in_frame)
    {
        if (
            b == CRSF_ADDR_FC ||
            b == CRSF_ADDR_BROADCAST
        )
        {
            crsf_buf[0] = b;

            crsf_buf_idx = 1;

            crsf_in_frame = true;
        }

        return false;
    }

    // --------------------------------------------------------
    // BUFFER OVERFLOW PROTECTION
    // --------------------------------------------------------

    if (crsf_buf_idx >= CRSF_MAX_FRAME_LEN)
    {
        crsf_in_frame = false;

        crsf_buf_idx = 0;

        crsf_frame_len = 0;

        return false;
    }

    // --------------------------------------------------------
    // STORE BYTE
    // --------------------------------------------------------

    crsf_buf[crsf_buf_idx++] = b;

    // --------------------------------------------------------
    // BYTE #2 = LENGTH
    // --------------------------------------------------------

    if (crsf_buf_idx == 2)
    {
        crsf_frame_len = b;

        // length:
        //
        // TYPE + PAYLOAD + CRC
        //
        // minimum = 2
        // maximum = 62

        if (
            crsf_frame_len < 2 ||
            crsf_frame_len > CRSF_MAX_FRAME_LEN - 2
        )
        {
            crsf_in_frame = false;

            crsf_buf_idx = 0;

            crsf_frame_len = 0;
        }

        return false;
    }

    // --------------------------------------------------------
    // FRAME NOT COMPLETE
    // --------------------------------------------------------

    if (
        crsf_buf_idx <
        (uint8_t)(crsf_frame_len + 2)
    )
    {
        return false;
    }

    // --------------------------------------------------------
    // COMPLETE FRAME
    // --------------------------------------------------------

    crsf_in_frame = false;

    // --------------------------------------------------------
    // CRC
    //
    // crsf_buf:
    //
    // [0] = address
    // [1] = length
    // [2] = type
    // [3...] = payload
    // [last] = CRC
    //
    // CRC input:
    //
    // type + payload
    //
    // number of bytes:
    //
    // frame_len - 1
    //
    // --------------------------------------------------------

    uint8_t crc_calc =
        crsf_crc8(
            &crsf_buf[2],
            crsf_frame_len - 1
        );

    uint8_t crc_recv =
        crsf_buf[crsf_frame_len + 1];

    // --------------------------------------------------------
    // CRC ERROR
    // --------------------------------------------------------

    if (crc_calc != crc_recv)
    {
        crsf_crc_error_count++;

        crsf_buf_idx = 0;

        crsf_frame_len = 0;

        return false;
    }

    // --------------------------------------------------------
    // VALID FRAME
    // --------------------------------------------------------

    uint8_t frame_type =
        crsf_buf[2];

    // --------------------------------------------------------
    // RC CHANNELS PACKED
    //
    // TYPE = 0x16
    // LENGTH = 24
    //
    // 1 byte type
    // 22 byte payload
    // 1 byte CRC
    // = 24
    //
    // --------------------------------------------------------

    if (
        frame_type == CRSF_TYPE_RC_CHANNELS &&
        crsf_frame_len == 24
    )
    {
        // payload starts at index 3

        crsf_decode_channels(
            &crsf_buf[3]
        );

        crsf_rc_frame_count++;

        crsf_buf_idx = 0;

        crsf_frame_len = 0;

        return true;
    }

    // --------------------------------------------------------
    // LINK STATISTICS
    // TYPE = 0x14
    // LENGTH = 12
    // --------------------------------------------------------
    if (
        frame_type == 0x14 &&
        crsf_frame_len == 12
    )
    {
        // Byte 0: RSSI1 (crsf_buf[3])
        // Byte 1: RSSI2 (crsf_buf[4])
        // Byte 2: LQ    (crsf_buf[5])
        crsf_uplink_lq = crsf_buf[5];
        
        crsf_buf_idx = 0;
        crsf_frame_len = 0;
        return false;
    }

    // --------------------------------------------------------
    // Valid CRSF frame but not RC channels
    // --------------------------------------------------------

    crsf_buf_idx = 0;

    crsf_frame_len = 0;

    return false;
}

// ============================================================================
// RC UPDATE
//
// Gọi hàm này thường xuyên từ rc_task.
//
// Ví dụ:
//
//   if (rc_update())
//   {
//       int roll = rc_get_channel(CH_ROLL);
//   }
//
// Hàm sẽ:
//   1. đọc tất cả byte đang nằm trong UART buffer
//   2. parse frame
//   3. kiểm tra CRC
//   4. decode CH1..CH16
//   5. cập nhật giá trị FC
//   6. kiểm tra timeout
//
// ============================================================================

bool rc_update()
{
    bool got_channels = false;

    // --------------------------------------------------------
    // Drain toàn bộ UART RX buffer
    //
    // Điều này quan trọng vì CRSF có tốc độ 420000 baud.
    //
    // Không chỉ đọc một byte mỗi lần gọi.
    // --------------------------------------------------------

    while (crsf_uart.available())
    {
        uint8_t b =
            crsf_uart.read();

        crsf_rx_byte_count++;

        if (crsf_process_byte(b))
        {
            got_channels = true;
        }
    }

    // --------------------------------------------------------
    // Có RC frame mới
    // --------------------------------------------------------

    if (got_channels)
    {
        for (
            int i = 0;
            i < CRSF_CHANNEL_COUNT;
            i++
        )
        {
            crsf_ch_us[i + 1] =
                crsf_map_channel(
                    crsf_raw_ch[i]
                );
        }

        crsf_last_frame_ms =
            millis();
    }

    // --------------------------------------------------------
    // FAILSAFE / LINK TIMEOUT
    // --------------------------------------------------------

    return (
        (millis() - crsf_last_frame_ms <= CRSF_TIMEOUT_MS) &&
        (crsf_uplink_lq > 0)
    );
}

// ============================================================================
// GET CHANNEL
//
// index:
//   1  -> CH1
//   2  -> CH2
//   ...
//   16 -> CH16
//
// Return:
//   approximately 990 ... 2010
//
// Center:
//   approximately 1500
//
// ============================================================================

int rc_get_channel(
    int index
)
{
    if (
        index < 1 ||
        index > CRSF_CHANNEL_COUNT
    )
    {
        return CRSF_OUT_CENTER;
    }

    return crsf_ch_us[index];
}

// ============================================================================
// OPTIONAL: GET RAW CRSF CHANNEL
//
// Đây là hàm bổ sung.
//
// Nếu muốn xem giá trị CRSF nguyên bản:
//
//   rc_get_raw_channel(1)
//
// sẽ trả:
//
//   ~172 ... 1811
//
// ============================================================================

uint16_t rc_get_raw_channel(
    int index
)
{
    if (
        index < 1 ||
        index > CRSF_CHANNEL_COUNT
    )
    {
        return CRSF_RAW_CENTER;
    }

    return crsf_raw_ch[index - 1];
}

// ============================================================================
// RX STATISTICS
// ============================================================================

uint32_t rc_get_rx_byte_count()
{
    return crsf_rx_byte_count;
}

uint32_t rc_get_frame_count()
{
    return crsf_rc_frame_count;
}

uint32_t rc_get_crc_error_count()
{
    return crsf_crc_error_count;
}

// ============================================================================
// OPTIONAL: RX HEALTH
//
// true  = đang nhận RC frame
// false = timeout / mất tín hiệu
//
// ============================================================================

bool rc_has_signal()
{
    return (
        millis() - crsf_last_frame_ms
        <= CRSF_TIMEOUT_MS
    );
}