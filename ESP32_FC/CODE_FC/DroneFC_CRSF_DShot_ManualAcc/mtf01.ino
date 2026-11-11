// MTF-01 Dual Protocol Driver (ho tro ca MicoLink va MSPv2 cua INAV)
// Chi dung telemetry / log; chua tham gia vao vong dieu khien bay kin.
#include <Arduino.h>
#include <HardwareSerial.h>

#ifndef PIN_MTF_RX
#define PIN_MTF_RX 4 
#endif
#ifndef PIN_MTF_TX
#define PIN_MTF_TX 13 
#endif

#define MTF_BAUDRATE 115200
#define MTF_TIMEOUT_MS 200
#define MTF_PARTIAL_TIMEOUT_MS 50
#define MTF_MAX_BYTES_PER_UPDATE 256

// MicoLink protocol defines
#define MTF_MICOLINK_START 0xEF
#define MTF_MICOLINK_HEADER_SIZE 6
#define MTF_MICOLINK_DATA_MSG 0x51

// MSPv2 protocol defines (INAV / Betaflight)
#define MSP2_HEADER_SIZE 8 // '$', 'X', '<', flag, cmd(2), len(2)
#define MSP2_CMD_RANGEFINDER 0x1F01
#define MSP2_CMD_OPTICAL_FLOW 0x1F02

#define MTF_MAX_PAYLOAD 64
#define MTF_BUF_SIZE 256

HardwareSerial mtf_uart(1);

static uint8_t mtf_buf[MTF_BUF_SIZE];
static uint8_t mtf_buf_len = 0;
static uint32_t mtf_partial_last_ms = 0;
static uint32_t mtf_distance_mm = 0;
static uint32_t mtf_sample_ms = 0; // Sensor's time_ms, not ESP32 millis().
static uint8_t mtf_dist_strength = 0;
static uint8_t mtf_precision = 0;
static uint8_t mtf_tof_status = 0;
static int16_t mtf_flow_x = 0;
static int16_t mtf_flow_y = 0;
static uint8_t mtf_flow_qual = 0;
static uint8_t mtf_flow_status = 0;
static uint32_t mtf_last_frame_ms = 0;
static uint32_t mtf_last_range_ms = 0;
static uint32_t mtf_last_flow_ms = 0;
static bool mtf_has_frame = false;
static uint32_t mtf_frame_count = 0;
static uint32_t mtf_error_count = 0;
static uint8_t mtf_detected_protocol = 0; // 0 = none, 1 = MSPv2, 2 = MicoLink

#ifdef CALIBRATE_MTF
struct MtfCalibration {
    bool collecting, gap, invalid_range;
    uint32_t count, first_ms, last_ms;
    double sum_x, sum_y, sum_x2, sum_y2, sum_distance;
    int16_t min_x, max_x, min_y, max_y;
    uint32_t min_distance, max_distance;
};
static MtfCalibration mtf_cal = {};

static void mtf_calibration_sample(uint32_t now)
{
    if (!mtf_cal.collecting) return;
    if (mtf_cal.count == 0) {
        mtf_cal.first_ms = now;
        mtf_cal.min_x = mtf_cal.max_x = mtf_flow_x;
        mtf_cal.min_y = mtf_cal.max_y = mtf_flow_y;
        mtf_cal.min_distance = mtf_cal.max_distance = mtf_distance_mm;
    } else {
        if ((uint32_t)(now - mtf_cal.last_ms) > MTF_TIMEOUT_MS) mtf_cal.gap = true;
        if (mtf_flow_x < mtf_cal.min_x) mtf_cal.min_x = mtf_flow_x;
        if (mtf_flow_x > mtf_cal.max_x) mtf_cal.max_x = mtf_flow_x;
        if (mtf_flow_y < mtf_cal.min_y) mtf_cal.min_y = mtf_flow_y;
        if (mtf_flow_y > mtf_cal.max_y) mtf_cal.max_y = mtf_flow_y;
        if (mtf_distance_mm < mtf_cal.min_distance) mtf_cal.min_distance = mtf_distance_mm;
        if (mtf_distance_mm > mtf_cal.max_distance) mtf_cal.max_distance = mtf_distance_mm;
    }
    if (mtf_distance_mm < 10 || mtf_distance_mm > 8000) mtf_cal.invalid_range = true;
    mtf_cal.last_ms = now;
    ++mtf_cal.count;
    mtf_cal.sum_x += mtf_flow_x;
    mtf_cal.sum_y += mtf_flow_y;
    mtf_cal.sum_x2 += (double)mtf_flow_x * mtf_flow_x;
    mtf_cal.sum_y2 += (double)mtf_flow_y * mtf_flow_y;
    mtf_cal.sum_distance += mtf_distance_mm;
}
#endif

// CRC8 DVB-S2 (poly 0xD5, init 0x00) cho giao thuc MSPv2
static uint8_t crc8_dvb_s2(uint8_t crc, uint8_t ch)
{
    crc ^= ch;
    for (int i = 0; i < 8; i++) {
        if (crc & 0x80) crc = (crc << 1) ^ 0xD5;
        else crc = crc << 1;
    }
    return crc;
}

static uint16_t mtf_read_u16(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint32_t mtf_read_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

// Retain the suffix after an invalid candidate.
static void mtf_discard_prefix(uint8_t count)
{
    if (count >= mtf_buf_len) {
        mtf_buf_len = 0;
        return;
    }
    mtf_buf_len -= count;
    for (uint8_t i = 0; i < mtf_buf_len; ++i)
        mtf_buf[i] = mtf_buf[i + count];
}

static bool mtf_parse_buffer(uint32_t now, bool expired)
{
    bool got_frame = false;

    while (mtf_buf_len > 0)
    {
        // ====================================================================
        // GIAO THUC 1: MSP v2 (INAV / Betaflight) — Preamble: '$' 'X' '<'
        // ====================================================================
        if (mtf_buf[0] == 0x24) // '$'
        {
            if (mtf_buf_len >= 2 && mtf_buf[1] != 0x58) { mtf_discard_prefix(1); continue; }
            if (mtf_buf_len >= 3 && mtf_buf[2] != 0x3C) { mtf_discard_prefix(1); continue; }
            if (mtf_buf_len < MSP2_HEADER_SIZE)
            {
                if (!expired) break;
                mtf_discard_prefix(1);
                continue;
            }

            uint16_t cmd = mtf_read_u16(mtf_buf + 4);
            uint16_t payload_len = mtf_read_u16(mtf_buf + 6);

            if (payload_len > MTF_MAX_PAYLOAD)
            {
                ++mtf_error_count;
                mtf_discard_prefix(1);
                continue;
            }

            uint16_t frame_len = MSP2_HEADER_SIZE + payload_len + 1; // +1 byte CRC
            if (mtf_buf_len < frame_len)
            {
                if (!expired) break;
                mtf_discard_prefix(1);
                continue;
            }

            // Kiem tra CRC-8 DVB-S2 tu byte flag (index 3) den het payload
            uint8_t crc = 0;
            for (uint16_t i = 3; i < frame_len - 1; ++i)
                crc = crc8_dvb_s2(crc, mtf_buf[i]);

            if (crc != mtf_buf[frame_len - 1])
            {
                ++mtf_error_count;
                mtf_discard_prefix(1);
                continue;
            }

            // Frame MSPv2 hop le! Giai ma payload
            if (mtf_detected_protocol != 1) {
                mtf_detected_protocol = 1;
                Serial.println(F("[MTF] Phat hien va ket noi thanh cong giao thuc MSPv2 (INAV)!"));
            }

            const uint8_t *p = mtf_buf + MSP2_HEADER_SIZE;
            if (cmd == MSP2_CMD_RANGEFINDER && payload_len >= 5)
            {
                // p[0]: quality (0-255), p[1..4]: distance (mm, int32 little-endian)
                mtf_dist_strength = p[0];
                int32_t d = (int32_t)mtf_read_u32(p + 1);
                if (d >= 0) mtf_distance_mm = (uint32_t)d;
                mtf_tof_status = (p[0] > 0 ? 1 : 0);
                mtf_precision = p[0];
                mtf_sample_ms = now;
                mtf_last_frame_ms = now;
                mtf_last_range_ms = now;
                mtf_has_frame = true;
                ++mtf_frame_count;
                got_frame = true;
            }
            else if (cmd == MSP2_CMD_OPTICAL_FLOW && payload_len >= 9)
            {
                // p[0]: quality, p[1..4]: motionX (int32), p[5..8]: motionY (int32)
                mtf_flow_qual = p[0];
                mtf_flow_x = (int16_t)(int32_t)mtf_read_u32(p + 1);
                mtf_flow_y = (int16_t)(int32_t)mtf_read_u32(p + 5);
                mtf_flow_status = (p[0] > 0 ? 1 : 0);
                mtf_sample_ms = now;
                mtf_last_frame_ms = now;
                mtf_last_flow_ms = now;
                mtf_has_frame = true;
                ++mtf_frame_count;
#ifdef CALIBRATE_MTF
                if (mtf_distance_mm >= 10)
                    mtf_calibration_sample(now);
#endif
                got_frame = true;
            }

            mtf_discard_prefix(frame_len);
            continue;
        }

        // ====================================================================
        // GIAO THUC 2: MicoLink — Preamble: 0xEF
        // ====================================================================
        if (mtf_buf[0] == MTF_MICOLINK_START)
        {
            if (mtf_buf_len < MTF_MICOLINK_HEADER_SIZE)
            {
                if (!expired) break;
                mtf_discard_prefix(1);
                continue;
            }

            const uint8_t payload_len = mtf_buf[5];
            if (payload_len > MTF_MAX_PAYLOAD ||
                (mtf_buf[3] == MTF_MICOLINK_DATA_MSG && payload_len < 20))
            {
                ++mtf_error_count;
                mtf_discard_prefix(1);
                continue;
            }

            const uint8_t frame_len = MTF_MICOLINK_HEADER_SIZE + payload_len + 1;
            if (mtf_buf_len < frame_len)
            {
                if (!expired) break;
                mtf_discard_prefix(1);
                continue;
            }

            uint8_t sum = 0;
            for (uint8_t i = 0; i < frame_len - 1; ++i)
                sum += mtf_buf[i];
            if (sum != mtf_buf[frame_len - 1])
            {
                ++mtf_error_count;
                mtf_discard_prefix(1);
                continue;
            }

            if (mtf_detected_protocol != 2) {
                mtf_detected_protocol = 2;
                Serial.println(F("[MTF] Phat hien va ket noi thanh cong giao thuc MicoLink!"));
            }

            if (mtf_buf[3] == MTF_MICOLINK_DATA_MSG)
            {
                const uint8_t *p = mtf_buf + MTF_MICOLINK_HEADER_SIZE;
                mtf_sample_ms = mtf_read_u32(p);
                mtf_distance_mm = mtf_read_u32(p + 4);
                mtf_dist_strength = p[8];
                mtf_precision = p[9];
                mtf_tof_status = p[10];
                mtf_flow_x = (int16_t)mtf_read_u16(p + 12);
                mtf_flow_y = (int16_t)mtf_read_u16(p + 14);
                mtf_flow_qual = p[16];
                mtf_flow_status = p[17];
                mtf_last_frame_ms = now;
                mtf_last_range_ms = now;
                mtf_last_flow_ms = now;
                mtf_has_frame = true;
                ++mtf_frame_count;
#ifdef CALIBRATE_MTF
                mtf_calibration_sample(now);
#endif
                got_frame = true;
            }
            mtf_discard_prefix(frame_len);
            continue;
        }

        // Byte khong phai mo dau cua bat ky giao thuc nao
        mtf_discard_prefix(1);
    }
    return got_frame;
}

static bool mtf_process_byte(uint8_t b, uint32_t now)
{
    if (mtf_buf_len == 0 && b != MTF_MICOLINK_START && b != 0x24) return false;
    if (mtf_buf_len >= sizeof(mtf_buf)) {
        mtf_discard_prefix(1);
    }
    mtf_buf[mtf_buf_len++] = b;
    mtf_partial_last_ms = now;
    return mtf_parse_buffer(now, false);
}

void mtf_init()
{
    mtf_uart.setRxBufferSize(256);
    mtf_uart.begin(MTF_BAUDRATE, SERIAL_8N1, PIN_MTF_RX, PIN_MTF_TX);
    for (unsigned int i = 0; i < MTF_MAX_BYTES_PER_UPDATE && mtf_uart.available() > 0; ++i)
        mtf_uart.read();
    mtf_buf_len = 0;
    mtf_partial_last_ms = 0;
    mtf_distance_mm = 0;
    mtf_sample_ms = 0;
    mtf_dist_strength = 0;
    mtf_precision = 0;
    mtf_tof_status = 0;
    mtf_flow_x = 0;
    mtf_flow_y = 0;
    mtf_flow_qual = 0;
    mtf_flow_status = 0;
    mtf_last_frame_ms = 0;
    mtf_last_range_ms = 0;
    mtf_last_flow_ms = 0;
    mtf_has_frame = false;
    mtf_frame_count = 0;
    mtf_error_count = 0;
    mtf_detected_protocol = 0;
}

bool mtf_update()
{
    bool got_frame = false;
    uint32_t now = millis();
    if (mtf_buf_len && (uint32_t)(now - mtf_partial_last_ms) > MTF_PARTIAL_TIMEOUT_MS)
    {
        ++mtf_error_count;
        got_frame = mtf_parse_buffer(mtf_partial_last_ms, true);
    }
    for (unsigned int i = 0; i < MTF_MAX_BYTES_PER_UPDATE && mtf_uart.available() > 0; ++i)
    {
        int value = mtf_uart.read();
        if (value < 0)
            break;
        if (mtf_process_byte((uint8_t)value, millis()))
            got_frame = true;
    }
    return got_frame;
}

float mtf_get_distance_m() { return (float)mtf_distance_mm / 1000.0f; }
float mtf_get_distance_cm() { return (float)mtf_distance_mm / 10.0f; }
uint8_t mtf_get_distance_strength() { return mtf_dist_strength; }
int16_t mtf_get_flow_x_raw() { return mtf_flow_x; }
int16_t mtf_get_flow_y_raw() { return mtf_flow_y; }
uint8_t mtf_get_flow_quality() { return mtf_flow_qual; }
uint8_t mtf_get_precision() { return mtf_precision; }
uint8_t mtf_get_tof_status() { return mtf_tof_status; }
uint8_t mtf_get_flow_status() { return mtf_flow_status; }
uint32_t mtf_get_sample_ms() { return mtf_sample_ms; }
uint32_t mtf_get_receive_ms() { return mtf_last_frame_ms; }

bool mtf_is_distance_valid()
{
    return mtf_has_frame &&
           (uint32_t)(millis() - mtf_last_range_ms) <= MTF_TIMEOUT_MS &&
           mtf_distance_mm >= 10 && mtf_distance_mm <= 8000;
}

uint32_t mtf_get_frame_count() { return mtf_frame_count; }
uint32_t mtf_get_error_count() { return mtf_error_count; }

#ifdef CALIBRATE_MTF
void mtf_calibrate()
{
    Serial.println("[MTF CAL] Thao canh, ngat nguon ESC neu co the; giu drone va be mat dung yen.");
    Serial.println("[MTF CAL] Cam bien huong xuong be mat co hoa tiet; khong co task bay.");
    Serial.printf("[MTF CAL] Khoang cach chuan: %d mm (0 = khong do offset ToF).\n", MTF_REFERENCE_MM);
    if (MTF_REFERENCE_MM != 0 && (MTF_REFERENCE_MM < 10 || MTF_REFERENCE_MM > 8000)) {
        Serial.println("[MTF CAL] THAT BAI: khoang cach chuan phai la 0 hoac 10..8000 mm.");
        return;
    }

    for (int i = 5; i > 0; --i) {
        Serial.printf("[MTF CAL] Bat dau sau %d giay\n", i);
        uint32_t second_start = millis();
        while ((uint32_t)(millis() - second_start) < 1000U) {
            mtf_update();
            delay(1);
        }
    }
    mtf_cal = {};
    uint32_t errors_start = mtf_error_count;
    uint32_t start = millis();
    mtf_cal.collecting = true;
    while ((uint32_t)(millis() - start) < 15000U) {
        mtf_update();
        if (mtf_cal.count >= 200 && (uint32_t)(mtf_cal.last_ms - mtf_cal.first_ms) >= 5000U) break;
        delay(1);
    }
    mtf_cal.collecting = false;
    uint32_t now = millis();
    uint32_t errors = mtf_error_count - errors_start;
    Serial.printf("[MTF CAL] frames:%lu duration_ms:%lu errors:%lu gap:%d range_invalid:%d\n",
                  (unsigned long)mtf_cal.count,
                  (unsigned long)(mtf_cal.count ? mtf_cal.last_ms - mtf_cal.first_ms : 0),
                  (unsigned long)errors, mtf_cal.gap, mtf_cal.invalid_range);
    if (mtf_cal.count < 200 || (uint32_t)(mtf_cal.last_ms - mtf_cal.first_ms) < 5000U ||
        (uint32_t)(now - mtf_cal.last_ms) > MTF_TIMEOUT_MS ||
        mtf_cal.gap || mtf_cal.invalid_range || errors != 0) {
        Serial.println("[MTF CAL] THAT BAI: du lieu khong du, gian doan, ngoai dai hoac loi parser.");
        return;
    }
    double x = mtf_cal.sum_x / mtf_cal.count, y = mtf_cal.sum_y / mtf_cal.count;
    double var_x = mtf_cal.sum_x2 / mtf_cal.count - x * x;
    double var_y = mtf_cal.sum_y2 / mtf_cal.count - y * y;
    double distance = mtf_cal.sum_distance / mtf_cal.count;
    Serial.printf("[MTF CAL] flow X mean:%.3f std:%.3f min:%d max:%d | Y mean:%.3f std:%.3f min:%d max:%d\n",
                  x, sqrt(var_x > 0 ? var_x : 0), mtf_cal.min_x, mtf_cal.max_x,
                  y, sqrt(var_y > 0 ? var_y : 0), mtf_cal.min_y, mtf_cal.max_y);
    Serial.printf("[MTF CAL] ToF mean:%.2f mm min:%lu max:%lu | quality:%u flow_status:%u strength:%u precision:%u tof_status:%u\n",
                  distance, (unsigned long)mtf_cal.min_distance, (unsigned long)mtf_cal.max_distance,
                  mtf_flow_qual, mtf_flow_status, mtf_dist_strength, mtf_precision, mtf_tof_status);
    Serial.println("[MTF CAL] Uoc luong chua xac minh do dung yen, don vi flow hay do moi frame; lap lai truoc khi su dung.");
    Serial.printf("#define MTF_FLOW_BIAS_X_RAW %.3ff \r\n#define MTF_FLOW_BIAS_Y_RAW %.3ff \r\n", x, y);
    if (MTF_REFERENCE_MM != 0)
        Serial.printf("#define MTF_TOF_OFFSET_MM %.3ff\n", (double)MTF_REFERENCE_MM - distance);
    else
        Serial.println("[MTF CAL] Chua nhap khoang cach chuan: khong tinh offset ToF.");
    Serial.println("[MTF CAL] Tat CALIBRATE_MTF va nap lai de tro ve firmware thuong.");
}
#endif
