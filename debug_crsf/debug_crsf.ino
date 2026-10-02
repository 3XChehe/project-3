#include <Arduino.h>

// ============================================================
// CRSF / ESP32 CONFIG
// ============================================================

#define PIN_CRSF_RX 16
#define PIN_CRSF_TX 17

#define CRSF_BAUD 420000

HardwareSerial CRSFSerial(2);

// ============================================================
// CRSF CONSTANTS
// ============================================================

#define CRSF_SYNC 0xC8
#define CRSF_RC_CHANNELS_PACKED 0x16

// RC Channels Packed:
// 16 channels × 11 bits = 176 bits = 22 bytes

#define CRSF_CHANNEL_COUNT 16
#define CRSF_PAYLOAD_SIZE 22

// ============================================================
// CRC8
// Polynomial = 0xD5
// CRC calculated over TYPE + PAYLOAD
// ============================================================

uint8_t crsf_crc8(const uint8_t *ptr, uint8_t len)
{
    uint8_t crc = 0;

    while (len--)
    {
        crc ^= *ptr++;

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

// ============================================================
// Decode 16 × 11-bit channels
//
// payload[0 ... 21] = 22 bytes
//
// Channel layout:
//
// CH1  = bits 0  ... 10
// CH2  = bits 11 ... 21
// CH3  = bits 22 ... 32
// ...
// CH16 = bits 165 ... 175
//
// ============================================================

bool decodeChannels(
    const uint8_t *payload,
    uint16_t channels[CRSF_CHANNEL_COUNT]
)
{
    uint32_t bitBuffer = 0;
    uint8_t bitsInBuffer = 0;

    uint8_t payloadIndex = 0;

    for (uint8_t ch = 0; ch < CRSF_CHANNEL_COUNT; ch++)
    {
        // ----------------------------------------------------
        // Đảm bảo buffer có ít nhất 11 bit
        // ----------------------------------------------------

        while (bitsInBuffer < 11)
        {
            if (payloadIndex >= CRSF_PAYLOAD_SIZE)
            {
                return false;
            }

            bitBuffer |=
                ((uint32_t)payload[payloadIndex])
                << bitsInBuffer;

            payloadIndex++;

            bitsInBuffer += 8;
        }

        // ----------------------------------------------------
        // Lấy 11 bit thấp nhất
        // ----------------------------------------------------

        channels[ch] =
            bitBuffer & 0x7FF;

        // ----------------------------------------------------
        // Bỏ 11 bit vừa sử dụng
        // ----------------------------------------------------

        bitBuffer >>= 11;
        bitsInBuffer -= 11;
    }

    return true;
}

// ============================================================
// CRSF raw value -> approximately 1000..2000 us
//
// Typical CRSF channel range:
// approximately 172 ... 1811
//
// Center:
// approximately 992
//
// NOTE:
// Đây là conversion để debug / hiển thị.
// Không phải giá trị vật lý tuyệt đối.
// ============================================================

int crsfToUs(uint16_t value)
{
    const int CRSF_MIN = 172;
    const int CRSF_MAX = 1811;

    const int PWM_MIN = 1000;
    const int PWM_MAX = 2000;

    value = constrain(
        value,
        CRSF_MIN,
        CRSF_MAX
    );

    return map(
        value,
        CRSF_MIN,
        CRSF_MAX,
        PWM_MIN,
        PWM_MAX
    );
}

// ============================================================
// Print channels
// ============================================================

void printChannels(
    const uint16_t channels[CRSF_CHANNEL_COUNT]
)
{
    Serial.println();
    Serial.println("========================================");
    Serial.println("          CRSF RC CHANNELS");
    Serial.println("========================================");

    for (uint8_t i = 0; i < CRSF_CHANNEL_COUNT; i++)
    {
        Serial.printf(
            "CH%-2d = %4u    ~ %4d us\n",
            i + 1,
            channels[i],
            crsfToUs(channels[i])
        );
    }

    Serial.println("========================================");
}

// ============================================================
// CRSF FRAME PARSER
// ============================================================

void processCRSF()
{
    static uint8_t frame[64];

    static uint8_t pos = 0;
    static uint8_t frameLength = 0;

    static uint32_t lastPrint = 0;

    while (CRSFSerial.available())
    {
        uint8_t b = CRSFSerial.read();

        // ----------------------------------------------------
        // STATE 0
        // Tìm SYNC
        // ----------------------------------------------------

        if (pos == 0)
        {
            if (b != CRSF_SYNC)
            {
                continue;
            }

            frame[pos++] = b;

            continue;
        }

        // ----------------------------------------------------
        // STATE 1
        // LENGTH
        // ----------------------------------------------------

        if (pos == 1)
        {
            frame[pos++] = b;

            frameLength = b;

            // length phải:
            //
            // TYPE + PAYLOAD + CRC
            //
            // tối thiểu 2 byte
            //

            if (
                frameLength < 2 ||
                frameLength > 62
            )
            {
                pos = 0;
                frameLength = 0;
            }

            continue;
        }

        // ----------------------------------------------------
        // STATE 2
        // Nhận phần còn lại
        // ----------------------------------------------------

        frame[pos++] = b;

        // Tổng frame:
        //
        // SYNC + LENGTH + LENGTH bytes
        //
        // = frameLength + 2

        if (pos < frameLength + 2)
        {
            continue;
        }

        // ====================================================
        // FULL FRAME RECEIVED
        // ====================================================

        uint8_t type = frame[2];

        // ----------------------------------------------------
        // CRC
        // ----------------------------------------------------

        uint8_t crcReceived =
            frame[pos - 1];

        uint8_t crcCalculated =
            crsf_crc8(
                &frame[2],
                frameLength - 1
            );

        // ----------------------------------------------------
        // CRC ERROR
        // ----------------------------------------------------

        if (crcReceived != crcCalculated)
        {
            Serial.printf(
                "CRSF CRC ERROR: recv=0x%02X calc=0x%02X\n",
                crcReceived,
                crcCalculated
            );

            pos = 0;
            frameLength = 0;

            continue;
        }

        // ====================================================
        // RC CHANNELS PACKED
        // ====================================================

        if (
            type == CRSF_RC_CHANNELS_PACKED &&
            frameLength == 24
        )
        {
            // ------------------------------------------------
            // Payload bắt đầu tại frame[3]
            //
            // frame:
            //
            // [0] SYNC
            // [1] LENGTH
            // [2] TYPE
            // [3..24] PAYLOAD 22 bytes
            // [25] CRC
            // ------------------------------------------------

            uint16_t channels[CRSF_CHANNEL_COUNT];

            bool ok =
                decodeChannels(
                    &frame[3],
                    channels
                );

            if (ok)
            {
                // ------------------------------------------------
                // Giới hạn tốc độ print
                // ------------------------------------------------

                uint32_t now = millis();

                if (now - lastPrint >= 100)
                {
                    lastPrint = now;

                    printChannels(channels);
                }
            }
        }

        // ----------------------------------------------------
        // Reset parser
        // ----------------------------------------------------

        pos = 0;
        frameLength = 0;
    }
}

// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(115200);

    delay(1000);

    Serial.println();
    Serial.println();
    Serial.println("========================================");
    Serial.println("       ESP32 CRSF CHANNEL DECODER");
    Serial.println("========================================");

    Serial.printf(
        "CRSF RX  : GPIO %d\n",
        PIN_CRSF_RX
    );

    Serial.printf(
        "CRSF TX  : GPIO %d\n",
        PIN_CRSF_TX
    );

    Serial.printf(
        "CRSF baud: %d\n",
        CRSF_BAUD
    );

    Serial.println();

    // ========================================================
    // UART2
    // ========================================================

    CRSFSerial.begin(
        CRSF_BAUD,
        SERIAL_8N1,
        PIN_CRSF_RX,
        PIN_CRSF_TX
    );

    Serial.println(
        "CRSF UART started."
    );

    Serial.println(
        "Waiting for RC Channels Packed..."
    );

    Serial.println();
}

// ============================================================
// LOOP
// ============================================================

void loop()
{
    processCRSF();
}