// altitude_kf.ino
//  ============================================================================
//   KyThuatUAV FC - firmware bay ESP32 (FreeRTOS), che do Angle + Alt Hold
//
//   Copyright (C) 2026  Nguyễn Văn Quý (Ky Thuat UAV)
//
//   This program is free software: you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation, either version 3 of the License, or
//   (at your option) any later version.
//
//   This program is distributed in the hope that it will be useful,
//   but WITHOUT ANY WARRANTY; without even the implied warranty of
//   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//   GNU General Public License for more details.
//
//   You should have received a copy of the GNU General Public License
//   along with this program.  If not, see <https://www.gnu.org/licenses/>.
//
//   Giu nguyen phan ghi cong nay khi dung lai hoac chinh sua ma nguon.
//
//   Nguon tham khao va ghi cong day du: xem file LOI_NHAN_VA_GHI_CONG.md
//   Lien he: Nguyen Van Quy - 0817 550 271 (Zalo)
//  ============================================================================
//  ============================================================================
//   Kalman 2 trạng thái ước lượng độ cao và tốc độ thẳng đứng.
//
//      trạng thái   S = [ z (cm) , vz (cm/s) ]'
//      đầu vào      u = gia tốc thẳng đứng (cm/s^2), từ IMU
//      phép đo      M = độ cao từ baro (cm)
//
//   Giữ nguyên dạng ma trận và bộ tham số của bản virex v14, chạy đúng nhịp
//   2ms của vòng điều khiển.
//
//   ------------------------------------------------------------------------
//   QUAN TRỌNG - vì sao gọi CẢ predict LẪN correct ở MỖI vòng 500Hz,
//   kể cả khi baro chưa có mẫu mới:
//
//   Bộ tham số này (sigma gia tốc = 4 cm/s^2, R = 30^2) cho ma trận P mọc rất
//   chậm, nên độ lợi Kalman K rất nhỏ. Nó chỉ kéo được ước lượng bám theo baro
//   nhờ ĐƯỢC ÁP DỤNG 500 LẦN MỖI GIÂY - cộng dồn lại mới đủ lực.
//
//   Từng thử đổi sang "chỉ correct khi baro có mẫu mới" (~50Hz) cho đúng lý
//   thuyết hơn. Kết quả: mất 9/10 số lần hiệu chỉnh mà K vẫn nhỏ như cũ, phần
//   kéo về gần như biến mất, vz thành tích phân gia tốc thuần và TRÔI VÔ HẠN.
//
//   Muốn làm bản đúng lý thuyết thì phải tăng sigma gia tốc lên cho P mọc
//   nhanh tương ứng (cỡ 3 lần khi giảm 10 lần số nhịp correct), rồi bay thử
//   lại từ đầu. Chưa kiểm chứng trên phần cứng thì ĐỪNG đổi.
//   ------------------------------------------------------------------------
//  ============================================================================

#include <BasicLinearAlgebra.h>
using namespace BLA;

// sigma của nhiễu gia tốc (cm/s^2) và của phép đo baro (cm)
#define KF_SIGMA_ACC_CMS2 4.0f
#define KF_SIGMA_BARO_CM 30.0f

BLA::Matrix<2, 2> Fz;    // ma trận chuyển trạng thái
BLA::Matrix<2, 1> Gz;    // ma trận đầu vào
BLA::Matrix<2, 2> Pz;    // hiệp phương sai sai số ước lượng
BLA::Matrix<2, 2> Qz;    // hiệp phương sai nhiễu quá trình
BLA::Matrix<2, 1> Sz;    // vector trạng thái
BLA::Matrix<1, 2> Hz;    // ma trận quan sát
BLA::Matrix<2, 2> Iz;    // ma trận đơn vị
BLA::Matrix<1, 1> Acczz; // đầu vào: gia tốc thẳng đứng
BLA::Matrix<2, 1> Kz;    // độ lợi Kalman
BLA::Matrix<1, 1> Rz;    // hiệp phương sai nhiễu đo
BLA::Matrix<1, 1> Lz;    // hiệp phương sai phần dư
BLA::Matrix<1, 1> Mz;    // phép đo

static float kf_alt_cm = 0.0f;
static float kf_climb_cms = 0.0f;

void alt_kf_init()
{
    Fz = {1, DT_CTRL,
          0, 1};
    Gz = {0.5f * DT_CTRL * DT_CTRL,
          DT_CTRL};
    Hz = {1, 0};
    Iz = {1, 0,
          0, 1};
    Qz = Gz * ~Gz * KF_SIGMA_ACC_CMS2 * KF_SIGMA_ACC_CMS2;
    Rz = {KF_SIGMA_BARO_CM * KF_SIGMA_BARO_CM};
    Pz = {0, 0,
          0, 0};
    Sz = {0,
          0};

    kf_alt_cm = 0.0f;
    kf_climb_cms = 0.0f;
}

// Gọi MỖI vòng điều khiển.
//   alt_meas_m     : độ cao baro, đơn vị MÉT (hàm tự đổi sang cm)
//   acc_earth_z_g  : gia tốc thẳng đứng đã trừ trọng lực, đơn vị g
void alt_kf_update(float alt_meas_m, float acc_earth_z_g)
{
    Acczz = {acc_earth_z_g * 981.0f}; // g -> cm/s^2

    Sz = Fz * Sz + Gz * Acczz; // dự đoán trạng thái
    Pz = Fz * Pz * ~Fz + Qz;   // dự đoán hiệp phương sai

    Lz = Hz * Pz * ~Hz + Rz; // hiệp phương sai phần dư

    // Độ lợi Kalman: K = P·H'·L⁻¹
    // Bản virex v14 viết Invert(Lz), nhưng đó là cú pháp của BasicLinearAlgebra
    // 3.x. Từ bản 4.x, Invert() đảo ma trận TẠI CHỖ và trả về bool, nên viết như
    // cũ sẽ không biên dịch được. Lz là ma trận 1x1 nên nghịch đảo của nó chính
    // là nghịch đảo số học - viết thẳng vừa đúng, vừa không phụ thuộc phiên bản
    // thư viện, lại nhanh hơn.
    float Lz_scalar = Lz(0, 0);
    if (Lz_scalar < 1e-6f)
        return; // chặn chia cho 0
    Kz = Pz * ~Hz * (1.0f / Lz_scalar);

    Mz = {alt_meas_m * 100.0f};    // m -> cm
    Sz = Sz + Kz * (Mz - Hz * Sz); // hiệu chỉnh theo phép đo
    Pz = (Iz - Kz * Hz) * Pz;

    kf_alt_cm = Sz(0, 0);
    kf_climb_cms = Sz(1, 0);
}

float alt_kf_get_altitude_cm() { return kf_alt_cm; }
float alt_kf_get_climb_cms() { return kf_climb_cms; }
//=================================================================================================
// baro_bmp388.ino
// ============================================================================
//  KyThuatUAV FC - firmware bay ESP32 (FreeRTOS), che do Angle + Alt Hold
//
//  Copyright (C) 2026  Nguyễn Văn Quý (Ky Thuat UAV)
//
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program.  If not, see <https://www.gnu.org/licenses/>.
//
//  Giu nguyen phan ghi cong nay khi dung lai hoac chinh sua ma nguon.
//
//  Nguon tham khao va ghi cong day du: xem file LOI_NHAN_VA_GHI_CONG.md
//  Lien he: Nguyen Van Quy - 0817 550 271 (Zalo)
// ============================================================================
// ============================================================================
//  BMP388 - driver thanh ghi, bus I2C, không dùng thư viện ngoài.
//  Chuyển từ bản SPI của FC_teensy41_v24 sang I2C.
//
//  KHÁC BIỆT SO VỚI BẢN SPI: khi đọc, SPI của BMP388 trả về một byte rác ngay
//  sau byte địa chỉ rồi mới tới dữ liệu thật. I2C KHÔNG có byte rác đó. Chép
//  nguyên hàm đọc SPI sang I2C là lệch một byte, mà lệch kiểu này không báo
//  lỗi gì, chỉ ra số sai.
//
//  API công khai:
//     bool  baro_init();              dò chip, cấu hình, đo cao độ gốc
//     bool  baro_update();            true khi có mẫu MỚI
//     float baro_get_altitude_m();    mét, đã trừ cao độ gốc lúc khởi động
//     bool  baro_is_present();
// ============================================================================

#include <Wire.h>

#define BMP388_CHIP_ID_VALUE 0x50

// Thanh ghi
#define BMP388_REG_CHIP_ID 0x00
#define BMP388_REG_ERR 0x02
#define BMP388_REG_STATUS 0x03
#define BMP388_REG_DATA 0x04
#define BMP388_REG_PWR_CTRL 0x1B
#define BMP388_REG_OSR 0x1C
#define BMP388_REG_ODR 0x1D
#define BMP388_REG_CONFIG 0x1F
#define BMP388_REG_CALIB 0x31
#define BMP388_REG_CMD 0x7E

// Bit của thanh ghi STATUS (0x03)
#define BMP388_STATUS_CMD_RDY (1 << 4)
#define BMP388_STATUS_DRDY_PRESS (1 << 5)

#define BMP388_CMD_SOFT_RESET 0xB6

// Thanh ghi PWR_CTRL
#define BMP388_PRESS_EN (1U << 0)
#define BMP388_TEMP_EN (1U << 1)
#define BMP388_MODE_NORMAL (3U << 4)

// Hệ số lấy mẫu bội (oversampling)
#define BMP388_OSR_X1 0x00
#define BMP388_OSR_X2 0x01
#define BMP388_OSR_X4 0x02
#define BMP388_OSR_X8 0x03
#define BMP388_OSR_X16 0x04
#define BMP388_OSR_X32 0x05

// Tần số lấy mẫu
#define BMP388_ODR_200HZ 0x00
#define BMP388_ODR_100HZ 0x01
#define BMP388_ODR_50HZ 0x02
#define BMP388_ODR_25HZ 0x03

// Bộ lọc IIR, nằm ở bit [3:1] của thanh ghi CONFIG
#define BMP388_IIR_OFF 0x00
#define BMP388_IIR_1 0x01
#define BMP388_IIR_3 0x02
#define BMP388_IIR_7 0x03

/* Cấu hình đang dùng: ODR 50 Hz, OSR áp suất x8.
 *
 * OSR phải ăn khớp với ODR, không chọn bừa được. Công thức thời gian đo của
 * datasheet (mục 3.9.2):
 *
 *     t = 234 + (392 + 2^osr_p * 2020) + (163 + 2^osr_t * 2020)   [us]
 *
 *     OSR_P x8, OSR_T x1 :  234 + 16552 + 2183 = 18969 us = 19.0 ms
 *
 * Chu kì ở 50 Hz là 20 ms nên vừa lọt. Đặt ODR nhanh hơn mà vẫn để OSR x8 thì
 * cảm biến không kịp đo và tự báo lỗi conf_err.
 *
 * Vì sao 50 Hz chứ không phải 200 Hz như bản teensy: ở đây chỉ cần giữ độ cao,
 * mà baro_task chạy 100 Hz nên 50 Hz đã dư. Đổi lại được OSR x8 thay vì x1,
 * nhiễu áp suất thấp hơn hẳn - với alt hold thì điều đó quan trọng hơn tốc độ.
 */
#define BARO_ODR_SETTING BMP388_ODR_50HZ
#define BARO_OSR_PRESS BMP388_OSR_X8
#define BARO_OSR_TEMP BMP388_OSR_X1
#define BARO_IIR_SETTING BMP388_IIR_3
#define BARO_SEA_LEVEL_PA 101325.0

static float baro_temp_c = 0.0f;
static float baro_press_pa = 0.0f;
static float baro_alt_abs_m = 0.0f; // cao độ tuyệt đối so với mực nước biển
static float baro_alt_ref_m = 0.0f; // cao độ đo được lúc khởi động
static bool baro_ready = false;

// Hệ số hiệu chỉnh riêng của từng con chip, đọc từ thanh ghi CALIB
struct BaroCalib
{
    double t1, t2, t3;
    double p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11;
};
static BaroCalib calib;

// ============================================================================
//  I2C mức thấp
// ============================================================================
static bool baro_write_reg(uint8_t reg, uint8_t val)
{
    Wire.beginTransmission(BARO_I2C_ADDR);
    Wire.write(reg);
    Wire.write(val);
    return (Wire.endTransmission() == 0);
}

static bool baro_read_regs(uint8_t reg, uint8_t *buf, uint8_t len)
{
    Wire.beginTransmission(BARO_I2C_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0)
        return false; // repeated start

    if (Wire.requestFrom((int)BARO_I2C_ADDR, (int)len) != len)
        return false;
    for (uint8_t i = 0; i < len; i++)
        buf[i] = Wire.read();
    return true;
}

static bool baro_read_reg(uint8_t reg, uint8_t &val)
{
    return baro_read_regs(reg, &val, 1);
}

static inline uint16_t u16le(const uint8_t *b)
{
    return (uint16_t)b[0] | ((uint16_t)b[1] << 8);
}
static inline int16_t s16le(const uint8_t *b)
{
    return (int16_t)((uint16_t)b[0] | ((uint16_t)b[1] << 8));
}
static inline uint32_t u24le(const uint8_t *b)
{
    return ((uint32_t)b[0]) | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16);
}

static uint8_t baro_sample_period_ms()
{
    switch (BARO_ODR_SETTING)
    {
    case BMP388_ODR_200HZ:
        return 5;
    case BMP388_ODR_100HZ:
        return 10;
    case BMP388_ODR_50HZ:
        return 20;
    case BMP388_ODR_25HZ:
        return 40;
    default:
        return 20;
    }
}

// ============================================================================
//  Đọc và giải mã hệ số hiệu chỉnh
// ============================================================================
static bool baro_read_calibration()
{
    uint8_t buf[21];
    if (!baro_read_regs(BMP388_REG_CALIB, buf, sizeof(buf)))
        return false;

    uint16_t t1 = u16le(&buf[0]);
    uint16_t t2 = u16le(&buf[2]);
    int8_t t3 = (int8_t)buf[4];
    int16_t p1 = s16le(&buf[5]);
    int16_t p2 = s16le(&buf[7]);
    int8_t p3 = (int8_t)buf[9];
    int8_t p4 = (int8_t)buf[10];
    uint16_t p5 = u16le(&buf[11]);
    uint16_t p6 = u16le(&buf[13]);
    int8_t p7 = (int8_t)buf[15];
    int8_t p8 = (int8_t)buf[16];
    int16_t p9 = s16le(&buf[17]);
    int8_t p10 = (int8_t)buf[19];
    int8_t p11 = (int8_t)buf[20];

    calib.t1 = ((double)t1 / 0.00390625);
    calib.t2 = ((double)t2 / 1073741824.0);
    calib.t3 = ((double)t3 / 281474976710656.0);

    calib.p1 = ((double)(p1 - 16384) / 1048576.0);
    calib.p2 = ((double)(p2 - 16384) / 536870912.0);
    calib.p3 = ((double)p3 / 4294967296.0);
    calib.p4 = ((double)p4 / 137438953472.0);
    calib.p5 = ((double)p5 / 0.125);
    calib.p6 = ((double)p6 / 64.0);
    calib.p7 = ((double)p7 / 256.0);
    calib.p8 = ((double)p8 / 32768.0);
    calib.p9 = ((double)p9 / 281474976710656.0);
    calib.p10 = ((double)p10 / 281474976710656.0);
    calib.p11 = ((double)p11 / 36893488147419103232.0);

    return true;
}

// ============================================================================
//  Bù nhiệt và bù áp suất, theo đúng datasheet Bosch
// ============================================================================
static double baro_compensate_temp(uint32_t raw_temp)
{
    double d1 = (double)raw_temp - calib.t1;
    double d2 = d1 * calib.t2;
    return d2 + (d1 * d1) * calib.t3;
}

static double baro_compensate_press(uint32_t raw_press, double temp_c)
{
    double d1 = calib.p6 * temp_c;
    double d2 = calib.p7 * temp_c * temp_c;
    double d3 = calib.p8 * temp_c * temp_c * temp_c;
    double out1 = calib.p5 + d1 + d2 + d3;

    d1 = calib.p2 * temp_c;
    d2 = calib.p3 * temp_c * temp_c;
    d3 = calib.p4 * temp_c * temp_c * temp_c;
    double out2 = (double)raw_press * (calib.p1 + d1 + d2 + d3);

    d1 = (double)raw_press * (double)raw_press;
    d2 = calib.p9 + calib.p10 * temp_c;
    d3 = d1 * d2;
    double out3 = d3 + ((double)raw_press * (double)raw_press * (double)raw_press) * calib.p11;

    return out1 + out2 + out3;
}

static double baro_press_to_altitude_m(double press_pa)
{
    return 44330.0 * (1.0 - pow(press_pa / BARO_SEA_LEVEL_PA, 0.1902949571836346));
}

// ============================================================================
//  Cấu hình cảm biến
// ============================================================================
static bool baro_wait_cmd_ready(uint32_t timeout_ms = 30)
{
    uint32_t t0 = millis();
    uint8_t status = 0;
    while ((millis() - t0) < timeout_ms)
    {
        if (baro_read_reg(BMP388_REG_STATUS, status))
        {
            if (status & BMP388_STATUS_CMD_RDY)
                return true;
        }
        delay(1);
    }
    return false;
}

static bool baro_configure()
{
    if (!baro_write_reg(BMP388_REG_PWR_CTRL, 0x00))
        return false; // về sleep
    delay(2);

    uint8_t osr = (uint8_t)((BARO_OSR_TEMP << 3) | BARO_OSR_PRESS);
    if (!baro_write_reg(BMP388_REG_OSR, osr))
        return false;
    if (!baro_write_reg(BMP388_REG_ODR, BARO_ODR_SETTING))
        return false;
    if (!baro_write_reg(BMP388_REG_CONFIG, (uint8_t)(BARO_IIR_SETTING << 1)))
        return false;

    uint8_t pwr = (uint8_t)(BMP388_PRESS_EN | BMP388_TEMP_EN | BMP388_MODE_NORMAL);
    if (!baro_write_reg(BMP388_REG_PWR_CTRL, pwr))
        return false;
    delay(5);

    uint8_t err = 0;
    if (!baro_read_reg(BMP388_REG_ERR, err))
        return false;
    if (err & 0x01)
    {
        Serial.println("BMP388 fatal_err");
        return false;
    }
    if (err & 0x02)
    {
        Serial.println("BMP388 cmd_err");
        return false;
    }
    if (err & 0x04)
    {
        Serial.println("BMP388 conf_err - OSR khong khop ODR");
        return false;
    }

    return true;
}

// ============================================================================
//  Đọc một mẫu. Trả về false khi cảm biến CHƯA có mẫu mới.
//
//  Bắt buộc phải xét cờ drdy: baro_task chạy 100 Hz còn cảm biến chỉ 50 Hz,
//  đọc mù sẽ lấy trùng một mẫu hai lần. Mẫu trùng đi vào KF độ cao thì bộ lọc
//  tưởng đó là phép đo mới độc lập rồi thu hẹp sai số ước lượng sai cách.
//
//  STATUS (0x03) nằm ngay trước DATA (0x04..0x09) nên đọc gộp 7 byte trong
//  MỘT giao dịch I2C, không tốn thêm lần nào.
// ============================================================================
bool baro_update()
{
    if (!baro_ready)
        return false;

    uint8_t buf[7];
    if (!baro_read_regs(BMP388_REG_STATUS, buf, 7))
        return false;
    if (!(buf[0] & BMP388_STATUS_DRDY_PRESS))
        return false;

    const uint8_t *data = &buf[1];
    uint32_t raw_press = u24le(&data[0]);
    uint32_t raw_temp = u24le(&data[3]);
    if (raw_press == 0 && raw_temp == 0)
        return false;

    double temp_c = baro_compensate_temp(raw_temp);
    double press_pa = baro_compensate_press(raw_press, temp_c);

    baro_temp_c = (float)temp_c;
    baro_press_pa = (float)press_pa;
    baro_alt_abs_m = (float)baro_press_to_altitude_m(press_pa);
    return true;
}

// Đo cao độ gốc lúc khởi động để sau này quy về "cao hơn điểm cất cánh"
static void baro_measure_reference(uint16_t samples)
{
    if (!baro_ready)
        return;

    double sum = 0.0;
    uint8_t period = baro_sample_period_ms();

    for (uint8_t i = 0; i < 10; i++)
    {
        baro_update();
        delay(period + 2);
    }

    uint16_t got = 0;
    for (uint16_t i = 0; i < samples * 3 && got < samples; i++)
    {
        if (baro_update())
        {
            sum += baro_alt_abs_m;
            got++;
        }
        delay(period / 2 + 1);
    }
    if (got == 0)
        return;

    baro_alt_ref_m = (float)(sum / got);
    Serial.print("Cao do goc = ");
    Serial.println(baro_alt_ref_m, 3);
}

static bool baro_detect()
{
    uint8_t chip_id = 0;
    // Đọc lại ba lần, đòi cả ba đều đúng 0x50 để chắc chắn đúng con chip
    for (uint8_t i = 0; i < 3; i++)
    {
        if (!baro_read_reg(BMP388_REG_CHIP_ID, chip_id))
            return false;
        if (chip_id != BMP388_CHIP_ID_VALUE)
            return false;
        delay(1);
    }
    return true;
}

bool baro_init()
{
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(400000);
    delay(10);

    baro_ready = false;

    if (!baro_detect())
    {
        Serial.println("BMP388 khong tim thay - kiem tra day I2C va dia chi");
        return false;
    }

    if (!baro_write_reg(BMP388_REG_CMD, BMP388_CMD_SOFT_RESET))
    {
        Serial.println("BMP388 reset that bai");
        return false;
    }
    delay(10);

    if (!baro_wait_cmd_ready(30))
    {
        Serial.println("BMP388 chua san sang nhan lenh");
        return false;
    }
    if (!baro_read_calibration())
    {
        Serial.println("BMP388 doc he so hieu chinh that bai");
        return false;
    }
    if (!baro_configure())
    {
        Serial.println("BMP388 cau hinh that bai");
        return false;
    }

    baro_ready = true;

    for (uint8_t i = 0; i < 5; i++)
    {
        baro_update();
        delay(baro_sample_period_ms() + 2);
    }
    baro_measure_reference(50);
    return true;
}

float baro_get_altitude_m() { return baro_alt_abs_m - baro_alt_ref_m; }

bool baro_is_present()
{
    uint8_t chip_id = 0;
    return (baro_read_reg(BMP388_REG_CHIP_ID, chip_id) && chip_id == BMP388_CHIP_ID_VALUE);
}
//=====================================================================================================
// control_task.ino
// ============================================================================
//  DroneFC_CRSF_DShot — Vòng điều khiển chính
//
//  Dựa trên mã nguồn gốc: KyThuatUAV FC — Copyright (C) 2026 Nguyễn Văn Quý
//  License: GPL-3.0
// ============================================================================
// ============================================================================
//  control_task  —  core 1, 500Hz. Đây là vòng điều khiển chính.
//
//  Thứ tự mỗi vòng:
//     đọc IMU -> ước lượng góc -> gia tốc thẳng đứng -> KF độ cao
//     -> lệnh tay điều khiển -> arm/mode -> PID -> mixer -> xuất ESC
// ============================================================================

// ---- Trạng thái đo được -----------------------------------------------------
float acc_x_g, acc_y_g, acc_z_g;                   // gia tốc thân máy, đơn vị g
float rate_roll_dps, rate_pitch_dps, rate_yaw_dps; // tốc độ góc, độ/giây
float roll_deg, pitch_deg;                         // góc nghiêng đã lọc
float acc_earth_z_g;                               // gia tốc thẳng đứng đã trừ trọng lực

// ---- Giá trị mong muốn ------------------------------------------------------
float roll_sp_deg, pitch_sp_deg;
float yaw_rate_sp_dps;
float climb_sp_cms;

// ---- Đầu ra bộ điều khiển ---------------------------------------------------
// u = tín hiệu điều khiển, ký hiệu chuẩn trong lý thuyết điều khiển
float u_roll, u_pitch, u_yaw;
int u_throttle;

// ---- Đầu ra cuối cùng cho 4 ESC ---------------------------------------------
int esc_1, esc_2, esc_3, esc_4;
int esc_lim_lo = ESC_IDLE, esc_lim_hi = ESC_IDLE;

// ---- Trạng thái bay ---------------------------------------------------------
int rc_ch[17];
bool rc_ok, rc_ok_prev;
bool armed = false;
int flight_mode = MODE_ANGLE;

// Khởi tạo bằng 2 chứ không phải -1: nếu cắm nguồn lúc công tắc arm ĐANG bật,
// điều kiện "vừa vào nấc 2" sẽ không thoả, nên drone không tự arm ngay khi
// khởi động. Muốn arm phải gạt công tắc xuống rồi gạt lên lại.
int arm_sw_prev = 2;

float baro_alt_m; // độ cao baro mới nhất, đơn vị mét

unsigned long failsafe_start_ms;
uint32_t loop_period_us;

void control_task(void *parameter)
{
    TickType_t wake = xTaskGetTickCount();
    for (;;)
    {
        static uint32_t t_prev_us = 0;
        uint32_t t_now_us = micros();
        loop_period_us = t_now_us - t_prev_us;
        t_prev_us = t_now_us;

        // ---------- 1. Cảm biến và ước lượng trạng thái ----------
        imu_update();
        acc_x_g = imu_get_acc_x_g();
        acc_y_g = imu_get_acc_y_g();
        acc_z_g = imu_get_acc_z_g();
        rate_roll_dps = imu_get_rate_roll_dps();
        rate_pitch_dps = imu_get_rate_pitch_dps();
        rate_yaw_dps = imu_get_rate_yaw_dps();
        roll_deg = imu_get_roll_deg();
        pitch_deg = imu_get_pitch_deg();

        // Gia tốc theo phương thẳng đứng của mặt đất, trừ đi 1g.
        float sr = sinf(roll_deg * DEG_TO_RAD), cr = cosf(roll_deg * DEG_TO_RAD);
        float sp = sinf(pitch_deg * DEG_TO_RAD), cp = cosf(pitch_deg * DEG_TO_RAD);
        acc_earth_z_g = -sp * acc_x_g + cp * sr * acc_y_g + cp * cr * acc_z_g - 1.0f;

        // KF độ cao
        baro_fetch_shared();
        alt_kf_update(baro_alt_m, acc_earth_z_g);

        // ---------- 2. Lệnh từ tay điều khiển ----------
        rc_ok_prev = rc_ok;
        rc_fetch_and_map();

        // ---------- 3. Arm / disarm ----------
        if (rc_ok)
        {
            if (rc_ch[CH_ARM] != 2)
            {
                armed = false;
            }
            else if (arm_sw_prev != 2 && rc_ch[CH_THROTTLE] < 1050)
            {
                armed = true;
            }
            arm_sw_prev = rc_ch[CH_ARM];

            if (rc_ch[CH_MODE] == 0)
                flight_mode = MODE_FOR_SW_0;
            else if (rc_ch[CH_MODE] == 1)
                flight_mode = MODE_FOR_SW_1;
            else
                flight_mode = MODE_FOR_SW_2;
        }
        else if (armed && sensor_present.baro)
        {
            if (rc_ok_prev)
                failsafe_start_ms = millis();
            flight_mode = MODE_FAILSAFE;
        }
        else
        {
            armed = false;
        }

        if (armed)
        {
            esc_lim_lo = ESC_MIN_ARMED;
            esc_lim_hi = ESC_MAX;
        }
        else
        {
            esc_lim_lo = ESC_IDLE;
            esc_lim_hi = ESC_IDLE;
            pid_reset_all();
        }

        // Hạ xong thì tự disarm
        if (!rc_ok && armed)
        {
            if (u_throttle < ESC_MIN_ARMED + 1 &&
                fabsf(alt_kf_get_climb_cms()) < 5.0f &&
                (millis() - failsafe_start_ms) > 2000)
            {
                armed = false;
            }
        }

        // ---------- 4. Bộ điều khiển ----------
        switch (flight_mode)
        {
        case MODE_ANGLE:
            if (u_throttle < 830)
                pid_bleed_integrators();
            mode_angle();
            break;
        case MODE_ALT_HOLD:
            mode_alt_hold();
            break;
        case MODE_FAILSAFE:
            mode_failsafe();
            break;
        default:
            mode_angle();
            break;
        }

        // ---------- 5. Mixer quad X và xuất ESC ----------
        esc_1 = u_throttle - u_roll - u_pitch - u_yaw;
        esc_2 = u_throttle + u_roll + u_pitch - u_yaw;
        esc_3 = u_throttle + u_roll - u_pitch + u_yaw;
        esc_4 = u_throttle - u_roll + u_pitch + u_yaw;
        esc_clamp(esc_lim_lo, esc_lim_hi);

        if (!armed)
        {
            esc_1 = esc_2 = esc_3 = esc_4 = ESC_IDLE;
        }
        esc_write(esc_1, esc_2, esc_3, esc_4);

        telemetry_snapshot();
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(PERIOD_CTRL_MS));
    }
}

// ============================================================================
//  Các chế độ bay
// ============================================================================

void mode_angle()
{
    u_throttle = rc_ch[CH_THROTTLE] * 0.8f;
    u_roll = pid_rate_roll(rate_roll_dps, pid_tilt_roll(roll_deg, roll_sp_deg));
    u_pitch = pid_rate_pitch(rate_pitch_dps, pid_tilt_pitch(pitch_deg, pitch_sp_deg));
    u_yaw = pid_rate_yaw(rate_yaw_dps, yaw_rate_sp_dps);
}

void mode_alt_hold()
{
    u_throttle = ESC_IDLE + pid_climb(alt_kf_get_climb_cms(), climb_sp_cms);
    u_roll = pid_rate_roll(rate_roll_dps, pid_tilt_roll(roll_deg, roll_sp_deg));
    u_pitch = pid_rate_pitch(rate_pitch_dps, pid_tilt_pitch(pitch_deg, pitch_sp_deg));
    u_yaw = pid_rate_yaw(rate_yaw_dps, yaw_rate_sp_dps);
}

void mode_failsafe()
{
    u_throttle = ESC_IDLE + pid_climb(alt_kf_get_climb_cms(), FAILSAFE_DESCENT_CMS);
    u_roll = pid_rate_roll(rate_roll_dps, pid_tilt_roll(roll_deg, 0));
    u_pitch = pid_rate_pitch(rate_pitch_dps, pid_tilt_pitch(pitch_deg, 0));
    u_yaw = pid_rate_yaw(rate_yaw_dps, 0);
}

// ============================================================================
//  Lấy dữ liệu dùng chung từ các task khác
// ============================================================================

void baro_fetch_shared()
{
    if (xSemaphoreTake(mtx_baro, 0) == pdTRUE)
    {
        baro_alt_m = shared_baro_alt_m;
        xSemaphoreGive(mtx_baro);
    }
}

int rc_switch_position(int ch_value)
{
    if (ch_value > 900 && ch_value < 1100)
        return 0;
    else if (ch_value > 1400 && ch_value < 1600)
        return 1;
    else if (ch_value > 1900 && ch_value < 2100)
        return 2;
    else
        return 0;
}

void rc_fetch_and_map()
{
    if (xSemaphoreTake(mtx_rc, 0) == pdTRUE)
    {
        rc_ok = shared_rc_ok;
        for (int i = 1; i <= 16; i++)
            rc_ch[i] = shared_rc_ch[i];
        xSemaphoreGive(mtx_rc);
    }
    for (int i = 5; i <= 8; i++)
        rc_ch[i] = rc_switch_position(rc_ch[i]);

    const float a = 0.05f;
    roll_sp_deg = roll_sp_deg * (1 - a) + (float(rc_ch[CH_ROLL] - 1500) / (500.0f / LIM_TILT_DEG)) * a;
    pitch_sp_deg = pitch_sp_deg * (1 - a) + (float(rc_ch[CH_PITCH] - 1500) / (500.0f / LIM_TILT_DEG)) * a;
    yaw_rate_sp_dps = yaw_rate_sp_dps * (1 - a) + (-float(rc_ch[CH_YAW] - 1500) / (500.0f / LIM_YAW_RATE_DPS)) * a;
    climb_sp_cms = climb_sp_cms * (1 - a) + (float(rc_ch[CH_THROTTLE] - 1500) / (500.0f / LIM_CLIMB_RATE_CMS)) * a;
}

void esc_clamp(int lo, int hi)
{
    if (esc_1 < lo)
        esc_1 = lo;
    if (esc_1 > hi)
        esc_1 = hi;
    if (esc_2 < lo)
        esc_2 = lo;
    if (esc_2 > hi)
        esc_2 = hi;
    if (esc_3 < lo)
        esc_3 = lo;
    if (esc_3 > hi)
        esc_3 = hi;
    if (esc_4 < lo)
        esc_4 = lo;
    if (esc_4 > hi)
        esc_4 = hi;
}

void telemetry_snapshot()
{
    if (xSemaphoreTake(mtx_tlm, 0) != pdTRUE)
        return;

    tlm_roll_deg = roll_deg;
    tlm_pitch_deg = pitch_deg;
    tlm_roll_sp_deg = roll_sp_deg;
    tlm_pitch_sp_deg = pitch_sp_deg;

    tlm_rate_roll_dps = rate_roll_dps;
    tlm_rate_pitch_dps = rate_pitch_dps;
    tlm_rate_yaw_dps = rate_yaw_dps;
    tlm_rate_roll_sp_dps = pid_tilt_roll_output();
    tlm_rate_pitch_sp_dps = pid_tilt_pitch_output();

    tlm_alt_cm = alt_kf_get_altitude_cm();
    tlm_climb_cms = alt_kf_get_climb_cms();
    tlm_climb_sp_cms = climb_sp_cms;

    tlm_acc_x_g = acc_x_g;
    tlm_acc_y_g = acc_y_g;
    tlm_acc_z_g = acc_z_g;

    for (int i = 1; i <= 16; i++)
        tlm_rc_ch[i] = rc_ch[i];
    tlm_esc[1] = esc_1;
    tlm_esc[2] = esc_2;
    tlm_esc[3] = esc_3;
    tlm_esc[4] = esc_4;

    tlm_flight_mode = flight_mode;
    tlm_armed = armed;
    tlm_rc_ok = rc_ok;
    tlm_loop_us = loop_period_us;

    xSemaphoreGive(mtx_tlm);
}
//======================================================================================
// DroneFC_CRFS_Dshot.ino
// ============================================================================
//  DroneFC_CRSF_DShot — firmware bay ESP32 (FreeRTOS)
//  Chế độ Angle + Alt Hold
//
//  Dựa trên mã nguồn gốc: KyThuatUAV FC — Copyright (C) 2026 Nguyễn Văn Quý
//  Chỉnh sửa để phù hợp bộ linh kiện:
//    - ESP32 WROOM32 38Pin
//    - ICM-20602 (SPI)
//    - BMP388 (I2C)
//    - Radiomaster RP3 ExpressLRS 2.4GHz (giao thức CRSF qua UART)
//    - GOKU G55M 4-in-1 ESC BLHeli32 (DShot600)
//    - Motor RS2205 2300KV, khung 5", cánh 5", pin 3S 2200mAh
//    - Tay cầm Pocket Crush (ELRS 2.4GHz, EdgeTX)
//
//  Thay đổi chính so với bản gốc:
//    1. RC: SBUS → CRSF (420000 baud, 8N1, không đảo tín hiệu)
//    2. ESC: PWM 391Hz → DShot600 (giao thức số, không cần hiệu chỉnh ESC)
//    3. Pin mapping phù hợp ESP32 WROOM32 38Pin
//
//  License: GPL-3.0 (giữ nguyên từ bản gốc)
// ============================================================================
// ============================================================================
//   DroneFC  —  ESP32 + ICM20602 (SPI) + BMP388 (I2C) + CRSF + DShot600
//
//   Hai chế độ bay: ANGLE và ALT HOLD.
//
//   Khung chạy FreeRTOS:
//       core 1 : control_task - IMU, ước lượng góc, KF độ cao, PID, ESC (500Hz)
//       core 0 : rc_task      - giải mã CRSF                          (500Hz)
//       core 0 : baro_task    - đọc BMP388 và in debug                (100Hz)
//
//   QUY ƯỚC ĐẶT TÊN
//     HẰNG SỐ CẤU HÌNH  viết HOA, có tiền tố miền:  PIN_ , CH_ , MODE_ , LIM_ , KP_
//     Hàm của một khối  có tiền tố tên khối:        imu_ , baro_ , rc_ , esc_ , pid_ , alt_
//     Biến dùng chung giữa các task có tiền tố      shared_
//     Bản chụp để in debug có tiền tố               tlm_
//     Giá trị mong muốn (setpoint) có hậu tố        _sp
//     Đơn vị nằm luôn trong tên khi dễ nhầm:        _deg _dps _cm _cms _m _g _us
//     Đầu ra bộ điều khiển đặt là                   u_  (ký hiệu chuẩn của
//                                                   tín hiệu điều khiển)
// ============================================================================

#include "types.h"

// ============================================================================
//                      CẤU HÌNH  —  SỬA TRỰC TIẾP TẠI ĐÂY
// ============================================================================

// ---- Chân kết nối ESC (DShot600) -------------------------------------------
// GOKU G55M 4-in-1 ESC: 4 tín hiệu motor vào 4 chân GPIO.
// Chọn chân có khả năng RMT (Remote Control Transceiver) của ESP32 để xuất
// DShot. ESP32 có 8 kênh RMT, ta dùng 4.
#define PIN_ESC_1 27 // Motor 1 — trước phải (CW)
#define PIN_ESC_2 26 // Motor 2 — sau trái   (CW)
#define PIN_ESC_3 25 // Motor 3 — trước trái (CCW)
#define PIN_ESC_4 33 // Motor 4 — sau phải   (CCW)

// ---- Chân IMU (SPI) --------------------------------------------------------
#define PIN_IMU_CS 5   // ICM-20602 chip select
#define PIN_IMU_LED 14 // LED báo trạng thái hiệu chỉnh gyro
// SPI mặc định: MOSI=23, MISO=19, SCK=18 (không cần define, SPI.begin() tự dùng)

// ---- Chân CRSF RX (Radiomaster RP3 ELRS) -----------------------------------
// RP3 xuất tín hiệu CRSF qua 1 chân TX. Nối chân TX của RP3 vào chân RX
// của UART2 trên ESP32. CRSF là giao thức KHÔNG ĐẢO, 420000 baud, 8N1.
#define PIN_CRSF_RX 16 // UART2 RX — nối tới TX của RP3
#define PIN_CRSF_TX 17 // UART2 TX — nối tới RX của RP3 (cho telemetry)

// ---- Chân I2C (BMP388) -----------------------------------------------------
#define PIN_I2C_SDA 21
#define PIN_I2C_SCL 22
#define BARO_I2C_ADDR 0x76 // BMP388, nối SDO xuống GND = 0x76, lên VCC = 0x77

// ---- Kênh tay điều khiển (mapping EdgeTX mặc định) -------------------------
// EdgeTX trên Pocket Crush gán: CH1=Aileron, CH2=Elevator, CH3=Throttle, CH4=Rudder
// Mảng CRSF bắt đầu từ 0, nhưng ta quy ước index 1-based cho đồng nhất với code gốc
#define CH_ROLL 1     // Aileron
#define CH_PITCH 2    // Elevator
#define CH_THROTTLE 3 // Throttle
#define CH_YAW 4      // Rudder
#define CH_ARM 5      // Công tắc arm (gán trong EdgeTX, ví dụ SB)
#define CH_MODE 6     // Công tắc 3 nấc chọn chế độ bay (gán SC)

// ---- Chế độ bay và cách gán cho từng nấc công tắc --------------------------
#define MODE_ANGLE 0
#define MODE_ALT_HOLD 1
#define MODE_FAILSAFE 2 // dùng nội bộ, không gán cho công tắc

#define MODE_FOR_SW_0 MODE_ANGLE
#define MODE_FOR_SW_1 MODE_ANGLE
#define MODE_FOR_SW_2 MODE_ALT_HOLD

// ---- Giới hạn lệnh từ cần gạt ----------------------------------------------
#define LIM_TILT_DEG 20.0f          // độ nghiêng tối đa ở chế độ angle
#define LIM_YAW_RATE_DPS 100.0f     // tốc độ xoay tối đa, độ/giây
#define LIM_CLIMB_RATE_CMS 100.0f   // tốc độ lên xuống tối đa, cm/giây
#define FAILSAFE_DESCENT_CMS -60.0f // tốc độ hạ khi mất sóng, cm/giây

// ---- Giới hạn DShot (0-2047) ------------------------------------------------
// DShot giá trị 0 = disarmed, 48 = idle khi armed, 2047 = full throttle
// Quy đổi nội bộ: code vẫn dùng thang 800-1600 như cũ, hàm esc_write() tự
// chuyển sang DShot 0-2047 trước khi gửi.
#define ESC_IDLE 800      // giá trị nội bộ khi chưa arm (sẽ map thành DShot 0)
#define ESC_MIN_ARMED 900 // đã arm thì không cho tụt dưới mức này
#define ESC_MAX 1600

// ---- Hệ số PID --------------------------------------------------------------
// RS2205 2300KV trên khung 5" với pin 3S gần tương đương cấu hình gốc (2204 2300KV),
// nên giữ nguyên bộ PID mặc định. Nếu rung thì giảm KP_RATE và KD_RATE.
const float KP_RATE_ROLL = 0.5f, KI_RATE_ROLL = 1.4f, KD_RATE_ROLL = 0.03f;
const float KP_RATE_PITCH = 0.5f, KI_RATE_PITCH = 1.4f, KD_RATE_PITCH = 0.03f;
const float KP_RATE_YAW = 1.0f, KI_RATE_YAW = 10.0f, KD_RATE_YAW = 0.0f;

const float KP_TILT_ROLL = 10.0f, KI_TILT_ROLL = 0.0f, KD_TILT_ROLL = 0.0f;
const float KP_TILT_PITCH = 10.0f, KI_TILT_PITCH = 0.0f, KD_TILT_PITCH = 0.0f;

const float KP_CLIMB = 3.0f, KI_CLIMB = 15.0f, KD_CLIMB = 0.0f;

// Chặn khâu tích phân và chặn đầu ra của từng tầng
const float I_LIM_RATE = 200.0f, U_LIM_RATE = 400.0f;
const float I_LIM_TILT = 200.0f, U_LIM_TILT = 400.0f;
const float I_LIM_YAW = 100.0f, U_LIM_YAW = 100.0f;
const float I_LIM_CLIMB = 700.0f, P_LIM_CLIMB = 200.0f;

// ---- Bù lệch gia tốc kế (đơn vị g) -----------------------------------------
#define ACC_OFFSET_X_G 0.00f
#define ACC_OFFSET_Y_G 0.00f
#define ACC_OFFSET_Z_G 0.00f

// ---- Bộ lọc thông thấp bên trong ICM20602 ----------------------------------
#define IMU_DLPF_GYRO 0x06
#define IMU_DLPF_ACC 0x05

// ---- Debug ------------------------------------------------------------------
// Mỗi lần chỉ bật MỘT dòng.
// #define DEBUG_ATTITUDE      // góc hiện tại so với góc mong muốn
// #define DEBUG_RATE          // tốc độ góc mong muốn so với thực tế
// #define DEBUG_ALTITUDE      // độ cao và tốc độ lên xuống sau KF
// #define DEBUG_RC            // giá trị các kênh tay điều khiển
// #define DEBUG_ACC_OFFSET    // để đo ACC_OFFSET_* bên trên
// #define DEBUG_LOOP_TIME     // chu kì vòng điều khiển, phải luôn ~2000 us

// ============================================================================
//                        HẾT PHẦN CẤU HÌNH
// ============================================================================

#define PERIOD_CTRL_MS 2  // 500 Hz
#define PERIOD_RC_MS 2    // 500 Hz
#define PERIOD_BARO_MS 10 // 100 Hz
#define DT_CTRL 0.002f    // chu kì vòng điều khiển, tính bằng giây

// Trạng thái cảm biến lúc khởi động
SensorPresent sensor_present;

// Mutex cho từng nhóm dữ liệu dùng chung
SemaphoreHandle_t mtx_rc;
SemaphoreHandle_t mtx_baro;
SemaphoreHandle_t mtx_tlm;

// rc_task ghi, control_task đọc
int shared_rc_ch[17];
bool shared_rc_ok = false;

// baro_task ghi, control_task đọc
float shared_baro_alt_m = 0.0f;
bool shared_baro_new = false;

// control_task ghi, baro_task đọc để in debug
float tlm_roll_deg, tlm_pitch_deg;
float tlm_roll_sp_deg, tlm_pitch_sp_deg;
float tlm_rate_roll_dps, tlm_rate_pitch_dps, tlm_rate_yaw_dps;
float tlm_rate_roll_sp_dps, tlm_rate_pitch_sp_dps;
float tlm_alt_cm, tlm_climb_cms, tlm_climb_sp_cms;
float tlm_acc_x_g, tlm_acc_y_g, tlm_acc_z_g;
int tlm_rc_ch[17];
int tlm_esc[5];
int tlm_flight_mode;
bool tlm_armed, tlm_rc_ok;
uint32_t tlm_loop_us;

void setup()
{
    Serial.begin(500000);

    esc_init();
    esc_write(ESC_IDLE, ESC_IDLE, ESC_IDLE, ESC_IDLE);

    rc_init();

    // Kiểm tra IMU TRƯỚC khi hiệu chỉnh gyro
    imu_init_bus();
    sensor_present.imu = imu_is_present();
    Serial.printf("ICM20602 = %d\n", sensor_present.imu);
    if (!sensor_present.imu)
    {
        while (1)
        {
            Serial.println("KHONG TIM THAY ICM20602 - dung lai, khong cho bay");
            digitalWrite(PIN_IMU_LED, !digitalRead(PIN_IMU_LED));
            delay(300);
        }
    }
    imu_init();

    sensor_present.baro = baro_init();
    Serial.printf("BMP388   = %d\n", sensor_present.baro);

    alt_kf_init();

    Serial.println("DroneFC CRSF+DShot - angle + alt hold");
    Serial.println("Linh kien: ESP32 WROOM32 + ICM20602 + BMP388");
    Serial.println("           RP3 ELRS (CRSF) + GOKU G55M (DShot600)");
    Serial.println("           RS2205 2300KV + 5\" frame + 3S 2200mAh");

    mtx_rc = xSemaphoreCreateMutex();
    mtx_baro = xSemaphoreCreateMutex();
    mtx_tlm = xSemaphoreCreateMutex();

    if (mtx_rc == NULL || mtx_baro == NULL || mtx_tlm == NULL)
    {
        Serial.println("Loi: khong tao duoc Mutex!");
        while (1)
            delay(1000);
    }

    xTaskCreatePinnedToCore(rc_task, "RC", 4096, NULL, 4, NULL, 0);
    xTaskCreatePinnedToCore(baro_task, "BARO", 4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(control_task, "CTRL", 8192, NULL, 5, NULL, 1);

    vTaskDelete(NULL);
}

void loop()
{
    // Không dùng, mọi thứ chạy trong các task
}

// ============================================================================
//  rc_task  —  core 0, 500Hz — giải mã CRSF từ Radiomaster RP3
// ============================================================================
void rc_task(void *parameter)
{
    TickType_t wake = xTaskGetTickCount();
    for (;;)
    {
        bool ok = rc_update();

        if (xSemaphoreTake(mtx_rc, 0) == pdTRUE)
        {
            shared_rc_ok = ok;
            for (int i = 1; i <= 16; i++)
                shared_rc_ch[i] = rc_get_channel(i);
            xSemaphoreGive(mtx_rc);
        }
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(PERIOD_RC_MS));
    }
}

// ============================================================================
//  baro_task  —  core 0, 100Hz. Kiêm luôn việc in debug.
// ============================================================================
void baro_task(void *parameter)
{
    TickType_t wake = xTaskGetTickCount();
    for (;;)
    {
        if (sensor_present.baro)
        {
            if (baro_update())
            {
                float alt_m = baro_get_altitude_m();
                if (xSemaphoreTake(mtx_baro, 0) == pdTRUE)
                {
                    shared_baro_alt_m = alt_m;
                    shared_baro_new = true;
                    xSemaphoreGive(mtx_baro);
                }
            }
        }

        debug_print();

        vTaskDelayUntil(&wake, pdMS_TO_TICKS(PERIOD_BARO_MS));
    }
}

// ============================================================================
//  In debug
// ============================================================================
void debug_print()
{
#if defined(DEBUG_ATTITUDE) || defined(DEBUG_RATE) || defined(DEBUG_ALTITUDE) || \
    defined(DEBUG_RC) || defined(DEBUG_ACC_OFFSET) || defined(DEBUG_LOOP_TIME)

    static uint32_t tick = 0;
    if (++tick % 5)
        return;

    if (xSemaphoreTake(mtx_tlm, 0) != pdTRUE)
        return;

#ifdef DEBUG_ATTITUDE
    Serial.printf("%.1f,%.1f,%.1f,%.1f\n",
                  tlm_roll_sp_deg, tlm_roll_deg, tlm_pitch_sp_deg, tlm_pitch_deg);
#endif

#ifdef DEBUG_RATE
    Serial.printf("%.0f,%.0f,%.0f,%.0f\n",
                  tlm_rate_roll_sp_dps, tlm_rate_roll_dps,
                  tlm_rate_pitch_sp_dps, tlm_rate_pitch_dps);
#endif

#ifdef DEBUG_ALTITUDE
    Serial.printf("%.1f,%.1f,%.1f\n", tlm_alt_cm, tlm_climb_cms, tlm_climb_sp_cms);
#endif

#ifdef DEBUG_RC
    Serial.printf("ch1:%d ch2:%d ch3:%d ch4:%d arm:%d mode:%d | rc_ok:%d armed:%d fm:%d\n",
                  tlm_rc_ch[CH_ROLL], tlm_rc_ch[CH_PITCH],
                  tlm_rc_ch[CH_THROTTLE], tlm_rc_ch[CH_YAW],
                  tlm_rc_ch[CH_ARM], tlm_rc_ch[CH_MODE],
                  tlm_rc_ok, tlm_armed, tlm_flight_mode);
#endif

#ifdef DEBUG_ACC_OFFSET
    static float ax = 0, ay = 0, az = 1;
    ax = ax * 0.98f + tlm_acc_x_g * 0.02f;
    ay = ay * 0.98f + tlm_acc_y_g * 0.02f;
    az = az * 0.98f + tlm_acc_z_g * 0.02f;
    Serial.printf("X:%.4f Y:%.4f Z:%.4f\n", ax, ay, az);
#endif

#ifdef DEBUG_LOOP_TIME
    Serial.printf("loop_us:%u\n", tlm_loop_us);
#endif

    xSemaphoreGive(mtx_tlm);
#endif
}
//===========================================================================================
// esc_dshot.ino
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

#define DSHOT_T1H_TICKS 100 // bit '1' high duration: 1.25 µs = 100 ticks
#define DSHOT_T1L_TICKS 34  // bit '1' low duration:  0.42 µs = 34 ticks
#define DSHOT_T0H_TICKS 50  // bit '0' high duration: 0.625 µs = 50 ticks
#define DSHOT_T0L_TICKS 84  // bit '0' low duration:  1.045 µs = 84 ticks

#define DSHOT_FRAME_BITS 16
#define DSHOT_THROTTLE_MIN 48 // giá trị throttle nhỏ nhất khi armed
#define DSHOT_THROTTLE_MAX 2047

// Mapping RMT channels cho 4 ESC
static const rmt_channel_t dshot_rmt_ch[4] = {
    RMT_CHANNEL_0, // Motor 1
    RMT_CHANNEL_1, // Motor 2
    RMT_CHANNEL_2, // Motor 3
    RMT_CHANNEL_3  // Motor 4
};

static const gpio_num_t dshot_pins[4] = {
    (gpio_num_t)PIN_ESC_1,
    (gpio_num_t)PIN_ESC_2,
    (gpio_num_t)PIN_ESC_3,
    (gpio_num_t)PIN_ESC_4};

// Buffer cho RMT items (16 bit + 1 end marker)
static rmt_item32_t dshot_items[4][DSHOT_FRAME_BITS + 1];

// ---- Tính CRC DShot ---------------------------------------------------------
// CRC = XOR của ba nhóm 4 bit: (throttle >> 7) ^ (throttle >> 3) ^ (throttle & 0x0F)
// Nhưng thực tế tính trên 12 bit gốc (11 bit throttle + 1 bit telemetry):
//   value = (throttle << 1) | telem
//   crc = (value ^ (value >> 4) ^ (value >> 8)) & 0x0F
static uint16_t dshot_encode(uint16_t throttle, bool telem_request)
{
    uint16_t value = (throttle << 1) | (telem_request ? 1 : 0);
    uint16_t crc = (value ^ (value >> 4) ^ (value >> 8)) & 0x0F;
    return (value << 4) | crc;
}

// ---- Chuyển frame 16 bit thành RMT items ------------------------------------
static void dshot_build_rmt_items(rmt_item32_t *items, uint16_t frame)
{
    for (int i = 0; i < DSHOT_FRAME_BITS; i++)
    {
        // MSB first
        if (frame & (1 << (15 - i)))
        {
            // Bit '1'
            items[i].duration0 = DSHOT_T1H_TICKS;
            items[i].level0 = 1;
            items[i].duration1 = DSHOT_T1L_TICKS;
            items[i].level1 = 0;
        }
        else
        {
            // Bit '0'
            items[i].duration0 = DSHOT_T0H_TICKS;
            items[i].level0 = 1;
            items[i].duration1 = DSHOT_T0L_TICKS;
            items[i].level1 = 0;
        }
    }
    // End marker
    items[DSHOT_FRAME_BITS].duration0 = 0;
    items[DSHOT_FRAME_BITS].level0 = 0;
    items[DSHOT_FRAME_BITS].duration1 = 0;
    items[DSHOT_FRAME_BITS].level1 = 0;
}

// ---- Gửi DShot frame cho 1 ESC qua RMT -------------------------------------
static void dshot_send_one(int motor_idx, uint16_t throttle)
{
    uint16_t frame = dshot_encode(throttle, false); // không yêu cầu telemetry
    dshot_build_rmt_items(dshot_items[motor_idx], frame);
    rmt_write_items(dshot_rmt_ch[motor_idx], dshot_items[motor_idx],
                    DSHOT_FRAME_BITS + 1, false); // false = không chờ xong
}

// ---- Chuyển đổi từ thang nội bộ 800-1600 sang DShot 0-2047 -----------------
// Giữ nguyên thang 800-1600 trong toàn bộ code điều khiển (PID, mixer, v.v.)
// để không phải sửa logic đã kiểm chứng. Chỉ chuyển đổi ở đây trước khi gửi.
static uint16_t internal_to_dshot(int internal_value)
{
    if (internal_value <= ESC_IDLE)
        return 0; // disarmed
    if (internal_value < ESC_MIN_ARMED)
        return 0; // dưới mức arm

    // Map ESC_MIN_ARMED..ESC_MAX → DSHOT_THROTTLE_MIN..DSHOT_THROTTLE_MAX
    long dshot_val = map((long)internal_value, ESC_MIN_ARMED, ESC_MAX,
                         DSHOT_THROTTLE_MIN, DSHOT_THROTTLE_MAX);

    if (dshot_val < DSHOT_THROTTLE_MIN)
        dshot_val = DSHOT_THROTTLE_MIN;
    if (dshot_val > DSHOT_THROTTLE_MAX)
        dshot_val = DSHOT_THROTTLE_MAX;

    return (uint16_t)dshot_val;
}

// ============================================================================
//  API công khai — cùng giao diện với bản PWM cũ
// ============================================================================

void esc_init()
{
    // Cấu hình RMT cho 4 kênh DShot
    for (int i = 0; i < 4; i++)
    {
        rmt_config_t cfg = RMT_DEFAULT_CONFIG_TX(dshot_pins[i], dshot_rmt_ch[i]);
        cfg.clk_div = 1;       // clock divider = 1 → 80 MHz, 12.5 ns/tick
        cfg.mem_block_num = 1; // 1 block = 64 items, đủ cho 16 bit DShot
        cfg.tx_config.carrier_en = false;
        cfg.tx_config.loop_en = false;
        cfg.tx_config.idle_output_en = true;
        cfg.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;

        rmt_config(&cfg);
        rmt_driver_install(dshot_rmt_ch[i], 0, 0);
    }

    // Gửi lệnh disarm (throttle = 0) vài lần để ESC nhận ra DShot
    for (int j = 0; j < 10; j++)
    {
        for (int i = 0; i < 4; i++)
        {
            dshot_send_one(i, 0);
        }
        delay(10);
    }

    // Chờ ESC khởi tạo
    delay(200);

    Serial.println("[DShot600 4 kenh khoi tao xong]");

    // THÁO CÁNH rồi mở khối này để dò chân nào ra motor nào.
    // Mỗi motor sẽ quay nhẹ lần lượt theo đúng thứ tự 1 → 2 → 3 → 4.
    //   esc_write(ESC_MIN_ARMED, ESC_IDLE, ESC_IDLE, ESC_IDLE); delay(400);
    //   esc_write(ESC_IDLE, ESC_MIN_ARMED, ESC_IDLE, ESC_IDLE); delay(400);
    //   esc_write(ESC_IDLE, ESC_IDLE, ESC_MIN_ARMED, ESC_IDLE); delay(400);
    //   esc_write(ESC_IDLE, ESC_IDLE, ESC_IDLE, ESC_MIN_ARMED); delay(400);
    //   esc_write(ESC_IDLE, ESC_IDLE, ESC_IDLE, ESC_IDLE);      delay(1000);
}

void esc_write(int m1, int m2, int m3, int m4)
{
    dshot_send_one(0, internal_to_dshot(m1));
    dshot_send_one(1, internal_to_dshot(m2));
    dshot_send_one(2, internal_to_dshot(m3));
    dshot_send_one(3, internal_to_dshot(m4));
}
//====================================================================================
// imu_icm20602.ino
// ============================================================================
//  KyThuatUAV FC - firmware bay ESP32 (FreeRTOS), che do Angle + Alt Hold
//
//  Copyright (C) 2026  Nguyễn Văn Quý (Ky Thuat UAV)
//
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program.  If not, see <https://www.gnu.org/licenses/>.
//
//  Giu nguyen phan ghi cong nay khi dung lai hoac chinh sua ma nguon.
//
//  Nguon tham khao va ghi cong day du: xem file LOI_NHAN_VA_GHI_CONG.md
//  Lien he: Nguyen Van Quy - 0817 550 271 (Zalo)
// ============================================================================
// ============================================================================
//  ICM20602 - driver thanh ghi qua SPI, kèm bộ lọc Kalman 1D ước lượng góc.
//
//  Góc roll/pitch lấy từ Kalman 1D: gyro cho phần biến thiên nhanh, gia tốc kế
//  kéo về cho khỏi trôi. Bản đầy đủ dùng thêm Madgwick/Mahony để có góc yaw từ
//  la bàn, ở đây bỏ vì angle và alt hold không cần giữ hướng - yaw chỉ điều
//  khiển theo tốc độ xoay.
//
//  GHI CÔNG: cách ước lượng góc bằng bộ lọc Kalman một chiều ở hàm
//  imu_kalman_1d() học từ tài liệu của Carbon Aeronautics
//  (https://github.com/CarbonAeronautics). Chi tiết xem file LOI_NHAN_VA_GHI_CONG.md.
// ============================================================================

#include <SPI.h>

#define IMU_SPI_HZ 10000000 // 10 MHz

// Địa chỉ thanh ghi
#define REG_WHO_AM_I 0x75
#define REG_PWR_MGMT_1 0x6B
#define REG_ACCEL_XOUT_H 0x3B
#define REG_GYRO_XOUT_H 0x43
#define REG_GYRO_CONFIG 0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_CONFIG 0x1A
#define REG_ACCEL_CONFIG2 0x1D

#define ICM20602_WHO_AM_I_VALUE 0x12

// Hệ số quy đổi số thô sang đơn vị vật lý
#define GYRO_LSB_PER_DPS 16.4f // thang do +-2000 do/giay
#define ACC_LSB_PER_G 2048.0f  // thang do +-16g

static SPISettings imu_spi(IMU_SPI_HZ, MSBFIRST, SPI_MODE0);

// Số đo thô đã quy đổi, CHƯA trừ bias
static float imu_acc_x_g, imu_acc_y_g, imu_acc_z_g;
static float imu_rate_roll_dps, imu_rate_pitch_dps, imu_rate_yaw_dps;

// Bias gyro đo được lúc khởi động
static float gyro_bias_roll_dps, gyro_bias_pitch_dps, gyro_bias_yaw_dps;

// Góc suy ra từ riêng gia tốc kế, dùng làm phép đo cho Kalman
static float acc_roll_deg, acc_pitch_deg;

// Kết quả ước lượng góc
static float att_roll_deg = 0.0f, att_roll_var = 2 * 2;
static float att_pitch_deg = 0.0f, att_pitch_var = 2 * 2;

// Chỉ mở bus, chưa đụng gì tới cảm biến. Tách riêng để setup() kiểm tra
// WHO_AM_I trước khi bước vào vòng hiệu chỉnh gyro.
void imu_init_bus()
{
    SPI.begin();
    pinMode(PIN_IMU_CS, OUTPUT);
    digitalWrite(PIN_IMU_CS, HIGH);
    pinMode(PIN_IMU_LED, OUTPUT);
    digitalWrite(PIN_IMU_LED, LOW);
}

bool imu_is_present()
{
    return (imu_read_reg(REG_WHO_AM_I) == ICM20602_WHO_AM_I_VALUE);
}

void imu_init()
{
    imu_write_reg(REG_PWR_MGMT_1, 0x00); // thoát chế độ ngủ
    delay(100);
    imu_write_reg(REG_ACCEL_CONFIG, 0x18); // thang đo gia tốc +-16g
    imu_write_reg(REG_GYRO_CONFIG, 0x18);  // thang đo gyro +-2000 độ/giây
    imu_write_reg(REG_CONFIG, IMU_DLPF_GYRO);
    imu_write_reg(REG_ACCEL_CONFIG2, IMU_DLPF_ACC);
    delay(100);

    Serial.println("[ICM20602 khoi tao xong]");
    Serial.print("CONFIG 0x");
    Serial.println(imu_read_reg(REG_CONFIG), HEX);
    Serial.print("ACCEL_CONFIG2 0x");
    Serial.println(imu_read_reg(REG_ACCEL_CONFIG2), HEX);

#ifdef CALIBRATE_ACCEL
    imu_calibrate_accel(); // hàm này không bao giờ trả về
#endif

    imu_calibrate_gyro();
}

#ifdef CALIBRATE_ACCEL
// ===== Hiệu chỉnh gia tốc kế =====
// Chỉ chạy khi bật #define CALIBRATE_ACCEL. Đo xong thì in ra đúng ba dòng cần
// dán vào khối CẤU HÌNH rồi dừng hẳn - cố tình không cho bay ở chế độ này.
//
// Nguyên lý: đặt drone nằm phẳng thì đúng ra gia tốc kế phải đọc
// X = 0, Y = 0, Z = +1g. Lệch bao nhiêu so với ba số đó chính là sai số cần bù.
void imu_calibrate_accel()
{
    const int SAMPLES = 2000; // 2000 mẫu x 2ms = 4 giây

    Serial.println();
    Serial.println("=========================================");
    Serial.println("     HIEU CHINH GIA TOC KE");
    Serial.println("=========================================");
    Serial.println("Dat drone nam phang tren mat ban phang.");
    Serial.println("Bo tay ra, dung cham vao ban trong luc do.");

    for (int s = 5; s > 0; s--)
    {
        Serial.printf("Bat dau sau %d giay...\n", s);
        delay(1000);
    }
    Serial.println("Dang do, giu yen...");

    double sum_x = 0, sum_y = 0, sum_z = 0;
    for (int i = 0; i < SAMPLES; i++)
    {
        imu_read_raw();
        sum_x += imu_acc_x_g; // lấy số THÔ, chưa cộng offset cũ
        sum_y += imu_acc_y_g;
        sum_z += imu_acc_z_g;
        if ((i % 500) == 0)
            digitalWrite(PIN_IMU_LED, !digitalRead(PIN_IMU_LED));
        delay(2);
    }
    digitalWrite(PIN_IMU_LED, LOW);

    float mean_x = sum_x / SAMPLES;
    float mean_y = sum_y / SAMPLES;
    float mean_z = sum_z / SAMPLES;

    Serial.println();
    Serial.printf("Do duoc:  X = %+.4f   Y = %+.4f   Z = %+.4f  (g)\n",
                  mean_x, mean_y, mean_z);
    Serial.println();

    // Kiểm tra tư thế trước khi tin kết quả
    if (mean_z < 0.80f)
    {
        Serial.println("!! LOI: Z phai gan +1.00 g khi drone nam phang.");
        Serial.println("   Z am  -> drone dang bi lat nguoc.");
        Serial.println("   Z nho -> drone dang dung nghieng.");
        Serial.println("   Dat lai cho dung roi khoi dong lai mach.");
    }
    else if (fabsf(mean_x) > 0.30f || fabsf(mean_y) > 0.30f)
    {
        Serial.println("!! LOI: X hoac Y lech qua nhieu (>0.30 g).");
        Serial.println("   Mat ban khong phang, hoac cam bien gan lech tren mach.");
        Serial.println("   Kiem tra lai roi khoi dong lai mach.");
    }
    else
    {
        Serial.println("Chep DUNG ba dong duoi day, thay vao khoi CAU HINH");
        Serial.println("trong file KyThuatUAV_FC_level_1.ino:");
        Serial.println();
        Serial.printf("#define ACC_OFFSET_X_G   %+.4ff\n", -mean_x);
        Serial.printf("#define ACC_OFFSET_Y_G   %+.4ff\n", -mean_y);
        Serial.printf("#define ACC_OFFSET_Z_G   %+.4ff\n", 1.0f - mean_z);
        Serial.println();
        Serial.println("Xong thi TAT #define CALIBRATE_ACCEL di roi nap lai.");
        Serial.println("Muon kiem tra: bat DEBUG_ACC_OFFSET, phai thay X~0 Y~0 Z~1.");
    }

    Serial.println("=========================================");
    while (1)
    { // dừng hẳn, không cho bay ở chế độ cali
        digitalWrite(PIN_IMU_LED, !digitalRead(PIN_IMU_LED));
        delay(500);
    }
}
#endif

// ===== Hiệu chỉnh gyro: lặp lại tới khi drone thật sự đứng yên =====
// So tổng của đợt này với đợt trước, lệch còn nhỏ mới nhận. Nhờ vậy cắm nguồn
// lúc tay còn cầm drone thì nó tự đo lại chứ không nhận bias sai.
void imu_calibrate_gyro()
{
    const int SAMPLES = 500;
    const float STABLE_LIMIT = 40.0f; // ngưỡng lệch giữa hai đợt liên tiếp

    float sum_roll = 0, sum_pitch = 0, sum_yaw = 0;
    float prev_roll = 0, prev_pitch = 0, prev_yaw = 0;

    Serial.println("bat dau hieu chinh gyro");

    while (1)
    {
        prev_roll = sum_roll;
        prev_pitch = sum_pitch;
        prev_yaw = sum_yaw;
        sum_roll = 0;
        sum_pitch = 0;
        sum_yaw = 0;

        digitalWrite(PIN_IMU_LED, HIGH);
        for (int i = 0; i < SAMPLES; i++)
        {
            imu_read_raw();
            sum_roll += imu_rate_roll_dps;
            sum_pitch += imu_rate_pitch_dps;
            sum_yaw += imu_rate_yaw_dps;
            delayMicroseconds(10);
        }

        Serial.println("Dang hieu chinh gyro - Khong di chuyen Drone!");

        if (fabsf(sum_roll - prev_roll) < STABLE_LIMIT &&
            fabsf(sum_pitch - prev_pitch) < STABLE_LIMIT &&
            fabsf(sum_yaw - prev_yaw) < STABLE_LIMIT)
        {
            Serial.println("cali gyro thanh cong");
            digitalWrite(PIN_IMU_LED, LOW);
            break;
        }
    }

    gyro_bias_roll_dps = sum_roll / SAMPLES;
    gyro_bias_pitch_dps = sum_pitch / SAMPLES;
    gyro_bias_yaw_dps = sum_yaw / SAMPLES;

    Serial.print("bias gyro (do/giay) ");
    Serial.print(gyro_bias_roll_dps);
    Serial.print(" | ");
    Serial.print(gyro_bias_pitch_dps);
    Serial.print(" | ");
    Serial.println(gyro_bias_yaw_dps);
}

// Gọi mỗi vòng điều khiển: đọc cảm biến rồi cập nhật ước lượng góc.
void imu_update()
{
    imu_read_raw();
    imu_kalman_1d(att_roll_deg, att_roll_var,
                  imu_rate_roll_dps - gyro_bias_roll_dps, acc_roll_deg);
    imu_kalman_1d(att_pitch_deg, att_pitch_var,
                  imu_rate_pitch_dps - gyro_bias_pitch_dps, acc_pitch_deg);
}

void imu_read_raw()
{
    int16_t ax_lsb, ay_lsb, az_lsb;
    imu_read_accel_raw(ax_lsb, ay_lsb, az_lsb);

    int16_t gx_lsb, gy_lsb, gz_lsb;
    imu_read_gyro_raw(gx_lsb, gy_lsb, gz_lsb);

    imu_rate_roll_dps = (float)gx_lsb / GYRO_LSB_PER_DPS;
    imu_rate_pitch_dps = (float)gy_lsb / GYRO_LSB_PER_DPS;
    imu_rate_yaw_dps = (float)gz_lsb / GYRO_LSB_PER_DPS;

    imu_acc_x_g = (float)ax_lsb / ACC_LSB_PER_G;
    imu_acc_y_g = (float)ay_lsb / ACC_LSB_PER_G;
    imu_acc_z_g = (float)az_lsb / ACC_LSB_PER_G;

    // Góc từ gia tốc kế, tính trên giá trị đã bù lệch.
    // Roll chủ yếu từ acc_Y và acc_Z, pitch chủ yếu từ acc_X và acc_Z.
    float ax = imu_acc_x_g + ACC_OFFSET_X_G;
    float ay = imu_acc_y_g + ACC_OFFSET_Y_G;
    float az = imu_acc_z_g + ACC_OFFSET_Z_G;

    acc_roll_deg = atanf(ay / sqrtf(ax * ax + az * az)) * RAD_TO_DEG;
    acc_pitch_deg = -atanf(ax / sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
}

// Kalman 1 chiều: gyro là đầu vào dự đoán, góc từ gia tốc kế là phép đo.
// var_process = 1 (độ/giây)^2, var_measure = 3^2 độ^2.
void imu_kalman_1d(float &state_deg, float &variance,
                   float rate_dps, float measure_deg)
{
    state_deg += DT_CTRL * rate_dps;
    variance += DT_CTRL * DT_CTRL * 1.0f * 1.0f;

    float gain = variance / (variance + 3.0f * 3.0f);
    state_deg += gain * (measure_deg - state_deg);
    variance = (1.0f - gain) * variance;
}

// ---- SPI mức thấp -----------------------------------------------------------
void imu_write_reg(uint8_t reg, uint8_t value)
{
    digitalWrite(PIN_IMU_CS, LOW);
    SPI.beginTransaction(imu_spi);
    SPI.transfer(reg & 0x7F); // bit7 = 0 -> ghi
    SPI.transfer(value);
    SPI.endTransaction();
    digitalWrite(PIN_IMU_CS, HIGH);
}

uint8_t imu_read_reg(uint8_t reg)
{
    digitalWrite(PIN_IMU_CS, LOW);
    SPI.beginTransaction(imu_spi);
    SPI.transfer(reg | 0x80); // bit7 = 1 -> đọc
    uint8_t value = SPI.transfer(0x00);
    SPI.endTransaction();
    digitalWrite(PIN_IMU_CS, HIGH);
    return value;
}

void imu_read_accel_raw(int16_t &x, int16_t &y, int16_t &z)
{
    digitalWrite(PIN_IMU_CS, LOW);
    SPI.beginTransaction(imu_spi);
    SPI.transfer(REG_ACCEL_XOUT_H | 0x80);
    x = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
    y = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
    z = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
    SPI.endTransaction();
    digitalWrite(PIN_IMU_CS, HIGH);
}

void imu_read_gyro_raw(int16_t &x, int16_t &y, int16_t &z)
{
    digitalWrite(PIN_IMU_CS, LOW);
    SPI.beginTransaction(imu_spi);
    SPI.transfer(REG_GYRO_XOUT_H | 0x80);
    x = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
    y = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
    z = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
    SPI.endTransaction();
    digitalWrite(PIN_IMU_CS, HIGH);
}

// ---- Các hàm lấy dữ liệu ----------------------------------------------------
float imu_get_acc_x_g() { return imu_acc_x_g + ACC_OFFSET_X_G; }
float imu_get_acc_y_g() { return imu_acc_y_g + ACC_OFFSET_Y_G; }
float imu_get_acc_z_g() { return imu_acc_z_g + ACC_OFFSET_Z_G; }

float imu_get_rate_roll_dps() { return imu_rate_roll_dps - gyro_bias_roll_dps; }
float imu_get_rate_pitch_dps() { return imu_rate_pitch_dps - gyro_bias_pitch_dps; }
float imu_get_rate_yaw_dps() { return imu_rate_yaw_dps - gyro_bias_yaw_dps; }

float imu_get_roll_deg() { return att_roll_deg; }
float imu_get_pitch_deg() { return att_pitch_deg; }
//===================================================================
// pid.ino
// ============================================================================
//  KyThuatUAV FC - firmware bay ESP32 (FreeRTOS), che do Angle + Alt Hold
//
//  Copyright (C) 2026  Nguyễn Văn Quý (Ky Thuat UAV)
//
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program.  If not, see <https://www.gnu.org/licenses/>.
//
//  Giu nguyen phan ghi cong nay khi dung lai hoac chinh sua ma nguon.
//
//  Nguon tham khao va ghi cong day du: xem file LOI_NHAN_VA_GHI_CONG.md
//  Lien he: Nguyen Van Quy - 0817 550 271 (Zalo)
// ============================================================================
// ============================================================================
//  PID xếp tầng
//
//     góc mong muốn --[pid_tilt_*]--> tốc độ góc mong muốn --[pid_rate_*]--> u
//     tốc độ lên xuống mong muốn --[pid_climb]--> ga
//
//  Hệ số KP/KI/KD nằm hết trong khối cấu hình ở file .ino chính.
//  Mỗi bộ điều khiển giữ riêng trạng thái trong một struct PidState, thay vì
//  rải ra thành ba biến toàn cục rời như trước - thêm bộ mới không phải nhớ
//  khai báo đủ ba biến nữa.
//
//  GHI CÔNG: cấu trúc PID xếp tầng (tầng ngoài sinh tốc độ góc mong muốn cho
//  tầng trong bám theo) học từ tài liệu của Carbon Aeronautics
//  (https://github.com/CarbonAeronautics). Chi tiết xem file LOI_NHAN_VA_GHI_CONG.md.
// ============================================================================

#include "types.h" // PidState

PidState pid_rate_roll_st, pid_rate_pitch_st, pid_rate_yaw_st;
PidState pid_tilt_roll_st, pid_tilt_pitch_st;
PidState pid_climb_st;

// Giữ lại đầu ra tầng góc để in debug
static float tilt_roll_out_dps, tilt_pitch_out_dps;

// PID rời rạc, tích phân theo quy tắc hình thang, vi phân theo sai phân lùi.
float pid_step(PidState &st, float error, float kp, float ki, float kd,
               float i_limit, float u_limit)
{
    float p_term = kp * error;

    st.integral += ki * (error + st.prev_error) * DT_CTRL / 2.0f;
    if (st.integral > i_limit)
        st.integral = i_limit;
    else if (st.integral < -i_limit)
        st.integral = -i_limit;

    float d_term = kd * (error - st.prev_error) / DT_CTRL;

    float u = p_term + st.integral + d_term;
    if (u > u_limit)
        u = u_limit;
    else if (u < -u_limit)
        u = -u_limit;

    st.prev_error = error;
    return u;
}

// Riêng trục thẳng đứng: khâu tích phân không cho âm vì nó gánh phần ga treo
// máy, và khâu tỉ lệ bị chặn riêng để một cú nhiễu baro không đẩy ga vọt lên.
float pid_step_climb(PidState &st, float error, float kp, float ki, float kd,
                     float i_limit, float p_limit)
{
    float p_term = kp * error;
    if (p_term > p_limit)
        p_term = p_limit;
    else if (p_term < -p_limit)
        p_term = -p_limit;

    st.integral += ki * (error + st.prev_error) * DT_CTRL / 2.0f;
    if (st.integral > i_limit)
        st.integral = i_limit;
    else if (st.integral < 0)
        st.integral = 0;

    float d_term = kd * (error - st.prev_error) / DT_CTRL;

    st.prev_error = error;
    return p_term + st.integral + d_term;
}

// ---- Tầng trong: tốc độ góc ------------------------------------------------
float pid_rate_roll(float rate_dps, float rate_sp_dps)
{
    return pid_step(pid_rate_roll_st, rate_sp_dps - rate_dps,
                    KP_RATE_ROLL, KI_RATE_ROLL, KD_RATE_ROLL,
                    I_LIM_RATE, U_LIM_RATE);
}

float pid_rate_pitch(float rate_dps, float rate_sp_dps)
{
    return pid_step(pid_rate_pitch_st, rate_sp_dps - rate_dps,
                    KP_RATE_PITCH, KI_RATE_PITCH, KD_RATE_PITCH,
                    I_LIM_RATE, U_LIM_RATE);
}

float pid_rate_yaw(float rate_dps, float rate_sp_dps)
{
    return pid_step(pid_rate_yaw_st, rate_sp_dps - rate_dps,
                    KP_RATE_YAW, KI_RATE_YAW, KD_RATE_YAW,
                    I_LIM_YAW, U_LIM_YAW);
}

// ---- Tầng ngoài: góc nghiêng -----------------------------------------------
float pid_tilt_roll(float angle_deg, float angle_sp_deg)
{
    tilt_roll_out_dps = pid_step(pid_tilt_roll_st, angle_sp_deg - angle_deg,
                                 KP_TILT_ROLL, KI_TILT_ROLL, KD_TILT_ROLL,
                                 I_LIM_TILT, U_LIM_TILT);
    return tilt_roll_out_dps;
}

float pid_tilt_pitch(float angle_deg, float angle_sp_deg)
{
    tilt_pitch_out_dps = pid_step(pid_tilt_pitch_st, angle_sp_deg - angle_deg,
                                  KP_TILT_PITCH, KI_TILT_PITCH, KD_TILT_PITCH,
                                  I_LIM_TILT, U_LIM_TILT);
    return tilt_pitch_out_dps;
}

float pid_tilt_roll_output() { return tilt_roll_out_dps; }
float pid_tilt_pitch_output() { return tilt_pitch_out_dps; }

// ---- Tốc độ lên xuống -------------------------------------------------------
float pid_climb(float climb_cms, float climb_sp_cms_in)
{
    return pid_step_climb(pid_climb_st, climb_sp_cms_in - climb_cms,
                          KP_CLIMB, KI_CLIMB, KD_CLIMB,
                          I_LIM_CLIMB, P_LIM_CLIMB);
}

// Xoá sạch khâu tích phân, dùng khi disarm
void pid_reset_all()
{
    pid_rate_roll_st.integral = 0;
    pid_rate_pitch_st.integral = 0;
    pid_rate_yaw_st.integral = 0;
    pid_tilt_roll_st.integral = 0;
    pid_tilt_pitch_st.integral = 0;
    pid_climb_st.integral = 0;
}

// Xả dần khâu tích phân, dùng khi hạ hết ga mà vẫn đang arm
void pid_bleed_integrators()
{
    pid_rate_roll_st.integral *= 0.9f;
    pid_rate_pitch_st.integral *= 0.9f;
    pid_rate_yaw_st.integral *= 0.9f;
    pid_tilt_roll_st.integral *= 0.9f;
    pid_tilt_pitch_st.integral *= 0.9f;
}
//=================================================================================
// rc_crfs.ino
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

HardwareSerial crsf_uart(2); // UART2

// ---- Hằng số giao thức CRSF ------------------------------------------------
#define CRSF_BAUDRATE 420000
#define CRSF_MAX_FRAME_LEN 64
#define CRSF_ADDR_FC 0xC8 // địa chỉ flight controller
#define CRSF_ADDR_BROADCAST 0xEE
#define CRSF_TYPE_RC_CHANNELS 0x16 // loại frame chứa dữ liệu kênh
#define CRSF_CHANNEL_COUNT 16
#define CRSF_CHANNEL_BITS 11

// Quy đổi giá trị kênh
#define CRSF_RAW_MIN 172
#define CRSF_RAW_MAX 1811
#define CRSF_OUT_MIN 990 // quy đổi ra dải quen thuộc 1000-2000
#define CRSF_OUT_MAX 2010
#define CRSF_TIMEOUT_MS 200 // quá thời gian này = mất sóng

// ---- Biến nội bộ ------------------------------------------------------------
static uint8_t crsf_buf[CRSF_MAX_FRAME_LEN];
static uint8_t crsf_buf_idx = 0;
static uint8_t crsf_frame_len = 0; // len field từ frame hiện tại
static bool crsf_in_frame = false;

static uint16_t crsf_raw_ch[CRSF_CHANNEL_COUNT]; // giá trị thô 172-1811
static int crsf_ch_us[17];                       // quy đổi sang 990-2010, index 1-16
static unsigned long crsf_last_frame_ms = 0;

// ---- CRC-8 DVB-S2 ----------------------------------------------------------
// Bảng tra CRC tính trước, đa thức 0xD5 (chuẩn DVB-S2), dùng cho CRSF.
static const uint8_t crsf_crc8_tab[256] = {
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54,
    0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06,
    0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0,
    0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2,
    0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9,
    0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B,
    0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D,
    0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F,
    0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB,
    0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9,
    0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F,
    0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D,
    0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0xD2, 0x07, 0xAD, 0x78, 0x2C, 0xF9, 0x53, 0x86,
    0xFB, 0x2E, 0x84, 0x51, 0x05, 0xD0, 0x7A, 0xAF,
    0x80, 0x55, 0xFF, 0x2A, 0x7E, 0xAB, 0x01, 0xD4,
    0xA9, 0x7C, 0xD6, 0x03, 0x57, 0x82, 0x28, 0xFD,
    0x76, 0xA3, 0x09, 0xDC, 0x88, 0x5D, 0xF7, 0x22,
    0x5F, 0x8A, 0x20, 0xF5, 0xA1, 0x74, 0xDE, 0x0B,
    0x24, 0xF1, 0x5B, 0x8E, 0xDA, 0x0F, 0xA5, 0x70,
    0x0D, 0xD8, 0x72, 0xA7, 0xF3, 0x26, 0x8C, 0x59};

static uint8_t crsf_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++)
    {
        crc = crsf_crc8_tab[crc ^ data[i]];
    }
    return crc;
}

// ---- Khởi tạo UART cho CRSF ------------------------------------------------
void rc_init()
{
    // CRSF: 420000 baud, 8N1, KHÔNG đảo tín hiệu
    crsf_uart.begin(CRSF_BAUDRATE, SERIAL_8N1, PIN_CRSF_RX, PIN_CRSF_TX);

    // Xoá buffer ban đầu
    while (crsf_uart.available())
        crsf_uart.read();

    crsf_buf_idx = 0;
    crsf_in_frame = false;

    // Khởi tạo kênh về giá trị giữa
    for (int i = 0; i < CRSF_CHANNEL_COUNT; i++)
    {
        crsf_raw_ch[i] = 992; // CRSF mid = 992
    }
    for (int i = 0; i <= 16; i++)
    {
        crsf_ch_us[i] = 1500;
    }
}

// ---- Tách 16 kênh từ payload 22 byte ----------------------------------------
// Cấu trúc giống hệt SBUS: 16 kênh x 11 bit = 176 bit = 22 byte, xếp liên tục.
static void crsf_decode_channels(const uint8_t *payload)
{
    crsf_raw_ch[0] = ((uint16_t)payload[0] | ((uint16_t)payload[1] << 8)) & 0x07FF;
    crsf_raw_ch[1] = ((uint16_t)payload[1] >> 3 | ((uint16_t)payload[2] << 5)) & 0x07FF;
    crsf_raw_ch[2] = ((uint16_t)payload[2] >> 6 | ((uint16_t)payload[3] << 2) | ((uint16_t)payload[4] << 10)) & 0x07FF;
    crsf_raw_ch[3] = ((uint16_t)payload[4] >> 1 | ((uint16_t)payload[5] << 7)) & 0x07FF;
    crsf_raw_ch[4] = ((uint16_t)payload[5] >> 4 | ((uint16_t)payload[6] << 4)) & 0x07FF;
    crsf_raw_ch[5] = ((uint16_t)payload[6] >> 7 | ((uint16_t)payload[7] << 1) | ((uint16_t)payload[8] << 9)) & 0x07FF;
    crsf_raw_ch[6] = ((uint16_t)payload[8] >> 2 | ((uint16_t)payload[9] << 6)) & 0x07FF;
    crsf_raw_ch[7] = ((uint16_t)payload[9] >> 5 | ((uint16_t)payload[10] << 3)) & 0x07FF;
    crsf_raw_ch[8] = ((uint16_t)payload[11] | ((uint16_t)payload[12] << 8)) & 0x07FF;
    crsf_raw_ch[9] = ((uint16_t)payload[12] >> 3 | ((uint16_t)payload[13] << 5)) & 0x07FF;
    crsf_raw_ch[10] = ((uint16_t)payload[13] >> 6 | ((uint16_t)payload[14] << 2) | ((uint16_t)payload[15] << 10)) & 0x07FF;
    crsf_raw_ch[11] = ((uint16_t)payload[15] >> 1 | ((uint16_t)payload[16] << 7)) & 0x07FF;
    crsf_raw_ch[12] = ((uint16_t)payload[16] >> 4 | ((uint16_t)payload[17] << 4)) & 0x07FF;
    crsf_raw_ch[13] = ((uint16_t)payload[17] >> 7 | ((uint16_t)payload[18] << 1) | ((uint16_t)payload[19] << 9)) & 0x07FF;
    crsf_raw_ch[14] = ((uint16_t)payload[19] >> 2 | ((uint16_t)payload[20] << 6)) & 0x07FF;
    crsf_raw_ch[15] = ((uint16_t)payload[20] >> 5 | ((uint16_t)payload[21] << 3)) & 0x07FF;
}

// ---- Đọc và xử lý frame CRSF -----------------------------------------------
// Máy trạng thái gom byte:
//   1. Chờ byte addr (0xC8 hoặc 0xEE)
//   2. Đọc byte len
//   3. Gom đúng len byte (type + payload + crc)
//   4. Kiểm tra CRC
//   5. Nếu type = 0x16 thì giải mã kênh
static bool crsf_process_byte(uint8_t b)
{
    if (!crsf_in_frame)
    {
        // Chờ byte địa chỉ
        if (b == CRSF_ADDR_FC || b == CRSF_ADDR_BROADCAST)
        {
            crsf_buf[0] = b;
            crsf_buf_idx = 1;
            crsf_in_frame = true;
        }
        return false;
    }

    crsf_buf[crsf_buf_idx++] = b;

    // Byte thứ 2: len
    if (crsf_buf_idx == 2)
    {
        crsf_frame_len = b;
        // Kiểm tra len hợp lệ (tối thiểu 2: type + crc, tối đa 62)
        if (crsf_frame_len < 2 || crsf_frame_len > CRSF_MAX_FRAME_LEN - 2)
        {
            crsf_in_frame = false;
            crsf_buf_idx = 0;
        }
        return false;
    }

    // Chưa đủ frame
    if (crsf_buf_idx < (uint8_t)(crsf_frame_len + 2))
    {
        return false;
    }

    // Đủ frame — kiểm tra CRC
    crsf_in_frame = false;

    // CRC tính từ type (byte index 2) đến hết payload (không bao gồm crc)
    uint8_t crc_calc = crsf_crc8(&crsf_buf[2], crsf_frame_len - 1);
    uint8_t crc_recv = crsf_buf[crsf_frame_len + 1];

    if (crc_calc != crc_recv)
    {
        return false;
    }

    // Frame hợp lệ — kiểm tra type
    uint8_t frame_type = crsf_buf[2];
    if (frame_type == CRSF_TYPE_RC_CHANNELS)
    {
        // Payload bắt đầu từ byte index 3 (sau addr, len, type)
        crsf_decode_channels(&crsf_buf[3]);
        return true;
    }

    return false; // frame hợp lệ nhưng không phải RC channels
}

// ---- API công khai (cùng giao diện với bản SBUS cũ) -------------------------

// Trả về true khi vẫn còn sóng, false khi mất sóng.
bool rc_update()
{
    bool got_channels = false;

    while (crsf_uart.available())
    {
        uint8_t b = crsf_uart.read();
        if (crsf_process_byte(b))
        {
            got_channels = true;
        }
    }

    if (got_channels)
    {
        // Quy đổi 16 kênh từ 172-1811 sang 990-2010
        for (int i = 0; i < CRSF_CHANNEL_COUNT; i++)
        {
            crsf_ch_us[i + 1] = map(crsf_raw_ch[i], CRSF_RAW_MIN, CRSF_RAW_MAX,
                                    CRSF_OUT_MIN, CRSF_OUT_MAX);
        }
        crsf_last_frame_ms = millis();
    }

    return (millis() - crsf_last_frame_ms <= CRSF_TIMEOUT_MS);
}

// Lấy giá trị kênh (index 1-16), đã quy đổi sang 990-2010
int rc_get_channel(int index)
{
    if (index < 1 || index > 16)
        return 1500;
    return crsf_ch_us[index];
}
//==========================================================================================
// types.h
// ============================================================================
//  DroneFC_CRSF_DShot - firmware bay ESP32 (FreeRTOS)
//  Dua tren ma nguon cua KyThuatUAV FC (Nguyen Van Quy)
//  Dieu chinh cho bo linh kien:
//    ESP32 WROOM32 38Pin + ICM-20602 + BMP388
//    Radiomaster RP3 ELRS (CRSF) + GOKU G55M 4in1 ESC (DShot600)
//    RS2205 2300KV + Khung 5" + Pin 3S 2200mAh
//
//  Nguon goc: KyThuatUAV FC - Copyright (C) 2026 Nguyen Van Quy
//  Chinh sua: Thich ung linh kien CRSF + DShot
//  License: GPL-3.0
// ============================================================================
#ifndef DRONEFC_TYPES_H
#define DRONEFC_TYPES_H

// Các kiểu dữ liệu dùng chung phải nằm trong header, không để trong file .ino.
// Lý do: Arduino tự sinh prototype cho mọi hàm rồi chèn lên đầu sketch. Nếu
// một hàm có tham số kiểu struct mà struct đó khai báo trong .ino, prototype
// sẽ xuất hiện trước phần khai báo struct và trình biên dịch báo lỗi
// "was not declared in this scope". Đặt trong .h thì #include chạy trước,
// nên kiểu luôn được biết trước prototype.

// Trạng thái nội bộ của một bộ điều khiển PID
struct PidState
{
    float prev_error;
    float integral;
};

// Cảm biến nào có mặt lúc khởi động
struct SensorPresent
{
    bool imu;
    bool baro;
};

// Khung dữ liệu CRSF đã giải mã
struct CrsfChannels
{
    uint16_t ch[16]; // 16 kênh, giá trị 172-1811 (CRSF native)
    bool failsafe;   // true khi receiver báo failsafe
};

#endif
