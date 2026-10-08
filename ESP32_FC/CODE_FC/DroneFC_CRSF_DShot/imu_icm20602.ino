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

#define IMU_SPI_HZ   10000000      // 1 MHz (Hạ tốc độ xuống để chống nhiễu nếu dùng dây cắm dài)

// Địa chỉ thanh ghi
#define REG_WHO_AM_I       0x75
#define REG_PWR_MGMT_1     0x6B
#define REG_ACCEL_XOUT_H   0x3B
#define REG_GYRO_XOUT_H    0x43
#define REG_GYRO_CONFIG    0x1B
#define REG_ACCEL_CONFIG   0x1C
#define REG_CONFIG         0x1A
#define REG_ACCEL_CONFIG2  0x1D

#define ICM20602_WHO_AM_I_VALUE  0x12

// Hệ số quy đổi số thô sang đơn vị vật lý
#define GYRO_LSB_PER_DPS   16.4f     // thang do +-2000 do/giay
#define ACC_LSB_PER_G      2048.0f   // thang do +-16g

static SPISettings imu_spi(IMU_SPI_HZ, MSBFIRST, SPI_MODE0);

// Số đo thô đã quy đổi, CHƯA trừ bias
static float imu_acc_x_g, imu_acc_y_g, imu_acc_z_g;
static float imu_rate_roll_dps, imu_rate_pitch_dps, imu_rate_yaw_dps;
static int16_t imu_acc_x_lsb, imu_acc_y_lsb, imu_acc_z_lsb;

// Bias gyro đo được lúc khởi động
static float gyro_bias_roll_dps, gyro_bias_pitch_dps, gyro_bias_yaw_dps;

// Bias acc đo được lúc khởi động
static float acc_bias_x_g = 0.0f, acc_bias_y_g = 0.0f, acc_bias_z_g = 0.0f;

// Góc suy ra từ riêng gia tốc kế, dùng làm phép đo cho Kalman
static float acc_roll_deg, acc_pitch_deg;

// Kết quả ước lượng góc
static float att_roll_deg  = 0.0f, att_roll_var  = 2 * 2;
static float att_pitch_deg = 0.0f, att_pitch_var = 2 * 2;


// Chỉ mở bus, chưa đụng gì tới cảm biến. Tách riêng để setup() kiểm tra
// WHO_AM_I trước khi bước vào vòng hiệu chỉnh gyro.
void imu_init_bus() {
  SPI.begin();
  pinMode(PIN_IMU_CS, OUTPUT);
  digitalWrite(PIN_IMU_CS, HIGH);
  pinMode(PIN_IMU_LED, OUTPUT);
  digitalWrite(PIN_IMU_LED, LOW);
}

bool imu_is_present() {
  uint8_t whoami = imu_read_reg(REG_WHO_AM_I);
  Serial.printf("Gia tri WHO_AM_I doc duoc: 0x%02X\n", whoami);
  if (whoami == ICM20602_WHO_AM_I_VALUE) {
    return true;
  }

  // Module đang chứng minh được dữ liệu accel hợp lệ nhưng dùng ID không chuẩn.
  // Chỉ cảnh báo để vẫn có thể chạy DEBUG/hiệu chỉnh; không chặn điều khiển.
  Serial.println("CANH BAO: WHO_AM_I khac 0x12, bo qua kiem tra ID.");
  return true;
}


void imu_init() {
  imu_write_reg(REG_PWR_MGMT_1, 0x00);          // thoát chế độ ngủ
  delay(100);
  imu_write_reg(REG_ACCEL_CONFIG,  0x18);       // thang đo gia tốc +-16g
  imu_write_reg(REG_GYRO_CONFIG,   0x18);       // thang đo gyro +-2000 độ/giây
  imu_write_reg(REG_CONFIG,        IMU_DLPF_GYRO);
  imu_write_reg(REG_ACCEL_CONFIG2, IMU_DLPF_ACC);
  delay(100);

  Serial.println("[ICM20602 khoi tao xong]");
  Serial.print("CONFIG 0x");        Serial.println(imu_read_reg(REG_CONFIG), HEX);
  Serial.print("ACCEL_CONFIG2 0x"); Serial.println(imu_read_reg(REG_ACCEL_CONFIG2), HEX);

  imu_calibrate_gyro();
  imu_calibrate_accel();
}


// ===== Hiệu chỉnh gia tốc kế =====
// Tự động chạy lúc khởi động để lấy bias.
// Lặp lại tới khi drone đứng yên và nằm phẳng.
void imu_calibrate_accel() {
  const int   SAMPLES        = 500;
  const float STABLE_LIMIT   = 5.0f;   // ngưỡng lệch cho gia tốc kế

  float sum_x = 0, sum_y = 0, sum_z = 0;
  float prev_x = 0, prev_y = 0, prev_z = 0;

  Serial.println("bat dau hieu chinh acc");

  while (1) {
    prev_x = sum_x;  prev_y = sum_y;  prev_z = sum_z;
    sum_x = 0;  sum_y = 0;  sum_z = 0;

    digitalWrite(PIN_IMU_LED, HIGH);
    for (int i = 0; i < SAMPLES; i++) {
      imu_read_raw();
      sum_x += imu_acc_x_g;            // lấy số THÔ
      sum_y += imu_acc_y_g;
      sum_z += imu_acc_z_g;
      delay(2);
    }

    Serial.println("Dang hieu chinh acc - Khong di chuyen Drone!");

    if (fabsf(sum_x - prev_x) < STABLE_LIMIT &&
        fabsf(sum_y - prev_y) < STABLE_LIMIT &&
        fabsf(sum_z - prev_z) < STABLE_LIMIT) {
      Serial.println("cali acc thanh cong");
      digitalWrite(PIN_IMU_LED, LOW);
      break;
    }
  }

  float mean_x = sum_x / SAMPLES;
  float mean_y = sum_y / SAMPLES;
  float mean_z = sum_z / SAMPLES;

  if (mean_z < 0.80f || fabsf(mean_x) > 0.30f || fabsf(mean_y) > 0.30f) {
    Serial.println("!! LOI: Drone khong nam phang, acc cali co the sai!");
  }

  acc_bias_x_g = -mean_x;
  acc_bias_y_g = -mean_y;
  acc_bias_z_g = 1.0f - mean_z;

  Serial.print("bias acc (g) ");
  Serial.print(acc_bias_x_g, 4); Serial.print(" | ");
  Serial.print(acc_bias_y_g, 4); Serial.print(" | ");
  Serial.println(acc_bias_z_g, 4);
}


// ===== Hiệu chỉnh gyro: lặp lại tới khi drone thật sự đứng yên =====
// So tổng của đợt này với đợt trước, lệch còn nhỏ mới nhận. Nhờ vậy cắm nguồn
// lúc tay còn cầm drone thì nó tự đo lại chứ không nhận bias sai.
void imu_calibrate_gyro() {
  const int   SAMPLES        = 500;
  const float STABLE_LIMIT   = 40.0f;   // ngưỡng lệch giữa hai đợt liên tiếp

  float sum_roll = 0, sum_pitch = 0, sum_yaw = 0;
  float prev_roll = 0, prev_pitch = 0, prev_yaw = 0;

  Serial.println("bat dau hieu chinh gyro");

  while (1) {
    prev_roll = sum_roll;  prev_pitch = sum_pitch;  prev_yaw = sum_yaw;
    sum_roll = 0;  sum_pitch = 0;  sum_yaw = 0;

    digitalWrite(PIN_IMU_LED, HIGH);
    for (int i = 0; i < SAMPLES; i++) {
      imu_read_raw();
      sum_roll  += imu_rate_roll_dps;
      sum_pitch += imu_rate_pitch_dps;
      sum_yaw   += imu_rate_yaw_dps;
      delayMicroseconds(10);
    }

    Serial.println("Dang hieu chinh gyro - Khong di chuyen Drone!");

    if (fabsf(sum_roll  - prev_roll)  < STABLE_LIMIT &&
        fabsf(sum_pitch - prev_pitch) < STABLE_LIMIT &&
        fabsf(sum_yaw   - prev_yaw)   < STABLE_LIMIT) {
      Serial.println("cali gyro thanh cong");
      digitalWrite(PIN_IMU_LED, LOW);
      break;
    }
  }

  gyro_bias_roll_dps  = sum_roll  / SAMPLES;
  gyro_bias_pitch_dps = sum_pitch / SAMPLES;
  gyro_bias_yaw_dps   = sum_yaw   / SAMPLES;

  Serial.print("bias gyro (do/giay) ");
  Serial.print(gyro_bias_roll_dps);  Serial.print(" | ");
  Serial.print(gyro_bias_pitch_dps); Serial.print(" | ");
  Serial.println(gyro_bias_yaw_dps);
}


// Gọi mỗi vòng điều khiển: đọc cảm biến rồi cập nhật ước lượng góc.
void imu_update() {
  imu_read_raw();
  imu_kalman_1d(att_roll_deg,  att_roll_var,
                imu_rate_roll_dps  - gyro_bias_roll_dps,  acc_roll_deg);
  imu_kalman_1d(att_pitch_deg, att_pitch_var,
                imu_rate_pitch_dps - gyro_bias_pitch_dps, acc_pitch_deg);
}


void imu_read_raw() {
  int16_t ax_lsb, ay_lsb, az_lsb;
  imu_read_accel_raw(ax_lsb, ay_lsb, az_lsb);
  imu_acc_x_lsb = ax_lsb;
  imu_acc_y_lsb = ay_lsb;
  imu_acc_z_lsb = az_lsb;

  int16_t gx_lsb, gy_lsb, gz_lsb;
  imu_read_gyro_raw(gx_lsb, gy_lsb, gz_lsb);

  imu_rate_roll_dps  = (float)gx_lsb / GYRO_LSB_PER_DPS;
  imu_rate_pitch_dps = (float)gy_lsb / GYRO_LSB_PER_DPS;
  imu_rate_yaw_dps   = (float)gz_lsb / GYRO_LSB_PER_DPS;

  imu_acc_x_g = (float)ax_lsb / ACC_LSB_PER_G;
  imu_acc_y_g = (float)ay_lsb / ACC_LSB_PER_G;
  imu_acc_z_g = (float)az_lsb / ACC_LSB_PER_G;

  // Góc từ gia tốc kế, tính trên giá trị đã bù lệch.
  // Roll chủ yếu từ acc_Y và acc_Z, pitch chủ yếu từ acc_X và acc_Z.
  float ax = imu_acc_x_g + acc_bias_x_g;
  float ay = imu_acc_y_g + acc_bias_y_g;
  float az = imu_acc_z_g + acc_bias_z_g;

  acc_roll_deg  =  atanf(ay / sqrtf(ax * ax + az * az)) * RAD_TO_DEG;
  acc_pitch_deg = -atanf(ax / sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
}


// Kalman 1 chiều: gyro là đầu vào dự đoán, góc từ gia tốc kế là phép đo.
// var_process = 1 (độ/giây)^2, var_measure = 3^2 độ^2.
void imu_kalman_1d(float &state_deg, float &variance,
                   float rate_dps, float measure_deg) {
  state_deg += DT_CTRL * rate_dps;
  variance  += DT_CTRL * DT_CTRL * 1.0f * 1.0f;

  float gain = variance / (variance + 3.0f * 3.0f);
  state_deg += gain * (measure_deg - state_deg);
  variance   = (1.0f - gain) * variance;
}


// ---- SPI mức thấp -----------------------------------------------------------
void imu_write_reg(uint8_t reg, uint8_t value) {
  SPI.beginTransaction(imu_spi);
  digitalWrite(PIN_IMU_CS, LOW);
  SPI.transfer(reg & 0x7F);          // bit7 = 0 -> ghi
  SPI.transfer(value);
  digitalWrite(PIN_IMU_CS, HIGH);
  SPI.endTransaction();
}

uint8_t imu_read_reg(uint8_t reg) {
  SPI.beginTransaction(imu_spi);
  digitalWrite(PIN_IMU_CS, LOW);
  SPI.transfer(reg | 0x80);          // bit7 = 1 -> đọc
  uint8_t value = SPI.transfer(0x00);
  digitalWrite(PIN_IMU_CS, HIGH);
  SPI.endTransaction();
  return value;
}

void imu_read_accel_raw(int16_t &x, int16_t &y, int16_t &z) {
  SPI.beginTransaction(imu_spi);
  digitalWrite(PIN_IMU_CS, LOW);
  SPI.transfer(REG_ACCEL_XOUT_H | 0x80);
  uint8_t xh = SPI.transfer(0x00); uint8_t xl = SPI.transfer(0x00);
  uint8_t yh = SPI.transfer(0x00); uint8_t yl = SPI.transfer(0x00);
  uint8_t zh = SPI.transfer(0x00); uint8_t zl = SPI.transfer(0x00);
  digitalWrite(PIN_IMU_CS, HIGH);
  SPI.endTransaction();
  x = (int16_t)((xh << 8) | xl);
  y = (int16_t)((yh << 8) | yl);
  z = (int16_t)((zh << 8) | zl);
}

void imu_read_gyro_raw(int16_t &x, int16_t &y, int16_t &z) {
  SPI.beginTransaction(imu_spi);
  digitalWrite(PIN_IMU_CS, LOW);
  SPI.transfer(REG_GYRO_XOUT_H | 0x80);
  uint8_t xh = SPI.transfer(0x00); uint8_t xl = SPI.transfer(0x00);
  uint8_t yh = SPI.transfer(0x00); uint8_t yl = SPI.transfer(0x00);
  uint8_t zh = SPI.transfer(0x00); uint8_t zl = SPI.transfer(0x00);
  digitalWrite(PIN_IMU_CS, HIGH);
  SPI.endTransaction();
  x = (int16_t)((xh << 8) | xl);
  y = (int16_t)((yh << 8) | yl);
  z = (int16_t)((zh << 8) | zl);
}

// ---- Các hàm lấy dữ liệu ----------------------------------------------------
float imu_get_acc_x_g() { return imu_acc_x_g + acc_bias_x_g; }
float imu_get_acc_y_g() { return imu_acc_y_g + acc_bias_y_g; }
float imu_get_acc_z_g() { return imu_acc_z_g + acc_bias_z_g; }
int16_t imu_get_acc_x_lsb() { return imu_acc_x_lsb; }
int16_t imu_get_acc_y_lsb() { return imu_acc_y_lsb; }
int16_t imu_get_acc_z_lsb() { return imu_acc_z_lsb; }

float imu_get_rate_roll_dps()  { return imu_rate_roll_dps  - gyro_bias_roll_dps;  }
float imu_get_rate_pitch_dps() { return imu_rate_pitch_dps - gyro_bias_pitch_dps; }
float imu_get_rate_yaw_dps()   { return imu_rate_yaw_dps   - gyro_bias_yaw_dps;   }

float imu_get_roll_deg()  { return att_roll_deg;  }
float imu_get_pitch_deg() { return att_pitch_deg; }
