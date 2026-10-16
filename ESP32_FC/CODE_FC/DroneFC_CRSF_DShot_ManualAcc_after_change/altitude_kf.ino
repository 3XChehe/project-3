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
//  Kalman 2 trạng thái ước lượng độ cao và tốc độ thẳng đứng.
//
//     trạng thái   S = [ z (cm) , vz (cm/s) ]'
//     đầu vào      u = gia tốc thẳng đứng (cm/s^2), từ IMU
//     phép đo      M = độ cao từ baro (cm)
//
//  Giữ Q/R và cadence 2ms; mẫu baro lặp không phải phép đo độc lập.
// ============================================================================

#include <BasicLinearAlgebra.h>
using namespace BLA;

// sigma của nhiễu gia tốc (cm/s^2) và của phép đo baro (cm)
#define KF_SIGMA_ACC_CMS2    4.0f
#define KF_SIGMA_BARO_CM    30.0f

BLA::Matrix<2,2> Fz;      // ma trận chuyển trạng thái
BLA::Matrix<2,1> Gz;      // ma trận đầu vào
BLA::Matrix<2,2> Pz;      // hiệp phương sai sai số ước lượng
BLA::Matrix<2,2> Qz;      // hiệp phương sai nhiễu quá trình
BLA::Matrix<2,1> Sz;      // vector trạng thái
BLA::Matrix<1,2> Hz;      // ma trận quan sát
BLA::Matrix<2,2> Iz;      // ma trận đơn vị
BLA::Matrix<1,1> Acczz;   // đầu vào: gia tốc thẳng đứng
BLA::Matrix<2,1> Kz;      // độ lợi Kalman
BLA::Matrix<1,1> Rz;      // hiệp phương sai nhiễu đo
BLA::Matrix<1,1> Lz;      // hiệp phương sai phần dư
BLA::Matrix<1,1> Mz;      // phép đo

static float kf_alt_cm   = 0.0f;
static float kf_climb_cms = 0.0f;


void alt_kf_init() {
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
bool alt_kf_update(float alt_meas_m, float acc_earth_z_g) {
  if (!isfinite(alt_meas_m) || !isfinite(acc_earth_z_g)) return false;
  float acceleration = acc_earth_z_g * 981.0f;
  float altitude_cm = alt_meas_m * 100.0f;
  if (!isfinite(acceleration) || !isfinite(altitude_cm)) return false;
  BLA::Matrix<1,1> next_acc = {acceleration};
  BLA::Matrix<1,1> next_measurement = {altitude_cm};
  BLA::Matrix<2,1> next_state = Fz * Sz + Gz * next_acc;
  BLA::Matrix<2,2> next_covariance = Fz * Pz * ~Fz + Qz;
  BLA::Matrix<1,1> next_residual = Hz * next_covariance * ~Hz + Rz;

  // Độ lợi Kalman: K = P·H'·L⁻¹
  // Bản virex v14 viết Invert(Lz), nhưng đó là cú pháp của BasicLinearAlgebra
  // 3.x. Từ bản 4.x, Invert() đảo ma trận TẠI CHỖ và trả về bool, nên viết như
  // cũ sẽ không biên dịch được. Lz là ma trận 1x1 nên nghịch đảo của nó chính
  // là nghịch đảo số học - viết thẳng vừa đúng, vừa không phụ thuộc phiên bản
  // thư viện, lại nhanh hơn.
  float Lz_scalar = next_residual(0, 0);
  if (!isfinite(Lz_scalar) || Lz_scalar < 1e-6f) return false;
  BLA::Matrix<2,1> next_gain = next_covariance * ~Hz * (1.0f / Lz_scalar);
  next_state = next_state + next_gain * (next_measurement - Hz * next_state);
  next_covariance = (Iz - next_gain * Hz) * next_covariance;
  for (int row = 0; row < 2; ++row) {
    if (!isfinite(next_state(row, 0)) || !isfinite(next_gain(row, 0))) return false;
    for (int col = 0; col < 2; ++col) {
      if (!isfinite(next_covariance(row, col))) return false;
    }
  }

  Acczz = next_acc;
  Mz = next_measurement;
  Lz = next_residual;
  Kz = next_gain;
  Sz = next_state;
  Pz = next_covariance;
  kf_alt_cm = Sz(0, 0);
  kf_climb_cms = Sz(1, 0);
  return true;
}

bool alt_kf_reset_to_measurement(float altitude_m) {
  float altitude_cm = altitude_m * 100.0f;
  if (!isfinite(altitude_cm)) return false;
  alt_kf_init();
  Sz(0, 0) = altitude_cm;
  kf_alt_cm = altitude_cm;
  return true;
}


float alt_kf_get_altitude_cm() { return kf_alt_cm;    }
float alt_kf_get_climb_cms()   { return kf_climb_cms; }
