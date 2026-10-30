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
//  Kalman 3 trạng thái ước lượng độ cao, tốc độ thẳng đứng và bias gia tốc.
//
//     trạng thái   S = [ z (cm),  vz (cm/s),  b (cm/s^2) ]'
//                        |         |            |
//                        |         |            +-- bias của acc_earth_z,
//                        |         |                tự ước lượng và cập nhật
//                        |         |                liên tục trong khi bay
//                        |         +-- tốc độ thẳng đứng
//                        +-- độ cao tương đối
//
//     đầu vào      u = acc_earth_z_g (cm/s^2), từ IMU (CHƯA trừ bias)
//     phép đo      M = độ cao từ baro (cm)
//
//  Trạng thái b ước lượng bias gia tốc; chất lượng hội tụ phụ thuộc
//  nhiễu đo, hiệu chuẩn IMU và điều kiện chuyển động.
//
//  ------------------------------------------------------------------------
//  MODEL RỜIRAC (DT = 2ms):
//
//    z(k+1)  = z(k) + DT·vz(k) + 0.5·DT²·(u(k) - b(k))
//    vz(k+1) = vz(k) + DT·(u(k) - b(k))
//    b(k+1)  = b(k)        ← bias thay đổi rất chậm, chỉ có process noise
//
//  Ma trận hệ thống:
//
//    F = [ 1,  DT,  -0.5·DT² ]      G = [ 0.5·DT² ]
//        [ 0,   1,  -DT      ]          [ DT      ]
//        [ 0,   0,   1       ]          [ 0       ]
//
//    H = [ 1, 0, 0 ]       (chỉ đo z)
//
//  Process noise Q:
//    - z, vz: nhiễu từ acc uncertainty:  G·G'·σ_acc²
//    - b:     nhiễu drift bias:          σ_b²  (nhỏ — bias đổi chậm)
//
//  ------------------------------------------------------------------------
//  Predict 500 Hz; correct chỉ khi có mẫu baro mới để không đếm
//  lặp một phép đo thành nhiều phép đo độc lập.
//  ------------------------------------------------------------------------
// ============================================================================

#include <BasicLinearAlgebra.h>
using namespace BLA;

// sigma của nhiễu gia tốc (cm/s^2) và của phép đo baro (cm).
//
// KF_SIGMA_ACC_CMS2: tăng lên 20 vì correct chỉ chạy khi baro CÓ MẪU MỚI
// (~50 Hz), không còn chạy 500 lần/giây với mẫu cũ. P cần mọc đủ nhanh giữa
// các lần correct để Kalman gain K đủ lực kéo vz. (Quy tắc ngón tay: khi
// giảm tần số correct 10×, tăng sigma_acc ~√10 ≈ 3×, đây là 5× để an toàn.)
#define KF_SIGMA_ACC_CMS2   20.0f
#define KF_SIGMA_BARO_CM    30.0f

// sigma của nhiễu drift bias (cm/s^2 per step).
// ICM20602 drift ~0.7 mg/°C × 10°C heating / 60s ≈ 0.12 mg/s
// → per 2ms step: 0.12e-3 × 981 × 0.002 ≈ 0.00024 cm/s^2
// Đặt σ_b = 0.05 cm/s^2 (rộng hơn thực tế ~200×) để KF đủ nhanh
// bám drift nhiệt độ mà không quá nhạy với nhiễu acc.
#define KF_SIGMA_BIAS_CMS2   0.05f

// ---- Ma trận KF 3×3 / 3×1 / 1×3 ------------------------------------------
BLA::Matrix<3,3> Fz;      // ma trận chuyển trạng thái
BLA::Matrix<3,1> Gz;      // ma trận đầu vào (chỉ u, không tính b)
BLA::Matrix<3,3> Pz;      // hiệp phương sai sai số ước lượng
BLA::Matrix<3,3> Qz;      // hiệp phương sai nhiễu quá trình
BLA::Matrix<3,1> Sz;      // vector trạng thái [z, vz, b]'
BLA::Matrix<1,3> Hz;      // ma trận quan sát
BLA::Matrix<3,3> Iz;      // ma trận đơn vị
BLA::Matrix<1,1> Acczz;   // đầu vào: gia tốc thẳng đứng (cm/s^2)
BLA::Matrix<3,1> Kz;      // độ lợi Kalman [3×1]
BLA::Matrix<1,1> Rz;      // hiệp phương sai nhiễu đo
BLA::Matrix<1,1> Lz;      // hiệp phương sai phần dư
BLA::Matrix<1,1> Mz;      // phép đo

static float kf_alt_cm    = 0.0f;
static float kf_climb_cms = 0.0f;
static float kf_bias_cms2 = 0.0f;   // bias ước lượng được (cm/s^2)


void alt_kf_init() {
  const float dt  = DT_CTRL;
  const float dt2 = dt * dt;
  const float dt3 = dt2 * dt;
  const float dt4 = dt3 * dt;
  const float sa2 = KF_SIGMA_ACC_CMS2  * KF_SIGMA_ACC_CMS2;
  const float sb2 = KF_SIGMA_BIAS_CMS2 * KF_SIGMA_BIAS_CMS2;

  // F = [ 1,  dt,  -0.5*dt² ]
  //     [ 0,   1,  -dt      ]
  //     [ 0,   0,   1       ]
  Fz = { 1.0f,  dt,   -0.5f * dt2,
         0.0f,  1.0f, -dt,
         0.0f,  0.0f,  1.0f };

  // G = [ 0.5*dt² ]   (chỉ u đi vào, b là trạng thái nội)
  //     [ dt      ]
  //     [ 0       ]
  Gz = { 0.5f * dt2,
         dt,
         0.0f };

  // H = [ 1, 0, 0 ]
  Hz = { 1.0f, 0.0f, 0.0f };

  // I (3×3)
  Iz = { 1.0f, 0.0f, 0.0f,
         0.0f, 1.0f, 0.0f,
         0.0f, 0.0f, 1.0f };

  // Q = G·G'·σ_acc² + diag(0, 0, σ_b²)
  //   [ 0.25·dt⁴·sa²,  0.5·dt³·sa²,  0   ]
  //   [ 0.5·dt³·sa²,   dt²·sa²,      0   ]
  //   [ 0,             0,            sb² ]
  Qz = { 0.25f * dt4 * sa2,   0.5f * dt3 * sa2,   0.0f,
         0.5f  * dt3 * sa2,   dt2  * sa2,          0.0f,
         0.0f,                0.0f,                sb2  };

  // R = σ_baro²
  Rz = { KF_SIGMA_BARO_CM * KF_SIGMA_BARO_CM };

  // Khởi tạo P và S về 0
  Pz = { 0.0f, 0.0f, 0.0f,
         0.0f, 0.0f, 0.0f,
         0.0f, 0.0f, 0.0f };
  Sz = { 0.0f,
         0.0f,
         0.0f };

  kf_alt_cm    = 0.0f;
  kf_climb_cms = 0.0f;
  kf_bias_cms2 = 0.0f;
}


// Gọi MỖI vòng điều khiển (500 Hz).
//   alt_meas_m     : độ cao baro, đơn vị MÉT (hàm tự đổi sang cm)
//   acc_earth_z_g  : gia tốc thẳng đứng đã trừ trọng lực, đơn vị g
//                    KHÔNG cần trừ bias trước — KF tự ước lượng b
//   baro_is_new    : true chỉ khi baro vừa có mẫu MỚI (~50 Hz).
//                    Correct chỉ chạy khi baro_is_new = true, tránh kích động
//                    vz bởi stale measurement được dùng lặp 10 lần/baro-step.
bool alt_kf_update(float alt_meas_m, float acc_earth_z_g, bool baro_is_new) {
  Acczz = { acc_earth_z_g * 981.0f };
  Sz = Fz * Sz + Gz * Acczz;
  Pz = Fz * Pz * ~Fz + Qz;

  bool correction_used = false;
  float measurement_cm = alt_meas_m * 100.0f;
  if (baro_is_new && isfinite(measurement_cm)) {
    Lz = Hz * Pz * ~Hz + Rz;
    float variance = Lz(0, 0);
    if (isfinite(variance) && variance >= 1e-6f) {
      Kz = Pz * ~Hz * (1.0f / variance);
      Mz = { measurement_cm };
      Sz = Sz + Kz * (Mz - Hz * Sz);
      Pz = (Iz - Kz * Hz) * Pz;
      correction_used = true;
    }
  }

  kf_alt_cm    = Sz(0, 0);
  kf_climb_cms = Sz(1, 0);
  kf_bias_cms2 = Sz(2, 0);
  return correction_used;
}


float alt_kf_get_altitude_cm()  { return kf_alt_cm;    }
float alt_kf_get_climb_cms()    { return kf_climb_cms; }
float alt_kf_get_acc_bias_cms2(){ return kf_bias_cms2; } // debug/telemetry
