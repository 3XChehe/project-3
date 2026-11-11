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

#include <stdint.h>

// Các kiểu dữ liệu dùng chung phải nằm trong header, không để trong file .ino.
// Lý do: Arduino tự sinh prototype cho mọi hàm rồi chèn lên đầu sketch. Nếu
// một hàm có tham số kiểu struct mà struct đó khai báo trong .ino, prototype
// sẽ xuất hiện trước phần khai báo struct và trình biên dịch báo lỗi
// "was not declared in this scope". Đặt trong .h thì #include chạy trước,
// nên kiểu luôn được biết trước prototype.

// Trạng thái nội bộ của một bộ điều khiển PID
struct PidState {
  float prev_error;
  float integral;
  float integral_before;
  float pd_term;
  float output;
  bool initialized;
};

enum AltPhase { ALT_WAIT_TAKEOFF, ALT_TAKEOFF, ALT_FLIGHT };
enum CollectiveSource { COLLECTIVE_DIRECT, COLLECTIVE_RAMP, COLLECTIVE_CLIMB_PI };
enum VerticalReason {
  VERTICAL_ACQUIRING, VERTICAL_OK, VERTICAL_STALE, VERTICAL_KF, VERTICAL_PREARM
};

struct VerticalReadiness {
  bool input_seen, accepted, new_sample, healthy, ready;
  bool window_active, window_static, trend_valid;
  uint32_t input_ms, accepted_ms, window_ms, ready_ms, valid_ms;
  float altitude_m, window_alt_cm, min_cm, max_cm, acc_sum, trend_cms;
  uint32_t acc_count, sample_count;
  int reason;
};

// Cảm biến nào có mặt lúc khởi động
struct SensorPresent {
  bool imu;
  bool baro;
};

// Khung dữ liệu CRSF đã giải mã
struct CrsfChannels {
  uint16_t ch[16];    // 16 kênh, giá trị 172-1811 (CRSF native)
  bool     failsafe;  // true khi receiver báo failsafe
};

// Dữ liệu từ module MTF-01 (ToF + Optical Flow)
struct MtfData {
    float distance_m;         // khoảng cách ToF (mét)
    float distance_cm;        // khoảng cách ToF (cm)
    uint8_t strength, precision, tof_status;
    int16_t flow_x_raw;       // Gia tri tho; don vi va chieu chua xac minh
    int16_t flow_y_raw;
    uint8_t flow_quality, flow_status;
    uint32_t sensor_time_ms, frame_count, error_count;
    bool distance_valid;      // chỉ kiểm tra tuổi mẫu và phạm vi
};

#endif