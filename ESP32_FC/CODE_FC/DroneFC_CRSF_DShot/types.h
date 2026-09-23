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
struct PidState {
  float prev_error;
  float integral;
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

#endif
