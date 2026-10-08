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
#define PIN_ESC_1        27   // Motor 1 — sau phải   (RR, CW)
#define PIN_ESC_2        26   // Motor 2 — trước phải (FR, CCW)
#define PIN_ESC_3        25   // Motor 3 — sau trái   (RL, CCW)
#define PIN_ESC_4        33   // Motor 4 — trước trái (FL, CW)

// ---- Chân IMU (SPI) --------------------------------------------------------
#define PIN_IMU_CS        5   // ICM-20602 chip select
#define PIN_IMU_LED      14   // LED báo trạng thái hiệu chỉnh gyro
// SPI mặc định: MOSI=23, MISO=19, SCK=18 (không cần define, SPI.begin() tự dùng)

// ---- Chân CRSF RX (Radiomaster RP3 ELRS) -----------------------------------
// RP3 xuất tín hiệu CRSF qua 1 chân TX. Nối chân TX của RP3 vào chân RX
// của UART2 trên ESP32. CRSF là giao thức KHÔNG ĐẢO, 420000 baud, 8N1.
#define PIN_CRSF_RX      16   // UART2 RX — nối tới TX của RP3
#define PIN_CRSF_TX      17   // UART2 TX — nối tới RX của RP3 (cho telemetry)

// ---- Chân I2C (BMP388) -----------------------------------------------------
#define PIN_I2C_SDA      21
#define PIN_I2C_SCL      22
#define BARO_I2C_ADDR  0x76   // BMP388, nối SDO xuống GND = 0x76, lên VCC = 0x77

// ---- Kênh tay điều khiển (mapping EdgeTX mặc định) -------------------------
// EdgeTX trên Pocket Crush gán: CH1=Aileron, CH2=Elevator, CH3=Throttle, CH4=Rudder
// Mảng CRSF bắt đầu từ 0, nhưng ta quy ước index 1-based cho đồng nhất với code gốc
#define CH_ROLL           1   // Aileron
#define CH_PITCH          2   // Elevator
#define CH_THROTTLE       3   // Throttle
#define CH_YAW            4   // Rudder
#define CH_ARM            5   // Công tắc arm (gán trong EdgeTX, ví dụ SB)
#define CH_MODE           6   // Công tắc 3 nấc chọn chế độ bay (gán SC)

// ---- Chế độ bay và cách gán cho từng nấc công tắc --------------------------
#define MODE_ANGLE        0
#define MODE_ALT_HOLD     1
#define MODE_FAILSAFE     2   // dùng nội bộ, không gán cho công tắc

#define MODE_FOR_SW_0     MODE_ANGLE
#define MODE_FOR_SW_1     MODE_ANGLE
#define MODE_FOR_SW_2     MODE_ALT_HOLD

// ---- Giới hạn lệnh từ cần gạt ----------------------------------------------
#define LIM_TILT_DEG           20.0f   // độ nghiêng tối đa ở chế độ angle
#define LIM_YAW_RATE_DPS      100.0f   // tốc độ xoay tối đa, độ/giây
#define LIM_CLIMB_RATE_CMS    100.0f   // tốc độ lên xuống tối đa, cm/giây
#define FAILSAFE_DESCENT_CMS  -60.0f   // tốc độ hạ khi mất sóng, cm/giây

// ---- Giới hạn DShot (0-2047) ------------------------------------------------
// DShot giá trị 0 = disarmed, 48 = idle khi armed, 2047 = full throttle
// Quy đổi nội bộ: code vẫn dùng thang 800-1600 như cũ, hàm esc_write() tự
// chuyển sang DShot 0-2047 trước khi gửi.
#define ESC_IDLE        800   // giá trị nội bộ khi chưa arm (sẽ map thành DShot 0)
#define ESC_MIN_ARMED   900   // đã arm thì không cho tụt dưới mức này
#define ESC_MAX        1600

// ---- Hệ số PID --------------------------------------------------------------
// RS2205 2300KV trên khung 5" với pin 3S gần tương đương cấu hình gốc (2204 2300KV),
// nên giữ nguyên bộ PID mặc định. Nếu rung thì giảm KP_RATE và KD_RATE.
const float KP_RATE_ROLL  = 0.5f,  KI_RATE_ROLL  = 1.4f,  KD_RATE_ROLL  = 0.03f;
const float KP_RATE_PITCH = 0.5f,  KI_RATE_PITCH = 1.4f,  KD_RATE_PITCH = 0.03f;
const float KP_RATE_YAW   = 1.0f,  KI_RATE_YAW   = 10.0f, KD_RATE_YAW   = 0.0f;

const float KP_TILT_ROLL  = 10.0f, KI_TILT_ROLL  = 0.0f,  KD_TILT_ROLL  = 0.0f;
const float KP_TILT_PITCH = 10.0f, KI_TILT_PITCH = 0.0f,  KD_TILT_PITCH = 0.0f;

const float KP_CLIMB      = 3.0f,  KI_CLIMB      = 15.0f, KD_CLIMB      = 0.0f;

// Chặn khâu tích phân và chặn đầu ra của từng tầng
const float I_LIM_RATE   = 200.0f,  U_LIM_RATE   = 400.0f;
const float I_LIM_TILT   = 200.0f,  U_LIM_TILT   = 400.0f;
const float I_LIM_YAW    = 100.0f,  U_LIM_YAW    = 100.0f;
const float I_LIM_CLIMB  = 700.0f,  P_LIM_CLIMB  = 200.0f;

// ---- Bù lệch gia tốc kế (đơn vị g) -----------------------------------------
// Đã chuyển sang tự động hiệu chỉnh lúc khởi động (xem imu_calibrate_accel)

// ---- Bộ lọc thông thấp bên trong ICM20602 ----------------------------------
#define IMU_DLPF_GYRO    0x06
#define IMU_DLPF_ACC     0x05

// ---- Debug ------------------------------------------------------------------
// Mỗi lần chỉ bật MỘT dòng.
 #define DEBUG_ATTITUDE      // góc hiện tại so với góc mong muốn
// #define DEBUG_RATE          // tốc độ góc mong muốn so với thực tế
// #define DEBUG_ALTITUDE      // độ cao và tốc độ lên xuống sau KF
// #define DEBUG_RC            // giá trị các kênh tay điều khiển
// #define DEBUG_ACC_OFFSET    // để đo ACC_OFFSET_* bên trên
// #define DEBUG_LOOP_TIME     // chu kì vòng điều khiển, phải luôn ~2000 us
// #define CALIBRATE_ACCEL
// ============================================================================
//                        HẾT PHẦN CẤU HÌNH
// ============================================================================
#define PERIOD_CTRL_MS    2       // 500 Hz
#define PERIOD_RC_MS      2       // 500 Hz
#define PERIOD_BARO_MS   10       // 100 Hz
#define DT_CTRL      0.002f       // chu kì vòng điều khiển, tính bằng giây

// Trạng thái cảm biến lúc khởi động
SensorPresent sensor_present;

// Mutex cho từng nhóm dữ liệu dùng chung
SemaphoreHandle_t mtx_rc;
SemaphoreHandle_t mtx_baro;
SemaphoreHandle_t mtx_tlm;

// rc_task ghi, control_task đọc
int  shared_rc_ch[17];
bool shared_rc_ok = false;

// baro_task ghi, control_task đọc
float shared_baro_alt_m = 0.0f;
bool  shared_baro_new   = false;

// control_task ghi, baro_task đọc để in debug
float tlm_roll_deg, tlm_pitch_deg;
float tlm_roll_sp_deg, tlm_pitch_sp_deg;
float tlm_rate_roll_dps, tlm_rate_pitch_dps, tlm_rate_yaw_dps;
float tlm_rate_roll_sp_dps, tlm_rate_pitch_sp_dps;
float tlm_alt_cm, tlm_climb_cms, tlm_climb_sp_cms;
float tlm_acc_x_g, tlm_acc_y_g, tlm_acc_z_g;
int   tlm_rc_ch[17];
int   tlm_esc[5];
int   tlm_flight_mode;
bool  tlm_armed, tlm_rc_ok;
uint32_t tlm_loop_us;


void setup() {
  Serial.begin(500000);
  // Phải đưa CS lên HIGH ngay: ICM-20602 không hỗ trợ khởi động khi CS/SCK thấp.
  imu_init_bus();
  delay(1000); // Cho các cảm biến có thêm thời gian khởi động

  // Kiểm tra IMU TRƯỚC khi khởi tạo các ngoại vi khác (RMT, UART)
  sensor_present.imu = imu_is_present();
  Serial.printf("ICM20602 = %d\n", sensor_present.imu);
  if (!sensor_present.imu) {
    while (1) {
      Serial.println("KHONG TIM THAY ICM20602 - dung lai, khong cho bay");
      digitalWrite(PIN_IMU_LED, !digitalRead(PIN_IMU_LED));
      delay(300);
    }
  }

  esc_init();
  esc_write(ESC_IDLE, ESC_IDLE, ESC_IDLE, ESC_IDLE);

  rc_init();

  imu_init();

  sensor_present.baro = baro_init();
  Serial.printf("BMP388   = %d\n", sensor_present.baro);

  alt_kf_init();

  Serial.println("DroneFC CRSF+DShot - angle + alt hold");
  Serial.println("Linh kien: ESP32 WROOM32 + ICM20602 + BMP388");
  Serial.println("           RP3 ELRS (CRSF) + GOKU G55M (DShot600)");
  Serial.println("           RS2205 2300KV + 5\" frame + 3S 2200mAh");

  mtx_rc   = xSemaphoreCreateMutex();
  mtx_baro = xSemaphoreCreateMutex();
  mtx_tlm  = xSemaphoreCreateMutex();

  if (mtx_rc == NULL || mtx_baro == NULL || mtx_tlm == NULL) {
    Serial.println("Loi: khong tao duoc Mutex!");
    while (1) delay(1000);
  }

  xTaskCreatePinnedToCore(rc_task,      "RC",   4096, NULL, 4, NULL, 0);
  xTaskCreatePinnedToCore(baro_task,    "BARO", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(control_task, "CTRL", 8192, NULL, 5, NULL, 1);

  vTaskDelete(NULL);
}

void loop() {
  // Không dùng, mọi thứ chạy trong các task
}


// ============================================================================
//  rc_task  —  core 0, 500Hz — giải mã CRSF từ Radiomaster RP3
// ============================================================================
void rc_task(void *parameter) {
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    bool ok = rc_update();

    if (xSemaphoreTake(mtx_rc, 0) == pdTRUE) {
      shared_rc_ok = ok;
      for (int i = 1; i <= 16; i++) shared_rc_ch[i] = rc_get_channel(i);
      xSemaphoreGive(mtx_rc);
    }
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(PERIOD_RC_MS));
  }
}


// ============================================================================
//  baro_task  —  core 0, 100Hz. Kiêm luôn việc in debug.
// ============================================================================
void baro_task(void *parameter) {
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    if (sensor_present.baro) {
      if (baro_update()) {
        float alt_m = baro_get_altitude_m();
        if (xSemaphoreTake(mtx_baro, 0) == pdTRUE) {
          shared_baro_alt_m = alt_m;
          shared_baro_new   = true;
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
void debug_print() {
#if defined(DEBUG_ATTITUDE) || defined(DEBUG_RATE) || defined(DEBUG_ALTITUDE) || \
    defined(DEBUG_RC) || defined(DEBUG_ACC_OFFSET) || defined(DEBUG_LOOP_TIME)

  static uint32_t tick = 0;
  if (++tick % 100) return;  // 1 dòng/giây để dễ đọc trên Serial Monitor

  if (xSemaphoreTake(mtx_tlm, 0) != pdTRUE) return;

  #ifdef DEBUG_ATTITUDE
    Serial.printf("roll_sp:%.1f roll:%.1f pitch_sp:%.1f pitch:%.1f | acc_g: X:%.3f Y:%.3f Z:%.3f \r\n",
                  tlm_roll_sp_deg, tlm_roll_deg, tlm_pitch_sp_deg, tlm_pitch_deg,
                  tlm_acc_x_g, tlm_acc_y_g, tlm_acc_z_g);
  #endif

  #ifdef DEBUG_RATE
    Serial.printf("%.0f,%.0f,%.0f,%.0f\n",
                  tlm_rate_roll_sp_dps,  tlm_rate_roll_dps,
                  tlm_rate_pitch_sp_dps, tlm_rate_pitch_dps);
  #endif

  #ifdef DEBUG_ALTITUDE
    Serial.printf("%.1f,%.1f,%.1f\n", tlm_alt_cm, tlm_climb_cms, tlm_climb_sp_cms);
  #endif

  #ifdef DEBUG_RC
    Serial.printf("ch1:%d ch2:%d ch3:%d ch4:%d arm:%d mode:%d | rc_ok:%d armed:%d fm:%d | uart:%lu frame:%lu crc_err:%lu\n",
                  tlm_rc_ch[CH_ROLL], tlm_rc_ch[CH_PITCH],
                  tlm_rc_ch[CH_THROTTLE], tlm_rc_ch[CH_YAW],
                  tlm_rc_ch[CH_ARM], tlm_rc_ch[CH_MODE],
                  tlm_rc_ok, tlm_armed, tlm_flight_mode,
                  (unsigned long)rc_get_rx_byte_count(),
                  (unsigned long)rc_get_frame_count(),
                  (unsigned long)rc_get_crc_error_count());
  #endif

  #ifdef DEBUG_ACC_OFFSET
    Serial.printf("ACC raw: X:%d Y:%d Z:%d | g: X:%.4f Y:%.4f Z:%.4f\n",
                  imu_get_acc_x_lsb(), imu_get_acc_y_lsb(), imu_get_acc_z_lsb(),
                  tlm_acc_x_g, tlm_acc_y_g, tlm_acc_z_g);
  #endif

  #ifdef DEBUG_LOOP_TIME
    Serial.printf("loop_us:%u\n", tlm_loop_us);
  #endif

  xSemaphoreGive(mtx_tlm);
#endif
}
