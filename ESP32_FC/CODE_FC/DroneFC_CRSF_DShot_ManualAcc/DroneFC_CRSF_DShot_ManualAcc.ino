// ============================================================================
//  DroneFC_CRSF_DShot_ManualAcc — firmware bay ESP32 (FreeRTOS)
//  Chế độ Angle + Alt Hold — BẢN HIỆU CHỈNH GIA TỐC KẾ (ACCEL) THỦ CÔNG
//
//  Dựa trên:
//    - DroneFC_CRSF_DShot (RP3 ELRS CRSF + GOKU G55M DShot600 + ESP32 38Pin)
//    - Cơ chế hiệu chỉnh gia tốc kế thủ công từ KyThuatUAV_FC_level_1
//      (Copyright (C) 2026 Nguyễn Văn Quý)
//
//  Mục đích bản này:
//    - Thay thế tự động cali acc mỗi khi bật nguồn bằng cơ chế hiệu chỉnh THỦ CÔNG:
//      Đo và lưu cố định các thông số bù lệch ACC_OFFSET_X_G, ACC_OFFSET_Y_G,
//      ACC_OFFSET_Z_G vào mã nguồn.
//    - Khắc phục nhược điểm của cali tự động: khi cắm pin trên địa hình dốc, gập
//      ghềnh hoặc cầm tay, drone không bị nhận sai mặt phẳng cân bằng.
//    - Khi khởi động, drone chỉ cali gyro (chỉ cần đứng yên, không bắt buộc phải phẳng).
//
//  Bộ linh kiện sử dụng:
//    - ESP32 WROOM32 38Pin
//    - ICM-20602 (SPI)
//    - BMP388 (I2C)
//    - Radiomaster RP3 ExpressLRS 2.4GHz (giao thức CRSF qua UART)
//    - GOKU G55M 4-in-1 ESC BLHeli32 (DShot600)
//    - Motor RS2205 2300KV, khung 5", cánh 5", pin 3S 2200mAh
//    - Tay cầm Pocket Crush (ELRS 2.4GHz, EdgeTX)
//
//  License: GPL-3.0
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
#include <esp_timer.h>

// ============================================================================
//                      CẤU HÌNH  —  SỬA TRỰC TIẾP TẠI ĐÂY
// ============================================================================

// ---- Chân kết nối ESC (DShot600) -------------------------------------------
// GOKU G55M 4-in-1 ESC: 4 tín hiệu motor vào 4 chân GPIO.
// Chọn chân có khả năng RMT (Remote Control Transceiver) của ESP32 để xuất
// DShot. ESP32 có 8 kênh RMT, ta dùng 4.
#define PIN_ESC_1 27 // Motor 1 — sau phải   (RR, CW)
#define PIN_ESC_2 26 // Motor 2 — trước phải (FR, CCW)
#define PIN_ESC_3 25 // Motor 3 — sau trái   (RL, CCW)
#define PIN_ESC_4 33 // Motor 4 — trước trái (FL, CW)

// ---- Chân IMU (SPI) --------------------------------------------------------
#define PIN_IMU_CS 5   // ICM-20602 chip select
#define PIN_IMU_LED 14 // LED báo trạng thái hiệu chỉnh gyro / cali acc
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

// ---- Chân MTF-01 (UART1) ---------------------------------------------------
// Micoair MTF-01: module ToF + Optical Flow.
// Cấu hình module dùng MicoLink, UART 115200 baud, 8N1; chỉ dùng telemetry.
#define PIN_MTF_RX 4  // UART1 RX — nối tới TX của MTF-01
#define PIN_MTF_TX 13 // UART1 TX — dự phòng (không dùng hiện tại)

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
#define LIM_CLIMB_RATE_CMS 30.0f    // tốc độ lên xuống tối đa, cm/giây
#define FAILSAFE_DESCENT_CMS -20.0f // tốc độ hạ khi mất sóng, cm/giây

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

// ---- Bù lệch gia tốc kế (đơn vị g) — HIỆU CHỈNH THỦ CÔNG -------------------
// Nguyên lý (tương tự KyThuatUAV_FC_level_1):
//   Cách 1 (Khuyên dùng):
//     - Bật dòng "#define CALIBRATE_ACCEL" trong phần Debug bên dưới.
//     - Nạp code vào ESP32, mở Serial Monitor (500000 baud).
//     - Đặt drone nằm thật phẳng trên mặt bàn tĩnh, không chạm vào.
//     - Sau khi đếm ngược 5 giây, hệ thống đo 2000 mẫu và in ra 3 dòng bên dưới.
//     - Copy đúng 3 dòng kết quả dán đè vào vị trí này.
//     - Tắt (comment lại) "#define CALIBRATE_ACCEL", nạp lại code là xong.
//   Cách 2:
//     - Bật "#define DEBUG_ACC_OFFSET", đặt phẳng, ghi nhận X, Y, Z.
//     - Điền thủ công: X = -giá_trị_X, Y = -giá_trị_Y, Z = 1.0 - giá_trị_Z.
//  Hiện tượng trôi khi … | Nguyên nhân góc ước lượng | Cần can thiệp thô… | Hành động điều ch… | Ví dụ chỉnh
//   -----------------------|---------------------------|--------------------|--------------------|---------------------
//    Trôi về PHÍA TRƯỚC    | FC tưởng đang ngửa sau    | ACC_OFFSET_X_G     | GIẢM (trừ bớt đi)  | Từ +0.0152 →
//    (Forward)             | (ax quá dương)            |                    |                    | +0.0052
//    Trôi về PHÍA SAU      | FC tưởng đang chúi trước  | ACC_OFFSET_X_G     | TĂNG (cộng thêm    | Từ +0.0152 →
//    (Backward)            | (ax quá âm)               |                    | vào)               | +0.0252
//    Trôi sang TRÁI (Left) | FC tưởng đang nghiêng     | ACC_OFFSET_Y_G     | GIẢM (trừ bớt đi)  | Từ +0.0141 →
//                          | phải (ay quá dương)       |                    |                    | +0.0041
//    Trôi sang PHẢI        | FC tưởng đang nghiêng     | ACC_OFFSET_Y_G     | TĂNG (cộng thêm    | Từ +0.0141 →
//    (Right)               | trái (ay quá âm)          |                    | vào)               | +0.0241

#define ACC_OFFSET_X_G +0.030745f
#define ACC_OFFSET_Y_G -0.00625f
#define ACC_OFFSET_Z_G +0.03507375f

// ---- Bộ lọc thông thấp bên trong ICM20602 ----------------------------------
#define IMU_DLPF_GYRO 0x06
#define IMU_DLPF_ACC 0x05

// ---- CẤU HÌNH & HIỆU CHỈNH CẢM BIẾN MTF-01 (ToF Laser + Optical Flow) -------
// 1. Cánh tay đòn (Lever Arm Vector) từ tâm IMU tới mắt MTF-01 trên khung máy bay:
//    - Đơn vị: Mét (m). Hệ trục thân máy: X (Tiến +), Y (Phải +), Z (Xuống +).
//    - Mục đích: Bù trừ vận tốc ảo do drone xoay/nghiêng khi cảm biến không nằm ở trọng tâm.
//    - Nếu lắp gần tâm drone: để 0.0f.
#define MTF_LEVER_X_M +0.06666f // 66.66 mm (đổi thành dấu - nếu nằm sau tâm)
#define MTF_LEVER_Y_M +0.00175f // 1.75 mm (đổi thành dấu - nếu lệch sang trái)
#define MTF_LEVER_Z_M +0.02173f // 21.73 mm (dưới tâm IMU nên luôn là +)

// 2. Điểm lệch tĩnh (Bias) của camera quang lưu (Optical Flow) khi drone đứng yên:
//    - Đơn vị: Giá trị số thô (Raw LSB).
//    - Mục đích: Khử hiện tượng trôi số cảm biến quang lưu khi đặt máy tĩnh trên bàn.
#define MTF_FLOW_BIAS_X_RAW 0.0f // Độ lệch điểm 0 trục X quang lưu (Raw LSB)
#define MTF_FLOW_BIAS_Y_RAW 0.0f // Độ lệch điểm 0 trục Y quang lưu (Raw LSB)

// 3. Sai số bù độ cao ToF (Offset) và Khoảng cách chuẩn đo bằng thước (Reference):
//    - Đơn vị: Milimét (mm).
//    - MTF_REFERENCE_MM: Khoảng cách thực tế từ cảm biến xuống sàn đo bằng thước dây (dùng khi cali bench).
//      + Đặt bằng 0: Khi chạy CALIBRATE_MTF sẽ chỉ đo bias Optical Flow, không tính sai số ToF.
//      + Đặt > 0 (ví dụ 500): Kê drone cao đúng 500mm, firmware sẽ tự tính offset ToF = 500 - ToF_đo_được.
//    - MTF_TOF_OFFSET_MM: Sai số bù độ cao (Khoảng cách chuẩn xác = Khoảng cách ToF đo được + Offset).
#define MTF_REFERENCE_MM 0     // Khoảng cách thực tế đo bằng thước (mm) dùng khi CALIBRATE_MTF (0 = tắt đo ToF)
#define MTF_TOF_OFFSET_MM 0.0f // Sai số bù khoảng cách ToF (mm): Khoảng cách đúng = Đo được + Offset
// Lưu ý: Các thông số trên hiện để phục vụ hiệu chuẩn/ghi nhận telemetry, chưa đưa vào vòng điều khiển kín.

// ---- Debug ------------------------------------------------------------------
// Mỗi lần chỉ bật MỘT dòng.
// #define DEBUG_ATTITUDE      // góc hiện tại so với góc mong muốn
// #define DEBUG_RATE          // tốc độ góc mong muốn so với thực tế
// #define DEBUG_ALTITUDE      // độ cao và tốc độ lên xuống sau KF
// #define DEBUG_MTF           // khoảng cách và optical flow MTF-01
// #define DEBUG_RC            // giá trị các kênh tay điều khiển
// #define DEBUG_ACC_OFFSET    // để kiểm tra kết quả bù lệch (drone phẳng: X~0, Y~0, Z~1)
// #define DEBUG_LOOP_TIME     // chu kì vòng điều khiển, phải luôn ~2000 us
// #define CALIBRATE_ACCEL     // BẬT ĐỂ CHẠY HIỆU CHỈNH ACC THỦ CÔNG (dừng bay, in 3 dòng offset)
// #define CALIBRATE_MTF       // Do bias tren ban; khong khoi tao ESC hoac task bay
#if defined(CALIBRATE_MTF) && defined(CALIBRATE_ACCEL)
#error "Chi bat mot che do hieu chuan"
#endif
#if !defined(DEBUG_ATTITUDE) && !defined(DEBUG_RATE) && !defined(DEBUG_ALTITUDE) && !defined(DEBUG_MTF) && \
    !defined(DEBUG_RC) && !defined(DEBUG_ACC_OFFSET) && !defined(DEBUG_LOOP_TIME) && !defined(CALIBRATE_ACCEL)
#define DEBUG_ACC_OFFSET
#endif
// ============================================================================
//                        HẾT PHẦN CẤU HÌNH
// ============================================================================
#define PERIOD_CTRL_MS 2  // 500 Hz
#define PERIOD_RC_MS 2    // 500 Hz
#define PERIOD_BARO_MS 10 // 100 Hz
#define BARO_STALE_US 200000UL
#define DT_CTRL 0.002f // chu kì vòng điều khiển, tính bằng giây

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
BaroSample shared_baro_sample = {};
BaroTiming shared_baro_timing = {};
bool shared_baro_new = false;

// baro_task ghi (MTF-01), control_task đọc
SemaphoreHandle_t mtx_mtf;
MtfData shared_mtf = {};
uint32_t shared_mtf_sample_ms = 0;

// control_task ghi, baro_task và rc_task đọc
float tlm_roll_deg, tlm_pitch_deg;
float tlm_roll_sp_deg, tlm_pitch_sp_deg;
float tlm_rate_roll_dps, tlm_rate_pitch_dps, tlm_rate_yaw_dps;
float tlm_rate_roll_sp_dps, tlm_rate_pitch_sp_dps;
float tlm_alt_cm, tlm_climb_cms, tlm_climb_sp_cms;
float tlm_baro_alt_cm;
float tlm_acc_x_g, tlm_acc_y_g, tlm_acc_z_g;
int tlm_rc_ch[17];
int tlm_esc[5];
int tlm_flight_mode;
bool tlm_armed, tlm_rc_ok;
uint32_t tlm_loop_us;
uint64_t tlm_sample_time_us;
BaroSample tlm_baro_sample = {};
float tlm_acc_earth_z_g, tlm_acc_bias_cms2, tlm_acc_corrected_cms2;
uint32_t tlm_baro_age_us = UINT32_MAX;
bool tlm_baro_healthy, tlm_baro_correction_used;
MtfData tlm_mtf = {};
bool tlm_mtf_valid = false;
uint32_t tlm_mtf_sample_ms = 0;

void setup()
{
  Serial.begin(500000);
#ifdef CALIBRATE_MTF
  void mtf_calibrate();
  mtf_init();
  mtf_calibrate();
  while (true)
    delay(1000);
#endif
  // Phải đưa CS lên HIGH ngay: ICM-20602 không hỗ trợ khởi động khi CS/SCK thấp.
  imu_init_bus();
  delay(1000); // Cho các cảm biến có thêm thời gian khởi động

  // Kiểm tra IMU TRƯỚC khi khởi tạo các ngoại vi khác (RMT, UART)
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

  esc_init();
  esc_write(ESC_IDLE, ESC_IDLE, ESC_IDLE, ESC_IDLE);

  rc_init();

  imu_init();

  sensor_present.baro = baro_init();
  Serial.printf("BMP388   = %d\n", sensor_present.baro);

  mtf_init();

  alt_kf_init();

  Serial.println("DroneFC CRSF+DShot (Manual Acc Calib) - angle + alt hold");
  Serial.println("Linh kien: ESP32 WROOM32 + ICM20602 + BMP388");
  Serial.println("           RP3 ELRS (CRSF) + GOKU G55M (DShot600)");
  Serial.println("           RS2205 2300KV + 5\" frame + 3S 2200mAh");

  mtx_rc = xSemaphoreCreateMutex();
  mtx_baro = xSemaphoreCreateMutex();
  mtx_mtf = xSemaphoreCreateMutex();
  mtx_tlm = xSemaphoreCreateMutex();

  if (mtx_rc == NULL || mtx_baro == NULL || mtx_mtf == NULL || mtx_tlm == NULL)
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
    rc_telemetry_update();
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(PERIOD_RC_MS));
  }
}

// ============================================================================
//  baro_task  —  core 0, 100Hz. Đọc BMP388 + MTF-01 và in debug.
// ============================================================================
void mtf_publish_sample()
{
  mtf_update();
  if (xSemaphoreTake(mtx_mtf, 0) == pdTRUE)
  {
    shared_mtf.distance_m = mtf_get_distance_m();
    shared_mtf.distance_cm = mtf_get_distance_cm();
    shared_mtf.strength = mtf_get_distance_strength();
    shared_mtf.precision = mtf_get_precision();
    shared_mtf.tof_status = mtf_get_tof_status();
    shared_mtf.flow_status = mtf_get_flow_status();
    shared_mtf.sensor_time_ms = mtf_get_sample_ms();
    shared_mtf.frame_count = mtf_get_frame_count();
    shared_mtf.error_count = mtf_get_error_count();
    shared_mtf.flow_x_raw = mtf_get_flow_x_raw();
    shared_mtf.flow_y_raw = mtf_get_flow_y_raw();
    shared_mtf.flow_quality = mtf_get_flow_quality();
    shared_mtf.distance_valid = mtf_is_distance_valid();
    shared_mtf_sample_ms = mtf_get_receive_ms();
    xSemaphoreGive(mtx_mtf);
  }
}

void baro_task(void *parameter)
{
  TickType_t wake = xTaskGetTickCount();
  uint32_t previous_start_us = 0;
  BaroTiming timing = {};
  for (;;)
  {
    uint32_t started_us = micros();
    timing.period_us = previous_start_us ? started_us - previous_start_us : 0;
    previous_start_us = started_us;

    if (sensor_present.baro && baro_update())
    {
      BaroSample sample = baro_get_sample();
      if (xSemaphoreTake(mtx_baro, 0) == pdTRUE)
      {
        shared_baro_sample = sample;
        shared_baro_new = true;
        xSemaphoreGive(mtx_baro);
      }
    }
    mtf_publish_sample();
    debug_print();
    timing.prev_work_us = micros() - started_us;
    if (timing.prev_work_us > PERIOD_BARO_MS * 1000UL)
      timing.work_overruns++;
    if (xSemaphoreTake(mtx_baro, 0) == pdTRUE)
    {
      shared_baro_timing = timing;
      xSemaphoreGive(mtx_baro);
    }
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(PERIOD_BARO_MS));
  }
}

// ============================================================================
//  In debug
// ============================================================================
void debug_print()
{
#if defined(DEBUG_ATTITUDE) || defined(DEBUG_RATE) || defined(DEBUG_ALTITUDE) || \
    defined(DEBUG_MTF) || defined(DEBUG_RC) || defined(DEBUG_ACC_OFFSET) || defined(DEBUG_LOOP_TIME)

  static uint32_t tick = 0;
  if (++tick % 20)
    return; // In ở tần số ~5Hz để quan sát ổn định trên Serial Monitor

  if (xSemaphoreTake(mtx_tlm, 0) != pdTRUE)
    return;

#ifdef DEBUG_ATTITUDE
  Serial.printf("roll_sp:%.1f roll:%.1f pitch_sp:%.1f pitch:%.1f | acc_g: X:%.3f Y:%.3f Z:%.3f \r\n",
                tlm_roll_sp_deg, tlm_roll_deg, tlm_pitch_sp_deg, tlm_pitch_deg,
                tlm_acc_x_g, tlm_acc_y_g, tlm_acc_z_g);
#endif

#ifdef DEBUG_RATE
  Serial.printf("%.0f,%.0f,%.0f,%.0f\n",
                tlm_rate_roll_sp_dps, tlm_rate_roll_dps,
                tlm_rate_pitch_sp_dps, tlm_rate_pitch_dps);
#endif

#ifdef DEBUG_ALTITUDE
  Serial.printf("[ALT] Baro:%s | Raw: %6.1f cm (%5.2f m) | KF: %6.1f cm (%5.2f m) | Vz: %5.1f cm/s | SP: %5.1f cm/s | Mode:%s\r\n",
                sensor_present.baro ? "OK" : "DISCONN",
                tlm_baro_alt_cm, tlm_baro_alt_cm / 100.0f,
                tlm_alt_cm, tlm_alt_cm / 100.0f,
                tlm_climb_cms,
                tlm_climb_sp_cms,
                (tlm_flight_mode == MODE_ALT_HOLD) ? "ALT_HOLD" : (tlm_flight_mode == MODE_FAILSAFE) ? "FAILSAFE"
                                                                                                     : "ANGLE");
#endif

#ifdef DEBUG_MTF
  Serial.printf("[MTF] cm:%.1f range_fresh:%d age_ms:%lu strength:%u precision:%u tof_status:%u flow_x_raw:%d flow_y_raw:%d quality:%u flow_status:%u frames:%lu errors:%lu \r\n",
                tlm_mtf.distance_cm, tlm_mtf_valid,
                (unsigned long)(millis() - tlm_mtf_sample_ms), tlm_mtf.strength,
                tlm_mtf.precision, tlm_mtf.tof_status, tlm_mtf.flow_x_raw,
                tlm_mtf.flow_y_raw, tlm_mtf.flow_quality, tlm_mtf.flow_status,
                (unsigned long)tlm_mtf.frame_count, (unsigned long)tlm_mtf.error_count);
#endif

#ifdef DEBUG_RC
  Serial.printf("ch1:%d ch2:%d ch3:%d ch4:%d arm:%d mode:%d | rc_ok:%d armed:%d fm:%d | uart:%lu frame:%lu crc_err:%lu \r\n",
                tlm_rc_ch[CH_ROLL], tlm_rc_ch[CH_PITCH],
                tlm_rc_ch[CH_THROTTLE], tlm_rc_ch[CH_YAW],
                tlm_rc_ch[CH_ARM], tlm_rc_ch[CH_MODE],
                tlm_rc_ok, tlm_armed, tlm_flight_mode,
                (unsigned long)rc_get_rx_byte_count(),
                (unsigned long)rc_get_frame_count(),
                (unsigned long)rc_get_crc_error_count());
#endif

#ifdef DEBUG_ACC_OFFSET
  static float ax = 0, ay = 0, az = 1;
  ax = ax * 0.98f + tlm_acc_x_g * 0.02f;
  ay = ay * 0.98f + tlm_acc_y_g * 0.02f;
  az = az * 0.98f + tlm_acc_z_g * 0.02f;
  Serial.printf("ACC raw LSB: X:%6d Y:%6d Z:%6d | Filtered g: X:%+.4f Y:%+.4f Z:%+.4f \r\n",
                imu_get_acc_x_lsb(), imu_get_acc_y_lsb(), imu_get_acc_z_lsb(),
                ax, ay, az);
#endif

#ifdef DEBUG_LOOP_TIME
  Serial.printf("loop_us:%u\n", tlm_loop_us);
#endif

  xSemaphoreGive(mtx_tlm);
#endif
}
