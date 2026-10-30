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
float acc_x_g,  acc_y_g,  acc_z_g;          // gia tốc thân máy, đơn vị g
float rate_roll_dps, rate_pitch_dps, rate_yaw_dps;   // tốc độ góc, độ/giây
float roll_deg, pitch_deg;                  // góc nghiêng đã lọc
float acc_earth_z_g;                        // gia tốc thẳng đứng đã trừ trọng lực

// ---- Giá trị mong muốn ------------------------------------------------------
float roll_sp_deg, pitch_sp_deg;
float yaw_rate_sp_dps;
float climb_sp_cms;

// ---- Đầu ra bộ điều khiển ---------------------------------------------------
// u = tín hiệu điều khiển, ký hiệu chuẩn trong lý thuyết điều khiển
float u_roll, u_pitch, u_yaw;
int   u_throttle;

// ---- Đầu ra cuối cùng cho 4 ESC ---------------------------------------------
int esc_1, esc_2, esc_3, esc_4;
int esc_lim_lo = ESC_IDLE, esc_lim_hi = ESC_IDLE;

// ---- Trạng thái bay ---------------------------------------------------------
int  rc_ch[17];
bool rc_ok, rc_ok_prev;
bool armed = false;
int  flight_mode = MODE_ANGLE;

// Khởi tạo bằng 2 chứ không phải -1: nếu cắm nguồn lúc công tắc arm ĐANG bật,
// điều kiện "vừa vào nấc 2" sẽ không thoả, nên drone không tự arm ngay khi
// khởi động. Muốn arm phải gạt công tắc xuống rồi gạt lên lại.
int  arm_sw_prev = 2;

BaroSample baro_sample = {};
float baro_alt_m;
bool baro_is_new;
bool baro_healthy;
bool baro_correction_used;
uint32_t baro_age_us = UINT32_MAX;

unsigned long failsafe_start_ms;
uint32_t loop_period_us;


void control_task(void *parameter) {
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    static uint32_t t_prev_us = 0;
    uint32_t t_now_us = micros();
    loop_period_us = t_now_us - t_prev_us;
    t_prev_us = t_now_us;

    // ---------- 1. Cảm biến và ước lượng trạng thái ----------
    uint64_t sample_time_us = esp_timer_get_time();
    imu_update();
    acc_x_g        = imu_get_acc_x_g();
    acc_y_g        = imu_get_acc_y_g();
    acc_z_g        = imu_get_acc_z_g();
    rate_roll_dps  = imu_get_rate_roll_dps();
    rate_pitch_dps = imu_get_rate_pitch_dps();
    rate_yaw_dps   = imu_get_rate_yaw_dps();
    roll_deg       = imu_get_roll_deg();
    pitch_deg      = imu_get_pitch_deg();

    // Gia tốc theo phương thẳng đứng của mặt đất, trừ đi 1g.
    float sr = sinf(roll_deg  * DEG_TO_RAD), cr = cosf(roll_deg  * DEG_TO_RAD);
    float sp = sinf(pitch_deg * DEG_TO_RAD), cp = cosf(pitch_deg * DEG_TO_RAD);
    acc_earth_z_g = -sp * acc_x_g + cp * sr * acc_y_g + cp * cr * acc_z_g - 1.0f;
    // Bias acc_earth_z duoc KF 3-state (altitude_kf.ino) tu uoc luong — khong can tru o day

    // KF độ cao
    baro_fetch_shared();
    baro_correction_used = alt_kf_update(baro_alt_m, acc_earth_z_g,
                                         baro_is_new && baro_healthy);

    // ---------- 2. Lệnh từ tay điều khiển ----------
    rc_ok_prev = rc_ok;
    rc_fetch_and_map();

    // ---------- 3. Arm / disarm ----------
    int previous_mode = flight_mode;
    bool was_armed = armed;
    if (rc_ok) {
      if (rc_ch[CH_ARM] != 2) {
        armed = false;
      } else if (arm_sw_prev != 2 && rc_ch[CH_THROTTLE] < 1150) {
        armed = true;
      }
      arm_sw_prev = rc_ch[CH_ARM];

      if      (rc_ch[CH_MODE] == 0) flight_mode = MODE_FOR_SW_0;
      else if (rc_ch[CH_MODE] == 1) flight_mode = MODE_FOR_SW_1;
      else                          flight_mode = MODE_FOR_SW_2;
      if (flight_mode == MODE_ALT_HOLD &&
          (!baro_healthy || !isfinite(alt_kf_get_altitude_cm()) ||
           !isfinite(alt_kf_get_climb_cms()) || !isfinite(alt_kf_get_acc_bias_cms2()))) {
        flight_mode = MODE_ANGLE;
      }

    } else if (armed && sensor_present.baro) {
      if (rc_ok_prev) failsafe_start_ms = millis();
      flight_mode = MODE_FAILSAFE;
    } else {
      armed = false;
    }

    if (armed) { esc_lim_lo = ESC_MIN_ARMED; esc_lim_hi = ESC_MAX; }
    else       { esc_lim_lo = ESC_IDLE;      esc_lim_hi = ESC_IDLE; pid_reset_all(); }

    // Hạ xong thì tự disarm
    if (!rc_ok && armed) {
      if (u_throttle < ESC_MIN_ARMED + 1 &&
          fabsf(alt_kf_get_climb_cms()) < 5.0f &&
          (millis() - failsafe_start_ms) > 2000) {
        armed = false;
      }
    }

    if (armed && was_armed && previous_mode == MODE_ANGLE &&
        (flight_mode == MODE_ALT_HOLD || flight_mode == MODE_FAILSAFE)) {
      float setpoint = flight_mode == MODE_FAILSAFE ? FAILSAFE_DESCENT_CMS : climb_sp_cms;
      pid_climb_seed(u_throttle, alt_kf_get_climb_cms(), setpoint);
    }

    // ---------- 4. Bộ điều khiển ----------
    switch (flight_mode) {
      case MODE_ANGLE:
        if (u_throttle < 830) pid_bleed_integrators();
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

    // ---------- 5. Mixer quad X — chuẩn Betaflight / GOKU G55M ---------------------
    // ESC GOKU G55M pad layout (nhìn từ trên, mũi hướng lên):
    //   M4(FL,CW)  ●───────● M2(FR,CCW)
    //              │       │
    //   M3(RL,CCW) ●───────● M1(RR,CW)
    //
    // Roll+  = nghiêng phải → tăng motor trái (M3,M4), giảm motor phải (M1,M2)
    // Pitch+ = chúi mũi    → tăng motor trước (M2,M4), giảm motor sau (M1,M3)
    // Yaw+   = xoay phải   → tăng CW (M1,M4), giảm CCW (M2,M3)
    esc_1 = u_throttle + u_yaw - u_roll + u_pitch;   // M1 rear-right  CW
    esc_2 = u_throttle - u_yaw - u_roll - u_pitch;   // M2 front-right CCW
    esc_3 = u_throttle - u_yaw + u_roll + u_pitch;   // M3 rear-left   CCW
    esc_4 = u_throttle + u_yaw + u_roll - u_pitch;   // M4 front-left  CW
    esc_clamp(esc_lim_lo, esc_lim_hi);

    if (!armed) { esc_1 = esc_2 = esc_3 = esc_4 = ESC_IDLE; }
    esc_write(esc_1, esc_2, esc_3, esc_4);

    telemetry_snapshot(sample_time_us);
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(PERIOD_CTRL_MS));
  }
}


// ============================================================================
//  Các chế độ bay
// ============================================================================

void mode_angle() {
  u_throttle = rc_ch[CH_THROTTLE] * 0.75f;
  u_roll  = pid_rate_roll (rate_roll_dps,  pid_tilt_roll (roll_deg,  roll_sp_deg));
  u_pitch = pid_rate_pitch(rate_pitch_dps, pid_tilt_pitch(pitch_deg, pitch_sp_deg));
  u_yaw   = pid_rate_yaw  (rate_yaw_dps,   yaw_rate_sp_dps);
}

void mode_alt_hold() {
  u_throttle = ESC_IDLE + pid_climb(alt_kf_get_climb_cms(), climb_sp_cms); //1150 thay cho ESC_IDLE
  u_roll  = pid_rate_roll (rate_roll_dps,  pid_tilt_roll (roll_deg,  roll_sp_deg));
  u_pitch = pid_rate_pitch(rate_pitch_dps, pid_tilt_pitch(pitch_deg, pitch_sp_deg));
  u_yaw   = pid_rate_yaw  (rate_yaw_dps,   yaw_rate_sp_dps);
}

void mode_failsafe() {
  u_throttle = ESC_IDLE + pid_climb(alt_kf_get_climb_cms(), FAILSAFE_DESCENT_CMS);
  u_roll  = pid_rate_roll (rate_roll_dps,  pid_tilt_roll (roll_deg,  0));
  u_pitch = pid_rate_pitch(rate_pitch_dps, pid_tilt_pitch(pitch_deg, 0));
  u_yaw   = pid_rate_yaw  (rate_yaw_dps,   0);
}


// ============================================================================
//  Lấy dữ liệu dùng chung từ các task khác
// ============================================================================

void baro_fetch_shared() {
  baro_is_new = false;
  if (xSemaphoreTake(mtx_baro, 0) == pdTRUE) {
    baro_sample = shared_baro_sample;
    baro_is_new = shared_baro_new;
    shared_baro_new = false;
    xSemaphoreGive(mtx_baro);
  }
  baro_alt_m = baro_sample.altitude_m;
  uint64_t now_us = esp_timer_get_time();
  uint64_t age_us = baro_sample.valid ? now_us - baro_sample.sample_time_us : UINT64_MAX;
  baro_age_us = age_us > UINT32_MAX ? UINT32_MAX : (uint32_t)age_us;
  baro_healthy = sensor_present.baro && baro_sample.valid &&
                 baro_sample.reference_valid && age_us <= BARO_STALE_US;
}

int rc_switch_position(int ch_value) {
  if      (ch_value > 900  && ch_value < 1100) return 0;
  else if (ch_value > 1400 && ch_value < 1600) return 1;
  else if (ch_value > 1900 && ch_value < 2100) return 2;
  else                                          return 0;
}

void rc_fetch_and_map() {
  if (xSemaphoreTake(mtx_rc, 0) == pdTRUE) {
    rc_ok = shared_rc_ok;
    for (int i = 1; i <= 16; i++) rc_ch[i] = shared_rc_ch[i];
    xSemaphoreGive(mtx_rc);
  }
  for (int i = 5; i <= 8; i++) rc_ch[i] = rc_switch_position(rc_ch[i]);

  const float a = 0.05f;
  roll_sp_deg     = roll_sp_deg     * (1 - a) + ( float(rc_ch[CH_ROLL]     - 1500) / (500.0f / LIM_TILT_DEG))        * a;
  pitch_sp_deg    = pitch_sp_deg    * (1 - a) + ( float(rc_ch[CH_PITCH]    - 1500) / (500.0f / LIM_TILT_DEG))        * a;
  yaw_rate_sp_dps = yaw_rate_sp_dps * (1 - a) + (-float(rc_ch[CH_YAW]      - 1500) / (500.0f / LIM_YAW_RATE_DPS))    * a;
  
  int thr_diff = rc_ch[CH_THROTTLE] - 1500;

  // Vùng chết ±60us (tương đương cần gạt từ -12 đến +12 trên EdgeTX):
  if (abs(thr_diff) < 60) {
    thr_diff = 0; // Nhích trong khoảng -12 đến +12: giữ nguyên cao độ tuyệt đối, không nhúc nhích ga
  } else if (thr_diff > 0) {
    thr_diff -= 60;
  } else {
    thr_diff += 60;
  }
  climb_sp_cms = climb_sp_cms * (1 - a) + ( float(thr_diff) / (440.0f / LIM_CLIMB_RATE_CMS) ) * a;
  
  //climb_sp_cms    = climb_sp_cms    * (1 - a) + ( float(rc_ch[CH_THROTTLE] - 1500) / (500.0f / LIM_CLIMB_RATE_CMS))  * a;
}

void esc_clamp(int lo, int hi) {
  if (esc_1 < lo) esc_1 = lo;  if (esc_1 > hi) esc_1 = hi;
  if (esc_2 < lo) esc_2 = lo;  if (esc_2 > hi) esc_2 = hi;
  if (esc_3 < lo) esc_3 = lo;  if (esc_3 > hi) esc_3 = hi;
  if (esc_4 < lo) esc_4 = lo;  if (esc_4 > hi) esc_4 = hi;
}

void telemetry_snapshot(uint64_t sample_time_us) {
  if (xSemaphoreTake(mtx_tlm, 0) != pdTRUE) return;

  tlm_sample_time_us = sample_time_us;

  tlm_roll_deg     = roll_deg;      tlm_pitch_deg    = pitch_deg;
  tlm_roll_sp_deg  = roll_sp_deg;   tlm_pitch_sp_deg = pitch_sp_deg;

  tlm_rate_roll_dps  = rate_roll_dps;
  tlm_rate_pitch_dps = rate_pitch_dps;
  tlm_rate_yaw_dps   = rate_yaw_dps;
  tlm_rate_roll_sp_dps  = pid_tilt_roll_output();
  tlm_rate_pitch_sp_dps = pid_tilt_pitch_output();

  tlm_alt_cm       = alt_kf_get_altitude_cm();
  tlm_climb_cms    = alt_kf_get_climb_cms();
  tlm_climb_sp_cms = climb_sp_cms;
  tlm_baro_alt_cm  = baro_alt_m * 100.0f;
  tlm_baro_sample = baro_sample;
  tlm_baro_age_us = baro_age_us;
  tlm_baro_healthy = baro_healthy;
  tlm_baro_correction_used = baro_correction_used;
  tlm_acc_earth_z_g = acc_earth_z_g;
  tlm_acc_bias_cms2 = alt_kf_get_acc_bias_cms2();
  tlm_acc_corrected_cms2 = acc_earth_z_g * 981.0f - tlm_acc_bias_cms2;

  tlm_acc_x_g = acc_x_g;  tlm_acc_y_g = acc_y_g;  tlm_acc_z_g = acc_z_g;

  for (int i = 1; i <= 16; i++) tlm_rc_ch[i] = rc_ch[i];
  tlm_esc[1] = esc_1;  tlm_esc[2] = esc_2;  tlm_esc[3] = esc_3;  tlm_esc[4] = esc_4;

  tlm_flight_mode = flight_mode;
  tlm_armed       = armed;
  tlm_rc_ok       = rc_ok;
  tlm_loop_us     = loop_period_us;

  xSemaphoreGive(mtx_tlm);
}
