// ============================================================================
//  DroneFC_CRSF_DShot — Vòng điều khiển chính
//  Dựa trên KyThuatUAV FC — Copyright (C) 2026 Nguyễn Văn Quý
//  License: GPL-3.0
// ============================================================================

float acc_x_g, acc_y_g, acc_z_g;
float rate_roll_dps, rate_pitch_dps, rate_yaw_dps;
float roll_deg, pitch_deg, acc_earth_z_g;
float roll_sp_deg, pitch_sp_deg, yaw_rate_sp_dps, climb_sp_cms;
float u_roll, u_pitch, u_yaw, u_throttle = ESC_IDLE;
int esc_1 = ESC_IDLE, esc_2 = ESC_IDLE, esc_3 = ESC_IDLE, esc_4 = ESC_IDLE;
int rc_raw_ch[17], rc_ch[17];
bool rc_ok, rc_ok_prev, rc_snapshot_ok, rc_snapshot_received;
uint32_t rc_frame_ms;
bool armed = false, control_fault = false, baro_healthy = false;
bool baro_seen = false, baro_snapshot_fault = true, vertical_inhibited = false;
bool rc_takeover_pending = false, climb_entry = false, climb_seed_limited = false;
bool airborne_confirmed = false, launch_aborted = false, recovery_angle_seen = false;
bool vertical_reacquire = true;
int arm_sw_prev = 2, recovery_mode_position = 0, last_mode_position = -1;
int flight_mode = MODE_ANGLE;
float baro_alt_m, applied_collective = ESC_IDLE, degraded_throttle = ESC_MIN_ARMED;
uint32_t baro_sample_ms, loop_period_us;
VerticalReadiness vertical_readiness = {};
AltPhase alt_phase = ALT_WAIT_TAKEOFF;
CollectiveSource collective_source = COLLECTIVE_DIRECT;
bool alt_launch_session = false, takeoff_intent = false, takeoff_evidence = false;
bool alt_arm_denied = false;
uint32_t takeoff_intent_ms, takeoff_evidence_ms;
float takeoff_alt_cm, ramp_collective = ESC_MIN_ARMED;
float limited_collective = ESC_MIN_ARMED;

float control_bound(float value, float lo, float hi) {
  if (!isfinite(value)) return NAN;
  return fminf(hi, fmaxf(lo, value));
}

bool control_vertical_mode(int mode) {
  return mode == MODE_ALT_HOLD || mode == MODE_FAILSAFE;
}

int control_requested_mode() {
  return rc_ch[CH_MODE] == 0 ? MODE_FOR_SW_0 :
      (rc_ch[CH_MODE] == 1 ? MODE_FOR_SW_1 : MODE_FOR_SW_2);
}

void control_task(void *parameter) {
  (void)parameter;
  TickType_t wake = xTaskGetTickCount();
  uint32_t previous_us = micros();
  for (;;) {
    uint32_t now_us = micros();
    loop_period_us = now_us - previous_us;
    previous_us = now_us;
    control_step();
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(PERIOD_CTRL_MS));
  }
}

void vertical_reset_readiness(int reason) {
  vertical_readiness = VerticalReadiness{};
  vertical_readiness.reason = reason;
  vertical_reacquire = true;
  takeoff_evidence = false;
}

void vertical_readiness_update() {
  VerticalReadiness &q = vertical_readiness;
  q.new_sample = false;
  if (!baro_healthy) {
    vertical_reset_readiness(VERTICAL_STALE);
    return;
  }
  if (vertical_reacquire) {
    // Không thay state estimator trong lúc đầu ra vẫn thuộc climb PI.
    if (collective_source == COLLECTIVE_CLIMB_PI) return;
    if (!alt_kf_reset_to_measurement(baro_alt_m)) {
      vertical_reset_readiness(VERTICAL_KF);
      return;
    }
    vertical_reacquire = false;
  }
  if (!alt_kf_update(baro_alt_m, acc_earth_z_g)) {
    vertical_reset_readiness(VERTICAL_KF);
    return;
  }
  if (!q.input_seen || baro_sample_ms != q.input_ms) {
    q.input_seen = q.accepted = q.new_sample = true;
    q.input_ms = q.accepted_ms = baro_sample_ms;
    q.altitude_m = baro_alt_m;
  }
  if (!q.window_active && q.new_sample) {
    q.window_active = q.window_static = true;
    q.window_ms = q.accepted_ms;
    q.window_alt_cm = q.min_cm = q.max_cm = q.altitude_m * 100.0f;
    q.acc_sum = 0;
    q.acc_count = q.sample_count = 0;
  }
  if (q.window_active) {
    q.acc_sum += acc_earth_z_g;
    ++q.acc_count;
    q.window_static = q.window_static && fabsf(roll_deg) < ALT_PREARM_TILT_DEG &&
        fabsf(pitch_deg) < ALT_PREARM_TILT_DEG &&
        fabsf(rate_roll_dps) < ALT_PREARM_RATE_DPS &&
        fabsf(rate_pitch_dps) < ALT_PREARM_RATE_DPS &&
        fabsf(rate_yaw_dps) < ALT_PREARM_RATE_DPS;
    if (!q.window_static) {
      q.ready = false;
      q.ready_ms = 0;
    }
    if (q.new_sample) {
      float height_cm = q.altitude_m * 100.0f;
      q.min_cm = fminf(q.min_cm, height_cm);
      q.max_cm = fmaxf(q.max_cm, height_cm);
      ++q.sample_count;
      uint32_t elapsed = q.accepted_ms - q.window_ms;
      if (elapsed >= ALT_QUALITY_WINDOW_MS && q.sample_count >= 3) {
        q.trend_cms = (height_cm - q.window_alt_cm) * (1000.0f / elapsed);
        q.trend_valid = true;
        bool stationary = q.window_static && fabsf(q.acc_sum / q.acc_count) < ALT_PREARM_ACC_G &&
            q.max_cm - q.min_cm < ALT_PREARM_BARO_RANGE_CM;
        q.valid_ms = q.valid_ms >= ALT_READY_MS ? ALT_READY_MS : q.valid_ms + elapsed;
        q.ready_ms = stationary ? (q.ready_ms >= ALT_READY_MS ? ALT_READY_MS : q.ready_ms + elapsed) : 0;
        q.ready = q.ready_ms >= ALT_READY_MS;
        q.window_active = false;
      }
    }
  }
  q.healthy = q.valid_ms >= ALT_READY_MS;
  q.reason = !q.healthy ? VERTICAL_ACQUIRING : (q.ready ? VERTICAL_OK : VERTICAL_PREARM);
}

void control_update_launch_evidence() {
  if (!armed || airborne_confirmed) return;
  if (!vertical_readiness.healthy) {
    takeoff_evidence = false;
    return;
  }
  if (launch_aborted || !vertical_readiness.new_sample ||
      (flight_mode != MODE_ANGLE && alt_phase != ALT_TAKEOFF)) return;
  bool rising = vertical_readiness.trend_valid && vertical_readiness.trend_cms > 0.0f &&
      vertical_readiness.altitude_m * 100.0f - takeoff_alt_cm > ALT_TAKEOFF_HEIGHT_CM;
  if (!rising) takeoff_evidence = false;
  else if (!takeoff_evidence) {
    takeoff_evidence = true;
    takeoff_evidence_ms = vertical_readiness.accepted_ms;
  } else if (uint32_t(vertical_readiness.accepted_ms - takeoff_evidence_ms) >= ALT_TAKEOFF_CONFIRM_MS) {
    airborne_confirmed = true;
    alt_phase = ALT_FLIGHT;
    climb_entry = true;
  }
}

void control_step() {
  bool imu_ok = imu_update();
  acc_x_g = imu_get_acc_x_g();
  acc_y_g = imu_get_acc_y_g();
  acc_z_g = imu_get_acc_z_g();
  rate_roll_dps = imu_get_rate_roll_dps();
  rate_pitch_dps = imu_get_rate_pitch_dps();
  rate_yaw_dps = imu_get_rate_yaw_dps();
  roll_deg = imu_get_roll_deg();
  pitch_deg = imu_get_pitch_deg();
  float sr = sinf(roll_deg * DEG_TO_RAD), cr = cosf(roll_deg * DEG_TO_RAD);
  float sp = sinf(pitch_deg * DEG_TO_RAD), cp = cosf(pitch_deg * DEG_TO_RAD);
  acc_earth_z_g = -sp * acc_x_g + cp * sr * acc_y_g + cp * cr * acc_z_g - 1.0f;
  if (!imu_ok || !isfinite(acc_earth_z_g)) control_fault = true;
  rc_ok_prev = rc_ok;
  rc_fetch_and_map();
  baro_fetch_shared();
  if (!control_fault) vertical_readiness_update();
  control_update_launch_evidence();
  control_update_state();
  if (!armed) {
    control_disarmed_output();
    telemetry_snapshot();
    return;
  }
  pid_set_attitude_integrating(true);
  switch (flight_mode) {
    case MODE_ALT_HOLD: mode_alt_hold(); break;
    case MODE_FAILSAFE: mode_failsafe(); break;
    case MODE_DEGRADED: mode_degraded(); break;
    default:
      collective_source = COLLECTIVE_DIRECT;
      if (!airborne_confirmed && rc_ch[CH_THROTTLE] * 0.6f < 830.0f) pid_bleed_integrators();
      mode_angle();
      break;
  }
  climb_entry = false;
  if (!control_outputs_valid()) {
    control_fault = true;
    armed = false;
    control_disarmed_output();
  } else {
    control_mix_and_write();
  }
  telemetry_snapshot();
}

void control_update_state() {
  int previous_mode = flight_mode;
  bool mode_changed = rc_ok && rc_ch[CH_MODE] != last_mode_position;
  if (control_fault) {
    armed = false;
    arm_sw_prev = 2;
  } else if (rc_ok) {
    if (!rc_ok_prev) {
      arm_sw_prev = rc_ch[CH_ARM];
      if (armed) recovery_mode_position = rc_ch[CH_MODE];
    }
    if (rc_ch[CH_ARM] != 2) {
      armed = false;
      alt_arm_denied = false;
    } else if (!armed && arm_sw_prev != 2 && rc_ch[CH_THROTTLE] < 1150) {
      alt_arm_denied = control_requested_mode() == MODE_ALT_HOLD && !vertical_readiness.ready;
      if (!alt_arm_denied) {
        armed = true;
        pid_reset_all();
        control_sync_commands();
        limited_collective = ramp_collective = degraded_throttle = ESC_MIN_ARMED;
        alt_launch_session = control_requested_mode() == MODE_ALT_HOLD;
        alt_phase = ALT_WAIT_TAKEOFF;
        takeoff_alt_cm = baro_alt_m * 100.0f;
        flight_mode = control_requested_mode();
        collective_source = alt_launch_session ? COLLECTIVE_RAMP : COLLECTIVE_DIRECT;
      }
    }
    arm_sw_prev = rc_ch[CH_ARM];
  } else {
    arm_sw_prev = 2;
  }
  if (!armed) {
    flight_mode = MODE_ANGLE;
    last_mode_position = rc_ok ? rc_ch[CH_MODE] : -1;
    return;
  }
  if (!rc_ok && alt_launch_session && alt_phase == ALT_WAIT_TAKEOFF) {
    armed = false;
    return;
  }
  bool vertical_ok = vertical_readiness.healthy;
  if (!vertical_ok && (airborne_confirmed || control_vertical_mode(flight_mode))) {
    vertical_inhibited = true;
    recovery_angle_seen = false;
    if (!airborne_confirmed) launch_aborted = true;
  }
  int next_mode = flight_mode;
  if (!rc_ok) {
    rc_takeover_pending = true;
    if (!airborne_confirmed) launch_aborted = true;
    next_mode = airborne_confirmed && vertical_ok && !vertical_inhibited && !launch_aborted
        ? MODE_FAILSAFE : MODE_DEGRADED;
  } else {
    bool takeover = !rc_takeover_pending;
    if (rc_takeover_pending && rc_ok_prev && rc_ch[CH_MODE] != recovery_mode_position) {
      rc_takeover_pending = false;
      takeover = true;
    }
    if (takeover) {
      int requested = control_requested_mode();
      if (vertical_inhibited || launch_aborted) {
        next_mode = flight_mode == MODE_ANGLE && requested == MODE_ANGLE && !launch_aborted
            ? MODE_ANGLE : MODE_DEGRADED;
        if (mode_changed && requested == MODE_ANGLE) recovery_angle_seen = true;
        if (mode_changed && requested == MODE_ALT_HOLD && recovery_angle_seen &&
            vertical_ok && airborne_confirmed && !launch_aborted) {
          vertical_inhibited = false;
          recovery_angle_seen = false;
          next_mode = MODE_ALT_HOLD;
        }
      } else if (requested == MODE_ALT_HOLD && flight_mode == MODE_ANGLE && !airborne_confirmed) {
        alt_arm_denied = true;
        next_mode = MODE_ANGLE;
      } else {
        next_mode = requested;
        alt_arm_denied = false;
      }
    }
  }
  if (control_vertical_mode(next_mode) && !vertical_ok) {
    vertical_inhibited = true;
    next_mode = MODE_DEGRADED;
  }
  if (next_mode == MODE_DEGRADED && flight_mode != MODE_DEGRADED) {
    degraded_throttle = control_bound(collective_source == COLLECTIVE_DIRECT ?
        applied_collective : limited_collective, ESC_MIN_ARMED, ESC_MAX);
    limited_collective = degraded_throttle;
  }
  if (control_vertical_mode(next_mode) && next_mode != previous_mode) climb_entry = true;
  flight_mode = next_mode;
  if (rc_ok) last_mode_position = rc_ch[CH_MODE];
}

void control_attitude(float roll_target, float pitch_target, float yaw_target) {
  u_roll = pid_rate_roll(rate_roll_dps, pid_tilt_roll(roll_deg, roll_target));
  u_pitch = pid_rate_pitch(rate_pitch_dps, pid_tilt_pitch(pitch_deg, pitch_target));
  u_yaw = pid_rate_yaw(rate_yaw_dps, yaw_target);
}

void mode_angle() {
  u_throttle = rc_ch[CH_THROTTLE] * 0.6f;
  control_attitude(roll_sp_deg, pitch_sp_deg, yaw_rate_sp_dps);
}

float control_collective_rate() {
  float command = control_climb_target() / LIM_CLIMB_RATE_CMS;
  return command * (command > 0.0f ? ALT_COLLECTIVE_UP_S : ALT_COLLECTIVE_DOWN_S);
}

void control_climb_output(float setpoint) {
  if (climb_entry || collective_source != COLLECTIVE_CLIMB_PI) {
    float seed_collective = control_bound(applied_collective, ESC_MIN_ARMED, ESC_MAX);
    if (collective_source == COLLECTIVE_DIRECT) limited_collective = seed_collective;
    climb_seed_limited = !pid_seed_climb(seed_collective, alt_kf_get_climb_cms(), setpoint);
    u_throttle = ESC_IDLE + pid_climb_output();
  } else {
    u_throttle = ESC_IDLE + pid_climb(alt_kf_get_climb_cms(), setpoint);
  }
  collective_source = COLLECTIVE_CLIMB_PI;
}

void mode_alt_hold() {
  if (alt_phase == ALT_WAIT_TAKEOFF) {
    pid_reset_all();
    u_roll = u_pitch = u_yaw = 0;
    u_throttle = ESC_MIN_ARMED;
    collective_source = COLLECTIVE_RAMP;
    if (rc_ch[CH_THROTTLE] > ALT_TAKEOFF_STICK && vertical_readiness.ready) {
      if (!takeoff_intent) {
        takeoff_intent = true;
        takeoff_intent_ms = millis();
      } else if (uint32_t(millis() - takeoff_intent_ms) >= ALT_TAKEOFF_INTENT_MS) {
        alt_phase = ALT_TAKEOFF;
        takeoff_alt_cm = vertical_readiness.altitude_m * 100.0f;
        ramp_collective = limited_collective;
        takeoff_evidence = false;
      }
    } else {
      takeoff_intent = false;
    }
    return;
  }
  if (alt_phase == ALT_TAKEOFF) {
    collective_source = COLLECTIVE_RAMP;
    ramp_collective = control_bound(ramp_collective + control_collective_rate() * DT_CTRL,
                                    ESC_MIN_ARMED, ESC_MAX);
    u_throttle = ramp_collective;
    pid_set_attitude_integrating(false);
  } else {
    control_climb_output(climb_sp_cms);
  }
  control_attitude(roll_sp_deg, pitch_sp_deg, yaw_rate_sp_dps);
}

void mode_failsafe() {
  control_climb_output(FAILSAFE_DESCENT_CMS);
  control_attitude(0.0f, 0.0f, 0.0f);
}

void mode_degraded() {
  collective_source = COLLECTIVE_RAMP;
  if (alt_launch_session && alt_phase == ALT_WAIT_TAKEOFF) {
    pid_set_attitude_integrating(false);
    degraded_throttle = u_throttle = ESC_MIN_ARMED;
    u_roll = u_pitch = u_yaw = 0;
    return;
  }
  bool can_control = rc_ok && !rc_takeover_pending;
  if (can_control) {
    float rate = control_collective_rate();
    if (airborne_confirmed || rate < 0.0f) {
      degraded_throttle = control_bound(degraded_throttle + rate * DT_CTRL, ESC_MIN_ARMED, ESC_MAX);
    }
  }
  u_throttle = degraded_throttle;
  if (!airborne_confirmed) pid_set_attitude_integrating(false);
  control_attitude(can_control ? roll_sp_deg : 0.0f,
                   can_control ? pitch_sp_deg : 0.0f,
                   can_control ? yaw_rate_sp_dps : 0.0f);
}

bool control_outputs_valid() {
  return isfinite(u_throttle) && isfinite(u_roll) && isfinite(u_pitch) && isfinite(u_yaw) &&
      isfinite(limited_collective);
}

void control_motor_values(float throttle, float roll, float pitch, float yaw, float *motors) {
  motors[0] = throttle + yaw - roll + pitch;
  motors[1] = throttle - yaw - roll - pitch;
  motors[2] = throttle - yaw + roll + pitch;
  motors[3] = throttle + yaw + roll - pitch;
}

float control_limit_collective(float requested, float previous) {
  return control_bound(control_bound(requested, ESC_MIN_ARMED, ESC_MAX),
      fmaxf(ESC_MIN_ARMED, previous - ALT_COLLECTIVE_DOWN_S * DT_CTRL),
      fminf(ESC_MAX, previous + ALT_COLLECTIVE_UP_S * DT_CTRL));
}

void control_mixer_residual(float collective, float *applied, float *delta) {
  control_motor_values(collective, u_roll, u_pitch, u_yaw, applied);
  for (int i = 0; i < 4; ++i) {
    float requested = applied[i];
    applied[i] = control_bound(requested, ESC_MIN_ARMED, ESC_MAX);
    delta[i] = applied[i] - requested;
  }
}

void control_mix_and_write() {
  bool limited = collective_source != COLLECTIVE_DIRECT;
  bool climb_active = collective_source == COLLECTIVE_CLIMB_PI;
  float previous = limited_collective;
  float collective = limited ? control_limit_collective(u_throttle, previous) : u_throttle;
  float applied[4], delta[4];
  control_mixer_residual(collective, applied, delta);
  bool changed = pid_motor_feedback(0,
      (-delta[0] - delta[1] + delta[2] + delta[3]) * 0.25f,
      (delta[0] - delta[1] + delta[2] - delta[3]) * 0.25f,
      (delta[0] - delta[1] - delta[2] + delta[3]) * 0.25f, false);
  if (changed) {
    u_roll = pid_rate_roll_output();
    u_pitch = pid_rate_pitch_output();
    u_yaw = pid_rate_yaw_output();
    control_mixer_residual(collective, applied, delta);
  }
  if (climb_active) {
    // Motor floor do rate không được giữ chặt bước giảm climb I.
    float ceiling_residual = 0;
    for (int i = 0; i < 4; ++i) ceiling_residual += fminf(0.0f, delta[i]) * 0.25f;
    float residual = collective - u_throttle + ceiling_residual;
    if (pid_climb_feedback(residual)) {
      u_throttle = ESC_IDLE + pid_climb_output();
      collective = control_limit_collective(u_throttle, previous);
      control_mixer_residual(collective, applied, delta);
      if (pid_motor_feedback(0,
          (-delta[0] - delta[1] + delta[2] + delta[3]) * 0.25f,
          (delta[0] - delta[1] + delta[2] - delta[3]) * 0.25f,
          (delta[0] - delta[1] - delta[2] + delta[3]) * 0.25f, false)) {
        u_roll = pid_rate_roll_output();
        u_pitch = pid_rate_pitch_output();
        u_yaw = pid_rate_yaw_output();
        control_mixer_residual(collective, applied, delta);
      }
    }
  }
  for (int i = 0; i < 4; ++i) {
    if (!isfinite(applied[i]) || !isfinite(delta[i])) {
      control_fault = true;
      armed = false;
      control_disarmed_output();
      return;
    }
  }
  limited_collective = limited ? collective : control_bound(collective, ESC_MIN_ARMED, ESC_MAX);
  if (collective_source == COLLECTIVE_RAMP) ramp_collective = degraded_throttle = limited_collective;
  esc_1 = int(applied[0]);
  esc_2 = int(applied[1]);
  esc_3 = int(applied[2]);
  esc_4 = int(applied[3]);
  applied_collective = (esc_1 + esc_2 + esc_3 + esc_4) * 0.25f;
  esc_write(esc_1, esc_2, esc_3, esc_4);
}

void control_disarmed_output() {
  pid_reset_all();
  roll_sp_deg = pitch_sp_deg = yaw_rate_sp_dps = climb_sp_cms = 0;
  alt_phase = ALT_WAIT_TAKEOFF;
  collective_source = COLLECTIVE_DIRECT;
  alt_launch_session = takeoff_intent = takeoff_evidence = false;
  airborne_confirmed = launch_aborted = recovery_angle_seen = false;
  vertical_inhibited = rc_takeover_pending = climb_entry = climb_seed_limited = false;
  limited_collective = ramp_collective = degraded_throttle = ESC_MIN_ARMED;
  takeoff_alt_cm = 0;
  takeoff_intent_ms = takeoff_evidence_ms = 0;
  u_roll = u_pitch = u_yaw = 0.0f;
  u_throttle = applied_collective = ESC_IDLE;
  esc_1 = esc_2 = esc_3 = esc_4 = ESC_IDLE;
  esc_write(esc_1, esc_2, esc_3, esc_4);
}

void baro_fetch_shared() {
  if (xSemaphoreTake(mtx_baro, 0) == pdTRUE) {
    baro_alt_m = shared_baro_alt_m;
    baro_seen = shared_baro_seen;
    baro_snapshot_fault = shared_baro_fault;
    baro_sample_ms = shared_baro_sample_ms;
    xSemaphoreGive(mtx_baro);
  }
  baro_healthy = sensor_present.baro && baro_seen && !baro_snapshot_fault && isfinite(baro_alt_m) &&
      uint32_t(millis() - baro_sample_ms) <= BARO_FRESH_MS;
}

int rc_switch_position(int value) {
  if (value > 1400 && value < 1600) return 1;
  if (value > 1900 && value < 2100) return 2;
  return 0;
}

float control_climb_target() {
  float offset = rc_ch[CH_THROTTLE] - 1500.0f;
  float magnitude = fmaxf(0.0f, fabsf(offset) - ALT_STICK_DEADBAND);
  float target = control_bound(magnitude / (500.0f - ALT_STICK_DEADBAND), 0.0f, 1.0f);
  return (offset < 0.0f ? -target : target) * LIM_CLIMB_RATE_CMS;
}

float control_roll_target() {
  return control_bound((rc_ch[CH_ROLL] - 1500.0f) * LIM_TILT_DEG / 500.0f, -LIM_TILT_DEG, LIM_TILT_DEG);
}

float control_pitch_target() {
  return control_bound((rc_ch[CH_PITCH] - 1500.0f) * LIM_TILT_DEG / 500.0f, -LIM_TILT_DEG, LIM_TILT_DEG);
}

float control_yaw_target() {
  return control_bound(-(rc_ch[CH_YAW] - 1500.0f) * LIM_YAW_RATE_DPS / 500.0f,
      -LIM_YAW_RATE_DPS, LIM_YAW_RATE_DPS);
}

void control_sync_commands() {
  roll_sp_deg = control_roll_target();
  pitch_sp_deg = control_pitch_target();
  yaw_rate_sp_dps = control_yaw_target();
  climb_sp_cms = control_climb_target();
}

void rc_fetch_and_map() {
  if (xSemaphoreTake(mtx_rc, 0) == pdTRUE) {
    rc_snapshot_ok = shared_rc_ok;
    rc_snapshot_received = shared_rc_received;
    rc_frame_ms = shared_rc_frame_ms;
    for (int i = 1; i <= 16; ++i) rc_raw_ch[i] = shared_rc_ch[i];
    xSemaphoreGive(mtx_rc);
  }
  rc_ok = rc_snapshot_ok && rc_snapshot_received && uint32_t(millis() - rc_frame_ms) <= RC_FRESH_MS;
  for (int i = 1; i <= 16; ++i) rc_ch[i] = rc_raw_ch[i];
  for (int i = 5; i <= 8; ++i) rc_ch[i] = rc_switch_position(rc_raw_ch[i]);
  if (!rc_ok) return;
  const float a = 0.05f;
  roll_sp_deg += (control_roll_target() - roll_sp_deg) * a;
  pitch_sp_deg += (control_pitch_target() - pitch_sp_deg) * a;
  yaw_rate_sp_dps += (control_yaw_target() - yaw_rate_sp_dps) * a;
  climb_sp_cms += (control_climb_target() - climb_sp_cms) * a;
}

void telemetry_snapshot() {
  if (xSemaphoreTake(mtx_tlm, 0) != pdTRUE) return;
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
  tlm_climb_sp_cms = flight_mode == MODE_FAILSAFE ? FAILSAFE_DESCENT_CMS : climb_sp_cms;
  tlm_baro_alt_cm = baro_alt_m * 100.0f;
  tlm_acc_x_g = acc_x_g;
  tlm_acc_y_g = acc_y_g;
  tlm_acc_z_g = acc_z_g;
  for (int i = 1; i <= 16; ++i) tlm_rc_ch[i] = rc_ch[i];
  tlm_esc[1] = esc_1;
  tlm_esc[2] = esc_2;
  tlm_esc[3] = esc_3;
  tlm_esc[4] = esc_4;
  tlm_flight_mode = flight_mode;
  tlm_armed = armed;
  tlm_rc_ok = rc_ok;
  tlm_baro_ok = baro_healthy;
  tlm_control_fault = control_fault;
  tlm_climb_seed_limited = climb_seed_limited;
  tlm_alt_phase = alt_phase;
  tlm_vertical_reason = vertical_readiness.reason;
  tlm_vertical_ok = vertical_readiness.healthy;
  tlm_alt_ready = vertical_readiness.ready;
  tlm_takeover_pending = rc_takeover_pending;
  tlm_arm_denied = alt_arm_denied;
  tlm_i_roll = pid_rate_roll_integral();
  tlm_i_pitch = pid_rate_pitch_integral();
  tlm_i_yaw = pid_rate_yaw_integral();
  tlm_i_climb = pid_climb_integral();
  tlm_collective_requested = u_throttle;
  tlm_collective_limited = limited_collective;
  tlm_loop_us = loop_period_us;
  xSemaphoreGive(mtx_tlm);
}
