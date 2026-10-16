// ============================================================================
//  KyThuatUAV FC - firmware bay ESP32 (FreeRTOS), che do Angle + Alt Hold
//  Copyright (C) 2026 Nguyễn Văn Quý (Ky Thuat UAV)
//
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program. If not, see <https://www.gnu.org/licenses/>.
//  Nguon tham khao va ghi cong: LOI_NHAN_VA_GHI_CONG.md
//  PID cascade tham khao Carbon Aeronautics. License: GPL-3.0.
// ============================================================================

#include "types.h"

PidState pid_rate_roll_st = {}, pid_rate_pitch_st = {}, pid_rate_yaw_st = {};
PidState pid_tilt_roll_st = {}, pid_tilt_pitch_st = {}, pid_climb_st = {};
static float tilt_roll_out_dps, tilt_pitch_out_dps;
static bool pid_attitude_integrating = true;

static void pid_prepare_attitude(PidState &st) {
  if (!pid_attitude_integrating) {
    st.integral = 0.0f;
    st.integral_before = 0.0f;
  }
}

void pid_set_attitude_integrating(bool enabled) {
  pid_attitude_integrating = enabled;
  pid_prepare_attitude(pid_rate_roll_st);
  pid_prepare_attitude(pid_rate_pitch_st);
  pid_prepare_attitude(pid_rate_yaw_st);
  pid_prepare_attitude(pid_tilt_roll_st);
  pid_prepare_attitude(pid_tilt_pitch_st);
}

float pid_bound(float value, float lo, float hi) {
  if (!isfinite(value)) return NAN;
  return fminf(hi, fmaxf(lo, value));
}

float pid_step(PidState &st, float error, float kp, float ki, float kd,
               float i_limit, float u_limit) {
  const float previous = st.initialized ? st.prev_error : error;
  st.integral_before = st.integral;
  st.pd_term = kp * error + (st.initialized ? kd * (error - previous) / DT_CTRL : 0.0f);
  float candidate = pid_bound(st.integral + ki * (error + previous) * DT_CTRL * 0.5f,
                             -i_limit, i_limit);
  float raw = st.pd_term + candidate;
  if (!((raw > u_limit && candidate > st.integral) ||
        (raw < -u_limit && candidate < st.integral))) {
    st.integral = candidate;
  }
  st.prev_error = error;
  st.initialized = true;
  st.output = pid_bound(st.pd_term + st.integral, -u_limit, u_limit);
  return st.output;
}

float pid_step_climb(PidState &st, float error, float kp, float ki, float kd,
                     float i_limit, float p_limit) {
  const float previous = st.initialized ? st.prev_error : error;
  st.integral_before = st.integral;
  st.pd_term = pid_bound(kp * error, -p_limit, p_limit) +
      (st.initialized ? kd * (error - previous) / DT_CTRL : 0.0f);
  float candidate = pid_bound(st.integral + ki * (error + previous) * DT_CTRL * 0.5f,
                             0.0f, i_limit);
  float throttle = ESC_IDLE + st.pd_term + candidate;
  if (!((throttle > ESC_MAX && candidate > st.integral) ||
        (throttle < ESC_MIN_ARMED && candidate < st.integral))) {
    st.integral = candidate;
  }
  st.prev_error = error;
  st.initialized = true;
  // Giữ raw output để feedback nhìn thấy cả phần bị chặn ngoài PID.
  st.output = st.pd_term + st.integral;
  return st.output;
}

float pid_rate_roll(float rate_dps, float rate_sp_dps) {
  pid_prepare_attitude(pid_rate_roll_st);
  return pid_step(pid_rate_roll_st, rate_sp_dps - rate_dps,
                  KP_RATE_ROLL, pid_attitude_integrating ? KI_RATE_ROLL : 0.0f,
                  KD_RATE_ROLL, I_LIM_RATE, U_LIM_RATE);
}

float pid_rate_pitch(float rate_dps, float rate_sp_dps) {
  pid_prepare_attitude(pid_rate_pitch_st);
  return pid_step(pid_rate_pitch_st, rate_sp_dps - rate_dps,
                  KP_RATE_PITCH, pid_attitude_integrating ? KI_RATE_PITCH : 0.0f,
                  KD_RATE_PITCH, I_LIM_RATE, U_LIM_RATE);
}

float pid_rate_yaw(float rate_dps, float rate_sp_dps) {
  pid_prepare_attitude(pid_rate_yaw_st);
  return pid_step(pid_rate_yaw_st, rate_sp_dps - rate_dps,
                  KP_RATE_YAW, pid_attitude_integrating ? KI_RATE_YAW : 0.0f,
                  KD_RATE_YAW, I_LIM_YAW, U_LIM_YAW);
}

float pid_tilt_roll(float angle_deg, float angle_sp_deg) {
  pid_prepare_attitude(pid_tilt_roll_st);
  tilt_roll_out_dps = pid_step(pid_tilt_roll_st, angle_sp_deg - angle_deg,
                              KP_TILT_ROLL, pid_attitude_integrating ? KI_TILT_ROLL : 0.0f,
                              KD_TILT_ROLL, I_LIM_TILT, U_LIM_TILT);
  return tilt_roll_out_dps;
}

float pid_tilt_pitch(float angle_deg, float angle_sp_deg) {
  pid_prepare_attitude(pid_tilt_pitch_st);
  tilt_pitch_out_dps = pid_step(pid_tilt_pitch_st, angle_sp_deg - angle_deg,
                               KP_TILT_PITCH, pid_attitude_integrating ? KI_TILT_PITCH : 0.0f,
                               KD_TILT_PITCH, I_LIM_TILT, U_LIM_TILT);
  return tilt_pitch_out_dps;
}

float pid_tilt_roll_output() { return tilt_roll_out_dps; }
float pid_tilt_pitch_output() { return tilt_pitch_out_dps; }
float pid_rate_roll_output() { return pid_rate_roll_st.output; }
float pid_rate_pitch_output() { return pid_rate_pitch_st.output; }
float pid_rate_yaw_output() { return pid_rate_yaw_st.output; }
float pid_climb_output() { return pid_climb_st.output; }
float pid_rate_roll_integral() { return pid_rate_roll_st.integral; }
float pid_rate_pitch_integral() { return pid_rate_pitch_st.integral; }
float pid_rate_yaw_integral() { return pid_rate_yaw_st.integral; }
float pid_climb_integral() { return pid_climb_st.integral; }

float pid_climb(float climb_cms, float climb_sp_cms_in) {
  return pid_step_climb(pid_climb_st, climb_sp_cms_in - climb_cms,
                        KP_CLIMB, KI_CLIMB, KD_CLIMB, I_LIM_CLIMB, P_LIM_CLIMB);
}

bool pid_seed_climb(float applied_throttle, float climb_cms, float setpoint_cms) {
  float error = setpoint_cms - climb_cms;
  float p = pid_bound(KP_CLIMB * error, -P_LIM_CLIMB, P_LIM_CLIMB);
  float requested_i = applied_throttle - ESC_IDLE - p;
  pid_climb_st = PidState{};
  pid_climb_st.integral = pid_bound(requested_i, 0.0f, I_LIM_CLIMB);
  pid_climb_st.integral_before = pid_climb_st.integral;
  pid_climb_st.prev_error = error;
  pid_climb_st.pd_term = p;
  pid_climb_st.output = p + pid_climb_st.integral;
  pid_climb_st.initialized = true;
  return fabsf(requested_i - pid_climb_st.integral) < 0.001f;
}

bool pid_reject_motor_windup(PidState &st, float residual, float lo, float hi) {
  float delta = st.integral - st.integral_before;
  if (delta * residual >= -0.0001f) return false;
  st.integral = st.integral_before;
  st.output = pid_bound(st.pd_term + st.integral, lo, hi);
  return true;
}

bool pid_climb_feedback(float collective_residual) {
  PidState &st = pid_climb_st;
  float delta = st.integral - st.integral_before;
  if (delta * collective_residual >= -0.0001f) return false;
  // Keep the attainable part of this I step so slew cannot stall a steady command.
  st.integral = pid_bound(st.integral + collective_residual,
      fminf(st.integral_before, st.integral), fmaxf(st.integral_before, st.integral));
  st.output = st.pd_term + st.integral;
  return true;
}

bool pid_motor_feedback(float throttle_residual, float roll_residual,
                        float pitch_residual, float yaw_residual, bool climb_active) {
  bool changed = pid_reject_motor_windup(pid_rate_roll_st, roll_residual, -U_LIM_RATE, U_LIM_RATE);
  changed |= pid_reject_motor_windup(pid_rate_pitch_st, pitch_residual, -U_LIM_RATE, U_LIM_RATE);
  changed |= pid_reject_motor_windup(pid_rate_yaw_st, yaw_residual, -U_LIM_YAW, U_LIM_YAW);
  if (climb_active) {
    changed |= pid_climb_feedback(throttle_residual);
  }
  return changed;
}

void pid_reset_all() {
  pid_attitude_integrating = true;
  pid_rate_roll_st = PidState{};
  pid_rate_pitch_st = PidState{};
  pid_rate_yaw_st = PidState{};
  pid_tilt_roll_st = PidState{};
  pid_tilt_pitch_st = PidState{};
  pid_climb_st = PidState{};
  tilt_roll_out_dps = tilt_pitch_out_dps = 0.0f;
}

void pid_bleed_integrators() {
  pid_rate_roll_st.integral *= 0.9f;
  pid_rate_pitch_st.integral *= 0.9f;
  pid_rate_yaw_st.integral *= 0.9f;
  pid_tilt_roll_st.integral *= 0.9f;
  pid_tilt_pitch_st.integral *= 0.9f;
}
