# SPDX-License-Identifier: BSD-3-Clause
# 真实差速控制器反馈，供校准验证与 ULog 使用；不是 MAVLink 线消息。
uint8 ORB_QUEUE_LENGTH = 8
uint32 MESSAGE_VERSION = 0

uint64 timestamp
uint64 timestamp_sample
uint32 session_id
uint32 request_sequence
uint32 parameter_update_instance
uint8 source
bool valid
bool closed_loop
bool saturated
bool input_limited
bool motor_slew_active
bool mixing_limited
bool shaping_active
bool arm_ramp_active
bool reversal_held
bool safety_output_limited
bool safety_slew_active
float32 speed_setpoint_m_s
float32 yaw_rate_setpoint_rad_s
float32 speed_m_s
float32 speed_raw_m_s
float32 forward_raw_m_s
float32 lateral_raw_m_s
float32 yaw_rate_rad_s
float32 longitudinal
float32 steering
float32 applied_longitudinal
float32 applied_steering
float32 speed_integral
float32 yaw_rate_integral
