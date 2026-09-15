# SPDX-License-Identifier: BSD-3-Clause
# Dima six-channel actuator backend status contract.
uint8 ORB_QUEUE_LENGTH = 8

uint32 MESSAGE_VERSION = 0
uint8 NUM_OUTPUTS = 6
uint8 STATE_STOPPED = 0
uint8 STATE_HARD_SAFE_OFF = 1
uint8 STATE_SAFE_OFF = 1 # Wire-compatible alias of STATE_HARD_SAFE_OFF.
uint8 STATE_ACTIVE = 2
uint8 STATE_RETRY = 3
uint8 STATE_FAULT = 4
uint8 STATE_DISARMED_NEUTRAL = 5
uint8 STATE_CONTROL_INHIBITED = 6 # 收到新鲜的显式失效控制帧，且 PWM 已确认停波。

uint64 timestamp
uint64 timestamp_sample
# 后端成功应用波形/停波的时间，不是控制或估计器采样时间。
uint64 timestamp_output
# 已应用的逻辑右/左轮归一化输出（已撤销 PWM REV 映射）；未确认时为 NaN。
float32 applied_right
float32 applied_left
uint32 sequence
uint16[6] pwm_us
uint8 active_output_mask
uint8 configured_output_mask
uint8 right_output_mask
uint8 left_output_mask
uint8 state
bool backend_ready
bool drive_available
bool safe_off
bool command_valid
bool parameter_update_pending
