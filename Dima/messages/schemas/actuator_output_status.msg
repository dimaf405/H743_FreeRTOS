# SPDX-License-Identifier: BSD-3-Clause
# Dima six-channel actuator backend status contract.
@queue 8

uint32 MESSAGE_VERSION = 0
uint8 NUM_OUTPUTS = 6
uint8 STATE_STOPPED = 0
uint8 STATE_HARD_SAFE_OFF = 1
uint8 STATE_SAFE_OFF = STATE_HARD_SAFE_OFF
uint8 STATE_ACTIVE = 2
uint8 STATE_RETRY = 3
uint8 STATE_FAULT = 4
uint8 STATE_DISARMED_NEUTRAL = 5

uint64 timestamp
uint64 timestamp_sample
uint32 sequence
uint16[NUM_OUTPUTS] pwm_us
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
