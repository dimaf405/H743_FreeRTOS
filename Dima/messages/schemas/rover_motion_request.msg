# SPDX-License-Identifier: BSD-3-Clause
# Dima Manual/Navigation rover motion boundary.
@queue 8

uint32 MESSAGE_VERSION = 0
uint8 SOURCE_MANUAL = 0
uint8 SOURCE_NAVIGATION = 1
uint8 MODE_NORMALIZED_AXES = 0
uint8 MODE_SPEED_YAW_RATE = 1

uint64 timestamp
uint64 timestamp_sample
uint32 sequence
bool valid
uint8 source
uint8 mode
float32 normalized_longitudinal
float32 normalized_steering
float32 speed_m_s
float32 yaw_rate_rad_s
