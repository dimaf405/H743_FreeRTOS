# SPDX-License-Identifier: BSD-3-Clause
# UM982 双天线原始航向与基线状态。该 schema 是 Topic/ID/字段的唯一权威源。
uint8 ORB_QUEUE_LENGTH = 8

uint32 MESSAGE_VERSION = 0

uint64 timestamp
uint64 timestamp_sample
uint32 device_id
uint32 parameter_update_instance
uint32 gps_milliseconds
uint16 gps_week
uint32 velocity_gps_milliseconds
uint16 velocity_gps_week

# array_heading_rad 使用与车体前向一致的约定：wrap_pi(UM982 raw heading + pi)，
# 尚未扣除 GPS_YAW_OFFSET，避免自动标定从已校正结果反推自身形成循环依赖。
float32 array_heading_rad
float32 heading_accuracy_rad
float32 baseline_m
float32 configured_baseline_m
float32 configured_yaw_offset_rad
float32 velocity_north_m_s
float32 velocity_east_m_s
float32 speed_accuracy_m_s
bool velocity_aligned
bool solution_computed
bool integer_fixed
bool baseline_consistent
