# SPDX-License-Identifier: BSD-3-Clause
# PX4 v1.17.0 VehicleImu compatibility contract.
@queue 1

uint8 CLIPPING_X = 1
uint8 CLIPPING_Y = 2
uint8 CLIPPING_Z = 4

uint64 timestamp
uint64 timestamp_sample
uint32 accel_device_id
uint32 gyro_device_id
float32[3] delta_angle
float32[3] delta_velocity
uint32 delta_angle_dt
uint32 delta_velocity_dt
uint8 delta_angle_clipping
uint8 delta_velocity_clipping
uint8 accel_calibration_count
uint8 gyro_calibration_count
