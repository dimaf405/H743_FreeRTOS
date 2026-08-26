# SPDX-License-Identifier: BSD-3-Clause
# PX4 v1.17.0 VehicleImuStatus compatibility contract.
@queue 1

uint64 timestamp
uint32 accel_device_id
uint32 gyro_device_id
uint32[3] accel_clipping
uint32[3] gyro_clipping
uint32 accel_error_count
uint32 gyro_error_count
float32 accel_rate_hz
float32 gyro_rate_hz
float32 accel_raw_rate_hz
float32 gyro_raw_rate_hz
float32 accel_vibration_metric
float32 gyro_vibration_metric
float32 delta_angle_coning_metric
float32[3] mean_accel
float32[3] mean_gyro
float32[3] var_accel
float32[3] var_gyro
float32 temperature_accel
float32 temperature_gyro
