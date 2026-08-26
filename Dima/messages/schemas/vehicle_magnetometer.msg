# SPDX-License-Identifier: BSD-3-Clause
# PX4 v1.17.0 VehicleMagnetometer compatibility contract.
@queue 1

uint64 timestamp
uint64 timestamp_sample
uint32 device_id
float32[3] magnetometer_ga
uint8 calibration_count
