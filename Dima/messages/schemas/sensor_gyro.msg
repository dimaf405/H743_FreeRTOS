# SPDX-License-Identifier: BSD-3-Clause
# PX4 v1.17.0 SensorGyro compatibility contract.
@queue 8

uint64 timestamp
uint64 timestamp_sample
uint32 device_id
float32 x
float32 y
float32 z
float32 temperature
uint32 error_count
uint8[3] clip_counter
uint8 samples
