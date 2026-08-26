# SPDX-License-Identifier: BSD-3-Clause
# PX4 v1.17.0 SensorMag compatibility contract.
@queue 4

uint64 timestamp
uint64 timestamp_sample
uint32 device_id
float32 x
float32 y
float32 z
float32 temperature
uint32 error_count
