# SPDX-License-Identifier: BSD-3-Clause
# PX4 v1.17.0 SensorAccelFifo compatibility contract.
@queue 1

uint8 MAX_SAMPLES = 32

uint64 timestamp
uint64 timestamp_sample
uint32 device_id
float32 dt
float32 scale
uint8 samples
int16[MAX_SAMPLES] x
int16[MAX_SAMPLES] y
int16[MAX_SAMPLES] z
