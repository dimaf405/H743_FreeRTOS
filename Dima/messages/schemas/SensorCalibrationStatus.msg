# SPDX-License-Identifier: BSD-3-Clause
# Dima sensor calibration lifecycle status.
uint8 ORB_QUEUE_LENGTH = 1

uint8 TYPE_NONE = 0
uint8 TYPE_GYRO = 1
uint8 TYPE_ACCEL = 2
uint8 TYPE_MAG = 3
uint8 TYPE_LEVEL = 4
uint8 RESULT_NONE = 0
uint8 RESULT_RUNNING = 1
uint8 RESULT_SUCCESS = 2
uint8 RESULT_FAILED = 3
uint8 RESULT_CANCELLED = 4

uint64 timestamp
uint64 request_timestamp
uint32 parameter_set_count
uint32 parameter_start_count
uint8 parameter_owned_changes
uint8 type
uint8 progress
bool active
uint8 result
