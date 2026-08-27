# SPDX-License-Identifier: BSD-3-Clause
# Dima sensor calibration lifecycle status.
uint8 ORB_QUEUE_LENGTH = 1

uint8 TYPE_NONE = 0
uint8 TYPE_GYRO = 1
uint8 TYPE_ACCEL = 2
uint8 TYPE_MAG = 3

uint64 timestamp
uint8 type
uint8 progress
bool active
