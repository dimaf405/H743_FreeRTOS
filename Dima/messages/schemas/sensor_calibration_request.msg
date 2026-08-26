# SPDX-License-Identifier: BSD-3-Clause
# Commander-to-sensor-calibration worker request contract.
@queue 4

uint32 MESSAGE_VERSION = 0
uint8 REQUEST_NONE = 0
uint8 REQUEST_GYRO = 1
uint8 REQUEST_ACCEL = 2
uint8 REQUEST_MAG = 3
uint8 REQUEST_CANCEL = 4

uint64 timestamp
uint8 request
