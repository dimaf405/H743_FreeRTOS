# SPDX-License-Identifier: BSD-3-Clause
# Commander-to-sensor-calibration worker request contract.
uint8 ORB_QUEUE_LENGTH = 4

uint32 MESSAGE_VERSION = 0
uint8 REQUEST_NONE = 0
uint8 REQUEST_GYRO = 1
uint8 REQUEST_ACCEL = 2
uint8 REQUEST_MAG = 3
uint8 REQUEST_CANCEL = 4
uint8 REQUEST_LEVEL = 5

# 仅本地反馈归属；不修改 MAVLink 命令、ACK 或锁定 PX4 payload。
uint8 FEEDBACK_NONE = 0
uint8 FEEDBACK_QGC = 1
uint8 FEEDBACK_AUTO = 2

uint64 timestamp
uint8 request
uint8 feedback_owner
uint32 parameter_set_count # AUTO Level 的受控入场计数；QGC 请求不使用此字段。
