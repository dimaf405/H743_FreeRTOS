# SPDX-License-Identifier: BSD-3-Clause
# PX4 v1.17.0 ManualControlSetpoint compatibility contract.
@queue 1

uint32 MESSAGE_VERSION = 0
uint8 SOURCE_UNKNOWN = 0
uint8 SOURCE_RC = 1
uint8 SOURCE_MAVLINK_0 = 2
uint8 SOURCE_MAVLINK_1 = 3
uint8 SOURCE_MAVLINK_2 = 4
uint8 SOURCE_MAVLINK_3 = 5
uint8 SOURCE_MAVLINK_4 = 6
uint8 SOURCE_MAVLINK_5 = 7

uint64 timestamp
uint64 timestamp_sample
bool valid
uint8 data_source
float32 roll
float32 pitch
float32 yaw
float32 throttle
float32 flaps
float32 aux1
float32 aux2
float32 aux3
float32 aux4
float32 aux5
float32 aux6
bool sticks_moving
uint16 buttons
