# SPDX-License-Identifier: BSD-3-Clause
# PX4 v1.17.0 InputRc compatibility contract.
@queue 1

uint8 RC_INPUT_SOURCE_UNKNOWN = 0
uint8 RC_INPUT_SOURCE_PX4FMU_PPM = 1
uint8 RC_INPUT_SOURCE_PX4IO_PPM = 2
uint8 RC_INPUT_SOURCE_PX4IO_SPEKTRUM = 3
uint8 RC_INPUT_SOURCE_PX4IO_SBUS = 4
uint8 RC_INPUT_SOURCE_PX4IO_ST24 = 5
uint8 RC_INPUT_SOURCE_MAVLINK = 6
uint8 RC_INPUT_SOURCE_QURT = 7
uint8 RC_INPUT_SOURCE_PX4FMU_SPEKTRUM = 8
uint8 RC_INPUT_SOURCE_PX4FMU_SBUS = 9
uint8 RC_INPUT_SOURCE_PX4FMU_ST24 = 10
uint8 RC_INPUT_SOURCE_PX4FMU_SUMD = 11
uint8 RC_INPUT_SOURCE_PX4FMU_DSM = 12
uint8 RC_INPUT_SOURCE_PX4IO_SUMD = 13
uint8 RC_INPUT_SOURCE_PX4FMU_CRSF = 14
uint8 RC_INPUT_SOURCE_PX4FMU_GHST = 15
uint8 RC_INPUT_MAX_CHANNELS = 18
int8 RSSI_MAX = 100

uint64 timestamp
uint64 timestamp_last_signal
uint8 channel_count
int32 rssi # Negative means unavailable.
bool rc_failsafe
bool rc_lost
uint16 rc_lost_frame_count
uint16 rc_total_frame_count
uint16 rc_ppm_frame_length
uint8 input_source
uint16[RC_INPUT_MAX_CHANNELS] values
int8 link_quality # 0..100, negative means unavailable.
float32 rssi_dbm # NaN means unavailable.
