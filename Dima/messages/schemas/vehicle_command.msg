# SPDX-License-Identifier: BSD-3-Clause
# PX4 v1.17.0 VehicleCommand version 1 compatibility contract.
@queue 8

uint32 MESSAGE_VERSION = 1
uint16 NAV_CMD_DO_SET_MODE = 176
uint16 NAV_CMD_COMPONENT_ARM_DISARM = 400
uint16 NAV_CMD_PREFLIGHT_CALIBRATION = 241
uint16 NAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN = 246
uint16 NAV_CMD_REQUEST_MESSAGE = 512
uint8 SOURCE_MAVLINK = 0
uint8 SOURCE_INTERNAL = 1

uint64 timestamp
uint32 param1_raw
uint32 param2_raw
uint32 param3_raw
uint32 param4_raw
uint32 param5_raw
uint32 param6_raw
uint32 param7_raw
uint16 command
uint8 target_system
uint8 target_component
uint8 source_system
uint8 source_component
uint8 confirmation
bool from_external
