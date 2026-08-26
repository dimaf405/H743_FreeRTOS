# SPDX-License-Identifier: BSD-3-Clause
# PX4 v1.17.0 VehicleCommandAck version 1 compatibility contract.
@queue 4

uint32 MESSAGE_VERSION = 1
uint8 RESULT_ACCEPTED = 0
uint8 RESULT_TEMPORARILY_REJECTED = 1
uint8 RESULT_DENIED = 2
uint8 RESULT_UNSUPPORTED = 3
uint8 RESULT_FAILED = 4
uint8 RESULT_IN_PROGRESS = 5
uint8 RESULT_CANCELLED = 6

uint64 timestamp
uint32 result_param2
uint16 command
uint8 result
bool from_external
uint8 target_system
uint8 target_component
