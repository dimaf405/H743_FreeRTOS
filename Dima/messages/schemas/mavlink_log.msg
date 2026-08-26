# SPDX-License-Identifier: BSD-3-Clause
# PX4 MavlinkLog compatibility contract.
@queue 8

uint32 MESSAGE_VERSION = 1
uint8 TEXT_LEN = 128

uint64 timestamp
char[TEXT_LEN] text
uint8 severity
