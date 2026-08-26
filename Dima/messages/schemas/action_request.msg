# SPDX-License-Identifier: BSD-3-Clause
# PX4 v1.17.0 ActionRequest compatibility contract.
@queue 8

uint8 ACTION_DISARM = 0
uint8 ACTION_ARM = 1
uint8 ACTION_TOGGLE_ARMING = 2
uint8 ACTION_UNKILL = 3
uint8 ACTION_KILL = 4
uint8 ACTION_SWITCH_MODE = 5
uint8 ACTION_VTOL_TRANSITION_TO_MULTICOPTER = 6
uint8 ACTION_VTOL_TRANSITION_TO_FIXEDWING = 7
uint8 ACTION_TERMINATION = 8
uint8 SOURCE_STICK_GESTURE = 0
uint8 SOURCE_RC_SWITCH = 1
uint8 SOURCE_RC_BUTTON = 2
uint8 SOURCE_RC_MODE_SLOT = 3

uint64 timestamp
uint8 action
uint8 source
uint8 mode # Only used by ACTION_SWITCH_MODE.
