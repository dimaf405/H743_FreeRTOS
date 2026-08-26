# SPDX-License-Identifier: BSD-3-Clause
# PX4 v1.17.0 ManualControlSwitches compatibility contract.
@queue 1

uint8 SWITCH_POS_NONE = 0
uint8 SWITCH_POS_ON = 1
uint8 SWITCH_POS_MIDDLE = 2
uint8 SWITCH_POS_OFF = 3
uint8 MODE_SLOT_NONE = 0
uint8 MODE_SLOT_1 = 1
uint8 MODE_SLOT_2 = 2
uint8 MODE_SLOT_3 = 3
uint8 MODE_SLOT_4 = 4
uint8 MODE_SLOT_5 = 5
uint8 MODE_SLOT_6 = 6
uint8 MODE_SLOT_NUM = 6

uint64 timestamp
uint64 timestamp_sample
uint8 mode_slot
uint8 arm_switch
uint8 return_switch
uint8 loiter_switch
uint8 offboard_switch
uint8 kill_switch
uint8 termination_switch
uint8 gear_switch
uint8 transition_switch
uint8 photo_switch
uint8 video_switch
uint8 engage_main_motor_switch
uint8 payload_power_switch
uint32 switch_changes
