# SPDX-License-Identifier: BSD-3-Clause
# PX4 v1.17.0 ActuatorArmed compatibility contract.
@queue 1

uint64 timestamp
bool armed
bool prearmed
bool ready_to_arm
bool lockdown
bool kill
bool termination
bool in_esc_calibration_mode
