# SPDX-License-Identifier: BSD-3-Clause
# PX4 v1.17.0 ActuatorMotors version 0 compatibility contract.
@queue 1

uint32 MESSAGE_VERSION = 0
uint8 ACTUATOR_FUNCTION_MOTOR1 = 101
uint8 NUM_CONTROLS = 12

uint64 timestamp
uint64 timestamp_sample
uint16 reversible_flags
float32[NUM_CONTROLS] control
