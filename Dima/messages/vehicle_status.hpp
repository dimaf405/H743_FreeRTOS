/****************************************************************************
 *
 *   Copyright (c) 2012-2025 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include "uorb/uORB.hpp"

#include <cstdint>

/** PX4 v1.17.0 versioned/VehicleStatus.msg 的完整 Dima uORB 契约。 */
struct vehicle_status_s {
    static constexpr std::uint32_t MESSAGE_VERSION = 1U;

    static constexpr std::uint8_t ARMING_STATE_DISARMED = 1U;
    static constexpr std::uint8_t ARMING_STATE_ARMED = 2U;

    static constexpr std::uint8_t ARM_DISARM_REASON_TRANSITION_TO_STANDBY = 0U;
    static constexpr std::uint8_t ARM_DISARM_REASON_STICK_GESTURE = 1U;
    static constexpr std::uint8_t ARM_DISARM_REASON_RC_SWITCH = 2U;
    static constexpr std::uint8_t ARM_DISARM_REASON_COMMAND_INTERNAL = 3U;
    static constexpr std::uint8_t ARM_DISARM_REASON_COMMAND_EXTERNAL = 4U;
    static constexpr std::uint8_t ARM_DISARM_REASON_MISSION_START = 5U;
    static constexpr std::uint8_t ARM_DISARM_REASON_SAFETY_BUTTON = 6U;
    static constexpr std::uint8_t ARM_DISARM_REASON_AUTO_DISARM_LAND = 7U;
    static constexpr std::uint8_t ARM_DISARM_REASON_AUTO_DISARM_PREFLIGHT = 8U;
    static constexpr std::uint8_t ARM_DISARM_REASON_KILL_SWITCH = 9U;
    static constexpr std::uint8_t ARM_DISARM_REASON_LOCKDOWN = 10U;
    static constexpr std::uint8_t ARM_DISARM_REASON_FAILURE_DETECTOR = 11U;
    static constexpr std::uint8_t ARM_DISARM_REASON_SHUTDOWN = 12U;
    static constexpr std::uint8_t ARM_DISARM_REASON_UNIT_TEST = 13U;

    static constexpr std::uint8_t NAVIGATION_STATE_MANUAL = 0U;
    static constexpr std::uint8_t NAVIGATION_STATE_ALTCTL = 1U;
    static constexpr std::uint8_t NAVIGATION_STATE_POSCTL = 2U;
    static constexpr std::uint8_t NAVIGATION_STATE_AUTO_MISSION = 3U;
    static constexpr std::uint8_t NAVIGATION_STATE_AUTO_LOITER = 4U;
    static constexpr std::uint8_t NAVIGATION_STATE_AUTO_RTL = 5U;
    static constexpr std::uint8_t NAVIGATION_STATE_POSITION_SLOW = 6U;
    static constexpr std::uint8_t NAVIGATION_STATE_FREE5 = 7U;
    static constexpr std::uint8_t NAVIGATION_STATE_ALTITUDE_CRUISE = 8U;
    static constexpr std::uint8_t NAVIGATION_STATE_FREE3 = 9U;
    static constexpr std::uint8_t NAVIGATION_STATE_ACRO = 10U;
    static constexpr std::uint8_t NAVIGATION_STATE_FREE2 = 11U;
    static constexpr std::uint8_t NAVIGATION_STATE_DESCEND = 12U;
    static constexpr std::uint8_t NAVIGATION_STATE_TERMINATION = 13U;
    static constexpr std::uint8_t NAVIGATION_STATE_OFFBOARD = 14U;
    static constexpr std::uint8_t NAVIGATION_STATE_STAB = 15U;
    static constexpr std::uint8_t NAVIGATION_STATE_FREE1 = 16U;
    static constexpr std::uint8_t NAVIGATION_STATE_AUTO_TAKEOFF = 17U;
    static constexpr std::uint8_t NAVIGATION_STATE_AUTO_LAND = 18U;
    static constexpr std::uint8_t NAVIGATION_STATE_AUTO_FOLLOW_TARGET = 19U;
    static constexpr std::uint8_t NAVIGATION_STATE_AUTO_PRECLAND = 20U;
    static constexpr std::uint8_t NAVIGATION_STATE_ORBIT = 21U;
    static constexpr std::uint8_t NAVIGATION_STATE_AUTO_VTOL_TAKEOFF = 22U;
    static constexpr std::uint8_t NAVIGATION_STATE_EXTERNAL1 = 23U;
    static constexpr std::uint8_t NAVIGATION_STATE_EXTERNAL2 = 24U;
    static constexpr std::uint8_t NAVIGATION_STATE_EXTERNAL3 = 25U;
    static constexpr std::uint8_t NAVIGATION_STATE_EXTERNAL4 = 26U;
    static constexpr std::uint8_t NAVIGATION_STATE_EXTERNAL5 = 27U;
    static constexpr std::uint8_t NAVIGATION_STATE_EXTERNAL6 = 28U;
    static constexpr std::uint8_t NAVIGATION_STATE_EXTERNAL7 = 29U;
    static constexpr std::uint8_t NAVIGATION_STATE_EXTERNAL8 = 30U;
    static constexpr std::uint8_t NAVIGATION_STATE_MAX = 31U;

    static constexpr std::uint16_t FAILURE_NONE = 0U;
    static constexpr std::uint16_t FAILURE_ROLL = 1U;
    static constexpr std::uint16_t FAILURE_PITCH = 2U;
    static constexpr std::uint16_t FAILURE_ALT = 4U;
    static constexpr std::uint16_t FAILURE_EXT = 8U;
    static constexpr std::uint16_t FAILURE_ARM_ESC = 16U;
    static constexpr std::uint16_t FAILURE_BATTERY = 32U;
    static constexpr std::uint16_t FAILURE_IMBALANCED_PROP = 64U;
    static constexpr std::uint16_t FAILURE_MOTOR = 128U;

    static constexpr std::uint8_t HIL_STATE_OFF = 0U;
    static constexpr std::uint8_t HIL_STATE_ON = 1U;

    static constexpr std::uint8_t VEHICLE_TYPE_UNSPECIFIED = 0U;
    static constexpr std::uint8_t VEHICLE_TYPE_ROTARY_WING = 1U;
    static constexpr std::uint8_t VEHICLE_TYPE_FIXED_WING = 2U;
    static constexpr std::uint8_t VEHICLE_TYPE_ROVER = 3U;

    static constexpr std::uint8_t FAILSAFE_DEFER_STATE_DISABLED = 0U;
    static constexpr std::uint8_t FAILSAFE_DEFER_STATE_ENABLED = 1U;
    static constexpr std::uint8_t FAILSAFE_DEFER_STATE_WOULD_FAILSAFE = 2U;

    std::uint64_t timestamp;
    std::uint64_t armed_time;
    std::uint64_t takeoff_time;
    std::uint8_t arming_state;
    std::uint8_t latest_arming_reason;
    std::uint8_t latest_disarming_reason;
    std::uint64_t nav_state_timestamp;
    std::uint8_t nav_state_user_intention;
    std::uint8_t nav_state;
    std::uint8_t executor_in_charge;
    std::uint32_t valid_nav_states_mask;
    std::uint32_t can_set_nav_states_mask;
    std::uint16_t failure_detector_status;
    std::uint8_t hil_state;
    std::uint8_t vehicle_type;
    bool failsafe;
    bool failsafe_and_user_took_over;
    std::uint8_t failsafe_defer_state;
    bool gcs_connection_lost;
    std::uint8_t gcs_connection_lost_counter;
    bool high_latency_data_link_lost;
    bool is_vtol;
    bool is_vtol_tailsitter;
    bool in_transition_mode;
    bool in_transition_to_fw;
    std::uint8_t system_type;
    std::uint8_t system_id;
    std::uint8_t component_id;
    bool safety_button_available;
    bool safety_off;
    bool power_input_valid;
    bool usb_connected;
    bool open_drone_id_system_present;
    bool open_drone_id_system_healthy;
    bool parachute_system_present;
    bool parachute_system_healthy;
    bool rc_calibration_in_progress;
    bool calibration_enabled;
    bool pre_flight_checks_pass;
};

ORB_DECLARE(vehicle_status);
