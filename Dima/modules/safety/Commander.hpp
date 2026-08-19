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

#include "action_request.hpp"
#include "actuator_armed.hpp"
#include "actuator_output_status.hpp"
#include "manual_control_setpoint.hpp"
#include "parameter_update.hpp"
#include "vehicle_command.hpp"
#include "vehicle_command_ack.hpp"
#include "vehicle_control_mode.hpp"
#include "vehicle_status.hpp"
#include "lifecycle/module_base.hpp"
#include "parameters/param.h"
#include "platform/api/Platform.hpp"
#include "uorb/Publication.hpp"
#include "uorb/SubscriptionData.hpp"
#include "work_queue/WorkQueue.hpp"

#include <cstdint>

namespace dima::modules::safety {

/**
 * PX4 Commander 的 Dima Rover 安全子集。
 *
 * 只维护安全状态并发布 uORB Topic，不拥有任何执行器或 HAL 输出。
 */
class Commander final : public dima::middleware::lifecycle::ModuleBase,
                        public px4::ScheduledWorkItem {
public:
    explicit Commander(
        dima::platform::ArmedFlashCoordinator &armed_flash) noexcept;
    ~Commander() override;

    bool start() override;
    void stop() override;
    dima::middleware::lifecycle::ModuleState state() const override;

    /** 供非 Commander WorkQueue 的存储门控读取。 */
    bool armed() const noexcept { return armed_flash_.armed(); }

private:
    static constexpr std::uint32_t kCheckIntervalUs = 20000U;
    static constexpr std::uint64_t kPublishIntervalUs = 500000ULL;
    static constexpr std::uint64_t kActuatorStatusTimeoutUs = 250000ULL;
    static constexpr std::uint64_t kActuatorArmTransitionUs = 250000ULL;
    // 安全正向动作不得在队列中滞留超过一个公开状态心跳周期。
    static constexpr std::uint64_t kActionRequestMaxAgeUs = kPublishIntervalUs;

    enum FailsafeCause : std::uint8_t {
        FailsafeNone = 0U,
        FailsafeRcLoss = 1U << 0,
        FailsafeParameters = 1U << 1,
        FailsafeActuatorOutput = 1U << 2,
    };

    enum class TransitionResult : std::uint8_t {
        Changed,
        NotChanged,
        Denied,
    };

    void Run() override;
    bool initialize_parameter_handles() noexcept;
    bool refresh_parameters() noexcept;
    bool refresh_manual_control() noexcept;
    bool refresh_actuator_output_status() noexcept;
    bool evaluate_safety(std::uint64_t now) noexcept;
    bool update_public_projection(std::uint64_t now) noexcept;
    bool execute_action(const action_request_s &request,
                        std::uint64_t now) noexcept;
    TransitionResult arm(std::uint8_t reason, std::uint64_t now) noexcept;
    TransitionResult disarm(std::uint8_t reason) noexcept;
    bool rc_input_valid(std::uint64_t now) const noexcept;
    bool sticks_centered() const noexcept;
    bool actuator_output_status_fresh(std::uint64_t now) const noexcept;
    bool actuator_output_mapping_valid() const noexcept;
    bool actuator_output_ready_for_arming(std::uint64_t now) const noexcept;
    bool actuator_output_recovered_disarmed(std::uint64_t now) const noexcept;
    bool actuator_output_fault_while_armed(std::uint64_t now) const noexcept;
    bool preflight_checks_pass(std::uint64_t now) const noexcept;
    bool action_request_fresh(const action_request_s &request,
                              std::uint64_t now) const noexcept;
    bool publish_state(std::uint64_t now) noexcept;
    void reset_runtime_state() noexcept;
    void initialize_public_state(std::uint64_t now) noexcept;
    void initialize_disarmed_snapshot(std::uint64_t now) noexcept;
    bool handle_vehicle_command(std::uint64_t now) noexcept;
    void answer_command(const vehicle_command_s &command, std::uint8_t result,
                        std::uint64_t now,
                        std::uint32_t result_param2 = 0U) noexcept;
    bool handle_publication_failure(std::uint64_t now) noexcept;
    void handle_scheduling_failure(std::uint64_t now) noexcept;
    void enter_error(const char *reason) noexcept;
    static std::uint8_t reason_from_source(std::uint8_t source) noexcept;

    dima::platform::ArmedFlashCoordinator &armed_flash_;
    uORB::SubscriptionCallbackWorkItem action_request_subscription_{
        ORB_ID(action_request), *this};
    uORB::SubscriptionCallbackWorkItem manual_control_subscription_{
        ORB_ID(manual_control_setpoint), *this};
    uORB::SubscriptionCallbackWorkItem parameter_update_subscription_{
        ORB_ID(parameter_update), *this};
    uORB::SubscriptionCallbackWorkItem vehicle_command_subscription_{
        ORB_ID(vehicle_command), *this};
    uORB::SubscriptionData<actuator_output_status_s>
        actuator_output_status_subscription_{ORB_ID(actuator_output_status)};
    uORB::Publication<actuator_armed_s> actuator_armed_publication_{
        ORB_ID(actuator_armed)};
    uORB::Publication<vehicle_control_mode_s> vehicle_control_mode_publication_{
        ORB_ID(vehicle_control_mode)};
    uORB::Publication<vehicle_status_s> vehicle_status_publication_{
        ORB_ID(vehicle_status)};
    uORB::Publication<vehicle_command_ack_s> vehicle_command_ack_publication_{
        ORB_ID(vehicle_command_ack)};

    actuator_armed_s actuator_armed_{};
    vehicle_control_mode_s vehicle_control_mode_{};
    vehicle_status_s vehicle_status_{};
    manual_control_setpoint_s manual_control_setpoint_{};
    actuator_output_status_s actuator_output_status_{};
    param_t rc_loss_timeout_handle_{PARAM_INVALID};
    param_t arm_stick_deadzone_handle_{PARAM_INVALID};
    param_t rc_loss_action_handle_{PARAM_INVALID};
    param_t data_link_loss_action_handle_{PARAM_INVALID};
    float rc_loss_timeout_s_{0.5F};
    float arm_stick_deadzone_{0.1F};
    std::int32_t rc_loss_action_{6};
    std::int32_t data_link_loss_action_{0};
    std::uint64_t last_publish_time_{0U};
    std::uint32_t last_actuator_output_sequence_{0U};
    std::uint8_t recoverable_failsafe_causes_{FailsafeNone};
    dima::middleware::lifecycle::ModuleState state_{
        dima::middleware::lifecycle::ModuleState::Stopped};
    bool parameter_handles_ready_{false};
    bool parameters_valid_{false};
    bool have_manual_control_{false};
    bool actuator_output_status_valid_{false};
    bool termination_latched_{false};
};

} // namespace dima::modules::safety
