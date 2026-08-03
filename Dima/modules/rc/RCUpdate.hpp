/****************************************************************************
 * PX4-Autopilot v1.17.0 RCUpdate Rover subset adapted to Dima FreeRTOS.
 ****************************************************************************/
#pragma once

#include "input_rc.hpp"
#include "manual_control_switches.hpp"
#include "parameter_update.hpp"
#include "rc_channels.hpp"
#include "lifecycle/module_base.hpp"
#include "parameters/param.h"
#include "perf/perf_counter.h"
#include "uorb/Publication.hpp"
#include "work_queue/WorkQueue.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace dima::modules::rc {

class RCUpdate final : public dima::middleware::lifecycle::ModuleBase,
                       public px4::ScheduledWorkItem {
public:
    RCUpdate() noexcept;
    bool start() override;
    void stop() override;
    dima::middleware::lifecycle::ModuleState state() const override;

private:
    static constexpr std::size_t kChannelCount = input_rc_s::RC_INPUT_MAX_CHANNELS;
    static constexpr std::size_t kCalibrationFieldCount = 5U;
    static constexpr std::size_t kMappingCount = 13U;
    static constexpr std::uint32_t kPollIntervalUs = 20000U;

    struct Calibration {
        float minimum{0.0F};
        float trim{0.0F};
        float maximum{0.0F};
        float reverse{0.0F};
        float deadzone{0.0F};
    };

    enum class Mapping : std::size_t {
        Roll,
        Pitch,
        Throttle,
        Yaw,
        Arm,
        Kill,
        FlightMode,
        Aux1,
        Aux2,
        Aux3,
        Aux4,
        Aux5,
        Aux6,
    };

    void Run() override;
    bool initialize_parameter_handles() noexcept;
    bool load_parameters() noexcept;
    bool validate_mapping(std::int32_t value) const noexcept;
    float normalize(std::size_t channel, std::uint16_t raw) const noexcept;
    std::uint8_t effective_channel_count() const noexcept;
    void rebuild_functions(std::uint8_t channel_count) noexcept;
    void publish_current(std::uint64_t now_us) noexcept;
    void publish_lost(std::uint64_t now_us) noexcept;
    void publish_switches(std::uint64_t sample_time) noexcept;
    std::uint8_t switch_position(std::uint8_t function, float threshold) const noexcept;
    std::uint8_t mode_slot() const noexcept;
    bool switches_equal(const manual_control_switches_s &lhs,
                        const manual_control_switches_s &rhs) const noexcept;
    void set_signal_lost(bool lost) noexcept;

    uORB::SubscriptionCallbackWorkItem input_rc_sub_{ORB_ID(input_rc), *this};
    uORB::SubscriptionCallbackWorkItem parameter_update_sub_{ORB_ID(parameter_update), *this};
    uORB::Publication<rc_channels_s> rc_channels_pub_{ORB_ID(rc_channels)};
    uORB::Publication<manual_control_switches_s> switches_pub_{ORB_ID(manual_control_switches)};

    std::array<std::array<param_t, kCalibrationFieldCount>, kChannelCount> calibration_handles_{};
    std::array<param_t, kMappingCount> mapping_handles_{};
    param_t channel_count_handle_{PARAM_INVALID};
    param_t arm_threshold_handle_{PARAM_INVALID};
    param_t kill_threshold_handle_{PARAM_INVALID};
    param_t loss_timeout_handle_{PARAM_INVALID};

    std::array<Calibration, kChannelCount> calibration_{};
    std::array<bool, kChannelCount> calibration_valid_{};
    std::array<std::int32_t, kMappingCount> mappings_{};
    std::array<bool, kMappingCount> mapping_runtime_invalid_reported_{};
    input_rc_s latest_input_{};
    rc_channels_s rc_{};
    manual_control_switches_s last_switches_{};

    std::int32_t configured_channel_count_{0};
    float arm_threshold_{0.75F};
    float kill_threshold_{0.75F};
    float loss_timeout_s_{0.5F};
    std::uint64_t last_input_time_us_{0U};
    std::uint64_t last_valid_time_us_{0U};
    dima::middleware::lifecycle::ModuleState state_{dima::middleware::lifecycle::ModuleState::Stopped};
    bool parameter_handles_ready_{false};
    bool parameters_valid_{false};
    bool have_input_{false};
    bool output_published_{false};
    bool signal_lost_{true};
    perf_counter_t publish_interval_{nullptr};
    bool switches_initialized_{false};
};

} // namespace dima::modules::rc
