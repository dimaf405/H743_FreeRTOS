/****************************************************************************
 * PX4-Autopilot v1.17.0 RCUpdate Rover subset adapted to the Dima platform.
 ****************************************************************************/
#include "RCUpdate.hpp"

#include "events/events.hpp"
#include "logging/logging.hpp"
#include "platform/api/Time.hpp"

#include <cmath>
#include <limits>

namespace dima::modules::rc {
namespace {

constexpr const char *kCalibrationNames[18][5] = {
#define RC_CAL_NAMES(n) {"RC" #n "_MIN", "RC" #n "_TRIM", "RC" #n "_MAX", "RC" #n "_REV", "RC" #n "_DZ"}
    RC_CAL_NAMES(1), RC_CAL_NAMES(2), RC_CAL_NAMES(3), RC_CAL_NAMES(4),
    RC_CAL_NAMES(5), RC_CAL_NAMES(6), RC_CAL_NAMES(7), RC_CAL_NAMES(8),
    RC_CAL_NAMES(9), RC_CAL_NAMES(10), RC_CAL_NAMES(11), RC_CAL_NAMES(12),
    RC_CAL_NAMES(13), RC_CAL_NAMES(14), RC_CAL_NAMES(15), RC_CAL_NAMES(16),
    RC_CAL_NAMES(17), RC_CAL_NAMES(18),
#undef RC_CAL_NAMES
};

constexpr std::uint32_t kEventInvalidCalibration = 0x52435501U;
constexpr std::uint32_t kEventInvalidMapping = 0x52435502U;
constexpr std::uint32_t kEventSignalState = 0x52435503U;

constexpr const char *kMappingNames[13] = {
    "RC_MAP_ROLL", "RC_MAP_PITCH", "RC_MAP_THROTTLE", "RC_MAP_YAW",
    "RC_MAP_ARM_SW", "RC_MAP_KILL_SW", "RC_MAP_FLAPS",
    "RC_MAP_AUX1", "RC_MAP_AUX2", "RC_MAP_AUX3", "RC_MAP_AUX4",
    "RC_MAP_AUX5", "RC_MAP_AUX6",
};

float constrain(float value, float minimum, float maximum) noexcept
{
    return value < minimum ? minimum : (value > maximum ? maximum : value);
}

} // namespace

RCUpdate::RCUpdate() noexcept
    : px4::ScheduledWorkItem("rc_update", px4::wq_configurations::io)
{
    reset_runtime_state();
}

bool RCUpdate::start()
{
    if (state_ == dima::middleware::lifecycle::ModuleState::Running) return true;
    if (!ScheduleEnable()) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        return false;
    }

    reset_runtime_state();
    parameter_handles_ready_ = initialize_parameter_handles();
    parameters_valid_ = parameter_handles_ready_ && load_parameters();
    publish_interval_ = perf_alloc(PC_INTERVAL, "rc_update:rc_channels_interval");

    if (!input_rc_sub_.registerCallback()) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        ScheduleCancelAndDrain();
        perf_free(publish_interval_);
        publish_interval_ = nullptr;
        PX4_ERR("RCUpdate input_rc callback registration failed");
        return false;
    }

    if (!parameter_update_sub_.registerCallback()) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        input_rc_sub_.unregisterCallback();
        ScheduleCancelAndDrain();
        perf_free(publish_interval_);
        publish_interval_ = nullptr;
        PX4_ERR("RCUpdate parameter callback registration failed");
        return false;
    }

    state_ = dima::middleware::lifecycle::ModuleState::Running;
    output_published_ = false;
    signal_lost_ = true;
    switches_initialized_ = false;

    if (!ScheduleOnInterval(kPollIntervalUs)) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        parameter_update_sub_.unregisterCallback();
        input_rc_sub_.unregisterCallback();
        ScheduleCancelAndDrain();
        perf_free(publish_interval_);
        publish_interval_ = nullptr;
        PX4_ERR("RCUpdate scheduling failed");
        return false;
    }

    return true;
}

void RCUpdate::stop()
{
    state_ = dima::middleware::lifecycle::ModuleState::Stopped;
    input_rc_sub_.unregisterCallback();
    parameter_update_sub_.unregisterCallback();
    ScheduleCancelAndDrain();
    perf_free(publish_interval_);
    publish_interval_ = nullptr;
    reset_runtime_state();
}

dima::middleware::lifecycle::ModuleState RCUpdate::state() const { return state_; }

void RCUpdate::Run()
{
    if (state_ != dima::middleware::lifecycle::ModuleState::Running) return;

    bool parameters_changed = false;
    parameter_update_s parameter_update{};

    if (parameter_update_sub_.copy(&parameter_update)) {
        const bool was_valid = parameters_valid_;
        parameters_valid_ = parameter_handles_ready_ && load_parameters();
        parameters_changed = true;

        if (parameters_valid_ != was_valid) {
            if (parameters_valid_) PX4_INFO("RCUpdate parameters valid");
            else PX4_ERR("RCUpdate parameters invalid");
        }
    }

    bool input_changed = false;
    input_rc_s input{};

    if (input_rc_sub_.copy(&input)) {
        latest_input_ = input;
        have_input_ = true;
        last_input_time_us_ = input.timestamp_last_signal != 0U
                                  ? input.timestamp_last_signal
                                  : input.timestamp;
        input_changed = true;
    }

    const std::uint64_t now_us = hrt_absolute_time();
    const std::uint64_t timeout_us = static_cast<std::uint64_t>(loss_timeout_s_ * 1000000.0F);
    const bool timed_out = !have_input_ || timeout_us == 0U ||
                           last_input_time_us_ == 0U ||
                           last_input_time_us_ > now_us ||
                           (now_us - last_input_time_us_) > timeout_us;
    const std::uint8_t required_channels = configured_channel_count_ > 0 ?
                                           static_cast<std::uint8_t>(configured_channel_count_) : 1U;
    // SBUS frame-lost 位只计入丢帧统计；Failsafe 或持续无新帧才切换为失联。
    const bool invalid_input = !have_input_ || latest_input_.rc_failsafe || latest_input_.rc_lost ||
                               latest_input_.channel_count < required_channels ||
                               latest_input_.channel_count > input_rc_s::RC_INPUT_MAX_CHANNELS;
    const bool unhealthy = !parameters_valid_ || timed_out || invalid_input;

    if (unhealthy) {
        recovery_start_time_us_ = 0U;
        if (input_changed || parameters_changed || !output_published_ || !signal_lost_) {
            publish_lost(now_us);
        }
        // uORB callback 的 ScheduleNow 会覆盖周期字段，因此每次运行后重新建立超时轮询。
        (void)ScheduleOnInterval(kPollIntervalUs);
        return;
    }

    if (signal_lost_) {
        if (recovery_start_time_us_ == 0U ||
            last_input_time_us_ < recovery_start_time_us_) {
            recovery_start_time_us_ = last_input_time_us_;
        }
        const bool recovery_stable = input_changed &&
            last_input_time_us_ >= recovery_start_time_us_ &&
            last_input_time_us_ - recovery_start_time_us_ >=
                kRecoveryStableUs;
        if (!recovery_stable) {
            if (input_changed || parameters_changed || !output_published_) {
                publish_lost(now_us);
            }
            (void)ScheduleOnInterval(kPollIntervalUs);
            return;
        }
        recovery_start_time_us_ = 0U;
    }

    if (input_changed || parameters_changed || signal_lost_) {
        publish_current(now_us);
    }
    (void)ScheduleOnInterval(kPollIntervalUs);
}

void RCUpdate::reset_runtime_state() noexcept
{
    for (auto &channel : calibration_handles_) {
        channel.fill(PARAM_INVALID);
    }
    mapping_handles_.fill(PARAM_INVALID);
    channel_count_handle_ = PARAM_INVALID;
    arm_threshold_handle_ = PARAM_INVALID;
    kill_threshold_handle_ = PARAM_INVALID;
    loss_timeout_handle_ = PARAM_INVALID;
    rc_input_mode_handle_ = PARAM_INVALID;

    calibration_.fill(Calibration{});
    calibration_valid_.fill(false);
    mappings_.fill(0);
    mapping_runtime_invalid_reported_.fill(false);
    latest_input_ = input_rc_s{};
    rc_ = rc_channels_s{};
    for (float &channel : rc_.channels) {
        channel = std::numeric_limits<float>::quiet_NaN();
    }
    for (std::int8_t &function : rc_.function) {
        function = -1;
    }
    last_switches_ = manual_control_switches_s{};

    configured_channel_count_ = 0;
    arm_threshold_ = 0.75F;
    kill_threshold_ = 0.75F;
    loss_timeout_s_ = 0.5F;
    rc_input_mode_ = 0;
    last_input_time_us_ = 0U;
    last_valid_time_us_ = 0U;
    recovery_start_time_us_ = 0U;
    parameter_handles_ready_ = false;
    parameters_valid_ = false;
    have_input_ = false;
    output_published_ = false;
    signal_lost_ = true;
    switches_initialized_ = false;
}

bool RCUpdate::initialize_parameter_handles() noexcept
{
    bool valid = true;

    for (std::size_t channel = 0U; channel < kChannelCount; ++channel) {
        for (std::size_t field = 0U; field < kCalibrationFieldCount; ++field) {
            calibration_handles_[channel][field] = param_find(kCalibrationNames[channel][field]);
            valid = valid && calibration_handles_[channel][field] != PARAM_INVALID;
        }
    }

    for (std::size_t mapping = 0U; mapping < kMappingCount; ++mapping) {
        mapping_handles_[mapping] = param_find(kMappingNames[mapping]);
        valid = valid && mapping_handles_[mapping] != PARAM_INVALID;
    }

    channel_count_handle_ = param_find("RC_CHAN_CNT");
    arm_threshold_handle_ = param_find("RC_ARMSWITCH_TH");
    kill_threshold_handle_ = param_find("RC_KILLSWITCH_TH");
    loss_timeout_handle_ = param_find("COM_RC_LOSS_T");
    rc_input_mode_handle_ = param_find("COM_RC_IN_MODE");
    return valid && channel_count_handle_ != PARAM_INVALID &&
           arm_threshold_handle_ != PARAM_INVALID && kill_threshold_handle_ != PARAM_INVALID &&
           loss_timeout_handle_ != PARAM_INVALID &&
           rc_input_mode_handle_ != PARAM_INVALID;
}

bool RCUpdate::load_parameters() noexcept
{
    bool valid = true;
    valid = valid && param_get(channel_count_handle_, &configured_channel_count_) == 0;
    valid = valid && param_get(arm_threshold_handle_, &arm_threshold_) == 0;
    valid = valid && param_get(kill_threshold_handle_, &kill_threshold_) == 0;
    valid = valid && param_get(loss_timeout_handle_, &loss_timeout_s_) == 0;
    valid = valid && param_get(rc_input_mode_handle_, &rc_input_mode_) == 0;

    for (std::size_t channel = 0U; channel < kChannelCount; ++channel) {
        Calibration loaded{};
        valid = valid && param_get(calibration_handles_[channel][0], &loaded.minimum) == 0;
        valid = valid && param_get(calibration_handles_[channel][1], &loaded.trim) == 0;
        valid = valid && param_get(calibration_handles_[channel][2], &loaded.maximum) == 0;
        valid = valid && param_get(calibration_handles_[channel][3], &loaded.reverse) == 0;
        valid = valid && param_get(calibration_handles_[channel][4], &loaded.deadzone) == 0;
        calibration_[channel] = loaded;
    }

    for (std::size_t mapping = 0U; mapping < kMappingCount; ++mapping) {
        valid = valid && param_get(mapping_handles_[mapping], &mappings_[mapping]) == 0;
    }

    valid = valid && configured_channel_count_ >= 0 &&
            configured_channel_count_ <= static_cast<std::int32_t>(kChannelCount) &&
            std::isfinite(arm_threshold_) && arm_threshold_ >= -1.0F && arm_threshold_ <= 1.0F &&
            std::isfinite(kill_threshold_) && kill_threshold_ >= 0.0F && kill_threshold_ <= 1.0F &&
            std::isfinite(loss_timeout_s_) && loss_timeout_s_ >= 0.1F &&
            loss_timeout_s_ <= 35.0F && rc_input_mode_ == 0;

    for (std::size_t channel = 0U; channel < kChannelCount; ++channel) {
        const Calibration &cal = calibration_[channel];
        calibration_valid_[channel] = std::isfinite(cal.minimum) && std::isfinite(cal.trim) &&
                                      std::isfinite(cal.maximum) && std::isfinite(cal.reverse) &&
                                      std::isfinite(cal.deadzone) && cal.deadzone >= 0.0F &&
                                      cal.minimum < (cal.trim - cal.deadzone) &&
                                      (cal.trim + cal.deadzone) < cal.maximum &&
                                      (cal.reverse == -1.0F || cal.reverse == 1.0F);
        if (!calibration_valid_[channel]) {
            const std::uint32_t argument = static_cast<std::uint32_t>(channel + 1U);
            (void)dima::events::report(kEventInvalidCalibration,
                                       dima::events::Severity::Warning, &argument, 1U);
        }
    }

    mapping_runtime_invalid_reported_.fill(false);
    for (std::size_t mapping = 0U; mapping < mappings_.size(); ++mapping) {
        if (!validate_mapping(mappings_[mapping])) {
            valid = false;
            const std::uint32_t arguments[2]{static_cast<std::uint32_t>(mapping),
                                             static_cast<std::uint32_t>(mappings_[mapping])};
            (void)dima::events::report(kEventInvalidMapping,
                                       dima::events::Severity::Warning, arguments, 2U);
        }
    }

    return valid;
}

bool RCUpdate::validate_mapping(std::int32_t value) const noexcept
{
    return value >= 0 && value <= static_cast<std::int32_t>(kChannelCount);
}

float RCUpdate::normalize(std::size_t channel, std::uint16_t raw) const noexcept
{
    const Calibration &cal = calibration_[channel];
    const float value = static_cast<float>(raw);
    float normalized = 0.0F;

    // 死区两侧分别线性映射，所有轴（含油门）保持中心双向语义。
    if (value > cal.trim + cal.deadzone) {
        normalized = (value - cal.trim - cal.deadzone) /
                     (cal.maximum - cal.trim - cal.deadzone);
    } else if (value < cal.trim - cal.deadzone) {
        normalized = (value - cal.trim + cal.deadzone) /
                     (cal.trim - cal.deadzone - cal.minimum);
    }

    return constrain(normalized * cal.reverse, -1.0F, 1.0F);
}

std::uint8_t RCUpdate::effective_channel_count() const noexcept
{
    const std::uint8_t received = latest_input_.channel_count > input_rc_s::RC_INPUT_MAX_CHANNELS ?
                                  input_rc_s::RC_INPUT_MAX_CHANNELS : latest_input_.channel_count;
    return configured_channel_count_ > 0 ?
           static_cast<std::uint8_t>(configured_channel_count_) : received;
}

void RCUpdate::rebuild_functions(std::uint8_t channel_count) noexcept
{
    for (std::int8_t &function : rc_.function) function = -1;

    const auto assign = [this, channel_count](Mapping mapping, std::uint8_t function) {
        const std::size_t mapping_index = static_cast<std::size_t>(mapping);
        const std::int32_t value = mappings_[mapping_index];
        if (value <= 0) return;

        const std::size_t channel = static_cast<std::size_t>(value - 1);
        if (value <= channel_count && calibration_valid_[channel]) {
            rc_.function[function] = static_cast<std::int8_t>(channel);
            mapping_runtime_invalid_reported_[mapping_index] = false;
        } else if (!mapping_runtime_invalid_reported_[mapping_index]) {
            const std::uint32_t arguments[2]{static_cast<std::uint32_t>(mapping_index),
                                             static_cast<std::uint32_t>(value)};
            (void)dima::events::report(kEventInvalidMapping,
                                       dima::events::Severity::Warning, arguments, 2U);
            mapping_runtime_invalid_reported_[mapping_index] = true;
        }
    };

    assign(Mapping::Roll, rc_channels_s::FUNCTION_ROLL);
    assign(Mapping::Pitch, rc_channels_s::FUNCTION_PITCH);
    assign(Mapping::Throttle, rc_channels_s::FUNCTION_THROTTLE);
    assign(Mapping::Yaw, rc_channels_s::FUNCTION_YAW);
    assign(Mapping::Arm, rc_channels_s::FUNCTION_ARMSWITCH);
    assign(Mapping::Kill, rc_channels_s::FUNCTION_KILLSWITCH);
    assign(Mapping::Flaps, rc_channels_s::FUNCTION_FLAPS);
    assign(Mapping::Aux1, rc_channels_s::FUNCTION_AUX_1);
    assign(Mapping::Aux2, rc_channels_s::FUNCTION_AUX_2);
    assign(Mapping::Aux3, rc_channels_s::FUNCTION_AUX_3);
    assign(Mapping::Aux4, rc_channels_s::FUNCTION_AUX_4);
    assign(Mapping::Aux5, rc_channels_s::FUNCTION_AUX_5);
    assign(Mapping::Aux6, rc_channels_s::FUNCTION_AUX_6);
}

void RCUpdate::publish_current(std::uint64_t now_us) noexcept
{
    rc_.timestamp = now_us;
    rc_.timestamp_last_valid = latest_input_.timestamp_last_signal != 0U ?
                               latest_input_.timestamp_last_signal : latest_input_.timestamp;
    last_valid_time_us_ = rc_.timestamp_last_valid;
    rc_.channel_count = effective_channel_count();
    rebuild_functions(rc_.channel_count);
    rc_.rssi = latest_input_.rssi >= 0 ?
               static_cast<std::uint8_t>(latest_input_.rssi > 100 ? 100 : latest_input_.rssi) : 0U;
    rc_.signal_lost = false;
    rc_.frame_drop_count = latest_input_.rc_lost_frame_count;

    for (std::size_t channel = 0U; channel < kChannelCount; ++channel) {
        rc_.channels[channel] = channel < rc_.channel_count && calibration_valid_[channel] ?
                                normalize(channel, latest_input_.values[channel]) :
                                std::numeric_limits<float>::quiet_NaN();
    }

    (void)rc_channels_pub_.publish(rc_);
    perf_count(publish_interval_);
    publish_switches(rc_.timestamp_last_valid);
    output_published_ = true;
    set_signal_lost(false);
}

void RCUpdate::publish_lost(std::uint64_t now_us) noexcept
{
    rc_.timestamp = now_us;
    rc_.timestamp_last_valid = last_valid_time_us_;
    rc_.channel_count = have_input_ ? effective_channel_count() : 0U;
    if (have_input_) {
        rebuild_functions(rc_.channel_count);
    } else {
        for (std::int8_t &function : rc_.function) function = -1;
    }
    rc_.rssi = 0U;
    rc_.signal_lost = true;
    rc_.frame_drop_count = have_input_ ? latest_input_.rc_lost_frame_count : 0U;
    for (float &channel : rc_.channels) channel = std::numeric_limits<float>::quiet_NaN();
    (void)rc_channels_pub_.publish(rc_);
    perf_count(publish_interval_);

    manual_control_switches_s switches{};
    switches.timestamp = now_us;
    switches.timestamp_sample = have_input_ ? latest_input_.timestamp : now_us;
    switches.switch_changes = last_switches_.switch_changes;
    (void)switches_pub_.publish(switches);
    last_switches_ = switches;
    switches_initialized_ = false;
    output_published_ = true;
    set_signal_lost(true);
}

void RCUpdate::publish_switches(std::uint64_t sample_time) noexcept
{
    manual_control_switches_s switches{};
    switches.timestamp = hrt_absolute_time();
    switches.timestamp_sample = sample_time;
    switches.arm_switch = switch_position(rc_channels_s::FUNCTION_ARMSWITCH, arm_threshold_);
    switches.kill_switch = switch_position(rc_channels_s::FUNCTION_KILLSWITCH, kill_threshold_);
    switches.photo_switch = switch_position(rc_channels_s::FUNCTION_AUX_3, 0.5F);
    switches.video_switch = switch_position(rc_channels_s::FUNCTION_AUX_4, 0.5F);
    switches.switch_changes = last_switches_.switch_changes;

    if (switches_initialized_ && !switches_equal(switches, last_switches_)) {
        ++switches.switch_changes;
    }

    (void)switches_pub_.publish(switches);
    last_switches_ = switches;
    switches_initialized_ = true;
}

std::uint8_t RCUpdate::switch_position(std::uint8_t function, float threshold) const noexcept
{
    const std::int8_t channel = rc_.function[function];
    if (channel < 0) return manual_control_switches_s::SWITCH_POS_NONE;

    float value = 0.5F * rc_.channels[static_cast<std::size_t>(channel)] + 0.5F;
    if (threshold < 0.0F) value = -value;
    const bool active = threshold == 1.0F ? value >= threshold
                                          : value > threshold;
    return active ? manual_control_switches_s::SWITCH_POS_ON :
                    manual_control_switches_s::SWITCH_POS_OFF;
}

bool RCUpdate::switches_equal(const manual_control_switches_s &lhs,
                              const manual_control_switches_s &rhs) const noexcept
{
    return lhs.mode_slot == rhs.mode_slot && lhs.arm_switch == rhs.arm_switch &&
           lhs.return_switch == rhs.return_switch && lhs.loiter_switch == rhs.loiter_switch &&
           lhs.offboard_switch == rhs.offboard_switch && lhs.kill_switch == rhs.kill_switch &&
           lhs.termination_switch == rhs.termination_switch && lhs.gear_switch == rhs.gear_switch &&
           lhs.transition_switch == rhs.transition_switch && lhs.photo_switch == rhs.photo_switch &&
           lhs.video_switch == rhs.video_switch &&
           lhs.engage_main_motor_switch == rhs.engage_main_motor_switch &&
           lhs.payload_power_switch == rhs.payload_power_switch;
}

void RCUpdate::set_signal_lost(bool lost) noexcept
{
    if (lost == signal_lost_) return;
    signal_lost_ = lost;
    const std::uint32_t active = lost ? 1U : 0U;
    if (lost) PX4_WARN("RC signal lost");
    else PX4_INFO("RC signal restored");
    (void)dima::events::report(kEventSignalState,
                               lost ? dima::events::Severity::Warning
                                    : dima::events::Severity::Info,
                               &active, 1U);
}

} // namespace dima::modules::rc
