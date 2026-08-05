#include "ManualMotionAdapter.hpp"

#include "events/events.hpp"
#include "platform/api/Time.hpp"
#include "vehicle_status.hpp"

#include <cmath>
#include <limits>

namespace dima::rover::control {
namespace {

constexpr std::uint32_t kEventParameterInvalid = 0x524D4101U;
constexpr std::uint32_t kEventPublishFailure = 0x524D4102U;
constexpr std::uint32_t kEventScheduleFailure = 0x524D4103U;
constexpr float kUnavailable = std::numeric_limits<float>::quiet_NaN();

} // namespace

ManualMotionAdapter::ManualMotionAdapter() noexcept
    : px4::ScheduledWorkItem("manual_motion",
                             px4::wq_configurations::hp_default)
{
}

ManualMotionAdapter::~ManualMotionAdapter()
{
    stop();
}

bool ManualMotionAdapter::start()
{
    if (state_ == dima::middleware::lifecycle::ModuleState::Running) {
        return true;
    }
    if (!ScheduleEnable()) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        return false;
    }

    reset_runtime_state();
    if (!bind_parameters()) {
        enter_error(kEventParameterInvalid);
        return false;
    }
    if (!manual_control_subscription_.registerCallback()) {
        enter_error(kEventScheduleFailure);
        return false;
    }
    if (!vehicle_control_mode_subscription_.registerCallback()) {
        manual_control_subscription_.unregisterCallback();
        enter_error(kEventScheduleFailure);
        return false;
    }
    if (!parameter_update_subscription_.registerCallback()) {
        vehicle_control_mode_subscription_.unregisterCallback();
        manual_control_subscription_.unregisterCallback();
        enter_error(kEventScheduleFailure);
        return false;
    }

    state_ = dima::middleware::lifecycle::ModuleState::Running;
    if (!ScheduleNow()) {
        enter_error(kEventScheduleFailure);
        return false;
    }
    return true;
}

void ManualMotionAdapter::stop()
{
    state_ = dima::middleware::lifecycle::ModuleState::Stopped;
    parameter_update_subscription_.unregisterCallback();
    vehicle_control_mode_subscription_.unregisterCallback();
    manual_control_subscription_.unregisterCallback();
    ScheduleCancelAndDrain();

    const std::uint64_t now = hrt_absolute_time();
    manual_control_ = manual_control_setpoint_s{};
    have_manual_control_ = false;
    (void)publish_current_request(now);
    reset_runtime_state();
}

dima::middleware::lifecycle::ModuleState ManualMotionAdapter::state() const
{
    return state_;
}

void ManualMotionAdapter::Run()
{
    if (state_ != dima::middleware::lifecycle::ModuleState::Running) {
        return;
    }

    parameter_update_s parameter_update{};
    if (parameter_update_subscription_.copy(&parameter_update)) {
        parameter_update_pending_ = true;
    }

    vehicle_control_mode_s control_mode{};
    if (vehicle_control_mode_subscription_.copy(&control_mode)) {
        vehicle_control_mode_ = control_mode;
        have_control_mode_ = true;
    }

    manual_control_setpoint_s manual_control{};
    if (manual_control_subscription_.copy(&manual_control)) {
        manual_control_ = manual_control;
        have_manual_control_ = true;
    }

    const std::uint64_t now = hrt_absolute_time();
    (void)apply_pending_parameters(now);
    if (!publish_current_request(now)) {
        enter_error(kEventPublishFailure);
    }
}

bool ManualMotionAdapter::bind_parameters() noexcept
{
    const bool bound = yaw_stick_deadzone_.bind() && yaw_expo_.bind() &&
                       yaw_superexpo_.bind() && yaw_stick_gain_.bind();
    if (!bound || !apply_parameter_snapshot()) {
        invalidate_parameter_bindings();
        parameters_valid_ = false;
        return false;
    }
    return true;
}

void ManualMotionAdapter::invalidate_parameter_bindings() noexcept
{
    yaw_stick_deadzone_.invalidate();
    yaw_expo_.invalidate();
    yaw_superexpo_.invalidate();
    yaw_stick_gain_.invalidate();
}

bool ManualMotionAdapter::apply_parameter_snapshot() noexcept
{
    if (!yaw_stick_deadzone_.bound() || !yaw_expo_.bound() ||
        !yaw_superexpo_.bound() || !yaw_stick_gain_.bound()) {
        parameters_valid_ = false;
        return false;
    }

    Config candidate{};
    bool loaded = false;
    {
        px4::AtomicTransaction transaction;
        loaded = param_get(yaw_stick_deadzone_.handle(),
                           &candidate.yaw_stick_deadzone) == 0 &&
                 param_get(yaw_expo_.handle(), &candidate.yaw_expo) == 0 &&
                 param_get(yaw_superexpo_.handle(),
                           &candidate.yaw_superexpo) == 0 &&
                 param_get(yaw_stick_gain_.handle(),
                           &candidate.yaw_stick_gain) == 0;
    }
    if (!loaded || !valid_config(candidate)) {
        parameters_valid_ = false;
        return false;
    }

    yaw_stick_deadzone_.set(candidate.yaw_stick_deadzone);
    yaw_expo_.set(candidate.yaw_expo);
    yaw_superexpo_.set(candidate.yaw_superexpo);
    yaw_stick_gain_.set(candidate.yaw_stick_gain);
    config_ = candidate;
    parameters_valid_ = true;
    return true;
}

bool ManualMotionAdapter::apply_pending_parameters(
    std::uint64_t now_us) noexcept
{
    if (!parameter_update_pending_ || !fresh_disarmed_mode(now_us)) {
        return false;
    }

    parameter_update_pending_ = false;
    if (apply_parameter_snapshot()) {
        return true;
    }
    (void)dima::events::report(kEventParameterInvalid,
                               dima::events::Severity::Error);
    return false;
}

bool ManualMotionAdapter::fresh_disarmed_mode(std::uint64_t now_us) const noexcept
{
    return have_control_mode_ && vehicle_control_mode_.timestamp != 0U &&
           vehicle_control_mode_.timestamp <= now_us &&
           now_us - vehicle_control_mode_.timestamp <= kControlModeTimeoutUs &&
           !vehicle_control_mode_.flag_armed;
}

bool ManualMotionAdapter::manual_mode_active(std::uint64_t now_us) const noexcept
{
    return have_control_mode_ && vehicle_control_mode_.timestamp != 0U &&
           vehicle_control_mode_.timestamp <= now_us &&
           now_us - vehicle_control_mode_.timestamp <= kControlModeTimeoutUs &&
           vehicle_control_mode_.source_id ==
               vehicle_status_s::NAVIGATION_STATE_MANUAL &&
           vehicle_control_mode_.flag_control_manual_enabled &&
           !vehicle_control_mode_.flag_control_termination_enabled &&
           !vehicle_control_mode_.flag_control_auto_enabled &&
           !vehicle_control_mode_.flag_control_offboard_enabled;
}

bool ManualMotionAdapter::manual_input_valid(std::uint64_t now_us) const noexcept
{
    return have_manual_control_ && manual_control_.valid &&
           manual_control_.data_source == manual_control_setpoint_s::SOURCE_RC &&
           manual_control_.timestamp != 0U &&
           manual_control_.timestamp_sample != 0U &&
           manual_control_.timestamp_sample <= manual_control_.timestamp &&
           manual_control_.timestamp <= now_us &&
           finite(manual_control_.throttle) &&
           manual_control_.throttle >= -1.0F &&
           manual_control_.throttle <= 1.0F && finite(manual_control_.yaw) &&
           manual_control_.yaw >= -1.0F && manual_control_.yaw <= 1.0F;
}

bool ManualMotionAdapter::publish_current_request(
    std::uint64_t now_us) noexcept
{
    rover_motion_request_s request{};
    request.timestamp = now_us;
    request.timestamp_sample = manual_control_.timestamp_sample;
    request.sequence = ++sequence_;
    request.source = rover_motion_request_s::SOURCE_MANUAL;
    request.mode = rover_motion_request_s::MODE_NORMALIZED_AXES;
    request.speed_m_s = kUnavailable;
    request.yaw_rate_rad_s = kUnavailable;

    request.valid = state_ ==
                        dima::middleware::lifecycle::ModuleState::Running &&
                    parameters_valid_ && manual_mode_active(now_us) &&
                    manual_input_valid(now_us);
    if (request.valid) {
        request.normalized_longitudinal = manual_control_.throttle;
        const float yaw = deadzone(manual_control_.yaw,
                                   config_.yaw_stick_deadzone);
        request.normalized_steering = clamp(
            config_.yaw_stick_gain *
                superexpo(yaw, config_.yaw_expo, config_.yaw_superexpo),
            -1.0F, 1.0F);
    } else {
        request.normalized_longitudinal = kUnavailable;
        request.normalized_steering = kUnavailable;
    }
    return motion_request_publication_.publish(request);
}

void ManualMotionAdapter::reset_runtime_state() noexcept
{
    config_ = Config{};
    manual_control_ = manual_control_setpoint_s{};
    vehicle_control_mode_ = vehicle_control_mode_s{};
    sequence_ = 0U;
    have_manual_control_ = false;
    have_control_mode_ = false;
    parameters_valid_ = false;
    parameter_update_pending_ = false;
    invalidate_parameter_bindings();
}

void ManualMotionAdapter::enter_error(std::uint32_t event_id) noexcept
{
    state_ = dima::middleware::lifecycle::ModuleState::Error;
    parameter_update_subscription_.unregisterCallback();
    vehicle_control_mode_subscription_.unregisterCallback();
    manual_control_subscription_.unregisterCallback();
    ScheduleCancelAndDrain();
    (void)dima::events::report(event_id, dima::events::Severity::Error);
}

bool ManualMotionAdapter::finite(float value) noexcept
{
    return std::isfinite(value);
}

float ManualMotionAdapter::clamp(float value, float lower, float upper) noexcept
{
    return value < lower ? lower : (value > upper ? upper : value);
}

float ManualMotionAdapter::deadzone(float value, float width) noexcept
{
    const float input = clamp(value, -1.0F, 1.0F);
    const float bounded_width = clamp(width, 0.0F, 0.99F);
    if (std::fabs(input) <= bounded_width) {
        return 0.0F;
    }
    const float sign = input < 0.0F ? -1.0F : 1.0F;
    return (input - sign * bounded_width) / (1.0F - bounded_width);
}

float ManualMotionAdapter::superexpo(float value, float expo,
                                     float superexpo_value) noexcept
{
    const float input = clamp(value, -1.0F, 1.0F);
    const float bounded_expo = clamp(expo, 0.0F, 1.0F);
    const float bounded_superexpo = clamp(superexpo_value, 0.0F, 0.99F);
    const float expo_value = (1.0F - bounded_expo) * input +
                             bounded_expo * input * input * input;
    return expo_value * (1.0F - bounded_superexpo) /
           (1.0F - std::fabs(input) * bounded_superexpo);
}

bool ManualMotionAdapter::valid_config(const Config &config) noexcept
{
    return finite(config.yaw_stick_deadzone) &&
           config.yaw_stick_deadzone >= 0.0F &&
           config.yaw_stick_deadzone <= 1.0F && finite(config.yaw_expo) &&
           config.yaw_expo >= 0.0F && config.yaw_expo <= 1.0F &&
           finite(config.yaw_superexpo) && config.yaw_superexpo >= 0.0F &&
           config.yaw_superexpo <= 0.95F &&
           finite(config.yaw_stick_gain) && config.yaw_stick_gain >= 0.1F &&
           config.yaw_stick_gain <= 1.0F;
}

} // namespace dima::rover::control
