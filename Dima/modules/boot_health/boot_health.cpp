#include "boot_health.hpp"

#include "mcuboot/mcuboot_app.h"
#include "freertos/platform_time.hpp"
#include "parameters/param.h"

namespace dima::modules::boot_health {

namespace {

uint64_t production_time_ms(void *)
{
    return dima::platform::platform_time_ms();
}

int production_confirm_running_image(void *)
{
    return mcuboot_confirm_running_image();
}

} // namespace

BootHealthService::BootHealthService() noexcept
    : px4::ScheduledWorkItem("boot_health", px4::wq_configurations::hp_default),
      heartbeat_subscription_(ORB_ID(app_heartbeat)),
      time_ms_(&production_time_ms),
      confirm_running_image_(&production_confirm_running_image)
{
}

void BootHealthService::bind_commander(
    const dima::middleware::lifecycle::ModuleBase &commander) noexcept
{
    commander_ = &commander;
}

bool BootHealthService::start()
{
    if (state_ == dima::middleware::lifecycle::ModuleState::Error) {
        return false;
    }
    if (state_ == dima::middleware::lifecycle::ModuleState::Running) {
        return true;
    }
    if (confirmation_attempted_) {
        state_ = dima::middleware::lifecycle::ModuleState::Running;
        return true;
    }
    if (commander_ == nullptr) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        return false;
    }
    if (!ScheduleEnable()) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        return false;
    }

    stable_window_start_ms_ = time_ms_(dependency_context_);
    heartbeat_observed_ = false;
    stable_window_active_ = false;
    last_heartbeat_progress_ms_ = 0U;
    last_heartbeat_timestamp_us_ = 0U;
    last_heartbeat_sequence_ = 0U;
    (void)heartbeat_subscription_.update();
    (void)actuator_armed_subscription_.update();
    (void)vehicle_control_mode_subscription_.update();
    (void)vehicle_status_subscription_.update();
    const bool scheduled = ScheduleOnInterval(kCheckIntervalMs * 1000U);
    if (!scheduled) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        ScheduleCancelAndDrain();
        return false;
    }

    state_ = dima::middleware::lifecycle::ModuleState::Running;
    return true;
}

void BootHealthService::stop()
{
    if (state_ != dima::middleware::lifecycle::ModuleState::Error) {
        state_ = dima::middleware::lifecycle::ModuleState::Stopped;
    }
    ScheduleCancelAndDrain();
    stable_window_active_ = false;
}

dima::middleware::lifecycle::ModuleState BootHealthService::state() const
{
    return state_;
}

void BootHealthService::Run()
{
    if (state_ != dima::middleware::lifecycle::ModuleState::Running ||
        confirmation_attempted_) {
        return;
    }

    const uint64_t now_us = dima::platform::platform_time_us();
    const uint64_t now_ms = time_ms_(dependency_context_);
    (void)actuator_armed_subscription_.update();
    (void)vehicle_control_mode_subscription_.update();
    (void)vehicle_status_subscription_.update();

    const bool healthy = param_is_ready() && commander_ != nullptr &&
                         commander_->state() ==
                             dima::middleware::lifecycle::ModuleState::Running &&
                         update_heartbeat_health(now_ms, now_us) &&
                         safety_topics_consistent(now_us);
    if (!healthy) {
        reset_stable_window(now_ms);
        return;
    }

    if (!stable_window_active_) {
        stable_window_active_ = true;
        stable_window_start_ms_ = now_ms;
        return;
    }

    if (now_ms - stable_window_start_ms_ < kStableWindowMs ||
        actuator_armed_subscription_.get().armed) {
        return;
    }

    const int result = confirm_running_image_(dependency_context_);
    switch (result) {
    case MCUBOOT_CONFIRM_OK:
    case MCUBOOT_CONFIRM_ALREADY_CONFIRMED:
    case MCUBOOT_CONFIRM_NOT_A_TEST_IMAGE:
        confirmation_attempted_ = true;
        ScheduleCancelAndDrain();
        break;
    case MCUBOOT_CONFIRM_DEFERRED:
        break;
    case MCUBOOT_CONFIRM_FLASH_ERROR:
    default:
        confirmation_attempted_ = true;
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        ScheduleCancelAndDrain();
        break;
    }
}

bool BootHealthService::update_heartbeat_health(uint64_t now_ms,
                                                uint64_t now_us) noexcept
{
    if (heartbeat_subscription_.update()) {
        const app_heartbeat_s &heartbeat = heartbeat_subscription_.get();
        const bool timestamp_valid = heartbeat.timestamp_us != 0U &&
                                     heartbeat.timestamp_us <= now_us &&
                                     now_us - heartbeat.timestamp_us <=
                                         kHeartbeatTimeoutMs * 1000ULL;
        const bool sequence_valid =
            !heartbeat_observed_ ||
            (heartbeat.sequence == last_heartbeat_sequence_ + 1U &&
             heartbeat.timestamp_us > last_heartbeat_timestamp_us_);
        if (!timestamp_valid || !sequence_valid) {
            heartbeat_observed_ = false;
            last_heartbeat_progress_ms_ = 0U;
            last_heartbeat_timestamp_us_ = 0U;
            last_heartbeat_sequence_ = 0U;
            return false;
        }

        heartbeat_observed_ = true;
        last_heartbeat_progress_ms_ = now_ms;
        last_heartbeat_timestamp_us_ = heartbeat.timestamp_us;
        last_heartbeat_sequence_ = heartbeat.sequence;
    }

    return heartbeat_observed_ && last_heartbeat_progress_ms_ <= now_ms &&
           now_ms - last_heartbeat_progress_ms_ <= kHeartbeatTimeoutMs &&
           last_heartbeat_timestamp_us_ <= now_us &&
           now_us - last_heartbeat_timestamp_us_ <=
               kHeartbeatTimeoutMs * 1000ULL;
}

bool BootHealthService::safety_topics_consistent(uint64_t now_us) const noexcept
{
    const actuator_armed_s &armed = actuator_armed_subscription_.get();
    const vehicle_control_mode_s &control =
        vehicle_control_mode_subscription_.get();
    const vehicle_status_s &status = vehicle_status_subscription_.get();

    if (armed.timestamp == 0U || armed.timestamp != control.timestamp ||
        armed.timestamp != status.timestamp || armed.timestamp > now_us ||
        now_us - armed.timestamp > kSafetyTopicTimeoutUs) {
        return false;
    }

    const bool status_armed =
        status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
    const bool status_disarmed =
        status.arming_state == vehicle_status_s::ARMING_STATE_DISARMED;
    const bool manual =
        status.nav_state == vehicle_status_s::NAVIGATION_STATE_MANUAL;
    const bool termination =
        status.nav_state == vehicle_status_s::NAVIGATION_STATE_TERMINATION;
    const uint32_t manual_mask =
        1UL << vehicle_status_s::NAVIGATION_STATE_MANUAL;
    const uint32_t termination_mask =
        1UL << vehicle_status_s::NAVIGATION_STATE_TERMINATION;

    return (status_armed || status_disarmed) &&
           armed.armed == status_armed && control.flag_armed == armed.armed &&
           armed.ready_to_arm ==
               (status.pre_flight_checks_pass || armed.armed) &&
           !armed.prearmed && !armed.lockdown &&
           !armed.in_esc_calibration_mode &&
           armed.termination == termination &&
           (!termination || status.failsafe) && (manual || termination) &&
           control.flag_control_manual_enabled == manual &&
           control.flag_control_termination_enabled == termination &&
           control.source_id == status.nav_state &&
           !control.flag_multicopter_position_control_enabled &&
           !control.flag_control_auto_enabled &&
           !control.flag_control_offboard_enabled &&
           !control.flag_control_position_enabled &&
           !control.flag_control_velocity_enabled &&
           !control.flag_control_altitude_enabled &&
           !control.flag_control_climb_rate_enabled &&
           !control.flag_control_acceleration_enabled &&
           !control.flag_control_attitude_enabled &&
           !control.flag_control_rates_enabled &&
           !control.flag_control_allocation_enabled &&
           status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROVER &&
           status.valid_nav_states_mask == (manual_mask | termination_mask) &&
           status.can_set_nav_states_mask == manual_mask;
}

void BootHealthService::reset_stable_window(uint64_t now_ms) noexcept
{
    stable_window_active_ = false;
    stable_window_start_ms_ = now_ms;
}

} // namespace dima::modules::boot_health
