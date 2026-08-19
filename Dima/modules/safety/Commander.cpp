/****************************************************************************
 * PX4-Autopilot v1.17.0 Commander Rover subset adapted to the Dima platform.
 ****************************************************************************/
#define MODULE_NAME "commander"
#include "Commander.hpp"

#include "logging/logging.hpp"
#include "platform/api/Time.hpp"

namespace dima::modules::safety {

Commander::Commander(
    dima::platform::ArmedFlashCoordinator &armed_flash) noexcept
    : px4::ScheduledWorkItem("commander", px4::wq_configurations::hp_default),
      armed_flash_(armed_flash)
{
}

Commander::~Commander()
{
    stop();
}

bool Commander::start()
{
    if (state_ == dima::middleware::lifecycle::ModuleState::Running) {
        return true;
    }
    if (!ScheduleEnable()) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        PX4_ERR("Commander WorkQueue unavailable");
        return false;
    }

    reset_runtime_state();
    armed_flash_.disarm();
    parameter_handles_ready_ = initialize_parameter_handles();
    if (!parameter_handles_ready_) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        ScheduleCancelAndDrain();
        PX4_ERR("Commander parameter handles unavailable");
        return false;
    }

    (void)refresh_parameters();

    if (!action_request_subscription_.registerCallback()) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        ScheduleCancelAndDrain();
        PX4_ERR("Commander action callback registration failed");
        return false;
    }
    if (!manual_control_subscription_.registerCallback()) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        action_request_subscription_.unregisterCallback();
        ScheduleCancelAndDrain();
        PX4_ERR("Commander manual callback registration failed");
        return false;
    }
    if (!parameter_update_subscription_.registerCallback()) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        manual_control_subscription_.unregisterCallback();
        action_request_subscription_.unregisterCallback();
        ScheduleCancelAndDrain();
        PX4_ERR("Commander parameter callback registration failed");
        return false;
    }
    if (!vehicle_command_subscription_.registerCallback()) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        parameter_update_subscription_.unregisterCallback();
        manual_control_subscription_.unregisterCallback();
        action_request_subscription_.unregisterCallback();
        ScheduleCancelAndDrain();
        PX4_ERR("Commander vehicle_command callback registration failed");
        return false;
    }

    const std::uint64_t now = hrt_absolute_time();
    initialize_public_state(now);
    state_ = dima::middleware::lifecycle::ModuleState::Running;

    if (!publish_state(now)) {
        return handle_publication_failure(now);
    }
    if (!ScheduleOnInterval(kCheckIntervalUs)) {
        handle_scheduling_failure(now);
        return false;
    }

    return true;
}

void Commander::stop()
{
    armed_flash_.disarm();
    state_ = dima::middleware::lifecycle::ModuleState::Stopped;
    vehicle_command_subscription_.unregisterCallback();
    parameter_update_subscription_.unregisterCallback();
    manual_control_subscription_.unregisterCallback();
    action_request_subscription_.unregisterCallback();
    ScheduleCancelAndDrain();
    armed_flash_.disarm();
    reset_runtime_state();
}

dima::middleware::lifecycle::ModuleState Commander::state() const
{
    return state_;
}

void Commander::Run()
{
    if (state_ != dima::middleware::lifecycle::ModuleState::Running) {
        return;
    }

    parameter_update_s parameter_update{};
    while (parameter_update_subscription_.copy(&parameter_update)) {
        // 深度为 1；循环形式仍保证未来扩大队列后不会只消费最后一条。
    }

    bool state_changed = refresh_parameters();
    (void)refresh_manual_control();
    (void)refresh_actuator_output_status();

    std::uint64_t now = hrt_absolute_time();
    state_changed = evaluate_safety(now) || state_changed;
    state_changed = update_public_projection(now) || state_changed;
    if (state_changed && !publish_state(now)) {
        (void)handle_publication_failure(now);
        return;
    }

    action_request_s request{};
    while (action_request_subscription_.copy(&request)) {
        now = hrt_absolute_time();
        bool action_changed = execute_action(request, now);
        action_changed = update_public_projection(now) || action_changed;

        // 每一个状态转换都完成一次固定顺序发布，保持 Arm/Kill 队列顺序。
        if (action_changed && !publish_state(now)) {
            (void)handle_publication_failure(now);
            return;
        }
    }

    /* ── External MAVLink vehicle_command ───────────────────────── */
    if (handle_vehicle_command(hrt_absolute_time())) {
        now = hrt_absolute_time();
        state_changed = true;
        state_changed = evaluate_safety(now) || state_changed;
        state_changed = update_public_projection(now) || state_changed;
        if (state_changed && !publish_state(now)) {
            (void)handle_publication_failure(now);
            return;
        }
    }

    now = hrt_absolute_time();
    state_changed = evaluate_safety(now);
    state_changed = update_public_projection(now) || state_changed;
    const bool heartbeat_due = now - last_publish_time_ >= kPublishIntervalUs;
    if ((state_changed || heartbeat_due) && !publish_state(now)) {
        (void)handle_publication_failure(now);
        return;
    }

    // uORB 回调的 ScheduleNow 会替换周期调度，每次运行后恢复 20 ms 检查。
    if (!ScheduleOnInterval(kCheckIntervalUs)) {
        handle_scheduling_failure(now);
    }
}

bool Commander::handle_publication_failure(std::uint64_t now) noexcept
{
    initialize_disarmed_snapshot(now);
    if (!publish_state(now)) {
        enter_error("Commander DISARMED state publication failed");
        return false;
    }

    PX4_WARN("Commander publication recovered with DISARMED snapshot");
    if (!ScheduleOnInterval(kCheckIntervalUs)) {
        handle_scheduling_failure(now);
        return false;
    }
    return true;
}

void Commander::handle_scheduling_failure(std::uint64_t now) noexcept
{
    initialize_disarmed_snapshot(now);
    (void)publish_state(now);
    enter_error("Commander scheduling failed");
}

void Commander::enter_error(const char *reason) noexcept
{
    state_ = dima::middleware::lifecycle::ModuleState::Error;
    armed_flash_.disarm();
    vehicle_command_subscription_.unregisterCallback();
    parameter_update_subscription_.unregisterCallback();
    manual_control_subscription_.unregisterCallback();
    action_request_subscription_.unregisterCallback();
    ScheduleCancelAndDrain();
    actuator_armed_.armed = false;
    vehicle_control_mode_.flag_armed = false;
    vehicle_status_.arming_state = vehicle_status_s::ARMING_STATE_DISARMED;
    armed_flash_.disarm();
    PX4_ERR("%s", reason);
}

} // namespace dima::modules::safety
