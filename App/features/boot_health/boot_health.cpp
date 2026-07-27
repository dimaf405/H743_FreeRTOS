#include "App/features/boot_health/boot_health.hpp"

#include "App/adapters/mcuboot/mcuboot_app.h"
#include "App/runtime/time/platform_time.hpp"

namespace app::features::boot_health {

#if !defined(APP_HOST_TEST)
namespace {

uint64_t production_time_ms(void *)
{
    return runtime::time::platform_time_ms();
}

int production_confirm_running_image(void *)
{
    return mcuboot_confirm_running_image();
}

} // namespace
#endif

#if defined(APP_HOST_TEST)
BootHealthService::BootHealthService(
    runtime::scheduling::WorkQueue &queue,
    runtime::messaging::Topic<app_heartbeat_s> &heartbeat_topic,
    HostDependencies dependencies)
    : ScheduledWorkItem(queue), heartbeat_subscription_(heartbeat_topic),
      dependency_context_(dependencies.context), time_ms_(dependencies.time_ms),
      confirm_running_image_(dependencies.confirm_running_image)
{
}
#else
BootHealthService::BootHealthService(
    runtime::scheduling::WorkQueue &queue,
    runtime::messaging::Topic<app_heartbeat_s> &heartbeat_topic)
    : ScheduledWorkItem(queue), heartbeat_subscription_(heartbeat_topic),
      time_ms_(&production_time_ms),
      confirm_running_image_(&production_confirm_running_image)
{
}
#endif

bool BootHealthService::start()
{
    if (state_ == runtime::lifecycle::ModuleState::Error) {
        return false;
    }
    if (state_ == runtime::lifecycle::ModuleState::Running) {
        return true;
    }
    if (confirmation_attempted_) {
        state_ = runtime::lifecycle::ModuleState::Running;
        return true;
    }

    stable_window_start_ms_ = time_ms_(dependency_context_);
    heartbeat_observed_ = false;
    app_heartbeat_s baseline_heartbeat{};
    (void)heartbeat_subscription_.copy(baseline_heartbeat);
    if (!ScheduleOnInterval(kCheckIntervalMs)) {
        state_ = runtime::lifecycle::ModuleState::Error;
        return false;
    }

    state_ = runtime::lifecycle::ModuleState::Running;
    return true;
}

void BootHealthService::stop()
{
    ScheduleClear();
    if (state_ != runtime::lifecycle::ModuleState::Error) {
        state_ = runtime::lifecycle::ModuleState::Stopped;
    }
}

runtime::lifecycle::ModuleState BootHealthService::state() const
{
    return state_;
}

void BootHealthService::Run()
{
    if (state_ != runtime::lifecycle::ModuleState::Running ||
        confirmation_attempted_) {
        return;
    }

    app_heartbeat_s heartbeat{};
    if (heartbeat_subscription_.copy(heartbeat)) {
        heartbeat_observed_ = true;
    }

    const uint64_t elapsed_ms =
        time_ms_(dependency_context_) - stable_window_start_ms_;
    if (!heartbeat_observed_ || elapsed_ms < kStableWindowMs) {
        return;
    }

    confirmation_attempted_ = true;
    ScheduleClear();
    const int result = confirm_running_image_(dependency_context_);
    switch (result) {
    case MCUBOOT_CONFIRM_OK:
    case MCUBOOT_CONFIRM_ALREADY_CONFIRMED:
    case MCUBOOT_CONFIRM_NOT_A_TEST_IMAGE:
        break;
    case MCUBOOT_CONFIRM_FLASH_ERROR:
    default:
        state_ = runtime::lifecycle::ModuleState::Error;
        break;
    }
}

#if defined(APP_HOST_TEST)
void BootHealthService::RunForTest()
{
    Run();
}
#endif

} // namespace app::features::boot_health
