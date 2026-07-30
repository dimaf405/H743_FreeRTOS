#include "Dima/modules/boot_health/boot_health.hpp"

#include "Dima/adapters/mcuboot/mcuboot_app.h"
#include "Dima/platform/freertos/platform_time.hpp"

namespace dima::modules::boot_health {

#if !defined(APP_HOST_TEST)
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
#endif

#if defined(APP_HOST_TEST)
BootHealthService::BootHealthService(
    dima::middleware::scheduling::WorkQueue &queue,
    dima::middleware::messaging::Topic<app_heartbeat_s> &heartbeat_topic,
    HostDependencies dependencies)
    : ScheduledWorkItem(queue), heartbeat_subscription_(heartbeat_topic),
      dependency_context_(dependencies.context), time_ms_(dependencies.time_ms),
      confirm_running_image_(dependencies.confirm_running_image)
{
}
#else
BootHealthService::BootHealthService() noexcept
    : px4::ScheduledWorkItem("boot_health", px4::wq_configurations::hp_default),
      heartbeat_subscription_(ORB_ID(app_heartbeat)),
      time_ms_(&production_time_ms),
      confirm_running_image_(&production_confirm_running_image)
{
}
#endif

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

    stable_window_start_ms_ = time_ms_(dependency_context_);
    heartbeat_observed_ = false;
#if defined(APP_HOST_TEST)
    app_heartbeat_s baseline_heartbeat{};
    (void)heartbeat_subscription_.copy(baseline_heartbeat);
#else
    (void)heartbeat_subscription_.update();
#endif
#if defined(APP_HOST_TEST)
    const bool scheduled = ScheduleOnInterval(kCheckIntervalMs);
#else
    const bool scheduled = ScheduleOnInterval(kCheckIntervalMs * 1000U);
#endif
    if (!scheduled) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        return false;
    }

    state_ = dima::middleware::lifecycle::ModuleState::Running;
    return true;
}

void BootHealthService::stop()
{
    ScheduleClear();
    if (state_ != dima::middleware::lifecycle::ModuleState::Error) {
        state_ = dima::middleware::lifecycle::ModuleState::Stopped;
    }
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

#if defined(APP_HOST_TEST)
    app_heartbeat_s heartbeat{};
    if (heartbeat_subscription_.copy(heartbeat)) {
        heartbeat_observed_ = true;
    }
#else
    if (heartbeat_subscription_.update()) {
        heartbeat_observed_ = true;
    }
#endif

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
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        break;
    }
}

#if defined(APP_HOST_TEST)
void BootHealthService::RunForTest()
{
    Run();
}
#endif

} // namespace dima::modules::boot_health
