#include "app_main.h"

#include "ApplicationContext.hpp"
#include "platform/api/Boot.hpp"
#include "platform/api/Services.hpp"
#include "platform/api/TaskRuntime.hpp"

namespace {

constexpr std::uint32_t kWatchdogTimeoutMs = 2048U;
constexpr std::uint32_t kWatchdogCheckIntervalMs = 100U;

} // namespace

extern "C" void app_main_task(void *argument)
{
    (void)argument;

    auto &services = dima::platform::services();
    services.diagnostics.set_stage(
        dima::platform::StartupStage::ApplicationTaskEnter);
    auto &application = dima::rover::application_context();
    if (!application.init() || !application.start()) {
        services.diagnostics.set_stage(
            dima::platform::StartupStage::ApplicationFailed);
        services.diagnostics.panic(
            dima::platform::FailureKind::ErrorHandler,
            static_cast<std::uint32_t>(
                dima::platform::StartupStage::ApplicationFailed),
            0U);
    } else {
        services.diagnostics.set_stage(
            dima::platform::StartupStage::ApplicationRunning);
    }

    /* Start (or shorten a watchdog carried through MCUboot) as soon as the
     * complete Runtime exists. From this point onward only a new BootHealth
     * generation may reload it. */
    if (!services.watchdog.start(kWatchdogTimeoutMs)) {
        services.diagnostics.panic(
            dima::platform::FailureKind::ErrorHandler,
            static_cast<std::uint32_t>(
                dima::platform::StartupStage::ApplicationRunning),
            kWatchdogTimeoutMs);
    }

    std::uint32_t last_health_generation = 0U;
    // appMain is the single application-side IWDG feed owner. BootHealth only
    // advances a generation after independently checking the Runtime.
    for (;;) {
        services.tasks.delay(
            dima::platform::Timeout::from_ms(kWatchdogCheckIntervalMs));
        std::uint32_t health_generation = last_health_generation;
        if (!application.watchdog_feed_allowed(last_health_generation,
                                               health_generation)) {
            continue;
        }
        services.watchdog.feed();
        application.watchdog_feed_completed();
        last_health_generation = health_generation;
    }
}
