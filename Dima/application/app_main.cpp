#include "app_main.h"

#include "ApplicationContext.hpp"
#include "platform/api/Platform.hpp"

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

    // 装配根具有静态生命周期；appMain 仅保留产品运行状态。
    for (;;) {
        services.tasks.suspend_current();
    }
}
