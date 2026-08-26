#include "app_main.h"

#include "ApplicationContext.hpp"
#include "api/Boot.hpp"
#include "api/Services.hpp"
#include "api/TaskRuntime.hpp"

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
    // ApplicationContext 负责完整模块图的构造与启动；任一步失败都不能进入
    // “部分 Runtime”继续喂狗，因此记录启动阶段后立即走不可返回的 panic。
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

    /* 完整 Runtime 建立后立即启动 IWDG；若 MCUboot 已经开启看门狗，此调用只会
     * 收紧/接管现有期限。此后必须由新的 BootHealth generation 证明一次完整健康
     * 周期，appMain 才允许真正 reload，避免单个模块自证健康。 */
    if (!services.watchdog.start(kWatchdogTimeoutMs)) {
        services.diagnostics.panic(
            dima::platform::FailureKind::ErrorHandler,
            static_cast<std::uint32_t>(
                dima::platform::StartupStage::ApplicationRunning),
            kWatchdogTimeoutMs);
    }

    std::uint32_t last_health_generation = 0U;
    // appMain 是应用侧唯一 IWDG feed owner。generation 是一次性握手票据：
    // 同一代只能喂一次，BootHealth 必须重新核对参数、安全 Topic 与输出状态后
    // 才能推进下一代；watchdog_feed_completed() 再关闭本轮维护窗口。
    for (;;) {
        services.tasks.delay(
            dima::platform::Timeout::from_ms(kWatchdogCheckIntervalMs));
        // 即便当前 generation 不允许喂狗，仍需服务 USB/串口等轮询后端；
        // 健康失败最终由 IWDG 截止时间收敛，而不是在此处阻塞 Runtime。
        application.service();
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
