#pragma once

#include "App/features/boot_health/boot_health.hpp"
#include "App/messages/app_heartbeat.hpp"
#include "App/runtime/lifecycle/module_manager.hpp"
#include "App/runtime/messaging/topic.hpp"

#if APP_HELLO_WORLD_ENABLED
#include "App/features/hello_world/hello_world.hpp"
#endif

namespace dima::product::rover {

class ApplicationContext {
public:
    ApplicationContext() noexcept;

    bool init() noexcept;
    bool start() noexcept;
    void stop() noexcept;

private:
    app::runtime::messaging::Topic<app_heartbeat_s> heartbeat_topic_{};
    app::runtime::lifecycle::ModuleManager module_manager_{};
    app::features::boot_health::BootHealthService boot_health_;
#if APP_HELLO_WORLD_ENABLED
    app::features::hello_world::HelloWorld hello_world_;
#endif
    bool initialized_{false};
    bool boot_started_{false};
#if APP_HELLO_WORLD_ENABLED
    bool hello_started_{false};
#endif
};

ApplicationContext &application_context() noexcept;

} // namespace dima::product::rover
