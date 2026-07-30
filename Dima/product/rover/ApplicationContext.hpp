#pragma once

#include "Dima/modules/boot_health/boot_health.hpp"
#include "Dima/middleware/lifecycle/module_manager.hpp"
#include "Dima/product/rover/LogService.hpp"
#include "Dima/product/rover/ParameterService.hpp"

#if APP_HELLO_WORLD_ENABLED
#include "Dima/modules/hello_world/hello_world.hpp"
#endif

namespace dima::product::rover {

class ApplicationContext {
public:
    ApplicationContext() noexcept;

    bool init() noexcept;
    bool start() noexcept;
    void stop() noexcept;

private:
    dima::middleware::lifecycle::ModuleManager module_manager_{};
    dima::modules::boot_health::BootHealthService boot_health_;
    LogService log_service_{};
    ParameterService parameter_service_{};
#if APP_HELLO_WORLD_ENABLED
    dima::modules::hello_world::HelloWorld hello_world_;
#endif
    bool initialized_{false};
    bool boot_started_{false};
    bool log_started_{false};
    bool parameter_started_{false};
#if APP_HELLO_WORLD_ENABLED
    bool hello_started_{false};
#endif
};

ApplicationContext &application_context() noexcept;

} // namespace dima::product::rover
