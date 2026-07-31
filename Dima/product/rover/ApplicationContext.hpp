#pragma once

#include "Dima/modules/boot_health/boot_health.hpp"
#include "Dima/modules/rc/ManualControl.hpp"
#include "Dima/modules/rc/RCUpdate.hpp"
#include "Dima/modules/rc/SbusRc.hpp"
#include "Dima/platform/freertos/sbus_uart_backend.hpp"
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
    bool start_rc_chain() noexcept;
    void stop_rc_chain() noexcept;

    dima::middleware::lifecycle::ModuleManager module_manager_{};
    dima::modules::boot_health::BootHealthService boot_health_;
    LogService log_service_{};
    ParameterService parameter_service_{};
    dima::modules::rc::SbusRc sbus_rc_;
    dima::modules::rc::RCUpdate rc_update_{};
    dima::modules::rc::ManualControl manual_control_{};
#if APP_HELLO_WORLD_ENABLED
    dima::modules::hello_world::HelloWorld hello_world_;
#endif
    bool initialized_{false};
    bool boot_started_{false};
    bool log_started_{false};
    bool parameter_started_{false};
    bool sbus_started_{false};
    bool rc_update_started_{false};
    bool manual_control_started_{false};
#if APP_HELLO_WORLD_ENABLED
    bool hello_started_{false};
#endif
};

ApplicationContext &application_context() noexcept;

} // namespace dima::product::rover
