#pragma once

#include "boot_health/boot_health.hpp"
#include "rc/ManualControl.hpp"
#include "rc/RCUpdate.hpp"
#include "rc/SbusRc.hpp"
#include "safety/Commander.hpp"
#include "freertos/sbus_uart_backend.hpp"
#include "lifecycle/module_manager.hpp"
#include "LogService.hpp"
#include "ParameterService.hpp"

#if APP_HELLO_WORLD_ENABLED
#include "hello_world/hello_world.hpp"
#endif

namespace dima::product::rover {

class ApplicationContext {
public:
    ApplicationContext() noexcept;

    bool init() noexcept;
    bool start() noexcept;
    void stop() noexcept;

private:
    static bool commander_allows_flash_write() noexcept;
    bool start_rc_chain() noexcept;
    void stop_rc_chain() noexcept;

    dima::middleware::lifecycle::ModuleManager module_manager_{};
    dima::modules::boot_health::BootHealthService boot_health_;
    LogService log_service_{};
    ParameterService parameter_service_{};
    dima::modules::safety::Commander commander_{};
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
    bool commander_started_{false};
    bool sbus_started_{false};
    bool rc_update_started_{false};
    bool manual_control_started_{false};
#if APP_HELLO_WORLD_ENABLED
    bool hello_started_{false};
#endif
};

ApplicationContext &application_context() noexcept;

} // namespace dima::product::rover
