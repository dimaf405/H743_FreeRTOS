#include "App/application/app_main.h"

#include "App/features/boot_health/boot_health.hpp"
#include "App/messages/app_heartbeat.hpp"
#include "App/runtime/lifecycle/module_manager.hpp"
#include "App/runtime/messaging/topic.hpp"
#include "App/runtime/scheduling/freertos_work_queue.hpp"
#include "usb_device.h"

#if !defined(APP_HELLO_WORLD_ENABLED)
#error "APP_HELLO_WORLD_ENABLED must be supplied by the build"
#elif APP_HELLO_WORLD_ENABLED != 0 && APP_HELLO_WORLD_ENABLED != 1
#error "APP_HELLO_WORLD_ENABLED must be 0 or 1"
#endif

#if APP_HELLO_WORLD_ENABLED
#include "App/features/hello_world/hello_world.hpp"
#endif

extern "C" {
#include "FreeRTOS.h"
#include "task.h"
}

extern "C" void app_main_task(void *argument)
{
    (void)argument;

    MX_USB_DEVICE_Init();
    const bool queues_initialized =
        app::runtime::scheduling::init_default_work_queues();

    app::runtime::messaging::Topic<app_heartbeat_s> heartbeat_topic;
    app::runtime::lifecycle::ModuleManager module_manager;
    app::features::boot_health::BootHealthService boot_health{
        app::runtime::scheduling::hp_default_work_queue(), heartbeat_topic};
#if APP_HELLO_WORLD_ENABLED
    app::features::hello_world::HelloWorld hello_world{
        app::runtime::scheduling::lp_default_work_queue(), heartbeat_topic};
#endif

    bool boot_started = false;
#if APP_HELLO_WORLD_ENABLED
    bool hello_started = false;
#endif
    bool assembly_ok = queues_initialized;

    if (assembly_ok) {
        assembly_ok = module_manager.register_module(boot_health);
    }
    if (assembly_ok) {
        boot_started = module_manager.start(boot_health);
        assembly_ok = boot_started;
    }
#if APP_HELLO_WORLD_ENABLED
    if (assembly_ok) {
        assembly_ok = module_manager.register_module(hello_world);
    }
    if (assembly_ok) {
        hello_started = module_manager.start(hello_world);
        assembly_ok = hello_started;
    }
#endif

    if (!assembly_ok) {
#if APP_HELLO_WORLD_ENABLED
        if (hello_started) {
            (void)module_manager.stop(hello_world);
        }
#endif
        if (boot_started) {
            (void)module_manager.stop(boot_health);
        }
    }

    for (;;) {
        vTaskSuspend(nullptr);
    }
}
