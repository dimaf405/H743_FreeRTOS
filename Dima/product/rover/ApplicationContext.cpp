#include "Dima/product/rover/ApplicationContext.hpp"

#include "App/runtime/scheduling/freertos_work_queue.hpp"
#include "Dima/middleware/uorb/uORB.hpp"
#include "Dima/middleware/work_queue/WorkQueue.hpp"
#include "Dima/platform/freertos/dima_platform.hpp"
#include "usb_device.h"

namespace dima::product::rover {
namespace {

void *uorb_allocate(size_t size, size_t alignment) noexcept
{
    if (alignment > portBYTE_ALIGNMENT) {
        return nullptr;
    }
    return dima::platform::allocate(
        size, dima::platform::AllocationDomain::Startup);
}

ApplicationContext g_application_context;

} // namespace

ApplicationContext::ApplicationContext() noexcept
    : boot_health_(app::runtime::scheduling::hp_default_work_queue())
#if APP_HELLO_WORLD_ENABLED
    , hello_world_(app::runtime::scheduling::lp_default_work_queue())
#endif
{
}

bool ApplicationContext::init() noexcept
{
    if (initialized_) {
        return true;
    }

    MX_USB_DEVICE_Init();
    if (!app::runtime::scheduling::init_default_work_queues()) {
        return false;
    }
    if (!px4::work_queue_init()) {
        return false;
    }
    if (!uORB::initialize(&uorb_allocate)) {
        px4::work_queue_shutdown();
        return false;
    }
    if (!module_manager_.register_module(boot_health_)) {
        uORB::shutdown();
        px4::work_queue_shutdown();
        return false;
    }
#if APP_HELLO_WORLD_ENABLED
    if (!module_manager_.register_module(hello_world_)) {
        uORB::shutdown();
        px4::work_queue_shutdown();
        return false;
    }
#endif

    initialized_ = true;
    return true;
}

bool ApplicationContext::start() noexcept
{
    if (!initialized_ || boot_started_) {
        return initialized_ && boot_started_;
    }

    boot_started_ = module_manager_.start(boot_health_);
    if (!boot_started_) {
        return false;
    }
#if APP_HELLO_WORLD_ENABLED
    hello_started_ = module_manager_.start(hello_world_);
    if (!hello_started_) {
        (void)module_manager_.stop(boot_health_);
        boot_started_ = false;
        return false;
    }
#endif
    log_started_ = log_service_.start();
    if (!log_started_) {
#if APP_HELLO_WORLD_ENABLED
        if (hello_started_) {
            (void)module_manager_.stop(hello_world_);
            hello_started_ = false;
        }
#endif
        (void)module_manager_.stop(boot_health_);
        boot_started_ = false;
        return false;
    }
    return true;
}

void ApplicationContext::stop() noexcept
{
    if (log_started_) {
        log_service_.stop();
        log_started_ = false;
    }
#if APP_HELLO_WORLD_ENABLED
    if (hello_started_) {
        (void)module_manager_.stop(hello_world_);
        hello_started_ = false;
    }
#endif
    if (boot_started_) {
        (void)module_manager_.stop(boot_health_);
        boot_started_ = false;
    }
    if (initialized_) {
        uORB::shutdown();
        px4::work_queue_shutdown();
        initialized_ = false;
    }
}

ApplicationContext &application_context() noexcept
{
    return g_application_context;
}

} // namespace dima::product::rover
