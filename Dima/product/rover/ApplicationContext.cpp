#include "Dima/product/rover/ApplicationContext.hpp"

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

ApplicationContext::ApplicationContext() noexcept = default;

bool ApplicationContext::init() noexcept
{
    if (initialized_) {
        return true;
    }

    MX_USB_DEVICE_Init();
    if (!px4::work_queue_init()) {
        return false;
    }
    const uORB::Allocator allocator{&uorb_allocate, &dima::platform::deallocate};
    if (!uORB::initialize(allocator)) {
        px4::work_queue_shutdown();
        return false;
    }
    if (!parameter_service_.init()) {
        uORB::shutdown();
        px4::work_queue_shutdown();
        return false;
    }
    if (!module_manager_.register_module(boot_health_)) {
        module_manager_.reset();
        uORB::shutdown();
        px4::work_queue_shutdown();
        return false;
    }
#if APP_HELLO_WORLD_ENABLED
    if (!module_manager_.register_module(hello_world_)) {
        module_manager_.reset();
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
    parameter_started_ = parameter_service_.start();
    if (!parameter_started_) {
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
    log_started_ = log_service_.start();
    if (!log_started_) {
        parameter_service_.stop();
        parameter_started_ = false;
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
    if (parameter_started_) {
        parameter_service_.stop();
        parameter_started_ = false;
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
        module_manager_.reset();
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
