#include "ApplicationContext.hpp"

#include "boot_diagnostics.h"
#include "logging/logging.hpp"
#include "uorb/uORB.hpp"
#include "work_queue/WorkQueue.hpp"
#include "freertos/dima_platform.hpp"
#include "usb_device.h"

namespace dima::rover {
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
    : sbus_rc_(dima::platform::sbus_uart_backend())
{
    boot_health_.bind_commander(commander_);
}

bool ApplicationContext::init() noexcept
{
    if (initialized_) {
        return true;
    }

    dima_boot_stage_set(DIMA_BOOT_STAGE_USB_INIT);
    MX_USB_DEVICE_Init();
    dima_boot_stage_set(DIMA_BOOT_STAGE_USB_READY);
    dima_boot_stage_set(DIMA_BOOT_STAGE_WORK_QUEUE_INIT);
    if (!px4::work_queue_init()) {
        return false;
    }
    dima_boot_stage_set(DIMA_BOOT_STAGE_UORB_INIT);
    const uORB::Allocator allocator{&uorb_allocate, &dima::platform::deallocate};
    if (!uORB::initialize(allocator)) {
        px4::work_queue_shutdown();
        return false;
    }
    dima_boot_stage_set(DIMA_BOOT_STAGE_PARAMETER_INIT);
    if (!parameter_service_.init()) {
        uORB::shutdown();
        px4::work_queue_shutdown();
        return false;
    }
    dima_boot_stage_set(DIMA_BOOT_STAGE_MODULE_REGISTER);
    if (!module_manager_.register_module(boot_health_) ||
        !module_manager_.register_module(parameter_service_) ||
        !module_manager_.register_module(log_service_)) {
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
    if (!module_manager_.register_module(commander_) ||
        !module_manager_.register_module(sbus_rc_) ||
        !module_manager_.register_module(rc_update_) ||
        !module_manager_.register_module(manual_control_)) {
        module_manager_.reset();
        uORB::shutdown();
        px4::work_queue_shutdown();
        return false;
    }

    initialized_ = true;
    dima_boot_stage_set(DIMA_BOOT_STAGE_APPLICATION_INITIALIZED);
    return true;
}

bool ApplicationContext::start() noexcept
{
    if (!initialized_ || boot_started_) {
        return initialized_ && boot_started_;
    }

    dima_boot_stage_set(DIMA_BOOT_STAGE_BOOT_HEALTH_START);
    boot_started_ = module_manager_.start(boot_health_);
    if (!boot_started_) return false;
#if APP_HELLO_WORLD_ENABLED
    dima_boot_stage_set(DIMA_BOOT_STAGE_HELLO_START);
    hello_started_ = module_manager_.start(hello_world_);
    if (!hello_started_) {
        (void)module_manager_.stop(boot_health_);
        boot_started_ = false;
        return false;
    }
#endif
    dima_boot_stage_set(DIMA_BOOT_STAGE_PARAMETER_START);
    parameter_started_ = module_manager_.start(parameter_service_);
    if (!parameter_started_) {
#if APP_HELLO_WORLD_ENABLED
        (void)module_manager_.stop(hello_world_);
        hello_started_ = false;
#endif
        (void)module_manager_.stop(boot_health_);
        boot_started_ = false;
        return false;
    }

    dima_boot_stage_set(DIMA_BOOT_STAGE_LOG_START);
    log_started_ = module_manager_.start(log_service_);
    if (!log_started_) {
        (void)module_manager_.stop(parameter_service_);
        parameter_started_ = false;
#if APP_HELLO_WORLD_ENABLED
        (void)module_manager_.stop(hello_world_);
        hello_started_ = false;
#endif
        (void)module_manager_.stop(boot_health_);
        boot_started_ = false;
        return false;
    }

    dima_boot_stage_set(DIMA_BOOT_STAGE_COMMANDER_START);
    commander_started_ = module_manager_.start(commander_);
    if (!commander_started_) {
        stop_rc_chain();
        dima::modules::parameters::set_flash_write_allowed_hook(nullptr);
        (void)module_manager_.stop(log_service_);
        log_started_ = false;
        (void)module_manager_.stop(parameter_service_);
        parameter_started_ = false;
#if APP_HELLO_WORLD_ENABLED
        (void)module_manager_.stop(hello_world_);
        hello_started_ = false;
#endif
        (void)module_manager_.stop(boot_health_);
        boot_started_ = false;
        return false;
    }
    dima::modules::parameters::set_flash_write_allowed_hook(
        &ApplicationContext::commander_allows_flash_write);

    // RC 链故障只降级手动输入，不能拖垮参数、日志和恢复服务。
    dima_boot_stage_set(DIMA_BOOT_STAGE_RC_START);
    if (!start_rc_chain()) {
        PX4_WARN("Dima RC chain unavailable; actuator output remains disabled");
    }
    return true;
}

bool ApplicationContext::commander_allows_flash_write() noexcept
{
    return !application_context().commander_.armed();
}

bool ApplicationContext::start_rc_chain() noexcept
{
    sbus_started_ = module_manager_.start(sbus_rc_);
    if (!sbus_started_) return false;

    rc_update_started_ = module_manager_.start(rc_update_);
    if (!rc_update_started_) {
        (void)module_manager_.stop(sbus_rc_);
        sbus_started_ = false;
        return false;
    }

    manual_control_started_ = module_manager_.start(manual_control_);
    if (!manual_control_started_) {
        (void)module_manager_.stop(rc_update_);
        rc_update_started_ = false;
        (void)module_manager_.stop(sbus_rc_);
        sbus_started_ = false;
        return false;
    }
    return true;
}

void ApplicationContext::stop_rc_chain() noexcept
{
    if (manual_control_started_) {
        (void)module_manager_.stop(manual_control_);
        manual_control_started_ = false;
    }
    if (rc_update_started_) {
        (void)module_manager_.stop(rc_update_);
        rc_update_started_ = false;
    }
    if (sbus_started_) {
        (void)module_manager_.stop(sbus_rc_);
        sbus_started_ = false;
    }
}

void ApplicationContext::stop() noexcept
{
    stop_rc_chain();
    if (commander_started_) {
        (void)module_manager_.stop(commander_);
        commander_started_ = false;
    }
    dima::modules::parameters::set_flash_write_allowed_hook(nullptr);
    if (log_started_) {
        (void)module_manager_.stop(log_service_);
        log_started_ = false;
    }
    if (parameter_started_) {
        (void)module_manager_.stop(parameter_service_);
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

} // namespace dima::rover
