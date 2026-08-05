#include "ApplicationContext.hpp"

#include "events/events.hpp"
#include "logging/logging.hpp"
#include "uorb/uORB.hpp"
#include "work_queue/WorkQueue.hpp"

namespace dima::rover {
namespace {

void *uorb_allocate(size_t size, size_t alignment) noexcept
{
    auto *services = dima::platform::try_services();
    if (services == nullptr || alignment > services->heap.alignment()) {
        return nullptr;
    }
    return services->heap.allocate(size,
                                   dima::platform::AllocationDomain::Startup);
}

} // namespace

ApplicationContext::ApplicationContext(
    dima::platform::Services &services) noexcept
    : services_(services),
      journal_(services.parameter_partition, services.flash_transactions,
               services.armed_flash, services.synchronization),
      boot_health_(services.boot_control, services.clock),
      log_service_(services.console),
      parameter_service_(journal_, services.console, services.boot_control,
                         services.armed_flash, services.synchronization,
                         services.critical),
      commander_(services.armed_flash), sbus_rc_(services.sbus)
{
    boot_health_.bind_commander(commander_);
}

bool ApplicationContext::owner_call(bool bind_if_unset) noexcept
{
    const dima::platform::TaskHandle current = services_.tasks.current();
    if (!current) {
        return false;
    }
    if (!owner_task_ && bind_if_unset) {
        owner_task_ = current;
    }
    return owner_task_ == current;
}

bool ApplicationContext::register_modules() noexcept
{
    if (!module_manager_.register_module(parameter_service_) ||
        !module_manager_.register_module(log_service_) ||
        !module_manager_.register_module(commander_) ||
        !module_manager_.register_module(sbus_rc_) ||
        !module_manager_.register_module(rc_update_) ||
        !module_manager_.register_module(manual_control_) ||
        !module_manager_.register_module(manual_motion_adapter_) ||
        !module_manager_.register_module(rover_differential_)) {
        module_manager_.reset();
        return false;
    }
    if (!module_manager_.register_module(boot_health_)) {
        module_manager_.reset();
        return false;
    }
    modules_registered_ = true;
    return true;
}

bool ApplicationContext::release_runtime_resources() noexcept
{
    if (modules_registered_) {
        module_manager_.reset();
        modules_registered_ = false;
    }
    if (parameter_initialized_) {
        if (!parameter_service_.shutdown()) {
            return false;
        }
        parameter_initialized_ = false;
    }
    if (uorb_initialized_) {
        uORB::shutdown();
        uorb_initialized_ = false;
    }
    if (work_queue_initialized_) {
        if (!px4::work_queue_shutdown()) {
            return false;
        }
        work_queue_initialized_ = false;
    }
    if (console_initialized_) {
        if (!services_.console.shutdown()) {
            return false;
        }
        console_initialized_ = false;
    }
    dima::events::reset();
    dima::logging::reset();
    return true;
}

bool ApplicationContext::rollback_initialization() noexcept
{
    const bool cleaned = release_runtime_resources();
    runtime_state_ = cleaned ? RuntimeState::Stopped : RuntimeState::Error;
    return cleaned;
}

bool ApplicationContext::init() noexcept
{
    if (!owner_call(true)) {
        return false;
    }
    if (runtime_state_ == RuntimeState::Initialized ||
        runtime_state_ == RuntimeState::Running) {
        return true;
    }
    if (runtime_state_ != RuntimeState::Stopped) {
        return false;
    }

    runtime_state_ = RuntimeState::Initializing;
    dima::events::reset();
    dima::logging::reset();
    PX4_INFO("Application Runtime initialization started");
    services_.diagnostics.set_stage(dima::platform::StartupStage::UsbInit);
    console_initialized_ = true;
    if (!services_.console.initialize()) {
        (void)rollback_initialization();
        return false;
    }
    services_.diagnostics.set_stage(dima::platform::StartupStage::UsbReady);

    services_.diagnostics.set_stage(
        dima::platform::StartupStage::WorkQueueInit);
    work_queue_initialized_ = true;
    if (!px4::work_queue_init()) {
        (void)rollback_initialization();
        return false;
    }

    services_.diagnostics.set_stage(dima::platform::StartupStage::UorbInit);
    const uORB::Allocator allocator{&uorb_allocate, &dima::platform::deallocate};
    uorb_initialized_ = true;
    if (!uORB::initialize(allocator)) {
        (void)rollback_initialization();
        return false;
    }

    services_.diagnostics.set_stage(
        dima::platform::StartupStage::ParameterInit);
    parameter_initialized_ = true;
    if (!parameter_service_.init()) {
        (void)rollback_initialization();
        return false;
    }

    services_.diagnostics.set_stage(
        dima::platform::StartupStage::ModuleRegister);
    if (!register_modules()) {
        (void)rollback_initialization();
        return false;
    }

    runtime_state_ = RuntimeState::Initialized;
    services_.diagnostics.set_stage(
        dima::platform::StartupStage::ApplicationInitialized);
    PX4_INFO("Application Runtime initialized");
    return true;
}

bool ApplicationContext::rollback_start() noexcept
{
    const bool modules_stopped = stop_started_modules();
    const bool resources_released =
        modules_stopped && release_runtime_resources();
    runtime_state_ = resources_released ? RuntimeState::Stopped
                                        : RuntimeState::Error;
    return resources_released;
}

bool ApplicationContext::start() noexcept
{
    if (!owner_call(false)) {
        return false;
    }
    if (runtime_state_ == RuntimeState::Running) {
        return true;
    }
    if (runtime_state_ != RuntimeState::Initialized) {
        return false;
    }
    runtime_state_ = RuntimeState::Starting;

    services_.diagnostics.set_stage(
        dima::platform::StartupStage::ParameterStart);
    parameter_started_ = module_manager_.start(parameter_service_);
    if (!parameter_started_) {
        (void)rollback_start();
        return false;
    }
    PX4_INFO("Parameter service started");

    services_.diagnostics.set_stage(dima::platform::StartupStage::LogStart);
    log_started_ = module_manager_.start(log_service_);
    if (!log_started_) {
        (void)rollback_start();
        return false;
    }

    services_.diagnostics.set_stage(
        dima::platform::StartupStage::CommanderStart);
    commander_started_ = module_manager_.start(commander_);
    if (!commander_started_) {
        (void)rollback_start();
        return false;
    }
    PX4_INFO("Commander started");

    // RC 链故障只降级手动输入，不能拖垮参数、日志和恢复服务。
    services_.diagnostics.set_stage(dima::platform::StartupStage::RcStart);
    if (!start_rc_chain()) {
        PX4_ERR("RC chain unavailable; actuator output remains inhibited");
    } else {
        PX4_INFO("RC input chain started");
    }

    if (!start_control_chain()) {
        (void)rollback_start();
        return false;
    }
    PX4_INFO("Rover Manual differential control started");

    services_.diagnostics.set_stage(
        dima::platform::StartupStage::BootHealthStart);
    boot_started_ = module_manager_.start(boot_health_);
    if (!boot_started_) {
        (void)rollback_start();
        return false;
    }

    runtime_state_ = RuntimeState::Running;
    PX4_INFO("Application Runtime running");
    return true;
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

bool ApplicationContext::stop_rc_chain() noexcept
{
    bool stopped = true;
    if (manual_control_started_) {
        const bool result = module_manager_.stop(manual_control_);
        manual_control_started_ = !result;
        stopped = result && stopped;
    }
    if (rc_update_started_) {
        const bool result = module_manager_.stop(rc_update_);
        rc_update_started_ = !result;
        stopped = result && stopped;
    }
    if (sbus_started_) {
        const bool result = module_manager_.stop(sbus_rc_);
        sbus_started_ = !result;
        stopped = result && stopped;
    }
    return stopped;
}

bool ApplicationContext::start_control_chain() noexcept
{
    manual_motion_started_ = module_manager_.start(manual_motion_adapter_);
    if (!manual_motion_started_) {
        return false;
    }

    rover_differential_started_ = module_manager_.start(rover_differential_);
    if (!rover_differential_started_) {
        (void)module_manager_.stop(manual_motion_adapter_);
        manual_motion_started_ = false;
        return false;
    }
    return true;
}

bool ApplicationContext::stop_control_chain() noexcept
{
    bool stopped = true;
    if (rover_differential_started_) {
        const bool result = module_manager_.stop(rover_differential_);
        rover_differential_started_ = !result;
        stopped = result && stopped;
    }
    if (manual_motion_started_) {
        const bool result = module_manager_.stop(manual_motion_adapter_);
        manual_motion_started_ = !result;
        stopped = result && stopped;
    }
    return stopped;
}

bool ApplicationContext::stop_started_modules() noexcept
{
    bool stopped = true;
    if (boot_started_) {
        const bool result = module_manager_.stop(boot_health_);
        boot_started_ = !result;
        stopped = result && stopped;
    }
    stopped = stop_control_chain() && stopped;
    stopped = stop_rc_chain() && stopped;
    if (commander_started_) {
        const bool result = module_manager_.stop(commander_);
        commander_started_ = !result;
        stopped = result && stopped;
    }
    if (log_started_) {
        const bool result = module_manager_.stop(log_service_);
        log_started_ = !result;
        stopped = result && stopped;
    }
    if (parameter_started_) {
        const bool result = module_manager_.stop(parameter_service_);
        parameter_started_ = !result;
        stopped = result && stopped;
    }
    return stopped;
}

bool ApplicationContext::shutdown() noexcept
{
    if (!owner_call(false)) {
        return false;
    }
    if (runtime_state_ == RuntimeState::Stopped) {
        return true;
    }
    if (runtime_state_ == RuntimeState::Initializing ||
        runtime_state_ == RuntimeState::Starting ||
        runtime_state_ == RuntimeState::Stopping) {
        return false;
    }

    runtime_state_ = RuntimeState::Stopping;
    if (!stop_started_modules() || !release_runtime_resources()) {
        runtime_state_ = RuntimeState::Error;
        return false;
    }
    runtime_state_ = RuntimeState::Stopped;
    return true;
}

ApplicationContext &application_context() noexcept
{
    static ApplicationContext instance{dima::platform::services()};
    return instance;
}

} // namespace dima::rover
