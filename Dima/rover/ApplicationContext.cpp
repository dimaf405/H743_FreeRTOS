#include "ApplicationContext.hpp"
#include "api/Boot.hpp"
#include "api/Console.hpp"
#include "api/Memory.hpp"
#include "api/TaskRuntime.hpp"

#include "events/events.hpp"
#include "logging/logging.hpp"
#include "uORB/uORB.hpp"
#include "work_queue/WorkQueue.hpp"

namespace dima::rover {
namespace {

void *uorb_allocate(size_t size, size_t alignment) noexcept
{
    // uORB 元数据/队列只允许从 Startup 域分配，并拒绝超过平台 heap 对齐能力的
    // 请求；实时运行阶段不会通过此适配器扩展消息池。
    auto *services = dima::platform::try_services();
    if (services == nullptr || alignment > services->heap.alignment()) {
        return nullptr;
    }
    return services->heap.allocate(size,
                                   dima::platform::AllocationDomain::Startup);
}

template <typename... Modules>
bool register_all_modules(
    dima::middleware::lifecycle::ModuleManager &manager,
    Modules &...modules) noexcept
{
    // 编译期约束组合根模块数不超过固定 ModuleManager 容量；fold expression
    // 按声明顺序注册，首个失败即停止并由调用者 reset。
    static_assert(
        sizeof...(Modules) <=
            dima::middleware::lifecycle::ModuleManager::kMaxModules,
        "Application module list exceeds ModuleManager capacity");
    return (... && manager.register_module(modules));
}

} // namespace

ApplicationContext::ApplicationContext(
    dima::platform::Services &services) noexcept
    : services_(services),
      maintenance_(services.critical),
      flashfs_(services.parameter_partition, services.flash_transactions,
               services.armed_flash, services.synchronization),
      boot_health_(services.boot_control, services.clock, maintenance_),
      log_service_(services.log_files),
      parameter_service_(flashfs_, services.atomic_files,
                         services.armed_flash,
                         services.synchronization, services.critical,
                         maintenance_),
      // SYS_DM_BACKEND=0 的 PX4 Dataman 默认后端复用组合根唯一 FlashFS；
      // Parameter、DroneCAN 与 Mission 不得各自创建板载 Flash owner。
      mission_service_(flashfs_, services.synchronization,
                       services.armed_flash),
      mavlink_service_(services.console, services.boot_control,
                       mission_service_, services.log_files),
      serial_config_(services.serial_ports),
      um982_gps_(services.async_serial_port, services.clock, serial_config_,
                 services.armed_flash, maintenance_),
      icm42688p_(services.spi, services.interrupt_sources),
      vehicle_imu_(services.armed_flash),
      vehicle_magnetometer_(services.armed_flash),
      sensor_calibration_(services.armed_flash, vehicle_imu_,
                          vehicle_magnetometer_),
      dronecan_mag2_(services.can, services.armed_flash, maintenance_,
                     flashfs_),
      motor_output_(services.actuator_pwm),
      commander_(services.armed_flash, maintenance_, mission_service_),
      sbus_rc_(services.timestamped_serial_input, serial_config_),
      auto_mode_(mission_service_)
{
}

bool ApplicationContext::owner_call(bool bind_if_unset) noexcept
{
    // init 首次绑定当前任务，之后 start/service/shutdown 必须由同一 task 调用；
    // 防止跨任务并发修改大量非原子的生命周期标志。
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
    if (!register_all_modules(
            module_manager_, parameter_service_, mission_service_,
            log_service_, serial_config_,
            um982_gps_, icm42688p_, vehicle_imu_, vehicle_magnetometer_,
            sensor_calibration_, dronecan_mag2_, ekf2_, motor_output_,
            commander_, mavlink_service_,
            sbus_rc_, rc_update_, rc_manual_input_, manual_mode_,
            auto_mode_, rover_differential_, boot_health_)) {
        module_manager_.reset();
        return false;
    }
    modules_registered_ = true;
    return true;
}

bool ApplicationContext::release_runtime_resources() noexcept
{
    // 资源按依赖逆序释放：模块注册表 -> 参数 -> 日志 -> uORB -> WorkQueue ->
    // Console。某一步失败立即保留其余状态，调用者把 Runtime 锁存 Error。
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
    if (log_initialized_) {
        log_service_.shutdown();
        log_initialized_ = false;
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

    // 初始化顺序是 Console -> WorkQueue -> uORB -> Logging -> Parameters ->
    // Module registry；每一步先置 ownership 标志，失败时统一逆序回滚。
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

    // 参数/日志先提供基础设施；MotorOutput 必须先证明 safe-off，随后 Commander/
    // MAVLink。任何安全关键启动失败都完整回滚。
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

    log_initialized_ = true;
    if (!log_service_.initialize()) {
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

    // Mission 失败只锁闭 AUTO，不得拖垮 Manual、参数、恢复和安全输出链。
    // PX4 Dataman 的 Mission State/bank 恢复在 wq:storage 异步执行；loaded 前
    // MAVLink 回读只报告空 RAM 快照，上传/清空仍由 backend readiness 门控。
    mission_started_ = module_manager_.start(mission_service_);
    if (!mission_started_) {
        PX4_ERR("Mission service unavailable; AUTO remains locked");
    } else {
        PX4_INFO("Mission service started");
    }

    services_.diagnostics.set_stage(dima::platform::StartupStage::LogStart);
    log_started_ = module_manager_.start(log_service_);
    if (!log_started_) {
        (void)rollback_start();
        return false;
    }

    serial_config_started_ = module_manager_.start(serial_config_);
    if (!serial_config_started_) {
        PX4_ERR("Board serial configuration invalid; RC input inhibited");
    } else {
        PX4_INFO("Board serial configuration started");
    }

    services_.diagnostics.set_stage(
        dima::platform::StartupStage::MotorOutputStart);
    motor_output_started_ = module_manager_.start(motor_output_);
    if (!motor_output_started_ || !motor_output_.safe_off_confirmed()) {
        (void)rollback_start();
        return false;
    }
    PX4_INFO("Motor output safe-off established");

    services_.diagnostics.set_stage(
        dima::platform::StartupStage::CommanderStart);
    commander_started_ = module_manager_.start(commander_);
    if (!commander_started_) {
        (void)rollback_start();
        return false;
    }
    PX4_INFO("Commander started");

    services_.diagnostics.set_stage(dima::platform::StartupStage::MavlinkStart);
    mavlink_started_ = module_manager_.start(mavlink_service_);
    if (!mavlink_started_) {
        (void)rollback_start();
        return false;
    }
    PX4_INFO("MAVLink service started");

    // RC 链故障只降级手动输入，不能拖垮参数、日志和恢复服务；但输出仍由
    // Commander/MotorOutput 的 command_valid/armed 门限保持禁止。
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

    // 传感器/GPS/校准是可降级观测能力：失败会记录但不剥夺手动 Rover 的基础
    // 参数、恢复与控制链。静态构建通过也不代表这些设备已在板上可用。
    icm42688p_started_ = module_manager_.start(icm42688p_);
    if (!icm42688p_started_) {
        PX4_ERR("ICM42688P unavailable; Manual control remains enabled");
    }
    vehicle_imu_started_ = module_manager_.start(vehicle_imu_);
    if (!vehicle_imu_started_) {
        PX4_ERR("vehicle_imu frontend unavailable; Manual control remains enabled");
    }
    vehicle_magnetometer_started_ =
        module_manager_.start(vehicle_magnetometer_);
    if (!vehicle_magnetometer_started_) {
        PX4_ERR("vehicle_magnetometer frontend unavailable; raw magnetometer remains enabled");
    }
    dronecan_mag2_started_ = module_manager_.start(dronecan_mag2_);
    if (!dronecan_mag2_started_) {
        PX4_ERR("DroneCAN magnetometer unavailable; Manual control remains enabled");
    }

    um982_gps_started_ = module_manager_.start(um982_gps_);
    if (!um982_gps_started_) {
        PX4_ERR("UM982 GPS unavailable; Manual control remains enabled");
    }

    // N1 EKF2 没有运行期开关或动态装卸路径。只在 IMU、Mag、GNSS 生产者完成
    // 唯一一次启动尝试后启动；失败仅降级自动导航观测，绝不反向关闭 Manual、
    // Commander、BootHealth、IWDG 或 PWM 安全链。
    ekf2_started_ = module_manager_.start(ekf2_);
    if (!ekf2_started_) {
        PX4_ERR("EKF2 unavailable; Manual control remains enabled");
    }

    // AUTO 外环只在 Mission 与 EKF 两个生产者均成功启动后运行；自身启动失败
    // 也只让 readiness 保持 false，不能回滚 Manual、Commander 或 PWM 安全链。
    if (mission_started_ && ekf2_started_) {
        auto_mode_started_ = module_manager_.start(auto_mode_);
    }
    if (!auto_mode_started_) {
        PX4_ERR("Rover AUTO mode unavailable; AUTO remains locked");
    } else {
        PX4_INFO("Rover AUTO navigation started");
    }

    sensor_calibration_started_ =
        module_manager_.start(sensor_calibration_);
    if (!sensor_calibration_started_) {
        PX4_ERR("Sensor calibration unavailable; Manual control remains enabled");
    }

    runtime_state_ = RuntimeState::Running;
    active_serial_signature_ = serial_config_.configuration_signature();
    PX4_INFO("Application Runtime running");
    return true;
}

bool ApplicationContext::apply_serial_configuration() noexcept
{
    // 串口映射是共享资源事务：先停 UM982 和完整 RC 链，再调用 SerialConfig
    // 重配，之后无论配置成功与否都尝试恢复消费者，避免遗留无人拥有的 UART。
    bool stopped = true;
    if (um982_gps_started_) {
        const bool result = module_manager_.stop(um982_gps_);
        um982_gps_started_ = !result;
        stopped = result && stopped;
    }
    stopped = stop_rc_chain() && stopped;
    if (!stopped) {
        if (!start_rc_chain()) {
            PX4_WARN("RC chain recovery failed after serial stop failure");
        }
        if (!um982_gps_started_) {
            um982_gps_started_ = module_manager_.start(um982_gps_);
        }
        return false;
    }

    const bool configured = serial_config_.reconfigure();
    if (!start_rc_chain()) {
        PX4_WARN("RC chain unavailable after serial reconfiguration");
    }
    um982_gps_started_ = module_manager_.start(um982_gps_);
    if (!um982_gps_started_) {
        PX4_WARN("UM982 unavailable after serial reconfiguration");
    }
    if (!configured) {
        PX4_ERR("runtime serial reconfiguration rejected; restored active ports");
        return false;
    }
    active_serial_signature_ = serial_config_.configuration_signature();
    return active_serial_signature_ != 0U;
}

void ApplicationContext::service() noexcept
{
    if (!owner_call(false) || runtime_state_ != RuntimeState::Running ||
        !serial_config_started_) {
        return;
    }
    const std::uint64_t now_us = services_.clock.now_us();
    // signature 是生成串口配置的内容签名；0/未变、已武装、退避期或已有维护
    // 事务均不触发重配。候选还必须先通过 SerialConfig 的完整冲突校验。
    const std::uint64_t signature = serial_config_.configuration_signature();
    if (serial_reconfigure_phase_ == SerialReconfigurePhase::Idle) {
        if (signature == 0U || signature == active_serial_signature_ ||
            commander_.armed() || now_us < serial_retry_after_us_ ||
            maintenance_.in_progress()) {
            return;
        }
        if (!serial_config_.pending_configuration_valid()) {
            serial_retry_after_us_ = now_us + 1000000ULL;
            return;
        }
        serial_maintenance_ticket_ = maintenance_.request(now_us);
        if (serial_maintenance_ticket_ == 0U) return;
        serial_reconfigure_phase_ =
            SerialReconfigurePhase::WaitForApproval;
        return;
    }

    // maintenance permit 采用非阻塞轮询；Denied/应用失败均取消票据并退避 1 s。
    if (serial_reconfigure_phase_ ==
        SerialReconfigurePhase::WaitForApproval) {
        const auto permit = maintenance_.permit(serial_maintenance_ticket_,
                                                now_us);
        if (permit == dima::middleware::maintenance::
                          RuntimeMaintenanceCoordinator::Permit::Waiting) {
            return;
        }
        if (permit == dima::middleware::maintenance::
                          RuntimeMaintenanceCoordinator::Permit::Denied) {
            maintenance_.cancel(serial_maintenance_ticket_);
            serial_maintenance_ticket_ = 0U;
            serial_reconfigure_phase_ = SerialReconfigurePhase::Idle;
            serial_retry_after_us_ = now_us + 1000000ULL;
            return;
        }
        serial_reconfigure_phase_ = SerialReconfigurePhase::Apply;
    }

    if (serial_reconfigure_phase_ == SerialReconfigurePhase::Apply) {
        const bool progress = maintenance_.report_progress(
            serial_maintenance_ticket_, 1U, now_us);
        const bool applied = progress && apply_serial_configuration();
        if (applied) {
            maintenance_.complete(serial_maintenance_ticket_);
        } else {
            maintenance_.cancel(serial_maintenance_ticket_);
            serial_retry_after_us_ = services_.clock.now_us() + 1000000ULL;
        }
        serial_maintenance_ticket_ = 0U;
        serial_reconfigure_phase_ = SerialReconfigurePhase::Idle;
    }
}

bool ApplicationContext::start_rc_chain() noexcept
{
    // RC 链启动依赖顺序：SBUS backend -> RCUpdate -> RcManualInput；任一后续模块
    // 失败都逆序停止已启动前缀，不能留下无人消费或陈旧的 input_rc。
    sbus_started_ = module_manager_.start(sbus_rc_);
    if (!sbus_started_) return false;

    rc_update_started_ = module_manager_.start(rc_update_);
    if (!rc_update_started_) {
        (void)stop_sbus();
        return false;
    }

    rc_manual_input_started_ = module_manager_.start(rc_manual_input_);
    if (!rc_manual_input_started_) {
        (void)module_manager_.stop(rc_update_);
        rc_update_started_ = false;
        (void)stop_sbus();
        return false;
    }
    return true;
}

bool ApplicationContext::stop_rc_chain() noexcept
{
    bool stopped = true;
    if (rc_manual_input_started_) {
        const bool result = module_manager_.stop(rc_manual_input_);
        rc_manual_input_started_ = !result;
        stopped = result && stopped;
    }
    if (rc_update_started_) {
        const bool result = module_manager_.stop(rc_update_);
        rc_update_started_ = !result;
        stopped = result && stopped;
    }
    stopped = stop_sbus() && stopped;
    return stopped;
}

bool ApplicationContext::stop_sbus() noexcept
{
    if (!sbus_started_) {
        return true;
    }
    const bool stopped = module_manager_.stop(sbus_rc_) &&
                         sbus_rc_.state() ==
                             dima::middleware::lifecycle::ModuleState::Stopped;
    sbus_started_ = !stopped;
    return stopped;
}

bool ApplicationContext::start_control_chain() noexcept
{
    // 控制链先把 RC setpoint 变成 rover request，再由差速混控生成 actuator_motors。
    manual_mode_started_ = module_manager_.start(manual_mode_);
    if (!manual_mode_started_) {
        return false;
    }

    rover_differential_started_ = module_manager_.start(rover_differential_);
    if (!rover_differential_started_) {
        (void)module_manager_.stop(manual_mode_);
        manual_mode_started_ = false;
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
    if (manual_mode_started_) {
        const bool result = module_manager_.stop(manual_mode_);
        manual_mode_started_ = !result;
        stopped = result && stopped;
    }
    return stopped;
}

bool ApplicationContext::stop_motor_output() noexcept
{
    // MotorOutput 停止成功还必须同时满足模块 Stopped 与六路 GPIO safe-off 证明；
    // 仅 WorkQueue 停止不足以宣称执行器安全。
    if (!motor_output_started_) {
        return true;
    }
    const bool stopped = module_manager_.stop(motor_output_) &&
                         motor_output_.state() ==
                             dima::middleware::lifecycle::ModuleState::Stopped &&
                         motor_output_.safe_off_confirmed();
    motor_output_started_ = !stopped;
    return stopped;
}

bool ApplicationContext::stop_started_modules() noexcept
{
    // 停止按消费者到生产者、执行器到基础设施逆序推进；每个 started 标志只在
    // stop 确认成功后清除，因此部分失败可准确保留未释放所有权。
    bool stopped = true;
    if (serial_maintenance_ticket_ != 0U) {
        maintenance_.cancel(serial_maintenance_ticket_);
        serial_maintenance_ticket_ = 0U;
        serial_reconfigure_phase_ = SerialReconfigurePhase::Idle;
    }
    if (sensor_calibration_started_) {
        const bool result = module_manager_.stop(sensor_calibration_);
        sensor_calibration_started_ = !result;
        stopped = result && stopped;
    }
    if (auto_mode_started_) {
        // 先 drain wq:nav，确保它不再读取 vehicle_local_position/Mission，再停止
        // EKF 和 Mission 生产者。
        const bool result = module_manager_.stop(auto_mode_);
        auto_mode_started_ = !result;
        stopped = result && stopped;
    }
    if (ekf2_started_) {
        // 先 drain estimator callback，再停止 GPS/Mag/IMU 生产者，防止停机窗口内
        // EKF 继续读取已开始释放生命周期的前端。
        const bool result = module_manager_.stop(ekf2_);
        ekf2_started_ = !result;
        stopped = result && stopped;
    }
    if (um982_gps_started_) {
        const bool result = module_manager_.stop(um982_gps_);
        um982_gps_started_ = !result;
        stopped = result && stopped;
    }
    if (dronecan_mag2_started_) {
        const bool result = module_manager_.stop(dronecan_mag2_);
        dronecan_mag2_started_ = !result;
        stopped = result && stopped;
    }
    if (vehicle_magnetometer_started_) {
        const bool result = module_manager_.stop(vehicle_magnetometer_);
        vehicle_magnetometer_started_ = !result;
        stopped = result && stopped;
    }
    if (vehicle_imu_started_) {
        const bool result = module_manager_.stop(vehicle_imu_);
        vehicle_imu_started_ = !result;
        stopped = result && stopped;
    }
    if (icm42688p_started_) {
        const bool result = module_manager_.stop(icm42688p_);
        icm42688p_started_ = !result;
        stopped = result && stopped;
    }
    if (boot_started_) {
        const bool result = module_manager_.stop(boot_health_);
        boot_started_ = !result;
        stopped = result && stopped;
    }
    stopped = stop_control_chain() && stopped;
    stopped = stop_rc_chain() && stopped;
    if (mavlink_started_) {
        const bool result = module_manager_.stop(mavlink_service_);
        mavlink_started_ = !result;
        stopped = result && stopped;
    }
    if (mission_started_) {
        const bool result = module_manager_.stop(mission_service_);
        mission_started_ = !result;
        stopped = result && stopped;
    }
    if (commander_started_) {
        const bool result = module_manager_.stop(commander_);
        commander_started_ = !result;
        stopped = result && stopped;
    }
    stopped = stop_motor_output() && stopped;
    if (serial_config_started_) {
        const bool result = module_manager_.stop(serial_config_) &&
            serial_config_.state() ==
                dima::middleware::lifecycle::ModuleState::Stopped;
        serial_config_started_ = !result;
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

bool ApplicationContext::watchdog_feed_allowed(
    std::uint32_t previous_health_generation,
    std::uint32_t &current_health_generation) const noexcept
{
    current_health_generation = boot_health_.health_generation();
    if (runtime_state_ == RuntimeState::Running) {
        return current_health_generation != 0U &&
               current_health_generation != previous_health_generation;
    }
    /* A successfully stopped Runtime has already proved MotorOutput stopped
     * both timers and restored all six GPIOs low. Keep IWDG alive in this
     * intentional safe-idle state so a same-power Runtime restart remains
     * possible. Error and partial lifecycle states deliberately stop feeding. */
    // Running 需要新健康代数；完整 Stopped 已证明 PWM 定时器关闭且六路 GPIO 拉低，
    // 可继续喂狗支持同电源重启。Error/中间态故意不喂，交给 IWDG 恢复。
    return runtime_state_ == RuntimeState::Stopped;
}

void ApplicationContext::watchdog_feed_completed() noexcept
{
    maintenance_.watchdog_fed(services_.clock.now_us());
}

ApplicationContext &application_context() noexcept
{
    // 函数局部静态是全应用唯一实例；构造依赖已安装的 platform Services。
    static ApplicationContext instance{dima::platform::services()};
    return instance;
}

} // namespace dima::rover
