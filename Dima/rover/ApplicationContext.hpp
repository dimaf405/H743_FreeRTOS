#pragma once

#include "boot_health/BootHealthService.hpp"
#include "dronecan_mag2/DroneCanMag2.hpp"
#include "magnetometer/VehicleMagnetometer.hpp"
#include "logging/LogService.hpp"
#include "um982/Um982Gps.hpp"
#include "mavlink/MavlinkService.hpp"
#include "mission/MissionService.hpp"
#include "parameters/ParameterService.hpp"
#include "parameters/flashfs.h"
#include "api/PlatformTypes.hpp"
#include "api/Services.hpp"
#include "control/RoverDifferential.hpp"
#include "rc/RcManualInput.hpp"
#include "modes/AutoMode.hpp"
#include "modes/ManualMode.hpp"
#include "motor/MotorOutput.hpp"
#include "rc/RCUpdate.hpp"
#include "sbus/SbusRc.hpp"
#include "safety/Commander.hpp"
#include "serial/SerialConfig.hpp"
#include "icm42688p/ICM42688P.hpp"
#include "imu/VehicleImu.hpp"
#include "ekf2/Ekf2.hpp"
#include "calibration/SensorCalibration.hpp"
#include "lifecycle/module_manager.hpp"
#include "maintenance/RuntimeMaintenanceCoordinator.hpp"

namespace dima::rover {

// 应用组合根：静态拥有全部模块及其依赖，并串行管理“资源初始化、模块启动、
// 运行期维护、逆序停机”。所有生命周期调用必须来自绑定的 appMain owner task。
class ApplicationContext {
public:
    explicit ApplicationContext(dima::platform::Services &services) noexcept;

    bool init() noexcept;
    bool start() noexcept;
    bool shutdown() noexcept;
    void service() noexcept;
    bool watchdog_feed_allowed(
        std::uint32_t previous_health_generation,
        std::uint32_t &current_health_generation) const noexcept;
    void watchdog_feed_completed() noexcept;

private:
    enum class RuntimeState : std::uint8_t {
        // Stopped -> Initializing -> Initialized -> Starting -> Running；
        // shutdown 进入 Stopping。任意无法完整回滚的失败锁存 Error。
        Stopped,
        Initializing,
        Initialized,
        Starting,
        Running,
        Stopping,
        Error,
    };

    bool owner_call(bool bind_if_unset) noexcept;
    bool register_modules() noexcept;
    bool release_runtime_resources() noexcept;
    bool rollback_initialization() noexcept;
    bool rollback_start() noexcept;
    bool stop_started_modules() noexcept;
    bool start_rc_chain() noexcept;
    bool stop_rc_chain() noexcept;
    bool stop_sbus() noexcept;
    bool start_control_chain() noexcept;
    bool stop_control_chain() noexcept;
    bool stop_motor_output() noexcept;
    bool apply_serial_configuration() noexcept;

    // 成员声明顺序也是构造/析构依赖顺序：底层协调器与 FlashFS 先于消费者，
    // ModuleManager 不拥有对象，只登记这些静态生命周期实例。
    dima::platform::Services &services_;
    dima::middleware::maintenance::RuntimeMaintenanceCoordinator
        maintenance_;
    dima::parameters::FlashFS flashfs_;
    dima::middleware::lifecycle::ModuleManager module_manager_{};
    dima::modules::boot_health::BootHealthService boot_health_;
    dima::modules::logging::LogService log_service_;
    dima::modules::parameters::ParameterService parameter_service_;
    dima::modules::mission::MissionService mission_service_;
    dima::modules::mavlink::MavlinkService mavlink_service_;
    dima::modules::serial::SerialConfig serial_config_;
    dima::drivers::gps::Um982Gps um982_gps_;
    dima::drivers::imu::ICM42688P icm42688p_;
    dima::modules::sensors::VehicleImu vehicle_imu_;
    dima::modules::sensors::VehicleMagnetometer vehicle_magnetometer_;
    dima::modules::sensors::SensorCalibration sensor_calibration_;
    dima::drivers::magnetometer::DroneCanMag2 dronecan_mag2_;
    dima::modules::ekf2::Ekf2 ekf2_{};
    dima::modules::motor::MotorOutput motor_output_;
    dima::modules::safety::Commander commander_;
    dima::drivers::rc::SbusRc sbus_rc_;
    dima::modules::rc::RCUpdate rc_update_{};
    dima::modules::rc::RcManualInput rc_manual_input_{};
    dima::rover::modes::ManualMode manual_mode_{};
    dima::rover::modes::AutoMode auto_mode_;
    dima::rover::control::RoverDifferential rover_differential_{};
    dima::platform::TaskHandle owner_task_{};
    RuntimeState runtime_state_{RuntimeState::Stopped};
    bool console_initialized_{false};
    bool work_queue_initialized_{false};
    bool uorb_initialized_{false};
    bool log_initialized_{false};
    bool parameter_initialized_{false};
    bool modules_registered_{false};
    bool boot_started_{false};
    bool log_started_{false};
    bool parameter_started_{false};
    bool mission_started_{false};
    bool mavlink_started_{false};
    bool serial_config_started_{false};
    bool um982_gps_started_{false};
    bool icm42688p_started_{false};
    bool vehicle_imu_started_{false};
    bool vehicle_magnetometer_started_{false};
    bool sensor_calibration_started_{false};
    bool dronecan_mag2_started_{false};
    bool ekf2_started_{false};
    bool motor_output_started_{false};
    bool commander_started_{false};
    bool sbus_started_{false};
    bool rc_update_started_{false};
    bool rc_manual_input_started_{false};
    bool manual_mode_started_{false};
    bool auto_mode_started_{false};
    bool rover_differential_started_{false};
    enum class SerialReconfigurePhase : std::uint8_t {
        // 参数签名变化后：Idle -> WaitForApproval -> Apply -> Idle。应用前停止 GPS/
        // RC 串口消费者，且只在未武装、无其他 maintenance 事务时执行。
        Idle,
        WaitForApproval,
        Apply,
    };
    SerialReconfigurePhase serial_reconfigure_phase_{
        SerialReconfigurePhase::Idle};
    dima::middleware::maintenance::RuntimeMaintenanceCoordinator::Ticket
        serial_maintenance_ticket_{0U};
    std::uint64_t active_serial_signature_{0U};
    std::uint64_t serial_retry_after_us_{0U};
};

ApplicationContext &application_context() noexcept;

} // namespace dima::rover
