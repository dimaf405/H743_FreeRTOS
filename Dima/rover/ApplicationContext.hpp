#pragma once

#include "boot_health/BootHealthService.hpp"
#include "logging/LogService.hpp"
#include "mavlink/MavlinkService.hpp"
#include "parameters/ParameterService.hpp"
#include "parameters/ParameterJournal.hpp"
#include "platform/api/Platform.hpp"
#include "control/RoverDifferential.hpp"
#include "rc/RcManualInput.hpp"
#include "modes/ManualMode.hpp"
#include "motor/MotorOutput.hpp"
#include "rc/RCUpdate.hpp"
#include "rc/SbusRc.hpp"
#include "safety/Commander.hpp"
#include "lifecycle/module_manager.hpp"

namespace dima::rover {

class ApplicationContext {
public:
    explicit ApplicationContext(dima::platform::Services &services) noexcept;

    bool init() noexcept;
    bool start() noexcept;
    bool shutdown() noexcept;

private:
    enum class RuntimeState : std::uint8_t {
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

    dima::platform::Services &services_;
    dima::parameters::ParameterJournal journal_;
    dima::middleware::lifecycle::ModuleManager module_manager_{};
    dima::modules::boot_health::BootHealthService boot_health_;
    dima::modules::logging::LogService log_service_;
    dima::modules::mavlink::MavlinkService mavlink_service_;
    dima::modules::parameters::ParameterService parameter_service_;
    dima::modules::motor::MotorOutput motor_output_;
    dima::modules::safety::Commander commander_;
    dima::modules::rc::SbusRc sbus_rc_;
    dima::modules::rc::RCUpdate rc_update_{};
    dima::modules::rc::RcManualInput rc_manual_input_{};
    dima::rover::modes::ManualMode manual_mode_{};
    dima::rover::control::RoverDifferential rover_differential_{};
    dima::platform::TaskHandle owner_task_{};
    RuntimeState runtime_state_{RuntimeState::Stopped};
    bool console_initialized_{false};
    bool work_queue_initialized_{false};
    bool uorb_initialized_{false};
    bool parameter_initialized_{false};
    bool modules_registered_{false};
    bool boot_started_{false};
    bool log_started_{false};
    bool mavlink_started_{false};
    bool parameter_started_{false};
    bool motor_output_started_{false};
    bool commander_started_{false};
    bool sbus_started_{false};
    bool rc_update_started_{false};
    bool rc_manual_input_started_{false};
    bool manual_mode_started_{false};
    bool rover_differential_started_{false};
};

ApplicationContext &application_context() noexcept;

} // namespace dima::rover
