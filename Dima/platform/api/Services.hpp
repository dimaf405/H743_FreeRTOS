#pragma once

namespace dima::platform {

class ActuatorPwm;
class ArmedFlashCoordinator;
class BootControl;
class Console;
class CriticalSection;
class DmaMemory;
class ExecutionContext;
class FlashPartition;
class FlashTransactionManager;
class Heap;
class IndependentWatchdog;
class MonotonicClock;
class ParameterFileStore;
class SbusInput;
class SensorInterrupts;
class SerialPorts;
class StartupDiagnostics;
class Synchronization;
class TaskRuntime;

struct Services {
    MonotonicClock &clock;
    ExecutionContext &execution;
    CriticalSection &critical;
    Synchronization &synchronization;
    TaskRuntime &tasks;
    Heap &heap;
    FlashTransactionManager &flash_transactions;
    FlashPartition &parameter_partition;
    ParameterFileStore &parameter_files;
    ArmedFlashCoordinator &armed_flash;
    Console &console;
    BootControl &boot_control;
    StartupDiagnostics &diagnostics;
    DmaMemory &dma;
    IndependentWatchdog &watchdog;
    SerialPorts &serial_ports;
    SbusInput &sbus;
    SensorInterrupts &sensor_interrupts;
    ActuatorPwm *actuator_pwm;
};

bool install_services(Services &services) noexcept;
bool services_installed() noexcept;
Services *try_services() noexcept;
Services &services() noexcept;

} // namespace dima::platform
