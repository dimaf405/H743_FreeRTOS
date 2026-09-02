#pragma once

namespace dima::platform {

class ActuatorPwm;
class ArmedFlashCoordinator;
class AsyncSerialPort;
class BootControl;
class CanTransport;
class Console;
class CriticalSection;
class DmaMemory;
class ExecutionContext;
class FlashPartition;
class FlashTransactionManager;
class Heap;
class IndependentWatchdog;
class LogFileStore;
class MonotonicClock;
class AtomicFileStore;
class InterruptSources;
class SerialPorts;
class SpiDevice;
class StartupDiagnostics;
class Synchronization;
class TimestampedSerialInput;
class TaskRuntime;

struct Services {
    /* 服务表仅保存引用，不拥有对象；组合根必须保证所有后端覆盖整个应用生命期。
     * actuator_pwm 允许为空，用于无执行器板型，其余服务均为启动硬依赖。 */
    MonotonicClock &clock;
    ExecutionContext &execution;
    CriticalSection &critical;
    Synchronization &synchronization;
    TaskRuntime &tasks;
    Heap &heap;
    FlashTransactionManager &flash_transactions;
    FlashPartition &parameter_partition;
    AtomicFileStore &atomic_files;
    LogFileStore &log_files;
    ArmedFlashCoordinator &armed_flash;
    Console &console;
    BootControl &boot_control;
    StartupDiagnostics &diagnostics;
    DmaMemory &dma;
    IndependentWatchdog &watchdog;
    SerialPorts &serial_ports;
    AsyncSerialPort &async_serial_port;
    TimestampedSerialInput &timestamped_serial_input;
    InterruptSources &interrupt_sources;
    SpiDevice &spi;
    CanTransport &can;
    ActuatorPwm *actuator_pwm;
};

bool install_services(Services &services) noexcept;
/* try_services 用于启动早期的可选查询；services() 是已安装后的强合同，未安装
 * 调用会 fail-stop，避免业务继续使用空后端。 */
bool services_installed() noexcept;
Services *try_services() noexcept;
Services &services() noexcept;

} // namespace dima::platform
