#pragma once

#include "api/ActuatorPwm.hpp"
#include "api/Boot.hpp"
#include "api/Can.hpp"
#include "api/Console.hpp"
#include "api/Execution.hpp"
#include "api/Flash.hpp"
#include "api/Memory.hpp"
#include "api/SensorInterrupts.hpp"
#include "api/Serial.hpp"
#include "api/Spi.hpp"

#include <cstdint>

namespace dima::platform::stm32h7 {

/* 以下函数返回板级单例引用；仅组合根负责调用 initialize 并把它们装入 Services，
 * 业务模块不得直接依赖 STM32H7 命名空间。 */
bool clock_initialize() noexcept;
MonotonicClock &clock() noexcept;
DmaMemory &dma_memory() noexcept;
IndependentWatchdog &independent_watchdog() noexcept;
bool flash_initialize() noexcept;
FlashPartition &parameter_partition() noexcept;
ConsoleTransport &usb_console_transport() noexcept;
BootControl &boot_control(FlashTransactionManager &transactions,
                          ArmedFlashCoordinator &armed_flash) noexcept;
SerialPorts &serial_ports() noexcept;
TimestampedSerialInput &timestamped_serial_input() noexcept;
AsyncSerialPort &async_serial_port() noexcept;
InterruptSources &interrupt_sources() noexcept;
SpiDevice &spi4() noexcept;
CanTransport &can1() noexcept;
ActuatorPwm &actuator_pwm() noexcept;

/** 读取 STM32H7 的 96-bit UID，并将低两个 word 合成为 64-bit 板级标识。 */
std::uint64_t board_hardware_uid() noexcept;

} // namespace dima::platform::stm32h7
