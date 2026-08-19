#pragma once

#include "platform/api/ActuatorPwm.hpp"
#include "platform/api/Boot.hpp"
#include "platform/api/Console.hpp"
#include "platform/api/Execution.hpp"
#include "platform/api/Flash.hpp"
#include "platform/api/Memory.hpp"
#include "platform/api/SensorInterrupts.hpp"
#include "platform/api/Serial.hpp"

#include <cstdint>

namespace dima::platform::stm32h7 {

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
SbusInput &sbus_input() noexcept;
SensorInterrupts &sensor_interrupts() noexcept;
ActuatorPwm &actuator_pwm() noexcept;

/** 读取 STM32H7 的 96-bit UID，并将低两个 word 合成为 64-bit 板级标识。 */
std::uint64_t board_hardware_uid() noexcept;

} // namespace dima::platform::stm32h7
