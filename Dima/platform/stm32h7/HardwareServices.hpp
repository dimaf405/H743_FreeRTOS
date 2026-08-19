#pragma once

#include "platform/api/Platform.hpp"

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

/**
 * Read the STM32H7 96-bit unique device ID and combine into a 64-bit value.
 * Uses the lower 64 bits of the 96-bit UID (word0 | word1<<32).
 */
uint64_t board_hardware_uid() noexcept;

} // namespace dima::platform::stm32h7
