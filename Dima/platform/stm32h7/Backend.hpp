#pragma once

#include "platform/api/Platform.hpp"

namespace dima::platform::stm32h7 {

bool clock_initialize() noexcept;
MonotonicClock &clock() noexcept;
DmaMemory &dma_memory() noexcept;
bool flash_initialize() noexcept;
FlashPartition &parameter_partition() noexcept;
ConsoleTransport &usb_console_transport() noexcept;
BootControl &boot_control(FlashTransactionManager &transactions,
                          ArmedFlashCoordinator &armed_flash) noexcept;
SbusInput &sbus_input() noexcept;
SensorInterrupts &sensor_interrupts() noexcept;

} // namespace dima::platform::stm32h7
