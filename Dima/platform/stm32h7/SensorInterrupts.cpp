#include "Backend.hpp"

#include "main.h"

namespace dima::platform::stm32h7 {
namespace {

constexpr std::uint32_t kExtiIrqPriority = 6U;
constexpr std::uint32_t kInterruptPins =
    ICM42688_INT1_Pin | ICM42688_INT2_Pin;

class Stm32SensorInterrupts final : public SensorInterrupts {
public:
    bool register_icm42688(IsrCallback notification) noexcept override
    {
        if (__get_IPSR() != 0U || !clock().initialized() ||
            notification.function == nullptr) {
            return false;
        }
        const std::uint32_t primask = __get_PRIMASK();
        __disable_irq();
        const bool accepted = notification_.function == nullptr ||
                              (notification_.function == notification.function &&
                               notification_.context == notification.context);
        if (accepted) {
            notification_ = notification;
            pending_mask_ = Icm42688InterruptNone;
            clear_pending();
            NVIC_SetPriority(EXTI15_10_IRQn, kExtiIrqPriority);
            NVIC_EnableIRQ(EXTI15_10_IRQn);
        }
        if (primask == 0U) {
            __enable_irq();
        }
        return accepted;
    }

    void unregister_icm42688() noexcept override
    {
        if (__get_IPSR() != 0U) {
            return;
        }
        const std::uint32_t primask = __get_PRIMASK();
        __disable_irq();
        NVIC_DisableIRQ(EXTI15_10_IRQn);
        clear_pending();
        notification_ = {};
        pending_mask_ = Icm42688InterruptNone;
        if (primask == 0U) {
            __enable_irq();
        }
    }

    Icm42688InterruptSnapshot consume_icm42688() noexcept override
    {
        Icm42688InterruptSnapshot snapshot{};
        if (__get_IPSR() != 0U) {
            return snapshot;
        }
        const std::uint32_t primask = __get_PRIMASK();
        __disable_irq();
        snapshot.pending_mask = pending_mask_;
        snapshot.int1_count = int1_count_;
        snapshot.int2_count = int2_count_;
        snapshot.int1_timestamp_us = int1_timestamp_us_;
        snapshot.int2_timestamp_us = int2_timestamp_us_;
        pending_mask_ = Icm42688InterruptNone;
        if (primask == 0U) {
            __enable_irq();
        }
        return snapshot;
    }

    void on_exti(std::uint16_t pending_pins) noexcept
    {
        if (notification_.function == nullptr || !clock().initialized()) {
            return;
        }
        const std::uint64_t timestamp = clock().now_us();
        if ((pending_pins & ICM42688_INT1_Pin) != 0U) {
            int1_timestamp_us_ = timestamp;
            ++int1_count_;
            pending_mask_ |= Icm42688InterruptInt1;
        }
        if ((pending_pins & ICM42688_INT2_Pin) != 0U) {
            int2_timestamp_us_ = timestamp;
            ++int2_count_;
            pending_mask_ |= Icm42688InterruptInt2;
        }
        __DMB();
        notification_.invoke();
    }

private:
    static void clear_pending() noexcept
    {
        __HAL_GPIO_EXTI_CLEAR_IT(kInterruptPins);
        NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
        __DSB();
        __ISB();
    }

    IsrCallback notification_{};
    volatile std::uint32_t pending_mask_{Icm42688InterruptNone};
    volatile std::uint32_t int1_count_{0U};
    volatile std::uint32_t int2_count_{0U};
    volatile std::uint64_t int1_timestamp_us_{0U};
    volatile std::uint64_t int2_timestamp_us_{0U};
};

Stm32SensorInterrupts &instance() noexcept
{
    static Stm32SensorInterrupts value;
    return value;
}

} // namespace

SensorInterrupts &sensor_interrupts() noexcept { return instance(); }

} // namespace dima::platform::stm32h7

extern "C" void dima_icm42688_exti_isr(std::uint16_t pending_pins)
{
    dima::platform::stm32h7::instance().on_exti(pending_pins);
}
