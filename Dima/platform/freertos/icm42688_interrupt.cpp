#include "icm42688_interrupt.hpp"

#include "freertos/hrt.hpp"
#include "main.h"
#include "work_queue/WorkQueue.hpp"

extern "C" {
#include "FreeRTOS.h"
#include "task.h"
}

namespace dima::platform {
namespace {

px4::WorkItem *volatile g_consumer{nullptr};
volatile std::uint32_t g_pending_mask{Icm42688InterruptNone};
volatile std::uint32_t g_int1_count{0U};
volatile std::uint32_t g_int2_count{0U};
volatile std::uint64_t g_int1_timestamp_us{0U};
volatile std::uint64_t g_int2_timestamp_us{0U};

bool in_isr() noexcept
{
    return __get_IPSR() != 0U;
}

constexpr std::uint32_t kExtiIrqPriority = 6U;
constexpr std::uint32_t kInterruptPins =
    ICM42688_INT1_Pin | ICM42688_INT2_Pin;
static_assert(kExtiIrqPriority >=
                  configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY &&
              kExtiIrqPriority <= configLIBRARY_LOWEST_INTERRUPT_PRIORITY,
              "ICM42688 EXTI priority must permit FreeRTOS FromISR calls");

void clear_exti_pending() noexcept
{
    __HAL_GPIO_EXTI_CLEAR_IT(kInterruptPins);
    NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
}

void enable_exti_irq() noexcept
{
    clear_exti_pending();
    NVIC_SetPriority(EXTI15_10_IRQn, kExtiIrqPriority);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void disable_exti_irq() noexcept
{
    NVIC_DisableIRQ(EXTI15_10_IRQn);
    clear_exti_pending();
    __DSB();
    __ISB();
}

} // namespace

bool icm42688_interrupt_register(px4::WorkItem &consumer) noexcept
{
    if (in_isr() || !hrt_is_initialized() ||
        xTaskGetSchedulerState() != taskSCHEDULER_RUNNING) {
        return false;
    }

    bool accepted = false;
    taskENTER_CRITICAL();
    if (g_consumer == nullptr) {
        g_consumer = &consumer;
        g_pending_mask = Icm42688InterruptNone;
        __DMB();
        enable_exti_irq();
        accepted = true;
    } else if (g_consumer == &consumer) {
        accepted = true;
    }
    taskEXIT_CRITICAL();
    return accepted;
}

void icm42688_interrupt_unregister(px4::WorkItem &consumer) noexcept
{
    if (in_isr()) {
        return;
    }

    taskENTER_CRITICAL();
    if (g_consumer == &consumer) {
        disable_exti_irq();
        g_consumer = nullptr;
        g_pending_mask = Icm42688InterruptNone;
    }
    taskEXIT_CRITICAL();
}

Icm42688InterruptSnapshot icm42688_interrupt_consume() noexcept
{
    Icm42688InterruptSnapshot snapshot{};
    if (in_isr()) {
        return snapshot;
    }

    taskENTER_CRITICAL();
    snapshot.pending_mask = g_pending_mask;
    snapshot.int1_count = g_int1_count;
    snapshot.int2_count = g_int2_count;
    snapshot.int1_timestamp_us = g_int1_timestamp_us;
    snapshot.int2_timestamp_us = g_int2_timestamp_us;
    g_pending_mask = Icm42688InterruptNone;
    taskEXIT_CRITICAL();
    return snapshot;
}

} // namespace dima::platform

extern "C" void dima_icm42688_exti_isr(std::uint16_t pending_pins)
{
    using namespace dima::platform;

    px4::WorkItem *const consumer = g_consumer;
    if (consumer == nullptr || !hrt_is_initialized()) {
        return;
    }

    const std::uint64_t timestamp_us = hrt_absolute_time();
    if ((pending_pins & ICM42688_INT1_Pin) != 0U) {
        g_int1_timestamp_us = timestamp_us;
        ++g_int1_count;
        g_pending_mask |= Icm42688InterruptInt1;
    }
    if ((pending_pins & ICM42688_INT2_Pin) != 0U) {
        g_int2_timestamp_us = timestamp_us;
        ++g_int2_count;
        g_pending_mask |= Icm42688InterruptInt2;
    }
    __DMB();

    (void)consumer->ScheduleNowFromISR();
}
