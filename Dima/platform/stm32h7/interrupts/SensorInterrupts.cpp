#include "stm32h7/HardwareServices.hpp"

#include "board_bus_resources.h"
#include "main.h"

namespace dima::platform::stm32h7 {
namespace {

constexpr std::uint32_t kExtiIrqPriority = 6U;
constexpr std::uint16_t kSourcePins[kInterruptSourceCount]{
    DIMA_INTERRUPT_SOURCE1_Pin,
    DIMA_INTERRUPT_SOURCE2_Pin,
};
constexpr std::uint32_t kSourcePinsMask = kSourcePins[0] | kSourcePins[1];

/* EXTI15_10 的唯一注册点。注册/注销/consume 在任务上下文执行，on_exti 在 ISR
 * 更新累计计数、末次时间和本批 pending 位，然后只发一次轻量通知。 */
class Stm32InterruptSources final : public InterruptSources {
public:
    bool register_sources(IsrCallback notification) noexcept override
    {
        if (__get_IPSR() != 0U || !clock().initialized() ||
            notification.function == nullptr) {
            return false;
        }
        const std::uint32_t primask = __get_PRIMASK();
        __disable_irq();
        /* 允许首次注册或同一 callback 的幂等注册，拒绝第二个消费者抢占中断源。 */
        const bool accepted = notification_.function == nullptr ||
                              (notification_.function == notification.function &&
                               notification_.context == notification.context);
        if (accepted) {
            notification_ = notification;
            pending_mask_ = InterruptSourceNone;
            clear_pending();
            NVIC_SetPriority(EXTI15_10_IRQn, kExtiIrqPriority);
            NVIC_EnableIRQ(EXTI15_10_IRQn);
        }
        if (primask == 0U) {
            __enable_irq();
        }
        return accepted;
    }

    void unregister_sources() noexcept override
    {
        if (__get_IPSR() != 0U) {
            return;
        }
        const std::uint32_t primask = __get_PRIMASK();
        __disable_irq();
        NVIC_DisableIRQ(EXTI15_10_IRQn);
        clear_pending();
        notification_ = {};
        pending_mask_ = InterruptSourceNone;
        if (primask == 0U) {
            __enable_irq();
        }
    }

    InterruptSourceSnapshot consume() noexcept override
    {
        InterruptSourceSnapshot snapshot{};
        if (__get_IPSR() != 0U) {
            return snapshot;
        }
        const std::uint32_t primask = __get_PRIMASK();
        __disable_irq();
        /* 临界区内取得一致快照；count/timestamp 保持累计，只清 pending 批次位，
         * 因此消费者可用 count 差值发现两次调度之间的多次边沿。 */
        snapshot.pending_mask = pending_mask_;
        for (std::size_t source = 0U; source < kInterruptSourceCount;
             ++source) {
            snapshot.count[source] = count_[source];
            snapshot.timestamp_us[source] = timestamp_us_[source];
        }
        pending_mask_ = InterruptSourceNone;
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
        /* 同一 EXTI 入口中同时 pending 的源共用一个时间戳，避免读取时钟开销造成
         * 人为先后顺序。 */
        const std::uint64_t timestamp = clock().now_us();
        bool matched = false;
        for (std::size_t source = 0U; source < kInterruptSourceCount;
             ++source) {
            if ((pending_pins & kSourcePins[source]) == 0U) {
                continue;
            }
            timestamp_us_[source] = timestamp;
            ++count_[source];
            pending_mask_ |= static_cast<std::uint32_t>(InterruptSource1)
                             << source;
            matched = true;
        }
        if (!matched) return;
        __DMB();
        notification_.invoke();
    }

private:
    static void clear_pending() noexcept
    {
        /* 同时清 EXTI 外设位与 NVIC pending，再用屏障保证重新使能前清除已生效。 */
        __HAL_GPIO_EXTI_CLEAR_IT(kSourcePinsMask);
        NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
        __DSB();
        __ISB();
    }

    IsrCallback notification_{};
    volatile std::uint32_t pending_mask_{InterruptSourceNone};
    volatile std::uint32_t count_[kInterruptSourceCount]{};
    volatile std::uint64_t timestamp_us_[kInterruptSourceCount]{};
};

Stm32InterruptSources &instance() noexcept
{
    static Stm32InterruptSources value;
    return value;
}

} // namespace

InterruptSources &interrupt_sources() noexcept { return instance(); }

} // namespace dima::platform::stm32h7

extern "C" void dima_interrupt_sources_exti_isr(std::uint16_t pending_pins)
{
    dima::platform::stm32h7::instance().on_exti(pending_pins);
}
