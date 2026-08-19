#include "platform/stm32h7/HardwareServices.hpp"

#include "platform/api/platform_config.h"

#include "stm32h7xx_hal.h"

namespace dima::platform::stm32h7 {
namespace {

constexpr std::uint32_t kTimerFrequencyHz = 1000000U;
constexpr std::uint32_t kExpectedTimerInputClockHz = 240000000U;

class Stm32Clock final : public MonotonicClock {
public:
    bool initialize() noexcept
    {
        if (initialized_) {
            return true;
        }
        const std::uint32_t timer_clock = timer_input_clock_hz();
        if (timer_clock != kExpectedTimerInputClockHz ||
            timer_clock < kTimerFrequencyHz ||
            (timer_clock % kTimerFrequencyHz) != 0U) {
            return false;
        }

        __HAL_RCC_TIM2_CLK_ENABLE();
        TIM2->CR1 = 0U;
        TIM2->CR2 = 0U;
        TIM2->SMCR = 0U;
        TIM2->DIER = 0U;
        TIM2->CCMR1 = 0U;
        TIM2->CCMR2 = 0U;
        TIM2->CCER = 0U;
        TIM2->CCR1 = 0U;
        TIM2->PSC = timer_clock / kTimerFrequencyHz - 1U;
        TIM2->ARR = UINT32_MAX;
        TIM2->CNT = 0U;
        TIM2->EGR = TIM_EGR_UG;
        TIM2->SR = 0U;

        overflow_high_ = 0U;
        sequence_ = 0U;
        NVIC_SetPriority(TIM2_IRQn, DIMA_MAX_SYSCALL_INTERRUPT_PRIORITY);
        NVIC_ClearPendingIRQ(TIM2_IRQn);
        NVIC_EnableIRQ(TIM2_IRQn);
        TIM2->DIER = TIM_DIER_UIE;
        TIM2->CR1 = TIM_CR1_CEN;
        initialized_ = true;
        return true;
    }

    bool initialized() const noexcept override { return initialized_; }

    TimeUs now_us() const noexcept override
    {
        if (!initialized_) {
            return 0U;
        }
        std::uint32_t sequence_before;
        std::uint32_t sequence_after;
        std::uint32_t high;
        std::uint32_t low;
        std::uint32_t status;
        do {
            sequence_before = sequence_;
            __DMB();
            high = overflow_high_;
            low = TIM2->CNT;
            status = TIM2->SR;
            __DMB();
            sequence_after = sequence_;
        } while (sequence_before != sequence_after ||
                 (sequence_before & 1U) != 0U);

        if ((status & TIM_SR_UIF) != 0U) {
            ++high;
            low = TIM2->CNT;
        }
        return (static_cast<TimeUs>(high) << 32U) | low;
    }

    void overflow_isr() noexcept
    {
        if ((TIM2->SR & TIM_SR_UIF) == 0U) {
            return;
        }
        ++sequence_;
        __DMB();
        TIM2->SR = ~TIM_SR_UIF;
        ++overflow_high_;
        __DMB();
        ++sequence_;
    }

private:
    static std::uint32_t timer_input_clock_hz() noexcept
    {
        RCC_ClkInitTypeDef clocks{};
        std::uint32_t flash_latency = 0U;
        HAL_RCC_GetClockConfig(&clocks, &flash_latency);
        const std::uint32_t pclk = HAL_RCC_GetPCLK1Freq();
        return clocks.APB1CLKDivider == RCC_APB1_DIV1 ? pclk : pclk * 2U;
    }

    volatile std::uint32_t overflow_high_{0U};
    volatile std::uint32_t sequence_{0U};
    bool initialized_{false};
};

Stm32Clock &instance() noexcept
{
    static Stm32Clock value;
    return value;
}

} // namespace

bool clock_initialize() noexcept { return instance().initialize(); }
MonotonicClock &clock() noexcept { return instance(); }

} // namespace dima::platform::stm32h7

extern "C" void dima_hrt_overflow_isr(void)
{
    static_cast<dima::platform::stm32h7::Stm32Clock &>(
        dima::platform::stm32h7::clock()).overflow_isr();
}
