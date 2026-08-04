#include "hrt.hpp"

#include "boot_diagnostics.h"
#include "FreeRTOS.h"
#include "stm32h7xx_hal.h"

namespace {

constexpr std::uint32_t kTimerFrequencyHz = 1000000U;
constexpr std::uint32_t kExpectedTimerInputClockHz = 240000000U;
static_assert(configTICK_RATE_HZ == 1000U,
              "the shared HAL and FreeRTOS SysTick must remain at 1 kHz");
static_assert(configUSE_TICKLESS_IDLE == 0,
              "TIM2 HRT needs an explicit low-power compensation design");
volatile std::uint32_t g_overflow_high{0U};
volatile std::uint32_t g_sequence{0U};
bool g_initialized{false};

std::uint32_t tim2_input_clock_hz() noexcept
{
    RCC_ClkInitTypeDef clocks{};
    std::uint32_t flash_latency = 0U;
    HAL_RCC_GetClockConfig(&clocks, &flash_latency);
    const std::uint32_t pclk = HAL_RCC_GetPCLK1Freq();
    return clocks.APB1CLKDivider == RCC_APB1_DIV1 ? pclk : pclk * 2U;
}

} // namespace

bool hrt_init() noexcept
{
    if (g_initialized) {
        return true;
    }

    const std::uint32_t timer_clock = tim2_input_clock_hz();
    dima_boot_detail_set(timer_clock);
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
    TIM2->ARR = 0xFFFFFFFFU;
    TIM2->CNT = 0U;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->SR = 0U;

    g_overflow_high = 0U;
    g_sequence = 0U;
    NVIC_SetPriority(TIM2_IRQn,
                     configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
    NVIC_ClearPendingIRQ(TIM2_IRQn);
    NVIC_EnableIRQ(TIM2_IRQn);
    TIM2->DIER = TIM_DIER_UIE;
    TIM2->CR1 = TIM_CR1_CEN;
    g_initialized = true;
    return true;
}

hrt_abstime hrt_absolute_time() noexcept
{
    if (!g_initialized) {
        return 0U;
    }

    std::uint32_t sequence_before;
    std::uint32_t sequence_after;
    std::uint32_t high;
    std::uint32_t low;
    std::uint32_t status;
    do {
        sequence_before = g_sequence;
        __DMB();
        high = g_overflow_high;
        low = TIM2->CNT;
        status = TIM2->SR;
        __DMB();
        sequence_after = g_sequence;
    } while ((sequence_before != sequence_after) ||
             ((sequence_before & 1U) != 0U));

    // 中断尚未获准执行时，UIF 表示已经跨过一次 32 位回绕。
    if ((status & TIM_SR_UIF) != 0U) {
        ++high;
        low = TIM2->CNT;
    }
    return (static_cast<hrt_abstime>(high) << 32U) | low;
}

hrt_abstime hrt_elapsed_time(const hrt_abstime *then) noexcept
{
    return then == nullptr ? 0U : hrt_absolute_time() - *then;
}

std::uint64_t hrt_absolute_time_ms() noexcept
{
    return hrt_absolute_time() / 1000ULL;
}

extern "C" void dima_hrt_overflow_isr(void)
{
    if ((TIM2->SR & TIM_SR_UIF) == 0U) {
        return;
    }
    ++g_sequence;
    __DMB();
    TIM2->SR = ~TIM_SR_UIF;
    ++g_overflow_high;
    __DMB();
    ++g_sequence;
}

namespace px4 {
std::uint64_t work_queue_time_us() noexcept
{
    return hrt_absolute_time();
}
} // namespace px4
