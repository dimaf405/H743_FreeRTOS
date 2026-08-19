#include "platform/stm32h7/HardwareServices.hpp"

#include "stm32h7xx.h"

namespace dima::platform::stm32h7 {
namespace {

constexpr std::uint32_t kWriteAccessKey = 0x5555U;
constexpr std::uint32_t kStartKey = 0xCCCCU;
constexpr std::uint32_t kReloadKey = 0xAAAAU;
constexpr std::uint32_t kPrescalerDiv32 = 3U;
constexpr std::uint32_t kMinimumTimeoutMs = 20U;
constexpr std::uint32_t kMaximumTimeoutMs = 4096U;
constexpr std::uint32_t kRegisterUpdatePollLimit = 1000000U;

class Stm32IndependentWatchdog final : public IndependentWatchdog {
public:
    bool start(std::uint32_t timeout_ms) noexcept override
    {
        if (timeout_ms < kMinimumTimeoutMs || timeout_ms > kMaximumTimeoutMs) {
            return false;
        }
        if (active_) {
            return timeout_ms == timeout_ms_;
        }

        /* Start the counter before requesting register updates. On a cold
         * boot the LSI kernel clock is still stopped, so PR/RLR/WINR update
         * flags cannot clear until the start key enables it. Repeating the
         * start key is harmless when MCUboot carried an active IWDG across a
         * reset. This is also the ordering used by the STM32 HAL. */
        IWDG1->KR = kStartKey;
        IWDG1->KR = kWriteAccessKey;
        IWDG1->PR = kPrescalerDiv32;
        IWDG1->RLR = timeout_ms - 1U;
        IWDG1->WINR = IWDG_WINR_WIN;
        std::uint32_t polls = 0U;
        while (IWDG1->SR != 0U && polls < kRegisterUpdatePollLimit) {
            ++polls;
        }
        if (IWDG1->SR != 0U) {
            return false;
        }

        /* A halted debugger must not turn an intentional inspection into a
         * watchdog reset. This freeze bit has no effect during normal run. */
        DBGMCU->APB4FZ1 |= DBGMCU_APB4FZ1_DBG_IWDG1;
        IWDG1->KR = kReloadKey;
        timeout_ms_ = timeout_ms;
        active_ = true;
        return true;
    }

    void feed() noexcept override
    {
        if (active_) {
            IWDG1->KR = kReloadKey;
        }
    }

    bool active() const noexcept override { return active_; }

private:
    std::uint32_t timeout_ms_{0U};
    bool active_{false};
};

Stm32IndependentWatchdog &instance() noexcept
{
    static Stm32IndependentWatchdog value;
    return value;
}

} // namespace

IndependentWatchdog &independent_watchdog() noexcept { return instance(); }

} // namespace dima::platform::stm32h7
