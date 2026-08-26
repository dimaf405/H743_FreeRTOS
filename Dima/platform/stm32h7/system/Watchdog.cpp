#include "stm32h7/HardwareServices.hpp"

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
        /* LSI 约 32 kHz，/32 后约 1 kHz，故 RLR=timeout_ms-1；LSI 有器差，毫秒值
         * 是安全窗口配置而非高精度计时。重复 start 只接受完全相同的超时合同。 */
        if (timeout_ms < kMinimumTimeoutMs || timeout_ms > kMaximumTimeoutMs) {
            return false;
        }
        if (active_) {
            return timeout_ms == timeout_ms_;
        }

        /* 冷启动时 LSI 内核时钟可能尚未运行，必须先写 start key，再开放并更新
         * PR/RLR/WINR；若 MCUboot 已跨复位启动 IWDG，重复 start key 仍是幂等的。 */
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

        /* 调试器 halt 时冻结 IWDG，避免有意断点被误判为运行故障；正常运行不受影响。 */
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
