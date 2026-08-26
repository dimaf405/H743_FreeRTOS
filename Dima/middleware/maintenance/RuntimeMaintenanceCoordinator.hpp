#pragma once

#include "api/Execution.hpp"
#include "api/PlatformTypes.hpp"

#include <cstdint>

namespace dima::middleware::maintenance {


class RuntimeMaintenanceCoordinator final {
public:
    using Ticket = std::uint32_t;

    enum class Permit : std::uint8_t {
        Waiting,
        Ready,
        Denied,
    };

    explicit RuntimeMaintenanceCoordinator(
        dima::platform::CriticalSection &critical) noexcept;

    Ticket request(dima::platform::TimeUs now_us) noexcept;
    bool boot_health_update(dima::platform::TimeUs now_us,
                            bool runtime_healthy,
                            bool maintenance_safe) noexcept;
    void watchdog_fed(dima::platform::TimeUs now_us) noexcept;
    Permit permit(Ticket ticket, dima::platform::TimeUs now_us) noexcept;
    bool report_progress(Ticket ticket, std::uint32_t progress,
                         dima::platform::TimeUs now_us) noexcept;
    void complete(Ticket ticket) noexcept;
    void cancel(Ticket ticket) noexcept;
    bool in_progress() const noexcept;

private:
    /* 维护票据状态机：Idle --request--> Requested --健康/安全--> Approved
     * --permit--> Active --complete/cancel/超时--> Idle/Cancelled。
     * 15 s 是整笔硬截止，Active 每 750 ms 必须报告不同 progress，防止 Flash/SD
     * 长操作在看门狗仍被喂养时无界卡住。 */
    enum class State : std::uint8_t {
        Idle,
        Requested,
        Approved,
        Active,
        Cancelled,
    };

    static constexpr dima::platform::TimeUs kHardDeadlineUs = 15000000ULL;
    static constexpr dima::platform::TimeUs kProgressTimeoutUs = 750000ULL;

    bool deadline_expired(dima::platform::TimeUs now_us) const noexcept;
    void reset_locked() noexcept;

    dima::platform::CriticalSection &critical_;
    Ticket ticket_{0U};
    Ticket next_ticket_{0U};
    std::uint32_t progress_{0U};
    dima::platform::TimeUs deadline_us_{0U};
    dima::platform::TimeUs last_progress_us_{0U};
    State state_{State::Idle};
};

} // namespace dima::middleware::maintenance
