#include "RuntimeMaintenanceCoordinator.hpp"

#include <cstdint>

namespace dima::middleware::maintenance {

RuntimeMaintenanceCoordinator::RuntimeMaintenanceCoordinator(
    dima::platform::CriticalSection &critical) noexcept
    : critical_(critical)
{
}

RuntimeMaintenanceCoordinator::Ticket
RuntimeMaintenanceCoordinator::request(
    dima::platform::TimeUs now_us) noexcept
{
    dima::platform::CriticalGuard guard{critical_};
    if (state_ != State::Idle) {
        return 0U;
    }

    ++next_ticket_;
    if (next_ticket_ == 0U) {
        ++next_ticket_;
    }
    ticket_ = next_ticket_;
    progress_ = 0U;
    last_progress_us_ = 0U;
    deadline_us_ = now_us > UINT64_MAX - kHardDeadlineUs
                       ? UINT64_MAX
                       : now_us + kHardDeadlineUs;
    state_ = State::Requested;
    return ticket_;
}

bool RuntimeMaintenanceCoordinator::boot_health_update(
    dima::platform::TimeUs now_us, bool runtime_healthy,
    bool maintenance_safe) noexcept
{
    dima::platform::CriticalGuard guard{critical_};
    if (!runtime_healthy) {
        if (state_ != State::Idle) {
            state_ = State::Cancelled;
        }
        return false;
    }

    if (state_ == State::Idle) {
        return true;
    }
    if (state_ == State::Cancelled || deadline_expired(now_us)) {
        state_ = State::Cancelled;
        return false;
    }
    if (state_ == State::Requested) {
        if (maintenance_safe) {
            state_ = State::Approved;
        }
        return true;
    }
    if (!maintenance_safe) {
        state_ = State::Cancelled;
        return false;
    }
    if (state_ == State::Active &&
        (last_progress_us_ == 0U || now_us < last_progress_us_ ||
         now_us - last_progress_us_ > kProgressTimeoutUs)) {
        state_ = State::Cancelled;
        return false;
    }
    return true;
}

void RuntimeMaintenanceCoordinator::watchdog_fed(
    dima::platform::TimeUs now_us) noexcept
{
    dima::platform::CriticalGuard guard{critical_};
    if (state_ == State::Approved && !deadline_expired(now_us)) {
        state_ = State::Active;
        last_progress_us_ = now_us;
    } else if (state_ == State::Approved) {
        state_ = State::Cancelled;
    }
}

RuntimeMaintenanceCoordinator::Permit
RuntimeMaintenanceCoordinator::permit(
    Ticket ticket, dima::platform::TimeUs now_us) noexcept
{
    dima::platform::CriticalGuard guard{critical_};
    if (ticket == 0U || ticket != ticket_ || state_ == State::Cancelled ||
        state_ == State::Idle || deadline_expired(now_us)) {
        if (ticket != 0U && ticket == ticket_ && state_ != State::Idle) {
            state_ = State::Cancelled;
        }
        return Permit::Denied;
    }
    return state_ == State::Active ? Permit::Ready : Permit::Waiting;
}

bool RuntimeMaintenanceCoordinator::report_progress(
    Ticket ticket, std::uint32_t progress,
    dima::platform::TimeUs now_us) noexcept
{
    dima::platform::CriticalGuard guard{critical_};
    if (ticket == 0U || ticket != ticket_ || state_ != State::Active ||
        deadline_expired(now_us) || progress <= progress_) {
        if (ticket != 0U && ticket == ticket_ && state_ != State::Idle) {
            state_ = State::Cancelled;
        }
        return false;
    }
    progress_ = progress;
    last_progress_us_ = now_us;
    return true;
}

void RuntimeMaintenanceCoordinator::complete(Ticket ticket) noexcept
{
    dima::platform::CriticalGuard guard{critical_};
    if (ticket != 0U && ticket == ticket_ && state_ != State::Idle) {
        reset_locked();
    }
}

void RuntimeMaintenanceCoordinator::cancel(Ticket ticket) noexcept
{
    dima::platform::CriticalGuard guard{critical_};
    if (ticket != 0U && ticket == ticket_) {
        reset_locked();
    }
}

bool RuntimeMaintenanceCoordinator::in_progress() const noexcept
{
    dima::platform::CriticalGuard guard{critical_};
    return state_ != State::Idle;
}

bool RuntimeMaintenanceCoordinator::deadline_expired(
    dima::platform::TimeUs now_us) const noexcept
{
    return deadline_us_ == 0U || now_us > deadline_us_;
}

void RuntimeMaintenanceCoordinator::reset_locked() noexcept
{
    ticket_ = 0U;
    progress_ = 0U;
    deadline_us_ = 0U;
    last_progress_us_ = 0U;
    state_ = State::Idle;
}

} // namespace dima::middleware::maintenance
