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
    failure_reason_ = FailureReason::None;
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
            cancel_locked(FailureReason::RuntimeUnhealthy);
        }
        return false;
    }

    if (state_ == State::Idle) {
        return true;
    }
    if (state_ == State::Cancelled) {
        return false;
    }
    if (deadline_expired(now_us)) {
        cancel_locked(FailureReason::HardDeadline);
        return false;
    }
    if (state_ == State::Requested) {
        if (maintenance_safe) {
            state_ = State::Approved;
        }
        return true;
    }
    if (!maintenance_safe) {
        cancel_locked(FailureReason::MaintenanceUnsafe);
        return false;
    }
    if (state_ == State::Active &&
        (last_progress_us_ == 0U || now_us < last_progress_us_ ||
         now_us - last_progress_us_ > kProgressTimeoutUs)) {
        cancel_locked(FailureReason::ProgressTimeout);
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
        cancel_locked(FailureReason::HardDeadline);
    }
}

RuntimeMaintenanceCoordinator::Permit
RuntimeMaintenanceCoordinator::permit(
    Ticket ticket, dima::platform::TimeUs now_us) noexcept
{
    dima::platform::CriticalGuard guard{critical_};
    if (ticket == 0U || ticket != ticket_ || state_ == State::Idle) {
        return Permit::Denied;
    }
    if (state_ == State::Cancelled) {
        return Permit::Denied;
    }
    if (deadline_expired(now_us)) {
        cancel_locked(FailureReason::HardDeadline);
        return Permit::Denied;
    }
    return state_ == State::Active ? Permit::Ready : Permit::Waiting;
}

bool RuntimeMaintenanceCoordinator::report_progress(
    Ticket ticket, std::uint32_t progress,
    dima::platform::TimeUs now_us) noexcept
{
    dima::platform::CriticalGuard guard{critical_};
    if (ticket == 0U || ticket != ticket_ || state_ == State::Idle) {
        return false;
    }
    if (state_ == State::Cancelled) {
        return false;
    }
    if (deadline_expired(now_us)) {
        cancel_locked(FailureReason::HardDeadline);
        return false;
    }
    if (state_ != State::Active || progress <= progress_) {
        cancel_locked(FailureReason::InvalidProgress);
        return false;
    }
    progress_ = progress;
    last_progress_us_ = now_us;
    return true;
}

RuntimeMaintenanceCoordinator::FailureReason
RuntimeMaintenanceCoordinator::failure_reason(Ticket ticket) const noexcept
{
    dima::platform::CriticalGuard guard{critical_};
    // 无效票据不能读取另一事务的失败原因，避免并发调用方的故障证据串线。
    if (ticket == 0U || ticket != ticket_) {
        return FailureReason::InvalidTicket;
    }
    return failure_reason_;
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

void RuntimeMaintenanceCoordinator::cancel_locked(FailureReason reason) noexcept
{
    // 第一原因最接近实际触发点；后续 permit/report 只能观察，不得覆盖它。
    if (failure_reason_ == FailureReason::None) {
        failure_reason_ = reason;
    }
    state_ = State::Cancelled;
}

void RuntimeMaintenanceCoordinator::reset_locked() noexcept
{
    ticket_ = 0U;
    progress_ = 0U;
    deadline_us_ = 0U;
    last_progress_us_ = 0U;
    state_ = State::Idle;
    failure_reason_ = FailureReason::None;
}

} // namespace dima::middleware::maintenance
