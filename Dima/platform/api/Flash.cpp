#include "Flash.hpp"

#include "Execution.hpp"

namespace dima::platform {

FlashTransaction::FlashTransaction(FlashTransactionManager &manager,
                                   Timeout timeout) noexcept
    : manager_(&manager), acquired_(manager.acquire(timeout))
{
}

FlashTransaction::~FlashTransaction()
{
    if (acquired_) {
        manager_->release();
    }
}

ArmedFlashCoordinator::ArmedFlashCoordinator(
    CriticalSection &critical) noexcept
    : critical_(critical)
{
}

bool ArmedFlashCoordinator::try_arm() noexcept
{
    CriticalGuard guard{critical_};
    if (state_ == State::FlashBusy) {
        return false;
    }
    state_ = State::Armed;
    return true;
}

void ArmedFlashCoordinator::disarm() noexcept
{
    CriticalGuard guard{critical_};
    if (state_ == State::Armed) {
        state_ = State::Idle;
    }
}

bool ArmedFlashCoordinator::begin_flash() noexcept
{
    CriticalGuard guard{critical_};
    if (state_ != State::Idle) {
        return false;
    }
    state_ = State::FlashBusy;
    return true;
}

void ArmedFlashCoordinator::end_flash() noexcept
{
    CriticalGuard guard{critical_};
    if (state_ == State::FlashBusy) {
        state_ = State::Idle;
    }
}

bool ArmedFlashCoordinator::armed() const noexcept
{
    CriticalGuard guard{critical_};
    return state_ == State::Armed;
}

bool ArmedFlashCoordinator::flash_busy() const noexcept
{
    CriticalGuard guard{critical_};
    return state_ == State::FlashBusy;
}

FlashWriteLease::FlashWriteLease(
    ArmedFlashCoordinator &coordinator) noexcept
    : coordinator_(&coordinator), acquired_(coordinator.begin_flash())
{
}

FlashWriteLease::~FlashWriteLease()
{
    if (acquired_) {
        coordinator_->end_flash();
    }
}

} // namespace dima::platform
