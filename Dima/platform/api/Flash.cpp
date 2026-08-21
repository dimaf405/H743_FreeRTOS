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
    if (flash_busy_ || maintenance_busy_) {
        return false;
    }
    armed_ = true;
    return true;
}

void ArmedFlashCoordinator::disarm() noexcept
{
    CriticalGuard guard{critical_};
    armed_ = false;
}

bool ArmedFlashCoordinator::begin_flash() noexcept
{
    CriticalGuard guard{critical_};
    if (armed_ || flash_busy_) {
        return false;
    }
    flash_busy_ = true;
    return true;
}

void ArmedFlashCoordinator::end_flash() noexcept
{
    CriticalGuard guard{critical_};
    flash_busy_ = false;
}

bool ArmedFlashCoordinator::begin_maintenance() noexcept
{
    CriticalGuard guard{critical_};
    if (armed_ || flash_busy_ || maintenance_busy_) {
        return false;
    }
    maintenance_busy_ = true;
    return true;
}

void ArmedFlashCoordinator::end_maintenance() noexcept
{
    CriticalGuard guard{critical_};
    maintenance_busy_ = false;
}

bool ArmedFlashCoordinator::armed() const noexcept
{
    CriticalGuard guard{critical_};
    return armed_;
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
