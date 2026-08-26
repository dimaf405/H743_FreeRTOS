#include "api/Flash.hpp"

#include "api/Execution.hpp"

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
    /* 互斥矩阵：armed 与 flash_busy/maintenance_busy 不得重叠。加锁只保护状态
     * 转移，不覆盖实际 Flash 操作，避免长临界区屏蔽中断。 */
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
    /* 普通写事务在已 armed 时拒绝；maintenance 由更外层生命周期协调，二者
     * 通过 begin_maintenance 的检查避免与写事务并发。 */
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
    /* RAII lease 确保所有返回路径都清除 flash_busy。 */
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
