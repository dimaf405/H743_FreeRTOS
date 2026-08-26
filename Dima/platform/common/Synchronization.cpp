#include "api/Synchronization.hpp"

namespace dima::platform {

Mutex::~Mutex() { reset(); }

bool Mutex::initialize(Synchronization &synchronization,
                       MutexKind kind) noexcept
{
    /* 已初始化对象只接受同一后端的幂等调用；不在运行中迁移原生句柄。 */
    if (handle_) {
        return synchronization_ == &synchronization;
    }
    const MutexHandle handle = synchronization.create_mutex(kind);
    if (!handle) {
        return false;
    }
    synchronization_ = &synchronization;
    handle_ = handle;
    return true;
}

void Mutex::reset() noexcept
{
    /* 先销毁后端对象，再清能力句柄和后端指针，析构路径可安全重复执行。 */
    if (synchronization_ != nullptr && handle_) {
        synchronization_->destroy_mutex(handle_);
    }
    handle_ = {};
    synchronization_ = nullptr;
}

bool Mutex::lock(Timeout timeout) noexcept
{
    return synchronization_ != nullptr && handle_ &&
           synchronization_->lock(handle_, timeout);
}

void Mutex::unlock() noexcept
{
    if (synchronization_ != nullptr && handle_) {
        synchronization_->unlock(handle_);
    }
}

RecursiveMutex::~RecursiveMutex() { reset(); }

bool RecursiveMutex::initialize(Synchronization &synchronization) noexcept
{
    return mutex_.initialize(synchronization, MutexKind::Recursive);
}

void RecursiveMutex::reset() noexcept { mutex_.reset(); }

bool RecursiveMutex::lock(Timeout timeout) noexcept
{
    return mutex_.lock(timeout);
}

void RecursiveMutex::unlock() noexcept { mutex_.unlock(); }

MutexGuard::MutexGuard(Mutex &mutex, Timeout timeout) noexcept
    : mutex_(&mutex), locked_(mutex.lock(timeout))
{
}

MutexGuard::MutexGuard(RecursiveMutex &mutex, Timeout timeout) noexcept
    : recursive_(&mutex), locked_(mutex.lock(timeout))
{
}

MutexGuard::~MutexGuard()
{
    /* 仅当构造期确实获得锁时释放，超时 guard 不会误解锁其他所有者。 */
    if (!locked_) {
        return;
    }
    if (mutex_ != nullptr) {
        mutex_->unlock();
    } else if (recursive_ != nullptr) {
        recursive_->unlock();
    }
}

Signal::~Signal() { reset(); }

bool Signal::initialize(Synchronization &synchronization) noexcept
{
    if (handle_) {
        return synchronization_ == &synchronization;
    }
    const SignalHandle handle = synchronization.create_signal();
    if (!handle) {
        return false;
    }
    synchronization_ = &synchronization;
    handle_ = handle;
    return true;
}

void Signal::reset() noexcept
{
    if (synchronization_ != nullptr && handle_) {
        synchronization_->destroy_signal(handle_);
    }
    handle_ = {};
    synchronization_ = nullptr;
}

bool Signal::wait(Timeout timeout) noexcept
{
    return synchronization_ != nullptr && handle_ &&
           synchronization_->wait(handle_, timeout);
}

void Signal::notify() noexcept
{
    if (synchronization_ != nullptr && handle_) {
        synchronization_->notify(handle_);
    }
}

void Signal::notify_from_isr() noexcept
{
    /* 显式走 ISR 原语，是否触发高优先级任务切换由具体后端决定。 */
    if (synchronization_ != nullptr && handle_) {
        synchronization_->notify_from_isr(handle_);
    }
}

} // namespace dima::platform
