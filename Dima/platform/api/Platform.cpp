#include "Platform.hpp"

namespace dima::platform {
namespace {

Services *g_services{nullptr};

} // namespace

CriticalGuard::CriticalGuard() noexcept
{
    if (g_services != nullptr) {
        section_ = &g_services->critical;
        token_ = section_->enter();
    }
}

CriticalGuard::CriticalGuard(CriticalSection &section) noexcept
    : section_(&section), token_(section.enter())
{
}

CriticalGuard::~CriticalGuard()
{
    if (section_ != nullptr) {
        section_->leave(token_);
    }
}

Mutex::~Mutex() { reset(); }

bool Mutex::initialize(Synchronization &synchronization,
                       MutexKind kind) noexcept
{
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
    if (synchronization_ != nullptr && handle_) {
        synchronization_->notify_from_isr(handle_);
    }
}

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

bool install_services(Services &value) noexcept
{
    if (g_services != nullptr) {
        return g_services == &value;
    }
    g_services = &value;
    return true;
}

bool services_installed() noexcept { return g_services != nullptr; }
Services *try_services() noexcept { return g_services; }

Services &services() noexcept
{
    if (g_services == nullptr) {
        for (;;) {
        }
    }
    return *g_services;
}

bool in_interrupt_context() noexcept
{
    return g_services != nullptr && g_services->execution.in_interrupt();
}

bool in_realtime_context() noexcept
{
    return g_services != nullptr && g_services->execution.in_realtime_task();
}

TimeUs platform_time_us() noexcept
{
    return g_services != nullptr ? g_services->clock.now_us() : 0U;
}

TimeMs platform_time_ms() noexcept { return platform_time_us() / 1000U; }

void *allocate(std::size_t size, AllocationDomain domain) noexcept
{
    return g_services != nullptr ? g_services->heap.allocate(size, domain)
                                 : nullptr;
}

void deallocate(void *pointer) noexcept
{
    if (g_services != nullptr) {
        g_services->heap.deallocate(pointer);
    }
}

HeapStats heap_stats() noexcept
{
    return g_services != nullptr ? g_services->heap.stats() : HeapStats{};
}

} // namespace dima::platform
