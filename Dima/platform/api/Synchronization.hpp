#pragma once

#include "PlatformTypes.hpp"

namespace dima::platform {

enum class MutexKind : std::uint8_t {
    Normal,
    Recursive,
};

class Synchronization {
public:
    virtual ~Synchronization() = default;
    virtual MutexHandle create_mutex(MutexKind kind) noexcept = 0;
    virtual void destroy_mutex(MutexHandle handle) noexcept = 0;
    virtual bool lock(MutexHandle handle, Timeout timeout) noexcept = 0;
    virtual void unlock(MutexHandle handle) noexcept = 0;

    virtual SignalHandle create_signal() noexcept = 0;
    virtual void destroy_signal(SignalHandle handle) noexcept = 0;
    virtual bool wait(SignalHandle handle, Timeout timeout) noexcept = 0;
    virtual void notify(SignalHandle handle) noexcept = 0;
    virtual void notify_from_isr(SignalHandle handle) noexcept = 0;
};

class Mutex final {
public:
    Mutex() noexcept = default;
    ~Mutex();

    bool initialize(Synchronization &synchronization,
                    MutexKind kind = MutexKind::Normal) noexcept;
    void reset() noexcept;
    bool lock(Timeout timeout = Timeout::forever()) noexcept;
    void unlock() noexcept;
    bool valid() const noexcept { return static_cast<bool>(handle_); }

    Mutex(const Mutex &) = delete;
    Mutex &operator=(const Mutex &) = delete;

private:
    Synchronization *synchronization_{nullptr};
    MutexHandle handle_{};
};

class RecursiveMutex final {
public:
    RecursiveMutex() noexcept = default;
    ~RecursiveMutex();

    bool initialize(Synchronization &synchronization) noexcept;
    void reset() noexcept;
    bool lock(Timeout timeout = Timeout::forever()) noexcept;
    void unlock() noexcept;
    bool valid() const noexcept { return mutex_.valid(); }

    RecursiveMutex(const RecursiveMutex &) = delete;
    RecursiveMutex &operator=(const RecursiveMutex &) = delete;

private:
    Mutex mutex_{};
};

class MutexGuard final {
public:
    explicit MutexGuard(Mutex &mutex,
                        Timeout timeout = Timeout::forever()) noexcept;
    explicit MutexGuard(RecursiveMutex &mutex,
                        Timeout timeout = Timeout::forever()) noexcept;
    ~MutexGuard();

    explicit operator bool() const noexcept { return locked_; }
    MutexGuard(const MutexGuard &) = delete;
    MutexGuard &operator=(const MutexGuard &) = delete;

private:
    Mutex *mutex_{nullptr};
    RecursiveMutex *recursive_{nullptr};
    bool locked_{false};
};

class Signal final {
public:
    Signal() noexcept = default;
    ~Signal();

    bool initialize(Synchronization &synchronization) noexcept;
    void reset() noexcept;
    bool wait(Timeout timeout = Timeout::forever()) noexcept;
    void notify() noexcept;
    void notify_from_isr() noexcept;
    bool valid() const noexcept { return static_cast<bool>(handle_); }

    Signal(const Signal &) = delete;
    Signal &operator=(const Signal &) = delete;

private:
    Synchronization *synchronization_{nullptr};
    SignalHandle handle_{};
};

} // namespace dima::platform
