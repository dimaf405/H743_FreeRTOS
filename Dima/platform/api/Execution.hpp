#pragma once

#include "PlatformTypes.hpp"

namespace dima::platform {

struct CriticalToken {
    std::uintptr_t state{0U};
    bool from_interrupt{false};
};

class MonotonicClock {
public:
    virtual ~MonotonicClock() = default;
    virtual bool initialized() const noexcept = 0;
    virtual TimeUs now_us() const noexcept = 0;
    TimeMs now_ms() const noexcept { return now_us() / 1000U; }
};

class ExecutionContext {
public:
    virtual ~ExecutionContext() = default;
    virtual bool in_interrupt() const noexcept = 0;
    virtual bool scheduler_running() const noexcept = 0;
    virtual bool in_realtime_task() const noexcept = 0;
};

class CriticalSection {
public:
    virtual ~CriticalSection() = default;
    virtual CriticalToken enter() noexcept = 0;
    virtual void leave(CriticalToken token) noexcept = 0;
};

class CriticalGuard final {
public:
    CriticalGuard() noexcept;
    explicit CriticalGuard(CriticalSection &section) noexcept;
    ~CriticalGuard();

    CriticalGuard(const CriticalGuard &) = delete;
    CriticalGuard &operator=(const CriticalGuard &) = delete;

private:
    CriticalSection *section_{nullptr};
    CriticalToken token_{};
};

bool in_interrupt_context() noexcept;
bool in_realtime_context() noexcept;
TimeUs platform_time_us() noexcept;
TimeMs platform_time_ms() noexcept;

} // namespace dima::platform
