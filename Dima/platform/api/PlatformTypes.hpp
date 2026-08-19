#pragma once

#include <cstddef>
#include <cstdint>

namespace dima::platform {

using TimeUs = std::uint64_t;
using TimeMs = std::uint64_t;

struct Timeout {
    std::uint64_t microseconds{0U};
    bool infinite{false};

    static constexpr Timeout no_wait() noexcept { return {}; }
    static constexpr Timeout forever() noexcept { return {0U, true}; }
    static constexpr Timeout from_us(std::uint64_t value) noexcept
    {
        return {value, false};
    }
    static constexpr Timeout from_ms(std::uint64_t value) noexcept
    {
        return value > UINT64_MAX / 1000U
                   ? Timeout{UINT64_MAX, false}
                   : Timeout{value * 1000U, false};
    }
};

struct OpaqueHandle {
    std::uintptr_t value{0U};

    constexpr explicit operator bool() const noexcept { return value != 0U; }
};

constexpr bool operator==(OpaqueHandle left, OpaqueHandle right) noexcept
{
    return left.value == right.value;
}

constexpr bool operator!=(OpaqueHandle left, OpaqueHandle right) noexcept
{
    return !(left == right);
}

using MutexHandle = OpaqueHandle;
using SignalHandle = OpaqueHandle;
using TaskHandle = OpaqueHandle;

struct IsrCallback {
    void (*function)(void *context) noexcept{nullptr};
    void *context{nullptr};

    void invoke() const noexcept
    {
        if (function != nullptr) {
            function(context);
        }
    }
};

} // namespace dima::platform
