#include "logging.hpp"

#include "platform/api/Platform.hpp"

#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <limits>

extern "C" {
const char *__px4_log_level_str[_PX4_LOG_LEVEL_PANIC + 1] = {
    "DEBUG", "INFO", "WARN", "ERROR", "PANIC"};
}

namespace dima::logging {
namespace {

constexpr std::size_t kFormatBufferSize = 256U;
constexpr std::size_t kServiceChunkSize = 256U;

struct LogState {
    std::uint8_t data[kLogRingCapacity]{};
    std::size_t head{0U};
    std::size_t count{0U};
    std::uint32_t records_written{0U};
    std::uint32_t records_truncated{0U};
    std::uint32_t formatting_rejections{0U};
    std::uint32_t overwritten_bytes{0U};
    std::uint32_t service_dropped_bytes{0U};
};

LogState g_state{};

void increment_saturated(std::uint32_t &value,
                         std::uint32_t amount = 1U) noexcept
{
    const std::uint32_t available =
        std::numeric_limits<std::uint32_t>::max() - value;
    value += (amount > available) ? available : amount;
}

bool formatting_allowed() noexcept
{
    if (dima::platform::in_interrupt_context()) {
        return false;
    }

    return !dima::platform::in_realtime_context();
}

void push_bytes_locked(const std::uint8_t *data, std::size_t length) noexcept
{
    if (length >= kLogRingCapacity) {
        const std::size_t skipped = length - kLogRingCapacity;
        increment_saturated(
            g_state.overwritten_bytes,
            static_cast<std::uint32_t>(std::min<std::size_t>(
                skipped + g_state.count,
                std::numeric_limits<std::uint32_t>::max())));
        data += skipped;
        length = kLogRingCapacity;
        g_state.head = 0U;
        g_state.count = 0U;
    }

    const std::size_t required = g_state.count + length;
    if (required > kLogRingCapacity) {
        const std::size_t overwritten = required - kLogRingCapacity;
        g_state.head = (g_state.head + overwritten) % kLogRingCapacity;
        g_state.count -= overwritten;
        increment_saturated(
            g_state.overwritten_bytes,
            static_cast<std::uint32_t>(overwritten));
    }

    std::size_t tail = (g_state.head + g_state.count) % kLogRingCapacity;
    const std::size_t first = std::min(length, kLogRingCapacity - tail);
    std::memcpy(&g_state.data[tail], data, first);
    if (length > first) {
        std::memcpy(&g_state.data[0], data + first, length - first);
    }
    g_state.count += length;
}

std::size_t pop_bytes(std::uint8_t *destination,
                      std::size_t capacity) noexcept
{
    dima::platform::CriticalGuard lock;
    const std::size_t length = std::min(capacity, g_state.count);
    if (length == 0U) {
        return 0U;
    }

    const std::size_t first =
        std::min(length, kLogRingCapacity - g_state.head);
    std::memcpy(destination, &g_state.data[g_state.head], first);
    if (length > first) {
        std::memcpy(destination + first, &g_state.data[0], length - first);
    }

    g_state.head = (g_state.head + length) % kLogRingCapacity;
    g_state.count -= length;
    return length;
}

} // namespace

namespace {

WriteResult write_v(Level level, const char *module_name, bool raw,
                    const char *format, va_list arguments) noexcept
{
    if (format == nullptr || (!raw && module_name == nullptr)) {
        return WriteResult::InvalidArgument;
    }
    if (!formatting_allowed()) {
        dima::platform::CriticalGuard lock;
        increment_saturated(g_state.formatting_rejections);
        return WriteResult::RejectedRealtime;
    }

    char buffer[kFormatBufferSize]{};
    std::size_t prefix_length = 0U;
    if (!raw) {
        const auto index = static_cast<std::size_t>(level);
        if (index > _PX4_LOG_LEVEL_PANIC) {
            return WriteResult::InvalidArgument;
        }
        const int prefix = std::snprintf(buffer, sizeof(buffer), "%-5s [%s] ",
                                         __px4_log_level_str[index], module_name);
        if (prefix < 0) {
            return WriteResult::InvalidArgument;
        }
        prefix_length = std::min<std::size_t>(static_cast<std::size_t>(prefix),
                                               sizeof(buffer) - 1U);
    }

    const int body_result = std::vsnprintf(buffer + prefix_length,
                                           sizeof(buffer) - prefix_length,
                                           format, arguments);
    if (body_result < 0) {
        return WriteResult::InvalidArgument;
    }

    const std::size_t body_capacity = sizeof(buffer) - prefix_length;
    const bool truncated = static_cast<std::size_t>(body_result) >= body_capacity;
    std::size_t length = std::min(prefix_length + static_cast<std::size_t>(body_result),
                                  sizeof(buffer) - 1U);

    // PX4 module logs always append one newline; PX4_INFO_RAW keeps exact bytes.
    if (!raw) {
        if (length < sizeof(buffer) - 1U) {
            buffer[length++] = '\n';
        } else {
            buffer[sizeof(buffer) - 1U] = '\n';
            length = sizeof(buffer);
        }
    }

    dima::platform::CriticalGuard lock;
    push_bytes_locked(reinterpret_cast<const std::uint8_t *>(buffer), length);
    increment_saturated(g_state.records_written);
    if (truncated) {
        increment_saturated(g_state.records_truncated);
    }
    return truncated ? WriteResult::Truncated : WriteResult::Ok;
}

} // namespace

WriteResult writef(Level level, const char *format, ...) noexcept
{
    va_list arguments;
    va_start(arguments, format);
    const WriteResult result = write_v(level, "dima", false, format, arguments);
    va_end(arguments);
    return result;
}

WriteResult write_literal(const char *text, std::size_t length) noexcept
{
    if ((text == nullptr) && (length != 0U)) {
        return WriteResult::InvalidArgument;
    }

    if (length == 0U) {
        return WriteResult::Ok;
    }

    dima::platform::CriticalGuard lock;
    push_bytes_locked(reinterpret_cast<const std::uint8_t *>(text), length);
    increment_saturated(g_state.records_written);
    return WriteResult::Ok;
}

std::size_t service_flush(const ServiceWriter &writer,
                          std::size_t max_bytes) noexcept
{
    if ((writer.write == nullptr) || (max_bytes == 0U) ||
        dima::platform::in_interrupt_context() ||
        dima::platform::in_realtime_context()) {
        return 0U;
    }

    std::uint8_t chunk[kServiceChunkSize]{};
    std::size_t total_written = 0U;

    while (total_written < max_bytes) {
        const std::size_t request = std::min(
            sizeof(chunk), max_bytes - total_written);
        const std::size_t popped = pop_bytes(chunk, request);
        if (popped == 0U) {
            break;
        }

        const std::size_t accepted =
            std::min(writer.write(writer.context, chunk, popped), popped);
        total_written += accepted;

        if (accepted < popped) {
            dima::platform::CriticalGuard lock;
            increment_saturated(
                g_state.service_dropped_bytes,
                static_cast<std::uint32_t>(popped - accepted));
            break;
        }
    }

    return total_written;
}

LogStats stats() noexcept
{
    dima::platform::CriticalGuard lock;
    return LogStats{
        g_state.records_written,
        g_state.records_truncated,
        g_state.formatting_rejections,
        g_state.overwritten_bytes,
        g_state.service_dropped_bytes,
        g_state.count,
    };
}

void reset() noexcept
{
    dima::platform::CriticalGuard lock;
    g_state = LogState{};
}

} // namespace dima::logging



extern "C" void px4_log_initialize(void)
{
    dima::logging::reset();
}

extern "C" void px4_log_modulename(int level, const char *module_name,
                                    const char *format, ...)
{
    if (level < _PX4_LOG_LEVEL_DEBUG || level > _PX4_LOG_LEVEL_PANIC) {
        return;
    }
    va_list arguments;
    va_start(arguments, format);
    (void)dima::logging::write_v(static_cast<dima::logging::Level>(level),
                                 module_name, false, format, arguments);
    va_end(arguments);
}

extern "C" void px4_log_raw(int level, const char *format, ...)
{
    if (level < _PX4_LOG_LEVEL_DEBUG || level > _PX4_LOG_LEVEL_PANIC) {
        return;
    }
    va_list arguments;
    va_start(arguments, format);
    (void)dima::logging::write_v(static_cast<dima::logging::Level>(level),
                                 nullptr, true, format, arguments);
    va_end(arguments);
}
