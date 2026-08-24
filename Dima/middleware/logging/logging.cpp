#include "logging.hpp"

#include "lib/format/Format.hpp"
#include "platform/api/Execution.hpp"

#include <algorithm>
#include <cstdarg>
#include <limits>

extern "C" {
const char *__px4_log_level_str[_PX4_LOG_LEVEL_PANIC + 1] = {
    "DEBUG", "INFO", "WARN", "ERROR", "PANIC"};
}

namespace dima::logging {
namespace {

constexpr std::size_t kFormatBufferSize = 256U;

struct LogState {
    std::uint32_t records_written{0U};
    std::uint32_t records_truncated{0U};
    std::uint32_t formatting_rejections{0U};
    std::uint32_t sink_dropped_records{0U};
};

struct StructuredSinkState {
    StructuredSink sink{nullptr};
    void *context{nullptr};
};

LogState g_state{};
StructuredSinkState g_sink{};

void increment_saturated(std::uint32_t &value) noexcept
{
    if (value != std::numeric_limits<std::uint32_t>::max()) {
        ++value;
    }
}

bool formatting_allowed() noexcept
{
    return !::dima::platform::in_interrupt_context() &&
           !::dima::platform::in_realtime_context();
}

void record_sink_drop() noexcept
{
    ::dima::platform::CriticalGuard lock;
    increment_saturated(g_state.sink_dropped_records);
}

} // namespace

void set_structured_sink(void *context, StructuredSink sink) noexcept
{
    ::dima::platform::CriticalGuard lock;
    g_sink.context = context;
    g_sink.sink = sink;
}

WriteResult write_v(Source source, Level level, const char *module_name, bool raw,
                    const char *format, va_list arguments) noexcept
{
    if (format == nullptr || (!raw && module_name == nullptr)) {
        return WriteResult::InvalidArgument;
    }
    if (level == Level::Off || (!raw && !config::enabled(source, level))) {
        return WriteResult::Filtered;
    }
    if (!formatting_allowed()) {
        ::dima::platform::CriticalGuard lock;
        increment_saturated(g_state.formatting_rejections);
        return WriteResult::RejectedRealtime;
    }

    char buffer[kFormatBufferSize]{};
    const int body_result = ::dima::format::vformat_to(
        buffer, sizeof(buffer), format, arguments);
    if (body_result < 0) {
        return WriteResult::InvalidArgument;
    }

    const bool truncated =
        static_cast<std::size_t>(body_result) >= sizeof(buffer);
    std::size_t length = std::min(
        static_cast<std::size_t>(body_result), sizeof(buffer) - 1U);
    while (length > 0U &&
           (buffer[length - 1U] == '\n' || buffer[length - 1U] == '\r')) {
        --length;
    }

    StructuredSinkState sink{};
    {
        ::dima::platform::CriticalGuard lock;
        increment_saturated(g_state.records_written);
        if (truncated) {
            increment_saturated(g_state.records_truncated);
        }
        sink = g_sink;
    }

    if (length == 0U || sink.sink == nullptr ||
        !sink.sink(sink.context, level, buffer, length)) {
        record_sink_drop();
    }
    return truncated ? WriteResult::Truncated : WriteResult::Ok;
}

WriteResult writef(Level level, const char *format, ...) noexcept
{
    va_list arguments;
    va_start(arguments, format);
    const WriteResult result = write_v(
        Source::System, level, "dima", false, format, arguments);
    va_end(arguments);
    return result;
}

WriteResult write_module(Source source, Level level, const char *module_name,
                         const char *format, ...) noexcept
{
    va_list arguments;
    va_start(arguments, format);
    const WriteResult result = write_v(
        source, level, module_name, false, format, arguments);
    va_end(arguments);
    return result;
}

LogStats stats() noexcept
{
    ::dima::platform::CriticalGuard lock;
    return LogStats{
        g_state.records_written,
        g_state.records_truncated,
        g_state.formatting_rejections,
        g_state.sink_dropped_records,
    };
}

void reset() noexcept
{
    ::dima::platform::CriticalGuard lock;
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
    (void)dima::logging::write_v(
        dima::logging::Source::System,
        static_cast<dima::logging::Level>(level), module_name, false, format,
        arguments);
    va_end(arguments);
}

extern "C" void px4_log_raw(int level, const char *format, ...)
{
    if (level < _PX4_LOG_LEVEL_DEBUG || level > _PX4_LOG_LEVEL_PANIC) {
        return;
    }
    va_list arguments;
    va_start(arguments, format);
    (void)dima::logging::write_v(
        dima::logging::Source::System,
        static_cast<dima::logging::Level>(level), nullptr, true, format,
        arguments);
    va_end(arguments);
}
