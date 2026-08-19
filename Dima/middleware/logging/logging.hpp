#pragma once

#include "debug_config.hpp"

#include <cstddef>
#include <cstdint>

// PX4 v1.17.0 logging compatibility surface.
// Upstream: platforms/common/include/px4_platform_common/log.h
#define _PX4_LOG_LEVEL_DEBUG 0
#define _PX4_LOG_LEVEL_INFO 1
#define _PX4_LOG_LEVEL_WARN 2
#define _PX4_LOG_LEVEL_ERROR 3
#define _PX4_LOG_LEVEL_PANIC 4

#ifndef MODULE_NAME
#define MODULE_NAME "dima"
#endif

#if defined(TRACE_BUILD) || defined(DEBUG_BUILD)
#error "Dima PX4 TRACE_BUILD/DEBUG_BUILD log formatting is not ported yet"
#endif

extern "C" {
extern const char *__px4_log_level_str[_PX4_LOG_LEVEL_PANIC + 1];
void px4_log_modulename(int level, const char *module_name,
                        const char *format, ...)
    __attribute__((format(printf, 3, 4)));
void px4_log_raw(int level, const char *format, ...)
    __attribute__((format(printf, 2, 3)));
void px4_log_initialize(void);
}

namespace dima::logging {

static_assert(static_cast<std::uint8_t>(Level::Debug) ==
              _PX4_LOG_LEVEL_DEBUG);
static_assert(static_cast<std::uint8_t>(Level::Info) ==
              _PX4_LOG_LEVEL_INFO);
static_assert(static_cast<std::uint8_t>(Level::Warning) ==
              _PX4_LOG_LEVEL_WARN);
static_assert(static_cast<std::uint8_t>(Level::Error) ==
              _PX4_LOG_LEVEL_ERROR);
static_assert(static_cast<std::uint8_t>(Level::Panic) ==
              _PX4_LOG_LEVEL_PANIC);

enum class WriteResult : std::uint8_t {
    Ok = 0U,
    Truncated,
    Filtered,
    RejectedRealtime,
    InvalidArgument,
};

struct LogStats {
    std::uint32_t records_written;
    std::uint32_t records_truncated;
    std::uint32_t formatting_rejections;
    std::uint32_t sink_dropped_records;
};

/**
 * Structured record sink (dependency inversion hook).
 *
 * Called once per formatted log record with the message body (without a
 * level/module prefix or trailing newline). PX4_INFO_RAW uses the supplied
 * level and the same structured path. Invoked in the writing task's context;
 * implementations must be non-blocking and report publication success.
 */
using StructuredSink = bool (*)(void *context, Level level,
                                const char *text, std::size_t length);
void set_structured_sink(void *context, StructuredSink sink) noexcept;

// Dima internal compatibility entry. Product modules should prefer PX4_* macros.
WriteResult writef(Level level, const char *format, ...) noexcept
    __attribute__((format(printf, 2, 3)));
WriteResult write_module(Source source, Level level, const char *module_name,
                         const char *format, ...) noexcept
    __attribute__((format(printf, 4, 5)));
LogStats stats() noexcept;
void reset() noexcept;

} // namespace dima::logging

#define __dima_px4_log_module(level, format, ...) \
    px4_log_modulename(level, MODULE_NAME, format, ##__VA_ARGS__)
#define __dima_px4_log_omit(level, format, ...) \
    do { if (false) { px4_log_modulename(level, MODULE_NAME, format, ##__VA_ARGS__); } } while (0)
#define DIMA_LOG_SOURCE(source, level, format, ...) \
    do { \
        (void)dima::logging::write_module(source, level, MODULE_NAME, format, \
                                           ##__VA_ARGS__); \
    } while (0)

#define PX4_INFO(format, ...) \
    __dima_px4_log_module(_PX4_LOG_LEVEL_INFO, format, ##__VA_ARGS__)
#define PX4_INFO_RAW(format, ...) \
    px4_log_raw(_PX4_LOG_LEVEL_INFO, format, ##__VA_ARGS__)
#define PX4_PANIC(format, ...) \
    __dima_px4_log_module(_PX4_LOG_LEVEL_PANIC, format, ##__VA_ARGS__)
#define PX4_ERR(format, ...) \
    __dima_px4_log_module(_PX4_LOG_LEVEL_ERROR, format, ##__VA_ARGS__)

#define PX4_WARN(format, ...) \
    __dima_px4_log_module(_PX4_LOG_LEVEL_WARN, format, ##__VA_ARGS__)
#define PX4_DEBUG(format, ...) \
    __dima_px4_log_module(_PX4_LOG_LEVEL_DEBUG, format, ##__VA_ARGS__)

#define PX4_LOG_NAMED(name, format, ...) \
    PX4_INFO("%s " format, name, ##__VA_ARGS__)
#define PX4_LOG_NAMED_COND(name, condition, format, ...) \
    do { if (condition) { PX4_LOG_NAMED(name, format, ##__VA_ARGS__); } } while (0)
