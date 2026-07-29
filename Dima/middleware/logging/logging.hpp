#pragma once

#include <cstddef>
#include <cstdint>

namespace dima::logging {

constexpr std::size_t kLogRingCapacity = 8U * 1024U;

enum class Level : std::uint8_t {
    Debug = 0U,
    Info,
    Warning,
    Error,
};

enum class WriteResult : std::uint8_t {
    Ok = 0U,
    Truncated,
    RejectedRealtime,
    InvalidArgument,
};

struct LogStats {
    std::uint32_t records_written;
    std::uint32_t records_truncated;
    std::uint32_t formatting_rejections;
    std::uint32_t overwritten_bytes;
    std::uint32_t service_dropped_bytes;
    std::size_t pending_bytes;
};

using ServiceWrite = std::size_t (*)(void *context,
                                     const std::uint8_t *data,
                                     std::size_t length);

struct ServiceWriter {
    void *context;
    ServiceWrite write;
};

// 格式化入口仅允许普通非实时任务调用；ISR 和实时任务会在格式化前拒绝。
WriteResult writef(Level level, const char *format, ...) noexcept
    __attribute__((format(printf, 2, 3)));

// 非格式化入口可用于已经准备好的字节串，包括 ISR/实时路径。
// 数据写满环形时覆盖最旧字节，调用方不会等待 service I/O。
WriteResult write_literal(const char *text, std::size_t length) noexcept;

// service/LP 任务调用。数据先从环形取到固定栈缓冲，再调用外部 writer，
// 因此外部 USB/串口阻塞不会持有生产者临界区。
std::size_t service_flush(const ServiceWriter &writer,
                          std::size_t max_bytes = kLogRingCapacity) noexcept;

LogStats stats() noexcept;
void reset() noexcept;

} // namespace dima::logging

#ifndef PX4_DEBUG
#define PX4_DEBUG(format, ...)                                                   \
    ((void)::dima::logging::writef(::dima::logging::Level::Debug, format,       \
                                    ##__VA_ARGS__))
#endif

#ifndef PX4_INFO
#define PX4_INFO(format, ...)                                                    \
    ((void)::dima::logging::writef(::dima::logging::Level::Info, format,        \
                                    ##__VA_ARGS__))
#endif

#ifndef PX4_WARN
#define PX4_WARN(format, ...)                                                    \
    ((void)::dima::logging::writef(::dima::logging::Level::Warning, format,     \
                                    ##__VA_ARGS__))
#endif

#ifndef PX4_ERR
#define PX4_ERR(format, ...)                                                     \
    ((void)::dima::logging::writef(::dima::logging::Level::Error, format,       \
                                    ##__VA_ARGS__))
#endif
