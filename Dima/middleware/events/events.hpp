#pragma once

#include <cstddef>
#include <cstdint>

namespace dima::events {

constexpr std::size_t kEventCapacity = 128U;
constexpr std::size_t kMaxArguments = 4U;

enum class Severity : std::uint8_t {
    Debug = 0U,
    Info,
    Warning,
    Error,
    Critical,
};

struct DimaEvent {
    std::uint64_t timestamp;
    std::uint32_t id;
    std::uint8_t severity;
    std::uint8_t argument_count;
    std::uint32_t arguments[kMaxArguments];
};

struct EventStats {
    std::uint32_t published;
    std::uint32_t consumed;
    std::uint32_t overflow_count;
    std::uint32_t dropped_count;
    std::size_t pending;
};

struct CriticalFaultLatch {
    bool active;
    std::uint32_t occurrence_count;
    DimaEvent first_event;
    DimaEvent last_event;
};

// 发布结构化事件。参数超过 4 个时只保留前 4 个。
// 函数不分配内存、不格式化字符串，可在任务或 ISR 上下文调用。
bool report(std::uint32_t id,
            Severity severity,
            const std::uint32_t *arguments = nullptr,
            std::size_t argument_count = 0U) noexcept;

// 读取并移除最旧事件；无事件时返回 false。
bool pop(DimaEvent &event) noexcept;

EventStats stats() noexcept;
CriticalFaultLatch critical_fault_latch() noexcept;

// 清除关键故障锁存。若 id 非 0，仅在锁存事件 ID 匹配时清除。
bool clear_critical_fault(std::uint32_t id = 0U) noexcept;

// 仅供产品启动/停机阶段调用，清空环形和锁存状态。
void reset() noexcept;

} // namespace dima::events

