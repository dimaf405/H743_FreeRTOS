#include "events.hpp"

#include <algorithm>
#include <cstring>
#include <limits>

#include "platform/api/Platform.hpp"
#include "platform/api/Time.hpp"

namespace dima::events {
namespace {

struct EventState {
    DimaEvent records[kEventCapacity]{};
    std::size_t head{0U};
    std::size_t count{0U};
    std::uint32_t published{0U};
    std::uint32_t consumed{0U};
    std::uint32_t overflow_count{0U};
    std::uint32_t dropped_count{0U};
    CriticalFaultLatch critical{};
};

EventState g_state{};

constexpr bool is_critical(const DimaEvent &event) noexcept
{
    return event.severity == static_cast<std::uint8_t>(Severity::Critical);
}

std::size_t physical_index(std::size_t logical_index) noexcept
{
    return (g_state.head + logical_index) % kEventCapacity;
}

void increment_saturated(std::uint32_t &value) noexcept
{
    if (value != std::numeric_limits<std::uint32_t>::max()) {
        ++value;
    }
}

void remove_logical_index(std::size_t logical_index) noexcept
{
    // 容量固定为 128，满环形中删除一个非关键记录的开销具有严格上限。
    for (std::size_t index = logical_index; index + 1U < g_state.count; ++index) {
        g_state.records[physical_index(index)] =
            g_state.records[physical_index(index + 1U)];
    }

    --g_state.count;
}

bool make_room_for(const DimaEvent &incoming) noexcept
{
    if (g_state.count < kEventCapacity) {
        return true;
    }

    increment_saturated(g_state.overflow_count);

    // 优先淘汰时间上最旧的非关键记录，关键故障仍由独立锁存保存。
    for (std::size_t index = 0U; index < g_state.count; ++index) {
        if (!is_critical(g_state.records[physical_index(index)])) {
            remove_logical_index(index);
            return true;
        }
    }

    if (!is_critical(incoming)) {
        increment_saturated(g_state.dropped_count);
        return false;
    }

    // 极端情况下环形全部为关键记录：允许新关键事件替换最旧记录，
    // 首次故障原因和最新故障仍保存在关键故障锁存中。
    g_state.head = (g_state.head + 1U) % kEventCapacity;
    --g_state.count;
    return true;
}

void update_critical_latch(const DimaEvent &event) noexcept
{
    if (!is_critical(event)) {
        return;
    }

    if (!g_state.critical.active) {
        g_state.critical.active = true;
        g_state.critical.first_event = event;
    }

    g_state.critical.last_event = event;
    increment_saturated(g_state.critical.occurrence_count);
}

} // namespace

bool report(std::uint32_t id,
            Severity severity,
            const std::uint32_t *arguments,
            std::size_t argument_count) noexcept
{
    DimaEvent event{};
    event.timestamp = hrt_absolute_time();
    event.id = id;
    event.severity = static_cast<std::uint8_t>(severity);
    event.argument_count = static_cast<std::uint8_t>(
        std::min(argument_count, kMaxArguments));

    if (arguments != nullptr) {
        std::copy_n(arguments, event.argument_count, event.arguments);
    }

    dima::platform::CriticalGuard lock;
    update_critical_latch(event);

    if (!make_room_for(event)) {
        return false;
    }

    const std::size_t tail = physical_index(g_state.count);
    g_state.records[tail] = event;
    ++g_state.count;
    increment_saturated(g_state.published);
    return true;
}

bool pop(DimaEvent &event) noexcept
{
    dima::platform::CriticalGuard lock;
    if (g_state.count == 0U) {
        return false;
    }

    event = g_state.records[g_state.head];
    g_state.head = (g_state.head + 1U) % kEventCapacity;
    --g_state.count;
    increment_saturated(g_state.consumed);
    return true;
}

EventStats stats() noexcept
{
    dima::platform::CriticalGuard lock;
    return EventStats{
        g_state.published,
        g_state.consumed,
        g_state.overflow_count,
        g_state.dropped_count,
        g_state.count,
    };
}

CriticalFaultLatch critical_fault_latch() noexcept
{
    dima::platform::CriticalGuard lock;
    return g_state.critical;
}

bool clear_critical_fault(std::uint32_t id) noexcept
{
    dima::platform::CriticalGuard lock;
    if (!g_state.critical.active) {
        return true;
    }

    if ((id != 0U) && (g_state.critical.last_event.id != id)) {
        return false;
    }

    g_state.critical = CriticalFaultLatch{};
    return true;
}

void reset() noexcept
{
    dima::platform::CriticalGuard lock;
    g_state = EventState{};
}

} // namespace dima::events
