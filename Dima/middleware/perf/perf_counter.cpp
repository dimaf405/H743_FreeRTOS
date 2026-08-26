#include "perf_counter.h"

#include <cstdint>
#include <limits>

#include "api/Execution.hpp"
#include "api/Time.hpp"

namespace {

constexpr std::size_t kMaxCounters = 64U;

/* perf 使用固定 64 槽池，不动态分配；PC_ELAPSED/PC_INTERVAL 的测量单位均为
 * hrt_absolute_time 的微秒，PC_COUNT 的 last 等于累计次数。 */
using CriticalSection = dima::platform::CriticalGuard;

void update_measurement(struct perf_ctr_header &counter,
                        std::uint64_t value) noexcept;
bool valid_handle(perf_counter_t handle) noexcept;

} // namespace

struct perf_ctr_header {
    enum perf_counter_type type{PC_COUNT};
    const char *name{nullptr};
    std::uint64_t event_count{0U};
    std::uint64_t total{0U};
    std::uint64_t minimum{0U};
    std::uint64_t maximum{0U};
    std::uint64_t last{0U};
    std::uint64_t begin_time{0U};
    std::uint64_t previous_event_time{0U};
    bool begin_active{false};
    bool allocated{false};
};

namespace {

perf_ctr_header g_counters[kMaxCounters]{};

void increment_saturated(std::uint64_t &value) noexcept
{
    if (value != std::numeric_limits<std::uint64_t>::max()) {
        ++value;
    }
}

void add_saturated(std::uint64_t &target, std::uint64_t value) noexcept
{
    /* total/event_count 饱和到 UINT64_MAX，不允许溢出后破坏均值或健康监控。 */
    if (value > (std::numeric_limits<std::uint64_t>::max() - target)) {
        target = std::numeric_limits<std::uint64_t>::max();
    } else {
        target += value;
    }
}

void update_measurement(perf_ctr_header &counter, std::uint64_t value) noexcept
{
    /* 第一笔样本同时初始化 min/max；后续维护 last/min/max/total/count。平均值由
     * 读取方使用 total/event_count 计算，本层避免浮点和除法。 */
    counter.last = value;
    if (counter.event_count == 0U) {
        counter.minimum = value;
        counter.maximum = value;
    } else {
        counter.minimum = (value < counter.minimum) ? value : counter.minimum;
        counter.maximum = (value > counter.maximum) ? value : counter.maximum;
    }
    add_saturated(counter.total, value);
    increment_saturated(counter.event_count);
}

bool valid_handle(perf_counter_t handle) noexcept
{
    if (handle == nullptr) {
        return false;
    }

    const auto address = reinterpret_cast<std::uintptr_t>(handle);
    const auto begin = reinterpret_cast<std::uintptr_t>(&g_counters[0]);
    const auto end = reinterpret_cast<std::uintptr_t>(&g_counters[kMaxCounters]);
    /* handle 必须恰好指向池中某槽首地址且仍 allocated，拒绝野指针和槽内偏移。 */
    const bool in_pool = (address >= begin) && (address < end);
    const bool aligned = in_pool && (((address - begin) % sizeof(perf_ctr_header)) == 0U);
    return aligned && handle->allocated;
}

void reset_values(perf_ctr_header &counter) noexcept
{
    counter.event_count = 0U;
    counter.total = 0U;
    counter.minimum = 0U;
    counter.maximum = 0U;
    counter.last = 0U;
    counter.begin_time = 0U;
    counter.previous_event_time = 0U;
    counter.begin_active = false;
}

} // namespace

extern "C" perf_counter_t perf_alloc(enum perf_counter_type type,
                                      const char *name)
{
    if ((type != PC_COUNT) && (type != PC_ELAPSED) &&
        (type != PC_INTERVAL)) {
        return nullptr;
    }

    CriticalSection lock;
    for (auto &counter : g_counters) {
        if (!counter.allocated) {
            counter = perf_ctr_header{};
            counter.type = type;
            counter.name = name;
            counter.allocated = true;
            return &counter;
        }
    }

    return nullptr;
}

extern "C" void perf_free(perf_counter_t handle)
{
    CriticalSection lock;
    if (valid_handle(handle)) {
        *handle = perf_ctr_header{};
    }
}

extern "C" void perf_begin(perf_counter_t handle)
{
    const std::uint64_t now = hrt_absolute_time();
    CriticalSection lock;
    if (valid_handle(handle) && (handle->type == PC_ELAPSED)) {
        handle->begin_time = now;
        handle->begin_active = true;
    }
}

extern "C" void perf_end(perf_counter_t handle)
{
    const std::uint64_t now = hrt_absolute_time();
    CriticalSection lock;
    if (!valid_handle(handle) || (handle->type != PC_ELAPSED) ||
        !handle->begin_active) {
        return;
    }

    /* 单调 64-bit 微秒自然相减；只有成对 begin_active 才产生一笔样本。 */
    const std::uint64_t elapsed = now - handle->begin_time;
    handle->begin_active = false;
    update_measurement(*handle, elapsed);
}

extern "C" void perf_count(perf_counter_t handle)
{
    const std::uint64_t now = hrt_absolute_time();
    CriticalSection lock;
    if (!valid_handle(handle)) {
        return;
    }

    if (handle->type == PC_COUNT) {
        increment_saturated(handle->event_count);
        handle->last = handle->event_count;
        return;
    }

    if (handle->type == PC_INTERVAL) {
        /* 第一次事件只建立 previous 时间，不计 0 间隔；第二次起记录相邻事件差。 */
        if (handle->previous_event_time != 0U) {
            update_measurement(*handle, now - handle->previous_event_time);
        }
        handle->previous_event_time = now;
    }
}

extern "C" void perf_set_elapsed(perf_counter_t handle, int64_t elapsed)
{
    if (elapsed < 0) {
        return;
    }

    CriticalSection lock;
    if (valid_handle(handle) && (handle->type == PC_ELAPSED)) {
        handle->begin_active = false;
        update_measurement(*handle, static_cast<std::uint64_t>(elapsed));
    }
}

extern "C" bool perf_get_snapshot(perf_counter_t handle,
                                   struct perf_counter_snapshot *snapshot)
{
    if (snapshot == nullptr) {
        return false;
    }

    CriticalSection lock;
    if (!valid_handle(handle)) {
        return false;
    }

    snapshot->type = handle->type;
    snapshot->name = handle->name;
    snapshot->event_count = handle->event_count;
    snapshot->total = handle->total;
    snapshot->minimum = handle->minimum;
    snapshot->maximum = handle->maximum;
    snapshot->last = handle->last;
    snapshot->active = handle->begin_active;
    return true;
}

extern "C" void perf_reset(perf_counter_t handle)
{
    CriticalSection lock;
    if (valid_handle(handle)) {
        reset_values(*handle);
    }
}

extern "C" size_t perf_allocated_count(void)
{
    CriticalSection lock;
    std::size_t count = 0U;
    for (const auto &counter : g_counters) {
        count += counter.allocated ? 1U : 0U;
    }
    return count;
}

