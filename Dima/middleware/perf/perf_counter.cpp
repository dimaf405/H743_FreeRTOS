#include "Dima/middleware/perf/perf_counter.h"

#include <cstdint>
#include <limits>

extern "C" {
#include "FreeRTOS.h"
#include "task.h"
}

using hrt_abstime = std::uint64_t;
hrt_abstime hrt_absolute_time() noexcept;

namespace {

constexpr std::size_t kMaxCounters = 64U;
constexpr std::uint64_t kUnsetMinimum =
    std::numeric_limits<std::uint64_t>::max();

class CriticalSection final {
public:
    CriticalSection() noexcept
        : from_isr_(xPortIsInsideInterrupt() != pdFALSE)
    {
        if (from_isr_) {
            saved_mask_ = taskENTER_CRITICAL_FROM_ISR();
        } else {
            taskENTER_CRITICAL();
        }
    }

    ~CriticalSection()
    {
        if (from_isr_) {
            taskEXIT_CRITICAL_FROM_ISR(saved_mask_);
        } else {
            taskEXIT_CRITICAL();
        }
    }

    CriticalSection(const CriticalSection &) = delete;
    CriticalSection &operator=(const CriticalSection &) = delete;

private:
    bool from_isr_{false};
    UBaseType_t saved_mask_{0U};
};

void update_measurement(struct perf_ctr_header &counter,
                        std::uint64_t value) noexcept;
bool valid_handle(perf_counter_t handle) noexcept;

} // namespace

struct perf_ctr_header {
    enum perf_counter_type type{PC_COUNT};
    const char *name{nullptr};
    std::uint64_t event_count{0U};
    std::uint64_t total{0U};
    std::uint64_t minimum{kUnsetMinimum};
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
    if (value > (std::numeric_limits<std::uint64_t>::max() - target)) {
        target = std::numeric_limits<std::uint64_t>::max();
    } else {
        target += value;
    }
}

void update_measurement(perf_ctr_header &counter, std::uint64_t value) noexcept
{
    counter.last = value;
    counter.minimum = (value < counter.minimum) ? value : counter.minimum;
    counter.maximum = (value > counter.maximum) ? value : counter.maximum;
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
    const bool in_pool = (address >= begin) && (address < end);
    const bool aligned = in_pool && (((address - begin) % sizeof(perf_ctr_header)) == 0U);
    return aligned && handle->allocated;
}

void reset_values(perf_ctr_header &counter) noexcept
{
    counter.event_count = 0U;
    counter.total = 0U;
    counter.minimum = kUnsetMinimum;
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
    snapshot->minimum =
        (handle->minimum == kUnsetMinimum) ? 0U : handle->minimum;
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

