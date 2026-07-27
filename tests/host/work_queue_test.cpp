#include "test_framework.hpp"

#include "App/runtime/scheduling/scheduled_work_item.hpp"
#include "App/runtime/scheduling/work_queue.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <type_traits>
#include <utility>

namespace {

using app::runtime::scheduling::IClock;
using app::runtime::scheduling::IWorkQueueBackend;
using app::runtime::scheduling::QueueClass;
using app::runtime::scheduling::ScheduledWorkItem;
using app::runtime::scheduling::WorkQueue;
using app::runtime::scheduling::deadline_reached;

template <typename T, typename = void>
struct has_public_Run : std::false_type {};

template <typename T>
struct has_public_Run<T, std::void_t<decltype(std::declval<T &>().Run())>> : std::true_type {};

class ScheduledWorkItemRunProbe final : public ScheduledWorkItem {
public:
    using ScheduledWorkItem::ScheduledWorkItem;

    static auto base_Run_is_accessible_from_derived(int)
        -> decltype(std::declval<ScheduledWorkItemRunProbe &>().ScheduledWorkItem::Run(),
                    std::true_type{});
    static auto base_Run_is_accessible_from_derived(...) -> std::false_type;

private:
    void Run() override {}
};

static_assert(!has_public_Run<ScheduledWorkItem>::value,
              "ScheduledWorkItem::Run must not be callable by external code");
static_assert(decltype(ScheduledWorkItemRunProbe::base_Run_is_accessible_from_derived(0))::value,
              "ScheduledWorkItem::Run must be protected so derived work items can invoke it");

class FakeClock final : public IClock {
public:
    std::uint32_t now{0};

    std::uint32_t now_ms() const override { return now; }
    void advance(std::uint32_t delta_ms) { now += delta_ms; }
};

class FakeBackend final : public IWorkQueueBackend {
public:
    struct Entry {
        bool active{false};
        QueueClass queue{QueueClass::LowPriority};
        void *context{nullptr};
        ClaimCallback claim{nullptr};
        RunCallback run{nullptr};
        std::uint32_t deadline_ms{0};
    };

    std::array<Entry, 64> entries{};
    unsigned schedule_calls{0};
    unsigned clear_calls{0};
    bool fail_next_schedule{false};

    bool schedule(QueueClass queue, void *context, ClaimCallback claim,
                  RunCallback run,
                  std::uint32_t deadline_ms) override
    {
        ++schedule_calls;
        if (fail_next_schedule) {
            fail_next_schedule = false;
            return false;
        }
        for (auto &entry : entries) {
            if (!entry.active) {
                entry = {true, queue, context, claim, run, deadline_ms};
                return true;
            }
        }
        return false;
    }

    void clear(QueueClass queue, void *context) override
    {
        ++clear_calls;
        for (auto &entry : entries) {
            if (entry.active && entry.queue == queue && entry.context == context) {
                entry.active = false;
            }
        }
    }

    unsigned active_count(QueueClass queue) const
    {
        unsigned count = 0;
        for (const auto &entry : entries) {
            count += entry.active && entry.queue == queue;
        }
        return count;
    }

    std::uint32_t first_deadline(QueueClass queue) const
    {
        for (const auto &entry : entries) {
            if (entry.active && entry.queue == queue) {
                return entry.deadline_ms;
            }
        }
        return UINT32_MAX;
    }

    void claim_first(QueueClass queue)
    {
        for (auto &entry : entries) {
            if (entry.active && entry.queue == queue) {
                const Entry ready = entry;
                entry.active = false;
                if (ready.claim(ready.context)) {
                    claimed_entry_ = ready;
                    claimed_active_ = true;
                    return;
                }
            }
        }
        throw std::runtime_error("no queued work item to claim");
    }

    void run_claimed()
    {
        if (!claimed_active_) {
            throw std::runtime_error("no claimed work item to run");
        }
        const Entry claimed = claimed_entry_;
        claimed_active_ = false;
        claimed.run(claimed.context);
    }

private:
    Entry claimed_entry_{};
    bool claimed_active_{false};
};

class CountingItem final : public ScheduledWorkItem {
public:
    explicit CountingItem(WorkQueue &queue) : ScheduledWorkItem(queue) {}

    unsigned runs{0};

private:
    void Run() override { ++runs; }
};

class ClearDuringRunItem final : public ScheduledWorkItem {
public:
    explicit ClearDuringRunItem(WorkQueue &queue) : ScheduledWorkItem(queue) {}

    unsigned runs{0};

private:
    void Run() override
    {
        ++runs;
        CHECK(ScheduleNow());
        ScheduleClear();
    }
};

class AdvancingItem final : public ScheduledWorkItem {
public:
    AdvancingItem(WorkQueue &queue, FakeClock &clock, std::uint32_t run_time_ms)
        : ScheduledWorkItem(queue), clock_(clock), run_time_ms_(run_time_ms)
    {
    }

    unsigned runs{0U};

private:
    void Run() override
    {
        ++runs;
        clock_.advance(run_time_ms_);
    }

    FakeClock &clock_;
    std::uint32_t run_time_ms_;
};

class ClearThenScheduleDuringRunItem final : public ScheduledWorkItem {
public:
    explicit ClearThenScheduleDuringRunItem(WorkQueue &queue)
        : ScheduledWorkItem(queue)
    {
    }

    unsigned runs{0U};

private:
    void Run() override
    {
        ++runs;
        if (runs == 1U) {
            ScheduleClear();
            CHECK(ScheduleNow());
        }
    }
};

} // namespace

HOST_TEST(work_queue_schedule_now_coalesces_to_one_pending_callback)
{
    FakeClock clock;
    FakeBackend backend;
    WorkQueue queue{QueueClass::HighPriority, clock, backend};
    CountingItem item{queue};

    CHECK(item.ScheduleNow());
    CHECK(item.ScheduleNow());
    CHECK_EQ(backend.schedule_calls, 1U);
    CHECK_EQ(backend.active_count(QueueClass::HighPriority), 1U);

    backend.claim_first(QueueClass::HighPriority);
    backend.run_claimed();
    CHECK_EQ(item.runs, 1U);
}

HOST_TEST(work_queue_periodic_deadline_is_anchored_not_accumulated_from_late_execution)
{
    FakeClock clock;
    FakeBackend backend;
    WorkQueue queue{QueueClass::LowPriority, clock, backend};
    CountingItem item{queue};

    CHECK(item.ScheduleOnInterval(1000U));
    CHECK_EQ(backend.first_deadline(QueueClass::LowPriority), 1000U);

    clock.advance(1025U); // The worker is 25 ms late, but the next phase is still 2000 ms.
    backend.claim_first(QueueClass::LowPriority);
    backend.run_claimed();
    CHECK_EQ(item.runs, 1U);
    CHECK_EQ(backend.first_deadline(QueueClass::LowPriority), 2000U);
}

HOST_TEST(work_queue_periodic_deadline_skips_missed_intervals_without_losing_phase)
{
    FakeClock clock;
    FakeBackend backend;
    WorkQueue queue{QueueClass::LowPriority, clock, backend};
    CountingItem item{queue};

    CHECK(item.ScheduleOnInterval(1000U));
    CHECK_EQ(backend.first_deadline(QueueClass::LowPriority), 1000U);

    clock.advance(3500U);
    backend.claim_first(QueueClass::LowPriority);
    backend.run_claimed();

    CHECK_EQ(item.runs, 1U);
    CHECK_EQ(backend.first_deadline(QueueClass::LowPriority), 4000U);
}

HOST_TEST(work_queue_periodic_deadline_strictly_advances_when_completion_is_on_phase)
{
    FakeClock clock;
    FakeBackend backend;
    WorkQueue queue{QueueClass::LowPriority, clock, backend};
    CountingItem item{queue};

    CHECK(item.ScheduleOnInterval(1000U));
    clock.advance(4000U);
    backend.claim_first(QueueClass::LowPriority);
    backend.run_claimed();

    CHECK_EQ(backend.first_deadline(QueueClass::LowPriority), 5000U);
}

HOST_TEST(work_queue_periodic_phase_uses_completion_time_after_a_long_run)
{
    FakeClock clock;
    FakeBackend backend;
    WorkQueue queue{QueueClass::LowPriority, clock, backend};
    AdvancingItem item{queue, clock, 2500U};

    CHECK(item.ScheduleOnInterval(1000U));
    clock.advance(1000U);
    backend.claim_first(QueueClass::LowPriority);
    backend.run_claimed();

    CHECK_EQ(item.runs, 1U);
    CHECK_EQ(backend.first_deadline(QueueClass::LowPriority), 4000U);
}

HOST_TEST(work_queue_periodic_phase_remains_correct_across_uint32_wrap)
{
    FakeClock clock;
    FakeBackend backend;
    WorkQueue queue{QueueClass::LowPriority, clock, backend};
    CountingItem item{queue};
    clock.now = 0xFFFFFFF0U;

    CHECK(item.ScheduleOnInterval(32U));
    clock.advance(80U);
    backend.claim_first(QueueClass::LowPriority);
    backend.run_claimed();

    CHECK_EQ(backend.first_deadline(QueueClass::LowPriority), 0x00000050U);
}

HOST_TEST(work_queue_schedule_clear_cancels_pending_callback)
{
    FakeClock clock;
    FakeBackend backend;
    WorkQueue queue{QueueClass::HighPriority, clock, backend};
    CountingItem item{queue};

    CHECK(item.ScheduleOnInterval(1000U));
    item.ScheduleClear();
    CHECK_EQ(backend.clear_calls, 1U);
    CHECK_EQ(backend.active_count(QueueClass::HighPriority), 0U);
}

HOST_TEST(work_queue_high_and_low_priority_capacity_are_isolated_at_sixteen_items_each)
{
    struct GuardedQueues {
        std::uint32_t before{0x13579BDFU};
        WorkQueue high;
        WorkQueue low;
        std::uint32_t after{0x2468ACE0U};
    };
    FakeClock clock;
    FakeBackend backend;
    GuardedQueues queues{0x13579BDFU,
                         WorkQueue{QueueClass::HighPriority, clock, backend},
                         WorkQueue{QueueClass::LowPriority, clock, backend},
                         0x2468ACE0U};
    std::array<std::optional<CountingItem>, 17> high_items{};
    std::array<std::optional<CountingItem>, 16> low_items{};
    for (auto &item : high_items) {
        item.emplace(queues.high);
    }
    for (auto &item : low_items) {
        item.emplace(queues.low);
    }

    CHECK_EQ(WorkQueue::kMaxItems, 16U);
    for (unsigned index = 0; index < WorkQueue::kMaxItems; ++index) {
        CHECK(high_items[index]->ScheduleNow());
        CHECK(low_items[index]->ScheduleNow());
    }
    CHECK(!high_items[16]->ScheduleNow());
    CHECK_EQ(backend.active_count(QueueClass::HighPriority), 16U);
    CHECK_EQ(backend.active_count(QueueClass::LowPriority), 16U);
    CHECK_EQ(queues.before, 0x13579BDFU);
    CHECK_EQ(queues.after, 0x2468ACE0U);
}

HOST_TEST(work_queue_deadline_comparison_is_safe_across_uint32_wraparound)
{
    CHECK(deadline_reached(0x00000010U, 0xFFFFFFF0U));
    CHECK(!deadline_reached(0xFFFFFFF0U, 0x00000010U));
    CHECK(deadline_reached(1234U, 1234U));
}

HOST_TEST(work_queue_schedule_now_after_claim_is_deferred_until_the_claimed_run_finishes)
{
    FakeClock clock;
    FakeBackend backend;
    WorkQueue queue{QueueClass::HighPriority, clock, backend};
    CountingItem item{queue};

    CHECK(item.ScheduleNow());
    backend.claim_first(QueueClass::HighPriority);
    CHECK_EQ(backend.active_count(QueueClass::HighPriority), 0U);

    CHECK(item.ScheduleNow());
    CHECK_EQ(backend.active_count(QueueClass::HighPriority), 0U);
    backend.run_claimed();

    CHECK_EQ(item.runs, 1U);
    CHECK_EQ(backend.active_count(QueueClass::HighPriority), 1U);
    backend.claim_first(QueueClass::HighPriority);
    backend.run_claimed();
    CHECK_EQ(item.runs, 2U);
    CHECK_EQ(backend.active_count(QueueClass::HighPriority), 0U);
}

HOST_TEST(work_queue_clear_after_claim_skips_run_and_leaves_no_entry)
{
    FakeClock clock;
    FakeBackend backend;
    WorkQueue queue{QueueClass::HighPriority, clock, backend};
    CountingItem item{queue};

    CHECK(item.ScheduleNow());
    backend.claim_first(QueueClass::HighPriority);
    item.ScheduleClear();
    backend.run_claimed();

    CHECK_EQ(item.runs, 0U);
    CHECK_EQ(backend.active_count(QueueClass::HighPriority), 0U);
}

HOST_TEST(work_queue_rejects_intervals_outside_the_wrap_safe_half_range)
{
    FakeClock clock;
    FakeBackend backend;
    WorkQueue queue{QueueClass::LowPriority, clock, backend};
    CountingItem item{queue};

    CHECK(!item.ScheduleOnInterval(0U));
    CHECK(!item.ScheduleOnInterval(0x80000000U));
    CHECK(!item.ScheduleOnInterval(UINT32_MAX));
    CHECK_EQ(backend.schedule_calls, 0U);
    CHECK_EQ(backend.active_count(QueueClass::LowPriority), 0U);
}

HOST_TEST(work_queue_backend_schedule_failure_rolls_back_state_and_capacity)
{
    FakeClock clock;
    FakeBackend backend;
    WorkQueue queue{QueueClass::HighPriority, clock, backend};
    CountingItem item{queue};
    backend.fail_next_schedule = true;

    CHECK(!item.ScheduleNow());
    CHECK_EQ(backend.active_count(QueueClass::HighPriority), 0U);
    CHECK(item.ScheduleNow());
    CHECK_EQ(backend.schedule_calls, 2U);
    CHECK_EQ(backend.active_count(QueueClass::HighPriority), 1U);

    backend.claim_first(QueueClass::HighPriority);
    backend.run_claimed();
    CHECK_EQ(item.runs, 1U);
}

HOST_TEST(work_queue_capacity_counts_claimed_items_until_run_finishes)
{
    FakeClock clock;
    FakeBackend backend;
    WorkQueue queue{QueueClass::HighPriority, clock, backend};
    std::array<std::optional<CountingItem>, 17> items{};
    for (auto &item : items) {
        item.emplace(queue);
    }
    for (unsigned index = 0U; index < WorkQueue::kMaxItems; ++index) {
        CHECK(items[index]->ScheduleNow());
    }

    backend.claim_first(QueueClass::HighPriority);
    CHECK(!items[16]->ScheduleNow());
    backend.run_claimed();
    CHECK(items[16]->ScheduleNow());
}

HOST_TEST(work_queue_clear_during_run_discards_deferred_rescheduling)
{
    FakeClock clock;
    FakeBackend backend;
    WorkQueue queue{QueueClass::HighPriority, clock, backend};
    ClearDuringRunItem item{queue};

    CHECK(item.ScheduleNow());
    backend.claim_first(QueueClass::HighPriority);
    backend.run_claimed();

    CHECK_EQ(item.runs, 1U);
    CHECK_EQ(backend.active_count(QueueClass::HighPriority), 0U);
}

HOST_TEST(work_queue_schedule_after_clear_during_running_creates_one_future_run)
{
    FakeClock clock;
    FakeBackend backend;
    WorkQueue queue{QueueClass::HighPriority, clock, backend};
    ClearThenScheduleDuringRunItem item{queue};

    CHECK(item.ScheduleNow());
    backend.claim_first(QueueClass::HighPriority);
    backend.run_claimed();
    CHECK_EQ(item.runs, 1U);
    CHECK_EQ(backend.active_count(QueueClass::HighPriority), 1U);

    backend.claim_first(QueueClass::HighPriority);
    backend.run_claimed();
    CHECK_EQ(item.runs, 2U);
    CHECK_EQ(backend.active_count(QueueClass::HighPriority), 0U);
}

HOST_TEST(work_queue_clear_then_schedule_while_claimed_skips_old_run_and_queues_new_run)
{
    FakeClock clock;
    FakeBackend backend;
    WorkQueue queue{QueueClass::HighPriority, clock, backend};
    CountingItem item{queue};

    CHECK(item.ScheduleNow());
    backend.claim_first(QueueClass::HighPriority);
    item.ScheduleClear();
    CHECK(item.ScheduleNow());
    backend.run_claimed();

    CHECK_EQ(item.runs, 0U);
    CHECK_EQ(backend.active_count(QueueClass::HighPriority), 1U);
    backend.claim_first(QueueClass::HighPriority);
    backend.run_claimed();
    CHECK_EQ(item.runs, 1U);
}

HOST_TEST(work_queue_schedule_then_clear_while_claimed_cancels_old_and_deferred_runs)
{
    FakeClock clock;
    FakeBackend backend;
    WorkQueue queue{QueueClass::HighPriority, clock, backend};
    CountingItem item{queue};

    CHECK(item.ScheduleNow());
    backend.claim_first(QueueClass::HighPriority);
    CHECK(item.ScheduleNow());
    item.ScheduleClear();
    backend.run_claimed();

    CHECK_EQ(item.runs, 0U);
    CHECK_EQ(backend.active_count(QueueClass::HighPriority), 0U);
}
