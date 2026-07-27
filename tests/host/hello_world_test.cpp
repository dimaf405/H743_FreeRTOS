#include "test_framework.hpp"

#include "App/features/hello_world/hello_world.hpp"
#include "App/runtime/lifecycle/module_base.hpp"
#include "App/runtime/messaging/topic.hpp"
#include "App/runtime/scheduling/scheduled_work_item.hpp"
#include "App/runtime/scheduling/work_queue.hpp"

#include <array>
#include <cstring>
#include <cstdint>
#include <stdexcept>
#include <type_traits>
#include <utility>

namespace {

using app::features::hello_world::HelloWorld;
using app::features::hello_world::HostDependencies;
using app::runtime::lifecycle::ModuleBase;
using app::runtime::lifecycle::ModuleState;
using app::runtime::messaging::Subscription;
using app::runtime::messaging::Topic;
using app::runtime::scheduling::IClock;
using app::runtime::scheduling::IWorkQueueBackend;
using app::runtime::scheduling::QueueClass;
using app::runtime::scheduling::ScheduledWorkItem;
using app::runtime::scheduling::WorkQueue;

template <typename T, typename = void>
struct has_public_Run : std::false_type {};

template <typename T>
struct has_public_Run<T, std::void_t<decltype(std::declval<T &>().Run())>>
    : std::true_type {};

static_assert(std::is_base_of<ModuleBase, HelloWorld>::value,
              "HelloWorld must implement ModuleBase");
static_assert(std::is_base_of<ScheduledWorkItem, HelloWorld>::value,
              "HelloWorld must execute as a ScheduledWorkItem");
static_assert(!has_public_Run<HelloWorld>::value,
              "HelloWorld::Run must not be callable by application code");

class FakeClock final : public IClock {
public:
    std::uint32_t now{0U};

    std::uint32_t now_ms() const override { return now; }
    void advance(std::uint32_t delta_ms) { now += delta_ms; }
};

class FakeBackend final : public IWorkQueueBackend {
public:
    bool fail_next_schedule{false};
    bool active{false};
    QueueClass queue{QueueClass::HighPriority};
    void *context{nullptr};
    ClaimCallback claim{nullptr};
    RunCallback run{nullptr};
    std::uint32_t deadline_ms{0U};
    unsigned schedule_calls{0U};
    unsigned clear_calls{0U};

    bool schedule(QueueClass requested_queue, void *requested_context,
                  ClaimCallback requested_claim, RunCallback requested_run,
                  std::uint32_t requested_deadline_ms) override
    {
        ++schedule_calls;
        if (fail_next_schedule) {
            fail_next_schedule = false;
            return false;
        }
        CHECK(!active);
        active = true;
        queue = requested_queue;
        context = requested_context;
        claim = requested_claim;
        run = requested_run;
        deadline_ms = requested_deadline_ms;
        return true;
    }

    void clear(QueueClass requested_queue, void *requested_context) override
    {
        ++clear_calls;
        if (active && queue == requested_queue && context == requested_context) {
            active = false;
        }
    }

    void run_once()
    {
        if (!active) {
            throw std::runtime_error("no active work item");
        }
        const auto queued_context = context;
        const auto queued_claim = claim;
        const auto queued_run = run;
        active = false;
        CHECK(queued_claim(queued_context));
        queued_run(queued_context);
    }
};

enum class Event : std::uint8_t {
    Print,
    Flush,
    Time,
};

struct FakeDependencies {
    std::array<Event, 16U> events{};
    std::size_t event_count{0U};
    std::uint64_t timestamp_us{0U};
    int print_result{0};
    int flush_result{0};

    void record(Event event)
    {
        CHECK(event_count < events.size());
        events[event_count++] = event;
    }

    static int print(void *context, const char *line)
    {
        auto &fake = *static_cast<FakeDependencies *>(context);
        fake.record(Event::Print);
        CHECK(std::strcmp(line, "Hello World\r\n") == 0);
        return fake.print_result;
    }

    static int flush(void *context)
    {
        auto &fake = *static_cast<FakeDependencies *>(context);
        fake.record(Event::Flush);
        return fake.flush_result;
    }

    static std::uint64_t time_us(void *context)
    {
        auto &fake = *static_cast<FakeDependencies *>(context);
        fake.record(Event::Time);
        return fake.timestamp_us;
    }

    HostDependencies dependencies()
    {
        return {this, &print, &flush, &time_us};
    }
};

} // namespace

HOST_TEST(hello_world_start_uses_the_passed_low_priority_queue_at_1000_ms)
{
    FakeClock clock;
    FakeBackend backend;
    WorkQueue low_priority_queue{QueueClass::LowPriority, clock, backend};
    Topic<app_heartbeat_s> heartbeat_topic;
    FakeDependencies dependencies;
    HelloWorld hello{low_priority_queue, heartbeat_topic,
                     dependencies.dependencies()};

    CHECK_EQ(hello.state(), ModuleState::Stopped);
    CHECK(hello.start());
    CHECK_EQ(hello.state(), ModuleState::Running);
    CHECK_EQ(backend.schedule_calls, 1U);
    CHECK(backend.active);
    CHECK_EQ(backend.queue, QueueClass::LowPriority);
    CHECK_EQ(backend.deadline_ms, 1000U);

    clock.advance(1000U);
    dependencies.timestamp_us = 1000000ULL;
    backend.run_once();
    CHECK_EQ(heartbeat_topic.generation(), 1U);
    CHECK(backend.active);
    CHECK_EQ(backend.deadline_ms, 2000U);

    hello.stop();
    CHECK_EQ(hello.state(), ModuleState::Stopped);
    CHECK(!backend.active);
}

HOST_TEST(hello_world_start_failure_is_error_and_stop_allows_a_clean_retry)
{
    FakeClock clock;
    FakeBackend backend;
    WorkQueue low_priority_queue{QueueClass::LowPriority, clock, backend};
    Topic<app_heartbeat_s> heartbeat_topic;
    FakeDependencies dependencies;
    HelloWorld hello{low_priority_queue, heartbeat_topic,
                     dependencies.dependencies()};
    backend.fail_next_schedule = true;

    CHECK(!hello.start());
    CHECK_EQ(hello.state(), ModuleState::Error);
    CHECK(!backend.active);

    hello.stop();
    CHECK_EQ(hello.state(), ModuleState::Stopped);
    CHECK(hello.start());
    CHECK_EQ(hello.state(), ModuleState::Running);
    hello.stop();
}

HOST_TEST(hello_world_prints_then_flushes_then_samples_time_and_publishes_payload)
{
    FakeClock clock;
    FakeBackend backend;
    WorkQueue low_priority_queue{QueueClass::LowPriority, clock, backend};
    Topic<app_heartbeat_s> heartbeat_topic;
    Subscription<app_heartbeat_s> subscriber{heartbeat_topic};
    FakeDependencies dependencies;
    HelloWorld hello{low_priority_queue, heartbeat_topic,
                     dependencies.dependencies()};
    app_heartbeat_s heartbeat{};
    CHECK(hello.start());

    dependencies.timestamp_us = 1234567ULL;
    hello.RunForTest();
    CHECK_EQ(dependencies.event_count, 3U);
    CHECK_EQ(dependencies.events[0], Event::Print);
    CHECK_EQ(dependencies.events[1], Event::Flush);
    CHECK_EQ(dependencies.events[2], Event::Time);
    CHECK_EQ(heartbeat_topic.generation(), 1U);
    CHECK(subscriber.copy(heartbeat));
    CHECK_EQ(heartbeat.timestamp_us, 1234567ULL);
    CHECK_EQ(heartbeat.sequence, 1U);

    dependencies.timestamp_us = 2234567ULL;
    hello.RunForTest();
    CHECK_EQ(heartbeat_topic.generation(), 2U);
    CHECK(subscriber.copy(heartbeat));
    CHECK_EQ(heartbeat.timestamp_us, 2234567ULL);
    CHECK_EQ(heartbeat.sequence, 2U);
    CHECK_EQ(hello.state(), ModuleState::Running);
    hello.stop();
}

HOST_TEST(hello_world_output_failures_still_publish_and_keep_the_module_running)
{
    FakeClock clock;
    FakeBackend backend;
    WorkQueue low_priority_queue{QueueClass::LowPriority, clock, backend};
    Topic<app_heartbeat_s> heartbeat_topic;
    Subscription<app_heartbeat_s> subscriber{heartbeat_topic};
    FakeDependencies dependencies;
    dependencies.timestamp_us = 9001ULL;
    dependencies.print_result = -1;
    dependencies.flush_result = -1;
    HelloWorld hello{low_priority_queue, heartbeat_topic,
                     dependencies.dependencies()};
    app_heartbeat_s heartbeat{};
    CHECK(hello.start());

    hello.RunForTest();

    CHECK_EQ(dependencies.event_count, 3U);
    CHECK_EQ(dependencies.events[0], Event::Print);
    CHECK_EQ(dependencies.events[1], Event::Flush);
    CHECK_EQ(dependencies.events[2], Event::Time);
    CHECK(subscriber.copy(heartbeat));
    CHECK_EQ(heartbeat.timestamp_us, 9001ULL);
    CHECK_EQ(heartbeat.sequence, 1U);
    CHECK_EQ(hello.state(), ModuleState::Running);
    CHECK(backend.active);
    hello.stop();
}
