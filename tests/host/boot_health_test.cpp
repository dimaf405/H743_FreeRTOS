#include "test_framework.hpp"

#include "Dima/adapters/mcuboot/mcuboot_app.h"
#include "Dima/modules/boot_health/boot_health.hpp"
#include "Dima/messages/app_heartbeat.hpp"
#include "Dima/middleware/lifecycle/module_base.hpp"
#include "Dima/middleware/messaging/topic.hpp"
#include "Dima/middleware/scheduling/scheduled_work_item.hpp"
#include "Dima/middleware/scheduling/work_queue.hpp"

#include <array>
#include <cstdint>
#include <stdexcept>
#include <type_traits>
#include <utility>

namespace {

using dima::modules::boot_health::BootHealthService;
using dima::modules::boot_health::HostDependencies;
using dima::middleware::lifecycle::ModuleBase;
using dima::middleware::lifecycle::ModuleState;
using dima::middleware::messaging::Publication;
using dima::middleware::messaging::Topic;
using dima::middleware::scheduling::IClock;
using dima::middleware::scheduling::IWorkQueueBackend;
using dima::middleware::scheduling::QueueClass;
using dima::middleware::scheduling::ScheduledWorkItem;
using dima::middleware::scheduling::WorkQueue;

template <typename T, typename = void>
struct has_public_Run : std::false_type {};

template <typename T>
struct has_public_Run<T, std::void_t<decltype(std::declval<T &>().Run())>>
    : std::true_type {};

static_assert(std::is_base_of<ModuleBase, BootHealthService>::value,
              "BootHealthService must implement ModuleBase");
static_assert(std::is_base_of<ScheduledWorkItem, BootHealthService>::value,
              "BootHealthService must execute as a ScheduledWorkItem");
static_assert(!has_public_Run<BootHealthService>::value,
              "BootHealthService::Run must not be public");

class FakeClock final : public IClock {
public:
    std::uint32_t now{0U};
    std::uint32_t now_ms() const override { return now; }
};

class FakeBackend final : public IWorkQueueBackend {
public:
    bool fail_next_schedule{false};
    bool active{false};
    QueueClass queue{QueueClass::LowPriority};
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

struct FakeDependencies {
    std::uint64_t now_ms{0U};
    int confirm_result{MCUBOOT_CONFIRM_OK};
    unsigned confirm_calls{0U};

    static std::uint64_t time_ms(void *context)
    {
        return static_cast<FakeDependencies *>(context)->now_ms;
    }

    static int confirm(void *context)
    {
        auto &fake = *static_cast<FakeDependencies *>(context);
        ++fake.confirm_calls;
        return fake.confirm_result;
    }

    HostDependencies dependencies()
    {
        return {this, &time_ms, &confirm};
    }
};

void publish_heartbeat(Topic<app_heartbeat_s> &topic, std::uint32_t sequence = 1U)
{
    Publication<app_heartbeat_s> publisher{topic};
    CHECK(publisher.publish({sequence * 1000ULL, sequence}));
}

} // namespace

HOST_TEST(boot_health_uses_the_passed_high_priority_queue_at_a_100_ms_period)
{
    FakeClock queue_clock;
    queue_clock.now = 40U;
    FakeBackend backend;
    WorkQueue high_priority_queue{QueueClass::HighPriority, queue_clock, backend};
    Topic<app_heartbeat_s> heartbeat_topic;
    FakeDependencies dependencies;
    dependencies.now_ms = 1000U;
    BootHealthService service{high_priority_queue, heartbeat_topic,
                              dependencies.dependencies()};

    CHECK_EQ(service.state(), ModuleState::Stopped);
    CHECK(service.start());
    CHECK_EQ(service.state(), ModuleState::Running);
    CHECK_EQ(backend.schedule_calls, 1U);
    CHECK(backend.active);
    CHECK_EQ(backend.queue, QueueClass::HighPriority);
    CHECK_EQ(backend.deadline_ms, 140U);

    queue_clock.now = 140U;
    backend.run_once();
    CHECK(backend.active);
    CHECK_EQ(backend.deadline_ms, 240U);
    service.stop();
}

HOST_TEST(boot_health_does_not_confirm_at_4999_ms_even_after_a_heartbeat)
{
    FakeClock queue_clock;
    FakeBackend backend;
    WorkQueue queue{QueueClass::HighPriority, queue_clock, backend};
    Topic<app_heartbeat_s> heartbeat_topic;
    FakeDependencies dependencies;
    dependencies.now_ms = 1000U;
    BootHealthService service{queue, heartbeat_topic, dependencies.dependencies()};
    CHECK(service.start());
    publish_heartbeat(heartbeat_topic);

    dependencies.now_ms = 5999U;
    service.RunForTest();

    CHECK_EQ(dependencies.confirm_calls, 0U);
    CHECK_EQ(service.state(), ModuleState::Running);
    CHECK(backend.active);
    service.stop();
}

HOST_TEST(boot_health_does_not_confirm_at_5000_ms_without_a_heartbeat)
{
    FakeClock queue_clock;
    FakeBackend backend;
    WorkQueue queue{QueueClass::HighPriority, queue_clock, backend};
    Topic<app_heartbeat_s> heartbeat_topic;
    FakeDependencies dependencies;
    BootHealthService service{queue, heartbeat_topic, dependencies.dependencies()};
    CHECK(service.start());

    dependencies.now_ms = 5000U;
    service.RunForTest();

    CHECK_EQ(dependencies.confirm_calls, 0U);
    CHECK_EQ(service.state(), ModuleState::Running);
    service.stop();
}

HOST_TEST(boot_health_ignores_a_heartbeat_published_before_start)
{
    FakeClock queue_clock;
    FakeBackend backend;
    WorkQueue queue{QueueClass::HighPriority, queue_clock, backend};
    Topic<app_heartbeat_s> heartbeat_topic;
    FakeDependencies dependencies;
    BootHealthService service{queue, heartbeat_topic, dependencies.dependencies()};
    publish_heartbeat(heartbeat_topic);

    CHECK(service.start());
    dependencies.now_ms = 5000U;
    service.RunForTest();

    CHECK_EQ(dependencies.confirm_calls, 0U);
    CHECK_EQ(service.state(), ModuleState::Running);
    CHECK(backend.active);
    service.stop();
}

HOST_TEST(boot_health_ignores_a_heartbeat_published_while_stopped)
{
    FakeClock queue_clock;
    FakeBackend backend;
    WorkQueue queue{QueueClass::HighPriority, queue_clock, backend};
    Topic<app_heartbeat_s> heartbeat_topic;
    FakeDependencies dependencies;
    BootHealthService service{queue, heartbeat_topic, dependencies.dependencies()};
    CHECK(service.start());
    service.stop();
    publish_heartbeat(heartbeat_topic);

    dependencies.now_ms = 100U;
    CHECK(service.start());
    dependencies.now_ms = 5100U;
    service.RunForTest();

    CHECK_EQ(dependencies.confirm_calls, 0U);
    CHECK_EQ(service.state(), ModuleState::Running);
    CHECK(backend.active);
    service.stop();
}

HOST_TEST(boot_health_confirms_once_when_heartbeat_precedes_the_stability_gate)
{
    FakeClock queue_clock;
    FakeBackend backend;
    WorkQueue queue{QueueClass::HighPriority, queue_clock, backend};
    Topic<app_heartbeat_s> heartbeat_topic;
    FakeDependencies dependencies;
    BootHealthService service{queue, heartbeat_topic, dependencies.dependencies()};
    CHECK(service.start());

    dependencies.now_ms = 100U;
    publish_heartbeat(heartbeat_topic);
    service.RunForTest();
    CHECK_EQ(dependencies.confirm_calls, 0U);

    dependencies.now_ms = 5000U;
    service.RunForTest();
    CHECK_EQ(dependencies.confirm_calls, 1U);
    CHECK_EQ(service.state(), ModuleState::Running);
    CHECK(!backend.active);
}

HOST_TEST(boot_health_confirms_once_when_stability_precedes_the_heartbeat_gate)
{
    FakeClock queue_clock;
    FakeBackend backend;
    WorkQueue queue{QueueClass::HighPriority, queue_clock, backend};
    Topic<app_heartbeat_s> heartbeat_topic;
    FakeDependencies dependencies;
    BootHealthService service{queue, heartbeat_topic, dependencies.dependencies()};
    CHECK(service.start());

    dependencies.now_ms = 5000U;
    service.RunForTest();
    CHECK_EQ(dependencies.confirm_calls, 0U);

    publish_heartbeat(heartbeat_topic);
    dependencies.now_ms = 5100U;
    service.RunForTest();
    CHECK_EQ(dependencies.confirm_calls, 1U);
    CHECK_EQ(service.state(), ModuleState::Running);
    CHECK(!backend.active);
}

HOST_TEST(boot_health_accepts_all_three_non_error_confirmation_results)
{
    constexpr std::array<int, 3U> accepted_results{
        MCUBOOT_CONFIRM_OK,
        MCUBOOT_CONFIRM_ALREADY_CONFIRMED,
        MCUBOOT_CONFIRM_NOT_A_TEST_IMAGE,
    };
    for (const int result : accepted_results) {
        FakeClock queue_clock;
        FakeBackend backend;
        WorkQueue queue{QueueClass::HighPriority, queue_clock, backend};
        Topic<app_heartbeat_s> heartbeat_topic;
        FakeDependencies dependencies;
        dependencies.confirm_result = result;
        BootHealthService service{queue, heartbeat_topic,
                                  dependencies.dependencies()};
        CHECK(service.start());
        publish_heartbeat(heartbeat_topic);
        dependencies.now_ms = 5000U;

        service.RunForTest();

        CHECK_EQ(dependencies.confirm_calls, 1U);
        CHECK_EQ(service.state(), ModuleState::Running);
        CHECK(!backend.active);
    }
}

HOST_TEST(boot_health_latches_flash_and_unknown_confirmation_failures_as_error)
{
    constexpr std::array<int, 2U> error_results{
        MCUBOOT_CONFIRM_FLASH_ERROR,
        77,
    };
    for (const int result : error_results) {
        FakeClock queue_clock;
        FakeBackend backend;
        WorkQueue queue{QueueClass::HighPriority, queue_clock, backend};
        Topic<app_heartbeat_s> heartbeat_topic;
        FakeDependencies dependencies;
        dependencies.confirm_result = result;
        BootHealthService service{queue, heartbeat_topic,
                                  dependencies.dependencies()};
        CHECK(service.start());
        publish_heartbeat(heartbeat_topic);
        dependencies.now_ms = 5000U;

        service.RunForTest();

        CHECK_EQ(dependencies.confirm_calls, 1U);
        CHECK_EQ(service.state(), ModuleState::Error);
        CHECK(!backend.active);
    }
}

HOST_TEST(boot_health_success_latch_survives_extra_runs_and_stop_start)
{
    FakeClock queue_clock;
    FakeBackend backend;
    WorkQueue queue{QueueClass::HighPriority, queue_clock, backend};
    Topic<app_heartbeat_s> heartbeat_topic;
    FakeDependencies dependencies;
    BootHealthService service{queue, heartbeat_topic, dependencies.dependencies()};
    CHECK(service.start());
    publish_heartbeat(heartbeat_topic);
    dependencies.now_ms = 5000U;
    service.RunForTest();
    CHECK_EQ(dependencies.confirm_calls, 1U);

    publish_heartbeat(heartbeat_topic, 2U);
    dependencies.now_ms = 10000U;
    service.RunForTest();
    CHECK_EQ(dependencies.confirm_calls, 1U);
    service.stop();
    CHECK_EQ(service.state(), ModuleState::Stopped);
    CHECK(service.start());
    CHECK_EQ(service.state(), ModuleState::Running);
    CHECK_EQ(backend.schedule_calls, 1U);
    service.RunForTest();
    CHECK_EQ(dependencies.confirm_calls, 1U);
}

HOST_TEST(boot_health_error_remains_observable_and_never_retries)
{
    FakeClock queue_clock;
    FakeBackend backend;
    WorkQueue queue{QueueClass::HighPriority, queue_clock, backend};
    Topic<app_heartbeat_s> heartbeat_topic;
    FakeDependencies dependencies;
    dependencies.confirm_result = MCUBOOT_CONFIRM_FLASH_ERROR;
    BootHealthService service{queue, heartbeat_topic, dependencies.dependencies()};
    CHECK(service.start());
    publish_heartbeat(heartbeat_topic);
    dependencies.now_ms = 5000U;
    service.RunForTest();
    CHECK_EQ(service.state(), ModuleState::Error);
    CHECK_EQ(dependencies.confirm_calls, 1U);

    service.RunForTest();
    service.stop();
    CHECK_EQ(service.state(), ModuleState::Error);
    CHECK(!service.start());
    CHECK_EQ(service.state(), ModuleState::Error);
    CHECK_EQ(dependencies.confirm_calls, 1U);
    CHECK_EQ(backend.schedule_calls, 1U);
}

HOST_TEST(boot_health_queue_failure_is_a_sticky_error)
{
    FakeClock queue_clock;
    FakeBackend backend;
    backend.fail_next_schedule = true;
    WorkQueue queue{QueueClass::HighPriority, queue_clock, backend};
    Topic<app_heartbeat_s> heartbeat_topic;
    FakeDependencies dependencies;
    BootHealthService service{queue, heartbeat_topic, dependencies.dependencies()};

    CHECK(!service.start());
    CHECK_EQ(service.state(), ModuleState::Error);
    CHECK(!backend.active);
    service.stop();
    CHECK_EQ(service.state(), ModuleState::Error);
    CHECK(!service.start());
    CHECK_EQ(backend.schedule_calls, 1U);
    CHECK_EQ(dependencies.confirm_calls, 0U);
}
