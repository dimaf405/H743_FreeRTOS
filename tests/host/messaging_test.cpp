#include "test_framework.hpp"

#include "App/runtime/messaging/topic.hpp"

#include <cstdint>
#include <string>
#include <type_traits>

namespace {

struct Sample {
    std::uint32_t sequence;
    float value;
};

struct NonTrivialSample {
    std::string text;
};

static_assert(std::is_trivially_copyable<Sample>::value, "test sample must be POD-like");
static_assert(app::runtime::messaging::is_topic_type_v<Sample>,
              "messaging must accept trivially-copyable messages");
static_assert(!app::runtime::messaging::is_topic_type_v<NonTrivialSample>,
              "messaging must reject non-trivially-copyable messages");

} // namespace

HOST_TEST(messaging_publish_increments_generation_and_subscription_copies_latest_value)
{
    app::runtime::messaging::Topic<Sample> topic;
    app::runtime::messaging::Publication<Sample> publisher{topic};
    app::runtime::messaging::Subscription<Sample> subscriber{topic};
    Sample copied{};

    CHECK_EQ(topic.generation(), 0U);
    CHECK(!subscriber.updated());

    CHECK(publisher.publish({1U, 1.5F}));
    CHECK_EQ(topic.generation(), 1U);
    CHECK(subscriber.updated());
    CHECK(subscriber.copy(copied));
    CHECK_EQ(copied.sequence, 1U);
    CHECK_EQ(copied.value, 1.5F);
    CHECK(!subscriber.updated());

    CHECK(publisher.publish({2U, 3.0F}));
    CHECK(publisher.publish({3U, 4.5F}));
    CHECK(subscriber.updated());
    CHECK(subscriber.copy(copied));
    CHECK_EQ(copied.sequence, 3U);
    CHECK_EQ(copied.value, 4.5F);
    CHECK_EQ(topic.generation(), 3U);
}

HOST_TEST(messaging_generation_update_check_remains_true_across_uint32_wraparound)
{
    using app::runtime::messaging::generation_updated;

    CHECK(generation_updated(0U, UINT32_MAX));
    CHECK(generation_updated(1U, UINT32_MAX));
    CHECK(!generation_updated(UINT32_MAX, UINT32_MAX));
}
