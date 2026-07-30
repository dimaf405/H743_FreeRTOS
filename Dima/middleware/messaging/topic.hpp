#pragma once

#include <stdint.h>

#if defined(__arm__) || defined(__thumb__)
extern "C" {
#include "FreeRTOS.h"
#include "task.h"
}
#endif

namespace dima::middleware::messaging {

template <typename T>
inline constexpr bool is_topic_type_v = __is_trivially_copyable(T);

inline bool generation_updated(uint32_t current_generation,
                               uint32_t subscriber_generation)
{
    return current_generation != subscriber_generation;
}

namespace detail {

inline void enter_critical()
{
#if defined(__arm__) || defined(__thumb__)
    taskENTER_CRITICAL();
#endif
}

inline void exit_critical()
{
#if defined(__arm__) || defined(__thumb__)
    taskEXIT_CRITICAL();
#endif
}

} // namespace detail

template <typename T>
class Publication;

template <typename T>
class Subscription;

template <typename T>
class Topic {
    static_assert(is_topic_type_v<T>,
                  "Topic requires trivially-copyable data");

public:
    constexpr Topic() : data_{}, generation_{0U} {}

    uint32_t generation() const
    {
        detail::enter_critical();
        const uint32_t result = generation_;
        detail::exit_critical();
        return result;
    }

private:
    friend class Publication<T>;
    friend class Subscription<T>;

    bool publish(const T &data)
    {
        detail::enter_critical();
        __builtin_memcpy(&data_, &data, sizeof(T));
        ++generation_;
        detail::exit_critical();
        return true;
    }

    bool copy_if_updated(T &destination, uint32_t &subscriber_generation) const
    {
        detail::enter_critical();
        if (!generation_updated(generation_, subscriber_generation)) {
            detail::exit_critical();
            return false;
        }

        __builtin_memcpy(&destination, &data_, sizeof(T));
        subscriber_generation = generation_;
        detail::exit_critical();
        return true;
    }

    T data_;
    uint32_t generation_;
};

template <typename T>
class Publication {
public:
    explicit constexpr Publication(Topic<T> &topic) : topic_(topic) {}

    bool publish(const T &data) { return topic_.publish(data); }

private:
    Topic<T> &topic_;
};

template <typename T>
class Subscription {
public:
    explicit constexpr Subscription(Topic<T> &topic) : topic_(topic) {}

    bool updated() const
    {
        return generation_updated(topic_.generation(), generation_);
    }

    bool copy(T &destination)
    {
        return topic_.copy_if_updated(destination, generation_);
    }

private:
    Topic<T> &topic_;
    uint32_t generation_{0U};
};

} // namespace dima::middleware::messaging
