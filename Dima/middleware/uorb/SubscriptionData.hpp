#pragma once

#include "uORB.hpp"

#include <type_traits>

namespace uORB {

template<typename T>
class SubscriptionData : public Subscription {
public:
    explicit SubscriptionData(const orb_metadata *metadata,
                              uint8_t instance = 0U) noexcept
        : Subscription(metadata, instance)
    {
        static_assert(std::is_trivially_copyable<T>::value,
                      "uORB 消息必须可按字节复制");
    }

    bool update() noexcept
    {
        if (synchronize_epoch()) {
            data_ = T{};
        }
        return copy(&data_);
    }

    const T &get() const noexcept
    {
        if (synchronize_epoch()) {
            data_ = T{};
        }
        return data_;
    }

    const T *operator->() const noexcept { return &get(); }

private:
    mutable T data_{};
};

} // namespace uORB
