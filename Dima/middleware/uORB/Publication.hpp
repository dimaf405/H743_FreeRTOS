#pragma once

#include "uORB.hpp"

#include <type_traits>

namespace uORB {

template<typename T>
class Publication {
public:
    explicit Publication(const orb_metadata *metadata,
                         uint8_t instance = 0U) noexcept
        : metadata_(metadata), instance_(instance)
    {
        static_assert(std::is_trivially_copyable<T>::value,
                      "uORB 消息必须可按字节复制");
    }

    ~Publication()
    {
        synchronize_epoch();
        if (advertised_) {
            orb_unadvertise(metadata_, instance_);
        }
    }
    Publication(const Publication &) = delete;
    Publication &operator=(const Publication &) = delete;

    bool publish(const T &data) noexcept
    {
        if (advertise() && orb_publish(metadata_, instance_, &data)) {
            return true;
        }
        release();
        if (advertise() && orb_publish(metadata_, instance_, &data)) {
            return true;
        }
        release();
        return false;
    }

    bool advertise() noexcept
    {
        synchronize_epoch();
        if (!advertised_) {
            advertised_ = orb_advertise(metadata_, instance_);
        }
        return advertised_;
    }

private:
    void synchronize_epoch() noexcept
    {
        const uint64_t current = lifecycle_epoch();
        if (epoch_ != current) {
            advertised_ = false;
            epoch_ = current;
        }
    }

    void release() noexcept
    {
        synchronize_epoch();
        if (advertised_) {
            orb_unadvertise(metadata_, instance_);
            advertised_ = false;
        }
    }

    const orb_metadata *metadata_;
    uint8_t instance_;
    bool advertised_{false};
    uint64_t epoch_{0U};
};

template<typename T>
class PublicationMulti {
public:
    explicit PublicationMulti(const orb_metadata *metadata) noexcept
        : metadata_(metadata)
    {
        static_assert(std::is_trivially_copyable<T>::value,
                      "uORB 消息必须可按字节复制");
    }

    ~PublicationMulti()
    {
        synchronize_epoch();
        if (instance_ >= 0) {
            orb_unadvertise(metadata_, static_cast<uint8_t>(instance_));
        }
    }
    PublicationMulti(const PublicationMulti &) = delete;
    PublicationMulti &operator=(const PublicationMulti &) = delete;

    bool advertise() noexcept
    {
        synchronize_epoch();
        if (instance_ < 0) {
            instance_ = orb_advertise_multi(metadata_);
        }
        return instance_ >= 0;
    }

    bool publish(const T &data) noexcept
    {
        if (advertise() &&
            orb_publish(metadata_, static_cast<uint8_t>(instance_), &data)) {
            return true;
        }
        release();
        if (advertise() &&
            orb_publish(metadata_, static_cast<uint8_t>(instance_), &data)) {
            return true;
        }
        release();
        return false;
    }

    int8_t instance() const noexcept
    {
        synchronize_epoch();
        return instance_;
    }

private:
    void synchronize_epoch() const noexcept
    {
        const uint64_t current = lifecycle_epoch();
        if (epoch_ != current) {
            instance_ = -1;
            epoch_ = current;
        }
    }

    void release() noexcept
    {
        synchronize_epoch();
        if (instance_ >= 0) {
            orb_unadvertise(metadata_, static_cast<uint8_t>(instance_));
            instance_ = -1;
        }
    }

    const orb_metadata *metadata_;
    mutable int8_t instance_{-1};
    mutable uint64_t epoch_{0U};
};

} // namespace uORB
