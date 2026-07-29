#pragma once

#include "Dima/middleware/work_queue/WorkQueue.hpp"

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <type_traits>

namespace uORB {

constexpr uint8_t kMaximumInstances = 4U;
constexpr uint8_t kMaximumCallbacksPerInstance = 8U;

struct Allocator {
    void *(*allocate)(size_t size, size_t alignment) noexcept;
    void (*deallocate)(void *ptr) noexcept;
};

struct orb_metadata;

struct orb_runtime_instance {
    uint8_t *buffer;
    uint64_t generation;
    px4::WorkItem *callbacks[kMaximumCallbacksPerInstance];
    bool advertised;
};

struct orb_metadata {
    const char *name;
    uint16_t object_size;
    uint8_t queue_size;
    uint8_t max_instances;
    orb_runtime_instance *instances;
    orb_metadata *next;
};

void register_metadata(orb_metadata *metadata) noexcept;
bool initialize(const Allocator &allocator) noexcept;
void shutdown() noexcept;
bool initialized() noexcept;

bool orb_publish(const orb_metadata *metadata, uint8_t instance,
                 const void *data) noexcept;
bool orb_copy(const orb_metadata *metadata, uint8_t instance,
              uint64_t &generation, void *destination) noexcept;
bool orb_updated(const orb_metadata *metadata, uint8_t instance,
                 uint64_t generation) noexcept;
bool orb_advertise(const orb_metadata *metadata, uint8_t instance) noexcept;
void orb_unadvertise(const orb_metadata *metadata, uint8_t instance) noexcept;
int8_t orb_advertise_multi(const orb_metadata *metadata) noexcept;

class MetadataRegistrar {
public:
    explicit MetadataRegistrar(orb_metadata *metadata) noexcept
    {
        register_metadata(metadata);
    }
};

class Subscription {
public:
    explicit Subscription(const orb_metadata *metadata,
                          uint8_t instance = 0U) noexcept
        : metadata_(metadata), instance_(instance)
    {
    }

    bool updated() const noexcept;
    bool copy(void *destination) noexcept;
    bool registerCallback(px4::WorkItem &work_item) noexcept;
    void unregisterCallback() noexcept;

protected:
    const orb_metadata *metadata_;
    uint8_t instance_;
    uint64_t generation_{0U};
    px4::WorkItem *callback_{nullptr};
};

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
        advertised_ = false;
        return advertise() && orb_publish(metadata_, instance_, &data);
    }

    bool advertise() noexcept
    {
        if (!advertised_) {
            advertised_ = orb_advertise(metadata_, instance_);
        }
        return advertised_;
    }

private:
    const orb_metadata *metadata_;
    uint8_t instance_;
    bool advertised_{false};
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
        if (instance_ >= 0) {
            orb_unadvertise(metadata_, static_cast<uint8_t>(instance_));
        }
    }
    PublicationMulti(const PublicationMulti &) = delete;
    PublicationMulti &operator=(const PublicationMulti &) = delete;

    bool advertise() noexcept
    {
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
        instance_ = orb_advertise_multi(metadata_);
        return instance_ >= 0 &&
               orb_publish(metadata_, static_cast<uint8_t>(instance_), &data);
    }

    int8_t instance() const noexcept { return instance_; }

private:
    const orb_metadata *metadata_;
    int8_t instance_{-1};
};

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

    bool update() noexcept { return copy(&data_); }
    const T &get() const noexcept { return data_; }
    const T *operator->() const noexcept { return &data_; }

private:
    T data_{};
};

class SubscriptionCallbackWorkItem : public Subscription {
public:
    SubscriptionCallbackWorkItem(const orb_metadata *metadata,
                                 px4::WorkItem &work_item,
                                 uint8_t instance = 0U) noexcept
        : Subscription(metadata, instance), work_item_(work_item)
    {
    }

    bool registerCallback() noexcept
    {
        return Subscription::registerCallback(work_item_);
    }

private:
    px4::WorkItem &work_item_;
};

} // namespace uORB

#define ORB_DECLARE(_name) extern uORB::orb_metadata __orb_##_name
#define ORB_ID(_name) (&__orb_##_name)
#define ORB_DEFINE(_name, _type, _queue_size)                                  \
    static uORB::orb_runtime_instance __orb_runtime_##_name[                  \
        uORB::kMaximumInstances]{};                                            \
    uORB::orb_metadata __attribute__((used, section(".dima_orb_meta")))       \
        __orb_##_name{#_name, static_cast<uint16_t>(sizeof(_type)),            \
                      static_cast<uint8_t>(_queue_size),                       \
                      uORB::kMaximumInstances, __orb_runtime_##_name, nullptr};\
    static uORB::MetadataRegistrar __orb_registrar_##_name(&__orb_##_name)
