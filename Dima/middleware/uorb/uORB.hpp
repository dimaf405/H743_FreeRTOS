#pragma once

#include "work_queue/WorkQueue.hpp"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

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
    uint16_t publisher_count;
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
uint64_t lifecycle_epoch() noexcept;

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
    bool synchronize_epoch() const noexcept;
    const orb_metadata *metadata_;
    uint8_t instance_;
    mutable uint64_t generation_{0U};
    mutable uint64_t epoch_{0U};
    mutable px4::WorkItem *callback_{nullptr};
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
