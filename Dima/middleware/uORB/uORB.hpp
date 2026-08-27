#pragma once

#include "work_queue/WorkQueue.hpp"

#include <stddef.h>
#include <stdint.h>

namespace uORB {

constexpr uint8_t kMaximumInstances = 4U;
constexpr uint8_t kMaximumCallbacksPerInstance = 8U;

struct Allocator {
    void *(*allocate)(size_t size, size_t alignment) noexcept;
    void (*deallocate)(void *ptr) noexcept;
};

struct orb_runtime_instance {
    /* 每个 Topic 实例拥有独立队列、64-bit generation 和 WorkItem 回调槽；
     * 这些运行时字段由薄 ORB_DEFINE 适配绑定到官方 metadata。 */
    uint8_t *buffer;
    uint64_t generation;
    px4::WorkItem *callbacks[kMaximumCallbacksPerInstance];
    uint16_t publisher_count;
};

} // namespace uORB

using orb_id_size_t = uint16_t;

struct orb_metadata {
    /* 前六项严格保持 PX4 v1.17 metadata ABI；末两项只承载 Dima 静态运行时，
     * Topic 名称、尺寸、hash、ID 与队列长度仍全部来自官方模板。 */
    const char *o_name;
    uint16_t o_size;
    uint16_t o_size_no_padding;
    uint32_t message_hash;
    orb_id_size_t o_id;
    uint8_t o_queue;
    uint8_t max_instances;
    uORB::orb_runtime_instance *instances;
};

using orb_id_t = const orb_metadata *;

namespace uORB {

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
    /* shutdown/reinitialize 会推进 epoch；订阅者下一次使用时重置 generation 与
     * callback，避免引用已经释放的实例缓冲或旧生命周期状态。 */
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
