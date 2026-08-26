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
    /* 每个实例拥有独立环形数据区、64-bit 发布代数、最多 8 个 WorkItem 回调和
     * 发布者引用计数；这些字段由生成的 metadata 指向，业务不得手写目录。 */
    uint8_t *buffer;
    uint64_t generation;
    px4::WorkItem *callbacks[kMaximumCallbacksPerInstance];
    uint16_t publisher_count;
};

struct orb_metadata {
    /* object_size/queue_size 来自权威 .msg 生成合同；max_instances 在当前运行时
     * 固定为 4，instances 指向同一生成单元中的静态运行态数组。 */
    const char *name;
    uint16_t object_size;
    uint8_t queue_size;
    uint8_t max_instances;
    orb_runtime_instance *instances;
};

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
    /* uORB shutdown/reinitialize 会推进 epoch；旧订阅在下次使用时清 generation 与
     * callback，避免把已释放实例缓冲区的状态带入新生命周期。 */
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

#define ORB_DECLARE(_name) extern const uORB::orb_metadata __orb_##_name
#define ORB_ID(_name) (&__orb_##_name)
#if defined(H743_APPLICATION_IMAGE)
#define DIMA_ORB_METADATA_SECTION ".dima_orb_meta"
#else
/* GNU host linker 只会为标识符形式的 section 合成 __start/__stop；固件则由链接
 * 脚本显式导出带点号 section 的边界。 */
#define DIMA_ORB_METADATA_SECTION "dima_orb_meta"
#endif
#define ORB_DEFINE(_name, _type, _queue_size)                                  \
    static_assert((_queue_size) > 0U && (_queue_size) <= 0xFFU,                \
                  "uORB queue size is outside metadata range");              \
    static_assert(sizeof(_type) <= 0xFFFFU,                                    \
                  "uORB object size is outside metadata range");             \
    static uORB::orb_runtime_instance __orb_runtime_##_name[                  \
        uORB::kMaximumInstances]{};                                            \
    const uORB::orb_metadata __attribute__((                                   \
        used, section(DIMA_ORB_METADATA_SECTION),                              \
        aligned(alignof(uORB::orb_metadata))))                                 \
        __orb_##_name{#_name, static_cast<uint16_t>(sizeof(_type)),            \
                      static_cast<uint8_t>(_queue_size),                       \
                      uORB::kMaximumInstances, __orb_runtime_##_name}
