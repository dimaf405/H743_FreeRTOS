#include "uORB.hpp"

#include "platform/api/Platform.hpp"

namespace uORB {
namespace {

orb_metadata *g_metadata_head{nullptr};
Allocator g_allocator{nullptr, nullptr};
bool g_initialized{false};
uint64_t g_lifecycle_epoch{0U};

bool in_isr() noexcept
{
    return dima::platform::in_interrupt_context();
}

orb_runtime_instance *runtime_for(const orb_metadata *metadata,
                                  uint8_t instance) noexcept
{
    if (metadata == nullptr || instance >= metadata->max_instances ||
        metadata->instances == nullptr) {
        return nullptr;
    }
    return &metadata->instances[instance];
}

} // namespace

void register_metadata(orb_metadata *metadata) noexcept
{
    if (metadata == nullptr) {
        return;
    }
    // 注册发生在静态构造阶段，此时调度器尚未启动，无需进入 RTOS 临界区。
    metadata->next = g_metadata_head;
    g_metadata_head = metadata;
}

bool initialize(const Allocator &allocator) noexcept
{
    if (g_initialized) {
        return true;
    }
    if (allocator.allocate == nullptr || allocator.deallocate == nullptr || in_isr()) {
        return false;
    }

    g_allocator = allocator;
    for (orb_metadata *metadata = g_metadata_head; metadata != nullptr;
         metadata = metadata->next) {
        if (metadata->object_size == 0U || metadata->queue_size == 0U ||
            metadata->max_instances == 0U ||
            metadata->max_instances > kMaximumInstances) {
            shutdown();
            return false;
        }
        const size_t bytes = static_cast<size_t>(metadata->object_size) *
                             metadata->queue_size;
        for (uint8_t index = 0U; index < metadata->max_instances; ++index) {
            auto &instance = metadata->instances[index];
            instance.buffer = static_cast<uint8_t *>(allocator.allocate(bytes, 8U));
            if (instance.buffer == nullptr) {
                shutdown();
                return false;
            }
            memset(instance.buffer, 0, bytes);
            instance.generation = 0U;
            instance.advertised = false;
            for (auto &callback : instance.callbacks) {
                callback = nullptr;
            }
        }
    }
    {
        dima::platform::CriticalGuard guard;
        ++g_lifecycle_epoch;
        if (g_lifecycle_epoch == 0U) {
            ++g_lifecycle_epoch;
        }
        g_initialized = true;
    }
    return true;
}

void shutdown() noexcept
{
    for (orb_metadata *metadata = g_metadata_head; metadata != nullptr;
         metadata = metadata->next) {
        for (uint8_t index = 0U; index < metadata->max_instances; ++index) {
            auto &instance = metadata->instances[index];
            if (instance.buffer != nullptr && g_allocator.deallocate != nullptr) {
                g_allocator.deallocate(instance.buffer);
            }
            instance.buffer = nullptr;
            instance.generation = 0U;
            instance.advertised = false;
            for (auto &callback : instance.callbacks) {
                callback = nullptr;
            }
        }
    }
    g_initialized = false;
    g_allocator = Allocator{nullptr, nullptr};
}

bool initialized() noexcept
{
    return g_initialized;
}

uint64_t lifecycle_epoch() noexcept
{
    dima::platform::CriticalGuard guard;
    return g_lifecycle_epoch;
}

bool orb_publish(const orb_metadata *metadata, uint8_t instance_index,
                 const void *data) noexcept
{
    // 首版明确禁止 ISR 发布，驱动需先 handoff 到任务上下文。
    if (!g_initialized || data == nullptr || in_isr()) {
        return false;
    }
    orb_runtime_instance *const instance = runtime_for(metadata, instance_index);
    if (instance == nullptr || instance->buffer == nullptr ||
        !instance->advertised) {
        return false;
    }

    px4::WorkItem *callbacks[kMaximumCallbacksPerInstance]{};
    {
        dima::platform::CriticalGuard guard;
        const uint64_t next_generation = instance->generation + 1U;
        const size_t slot = static_cast<size_t>((next_generation - 1U) %
                                                metadata->queue_size);
        memcpy(instance->buffer + slot * metadata->object_size, data,
               metadata->object_size);
        instance->generation = next_generation;
        for (uint8_t index = 0U; index < kMaximumCallbacksPerInstance;
             ++index) {
            callbacks[index] = instance->callbacks[index];
        }
    }

    // 回调只触发 WorkItem 调度，不在发布者上下文执行模块 Run()。
    for (auto *callback : callbacks) {
        if (callback != nullptr) {
            (void)callback->ScheduleNow();
        }
    }
    return true;
}

bool orb_copy(const orb_metadata *metadata, uint8_t instance_index,
              uint64_t &generation, void *destination) noexcept
{
    if (!g_initialized || destination == nullptr || in_isr()) {
        return false;
    }
    orb_runtime_instance *const instance = runtime_for(metadata, instance_index);
    if (instance == nullptr || instance->buffer == nullptr) {
        return false;
    }

    bool copied = false;
    dima::platform::CriticalGuard guard;
    const uint64_t newest = instance->generation;
    if (newest == 0U) {
        generation = 0U;
        return false;
    }
    if (newest != generation) {
        uint64_t target = newest;
        if (metadata->queue_size > 1U) {
            const uint64_t oldest = newest > metadata->queue_size
                                        ? newest - metadata->queue_size + 1U
                                        : 1U;
            if (generation == 0U || generation > newest ||
                generation < oldest - 1U) {
                target = oldest;
            } else {
                target = generation + 1U;
            }
        }
        const size_t slot = static_cast<size_t>((target - 1U) %
                                                metadata->queue_size);
        memcpy(destination, instance->buffer + slot * metadata->object_size,
               metadata->object_size);
        generation = target;
        copied = true;
    }
    return copied;
}

bool orb_updated(const orb_metadata *metadata, uint8_t instance_index,
                 uint64_t generation) noexcept
{
    if (!g_initialized || in_isr()) {
        return false;
    }
    orb_runtime_instance *const instance = runtime_for(metadata, instance_index);
    if (instance == nullptr) {
        return false;
    }
    dima::platform::CriticalGuard guard;
    const bool result = instance->generation != generation;
    return result;
}

bool orb_advertise(const orb_metadata *metadata, uint8_t instance_index) noexcept
{
    if (!g_initialized || in_isr()) {
        return false;
    }
    orb_runtime_instance *const instance = runtime_for(metadata, instance_index);
    if (instance == nullptr || instance->buffer == nullptr) {
        return false;
    }
    bool accepted = false;
    dima::platform::CriticalGuard guard;
    if (!instance->advertised) {
        instance->advertised = true;
        accepted = true;
    }
    return accepted;
}

void orb_unadvertise(const orb_metadata *metadata, uint8_t instance_index) noexcept
{
    if (!g_initialized || in_isr()) {
        return;
    }
    orb_runtime_instance *const instance = runtime_for(metadata, instance_index);
    if (instance == nullptr) {
        return;
    }
    dima::platform::CriticalGuard guard;
    instance->advertised = false;
}

int8_t orb_advertise_multi(const orb_metadata *metadata) noexcept
{
    if (!g_initialized || metadata == nullptr || in_isr()) {
        return -1;
    }
    int8_t result = -1;
    dima::platform::CriticalGuard guard;
    for (uint8_t index = 0U; index < metadata->max_instances; ++index) {
        auto &instance = metadata->instances[index];
        if (!instance.advertised) {
            instance.advertised = true;
            result = static_cast<int8_t>(index);
            break;
        }
    }
    return result;
}

bool Subscription::updated() const noexcept
{
    (void)synchronize_epoch();
    return orb_updated(metadata_, instance_, generation_);
}

bool Subscription::copy(void *destination) noexcept
{
    (void)synchronize_epoch();
    return orb_copy(metadata_, instance_, generation_, destination);
}

bool Subscription::synchronize_epoch() const noexcept
{
    const uint64_t current = lifecycle_epoch();
    if (epoch_ == current) {
        return false;
    }
    generation_ = 0U;
    callback_ = nullptr;
    epoch_ = current;
    return true;
}

bool Subscription::registerCallback(px4::WorkItem &work_item) noexcept
{
    (void)synchronize_epoch();
    if (!g_initialized || in_isr()) {
        return false;
    }
    orb_runtime_instance *const instance = runtime_for(metadata_, instance_);
    if (instance == nullptr) {
        return false;
    }

    bool registered = false;
    dima::platform::CriticalGuard guard;
    for (auto *entry : instance->callbacks) {
        if (entry == &work_item) {
            callback_ = &work_item;
            registered = true;
            break;
        }
    }
    if (!registered) {
        for (auto &entry : instance->callbacks) {
            if (entry == nullptr) {
                entry = &work_item;
                callback_ = &work_item;
                registered = true;
                break;
            }
        }
    }
    return registered;
}

void Subscription::unregisterCallback() noexcept
{
    if (synchronize_epoch()) {
        return;
    }
    if (callback_ == nullptr || in_isr()) {
        return;
    }
    orb_runtime_instance *const instance = runtime_for(metadata_, instance_);
    if (instance == nullptr) {
        callback_ = nullptr;
        return;
    }
    dima::platform::CriticalGuard guard;
    for (auto &entry : instance->callbacks) {
        if (entry == callback_) {
            entry = nullptr;
        }
    }
    callback_ = nullptr;
}

} // namespace uORB

