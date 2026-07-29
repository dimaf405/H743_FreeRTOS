#include "Dima/middleware/uorb/uORB.hpp"

extern "C" {
#include "FreeRTOS.h"
#include "task.h"
}

#include "stm32h743xx.h"

namespace uORB {
namespace {

orb_metadata *g_metadata_head{nullptr};
Allocator g_allocator{nullptr};
bool g_initialized{false};

bool in_isr() noexcept
{
    return __get_IPSR() != 0U;
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

bool initialize(Allocator allocator) noexcept
{
    if (g_initialized) {
        return true;
    }
    if (allocator == nullptr || in_isr()) {
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
            instance.buffer = static_cast<uint8_t *>(allocator(bytes, 8U));
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
    g_initialized = true;
    return true;
}

void shutdown() noexcept
{
    // Phase 1 的 Topic Buffer 归 ApplicationContext/Heap 生命周期管理；
    // 当前接口只解除可用状态，不在实时路径释放内存。
    g_initialized = false;
    g_allocator = nullptr;
}

bool initialized() noexcept
{
    return g_initialized;
}

bool orb_publish(const orb_metadata *metadata, uint8_t instance_index,
                 const void *data) noexcept
{
    // 首版明确禁止 ISR 发布，驱动需先 handoff 到任务上下文。
    if (!g_initialized || data == nullptr || in_isr()) {
        return false;
    }
    orb_runtime_instance *const instance = runtime_for(metadata, instance_index);
    if (instance == nullptr || instance->buffer == nullptr) {
        return false;
    }

    px4::WorkItem *callbacks[kMaximumCallbacksPerInstance]{};
    taskENTER_CRITICAL();
    const uint64_t next_generation = instance->generation + 1U;
    const size_t slot = static_cast<size_t>((next_generation - 1U) %
                                            metadata->queue_size);
    memcpy(instance->buffer + slot * metadata->object_size, data,
           metadata->object_size);
    instance->generation = next_generation;
    instance->advertised = true;
    for (uint8_t index = 0U; index < kMaximumCallbacksPerInstance; ++index) {
        callbacks[index] = instance->callbacks[index];
    }
    taskEXIT_CRITICAL();

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
    taskENTER_CRITICAL();
    const uint64_t newest = instance->generation;
    if (newest != generation) {
        uint64_t target = newest;
        if (metadata->queue_size > 1U) {
            const uint64_t oldest = newest > metadata->queue_size
                                        ? newest - metadata->queue_size + 1U
                                        : 1U;
            target = generation + 1U;
            if (target < oldest || generation == 0U) {
                target = oldest;
            }
        }
        const size_t slot = static_cast<size_t>((target - 1U) %
                                                metadata->queue_size);
        memcpy(destination, instance->buffer + slot * metadata->object_size,
               metadata->object_size);
        generation = target;
        copied = true;
    }
    taskEXIT_CRITICAL();
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
    taskENTER_CRITICAL();
    const bool result = instance->generation != generation;
    taskEXIT_CRITICAL();
    return result;
}

int8_t orb_advertise_multi(const orb_metadata *metadata) noexcept
{
    if (!g_initialized || metadata == nullptr || in_isr()) {
        return -1;
    }
    int8_t result = -1;
    taskENTER_CRITICAL();
    for (uint8_t index = 0U; index < metadata->max_instances; ++index) {
        auto &instance = metadata->instances[index];
        if (!instance.advertised) {
            instance.advertised = true;
            result = static_cast<int8_t>(index);
            break;
        }
    }
    taskEXIT_CRITICAL();
    return result;
}

bool Subscription::updated() const noexcept
{
    return orb_updated(metadata_, instance_, generation_);
}

bool Subscription::copy(void *destination) noexcept
{
    return orb_copy(metadata_, instance_, generation_, destination);
}

bool Subscription::registerCallback(px4::WorkItem &work_item) noexcept
{
    if (!g_initialized || in_isr()) {
        return false;
    }
    orb_runtime_instance *const instance = runtime_for(metadata_, instance_);
    if (instance == nullptr) {
        return false;
    }

    bool registered = false;
    taskENTER_CRITICAL();
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
    taskEXIT_CRITICAL();
    return registered;
}

void Subscription::unregisterCallback() noexcept
{
    if (callback_ == nullptr || in_isr()) {
        return;
    }
    orb_runtime_instance *const instance = runtime_for(metadata_, instance_);
    if (instance == nullptr) {
        callback_ = nullptr;
        return;
    }
    taskENTER_CRITICAL();
    for (auto &entry : instance->callbacks) {
        if (entry == callback_) {
            entry = nullptr;
        }
    }
    callback_ = nullptr;
    taskEXIT_CRITICAL();
}

} // namespace uORB

