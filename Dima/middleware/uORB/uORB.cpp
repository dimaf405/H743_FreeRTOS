#include "uORB.hpp"

#include "api/Execution.hpp"
#include <uORB/topics/uORBTopics.hpp>

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <limits>

namespace uORB {

namespace {

Allocator g_allocator{nullptr, nullptr};
bool g_initialized{false};
uint64_t g_lifecycle_epoch{0U};

/* metadata 目录与顺序由 PX4 官方 uORBTopics.cpp 汇总，不维护手写消息表。
 * 启动时仍验证元素完整性、ID 连续性、名称唯一性和运行态数组不别名。 */
struct MetadataRange {
    const orb_metadata *const *topics{nullptr};
    size_t count{0U};
};

bool in_isr() noexcept
{
    return dima::platform::in_interrupt_context();
}

bool metadata_range(MetadataRange &range) noexcept
{
    // Topic 指针数组及其顺序由 PX4 官方 uORBTopics.cpp 唯一生成。
    range.topics = orb_get_topics();
    range.count = orb_topics_count();
    return range.topics != nullptr && range.count > 0U;
}

bool valid_topic_name(const char *name) noexcept
{
    constexpr size_t kMaximumTopicNameLength = 63U;
    if (name == nullptr || name[0] == '\0') {
        return false;
    }
    for (size_t index = 1U; index <= kMaximumTopicNameLength; ++index) {
        if (name[index] == '\0') {
            return true;
        }
    }
    return false;
}

bool valid_metadata(const orb_metadata &metadata) noexcept
{
    return valid_topic_name(metadata.o_name) && metadata.o_size != 0U &&
           metadata.o_size_no_padding <= metadata.o_size &&
           metadata.o_queue != 0U &&
           metadata.max_instances == kMaximumInstances &&
           metadata.instances != nullptr &&
           (reinterpret_cast<uintptr_t>(metadata.instances) %
            alignof(orb_runtime_instance)) == 0U;
}

bool validate_catalog(const MetadataRange &range) noexcept
{
    for (size_t index = 0U; index < range.count; ++index) {
        const orb_metadata *const metadata = range.topics[index];
        if (metadata == nullptr || !valid_metadata(*metadata) ||
            metadata->o_id != index) {
            return false;
        }
        for (size_t prior = 0U; prior < index; ++prior) {
            const orb_metadata *const candidate = range.topics[prior];
            if (candidate == nullptr ||
                strcmp(metadata->o_name, candidate->o_name) == 0 ||
                metadata->instances == candidate->instances) {
                return false;
            }
        }
    }
    return true;
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

bool initialize(const Allocator &allocator) noexcept
{
    if (g_initialized) {
        return true;
    }
    if (allocator.allocate == nullptr || allocator.deallocate == nullptr || in_isr()) {
        return false;
    }

    MetadataRange range{};
    if (!metadata_range(range) || !validate_catalog(range)) {
        return false;
    }

    /* 每实例缓冲字节数 = object_size * queue_size。任一分配失败调用 shutdown
     * 回收此前全部实例，只有目录完整就绪后才发布 initialized 与新 epoch。 */
    g_allocator = allocator;
    for (size_t metadata_index = 0U; metadata_index < range.count;
         ++metadata_index) {
        const orb_metadata *const metadata = range.topics[metadata_index];
        const size_t bytes = static_cast<size_t>(metadata->o_size) *
                             metadata->o_queue;
        for (uint8_t index = 0U; index < metadata->max_instances; ++index) {
            auto &instance = metadata->instances[index];
            instance.buffer = static_cast<uint8_t *>(allocator.allocate(bytes, 8U));
            if (instance.buffer == nullptr) {
                shutdown();
                return false;
            }
            memset(instance.buffer, 0, bytes);
            instance.generation = 0U;
            instance.publisher_count = 0U;
            for (auto &callback : instance.callbacks) {
                callback = nullptr;
            }
        }
    }
    {
        dima::platform::CriticalGuard guard;
        /* epoch 0 保留给尚未同步的包装器；自然回绕到 0 时再跳一次。 */
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
    MetadataRange range{};
    if (metadata_range(range)) {
        for (size_t metadata_index = 0U; metadata_index < range.count;
             ++metadata_index) {
            const orb_metadata *const metadata = range.topics[metadata_index];
            if (metadata->instances == nullptr ||
                metadata->max_instances > kMaximumInstances) {
                continue;
            }
            for (uint8_t index = 0U; index < metadata->max_instances; ++index) {
                auto &instance = metadata->instances[index];
                if (instance.buffer != nullptr &&
                    g_allocator.deallocate != nullptr) {
                    g_allocator.deallocate(instance.buffer);
                }
                instance.buffer = nullptr;
                instance.generation = 0U;
                instance.publisher_count = 0U;
                for (auto &callback : instance.callbacks) {
                    callback = nullptr;
                }
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
        instance->publisher_count == 0U) {
        return false;
    }

    px4::WorkItem *callbacks[kMaximumCallbacksPerInstance]{};
    {
        dima::platform::CriticalGuard guard;
        /* 第 g 代存放槽 (g-1) % queue_size。先复制完整对象，再发布 generation；
         * callback 指针在锁内快照，实际调度移到锁外，避免 Run/取消路径重入锁。 */
        const uint64_t next_generation = instance->generation + 1U;
        const size_t slot = static_cast<size_t>((next_generation - 1U) %
                                                metadata->o_queue);
        memcpy(instance->buffer + slot * metadata->o_size, data,
               metadata->o_size);
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

namespace {

bool copy_sample(const orb_metadata *metadata, uint8_t instance_index,
                 uint64_t &generation, void *destination, bool latest) noexcept
{
    if (!g_initialized || destination == nullptr || in_isr()) {
        return false;
    }
    orb_runtime_instance *const instance = runtime_for(metadata, instance_index);
    if (instance == nullptr || instance->buffer == nullptr) {
        return false;
    }

    // 两种读取共用同一临界区：选代、复制完整样本、推进订阅代次不可分开，
    // 否则高优先级发布者可能在选槽后覆盖该槽。
    dima::platform::CriticalGuard guard;
    const uint64_t newest = instance->generation;
    if (newest == 0U) {
        generation = 0U;
        return false;
    }
    if (newest == generation) {
        return false;
    }

    uint64_t target = newest;
    if (!latest && metadata->o_queue > 1U) {
        /* 普通订阅逐代读取；首次、越界或落后于保留窗口时，
         * 从 oldest=max(1,newest-queue+1) 恢复，不能无意跳过队列事件。 */
        const uint64_t oldest = newest > metadata->o_queue
                                    ? newest - metadata->o_queue + 1U
                                    : 1U;
        if (generation == 0U || generation > newest ||
            generation < oldest - 1U) {
            target = oldest;
        } else {
            target = generation + 1U;
        }
    }
    const size_t slot = static_cast<size_t>((target - 1U) % metadata->o_queue);
    memcpy(destination, instance->buffer + slot * metadata->o_size,
           metadata->o_size);
    generation = target;
    return true;
}

} // namespace

bool orb_copy(const orb_metadata *metadata, uint8_t instance_index,
              uint64_t &generation, void *destination) noexcept
{
    return copy_sample(metadata, instance_index, generation, destination, false);
}

bool orb_copy_latest(const orb_metadata *metadata, uint8_t instance_index,
                     uint64_t &generation, void *destination) noexcept
{
    // 固定频率 Logger 主动降采样时直接取最新状态，避免追赶旧代产生突发；
    // 被采样策略跳过的消息不计为系统 dropout。
    return copy_sample(metadata, instance_index, generation, destination, true);
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
    /* publisher_count 是饱和前拒绝的引用计数；publish 要求至少一个活跃发布者，
     * 防止已析构 Publication 继续借旧 metadata 发送。 */
    if (instance->publisher_count <
        std::numeric_limits<uint16_t>::max()) {
        ++instance->publisher_count;
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
    if (instance->publisher_count > 0U) {
        --instance->publisher_count;
    }
}

int8_t orb_advertise_multi(const orb_metadata *metadata) noexcept
{
    if (!g_initialized || metadata == nullptr || in_isr()) {
        return -1;
    }
    int8_t result = -1;
    dima::platform::CriticalGuard guard;
    /* 从最低编号查找 publisher_count==0 的空实例并原子占用，返回值固定在 int8。 */
    for (uint8_t index = 0U; index < metadata->max_instances; ++index) {
        auto &instance = metadata->instances[index];
        if (instance.publisher_count == 0U) {
            instance.publisher_count = 1U;
            result = static_cast<int8_t>(index);
            break;
        }
    }
    return result;
}

bool orb_register_callback(const orb_metadata *metadata,
                           uint8_t instance_index,
                           px4::WorkItem &work_item) noexcept
{
    if (!g_initialized || in_isr()) {
        return false;
    }
    orb_runtime_instance *const instance = runtime_for(
        metadata, instance_index);
    if (instance == nullptr) {
        return false;
    }

    bool registered = false;
    dima::platform::CriticalGuard guard;
    /* 由生成策略批量注册时没有 Subscription 对象可保存 Topic；底层槽位仍按
     * metadata+instance 唯一管理，同一 WorkItem 重复注册保持幂等。 */
    for (auto *entry : instance->callbacks) {
        if (entry == &work_item) {
            registered = true;
            break;
        }
    }
    if (!registered) {
        for (auto &entry : instance->callbacks) {
            if (entry == nullptr) {
                entry = &work_item;
                registered = true;
                break;
            }
        }
    }
    return registered;
}

void orb_unregister_callback(const orb_metadata *metadata,
                             uint8_t instance_index,
                             px4::WorkItem &work_item) noexcept
{
    if (!g_initialized || in_isr()) {
        return;
    }
    orb_runtime_instance *const instance = runtime_for(
        metadata, instance_index);
    if (instance == nullptr) {
        return;
    }
    dima::platform::CriticalGuard guard;
    for (auto &entry : instance->callbacks) {
        if (entry == &work_item) {
            entry = nullptr;
        }
    }
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
    const bool registered = orb_register_callback(
        metadata_, instance_, work_item);
    if (registered) {
        callback_ = &work_item;
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
    orb_unregister_callback(metadata_, instance_, *callback_);
    callback_ = nullptr;
}

} // namespace uORB

const char *orb_get_c_type(unsigned char short_type)
{
    /* 该映射直接同步 PX4 v1.17 uORB.cpp，并与
     * px_generate_uorb_topic_helper.py::type_map_short 形成单一 wire 合同。 */
    switch (short_type) {
    case 0x82: return "int8_t";
    case 0x83: return "int16_t";
    case 0x84: return "int32_t";
    case 0x85: return "int64_t";
    case 0x86: return "uint8_t";
    case 0x87: return "uint16_t";
    case 0x88: return "uint32_t";
    case 0x89: return "uint64_t";
    case 0x8a: return "float";
    case 0x8b: return "double";
    case 0x8c: return "bool";
    case 0x8d: return "char";
    default: return nullptr;
    }
}

void orb_print_message_internal(const orb_metadata *meta, const void *data,
                                bool print_topic_name)
{
    // 固件不实现 PX4 shell 的字段格式化器；保留官方生成符号并只输出 Topic 名称，
    // 避免为调试打印重新解释消息字段描述。
    (void)data;
    if (print_topic_name && meta != nullptr && meta->o_name != nullptr) {
        std::printf("%s\n", meta->o_name);
    }
}



// 普通运行期实现从对应头文件移出；保持原状态、错误分支和计算顺序。

namespace uORB {

Subscription::Subscription(const orb_metadata *metadata,
                          uint8_t instance) noexcept
: metadata_(metadata), instance_(instance)
{
}

SubscriptionCallbackWorkItem::SubscriptionCallbackWorkItem(const orb_metadata *metadata,
                                 px4::WorkItem &work_item,
                                 uint8_t instance) noexcept
: Subscription(metadata, instance), work_item_(work_item)
{
}

bool SubscriptionCallbackWorkItem::registerCallback() noexcept
{
    return Subscription::registerCallback(work_item_);
}

} // namespace uORB
