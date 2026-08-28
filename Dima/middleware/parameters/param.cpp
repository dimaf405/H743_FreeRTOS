/****************************************************************************
 *
 *   Copyright (c) 2012-2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/
/** Adapted from src/lib/parameters/parameters.cpp @ d6f12ad1. */
#include "param.h"

#include "ConstLayer.h"
#include "param_internal.hpp"
#include "api/Execution.hpp"

/* parameter_update_s 由锁定 PX4 uORB 生成器从 ParameterUpdate.msg 生成；
 * 参数核心只消费该权威结构，不在 C ABI 头中重复定义布局。 */
#include <uORB/topics/parameter_update.h>

#include <cerrno>
#include <cstring>
#include <new>
#include <type_traits>

namespace dima::parameters::internal {
static ConstLayer g_firmware_defaults;
using LayerStorage = typename std::aligned_storage<sizeof(DynamicSparseLayer),
                                                   alignof(DynamicSparseLayer)>::type;
static LayerStorage g_runtime_storage{};
static LayerStorage g_user_storage{};
DynamicSparseLayer *g_runtime_defaults{};
DynamicSparseLayer *g_user_config{};
static px4::AtomicBitset<kCount> g_active;
px4::AtomicBitset<kCount> g_unsaved;
static uint32_t g_get_count{};
uint32_t g_set_count{};
static uint32_t g_find_count{};
uint32_t g_export_count{};
static uint32_t g_instance{};
uint32_t g_default_generation{};
static unsigned g_transaction_depth{};
static bool g_notification_pending{};
static bool g_runtime_constructed{};
static bool g_user_constructed{};
bool g_initialized{};
static param_notify_callback_t g_notify{};
static void *g_notify_context{};
static param_lock_callback_t g_lock{};
static param_lock_callback_t g_unlock{};
static void *g_lock_context{};
const param_storage_backend_s *g_storage{};
void *g_storage_context{};

/* 参数值分层：ConstLayer 固件默认值；runtime_defaults 保存运行期自定义默认；
 * user_config 保存用户覆盖。g_unsaved 与 user_config 分离：值可等于默认而仍需
 * 记录一次 reset 的持久化意图，直到成功保存后由存储层清理。 */

bool valid(param_t param) noexcept
{
    return static_cast<size_t>(param) < kCount;
}

bool task_read_allowed() noexcept
{
    return !dima::platform::in_interrupt_context();
}

bool service_write_allowed() noexcept
{
    /* 写参数可能增长稀疏层并触发通知，ISR 与 realtime 任务一律禁止。 */
    return !dima::platform::in_realtime_context();
}

param_value_u read_value(param_t param, const void *value) noexcept
{
    param_value_u result{};
    if (dima::parameter_catalog::parameters_type[param] == PARAM_TYPE_FLOAT) {
        result.f = *static_cast<const float *>(value);
    } else {
        result.i = *static_cast<const int32_t *>(value);
    }
    return result;
}

static void write_value(param_t param, param_value_u value, void *destination) noexcept
{
    if (dima::parameter_catalog::parameters_type[param] == PARAM_TYPE_FLOAT) {
        *static_cast<float *>(destination) = value.f;
    } else {
        *static_cast<int32_t *>(destination) = value.i;
    }
}

bool equal_value(param_t param, param_value_u lhs, param_value_u rhs) noexcept
{
    return dima::parameter_catalog::parameters_type[param] == PARAM_TYPE_FLOAT
               ? lhs.f == rhs.f
               : lhs.i == rhs.i;
}

static parameter_update_s update_snapshot() noexcept
{
    parameter_update_s update{};
    update.instance = g_instance++;
    update.get_count = g_get_count;
    update.set_count = g_set_count;
    update.find_count = g_find_count;
    update.export_count = g_export_count;
    update.active = static_cast<uint16_t>(g_active.count());
    update.changed = g_user_config ? static_cast<uint16_t>(g_user_config->size()) : 0U;
    update.custom_default = g_runtime_defaults ? static_cast<uint16_t>(g_runtime_defaults->size()) : 0U;
    return update;
}

void request_notification() noexcept
{
    g_notification_pending = true;
}

static int set_internal(param_t param, const void *value, bool notify,
                        bool mark_unsaved) noexcept
{
    if (!g_initialized || !valid(param) || value == nullptr) {
        return -EINVAL;
    }

    const param_value_u next = read_value(param, value);
    const param_value_u current = g_user_config->get(param);
    if (equal_value(param, current, next)) {
        return 0;
    }

    /* 新值等于当前 runtime default 时删除 user override，否则存入稀疏层；这使
     * param_value_is_default 可由“是否 contains”精确判断，而非重复保存默认值。 */
    const param_value_u current_default = g_runtime_defaults->get(param);
    const bool stored = equal_value(param, next, current_default)
                            ? (g_user_config->reset(param), true)
                            : g_user_config->store(param, next);
    if (!stored) {
        return -ENOMEM;
    }

    g_unsaved.set(param, mark_unsaved);
    ++g_set_count;
    if (notify) {
        request_notification();
    }
    return 0;
}

} // namespace dima::parameters::internal

using namespace dima::parameters::internal;

namespace dima::parameters::detail {
void transaction_lock() noexcept
{
    if (g_lock != nullptr) {
        g_lock(g_lock_context);
    }
}

void transaction_unlock() noexcept
{
    if (g_unlock != nullptr) {
        g_unlock(g_lock_context);
    }
}

void transaction_begin() noexcept
{
    transaction_lock();
    ++g_transaction_depth;
}

void transaction_end() noexcept
{
    if (g_transaction_depth == 0U) {
        return;
    }

    /* 嵌套事务只在最外层退出时合并发布一次 parameter_update；snapshot 记录操作
     * 计数与 active/changed/default 数量，避免逐参数通知风暴。 */
    const bool emit = (--g_transaction_depth == 0U) && g_notification_pending;
    if (emit) {
        g_notification_pending = false;
        const parameter_update_s update = update_snapshot();
        const param_notify_callback_t callback = g_notify;
        void *const context = g_notify_context;
        // 注册锁是递归锁。锁内按 sequence 串行回调，避免多任务倒序发布。
        if (callback != nullptr) {
            callback(&update, context);
        }
    }
    transaction_unlock();
}
} // namespace dima::parameters::detail

extern "C" {
void param_init(void)
{
    if (!task_read_allowed()) {
        return;
    }

    px4::AtomicTransaction transaction;
    if (g_initialized) {
        return;
    }
    // ready 与对象构造状态分离，保证部分分配失败后的重试不会覆盖并泄漏旧对象。
    /* placement-new 使用静态 storage，重新初始化前先按构造标记逆序析构旧层；
     * 任一层分配失败时 ready=false，但不会覆盖或泄漏上代对象。 */
    if (g_user_constructed) {
        g_user_config->~DynamicSparseLayer();
        g_user_constructed = false;
    }
    if (g_runtime_constructed) {
        g_runtime_defaults->~DynamicSparseLayer();
        g_runtime_constructed = false;
    }

    g_runtime_defaults = new (&g_runtime_storage) DynamicSparseLayer(&g_firmware_defaults);
    g_runtime_constructed = true;
    g_user_config = new (&g_user_storage) DynamicSparseLayer(g_runtime_defaults);
    g_user_constructed = true;
    g_initialized = g_runtime_defaults->valid() && g_user_config->valid();
    g_active.reset();
    g_unsaved.reset();
    g_get_count = 0U;
    g_set_count = 0U;
    g_find_count = 0U;
    g_export_count = 0U;
    g_instance = 0U;
    g_default_generation = 0U;
    g_notification_pending = false;
}

bool param_shutdown(void) noexcept
{
    if (!task_read_allowed()) {
        return false;
    }

    const param_lock_callback_t lock = g_lock;
    const param_lock_callback_t unlock = g_unlock;
    void *const lock_context = g_lock_context;
    if ((lock == nullptr) != (unlock == nullptr)) {
        return false;
    }
    if (lock != nullptr) {
        lock(lock_context);
    }

    // Holding the registered recursive mutex excludes all other parameter
    // transactions. A non-zero depth here would mean the caller attempted to
    // tear the core down from inside a parameter callback.
    if (g_transaction_depth != 0U) {
        if (unlock != nullptr) {
            unlock(lock_context);
        }
        return false;
    }

    g_initialized = false;
    g_notify = nullptr;
    g_notify_context = nullptr;
    g_storage = nullptr;
    g_storage_context = nullptr;
    g_notification_pending = false;

    if (g_user_constructed) {
        g_user_config->~DynamicSparseLayer();
        g_user_constructed = false;
    }
    if (g_runtime_constructed) {
        g_runtime_defaults->~DynamicSparseLayer();
        g_runtime_constructed = false;
    }
    g_user_config = nullptr;
    g_runtime_defaults = nullptr;
    g_active.reset();
    g_unsaved.reset();
    g_get_count = 0U;
    g_set_count = 0U;
    g_find_count = 0U;
    g_export_count = 0U;
    g_instance = 0U;
    g_default_generation = 0U;

    if (unlock != nullptr) {
        unlock(lock_context);
    }
    g_lock = nullptr;
    g_unlock = nullptr;
    g_lock_context = nullptr;
    return true;
}

bool param_is_ready(void)
{
    if (!task_read_allowed()) {
        return false;
    }
    px4::AtomicTransaction transaction;
    return g_initialized;
}

param_t param_find_no_notification(const char *name)
{
    if (!task_read_allowed() || name == nullptr) {
        return PARAM_INVALID;
    }
    px4::AtomicTransaction transaction;
    ++g_find_count;
    for (param_t param = 0U; param < kCount; ++param) {
        if (std::strcmp(name, dima::parameter_catalog::parameters[param].name) == 0) {
            return param;
        }
    }
    return PARAM_INVALID;
}

param_t param_find(const char *name)
{
    /* find 会把参数标记为 used，供 MAVLink used-index/元数据裁剪；
     * find_no_notification 只查句柄，不改变 active 集合。 */
    const param_t param = param_find_no_notification(name);
    if (valid(param)) {
        param_set_used(param);
    }
    return param;
}

unsigned param_count(void) { return static_cast<unsigned>(kCount); }

unsigned param_count_used(void)
{
    if (!task_read_allowed()) { return 0U; }
    px4::AtomicTransaction transaction;
    return static_cast<unsigned>(g_active.count());
}

bool param_used(param_t param)
{
    if (!task_read_allowed()) { return false; }
    px4::AtomicTransaction transaction;
    return valid(param) && g_active[param];
}

void param_set_used(param_t param)
{
    if (!task_read_allowed()) { return; }
    px4::AtomicTransaction transaction;
    if (g_initialized && valid(param)) { g_active.set(param); }
}

param_t param_for_index(unsigned index)
{
    return index < kCount ? static_cast<param_t>(index) : PARAM_INVALID;
}

param_t param_for_used_index(unsigned index)
{
    if (!task_read_allowed()) { return PARAM_INVALID; }
    px4::AtomicTransaction transaction;
    for (param_t param = 0U; param < kCount; ++param) {
        if (g_active[param] && index-- == 0U) { return param; }
    }
    return PARAM_INVALID;
}

int param_get_index(param_t param) { return valid(param) ? static_cast<int>(param) : -1; }

int param_get_used_index(param_t param)
{
    if (!task_read_allowed()) { return -1; }
    px4::AtomicTransaction transaction;
    if (!valid(param) || !g_active[param]) { return -1; }
    int index = 0;
    for (param_t current = 0U; current < param; ++current) {
        index += g_active[current] ? 1 : 0;
    }
    return index;
}

const char *param_name(param_t param)
{
    return valid(param) ? dima::parameter_catalog::parameters[param].name : nullptr;
}

param_type_t param_type(param_t param)
{
    return valid(param) ? dima::parameter_catalog::parameters_type[param] : PARAM_TYPE_UNKNOWN;
}
size_t param_size(param_t param)
{
    return param_type(param) == PARAM_TYPE_FLOAT ? sizeof(float)
           : param_type(param) == PARAM_TYPE_INT32 ? sizeof(int32_t) : 0U;
}

bool param_is_volatile(param_t param)
{
    if (!valid(param)) {
        return false;
    }

    // volatile 集合由上游原始头从 YAML 语义生成，持久化层不得复制名称名单。
    for (const dima::params candidate : dima::parameter_catalog::parameters_volatile) {
        if (param_handle(candidate) == param) {
            return true;
        }
    }
    return false;
}

bool param_value_is_default(param_t param)
{
    if (!task_read_allowed()) { return false; }
    px4::AtomicTransaction transaction;
    return g_initialized && valid(param) && !g_user_config->contains(param);
}

bool param_value_unsaved(param_t param)
{
    if (!task_read_allowed()) { return false; }
    px4::AtomicTransaction transaction;
    return g_initialized && valid(param) && g_unsaved[param];
}

int param_get(param_t param, void *value)
{
    if (!task_read_allowed()) { return -EPERM; }
    px4::AtomicTransaction transaction;
    if (!g_initialized || !valid(param) || value == nullptr) { return -EINVAL; }
    ++g_get_count;
    write_value(param, g_user_config->get(param), value);
    return 0;
}

int param_get_default_value(param_t param, void *value)
{
    if (!task_read_allowed()) { return -EPERM; }
    px4::AtomicTransaction transaction;
    if (!g_initialized || !valid(param) || value == nullptr) { return -EINVAL; }
    write_value(param, g_runtime_defaults->get(param), value);
    return 0;
}

int param_get_system_default_value(param_t param, void *value)
{
    if (!task_read_allowed()) { return -EPERM; }
    if (!valid(param) || value == nullptr) { return -EINVAL; }
    write_value(param, g_firmware_defaults.get(param), value);
    return 0;
}

int param_set_default_value(param_t param, const void *value)
{
    if (!service_write_allowed()) { return -EPERM; }
    px4::AtomicTransaction transaction;
    if (!g_initialized || !valid(param) || value == nullptr) { return -EINVAL; }

    const param_value_u next = read_value(param, value);
    const param_value_u previous = g_runtime_defaults->get(param);
    if (equal_value(param, previous, next)) { return 0; }

    /* 自定义默认等于固件默认时删除 runtime override，否则存入 runtime 稀疏层；
     * 仅当该参数已 active 时请求通知，未使用参数不制造无意义更新。 */
    const param_value_u firmware = g_firmware_defaults.get(param);
    const bool stored = equal_value(param, firmware, next)
                            ? (g_runtime_defaults->reset(param), true)
                            : g_runtime_defaults->store(param, next);
    if (!stored) { return -ENOMEM; }
    ++g_default_generation;
    if (g_active[param]) { request_notification(); }
    return 0;
}

int param_set(param_t param, const void *value)
{
    if (!service_write_allowed()) { return -EPERM; }
    px4::AtomicTransaction transaction;
    return set_internal(param, value, true, true);
}

int param_set_no_notification(param_t param, const void *value)
{
    if (!service_write_allowed()) { return -EPERM; }
    px4::AtomicTransaction transaction;
    return set_internal(param, value, false, true);
}

int param_reset(param_t param)
{
    if (!service_write_allowed()) { return -EPERM; }
    px4::AtomicTransaction transaction;
    if (!g_initialized || !valid(param)) { return -EINVAL; }
    if (!g_user_config->contains(param)) { return 0; }
    g_user_config->reset(param);
    g_unsaved.set(param);
    ++g_set_count;
    request_notification();
    return 0;
}

int param_reset_no_notification(param_t param)
{
    if (!service_write_allowed()) { return -EPERM; }
    px4::AtomicTransaction transaction;
    if (!g_initialized || !valid(param)) { return -EINVAL; }
    if (g_user_config->contains(param)) {
        g_user_config->reset(param);
        g_unsaved.set(param);
        ++g_set_count;
    }
    return 0;
}

void param_reset_all(void)
{
    if (!service_write_allowed()) { return; }
    px4::AtomicTransaction transaction;
    if (!g_initialized) { return; }
    bool changed = false;
    for (param_t param = 0U; param < kCount; ++param) {
        if (g_user_config->contains(param)) {
            g_user_config->reset(param);
            g_unsaved.set(param);
            changed = true;
        }
    }
    if (changed) {
        ++g_set_count;
        request_notification();
    }
}

void param_foreach(param_foreach_func_t callback, void *context,
                   bool only_changed, bool only_used)
{
    if (!task_read_allowed() || callback == nullptr) { return; }
    px4::AtomicTransaction transaction;
    if (!g_initialized) { return; }
    for (param_t param = 0U; param < kCount; ++param) {
        if ((!only_changed || g_user_config->contains(param))
            && (!only_used || g_active[param])) {
            callback(context, param);
        }
    }
}

void param_notify_changes(void) noexcept
{
    if (!task_read_allowed()) { return; }
    px4::AtomicTransaction transaction;
    request_notification();
}

void param_register_notify_callback(param_notify_callback_t callback,
                                    void *context) noexcept
{
    if (!task_read_allowed()) { return; }
    px4::AtomicTransaction transaction;
    g_notify = callback;
    g_notify_context = context;
}

void param_register_lock_callbacks(param_lock_callback_t lock,
                                   param_lock_callback_t unlock,
                                   void *context) noexcept
{
    if (dima::platform::in_interrupt_context()) { return; }
    g_lock = lock;
    g_unlock = unlock;
    g_lock_context = context;
}

} // extern "C"
