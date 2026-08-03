/****************************************************************************
 *
 *   Copyright (c) 2012-2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/
/** Adapted from src/lib/parameters/parameters.cpp @ d6f12ad1. */
#include "param.h"

#include "ConstLayer.h"
#include "DynamicSparseLayer.h"
#include "containers/AtomicBitset.hpp"
#include "freertos/dima_platform.hpp"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <new>
#include <type_traits>

namespace {
constexpr size_t kCount = px4::param_info_count;
ConstLayer g_firmware_defaults;
using LayerStorage = typename std::aligned_storage<sizeof(DynamicSparseLayer),
                                                   alignof(DynamicSparseLayer)>::type;
LayerStorage g_runtime_storage{};
LayerStorage g_user_storage{};
DynamicSparseLayer *g_runtime_defaults{};
DynamicSparseLayer *g_user_config{};
px4::AtomicBitset<kCount> g_active;
px4::AtomicBitset<kCount> g_unsaved;
uint32_t g_get_count{};
uint32_t g_set_count{};
uint32_t g_find_count{};
uint32_t g_export_count{};
uint32_t g_instance{};
uint32_t g_default_generation{};
unsigned g_transaction_depth{};
bool g_notification_pending{};
bool g_runtime_constructed{};
bool g_user_constructed{};
bool g_initialized{};
param_notify_callback_t g_notify{};
void *g_notify_context{};
param_lock_callback_t g_lock{};
param_lock_callback_t g_unlock{};
void *g_lock_context{};
const param_storage_backend_s *g_storage{};
void *g_storage_context{};

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
    return !dima::platform::in_realtime_context();
}

param_value_u read_value(param_t param, const void *value) noexcept
{
    param_value_u result{};
    if (param_info[param].type == PARAM_TYPE_FLOAT) {
        result.f = *static_cast<const float *>(value);
    } else {
        result.i = *static_cast<const int32_t *>(value);
    }
    return result;
}

void write_value(param_t param, param_value_u value, void *destination) noexcept
{
    if (param_info[param].type == PARAM_TYPE_FLOAT) {
        *static_cast<float *>(destination) = value.f;
    } else {
        *static_cast<int32_t *>(destination) = value.i;
    }
}

bool equal_value(param_t param, param_value_u lhs, param_value_u rhs) noexcept
{
    return param_info[param].type == PARAM_TYPE_FLOAT ? lhs.f == rhs.f : lhs.i == rhs.i;
}

parameter_update_s update_snapshot() noexcept
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

int set_internal(param_t param, const void *value, bool notify, bool mark_unsaved) noexcept
{
    if (!g_initialized || !valid(param) || value == nullptr) {
        return -EINVAL;
    }

    const param_value_u next = read_value(param, value);
    const param_value_u current = g_user_config->get(param);
    if (equal_value(param, current, next)) {
        return 0;
    }

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

int enumerate_changed(param_storage_visitor_t visitor, void *visitor_context,
                      void *) noexcept
{
    if (!task_read_allowed() || visitor == nullptr) {
        return -EPERM;
    }

    px4::AtomicTransaction transaction;
    if (!g_initialized) {
        return -EINVAL;
    }

    for (param_t param = 0U; param < kCount; ++param) {
        if (!g_user_config->contains(param)) {
            continue;
        }
        const param_value_u value = g_user_config->get(param);
        const void *source = param_info[param].type == PARAM_TYPE_FLOAT
                                 ? static_cast<const void *>(&value.f)
                                 : static_cast<const void *>(&value.i);
        const int result = visitor(param_info[param].name, param_info[param].type,
                                   source, visitor_context);
        if (result != 0) {
            return result;
        }
    }
    return 0;
}

int load_value_to_layer(const char *name, param_type_t type, const void *value,
                        void *context) noexcept
{
    if (name == nullptr || value == nullptr || context == nullptr) {
        return -EINVAL;
    }

    const param_t param = param_find_no_notification(name);
    // 固件升级删除参数时忽略旧键；BSON 和记录 CRC 仍负责判断结构损坏。
    if (!valid(param)) {
        return 0;
    }
    if (param_info[param].type != type) {
        return -EINVAL;
    }

    auto &layer = *static_cast<DynamicSparseLayer *>(context);
    const param_value_u next = read_value(param, value);
    const param_value_u current_default = g_runtime_defaults->get(param);
    if (equal_value(param, next, current_default)) {
        return 0;
    }
    return layer.store(param, next) ? 0 : -ENOMEM;
}
} // namespace

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
        if (std::strcmp(name, param_info[param].name) == 0) {
            return param;
        }
    }
    return PARAM_INVALID;
}

param_t param_find(const char *name)
{
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
    if (valid(param)) { g_active.set(param); }
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

const char *param_name(param_t param) { return valid(param) ? param_info[param].name : nullptr; }
param_type_t param_type(param_t param) { return valid(param) ? param_info[param].type : PARAM_TYPE_UNKNOWN; }
size_t param_size(param_t param)
{
    return param_type(param) == PARAM_TYPE_FLOAT ? sizeof(float)
           : param_type(param) == PARAM_TYPE_INT32 ? sizeof(int32_t) : 0U;
}

bool param_is_volatile(param_t) { return false; }

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
    for (param_t param = 0U; param < kCount; ++param) {
        if ((!only_changed || g_user_config->contains(param))
            && (!only_used || g_active[param])) {
            callback(context, param);
        }
    }
}

int param_save_default(bool)
{
    if (!service_write_allowed()) { return -EPERM; }

    const param_storage_backend_s *backend{};
    void *backend_context{};
    uint32_t set_count_snapshot{};
    {
        px4::AtomicTransaction transaction;
        if (!g_storage || !g_storage->save) { return -ENOSYS; }
        backend = g_storage;
        backend_context = g_storage_context;
        set_count_snapshot = g_set_count;
    }

    const int result = backend->save(enumerate_changed, nullptr, backend_context);
    if (result == 0) {
        px4::AtomicTransaction transaction;
        if (g_set_count == set_count_snapshot) {
            g_unsaved.reset();
        }
        ++g_export_count;
    }
    return result;
}

int param_load_default(void)
{
    if (!service_write_allowed()) { return -EPERM; }

    const param_storage_backend_s *backend{};
    void *backend_context{};
    uint32_t set_count_snapshot{};
    uint32_t default_generation_snapshot{};
    {
        px4::AtomicTransaction transaction;
        if (!g_initialized || !g_storage || !g_storage->load) { return -ENOSYS; }
        backend = g_storage;
        backend_context = g_storage_context;
        set_count_snapshot = g_set_count;
        default_generation_snapshot = g_default_generation;
    }

    DynamicSparseLayer staged(g_runtime_defaults);
    if (!staged.valid()) { return -ENOMEM; }
    const int result = backend->load(load_value_to_layer, &staged, backend_context);
    if (result != 0) { return result; }

    {
        px4::AtomicTransaction transaction;
        // 运行期加载期间若已有新设置，拒绝覆盖，调用方可稍后重试。
        if (g_set_count != set_count_snapshot
            || g_default_generation != default_generation_snapshot) {
            return -EAGAIN;
        }
        g_user_config->swapContents(staged);
        g_unsaved.reset();
        request_notification();
    }
    return 0;
}

void param_print_status(void)
{
    if (!task_read_allowed()) { return; }
    param_storage_status_s status{};
    const int result = param_storage_get_status(&status);
    std::printf("param: count=%u used=%u storage=%d seq=%lu free=%lu\n",
                param_count(), param_count_used(), result,
                static_cast<unsigned long>(status.sequence),
                static_cast<unsigned long>(status.free_bytes));
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

int param_register_storage_backend(const param_storage_backend_s *backend,
                                   void *context) noexcept
{
    if (!task_read_allowed()) { return -EPERM; }
    px4::AtomicTransaction transaction;
    g_storage = backend;
    g_storage_context = context;
    return backend ? 0 : -EINVAL;
}

int param_storage_get_status(param_storage_status_s *status) noexcept
{
    if (!service_write_allowed() || status == nullptr) { return -EPERM; }
    const param_storage_backend_s *backend{};
    void *backend_context{};
    {
        px4::AtomicTransaction transaction;
        std::memset(status, 0, sizeof(*status));
        backend = g_storage;
        backend_context = g_storage_context;
    }
    return backend && backend->status ? backend->status(status, backend_context) : -ENOSYS;
}

int param_storage_erase(void) noexcept
{
    if (!service_write_allowed()) { return -EPERM; }
    const param_storage_backend_s *backend{};
    void *backend_context{};
    {
        px4::AtomicTransaction transaction;
        backend = g_storage;
        backend_context = g_storage_context;
    }
    return backend && backend->erase ? backend->erase(backend_context) : -ENOSYS;
}
} // extern "C"
