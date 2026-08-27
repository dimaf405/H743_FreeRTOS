/****************************************************************************
 *
 *   Copyright (c) 2012-2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/
/** Parameter persistence adapter split from PX4 parameters.cpp @ d6f12ad1. */

#include "param_internal.hpp"

#include "atomic_transaction.h"

#include <cerrno>
#include <cstdio>
#include <cstring>

using namespace dima::parameters::internal;

namespace {

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
        // PX4 YAML 标记为 volatile 的参数只存在于 RAM，不进入任何持久化后端。
        if (!g_user_config->contains(param) || param_is_volatile(param)) {
            continue;
        }
        const param_value_u value = g_user_config->get(param);
        const void *source = px4::parameters_type[param] == PARAM_TYPE_FLOAT
                                 ? static_cast<const void *>(&value.f)
                                 : static_cast<const void *>(&value.i);
        const int result = visitor(px4::parameters[param].name,
                                   px4::parameters_type[param],
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
    if (!valid(param)) {
        return -ENOENT;
    }
    if (px4::parameters_type[param] != type) {
        return -EINVAL;
    }
    if (param_is_volatile(param)) {
        return 0;
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

extern "C" {

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

int param_register_storage_backend(const param_storage_backend_s *backend,
                                   void *context) noexcept
{
    if (!task_read_allowed()) { return -EPERM; }
    px4::AtomicTransaction transaction;
    g_storage = backend;
    g_storage_context = context;
    return 0;
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

} // extern "C"
