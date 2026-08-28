/****************************************************************************
 *
 *   Copyright (c) 2012-2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include "param.h"

#include <cstddef>
#include <cstdint>

#include "DynamicSparseLayer.h"
#include "containers/AtomicBitset.hpp"

namespace dima::parameters::internal {

// 参数 Core 容量直接取官方生成值表长度，不维护第二份目录计数。
constexpr size_t kCount =
    sizeof(dima::parameter_catalog::parameters) /
    sizeof(dima::parameter_catalog::parameters[0]);

// Core 与持久化适配仅通过此私有边界共享状态，模块层不得直接引用。
extern DynamicSparseLayer *g_runtime_defaults;
extern DynamicSparseLayer *g_user_config;
extern px4::AtomicBitset<kCount> g_unsaved;
extern uint32_t g_set_count;
extern uint32_t g_export_count;
extern uint32_t g_default_generation;
extern bool g_initialized;
extern const param_storage_backend_s *g_storage;
extern void *g_storage_context;

bool valid(param_t param) noexcept;
bool task_read_allowed() noexcept;
bool service_write_allowed() noexcept;
param_value_u read_value(param_t param, const void *value) noexcept;
bool equal_value(param_t param, param_value_u lhs, param_value_u rhs) noexcept;
void request_notification() noexcept;

} // namespace dima::parameters::internal
