/****************************************************************************
 *
 *   Copyright (c) 2012-2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be used to
 *    endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS".
 ****************************************************************************/

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
#define PARAM_NOEXCEPT noexcept
extern "C" {
#else
#define PARAM_NOEXCEPT
#endif

#define PARAM_TYPE_UNKNOWN 0
#define PARAM_TYPE_INT32 1
#define PARAM_TYPE_FLOAT 2

typedef uint8_t param_type_t;
typedef uint16_t param_t;
#define PARAM_INVALID ((param_t)UINT16_MAX)

typedef union param_value_u {
    int32_t i;
    float f;
} param_value_u;

typedef struct param_info_s {
    const char *name;
    param_type_t type;
    param_value_u default_value;
} param_info_s;

typedef struct parameter_update_s {
    uint64_t timestamp;
    uint32_t instance;
    uint32_t get_count;
    uint32_t set_count;
    uint32_t find_count;
    uint32_t export_count;
    uint16_t active;
    uint16_t changed;
    uint16_t custom_default;
} parameter_update_s;

typedef void (*param_notify_callback_t)(const parameter_update_s *update, void *context);
typedef void (*param_lock_callback_t)(void *context);
typedef int (*param_storage_visitor_t)(const char *name, param_type_t type, const void *value,
                                      void *context);
typedef int (*param_storage_enumerator_t)(param_storage_visitor_t visitor, void *visitor_context,
                                         void *context);
typedef void (*param_foreach_func_t)(void *context, param_t param);

typedef struct param_storage_status_s {
    uint32_t sequence;
    uint32_t used_bytes;
    uint32_t free_bytes;
    uint32_t crc_failures;
    uint32_t write_failures;
    uint32_t enospc_failures;
    uint64_t last_save_timestamp;
    bool autosave_enabled;
} param_storage_status_s;

typedef struct param_storage_backend_s {
    int (*load)(param_storage_visitor_t visitor, void *visitor_context,
                void *backend_context);
    int (*save)(param_storage_enumerator_t enumerate, void *enumerate_context,
                void *backend_context);
    int (*status)(param_storage_status_s *status, void *backend_context);
} param_storage_backend_s;

extern const param_info_s param_info[];
extern const uint16_t param_info_count;

void param_init(void);
bool param_shutdown(void) PARAM_NOEXCEPT;
bool param_is_ready(void);
param_t param_find(const char *name);
param_t param_find_no_notification(const char *name);
unsigned param_count(void);
unsigned param_count_used(void);
bool param_used(param_t param);
param_t param_for_index(unsigned index);
param_t param_for_used_index(unsigned index);
int param_get_index(param_t param);
int param_get_used_index(param_t param);
const char *param_name(param_t param);
param_type_t param_type(param_t param);
size_t param_size(param_t param);
bool param_is_volatile(param_t param);
bool param_value_is_default(param_t param);
bool param_value_unsaved(param_t param);
int param_get(param_t param, void *value);
int param_get_default_value(param_t param, void *value);
int param_get_system_default_value(param_t param, void *value);
int param_set_default_value(param_t param, const void *value);
int param_set(param_t param, const void *value);
int param_set_no_notification(param_t param, const void *value);
void param_set_used(param_t param);
int param_reset(param_t param);
int param_reset_no_notification(param_t param);
void param_reset_all(void);
void param_foreach(param_foreach_func_t callback, void *context, bool only_changed, bool only_used);
int param_save_default(bool blocking);
int param_load_default(void);
void param_print_status(void);
void param_notify_changes(void) PARAM_NOEXCEPT;

void param_register_notify_callback(param_notify_callback_t callback, void *context) PARAM_NOEXCEPT;
void param_register_lock_callbacks(param_lock_callback_t lock, param_lock_callback_t unlock,
                                   void *context) PARAM_NOEXCEPT;
int param_register_storage_backend(const param_storage_backend_s *backend,
                                   void *backend_context) PARAM_NOEXCEPT;
int param_storage_get_status(param_storage_status_s *status) PARAM_NOEXCEPT;

#ifdef __cplusplus
}

#include "param_macros.h"
#include <parameters/px4_parameters.hpp>

inline param_t param_handle(px4::params parameter) noexcept
{
    return static_cast<param_t>(parameter);
}

#include "atomic_transaction.h"

namespace do_not_explicitly_use_this_namespace
{
template<typename T, px4::params p> struct ParamTraits;
template<px4::params p> struct ParamTraits<float, p> {
    static constexpr param_type_t type = PARAM_TYPE_FLOAT;
    static int get(param_t h, float &v) { return param_get(h, &v); }
    static int set(param_t h, const float &v, bool notify) { return notify ? param_set(h, &v) : param_set_no_notification(h, &v); }
};
template<px4::params p> struct ParamTraits<int32_t, p> {
    static constexpr param_type_t type = PARAM_TYPE_INT32;
    static int get(param_t h, int32_t &v) { return param_get(h, &v); }
    static int set(param_t h, const int32_t &v, bool notify) { return notify ? param_set(h, &v) : param_set_no_notification(h, &v); }
};
template<px4::params p> struct ParamTraits<bool, p> {
    static constexpr param_type_t type = PARAM_TYPE_INT32;
    static int get(param_t h, bool &v) { int32_t raw{}; const int ret = param_get(h, &raw); v = raw != 0; return ret; }
    static int set(param_t h, const bool &v, bool notify) { const int32_t raw = v ? 1 : 0; return notify ? param_set(h, &raw) : param_set_no_notification(h, &raw); }
};

template<typename T, px4::params p>
class Param
{
public:
    constexpr Param() noexcept
    {
        static_assert(px4::parameters_type[static_cast<unsigned>(p)] == ParamTraits<T, p>::type,
                      "parameter type mismatch");
    }
    bool bind()
    {
        T value{};
        if (ParamTraits<T, p>::get(handle(), value) != 0) {
            invalidate();
            return false;
        }
        param_set_used(handle());
        _value = value;
        _bound = true;
        return true;
    }
    void invalidate() noexcept { _value = T{}; _bound = false; }
    bool bound() const noexcept { return _bound; }
    T get() const { return _value; }
    const T &reference() const { return _value; }
    void set(T value) { _value = value; }
    bool update()
    {
        if (!_bound) {
            _value = T{};
            return false;
        }
        T value{};
        if (ParamTraits<T, p>::get(handle(), value) != 0) {
            invalidate();
            return false;
        }
        _value = value;
        return true;
    }
    bool commit() const { return _bound && ParamTraits<T, p>::set(handle(), _value, true) == 0; }
    bool commit_no_notification() const { return _bound && ParamTraits<T, p>::set(handle(), _value, false) == 0; }
    bool commit_no_notification(T value) { if (value == _value) { return false; } _value = value; return commit_no_notification(); }
    void reset() { if (_bound) { param_reset_no_notification(handle()); (void)update(); } }
    static constexpr param_t handle() { return static_cast<param_t>(p); }
private:
    T _value{};
    bool _bound{false};
};

template<px4::params p> using ParamFloat = Param<float, p>;
template<px4::params p> using ParamInt = Param<int32_t, p>;
template<px4::params p> using ParamBool = Param<bool, p>;
}

namespace px4
{
template<typename T, params p> using Param = do_not_explicitly_use_this_namespace::Param<T, p>;
template<params p> using ParamFloat = do_not_explicitly_use_this_namespace::ParamFloat<p>;
template<params p> using ParamInt = do_not_explicitly_use_this_namespace::ParamInt<p>;
template<params p> using ParamBool = do_not_explicitly_use_this_namespace::ParamBool<p>;
}

#define _DEFINE_SINGLE_PARAMETER(x) do_not_explicitly_use_this_namespace::PAIR(x);
#define _CALL_UPDATE(x) STRIP(x).update();
#define _DEFINE_PARAMETER_UPDATE_METHOD(...) protected: void updateParamsImpl() final { APPLY_ALL(_CALL_UPDATE, __VA_ARGS__) } private:
#define DEFINE_PARAMETERS(...) APPLY_ALL(_DEFINE_SINGLE_PARAMETER, __VA_ARGS__) _DEFINE_PARAMETER_UPDATE_METHOD(__VA_ARGS__)
#define _DEFINE_PARAMETER_UPDATE_METHOD_CUSTOM_PARENT(parent_class, ...) protected: void updateParamsImpl() override { parent_class::updateParamsImpl(); APPLY_ALL(_CALL_UPDATE, __VA_ARGS__) } private:
#define DEFINE_PARAMETERS_CUSTOM_PARENT(parent_class, ...) APPLY_ALL(_DEFINE_SINGLE_PARAMETER, __VA_ARGS__) _DEFINE_PARAMETER_UPDATE_METHOD_CUSTOM_PARENT(parent_class, __VA_ARGS__)
#endif
