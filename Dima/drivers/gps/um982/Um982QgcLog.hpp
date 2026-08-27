#pragma once

#include "logging/logging.hpp"

// UM982 的运行与配置诊断会通过结构化日志转成 QGC STATUSTEXT。产品默认关闭，
// 现场需要取证时可把宏改为 1 后重新编译；该开关不参与 GPS 数据解析与发布。
#ifndef DIMA_UM982_QGC_LOG_ENABLED
#define DIMA_UM982_QGC_LOG_ENABLED 0
#endif

#if DIMA_UM982_QGC_LOG_ENABLED != 0 && \
    DIMA_UM982_QGC_LOG_ENABLED != 1
#error "DIMA_UM982_QGC_LOG_ENABLED must be 0 or 1"
#endif

#if DIMA_UM982_QGC_LOG_ENABLED
#define UM982_QGC_INFO(format, ...) \
    PX4_INFO(format, ##__VA_ARGS__)
#define UM982_QGC_ERR(format, ...) \
    PX4_ERR(format, ##__VA_ARGS__)
#else
// 关闭时保留不可达的 printf 属性调用，使格式串和参数继续接受编译器检查，
// 同时让优化器彻底移除格式化及 QGC 投递，不产生仅供日志变量的未使用告警。
#define UM982_QGC_INFO(format, ...) \
    __dima_px4_log_omit(_PX4_LOG_LEVEL_INFO, format, ##__VA_ARGS__)
#define UM982_QGC_ERR(format, ...) \
    __dima_px4_log_omit(_PX4_LOG_LEVEL_ERROR, format, ##__VA_ARGS__)
#endif
