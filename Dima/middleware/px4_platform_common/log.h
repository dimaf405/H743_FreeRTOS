#pragma once

#include <cstdio>

// EKF Core 在 MODULE_NAME 构建下沿用项目已有的有界日志队列；这样状态切换
// 仍保留 PX4 的 ECL_* 诊断语义，同时不会从 estimator 实时任务直接 printf。
// PX4 官方 uORB 打印模板也依赖 log.h 暴露 printf 声明；保留该标准头可让
// 生成单元继续原样编译，不能逐个修改 build/generated 下的派生源码。
#include "logging/logging.hpp"
