#pragma once

// PX4 原始 TrajMath 头的后半部公开了 matrix::Vector2f，但依赖原工程的全局
// include 顺序。本入口先补齐 Matrix 类型再包含原始算法头，使 Rover 控制核只
// 依赖平台无关数学库，避免通过完整 mathlib.h 反向引入 px4_platform_common。
#include <matrix/matrix/math.hpp>

#include "math/TrajMath.hpp"
