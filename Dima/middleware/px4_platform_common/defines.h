#pragma once

#define PX4_ERROR (-1)
#define PX4_OK 0

// PX4 matrix/mathlib 与 EKF2 通过同一有限值谓词拒绝 NaN/Inf；保留上游
// constexpr 语义，避免在协方差和观测门限处引入第二套浮点判定。
#ifdef __cplusplus
static inline constexpr bool PX4_ISFINITE(float value)
{
    return __builtin_isfinite(value);
}
static inline constexpr bool PX4_ISFINITE(double value)
{
    return __builtin_isfinite(value);
}
#else
#define PX4_ISFINITE(value) __builtin_isfinite(value)
#endif

#ifndef __EXPORT
#define __EXPORT __attribute__((visibility("default")))
#endif

// PX4 v1.17 的 mathlib 使用显式 float 常量，不能依赖 libc 是否暴露非标准
// M_PI 宏；这些数值保持与上游 defines.h 一致。
#define M_E_F 2.71828183f
#define M_LOG2E_F 1.44269504f
#define M_LOG10E_F 0.43429448f
#define M_LN2_F 0.69314718f
#define M_LN10_F 2.30258509f
#define M_PI_F 3.14159265f
#define M_TWOPI_F 6.28318531f
#define M_PI_2_F 1.57079632f
#define M_PI_4_F 0.78539816f
#define M_3PI_4_F 2.35619449f
#define M_SQRTPI_F 1.77245385f
#define M_1_PI_F 0.31830989f
#define M_2_PI_F 0.63661977f
#define M_2_SQRTPI_F 1.12837917f
#define M_DEG_TO_RAD_F 0.0174532925f
#define M_RAD_TO_DEG_F 57.2957795f
#define M_SQRT2_F 1.41421356f
#define M_SQRT1_2_F 0.70710678f
#define M_LN2LO_F 1.90821484E-10f
#define M_LN2HI_F 0.69314718f
#define M_SQRT3_F 1.73205081f
#define M_IVLN10_F 0.43429448f
#define M_LOG2_E_F 0.69314718f
#define M_INVLN2_F 1.44269504f

#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502884
#endif

#define M_DEG_TO_RAD 0.017453292519943295
#define M_RAD_TO_DEG 57.295779513082323
