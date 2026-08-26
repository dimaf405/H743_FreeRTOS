/****************************************************************************
 * PX4-Autopilot v1.17.0 sensor rotation contract adapted for Dima.
 * Upstream: src/lib/conversion/rotation.h
 * @ d6f12ad1c4f70ad3230afd7d86e971421e02fef4.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#include "SensorRotation.hpp"

namespace dima::lib::sensors {
namespace {

struct EulerDegrees {
    std::uint16_t roll;
    std::uint16_t pitch;
    std::uint16_t yaw;
};

constexpr EulerDegrees kRotations[kSensorRotationCount]{
    {0U, 0U, 0U},       {0U, 0U, 45U},      {0U, 0U, 90U},
    {0U, 0U, 135U},     {0U, 0U, 180U},     {0U, 0U, 225U},
    {0U, 0U, 270U},     {0U, 0U, 315U},     {180U, 0U, 0U},
    {180U, 0U, 45U},    {180U, 0U, 90U},    {180U, 0U, 135U},
    {0U, 180U, 0U},     {180U, 0U, 225U},   {180U, 0U, 270U},
    {180U, 0U, 315U},   {90U, 0U, 0U},      {90U, 0U, 45U},
    {90U, 0U, 90U},     {90U, 0U, 135U},    {270U, 0U, 0U},
    {270U, 0U, 45U},    {270U, 0U, 90U},    {270U, 0U, 135U},
    {0U, 90U, 0U},      {0U, 270U, 0U},     {0U, 180U, 90U},
    {0U, 180U, 270U},   {90U, 90U, 0U},     {180U, 90U, 0U},
    {270U, 90U, 0U},    {90U, 180U, 0U},    {270U, 180U, 0U},
    {90U, 270U, 0U},    {180U, 270U, 0U},   {270U, 270U, 0U},
    {90U, 180U, 90U},   {90U, 0U, 270U},    {90U, 68U, 293U},
    {0U, 315U, 0U},     {90U, 315U, 0U},
};

constexpr std::int32_t kPrecomputedRotation = 38;
constexpr float kPrecomputedMatrix[9]{
    0.14637052F, -0.3448272F, -0.92718387F,
    0.36227965F, -0.85347724F, 0.37460664F,
    -0.9205048F, -0.39073122F, -1.6374576e-8F,
};
constexpr float kSqrtHalf = 0.70710678118654752440F;

/* 常见 45° 倍数使用精确常量表，避免嵌入式运行时 sin/cos 和不同 libm 舍入；
 * rotation 38 含非 45° Euler 角，使用上游预计算矩阵保持数值一致。 */
constexpr bool sine_cosine(std::uint16_t degrees, float &sine,
                           float &cosine) noexcept
{
    switch (degrees % 360U) {
    case 0U:   sine = 0.0F;       cosine = 1.0F;       return true;
    case 45U:  sine = kSqrtHalf;  cosine = kSqrtHalf; return true;
    case 90U:  sine = 1.0F;       cosine = 0.0F;       return true;
    case 135U: sine = kSqrtHalf;  cosine = -kSqrtHalf; return true;
    case 180U: sine = 0.0F;       cosine = -1.0F;      return true;
    case 225U: sine = -kSqrtHalf; cosine = -kSqrtHalf; return true;
    case 270U: sine = -1.0F;      cosine = 0.0F;       return true;
    case 315U: sine = -kSqrtHalf; cosine = kSqrtHalf;  return true;
    default:   return false;
    }
}

void copy_matrix(const float source[9], float (&destination)[9]) noexcept
{
    for (std::size_t index = 0U; index < 9U; ++index) {
        destination[index] = source[index];
    }
}

} // namespace

bool valid_rotation(std::int32_t rotation) noexcept
{
    return rotation >= 0 &&
           static_cast<std::size_t>(rotation) < kSensorRotationCount;
}

bool make_rotation_matrix(std::int32_t rotation,
                          float (&matrix)[9]) noexcept
{
    if (!valid_rotation(rotation)) {
        return false;
    }
    if (rotation == kPrecomputedRotation) {
        copy_matrix(kPrecomputedMatrix, matrix);
        return true;
    }

    const EulerDegrees euler = kRotations[rotation];
    float sr = 0.0F;
    float cr = 1.0F;
    float sp = 0.0F;
    float cp = 1.0F;
    float sy = 0.0F;
    float cy = 1.0F;
    if (!sine_cosine(euler.roll, sr, cr) ||
        !sine_cosine(euler.pitch, sp, cp) ||
        !sine_cosine(euler.yaw, sy, cy)) {
        return false;
    }

    /* 采用 yaw-pitch-roll 的 Rz(yaw)*Ry(pitch)*Rx(roll)，row-major 展开；
     * 生成的矩阵把传感器坐标向量左乘转换到机体 FRD 坐标。 */
    const float next[9]{
        cp * cy,
        sr * sp * cy - cr * sy,
        cr * sp * cy + sr * sy,
        cp * sy,
        sr * sp * sy + cr * cy,
        cr * sp * sy - sr * cy,
        -sp,
        sr * cp,
        cr * cp,
    };
    copy_matrix(next, matrix);
    return true;
}

Vector3 rotate(const float (&matrix)[9], const Vector3 &value) noexcept
{
    /* [xb,yb,zb]^T = R_sensor_to_body * [xs,ys,zs]^T。 */
    return {matrix[0] * value.x + matrix[1] * value.y +
                matrix[2] * value.z,
            matrix[3] * value.x + matrix[4] * value.y +
                matrix[5] * value.z,
            matrix[6] * value.x + matrix[7] * value.y +
                matrix[8] * value.z};
}

} // namespace dima::lib::sensors
