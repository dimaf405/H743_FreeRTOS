/****************************************************************************
 * PX4-Autopilot v1.17.0 sensor rotation contract adapted for Dima.
 * Upstream: src/lib/conversion/rotation.h
 * @ d6f12ad1c4f70ad3230afd7d86e971421e02fef4.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <cstddef>
#include <cstdint>

namespace dima::lib::sensors {

constexpr std::size_t kSensorRotationCount = 41U;

/* 旋转编号与 PX4 ROTATION_* 数值合同一致；矩阵为 row-major sensor->body，
 * Vector3 不携带单位，输入输出保持同一物理单位。 */
struct Vector3 {
    float x{0.0F};
    float y{0.0F};
    float z{0.0F};
};

bool valid_rotation(std::int32_t rotation) noexcept;
bool make_rotation_matrix(std::int32_t rotation,
                          float (&matrix)[9]) noexcept;
Vector3 rotate(const float (&matrix)[9], const Vector3 &value) noexcept;

} // namespace dima::lib::sensors
