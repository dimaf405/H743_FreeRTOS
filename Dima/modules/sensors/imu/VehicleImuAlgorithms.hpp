/****************************************************************************
 * PX4-Autopilot v1.17.0 VehicleIMU algorithms adapted for Dima.
 * Upstream: src/modules/sensors/vehicle_imu and Integrator.hpp
 * @ d6f12ad1c4f70ad3230afd7d86e971421e02fef4.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include "sensors/SensorRotation.hpp"

#include <cmath>
#include <cstdint>

namespace dima::modules::sensors::vehicle_imu_algorithms {

using Vector3 = dima::lib::sensors::Vector3;

std::uint32_t integration_interval_us(
    std::int32_t rate_hz) noexcept;

bool supported_integration_rate(std::int32_t rate_hz) noexcept;

/** Select the closest achievable reset point for a batched sensor stream.
 *  This follows PX4 VehicleIMU's half-sample relaxation: if the requested
 *  interval lies within half of the latest update interval, publishing now is
 *  closer than waiting for another complete batch. */
// PX4 半样本松弛：accumulated + latest_update/2 >= requested 时立即发布，
// 对批量 FIFO 输入选择离目标周期最近的可实现积分边界。
bool integration_ready(std::uint32_t accumulated_us,
                                 std::uint32_t latest_update_us,
                                 std::uint32_t requested_interval_us) noexcept;

struct Calibration {
    // 加速度校正为 (raw-offset)*scale，陀螺只减 offset；随后统一 sensor->body
    // 旋转。count 是校准变化计数，不是校准质量或执行次数。
    Vector3 offset{};
    Vector3 scale{1.0F, 1.0F, 1.0F};
    std::int32_t configured_device_id{0};
    std::uint8_t count{0U};
    bool enabled{false};
};

struct Configuration {
    Calibration accel{};
    Calibration gyro{};
    float rotation_matrix[9]{1.0F, 0.0F, 0.0F,
                             0.0F, 1.0F, 0.0F,
                             0.0F, 0.0F, 1.0F};
    std::int32_t rotation{0};
    Vector3 fine_rotation_degrees{};
    std::int32_t integration_rate_hz{200};
    bool clipping_notifications{true};
};

Vector3 add(const Vector3 &left, const Vector3 &right) noexcept;

Vector3 multiply(const Vector3 &value, float scalar) noexcept;

Vector3 cross(const Vector3 &left, const Vector3 &right) noexcept;

bool finite_vector(const Vector3 &value) noexcept;

bool calibration_id_matches(std::int32_t configured_device_id,
                                       std::uint32_t device_id) noexcept;

bool valid_accel_calibration(const Calibration &calibration) noexcept;

bool valid_gyro_calibration(const Calibration &calibration) noexcept;

bool calibration_equal(const Calibration &left,
                                  const Calibration &right) noexcept;

bool calibration_is_identity(const Calibration &calibration) noexcept;

bool configuration_equal(const Configuration &left,
                                   const Configuration &right) noexcept;

std::uint8_t next_calibration_count(
    std::uint8_t current) noexcept;

bool make_rotation_matrix(std::int32_t rotation,
                                 float (&matrix)[9]) noexcept;

Vector3 correct_accel(const Vector3 &value,
                             const Calibration &calibration,
                             const float (&rotation)[9]) noexcept;

Vector3 correct_gyro(const Vector3 &value,
                            const Calibration &calibration,
                            const float (&rotation)[9]) noexcept;

enum class SampleTimeAction : std::uint8_t {
    Prime = 0U,
    Integrate,
    Reset,
};

struct SampleTimeStep {
    SampleTimeAction action{SampleTimeAction::Reset};
    std::uint32_t dt_us{0U};
};

SampleTimeStep classify_sample_time(
    std::uint64_t previous_timestamp_us, std::uint64_t timestamp_us,
    std::uint32_t maximum_gap_us) noexcept;

Vector3 trapezoid_delta(const Vector3 &previous,
                                  const Vector3 &current,
                                  std::uint32_t dt_us) noexcept;

Vector3 coning_increment(const Vector3 &last_angle_integral,
                                   const Vector3 &last_delta_angle,
                                   const Vector3 &delta_angle) noexcept;

} // namespace dima::modules::sensors::vehicle_imu_algorithms
