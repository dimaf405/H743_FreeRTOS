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

constexpr std::uint32_t integration_interval_us(
    std::int32_t rate_hz) noexcept
{
    // IMU_INTEG_RATE 是 vehicle_imu 输出/积分频率，不是芯片 ODR：
    // interval_us=1e6/rate，仅允许能精确表示的 100/200/250/400 Hz。
    switch (rate_hz) {
    case 100: return 10000U;
    case 200: return 5000U;
    case 250: return 4000U;
    case 400: return 2500U;
    default: return 0U;
    }
}

constexpr bool supported_integration_rate(std::int32_t rate_hz) noexcept
{
    return integration_interval_us(rate_hz) != 0U;
}

/** Select the closest achievable reset point for a batched sensor stream.
 *  This follows PX4 VehicleIMU's half-sample relaxation: if the requested
 *  interval lies within half of the latest update interval, publishing now is
 *  closer than waiting for another complete batch. */
// PX4 半样本松弛：accumulated + latest_update/2 >= requested 时立即发布，
// 对批量 FIFO 输入选择离目标周期最近的可实现积分边界。
constexpr bool integration_ready(std::uint32_t accumulated_us,
                                 std::uint32_t latest_update_us,
                                 std::uint32_t requested_interval_us) noexcept
{
    if (accumulated_us == 0U || latest_update_us == 0U ||
        requested_interval_us == 0U) {
        return false;
    }
    return static_cast<std::uint64_t>(accumulated_us) +
               latest_update_us / 2U >=
           requested_interval_us;
}

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
    std::int32_t integration_rate_hz{200};
    bool clipping_notifications{true};
};

constexpr Vector3 add(const Vector3 &left, const Vector3 &right) noexcept
{
    return {left.x + right.x, left.y + right.y, left.z + right.z};
}

constexpr Vector3 multiply(const Vector3 &value, float scalar) noexcept
{
    return {value.x * scalar, value.y * scalar, value.z * scalar};
}

constexpr Vector3 cross(const Vector3 &left, const Vector3 &right) noexcept
{
    return {left.y * right.z - left.z * right.y,
            left.z * right.x - left.x * right.z,
            left.x * right.y - left.y * right.x};
}

inline bool finite_vector(const Vector3 &value) noexcept
{
    return std::isfinite(value.x) && std::isfinite(value.y) &&
           std::isfinite(value.z);
}

constexpr bool calibration_id_matches(std::int32_t configured_device_id,
                                       std::uint32_t device_id) noexcept
{
    return configured_device_id > 0 && device_id != 0U &&
           configured_device_id == static_cast<std::int32_t>(device_id);
}

inline bool valid_accel_calibration(const Calibration &calibration) noexcept
{
    // diagonal scale 与参数元数据一致限制为 0.1..3.0，所有量必须有限。
    return finite_vector(calibration.offset) &&
           finite_vector(calibration.scale) &&
           calibration.scale.x >= 0.1F && calibration.scale.x <= 3.0F &&
           calibration.scale.y >= 0.1F && calibration.scale.y <= 3.0F &&
           calibration.scale.z >= 0.1F && calibration.scale.z <= 3.0F;
}

inline bool valid_gyro_calibration(const Calibration &calibration) noexcept
{
    return finite_vector(calibration.offset);
}

constexpr bool calibration_equal(const Calibration &left,
                                  const Calibration &right) noexcept
{
    return left.offset.x == right.offset.x &&
           left.offset.y == right.offset.y &&
           left.offset.z == right.offset.z &&
           left.scale.x == right.scale.x &&
           left.scale.y == right.scale.y &&
           left.scale.z == right.scale.z &&
           left.configured_device_id == right.configured_device_id &&
           left.enabled == right.enabled;
}

constexpr bool calibration_is_identity(const Calibration &calibration) noexcept
{
    return calibration.offset.x == 0.0F &&
           calibration.offset.y == 0.0F &&
           calibration.offset.z == 0.0F &&
           calibration.scale.x == 1.0F &&
           calibration.scale.y == 1.0F &&
           calibration.scale.z == 1.0F;
}

constexpr bool configuration_equal(const Configuration &left,
                                   const Configuration &right) noexcept
{
    return left.rotation == right.rotation &&
           left.integration_rate_hz == right.integration_rate_hz &&
           left.clipping_notifications == right.clipping_notifications &&
           calibration_equal(left.accel, right.accel) &&
           calibration_equal(left.gyro, right.gyro);
}

constexpr std::uint8_t next_calibration_count(
    std::uint8_t current) noexcept
{
    // 变化计数饱和到 255，不回绕为 0，避免消费者把新校准误认作初始状态。
    return current == UINT8_MAX ? UINT8_MAX
                                : static_cast<std::uint8_t>(current + 1U);
}

inline bool make_rotation_matrix(std::int32_t rotation,
                                 float (&matrix)[9]) noexcept
{
    return dima::lib::sensors::make_rotation_matrix(rotation, matrix);
}

inline Vector3 correct_accel(const Vector3 &value,
                             const Calibration &calibration,
                             const float (&rotation)[9]) noexcept
{
    // 先在传感器轴上执行 corrected=(raw-offset)*scale，再以 row-major 旋转矩阵
    // 转到机体系；次序不可交换，offset/scale 参数定义在传感器坐标系。
    Vector3 corrected = value;
    if (calibration.enabled) {
        corrected.x = (corrected.x - calibration.offset.x) *
                      calibration.scale.x;
        corrected.y = (corrected.y - calibration.offset.y) *
                      calibration.scale.y;
        corrected.z = (corrected.z - calibration.offset.z) *
                      calibration.scale.z;
    }
    return dima::lib::sensors::rotate(rotation, corrected);
}

inline Vector3 correct_gyro(const Vector3 &value,
                            const Calibration &calibration,
                            const float (&rotation)[9]) noexcept
{
    // 陀螺校正只移除零偏：corrected=raw-offset，之后再转机体系。
    Vector3 corrected = value;
    if (calibration.enabled) {
        corrected.x -= calibration.offset.x;
        corrected.y -= calibration.offset.y;
        corrected.z -= calibration.offset.z;
    }
    return dima::lib::sensors::rotate(rotation, corrected);
}

enum class SampleTimeAction : std::uint8_t {
    Prime = 0U,
    Integrate,
    Reset,
};

struct SampleTimeStep {
    SampleTimeAction action{SampleTimeAction::Reset};
    std::uint32_t dt_us{0U};
};

constexpr SampleTimeStep classify_sample_time(
    std::uint64_t previous_timestamp_us, std::uint64_t timestamp_us,
    std::uint32_t maximum_gap_us) noexcept
{
    // 首样本只建立梯形积分左端点；时间不递增或间隔超过 maximum_gap 时重置，
    // 禁止跨丢样/时钟跳变积分出虚假 delta angle/velocity。
    if (previous_timestamp_us == 0U) {
        return {SampleTimeAction::Prime, 0U};
    }
    if (timestamp_us <= previous_timestamp_us ||
        timestamp_us - previous_timestamp_us > maximum_gap_us) {
        return {SampleTimeAction::Reset, 0U};
    }
    return {SampleTimeAction::Integrate,
            static_cast<std::uint32_t>(timestamp_us -
                                       previous_timestamp_us)};
}

constexpr Vector3 trapezoid_delta(const Vector3 &previous,
                                  const Vector3 &current,
                                  std::uint32_t dt_us) noexcept
{
    // 梯形积分：delta=(previous+current)/2 * dt_us*1e-6。
    return multiply(add(previous, current),
                    static_cast<float>(dt_us) * 0.5e-6F);
}

constexpr Vector3 coning_increment(const Vector3 &last_angle_integral,
                                   const Vector3 &last_delta_angle,
                                   const Vector3 &delta_angle) noexcept
{
    // PX4 coning 增量：0.5*(last_integral+last_delta/6) x delta_angle。
    const Vector3 base = add(
        last_angle_integral, multiply(last_delta_angle, 1.0F / 6.0F));
    return multiply(cross(base, delta_angle), 0.5F);
}

} // namespace dima::modules::sensors::vehicle_imu_algorithms
