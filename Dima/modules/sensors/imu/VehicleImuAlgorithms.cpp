#include "VehicleImuAlgorithms.hpp"


// 普通运行期实现从对应头文件移出；保持原状态、错误分支和计算顺序。

namespace dima::modules::sensors::vehicle_imu_algorithms {

std::uint32_t integration_interval_us(
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

bool supported_integration_rate(std::int32_t rate_hz) noexcept
{
    return integration_interval_us(rate_hz) != 0U;
}

bool integration_ready(std::uint32_t accumulated_us,
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

Vector3 add(const Vector3 &left, const Vector3 &right) noexcept
{
    return {left.x + right.x, left.y + right.y, left.z + right.z};
}

Vector3 multiply(const Vector3 &value, float scalar) noexcept
{
    return {value.x * scalar, value.y * scalar, value.z * scalar};
}

Vector3 cross(const Vector3 &left, const Vector3 &right) noexcept
{
    return {left.y * right.z - left.z * right.y,
            left.z * right.x - left.x * right.z,
            left.x * right.y - left.y * right.x};
}

bool finite_vector(const Vector3 &value) noexcept
{
    return std::isfinite(value.x) && std::isfinite(value.y) &&
           std::isfinite(value.z);
}

bool calibration_id_matches(std::int32_t configured_device_id,
                                       std::uint32_t device_id) noexcept
{
    return configured_device_id > 0 && device_id != 0U &&
           configured_device_id == static_cast<std::int32_t>(device_id);
}

bool valid_accel_calibration(const Calibration &calibration) noexcept
{
    // diagonal scale 与参数元数据一致限制为 0.1..3.0，所有量必须有限。
    return finite_vector(calibration.offset) &&
           finite_vector(calibration.scale) &&
           calibration.scale.x >= 0.1F && calibration.scale.x <= 3.0F &&
           calibration.scale.y >= 0.1F && calibration.scale.y <= 3.0F &&
           calibration.scale.z >= 0.1F && calibration.scale.z <= 3.0F;
}

bool valid_gyro_calibration(const Calibration &calibration) noexcept
{
    return finite_vector(calibration.offset);
}

bool calibration_equal(const Calibration &left,
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

bool calibration_is_identity(const Calibration &calibration) noexcept
{
    return calibration.offset.x == 0.0F &&
           calibration.offset.y == 0.0F &&
           calibration.offset.z == 0.0F &&
           calibration.scale.x == 1.0F &&
           calibration.scale.y == 1.0F &&
           calibration.scale.z == 1.0F;
}

bool configuration_equal(const Configuration &left,
                                   const Configuration &right) noexcept
{
    return left.rotation == right.rotation &&
           left.fine_rotation_degrees.x == right.fine_rotation_degrees.x &&
           left.fine_rotation_degrees.y == right.fine_rotation_degrees.y &&
           left.fine_rotation_degrees.z == right.fine_rotation_degrees.z &&
           left.integration_rate_hz == right.integration_rate_hz &&
           left.clipping_notifications == right.clipping_notifications &&
           calibration_equal(left.accel, right.accel) &&
           calibration_equal(left.gyro, right.gyro);
}

std::uint8_t next_calibration_count(
    std::uint8_t current) noexcept
{
    // 变化计数饱和到 255，不回绕为 0，避免消费者把新校准误认作初始状态。
    return current == UINT8_MAX ? UINT8_MAX
                                : static_cast<std::uint8_t>(current + 1U);
}

bool make_rotation_matrix(std::int32_t rotation,
                                 float (&matrix)[9]) noexcept
{
    return dima::lib::sensors::make_rotation_matrix(rotation, matrix);
}

Vector3 correct_accel(const Vector3 &value,
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

Vector3 correct_gyro(const Vector3 &value,
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

SampleTimeStep classify_sample_time(
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

Vector3 trapezoid_delta(const Vector3 &previous,
                                  const Vector3 &current,
                                  std::uint32_t dt_us) noexcept
{
    // 梯形积分：delta=(previous+current)/2 * dt_us*1e-6。
    return multiply(add(previous, current),
                    static_cast<float>(dt_us) * 0.5e-6F);
}

Vector3 coning_increment(const Vector3 &last_angle_integral,
                                   const Vector3 &last_delta_angle,
                                   const Vector3 &delta_angle) noexcept
{
    // PX4 coning 增量：0.5*(last_integral+last_delta/6) x delta_angle。
    const Vector3 base = add(
        last_angle_integral, multiply(last_delta_angle, 1.0F / 6.0F));
    return multiply(cross(base, delta_angle), 0.5F);
}

} // namespace dima::modules::sensors::vehicle_imu_algorithms
