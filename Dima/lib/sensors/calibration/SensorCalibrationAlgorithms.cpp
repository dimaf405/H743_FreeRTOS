#include "SensorCalibrationAlgorithms.hpp"

#include <algorithm>
#include <cmath>

namespace dima::lib::sensors::calibration {
namespace {

constexpr double kGravity = 9.80665;
constexpr double kMaximumAccelResidual = 2.5;

bool finite_vector(const Vector3d &value) noexcept
{
    return std::isfinite(value.x) && std::isfinite(value.y) &&
           std::isfinite(value.z);
}

bool invert_3x3(const double input[3][3], double (&inverse)[3][3]) noexcept
{
    /* Gauss-Jordan + 部分主元：每列选绝对值最大 pivot，阈值 1e-12 判奇异。
     * 增广矩阵 [A|I] 消元后右半部即 A^-1。 */
    double augmented[3][6]{};
    for (std::size_t row = 0U; row < 3U; ++row) {
        for (std::size_t column = 0U; column < 3U; ++column) {
            augmented[row][column] = input[row][column];
        }
        augmented[row][row + 3U] = 1.0;
    }

    for (std::size_t pivot = 0U; pivot < 3U; ++pivot) {
        std::size_t selected = pivot;
        for (std::size_t row = pivot + 1U; row < 3U; ++row) {
            if (std::fabs(augmented[row][pivot]) >
                std::fabs(augmented[selected][pivot])) {
                selected = row;
            }
        }
        if (!std::isfinite(augmented[selected][pivot]) ||
            std::fabs(augmented[selected][pivot]) < 1.0e-12) {
            return false;
        }
        if (selected != pivot) {
            for (std::size_t column = 0U; column < 6U; ++column) {
                std::swap(augmented[pivot][column],
                          augmented[selected][column]);
            }
        }

        const double divisor = augmented[pivot][pivot];
        for (std::size_t column = 0U; column < 6U; ++column) {
            augmented[pivot][column] /= divisor;
        }
        for (std::size_t row = 0U; row < 3U; ++row) {
            if (row == pivot) continue;
            const double factor = augmented[row][pivot];
            for (std::size_t column = 0U; column < 6U; ++column) {
                augmented[row][column] -=
                    factor * augmented[pivot][column];
            }
        }
    }

    for (std::size_t row = 0U; row < 3U; ++row) {
        for (std::size_t column = 0U; column < 3U; ++column) {
            inverse[row][column] = augmented[row][column + 3U];
        }
    }
    return true;
}

} // namespace

PreflightCalibrationRequest classify_preflight_calibration_request(
    const float (&values)[7]) noexcept
{
    // MAV_CMD_PREFLIGHT_CALIBRATION 的 param1..7 是互斥选择器。本实现只接受
    // “全零取消”或恰好一个受支持位置为 1，拒绝组合请求和非有限值，避免一次
    // 命令同时改写多组校准状态。
    bool all_zero = true;
    for (const float value : values) {
        if (!std::isfinite(value)) {
            return PreflightCalibrationRequest::Unsupported;
        }
        all_zero = all_zero && value == 0.0F;
    }
    if (all_zero) return PreflightCalibrationRequest::Cancel;

    const auto single_selector = [&values](std::size_t selected) noexcept {
        for (std::size_t index = 0U; index < 7U; ++index) {
            const float expected = index == selected ? 1.0F : 0.0F;
            if (values[index] != expected) return false;
        }
        return true;
    };
    if (single_selector(0U)) return PreflightCalibrationRequest::Gyro;
    if (single_selector(1U)) {
        return PreflightCalibrationRequest::Magnetometer;
    }
    if (single_selector(3U)) return PreflightCalibrationRequest::Radio;
    if (single_selector(4U)) {
        return PreflightCalibrationRequest::Accelerometer;
    }
    return PreflightCalibrationRequest::Unsupported;
}

void RunningStats3::reset() noexcept
{
    mean_[0] = mean_[1] = mean_[2] = 0.0;
    m2_[0] = m2_[1] = m2_[2] = 0.0;
    count_ = 0U;
}

void RunningStats3::add(double x, double y, double z) noexcept
{
    const double sample[3]{x, y, z};
    ++count_;
    for (std::size_t axis = 0U; axis < 3U; ++axis) {
        /* Welford：mean_n=mean_(n-1)+delta/n；M2_n=M2_(n-1)+delta*delta2。 */
        const double delta = sample[axis] - mean_[axis];
        mean_[axis] += delta / static_cast<double>(count_);
        const double delta2 = sample[axis] - mean_[axis];
        m2_[axis] += delta * delta2;
    }
}

Vector3d RunningStats3::mean() const noexcept
{
    return {mean_[0], mean_[1], mean_[2]};
}

Vector3d RunningStats3::variance() const noexcept
{
    if (count_ < 2U) return {};
    /* 使用样本方差 M2/(n-1)，n<2 时返回零向量。 */
    const double divisor = static_cast<double>(count_ - 1U);
    return {m2_[0] / divisor, m2_[1] / divisor, m2_[2] / divisor};
}

bool finite(float x, float y, float z) noexcept
{
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

double norm(double x, double y, double z) noexcept
{
    return std::sqrt(x * x + y * y + z * z);
}

bool solve_sphere(const double normal[4][4], const double rhs[4],
                  Vector3d &center, double &radius) noexcept
{
    // 求解最小二乘法线方程 [x y z 1] * [2cx 2cy 2cz d]^T = x²+y²+z²；
    // 调用方已累计 A^T*A 与 A^T*b，这里只做带部分主元的 4x4 消元。
    double augmented[4][5]{};
    for (std::size_t row = 0U; row < 4U; ++row) {
        for (std::size_t column = 0U; column < 4U; ++column) {
            augmented[row][column] = normal[row][column];
        }
        augmented[row][4] = rhs[row];
    }

    for (std::size_t pivot = 0U; pivot < 4U; ++pivot) {
        std::size_t selected = pivot;
        for (std::size_t row = pivot + 1U; row < 4U; ++row) {
            if (std::fabs(augmented[row][pivot]) >
                std::fabs(augmented[selected][pivot])) {
                selected = row;
            }
        }
        if (!std::isfinite(augmented[selected][pivot]) ||
            std::fabs(augmented[selected][pivot]) < 1.0e-12) {
            return false;
        }
        if (selected != pivot) {
            for (std::size_t column = pivot; column < 5U; ++column) {
                std::swap(augmented[pivot][column],
                          augmented[selected][column]);
            }
        }
        const double divisor = augmented[pivot][pivot];
        for (std::size_t column = pivot; column < 5U; ++column) {
            augmented[pivot][column] /= divisor;
        }
        for (std::size_t row = 0U; row < 4U; ++row) {
            if (row == pivot) continue;
            const double factor = augmented[row][pivot];
            for (std::size_t column = pivot; column < 5U; ++column) {
                augmented[row][column] -=
                    factor * augmented[pivot][column];
            }
        }
    }

    /* 解向量前三项是球心 c，第四项 d 满足 r²=d+|c|²。 */
    center = {augmented[0][4], augmented[1][4], augmented[2][4]};
    const double radius_squared = augmented[3][4] +
        center.x * center.x + center.y * center.y + center.z * center.z;
    if (!std::isfinite(center.x) || !std::isfinite(center.y) ||
        !std::isfinite(center.z) || !std::isfinite(radius_squared) ||
        radius_squared <= 0.0) {
        return false;
    }
    radius = std::sqrt(radius_squared);
    return std::isfinite(radius);
}

bool fit_accel_six_side(const Vector3d (&measurements)[6],
                        const float (&sensor_to_body)[9],
                        Vector3d &sensor_offset,
                        Vector3d &sensor_scale) noexcept
{
    for (const Vector3d &measurement : measurements) {
        if (!finite_vector(measurement)) return false;
    }
    for (const float coefficient : sensor_to_body) {
        if (!std::isfinite(coefficient)) return false;
    }

    Vector3d body_offset{};
    double body_samples[3][3]{};
    for (std::size_t axis = 0U; axis < 3U; ++axis) {
        const Vector3d &positive = measurements[axis * 2U];
        const Vector3d &negative = measurements[axis * 2U + 1U];
        /* 每轴正反面均值贡献 bias，总计六面故除以 6；半差组成 3x3 响应矩阵，
         * 其逆再乘标准重力 g=9.80665 得到机体系校正变换。 */
        body_offset.x += (positive.x + negative.x) / 6.0;
        body_offset.y += (positive.y + negative.y) / 6.0;
        body_offset.z += (positive.z + negative.z) / 6.0;
        body_samples[0][axis] = (positive.x - negative.x) * 0.5;
        body_samples[1][axis] = (positive.y - negative.y) * 0.5;
        body_samples[2][axis] = (positive.z - negative.z) * 0.5;
    }
    double body_transform[3][3]{};
    if (!invert_3x3(body_samples, body_transform)) return false;
    for (auto &row : body_transform) {
        for (double &value : row) value *= kGravity;
    }

    const double body_offset_values[3]{
        body_offset.x, body_offset.y, body_offset.z};
    double sensor_offset_values[3]{};
    for (std::size_t row = 0U; row < 3U; ++row) {
        for (std::size_t column = 0U; column < 3U; ++column) {
            sensor_offset_values[row] +=
                static_cast<double>(sensor_to_body[column * 3U + row]) *
                body_offset_values[column];
        }
    }

    // body_transform 是机体系校正矩阵，R 是 sensor->body 旋转；要把校准结果
    // 写回传感器轴，需做相似变换 R^T * body_transform * R。
    double transform_times_rotation[3][3]{};
    for (std::size_t row = 0U; row < 3U; ++row) {
        for (std::size_t column = 0U; column < 3U; ++column) {
            for (std::size_t index = 0U; index < 3U; ++index) {
                transform_times_rotation[row][column] +=
                    body_transform[row][index] *
                    static_cast<double>(
                        sensor_to_body[index * 3U + column]);
            }
        }
    }
    double sensor_transform[3][3]{};
    for (std::size_t row = 0U; row < 3U; ++row) {
        for (std::size_t column = 0U; column < 3U; ++column) {
            for (std::size_t index = 0U; index < 3U; ++index) {
                sensor_transform[row][column] +=
                    static_cast<double>(
                        sensor_to_body[index * 3U + row]) *
                    transform_times_rotation[index][column];
            }
        }
    }

    sensor_offset = {sensor_offset_values[0], sensor_offset_values[1],
                     sensor_offset_values[2]};
    /* 当前产品只持久化传感器系对角 scale，忽略交叉轴项；随后对全部六面重放
     * corrected=(raw-offset)*scale，并要求机体系残差范数 <=2.5 m/s²。 */
    sensor_scale = {sensor_transform[0][0], sensor_transform[1][1],
                    sensor_transform[2][2]};
    if (!finite_vector(sensor_offset) || !finite_vector(sensor_scale) ||
        std::fabs(sensor_offset.x) > 3.0 ||
        std::fabs(sensor_offset.y) > 3.0 ||
        std::fabs(sensor_offset.z) > 3.0 ||
        sensor_scale.x < 0.5 || sensor_scale.x > 1.5 ||
        sensor_scale.y < 0.5 || sensor_scale.y > 1.5 ||
        sensor_scale.z < 0.5 || sensor_scale.z > 1.5) {
        return false;
    }

    const double offset[3]{sensor_offset.x, sensor_offset.y,
                           sensor_offset.z};
    const double scale[3]{sensor_scale.x, sensor_scale.y, sensor_scale.z};
    for (std::size_t side = 0U; side < 6U; ++side) {
        const double body[3]{measurements[side].x, measurements[side].y,
                             measurements[side].z};
        double corrected_sensor[3]{};
        for (std::size_t sensor_axis = 0U; sensor_axis < 3U;
             ++sensor_axis) {
            double raw_sensor = 0.0;
            for (std::size_t body_axis = 0U; body_axis < 3U;
                 ++body_axis) {
                raw_sensor += static_cast<double>(sensor_to_body[
                    body_axis * 3U + sensor_axis]) * body[body_axis];
            }
            corrected_sensor[sensor_axis] =
                (raw_sensor - offset[sensor_axis]) * scale[sensor_axis];
        }
        double corrected_body[3]{};
        for (std::size_t body_axis = 0U; body_axis < 3U; ++body_axis) {
            for (std::size_t sensor_axis = 0U; sensor_axis < 3U;
                 ++sensor_axis) {
                corrected_body[body_axis] += static_cast<double>(
                    sensor_to_body[body_axis * 3U + sensor_axis]) *
                    corrected_sensor[sensor_axis];
            }
        }
        const std::size_t expected_axis = side / 2U;
        corrected_body[expected_axis] -=
            (side % 2U == 0U ? kGravity : -kGravity);
        if (norm(corrected_body[0], corrected_body[1], corrected_body[2]) >
            kMaximumAccelResidual) {
            return false;
        }
    }
    return true;
}

} // namespace dima::lib::sensors::calibration
