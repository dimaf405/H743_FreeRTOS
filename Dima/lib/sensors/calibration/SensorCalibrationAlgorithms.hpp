/****************************************************************************
 * Fixed-memory calibration math for the Dima single-IMU/single-mag product.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <cstddef>
#include <cstdint>

namespace dima::lib::sensors::calibration {

struct Vector3d {
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

enum class PreflightCalibrationRequest : std::uint8_t {
    Unsupported,
    Cancel,
    Gyro,
    Magnetometer,
    Radio,
    Accelerometer,
};

/** 解码 MAV_CMD_PREFLIGHT_CALIBRATION 的 7 个参数：仅允许一个受支持 selector
 * 精确为 1 且其余为 0；七个 0 表示标准取消，NaN/组合请求均拒绝。 */
PreflightCalibrationRequest classify_preflight_calibration_request(
    const float (&values)[7]) noexcept;

class RunningStats3 final {
public:
    /* 三轴独立使用 Welford 在线算法，固定内存且比 sum/sum² 更抗消减误差。 */
    void reset() noexcept;
    void add(double x, double y, double z) noexcept;
    std::uint32_t count() const noexcept { return count_; }
    Vector3d mean() const noexcept;
    Vector3d variance() const noexcept;

private:
    double mean_[3]{};
    double m2_[3]{};
    std::uint32_t count_{0U};
};

bool finite(float x, float y, float z) noexcept;
double norm(double x, double y, double z) noexcept;

/** 解代数最小二乘球：A=[2x,2y,2z,1]，b=x²+y²+z²，输入为累计的
 * A^T A 与 A^T b；输出 center 与 radius 使用输入坐标的同一单位。 */
bool solve_sphere(const double normal[4][4], const double rhs[4],
                  Vector3d &center, double &radius) noexcept;

/** 在机体系拟合 PX4 六面加速度计模型，再把 offset 与对角 scale 变换回传感器系。
 * 测量顺序固定 +X,-X,+Y,-Y,+Z,-Z；输入/offset 为 m/s²，scale 无量纲。 */
bool fit_accel_six_side(const Vector3d (&measurements)[6],
                        const float (&sensor_to_body)[9],
                        Vector3d &sensor_offset,
                        Vector3d &sensor_scale) noexcept;

} // namespace dima::lib::sensors::calibration
