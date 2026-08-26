#define MODULE_NAME "sensor_cal"
#include "SensorCalibration.hpp"

#include "logging/logging.hpp"
#include "SensorRotation.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace dima::modules::sensors {
namespace {

constexpr const char *kMagSideNames[6]{
    "back", "front", "left", "right", "up", "down"};
constexpr double kMagReferenceRadiusGa = 0.2;

std::uint32_t completed_sides(const bool (&sides)[6]) noexcept
{
    std::uint32_t count = 0U;
    for (const bool done : sides) count += done ? 1U : 0U;
    return count;
}

std::uint64_t sample_time(std::uint64_t timestamp_sample,
                          std::uint64_t timestamp) noexcept
{
    return timestamp_sample != 0U ? timestamp_sample : timestamp;
}

void report_pending_sides(const bool (&sides)[6]) noexcept
{
    PX4_INFO_RAW("[cal] pending:%s%s%s%s%s%s",
                 sides[0] ? "" : " back",
                 sides[1] ? "" : " front",
                 sides[2] ? "" : " left",
                 sides[3] ? "" : " right",
                 sides[4] ? "" : " up",
                 sides[5] ? "" : " down");
}

} // namespace

void SensorCalibration::reset_mag_accumulator() noexcept
{
    // 重置六面、球拟合法方程、包围盒、空间点集和有符号转角积分；minimum/
    // maximum 用 +/-Inf 初始化，使首个有限样本自然建立轴范围。
    mag_ = MagAccumulator{};
    for (std::size_t axis = 0U; axis < 3U; ++axis) {
        mag_.minimum[axis] = std::numeric_limits<double>::infinity();
        mag_.maximum[axis] = -std::numeric_limits<double>::infinity();
        mag_rotation_integral_[axis] = 0.0;
    }
    for (bool &done : mag_sides_done_) done = false;
    mag_orientation_stats_.reset();
    mag_last_accel_sample_us_ = 0U;
    mag_last_gyro_sample_us_ = 0U;
    mag_side_started_us_ = 0U;
    mag_collection_started_us_ = 0U;
    mag_accel_device_id_ = 0U;
    mag_gyro_device_id_ = 0U;
    mag_side_samples_ = 0U;
    mag_orientation_candidate_ = -1;
    mag_active_side_ = -1;
    mag_reported_completed_side_ = -1;
    mag_rotation_detected_ = false;
}

bool SensorCalibration::add_mag_sample(const sensor_mag_s &sample) noexcept
{
    if (mag_.samples >= kMagMinimumSamples) return false;

    const double x = sample.x;
    const double y = sample.y;
    const double z = sample.z;
    /* PX4 mag_calibration.cpp rejects points which are too close to an
     * already accepted point. Its no-GPS reference radius is 0.2 gauss and
     * the fixed six-side set contains 240 samples. Preserve that spacing
     * rule so a stationary/noisy side cannot fill the fit by repetition. */
    // PX4 无 GPS 参考半径为 0.2 gauss，最小空间距离：
    // abs(5.4*0.2/sqrt(240))/3。与任一已收样本更近则拒绝，防止静止噪声凑数。
    const double minimum_distance = std::fabs(
        5.4 * kMagReferenceRadiusGa /
        std::sqrt(static_cast<double>(kMagMinimumSamples))) / 3.0;
    for (std::uint32_t index = 0U; index < mag_.samples; ++index) {
        const double dx = x - static_cast<double>(mag_.x[index]);
        const double dy = y - static_cast<double>(mag_.y[index]);
        const double dz = z - static_cast<double>(mag_.z[index]);
        if (algorithms::norm(dx, dy, dz) < minimum_distance) return false;
    }

    mag_.x[mag_.samples] = sample.x;
    mag_.y[mag_.samples] = sample.y;
    mag_.z[mag_.samples] = sample.z;
    // 球方程线性化：(2x,2y,2z,1)*[cx,cy,cz,d]^T=x^2+y^2+z^2；
    // 在线累积 normal=A^T A、rhs=A^T b，结束后解 center 与 radius。
    const double design[4]{2.0 * x, 2.0 * y, 2.0 * z, 1.0};
    const double observation = x * x + y * y + z * z;
    for (std::size_t row = 0U; row < 4U; ++row) {
        for (std::size_t column = 0U; column < 4U; ++column) {
            mag_.normal[row][column] += design[row] * design[column];
        }
        mag_.rhs[row] += design[row] * observation;
    }
    const double values[3]{x, y, z};
    for (std::size_t axis = 0U; axis < 3U; ++axis) {
        mag_.minimum[axis] = std::min(mag_.minimum[axis], values[axis]);
        mag_.maximum[axis] = std::max(mag_.maximum[axis], values[axis]);
    }
    ++mag_.samples;
    return true;
}

void SensorCalibration::process_mag(std::uint64_t now) noexcept
{
    // 磁校准锁定 mag/accel/gyro 三个 device_id；任何辅助传感器切换或非有限值
    // 都使整个事务失败，避免不同物理设备的数据混入同一拟合。
    const std::uint64_t elapsed = now - started_us_;
    if (elapsed > kMagTimeoutUs) {
        fail("magnetometer six-side timeout");
        return;
    }
    if (!fresh(now, sensor_mag_.timestamp, kSensorFreshnessUs) ||
        !fresh(now, sensor_accel_.timestamp, kSensorFreshnessUs) ||
        !fresh(now, sensor_gyro_.timestamp, kSensorFreshnessUs)) {
        if (elapsed > kSensorFreshnessUs) {
            fail("magnetometer, accelerometer or gyro data timeout");
        }
        return;
    }
    if (sensor_mag_.device_id != device_id_ ||
        sensor_accel_.device_id != mag_accel_device_id_ ||
        sensor_gyro_.device_id != mag_gyro_device_id_ ||
        !algorithms::finite(sensor_mag_.x, sensor_mag_.y, sensor_mag_.z) ||
        !algorithms::finite(sensor_accel_.x, sensor_accel_.y,
                            sensor_accel_.z) ||
        !algorithms::finite(sensor_gyro_.x, sensor_gyro_.y,
                            sensor_gyro_.z)) {
        fail("magnetometer support sensor identity or sample invalid");
        return;
    }

    // 阶段一：车辆静止时用旋转后的加速度连续 25 点识别一个未完成方向。
    if (mag_active_side_ < 0) {
        const std::uint64_t accel_time = sample_time(
            sensor_accel_.timestamp_sample, sensor_accel_.timestamp);
        if (accel_time <= mag_last_accel_sample_us_) return;
        mag_last_accel_sample_us_ = accel_time;

        if (algorithms::norm(sensor_gyro_.x, sensor_gyro_.y,
                             sensor_gyro_.z) > 0.15) {
            mag_orientation_candidate_ = -1;
            mag_orientation_stats_.reset();
            return;
        }

        const dima::lib::sensors::Vector3 body_accel =
            dima::lib::sensors::rotate(
                board_rotation_matrix_,
                {sensor_accel_.x, sensor_accel_.y, sensor_accel_.z});
        sensor_accel_s body_sample = sensor_accel_;
        body_sample.x = body_accel.x;
        body_sample.y = body_accel.y;
        body_sample.z = body_accel.z;
        const int side = classify_accel_side(body_sample);
        if (side < 0) {
            mag_orientation_candidate_ = -1;
            mag_orientation_stats_.reset();
            return;
        }
        if (mag_orientation_candidate_ != side) {
            mag_orientation_candidate_ = side;
            mag_orientation_stats_.reset();
            if (mag_reported_completed_side_ != side) {
                mag_reported_completed_side_ = -1;
            }
        }
        mag_orientation_stats_.add(
            body_accel.x, body_accel.y, body_accel.z);
        if (mag_orientation_stats_.count() < kMagOrientationSamples) return;

        const auto variance = mag_orientation_stats_.variance();
        if (variance.x > 0.09 || variance.y > 0.09 || variance.z > 0.09) {
            mag_orientation_candidate_ = -1;
            mag_orientation_stats_.reset();
            return;
        }
        if (mag_sides_done_[side]) {
            if (mag_reported_completed_side_ != side) {
                PX4_INFO_RAW("[cal] %s side already completed",
                             kMagSideNames[side]);
                mag_reported_completed_side_ = side;
            }
            mag_orientation_candidate_ = -1;
            mag_orientation_stats_.reset();
            return;
        }

        mag_active_side_ = side;
        mag_orientation_candidate_ = -1;
        mag_orientation_stats_.reset();
        mag_reported_completed_side_ = -1;
        mag_side_started_us_ = now;
        mag_collection_started_us_ = 0U;
        mag_side_samples_ = 0U;
        mag_rotation_detected_ = false;
        for (double &integral : mag_rotation_integral_) integral = 0.0;
        mag_last_gyro_sample_us_ = sample_time(
            sensor_gyro_.timestamp_sample, sensor_gyro_.timestamp);
        /* PX4 repeats the orientation token to make the QGC state transition
         * robust against one lost STATUSTEXT record. */
        // 重复 orientation token 是 PX4/QGC 的抗丢包协议行为，不是冗余日志。
        PX4_INFO_RAW("[cal] %s orientation detected",
                     kMagSideNames[side]);
        PX4_INFO_RAW("[cal] %s orientation detected",
                     kMagSideNames[side]);
        PX4_INFO_RAW("[cal] Rotate vehicle");
        return;
    }

    // 阶段二：逐轴积分有符号角速度 theta+=omega*dt；取三轴净转角绝对值最大者，
    // 达到 0.5 rad 才开始收磁场点，往复振动不会累计成有效旋转。
    const std::uint64_t gyro_time = sample_time(
        sensor_gyro_.timestamp_sample, sensor_gyro_.timestamp);
    if (gyro_time > mag_last_gyro_sample_us_) {
        const std::uint64_t gyro_dt_us = gyro_time -
            mag_last_gyro_sample_us_;
        if (mag_last_gyro_sample_us_ != 0U &&
            gyro_dt_us <= kSensorFreshnessUs) {
            const double dt_s = static_cast<double>(gyro_dt_us) * 1.0e-6;
            /* PX4 integrates signed angular rate and tests the absolute net
             * angle. Integrating per-sample magnitudes would allow vibration
             * or alternating motion to masquerade as a real rotation. */
            mag_rotation_integral_[0] +=
                static_cast<double>(sensor_gyro_.x) * dt_s;
            mag_rotation_integral_[1] +=
                static_cast<double>(sensor_gyro_.y) * dt_s;
            mag_rotation_integral_[2] +=
                static_cast<double>(sensor_gyro_.z) * dt_s;
        }
        mag_last_gyro_sample_us_ = gyro_time;
    }

    if (!mag_rotation_detected_) {
        const double maximum_rotation = std::max(
            std::fabs(mag_rotation_integral_[0]), std::max(
                std::fabs(mag_rotation_integral_[1]),
                std::fabs(mag_rotation_integral_[2])));
        if (maximum_rotation >= kMagMinimumRotationRad) {
            mag_rotation_detected_ = true;
            mag_collection_started_us_ = now;
            last_sample_us_ = sample_time(
                sensor_mag_.timestamp_sample, sensor_mag_.timestamp);
        } else if (now - mag_side_started_us_ >= kMagRotationTimeoutUs) {
            fail("magnetometer side requires rotation");
        }
        return;
    }

    // 阶段三：旋转确认后每面最多 7 s 收 40 个空间去重样本；磁场模长合法范围
    // 为 0.05..2.0 gauss，超出范围不进入拟合。
    const std::uint64_t collection_elapsed = now - mag_collection_started_us_;

    const std::uint64_t mag_time = sample_time(
        sensor_mag_.timestamp_sample, sensor_mag_.timestamp);
    if (mag_time > last_sample_us_) {
        last_sample_us_ = mag_time;
        const double magnitude = algorithms::norm(
            sensor_mag_.x, sensor_mag_.y, sensor_mag_.z);
        if (magnitude >= 0.05 && magnitude <= 2.0 &&
            add_mag_sample(sensor_mag_)) {
            ++mag_side_samples_;
        }
    }

    const std::uint32_t done = completed_sides(mag_sides_done_);
    const std::uint8_t side_progress = static_cast<std::uint8_t>(
        std::min<std::uint32_t>(
            15U, mag_side_samples_ * 15U / kMagSideSamples));
    update_progress(static_cast<std::uint8_t>(done * 90U / 6U +
                                              side_progress), now);

    if (mag_side_samples_ < kMagSideSamples) {
        if (collection_elapsed >= kMagSideCollectionUs) {
            fail("magnetometer side has insufficient samples");
        }
        return;
    }

    const int completed_side = mag_active_side_;
    if (completed_side < 0 || completed_side >= 6) {
        fail("magnetometer side state invalid");
        return;
    }
    mag_sides_done_[completed_side] = true;
    mag_active_side_ = -1;
    mag_side_started_us_ = 0U;
    mag_collection_started_us_ = 0U;
    mag_side_samples_ = 0U;
    mag_rotation_detected_ = false;
    mag_last_gyro_sample_us_ = 0U;
    PX4_INFO_RAW("[cal] %s side done, rotate to a different side",
                 kMagSideNames[completed_side]);
    PX4_INFO_RAW("[cal] %s side done, rotate to a different side",
                 kMagSideNames[completed_side]);
    const std::uint32_t completed = completed_sides(mag_sides_done_);
    update_progress(static_cast<std::uint8_t>(completed * 90U / 6U), now);
    if (completed != 6U) {
        report_pending_sides(mag_sides_done_);
        PX4_INFO_RAW("[cal] hold vehicle still on a pending side");
        return;
    }

    if (mag_.samples < kMagMinimumSamples) {
        fail("magnetometer sample set incomplete");
        return;
    }
    // 六面完成后每轴 half_span=(max-min)/2，至少 0.20 gauss；diagonal scale
    // 取 fitted_radius/half_span，当前产品不声称支持非对角 soft-iron 矩阵。
    double half_span[3]{};
    for (std::size_t axis = 0U; axis < 3U; ++axis) {
        half_span[axis] =
            (mag_.maximum[axis] - mag_.minimum[axis]) * 0.5;
        if (!std::isfinite(half_span[axis]) || half_span[axis] < 0.20) {
            fail("magnetometer coverage incomplete");
            return;
        }
    }

    algorithms::Vector3d center{};
    double radius = 0.0;
    if (!algorithms::solve_sphere(mag_.normal, mag_.rhs,
                                  center, radius) ||
        radius < 0.2 || radius >= 0.7) {
        fail("magnetometer sphere fit invalid");
        return;
    }
    /* PX4 warns about an axis offset above 1.3 gauss because saturation is
     * likely, but it does not turn that warning into a protocol failure. */
    // 单轴 hard-iron offset >1.3 gauss 可能接近饱和，仅告警而不改变 PX4 的
    // 接受语义；球半径仍必须在 [0.2,0.7) gauss，scale 必须在 0.1..3.0。
    if (std::fabs(center.x) > 1.3 || std::fabs(center.y) > 1.3 ||
        std::fabs(center.z) > 1.3) {
        PX4_WARN("magnetometer calibration has large offsets");
    }
    algorithms::Vector3d scale{
        radius / half_span[0], radius / half_span[1],
        radius / half_span[2]};
    if (scale.x < kMagMinimumScale || scale.x > kMagMaximumScale ||
        scale.y < kMagMinimumScale || scale.y > kMagMaximumScale ||
        scale.z < kMagMinimumScale || scale.z > kMagMaximumScale) {
        fail("magnetometer scale fit invalid");
        return;
    }
    if (!commit_mag(center, scale, device_id_)) {
        fail("unable to commit magnetometer parameters");
        return;
    }
    begin_wait_for_apply(now);
}

} // namespace dima::modules::sensors
