#define MODULE_NAME "sensor_cal"
#include "SensorCalibration.hpp"

#include "api/Time.hpp"
#include "imu/VehicleImu.hpp"
#include "magnetometer/VehicleMagnetometer.hpp"
#include "parameters/param.h"

#include <algorithm>
#include <cmath>
#include <iterator>

namespace dima::modules::sensors {

bool SensorCalibration::level_parameters_unchanged() const noexcept
{
    std::int32_t rotation{};
    float roll{}, pitch{}, yaw{};
    px4::AtomicTransaction transaction;
    if (feedback_owner_ == sensor_calibration_request_s::FEEDBACK_AUTO &&
        param_set_count() != level_start_set_count_) return false;
    return param_get(param_handle(dima::params::SENS_BOARD_ROT), &rotation) == 0 &&
        param_get(param_handle(dima::params::SENS_BOARD_X_OFF), &roll) == 0 &&
        param_get(param_handle(dima::params::SENS_BOARD_Y_OFF), &pitch) == 0 &&
        param_get(param_handle(dima::params::SENS_BOARD_Z_OFF), &yaw) == 0 &&
        rotation == level_board_rotation_ && roll == level_old_offsets_[0] &&
        pitch == level_old_offsets_[1] && yaw == level_old_offsets_[2];
}

void SensorCalibration::process_level(std::uint64_t now) noexcept
{
    // 静止窗口为 PX4 的 500 ms，最多尝试 25 s；重复姿态样本不能增加样本数，
    // 参数中途改变则终止，防止把不同坐标系或不同校正混进同一个均值。
    if (now < started_us_ || now - started_us_ > 25000000ULL) {
        fail("level calibration motion timeout");
        return;
    }
    if (!level_parameters_unchanged()) {
        fail("board parameters changed during level calibration");
        return;
    }
    if (!fresh(now, attitude_.timestamp, 100000ULL) ||
        !fresh(now, imu_.timestamp, 100000ULL) ||
        !fresh(now, sensor_gyro_.timestamp, kSensorFreshnessUs) ||
        !fresh(now, sensor_accel_.timestamp, kSensorFreshnessUs) ||
        sensor_accel_.device_id != device_id_ ||
        imu_.accel_device_id != device_id_) {
        fail("fresh attitude and IMU required for level calibration");
        return;
    }
    if (attitude_.timestamp_sample == last_sample_us_) return;
    if (attitude_.timestamp_sample == 0U ||
        attitude_.timestamp_sample > attitude_.timestamp ||
        (last_sample_us_ != 0U && attitude_.timestamp_sample < last_sample_us_)) {
        fail("level attitude timestamp invalid");
        return;
    }
    last_sample_us_ = attitude_.timestamp_sample;
    double roll{}, pitch{};
    if (!algorithms::level_sample(attitude_.q, level_old_offsets_[0],
                                  level_old_offsets_[1], roll, pitch)) {
        fail("level attitude or board offset out of range");
        return;
    }
    // 单窗口极差控制角度稳定性，陀螺/重力幅值另行排除缓慢转动及加减速。
    const double spin = algorithms::norm(sensor_gyro_.x, sensor_gyro_.y, sensor_gyro_.z);
    const double gravity = algorithms::norm(sensor_accel_.x, sensor_accel_.y, sensor_accel_.z);
    if (!std::isfinite(spin) || spin > 0.08 || !std::isfinite(gravity) ||
        std::fabs(gravity - 9.80665) > 1.0) {
        level_stats_.reset();
        level_window_started_us_ = 0U;
        return;
    }
    if (level_window_started_us_ == 0U) {
        level_window_started_us_ = now;
        level_min_[0] = level_max_[0] = roll;
        level_min_[1] = level_max_[1] = pitch;
    }
    level_min_[0] = std::min(level_min_[0], roll);
    level_min_[1] = std::min(level_min_[1], pitch);
    level_max_[0] = std::max(level_max_[0], roll);
    level_max_[1] = std::max(level_max_[1], pitch);
    level_stats_.add(roll, pitch, 0.0);
    if (level_max_[0] - level_min_[0] >= 0.5 ||
        level_max_[1] - level_min_[1] >= 0.5) {
        level_stats_.reset();
        level_window_started_us_ = 0U;
        return;
    }
    if (now - level_window_started_us_ < 500000ULL || level_stats_.count() < 20U) return;
    const auto mean = level_stats_.mean();
    if (!commit_level(static_cast<float>(mean.x), static_cast<float>(mean.y))) {
        fail("unable to commit level parameters");
        return;
    }
    begin_wait_for_apply(now);
}

bool SensorCalibration::commit_level(float roll_deg, float pitch_deg) noexcept
{
    if (!std::isfinite(roll_deg) || !std::isfinite(pitch_deg) ||
        std::fabs(roll_deg) > 45.0F || std::fabs(pitch_deg) > 45.0F) return false;
    px4::AtomicTransaction transaction;
    if (!level_parameters_unchanged()) return false;
    clear_parameter_snapshot();
    parameter_snapshot_.type = Type::Level;
    parameter_snapshot_.id = level_board_rotation_;
    parameter_snapshot_.value_count = 2U;
    parameter_snapshot_.valid = true;
    std::copy(std::begin(level_old_offsets_), std::end(level_old_offsets_),
              parameter_snapshot_.values);
    parameter_expectation_ = parameter_snapshot_;
    parameter_expectation_.values[0] = roll_deg;
    parameter_expectation_.values[1] = pitch_deg;
    // 只写 X/Y；snapshot 的第三项和 id 仅用于确认 Z/离散旋转未被并发改变，
    // 故意不写它们。两项写入失败由现有 rollback 状态机恢复原 X/Y。
    if (param_set_no_notification(param_handle(dima::params::SENS_BOARD_X_OFF), &roll_deg) != 0 ||
        param_set_no_notification(param_handle(dima::params::SENS_BOARD_Y_OFF), &pitch_deg) != 0) {
        return false;
    }
    // 持锁记录仅由本 routine 两次写入造成的实际变化数；相同值不递增计数。
    // 协调器据此确认完整历史，不能把外部并发写入吞进 Level 的新基线。
    level_owned_changes_ = static_cast<std::uint8_t>(param_set_count() - level_start_set_count_);
    notify_parameter_changes();
    return true;
}

bool SensorCalibration::level_applied(std::uint64_t now) const noexcept
{
    const float expected[3]{parameter_expectation_.values[0],
        parameter_expectation_.values[1], parameter_expectation_.values[2]};
    const bool mag_applied = !level_mag_present_ ||
        (vehicle_magnetometer_frontend_.calibration_parameter_update_applied(required_parameter_update_instance_) &&
         vehicle_magnetometer_frontend_.board_adjustment_matches(required_parameter_update_instance_, expected) &&
         fresh(now, vehicle_magnetometer_.timestamp, kSensorFreshnessUs) &&
         vehicle_magnetometer_.timestamp_sample >= parameter_notification_time_us_);
    return parameter_expectation_.valid && parameter_expectation_.type == Type::Level &&
        required_parameter_update_valid_ &&
        vehicle_imu_frontend_.calibration_parameter_update_applied(required_parameter_update_instance_) &&
        vehicle_imu_frontend_.board_rotation_matches(required_parameter_update_instance_, parameter_expectation_.id, expected) &&
        imu_.accel_device_id == device_id_ && fresh(now, imu_.timestamp, 100000ULL) &&
        imu_.timestamp_sample >= parameter_notification_time_us_ && mag_applied;
}

} // namespace dima::modules::sensors
