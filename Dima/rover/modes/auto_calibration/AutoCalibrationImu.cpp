#define MODULE_NAME "auto_cal"
#include "AutoCalibrationMode.hpp"
#include "imu/VehicleImu.hpp"
#include "logging/logging.hpp"

#include <algorithm>
#include <cmath>
#include <climits>

namespace dima::rover::modes {
namespace {

bool stable_bias(const float (&bias)[3], const float (&variance)[3], float &length, float &noise) noexcept
{
    float maximum = 0.0F, minimum = INFINITY, energy = 0.0F;
    for (unsigned i = 0U; i < 3U; ++i) {
        if (!std::isfinite(bias[i]) || !std::isfinite(variance[i]) || variance[i] <= 0.0F) return false;
        maximum = std::max(maximum, variance[i]); minimum = std::min(minimum, variance[i]);
        energy += bias[i] * bias[i];
    }
    length = std::sqrt(energy); noise = std::sqrt(3.0F * maximum);
    return std::isfinite(length) && maximum < 1.0e-3F && maximum < 100.0F * minimum;
}

} // namespace

bool AutoCalibrationMode::begin_imu_bias(std::uint64_t now) noexcept
{
    if (imu_bias_attempted_) return false;
    imu_bias_attempted_ = true;
    const auto &bias = bias_sub_.get();
    std::int32_t enabled{};
    float accel[3]{}, gyro[3]{}, matrix[9]{}, active_accel[6]{};
    px4::AtomicTransaction atomic;
    if (armed_.armed() || !stopped() || !rtk_yaw_fused(now) ||
        param_get(param_handle(dima::params::SENS_IMU_AUTOCAL), &enabled) != 0 || enabled == 0 ||
        !fresh(bias.timestamp, now, 1000000ULL) || bias.timestamp_sample <= session_started_ ||
        !fresh(bias.timestamp_sample, now, 1500000ULL) ||
        !imu_frontend_.calibration_snapshot(imu_sub_.get().accel_device_id, imu_sub_.get().gyro_device_id,
            active_accel, gyro, matrix)) return false;
    imu_bias_accel_ = bias.accel_bias_valid && bias.accel_bias_stable &&
        bias.accel_device_id == imu_sub_.get().accel_device_id && bias.accel_device_id != 0U && bias.accel_device_id <= INT32_MAX &&
        stable_bias(bias.accel_bias, bias.accel_bias_variance, imu_bias_old_norm_[0], imu_bias_noise_[0]);
    imu_bias_gyro_ = bias.gyro_bias_valid && bias.gyro_bias_stable &&
        bias.gyro_device_id == imu_sub_.get().gyro_device_id && bias.gyro_device_id != 0U && bias.gyro_device_id <= INT32_MAX &&
        stable_bias(bias.gyro_bias, bias.gyro_bias_variance, imu_bias_old_norm_[1], imu_bias_noise_[1]);
    if (!imu_bias_accel_ && !imu_bias_gyro_) return false;
    if (imu_bias_accel_) {
        float stored_scale[3]{};
        if (param_get(param_handle(dima::params::CAL_ACC0_XSCALE), &stored_scale[0]) != 0 ||
            param_get(param_handle(dima::params::CAL_ACC0_YSCALE), &stored_scale[1]) != 0 ||
            param_get(param_handle(dima::params::CAL_ACC0_ZSCALE), &stored_scale[2]) != 0) return false;
        // 本事务不标定 scale。旧设备遗留比例与实际 identity 不同，就不能
        // 只换 ID 将其意外启用；该轴组保留原值，仍可独立处理可靠 gyro 偏置。
        for (unsigned i = 0U; i < 3U; ++i)
            if (stored_scale[i] != active_accel[i + 3U]) imu_bias_accel_ = false;
        if (!imu_bias_accel_ && !imu_bias_gyro_) return false;
    }
    for (unsigned i = 0U; i < 3U; ++i) {
        accel[i] = active_accel[i]; imu_bias_scale_[i] = active_accel[i + 3U];
        if (!std::isfinite(imu_bias_scale_[i]) || imu_bias_scale_[i] < 0.5F || imu_bias_scale_[i] > 1.5F) return false;
        // 使用已有 EKF stable/variance 判据，不另造估计器。机体系残余偏置按
        // R^T 回到传感器系：accel offset += R^T*bias/scale，gyro 没有 scale。
        float da = 0.0F, dg = 0.0F;
        for (unsigned j = 0U; j < 3U; ++j) {
            if (imu_bias_accel_) da += matrix[j * 3U + i] * bias.accel_bias[j];
            if (imu_bias_gyro_) dg += matrix[j * 3U + i] * bias.gyro_bias[j];
        }
        accel[i] += da / imu_bias_scale_[i]; gyro[i] += dg;
        if (!std::isfinite(accel[i]) || !std::isfinite(gyro[i])) return false;
    }
    if (!transaction_.prepare()) return false;
    bool added = true;
    if (imu_bias_accel_) added = transaction_.add_int(dima::params::CAL_ACC0_ID, static_cast<std::int32_t>(bias.accel_device_id)) &&
        transaction_.add_float(dima::params::CAL_ACC0_XOFF, accel[0]) && transaction_.add_float(dima::params::CAL_ACC0_YOFF, accel[1]) &&
        transaction_.add_float(dima::params::CAL_ACC0_ZOFF, accel[2]);
    if (imu_bias_gyro_) added = added && transaction_.add_int(dima::params::CAL_GYRO0_ID, static_cast<std::int32_t>(bias.gyro_device_id)) &&
        transaction_.add_float(dima::params::CAL_GYRO0_XOFF, gyro[0]) && transaction_.add_float(dima::params::CAL_GYRO0_YOFF, gyro[1]) &&
        transaction_.add_float(dima::params::CAL_GYRO0_ZOFF, gyro[2]);
    transaction_stage_ = Status::STATE_COMMIT_IMU;
    if (!added || !transaction_.apply(now, expected_set_count_, true)) return false;
    imu_bias_finalizing_ = false;
    transition(Status::STATE_COMMIT_IMU, now);
    return true;
}

bool AutoCalibrationMode::imu_bias_confirmed(std::uint64_t now) const noexcept
{
    if (!transaction_.generation_valid() || !fresh(imu_sub_.get().timestamp_sample, now, 100000ULL) ||
        imu_sub_.get().timestamp_sample <= transaction_.applied_at()) return false;
    bool applied = true;
    if (imu_bias_accel_) {
        const float expected[6]{transaction_.expected_float(dima::params::CAL_ACC0_XOFF),
            transaction_.expected_float(dima::params::CAL_ACC0_YOFF), transaction_.expected_float(dima::params::CAL_ACC0_ZOFF),
            imu_bias_scale_[0], imu_bias_scale_[1], imu_bias_scale_[2]};
        applied = imu_frontend_.accel_calibration_matches(transaction_.generation(),
            transaction_.expected_int(dima::params::CAL_ACC0_ID), expected);
    }
    if (imu_bias_gyro_) {
        const float expected[3]{transaction_.expected_float(dima::params::CAL_GYRO0_XOFF),
            transaction_.expected_float(dima::params::CAL_GYRO0_YOFF), transaction_.expected_float(dima::params::CAL_GYRO0_ZOFF)};
        applied = applied && imu_frontend_.gyro_calibration_matches(transaction_.generation(),
            transaction_.expected_int(dima::params::CAL_GYRO0_ID), expected);
    }
    return applied;
}

bool AutoCalibrationMode::imu_bias_residual_valid(std::uint64_t now) const noexcept
{
    const auto &bias = bias_sub_.get();
    if (!imu_bias_confirmed(now) || !imu_quality(now) || !rtk_yaw_fused(now) || !stopped() ||
        !fresh(bias.timestamp, now, 1000000ULL) || !fresh(bias.timestamp_sample, now, 1500000ULL) ||
        bias.timestamp_sample <= transaction_.applied_at()) return false;
    // 重融合后的验证也要求 EKF 自身的 stable 标志和新鲜采样，不能靠重复发布
    // 同一份旧 bias 将等待时长累计成三秒的有效残差证据。
    float length{}, noise{};
    return (!imu_bias_accel_ || (bias.accel_bias_valid && bias.accel_bias_stable && bias.accel_device_id == imu_sub_.get().accel_device_id &&
        stable_bias(bias.accel_bias, bias.accel_bias_variance, length, noise) &&
        length <= std::max(3.0F * imu_bias_noise_[0], 0.5F * imu_bias_old_norm_[0]))) &&
        (!imu_bias_gyro_ || (bias.gyro_bias_valid && bias.gyro_bias_stable && bias.gyro_device_id == imu_sub_.get().gyro_device_id &&
        stable_bias(bias.gyro_bias, bias.gyro_bias_variance, length, noise) &&
        length <= std::max(3.0F * imu_bias_noise_[1], 0.5F * imu_bias_old_norm_[1])));
}

bool AutoCalibrationMode::step_imu_bias(std::uint64_t now) noexcept
{
    if (status_.state != Status::STATE_COMMIT_IMU && status_.state != Status::STATE_WAIT_IMU_RELOCK) return false;
    const bool applied = imu_bias_confirmed(now);
    const bool valid = transaction_.rolling_back() || (!armed_.armed() && applied &&
        (imu_bias_finalizing_ ? imu_bias_residual_valid(now) : imu_quality(now)));
    transaction_.poll(applied, valid, now);
    if (transaction_.phase() == CalibrationParameters::Phase::Provisional) {
        expected_set_count_ = transaction_.set_count_snapshot();
        if (status_.state != Status::STATE_WAIT_IMU_RELOCK) transition(Status::STATE_WAIT_IMU_RELOCK, now);
        const bool residual = imu_bias_residual_valid(now);
        if (residual) { if (stable_since_ == 0U) stable_since_ = now; } else stable_since_ = 0U;
        if (stable_since_ != 0U && now - stable_since_ >= 3000000ULL) {
            if (!transaction_.finalize_provisional(now, expected_set_count_)) transaction_.cancel(now);
            else imu_bias_finalizing_ = true;
            transition(Status::STATE_COMMIT_IMU, now);
        } else if (now - state_started_ > 30000000ULL) {
            transaction_.cancel(now);
            transition(Status::STATE_COMMIT_IMU, now);
        }
    } else if (transaction_.phase() == CalibrationParameters::Phase::Done ||
               transaction_.phase() == CalibrationParameters::Phase::Failed) {
        expected_set_count_ = transaction_.set_count_snapshot();
        if (transaction_.phase() == CalibrationParameters::Phase::Done && imu_bias_accel_ && imu_bias_gyro_)
            status_.completed_stages |= Status::STAGE_IMU_BIAS;
        else status_.unavailable_stages |= Status::STAGE_IMU_BIAS;
        // 新 IMU 校准代重新建立后才启动响应采集；不重设全球圆心，不叠加旧 bias。
        angular_acceleration_sample_ = acceleration_epoch_ = 0U;
        filtered_acceleration_[0] = filtered_acceleration_[1] = filtered_angular_acceleration_ = 0.0F;
        start_response_profile(now);
    } else if (transaction_.phase() == CalibrationParameters::Phase::Fault) {
        terminate(Status::FAILURE_PARAMETER, false, now);
    }
    return true;
}

} // namespace dima::rover::modes
