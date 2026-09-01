#include "Ekf2.hpp"

#include "api/Time.hpp"

#include <cmath>

namespace dima::modules::ekf2 {

void Ekf2::update_calibration(std::uint64_t timestamp,
                              InFlightCalibration &cal,
                              const matrix::Vector3f &bias,
                              const matrix::Vector3f &bias_variance,
                              float bias_limit, bool bias_valid,
                              bool learning_valid) noexcept
{
    // 与 PX4 v1.17 一致：进入 in_air 时丢弃地面累计。当前 Rover flags.in_air
    // 固定为 false，但保留该 Core 合同，避免未来状态来源改变后沿用旧候选。
    if (!ekf_.control_status_prev_flags().in_air &&
        ekf_.control_status_flags().in_air) {
        cal = {};
    }

    constexpr float kMaximumVariance = 1.0e-3F;
    constexpr float kMaximumVarianceRatio = 100.0F;
    const bool variance_valid =
        bias_variance.max() < kMaximumVariance &&
        bias_variance.max() <
            kMaximumVarianceRatio * bias_variance.min();
    const bool valid = bias_valid && variance_valid;

    if (valid && learning_valid) {
        // 稳定判定严格沿用 PX4：估计变化不超过 bias limit 的 10%，并累计超过
        // 10 s。累计只跨连续、单调时间片；时钟回退不能下溢成伪造长稳定期。
        const float bias_change_limit = 0.1F * bias_limit;
        if (!(cal.bias - bias).longerThan(bias_change_limit)) {
            if (cal.last_us != 0U) {
                if (timestamp > cal.last_us) {
                    cal.total_time_us += timestamp - cal.last_us;
                } else {
                    cal.total_time_us = 0U;
                    cal.cal_available = false;
                }
            }
            if (cal.total_time_us > 10000000ULL) {
                cal.cal_available = true;
            }
        } else {
            cal.total_time_us = 0U;
            cal.bias = bias;
            cal.cal_available = false;
        }
        cal.last_us = timestamp;
        return;
    }

    // learning 暂停时保留已经通过的累计时间；若方差或 fault 使估计本身失效，
    // 则整份候选清零，防止故障前的稳定时间在恢复后继续计入。
    cal.last_us = 0U;
    if (!valid && cal.total_time_us != 0U) {
        cal = {};
    }
}

void Ekf2::update_bias_stability(std::uint64_t timestamp) noexcept
{
    const ::parameters *const params = ekf_.getParamHandle();
    if (params == nullptr) {
        accel_cal_ = {};
        gyro_cal_ = {};
        mag_cal_ = {};
        return;
    }

    const bool accel_valid =
        (params->ekf2_imu_ctrl &
         static_cast<std::int32_t>(ImuCtrl::AccelBias)) != 0 &&
        ekf_.control_status_flags().tilt_align &&
        ekf_.fault_status().value == 0U &&
        !ekf_.fault_status_flags().bad_acc_clipping &&
        !ekf_.fault_status_flags().bad_acc_vertical;
    update_calibration(timestamp, accel_cal_, ekf_.getAccelBias(),
                       ekf_.getAccelBiasVariance(),
                       ekf_.getAccelBiasLimit(), accel_valid,
                       accel_valid && !ekf_.accel_bias_inhibited());

    const bool gyro_valid =
        (params->ekf2_imu_ctrl &
         static_cast<std::int32_t>(ImuCtrl::GyroBias)) != 0 &&
        ekf_.control_status_flags().tilt_align &&
        ekf_.fault_status().value == 0U;
    update_calibration(timestamp, gyro_cal_, ekf_.getGyroBias(),
                       ekf_.getGyroBiasVariance(),
                       ekf_.getGyroBiasLimit(), gyro_valid,
                       gyro_valid && !ekf_.gyro_bias_inhibited());

    const matrix::Vector3f mag_variance{ekf_.getMagBiasVariance()};
    const bool mag_valid = ekf_.fault_status().value == 0U &&
                           ekf_.control_status_flags().yaw_align &&
                           mag_variance.longerThan(0.0F) &&
                           !mag_variance.longerThan(0.02F);
    update_calibration(timestamp, mag_cal_, ekf_.getMagBias(),
                       mag_variance, ekf_.getMagBiasLimit(), mag_valid,
                       mag_valid && ekf_.control_status_flags().mag);
}

void Ekf2::publish_sensor_bias(std::uint64_t now_us) noexcept
{
    const matrix::Vector3f gyro_bias{ekf_.getGyroBias()};
    const matrix::Vector3f accel_bias{ekf_.getAccelBias()};
    const matrix::Vector3f mag_bias{ekf_.getMagBias()};
    const bool periodic = last_sensor_bias_published_ == 0U ||
                          now_us < last_sensor_bias_published_ ||
                          now_us - last_sensor_bias_published_ >=
                              kPeriodicStatusUs;
    if (!periodic &&
        !(gyro_bias - last_gyro_bias_published_).longerThan(0.001F) &&
        !(accel_bias - last_accel_bias_published_).longerThan(0.001F) &&
        !(mag_bias - last_mag_bias_published_).longerThan(0.001F)) {
        return;
    }

    estimator_sensor_bias_s output{};
    output.timestamp_sample = ekf_.time_delayed_us();
    const ::parameters *const params = ekf_.getParamHandle();
    if (gyro_device_id_ != 0U && params != nullptr &&
        (params->ekf2_imu_ctrl &
         static_cast<std::int32_t>(ImuCtrl::GyroBias)) != 0) {
        const matrix::Vector3f variance{ekf_.getGyroBiasVariance()};
        output.gyro_device_id = gyro_device_id_;
        gyro_bias.copyTo(output.gyro_bias);
        output.gyro_bias_limit = ekf_.getGyroBiasLimit();
        variance.copyTo(output.gyro_bias_variance);
        output.gyro_bias_valid =
            variance.longerThan(0.0F) && !variance.longerThan(0.1F);
        output.gyro_bias_stable = gyro_cal_.cal_available;
        last_gyro_bias_published_ = gyro_bias;
    }

    if (accel_device_id_ != 0U && params != nullptr &&
        (params->ekf2_imu_ctrl &
         static_cast<std::int32_t>(ImuCtrl::AccelBias)) != 0) {
        const matrix::Vector3f variance{ekf_.getAccelBiasVariance()};
        output.accel_device_id = accel_device_id_;
        accel_bias.copyTo(output.accel_bias);
        output.accel_bias_limit = ekf_.getAccelBiasLimit();
        variance.copyTo(output.accel_bias_variance);
        output.accel_bias_valid =
            variance.longerThan(0.0F) && !variance.longerThan(0.1F);
        output.accel_bias_stable = accel_cal_.cal_available;
        last_accel_bias_published_ = accel_bias;
    }

    if (mag_device_id_ != 0U) {
        const matrix::Vector3f variance{ekf_.getMagBiasVariance()};
        output.mag_device_id = mag_device_id_;
        mag_bias.copyTo(output.mag_bias);
        output.mag_bias_limit = ekf_.getMagBiasLimit();
        variance.copyTo(output.mag_bias_variance);
        output.mag_bias_valid =
            variance.longerThan(0.0F) && !variance.longerThan(0.1F);
        output.mag_bias_stable = mag_cal_.cal_available;
        last_mag_bias_published_ = mag_bias;
    }

    output.timestamp = now_us;
    (void)sensor_bias_pub_.publish(output);
    last_sensor_bias_published_ = now_us;
}

void Ekf2::update_mag_declination(std::uint64_t now_us) noexcept
{
    const auto commit_state = static_cast<MagDeclinationCommitState>(
        __atomic_load_n(&mag_declination_commit_state_, __ATOMIC_ACQUIRE));
    if (commit_state == MagDeclinationCommitState::Succeeded ||
        commit_state == MagDeclinationCommitState::Failed) {
        if (commit_state == MagDeclinationCommitState::Succeeded) {
            mag_declination_saved_ = true;
            mag_declination_retry_after_us_ = 0U;
        } else {
            // 参数服务失败只以 1 Hz 重试；不能由每个 IMU 更新周期持续调度
            // lp_default，更不能让失败反向阻塞 EKF Core 的状态更新与发布。
            mag_declination_retry_after_us_ =
                now_us > UINT64_MAX - kParameterCommitRetryUs
                    ? UINT64_MAX
                    : now_us + kParameterCommitRetryUs;
        }
        mag_declination_commit_value_deg_ = 0.0F;
        __atomic_store_n(
            &mag_declination_commit_state_,
            static_cast<std::uint8_t>(MagDeclinationCommitState::Idle),
            __ATOMIC_RELEASE);
    }

    if (mag_declination_saved_) {
        return;
    }
    if (now_us < mag_declination_retry_after_us_ ||
        __atomic_load_n(&mag_declination_commit_state_,
                        __ATOMIC_ACQUIRE) !=
            static_cast<std::uint8_t>(MagDeclinationCommitState::Idle)) {
        return;
    }

    float declination_deg = 0.0F;
    if (!ekf_.get_mag_decl_deg(declination_deg) ||
        !std::isfinite(declination_deg)) {
        return;
    }

    // estimator 只发布一个完整 float 快照，参数 bind/read/write 全部在非实时
    // worker 中执行；release/acquire 保证 consumer 不会看见半写的浮点值。
    mag_declination_commit_value_deg_ = declination_deg;
    __atomic_store_n(
        &mag_declination_commit_state_,
        static_cast<std::uint8_t>(MagDeclinationCommitState::Pending),
        __ATOMIC_RELEASE);
    if (!mag_declination_commit_worker_.ScheduleNow()) {
        mag_declination_commit_value_deg_ = 0.0F;
        __atomic_store_n(
            &mag_declination_commit_state_,
            static_cast<std::uint8_t>(MagDeclinationCommitState::Idle),
            __ATOMIC_RELEASE);
        mag_declination_retry_after_us_ =
            now_us > UINT64_MAX - kParameterCommitRetryUs
                ? UINT64_MAX
                : now_us + kParameterCommitRetryUs;
    }
}

void Ekf2::run_mag_declination_commit() noexcept
{
    std::uint8_t expected =
        static_cast<std::uint8_t>(MagDeclinationCommitState::Pending);
    if (!__atomic_compare_exchange_n(
            &mag_declination_commit_state_, &expected,
            static_cast<std::uint8_t>(MagDeclinationCommitState::Running),
            false, __ATOMIC_ACQUIRE, __ATOMIC_RELAXED)) {
        return;
    }

    const float declination_deg = mag_declination_commit_value_deg_;
    bool success = std::isfinite(declination_deg);
    bool changed = false;
    dima::ParamFloat<dima::params::EKF2_MAG_DECL> parameter{};
    success = success && parameter.bind();
    if (success &&
        std::fabs(declination_deg - parameter.get()) > 0.1F) {
        parameter.set(declination_deg);
        // 与 PX4 一致只更新参数内存且不发送 parameter_update；物理持久化仍由
        // 项目已有 autosave 路径负责，本模块绝不直接访问 Flash。
        success = parameter.commit_no_notification();
        changed = success;
    }
    if (success && changed) {
        PX4_INFO("EKF2 magnetic declination committed %.2f deg",
                 static_cast<double>(declination_deg));
    } else if (!success) {
        PX4_WARN("EKF2 magnetic declination parameter commit failed");
    }

    __atomic_store_n(
        &mag_declination_commit_state_,
        static_cast<std::uint8_t>(
            success ? MagDeclinationCommitState::Succeeded
                    : MagDeclinationCommitState::Failed),
        __ATOMIC_RELEASE);
    // 主动唤醒 estimator 只为消费结果；shutdown 时 owner 已关闭调度，失败
    // 返回不会破坏 stop 的 producer->consumer drain 顺序。
    (void)ScheduleNow();
}

void Ekf2::reset_mag_declination_commit() noexcept
{
    mag_declination_commit_value_deg_ = 0.0F;
    __atomic_store_n(
        &mag_declination_commit_state_,
        static_cast<std::uint8_t>(MagDeclinationCommitState::Idle),
        __ATOMIC_RELEASE);
}

} // namespace dima::modules::ekf2
