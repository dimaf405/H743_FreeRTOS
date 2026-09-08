#include "AutoCalibrationMode.hpp"

#include "magnetometer/VehicleMagnetometer.hpp"
#include "control/RoverDifferential.hpp"
#include "api/Time.hpp"

#include <cmath>

namespace dima::rover::modes {

bool AutoCalibrationMode::begin_rtk_transaction(std::uint64_t now) noexcept
{
    transaction_stage_ = Status::STATE_COMMIT_RTK;
    return transaction_.prepare() &&
        transaction_.add_float(dima::params::GPS_YAW_BASELINE, status_.rtk_baseline_m) &&
        transaction_.add_float(dima::params::GPS_YAW_OFFSET, status_.rtk_yaw_offset_deg) &&
        transaction_.add_int(dima::params::EKF2_GPS_CTRL, config_.gps_control | (1 << 3)) && transaction_.apply(now, expected_set_count_);
}

bool AutoCalibrationMode::begin_dynamics_transaction(std::uint64_t now) noexcept
{
    transaction_stage_ = Status::STATE_COMMIT_DYNAMICS;
    // 最大速度出现在 yaw 前馈分母，两参数必须作为同一事务应用/回滚。
    return transaction_.prepare() &&
        transaction_.add_float(dima::params::RO_MAX_THR_SPEED, status_.maximum_speed_m_s) &&
        transaction_.add_float(dima::params::RO_YAW_RATE_CORR, status_.yaw_rate_correction) && transaction_.apply(now, expected_set_count_);
}

bool AutoCalibrationMode::begin_mag_transaction(std::uint64_t now, bool restore) noexcept
{
    px4::AtomicTransaction atomic;
    if (restore) {
        transaction_stage_ = Status::STATE_RESTORE_MAG;
        transaction_.cancel(now);
        return transaction_.phase() != CalibrationParameters::Phase::Fault;
    }
    if (transaction_.phase() == CalibrationParameters::Phase::Provisional) {
        transaction_stage_ = Status::STATE_COMMIT_MAG;
        return transaction_.refine(now, expected_set_count_, candidate_mag_);
    }
    transaction_stage_ = Status::STATE_COMMIT_MAG;
    const float *offset = candidate_mag_;
    const auto id = static_cast<std::int32_t>(mag_device_id_);
    if (mag_device_id_ == 0U || mag_device_id_ > static_cast<std::uint32_t>(INT32_MAX)) return false;
    return transaction_.prepare() && transaction_.add_int(dima::params::CAL_MAG0_ID, id) &&
        transaction_.add_float(dima::params::CAL_MAG0_XOFF, offset[0]) &&
        transaction_.add_float(dima::params::CAL_MAG0_YOFF, offset[1]) &&
        transaction_.add_float(dima::params::CAL_MAG0_ZOFF, offset[2]) && transaction_.apply(now, expected_set_count_, !mag_ready_);
}

bool AutoCalibrationMode::transaction_frontend_confirmed(std::uint64_t now) const noexcept
{
    if (!transaction_.generation_valid()) return false;
    if (transaction_stage_ == Status::STATE_COMMIT_IMU) return imu_bias_confirmed(now);
    if (transaction_stage_ == Status::STATE_APPLY_RUNTIME) return runtime_frontend_confirmed();
    if (transaction_stage_ == Status::STATE_APPLY_GAINS) return gain_frontend_confirmed();
    if (transaction_stage_ == Status::STATE_COMMIT_RTK) {
        const auto &rtk = rtk_sub_.get();
        const float offset = dima::lib::rover::calibration::wrap_pi(
            transaction_.expected_float(dima::params::GPS_YAW_OFFSET) * kRadians);
        return rtk.parameter_update_instance == transaction_.generation() &&
            fresh(rtk.timestamp_sample, now, 300000ULL) && rtk.timestamp_sample > transaction_.applied_at() &&
            rtk.configured_baseline_m == transaction_.expected_float(dima::params::GPS_YAW_BASELINE) &&
            std::fabs(dima::lib::rover::calibration::wrap_pi(rtk.configured_yaw_offset_rad - offset)) < 1.0e-6F;
    }
    if (transaction_stage_ == Status::STATE_COMMIT_DYNAMICS) {
        return drive_.calibration_parameters_applied(transaction_.generation(),
            transaction_.expected_float(dima::params::RO_MAX_THR_SPEED),
            transaction_.expected_float(dima::params::RO_YAW_RATE_CORR));
    }
    const float expected[6]{transaction_.expected_float(dima::params::CAL_MAG0_XOFF),
        transaction_.expected_float(dima::params::CAL_MAG0_YOFF),
        transaction_.expected_float(dima::params::CAL_MAG0_ZOFF),
        config_.mag_scale[0], config_.mag_scale[1], config_.mag_scale[2]};
    return mag_frontend_.calibration_parameter_update_applied(transaction_.generation()) &&
        mag_frontend_.mag_calibration_matches(transaction_.generation(), transaction_.expected_int(dima::params::CAL_MAG0_ID), expected) &&
        fresh(mag_sub_.get().timestamp_sample, now, 300000ULL) &&
        mag_sub_.get().timestamp_sample > transaction_.applied_at();
}

void AutoCalibrationMode::poll_transaction(std::uint64_t now) noexcept
{
    const bool applied = transaction_frontend_confirmed(now);
    bool valid = applied;
    if (!transaction_.rolling_back()) {
        if (transaction_stage_ == Status::STATE_COMMIT_RTK)
            valid = valid && rtk_yaw_fused(now) && yaw_aid_sub_.get().time_last_fuse > transaction_.applied_at();
        else if (transaction_stage_ == Status::STATE_COMMIT_MAG) valid = valid && mag_residual(now) < 0.08F;
    }
    if (valid) { if (stable_since_ == 0U) stable_since_ = now; }
    else stable_since_ = 0U;
    transaction_.poll(applied, stable_since_ != 0U && now - stable_since_ >= 2000000ULL, now);
    if (status_.state == Status::STATE_APPLY_MAG_BOOTSTRAP && transaction_.phase() == CalibrationParameters::Phase::Provisional) {
        expected_set_count_ = transaction_.set_count_snapshot();
        bootstrap_applied_ = true;
        for (unsigned axis = 0U; axis < 3U; ++axis) bootstrap_offset_[axis] = candidate_mag_[axis];
        transition(Status::STATE_WAIT_RTK_RELOCK, now);
        return;
    }
    if (transaction_.phase() == CalibrationParameters::Phase::Failed && bootstrap_applied_) {
        bootstrap_applied_ = false;
        if (status_.state == Status::STATE_RESTORE_MAG) { start_tuning(now); return; }
    }
    if (transaction_.phase() == CalibrationParameters::Phase::Failed || transaction_.phase() == CalibrationParameters::Phase::Fault) {
        terminate(Status::FAILURE_FRONTEND_CONFIRMATION, false, now);
        return;
    }
    if (transaction_.phase() != CalibrationParameters::Phase::Done) return;
    expected_set_count_ = transaction_.set_count_snapshot();
    if (status_.state == Status::STATE_COMMIT_RTK) {
        status_.completed_stages |= Status::STAGE_RTK;
        status_.progress = 55U;
        transition(Status::STATE_WAIT_RTK_RELOCK, now);
    } else if (status_.state == Status::STATE_COMMIT_DYNAMICS) {
        if (status_.failure_reason == Status::FAILURE_MECHANICAL_ASYMMETRY) status_.failure_reason = Status::FAILURE_NONE;
        status_.completed_stages |= Status::STAGE_SPEED | Status::STAGE_YAW;
        status_.progress = 80U;
        finish_movement(now);
    } else if (status_.state == Status::STATE_RESTORE_MAG) {
        bootstrap_applied_ = false;
        start_tuning(now);
    } else {
        bootstrap_applied_ = false;
        status_.completed_stages |= Status::STAGE_MAG;
        start_tuning(now);
    }
}

} // namespace dima::rover::modes
