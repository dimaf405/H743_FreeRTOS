#define MODULE_NAME "auto_cal"
#include "AutoCalibrationMode.hpp"

#include "magnetometer/VehicleMagnetometer.hpp"
#include "sensors/SensorRotation.hpp"
#include "world_magnetic_model/geo_mag_declination.h"
#include "matrix/math.hpp"
#include "logging/logging.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace dima::rover::modes {
namespace {

bool predicted_field(const vehicle_attitude_s &attitude, const sensor_gps_s &gps,
                     matrix::Vector3f &body) noexcept
{
    const matrix::Quatf q(attitude.q);
    if (!q.isAllFinite() || std::fabs(q.norm() - 1.0F) > 0.01F) return false;
    constexpr float radians = 0.01745329251994329577F;
    const matrix::Eulerf tilt(q);
    // 调用方先证明 GNSS yaw 实际融合且 mag_hdg/mag_3d 均关闭。使用 EKF
    // 已按 GNSS delay 对齐并传播至 attitude 时间的姿态，避免把接收机到达
    // 时刻的旧阵列 heading 当成当前时刻的 yaw。磁场不能参与该 yaw 闭环。
    if (std::fabs(tilt.phi()) > 15.0F * radians || std::fabs(tilt.theta()) > 15.0F * radians) return false;
    const float declination = get_mag_declination_degrees(gps.latitude_deg, gps.longitude_deg) * radians;
    const float inclination = get_mag_inclination_degrees(gps.latitude_deg, gps.longitude_deg) * radians;
    const float strength = get_mag_strength_gauss(gps.latitude_deg, gps.longitude_deg);
    if (!std::isfinite(strength) || strength < 0.1F || strength > 0.8F) return false;
    const matrix::Vector3f earth(strength * std::cos(inclination) * std::cos(declination),
        strength * std::cos(inclination) * std::sin(declination), strength * std::sin(inclination));
    body = matrix::Dcmf(q).transpose() * earth;
    return body.isAllFinite();
}

unsigned sectors(std::uint32_t mask) noexcept
{
    unsigned count = 0U;
    for (unsigned bit = 0U; bit < 24U; ++bit) if ((mask & (1UL << bit)) != 0U) ++count;
    return count;
}

} // namespace

void AutoCalibrationMode::collect_mag(std::uint64_t now, unsigned direction) noexcept
{
    if (!status_.magnetometer_present || direction >= 2U) return;
    const auto &raw = raw_mag_sub_.get();
    const auto &rtk = rtk_sub_.get();
    const bool identity_ok = raw.device_id != 0U && raw.device_id == mag_device_id_ &&
        (config_.mag_id == 0 || static_cast<std::uint32_t>(config_.mag_id) == raw.device_id);
    const bool source_ok = identity_ok && fresh(raw.timestamp_sample, now, 200000ULL) &&
        rtk_yaw_fused(now) && std::isfinite(raw.x) && std::isfinite(raw.y) && std::isfinite(raw.z) &&
        std::sqrt(raw.x * raw.x + raw.y * raw.y + raw.z * raw.z) <= 2.0F;
    if (!source_ok) { mag_stable_since_ = 0U; return; }
    const auto attitude_sample = attitude_sub_.get().timestamp_sample;
    const auto skew = raw.timestamp_sample > attitude_sample ? raw.timestamp_sample - attitude_sample : attitude_sample - raw.timestamp_sample;
    // 仅匹配相差 <=50 ms 的磁场/已传播姿态样本；分别 fresh 不能证明同一姿态。
    float rotation[9]{};
    const dima::lib::sensors::Vector3 fine{config_.board_offset[0], config_.board_offset[1], config_.board_offset[2]};
    if (!dima::lib::sensors::make_board_rotation_matrix(config_.mag_rotation, fine, rotation)) return;
    for (float scale : config_.mag_scale) if (!std::isfinite(scale) || scale < 0.1F || scale > 3.0F) return;

    if (raw.timestamp_sample > last_mag_sample_) {
        last_mag_sample_ = raw.timestamp_sample;
        matrix::Vector3f field{};
        if (skew <= 50000ULL && predicted_field(attitude_sub_.get(), gps_sub_.get(), field)) {
            // 覆盖度描述实际采到的方向，与 bootstrap 的偏置幅值门禁分开；
            // 已有有效大偏置时，正常 EKF 残差学习仍可独立完成。
            const float phase = dima::lib::rover::calibration::wrap_pi(rtk.array_heading_rad) + kPi;
            const unsigned sector = std::min(23U, static_cast<unsigned>(24.0F * phase / (2.0F * kPi)));
            coverage_[direction] |= 1UL << sector;
            const float sensor[3]{raw.x, raw.y, raw.z};
            double candidate[3]{};
            for (unsigned axis = 0U; axis < 3U; ++axis) {
                // known-yaw bootstrap：offset=raw-S^-1*R_sensor_to_body^T*B_body。
                // WMM 给出场强/倾角先验，属于有界初始化，不是全三轴软铁标定。
                const float expected = rotation[axis] * field(0) + rotation[3U + axis] * field(1) + rotation[6U + axis] * field(2);
                candidate[axis] = sensor[axis] - expected / config_.mag_scale[axis];
            }
            if (std::hypot(std::hypot(candidate[0], candidate[1]), candidate[2]) <= 1.0) {
                bootstrap_fit_[direction].add(candidate[0], candidate[1], candidate[2]);
            }
        }
    }

    const auto &flags = flags_sub_.get();
    const auto &bias = bias_sub_.get();
    const auto &aid = mag_aid_sub_.get();
    bool valid = (bootstrap_applied_ || config_.mag_id == static_cast<std::int32_t>(raw.device_id)) &&
        flags.cs_mag && !flags.cs_mag_hdg && !flags.cs_mag_3d && !flags.cs_mag_fault && !flags.cs_mag_field_disturbed &&
        fresh(aid.timestamp, now, 1500000ULL) && fresh(aid.time_last_fuse, now, 500000ULL) && aid.fused && !aid.innovation_rejected &&
        bias.mag_device_id == raw.device_id && fresh(bias.timestamp, now, 1500000ULL) &&
        bias.timestamp_sample > arm_started_ && bias.mag_bias_valid && bias.mag_bias_stable;
    for (unsigned axis = 0U; axis < 3U; ++axis) {
        valid = valid && std::isfinite(bias.mag_bias[axis]) && std::isfinite(bias.mag_bias_variance[axis]) &&
            bias.mag_bias_variance[axis] > 2.5e-8F && bias.mag_bias_variance[axis] < 1.0e-3F;
    }
    if (!valid) { mag_stable_since_ = 0U; return; }
    if (mag_stable_since_ == 0U) mag_stable_since_ = now;
    // EKF 的 stable 可能沿用历史累计；此处还需要本会话连续 10 s，再按唯一
    // bias 时间戳采样。Armed 期间只写固定 RAM 累计器。
    if (now - mag_stable_since_ < 10000000ULL || bias.timestamp <= last_bias_sample_) return;
    last_bias_sample_ = bias.timestamp;
    double candidate[3]{};
    double correction_squared = 0.0;
    for (unsigned axis = 0U; axis < 3U; ++axis) {
        const float correction = (rotation[axis] * bias.mag_bias[0] + rotation[3U + axis] * bias.mag_bias[1] +
            rotation[6U + axis] * bias.mag_bias[2]) / config_.mag_scale[axis];
        correction_squared += correction * correction;
        candidate[axis] = (bootstrap_applied_ ? bootstrap_offset_[axis] : config_.mag_offset[axis]) + correction;
    }
    if (correction_squared <= 0.09) bias_fit_[direction].add(candidate[0], candidate[1], candidate[2]);
}

bool AutoCalibrationMode::finish_mag(bool bootstrap) noexcept
{
    if (sectors(coverage_[0]) < 20U || sectors(coverage_[1]) < 20U) return false;
    const Stats3 *stats = bootstrap ? bootstrap_fit_ : bias_fit_;
    const unsigned required = bootstrap ? 60U : 5U;
    if (stats[0].count() < required || stats[1].count() < required) return false;
    const auto a = stats[0].mean(), b = stats[1].mean();
    const auto va = stats[0].variance(), vb = stats[1].variance();
    const double av[3]{a.x, a.y, a.z}, bv[3]{b.x, b.y, b.z};
    const double var_a[3]{va.x, va.y, va.z}, var_b[3]{vb.x, vb.y, vb.z};
    double difference_squared = 0.0;
    for (unsigned axis = 0U; axis < 3U; ++axis) {
        if (!std::isfinite(av[axis]) || !std::isfinite(bv[axis]) ||
            var_a[axis] > 0.0004 || var_b[axis] > 0.0004) return false;
        difference_squared += (av[axis] - bv[axis]) * (av[axis] - bv[axis]);
        candidate_mag_[axis] = static_cast<float>(0.5 * (av[axis] + bv[axis]));
    }
    // CW/CCW 的电流磁干扰不同；固定偏置无法解释的方向差异必须拒绝。
    return difference_squared <= 0.0009;
}

float AutoCalibrationMode::mag_residual(std::uint64_t now) const noexcept
{
    const auto &mag = mag_sub_.get();
    matrix::Vector3f expected{};
    if (!fresh(mag.timestamp_sample, now, 300000ULL) || mag.device_id != mag_device_id_ ||
        !stopped() || !rtk_yaw_fused(now) || !predicted_field(attitude_sub_.get(), gps_sub_.get(), expected))
        return std::numeric_limits<float>::infinity();
    // 提交后的验证只在确认停车时使用窗口均值；窗口内姿态不变，不再把同一
    // 前端样本与每个新姿态比较时间差并反复清空稳定计时。
    const matrix::Vector3f measured(mag.magnetometer_ga);
    return measured.isAllFinite() ? (measured - expected).norm() : std::numeric_limits<float>::infinity();
}

void AutoCalibrationMode::sample_mag_throttle(std::uint64_t now) noexcept
{
    // 只学未补偿的原始传感器值，避免旧 K 反馈进入下一代。去/返程单独截距，
    // 掉头引起的环境磁场方向变化不应成为油门斜率。
    const auto &raw = raw_mag_sub_.get();
    if (!status_.magnetometer_present || mag_device_id_ == 0U || raw.device_id != mag_device_id_ ||
        !fresh(raw.timestamp_sample, now, 200000ULL) || raw.timestamp_sample > raw.timestamp ||
        raw.timestamp_sample <= last_mag_mot_sample_ || matched_mag_sample_ != raw.timestamp_sample ||
        ground_speed() < 0.1F || std::fabs(yaw_rate()) > 0.05F ||
        std::fabs(dima::lib::rover::calibration::wrap_pi(
            leg_heading_ - rtk_sub_.get().array_heading_rad)) > 3.0F * kRadians) return;
    const matrix::Vector3f field(raw.x, raw.y, raw.z);
    const matrix::Quatf attitude(attitude_sub_.get().q);
    if (!field.isAllFinite() || field.norm() > 2.0F || !attitude.isAllFinite() ||
        std::fabs(attitude.norm() - 1.0F) > 0.01F) return;
    const matrix::Eulerf tilt(attitude);
    if (std::fabs(tilt.phi()) > 15.0F * kRadians || std::fabs(tilt.theta()) > 15.0F * kRadians) return;
    float u{};
    if (!motor_history_.forward_output(raw.timestamp_sample, u) || u < 0.3F * config_.motor_maximum) return;
    auto &fit = mag_mot_fit_[status_.state == Status::STATE_STRAIGHT_BACK ? 1U : 0U];
    if (fit.count == 0U) { fit.roll = tilt.phi(); fit.pitch = tilt.theta(); }
    if (std::fabs(tilt.phi() - fit.roll) > 2.0F * kRadians ||
        std::fabs(tilt.theta() - fit.pitch) > 2.0F * kRadians || fit.count == UINT32_MAX) return;
    last_mag_mot_sample_ = raw.timestamp_sample;
    ++fit.count;
    const double delta_u = u - fit.mean_u;
    fit.mean_u += delta_u / fit.count;
    fit.variance_u += delta_u * (u - fit.mean_u);
    fit.minimum_u = std::min(fit.minimum_u, u);
    fit.maximum_u = std::max(fit.maximum_u, u);
    for (unsigned axis = 0U; axis < 3U; ++axis) {
        const double delta_b = field(axis) - fit.mean_b[axis];
        fit.mean_b[axis] += delta_b / fit.count;
        fit.covariance[axis] += delta_u * (field(axis) - fit.mean_b[axis]);
        fit.variance_b[axis] += delta_b * (field(axis) - fit.mean_b[axis]);
    }
}

bool AutoCalibrationMode::commit_mag_throttle(std::uint64_t now) noexcept
{
    if (mag_mot_reported_) return false;
    mag_mot_reported_ = true;
    status_.unavailable_stages |= Status::STAGE_MAG_MOT;
    const auto &out = mag_mot_fit_[0];
    const auto &back = mag_mot_fit_[1];
    const double n = static_cast<double>(out.count) + back.count;
    const double variance_u = out.variance_u + back.variance_u;
    const double sum_uu = variance_u + out.count * out.mean_u * out.mean_u +
        back.count * back.mean_u * back.mean_u;
    const float span = std::max(out.maximum_u - out.minimum_u, back.maximum_u - back.minimum_u);
    bool usable = mag_device_id_ != 0U && mag_device_id_ <= static_cast<std::uint32_t>(INT32_MAX) &&
        out.count >= 30U && back.count >= 30U && n >= 100.0 &&
        span >= 0.15F * config_.motor_maximum && variance_u > 1.0e-4 * sum_uu;
    float sensor_slope[3]{};
    double residual_squared = 0.0;
    for (unsigned axis = 0U; axis < 3U && usable; ++axis) {
        // 共用斜率 K=Σ段 cov(u,B)/Σ段 var(u)，各段截距独立；静态偏置变化
        // 只改变截距。残差先用最终 scale 换到校正后的 gauss 域再判 0.08 G。
        const double covariance = out.covariance[axis] + back.covariance[axis];
        const double k = covariance / variance_u;
        usable = std::isfinite(k) && std::fabs(k) <= 1.0 &&
            std::isfinite(config_.mag_scale[axis]) && config_.mag_scale[axis] >= 0.1F && config_.mag_scale[axis] <= 3.0F;
        sensor_slope[axis] = static_cast<float>(k) * config_.mag_scale[axis];
        residual_squared += std::max(0.0, out.variance_b[axis] + back.variance_b[axis] -
            covariance * covariance / variance_u) * config_.mag_scale[axis] * config_.mag_scale[axis];
    }
    float rotation[9]{};
    const dima::lib::sensors::Vector3 fine{config_.board_offset[0], config_.board_offset[1], config_.board_offset[2]};
    usable = usable && residual_squared <= 0.08 * 0.08 * n &&
        dima::lib::sensors::make_board_rotation_matrix(config_.mag_rotation, fine, rotation);
    float coefficient[3]{};
    for (unsigned axis = 0U; axis < 3U; ++axis)
        coefficient[axis] = rotation[axis * 3U] * sensor_slope[0] +
            rotation[axis * 3U + 1U] * sensor_slope[1] + rotation[axis * 3U + 2U] * sensor_slope[2];
    const float reference = get_mag_strength_gauss(gps_sub_.get().latitude_deg, gps_sub_.get().longitude_deg);
    usable = usable && std::isfinite(reference) && reference >= 0.1F && reference <= 0.8F;
    if (!usable) {
        PX4_INFO("[autocal] mag throttle interference unobservable; compensation incomplete");
        return false;
    }
    // 干扰率是完整矢量模长相对当地 WMM 场强，允许超过 100%，未知用 -1。
    status_.mag_interference_pct = 100.0F * std::hypot(std::hypot(coefficient[0], coefficient[1]),
        coefficient[2]) * config_.motor_maximum / reference;
    PX4_INFO("[autocal] mag throttle %.0f%% interference; awaiting parameter commit",
        static_cast<double>(status_.mag_interference_pct));
    if (status_.mag_interference_pct > 30.0F)
        PX4_WARN("[autocal] magnetic interference above 30%%; check hardware placement");
    px4::AtomicTransaction atomic;
    std::int32_t generation{};
    if (param_get(param_handle(dima::params::CAL_MAG_MOT_GEN), &generation) != 0 ||
        generation < 0 || generation == INT32_MAX) {
        PX4_WARN("[autocal] mag throttle generation unavailable/exhausted; compensation incomplete");
        return false;
    }
    transaction_stage_ = Status::STATE_COMMIT_MAG_MOT;
    // 停车/Disarmed、前端代次确认、显式保存和回滚沿用同一有限事务，不允许
    // 五次裸写成功就宣称持久化完成。GEN 最后发布，整组通知在锁退出时合并。
    const bool prepared = transaction_.prepare() &&
        transaction_.add_float(dima::params::CAL_MAG_MOT_KX, coefficient[0]) &&
        transaction_.add_float(dima::params::CAL_MAG_MOT_KY, coefficient[1]) &&
        transaction_.add_float(dima::params::CAL_MAG_MOT_KZ, coefficient[2]) &&
        transaction_.add_int(dima::params::CAL_MAG_MOT_ID, static_cast<std::int32_t>(mag_device_id_)) &&
        transaction_.add_int(dima::params::CAL_MAG_MOT_GEN, generation + 1);
    if (prepared && transaction_.apply(now, expected_set_count_)) {
        transition(Status::STATE_COMMIT_MAG_MOT, now);
        return true;
    }
    if (transaction_.active()) {
        transition(Status::STATE_COMMIT_MAG_MOT, now);
        return true; // 进入回滚/故障确认，不覆盖仍持有互锁的事务。
    }
    PX4_WARN("[autocal] mag throttle transaction unavailable; compensation incomplete");
    return false;
}

void AutoCalibrationMode::finish_movement(std::uint64_t now) noexcept
{
    if (!status_.magnetometer_present) {
        status_.completed_stages |= Status::STAGE_MAG_SKIPPED;
        start_tuning(now);
        return;
    }
    if (!mag_path_ready(now)) {
        if (status_.failure_reason == Status::FAILURE_NONE) status_.failure_reason = Status::FAILURE_SENSOR_STALE;
        PX4_WARN("[autocal] magnetic path unavailable; finite calibration and >=10 Hz frontend required");
        if (bootstrap_applied_) {
            if (begin_mag_transaction(now, true)) transition(Status::STATE_RESTORE_MAG, now);
            else terminate(Status::FAILURE_PARAMETER, false, now);
        } else start_tuning(now);
        return;
    }
    mag_ready_ = finish_mag(false);
    if (mag_ready_) {
        if (begin_mag_transaction(now, false)) transition(Status::STATE_COMMIT_MAG, now);
        else terminate(Status::FAILURE_PARAMETER, false, now);
    } else if (!bootstrap_applied_ && finish_mag(true)) {
        // 只允许一次由完整双向覆盖验证的 WMM 初始化；验证后只应用 RAM，
        // 保留原参数快照且暂停保存。继续运动仍需新的人工 Arm。
        if (begin_mag_transaction(now, false)) transition(Status::STATE_APPLY_MAG_BOOTSTRAP, now);
        else terminate(Status::FAILURE_PARAMETER, false, now);
    } else {
        if (status_.failure_reason == Status::FAILURE_NONE) status_.failure_reason = Status::FAILURE_MAG_DISTURBED;
        PX4_WARN("[autocal] magnetic calibration did not converge");
        if (bootstrap_applied_) {
            if (begin_mag_transaction(now, true)) transition(Status::STATE_RESTORE_MAG, now);
            else terminate(Status::FAILURE_PARAMETER, false, now);
        } else start_tuning(now);
    }
}

} // namespace dima::rover::modes
