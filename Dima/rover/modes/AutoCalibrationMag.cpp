#define MODULE_NAME "auto_cal"
#include "AutoCalibrationMode.hpp"

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
